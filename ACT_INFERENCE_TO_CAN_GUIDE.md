# Hướng Dẫn Kỹ Thuật: Từ Output Của Model ACT Đến Tín Hiệu Dây CAN
**Dự án**: VinRobotics - OpenArm Autonomous Manipulation  
**Cơ chế truyền thông**: PyTorch Tensor (RAM) $\longrightarrow$ SocketCAN CAN-FD $\longrightarrow$ Động Cơ Damiao

---

## 1. MÔ HÌNH ACT TRẢ VỀ DẠNG FILE GÌ?

> [!IMPORTANT]
> **LÚC CHẠY ĐIỀU KHIỂN ROBOT THẬT (REAL-TIME INFERENCE): KHÔNG CÓ FILE NÀO ĐƯỢC TẠO RA CẢ!**

* **Tại sao không dùng file (như `.txt`, `.json`, `.csv`)?**
  * Việc ghi ra ổ cứng rồi đọc lại tốn từ $10 - 30\text{ mili-giây}$ (I/O Disk Latency), sẽ làm vỡ vòng lặp điều khiển 50Hz và khiến robot bị khựng giật ngay lập tức.
* **Thực tế nó là cái gì?**
  * Output của ACT nằm **trực tiếp trong bộ nhớ RAM / VRAM** dưới dạng một biến **PyTorch Tensor** (hoặc mảng **NumPy ndarray**):
    ```python
    predicted_actions = policy(image_tensor, qpos_tensor)
    # predicted_actions là một torch.Tensor có kích thước [50, 8] nằm trên GPU/RAM
    ```

---

## 2. QUY TRÌNH CHUYỂN ĐỔI 4 TẦNG (TỪ TENSOR ĐẾN DÂY CAN)

Dưới đây là hành trình của dữ liệu từ khi AI tính toán xong cho đến khi motor quay:

```text
[TẦNG 1: AI POLICY (PyTorch trên GPU/CPU)]
  Output: Tensor [50, 8] (Ma trận góc của 50 bước tương lai)
       │
       ▼
[TẦNG 2: TEMPORAL ENSEMBLING (Xử lý trên RAM)]
  Gộp với các chunk trước đó bằng trung bình trọng số mũ
  Output: Đúng 1 vector 8 phần tử: q_target = [q1, q2, ..., q8] (NumPy array float32)
       │
       ▼
[TẦNG 3: DRIVER OPENARM_CAN (Thư viện C++ / Python)]
  Đóng gói 8 góc này thành 8 đối tượng MITParam:
  MITParam(kp=20.0, kd=1.0, q_des=q_target[i], v_des=0.0, tau_ff=0.0)
       │
       ▼ (Đóng gói nhị phân thành 8 bytes CAN Data Frame)
[TẦNG 4: SOCKETCAN & PHẦN CỨNG CAN BUS (/dev/can0)]
  Linux SocketCAN gửi gói tin qua cổng USB-to-CAN Adapter (5 Mbps)
  ──> Dây điện CAN_H / CAN_L (Chênh lệch điện áp High/Low)
  ──> Driver vi xử lý bên trong 8 động cơ Damiao
  ──> Cuộn dây stator sinh mô-men xoắn xoay cánh tay robot!
```

---

## 3. BÊN TRONG GÓI TIN CAN GỬI XUỐNG MOTOR CHỨA GÌ?

Mỗi động cơ Damiao nhận một gói tin nhị phân tiêu chuẩn gồm **8 bytes** (giao thức Damiao MIT Mode):

```text
Gói tin CAN (8 bytes payload):
┌───────────────┬───────────────┬───────────────┬───────────────┬───────────────┐
│ Byte 0 - 1    │ Byte 2 - 3    │ Byte 3 - 4    │ Byte 4 - 5    │ Byte 6 - 7    │
│ Vị trí q_des  │ Vận tốc v_des │ Độ cứng Kp    │ Giảm chấn Kd  │ Mô-men tau_ff │
│ (16-bit uint) │ (12-bit uint) │ (12-bit uint) │ (12-bit uint) │ (12-bit uint) │
└───────────────┴───────────────┴───────────────┴───────────────┴───────────────┘
```
Thư viện `openarm_can` viết bằng C++ đã làm sẵn việc ép kiểu float thành 8 bytes nhị phân này với tốc độ micro-giây, bạn chỉ cần gọi hàm Python!

---

## 4. CODE MẪU HOÀN CHỈNH VÒNG LẶP ĐIỀU KHIỂN (INFERENCE LOOP 50HZ)

Đây là code khung chuẩn nối toàn bộ từ Camera $\rightarrow$ Model ACT $\rightarrow$ CAN Bus OpenArm:

```python
#!/usr/bin/env python3
import time
import cv2
import torch
import numpy as np
import openarm_can as oa

# 1. KHỞI TẠO PHẦN CỨNG CAN OPENARM
arm = oa.OpenArm("can0", True)  # Khởi động SocketCAN CAN-FD trên can0

motor_types = [
    oa.MotorType.DM8009, oa.MotorType.DM8009,  # J1, J2 (Vai)
    oa.MotorType.DM4340, oa.MotorType.DM4340,  # J3, J4 (Khuỷu)
    oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310 # J5, J6, J7 (Cổ tay)
]
send_ids = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
recv_ids = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]

arm.init_arm_motors(motor_types, send_ids, recv_ids)
arm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18)

# Bật kích hoạt động cơ (Torque ON)
arm.enable_all()
time.sleep(0.05)
arm.recv_all(100)

# 2. KHỞI TẠO CAMERA NGỰC & LOAD MODEL ACT
cap = cv2.VideoCapture(0)  # Camera trước ngực
policy = torch.load("act_model.pth").cuda().eval()

# Khởi tạo buffer cho Temporal Ensembling
k = 50
all_time_actions = np.zeros([1000, 1000 + k, 8])

print("[*] BẮT ĐẦU ĐIỀU KHIỂN TỰ HÀNH 50 HZ...")

DT = 0.02  # Chu kỳ 20ms (50Hz)
step = 0

try:
    while True:
        loop_start = time.perf_counter()

        # BƯỚC A: ĐỌC DỮ LIỆU ĐẦU VÀO
        ret, frame = cap.read()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # [480, 640, 3]

        # Đọc góc khớp hiện tại từ robot qua CAN
        arm.refresh_all()
        arm.recv_all(50)
        current_q = [m.get_position() for m in arm.get_arm().get_motors()]
        current_q.append(arm.get_gripper().get_motors()[0].get_position()) # 8 góc thực tế

        # BƯỚC B: CHẠY MODEL ACT (Forward Pass trong RAM/VRAM)
        img_tensor = torch.from_numpy(frame_rgb).permute(2, 0, 1).float().unsqueeze(0).cuda() / 255.0
        qpos_tensor = torch.tensor(current_q).float().unsqueeze(0).cuda()

        with torch.no_grad():
            # Output là Tensor [1, 50, 8] nằm ngay trên VRAM
            action_chunk = policy(img_tensor, qpos_tensor).squeeze(0).cpu().numpy()

        # BƯỚC C: TEMPORAL ENSEMBLING (Lấy trung bình trọng số)
        all_time_actions[step, step:step+k] = action_chunk
        actions_for_curr_step = all_time_actions[:, step]
        actions_populated = actions_for_curr_step[actions_for_curr_step.any(axis=1)]
        
        # Trọng số mũ w = exp(-m * i)
        weights = np.exp(-0.01 * np.arange(len(actions_populated)))[::-1]
        target_q = np.sum(actions_populated * weights[:, None], axis=0) / np.sum(weights)
        # target_q là 1 vector 8 phần tử: [q1, q2, ..., q8]

        # BƯỚC D: ĐÓNG GÓI VÀ BƠM THẲNG XUỐNG DÂY CAN QUA OPENARM_CAN
        arm_commands = [
            oa.MITParam(20.0, 1.0, float(target_q[i]), 0.0, 0.0) for i in range(7)
        ]
        gripper_command = [
            oa.MITParam(8.0, 0.5, float(target_q[7]), 0.0, 0.0)
        ]

        # Hàm này sẽ chuyển thành 8 gói CAN-FD và bắn qua cổng CAN tức thì
        arm.get_arm().mit_control_all(arm_commands)
        arm.get_gripper().mit_control_all(gripper_command)

        # BƯỚC E: GIỮ NHỊP 50 HZ ĐỀU ĐẶN
        elapsed = time.perf_counter() - loop_start
        sleep_time = DT - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

        step += 1

except KeyboardInterrupt:
    print("\n[!] Dừng khẩn cấp: Đang tắt lực motor...")
    arm.disable_all()
    arm.recv_all(200)
    cap.release()
    print("[✓] Robot đã dừng an toàn.")
```
