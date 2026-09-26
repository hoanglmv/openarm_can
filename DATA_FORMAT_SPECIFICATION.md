# Quy Chuẩn Định Dạng Dữ Liệu Đầu Vào Chuẩn RGB-D (Data Format Specification)
**Dự án**: VinRobotics - OpenArm Autonomous Manipulation  
**Mô hình**: ACT (Action Chunking with Transformers)  
**Phần cứng áp dụng**: Robot Bimanual OpenArm (16-DOF: 2 tay x [7 khớp xoay + 1 kẹp Gripper]) + **01 Camera trước ngực RGB-D (Chest RGB-D)**

---

## 1. TỔNG QUAN

Tài liệu này là **quy chuẩn kỹ thuật thống nhất (Single Source of Truth)** cho toàn bộ pipeline dữ liệu của dự án:
1. **Team Data Recorder**: Viết script thu thập dữ liệu Teleoperation, đồng bộ hóa thời gian giữa Camera RGB-D và CAN-FD bus, đóng gói vào file `.hdf5`.
2. **Team Model & DataLoader**: Đọc dữ liệu, tiền xử lý chuẩn hóa RGB-D 4 kênh, tạo mini-batch đưa vào mô hình ACT để huấn luyện.
3. **Team Deployment / Integration**: Lấy luồng ảnh RGB-D trực tiếp từ camera, đọc góc 16 motor qua SocketCAN, chuẩn hóa theo đúng công thức lúc train để chạy Inference.

---

## 2. QUY CHUẨN THU THẬP VÀ ĐÓNG GÓI (OFFLINE DATASET)

* **Định dạng file**: **HDF5 (`.hdf5`)** — Chuẩn lưu trữ dữ liệu robotics quốc tế (ALOHA, LeRobot, RoboMimic).
* **Quy tắc đóng gói**: Mỗi lần robot hoàn thành 1 chu kỳ thao tác (1 Episode) sẽ được lưu thành **1 file `.hdf5` riêng biệt**:
  ```text
  dataset/
  ├── episode_0.hdf5
  ├── episode_1.hdf5
  ├── ...
  └── episode_49.hdf5
  ```
* **Tần số lấy mẫu (Sampling Rate)**: **50 Hz** cố định (chu kỳ đúng $20\text{ ms} \pm 1\text{ ms}$).
* **Thời lượng 1 Episode**: Khoảng $15 - 25\text{ giây}$ (tương đương $T = 750 - 1250\text{ timesteps}$).

---

## 3. CẤU TRÚC CHI TIẾT FILE HDF5 (SCHEMA CHUẨN RGB-D)

```text
root (HDF5 File: episode_XX.hdf5)
│
├── [Attributes]
│   ├── "sim"               : False (bool)
│   ├── "frequency_hz"      : 50 (int)
│   ├── "robot_type"        : "OpenArm_Bimanual_16DOF" (str)
│   ├── "num_joints"        : 16 (int)
│   ├── "depth_scale"       : 0.001 (float) — 1 đơn vị uint16 = 1 mm (0.001 m)
│   └── "depth_range_m"     : [0.2, 1.2] (float array) — Khoảng cách hợp lệ tới mặt bàn
│
├── /observations (Group)
│   ├── /images (Group)
│   │   ├── chest_rgb       : [T, 480, 640, 3] | uint8   | Ảnh màu RGB (0-255)
│   │   └── chest_depth     : [T, 480, 640]    | uint16  | Bản đồ độ sâu Depth Z (milimét: 0 - 65535)
│   │
│   ├── qpos                : [T, 16]          | float32 | Góc vị trí 16 motor (Radian)
│   ├── qvel                : [T, 16]          | float32 | Vận tốc góc 16 motor (Radian/s)
│   └── effort              : [T, 16]          | float32 | Mô-men xoắn 16 motor (Nm - Tùy chọn)
│
└── /action                 : [T, 16]          | float32 | Góc mục tiêu 16 motor cho bước kế tiếp (Radian)
```

### Bảng chi tiết từng trường dữ liệu:

| Tên trường (Key) | Kích thước (Shape) | Kiểu dữ liệu | Đơn vị | Quy định kỹ thuật bắt buộc |
| :--- | :--- | :--- | :---: | :--- |
| `/observations/images/chest_rgb` | `[T, 480, 640, 3]` | `uint8` ($0 - 255$) | Pixel | **Chuẩn RGB** (OpenCV đọc mặc định BGR $\to$ phải convert `cv2.cvtColor(img, cv2.COLOR_BGR2RGB)` trước khi ghi file). |
| `/observations/images/chest_depth`| `[T, 480, 640]` | `uint16` ($0 - 65535$) | Milimét (mm) | **Bắt buộc bật `align_to_color`** từ SDK camera để tọa độ $(u, v)$ của Depth trùng khớp $100\%$ với điểm ảnh RGB. Lưu `uint16` để tối ưu $50\%$ dung lượng ổ cứng so với `float32`. |
| `/observations/qpos` | `[T, 16]` | `float32` | Radian | Góc thực tế của 16 động cơ: `[Tay Trái J1..J7, Kẹp Trái J8, Tay Phải J1..J7, Kẹp Phải J8]`. |
| `/observations/qvel` | `[T, 16]` | `float32` | Rad/s | Vận tốc góc tức thời của 16 động cơ. |
| `/observations/effort` | `[T, 16]` | `float32` | Nm | Dòng tải/Mô-men xoắn của 16 động cơ (Tùy chọn ghi nhận từ telemetry CAN). |
| `/action` | `[T, 16]` | `float32` | Radian | Góc mục tiêu $q_{\text{des}}$ của bước kế tiếp ($t+1$). Trong teleop bimanual, đây là góc của cặp tay Leader hoặc vị trí mong muốn của Follower. |

---

## 4. YÊU CẦU THU THẬP & XỬ LÝ CẢM BIẾN RGB-D (HARDWARE CONSTRAINTS)

1. **Căn chỉnh không gian (Spatial Alignment)**:
   - Cảm biến RGB và cảm biến hồng ngoại Depth nằm cách nhau một khoảng vật lý. 
   - Script thu thập **bắt buộc phải gọi hàm căn chỉnh phần cứng**:
     - *Intel RealSense*: `rs.align(rs.stream.color).process(frames)`
     - *Orbbec SDK*: `align_filter = ob.Align(ob.StreamType.COLOR)`
2. **Khoảng cách làm việc (Depth Working Range)**:
   - Cự ly từ ngực robot đến mặt bàn thao tác nằm trong khoảng: **$0.2\text{ m} - 1.2\text{ m}$ (200 - 1200 mm)**.
   - Các điểm ngoài dải này (nền nhà, tường phòng) sẽ được thuật toán kẹp (clip) về biên.
3. **Môi trường & Ánh sáng**:
   - **Tránh ánh nắng mặt trời trực tiếp**: Ánh sáng mặt trời chứa phổ hồng ngoại mạnh sẽ làm "chói mù" cảm biến Depth cấu trúc ánh sáng (Structured Light / Active Stereo).
   - **Mặt bàn / Mặt bếp nấu**: Sử dụng mặt bàn bếp hoặc chảo có bề mặt nhám/mờ (matte), **tuyệt đối không dùng mặt kính bóng loáng hoặc kim loại phản xạ gương** gây lỗ thủng độ sâu (Hole Depth = 0).
4. **Quy tắc bắt đầu và kết thúc Episode (Implicit Termination)**:
   - **Bắt đầu**: Hai cánh tay robot đứng ở vị trí Home ($q_{\text{home}} = \mathbf{0}$) trong 1-2 giây.
   - **Thao tác**: Thực hiện xào nấu, đảo lật thức ăn trong chảo mượt mà (Bimanual Cooking & Stir-Frying).
   - **Kết thúc**: Nhấc vá xào, nâng tay lên cao, **thu 2 tay trở về vị trí Home $q_{\text{home}} = \mathbf{0}$** và giữ yên bất động tại đó 1-2 giây (khoảng 50-100 timesteps với $\Delta q = 0$). Điều này giúp mô hình tự học dấu hiệu dừng tự nhiên.

---

## 5. CÔNG THỨC TIỀN XỬ LÝ & TẠO BATCH (DATALOADER PIPELINE)

Khi DataLoader nạp dữ liệu vào mô hình PyTorch, các mảng thô trong file HDF5 được biến đổi thành tensor 4 kênh:

```python
import torch
import numpy as np

def preprocess_rgbd(rgb_raw, depth_raw):
    """
    Tiền xử lý cặp ảnh RGB và Depth thành Tensor 4 kênh cho ACT:
    Input:
        rgb_raw   : [480, 640, 3] uint8 (0 - 255)
        depth_raw : [480, 640] uint16 (mm)
    Output:
        rgbd_tensor: [4, 480, 640] float32 trên GPU
    """
    # 1. Xử lý RGB: Chia 255 và chuẩn hóa theo chuẩn ImageNet
    rgb_float = torch.from_numpy(rgb_raw).permute(2, 0, 1).float() / 255.0 # [3, 480, 640]
    mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
    rgb_norm = (rgb_float - mean) / std

    # 2. Xử lý Depth: Đổi từ mm sang mét, clip trong khoảng bàn [0.2m - 1.2m] và chuẩn hóa tuyến tính về [0, 1]
    depth_m = np.clip(depth_raw.astype(np.float32) / 1000.0, 0.2, 1.2)
    depth_norm = torch.from_numpy((depth_m - 0.2) / (1.2 - 0.2)).unsqueeze(0).float() # [1, 480, 640]

    # 3. Ghép 4 kênh: [3 kênh RGB, 1 kênh Depth]
    rgbd_tensor = torch.cat([rgb_norm, depth_norm], dim=0) # [4, 480, 640]
    return rgbd_tensor
```

### Cấu trúc Tensor Mini-Batch đưa vào mô hình ACT:
```text
batch = {
    # 1. Ảnh camera ngực RGB-D 4 kênh
    "image"   : Tensor [Batch, 1, 4, 480, 640], dtype=torch.float32

    # 2. Góc khớp hiện tại của 16 động cơ tại thời điểm t
    "qpos"    : Tensor [Batch, 16],             dtype=torch.float32

    # 3. Chuỗi 50 hành động mục tiêu tương lai [t, t+1, ..., t+49] của 16 động cơ
    "actions" : Tensor [Batch, 50, 16],         dtype=torch.float32

    # 4. Mask đệm (Padding mask) nếu gần cuối episode
    "is_pad"  : Tensor [Batch, 50],             dtype=torch.bool
}
```

---

## 6. SCRIPT KIỂM TRA TÍNH HỢP LỆ FILE HDF5 (VALIDATOR SCRIPT)

Team Data Recorder cần chạy script này để nghiệm thu từng file `.hdf5` trước khi đẩy lên bộ nhớ huấn luyện:

```python
import h5py
import numpy as np
import sys

def validate_hdf5(filepath):
    print(f"[*] Đang kiểm tra tính hợp lệ file: {filepath}")
    with h5py.File(filepath, "r") as f:
        # 1. Kiểm tra cấu trúc các Group và Dataset bắt buộc
        assert "observations" in f, "LỖI: Thiếu group 'observations'"
        assert "action" in f, "LỖI: Thiếu dataset 'action'"
        assert "images/chest_rgb" in f["observations"], "LỖI: Thiếu 'observations/images/chest_rgb'"
        assert "images/chest_depth" in f["observations"], "LỖI: Thiếu 'observations/images/chest_depth'"
        assert "qpos" in f["observations"], "LỖI: Thiếu 'observations/qpos'"
        assert "qvel" in f["observations"], "LỖI: Thiếu 'observations/qvel'"
        
        # 2. Lấy shape và kiểu dữ liệu
        chest_rgb = f["observations/images/chest_rgb"]
        chest_depth = f["observations/images/chest_depth"]
        qpos = f["observations/qpos"]
        action = f["action"]
        T = action.shape[0]
        
        print(f"  + Số bước thời gian (T)   : {T} timesteps (~{T/50:.1f}s)")
        print(f"  + Shape ảnh RGB ngực      : {chest_rgb.shape} | Dtype: {chest_rgb.dtype}")
        print(f"  + Shape ảnh Depth ngực    : {chest_depth.shape} | Dtype: {chest_depth.dtype}")
        print(f"  + Shape góc khớp qpos     : {qpos.shape}  | Dtype: {qpos.dtype}")
        print(f"  + Shape hành động action  : {action.shape}| Dtype: {action.dtype}")
        
        # 3. Ràng buộc khắt khe về kích thước và định dạng
        assert chest_rgb.shape == (T, 480, 640, 3), f"LỖI: Shape RGB {chest_rgb.shape} != ({T}, 480, 640, 3)"
        assert chest_depth.shape == (T, 480, 640), f"LỖI: Shape Depth {chest_depth.shape} != ({T}, 480, 640)"
        assert qpos.shape == (T, 16), f"LỖI: Shape qpos {qpos.shape} != ({T}, 16) (Bắt buộc 16 khớp 2 tay)"
        assert action.shape == (T, 16), f"LỖI: Shape action {action.shape} != ({T}, 16) (Bắt buộc 16 khớp 2 tay)"
        assert chest_rgb.dtype == np.uint8, "LỖI: RGB phải là uint8 (0-255)"
        assert chest_depth.dtype == np.uint16, "LỖI: Depth phải là uint16 (mm) để tối ưu dung lượng"
        assert qpos.dtype == np.float32, "LỖI: qpos phải là float32"
        assert action.dtype == np.float32, "LỖI: action phải là float32"
        assert not np.isnan(action[:]).any(), "LỖI: Dữ liệu action chứa giá trị NaN!"
        assert not np.isnan(qpos[:]).any(), "LỖI: Dữ liệu qpos chứa giá trị NaN!"
        
        # 4. Kiểm tra dải giá trị chiều sâu Depth
        d_sample = chest_depth[0]
        valid_pixels = np.count_nonzero(d_sample)
        total_pixels = 480 * 640
        valid_ratio = (valid_pixels / total_pixels) * 100.0
        print(f"  + Tỷ lệ điểm ảnh Depth hợp lệ: {valid_ratio:.1f}%")
        assert valid_ratio > 85.0, "CẢNH BÁO: Quá nhiều điểm mù (Holes) trên kênh Depth (< 85%)!"

    print("[✓] CHÚC MỪNG: File đạt chuẩn 100% RGB-D + 16-DOF cho Bimanual OpenArm ACT Pipeline!\n")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        validate_hdf5(sys.argv[1])
    else:
        print("Cách dùng: python validate_dataset.py <đường_dẫn_file.hdf5>")
```
