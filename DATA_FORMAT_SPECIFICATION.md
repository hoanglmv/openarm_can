# Quy Chuẩn Định Dạng Dữ Liệu Đầu Vào (Data Format Specification)
**Dự án**: VinRobotics - OpenArm Autonomous Manipulation  
**Mô hình**: ACT (Action Chunking with Transformers)  
**Phần cứng áp dụng**: Robot OpenArm (8-DOF: 7 khớp tay + 1 kẹp Gripper) + **01 Camera trước ngực (Chest Camera)**

---

## 1. TỔNG QUAN

Tài liệu này đặc tả quy chuẩn định dạng dữ liệu cho:
1. **Team Data Recorder**: Viết script thu thập và đóng gói dữ liệu mẫu (Demonstrations) vào file `.hdf5`.
2. **Team Model & DataLoader**: Đọc dữ liệu, tiền xử lý và đưa vào mô hình ACT để huấn luyện.
3. **Team Deployment / Integration**: Truyền dữ liệu thời gian thực (Online Streaming) lúc chạy Inference trên robot thật.

---

## 2. QUY CHUẨN FILE DỮ LIỆU THU THẬP (OFFLINE DATASET)

* **Định dạng file**: **HDF5 (`.hdf5`)** — Chuẩn dữ liệu robot quốc tế (ALOHA, LeRobot, RoboMimic).
* **Quy tắc đóng gói**: Mỗi lần robot hoàn thành 1 chu kỳ thao tác (1 Episode) sẽ được lưu thành **1 file `.hdf5` riêng biệt**:
  ```text
  data/
  ├── episode_0.hdf5
  ├── episode_1.hdf5
  ├── ...
  └── episode_49.hdf5
  ```
* **Tần số lấy mẫu (Sampling Rate)**: **50 Hz** cố định (chu kỳ lấy mẫu đúng $20\text{ ms} \pm 1\text{ ms}$).

---

## 3. CẤU TRÚC CHI TIẾT FILE HDF5 (SCHEMA)

```text
root (HDF5 File)
│
├── [Attributes]
│   ├── "sim"           : False (bool)
│   ├── "frequency_hz"  : 50 (int)
│   ├── "robot_type"    : "OpenArm_7DOF_Gripper" (str)
│   └── "num_joints"    : 8 (int)
│
├── /observations (Group)
│   ├── /images (Group)
│   │   └── chest       : [T, 480, 640, 3] | uint8   | Khuyến nghị nén gzip
│   │
│   ├── qpos            : [T, 8]           | float32 | Đơn vị: Radian
│   ├── qvel            : [T, 8]           | float32 | Đơn vị: Radian/s
│   └── effort          : [T, 8]           | float32 | Đơn vị: Nm (Tùy chọn)
│
└── /action             : [T, 8]           | float32 | Đơn vị: Radian
```

### Chi tiết từng trường dữ liệu:

| Tên trường (Key) | Kích thước (Shape) | Kiểu dữ liệu | Ý nghĩa & Đơn vị | Lưu ý quan trọng |
| :--- | :--- | :--- | :--- | :--- |
| `/observations/images/chest` | `[T, 480, 640, 3]` | `uint8` ($0 - 255$) | Ảnh màu RGB từ camera gắn trước ngực | **Bắt buộc chuẩn RGB** (OpenCV đọc mặc định là BGR, phải convert `cv2.cvtColor(img, cv2.COLOR_BGR2RGB)` trước khi lưu). |
| `/observations/qpos` | `[T, 8]` | `float32` | Góc vị trí thực tế của 8 motor tại thời điểm $t$ | Đơn vị: **Radian**. Thứ tự khớp: `[Joint 1, ..., Joint 7, Gripper]`. |
| `/observations/qvel` | `[T, 8]` | `float32` | Vận tốc góc thực tế của 8 motor tại thời điểm $t$ | Đơn vị: **Radian/giây (rad/s)**. |
| `/observations/effort` | `[T, 8]` | `float32` | Mô-men xoắn thực tế của motor | Đơn vị: **Newton-mét (Nm)**. Trường này tùy chọn. |
| `/action` | `[T, 8]` | `float32` | Lệnh góc mục tiêu ($q_{des}$) điều khiển robot | Đơn vị: **Radian**. Trong teleop, đây là góc của tay dẫn đường (Leader arm) hoặc góc mục tiêu bước kế tiếp $t+1$. |

*(Ghi chú: $T$ là tổng số bước thời gian trong episode đó. Ví dụ bài toán gấp áo mất 12–15 giây ở 50Hz thì $T \approx 600 - 750$).*

---

## 4. ĐỊNH DẠNG TENSOR ĐẦU VÀO CHO MÔ HÌNH ACT (TRAINING BATCH)

Khi DataLoader đọc file `.hdf5` và đưa vào mạng PyTorch lúc train, mỗi mini-batch kích thước $B$ sẽ có định dạng:

```text
batch = {
    # 1. Ảnh camera ngực (đã chuẩn hóa ImageNet: (img/255 - mean) / std)
    "image"   : Tensor [B, 1, 3, 480, 640], dtype=torch.float32

    # 2. Góc khớp hiện tại của robot tại thời điểm t
    "qpos"    : Tensor [B, 8],              dtype=torch.float32

    # 3. Chuỗi hành động mục tiêu 50 bước tương lai [t, t+1, ..., t+49]
    "actions" : Tensor [B, 50, 8],          dtype=torch.float32

    # 4. Mask kiểm tra bước đệm (padding) nếu ở sát cuối episode
    "is_pad"  : Tensor [B, 50],             dtype=torch.bool
}
```

---

## 5. DỮ LIỆU ĐẦU VÀO KHI SUY LUẬN TRỰC TIẾP (ONLINE INFERENCE)

Tại mỗi chu kỳ 20ms trên robot OpenArm thật:
1. **Camera ngực chụp 1 frame**: Mảng `numpy.ndarray` kích thước `(480, 640, 3)` RGB.
2. **CAN bus đọc góc 8 motor**: Mảng `numpy.ndarray` kích thước `(8,)` Radian.
3. **Đóng gói Tensor gửi vào PyTorch**:
   ```python
   # Chuẩn hóa ảnh
   img_tensor = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
   img_tensor = normalize(img_tensor).unsqueeze(0).unsqueeze(0).cuda() # [1, 1, 3, 480, 640]
   
   # Chuẩn hóa qpos
   qpos_tensor = torch.from_numpy(current_qpos).float().unsqueeze(0).cuda() # [1, 8]
   ```

---

## 6. SCRIPT KIỂM TRA HỢP LỆ DỮ LIỆU (VALIDATOR SCRIPT)

Team Data Recorder có thể chạy script kiểm tra nhanh tính toàn vẹn của file HDF5 trước khi bàn giao cho Team Huấn luyện:

```python
import h5py
import numpy as np
import sys

def validate_hdf5(filepath):
    print(f"[*] Đang kiểm tra file: {filepath}")
    with h5py.File(filepath, "r") as f:
        # 1. Kiểm tra các nhóm chính
        assert "observations" in f, "LỖI: Thiếu group 'observations'"
        assert "action" in f, "LỖI: Thiếu dataset 'action'"
        assert "images/chest" in f["observations"], "LỖI: Thiếu camera ngực 'observations/images/chest'"
        
        # 2. Lấy kích thước
        chest = f["observations/images/chest"]
        qpos = f["observations/qpos"]
        action = f["action"]
        T = action.shape[0]
        
        print(f"  + Số lượng timesteps T = {T}")
        print(f"  + Shape ảnh ngực       : {chest.shape} | Dtype: {chest.dtype}")
        print(f"  + Shape qpos           : {qpos.shape}  | Dtype: {qpos.dtype}")
        print(f"  + Shape action         : {action.shape}| Dtype: {action.dtype}")
        
        # 3. Kiểm tra tính hợp lệ
        assert chest.shape == (T, 480, 640, 3), f"Lỗi shape ảnh: {chest.shape} != ({T}, 480, 640, 3)"
        assert qpos.shape == (T, 8), f"Lỗi shape qpos: {qpos.shape} != ({T}, 8)"
        assert action.shape == (T, 8), f"Lỗi shape action: {action.shape} != ({T}, 8)"
        assert chest.dtype == np.uint8, "Lỗi: Ảnh phải là uint8 (0-255)"
        assert qpos.dtype == np.float32, "Lỗi: qpos phải là float32"
        assert not np.isnan(action[:]).any(), "LỖI: action chứa giá trị NaN!"
        
    print("[✓] FILE HỢP LỆ 100%! Đạt chuẩn huấn luyện ACT.")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        validate_hdf5(sys.argv[1])
    else:
        print("Cách dùng: python validate_dataset.py <đường_dẫn_file.hdf5>")
```
