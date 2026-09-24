# Đặc Tả Use Case: Tabletop Pick-and-Place (Cấu Hình 1 Camera Ngực)
**Dự án**: VinRobotics - OpenArm Autonomous Manipulation  
**Phần cứng**: Robot OpenArm + **Duy nhất 01 Camera trước ngực (Chest Camera)**

---

## 1. TỔNG QUAN USE CASE

* **Tên tác vụ**: Tabletop Object Pick-and-Place (Gắp vật thể trên bàn bỏ vào khay).
* **Mục tiêu**: Huấn luyện mô hình ACT (Action Chunking with Transformers) điều khiển cánh tay OpenArm gắp vật thể hoàn toàn tự động chỉ dựa vào **01 luồng camera duy nhất đặt trước ngực**.
* **Đặc thù phần cứng**: 
  * Cánh tay OpenArm (7 khớp xoay + 1 motor kẹp Gripper).
  * **Hệ thống thị giác**: **01 Camera RGB duy nhất gắn trước ngực** (Chest-mounted Egocentric View), góc nhìn bao quát toàn bộ bàn thao tác và tầm với của cánh tay. Không có camera gắn trên cổ tay.

---

## 2. THIẾT LẬP MÔI TRƯỜNG & TẦM NHÌN (Setup)

| Thành phần | Đặc tả kỹ thuật | Ghi chú vận hành |
| :--- | :--- | :--- |
| **Vị trí Camera ngực** | Gắn cố định chính diện ngực robot, chúc góc xuống bàn $\approx 35^\circ - 45^\circ$ | Bao quát toàn bộ mặt bàn $60 \times 60$ cm và cả cánh tay robot |
| **Độ phân giải & Tần số** | RGB $640 \times 480$ pixels @ 30 FPS hoặc 60 FPS | Khuyến nghị camera góc rộng (FOV $\ge 80^\circ$) để không bị góc chết |
| **Vật thể thao tác** | Khối lập phương ($4 \times 4$ cm), chai nước nhỏ, hoặc hộp đồ vật | Màu sắc tương phản với mặt bàn |
| **Khay đích (Bin)** | Khay nhựa/hộp carton đặt cố định trong tầm với | Kích thước $\approx 15 \times 15$ cm, thành cao 3–5 cm |

> [!IMPORTANT]
> **Lưu ý về góc nhìn (Occlusion Strategy)**:
> Vì chỉ có 1 camera trước ngực, khi kẹp gắp hạ sát xuống vật thể, thân cánh tay có thể che khuất một phần tầm nhìn.
> **Quy tắc khi điều khiển mẫu (Teleop):** Dẫn hướng cánh tay tiếp cận vật thể từ phía trên dốc xuống (Top-down approach), giữ kẹp mở rõ ràng trong khung hình ngực để mô hình luôn nhìn thấy tương quan vị trí giữa kẹp và vật.

---

## 3. QUY TRÌNH THAO TÁC 6 BƯỚC

```text
[1. Home / Standby] ──> [2. Approach] ──> [3. Grasp] ──> [4. Lift] ──> [5. Transport] ──> [6. Release & Home]
```

1. **Bước 1 - Standby (Trạng thái chờ)**: Cánh tay ở tư thế Home (co gọn phía trước), kẹp mở. Camera ngực quan sát thấy rõ vật thể và khay đích.
2. **Bước 2 - Approach (Tiếp cận)**: Dựa vào ảnh từ camera ngực, model dự đoán quỹ đạo vươn cánh tay hạ kẹp từ trên xuống phía trên vật thể khoảng 3–5 cm.
3. **Bước 3 - Grasp (Kẹp giữ)**: Hạ thẳng đầu kẹp ôm sát thân vật và đóng motor kẹp (Joint 8) với lực an toàn.
4. **Bước 4 - Lift (Nhấc cao)**: Nâng thẳng vật thể lên cao cách mặt bàn 10–15 cm.
5. **Bước 5 - Transport (Vận chuyển)**: Xoay các khớp vai và khuỷu tay đưa vật thể sang vị trí phía trên miệng khay.
6. **Bước 6 - Release & Home (Nhả vật & Về chỗ)**: Mở kẹp thả vật rơi gọn vào khay, sau đó thu tay về tư thế Standby ban đầu.

---

## 4. TIÊU CHÍ ĐÁNH GIÁ THÀNH CÔNG (KPIs)

* **Tỷ lệ thành công (Success Rate)**: $\ge 80\%$ trong 20 lần thử nghiệm ngẫu nhiên vị trí vật thể trên bàn.
* **Thời gian 1 chu kỳ (Cycle Time)**: $\le 8 - 12$ giây.
* **Định nghĩa thất bại**: Kẹp trượt vật, làm rơi vật ngoài khay, hoặc va chạm mạnh vào mặt bàn / thành khay.

---

## 5. ĐỊNH DẠNG DỮ LIỆU CHUẨN (HDF5 RECORDER SCHEMA - 1 CAMERA NGỰC)

Team Data Recorder cấu hình cấu trúc file `.hdf5` như sau (chỉ có 1 dataset ảnh `chest`):

```text
episode_X.hdf5
├── /observations
│   ├── /images
│   │   └── chest        : [T, 480, 640, 3] uint8      (Ảnh RGB duy nhất từ camera trước ngực)
│   ├── qpos             : [T, 8] float32              (Vị trí góc 8 khớp hiện tại, Radian)
│   ├── qvel             : [T, 8] float32              (Vận tốc 8 khớp hiện tại, Rad/s)
│   └── effort           : [T, 8] float32 (Optional)   (Mô-men xoắn motor, Nm)
└── /action              : [T, 8] float32              (Góc mục tiêu q_des gửi xuống motor ở bước tiếp theo)
```

### Code Python mẫu cho Team Recorder:
```python
import h5py
import numpy as np

def save_episode_hdf5(filepath, chest_images, qpos_list, qvel_list, action_list):
    """
    Ghi 1 Episode thu thập vào file HDF5 chuẩn cho OpenArm (1 Camera ngực duy nhất)
    """
    with h5py.File(filepath, "w") as root:
        root.attrs["sim"] = False
        root.attrs["frequency_hz"] = 50

        obs = root.create_group("observations")
        images = obs.create_group("images")

        # Lưu luồng ảnh Camera ngực duy nhất (nén gzip)
        images.create_dataset("chest", data=np.array(chest_images, dtype=np.uint8),
                              chunks=(1, 480, 640, 3), compression="gzip")

        # Lưu trạng thái 8 khớp (7 khớp tay + 1 khớp gripper)
        obs.create_dataset("qpos", data=np.array(qpos_list, dtype=np.float32))
        obs.create_dataset("qvel", data=np.array(qvel_list, dtype=np.float32))

        # Lưu chuỗi target action
        root.create_dataset("action", data=np.array(action_list, dtype=np.float32))

    print(f"[Recorder] Đã lưu thành công: {filepath} ({len(action_list)} timesteps)")
```
