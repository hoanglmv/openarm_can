# 🤖 Hướng Dẫn Nạp Trọng Số & Chạy Inference Model ACT Trên Dashboard UI

Tài liệu này hướng dẫn cách sử dụng tính năng **Inference Model ACT** và **Trực Quan Hóa Quỹ Đạo Tương Lai (3D Future Action Path)** trực tiếp trên giao diện Web Dashboard của OpenArm.

---

## 🌟 1. Tổng Quan Tính Năng Mới

1. **Đầu Vào Thời Gian Thực (Real Telemetry Input)**:
   - Tự động lấy **16 góc khớp thực tế** của 2 cánh tay từ hệ thống SocketCAN (`can0` / `can1`) hoặc mô phỏng `vcan0`.
   - Kết hợp luồng ảnh **RGB-D** từ camera Intel RealSense (chế độ non-blocking @ 25-60 FPS).
2. **Dự Đoán Quỹ Đạo Tương Lai 50 Bước (Future Action Horizon - 1.0 Giây)**:
   - Mô hình ACT dự đoán một chunk 50 bước hành động $t=1..50$ ($50 \times 16$).
   - Sử dụng **Forward Kinematics (FK)** tính toán tọa độ không gian 3D $(x, y, z)$ của Tool Center Point (TCP) cho cả 2 cánh tay:
     - 🔵 **Tay Trái (Left TCP)**: Đường cong neon **Cyan** (`#00e5ff`) + Waypoint dots.
     - 🟠 **Tay Phải (Right TCP)**: Đường cong neon **Orange** (`#ff6d00`) + Waypoint dots.
     - 🎯 **Target Goal Sphere**: Quả cầu đánh dấu vị trí đích tại bước $t=50$.
3. **Hai Chế Độ Thực Thi An Toàn (Dual Execution Modes)**:
   - 🛡️ **3D Simulation Preview (Mô Phỏng 3D)**: Lấy dữ liệu góc thực làm input, mô hình suy luận, vẽ đường cong 3D và mô phỏng cử động cánh tay trên Three.js. **Tuyệt đối KHÔNG gửi xung CAN xuống động cơ**, đảm bảo an toàn 100% khi thử nghiệm weights mới.
   - ⚡ **Physical Robot Execution (Robot Thật)**: Bơm xung điều khiển mượt mà qua **S-Curve Warm-up** và **Trajectory Smoother** (giới hạn gia tốc, vận tốc động cơ Damiao DM8009, DM4340, DM4310) xuống 16 động cơ CAN bus ở 40Hz.

---

## 🚀 2. Các Bước Khởi Động & Sử Dụng

### Bước 1: Khởi động Dashboard Server
Mở terminal và chạy lệnh:
```bash
python3 sim/server.py
```
*(Nếu muốn chạy thử nghiệm ảo không cần cắm USB CAN: `python3 sim/server.py --sim`)*

Mở trình duyệt truy cập:
👉 **[http://localhost:8888](http://localhost:8888)**

---

### Bước 2: Nạp Trọng Số Mô Hình (Weights Checkpoint)
1. Trên cột điều khiển bên trái, bấm vào tab **`Model ACT (AI)`**.
2. Nhập đường dẫn file checkpoint `.pth` vừa train (hoặc click vào các chip gợi ý):
   - `checkpoints/best_checkpoint.pth`
   - `checkpoints/act_deploy_weights.pth`
   - `dataset/act_openarm_model.pth`
3. Chọn thiết bị tính toán: **CUDA (GPU)** hoặc **CPU**.
4. Bấm nút **`Nạp Model`**.
   - Trạng thái trên badge sẽ chuyển sang: **`SẴN SÀNG (TORCH)`** (hoặc `PREVIEW (MOCK)` nếu chưa cài đặt PyTorch).

---

### Bước 3: Kiểm Tra Quỹ Đạo Với Chế Độ 3D Preview (Mô Phỏng An Toàn)
1. Giữ nguyên tùy chọn: **🛡️ 3D Simulation Preview (Mô Phỏng 3D)**.
2. Bấm nút **`▶ Bắt Đầu Chạy (Start)`** hoặc **`⏭ Bước Đơn (Step)`**.
3. Quan sát trên khung nhìn 3D Digital Twin:
   - 2 đường quỹ đạo tương lai 3D xuất hiện uốn lượn từ 2 đầu gắp robot trong không gian.
   - Các thông số suy luận hiển thị trực tiếp theo thời gian thực:
     - **Độ trễ (Latency)**: ~8 - 15 ms
     - **Tần số (FPS)**: ~40 FPS
     - **Số bước (Steps)**: Bộ đếm bước suy luận
     - **Max |Δq|**: Độ biến thiên góc lớn nhất giữa 2 bước liên tiếp.

---

### Bước 4: Tùy Chỉnh Hiển Thị Quỹ Đạo 3D
- Trên thẻ **Hiển Thị Quỹ Đạo Tương Lai 3D**:
  - Bật/tắt đường đi của từng cánh tay: `Tay Trái (Cyan)` / `Tay Phải (Orange)`.
  - Thanh trượt **Tầm nhìn dự đoán (Horizon)**: Điều chỉnh từ 10 bước (0.2s) đến 50 bước (1.0s).
- Trên thanh công cụ Viewport 3D (góc trên bên phải):
  - Bấm nút **`AI Path: ON / OFF`** để ẩn/hiện nhanh đường quỹ đạo mà không cần mở lại panel.

---

### Bước 5: Chuyển Sang Điều Khiển Robot Thật (Physical CAN Bus)
Sau khi đã quan sát đường quỹ đạo trên mô phỏng 3D và thấy robot di chuyển hợp lý:
1. Bấm **`⏹ Dừng (Stop)`**.
2. Chuyển radio sang **⚡ Physical Robot Control (Robot Thật)**.
   - Hệ thống sẽ hiện hộp thoại xác nhận an toàn.
3. Điều chỉnh thanh trượt **Tốc Độ Vận Tốc** (khuyến nghị để `0.5x` hoặc `1.0x` cho lần chạy đầu tiên).
4. Bấm **`▶ Bắt Đầu Chạy (Start)`**:
   - Robot thật ngoài đời sẽ bắt đầu di chuyển mượt mà bám theo quỹ đạo của mô hình ACT!
   - Bấm **`⏹ Dừng (Stop)`** hoặc `Ctrl + C` bất kỳ lúc nào để ngắt chuyển động an toàn.

---

## 🛡️ 3. Kiến Trúc Kỹ Thuật

```
[Camera RGB-D]  ──────────────┐
                              ▼
[16 Motors CAN State] ──> [ModelInferenceEngine] ──> [ACTPolicy Forward]
                                                           │
                                                           ▼
                                                50-step Future Chunk
                                                (Shape: 50 x 16 joints)
                                                           │
                                                           ▼
                           ┌───────────────────────────────┴───────────────────────────────┐
                           │ (Mode: 'preview')                                             │ (Mode: 'hardware')
                           ▼                                                               ▼
                 [WebSocket 40Hz]                                              [TemporalEnsemblePolicy]
                           │                                                               │
                           ▼                                                               ▼
             [Browser Three.js Viewport]                                          [TrajectorySmoother]
             - Forward Kinematics (FK)                                           (Vel & Accel Clamping)
             - Neon Cyan Path (Left TCP)                                                   │
             - Neon Orange Path (Right TCP)                                                ▼
             - Goal Markers (t=50)                                                 [SocketCAN Bus]
                                                                                   (16 Damiao Motors)
```
