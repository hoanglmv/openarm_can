# UX / UI Dashboard

Giao diện người dùng giám sát và điều khiển hệ thống thời gian thực:
- **Giám sát thời gian thực:**
  - Nhận luồng video từ Camera RGB-D (25 Hz) từ Backend.
  - Nhận trạng thái góc khớp (`/joint_states` 100 Hz) từ Backend để hiển thị đồ thị / mô hình 3D trực quan.
- **Kích hoạt Dừng Khẩn Cấp (E-Stop):**
  - Gửi tín hiệu E-Stop lập tức về Backend Middleware (kênh ưu tiên cao nhất, bypass mọi luồng).
- **Quản lý thu thập dữ liệu (Record Control):**
  - Gửi lệnh `START_RECORD` và `STOP_RECORD` trực tiếp tới Data Recorder để bắt đầu hoặc kết thúc một episode thu thập.
