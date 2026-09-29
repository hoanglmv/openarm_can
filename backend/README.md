# Backend Middleware (Core Hub)

Backend đóng vai trò hub điều phối trung tâm của toàn hệ thống OpenArm CAN:
- **CAN Node (400 Hz):** Giao tiếp hai chiều thời gian thực cứng với Robot Hardware qua bus CAN. Nhận phản hồi góc khớp, tải, vận tốc và phát lệnh điều khiển động cơ chu kỳ 2.5 ms.
- **Driver Video & Quản lý State (100 Hz):** Thu nhận luồng hình ảnh màu và độ sâu từ Camera RGB-D (25 Hz), đồng thời xuất bản dữ liệu góc khớp chuẩn hóa (`/joint_states` 100 Hz) và video tới Teleop cùng UX/UI.
- **Safety Guard & Nội suy Spline:**
  - Tiếp nhận lệnh điều khiển góc khớp từ Teleop (`/joint_state_cmd` 50–100 Hz) và Model AI (`/joint_state_cmd` 50 Hz).
  - Tiếp nhận lệnh Dừng Khẩn Cấp (E-Stop) từ UX/UI với mức ưu tiên cao nhất (Priority 0).
  - Kiểm tra an toàn (Safety Check: giới hạn góc khớp, vận tốc, gia tốc, timeout 50 ms).
  - Nội suy Spline (Cubic/Quintic Spline) nâng xung mượt mà từ 50 Hz lên 400 Hz trước khi nạp xuống Robot Hardware.
