# OpenArm CAN Architecture Compliance Rule

Tất cả các agent làm việc trong repository `openarm_can` phải tuân theo các nguyên tắc kỹ thuật sau:

## 1. Phân định ranh giới giữa các tầng
- **Tầng Thiết Bị Ngoại Vi (1 & 2):**
  - Robot Hardware: Giao tiếp qua bus CAN 400 Hz.
  - Camera RGB-D: Giao tiếp qua USB/PCIe 25 Hz.
- **Tầng Trung Tâm - Backend Middleware (3):**
  - CAN Node: 400 Hz (đọc feedback, phát command).
  - Video Driver & State Manager: 25 Hz (camera) & 100 Hz (`/joint_states`).
  - Safety Guard & Spline Interpolation: Kiểm tra an toàn, đón E-Stop và nội suy từ 50 Hz lên 400 Hz.
- **Tầng Giao Tiếp Người Dùng (4 & 5):**
  - Teleop (Master Arm): Nhận 100 Hz `/joint_states`, phát 50-100 Hz `/joint_state_cmd`.
  - UX/UI Dashboard: Nhận 25 Hz video & 100 Hz `/joint_states`. Phát lệnh E-Stop tới Backend và Start/Stop Record tới Data Recorder.
- **Tầng Dữ Liệu - Data Recorder (6):**
  - Đồng bộ thời gian đa kênh (Temporal Alignment) ở tần số 50 Hz (chu kỳ 20–25 ms/record).
  - Xuất dữ liệu HDF5/Zarr (offline training) hoặc Shared Memory buffer (online inference).
- **Tầng Trí Tuệ Nhân Tạo - Model AI (7):**
  - Nhận dữ liệu quan sát đã qua gióng hàng thời gian từ Data Recorder (triệt tiêu độ lệch pha tín hiệu).
  - Thực hiện suy luận Action Chunking (1+49 actions) ở tần số 50 Hz.
  - Xuất bản `/joint_state_cmd` (50 Hz) ngược về Backend để kiểm tra an toàn và nội suy trước khi nạp xuống Robot.

## 2. Kiểm tra tuân thủ trước khi hoàn thành mã nguồn
- [ ] Không có file nào ngoài `backend/` kết nối trực tiếp đến socket CAN phần cứng.
- [ ] Lệnh điều khiển từ Model AI và Teleop đều được xử lý qua Safety Guard và Spline Interpolator.
- [ ] Tần số vòng lặp và timer tuân thủ đúng hằng số tại `common/constants.py`.
- [ ] Cấu trúc dữ liệu tuân thủ dataclass tại `common/messages.py`.
