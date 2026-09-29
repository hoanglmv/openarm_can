# Data Recorder

Tầng thu thập, gióng hàng thời gian và chuẩn hóa dữ liệu:
- **Đầu vào đa luồng:**
  - Nhận trạng thái góc khớp `/joint_states` (100 Hz) từ Backend Middleware.
  - Nhận luồng hình ảnh `/camera` (25 Hz) từ Backend Middleware.
  - Nhận lệnh điều khiển thủ công `/joint_state_cmd` (50–100 Hz) từ Teleop.
  - Nhận lệnh `START_RECORD` / `STOP_RECORD` từ UX/UI Dashboard.
- **Temporal Alignment (Gióng hàng thời gian):**
  - Đồng bộ đa nguồn dữ liệu bất đồng bộ về cùng một mốc thời gian chuẩn hóa ở chu kỳ **50 Hz (20–25 ms/record)**.
  - Triệt tiêu độ lệch pha tín hiệu (zero-phase-shift) và timestamp jitter.
- **Đầu ra:**
  - File tập dữ liệu HDF5 / Zarr chuẩn hóa cho huấn luyện Offline (ACT / Diffusion Policy).
  - Ring Buffer Shared Memory / IPC cho suy luận Online trực tiếp.
