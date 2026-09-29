# Model AI (Policy Engine)

Tác tử mô hình AI chính sách tự hành:
- **Kiến trúc mạng:** Mạng nơ-ron chính sách học bắt chước (ACT - Action Chunking with Transformers hoặc Diffusion Policy).
- **Thu nhận dữ liệu quan sát:**
  - Nhận dữ liệu quan sát đã được đồng bộ chuẩn hóa trực tiếp từ **Data Recorder** (file HDF5 khi huấn luyện Offline, Shared Memory khi suy luận Online).
  - Không đọc trực tiếp từ các luồng raw rời rạc để triệt tiêu độ lệch pha tín hiệu.
- **Suy luận Action Chunking @ 50 Hz:**
  - Tính toán và dự đoán chuỗi hành động chunking **1 + 49 = 50 actions** (1 bước hiện tại + 49 bước tương lai).
  - Xuất bản topic `/joint_state_cmd` ở tần số **50 Hz** ngược về Backend Middleware để kiểm tra an toàn và nội suy trước khi nạp xuống Robot.
