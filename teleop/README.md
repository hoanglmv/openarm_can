# Teleop (Master Arm / Leader)

Module giao tiếp tay điều khiển từ xa:
- Nhận `/joint_states` (100 Hz) từ Backend Middleware để đồng bộ trạng thái cánh tay (hoặc phản hồi lực).
- Đọc vị trí cơ cấu Master Arm do Người Vận Hành trực tiếp thao tác.
- Xuất bản topic `/joint_state_cmd` ở tần số 50–100 Hz tới:
  - **Backend Middleware:** Để điều khiển robot theo cơ chế follower.
  - **Data Recorder:** Để ghi nhận dữ liệu hành động mẫu của người vận hành phục vụ huấn luyện Imitation Learning.
