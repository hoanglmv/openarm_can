# Kiến Trúc Hệ Thống OpenArm (System Architecture)

## 1. Backend (Core Engine & ROS 2 Bridge)
- **CAN Control Loop (400 Hz)**:
  - Vòng lặp điều khiển thời gian thực chu kỳ 2.5 ms (`CONTROL_FREQ = 400 Hz`).
  - Giao tiếp SocketCAN CAN-FD trên `can0` (Tay Phải) và `can1` (Tay Trái) qua thư viện C++ Core (`libopenarm-can.so`) hoặc card ảo `vcan0`.
  - Bộ sinh quỹ đạo làm mượt vận tốc (`v_lim = 0.25 rad/s` an toàn cơ khí).
- **ROS 2 State Publishers (100 Hz)**:
  - `pub /openarm/joint_states` (100 Hz, RELIABLE): Trạng thái 16 khớp thực tế (`position`, `velocity`, `effort`).
  - `pub /openarm/joint_commands` (100 Hz): Lệnh góc mục tiêu đang áp dụng (dùng đối chiếu sai số và ghi trường `action`).
  - Khớp tay theo quy ước URDF chính thức (rad, góc khớp = góc motor); gripper là hành trình ngón kẹp (m). Chi tiết: [ROS2_INTERFACE.md](ROS2_INTERFACE.md).
- **Camera RGB-D Publisher (25 Hz)**:
  - `pub /camera/act/rgb` (25 Hz): Ảnh màu RGB từ RealSense D435i/D405 (chuẩn 424x240 hoặc 640x480).
  - `pub /camera/act/depth` (25 Hz): Bản đồ độ sâu Depth căn chỉnh đồng bộ với RGB.
- **External Motion Command Subscribers (Zero-Latency Streaming Joint Commands)**:
  - `sub /openarm/teleop/joint_commands` (`std_msgs/msg/Float64MultiArray`): 14 góc khớp tay (rad), left J1..J7 rồi right J1..J7.
  - `sub /openarm/teleop/left_gripper`, `/openarm/teleop/right_gripper` (`std_msgs/msg/Float64MultiArray`): `data[0]` là hành trình kẹp (m, 0..0.043).
  - `sub /teleop/joint_commands`, `/meta/joint_states` (`sensor_msgs/msg/JointState`, alias): góc khớp tay theo tên.
  - Bridge gộp tay + gripper theo thứ tự 16 khớp và gửi xuống backend; không cần `joint_trajectory` nên gần như không có độ trễ. Bộ nội suy 400 Hz tại backend tự động làm mượt và bảo vệ cơ khí. Chi tiết: [ROS2_INTERFACE.md](ROS2_INTERFACE.md).
- **Data Recorder (Dataset Logger - 50 Hz)**:
  - `sub /camera/act/rgb`, `/camera/act/depth` và `/openarm/joint_states`.
  - Ghi file HDF5 ở tần số cố định **50 Hz** (chứa `observations/qpos`, `qvel`, `effort`, `images`, `action`) phục vụ huấn luyện mô hình Imitation Learning (ACT). Frame camera 25 Hz được tự động giữ cho timestep kế tiếp.

---

## 2. Frontend (Web Dashboard & 3D Digital Twin)
- **State Visualization**:
  - Nhận telemetry qua WebSocket (cổng 8889) ở tần số 40 Hz.
  - Dựng mô hình 3D OpenArm URDF thời gian thực bằng Three.js với thuật toán nội suy LERP 60 FPS chống giật.
  - Hiển thị bảng số đo 16 khớp (góc độ, radian, dòng điện, lực Nm, nhiệt độ MOSFET/Rotor).
- **Control Sliders & Action Dispatcher**:
  - Điều khiển góc từng khớp (J1..J7) có khóa an toàn Lock/Unlock.
  - Điều khiển thanh kẹp ngang song song J8 (0 - 43 mm) với các phím tắt Đóng / Mở / 50% / Đảo chiều.
  - Gửi WebSocket command `set_mit` và `set_gripper` lên server.
- **AI Model Inference Shadow Preview**:
  - Tải trọng số mô hình ACT `.pth`.
  - Hiển thị mô hình bóng mờ song sinh số **Shadow Robot** và đường cong quỹ đạo tương lai 3D (**Future Trajectory Chunk 32 bước**) mà hoàn toàn không can thiệp robot vật lý.
- **2 Chế độ Hoạt động**:
  - **Chỉ Mô Phỏng (Sim Only)**: Chạy thử thuật toán, test UI và AI không gửi xung CAN.
  - **Chạy Cả Hai (Dual Mode)**: Điều khiển đồng thời robot cơ khí và mô phỏng 3D trên Web.

---

## 3. Test Tools & Scripts (`tools/`)
- **`tools/test_meta_trajectory.py`**:
  - Stream danh sách joint commands (waypoints) hoặc cử động kính VR Meta Quest trực tiếp lên alias `/teleop/joint_commands` (100 Hz).
  - Không dùng `joint_trajectory`, tối ưu độ trễ 0 ms (Zero-latency).
  - Hỗ trợ stream trực tiếp qua UDP port 9870 mà không cần ROS 2.
- **`tools/send_joint_state.py`**: Gửi pose đơn lẻ dạng độ, radian hoặc file JSON.
- **`tools/stream_joint_states.py`**: Giả lập cử động sóng vẫy tay (wave), hình sin (sine) liên tục.
- **`tools/control_gripper.py`**: Test chu kỳ đóng/mở kẹp độc lập qua CAN-FD.
- **`tools/infer_robot.py`**: Suy luận mô hình ACT trực tiếp kết hợp camera RealSense và lọc làm mượt S-curve.
