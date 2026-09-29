# OpenArm CAN

Nền tảng điều khiển cánh tay robot OpenArm thời gian thực qua CAN Bus, tích hợp thu thập dữ liệu Demonstration Teleoperation và suy luận mô hình AI tự hành (ACT / Diffusion Policy).

## Tài liệu Kỹ thuật và Quy tắc Bắt buộc
- **Kiến trúc Hệ thống Toàn diện:** Xem tại [ARCHITECTURE.md](file:///home/quan/openarm_can/ARCHITECTURE.md).
- **Quy tắc Phát triển Bắt buộc cho Agent & Dev:** Xem tại [GEMINI.md](file:///home/quan/openarm_can/GEMINI.md) và [AGENTS.md](file:///home/quan/openarm_can/AGENTS.md).
- **Quy tắc Workspace (.agents/rules):** Xem tại [.agents/rules/openarm_architecture_rules.md](file:///home/quan/openarm_can/.agents/rules/openarm_architecture_rules.md).

## Sơ đồ Tổng quan Hệ thống
```mermaid
flowchart TD
    Teleop["Teleop (Master Arm)<br/>/joint_state_cmd (50-100Hz)"]
    UI["UX / UI Dashboard<br/>E-Stop & Record Triggers"]
    Backend["Backend Middleware (Core Hub)<br/>CAN Node (400Hz) | State (100Hz) | Safety & Spline"]
    Recorder["Data Recorder (50Hz / 20-25ms)<br/>Temporal Alignment | HDF5 & IPC Ring Buffer"]
    ModelAI["Model AI (Policy Engine)<br/>ACT / Diffusion | 1+49 Action Chunking @ 50Hz"]
    RobotHW["Robot Hardware (CAN 400Hz)"]
    Camera["Camera RGB-D (25Hz)"]

    RobotHW <-->|"CAN 400Hz"| Backend
    Camera -->|"USB/PCIe 25Hz"| Backend
    Backend -->|"/joint_states 100Hz"| Teleop
    Teleop -->|"/joint_state_cmd 50-100Hz"| Backend
    Backend -->|"/camera 25Hz & /joint_states 100Hz"| UI
    UI -.->|"E-Stop (Priority 0)"| Backend
    UI -.->|"Start/Stop Record"| Recorder
    Backend -->|"/joint_states 100Hz & /camera 25Hz"| Recorder
    Teleop -->|"/joint_state_cmd 50-100Hz"| Recorder
    Recorder ==>|"Dữ liệu đồng bộ 50Hz"| ModelAI
    ModelAI -->|"/joint_state_cmd 50Hz"| Backend
```

## Khế ước Tần số Chuẩn
- **Robot CAN Bus:** 400 Hz (2.5 ms)
- **Camera RGB-D:** 25 Hz (40 ms)
- **`/joint_states`:** 100 Hz (10 ms)
- **Teleop `/joint_state_cmd`:** 50–100 Hz (10–20 ms)
- **Data Recorder Chu kỳ đồng bộ:** 50 Hz (20–25 ms/record)
- **Model AI Policy Suy luận:** 50 Hz (20 ms, xuất 1+49 action chunking)
