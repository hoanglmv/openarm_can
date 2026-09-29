# OPENARM CAN DEVELOPMENT RULES (AGENTS.md)

All AI agents, coding assistants, and automated tools working on this codebase must strictly comply with:
1. Architecture Specifications: [ARCHITECTURE.md](file:///home/quan/openarm_can/ARCHITECTURE.md)
2. Detailed Operational Rules: [GEMINI.md](file:///home/quan/openarm_can/GEMINI.md)

### Key Architectural Directives:
- **Central Core Hub**: All hardware (CAN 400Hz, RGB-D 25Hz) and user/AI commands pass through `backend/`. Direct hardware access from Teleop or AI Policy is forbidden.
- **Data Ingestion for AI**: AI models must consume observations strictly through `recorder/` (HDF5 datasets or IPC Shared Memory) to eliminate signal phase shift.
- **Safety & Interpolation**: Every joint command is verified by `Safety Guard` and spline-interpolated from 50Hz to 400Hz before writing to CAN bus. E-Stop from UX/UI has priority 0.
- **Timing & Frequency Contracts**:
  - CAN Bus: 400 Hz (2.5 ms)
  - Camera RGB-D: 25 Hz (40 ms)
  - `/joint_states`: 100 Hz (10 ms)
  - Teleop `/joint_state_cmd`: 50–100 Hz (10–20 ms)
  - Data Recorder temporal alignment: 50 Hz (20–25 ms/record)
  - Model AI policy: 50 Hz, 1+49 action chunking
