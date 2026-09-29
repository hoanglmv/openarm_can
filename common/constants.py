"""Hằng số kỹ thuật và khế ước thời gian toàn hệ thống OpenArm CAN."""

from typing import Final

# ==========================================
# 1 & 2. Tầng Thiết Bị Ngoại Vi (Hardware)
# ==========================================
CAN_BUS_FREQUENCY_HZ: Final[float] = 400.0          # Tần số bus CAN thời gian thực cứng
CAN_BUS_CYCLE_SEC: Final[float] = 1.0 / CAN_BUS_FREQUENCY_HZ  # 0.0025s (2.5 ms)

CAMERA_FREQUENCY_HZ: Final[float] = 25.0           # Tần số khung hình RGB-D
CAMERA_CYCLE_SEC: Final[float] = 1.0 / CAMERA_FREQUENCY_HZ   # 0.040s (40 ms)

# ==========================================
# 3. Tầng Trung Tâm (Backend Middleware)
# ==========================================
JOINT_STATE_PUB_FREQUENCY_HZ: Final[float] = 100.0  # Tần số xuất bản /joint_states
JOINT_STATE_PUB_CYCLE_SEC: Final[float] = 1.0 / JOINT_STATE_PUB_FREQUENCY_HZ # 0.010s (10 ms)

SAFETY_WATCHDOG_TIMEOUT_SEC: Final[float] = 0.050   # Timeout mất tín hiệu (50 ms)
SPLINE_TARGET_FREQ_HZ: Final[float] = CAN_BUS_FREQUENCY_HZ  # Tần số đích nội suy (400 Hz)

# ==========================================
# 4 & 5. Tầng Giao Tiếp Người Dùng (Teleop & UI)
# ==========================================
TELEOP_MIN_FREQ_HZ: Final[float] = 50.0             # Tần số tối thiểu Teleop
TELEOP_MAX_FREQ_HZ: Final[float] = 100.0            # Tần số tối đa Teleop

# ==========================================
# 6. Tầng Dữ Liệu (Data Recorder)
# ==========================================
RECORDER_ALIGNMENT_FREQ_HZ: Final[float] = 50.0     # Tần số gióng hàng chuẩn hóa
RECORDER_CYCLE_SEC: Final[float] = 1.0 / RECORDER_ALIGNMENT_FREQ_HZ # 0.020s (20 ms)
RECORDER_CYCLE_WINDOW_MIN_MS: Final[float] = 20.0   # Cửa sổ thời gian tối thiểu (20 ms)
RECORDER_CYCLE_WINDOW_MAX_MS: Final[float] = 25.0   # Cửa sổ thời gian tối đa (25 ms)

# ==========================================
# 7. Tầng Trí Tuệ Nhân Tạo (Model AI)
# ==========================================
MODEL_AI_INFERENCE_FREQ_HZ: Final[float] = 50.0     # Tần số suy luận mô hình AI (50 Hz)
ACTION_CHUNK_CURRENT_STEPS: Final[int] = 1          # 1 bước hiện tại
ACTION_CHUNK_FUTURE_STEPS: Final[int] = 49          # 49 bước tương lai
ACTION_CHUNK_TOTAL_HORIZON: Final[int] = ACTION_CHUNK_CURRENT_STEPS + ACTION_CHUNK_FUTURE_STEPS # 50 bước
