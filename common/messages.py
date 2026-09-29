"""Định nghĩa cấu trúc thông điệp (Message Schemas) trao đổi giữa các tầng trong OpenArm CAN."""

from dataclasses import dataclass, field
from enum import Enum, auto
import time
from typing import List, Optional
import numpy as np


class ControlMode(Enum):
    """Chế độ điều khiển hệ thống."""
    IDLE = auto()
    TELEOP = auto()       # Người vận hành điều khiển qua Master Arm
    AUTONOMOUS = auto()   # Model AI tự hành điều khiển
    EMERGENCY_STOP = auto() # Trạng thái dừng khẩn cấp


class RecordAction(Enum):
    """Lệnh điều khiển ghi dữ liệu từ UX/UI."""
    START = auto()
    STOP = auto()


@dataclass
class EStopSignal:
    """Tín hiệu dừng khẩn cấp từ UX/UI hoặc Safety Guard (Priority 0)."""
    is_active: bool = True
    reason: str = "User triggered E-Stop"
    timestamp: float = field(default_factory=time.monotonic)


@dataclass
class RecordCommand:
    """Lệnh bắt đầu / dừng thu thập dữ liệu từ UX/UI đến Data Recorder."""
    action: RecordAction
    session_id: str
    metadata: dict = field(default_factory=dict)
    timestamp: float = field(default_factory=time.monotonic)


@dataclass
class CANFeedbackFrame:
    """Dữ liệu phản hồi trạng thái từ Robot Hardware qua CAN Bus (400 Hz)."""
    joint_positions: np.ndarray  # rad
    joint_velocities: np.ndarray # rad/s
    joint_efforts: np.ndarray    # Nm / Current (A)
    temperatures: Optional[np.ndarray] = None # Độ C
    fault_code: int = 0
    timestamp: float = field(default_factory=time.monotonic)


@dataclass
class CANCommandFrame:
    """Lệnh điều khiển nạp trực tiếp xuống Robot Hardware qua CAN Bus (400 Hz)."""
    target_positions: np.ndarray
    target_velocities: Optional[np.ndarray] = None
    target_torques: Optional[np.ndarray] = None
    timestamp: float = field(default_factory=time.monotonic)


@dataclass
class RGBDFrame:
    """Khung hình thu nhận từ Camera RGB-D qua USB/PCIe (25 Hz)."""
    color_image: np.ndarray     # (H, W, 3) uint8
    depth_image: np.ndarray     # (H, W) uint16 (mm)
    frame_id: int = 0
    timestamp: float = field(default_factory=time.monotonic)


@dataclass
class JointState:
    """Trạng thái góc khớp xuất bản từ Backend Middleware (/joint_states 100 Hz)."""
    positions: np.ndarray       # rad (shape: [num_joints])
    velocities: np.ndarray      # rad/s
    efforts: np.ndarray         # Nm
    timestamp: float = field(default_factory=time.monotonic)


@dataclass
class JointCommand:
    """Lệnh điều khiển góc khớp (/joint_state_cmd 50-100 Hz từ Teleop, 50 Hz từ Model AI)."""
    target_positions: np.ndarray # rad (shape: [num_joints])
    target_velocities: Optional[np.ndarray] = None
    gripper_cmd: float = 0.0     # 0.0 (mở) -> 1.0 (đóng)
    source: str = "teleop"       # "teleop" hoặc "model_ai"
    timestamp: float = field(default_factory=time.monotonic)


@dataclass
class AlignedRecordPacket:
    """Gói dữ liệu đã gióng hàng thời gian từ Data Recorder (50 Hz, 20-25 ms/record)."""
    timestamp: float            # Thời điểm chuẩn hoá của nhịp 50 Hz
    color_image: np.ndarray     # (H, W, 3)
    depth_image: np.ndarray     # (H, W)
    joint_positions: np.ndarray # (num_joints,)
    joint_velocities: np.ndarray# (num_joints,)
    action: np.ndarray          # (num_actions,) tương ứng với lệnh điều khiển tại nhịp đó


@dataclass
class AIActionChunk:
    """Dự đoán chính sách hành động từ Model AI (1 + 49 = 50 steps @ 50 Hz)."""
    actions: np.ndarray         # Shape: (50, action_dim) - [0]: 1 bước hiện tại, [1..49]: 49 bước tương lai
    inference_time_ms: float
    timestamp: float = field(default_factory=time.monotonic)
