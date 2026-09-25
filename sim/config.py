#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
Configuration constants, joint limits, socket definitions, and ROS 2 mapping
for OpenArm Bimanual 7-DOF Dual-Arm + Gripper System.
"""

import os
import socket
import struct

# Web & Network configuration
WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
HTTP_PORT = 8888
WS_PORT = 8889
UDP_STREAM_PORT = 9870   # Inbound high-speed joint stream (from ROS 2 / teleop)
UDP_EXPORT_PORT = 9871   # Outbound 100Hz telemetry stream broadcast

# Loop frequencies
CONTROL_FREQ = 400.0     # Hz for smooth trajectory generator & interpolation loop
TELEMETRY_FREQ = 40.0    # Hz for WebSocket state broadcasting
POLL_FREQ = 50.0         # Hz for hardware state query frames (matched to ACT dataset frequency)
DATA_FREQUENCY_HZ = 50.0 # Hz for ACT dataset and ROS 2 bridges

# RealSense Camera & ROS 2 ACT Pipeline Bridge configuration
CAMERA_STREAM_PORT = int(os.environ.get("OPENARM_CAMERA_STREAM_PORT", "8890"))
CAMERA_WIDTH = int(os.environ.get("OPENARM_CAMERA_WIDTH", "424"))
CAMERA_HEIGHT = int(os.environ.get("OPENARM_CAMERA_HEIGHT", "240"))
CAMERA_FREQUENCY_HZ = float(os.environ.get("OPENARM_CAMERA_FREQUENCY_HZ", "25"))
CAMERA_RAW_RGB_TOPIC = os.environ.get(
    "OPENARM_CAMERA_RAW_RGB_TOPIC",
    "/camera/camera/color/image_raw",
)
CAMERA_RAW_DEPTH_TOPIC = os.environ.get(
    "OPENARM_CAMERA_RAW_DEPTH_TOPIC",
    "/camera/camera/depth/image_rect_raw",
)
CAMERA_ALIGNED_DEPTH_TOPIC = os.environ.get(
    "OPENARM_CAMERA_ALIGNED_DEPTH_TOPIC",
    "/camera/camera/aligned_depth_to_color/image_raw",
)
CAMERA_TOPIC = os.environ.get("OPENARM_CAMERA_TOPIC", "/camera/act/rgb")
CAMERA_DEPTH_TOPIC = os.environ.get("OPENARM_CAMERA_DEPTH_TOPIC", "/camera/act/depth")
JOINT_BRIDGE_PORT = int(os.environ.get("OPENARM_JOINT_BRIDGE_PORT", "8891"))

JOINT_NAMES = [
    "left_j1", "left_j2", "left_j3", "left_j4",
    "left_j5", "left_j6", "left_j7", "left_gripper",
    "right_j1", "right_j2", "right_j3", "right_j4",
    "right_j5", "right_j6", "right_j7", "right_gripper",
]

# SocketCAN low-level definitions
SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_RAW_FD_FRAMES = getattr(socket, "CAN_RAW_FD_FRAMES", 5)

CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SZ = struct.calcsize(CAN_FRAME_FMT)

CANFD_FRAME_FMT = "=IBBB1x64s"
CANFD_FRAME_SZ = 72


def double_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """Quantize a float value into an unsigned integer with specified bits."""
    span = x_max - x_min
    val = max(x_min, min(x_max, x))
    return int((val - x_min) * ((1 << bits) - 1) / span)


def uint_to_double(x: int, x_min: float, x_max: float, bits: int) -> float:
    """Convert an unsigned integer back to float value."""
    span = x_max - x_min
    norm = float(x) / float((1 << bits) - 1)
    return norm * span + x_min


# Mechanical Joint Limits defined for OpenArm 7-DOF + Gripper
JOINT_LIMITS = {
    # Left Arm (IDs 1..7) & Left Gripper (ID 8)
    1: (-1.3963, 3.4907),
    2: (-3.3161, 0.17453),  # Left J2: Shoulder Roll (-3.3161 .. 0.17453 rad)
    3: (-1.5708, 1.5708),
    4: (0.0, 2.4435),
    5: (-1.5708, 1.5708),
    6: (-0.7854, 0.7854),
    7: (-1.5708, 1.5708),
    8: (0.000, 0.043),      # Left Gripper: Stroke: 0.0 - 0.043 m (0 - 43 mm)

    # Right Arm (IDs 9..15) & Right Gripper (ID 16)
    9:  (-1.3963, 3.4907),
    10: (-0.17453, 3.3161),
    11: (-1.5708, 1.5708),
    12: (0.0, 2.4435),
    13: (-1.5708, 1.5708),
    14: (-0.7854, 0.7854),
    15: (-1.5708, 1.5708),
    16: (0.000, 0.043),     # Right Gripper: Stroke: 0.0 - 0.043 m (0 - 43 mm)
}

# OpenArm v2.0 Kinematic Joint Origins (from Enactic OpenArm v2.0 joint_origins.yaml)
JOINT_ORIGINS = {
    "joint1": {"x": 0.0, "y": -0.0625, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "joint2": {"x": 0.0, "y": -0.0600000000000001, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "joint3": {"x": 0.0, "y": 0.0, "z": -0.06625000000000079, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "joint4": {"x": 0.0, "y": 0.0, "z": -0.15375, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "joint5": {"x": 0.0, "y": 0.0, "z": -0.0955000000000005, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "joint6": {"x": 0.0, "y": 0.0, "z": -0.120499999999998, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "joint7": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
}

# Motor physical mounting direction multiplier (+1.0: normal, -1.0: inverted physical mounting)
# On OpenArm, Left Shoulder Pitch (Motor 1) is mechanically mirrored relative to Right Shoulder Pitch (Motor 9).
# Setting -1.0 ensures that a positive angle (+q) moves both left and right arms forward.
MOTOR_DIRECTIONS = {
    1: -1.0,  # Left J1 (Shoulder Pitch): Inverted physical mounting so +q pushes forward
}

# Standard ROS 2 (MoveIt / sensor_msgs.JointState) naming mapping to OpenArm Motor IDs
JOINT_NAME_TO_ID = {
    # Left Arm Joints (1..7)
    "openarm_left_joint1": 1, "left_joint1": 1, "left_j1": 1, "l_j1": 1, "joint1": 1, "j1": 1,
    "openarm_left_joint2": 2, "left_joint2": 2, "left_j2": 2, "l_j2": 2, "joint2": 2, "j2": 2,
    "openarm_left_joint3": 3, "left_joint3": 3, "left_j3": 3, "l_j3": 3, "joint3": 3, "j3": 3,
    "openarm_left_joint4": 4, "left_joint4": 4, "left_j4": 4, "l_j4": 4, "joint4": 4, "j4": 4,
    "openarm_left_joint5": 5, "left_joint5": 5, "left_j5": 5, "l_j5": 5, "joint5": 5, "j5": 5,
    "openarm_left_joint6": 6, "left_joint6": 6, "left_j6": 6, "l_j6": 6, "joint6": 6, "j6": 6,
    "openarm_left_joint7": 7, "left_joint7": 7, "left_j7": 7, "l_j7": 7, "joint7": 7, "j7": 7,
    # Left Gripper (8)
    "openarm_left_finger_joint": 8, "openarm_left_gripper": 8, "left_gripper": 8, "left_grip": 8, "l_grip": 8, "joint8": 8, "j8": 8,

    # Right Arm Joints (9..15)
    "openarm_right_joint1": 9, "right_joint1": 9, "right_j1": 9, "r_j1": 9, "joint9": 9, "j9": 9,
    "openarm_right_joint2": 10, "right_joint2": 10, "right_j2": 10, "r_j2": 10, "joint10": 10, "j10": 10,
    "openarm_right_joint3": 11, "right_joint3": 11, "right_j3": 11, "r_j3": 11, "joint11": 11, "j11": 11,
    "openarm_right_joint4": 12, "right_joint4": 12, "right_j4": 12, "r_j4": 12, "joint12": 12, "j12": 12,
    "openarm_right_joint5": 13, "right_joint5": 13, "right_j5": 13, "r_j5": 13, "joint13": 13, "j13": 13,
    "openarm_right_joint6": 14, "right_joint6": 14, "right_j6": 14, "r_j6": 14, "joint14": 14, "j14": 14,
    "openarm_right_joint7": 15, "right_joint7": 15, "right_j7": 15, "r_j7": 15, "joint15": 15, "j15": 15,
    # Right Gripper (16)
    "openarm_right_finger_joint": 16, "openarm_right_gripper": 16, "right_gripper": 16, "right_grip": 16, "r_grip": 16, "joint16": 16, "j16": 16,
}
