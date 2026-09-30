#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
Data models for real and virtual Damiao actuator states.
"""

import math
import time
from typing import Dict, Any, Optional

try:
    from config import gripper_rad_rate_to_stroke_rate, gripper_rad_to_stroke
except ImportError:
    from .config import gripper_rad_rate_to_stroke_rate, gripper_rad_to_stroke


class RealDamiaoMotorState:
    """Represents real-time telemetry and command state of a single physical Damiao motor."""

    def __init__(self, motor_id: int, name: str, arm: str, joint_idx: int,
                 motor_type: str, send_id: int, recv_id: int, can_if: str,
                 pMax: float, vMax: float, tMax: float):
        self.id = motor_id
        self.name = name
        self.arm = arm               # "left" or "right"
        self.joint_idx = joint_idx   # 1..7 for arm, 8 for gripper
        self.motor_type = motor_type
        self.send_id = send_id
        self.recv_id = recv_id
        self.can_if = can_if
        self.pMax = pMax
        self.vMax = vMax
        self.tMax = tMax

        # Operational state
        self.enabled = False
        self.error_code = 0
        self.q = 0.0
        self.dq = 0.0
        self.tau = 0.0
        self.t_mos = 30.0
        self.t_rotor = 28.0
        self.q_des = 0.0
        self.q_target = 0.0
        self.q_cmd = 0.0
        self.kp = 18.0
        self.kd = 2.0
        self.tau_ff = 0.0
        self.last_update = 0.0
        self.has_physical_sync = False
        self.invert = False
        self.gripper_ready = False

        # Internal rate limiters & recovery timestamps
        self._last_posforce_tx = 0.0
        self._last_clear_err = 0.0
        self._last_rearm = 0.0

    @property
    def is_gripper(self) -> bool:
        return self.joint_idx == 8

    # Joint-space values exposed outside the CAN layer (ROS 2 topics, datasets, ACT inference):
    # arm joints in rad / rad/s, grippers as finger stroke in m / m/s (same unit as teleop commands).
    def joint_position(self) -> float:
        return gripper_rad_to_stroke(self.q, self.invert) if self.is_gripper else self.q

    def joint_velocity(self) -> float:
        return gripper_rad_rate_to_stroke_rate(self.dq, self.invert) if self.is_gripper else self.dq

    def joint_target(self) -> float:
        return gripper_rad_to_stroke(self.q_target, self.invert) if self.is_gripper else self.q_target

    def to_dict(self) -> Dict[str, Any]:
        """Serialize motor state into JSON-compatible dictionary for WebSocket/REST API."""
        feedback_age_s = time.time() - self.last_update if self.last_update > 0.0 else None
        feedback_fresh = feedback_age_s is not None and feedback_age_s < 0.5
        # For Gripper (Joint 8): output linear stroke in meters (0.0 .. 0.043) and mm
        if self.is_gripper:
            stroke_m = self.joint_position()
            q_val = round(stroke_m, 4)
            q_deg_val = round(stroke_m * 1000.0, 1)  # displayed as mm in UI
            stroke_mm_val = round(stroke_m * 1000.0, 1)
        else:
            q_val = round(self.q, 4)
            q_deg_val = round(math.degrees(self.q), 1)
            stroke_mm_val = None

        return {
            "id": self.id,
            "name": self.name,
            "arm": self.arm,
            "joint_idx": self.joint_idx,
            "type": self.motor_type,
            "send_id": hex(self.send_id),
            "recv_id": hex(self.recv_id),
            "enabled": self.enabled,
            "error_code": self.error_code,
            "q": q_val,
            "q_deg": q_deg_val,
            "stroke_mm": stroke_mm_val,
            "q_rad": round(self.q, 4),
            "dq": round(self.dq, 4),
            "tau": round(self.tau, 3),
            "t_mos": round(self.t_mos, 1),
            "t_rotor": round(self.t_rotor, 1),
            "q_des": round(self.q_des, 4),
            "kp": round(self.kp, 1),
            "kd": round(self.kd, 2),
            "has_sync": self.has_physical_sync,
            "feedback_fresh": feedback_fresh,
            "feedback_age_s": round(feedback_age_s, 3) if feedback_age_s is not None else None,
        }
