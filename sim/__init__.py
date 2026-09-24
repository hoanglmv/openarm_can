#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
OpenArm CAN & Simulation Package.
Provides simulation, hardware bridge, web dashboard, and telemetry streaming.
"""

import os
import sys

_pkg_dir = os.path.dirname(os.path.abspath(__file__))
if _pkg_dir not in sys.path:
    sys.path.insert(0, _pkg_dir)

from config import (
    CONTROL_FREQ,
    HTTP_PORT,
    JOINT_LIMITS,
    JOINT_NAME_TO_ID,
    TELEMETRY_FREQ,
    UDP_EXPORT_PORT,
    UDP_STREAM_PORT,
    WEB_DIR,
    WS_PORT,
    double_to_uint,
    uint_to_double,
)
from models import RealDamiaoMotorState
from can_bridge import RealRobotHardwareBridge
from exporter import JointStateExporter100Hz
from http_server import CustomHTTPHandler
from motor_simulator import DamiaoArmSimulator, VirtualDamiaoMotor
from dashboard_server import OpenArmDashboardServer

__all__ = [
    "OpenArmDashboardServer",
    "RealRobotHardwareBridge",
    "RealDamiaoMotorState",
    "JointStateExporter100Hz",
    "CustomHTTPHandler",
    "DamiaoArmSimulator",
    "VirtualDamiaoMotor",
    "JOINT_LIMITS",
    "JOINT_NAME_TO_ID",
    "HTTP_PORT",
    "WS_PORT",
    "UDP_STREAM_PORT",
    "UDP_EXPORT_PORT",
    "CONTROL_FREQ",
    "TELEMETRY_FREQ",
    "WEB_DIR",
    "double_to_uint",
    "uint_to_double",
]
