#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
Unified Server Entry Point for OpenArm Bimanual 7-DOF Control Dashboard & Digital Twin:
- Supports REAL ROBOT mode (connecting directly to physical SocketCAN can0 and can1)
- Supports SIMULATION mode (running Virtual Damiao CAN-FD Simulator on vcan0)
- Serves HTTP static files (HTML/CSS/JS) on port 8888
- Serves WebSocket real-time telemetry and dual-arm control on port 8889 (40 Hz)
- Streams high-frequency joint state teleop on UDP ports 9870 / 9871
- Streams ACT camera feed (8890) and ROS 2 joint states / commands (8891)
"""

import os
import sys

# Ensure local sim directory is in sys.path for direct script execution
_sim_dir = os.path.dirname(os.path.abspath(__file__))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)

# Re-export core components for backward compatibility
from config import (
    CAMERA_ALIGNED_DEPTH_TOPIC,
    CAMERA_DEPTH_TOPIC,
    CAMERA_RAW_DEPTH_TOPIC,
    CAMERA_RAW_RGB_TOPIC,
    CAMERA_STREAM_PORT,
    CAMERA_TOPIC,
    CONTROL_FREQ,
    DATA_FREQUENCY_HZ,
    HTTP_PORT,
    JOINT_BRIDGE_PORT,
    JOINT_LIMITS,
    JOINT_NAME_TO_ID,
    JOINT_NAMES,
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
    "WEB_DIR",
    "CAMERA_STREAM_PORT",
    "CAMERA_RAW_RGB_TOPIC",
    "CAMERA_RAW_DEPTH_TOPIC",
    "CAMERA_ALIGNED_DEPTH_TOPIC",
    "CAMERA_TOPIC",
    "CAMERA_DEPTH_TOPIC",
    "JOINT_BRIDGE_PORT",
    "DATA_FREQUENCY_HZ",
    "JOINT_NAMES",
]


def main():
    # Check command-line arguments or auto-detect
    can0_available = os.path.exists("/sys/class/net/can0")
    if "--sim" in sys.argv:
        mode = "sim"
    elif "--real" in sys.argv:
        mode = "real"
    else:
        mode = "real" if can0_available else "sim"

    print(f"[Dashboard] Mode selected: {mode.upper()} (can0 available: {can0_available})")
    server = OpenArmDashboardServer(mode=mode, can0_if="can0", can1_if="can1")
    try:
        server.start()
    except KeyboardInterrupt:
        print("\n[Dashboard] Stopping...")
    finally:
        server.stop()


if __name__ == "__main__":
    main()
