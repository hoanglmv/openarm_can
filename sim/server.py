#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
Unified Server for OpenArm Bimanual 7-DOF Control Dashboard & Digital Twin:
- Supports REAL ROBOT mode (connecting directly to physical SocketCAN can0 and can1)
- Supports SIMULATION mode (running Virtual Damiao CAN-FD Simulator on vcan0)
- Serves HTTP static files (HTML/CSS/JS) on port 8888
- Serves WebSocket real-time telemetry and dual-arm control on port 8889 (40 Hz)
- Executes openarm-can-cli subcommands on demand
"""

import os
import sys
import time
import json
import math
import socket
import struct
import asyncio
import threading
import subprocess
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List, Optional

import websockets

from motor_simulator import (
    DamiaoArmSimulator,
    VirtualDamiaoMotor,
    double_to_uint,
    uint_to_double,
    CANFD_FRAME_FMT,
    CAN_FRAME_FMT,
    CAN_RAW_FD_FRAMES,
    SOL_CAN_RAW
)

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
HTTP_PORT = 8888
WS_PORT = 8889

class CustomHTTPHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def log_message(self, format, *args):
        pass


class RealDamiaoMotorState:
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
        self.last_update = 0.0

    def to_dict(self):
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
            "q": round(self.q, 4),
            "q_deg": round(math.degrees(self.q), 1),
            "dq": round(self.dq, 4),
            "tau": round(self.tau, 3),
            "t_mos": round(self.t_mos, 1),
            "t_rotor": round(self.t_rotor, 1),
            "q_des": round(self.q_des, 4),
            "kp": round(self.kp, 1),
            "kd": round(self.kd, 2)
        }


# Mechanical Joint Limits defined for OpenArm 7-DOF + Gripper
JOINT_LIMITS = {
    # Left Arm (IDs 1..7) & Left Gripper (ID 8)
    1: (-1.3963, 3.4907),
    2: (-0.17453, 3.3161),
    3: (-1.5708, 1.5708),
    4: (0.0, 2.4435),
    5: (-1.5708, 1.5708),
    6: (-0.7854, 0.7854),
    7: (-1.5708, 1.5708),
    8: (0.000, 0.043), # Left Gripper: Thanh kẹp ngang (Stroke: 0.0 - 0.043 m / 0 - 43 mm)

    # Right Arm (IDs 9..15) & Right Gripper (ID 16)
    9:  (-1.3963, 3.4907),
    10: (-0.17453, 3.3161),
    11: (-1.5708, 1.5708),
    12: (0.0, 2.4435),
    13: (-1.5708, 1.5708),
    14: (-0.7854, 0.7854),
    15: (-1.5708, 1.5708),
    16: (0.000, 0.043), # Right Gripper: Thanh kẹp ngang (Stroke: 0.0 - 0.043 m / 0 - 43 mm)
}


class RealRobotHardwareBridge:
    def __init__(self, can0_if: str = "can0", can1_if: Optional[str] = "can1"):
        self.can0_if = can0_if
        self.can1_if = can1_if
        self.running = False
        self.lock = threading.Lock()
        self.frames_rx = 0
        self.frames_tx = 0
        self.traffic_log: List[dict] = []
        self._last_tx_log = 0.0
        self._last_rx_log = 0.0

        self.socks: Dict[str, socket.socket] = {}

        # CAN Bus Physical Mapping:
        # can1 = LEFT ARM  (Physical Left Arm -> Motors 1..8)
        # can0 = RIGHT ARM (Physical Right Arm -> Motors 9..16)
        self.can_left_if = can1_if if (can1_if and can1_if != "") else "can1"
        self.can_right_if = can0_if

        # 16-actuator dictionary (Dual 7-DOF Arms + 2 Grippers)
        self.motors: Dict[int, RealDamiaoMotorState] = {
            # LEFT ARM (1..8) on can1 (Physical Left Arm)
            1: RealDamiaoMotorState(1, "Left J1 (Shoulder Pitch)", "left", 1, "DM8009", 0x01, 0x11, self.can_left_if, 12.5, 45.0, 54.0),
            2: RealDamiaoMotorState(2, "Left J2 (Shoulder Roll)",  "left", 2, "DM8009", 0x02, 0x12, self.can_left_if, 12.5, 45.0, 54.0),
            3: RealDamiaoMotorState(3, "Left J3 (Arm Twist)",      "left", 3, "DM4340", 0x03, 0x13, self.can_left_if, 12.5, 10.0, 28.0),
            4: RealDamiaoMotorState(4, "Left J4 (Elbow Pitch)",    "left", 4, "DM4340", 0x04, 0x14, self.can_left_if, 12.5, 10.0, 28.0),
            5: RealDamiaoMotorState(5, "Left J5 (Forearm Twist)",  "left", 5, "DM4310", 0x05, 0x15, self.can_left_if, 12.5, 30.0, 10.0),
            6: RealDamiaoMotorState(6, "Left J6 (Wrist Pitch)",    "left", 6, "DM4310", 0x06, 0x16, self.can_left_if, 12.5, 30.0, 10.0),
            7: RealDamiaoMotorState(7, "Left J7 (Wrist Roll)",     "left", 7, "DM4310", 0x07, 0x17, self.can_left_if, 12.5, 30.0, 10.0),
            8: RealDamiaoMotorState(8, "Left Gripper (J8 Kẹp Ngang)",    "left", 8, "DM4310", 0x08, 0x18, self.can_left_if, 12.5, 30.0, 10.0),

            # RIGHT ARM (9..16) on can0 (Physical Right Arm)
            9:  RealDamiaoMotorState(9,  "Right J1 (Shoulder Pitch)", "right", 1, "DM8009", 0x01, 0x11, self.can_right_if, 12.5, 45.0, 54.0),
            10: RealDamiaoMotorState(10, "Right J2 (Shoulder Roll)",  "right", 2, "DM8009", 0x02, 0x12, self.can_right_if, 12.5, 45.0, 54.0),
            11: RealDamiaoMotorState(11, "Right J3 (Arm Twist)",      "right", 3, "DM4340", 0x03, 0x13, self.can_right_if, 12.5, 10.0, 28.0),
            12: RealDamiaoMotorState(12, "Right J4 (Elbow Pitch)",    "right", 4, "DM4340", 0x04, 0x14, self.can_right_if, 12.5, 10.0, 28.0),
            13: RealDamiaoMotorState(13, "Right J5 (Forearm Twist)",  "right", 5, "DM4310", 0x05, 0x15, self.can_right_if, 12.5, 30.0, 10.0),
            14: RealDamiaoMotorState(14, "Right J6 (Wrist Pitch)",    "right", 6, "DM4310", 0x06, 0x16, self.can_right_if, 12.5, 30.0, 10.0),
            15: RealDamiaoMotorState(15, "Right J7 (Wrist Roll)",     "right", 7, "DM4310", 0x07, 0x17, self.can_right_if, 12.5, 30.0, 10.0),
            16: RealDamiaoMotorState(16, "Right Gripper (J8 Kẹp Ngang)", "right", 8, "DM4310", 0x08, 0x18, self.can_right_if, 12.5, 30.0, 10.0),
        }

    def start(self):
        self.running = True

        for iface in [self.can0_if, self.can1_if]:
            if not iface:
                continue
            try:
                s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                try:
                    s.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
                except Exception:
                    pass
                s.bind((iface,))
                self.socks[iface] = s
                print(f"[Hardware Bridge] Bound to physical SocketCAN interface: {iface}")

                # Start reader thread for this interface
                t = threading.Thread(target=self._rx_loop, args=(iface, s), daemon=True)
                t.start()
            except Exception as e:
                print(f"[Hardware Bridge] Warning: Could not bind to {iface}: {e}")

        # Start periodic telemetry polling thread (25 Hz)
        self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.poll_thread.start()

    def stop(self):
        self.running = False
        for s in self.socks.values():
            try:
                s.close()
            except Exception:
                pass

    def send_frame(self, iface: str, can_id: int, data: bytes):
        sock = self.socks.get(iface) or self.socks.get(self.can0_if)
        if not sock:
            return
        pad = 64 - len(data)
        frame = struct.pack(CANFD_FRAME_FMT, can_id, len(data), 0, 0, data + (b'\x00' * pad))
        try:
            sock.send(frame)
            self.frames_tx += 1
            now = time.time()
            if now - self._last_tx_log > 0.04:
                self._last_tx_log = now
                if len(self.traffic_log) > 100:
                    self.traffic_log.pop(0)
                self.traffic_log.append({
                    "time": now,
                    "id": hex(can_id),
                    "dir": "TX",
                    "dlc": len(data),
                    "hex": data.hex().upper()
                })
        except Exception:
            pass

        # For 8-byte Damiao management commands (0xFC, 0xFD, 0xFE, 0xFB), also transmit Classic CAN 16-byte frame
        if len(data) == 8 and data[-1] in (0xFC, 0xFD, 0xFE, 0xFB):
            try:
                frame_classic = struct.pack(CAN_FRAME_FMT, can_id, 8, data)
                sock.send(frame_classic)
            except Exception:
                pass

    def _poll_loop(self):
        """Periodically query motor status so dashboard displays real angles even at standstill"""
        while self.running:
            try:
                # Query Left arm on can_left_if (can1)
                if self.can_left_if and self.can_left_if in self.socks:
                    for motor_id in range(1, 9):
                        m = self.motors[motor_id]
                        # Damiao management query: 0x7FF [send_id & FF, send_id >> 8, 0xCC, 0, 0, 0, 0, 0]
                        query_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
                        self.send_frame(m.can_if, 0x7FF, query_data)

                # Query Right arm on can_right_if (can0)
                if self.can_right_if and self.can_right_if in self.socks:
                    for motor_id in range(9, 17):
                        m = self.motors[motor_id]
                        query_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
                        self.send_frame(m.can_if, 0x7FF, query_data)

            except Exception as e:
                pass
            time.sleep(0.04) # 25 Hz

    def _rx_loop(self, iface: str, sock: socket.socket):
        """Continuously receive telemetry feedback from physical motors"""
        while self.running:
            try:
                raw_frame = sock.recv(72)
                if not raw_frame:
                    continue

                if len(raw_frame) == 72:
                    can_id, dlc, flags, res, data_64 = struct.unpack(CANFD_FRAME_FMT, raw_frame)
                    data = data_64[:dlc]
                elif len(raw_frame) == 16:
                    can_id, dlc, data_8 = struct.unpack(CAN_FRAME_FMT, raw_frame)
                    data = data_8[:dlc]
                else:
                    continue

                can_id = can_id & 0x1FFFFFFF
                self.frames_rx += 1

                now = time.time()
                if now - self._last_rx_log > 0.04:
                    self._last_rx_log = now
                    if len(self.traffic_log) > 100:
                        self.traffic_log.pop(0)
                    self.traffic_log.append({
                        "time": now,
                        "id": hex(can_id),
                        "dir": "RX",
                        "dlc": dlc,
                        "hex": data.hex().upper()
                    })

                if len(data) >= 8:
                    self._decode_feedback(iface, can_id, data)

            except Exception:
                if not self.running:
                    break

    def _decode_feedback(self, iface: str, can_id: int, data: bytes):
        """Decode Damiao State Feedback Frame (D[0]..D[7])"""
        d0, d1, d2, d3, d4, d5, d6, d7 = data[:8]

        error_code = (d0 >> 4) & 0x0F
        slave_id = d0 & 0x0F

        # Match motor:
        # can_left_if (can1) -> Physical Left Arm (offset 0, motors 1..8)
        # can_right_if (can0) -> Physical Right Arm (offset 8, motors 9..16)
        is_left_arm = (iface == self.can_left_if)
        offset = 0 if is_left_arm else 8

        # Slave ID is 1..8
        motor_idx = slave_id if (1 <= slave_id <= 8) else (can_id - 0x10)
        motor_id = offset + motor_idx

        motor = self.motors.get(motor_id)
        if not motor:
            return

        with self.lock:
            motor.error_code = error_code
            if error_code >= 8:
                motor.enabled = False
            elif error_code == 1:
                motor.enabled = True

            q_uint = (d1 << 8) | d2
            dq_uint = (d3 << 4) | (d4 >> 4)
            tau_uint = ((d4 & 0x0F) << 8) | d5

            motor.q = uint_to_double(q_uint, -motor.pMax, motor.pMax, 16)
            motor.dq = uint_to_double(dq_uint, -motor.vMax, motor.vMax, 12)
            motor.tau = uint_to_double(tau_uint, -motor.tMax, motor.tMax, 12)
            motor.t_mos = float(d6)
            motor.t_rotor = float(d7)
            motor.last_update = time.time()


class OpenArmDashboardServer:
    def __init__(self, mode: str = "real", can0_if: str = "can0", can1_if: Optional[str] = "can1"):
        self.mode = mode
        self.can0_if = can0_if
        self.can1_if = can1_if
        self.clients = set()
        self.running = True
        self.velocity_limit = 0.25 # rad/s (~14°/s) gentle & safe velocity limit

        if self.mode == "real":
            print(f"[Dashboard] Initializing in REAL ROBOT HARDWARE MODE on {can0_if} / {can1_if}")
            self.hw = RealRobotHardwareBridge(can0_if, can1_if)
            self.motors = self.hw.motors
        else:
            print(f"[Dashboard] Initializing in SIMULATION MODE on vcan0")
            self.hw = DamiaoArmSimulator("vcan0")
            self.motors = self.hw.motors

        # Initialize smooth command states
        for m in self.motors.values():
            m.q_target = 0.0
            m.q_cmd = 0.0
            m.kp = 18.0
            m.kd = 2.0

    def start(self):
        # 1. Start hardware bridge or simulator
        self.hw.start()

        # 2. Start smooth trajectory generator thread (400 Hz)
        self.traj_thread = threading.Thread(target=self._trajectory_loop, daemon=True)
        self.traj_thread.start()

        # 3. Start auto-hotplug interface monitor
        self.hotplug_thread = threading.Thread(target=self._hotplug_monitor_loop, daemon=True)
        self.hotplug_thread.start()

        # 4. Start HTTP server thread
        self.httpd = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), CustomHTTPHandler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        print(f"[Dashboard] HTTP Server running on http://localhost:{HTTP_PORT}")

        # 5. Start WebSocket & Telemetry Broadcaster
        asyncio.run(self.run_ws_server())

    def _hotplug_monitor_loop(self):
        """Continuously check for physical CAN hotplug (can0/can1) without needing restart"""
        while self.running:
            time.sleep(2.0)
            can0_present = os.path.exists("/sys/class/net/can0")
            if self.mode == "sim" and can0_present:
                print("[Hotplug] Detected physical can0 interface! Switching to REAL ROBOT HARDWARE MODE...")
                try:
                    new_hw = RealRobotHardwareBridge(self.can0_if, self.can1_if)
                    new_hw.start()
                    self.hw.stop()
                    self.hw = new_hw
                    self.motors = self.hw.motors
                    self.mode = "real"
                    print(f"[Hotplug] Switched to REAL HARDWARE MODE ({len(self.motors)} motors on {self.can0_if}/{self.can1_if})")
                except Exception as e:
                    print(f"[Hotplug] Error switching to real mode: {e}")
            elif self.mode == "real" and not can0_present:
                print("[Hotplug] Physical can0 detached. Falling back to SIMULATION MODE...")
                try:
                    self.hw.stop()
                    new_sim = DamiaoArmSimulator("vcan0")
                    new_sim.start()
                    self.hw = new_sim
                    self.motors = self.hw.motors
                    self.mode = "sim"
                    print("[Hotplug] Fallback to SIMULATION MODE active")
                except Exception as e:
                    print(f"[Hotplug] Error falling back to sim mode: {e}")

    async def ws_handler(self, websocket):
        self.clients.add(websocket)
        try:
            async for msg in websocket:
                try:
                    payload = json.loads(msg)
                    action = payload.get("action")
                    await self.handle_action(action, payload, websocket)
                except Exception as e:
                    print("[WS Action Error]:", e)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)

    async def handle_action(self, action: str, payload: dict, ws):
        if action == "set_velocity_limit":
            val = float(payload.get("v_limit", 0.25))
            self.velocity_limit = max(0.02, min(3.0, val))
            print(f"[Speed Profile] Velocity limit set to: {self.velocity_limit:.3f} rad/s ({math.degrees(self.velocity_limit):.1f}°/s)")

        elif action == "enable_all":
            print("[Command] Enable All Motors")
            # Sync target and commanded positions with current physical reading to prevent jerking
            with self.hw.lock:
                for m in self.motors.values():
                    m.enabled = True
                    m.error_code = 1
                    m.q_cmd = m.q
                    m.q_target = m.q
                    m.q_des = m.q
            if self.mode == "real":
                for m in self.motors.values():
                    self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFC]))
            else:
                for m in self.hw.motors.values():
                    m.enabled = True
                    m.error_code = 1
                    self.hw._send_can(m.send_id, bytes([0xFF]*7 + [0xFC]))

        elif action == "disable_all":
            print("[Command] Disarm / Disable All Motors")
            with self.hw.lock:
                for m in self.motors.values():
                    m.enabled = False
                    m.error_code = 0
                    m.q_cmd = m.q
                    m.q_target = m.q
            if self.mode == "real":
                for m in self.motors.values():
                    self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFD]))
            else:
                for m in self.hw.motors.values():
                    m.enabled = False
                    m.error_code = 0
                    self.hw._send_can(m.send_id, bytes([0xFF]*7 + [0xFD]))

        elif action == "enable_motor":
            motor_id = int(payload.get("id", 1))
            m = self.motors.get(motor_id)
            if m:
                with self.hw.lock:
                    m.enabled = True
                    m.error_code = 1
                    m.q_cmd = m.q
                    m.q_target = m.q
                    m.q_des = m.q
                if self.mode == "real":
                    self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFC]))
                else:
                    m.enabled = True
                    m.error_code = 1
                    self.hw._send_can(m.send_id, bytes([0xFF]*7 + [0xFC]))

        elif action == "disable_motor":
            motor_id = int(payload.get("id", 1))
            m = self.motors.get(motor_id)
            if m:
                with self.hw.lock:
                    m.enabled = False
                    m.error_code = 0
                    m.q_cmd = m.q
                    m.q_target = m.q
                if self.mode == "real":
                    self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFD]))
                else:
                    m.enabled = False
                    m.error_code = 0
                    self.hw._send_can(m.send_id, bytes([0xFF]*7 + [0xFD]))

        elif action == "set_zero_all":
            print("[Command] Set Zero All")
            if self.mode == "real":
                threading.Thread(target=self._exec_set_zero_all, daemon=True).start()
            else:
                for m in self.hw.motors.values():
                    m.q = 0.0
                    m.dq = 0.0
                    m.q_des = 0.0
                    self.hw._send_can(m.send_id, bytes([0xFF]*7 + [0xFE]))

        elif action == "set_zero_single":
            motor_id = int(payload.get("id", 1))
            m = self.motors.get(motor_id)
            if m:
                print(f"[Command] Set Zero Single Motor {motor_id}")
                if self.mode == "real":
                    threading.Thread(target=self._exec_set_zero_single, args=(m,), daemon=True).start()
                else:
                    m.q = 0.0
                    m.dq = 0.0
                    m.q_des = 0.0

        elif action == "clear_error_all":
            print("[Command] Clear Errors")
            if self.mode == "real":
                for m in self.motors.values():
                    self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFB]))
            else:
                for m in self.hw.motors.values():
                    m.error_code = 0
                    self.hw._send_can(m.send_id, bytes([0xFF]*7 + [0xFB]))

        elif action == "set_mit":
            motor_id = int(payload.get("id", 1))
            q_raw = float(payload.get("q", 0.0))
            kp = float(payload.get("kp", 18.0))
            kd = float(payload.get("kd", 2.0))
            tau = float(payload.get("tau", 0.0))

            motor = self.motors.get(motor_id)
            if motor:
                lim = JOINT_LIMITS.get(motor_id, (-12.5, 12.5))
                q = max(lim[0], min(lim[1], q_raw))
                if not motor.enabled:
                    motor.enabled = True
                    motor.q_cmd = motor.q
                    if self.mode == "real":
                        self.hw.send_frame(motor.can_if, motor.send_id, bytes([0xFF]*7 + [0xFC]))
                motor.q_target = q
                motor.kp = kp
                motor.kd = kd
                motor.tau_ff = tau

        elif action == "set_gripper":
            pos_raw = float(payload.get("pos", 0.0))
            # Support both meters (0.0 .. 0.043 m) and mm (0.0 .. 43.0 mm)
            if pos_raw > 0.043 and pos_raw <= 43.0:
                pos_m = pos_raw / 1000.0
            else:
                pos_m = pos_raw
            pos = max(0.0, min(0.043, pos_m)) # Clamp 0.0 .. 0.043 m (thanh kẹp ngang)
            target_arm = payload.get("arm", "both")
            target_id = payload.get("id")

            target_motors = []
            if target_id is not None:
                m = self.motors.get(int(target_id))
                if m: target_motors.append(m)
            elif target_arm == "left":
                m = self.motors.get(8)
                if m: target_motors.append(m)
            elif target_arm == "right":
                m = self.motors.get(16)
                if m: target_motors.append(m)
            else:
                for gid in [8, 16]:
                    m = self.motors.get(gid)
                    if m: target_motors.append(m)

            for m in target_motors:
                if not m.enabled:
                    m.enabled = True
                    m.q_cmd = m.q
                    if self.mode == "real":
                        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFC]))
                m.q_target = pos

        elif action == "run_cli":
            cmd = payload.get("cmd", "")
            target_iface = self.can0_if if self.mode == "real" else "vcan0"
            threading.Thread(target=self._exec_cli, args=(cmd, target_iface), daemon=True).start()

    def _trajectory_loop(self):
        """400 Hz trajectory generator & control loop that smoothly moves motors at limited velocity"""
        CONTROL_FREQ = 400.0
        dt = 1.0 / CONTROL_FREQ # 0.0025s (2.5 ms)
        next_tick = time.perf_counter()

        while self.running:
            try:
                for motor_id, m in self.motors.items():
                    if not m.enabled:
                        # When disabled, track physical encoder reading so enabling won't jerk
                        m.q_cmd = m.q
                        m.q_target = m.q
                        m.q_des = m.q
                        continue

                    # Velocity-limited step towards target
                    diff = m.q_target - m.q_cmd
                    # If this is gripper, use gentle speed limit
                    v_lim = min(self.velocity_limit, 0.6) if m.joint_idx == 8 else self.velocity_limit
                    max_step = v_lim * dt

                    if abs(diff) <= max_step:
                        m.q_cmd = m.q_target
                    else:
                        m.q_cmd += math.copysign(max_step, diff)

                    m.q_des = m.q_cmd

                    # If in real mode and motor is enabled, send smooth MIT command
                    if self.mode == "real":
                        q_uint = double_to_uint(m.q_cmd, -m.pMax, m.pMax, 16)
                        dq_uint = double_to_uint(0.0, -m.vMax, m.vMax, 12)
                        kp_uint = double_to_uint(m.kp, 0.0, 500.0, 12)
                        kd_uint = double_to_uint(m.kd, 0.0, 5.0, 12)
                        tau_uint = double_to_uint(0.0, -m.tMax, m.tMax, 12)

                        d0 = (q_uint >> 8) & 0xFF
                        d1 = q_uint & 0xFF
                        d2 = (dq_uint >> 4) & 0xFF
                        d3 = ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)
                        d4 = kp_uint & 0xFF
                        d5 = (kd_uint >> 4) & 0xFF
                        d6 = ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)
                        d7 = tau_uint & 0xFF
                        mit_data = bytes([d0, d1, d2, d3, d4, d5, d6, d7])

                        self.hw.send_frame(m.can_if, m.send_id, mit_data)

                        # If gripper (joint_idx == 8), also send posforce with slow velocity
                        if m.joint_idx == 8:
                            pos_bytes = struct.pack("<f", float(m.q_cmd))
                            vel_uint = int(min(10000, 1.0 * 100)) # 1.0 rad/s gentle speed
                            i_uint = int(min(10000, 0.10 * 10000)) # 0.10 pu current limit
                            posforce_data = pos_bytes + struct.pack("<HH", vel_uint, i_uint)
                            self.hw.send_frame(m.can_if, m.send_id, posforce_data)
                    else:
                        m.q = m.q_cmd

            except Exception:
                pass

            next_tick += dt
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0.0005:
                time.sleep(sleep_time)
            elif sleep_time < -0.05:
                next_tick = time.perf_counter()

    def _exec_set_zero_all(self):
        """Execute true DaMiao Zero Calibration sequence: Disable -> Set Zero -> Disable, then reset state"""
        print("[Hardware] Executing mechanical zero calibration sequence on all motors...")
        # Step 1: Disable all motors (0xFD)
        for m in self.motors.values():
            self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFD]))
        time.sleep(0.1)

        # Step 2: Set Zero command (0xFE)
        for m in self.motors.values():
            self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFE]))
        time.sleep(0.12)

        # Step 3: Disable to confirm (0xFD)
        for m in self.motors.values():
            self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFD]))
        time.sleep(0.08)

        # Step 4: Reset memory state in bridge
        with self.hw.lock:
            for m in self.motors.values():
                m.q = 0.0
                m.q_cmd = 0.0
                m.q_target = 0.0
                m.dq = 0.0
                m.q_des = 0.0
                m.tau = 0.0
        print("[Hardware] Zero calibration complete. All motor positions reset to 0.0 rad.")

    def _exec_set_zero_single(self, m):
        """Execute true DaMiao Zero Calibration sequence for a single motor"""
        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFD]))
        time.sleep(0.1)
        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFE]))
        time.sleep(0.12)
        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFD]))
        time.sleep(0.05)
        with self.hw.lock:
            m.q = 0.0
            m.q_cmd = 0.0
            m.q_target = 0.0
            m.dq = 0.0
            m.q_des = 0.0
            m.tau = 0.0

    def _exec_cli(self, cmd: str, iface: str):
        full_cmd = ["openarm-can-cli", "-i", iface]
        if cmd == "discover":
            full_cmd += ["discover"]
        elif cmd == "show_param":
            full_cmd += ["show_param", "--id", "1"]
        elif cmd == "monitor":
            full_cmd += ["monitor", "--id", "1,2,3,4,5,6,7,8"]
        elif cmd == "diagnose":
            full_cmd += ["diagnose"]
        else:
            return

        try:
            res = subprocess.run(full_cmd, capture_output=True, text=True, timeout=10)
            out = res.stdout if res.stdout else res.stderr
        except Exception as e:
            out = f"Execution error: {e}"

        asyncio.run_coroutine_threadsafe(self.broadcast_cli_output(out), self.loop)

    async def broadcast_cli_output(self, text: str):
        msg = json.dumps({"type": "cli_output", "data": text})
        for ws in list(self.clients):
            try:
                await ws.send(msg)
            except Exception:
                pass

    async def broadcast_telemetry_loop(self):
        traffic_tick = 0
        while self.running:
            if self.clients:
                with self.hw.lock:
                    motors_data = [m.to_dict() for m in self.motors.values()]
                    rx_count = self.hw.frames_rx
                    tx_count = self.hw.frames_tx

                telem_msg = json.dumps({
                    "type": "telemetry",
                    "data": {
                        "motors": motors_data,
                        "frames_rx": rx_count,
                        "frames_tx": tx_count,
                        "mode": self.mode
                    }
                })

                traffic_tick += 1
                traffic_msg = None
                if traffic_tick % 2 == 0:
                    with self.hw.lock:
                        recent_traffic = list(self.hw.traffic_log)
                    traffic_msg = json.dumps({
                        "type": "traffic",
                        "data": recent_traffic
                    })

                dead_clients = set()
                for ws in list(self.clients):
                    try:
                        await ws.send(telem_msg)
                        if traffic_msg:
                            await ws.send(traffic_msg)
                    except Exception:
                        dead_clients.add(ws)

                self.clients.difference_update(dead_clients)

            await asyncio.sleep(0.025) # 40 Hz

    async def run_ws_server(self):
        self.loop = asyncio.get_running_loop()
        server = await websockets.serve(self.ws_handler, "0.0.0.0", WS_PORT)
        print(f"[Dashboard] WebSocket Server running on ws://localhost:{WS_PORT} (Mode: {self.mode.upper()})")
        await self.broadcast_telemetry_loop()


if __name__ == "__main__":
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
    server.start()
