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
        self.has_physical_sync = False

    def to_dict(self):
        # For Gripper (Joint 8): output linear stroke in meters (0.0 .. 0.043) and mm
        if self.joint_idx == 8:
            stroke_m = max(0.0, min(0.043, (abs(self.q) / 1.20) * 0.043))
            q_val = round(stroke_m, 4)
            q_deg_val = round(stroke_m * 1000.0, 1) # displayed as mm
        else:
            q_val = round(self.q, 4)
            q_deg_val = round(math.degrees(self.q), 1)

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
            "stroke_mm": round((abs(self.q) / 1.20) * 43.0, 1) if self.joint_idx == 8 else None,
            "q_rad": round(self.q, 4),
            "dq": round(self.dq, 4),
            "tau": round(self.tau, 3),
            "t_mos": round(self.t_mos, 1),
            "t_rotor": round(self.t_rotor, 1),
            "q_des": round(self.q_des, 4),
            "kp": round(self.kp, 1),
            "kd": round(self.kd, 2),
            "has_sync": self.has_physical_sync
        }


# Mechanical Joint Limits defined for OpenArm 7-DOF + Gripper
JOINT_LIMITS = {
    # Left Arm (IDs 1..7) & Left Gripper (ID 8)
    1: (-1.3963, 3.4907),
    2: (-3.3161, 0.17453), # Left J2: Shoulder Roll (-3.3161 .. 0.17453 rad)
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

        # Also transmit Classic CAN frame for any 8-byte management, query, or control frame
        # to ensure compatibility whether the motor is operating in CAN-FD or Classic CAN
        if len(data) == 8:
            try:
                frame_classic = struct.pack(CAN_FRAME_FMT, can_id, 8, data)
                sock.send(frame_classic)
            except Exception:
                pass

    def query_all_physical(self):
        """Immediately broadcast state query frames (0xCC) to all 16 physical motors"""
        for m in self.motors.values():
            query_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
            self.send_frame(m.can_if, 0x7FF, query_data)

    def _poll_loop(self):
        """Periodically query motor status so dashboard displays real angles even at standstill"""
        # Initial burst query upon thread startup
        time.sleep(0.1)
        self.query_all_physical()

        while self.running:
            try:
                self.query_all_physical()
            except Exception:
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
                time.sleep(0.01)

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
            if motor.joint_idx == 8 and error_code >= 8:
                # Motor 8 reached mechanical limit or grasped object (stall flag)
                # Auto-clear fault and keep enabled so user can immediately close/open
                self.send_frame(iface, motor.send_id, bytes([0xFF]*7 + [0xFB]))
                motor.error_code = 1
                motor.enabled = True
            else:
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

            # First time receiving physical reading: synchronize initial target/command positions
            if not motor.has_physical_sync:
                motor.has_physical_sync = True
                motor.q_target = motor.q
                motor.q_cmd = motor.q
                motor.q_des = motor.q
                print(f"[Physical Sync] Motor {motor.id} ({motor.name}) synced real angle: {motor.q:.4f} rad ({math.degrees(motor.q):.1f}°)")


class OpenArmDashboardServer:
    def __init__(self, mode: str = "real", can0_if: str = "can0", can1_if: Optional[str] = "can1"):
        self.mode = mode
        self.can0_if = can0_if
        self.can1_if = can1_if
        self.clients = set()
        self.running = True
        self.velocity_limit = 0.25 # rad/s (~14°/s) gentle & safe velocity limit
        self.gripper_invert = {8: False, 16: False} # Direction invert flag if needed

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

    def _find_can_usb(self):
        try:
            # 1. Try direct usbipd state (fast JSON)
            res = subprocess.run(["usbipd", "state"], capture_output=True, text=True, timeout=3)
            if res.returncode == 0 and res.stdout.strip():
                data = json.loads(res.stdout)
                devices = data.get("Devices", [])
                for d in devices:
                    desc = d.get("Description", "") or ""
                    inst = d.get("InstanceId", "") or ""
                    busid = d.get("BusId")
                    if ('PCAN' in desc.upper() or '0C72' in inst.upper() or 'CAN' in desc.upper()) and busid:
                        return str(busid), desc
        except Exception as e:
            print("[USB Search Error]:", e)

        # 2. Fallback to parsing usbipd list
        try:
            res = subprocess.run(["usbipd", "list"], capture_output=True, text=True, timeout=3)
            for line in res.stdout.splitlines():
                if "PCAN" in line.upper() or "0C72:0011" in line.lower():
                    parts = line.split()
                    if parts and '-' in parts[0]:
                        return parts[0], "PCAN-USB Pro FD"
        except Exception as e:
            pass

        return None, None

    def _exec_connect_usb(self):
        try:
            print("[USB] Bắt đầu kết nối USB Robot...")
            if hasattr(self, 'loop') and self.loop:
                asyncio.run_coroutine_threadsafe(self.broadcast_notice("info", "Đang quét cổng USB Robot..."), self.loop)

            can0_present = os.path.exists("/sys/class/net/can0")
            busid = None
            desc = "PCAN-USB Pro FD"

            if not can0_present:
                busid, found_desc = self._find_can_usb()
                if found_desc:
                    desc = found_desc
                if not busid:
                    msg = "Chưa phát hiện thiết bị USB CAN cắm trên máy tính. Vui lòng cắm cáp USB nối đến robot."
                    print(f"[USB Error] {msg}")
                    if hasattr(self, 'loop') and self.loop:
                        asyncio.run_coroutine_threadsafe(self.broadcast_notice("error", msg), self.loop)
                    return

                print(f"[USB] Đang gắn thiết bị USB (BusID: {busid}, {desc}) vào WSL2...")
                if hasattr(self, 'loop') and self.loop:
                    asyncio.run_coroutine_threadsafe(self.broadcast_notice("info", f"Đang gắn USB {busid} ({desc}) vào WSL2..."), self.loop)

                # Attach via usbipd
                att_res = subprocess.run(["usbipd", "attach", "--wsl", "--busid", busid], capture_output=True, text=True, timeout=8)
                if att_res.returncode != 0 and "already attached" not in att_res.stderr.lower() and "already attached" not in att_res.stdout.lower():
                    print(f"[USB] Thử detach rồi attach lại: {att_res.stderr.strip() or att_res.stdout.strip()}")
                    subprocess.run(["usbipd", "detach", "--busid", busid], capture_output=True, text=True, timeout=5)
                    time.sleep(0.5)
                    subprocess.run(["usbipd", "attach", "--wsl", "--busid", busid], capture_output=True, text=True, timeout=8)

                # Load required kernel modules
                for mod in ["vhci-hcd", "can", "can-raw", "can-dev", "peak_usb"]:
                    subprocess.run(["sudo", "modprobe", mod], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                # Wait for can0 to appear in /sys/class/net
                for _ in range(12):
                    if os.path.exists("/sys/class/net/can0"):
                        break
                    time.sleep(0.3)

            # Bring up CAN-FD on can0 and can1
            configured_any = False
            for iface in ["can0", "can1"]:
                if os.path.exists(f"/sys/class/net/{iface}"):
                    subprocess.run(["sudo", "ip", "link", "set", iface, "down"], check=False)
                    cmd = [
                        "sudo", "ip", "link", "set", iface, "type", "can",
                        "bitrate", "1000000", "sample-point", "0.75",
                        "dbitrate", "5000000", "dsample-point", "0.75",
                        "dsjw", "2", "fd", "on"
                    ]
                    r = subprocess.run(cmd, check=False)
                    if r.returncode != 0:
                        subprocess.run(["sudo", "ip", "link", "set", iface, "type", "can", "bitrate", "1000000"], check=False)
                    subprocess.run(["sudo", "ip", "link", "set", iface, "up"], check=False)
                    configured_any = True

            if configured_any:
                time.sleep(0.3)
                # ALWAYS recreate hardware bridge so fresh SocketCAN file descriptors are bound
                try:
                    self.hw.stop()
                    new_hw = RealRobotHardwareBridge(self.can0_if, self.can1_if)
                    new_hw.start()
                    self.hw = new_hw
                    self.motors = self.hw.motors
                    self.mode = "real"
                    print(f"[USB] Kết nối thành công! Đang ở chế độ REAL ROBOT HARDWARE MODE ({self.can0_if}/{self.can1_if})")
                    # Query all physical motor positions immediately
                    self.hw.query_all_physical()
                except Exception as e:
                    print(f"[USB Switch Error]: {e}")

                dev_info = f" ({desc}, Bus: {busid})" if busid else " (can0/can1)"
                msg = f"Đã kết nối thành công Robot thật qua USB{dev_info}!"
                if hasattr(self, 'loop') and self.loop:
                    asyncio.run_coroutine_threadsafe(self.broadcast_notice("success", msg), self.loop)
            else:
                msg = "Không tìm thấy interface can0/can1 sau khi gắn USB. Vui lòng kiểm tra nguồn và cáp robot."
                print(f"[USB Warning] {msg}")
                if hasattr(self, 'loop') and self.loop:
                    asyncio.run_coroutine_threadsafe(self.broadcast_notice("warning", msg), self.loop)

        except Exception as e:
            err_msg = f"Lỗi trong quá trình kết nối USB: {e}"
            print(f"[USB Exception] {err_msg}")
            if hasattr(self, 'loop') and self.loop:
                asyncio.run_coroutine_threadsafe(self.broadcast_notice("error", err_msg), self.loop)

    def _exec_disconnect_usb(self):
        try:
            print("[USB] Ngắt kết nối USB Robot...")
            if hasattr(self, 'loop') and self.loop:
                asyncio.run_coroutine_threadsafe(self.broadcast_notice("info", "Đang ngắt kết nối USB và ngắt torque an toàn..."), self.loop)

            # 1. Disarm all motors safely first
            with self.hw.lock:
                for m in self.motors.values():
                    m.enabled = False
                    m.error_code = 0
            if self.mode == "real":
                for m in self.motors.values():
                    self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFD]))

            time.sleep(0.3)

            # 2. Down CAN interfaces
            for iface in ["can0", "can1"]:
                if os.path.exists(f"/sys/class/net/{iface}"):
                    subprocess.run(["sudo", "ip", "link", "set", iface, "down"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            # 3. Detach via usbipd
            busid, _ = self._find_can_usb()
            if busid:
                subprocess.run(["usbipd", "detach", "--busid", busid], check=False, timeout=5)

            # 4. Fallback to SIM
            try:
                self.hw.stop()
                new_sim = DamiaoArmSimulator("vcan0")
                new_sim.start()
                self.hw = new_sim
                self.motors = self.hw.motors
                self.mode = "sim"
                print("[USB] Đã ngắt kết nối robot thật. Chuyển sang SIMULATION MODE (vcan0)")
            except Exception as e:
                print(f"[USB Fallback Error]: {e}")

            if hasattr(self, 'loop') and self.loop:
                asyncio.run_coroutine_threadsafe(self.broadcast_notice("warning", "Đã ngắt kết nối USB Robot. Hệ thống đang ở chế độ Mô phỏng (SIM)."), self.loop)
        except Exception as e:
            print(f"[USB Disconnect Error]: {e}")
            if hasattr(self, 'loop') and self.loop:
                asyncio.run_coroutine_threadsafe(self.broadcast_notice("error", f"Lỗi ngắt kết nối: {e}"), self.loop)

    async def broadcast_notice(self, level: str, message: str):
        msg = json.dumps({
            "type": "notice",
            "level": level,
            "message": message,
            "mode": self.mode
        })
        for ws in list(self.clients):
            try:
                await ws.send(msg)
            except Exception:
                pass

    def _hotplug_monitor_loop(self):
        """Continuously check for physical CAN hotplug (can0/can1) without needing restart"""
        while self.running:
            time.sleep(2.0)
            can0_present = os.path.exists("/sys/class/net/can0")
            if self.mode == "sim" and can0_present:
                print("[Hotplug] Detected physical can0 interface! Switching to REAL ROBOT HARDWARE MODE...")
                try:
                    for iface in ["can0", "can1"]:
                        if os.path.exists(f"/sys/class/net/{iface}"):
                            subprocess.run(["sudo", "ip", "link", "set", iface, "type", "can",
                                            "bitrate", "1000000", "sample-point", "0.75",
                                            "dbitrate", "5000000", "dsample-point", "0.75",
                                            "dsjw", "2", "fd", "on"], check=False)
                            subprocess.run(["sudo", "ip", "link", "set", iface, "up"], check=False)
                    new_hw = RealRobotHardwareBridge(self.can0_if, self.can1_if)
                    new_hw.start()
                    self.hw.stop()
                    self.hw = new_hw
                    self.motors = self.hw.motors
                    self.mode = "real"
                    print(f"[Hotplug] Switched to REAL HARDWARE MODE ({len(self.motors)} motors on {self.can0_if}/{self.can1_if})")
                    self.hw.query_all_physical()
                    if hasattr(self, 'loop') and self.loop:
                        asyncio.run_coroutine_threadsafe(self.broadcast_notice("success", "Đã tự động kết nối robot thật trên can0/can1!"), self.loop)
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
                    if hasattr(self, 'loop') and self.loop:
                        asyncio.run_coroutine_threadsafe(self.broadcast_notice("warning", "Cáp USB đã tháo. Đang hoạt động ở chế độ Mô phỏng (vcan0)"), self.loop)
                except Exception as e:
                    print(f"[Hotplug] Error falling back to sim mode: {e}")

    async def ws_handler(self, websocket):
        self.clients.add(websocket)
        # Send initial state immediately upon connection so frontend sliders & 3D model synchronize
        try:
            with self.hw.lock:
                motors_data = [m.to_dict() for m in self.motors.values()]
                rx_count = self.hw.frames_rx
                tx_count = self.hw.frames_tx
            init_msg = json.dumps({
                "type": "telemetry",
                "data": {
                    "motors": motors_data,
                    "frames_rx": rx_count,
                    "frames_tx": tx_count,
                    "mode": self.mode,
                    "initial": True
                }
            })
            await websocket.send(init_msg)
        except Exception:
            pass

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
        if action == "connect_usb":
            print("[Command] Connect USB Robot requested from Web UI")
            threading.Thread(target=self._exec_connect_usb, daemon=True).start()

        elif action == "disconnect_usb":
            print("[Command] Disconnect USB Robot requested from Web UI")
            threading.Thread(target=self._exec_disconnect_usb, daemon=True).start()

        elif action == "sync_robot_state":
            print("[Command] Sync state from physical robot requested")
            if self.mode == "real":
                self.hw.query_all_physical()
                time.sleep(0.08)
                with self.hw.lock:
                    for m in self.motors.values():
                        m.q_cmd = m.q
                        m.q_target = m.q
                        m.q_des = m.q
                        m.has_physical_sync = True
                if hasattr(self, 'loop') and self.loop:
                    asyncio.run_coroutine_threadsafe(self.broadcast_notice("success", "Đã đọc và đồng bộ góc khớp thực tế từ Robot!"), self.loop)

        elif action == "set_velocity_limit":
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
                    if m.joint_idx == 8:
                        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFB]))
                        time.sleep(0.01)
                        set_mode_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0x55, 10, 4, 0, 0, 0])
                        self.hw.send_frame(m.can_if, 0x7FF, set_mode_data)
                        time.sleep(0.02)
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
                    if m.joint_idx == 8:
                        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFB]))
                        time.sleep(0.01)
                        set_mode_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0x55, 10, 4, 0, 0, 0])
                        self.hw.send_frame(m.can_if, 0x7FF, set_mode_data)
                        time.sleep(0.02)
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

            # If Joint 8 received via set_mit, route to set_gripper logic
            if motor_id in [8, 16]:
                m = self.motors.get(motor_id)
                if m:
                    pos_m = q_raw / 1000.0 if (q_raw > 0.043 and q_raw <= 43.0) else q_raw
                    pos = max(0.0, min(0.043, pos_m))
                    invert = self.gripper_invert.get(m.id, False)
                    stroke_ratio = pos / 0.043
                    ratio = (1.0 - stroke_ratio) if invert else stroke_ratio
                    # On OpenArm physical hardware, closing is 0.0 rad, opening is -1.20 rad
                    rad_target = -ratio * 1.20

                    if not m.enabled or m.error_code >= 8:
                        m.enabled = True
                        m.error_code = 1
                        m.q_cmd = m.q
                        if self.mode == "real":
                            self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFB]))
                            time.sleep(0.01)
                            set_mode_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0x55, 10, 4, 0, 0, 0])
                            self.hw.send_frame(m.can_if, 0x7FF, set_mode_data)
                            time.sleep(0.02)
                            self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFC]))
                            time.sleep(0.01)

                    m.q_target = rad_target
                    m.q_cmd = rad_target
                    if self.mode == "real":
                        posforce_can_id = m.send_id + 0x300
                        vel_uint = 2500  # 25.0 rad/s
                        i_uint = 1500    # 0.15 pu
                        posforce_data = struct.pack("<fHH", float(rad_target), vel_uint, i_uint)
                        self.hw.send_frame(m.can_if, posforce_can_id, posforce_data)
                        refresh_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
                        self.hw.send_frame(m.can_if, 0x7FF, refresh_data)
                return

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
                # Convert linear stroke (0.0 .. 0.043 m) to motor target angle in radians (0.0 .. -1.20 rad)
                invert = self.gripper_invert.get(m.id, False)
                stroke_ratio = pos / 0.043
                ratio = (1.0 - stroke_ratio) if invert else stroke_ratio
                # On OpenArm physical hardware, closing is 0.0 rad, opening is -1.20 rad
                rad_target = -ratio * 1.20

                if not m.enabled or m.error_code >= 8:
                    m.enabled = True
                    m.error_code = 1
                    m.q_cmd = m.q
                    if self.mode == "real":
                        # 1. Clear any active motor fault (stall/overload)
                        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFB]))
                        time.sleep(0.01)
                        # 2. Ensure motor is in POS_FORCE control mode (RID 10 = 4)
                        set_mode_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0x55, 10, 4, 0, 0, 0])
                        self.hw.send_frame(m.can_if, 0x7FF, set_mode_data)
                        time.sleep(0.02)
                        # 3. Enable motor
                        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF]*7 + [0xFC]))
                        time.sleep(0.01)

                m.q_target = rad_target
                m.q_cmd = rad_target
                if self.mode == "real":
                    posforce_can_id = m.send_id + 0x300
                    vel_uint = 2500  # 25.0 rad/s
                    i_uint = 1500    # 0.15 pu safe current limit
                    posforce_data = struct.pack("<fHH", float(rad_target), vel_uint, i_uint)
                    self.hw.send_frame(m.can_if, posforce_can_id, posforce_data)
                    refresh_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
                    self.hw.send_frame(m.can_if, 0x7FF, refresh_data)

                can_name = getattr(m, 'can_if', 'vcan0')
                print(f"[Gripper] Motor {m.id} ({m.name}) on {can_name} target -> stroke {pos*1000:.1f} mm ({rad_target:.3f} rad)")

        elif action == "toggle_gripper_invert":
            target_id = int(payload.get("id", 8))
            self.gripper_invert[target_id] = not self.gripper_invert.get(target_id, False)
            print(f"[Gripper] Motor {target_id} invert set to: {self.gripper_invert[target_id]}")

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
                    # If this is gripper, allow fast responsive travel up to 2.5 rad/s
                    v_lim = 2.5 if m.joint_idx == 8 else self.velocity_limit
                    max_step = v_lim * dt

                    if abs(diff) <= max_step:
                        m.q_cmd = m.q_target
                    else:
                        m.q_cmd += math.copysign(max_step, diff)

                    m.q_des = m.q_cmd

                    # If in real mode and motor is enabled, send CAN command
                    if self.mode == "real":
                        if m.joint_idx == 8:
                            # -------------------------------------------------------------
                            # JOINT 8: END-EFFECTOR PARALLEL GRIPPER (DM4310 in POS_FORCE)
                            # -------------------------------------------------------------
                            # Periodic keep-alive and telemetry refresh at 20 Hz (every 50 ms)
                            now = time.time()
                            if now - getattr(m, '_last_posforce_tx', 0) > 0.05:
                                m._last_posforce_tx = now
                                posforce_can_id = m.send_id + 0x300
                                vel_uint = 2500  # 25.0 rad/s
                                i_uint = 1500    # 0.15 pu
                                posforce_data = struct.pack("<fHH", float(m.q_target), vel_uint, i_uint)
                                self.hw.send_frame(m.can_if, posforce_can_id, posforce_data)
                                refresh_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
                                self.hw.send_frame(m.can_if, 0x7FF, refresh_data)
                        else:
                            # -------------------------------------------------------------
                            # JOINTS 1..7: 7-DOF ARM MOTORS (MIT MODE)
                            # -------------------------------------------------------------
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
