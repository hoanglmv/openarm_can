#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
Physical CAN-FD / SocketCAN Hardware Bridge for Dual OpenArm 7-DOF + Grippers.
Handles real-time transmission, reception, feedback decoding, and fault recovery.
"""

import math
import socket
import struct
import threading
import time
from typing import Dict, List, Optional

try:
    from config import (
        CANFD_FRAME_FMT,
        CAN_FRAME_FMT,
        CAN_RAW_FD_FRAMES,
        MOTOR_DIRECTIONS,
        SOL_CAN_RAW,
        uint_to_double,
    )
    from models import RealDamiaoMotorState
except ImportError:
    from .config import (
        CANFD_FRAME_FMT,
        CAN_FRAME_FMT,
        CAN_RAW_FD_FRAMES,
        MOTOR_DIRECTIONS,
        SOL_CAN_RAW,
        uint_to_double,
    )
    from .models import RealDamiaoMotorState


class RealRobotHardwareBridge:
    """SocketCAN hardware bridge connecting physical CAN buses (can0/can1) to OpenArm actuators."""

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
            1: RealDamiaoMotorState(1, "Left J1 (Shoulder Pitch)", "left", 1, "DM8009", 0x01, 0x11, self.can_left_if, 12.5, 45.0, 54.0, direction=MOTOR_DIRECTIONS.get(1, -1.0)),
            2: RealDamiaoMotorState(2, "Left J2 (Shoulder Roll)",  "left", 2, "DM8009", 0x02, 0x12, self.can_left_if, 12.5, 45.0, 54.0, direction=MOTOR_DIRECTIONS.get(2, 1.0)),
            3: RealDamiaoMotorState(3, "Left J3 (Arm Twist)",      "left", 3, "DM4340", 0x03, 0x13, self.can_left_if, 12.5, 10.0, 28.0, direction=MOTOR_DIRECTIONS.get(3, 1.0)),
            4: RealDamiaoMotorState(4, "Left J4 (Elbow Pitch)",    "left", 4, "DM4340", 0x04, 0x14, self.can_left_if, 12.5, 10.0, 28.0, direction=MOTOR_DIRECTIONS.get(4, 1.0)),
            5: RealDamiaoMotorState(5, "Left J5 (Forearm Twist)",  "left", 5, "DM4310", 0x05, 0x15, self.can_left_if, 12.5, 30.0, 10.0, direction=MOTOR_DIRECTIONS.get(5, 1.0)),
            6: RealDamiaoMotorState(6, "Left J6 (Wrist Pitch)",    "left", 6, "DM4310", 0x06, 0x16, self.can_left_if, 12.5, 30.0, 10.0, direction=MOTOR_DIRECTIONS.get(6, 1.0)),
            7: RealDamiaoMotorState(7, "Left J7 (Wrist Roll)",     "left", 7, "DM4310", 0x07, 0x17, self.can_left_if, 12.5, 30.0, 10.0, direction=MOTOR_DIRECTIONS.get(7, 1.0)),
            8: RealDamiaoMotorState(8, "Left Gripper (J8 Kẹp Ngang)", "left", 8, "DM4310", 0x08, 0x18, self.can_left_if, 12.5, 30.0, 10.0, direction=MOTOR_DIRECTIONS.get(8, 1.0)),

            # RIGHT ARM (9..16) on can0 (Physical Right Arm)
            9:  RealDamiaoMotorState(9,  "Right J1 (Shoulder Pitch)", "right", 1, "DM8009", 0x01, 0x11, self.can_right_if, 12.5, 45.0, 54.0, direction=MOTOR_DIRECTIONS.get(9, 1.0)),
            10: RealDamiaoMotorState(10, "Right J2 (Shoulder Roll)",  "right", 2, "DM8009", 0x02, 0x12, self.can_right_if, 12.5, 45.0, 54.0, direction=MOTOR_DIRECTIONS.get(10, 1.0)),
            11: RealDamiaoMotorState(11, "Right J3 (Arm Twist)",      "right", 3, "DM4340", 0x03, 0x13, self.can_right_if, 12.5, 10.0, 28.0, direction=MOTOR_DIRECTIONS.get(11, 1.0)),
            12: RealDamiaoMotorState(12, "Right J4 (Elbow Pitch)",    "right", 4, "DM4340", 0x04, 0x14, self.can_right_if, 12.5, 10.0, 28.0, direction=MOTOR_DIRECTIONS.get(12, 1.0)),
            13: RealDamiaoMotorState(13, "Right J5 (Forearm Twist)",  "right", 5, "DM4310", 0x05, 0x15, self.can_right_if, 12.5, 30.0, 10.0, direction=MOTOR_DIRECTIONS.get(13, 1.0)),
            14: RealDamiaoMotorState(14, "Right J6 (Wrist Pitch)",    "right", 6, "DM4310", 0x06, 0x16, self.can_right_if, 12.5, 30.0, 10.0, direction=MOTOR_DIRECTIONS.get(14, 1.0)),
            15: RealDamiaoMotorState(15, "Right J7 (Wrist Roll)",     "right", 7, "DM4310", 0x07, 0x17, self.can_right_if, 12.5, 30.0, 10.0, direction=MOTOR_DIRECTIONS.get(15, 1.0)),
            16: RealDamiaoMotorState(16, "Right Gripper (J8 Kẹp Ngang)", "right", 8, "DM4310", 0x08, 0x18, self.can_right_if, 12.5, 30.0, 10.0, direction=MOTOR_DIRECTIONS.get(16, 1.0)),
        }

    def start(self):
        """Bind SocketCAN interfaces and launch reader and telemetry polling threads."""
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

        # Arm gripper motors (Joint 8 on Left Arm, Joint 16 on Right Arm) in POS_FORCE mode
        for gid in [8, 16]:
            try:
                self.init_gripper_motor(gid, save_flash=True)
            except Exception as e:
                print(f"[Hardware Bridge] Warning initializing gripper {gid}: {e}")

    def init_gripper_motor(self, motor_id: int, save_flash: bool = False):
        """
        Configure and enable gripper motor (Joint 8 / Joint 16).
        Clears any hardware faults, sets RID 10 (CTRL_MODE) = 4 (POS_FORCE),
        enables motor output (0xFC), and queries initial position.
        """
        m = self.motors.get(motor_id)
        if not m or m.joint_idx != 8:
            return

        send_id = m.send_id
        iface = m.can_if

        # 1. Clear any fault/stall state (0xFB)
        self.send_frame(iface, send_id, bytes([0xFF] * 7 + [0xFB]))
        time.sleep(0.015)

        # 2. Write register RID 10 (CTRL_MODE) = 4 (POS_FORCE) via management ID 0x7FF
        # Frame format: [send_id_low, send_id_high, 0x55 (write), RID (10), val0 (4), val1, val2, val3]
        mode_write_data = bytes([send_id & 0xFF, (send_id >> 8) & 0xFF, 0x55, 10, 4, 0, 0, 0])
        self.send_frame(iface, 0x7FF, mode_write_data)
        time.sleep(0.015)

        # 3. Enable motor output (0xFC)
        self.send_frame(iface, send_id, bytes([0xFF] * 7 + [0xFC]))
        time.sleep(0.015)

        # 4. Request initial state (0xCC)
        query_data = bytes([send_id & 0xFF, (send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
        self.send_frame(iface, 0x7FF, query_data)

        m.enabled = True
        m.error_code = 1
        m.gripper_ready = True
        print(f"[Hardware Bridge] Gripper Motor {motor_id} ({m.name}) armed in POS_FORCE mode on {iface}")

    def stop(self):
        """Stop reader loops and close SocketCAN sockets."""
        self.running = False
        for s in self.socks.values():
            try:
                s.close()
            except Exception:
                pass

    def send_frame(self, iface: str, can_id: int, data: bytes):
        """Transmit frame via CAN-FD with optional Classic CAN compatibility fallback."""
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
        """Immediately broadcast state query frames (0xCC) to all 16 physical motors."""
        for m in self.motors.values():
            query_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
            self.send_frame(m.can_if, 0x7FF, query_data)

    def _poll_loop(self):
        """Periodically query motor status so dashboard displays real angles even at standstill."""
        time.sleep(0.1)
        self.query_all_physical()

        while self.running:
            try:
                self.query_all_physical()
            except Exception:
                pass
            time.sleep(0.04)  # 25 Hz

    def _rx_loop(self, iface: str, sock: socket.socket):
        """Continuously receive telemetry feedback from physical motors."""
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
        """Decode Damiao State Feedback Frame (D[0]..D[7])."""
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
            if motor.joint_idx == 8:
                if error_code == 1:
                    motor.enabled = True
                elif error_code == 0 and motor.enabled:
                    # Motor was power-cycled / unplugged and replugged!
                    now = time.time()
                    if now - getattr(motor, '_last_rearm', 0) > 1.0:
                        motor._last_rearm = now
                        self.init_gripper_motor(motor.id, save_flash=False)
            else:
                if error_code >= 8:
                    motor.enabled = False
                elif error_code == 1:
                    motor.enabled = True

            q_uint = (d1 << 8) | d2
            dq_uint = (d3 << 4) | (d4 >> 4)
            tau_uint = ((d4 & 0x0F) << 8) | d5

            raw_q = uint_to_double(q_uint, -motor.pMax, motor.pMax, 16)
            raw_dq = uint_to_double(dq_uint, -motor.vMax, motor.vMax, 12)
            raw_tau = uint_to_double(tau_uint, -motor.tMax, motor.tMax, 12)

            motor_dir = getattr(motor, 'direction', 1.0)
            motor.q = raw_q * motor_dir
            motor.dq = raw_dq * motor_dir
            motor.tau = raw_tau * motor_dir
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
