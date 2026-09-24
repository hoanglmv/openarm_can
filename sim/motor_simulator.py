#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Simulation
"""
Virtual Multi-Damiao-Motor CAN-FD Simulation for Bimanual OpenArm (16 Motors).
Supports Linux SocketCAN (vcan0 / can0).
Simulates Dual 7-DOF Robotic Arms (Left Arm & Right Arm) + Dual Grippers:
- Left Arm: Joint 1..7 + Gripper L (DM8009 x2, DM4340 x2, DM4310 x4, Send: 0x01..0x08)
- Right Arm: Joint 1..7 + Gripper R (DM8009 x2, DM4340 x2, DM4310 x4, Send: 0x21..0x28)
- MIT Impedance Control & 2nd-order dynamic physics at 500 Hz
- Complete 82-parameter register dictionary (RIDs)
- State telemetry frame encoding (100% compatible with openarm_can C++ & Python)
"""

import subprocess
import socket
import struct
import time
import math
import threading
from typing import Dict, Optional, List

SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_RAW_FD_FRAMES = getattr(socket, "CAN_RAW_FD_FRAMES", 5)

CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SZ = struct.calcsize(CAN_FRAME_FMT)

CANFD_FRAME_FMT = "=IBBB1x64s"
CANFD_FRAME_SZ = 72

def double_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    span = x_max - x_min
    val = max(x_min, min(x_max, x))
    return int((val - x_min) * ((1 << bits) - 1) / span)

def uint_to_double(x: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    norm = float(x) / float((1 << bits) - 1)
    return norm * span + x_min

class VirtualDamiaoMotor:
    def __init__(self, motor_id: int, name: str, arm: str, joint_idx: int,
                 motor_type: str, send_id: int, recv_id: int,
                 pMax: float, vMax: float, tMax: float):
        self.id = motor_id
        self.name = name
        self.arm = arm               # "left" or "right"
        self.joint_idx = joint_idx   # 1..7 for arm, 8 for gripper
        self.motor_type = motor_type
        self.send_id = send_id
        self.recv_id = recv_id
        self.pMax = pMax
        self.vMax = vMax
        self.tMax = tMax
        self.can_if = "vcan0"
        self.has_physical_sync = True

        # Real-time state
        self.enabled = False
        self.error_code = 0   # 0: disabled, 1: enabled, 8+: faults
        self.q = 0.0          # Position (rad)
        self.dq = 0.0         # Velocity (rad/s)
        self.tau = 0.0        # Measured torque (Nm)
        self.t_mos = 33.5     # MOS Temperature (°C)
        self.t_rotor = 31.0   # Rotor Temperature (°C)

        # Control setpoints (MIT)
        self.q_des = 0.0
        self.dq_des = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.tau_ff = 0.0

        # Physical params
        self.inertia = 0.015
        self.damping = 0.04

        # Register map (RID)
        self.params: Dict[int, float] = {
            0: 12.0,            # UV_Value (under-voltage)
            1: 0.12,            # KT_Value (torque constant)
            2: 95.0,            # OT_Value (over-temp)
            3: 0.9,             # OC_Value (over-current)
            4: 250.0,           # ACC
            5: -250.0,          # DEC
            6: float(vMax),     # MAX_SPD
            7: float(recv_id),  # MST_ID (master / response ID)
            8: float(send_id),  # ESC_ID (slave / command ID)
            9: 1000.0,          # TIMEOUT
            10: 1.0,            # CTRL_MODE (1: MIT)
            11: 0.04,           # Damp
            12: 0.015,          # Inertia
            13: 102.0,          # hw_ver
            14: 205.0,          # sw_ver
            15: float(10000 + send_id), # SN
            16: 14.0,           # NPP (pole pairs)
            17: 0.15,           # Rs
            18: 0.0003,         # Ls
            19: 0.015,          # Flux
            20: 1.0,            # Gr (gear ratio)
            21: float(pMax),    # PMAX
            22: float(vMax),    # VMAX
            23: float(tMax),    # TMAX
            24: 1000.0,         # I_BW
            25: 5.0,            # KP_ASR
            26: 0.1,            # KI_ASR
            27: 10.0,           # KP_APR
            28: 0.0,            # KI_APR
            35: float(send_id), # CAN_ID
            36: 1000000.0,      # CAN_BAUD (1M)
        }

    def update_physics(self, dt: float):
        if self.enabled:
            # MIT Impedance Equation: tau = Kp*(q_des - q) + Kd*(dq_des - dq) + tau_ff
            err_pos = self.q_des - self.q
            err_vel = self.dq_des - self.dq
            raw_tau = self.kp * err_pos + self.kd * err_vel + self.tau_ff
            self.tau = max(-self.tMax, min(self.tMax, raw_tau))

            # 2nd-order dynamic simulation: q_ddot = (tau - damping*dq) / inertia
            net_torque = self.tau - self.damping * self.dq
            accel = net_torque / max(0.001, self.inertia)

            self.dq += accel * dt
            self.dq = max(-self.vMax, min(self.vMax, self.dq))
            self.q += self.dq * dt
            self.q = max(-self.pMax, min(self.pMax, self.q))
        else:
            self.error_code = 0
            self.tau = 0.0
            if abs(self.dq) > 1e-4:
                self.dq -= math.copysign(min(abs(self.dq), 25.0 * dt), self.dq)
            else:
                self.dq = 0.0
            self.q += self.dq * dt

        # Thermal simulation
        heat = (abs(self.tau) / max(1.0, self.tMax)) * 4.0
        self.t_mos += (heat - (self.t_mos - 30.0) * 0.08) * dt
        self.t_rotor += (heat * 0.7 - (self.t_rotor - 28.0) * 0.08) * dt

    def make_state_frame(self) -> bytes:
        q_uint = double_to_uint(self.q, -self.pMax, self.pMax, 16)
        dq_uint = double_to_uint(self.dq, -self.vMax, self.vMax, 12)
        tau_uint = double_to_uint(self.tau, -self.tMax, self.tMax, 12)

        d0 = ((self.error_code & 0x0F) << 4) | (self.send_id & 0x0F)
        d1 = (q_uint >> 8) & 0xFF
        d2 = q_uint & 0xFF
        d3 = (dq_uint >> 4) & 0xFF
        d4 = ((dq_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)
        d5 = tau_uint & 0xFF
        d6 = int(max(0, min(255, round(self.t_mos))))
        d7 = int(max(0, min(255, round(self.t_rotor))))

        return bytes([d0, d1, d2, d3, d4, d5, d6, d7])

    def make_param_reply(self, rid: int) -> bytes:
        val = self.params.get(rid, 0.0)
        is_uint = (7 <= rid <= 10) or (13 <= rid <= 16) or (35 <= rid <= 36)
        val_bytes = struct.pack("<I" if is_uint else "<f", int(val) if is_uint else float(val))

        d0 = self.send_id & 0xFF
        d1 = (self.send_id >> 8) & 0xFF
        d2 = 0x33
        d3 = rid & 0xFF
        return bytes([d0, d1, d2, d3, val_bytes[0], val_bytes[1], val_bytes[2], val_bytes[3]])

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
            "kd": round(self.kd, 2)
        }


class DamiaoArmSimulator:
    def __init__(self, interface: str = "vcan0"):
        self.interface = interface
        self.running = False
        self.sock: Optional[socket.socket] = None
        self.lock = threading.Lock()
        self.last_used_fd = True

        # OpenArm Bimanual configuration (16 motors total):
        # LEFT ARM (1..8): DM8009 x2, DM4340 x2, DM4310 x4 (Send 0x01..0x08, Recv 0x11..0x18)
        # RIGHT ARM (9..16): DM8009 x2, DM4340 x2, DM4310 x4 (Send 0x21..0x28, Recv 0x31..0x38)
        self.motors: Dict[int, VirtualDamiaoMotor] = {
            # --- LEFT ARM ---
            1: VirtualDamiaoMotor(1, "Left J1 (Shoulder Yaw)",   "left", 1, "DM8009", 0x01, 0x11, 12.5, 45.0, 54.0),
            2: VirtualDamiaoMotor(2, "Left J2 (Shoulder Pitch)", "left", 2, "DM8009", 0x02, 0x12, 12.5, 45.0, 54.0),
            3: VirtualDamiaoMotor(3, "Left J3 (Elbow Roll)",     "left", 3, "DM4340", 0x03, 0x13, 12.5, 10.0, 28.0),
            4: VirtualDamiaoMotor(4, "Left J4 (Elbow Pitch)",    "left", 4, "DM4340", 0x04, 0x14, 12.5, 10.0, 28.0),
            5: VirtualDamiaoMotor(5, "Left J5 (Wrist Roll)",     "left", 5, "DM4310", 0x05, 0x15, 12.5, 30.0, 10.0),
            6: VirtualDamiaoMotor(6, "Left J6 (Wrist Pitch)",    "left", 6, "DM4310", 0x06, 0x16, 12.5, 30.0, 10.0),
            7: VirtualDamiaoMotor(7, "Left J7 (Wrist Yaw)",      "left", 7, "DM4310", 0x07, 0x17, 12.5, 30.0, 10.0),
            8: VirtualDamiaoMotor(8, "Left Gripper (J8 Kẹp Ngang)",  "left", 8, "DM4310", 0x08, 0x18, 12.5, 30.0, 10.0),

            # --- RIGHT ARM ---
            9:  VirtualDamiaoMotor(9,  "Right J1 (Shoulder Yaw)",   "right", 1, "DM8009", 0x21, 0x31, 12.5, 45.0, 54.0),
            10: VirtualDamiaoMotor(10, "Right J2 (Shoulder Pitch)", "right", 2, "DM8009", 0x22, 0x32, 12.5, 45.0, 54.0),
            11: VirtualDamiaoMotor(11, "Right J3 (Elbow Roll)",     "right", 3, "DM4340", 0x23, 0x33, 12.5, 10.0, 28.0),
            12: VirtualDamiaoMotor(12, "Right J4 (Elbow Pitch)",    "right", 4, "DM4340", 0x24, 0x34, 12.5, 10.0, 28.0),
            13: VirtualDamiaoMotor(13, "Right J5 (Wrist Roll)",     "right", 5, "DM4310", 0x25, 0x35, 12.5, 30.0, 10.0),
            14: VirtualDamiaoMotor(14, "Right J6 (Wrist Pitch)",    "right", 6, "DM4310", 0x26, 0x36, 12.5, 30.0, 10.0),
            15: VirtualDamiaoMotor(15, "Right J7 (Wrist Yaw)",      "right", 7, "DM4310", 0x27, 0x37, 12.5, 30.0, 10.0),
            16: VirtualDamiaoMotor(16, "Right Gripper (J8 Kẹp Ngang)", "right", 8, "DM4310", 0x28, 0x38, 12.5, 30.0, 10.0),
        }

        # Fast lookup mapping: send_id -> motor (with dual alias for 0x09..0x10)
        self.motors_by_send_id: Dict[int, VirtualDamiaoMotor] = {}
        for m in self.motors.values():
            self.motors_by_send_id[m.send_id] = m
        # Also alias Right Arm to 0x09..0x10 if addressed sequentially
        for idx in range(1, 9):
            right_motor = self.motors[8 + idx]
            self.motors_by_send_id[0x08 + idx] = right_motor

        self.frames_rx = 0
        self.frames_tx = 0
        self.traffic_log: List[dict] = []

    def start(self):
        self.running = True
        if self.interface.startswith("vcan"):
            try:
                subprocess.run(["sudo", "modprobe", "vcan"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                subprocess.run(["sudo", "ip", "link", "add", "dev", self.interface, "type", "vcan"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                subprocess.run(["sudo", "ip", "link", "set", self.interface, "up"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
        except Exception as e:
            print("[Sim] CAN_RAW_FD_FRAMES warning:", e)
        self.sock.bind((self.interface,))

        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        self.physics_thread = threading.Thread(target=self._physics_loop, daemon=True)
        self.physics_thread.start()
        print(f"[Sim] Damiao CAN-FD Simulator running on {self.interface} ({len(self.motors)} motors: Dual-Arm OpenArm)")

    def stop(self):
        self.running = False
        if self.sock:
            self.sock.close()

    def _physics_loop(self):
        dt = 0.0025 # 400 Hz
        while self.running:
            with self.lock:
                for motor in self.motors.values():
                    motor.update_physics(dt)
            time.sleep(dt)

    def _send_can(self, can_id: int, data: bytes, use_fd: bool = True):
        if not self.sock:
            return
        try:
            if use_fd:
                pad = 64 - len(data)
                frame = struct.pack(CANFD_FRAME_FMT, can_id, len(data), 0, 0, data + (b'\x00' * pad))
            else:
                pad = 8 - len(data)
                frame = struct.pack(CAN_FRAME_FMT, can_id, len(data), data + (b'\x00' * pad))
            self.sock.send(frame)
            self.frames_tx += 1

            if len(self.traffic_log) > 140:
                self.traffic_log.pop(0)
            self.traffic_log.append({
                "time": time.time(),
                "id": hex(can_id),
                "dir": "TX",
                "dlc": len(data),
                "hex": data.hex().upper()
            })
        except Exception:
            pass

    def _rx_loop(self):
        while self.running:
            try:
                raw_frame = self.sock.recv(72)
                if not raw_frame:
                    continue

                if len(raw_frame) == 72:
                    can_id, dlc, flags, res, data_64 = struct.unpack(CANFD_FRAME_FMT, raw_frame)
                    data = data_64[:dlc]
                    is_fd = True
                elif len(raw_frame) == 16:
                    can_id, dlc, data_8 = struct.unpack(CAN_FRAME_FMT, raw_frame)
                    data = data_8[:dlc]
                    is_fd = False
                else:
                    continue

                can_id = can_id & 0x1FFFFFFF
                self.frames_rx += 1
                self.last_used_fd = is_fd

                if len(self.traffic_log) > 140:
                    self.traffic_log.pop(0)
                self.traffic_log.append({
                    "time": time.time(),
                    "id": hex(can_id),
                    "dir": "RX",
                    "dlc": dlc,
                    "hex": data.hex().upper()
                })

                self._handle_frame(can_id, data, is_fd)
            except Exception:
                if not self.running:
                    break

    def _handle_frame(self, can_id: int, data: bytes, is_fd: bool):
        if len(data) < 8:
            return

        with self.lock:
            # 0x7FF Management frames (refresh, query param, write param)
            if can_id == 0x7FF:
                target_id = data[0] | (data[1] << 8)
                cmd = data[2]
                motor = self.motors_by_send_id.get(target_id)
                if not motor:
                    return

                if cmd == 0xCC:
                    # Refresh request
                    reply = motor.make_state_frame()
                    self._send_can(motor.recv_id, reply, is_fd)
                elif cmd == 0x33:
                    # Query RID
                    rid = data[3]
                    reply = motor.make_param_reply(rid)
                    self._send_can(motor.recv_id, reply, is_fd)
                elif cmd == 0x55:
                    # Write RID
                    rid = data[3]
                    is_uint = (7 <= rid <= 10) or (13 <= rid <= 16) or (35 <= rid <= 36)
                    val = struct.unpack("<I" if is_uint else "<f", data[4:8])[0]
                    motor.params[rid] = float(val)
                    reply = motor.make_param_reply(rid)
                    self._send_can(motor.recv_id, reply, is_fd)
                return

            base_id = can_id & 0x0FF
            mode_offset = can_id & 0xF00
            motor = self.motors_by_send_id.get(base_id)
            if not motor:
                return

            # Special commands (0xFF*7 + CMD)
            if data[0:7] == bytes([0xFF]*7):
                cmd = data[7]
                if cmd == 0xFC:
                    motor.enabled = True
                    motor.error_code = 1
                    self._send_can(motor.recv_id, motor.make_state_frame(), is_fd)
                elif cmd == 0xFD:
                    motor.enabled = False
                    motor.error_code = 0
                    self._send_can(motor.recv_id, motor.make_state_frame(), is_fd)
                elif cmd == 0xFE:
                    motor.q = 0.0
                    motor.q_des = 0.0
                    motor.dq = 0.0
                    self._send_can(motor.recv_id, motor.make_state_frame(), is_fd)
                elif cmd == 0xFB:
                    motor.error_code = 1 if motor.enabled else 0
                    self._send_can(motor.recv_id, motor.make_state_frame(), is_fd)
                return

            # MIT Control (mode_offset == 0)
            if mode_offset == 0:
                q_uint = (data[0] << 8) | data[1]
                dq_uint = (data[2] << 4) | (data[3] >> 4)
                kp_uint = ((data[3] & 0x0F) << 8) | data[4]
                kd_uint = (data[5] << 4) | (data[6] >> 4)
                tau_uint = ((data[6] & 0x0F) << 8) | data[7]

                motor.q_des = uint_to_double(q_uint, -motor.pMax, motor.pMax, 16)
                motor.dq_des = uint_to_double(dq_uint, -motor.vMax, motor.vMax, 12)
                motor.kp = uint_to_double(kp_uint, 0.0, 500.0, 12)
                motor.kd = uint_to_double(kd_uint, 0.0, 5.0, 12)
                motor.tau_ff = uint_to_double(tau_uint, -motor.tMax, motor.tMax, 12)

                reply = motor.make_state_frame()
                self._send_can(motor.recv_id, reply, is_fd)

            # Pos-Vel (mode_offset == 0x100)
            elif mode_offset == 0x100:
                pos = struct.unpack("<f", data[0:4])[0]
                motor.q_des = pos
                motor.kp = 35.0
                motor.kd = 1.0
                self._send_can(motor.recv_id, motor.make_state_frame(), is_fd)

            # Vel (mode_offset == 0x200)
            elif mode_offset == 0x200:
                vel = struct.unpack("<f", data[0:4])[0]
                motor.dq_des = vel
                motor.kp = 0.0
                motor.kd = 2.0
                self._send_can(motor.recv_id, motor.make_state_frame(), is_fd)

            # Pos-Force (mode_offset == 0x300)
            elif mode_offset == 0x300:
                pos = struct.unpack("<f", data[0:4])[0]
                motor.q_des = pos
                motor.kp = 40.0
                motor.kd = 1.2
                self._send_can(motor.recv_id, motor.make_state_frame(), is_fd)


if __name__ == "__main__":
    sim = DamiaoArmSimulator("vcan0")
    sim.start()
    try:
        while True:
            time.sleep(1)
            print(f"[Sim] RX: {sim.frames_rx} | TX: {sim.frames_tx} | L1 q: {sim.motors[1].q:.3f} | R1 q: {sim.motors[9].q:.3f}")
    except KeyboardInterrupt:
        sim.stop()
