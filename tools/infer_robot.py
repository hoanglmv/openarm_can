#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Production Hardware Inference Script for Bimanual OpenArm (16-DOF) + Chest RGB-D
Tác vụ: Thao tác tự động hai tay (Bimanual Manipulation / Cooking / Pick-and-Place).
Nối trực tiếp từ Camera RGB-D -> Model ACT -> Temporal Ensembling -> SocketCAN 16 Động cơ Damiao.

Tích hợp điều khiển quỹ đạo mượt mà (Trajectory Generator & S-Curve Smoothing):
1. S-Curve Cosine Warm-up: Hòa nhập êm ái từ vị trí thực tế tới tư thế bắt đầu (triệt tiêu giật khởi động).
2. Trajectory Velocity & Acceleration Clamping: Bọc từng khớp theo đặc tính động cơ (DM8009, DM4340, DM4310).
3. Temporal Ensembling: Làm mượt dự đoán hành động ACT chunk theo hàm trọng số exp(-m*i).
4. Gripper Dual-Mode: Hỗ trợ POS_FORCE với giới hạn lực kẹp an toàn (torque_pu=0.15) hoặc MIT Mode.
5. Safe S-Curve Home / Soft E-Stop: Đưa robot lùi về Home an toàn bằng quỹ đạo cong trước khi ngắt mô-men, chống rơi tự do.
6. 3 tầng ngắt dừng an toàn: Dừng tự nhiên khi về Home (|Δq| < 0.008 rad liên tục 1 giây), Timeout tối đa, Dừng khẩn cấp Soft E-Stop.
"""

import os
import sys
import time
import math
import argparse
from typing import Optional, Tuple

# Đảm bảo UTF-8 an toàn
if sys.stdout.encoding != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# Thêm đường dẫn gốc vào sys.path
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

import numpy as np
import torch
import cv2

try:
    import openarm_can as oa
    OPENARM_CAN_AVAILABLE = hasattr(oa, "MotorType")
except ImportError:
    OPENARM_CAN_AVAILABLE = False

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False

from act_pipeline.models.act_model import ACTPolicy
from act_pipeline.config import ModelConfig
from act_pipeline.data.normalization import load_norm_stats, normalize_data, unnormalize_data
from act_pipeline.data.preprocess import preprocess_rgbd
from act_pipeline.utils.temporal_ensemble import TemporalEnsemblePolicy

# Danh sách động cơ chuẩn cho 1 cánh tay OpenArm 7-DOF + 1 Gripper
_motor_cls = getattr(oa, "MotorType", None) if OPENARM_CAN_AVAILABLE else None
ARM_MOTOR_TYPES = [
    getattr(_motor_cls, "DM8009", None) if _motor_cls else None,
    getattr(_motor_cls, "DM8009", None) if _motor_cls else None,
    getattr(_motor_cls, "DM4340", None) if _motor_cls else None,
    getattr(_motor_cls, "DM4340", None) if _motor_cls else None,
    getattr(_motor_cls, "DM4310", None) if _motor_cls else None,
    getattr(_motor_cls, "DM4310", None) if _motor_cls else None,
    getattr(_motor_cls, "DM4310", None) if _motor_cls else None,
]
ARM_SEND_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
ARM_RECV_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
GRIPPER_SEND_ID = 0x08
GRIPPER_RECV_ID = 0x18


class CameraHandler:
    """Quản lý luồng ảnh RGB-D từ RealSense hoặc WebCam/Mock với chế độ non-blocking linh hoạt"""
    def __init__(
        self,
        camera_type: str = "realsense",
        camera_id: int = 0,
        camera_fps: int = 60,
        img_width: int = 640,
        img_height: int = 480,
        non_blocking: bool = True,
    ):
        self.camera_type = camera_type
        self.camera_fps = camera_fps
        self.img_width = img_width
        self.img_height = img_height
        self.non_blocking = non_blocking
        self.pipeline = None
        self.align = None
        self.cap = None
        self._last_rgb: Optional[np.ndarray] = None
        self._last_depth: Optional[np.ndarray] = None

        if camera_type == "realsense" and REALSENSE_AVAILABLE:
            print(f"[*] Đang khởi động camera Intel RealSense (RGB + Depth) @ {self.camera_fps} FPS...")
            try:
                self.pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.color, self.img_width, self.img_height, rs.format.bgr8, self.camera_fps)
                config.enable_stream(rs.stream.depth, self.img_width, self.img_height, rs.format.z16, self.camera_fps)
                self.pipeline.start(config)
                self.align = rs.align(rs.stream.color)

                # Khởi động trước (Warm-up 5 frames) để nạp sẵn cache ban đầu
                for _ in range(5):
                    frames = self.pipeline.wait_for_frames()
                    aligned = self.align.process(frames)
                    c_f = aligned.get_color_frame()
                    d_f = aligned.get_depth_frame()
                    if c_f and d_f:
                        bgr = np.asanyarray(c_f.get_data())
                        self._last_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                        self._last_depth = np.asanyarray(d_f.get_data())

                print(f"[✓] Camera Intel RealSense sẵn sàng ({self.img_width}x{self.img_height} @ {self.camera_fps}fps, align_to_color bật, non_blocking={self.non_blocking}).")
            except Exception as e:
                print(f"[!] Lỗi mở RealSense: {e}. Chuyển sang chế độ giả lập.")
                self.pipeline = None
        elif camera_type == "opencv":
            print(f"[*] Sử dụng camera OpenCV (ID: {camera_id})...")
            self.cap = cv2.VideoCapture(camera_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
        else:
            print("[*] Sử dụng Camera Mock ảo (RGB-D synthetic frame)...")

    def get_rgbd(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if self.pipeline is not None:
            frames = None
            if self.non_blocking:
                # Chế độ non-blocking: Thử thăm dò 1ms, nếu chưa có frame mới thì trả về frame cache gần nhất
                success, frames = self.pipeline.try_wait_for_frames(timeout_ms=1)
                if not success:
                    frames = None
            else:
                frames = self.pipeline.wait_for_frames()

            if frames is not None:
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                if color_frame and depth_frame:
                    bgr = np.asanyarray(color_frame.get_data())
                    self._last_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                    self._last_depth = np.asanyarray(depth_frame.get_data())

            return self._last_rgb, self._last_depth
        elif self.cap is not None:
            ret, frame = self.cap.read()
            if ret:
                self._last_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return self._last_rgb, None
        else:
            # Mock synthetic frame
            rgb = np.full((self.img_height, self.img_width, 3), fill_value=128, dtype=np.uint8)
            depth = np.full((self.img_height, self.img_width), fill_value=600, dtype=np.uint16)
            return rgb, depth

    def stop(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        if self.cap is not None:
            self.cap.release()


class TrajectorySmoother:
    """
    Bộ làm mượt quỹ đạo thời gian thực (Real-time Kinematic Trajectory Filter).
    Kế thừa từ kiến trúc sim/server.py và test_wrist_pitch.py.
    Giới hạn cả Vận tốc (Velocity) và Gia tốc (Acceleration) trên từng khớp riêng biệt:
      - DM8009 (Vai J1..J2): Quán tính lớn, khống chế v_max = 1.0 rad/s, a_max = 3.0 rad/s^2
      - DM4340 (Khuỷu J3..J4): v_max = 1.5 rad/s, a_max = 5.0 rad/s^2
      - DM4310 (Cổ tay J5..J7): v_max = 2.5 rad/s, a_max = 8.0 rad/s^2
      - DM4310 (Kẹp ngón tay J8): v_max = 3.5 rad/s, a_max = 12.0 rad/s^2
    """
    def __init__(self, vel_scale: float = 1.0, dt: float = 0.02):
        self.dt = dt
        self.scale = max(0.1, min(2.0, vel_scale))

        # Giới hạn vận tốc từng khớp đơn vị rad/s cho 1 cánh tay (8 bậc: J1..J8)
        base_v_lim = np.array([
            1.0, 1.0,        # J1, J2 (DM8009)
            1.5, 1.5,        # J3, J4 (DM4340)
            2.5, 2.5, 2.5,   # J5, J6, J7 (DM4310)
            3.5              # J8 (Gripper)
        ], dtype=np.float32)

        base_a_lim = np.array([
            3.0, 3.0,        # J1, J2
            5.0, 5.0,        # J3, J4
            8.0, 8.0, 8.0,   # J5, J6, J7
            12.0             # J8
        ], dtype=np.float32)

        # Ghép 16 bậc cho 2 tay: [Tay Trái J1..J8, Tay Phải J1..J8]
        self.v_lim = np.concatenate([base_v_lim, base_v_lim]) * self.scale
        self.a_lim = np.concatenate([base_a_lim, base_a_lim]) * self.scale

        self.q_cmd = None
        self.v_cmd = np.zeros(16, dtype=np.float32)

    def reset(self, initial_qpos: np.ndarray):
        """Khởi tạo trạng thái quỹ đạo trùng với vị trí vật lý ban đầu"""
        self.q_cmd = np.array(initial_qpos, dtype=np.float32).copy()
        self.v_cmd = np.zeros(16, dtype=np.float32)

    def step(self, target_qpos: np.ndarray) -> np.ndarray:
        """
        Nội suy bước tiếp theo có ràng buộc vận tốc & gia tốc.
        Triệt tiêu mọi xung động giật khi model dự đoán bước nhảy đột ngột.
        """
        if self.q_cmd is None:
            self.reset(target_qpos)
            return self.q_cmd.copy()

        # 1. Tính toán vận tốc mong muốn để tiến về mục tiêu
        diff = target_qpos - self.q_cmd
        desired_vel = diff / self.dt

        # 2. Cắt ngọn vận tốc theo v_lim (Velocity Clamping như sim/server.py)
        desired_vel = np.clip(desired_vel, -self.v_lim, self.v_lim)

        # 3. Giới hạn gia tốc (Slew-rate acceleration limiting)
        max_dv = self.a_lim * self.dt
        dv = np.clip(desired_vel - self.v_cmd, -max_dv, max_dv)
        self.v_cmd += dv

        # 4. Cập nhật vị trí lệnh mượt mà
        self.q_cmd += self.v_cmd * self.dt
        return self.q_cmd.copy()


class BimanualOpenArmHardware:
    """
    Quản lý luồng lệnh góc khớp của Model AI tuân thủ kiến trúc 7 tầng (CONTEXT.md):
    - Mặc định (use_backend=True): Gửi /joint_state_cmd (50 Hz) tới Backend Middleware qua UDP port 9870.
      Backend tiếp nhận, đưa qua Safety Guard và Spline Interpolator (50Hz -> 400Hz) rồi mới nạp xuống CAN.
      Tuyệt đối KHÔNG bypass Backend.
    - Direct CAN (use_backend=False): Chỉ dùng cho mục đích Diagnostic phần cứng cô lập.
    """
    def __init__(self,
                 can_right: str = "can0",
                 can_left: str = "can1",
                 gripper_mode: str = "pos_force",
                 invert_left_j1: bool = True,
                 gripper_unit: str = "auto",
                 dry_run: bool = False,
                 use_backend: bool = True,
                 backend_host: str = "127.0.0.1",
                 backend_port: int = 9870,
                 telemetry_port: int = 9871):
        self.dry_run = dry_run
        self.use_backend = use_backend and not dry_run
        self.backend_host = backend_host
        self.backend_port = backend_port
        self.telemetry_port = telemetry_port
        self.can_right = can_right
        self.can_left = can_left
        self.gripper_mode = gripper_mode.lower()
        self.invert_left_j1 = invert_left_j1
        self.gripper_unit = gripper_unit
        self.arm_right = None
        self.arm_left = None
        self.latest_backend_qpos = np.zeros(16, dtype=np.float32)

        if self.use_backend:
            import socket, json, threading
            print(f"[*] [Backend Core Hub] Model AI phát /joint_state_cmd (50 Hz) tới Backend ({backend_host}:{backend_port}) - CẤM CAN Bypass!")
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.telem_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                self.telem_sock.bind(("0.0.0.0", telemetry_port))
                threading.Thread(target=self._telemetry_listener, daemon=True).start()
                print(f"[*] [Backend Core Hub] Lắng nghe telemetry /joint_states từ Backend trên port {telemetry_port}")
            except Exception as e:
                print(f"[!] Warning: Could not bind telemetry listener on port {telemetry_port}: {e}")
        elif not dry_run:
            print("[!] [CẢNH BÁO KIẾN TRÚC] Đang chạy Direct SocketCAN mode (chỉ dùng cho Diagnostic phần cứng cấp thấp).")
            if not OPENARM_CAN_AVAILABLE:
                raise ImportError("Chưa cài đặt thư viện 'openarm_can'! Vui lòng chạy trong môi trường có openarm_can.")
            print(f"[*] Đang kết nối CAN Bus: Tay Phải [{can_right}], Tay Trái [{can_left}]...")
            print(f"[*] Chế độ Gripper (Joint 8): {self.gripper_mode.upper()}")

            # Khởi tạo tay phải
            try:
                self.arm_right = oa.OpenArm(can_right, True)
                if self.gripper_mode == "pos_force" and hasattr(oa, "ControlMode"):
                    self.arm_right.init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS, [oa.ControlMode.MIT] * 7)
                    self.arm_right.init_gripper_motor(oa.MotorType.DM4310, GRIPPER_SEND_ID, GRIPPER_RECV_ID, oa.ControlMode.POS_FORCE)
                else:
                    self.arm_right.init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS)
                    self.arm_right.init_gripper_motor(oa.MotorType.DM4310, GRIPPER_SEND_ID, GRIPPER_RECV_ID)

                self.arm_right.set_callback_mode_all(oa.CallbackMode.STATE)
                self.arm_right.enable_all()
                self.arm_right.recv_all(1000)
                print(f"[✓] Đã kích hoạt tay phải trên {can_right} (Torque ON).")
            except Exception as e:
                print(f"[!] Cảnh báo tay phải {can_right}: {e}")

            # Khởi tạo tay trái
            try:
                self.arm_left = oa.OpenArm(can_left, True)
                if self.gripper_mode == "pos_force" and hasattr(oa, "ControlMode"):
                    self.arm_left.init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS, [oa.ControlMode.MIT] * 7)
                    self.arm_left.init_gripper_motor(oa.MotorType.DM4310, GRIPPER_SEND_ID, GRIPPER_RECV_ID, oa.ControlMode.POS_FORCE)
                else:
                    self.arm_left.init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS)
                    self.arm_left.init_gripper_motor(oa.MotorType.DM4310, GRIPPER_SEND_ID, GRIPPER_RECV_ID)

                self.arm_left.set_callback_mode_all(oa.CallbackMode.STATE)
                self.arm_left.enable_all()
                self.arm_left.recv_all(1000)
                print(f"[✓] Đã kích hoạt tay trái trên {can_left} (Torque ON).")
            except Exception as e:
                print(f"[!] Cảnh báo tay trái {can_left}: {e}")

            time.sleep(0.1)
            self.refresh_all()

    def _telemetry_listener(self):
        """Lắng nghe gói tin telemetry từ Backend để cập nhật qpos hiện tại."""
        import json
        while True:
            try:
                data, _ = self.telem_sock.recvfrom(8192)
                telem = json.loads(data.decode('utf-8'))
                if "positions" in telem:
                    self.latest_backend_qpos = np.array(telem["positions"][:16], dtype=np.float32)
                elif "qpos" in telem:
                    self.latest_backend_qpos = np.array(telem["qpos"][:16], dtype=np.float32)
            except Exception:
                pass

    def refresh_all(self):
        if not self.dry_run:
            if self.arm_right:
                self.arm_right.refresh_all()
                self.arm_right.recv_all(100)
            if self.arm_left:
                self.arm_left.refresh_all()
                self.arm_left.recv_all(100)

    def _to_gripper_cmd_rad(self, val: float) -> float:
        """
        Chuyển đổi lệnh điều khiển kẹp sang Radian motor (0.0 .. 1.20 rad):
        - Nếu đầu ra ACT là hành trình mét (<= 0.043m từ HDF5 dataset): scale sang 1.20 rad.
        - Nếu đầu ra ACT đã là Radian (> 0.043 rad): giữ nguyên.
        (Chuẩn tương thích 100% với tools/control_gripper.py và sim/models.py)
        """
        if abs(val) <= 0.043:
            return float((val / 0.043) * 1.20)
        return float(val)

    def get_qpos(self) -> np.ndarray:
        """Đọc vị trí hiện tại của 16 động cơ: [8 Khớp Trái, 8 Khớp Phải]"""
        if self.dry_run:
            return np.zeros(16, dtype=np.float32)

        if self.use_backend:
            return self.latest_backend_qpos.copy()

        self.refresh_all()
        qpos = np.zeros(16, dtype=np.float32)

        # Tay trái (Chỉ số 0..7)
        if self.arm_left:
            left_arm_motors = self.arm_left.get_arm().get_motors()
            for i, m in enumerate(left_arm_motors[:7]):
                raw_pos = m.get_position()
                # Khớp 0 (Vai Trái J1): Áp dụng MOTOR_DIRECTIONS[1] = -1.0 nếu invert_left_j1 bật
                if i == 0 and self.invert_left_j1:
                    qpos[0] = -raw_pos
                else:
                    qpos[i] = raw_pos

            left_grip_motors = self.arm_left.get_gripper().get_motors()
            if len(left_grip_motors) > 0:
                raw_grip = left_grip_motors[0].get_position()
                # Chuyển đổi sang mét nếu model học theo chuẩn stroke_m (0..0.043m)
                if self.gripper_unit == "stroke_m":
                    ratio = max(0.0, min(1.0, abs(raw_grip) / 1.20))
                    qpos[7] = float(ratio * 0.043)
                else:
                    qpos[7] = raw_grip

        # Tay phải (Chỉ số 8..15)
        if self.arm_right:
            right_arm_motors = self.arm_right.get_arm().get_motors()
            for i, m in enumerate(right_arm_motors[:7]):
                qpos[8 + i] = m.get_position()

            right_grip_motors = self.arm_right.get_gripper().get_motors()
            if len(right_grip_motors) > 0:
                raw_grip = right_grip_motors[0].get_position()
                if self.gripper_unit == "stroke_m":
                    ratio = max(0.0, min(1.0, abs(raw_grip) / 1.20))
                    qpos[15] = float(ratio * 0.043)
                else:
                    qpos[15] = raw_grip

        return qpos

    def send_target_positions(self,
                              target_qpos: np.ndarray,
                              kp_arm: float = 35.0,
                              kd_arm: float = 1.2,
                              kp_grip: float = 20.0,
                              kd_grip: float = 0.8,
                              gripper_speed: float = 25.0,
                              gripper_torque_pu: float = 0.15):
        """
        Gửi lệnh vị trí tới 16 động cơ:
        - Mặc định: Phát luồng /joint_state_cmd (50 Hz) tới Backend Middleware Core Hub qua UDP 9870.
          Backend chịu trách nhiệm Safety Guard và Spline Interpolator (50Hz -> 400Hz).
        - Direct CAN: Chỉ dùng khi người dùng bật cờ --direct_can.
        """
        if self.dry_run:
            return

        if self.use_backend:
            import json
            payload = {
                "source": "ACT_Policy_Inference",
                "timestamp": time.time(),
                "positions": [float(p) for p in target_qpos[:16]],
            }
            try:
                msg = json.dumps(payload).encode('utf-8')
                self.udp_sock.sendto(msg, (self.backend_host, self.backend_port))
            except Exception as e:
                print(f"[!] Lỗi gửi lệnh sang Backend: {e}")
            return

        # 1. Gửi tay trái (Chỉ số 0..7)
        if self.arm_left:
            left_cmds = [
                -target_qpos[0] if (i == 0 and self.invert_left_j1) else target_qpos[i]
                for i in range(7)
            ]
            left_arm_params = [oa.MITParam(kp_arm, kd_arm, left_cmds[i], 0.0, 0.0) for i in range(7)]
            self.arm_left.get_arm().mit_control_all(left_arm_params)

            grip_left_rad = self._to_gripper_cmd_rad(target_qpos[7])
            if self.gripper_mode == "pos_force":
                # Kẹp gắp trong POS_FORCE mode (an toàn lực kẹp 0.15 pu theo tools/control_gripper.py)
                self.arm_left.get_gripper().set_position(
                    float(grip_left_rad),
                    speed_rad_s=gripper_speed,
                    torque_pu=gripper_torque_pu
                )
            else:
                self.arm_left.get_gripper().mit_control_all([oa.MITParam(kp_grip, kd_grip, grip_left_rad, 0.0, 0.0)])
            self.arm_left.recv_all(50)

        # 2. Gửi tay phải (Chỉ số 8..15)
        if self.arm_right:
            right_arm_params = [oa.MITParam(kp_arm, kd_arm, target_qpos[8 + i], 0.0, 0.0) for i in range(7)]
            self.arm_right.get_arm().mit_control_all(right_arm_params)

            grip_right_rad = self._to_gripper_cmd_rad(target_qpos[15])
            if self.gripper_mode == "pos_force":
                self.arm_right.get_gripper().set_position(
                    float(grip_right_rad),
                    speed_rad_s=gripper_speed,
                    torque_pu=gripper_torque_pu
                )
            else:
                self.arm_right.get_gripper().mit_control_all([oa.MITParam(kp_grip, kd_grip, grip_right_rad, 0.0, 0.0)])
            self.arm_right.recv_all(50)

    def disable_all(self):
        """Ngắt toàn bộ lực (Torque OFF) đưa đèn LED về màu ĐỎ an toàn"""
        if self.dry_run:
            return
        if self.use_backend:
            import json
            payload = {"command": "e_stop", "source": "ACT_Inference"}
            try:
                self.udp_sock.sendto(json.dumps(payload).encode('utf-8'), (self.backend_host, self.backend_port))
            except Exception:
                pass
            print("[✓] Đã gửi tín hiệu E-Stop an toàn tới Backend.")
            return

        print("[*] Đang gửi lệnh ngắt mô-men tới 16 động cơ (disable_all)...")
        if self.arm_left:
            try:
                self.arm_left.disable_all()
                self.arm_left.recv_all(500)
            except Exception:
                pass
        if self.arm_right:
            try:
                self.arm_right.disable_all()
                self.arm_right.recv_all(500)
            except Exception:
                pass
        print("[✓] Đã ngắt mô-men an toàn trên cả 2 cánh tay.")


def smooth_s_curve_move(robot: BimanualOpenArmHardware,
                        q_from: np.ndarray,
                        q_to: np.ndarray,
                        duration: float = 2.0,
                        control_dt: float = 0.02,
                        description: str = "Chuyển động"):
    """
    Nội suy quỹ đạo đường cong S-curve Cosine Profile (dựa trên test_wrist_pitch.py):
        alpha = 0.5 * (1.0 - cos(pi * step / steps))
        q_cmd = q_from + (q_to - q_from) * alpha
    Đảm bảo đạo hàm vận tốc và gia tốc bằng 0 tại 2 đầu mút -> Không bao giờ gây sốc mô-men động cơ!
    """
    steps = max(15, int(duration / control_dt))
    print(f"[*] Đang thực hiện {description} trong {duration:.1f}s ({steps} bước S-curve)...")
    for step in range(steps + 1):
        alpha = 0.5 * (1.0 - math.cos(math.pi * step / steps))
        q_interp = q_from + (q_to - q_from) * alpha
        robot.send_target_positions(q_interp)
        time.sleep(control_dt)
    print(f"[✓] Đã hoàn thành {description}.")


def main():
    parser = argparse.ArgumentParser(description="Chạy Inference điều khiển robot OpenArm thời gian thực với Trajectory mượt mà")
    parser.add_argument("--checkpoint", type=str, default="dataset/act_openarm_model.pth", help="Đường dẫn file trọng số checkpoint .pth")
    parser.add_argument("--can_right", type=str, default="can0", help="Cổng CAN tay phải (mặc định: can0)")
    parser.add_argument("--can_left", type=str, default="can1", help="Cổng CAN tay trái (mặc định: can1)")
    parser.add_argument("--camera", type=str, choices=["realsense", "opencv", "mock"], default="realsense", help="Loại camera")
    parser.add_argument("--gripper_mode", type=str, choices=["pos_force", "mit"], default="pos_force", help="Chế độ điều khiển kẹp gắp Joint 8 (pos_force hoặc mit)")
    parser.add_argument("--dry_run", action="store_true", help="Chạy thử mô phỏng không gửi lệnh CAN thật")
    parser.add_argument("--max_timesteps", type=int, default=1000, help="Số bước tối đa trước khi ngắt timeout (1000 steps = 20s @ 50Hz)")
    parser.add_argument("--ensemble_m", type=float, default=0.01, help="Hệ số suy giảm trọng số exp(-m*i) của Temporal Ensembling")
    parser.add_argument("--vel_scale", type=float, default=1.0, help="Tỉ lệ điều chỉnh giới hạn vận tốc các khớp (0.2 -> 2.0, mặc định 1.0)")
    parser.add_argument("--smooth_warmup", action="store_true", default=True, help="Bật S-curve hòa nhập từ vị trí hiện tại tới vị trí xuất phát")
    parser.add_argument("--warmup_duration", type=float, default=1.5, help="Thời gian hòa nhập S-curve lúc bắt đầu (giây)")
    parser.add_argument("--safe_home_on_stop", action="store_true", default=True, help="Tự động lùi về Home bằng S-curve khi kết thúc hoặc E-Stop")
    parser.add_argument("--stop_delta_rad", type=float, default=0.008, help="Ngưỡng biến thiên góc để nhận diện dừng tự nhiên")
    parser.add_argument("--stop_home_dist", type=float, default=0.15, help="Khoảng cách tới vị trí Home để kích hoạt dừng tự nhiên")
    parser.add_argument("--device", type=str, default="cuda", help="Thiết bị tính toán (cuda hoặc cpu)")
    parser.add_argument("--control_hz", type=float, default=50.0, help="Tần số chu kỳ điều khiển robot (Hz, mặc định: 50.0)")
    parser.add_argument("--camera_fps", type=int, default=60, help="Tần số camera RealSense (FPS, mặc định: 60, hỗ trợ: 15, 30, 60)")
    parser.add_argument("--camera_width", type=int, default=640, help="Chiều rộng ảnh camera (mặc định: 640)")
    parser.add_argument("--camera_height", type=int, default=480, help="Chiều cao ảnh camera (mặc định: 480)")
    parser.add_argument("--blocking_cam", action="store_true", help="Bắt buộc chờ frame mới từ camera (mặc định: False - dùng non-blocking poll)")
    parser.add_argument("--stop_duration", type=float, default=1.0, help="Thời gian bất động tại Home để nhận diện dừng tự nhiên (giây, mặc định: 1.0s)")
    parser.add_argument("--invert_left_j1", action="store_true", default=True, help="Đảo chiều vật lý Motor 1 (Vai Trái J1) theo quy ước động học OpenArm (+q vươn tới trước)")
    parser.add_argument("--no_invert_left_j1", dest="invert_left_j1", action="store_false", help="Tắt đảo chiều Motor 1 nếu phần cứng đã được cấu hình trong motor firmware")
    parser.add_argument("--gripper_unit", choices=["auto", "stroke_m", "rad"], default="auto", help="Đơn vị kẹp gắp: 'auto' (tự động phát hiện), 'stroke_m' (mét: 0..0.043m), 'rad' (radian: 0..1.20 rad)")
    parser.add_argument("--backend", action="store_true", default=True, help="Truyền lệnh và telemetry qua Backend Middleware Core Hub (UDP 9870/9871, cấm bypass)")
    parser.add_argument("--direct_can", dest="backend", action="store_false", help="Bypass Backend để gửi trực tiếp SocketCAN (chỉ dùng cho Diagnostic phần cứng)")
    parser.add_argument("--backend_host", type=str, default="127.0.0.1", help="Địa chỉ IP host của Backend Server")
    parser.add_argument("--backend_port", type=int, default=9870, help="Cổng UDP nhận /joint_state_cmd của Backend (mặc định: 9870)")
    parser.add_argument("--telemetry_port", type=int, default=9871, help="Cổng UDP phát telemetry của Backend (mặc định: 9871)")
    args = parser.parse_args()

    control_dt = 1.0 / max(1.0, args.control_hz)
    device = torch.device(args.device if torch.cuda.is_available() and args.device == "cuda" else "cpu")
    print("=" * 80)
    print(f"     🤖 KHỞI ĐỘNG ROBOT INFERENCE: BIMANUAL OPENARM (16-DOF) RGB-D @ {args.control_hz:.1f}HZ")
    print("        TÍCH HỢP QUỸ ĐẠO MƯỢT MÀ: S-CURVE + TRAJECTORY VELOCITY FILTER")
    print("=" * 80)
    print(f"[*] Checkpoint nạp vào      : {args.checkpoint}")
    print(f"[*] Thiết bị tính toán      : {device} ({torch.cuda.get_device_name(0) if device.type == 'cuda' else 'CPU'})")
    print(f"[*] Tần số điều khiển       : {args.control_hz:.1f} Hz (Chu kỳ dt = {control_dt*1000:.1f} ms)")
    print(f"[*] Tần số Camera           : {args.camera_fps} FPS (Độ phân giải: {args.camera_width}x{args.camera_height}, Non-blocking: {not args.blocking_cam})")
    print(f"[*] Chế độ phần cứng        : {'DRY RUN (Thử nghiệm ảo)' if args.dry_run else 'REAL HARDWARE (Robot thật)'}")
    print(f"[*] Chế độ Gripper (J8)     : {args.gripper_mode.upper()} {'(Lực an toàn 0.15 pu)' if args.gripper_mode == 'pos_force' else '(MIT mode)'}")
    print(f"[*] Trajectory Vel Scale    : {args.vel_scale:.2f}x")
    print(f"[*] S-Curve Warm-up         : {'BẬT (' + str(args.warmup_duration) + 's)' if args.smooth_warmup else 'TẮT'}")
    print(f"[*] Safe Home on Stop       : {'BẬT (Bảo vệ chống rơi tự do)' if args.safe_home_on_stop else 'TẮT'}")
    print(f"[*] Timeout tối đa          : {args.max_timesteps} steps (~{args.max_timesteps/args.control_hz:.1f}s)")
    print(f"[*] Temporal Ensembling     : BẬT (m = {args.ensemble_m})")
    print("=" * 80)

    # 1. Tải Model & Thống kê chuẩn hóa (Stats)
    if not os.path.exists(args.checkpoint):
        print(f"[X] LỖI: Không tìm thấy checkpoint tại: {args.checkpoint}")
        sys.exit(1)

    checkpoint = torch.load(args.checkpoint, map_location=device, weights_only=False)
    state_dict = checkpoint["model_state_dict"] if "model_state_dict" in checkpoint else checkpoint
    stats = checkpoint.get("stats")
    if stats is None:
        stats_path = os.path.join(os.path.dirname(args.checkpoint), "dataset_stats.pkl")
        if os.path.exists(stats_path):
            stats = load_norm_stats(stats_path)
        else:
            raise FileNotFoundError("Không tìm thấy thống kê chuẩn hóa stats trong checkpoint hoặc thư mục!")

    saved_cfg = checkpoint.get("config", {})

    # Tự động dò kích thước d_model và action_dim từ state_dict để tương thích 100%
    detected_d_model = saved_cfg.get("d_model", 512)
    detected_action_dim = saved_cfg.get("action_dim", 16)
    detected_chunk_size = saved_cfg.get("chunk_size", 50)
    detected_decoder_layers = saved_cfg.get("decoder_layers", 7)
    detected_cvae_layers = saved_cfg.get("cvae_layers", 4)
    detected_latent_dim = saved_cfg.get("latent_dim", 32)

    if "qpos_proj.weight" in state_dict:
        detected_d_model = state_dict["qpos_proj.weight"].shape[0]
        detected_action_dim = state_dict["qpos_proj.weight"].shape[1]
    if "cvae.latent_mu.weight" in state_dict:
        detected_latent_dim = state_dict["cvae.latent_mu.weight"].shape[0]
    elif "latent_mu.weight" in state_dict:
        detected_latent_dim = state_dict["latent_mu.weight"].shape[0]

    decoder_layer_indices = [
        int(k.split(".layers.")[1].split(".")[0])
        for k in state_dict.keys()
        if ".layers." in k and ("policy_decoder" in k or "decoder" in k)
    ]
    if len(decoder_layer_indices) > 0:
        detected_decoder_layers = max(decoder_layer_indices) + 1

    cvae_layer_indices = [
        int(k.split(".layers.")[1].split(".")[0])
        for k in state_dict.keys()
        if ".layers." in k and "cvae" in k
    ]
    if len(cvae_layer_indices) > 0:
        detected_cvae_layers = max(cvae_layer_indices) + 1

    cfg = ModelConfig(
        in_channels=4,
        backbone_type="resnet18",
        freeze_backbone=True,
        d_model=detected_d_model,
        nheads=8 if detected_d_model % 8 == 0 else 4,
        dim_feedforward=detected_d_model * 4,
        cvae_layers=detected_cvae_layers,
        latent_dim=detected_latent_dim,
        decoder_layers=detected_decoder_layers,
        chunk_size=detected_chunk_size,
        action_dim=detected_action_dim,
        qpos_dim=detected_action_dim,
    )
    print(f"[*] Cấu hình mô hình nhận diện: d_model={cfg.d_model}, chunk_size={cfg.chunk_size}, action_dim={cfg.action_dim}, decoder_layers={cfg.decoder_layers}")
    model = ACTPolicy(cfg).to(device)

    # Tự động đồng bộ tên key (đặc biệt là 50 trainable action queries và module prefix)
    cleaned_state_dict = {}
    for k, v in state_dict.items():
        new_k = k
        if new_k.startswith("module."):
            new_k = new_k[7:]
        if new_k == "action_queries" and "policy_decoder.action_queries" not in state_dict:
            new_k = "policy_decoder.action_queries"
        cleaned_state_dict[new_k] = v

    try:
        model.load_state_dict(cleaned_state_dict, strict=True)
    except Exception as e:
        print(f"[!] Thử nạp linh hoạt (strict=False) do tên prefix: {e}")
        model.load_state_dict(cleaned_state_dict, strict=False)
    model.eval()
    print("[✓] Đã nạp thành công trọng số mô hình ACT!")

    # 2. Khởi tạo Camera, Robot Hardware, Trajectory Smoother & Temporal Ensemble
    target_img_size = (cfg.img_height, cfg.img_width)
    camera = CameraHandler(
        camera_type=args.camera,
        camera_fps=args.camera_fps,
        img_width=args.camera_width,
        img_height=args.camera_height,
        non_blocking=not args.blocking_cam,
    )
    # Tự động phát hiện đơn vị Gripper nếu để 'auto'
    gripper_unit_resolved = args.gripper_unit
    if gripper_unit_resolved == "auto":
        # Nếu mean/std của gripper trong dataset <= 0.1, model được huấn luyện theo đơn vị mét (stroke: 0..0.043m)
        if stats is not None and "qpos_mean" in stats and float(stats["qpos_mean"][7]) <= 0.1:
            gripper_unit_resolved = "stroke_m"
        else:
            gripper_unit_resolved = "rad"
    print(f"[*] Đơn vị Gripper hoạt động : {gripper_unit_resolved.upper()} (Đảo chiều Vai Trái J1: {args.invert_left_j1})")

    robot = BimanualOpenArmHardware(
        can_right=args.can_right,
        can_left=args.can_left,
        gripper_mode=args.gripper_mode,
        invert_left_j1=args.invert_left_j1,
        gripper_unit=gripper_unit_resolved,
        dry_run=args.dry_run,
        use_backend=args.backend,
        backend_host=args.backend_host,
        backend_port=args.backend_port,
        telemetry_port=args.telemetry_port,
    )
    ensemble = TemporalEnsemblePolicy(chunk_size=cfg.chunk_size, action_dim=16, ensemble_m=args.ensemble_m)
    smoother = TrajectorySmoother(vel_scale=args.vel_scale, dt=control_dt)

    q_home = np.zeros(16, dtype=np.float32)
    stable_stop_steps = 0
    stop_threshold_steps = max(5, int(args.stop_duration * args.control_hz))
    log_interval_steps = max(1, int(args.control_hz))
    start_time = time.time()

    # Đọc tư thế hiện tại của robot
    current_qpos = robot.get_qpos()
    smoother.reset(current_qpos)

    # 3. S-CURVE WARMUP: BƯỚC KHỞI ĐỘNG HÒA NHẬP ÊM ÁI
    print("\n[*] Đang lấy dữ liệu cảm biến đầu tiên để định hình quỹ đạo xuất phát...")
    rgb_init, depth_init = camera.get_rgbd()
    if rgb_init is not None:
        rgbd_init_tensor = preprocess_rgbd(rgb_init, depth_init, target_size=target_img_size).unsqueeze(0).to(device)
        norm_qpos_init = normalize_data(current_qpos, stats["qpos_mean"], stats["qpos_std"])
        qpos_init_tensor = torch.tensor(norm_qpos_init, dtype=torch.float32).unsqueeze(0).to(device)

        with torch.no_grad():
            first_pred_chunk, _, _ = model(image=rgbd_init_tensor, qpos=qpos_init_tensor, actions=None)
            first_pred_chunk = first_pred_chunk.squeeze(0).cpu().numpy()

        first_pred_rad = unnormalize_data(first_pred_chunk, stats["action_mean"], stats["action_std"])
        first_target = ensemble.update(first_pred_rad)

        # Nếu vị trí hiện tại lệch so với điểm bắt đầu của nhiệm vụ
        init_gap = np.max(np.abs(first_target - current_qpos))
        if args.smooth_warmup and init_gap > 0.03:
            print(f"[*] Phát hiện độ lệch vị trí ban đầu |Δq_max| = {init_gap:.4f} rad.")
            smooth_s_curve_move(
                robot=robot,
                q_from=current_qpos,
                q_to=first_target,
                duration=args.warmup_duration,
                control_dt=control_dt,
                description="S-Curve Khởi động hòa nhập vào quỹ đạo"
            )
            current_qpos = first_target.copy()
            smoother.reset(first_target)
        else:
            smoother.reset(current_qpos)

    print(f"\n[*] SẴN SÀNG! Robot bắt đầu thực thi chu trình điều khiển tự hành ở tần số {args.control_hz:.1f}Hz...")
    print("    (Bấm Ctrl + C bất kỳ lúc nào để DỪNG KHẨN CẤP / E-STOP)\n")

    executed_steps = 0
    try:
        for t in range(args.max_timesteps):
            loop_start = time.time()
            executed_steps = t

            # --- BƯỚC 1: ĐỌC CẢM BIẾN (CAMERA RGB-D + GÓC KHỚP QPOS) ---
            rgb, depth = camera.get_rgbd()
            if rgb is None:
                continue

            current_qpos = robot.get_qpos() # [16] float32

            # --- BƯỚC 2: TIỀN XỬ LÝ & DỰ ĐOÁN ACT ---
            rgbd_tensor = preprocess_rgbd(rgb, depth, target_size=target_img_size).unsqueeze(0).to(device)
            norm_qpos = normalize_data(current_qpos, stats["qpos_mean"], stats["qpos_std"])
            qpos_tensor = torch.tensor(norm_qpos, dtype=torch.float32).unsqueeze(0).to(device)

            with torch.no_grad():
                pred_chunk_norm, _, _ = model(image=rgbd_tensor, qpos=qpos_tensor, actions=None)
                pred_chunk_norm = pred_chunk_norm.squeeze(0).cpu().numpy()

            # Giải chuẩn hóa về Radian
            pred_chunk_rad = unnormalize_data(pred_chunk_norm, stats["action_mean"], stats["action_std"])

            # --- BƯỚC 3: TEMPORAL ENSEMBLING LÀM MƯỢT CẤP ĐỘ MODEL ---
            target_qpos = ensemble.update(pred_chunk_rad) # [16] góc mục tiêu sau ensembling

            # --- BƯỚC 4: TRAJECTORY SMOOTHER CẤP ĐỘ ĐỘNG HỌC PHẦN CỨNG ---
            # Giới hạn vận tốc và gia tốc cơ khí từng động cơ (DM8009, DM4340, DM4310)
            smooth_cmd_qpos = smoother.step(target_qpos)

            # --- BƯỚC 5: KIỂM TRA ĐIỀU KIỆN DỪNG TỰ NHIÊN (STOPPING CRITERIA) ---
            delta_q = np.max(np.abs(smooth_cmd_qpos - current_qpos))
            dist_home = np.linalg.norm(current_qpos - q_home)

            # Nếu biến thiên góc cực nhỏ và đã về gần vị trí Home
            if delta_q < args.stop_delta_rad and dist_home < args.stop_home_dist:
                stable_stop_steps += 1
                if stable_stop_steps >= stop_threshold_steps: # Đã đứng yên đủ thời gian tại Home
                    print("\n" + "=" * 80)
                    print(f"🎉 [DỪNG TỰ NHIÊN THÀNH CÔNG] Robot đã hoàn thành nhiệm vụ và trở về Home!")
                    print(f"   + Tổng số bước thực hiện : {t} timesteps ({t/args.control_hz:.2f}s)")
                    print(f"   + Khoảng cách tới Home   : {dist_home:.4f} rad")
                    print("=" * 80)
                    break
            else:
                stable_stop_steps = 0

            # --- BƯỚC 6: GỬI LỆNH XUỐNG DÂY CAN ---
            robot.send_target_positions(smooth_cmd_qpos)

            # In telemetry tóm tắt định kỳ
            if t % log_interval_steps == 0:
                elapsed = time.time() - start_time
                print(f"Step [{t:4d}/{args.max_timesteps}] | Time: {elapsed:5.1f}s | Max |Δq|: {delta_q:.4f} rad | Dist to Home: {dist_home:.3f} rad")

            # Duy trì chính xác chu kỳ control_dt
            loop_duration = time.time() - loop_start
            sleep_time = control_dt - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)

        else:
            # Vượt quá max_timesteps (Tầng dừng 2: Timeout)
            print("\n" + "=" * 80)
            print(f"⚠️ [DỪNG DO HẾT THỜI GIAN] Đã chạm giới hạn timeout {args.max_timesteps} steps (~{args.max_timesteps/args.control_hz:.1f}s). Tự động ngắt robot.")
            print("=" * 80)

    except KeyboardInterrupt:
        print("\n\n[!] PHÁT HIỆN LỆNH DỪNG KHẨN CẤP TỪ NGƯỜI DÙNG (CTRL + C)!")

    finally:
        # DỪNG AN TOÀN (SAFE HOME & SHUTDOWN):
        # Không ngắt điện đột ngột khiến tay rơi tự do!
        # Thực hiện S-curve đưa tay robot về vị trí Home an toàn hoặc giữ vững vị trí trước khi ngắt mô-men.
        if args.safe_home_on_stop and not args.dry_run:
            try:
                curr_pose = robot.get_qpos()
                smooth_s_curve_move(
                    robot=robot,
                    q_from=curr_pose,
                    q_to=q_home,
                    duration=2.0,
                    control_dt=control_dt,
                    description="S-Curve Thu tay về vị trí an toàn (Safe Home)"
                )
            except Exception as e:
                print(f"[!] Lỗi khi lùi về Home: {e}")

        # Ngắt mềm toàn bộ động cơ an toàn
        robot.disable_all()
        camera.stop()
        print("[✓] Toàn bộ hệ thống phần cứng đã được ngắt an toàn.\n")


if __name__ == "__main__":
    main()
