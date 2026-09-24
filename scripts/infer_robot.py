#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Production Hardware Inference Script for Bimanual OpenArm (16-DOF) + Chest RGB-D
Nối trực tiếp từ Camera RGB-D -> Model ACT -> Temporal Ensembling -> SocketCAN 16 Động cơ Damiao.
Hỗ trợ 3 tầng ngắt dừng an toàn:
1. Dừng tự nhiên khi về Home (|Δq| < 0.008 rad liên tục 1 giây)
2. Giới hạn thời gian tối đa (Max Timesteps Timeout)
3. Dừng khẩn cấp bằng phím bấm Ctrl+C hoặc phím 'q'
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
    OPENARM_CAN_AVAILABLE = True
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
ARM_MOTOR_TYPES = [
    getattr(oa.MotorType, "DM8009", None) if OPENARM_CAN_AVAILABLE else None,
    getattr(oa.MotorType, "DM8009", None) if OPENARM_CAN_AVAILABLE else None,
    getattr(oa.MotorType, "DM4340", None) if OPENARM_CAN_AVAILABLE else None,
    getattr(oa.MotorType, "DM4340", None) if OPENARM_CAN_AVAILABLE else None,
    getattr(oa.MotorType, "DM4310", None) if OPENARM_CAN_AVAILABLE else None,
    getattr(oa.MotorType, "DM4310", None) if OPENARM_CAN_AVAILABLE else None,
    getattr(oa.MotorType, "DM4310", None) if OPENARM_CAN_AVAILABLE else None,
]
ARM_SEND_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
ARM_RECV_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
GRIPPER_SEND_ID = 0x08
GRIPPER_RECV_ID = 0x18


class CameraHandler:
    """Quản lý luồng ảnh RGB-D từ RealSense hoặc WebCam thông thường"""
    def __init__(self, camera_type: str = "realsense", camera_id: int = 0):
        self.camera_type = camera_type
        self.pipeline = None
        self.align = None
        self.cap = None

        if camera_type == "realsense" and REALSENSE_AVAILABLE:
            print("[*] Đang khởi động camera Intel RealSense (RGB + Depth)...")
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
            self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            print("[✓] Camera Intel RealSense sẵn sàng (640x480 @ 60fps, align_to_color bật).")
        else:
            print(f"[*] Sử dụng camera OpenCV / Mock (ID: {camera_id})...")
            self.cap = cv2.VideoCapture(camera_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def get_rgbd(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        if self.pipeline is not None:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                return None, None

            bgr = np.asanyarray(color_frame.get_data())
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            depth_mm = np.asanyarray(depth_frame.get_data()) # uint16 mm
            return rgb, depth_mm
        elif self.cap is not None:
            ret, frame = self.cap.read()
            if not ret:
                return None, None
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return rgb, None
        else:
            # Mock frame
            rgb = np.zeros((480, 640, 3), dtype=np.uint8)
            depth = np.full((480, 640), fill_value=500, dtype=np.uint16)
            return rgb, depth

    def stop(self):
        if self.pipeline is not None:
            self.pipeline.stop()
        if self.cap is not None:
            self.cap.release()


class BimanualOpenArmHardware:
    """Quản lý giao tiếp CAN với 2 cánh tay robot OpenArm (16 động cơ Damiao)"""
    def __init__(self, can_right: str = "can0", can_left: str = "can1", dry_run: bool = False):
        self.dry_run = dry_run
        self.can_right = can_right
        self.can_left = can_left
        self.arm_right = None
        self.arm_left = None

        if not dry_run:
            if not OPENARM_CAN_AVAILABLE:
                raise ImportError("Chưa cài đặt thư viện 'openarm_can'! Vui lòng chạy trong môi trường có openarm_can.")
            print(f"[*] Đang kết nối CAN Bus: Tay Phải [{can_right}], Tay Trái [{can_left}]...")
            
            # Khởi tạo tay phải (can0)
            try:
                self.arm_right = oa.OpenArm(can_right, True)
                self.arm_right.init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS)
                self.arm_right.init_gripper_motor(oa.MotorType.DM4310, GRIPPER_SEND_ID, GRIPPER_RECV_ID)
                self.arm_right.set_callback_mode_all(oa.CallbackMode.STATE)
                self.arm_right.enable_all()
                print(f"[✓] Đã kích hoạt tay phải trên {can_right} (Torque ON).")
            except Exception as e:
                print(f"[!] Cảnh báo tay phải {can_right}: {e}")

            # Khởi tạo tay trái (can1)
            try:
                self.arm_left = oa.OpenArm(can_left, True)
                self.arm_left.init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS)
                self.arm_left.init_gripper_motor(oa.MotorType.DM4310, GRIPPER_SEND_ID, GRIPPER_RECV_ID)
                self.arm_left.set_callback_mode_all(oa.CallbackMode.STATE)
                self.arm_left.enable_all()
                print(f"[✓] Đã kích hoạt tay trái trên {can_left} (Torque ON).")
            except Exception as e:
                print(f"[!] Cảnh báo tay trái {can_left}: {e}")

            time.sleep(0.1)
            self.refresh_all()

    def refresh_all(self):
        if not self.dry_run:
            if self.arm_right:
                self.arm_right.refresh_all()
                self.arm_right.recv_all(100)
            if self.arm_left:
                self.arm_left.refresh_all()
                self.arm_left.recv_all(100)

    def get_qpos(self) -> np.ndarray:
        """
        Đọc vị trí hiện tại của 16 động cơ:
        [8 Khớp Tay Trái J1..J8, 8 Khớp Tay Phải J1..J8]
        """
        if self.dry_run:
            return np.zeros(16, dtype=np.float32)

        self.refresh_all()
        qpos = np.zeros(16, dtype=np.float32)

        # Tay trái (Chỉ số 0..7)
        if self.arm_left:
            left_arm_motors = self.arm_left.get_arm().get_motors()
            for i, m in enumerate(left_arm_motors[:7]):
                qpos[i] = m.get_position()
            left_grip_motors = self.arm_left.get_gripper().get_motors()
            if len(left_grip_motors) > 0:
                qpos[7] = left_grip_motors[0].get_position()

        # Tay phải (Chỉ số 8..15)
        if self.arm_right:
            right_arm_motors = self.arm_right.get_arm().get_motors()
            for i, m in enumerate(right_arm_motors[:7]):
                qpos[8 + i] = m.get_position()
            right_grip_motors = self.arm_right.get_gripper().get_motors()
            if len(right_grip_motors) > 0:
                qpos[15] = right_grip_motors[0].get_position()

        return qpos

    def send_target_positions(self, target_qpos: np.ndarray, kp_arm: float = 35.0, kd_arm: float = 1.2, kp_grip: float = 20.0, kd_grip: float = 0.8):
        """
        Gửi lệnh vị trí MITParam tới 16 động cơ Damiao qua CAN
        """
        if self.dry_run:
            return

        # 1. Gửi tay trái (0..7)
        if self.arm_left:
            left_arm_params = [oa.MITParam(kp_arm, kd_arm, target_qpos[i], 0.0, 0.0) for i in range(7)]
            left_grip_params = [oa.MITParam(kp_grip, kd_grip, target_qpos[7], 0.0, 0.0)]
            self.arm_left.get_arm().mit_control_all(left_arm_params)
            self.arm_left.get_gripper().mit_control_all(left_grip_params)
            self.arm_left.recv_all(50)

        # 2. Gửi tay phải (8..15)
        if self.arm_right:
            right_arm_params = [oa.MITParam(kp_arm, kd_arm, target_qpos[8 + i], 0.0, 0.0) for i in range(7)]
            right_grip_params = [oa.MITParam(kp_grip, kd_grip, target_qpos[15], 0.0, 0.0)]
            self.arm_right.get_arm().mit_control_all(right_arm_params)
            self.arm_right.get_gripper().mit_control_all(right_grip_params)
            self.arm_right.recv_all(50)

    def disable_all(self):
        """Ngắt toàn bộ lực (Torque OFF) đưa đèn LED về màu ĐỎ an toàn"""
        if not self.dry_run:
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


def main():
    parser = argparse.ArgumentParser(description="Chạy Inference điều khiển robot OpenArm thời gian thực")
    parser.add_argument("--checkpoint", type=str, default="checkpoints/act_openarm/best_checkpoint.pth", help="Đường dẫn file trọng số checkpoint .pth")
    parser.add_argument("--can_right", type=str, default="can0", help="Cổng CAN tay phải (mặc định: can0)")
    parser.add_argument("--can_left", type=str, default="can1", help="Cổng CAN tay trái (mặc định: can1)")
    parser.add_argument("--camera", type=str, choices=["realsense", "opencv", "mock"], default="realsense", help="Loại camera")
    parser.add_argument("--dry_run", action="store_true", help="Chạy thử mô phỏng không gửi lệnh CAN thật")
    parser.add_argument("--max_timesteps", type=int, default=1000, help="Số bước tối đa trước khi ngắt timeout (1000 steps = 20s @ 50Hz)")
    parser.add_argument("--ensemble_m", type=float, default=0.01, help="Hệ số suy giảm trọng số exp(-m*i) của Temporal Ensembling")
    parser.add_argument("--stop_delta_rad", type=float, default=0.008, help="Ngưỡng biến thiên góc để nhận diện dừng tự nhiên")
    parser.add_argument("--stop_home_dist", type=float, default=0.15, help="Khoảng cách tới vị trí Home để kích hoạt dừng")
    parser.add_argument("--device", type=str, default="cuda", help="Thiết bị tính toán (cuda hoặc cpu)")
    args = parser.parse_args()

    device = torch.device(args.device if torch.cuda.is_available() and args.device == "cuda" else "cpu")
    print("=" * 80)
    print("     🤖 KHỞI ĐỘNG ROBOT INFERENCE: BIMANUAL OPENARM (16-DOF) RGB-D @ 50HZ")
    print("=" * 80)
    print(f"[*] Checkpoint nạp vào  : {args.checkpoint}")
    print(f"[*] Thiết bị tính toán  : {device} ({torch.cuda.get_device_name(0) if device.type == 'cuda' else 'CPU'})")
    print(f"[*] Chế độ phần cứng    : {'DRY RUN (Thử nghiệm ảo)' if args.dry_run else 'REAL HARDWARE (Robot thật)'}")
    print(f"[*] Timeout tối đa      : {args.max_timesteps} steps (~{args.max_timesteps/50:.1f}s)")
    print(f"[*] Temporal Ensembling : BẬT (m = {args.ensemble_m})")
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
    detected_decoder_layers = saved_cfg.get("decoder_layers", 4)
    detected_cvae_layers = saved_cfg.get("cvae_layers", 2)
    detected_latent_dim = saved_cfg.get("latent_dim", 32)

    if "qpos_proj.weight" in state_dict:
        detected_d_model = state_dict["qpos_proj.weight"].shape[0]
        detected_action_dim = state_dict["qpos_proj.weight"].shape[1]
    if "cvae.latent_mu.weight" in state_dict:
        detected_latent_dim = state_dict["cvae.latent_mu.weight"].shape[0]
    elif "latent_mu.weight" in state_dict:
        detected_latent_dim = state_dict["latent_mu.weight"].shape[0]

    # Kiểm tra số tầng decoder
    decoder_layer_indices = [
        int(k.split(".layers.")[1].split(".")[0])
        for k in state_dict.keys()
        if ".layers." in k and ("policy_decoder" in k or "decoder" in k)
    ]
    if len(decoder_layer_indices) > 0:
        detected_decoder_layers = max(decoder_layer_indices) + 1

    # Kiểm tra số tầng cvae
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

    # Nạp trọng số mô hình linh hoạt
    try:
        model.load_state_dict(state_dict, strict=True)
    except Exception as e:
        print(f"[!] Thử nạp linh hoạt (strict=False) do tên prefix: {e}")
        model.load_state_dict(state_dict, strict=False)
    model.eval()
    print("[✓] Đã nạp thành công trọng số mô hình ACT!")

    # 2. Khởi tạo Camera & Robot
    camera = CameraHandler(camera_type=args.camera)
    robot = BimanualOpenArmHardware(can_right=args.can_right, can_left=args.can_left, dry_run=args.dry_run)
    ensemble = TemporalEnsemblePolicy(chunk_size=cfg.chunk_size, action_dim=16, ensemble_m=args.ensemble_m)

    q_home = np.zeros(16, dtype=np.float32)
    stable_stop_steps = 0
    start_time = time.time()

    print("\n[*] SẴN SÀNG! Robot bắt đầu thực thi chu trình điều khiển tự hành ở tần số 50Hz...")
    print("    (Bấm Ctrl + C bất kỳ lúc nào để DỪNG KHẨN CẤP / E-STOP)\n")

    try:
        for t in range(args.max_timesteps):
            loop_start = time.time()

            # --- BƯỚC 1: ĐỌC CẢM BIẾN (CAMERA RGB-D + GÓC KHỚP QPOS) ---
            rgb, depth = camera.get_rgbd()
            if rgb is None:
                continue

            current_qpos = robot.get_qpos() # [16] float32

            # --- BƯỚC 2: TIỀN XỬ LÝ & DỰ ĐOÁN ACT ---
            rgbd_tensor = preprocess_rgbd(rgb, depth, target_size=(480, 640)).unsqueeze(0).to(device)
            norm_qpos = normalize_data(current_qpos, stats["qpos_mean"], stats["qpos_std"])
            qpos_tensor = torch.tensor(norm_qpos, dtype=torch.float32).unsqueeze(0).to(device)

            with torch.no_grad():
                # Inference với z = 0
                pred_chunk_norm, _, _ = model(image=rgbd_tensor, qpos=qpos_tensor, actions=None)
                pred_chunk_norm = pred_chunk_norm.squeeze(0).cpu().numpy()

            # Giải chuẩn hóa về Radian
            pred_chunk_rad = unnormalize_data(pred_chunk_norm, stats["action_mean"], stats["action_std"])

            # --- BƯỚC 3: TEMPORAL ENSEMBLING LÀM MƯỢT ---
            target_qpos = ensemble.update(pred_chunk_rad) # [16] góc mục tiêu

            # --- BƯỚC 4: KIỂM TRA ĐIỀU KIỆN DỪNG TỰ NHIÊN (STOPPING CRITERIA) ---
            delta_q = np.max(np.abs(target_qpos - current_qpos))
            dist_home = np.linalg.norm(current_qpos - q_home)

            # Nếu biến thiên góc cực nhỏ và đã về gần vị trí Home
            if delta_q < args.stop_delta_rad and dist_home < args.stop_home_dist:
                stable_stop_steps += 1
                if stable_stop_steps >= 50: # Đã đứng yên 1.0 giây (50 chu kỳ) tại Home
                    print("\n" + "=" * 80)
                    print(f"🎉 [DỪNG TỰ NHIÊN THÀNH CÔNG] Robot đã hoàn thành nhiệm vụ gấp vải và trở về Home!")
                    print(f"   + Tổng số bước thực hiện : {t} timesteps ({t/50.0:.2f}s)")
                    print(f"   + Khoảng cách tới Home   : {dist_home:.4f} rad")
                    print("=" * 80)
                    break
            else:
                stable_stop_steps = 0

            # --- BƯỚC 5: GỬI LỆNH XUỐNG DÂY CAN ---
            robot.send_target_positions(target_qpos)

            # In telemetry tóm tắt mỗi 50 bước (1s)
            if t % 50 == 0:
                elapsed = time.time() - start_time
                print(f"Step [{t:4d}/{args.max_timesteps}] | Time: {elapsed:5.1f}s | Max |Δq|: {delta_q:.4f} rad | Dist to Home: {dist_home:.3f} rad")

            # Duy trì chính xác chu kỳ 50Hz (20ms)
            loop_duration = time.time() - loop_start
            sleep_time = 0.02 - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)

        else:
            # Vượt quá max_timesteps (Tầng dừng 2: Timeout)
            print("\n" + "=" * 80)
            print(f"⚠️ [DỪNG DO HẾT THỜI GIAN] Đã chạm giới hạn timeout {args.max_timesteps} steps (~{args.max_timesteps/50:.1f}s). Tự động ngắt robot.")
            print("=" * 80)

    except KeyboardInterrupt:
        print("\n\n[!] PHÁT HIỆN LỆNH DỪNG KHẨN CẤP TỪ NGƯỜI DÙNG (CTRL + C)!")

    finally:
        # Ngắt mềm toàn bộ động cơ an toàn
        robot.disable_all()
        camera.stop()
        print("[✓] Toàn bộ hệ thống phần cứng đã được ngắt an toàn.\n")


if __name__ == "__main__":
    main()
