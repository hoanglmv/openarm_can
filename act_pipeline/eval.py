#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Production Evaluation Pipeline for ACT (Action Chunking with Transformers)
Evaluates trained ACT checkpoints on Bimanual OpenArm (16-DOF) RGB-D dataset:
1. Reconstruction Accuracy: L1, MSE, Per-Joint MAE for 16 Damiao motors
2. Gripper Error Analysis: Left and Right Grippers
3. Real-Time Temporal Ensembling Simulation: Compares raw chunk vs exponential ensemble
4. Trajectory Smoothness / Jerk Metric: Evaluates motor wear & vibration reduction
5. Exports detailed evaluation report to JSON & pretty terminal table
"""

import os
import sys
import time
import json
import glob
import argparse
from typing import Dict, List, Optional, Tuple, Any

# Đảm bảo mã hóa UTF-8 an toàn trên Windows và Linux
if sys.stdout.encoding != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
import h5py

from act_pipeline.config import ModelConfig
from act_pipeline.models.act_model import ACTPolicy
from act_pipeline.data.dataset import BimanualEpisodicDataset
from act_pipeline.data.normalization import load_norm_stats, unnormalize_data, normalize_data
from act_pipeline.data.preprocess import preprocess_rgbd
from act_pipeline.utils.checkpoint import CheckpointManager
from act_pipeline.utils.temporal_ensemble import TemporalEnsemblePolicy

# Tên 16 khớp của OpenArm theo chuẩn DATA_FORMAT_SPECIFICATION.md
JOINT_NAMES = [
    "Left_J1_Base", "Left_J2_Shoulder", "Left_J3_Elbow_Pitch", "Left_J4_Elbow_Roll",
    "Left_J5_Wrist_Pitch", "Left_J6_Wrist_Roll", "Left_J7_Wrist_Yaw", "Left_J8_Gripper",
    "Right_J1_Base", "Right_J2_Shoulder", "Right_J3_Elbow_Pitch", "Right_J4_Elbow_Roll",
    "Right_J5_Wrist_Pitch", "Right_J6_Wrist_Roll", "Right_J7_Wrist_Yaw", "Right_J8_Gripper"
]


def evaluate_dataset_batch(
    model: nn.Module,
    dataloader: DataLoader,
    stats: Dict[str, np.ndarray],
    device: torch.device,
) -> Dict[str, Any]:
    """
    Đánh giá độ chính xác phục dựng (Reconstruction Metrics) trên toàn bộ DataLoader.
    """
    model.eval()
    total_l1_norm = 0.0
    total_l1_rad = 0.0
    total_mse_rad = 0.0
    total_samples = 0

    per_joint_mae_rad = np.zeros(16, dtype=np.float64)
    action_std = stats["action_std"]

    with torch.no_grad():
        for batch in dataloader:
            image = batch["image"].to(device, non_blocking=True)
            qpos = batch["qpos"].to(device, non_blocking=True)
            actions = batch["actions"].to(device, non_blocking=True)
            is_pad = batch["is_pad"].to(device, non_blocking=True)

            # Dự đoán với z = 0 (chuẩn inference)
            pred_actions, _, _ = model(image=image, qpos=qpos, actions=None)

            valid_mask = (~is_pad).unsqueeze(-1).float() # [B, chunk_size, 1]
            diff = torch.abs(pred_actions - actions) * valid_mask # [B, chunk_size, 16]

            # Loss trên không gian đã chuẩn hóa
            total_l1_norm += (diff.sum() / valid_mask.sum().clamp(min=1.0) / 16.0).item() * image.shape[0]

            # Chuyển đổi về đơn vị Radian thực tế
            diff_rad = diff.cpu().numpy() * action_std
            valid_mask_np = valid_mask.cpu().numpy()

            per_joint_mae_rad += (diff_rad * valid_mask_np).sum(axis=(0, 1))
            total_samples += valid_mask_np.sum()

            diff_rad_sq = (diff_rad ** 2) * valid_mask_np
            total_mse_rad += diff_rad_sq.sum()

    avg_l1_norm = total_l1_norm / len(dataloader.dataset)
    avg_per_joint_mae_rad = (per_joint_mae_rad / max(total_samples / 16.0, 1.0)).tolist()
    avg_l1_rad = float(np.mean(avg_per_joint_mae_rad))
    avg_mse_rad = float(total_mse_rad / max(total_samples, 1.0))

    # Tách sai số 2 tay và 2 kẹp
    left_arm_mae = float(np.mean(avg_per_joint_mae_rad[:7]))
    left_gripper_mae = float(avg_per_joint_mae_rad[7])
    right_arm_mae = float(np.mean(avg_per_joint_mae_rad[8:15]))
    right_gripper_mae = float(avg_per_joint_mae_rad[15])

    return {
        "l1_norm": avg_l1_norm,
        "l1_rad": avg_l1_rad,
        "mse_rad": avg_mse_rad,
        "left_arm_mae_rad": left_arm_mae,
        "left_gripper_mae_rad": left_gripper_mae,
        "right_arm_mae_rad": right_arm_mae,
        "right_gripper_mae_rad": right_gripper_mae,
        "per_joint_mae_rad": avg_per_joint_mae_rad,
        "max_joint_error_rad": float(np.max(avg_per_joint_mae_rad)),
        "max_joint_name": JOINT_NAMES[int(np.argmax(avg_per_joint_mae_rad))],
    }


def simulate_episode_rollout(
    model: nn.Module,
    hdf5_path: str,
    stats: Dict[str, np.ndarray],
    device: torch.device,
    ensemble_m: float = 0.01,
    chunk_size: int = 50,
) -> Dict[str, Any]:
    """
    Giả lập Rollout liên tục theo thời gian thực (50Hz) trên 1 file Episode:
    So sánh quỹ đạo hành động thô (Raw Chunk) vs Làm mượt bằng Temporal Ensembling (exp(-m*i)).
    """
    model.eval()
    ensemble = TemporalEnsemblePolicy(chunk_size=chunk_size, action_dim=16, ensemble_m=ensemble_m)
    ensemble.reset()

    with h5py.File(hdf5_path, "r") as f:
        gt_actions = f["action"][:].astype(np.float32)
        qpos_seq = f["observations/qpos"][:].astype(np.float32)
        T = len(gt_actions)

        obs_img = f["observations/images"]
        raw_rgb = None
        raw_depth = None
        if "chest_rgb" in obs_img and "chest_depth" in obs_img:
            raw_rgb = obs_img["chest_rgb"]
            raw_depth = obs_img["chest_depth"]
        elif "chest" in obs_img:
            raw_rgb = obs_img["chest"]
            raw_depth = obs_img["chest_depth"] if "chest_depth" in obs_img else None

        raw_pred_actions = []
        ensemble_pred_actions = []

        for t in range(T):
            # 1. Tiền xử lý RGB-D tại bước t
            rgb_t = raw_rgb[t] if raw_rgb.ndim == 4 else raw_rgb[t][..., :3]
            depth_t = raw_depth[t] if raw_depth is not None else None
            
            rgbd_tensor = preprocess_rgbd(rgb_t, depth_t).unsqueeze(0).to(device) # [1, 4, H, W]

            # 2. Tiền xử lý qpos
            norm_qpos = normalize_data(qpos_seq[t], stats["qpos_mean"], stats["qpos_std"])
            qpos_tensor = torch.tensor(norm_qpos, dtype=torch.float32).unsqueeze(0).to(device)

            # 3. Model Inference (z = 0)
            with torch.no_grad():
                pred_chunk_norm, _, _ = model(image=rgbd_tensor, qpos=qpos_tensor, actions=None)
                pred_chunk_norm = pred_chunk_norm.squeeze(0).cpu().numpy()

            # Giải chuẩn hóa về Radian
            pred_chunk_rad = unnormalize_data(pred_chunk_norm, stats["action_mean"], stats["action_std"])

            # Hành động thô bước đầu tiên của chunk
            raw_pred_actions.append(pred_chunk_rad[0])

            # Hành động làm mượt qua Temporal Ensembling
            smoothed_act = ensemble.update(pred_chunk_rad)
            ensemble_pred_actions.append(smoothed_act)

    raw_pred_actions = np.array(raw_pred_actions)       # [T, 16]
    ensemble_pred_actions = np.array(ensemble_pred_actions) # [T, 16]

    # Tính độ giật (Jerk) = Gia tốc bậc 2 của quỹ đạo: diff(diff(act))
    raw_jerk = np.mean(np.abs(np.diff(np.diff(raw_pred_actions, axis=0), axis=0)))
    ensemble_jerk = np.mean(np.abs(np.diff(np.diff(ensemble_pred_actions, axis=0), axis=0)))
    jerk_reduction_pct = ((raw_jerk - ensemble_jerk) / max(raw_jerk, 1e-6)) * 100.0

    raw_mae = float(np.mean(np.abs(raw_pred_actions - gt_actions)))
    ensemble_mae = float(np.mean(np.abs(ensemble_pred_actions - gt_actions)))

    return {
        "episode_name": os.path.basename(hdf5_path),
        "timesteps": T,
        "raw_mae_rad": raw_mae,
        "ensemble_mae_rad": ensemble_mae,
        "raw_jerk": float(raw_jerk),
        "ensemble_jerk": float(ensemble_jerk),
        "jerk_reduction_pct": float(jerk_reduction_pct),
    }


def eval_pipeline(args):
    """
    Pipeline đánh giá chính thức
    """
    device = torch.device(args.device if torch.cuda.is_available() and args.device == "cuda" else "cpu")
    os.makedirs(args.output_dir, exist_ok=True)

    print("=" * 80)
    print("   🔍 KHỞI ĐỘNG PIPELINE ĐÁNH GIÁ MÔ HÌNH ACT - OPENARM BIMANUAL (16-DOF)")
    print("=" * 80)
    print(f"[*] Checkpoint nạp vào  : {args.checkpoint_path}")
    print(f"[*] Thư mục Dataset     : {args.dataset_dir}")
    print(f"[*] Thiết bị kiểm tra   : {device} ({torch.cuda.get_device_name(0) if device.type == 'cuda' else 'CPU'})")
    print(f"[*] Temporal Ensembling : {'BẬT (exp(-m*i))' if args.temporal_ensemble else 'TẮT'}")
    print("=" * 80)

    # 1. Tải Checkpoint & Thống kê chuẩn hóa (Stats)
    if not os.path.exists(args.checkpoint_path):
        print(f"[X] LỖI: Không tìm thấy file checkpoint tại: {args.checkpoint_path}")
        sys.exit(1)

    checkpoint_data = torch.load(args.checkpoint_path, map_location="cpu", weights_only=False)
    stats = checkpoint_data.get("stats")
    
    # Nếu trong checkpoint chưa có stats, thử tìm file stats.pkl cùng thư mục
    if stats is None:
        stats_fallback = os.path.join(os.path.dirname(args.checkpoint_path), "dataset_stats.pkl")
        if os.path.exists(stats_fallback):
            stats = load_norm_stats(stats_fallback)
            print(f"[✓] Đã nạp stats từ fallback: {stats_fallback}")
        else:
            raise FileNotFoundError("Không tìm thấy thống kê chuẩn hóa trong checkpoint hoặc thư mục!")

    # 2. Khởi tạo mô hình
    saved_cfg = checkpoint_data.get("config", {})
    model_cfg = ModelConfig(
        in_channels=4,
        backbone_type="resnet18",
        freeze_backbone=True,
        d_model=saved_cfg.get("d_model", 512),
        nheads=saved_cfg.get("nheads", 8),
        dim_feedforward=saved_cfg.get("dim_feedforward", 2048),
        dropout=0.0,
        cvae_layers=saved_cfg.get("cvae_layers", 2),
        latent_dim=saved_cfg.get("latent_dim", 32),
        decoder_layers=saved_cfg.get("decoder_layers", 4),
        chunk_size=saved_cfg.get("chunk_size", 50),
        action_dim=saved_cfg.get("action_dim", 16),
        qpos_dim=saved_cfg.get("qpos_dim", 16),
    )
    model = ACTPolicy(config=model_cfg).to(device)

    # Nạp trọng số
    state_dict = checkpoint_data["model_state_dict"] if "model_state_dict" in checkpoint_data else checkpoint_data
    model.load_state_dict(state_dict, strict=True)
    model.eval()
    print("[✓] Đã nạp thành công 100% trọng số mô hình.")

    # 3. Tạo DataLoader Đánh giá
    eval_files = sorted(glob.glob(os.path.join(args.dataset_dir, "*.hdf5")))
    if len(eval_files) == 0:
        print(f"[X] LỖI: Không tìm thấy file HDF5 nào trong: {args.dataset_dir}")
        sys.exit(1)

    eval_dataset = BimanualEpisodicDataset(
        file_paths=eval_files,
        stats=stats,
        chunk_size=model_cfg.chunk_size,
        step_stride=2,
        target_img_size=(model_cfg.img_height, model_cfg.img_width),
    )
    eval_loader = DataLoader(
        eval_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
    )

    # 4. Tính toán độ chính xác Batch
    print("\n[*] Đang chạy đánh giá định lượng trên toàn bộ dataset...")
    batch_results = evaluate_dataset_batch(model, eval_loader, stats, device)

    # 5. Giả lập Rollout thời gian thực có Temporal Ensembling trên episode đầu tiên
    print("[*] Đang giả lập kiểm tra Temporal Ensembling thời gian thực (50Hz)...")
    rollout_results = simulate_episode_rollout(
        model=model,
        hdf5_path=eval_files[0],
        stats=stats,
        device=device,
        ensemble_m=args.ensemble_m,
        chunk_size=model_cfg.chunk_size,
    )

    # 6. Hiển thị Bảng Báo Cáo Chi Tiết
    print("\n" + "=" * 80)
    print("                      📊 KẾT QUẢ ĐÁNH GIÁ MÔ HÌNH ACT")
    print("=" * 80)
    print(f"  + Sai số L1 chuẩn hóa (Norm L1 Loss)       : {batch_results['l1_norm']:.4f}")
    print(f"  + Sai số trung bình góc (Mean L1 Radian)   : {batch_results['l1_rad']:.4f} rad (~{np.degrees(batch_results['l1_rad']):.2f}°)")
    print(f"  + Sai số bình phương (MSE Radian)          : {batch_results['mse_rad']:.6f} rad²")
    print(f"  + Sai số trung bình Tay Trái (J1..J7)      : {batch_results['left_arm_mae_rad']:.4f} rad (~{np.degrees(batch_results['left_arm_mae_rad']):.2f}°)")
    print(f"  + Sai số Kẹp Trái (Gripper J8)             : {batch_results['left_gripper_mae_rad']:.4f} rad")
    print(f"  + Sai số trung bình Tay Phải (J1..J7)     : {batch_results['right_arm_mae_rad']:.4f} rad (~{np.degrees(batch_results['right_arm_mae_rad']):.2f}°)")
    print(f"  + Sai số Kẹp Phải (Gripper J8)            : {batch_results['right_gripper_mae_rad']:.4f} rad")
    print(f"  + Khớp có độ lệch lớn nhất                 : {batch_results['max_joint_name']} ({batch_results['max_joint_error_rad']:.4f} rad)")
    print("-" * 80)
    print("  🌀 HIỆU QUẢ CỦA BỘ GHÉP THỜI GIAN (TEMPORAL ENSEMBLING @ 50Hz):")
    print(f"  + Quỹ đạo thử nghiệm                       : {rollout_results['episode_name']} ({rollout_results['timesteps']} steps)")
    print(f"  + Sai số Hành động Thô (Raw Chunk MAE)     : {rollout_results['raw_mae_rad']:.4f} rad")
    print(f"  + Sai số Làm Mượt (Ensemble MAE)           : {rollout_results['ensemble_mae_rad']:.4f} rad")
    print(f"  + Độ giật thô (Raw Chunk Jerk)             : {rollout_results['raw_jerk']:.6f}")
    print(f"  + Độ giật sau làm mượt (Smoothed Jerk)     : {rollout_results['ensemble_jerk']:.6f}")
    print(f"  + Mức độ giảm rung lắc cơ khí cho motor   : 🌟 Giảm {rollout_results['jerk_reduction_pct']:.1f}% rung giật!")
    print("=" * 80)

    # Hiển thị bảng chi tiết 16 khớp
    print("\n📋 CHI TIẾT SAI SỐ TỪNG KHỚP (PER-JOINT MAE):")
    print(f"{'Khớp (Joint)':<26} | {'MAE (Rad)':<12} | {'MAE (Độ)':<10} | {'Đánh giá':<15}")
    print("-" * 70)
    for i, name in enumerate(JOINT_NAMES):
        mae_r = batch_results['per_joint_mae_rad'][i]
        mae_d = np.degrees(mae_r)
        status = "Tuyệt vời (✓)" if mae_d < 3.0 else ("Tốt (✓)" if mae_d < 6.0 else "Cần thêm data (!)")
        print(f"{name:<26} | {mae_r:<12.4f} | {mae_d:<10.2f}° | {status:<15}")
    print("-" * 70)

    # 7. Xuất file JSON Báo Cáo
    report_data = {
        "checkpoint": args.checkpoint_path,
        "dataset": args.dataset_dir,
        "batch_metrics": batch_results,
        "rollout_metrics": rollout_results,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    report_file = os.path.join(args.output_dir, "eval_report.json")
    with open(report_file, "w", encoding="utf-8") as f:
        json.dump(report_data, f, indent=2)
    print(f"\n[✓] Đã xuất toàn bộ báo cáo đánh giá ra file: {report_file}\n")


def parse_args():
    parser = argparse.ArgumentParser(description="Đánh giá mô hình ACT Bimanual OpenArm")
    parser.add_argument("--checkpoint_path", type=str, default="checkpoints/act_openarm/best_checkpoint.pth", help="Đường dẫn file checkpoint .pth")
    parser.add_argument("--dataset_dir", type=str, default="dataset/real_towel_folding", help="Thư mục chứa các file .hdf5 kiểm thử")
    parser.add_argument("--output_dir", type=str, default="evaluation_results", help="Thư mục xuất báo cáo đánh giá")
    parser.add_argument("--batch_size", type=int, default=16, help="Kích thước batch đánh giá")
    parser.add_argument("--num_workers", type=int, default=2, help="Số worker nạp dữ liệu")
    parser.add_argument("--device", type=str, default="cuda", help="Thiết bị (cuda hoặc cpu)")
    parser.add_argument("--no_ensemble", action="store_true", help="Tắt giả lập Temporal Ensembling")
    parser.add_argument("--ensemble_m", type=float, default=0.01, help="Hệ số suy giảm trọng số exp(-m*i)")

    args = parser.parse_args()
    args.temporal_ensemble = not args.no_ensemble
    return args


if __name__ == "__main__":
    args = parse_args()
    eval_pipeline(args)
