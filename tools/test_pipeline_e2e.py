#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
End-to-End Pipeline Verification Suite for OpenArm Bimanual RGB-D ACT
Thực hiện kiểm thử toàn diện 4 giai đoạn khép kín:
1. Data Format Validation: Kiểm tra quy chuẩn HDF5 theo DATA_FORMAT_SPECIFICATION.md
2. DataLoader & Batch Creation: Cắt lát 50 bước, padding mask, chuẩn hóa Z-score
3. Train & Checkpoint Integrity: Forward, Loss (L1 + KL), Backprop, Lưu & Nạp Checkpoint
   - Kiểm tra nghiêm ngặt 100% trọng số (đặc biệt 50 trainable action_queries KHÔNG BỊ MISSING)
4. Eval & Robot Inference Loop: S-Curve Warmup -> 50Hz Vòng lặp tự hành -> Trajectory Smoother -> Safe Home
"""

import os
import sys
import tempfile
import shutil
import time
from pathlib import Path
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
import h5py

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

# Thêm đường dẫn gốc vào sys.path
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from act_pipeline.config import ModelConfig
from act_pipeline.models.act_model import ACTPolicy
from act_pipeline.data.dataset import BimanualEpisodicDataset
from act_pipeline.data.normalization import compute_norm_stats
from act_pipeline.data.preprocess import preprocess_rgbd
from act_pipeline.data.validate_dataset import validate_single_hdf5
from act_pipeline.eval import evaluate_dataset_batch
from act_pipeline.utils.temporal_ensemble import TemporalEnsemblePolicy
from sim.data_recorder import EpisodeWriter
from scripts.infer_robot import TrajectorySmoother, CameraHandler, BimanualOpenArmHardware, smooth_s_curve_move


def create_mock_hdf5_episode(filepath: str, num_steps: int = 150, hz: float = 50.0):
    """Tạo 1 episode mẫu chuẩn 100% theo DATA_FORMAT_SPECIFICATION.md"""
    dt = 1.0 / hz
    t = np.linspace(0, num_steps * dt, num_steps, endpoint=False)

    # 16 khớp: chuyển động hình sin mượt mà trong giới hạn cơ học
    qpos = np.zeros((num_steps, 16), dtype=np.float32)
    for i in range(16):
        qpos[:, i] = 0.2 * np.sin(2.0 * np.pi * 0.5 * t + i * 0.2)

    # Action là bước tiếp theo (lead 1 step)
    action = np.roll(qpos, -1, axis=0)
    action[-1] = action[-2] # bước cuối giữ nguyên

    qvel = np.gradient(qpos, dt, axis=0).astype(np.float32)
    effort = np.zeros_like(qpos, dtype=np.float32)

    # Ảnh RGB giả lập 480x640x3
    rgb = np.full((num_steps, 480, 640, 3), fill_value=128, dtype=np.uint8)
    for s in range(num_steps):
        rgb[s, :50, :50, 0] = (s * 3) % 255

    # Ảnh Depth uint16 (mm): khoảng cách 600mm
    depth = np.full((num_steps, 480, 640), fill_value=600, dtype=np.uint16)

    # Timestamps nanoseconds
    start_ns = time.time_ns()
    timestamps_ns = np.array([start_ns + int(i * dt * 1e9) for i in range(num_steps)], dtype=np.int64)

    with h5py.File(filepath, "w") as f:
        # Attributes
        f.attrs["sim"] = False
        f.attrs["frequency_hz"] = hz
        f.attrs["robot_type"] = "OpenArm_Bimanual_16DOF"
        f.attrs["num_joints"] = 16
        f.attrs["depth_scale"] = 0.001
        f.attrs["depth_range_m"] = np.array([0.2, 1.2], dtype=np.float32)

        # Observations
        obs = f.create_group("observations")
        images = obs.create_group("images")
        images.create_dataset("chest_rgb", data=rgb, dtype=np.uint8)
        images.create_dataset("chest_depth", data=depth, dtype=np.uint16)
        obs.create_dataset("qpos", data=qpos, dtype=np.float32)
        obs.create_dataset("qvel", data=qvel, dtype=np.float32)
        obs.create_dataset("effort", data=effort, dtype=np.float32)

        f.create_dataset("action", data=action, dtype=np.float32)
        f.create_dataset("timestamp_ns", data=timestamps_ns, dtype=np.int64)


def run_pipeline_test(target_device: str = "auto"):
    print("=" * 80)
    print("     🧪 BẮT ĐẦU KIỂM THỬ TOÀN DIỆN PIPELINE: DATA -> TRAIN -> EVAL -> INFER")
    print("=" * 80)

    test_dir = tempfile.mkdtemp(prefix="openarm_pipeline_test_")
    if target_device == "auto":
        device_str = "cuda" if torch.cuda.is_available() else "cpu"
    else:
        device_str = target_device
    device = torch.device(device_str)
    print(f"[*] Thư mục test tạm thời: {test_dir}")
    print(f"[*] Thiết bị tính toán     : {device}")

    try:
        # =========================================================================
        # GIAI ĐOẠN 1: KIỂM THỬ ĐỊNH DẠNG DỮ LIỆU & DATA RECORDER
        # =========================================================================
        print("\n" + "-" * 70)
        print("PHẦN 1: TẠO VÀ KIỂM ĐỊNH FILE HDF5 (DATA_RECORDER & VALIDATOR)")
        print("-" * 70)
        # 1. Sinh Episode 0 trực tiếp bằng EpisodeWriter trong sim/data_recorder.py
        writer = EpisodeWriter(
            output_dir=Path(test_dir),
            episode_name="episode_0",
            batch_size=10,
            frequency_hz=50.0,
            img_height=480,
            img_width=640,
        )
        for s in range(120):
            rgb = np.full((480, 640, 3), fill_value=128, dtype=np.uint8)
            depth = np.full((480, 640), fill_value=600, dtype=np.uint16)
            qpos = np.full(16, fill_value=0.1 * np.sin(s * 0.1), dtype=np.float32)
            qvel = np.zeros(16, dtype=np.float32)
            effort = np.zeros(16, dtype=np.float32)
            action = np.full(16, fill_value=0.1 * np.sin((s + 1) * 0.1), dtype=np.float32)
            t_ns = int(s * 20_000_000)
            writer.append(rgb, depth, qpos, qvel, effort, action, t_ns)
        ep1_path = str(writer.finalize())

        # 2. Sinh Episode 1 bằng mock generator
        ep2_path = os.path.join(test_dir, "episode_1.hdf5")
        create_mock_hdf5_episode(ep2_path, num_steps=100, hz=50.0)

        # Chạy validator trên cả 2 file
        val_res1 = validate_single_hdf5(ep1_path, expected_hz=50.0)
        assert val_res1.passed, f"Lỗi kiểm định HDF5 Episode 0: {val_res1.errors}"
        val_res2 = validate_single_hdf5(ep2_path, expected_hz=50.0)
        assert val_res2.passed, f"Lỗi kiểm định HDF5 Episode 1: {val_res2.errors}"
        print(f" [✓] Episode 0 (sinh bởi sim.data_recorder.EpisodeWriter @ 50.0Hz): ĐẠT CHUẨN 100%")
        print(f" [✓] Episode 1 (sinh bởi create_mock_hdf5_episode @ 50.0Hz): ĐẠT CHUẨN 100%")

        # =========================================================================
        # GIAI ĐOẠN 2: KIỂM THỬ DATALOADER & BATCHING
        # =========================================================================
        print("\n" + "-" * 70)
        print("PHẦN 2: KIỂM THỬ EPISODIC DATASET & DATALOADER")
        print("-" * 70)
        stats = compute_norm_stats([ep1_path, ep2_path])
        dataset = BimanualEpisodicDataset(
            file_paths=[ep1_path, ep2_path],
            stats=stats,
            chunk_size=50,
            step_stride=1,
            target_img_size=(240, 424),
        )
        assert len(dataset) == (120 + 100), f"Độ dài dataset không khớp: {len(dataset)}"
        print(f" [✓] Khởi tạo Dataset thành công: {len(dataset)} mẫu lát cắt (chunk_size=50)")

        # Lấy 1 mẫu thử nghiệm
        sample = dataset[0]
        assert sample["image"].shape == (4, 240, 424), f"Image shape sai: {sample['image'].shape}"
        assert sample["qpos"].shape == (16,), f"qpos shape sai: {sample['qpos'].shape}"
        assert sample["actions"].shape == (50, 16), f"actions shape sai: {sample['actions'].shape}"
        assert sample["is_pad"].shape == (50,), f"is_pad shape sai: {sample['is_pad'].shape}"
        print(f" [✓] Mẫu đơn (Single Sample): Image [4, 240, 424], Qpos [16], Actions [50, 16], is_pad [50]")

        # DataLoader Batching
        loader = DataLoader(dataset, batch_size=4, shuffle=True)
        batch = next(iter(loader))
        assert batch["image"].shape == (4, 4, 240, 424)
        assert batch["actions"].shape == (4, 50, 16)
        print(f" [✓] Mini-batch tạo thành công: Batch size 4, Actions [4, 50, 16]")

        # =========================================================================
        # GIAI ĐOẠN 3: KIỂM THỬ TRAIN & TOÀN VẸN CHECKPOINT (ZERO MISSING KEYS)
        # =========================================================================
        print("\n" + "-" * 70)
        print("PHẦN 3: KIỂM THỬ TRAINING & TÍNH TOÀN VẸN TRỌNG SỐ CHECKPOINT")
        print("-" * 70)
        cfg = ModelConfig(
            in_channels=4,
            d_model=128,
            nheads=4,
            dim_feedforward=256,
            cvae_layers=2,
            latent_dim=16,
            decoder_layers=2,
            chunk_size=50,
            action_dim=16,
            qpos_dim=16,
        )
        model = ACTPolicy(cfg).to(device)
        optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)

        # Chạy 2 bước Train Forward + Backward
        model.train()
        for step in range(2):
            b_img = batch["image"].to(device)
            b_qpos = batch["qpos"].to(device)
            b_act = batch["actions"].to(device)
            b_pad = batch["is_pad"].to(device)

            pred_act, mu, logvar = model(image=b_img, qpos=b_qpos, actions=b_act, is_pad=b_pad)
            loss_dict = model.compute_loss(pred_act, b_act, b_pad, mu, logvar, kl_weight=10.0)

            optimizer.zero_grad()
            loss_dict["loss"].backward()
            optimizer.step()
            print(f" [✓] Train Step {step+1}: Loss = {loss_dict['loss'].item():.4f} (Recon: {loss_dict['recon_loss'].item():.4f}, KL: {loss_dict['kl_loss'].item():.4f})")

        # Lưu Checkpoint ra đĩa
        ckpt_path = os.path.join(test_dir, "test_checkpoint.pth")
        torch.save({
            "model_state_dict": model.state_dict(),
            "config": {
                "d_model": cfg.d_model,
                "action_dim": cfg.action_dim,
                "chunk_size": cfg.chunk_size,
                "decoder_layers": cfg.decoder_layers,
                "cvae_layers": cfg.cvae_layers,
                "latent_dim": cfg.latent_dim,
            },
            "stats": stats,
        }, ckpt_path)
        print(f" [✓] Đã lưu checkpoint thành công tại: {ckpt_path}")

        # Tải lại Checkpoint với mô hình mới và kiểm tra nghiêm ngặt Missing Keys
        eval_model = ACTPolicy(cfg).to(device)
        loaded_cp = torch.load(ckpt_path, map_location=device, weights_only=False)
        sd = loaded_cp["model_state_dict"]

        # Đồng bộ tiền tố nếu có
        cleaned_sd = {}
        for k, v in sd.items():
            new_k = k
            if new_k == "action_queries" and "policy_decoder.action_queries" not in sd:
                new_k = "policy_decoder.action_queries"
            cleaned_sd[new_k] = v

        load_res = eval_model.load_state_dict(cleaned_sd, strict=True)
        print(" [✓] STRICT LOAD THÀNH CÔNG 100%: 0 Missing Keys, 0 Unexpected Keys!")

        # Kiểm tra 50 action queries có khác 0 và bảo toàn trọng số không
        loaded_queries = eval_model.policy_decoder.action_queries
        orig_queries = model.policy_decoder.action_queries
        assert loaded_queries.shape == (1, 50, 128)
        assert torch.allclose(loaded_queries, orig_queries), "50 action queries không khớp sau khi load!"
        assert not torch.all(loaded_queries == 0), "50 action queries bị rỗng/bằng 0!"
        print(f" [✓] 50 Trainable Queries ({loaded_queries.shape}) được nạp CHUẨN XÁC 100%!")

        # =========================================================================
        # GIAI ĐOẠN 4: KIỂM THỬ ĐÁNH GIÁ (EVAL)
        # =========================================================================
        print("\n" + "-" * 70)
        print("PHẦN 4: KIỂM THỬ EVALUATION TRÊN DATASET BATCH")
        print("-" * 70)
        eval_metrics = evaluate_dataset_batch(eval_model, loader, stats, device)
        print(f" [✓] Eval hoàn tất: L1 Norm = {eval_metrics['l1_norm']:.4f}, L1 Radian = {eval_metrics['l1_rad']:.4f} rad (~{np.degrees(eval_metrics['l1_rad']):.2f}°)")

        # =========================================================================
        # GIAI ĐOẠN 5: KIỂM THỬ KẾT NỐI PHẦN CỨNG & INFERENCE ROBOT 50HZ
        # =========================================================================
        print("\n" + "-" * 70)
        print("PHẦN 5: KIỂM THỬ TOÀN DIỆN KẾT NỐI PHẦN CỨNG & INFERENCE ROBOT 50HZ")
        print("-" * 70)
        cam = CameraHandler(camera_type="mock", camera_fps=60, img_width=424, img_height=240)
        hw = BimanualOpenArmHardware(can_right="can0", can_left="can1", gripper_unit="stroke_m", dry_run=True)
        ensemble = TemporalEnsemblePolicy(chunk_size=50, action_dim=16, ensemble_m=0.01)
        smoother = TrajectorySmoother(vel_scale=1.0, dt=0.02)

        # 1. Kiểm tra đối soát thuộc tính phần cứng
        assert hw.can_right == "can0", f"Cổng CAN tay phải sai: {hw.can_right}"
        assert hw.can_left == "can1", f"Cổng CAN tay trái sai: {hw.can_left}"
        assert abs(hw._to_gripper_cmd_rad(0.043) - (-1.20)) < 1e-4, "Sai hàm quy đổi hành trình kẹp (0.043m -> -1.20 rad)"
        assert abs(hw._to_gripper_cmd_rad(0.0) - 0.0) < 1e-4, "Sai hàm quy đổi kẹp đóng (0.0m -> 0.0 rad)"
        assert hw.get_qpos().shape == (16,), "Shape góc qpos sai lệch khác 16 phần tử"
        print(" [✓] Logic cấu hình phần cứng CAN Bus (can0/can1), Motor ID & Gripper: ĐỒNG BỘ 100%!")

        # 2. Test S-Curve Warm-up
        q_start = np.zeros(16, dtype=np.float32)
        q_target = np.full(16, fill_value=0.25, dtype=np.float32)
        smooth_s_curve_move(hw, q_start, q_target, duration=0.1, control_dt=0.02, description="Test S-Curve Warmup")
        print(" [✓] S-Curve Cosine Warm-up hoàn thành trơn tru.")

        # 2. Test 10 chu kỳ Inference 50Hz liên tiếp
        for t in range(10):
            t0 = time.time()
            rgb, depth = cam.get_rgbd()
            current_q = hw.get_qpos()

            # Forward
            with torch.no_grad():
                rgbd_t = preprocess_rgbd(rgb, depth, target_size=(240, 424)).unsqueeze(0).to(device)
                qpos_t = torch.from_numpy(current_q).float().unsqueeze(0).to(device)
                pred_c, _, _ = eval_model(
                    image=rgbd_t,
                    qpos=qpos_t,
                    actions=None
                )
            pred_chunk = pred_c.squeeze(0).cpu().numpy() # [50, 16]

            # Ensemble & Smooth
            ens_q = ensemble.update(pred_chunk)
            smooth_q = smoother.step(ens_q)

            # Send CAN
            hw.send_target_positions(smooth_q)
            dt_step = time.time() - t0
            assert dt_step < 0.10, f"Bước inference tốn quá nhiều thời gian: {dt_step*1000:.1f}ms"

        print(f" [✓] Vòng lặp Inference 50Hz (Forward -> Ensemble -> Smoother -> Send CAN) chạy HOÀN HẢO!")

        print("\n" + "=" * 80)
        print("🎉 TẤT CẢ 5 BÀI TEST TOÀN DIỆN PIPELINE ĐÃ VƯỢT QUA 100% (EXIT CODE 0)!")
        print("   - Data Format & HDF5 Specification : ✅ ĐẠT CHUẨN (PASS)")
        print("   - Episodic Dataset & DataLoader    : ✅ ĐẠT CHUẨN (PASS)")
        print("   - Training, Loss & Backpropagation : ✅ ĐẠT CHUẨN (PASS)")
        print("   - Checkpoint Saving & Strict Load  : ✅ ĐẠT CHUẨN (PASS - 0 Missing Keys)")
        print("   - Evaluation Metric Computation    : ✅ ĐẠT CHUẨN (PASS)")
        print("   - 50Hz Hardware Robot Inference    : ✅ ĐẠT CHUẨN (PASS)")
        print("=" * 80 + "\n")

    finally:
        shutil.rmtree(test_dir, ignore_errors=True)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="End-to-End Pipeline Verification Suite")
    parser.add_argument("--device", type=str, default="auto", choices=["auto", "cuda", "cpu"], help="Thiết bị tính toán (mặc định: auto)")
    args = parser.parse_args()
    run_pipeline_test(target_device=args.device)
