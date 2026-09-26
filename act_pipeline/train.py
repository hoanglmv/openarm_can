#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Production Training Pipeline for ACT (Action Chunking with Transformers)
Bimanual OpenArm (16-DOF: 2 arms x 8 motors) + 01 Chest RGB-D Camera for Autonomous Cooking & Stir-Frying

Tính năng chính:
- Tự động dò và tính toán Thống kê chuẩn hóa (Mean & Std) cho 16 khớp
- Phân chia Train/Validation tập demo chống Data Leakage
- Tự động đóng băng ResNet-18 Layers 1..4, tối ưu Conv1 4-Channel Adapter
- Hỗ trợ Mixed Precision FP16 (torch.cuda.amp) tăng tốc trên GPU NVIDIA
- Gradient Clipping chống hiện tượng bùng nổ Gradient trong Transformer
- Quản lý Checkpoint toàn diện (Best, Latest, Periodic, Deployment weights)
- Cơ chế khôi phục huấn luyện liền mạch (--resume)
- Hệ thống Logging đa kênh: Console, File train.log, metrics.jsonl, TensorBoard, WandB
"""

import os
import sys
import time
import math
import random
import argparse
import glob
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

from act_pipeline.config import ModelConfig, TrainConfig
from act_pipeline.models.act_model import ACTPolicy
from act_pipeline.data.dataset import BimanualEpisodicDataset
from act_pipeline.data.normalization import compute_norm_stats, save_norm_stats, load_norm_stats
from act_pipeline.utils.logger import MetricLogger
from act_pipeline.utils.checkpoint import CheckpointManager


def set_seed(seed: int = 42):
    """Cố định seed ngẫu nhiên để đảm bảo tính tái lập kết quả (Reproducibility)"""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False


def evaluate(
    model: nn.Module,
    dataloader: DataLoader,
    device: torch.device,
    kl_weight: float = 10.0,
    loss_type: str = "l1",
) -> Dict[str, float]:
    """
    Vòng lặp đánh giá trên tập Validation:
    Tính toán Total Loss, Reconstruction Loss, KL Loss và sai số trung bình từng khớp (Per-Joint MAE).
    """
    model.eval()
    total_loss = 0.0
    total_recon = 0.0
    total_kl = 0.0
    num_batches = len(dataloader)

    # Theo dõi sai số MAE cho từng khớp trong số 16 khớp
    per_joint_errors = torch.zeros(16, device=device)
    total_valid_steps = 0

    with torch.no_grad():
        for batch in dataloader:
            image = batch["image"].to(device, non_blocking=True)
            qpos = batch["qpos"].to(device, non_blocking=True)
            actions = batch["actions"].to(device, non_blocking=True)
            is_pad = batch["is_pad"].to(device, non_blocking=True)

            pred_actions, mu, logvar = model(
                image=image, qpos=qpos, actions=actions, is_pad=is_pad
            )

            loss_dict = model.compute_loss(
                pred_actions=pred_actions,
                actions=actions,
                is_pad=is_pad,
                mu=mu,
                logvar=logvar,
                kl_weight=kl_weight,
                loss_type=loss_type,
            )

            total_loss += loss_dict["loss"].item()
            total_recon += loss_dict["recon_loss"].item()
            total_kl += loss_dict["kl_loss"].item()

            # Tính MAE per-joint loại trừ các bước bị padding
            valid_mask = (~is_pad).unsqueeze(-1).float() # [B, chunk_size, 1]
            abs_diff = torch.abs(pred_actions - actions) * valid_mask # [B, chunk_size, 16]
            per_joint_errors += abs_diff.sum(dim=(0, 1))
            total_valid_steps += valid_mask.sum().item()

    avg_loss = total_loss / max(num_batches, 1)
    avg_recon = total_recon / max(num_batches, 1)
    avg_kl = total_kl / max(num_batches, 1)
    avg_joint_mae = (per_joint_errors / max(total_valid_steps, 1)).cpu().tolist()

    return {
        "loss": avg_loss,
        "recon_loss": avg_recon,
        "l1": avg_recon,
        "kl": avg_kl,
        "per_joint_mae": avg_joint_mae,
        "max_joint_mae": max(avg_joint_mae) if len(avg_joint_mae) > 0 else 0.0,
    }


def train_pipeline(args):
    """
    Pipeline huấn luyện hoàn chỉnh cho ACT Policy
    """
    # 1. Cấu hình thiết bị và seed
    set_seed(args.seed)
    device = torch.device(args.device if torch.cuda.is_available() and args.device == "cuda" else "cpu")
    os.makedirs(args.output_dir, exist_ok=True)

    # 2. Khởi tạo Metric Logger & Checkpoint Manager
    logger = MetricLogger(
        log_dir=args.output_dir,
        exp_name="act_openarm_bimanual",
        use_tensorboard=args.use_tensorboard,
        use_wandb=args.use_wandb,
        wandb_project=args.wandb_project,
        wandb_entity=args.wandb_entity,
        config_dict=vars(args),
    )
    checkpoint_mgr = CheckpointManager(checkpoint_dir=args.output_dir)

    logger.info("=" * 80)
    logger.info("   🚀 KHỞI ĐỘNG PIPELINE HUẤN LUYỆN ACT - BIMANUAL OPENARM (16-DOF) RGB-D")
    logger.info("=" * 80)
    logger.info(f"[*] Thiết bị thực thi   : {device} ({torch.cuda.get_device_name(0) if device.type == 'cuda' else 'CPU'})")
    logger.info(f"[*] Thư mục Dataset     : {args.dataset_dir}")
    logger.info(f"[*] Thư mục Checkpoints : {args.output_dir}")
    logger.info(f"[*] Tổng số Epochs      : {args.epochs}")
    logger.info(f"[*] Kích thước Batch    : {args.batch_size}")
    logger.info(f"[*] Tốc độ học (LR)     : {args.lr} (Backbone: {args.lr_backbone})")
    logger.info(f"[*] Mixed Precision     : {'BẬT (FP16 AMP)' if args.use_amp and device.type == 'cuda' else 'TẮT'}")
    logger.info("=" * 80)

    # 3. Quét danh sách file HDF5 và chia Train/Validation
    all_files = sorted(glob.glob(os.path.join(args.dataset_dir, "*.hdf5")))
    if len(all_files) == 0:
        logger.error(f"[X] LỖI: Không tìm thấy file .hdf5 nào trong: {args.dataset_dir}")
        sys.exit(1)

    # Xáo trộn file ngẫu nhiên để chia train/val
    random.shuffle(all_files)
    num_val = max(1, int(len(all_files) * args.val_split))
    val_files = sorted(all_files[:num_val])
    train_files = sorted(all_files[num_val:])

    logger.info(f"[*] Tổng số Episodes tìm thấy: {len(all_files)}")
    logger.info(f"    - Tập Huấn luyện (Train) : {len(train_files)} episodes")
    logger.info(f"    - Tập Đánh giá (Val)     : {len(val_files)} episodes")

    # 4. Tính toán Thống kê chuẩn hóa (chỉ tính trên train_files để tránh rò rỉ dữ liệu)
    stats_path = os.path.join(args.output_dir, "dataset_stats.pkl")
    if args.resume and os.path.exists(stats_path):
        stats = load_norm_stats(stats_path)
        logger.info(f"[✓] Đã khôi phục thống kê chuẩn hóa từ: {stats_path}")
    else:
        stats = compute_norm_stats(train_files)
        save_norm_stats(stats, stats_path)
        logger.info(f"[✓] Đã lưu thống kê chuẩn hóa tại: {stats_path}")

    # 5. Khởi tạo Dataset & DataLoader
    train_dataset = BimanualEpisodicDataset(
        file_paths=train_files,
        stats=stats,
        chunk_size=args.chunk_size,
        step_stride=args.step_stride,
        target_img_size=(args.img_height, args.img_width),
        depth_min_m=args.depth_min_m,
        depth_max_m=args.depth_max_m,
    )
    val_dataset = BimanualEpisodicDataset(
        file_paths=val_files,
        stats=stats,
        chunk_size=args.chunk_size,
        step_stride=args.step_stride * 2, # Val có thể lấy thưa hơn để tăng tốc
        target_img_size=(args.img_height, args.img_width),
        depth_min_m=args.depth_min_m,
        depth_max_m=args.depth_max_m,
    )

    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=True if device.type == "cuda" else False,
        drop_last=True if len(train_dataset) > args.batch_size else False,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=True if device.type == "cuda" else False,
        drop_last=False,
    )

    # 6. Khởi tạo Mô hình ACT
    model_cfg = ModelConfig(
        in_channels=4,
        backbone_type="resnet18",
        freeze_backbone=not args.unfreeze_backbone,
        img_height=args.img_height,
        img_width=args.img_width,
        d_model=args.d_model,
        nheads=args.nheads,
        dim_feedforward=args.dim_feedforward,
        dropout=args.dropout,
        cvae_layers=args.cvae_layers,
        latent_dim=args.latent_dim,
        decoder_layers=args.decoder_layers,
        chunk_size=args.chunk_size,
        action_dim=args.action_dim,
        qpos_dim=args.qpos_dim,
        depth_min_m=args.depth_min_m,
        depth_max_m=args.depth_max_m,
    )
    model = ACTPolicy(config=model_cfg).to(device)

    # Hiển thị cấu trúc tham số theo bảng ACT_ARCHITECTURE.md
    param_summary = model.get_parameter_summary()
    logger.info("-" * 80)
    logger.info(f"📊 BẢNG PHÂN BỔ THAM SỐ MÔ HÌNH (Trainable vs Frozen):")
    logger.info(f"   + Tổng số tham số (Total)    : {param_summary['total_params']:,}")
    logger.info(f"   + 🔥 Tham số học được (Train): {param_summary['trainable_params']:,} ({param_summary['trainable_pct']:.2f}%)")
    logger.info(f"   + ❄️ Tham số đóng băng (Freeze): {param_summary['frozen_params']:,}")
    logger.info(f"   + Conv1 4-Channel Adapter    : {param_summary['modules']['conv1_adapter']:,}")
    logger.info(f"   + ResNet-18 Layers 1..4      : {param_summary['modules']['resnet_frozen']:,} (Frozen)")
    logger.info(f"   + CVAE Transformer Encoder   : {param_summary['modules']['cvae']:,}")
    logger.info(f"   + Transformer Policy Decoder : {param_summary['modules']['policy_decoder']:,}")
    logger.info("-" * 80)

    # 7. Optimizer với Param Groups (Tách Learning Rate cho Backbone Conv1 Adapter và Transformer)
    param_groups = [
        {
            "params": [p for n, p in model.named_parameters() if "conv1_adapter" in n and p.requires_grad],
            "lr": args.lr_backbone,
            "weight_decay": args.weight_decay,
        },
        {
            "params": [p for n, p in model.named_parameters() if "conv1_adapter" not in n and p.requires_grad],
            "lr": args.lr,
            "weight_decay": args.weight_decay,
        },
    ]
    optimizer = torch.optim.AdamW(param_groups)

    # 8. Learning Rate Scheduler (Cosine Annealing with Warmup theo Epoch)
    warmup_epochs = args.warmup_epochs
    total_epochs = args.epochs
    
    def lr_lambda(current_epoch: int):
        if current_epoch < warmup_epochs:
            return float(current_epoch + 1) / float(max(1, warmup_epochs))
        progress = float(current_epoch - warmup_epochs) / float(max(1, total_epochs - warmup_epochs))
        return max(args.min_lr / args.lr, 0.5 * (1.0 + math.cos(math.pi * progress)))

    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda=lr_lambda)

    # 9. Mixed Precision GradScaler
    scaler = torch.amp.GradScaler("cuda", enabled=(args.use_amp and device.type == "cuda"))

    # 10. Khôi phục nếu có cờ --resume
    start_epoch = 1
    global_step = 0
    best_val_loss = float("inf")

    if args.resume:
        resume_file = args.resume
        if resume_file == "auto":
            resume_file = checkpoint_mgr.find_latest_checkpoint()

        if resume_file and os.path.exists(resume_file):
            state = checkpoint_mgr.load_checkpoint(
                checkpoint_path=resume_file,
                model=model,
                optimizer=optimizer,
                scheduler=scheduler,
                scaler=scaler,
                device=device,
            )
            start_epoch = state["epoch"] + 1
            global_step = state["global_step"]
            best_val_loss = state.get("metrics", {}).get("best_val_loss", float("inf"))
            logger.info(f"[✓] Đã phục hồi thành công. Tiếp tục huấn luyện từ Epoch {start_epoch}...")
        else:
            logger.warning(f"[!] Không tìm thấy checkpoint resume tại '{args.resume}'. Bắt đầu huấn luyện mới.")

    # 11. VÒNG LẶP HUẤN LUYỆN CHÍNH THỨC
    logger.info(f"\n[*] BẮT ĐẦU VÒNG LẶP HUẤN LUYỆN (EPOCHS: {start_epoch} -> {args.epochs})...")
    start_train_time = time.time()

    for epoch in range(start_epoch, args.epochs + 1):
        epoch_start_time = time.time()
        model.train()
        
        train_loss_accum = 0.0
        train_recon_accum = 0.0
        train_kl_accum = 0.0
        batch_count = 0

        for step_idx, batch in enumerate(train_loader):
            global_step += 1
            image = batch["image"].to(device, non_blocking=True)
            qpos = batch["qpos"].to(device, non_blocking=True)
            actions = batch["actions"].to(device, non_blocking=True)
            is_pad = batch["is_pad"].to(device, non_blocking=True)

            optimizer.zero_grad(set_to_none=True)

            # Chạy Mixed Precision (FP16)
            with torch.amp.autocast("cuda", enabled=(args.use_amp and device.type == "cuda")):
                pred_actions, mu, logvar = model(
                    image=image, qpos=qpos, actions=actions, is_pad=is_pad
                )
                loss_dict = model.compute_loss(
                    pred_actions=pred_actions,
                    actions=actions,
                    is_pad=is_pad,
                    mu=mu,
                    logvar=logvar,
                    kl_weight=args.kl_weight,
                    loss_type=args.loss_type,
                )
                loss = loss_dict["loss"]

            # Lan truyền ngược và cập nhật trọng số
            scaler.scale(loss).backward()
            
            # Gradient Clipping chống bùng nổ gradient trong Transformer
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=args.grad_clip)

            scaler.step(optimizer)
            scaler.update()

            train_loss_accum += loss.item()
            train_recon_accum += loss_dict["recon_loss"].item()
            train_kl_accum += loss_dict["kl_loss"].item()
            batch_count += 1

            # Ghi nhận step metrics định kỳ
            if global_step % args.log_every_steps == 0:
                current_lr = scheduler.get_last_lr()[-1]
                logger.log_metrics(
                    {
                        "step_loss": loss.item(),
                        "step_recon": loss_dict["recon_loss"].item(),
                        "step_kl": loss_dict["kl_loss"].item(),
                        "lr": current_lr,
                    },
                    step=global_step,
                    prefix="train/",
                )

        # Cập nhật Learning Rate Scheduler theo Epoch
        scheduler.step()

        # Tính trung bình epoch train
        avg_train_loss = train_loss_accum / max(batch_count, 1)
        avg_train_recon = train_recon_accum / max(batch_count, 1)
        avg_train_kl = train_kl_accum / max(batch_count, 1)
        current_lr = scheduler.get_last_lr()[-1]

        logger.log_metrics(
            {
                "loss": avg_train_loss,
                "l1": avg_train_recon,
                "kl": avg_train_kl,
                "lr": current_lr,
            },
            step=epoch,
            prefix="train_epoch/",
        )

        # 12. Đánh giá Validation định kỳ
        val_metrics = None
        is_best = False
        if epoch % args.eval_every == 0 or epoch == 1 or epoch == args.epochs:
            val_metrics = evaluate(
                model=model,
                dataloader=val_loader,
                device=device,
                kl_weight=args.kl_weight,
                loss_type=args.loss_type,
            )
            logger.log_metrics(
                {
                    "loss": val_metrics["loss"],
                    "l1": val_metrics["l1"],
                    "kl": val_metrics["kl"],
                    "max_joint_mae": val_metrics["max_joint_mae"],
                },
                step=epoch,
                prefix="val_epoch/",
            )

            # Kiểm tra xem có đạt mức loss tốt nhất không
            if val_metrics["loss"] < best_val_loss:
                best_val_loss = val_metrics["loss"]
                is_best = True

        # In log tóm tắt Epoch
        epoch_elapsed = time.time() - epoch_start_time
        logger.log_epoch_summary(
            epoch=epoch,
            total_epochs=args.epochs,
            train_metrics={"loss": avg_train_loss, "l1": avg_train_recon, "kl": avg_train_kl},
            val_metrics=val_metrics,
            lr=current_lr,
            elapsed_sec=epoch_elapsed,
        )

        # 13. Lưu trữ Checkpoint
        save_periodic = (epoch % args.save_every == 0)
        checkpoint_metrics = {
            "train_loss": avg_train_loss,
            "train_l1": avg_train_recon,
            "val_loss": val_metrics["loss"] if val_metrics else avg_train_loss,
            "val_l1": val_metrics["l1"] if val_metrics else avg_train_recon,
            "best_val_loss": best_val_loss,
        }

        saved_files = checkpoint_mgr.save_checkpoint(
            epoch=epoch,
            global_step=global_step,
            model=model,
            optimizer=optimizer,
            scheduler=scheduler,
            scaler=scaler,
            stats=stats,
            config=vars(args),
            metrics=checkpoint_metrics,
            is_best=is_best,
            save_periodic=save_periodic,
        )

        if is_best:
            logger.info(f"   🏆 ĐẠT KỶ LỤC MỚI! Best Val Loss: {best_val_loss:.4f} -> Đã lưu: {saved_files.get('best')}")

    # Hoàn tất huấn luyện
    total_duration_min = (time.time() - start_train_time) / 60.0
    logger.info("=" * 80)
    logger.info(f"   🎉 HUẤN LUYỆN HOÀN TẤT THÀNH CÔNG TRONG {total_duration_min:.2f} PHÚT!")
    logger.info(f"   + Best Checkpoint     : {checkpoint_mgr.best_checkpoint_path}")
    logger.info(f"   + Latest Checkpoint   : {checkpoint_mgr.latest_checkpoint_path}")
    logger.info(f"   + Deployment Weights  : {checkpoint_mgr.deploy_weights_path}")
    logger.info("=" * 80)
    logger.close()


def parse_args():
    parser = argparse.ArgumentParser(description="Huấn luyện mô hình ACT Bimanual OpenArm RGB-D")
    
    # Dữ liệu & Đường dẫn
    parser.add_argument("--dataset_dir", type=str, default="dataset/real_cooking_stir_fry", help="Đường dẫn thư mục chứa các file .hdf5")
    parser.add_argument("--output_dir", type=str, default="checkpoints/act_openarm", help="Thư mục lưu trữ checkpoints và log files")
    parser.add_argument("--resume", type=str, default=None, help="Đường dẫn checkpoint để khôi phục hoặc 'auto' để tìm checkpoint mới nhất")
    
    # Vòng lặp huấn luyện
    parser.add_argument("--epochs", type=int, default=500, help="Tổng số epochs huấn luyện")
    parser.add_argument("--batch_size", type=int, default=16, help="Kích thước batch (GPU 16GB tối ưu ở 16)")
    parser.add_argument("--lr", type=float, default=1e-4, help="Learning rate cho Transformer Policy và CVAE")
    parser.add_argument("--lr_backbone", type=float, default=1e-5, help="Learning rate nhỏ cho Conv1 Adapter")
    parser.add_argument("--weight_decay", type=float, default=1e-4, help="Hệ số Weight Decay cho AdamW")
    parser.add_argument("--warmup_epochs", type=int, default=10, help="Số epochs khởi động tuyến tính Warmup")
    parser.add_argument("--min_lr", type=float, default=1e-6, help="Learning rate tối thiểu của Cosine Annealing")
    parser.add_argument("--grad_clip", type=float, default=10.0, help="Ngưỡng Gradient Clipping")

    # Kiến trúc ACT
    parser.add_argument("--d_model", type=int, default=512, help="Kích thước ẩn Transformer d_model")
    parser.add_argument("--nheads", type=int, default=8, help="Số heads của Multi-Head Attention")
    parser.add_argument("--dim_feedforward", type=int, default=2048, help="Kích thước lớp ẩn FFN")
    parser.add_argument("--dropout", type=float, default=0.1, help="Tỷ lệ Dropout")
    parser.add_argument("--chunk_size", type=int, default=50, help="Số bước hành động tương lai (k=50 bước = 1 giây)")
    parser.add_argument("--action_dim", type=int, default=16, help="Số bậc tự do 2 tay robot (16 động cơ)")
    parser.add_argument("--qpos_dim", type=int, default=16, help="Số chiều góc khớp đầu vào")
    parser.add_argument("--latent_dim", type=int, default=32, help="Số chiều vector ẩn z của CVAE")
    parser.add_argument("--cvae_layers", type=int, default=2, help="Số layers Transformer Encoder của CVAE")
    parser.add_argument("--decoder_layers", type=int, default=4, help="Số layers Transformer Decoder của Policy")
    parser.add_argument("--unfreeze_backbone", action="store_true", help="Nếu đặt cờ này, sẽ fine-tune toàn bộ ResNet18 thay vì đóng băng")

    # Hàm mất mát Loss
    parser.add_argument("--kl_weight", type=float, default=10.0, help="Trọng số beta cho KL Divergence Loss")
    parser.add_argument("--loss_type", type=str, default="l1", choices=["l1", "l2"], help="Hàm mất mát phục dựng (l1 hoặc l2)")

    # Tiền xử lý RGB-D
    parser.add_argument("--img_height", type=int, default=480, help="Chiều cao ảnh RGB-D")
    parser.add_argument("--img_width", type=int, default=640, help="Chiều rộng ảnh RGB-D")
    parser.add_argument("--depth_min_m", type=float, default=0.2, help="Khoảng cách bàn tối thiểu (m)")
    parser.add_argument("--depth_max_m", type=float, default=1.2, help="Khoảng cách bàn tối đa (m)")
    parser.add_argument("--step_stride", type=int, default=1, help="Bước nhảy lấy mẫu start_t trong episode")

    # Đánh giá & Lưu trữ
    parser.add_argument("--val_split", type=float, default=0.15, help="Tỷ lệ tập validation (ví dụ 0.15 = 15%)")
    parser.add_argument("--eval_every", type=int, default=10, help="Đánh giá validation mỗi N epochs")
    parser.add_argument("--save_every", type=int, default=50, help="Lưu checkpoint định kỳ mỗi N epochs")
    parser.add_argument("--log_every_steps", type=int, default=10, help="Ghi nhận metric mỗi N bước batch")

    # Phần cứng & Môi trường
    parser.add_argument("--device", type=str, default="cuda", help="Thiết bị thực thi (cuda hoặc cpu)")
    parser.add_argument("--num_workers", type=int, default=2, help="Số workers nạp dữ liệu DataLoader")
    parser.add_argument("--no_amp", action="store_true", help="Tắt Automatic Mixed Precision")
    parser.add_argument("--seed", type=int, default=42, help="Seed ngẫu nhiên")

    # Logging nâng cao
    parser.add_argument("--no_tensorboard", action="store_true", help="Tắt ghi TensorBoard")
    parser.add_argument("--use_wandb", action="store_true", help="Bật theo dõi với Weights & Biases")
    parser.add_argument("--wandb_project", type=str, default="act-openarm", help="Tên project trên WandB")
    parser.add_argument("--wandb_entity", type=str, default=None, help="Tên entity trên WandB")

    args = parser.parse_args()
    args.use_amp = not args.no_amp
    args.use_tensorboard = not args.no_tensorboard
    return args


if __name__ == "__main__":
    args = parse_args()
    train_pipeline(args)
