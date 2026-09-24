#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Production Training Script for ACT (Action Chunking with Transformers)
Dành cho huấn luyện trên GPU cục bộ hoặc Google Colab (Tesla T4 / V100 / A100 / RTX 30xx/40xx)
Phần cứng mục tiêu: OpenArm (8-DOF: 7 khớp tay + 1 kẹp Gripper), 01 Camera trước ngực
"""

import os
import sys

# Đảm bảo mã hóa UTF-8 an toàn trên mọi console Windows và Linux/Colab
if sys.stdout.encoding != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import glob
import math
import time
import argparse
import pickle
import h5py
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
import torchvision.models as models

# Tự động chọn thiết bị
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


# ==============================================================================
# 1. TÍNH TOÁN CHUẨN HÓA DỮ LIỆU (NORMALIZATION STATS)
# ==============================================================================
def get_norm_stats(dataset_dir):
    """
    Tính Mean và Std của qpos và action trên toàn bộ dataset để chuẩn hóa Z-score.
    Điều này giúp mô hình hội tụ nhanh hơn gấp 3 lần và tránh lỗi số học.
    """
    files = sorted(glob.glob(os.path.join(dataset_dir, "*.hdf5")))
    if len(files) == 0:
        raise FileNotFoundError(f"Không tìm thấy file .hdf5 nào trong: {dataset_dir}")

    all_qpos = []
    all_actions = []

    print(f"[*] Đang quét {len(files)} file HDF5 để tính toán thống kê (Mean & Std)...")
    for fpath in files:
        with h5py.File(fpath, "r") as f:
            all_qpos.append(f["observations/qpos"][:])
            all_actions.append(f["action"][:])

    all_qpos = np.concatenate(all_qpos, axis=0)
    all_actions = np.concatenate(all_actions, axis=0)

    stats = {
        "qpos_mean": np.mean(all_qpos, axis=0).astype(np.float32),
        "qpos_std": np.clip(np.std(all_qpos, axis=0), 1e-4, np.inf).astype(np.float32),
        "action_mean": np.mean(all_actions, axis=0).astype(np.float32),
        "action_std": np.clip(np.std(all_actions, axis=0), 1e-4, np.inf).astype(np.float32),
    }
    print("[✓] Đã tính xong thống kê chuẩn hóa dữ liệu.")
    return stats


# ==============================================================================
# 2. EPISODIC DATASET
# ==============================================================================
class EpisodicDataset(Dataset):
    def __init__(self, dataset_dir, stats, chunk_size=50, step_stride=3):
        self.chunk_size = chunk_size
        self.files = sorted(glob.glob(os.path.join(dataset_dir, "*.hdf5")))
        self.stats = stats
        self.samples = []

        for file_idx, fpath in enumerate(self.files):
            with h5py.File(fpath, "r") as f:
                T = len(f["action"])
                for start_t in range(0, T - chunk_size, step_stride):
                    self.samples.append((file_idx, start_t))

        print(f"[✓] Tổng số mẫu Action Chunk ({chunk_size} bước): {len(self.samples)}")

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        file_idx, start_t = self.samples[idx]
        with h5py.File(self.files[file_idx], "r") as f:
            # 1. Ảnh camera ngực [480, 640, 3] -> Resize [240, 320] để tăng tốc 3x
            raw_img = f["observations/images/chest"][start_t]
            img = torch.from_numpy(raw_img).permute(2, 0, 1).float() / 255.0
            img = F.interpolate(img.unsqueeze(0), size=(240, 320), mode="bilinear").squeeze(0)

            # Chuẩn hóa ImageNet
            mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
            std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
            img = (img - mean) / std

            # 2. qpos hiện tại (Chuẩn hóa Z-score)
            raw_qpos = f["observations/qpos"][start_t]
            norm_qpos = (raw_qpos - self.stats["qpos_mean"]) / self.stats["qpos_std"]
            qpos = torch.tensor(norm_qpos, dtype=torch.float32)

            # 3. Actions chunk (Chuẩn hóa Z-score)
            raw_actions = f["action"][start_t : start_t + self.chunk_size]
            norm_actions = (raw_actions - self.stats["action_mean"]) / self.stats["action_std"]
            actions = torch.tensor(norm_actions, dtype=torch.float32)

        return img, qpos, actions


# ==============================================================================
# 3. KIẾN TRÚC MÔ HÌNH ACT HOÀN CHỈNH
# ==============================================================================
class SinusoidalPositionEmbedding2D(nn.Module):
    """Mã hóa vị trí không gian 2D cho đặc trưng ảnh"""
    def __init__(self, d_model=256):
        super().__init__()
        self.d_model = d_model

    def forward(self, x):
        B, C, H, W = x.shape
        num_pos_feats = self.d_model // 2
        y_embed = torch.arange(1, H + 1, dtype=torch.float32, device=x.device).unsqueeze(1).repeat(1, W)
        x_embed = torch.arange(1, W + 1, dtype=torch.float32, device=x.device).unsqueeze(0).repeat(H, 1)

        dim_t = torch.arange(num_pos_feats, dtype=torch.float32, device=x.device)
        dim_t = 10000 ** (2 * (dim_t // 2) / num_pos_feats)

        pos_x = x_embed[:, :, None] / dim_t
        pos_y = y_embed[:, :, None] / dim_t
        pos_x = torch.stack((pos_x[:, :, 0::2].sin(), pos_x[:, :, 1::2].cos()), dim=3).flatten(2)
        pos_y = torch.stack((pos_y[:, :, 0::2].sin(), pos_y[:, :, 1::2].cos()), dim=3).flatten(2)
        pos = torch.cat((pos_y, pos_x), dim=2).permute(2, 0, 1).unsqueeze(0).repeat(B, 1, 1, 1)
        return pos


class ACTPolicy(nn.Module):
    def __init__(self, d_model=256, chunk_size=50, action_dim=8, latent_dim=16):
        super().__init__()
        self.chunk_size = chunk_size
        self.action_dim = action_dim
        self.latent_dim = latent_dim
        self.d_model = d_model

        # 1. Visual Backbone (ResNet-18)
        resnet = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
        self.backbone = nn.Sequential(*list(resnet.children())[:-2])
        self.pos_embed_2d = SinusoidalPositionEmbedding2D(d_model)
        self.img_proj = nn.Conv2d(512, d_model, kernel_size=1)

        # 2. Linear Projections
        self.qpos_proj = nn.Linear(action_dim, d_model)
        self.action_proj = nn.Linear(action_dim, d_model)

        # 3. CVAE Encoder
        cvae_layer = nn.TransformerEncoderLayer(d_model=d_model, nhead=4, dim_feedforward=512, batch_first=True)
        self.cvae_encoder = nn.TransformerEncoder(cvae_layer, num_layers=2)
        self.cls_token = nn.Parameter(torch.randn(1, 1, d_model))
        self.latent_mu = nn.Linear(d_model, latent_dim)
        self.latent_logvar = nn.Linear(d_model, latent_dim)
        self.latent_proj = nn.Linear(latent_dim, d_model)

        # 4. Transformer Decoder Policy
        decoder_layer = nn.TransformerDecoderLayer(d_model=d_model, nhead=4, dim_feedforward=512, batch_first=True)
        self.policy_decoder = nn.TransformerDecoder(decoder_layer, num_layers=4)
        self.action_queries = nn.Parameter(torch.randn(1, chunk_size, d_model))
        self.action_head = nn.Linear(d_model, action_dim)

    def forward(self, img, qpos, actions=None):
        B = img.shape[0]

        # 1. Trích xuất ảnh
        feat = self.backbone(img)
        pos = self.pos_embed_2d(feat)
        feat = self.img_proj(feat) + pos
        visual_tokens = feat.flatten(2).permute(0, 2, 1)

        # 2. Token góc khớp
        qpos_token = self.qpos_proj(qpos).unsqueeze(1)

        # 3. Xử lý CVAE
        if actions is not None:
            # Khi Training: Encode actions thành z
            action_tokens = self.action_proj(actions)
            cls_tokens = self.cls_token.repeat(B, 1, 1)
            enc_input = torch.cat([cls_tokens, qpos_token, action_tokens], dim=1)
            enc_out = self.cvae_encoder(enc_input)
            cls_out = enc_out[:, 0]

            mu = self.latent_mu(cls_out)
            logvar = self.latent_logvar(cls_out)
            std = torch.exp(0.5 * logvar)
            eps = torch.randn_like(std)
            z = mu + eps * std
        else:
            # Khi Inference: Gán cứng z = 0
            z = torch.zeros(B, self.latent_dim, device=img.device)
            mu, logvar = None, None

        z_token = self.latent_proj(z).unsqueeze(1)

        # 4. Giải mã Action Chunk
        memory = torch.cat([visual_tokens, qpos_token, z_token], dim=1)
        tgt_queries = self.action_queries.repeat(B, 1, 1)
        decoded = self.policy_decoder(tgt=tgt_queries, memory=memory)
        pred_actions = self.action_head(decoded)

        return pred_actions, mu, logvar


# ==============================================================================
# 4. HUẤN LUYỆN CHÍNH THỨC (TRAINING LOOP WITH AMP)
# ==============================================================================
def train_act(args):
    print("=" * 70)
    print(f"[*] KHỞI ĐỘNG HUẤN LUYỆN ACT CHO ROBOT OPENARM")
    print(f"[*] Thiết bị GPU       : {DEVICE} ({torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU'})")
    print(f"[*] Thư mục Dataset    : {args.data_dir}")
    print(f"[*] Tổng số Epochs     : {args.epochs}")
    print(f"[*] Batch Size         : {args.batch_size}")
    print(f"[*] Learning Rate      : {args.lr}")
    print("=" * 70)

    # 1. Tính toán thống kê dữ liệu
    stats = get_norm_stats(args.data_dir)

    # 2. Khởi tạo Dataset & DataLoader
    dataset = EpisodicDataset(args.data_dir, stats, chunk_size=args.chunk_size)
    dataloader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True, 
                            num_workers=args.num_workers, pin_memory=True if torch.cuda.is_available() else False)

    # 3. Khởi tạo mô hình & Optimizer
    model = ACTPolicy(d_model=256, chunk_size=args.chunk_size, action_dim=8, latent_dim=16).to(DEVICE)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs, eta_min=1e-6)
    scaler = torch.cuda.amp.GradScaler() # Hỗ trợ Mixed Precision FP16 cho T4/V100/A100

    best_loss = float("inf")
    start_time = time.time()

    print(f"\n[*] BẮT ĐẦU VÒNG LẶP HUẤN LUYỆN...")

    for epoch in range(1, args.epochs + 1):
        model.train()
        total_loss, total_l1, total_kl = 0.0, 0.0, 0.0

        for img, qpos, actions in dataloader:
            img = img.to(DEVICE, non_blocking=True)
            qpos = qpos.to(DEVICE, non_blocking=True)
            actions = actions.to(DEVICE, non_blocking=True)

            optimizer.zero_grad()

            # Mixed precision FP16 tăng tốc độ 2x trên Tesla T4
            with torch.cuda.amp.autocast():
                pred_actions, mu, logvar = model(img, qpos, actions)
                l1_loss = F.l1_loss(pred_actions, actions)
                kl_loss = -0.5 * torch.sum(1 + logvar - mu.pow(2) - logvar.exp()) / img.shape[0]
                loss = l1_loss + args.kl_weight * kl_loss

            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()

            total_loss += loss.item()
            total_l1 += l1_loss.item()
            total_kl += kl_loss.item()

        scheduler.step()
        n_batches = len(dataloader)
        avg_loss = total_loss / n_batches
        avg_l1 = total_l1 / n_batches
        avg_kl = total_kl / n_batches

        # In log định kỳ mỗi 10 epochs hoặc epoch đầu/cuối
        if epoch % 10 == 0 or epoch == 1 or epoch == args.epochs:
            elapsed = time.time() - start_time
            print(f"Epoch [{epoch:4d}/{args.epochs}] | Loss: {avg_loss:.4f} (L1: {avg_l1:.4f}, KL: {avg_kl:.4f}) | LR: {scheduler.get_last_lr()[0]:.2e} | Time: {elapsed:.1f}s")

        # Lưu checkpoint tốt nhất
        if avg_loss < best_loss and epoch >= 50:
            best_loss = avg_loss
            checkpoint = {
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "stats": stats,
                "loss": best_loss,
            }
            torch.save(checkpoint, args.save_path)

    # Lưu checkpoint hoàn tất cuối cùng
    final_checkpoint = {
        "epoch": args.epochs,
        "model_state_dict": model.state_dict(),
        "stats": stats,
        "loss": avg_loss,
    }
    torch.save(final_checkpoint, args.save_path)
    total_time = (time.time() - start_time) / 60.0
    print("=" * 70)
    print(f"[✓] HUẤN LUYỆN HOÀN TẤT TRONG: {total_time:.2f} PHÚT!")
    print(f"[✓] CHECKPOINT VÀ STATS ĐÃ ĐƯỢC LƯU TẠI: {args.save_path}")
    print("=" * 70)


# ==============================================================================
# 5. CLI ARGUMENTS
# ==============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Huấn luyện mô hình ACT cho OpenArm")
    parser.add_argument("--data_dir", type=str, default="dataset/mock_episodes", help="Đường dẫn thư mục chứa các file .hdf5")
    parser.add_argument("--epochs", type=int, default=500, help="Số epochs huấn luyện (Khuyến nghị: 500 - 1000)")
    parser.add_argument("--batch_size", type=int, default=16, help="Kích thước batch (T4 16GB tối ưu ở 16)")
    parser.add_argument("--lr", type=float, default=1e-4, help="Learning rate (Tối ưu 1e-4)")
    parser.add_argument("--chunk_size", type=int, default=50, help="Số bước tương lai dự đoán (50 steps = 1s)")
    parser.add_argument("--kl_weight", type=float, default=10.0, help="Trọng số beta của KL loss")
    parser.add_argument("--num_workers", type=int, default=2, help="Số workers nạp dữ liệu")
    parser.add_argument("--save_path", type=str, default="dataset/act_openarm_model.pth", help="Đường dẫn lưu file trọng số .pth")

    args = parser.parse_args()
    train_act(args)
