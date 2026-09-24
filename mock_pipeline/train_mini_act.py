#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Train Mini-ACT Model on Mock Dataset
Huấn luyện mô hình ACT mini trên dữ liệu giả lập để kiểm tra tính thông suốt của toàn bộ Pipeline.
"""

import os
import glob
import math
import h5py
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
import torchvision.models as models

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# ==============================================================================
# 1. DATASET & DATALOADER
# ==============================================================================
class EpisodicDataset(Dataset):
    def __init__(self, data_dir, chunk_size=50):
        self.chunk_size = chunk_size
        self.files = sorted(glob.glob(os.path.join(data_dir, "*.hdf5")))
        self.samples = []

        print(f"[*] Đang nạp dữ liệu từ {len(self.files)} file HDF5...")
        for file_idx, fpath in enumerate(self.files):
            with h5py.File(fpath, "r") as f:
                T = len(f["action"])
                for start_t in range(0, T - chunk_size, 5):  # Lấy mẫu bước nhảy 5
                    self.samples.append((file_idx, start_t))

        print(f"[✓] Tổng số mẫu huấn luyện (Action Chunks): {len(self.samples)}")

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        file_idx, start_t = self.samples[idx]
        with h5py.File(self.files[file_idx], "r") as f:
            # 1. Ảnh camera ngực tại thời điểm start_t
            img = f["observations/images/chest"][start_t]  # [480, 640, 3] uint8
            # Resize nhẹ để tăng tốc độ train thử: 240x320
            img = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
            img = F.interpolate(img.unsqueeze(0), size=(240, 320), mode="bilinear").squeeze(0)

            # 2. qpos tại thời điểm start_t
            qpos = torch.tensor(f["observations/qpos"][start_t], dtype=torch.float32)

            # 3. Chuỗi hành động 50 bước tương lai [k, 8]
            actions = torch.tensor(
                f["action"][start_t : start_t + self.chunk_size], dtype=torch.float32
            )

        return img, qpos, actions


# ==============================================================================
# 2. MÔ HÌNH ACT MINI (ResNet-18 + CVAE + Transformer Decoder)
# ==============================================================================
class SinusoidalPositionEmbedding2D(nn.Module):
    """Mã hóa vị trí 2D hình sin cố định cho ảnh"""
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


class MiniACT(nn.Module):
    def __init__(self, d_model=256, chunk_size=50, action_dim=8, latent_dim=16):
        super().__init__()
        self.chunk_size = chunk_size
        self.action_dim = action_dim
        self.latent_dim = latent_dim
        self.d_model = d_model

        # 1. Visual Backbone: ResNet-18
        resnet = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
        # Bỏ 2 tầng cuối (avgpool và fc)
        self.backbone = nn.Sequential(*list(resnet.children())[:-2])
        self.pos_embed_2d = SinusoidalPositionEmbedding2D(d_model)
        self.img_proj = nn.Conv2d(512, d_model, kernel_size=1)

        # 2. Projection cho qpos và actions
        self.qpos_proj = nn.Linear(action_dim, d_model)
        self.action_proj = nn.Linear(action_dim, d_model)

        # 3. CVAE Encoder: Nén [action_chunk + qpos] thành latent z
        cvae_layer = nn.TransformerEncoderLayer(d_model=d_model, nhead=4, dim_feedforward=512, batch_first=True)
        self.cvae_encoder = nn.TransformerEncoder(cvae_layer, num_layers=2)
        self.cls_token = nn.Parameter(torch.randn(1, 1, d_model))
        self.latent_mu = nn.Linear(d_model, latent_dim)
        self.latent_logvar = nn.Linear(d_model, latent_dim)

        # 4. Latent z projector
        self.latent_proj = nn.Linear(latent_dim, d_model)

        # 5. Transformer Decoder Policy: Nhận visual tokens + qpos + latent z -> Action Chunk
        decoder_layer = nn.TransformerDecoderLayer(d_model=d_model, nhead=4, dim_feedforward=512, batch_first=True)
        self.policy_decoder = nn.TransformerDecoder(decoder_layer, num_layers=3)
        self.action_queries = nn.Parameter(torch.randn(1, chunk_size, d_model))
        self.action_head = nn.Linear(d_model, action_dim)

    def forward(self, img, qpos, actions=None):
        B = img.shape[0]

        # A. Trích xuất đặc trưng thị giác từ Camera Ngực
        feat = self.backbone(img) # [B, 512, H', W']
        pos = self.pos_embed_2d(feat)
        feat = self.img_proj(feat) + pos
        visual_tokens = feat.flatten(2).permute(0, 2, 1) # [B, N_patches, d_model]

        # B. Token góc khớp hiện tại
        qpos_token = self.qpos_proj(qpos).unsqueeze(1) # [B, 1, d_model]

        # C. Xử lý CVAE Latent z
        if actions is not None:
            # GIAI ĐOẠN TRAINING: Encode actions thành z
            action_tokens = self.action_proj(actions) # [B, k, d_model]
            cls_tokens = self.cls_token.repeat(B, 1, 1)
            enc_input = torch.cat([cls_tokens, qpos_token, action_tokens], dim=1)
            enc_out = self.cvae_encoder(enc_input)
            cls_out = enc_out[:, 0]

            mu = self.latent_mu(cls_out)
            logvar = self.latent_logvar(cls_out)
            std = torch.exp(0.5 * logvar)
            eps = torch.randn_like(std)
            z = mu + eps * std # Reparameterization trick
        else:
            # GIAI ĐOẠN INFERENCE: Gán z = 0 (tâm của phân phối)
            z = torch.zeros(B, self.latent_dim, device=img.device)
            mu, logvar = None, None

        z_token = self.latent_proj(z).unsqueeze(1) # [B, 1, d_model]

        # D. Ghép các tokens đưa vào Memory của Transformer Decoder
        memory = torch.cat([visual_tokens, qpos_token, z_token], dim=1)

        # E. Giải mã ra Action Chunk
        tgt_queries = self.action_queries.repeat(B, 1, 1)
        decoded = self.policy_decoder(tgt=tgt_queries, memory=memory)
        pred_actions = self.action_head(decoded) # [B, k=50, 8]

        return pred_actions, mu, logvar


# ==============================================================================
# 3. VÒNG LẶP HUẤN LUYỆN THỬ NGHIỆM (TRAIN 5 EPOCHS)
# ==============================================================================
def train():
    dataset = EpisodicDataset("dataset/mock_episodes", chunk_size=50)
    dataloader = DataLoader(dataset, batch_size=4, shuffle=True)

    model = MiniACT(d_model=256, chunk_size=50, action_dim=8, latent_dim=16).to(DEVICE)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4, weight_decay=1e-4)

    print(f"\n[*] BẮT ĐẦU HUẤN LUYỆN MINI-ACT TRÊN THIẾT BỊ: {DEVICE} (5 Epochs)...")
    model.train()

    for epoch in range(1, 6):
        total_loss, total_l1, total_kl = 0.0, 0.0, 0.0
        for img, qpos, actions in dataloader:
            img = img.to(DEVICE)
            qpos = qpos.to(DEVICE)
            actions = actions.to(DEVICE)

            optimizer.zero_grad()
            pred_actions, mu, logvar = model(img, qpos, actions)

            # 1. L1 Reconstruction Loss
            l1_loss = F.l1_loss(pred_actions, actions)

            # 2. KL Divergence Loss
            kl_loss = -0.5 * torch.sum(1 + logvar - mu.pow(2) - logvar.exp()) / img.shape[0]

            # Tổng hợp loss: L = L1 + beta * KL (beta = 10)
            loss = l1_loss + 10.0 * kl_loss

            loss.backward()
            optimizer.step()

            total_loss += loss.item()
            total_l1 += l1_loss.item()
            total_kl += kl_loss.item()

        n_batches = len(dataloader)
        print(f"Epoch [{epoch}/5] -> Total Loss: {total_loss/n_batches:.4f} | L1 Loss: {total_l1/n_batches:.4f} | KL Loss: {total_kl/n_batches:.4f}")

    # Lưu mô hình checkpoint
    ckpt_path = "dataset/mini_act_model.pth"
    torch.save(model.state_dict(), ckpt_path)
    print(f"\n[✓] ĐÃ LƯU CHECKPOINT THÀNH CÔNG TẠI: {ckpt_path}")


if __name__ == "__main__":
    train()
