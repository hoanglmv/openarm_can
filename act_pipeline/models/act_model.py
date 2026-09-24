#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unified ACT Policy Architecture for Bimanual OpenArm (16-DOF) + Chest RGB-D
Implements exact architecture specified in ACT_ARCHITECTURE.md:
- 01 Chest Camera RGB-D [B, 4, 480, 640]
- Conv1 4-Channel Adapter (Trainable)
- ResNet-18 Layers 1..4 (Frozen)
- 2D Sinusoidal PE (Fixed) -> 300 Visual Tokens [B, 300, 512]
- Joint State Projection Linear 16 -> 512
- CVAE Transformer Encoder (2 Layers) + Latent Heads 512 -> 32
- Latent Projection Linear 32 -> 512
- Memory Tokens [B, 302, 512]
- 50 Action Queries + 1D Temporal Sinusoidal PE
- Transformer Decoder (4 Layers)
- Action Linear Head 512 -> 16
"""

from typing import Tuple, Optional, Dict, Any
import torch
import torch.nn as nn
import torch.nn.functional as F

from act_pipeline.config import ModelConfig
from act_pipeline.models.backbone import RGBDResNetBackbone
from act_pipeline.models.cvae import CVAEEncoder
from act_pipeline.models.policy import TransformerPolicyDecoder


class ACTPolicy(nn.Module):
    """
    Toàn bộ mô hình ACT cho robot OpenArm hai tay (16-DOF):
    Ghép nối Backbone thị giác RGB-D, CVAE Latent Generator và Policy Decoder.
    """
    def __init__(self, config: Optional[ModelConfig] = None):
        super().__init__()
        if config is None:
            config = ModelConfig()
        self.config = config

        self.d_model = config.d_model
        self.chunk_size = config.chunk_size
        self.action_dim = config.action_dim
        self.qpos_dim = config.qpos_dim
        self.latent_dim = config.latent_dim

        # 1. THỊ GIÁC (Vision Backbone)
        self.backbone = RGBDResNetBackbone(
            d_model=config.d_model,
            freeze_backbone=config.freeze_backbone,
            in_channels=config.in_channels,
        )

        # 2. TRẠNG THÁI KHỚP (Joint State Projection)
        self.qpos_proj = nn.Linear(config.qpos_dim, config.d_model)

        # 3. CVAE (Latent Space)
        self.cvae = CVAEEncoder(
            d_model=config.d_model,
            nheads=config.nheads,
            dim_feedforward=config.dim_feedforward,
            num_layers=config.cvae_layers,
            dropout=config.dropout,
            action_dim=config.action_dim,
            qpos_dim=config.qpos_dim,
            latent_dim=config.latent_dim,
        )

        # 4. POLICY DECODER
        self.policy_decoder = TransformerPolicyDecoder(
            d_model=config.d_model,
            nheads=config.nheads,
            dim_feedforward=config.dim_feedforward,
            num_layers=config.decoder_layers,
            dropout=config.dropout,
            chunk_size=config.chunk_size,
            action_dim=config.action_dim,
        )

    def forward(
        self,
        image: torch.Tensor,
        qpos: torch.Tensor,
        actions: Optional[torch.Tensor] = None,
        is_pad: Optional[torch.Tensor] = None,
    ) -> Tuple[torch.Tensor, Optional[torch.Tensor], Optional[torch.Tensor]]:
        """
        Inputs:
            image: [B, 4, H, W] - Ảnh 4 kênh RGB-D (RGB + Depth)
            qpos: [B, 16] - Vector góc khớp 16 motor hiện tại
            actions: [B, chunk_size, 16] - Chuỗi hành động tương lai (Chỉ truyền khi Train)
            is_pad: [B, chunk_size] bool - Mask padding nếu gần cuối episode
        Outputs:
            pred_actions: [B, chunk_size, 16] - Quỹ đạo 50 bước dự đoán cho 16 motor
            mu: [B, latent_dim] hoặc None khi Inference
            logvar: [B, latent_dim] hoặc None khi Inference
        """
        B = image.shape[0]

        # 1. Trích xuất đặc trưng thị giác -> 300 Visual Tokens [B, 300, d_model]
        visual_tokens = self.backbone(image)

        # 2. Chiếu góc khớp hiện tại -> 1 qpos Token [B, 1, d_model]
        qpos_token = self.qpos_proj(qpos).unsqueeze(1)

        # 3. CVAE: Lấy mẫu latent vector z
        # Train: z = mu + eps * sigma từ (actions, qpos)
        # Eval / Inference: z = 0
        z_token, mu, logvar = self.cvae(qpos=qpos, actions=actions, is_pad=is_pad)

        # 4. Trộn Memory Tokens: [300 Visual + 1 qpos + 1 z] -> [B, 302, d_model]
        memory = torch.cat([visual_tokens, qpos_token, z_token], dim=1)

        # 5. Giải mã Action Chunk qua Policy Decoder
        pred_actions = self.policy_decoder(memory) # [B, chunk_size, action_dim]

        return pred_actions, mu, logvar

    def compute_loss(
        self,
        pred_actions: torch.Tensor,
        actions: torch.Tensor,
        is_pad: Optional[torch.Tensor],
        mu: Optional[torch.Tensor],
        logvar: Optional[torch.Tensor],
        kl_weight: float = 10.0,
        loss_type: str = "l1",
    ) -> Dict[str, torch.Tensor]:
        """
        Tính toán hàm mất mát Loss:
        Loss = L_recon (L1 hoặc L2 có padding mask) + beta * KL_Loss
        """
        B = actions.shape[0]

        # 1. Reconstruction Loss (L1 hoặc MSE có mask loại bỏ padding)
        if loss_type == "l1":
            raw_recon_loss = F.l1_loss(pred_actions, actions, reduction="none") # [B, chunk_size, action_dim]
        else:
            raw_recon_loss = F.mse_loss(pred_actions, actions, reduction="none")

        if is_pad is not None:
            # is_pad có shape [B, chunk_size], True là padding (bị loại)
            valid_mask = (~is_pad).unsqueeze(-1).float() # [B, chunk_size, 1]
            recon_loss = (raw_recon_loss * valid_mask).sum() / valid_mask.sum().clamp(min=1.0) / self.action_dim
        else:
            recon_loss = raw_recon_loss.mean()

        # 2. KL Divergence Loss giữa q(z | actions, qpos) và p(z) ~ N(0, I)
        if mu is not None and logvar is not None:
            # D_KL = -0.5 * sum(1 + log(sigma^2) - mu^2 - sigma^2)
            kl_loss = -0.5 * torch.sum(1 + logvar - mu.pow(2) - logvar.exp()) / B
        else:
            kl_loss = torch.tensor(0.0, device=pred_actions.device)

        # 3. Tổng hàm mất mát
        total_loss = recon_loss + kl_weight * kl_loss

        return {
            "loss": total_loss,
            "recon_loss": recon_loss,
            "kl_loss": kl_loss,
        }

    @torch.no_grad()
    def predict_action_chunk(
        self,
        image: torch.Tensor,
        qpos: torch.Tensor,
    ) -> torch.Tensor:
        """
        Dự đoán Action Chunk trong quá trình Inference / Robot Deployment.
        Tự động gán cứng z = 0 và không tính gradient.
        Inputs:
            image: [B, 4, H, W]
            qpos: [B, 16]
        Output:
            pred_actions: [B, chunk_size, 16]
        """
        self.eval()
        pred_actions, _, _ = self.forward(image=image, qpos=qpos, actions=None, is_pad=None)
        return pred_actions

    def get_parameter_summary(self) -> Dict[str, Any]:
        """
        Thống kê chi tiết số tham số Trainable vs Frozen theo đúng bảng ACT_ARCHITECTURE.md
        """
        total_params = sum(p.numel() for p in self.parameters())
        trainable_params = sum(p.numel() for p in self.parameters() if p.requires_grad)
        frozen_params = total_params - trainable_params

        summary = {
            "total_params": total_params,
            "trainable_params": trainable_params,
            "frozen_params": frozen_params,
            "trainable_pct": (trainable_params / total_params) * 100.0 if total_params > 0 else 0.0,
            "modules": {
                "conv1_adapter": sum(p.numel() for p in self.backbone.conv1_adapter.parameters() if p.requires_grad),
                "resnet_frozen": sum(p.numel() for p in [self.backbone.layer1, self.backbone.layer2, self.backbone.layer3, self.backbone.layer4] for p in p.parameters() if not p.requires_grad),
                "img_proj": sum(p.numel() for p in self.backbone.img_proj.parameters() if p.requires_grad),
                "qpos_proj": sum(p.numel() for p in self.qpos_proj.parameters() if p.requires_grad),
                "cvae": sum(p.numel() for p in self.cvae.parameters() if p.requires_grad),
                "policy_decoder": sum(p.numel() for p in self.policy_decoder.parameters() if p.requires_grad),
            }
        }
        return summary
