#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Vision Backbone for ACT:
- 4-Channel Conv1 Adapter (RGB + Depth), depth channel initialized as mean(w_RGB)
- ResNet-18 (Layers 1..4) pretrained on ImageNet (Frozen)
- 2D Sinusoidal Positional Encoding (Fixed)
- 1x1 Conv Image Projection 512 -> d_model (512)
- Produces 300 Visual Tokens [B, 300, d_model] for 480x640 input
"""

import math
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as models


class SinusoidalPositionEmbedding2D(nn.Module):
    """
    2D Sinusoidal Positional Encoding cố định (không học - Fixed)
    Mã hóa không gian 2D cho đặc trưng thị giác theo công thức DETR/ACT.
    """
    def __init__(self, d_model: int = 512, temperature: float = 10000.0):
        super().__init__()
        assert d_model % 4 == 0, f"d_model ({d_model}) phải chia hết cho 4 để chia đều cho sin/cos x và y."
        self.d_model = d_model
        self.temperature = temperature
        self.num_pos_feats = d_model // 2

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        x: [B, C, H, W]
        returns: pos: [B, d_model, H, W]
        """
        B, _, H, W = x.shape
        device = x.device
        
        # Tạo lưới tọa độ 1..H và 1..W
        y_embed = torch.arange(1, H + 1, dtype=torch.float32, device=device).unsqueeze(1).repeat(1, W)
        x_embed = torch.arange(1, W + 1, dtype=torch.float32, device=device).unsqueeze(0).repeat(H, 1)

        dim_t = torch.arange(self.num_pos_feats, dtype=torch.float32, device=device)
        dim_t = self.temperature ** (2 * torch.div(dim_t, 2, rounding_mode="floor") / self.num_pos_feats)

        pos_x = x_embed[:, :, None] / dim_t
        pos_y = y_embed[:, :, None] / dim_t
        
        pos_x = torch.stack((pos_x[:, :, 0::2].sin(), pos_x[:, :, 1::2].cos()), dim=3).flatten(2)
        pos_y = torch.stack((pos_y[:, :, 0::2].sin(), pos_y[:, :, 1::2].cos()), dim=3).flatten(2)
        
        # Ghép [H, W, d_model] -> Permute [d_model, H, W] -> Repeat cho Batch [B, d_model, H, W]
        pos = torch.cat((pos_y, pos_x), dim=2).permute(2, 0, 1).unsqueeze(0).repeat(B, 1, 1, 1)
        return pos


class RGBDResNetBackbone(nn.Module):
    """
    Khối thị giác RGB-D cho mô hình ACT:
    - Nhận vào ảnh 4 kênh [B, 4, H, W] (3 RGB + 1 Depth).
    - Conv1 4-Channel Adapter: Khởi tạo RGB từ ImageNet, kênh Depth khởi tạo từ trung bình w_RGB.
    - ResNet-18 Layers 1..4: Đóng băng (Frozen) để chống overfitting trên tập demo nhỏ.
    - 1x1 Conv Image Projection: Chiếu 512 kênh về d_model (512).
    - 2D Sinusoidal PE: Cộng vào đặc trưng trước khi làm phẳng (flatten).
    - Đầu ra: visual_tokens [B, N_patches, d_model] (với 480x640 -> 15x20 = 300 tokens).
    """
    def __init__(
        self,
        d_model: int = 512,
        freeze_backbone: bool = True,
        in_channels: int = 4,
    ):
        super().__init__()
        self.d_model = d_model
        self.in_channels = in_channels

        # Tải trọng số ResNet-18 pretrained ImageNet
        resnet = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
        pretrained_conv1 = resnet.conv1

        # Tạo Conv1 4-Channel Adapter (Trainable)
        self.conv1_adapter = nn.Conv2d(
            in_channels=in_channels,
            out_channels=pretrained_conv1.out_channels,
            kernel_size=pretrained_conv1.kernel_size,
            stride=pretrained_conv1.stride,
            padding=pretrained_conv1.padding,
            bias=False,
        )

        # Khởi tạo trọng số Conv1 Adapter:
        # Kênh 0, 1, 2 (RGB) giữ nguyên trọng số ImageNet
        # Kênh 3 (Depth) khởi tạo bằng mean của 3 kênh RGB: mean(w_RGB)
        with torch.no_grad():
            self.conv1_adapter.weight[:, :3, :, :] = pretrained_conv1.weight
            if in_channels > 3:
                depth_init = pretrained_conv1.weight.mean(dim=1, keepdim=True)
                self.conv1_adapter.weight[:, 3:4, :, :] = depth_init

        # ResNet-18 Layers 1..4
        self.bn1 = resnet.bn1
        self.relu = resnet.relu
        self.maxpool = resnet.maxpool
        self.layer1 = resnet.layer1
        self.layer2 = resnet.layer2
        self.layer3 = resnet.layer3
        self.layer4 = resnet.layer4

        # Đóng băng các tầng ResNet18 nếu freeze_backbone = True
        if freeze_backbone:
            for module in [self.bn1, self.layer1, self.layer2, self.layer3, self.layer4]:
                for param in module.parameters():
                    param.requires_grad = False
                module.eval()

        # 1x1 Conv Image Projection (Trainable)
        self.img_proj = nn.Conv2d(512, d_model, kernel_size=1)
        
        # 2D Sinusoidal Positional Encoding (Fixed)
        self.pos_embed_2d = SinusoidalPositionEmbedding2D(d_model=d_model)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Input:
            x: [B, 4, H, W] - Ảnh 4 kênh RGB-D đã chuẩn hóa
        Output:
            visual_tokens: [B, N_patches, d_model] (Ví dụ: [B, 300, 512] khi H=480, W=640)
        """
        B = x.shape[0]

        # 1. Đi qua 4-Channel Conv1 Adapter
        out = self.conv1_adapter(x)
        out = self.bn1(out)
        out = self.relu(out)
        out = self.maxpool(out)

        # 2. Đi qua ResNet-18 Layers 1..4 (Frozen)
        out = self.layer1(out)
        out = self.layer2(out)
        out = self.layer3(out)
        feat = self.layer4(out)  # [B, 512, H_feat, W_feat]

        # 3. 1x1 Conv Projection + 2D Sinusoidal PE
        proj_feat = self.img_proj(feat)           # [B, d_model, H_feat, W_feat]
        pos = self.pos_embed_2d(proj_feat)         # [B, d_model, H_feat, W_feat]
        feat_with_pos = proj_feat + pos

        # 4. Làm phẳng thành chuỗi Visual Tokens: [B, d_model, H_feat, W_feat] -> [B, N_patches, d_model]
        visual_tokens = feat_with_pos.flatten(2).permute(0, 2, 1)
        return visual_tokens
