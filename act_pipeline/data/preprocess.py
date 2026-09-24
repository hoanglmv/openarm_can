#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RGB-D Data Preprocessing Pipeline
Tuân thủ chuẩn kỹ thuật trong DATA_FORMAT_SPECIFICATION.md (Mục 5):
- RGB: chuẩn hóa theo chuẩn ImageNet (Mean: [0.485, 0.456, 0.406], Std: [0.229, 0.224, 0.225])
- Depth: Đổi từ mm sang mét, clip khoảng bàn [0.2m - 1.2m] và chuẩn hóa tuyến tính về [0, 1]
- Ghép 4 kênh: [3 RGB, 1 Depth] -> Tensor [4, H, W]
"""

from typing import Optional, Union, Tuple
import numpy as np
import torch
import torch.nn.functional as F

# ImageNet statistics
IMAGENET_MEAN = torch.tensor([0.485, 0.456, 0.406], dtype=torch.float32).view(3, 1, 1)
IMAGENET_STD = torch.tensor([0.229, 0.224, 0.225], dtype=torch.float32).view(3, 1, 1)


def preprocess_rgbd(
    rgb_raw: np.ndarray,
    depth_raw: Optional[np.ndarray] = None,
    depth_min_m: float = 0.2,
    depth_max_m: float = 1.2,
    target_size: Optional[Tuple[int, int]] = None,
) -> torch.Tensor:
    """
    Tiền xử lý cặp ảnh RGB và Depth thành Tensor 4 kênh cho ACT:
    Inputs:
        rgb_raw: [H, W, 3] uint8 (0 - 255)
        depth_raw: [H, W] uint16 (mm) hoặc float32 (m). Nếu None, tự sinh kênh depth ước lượng.
        depth_min_m: Khoảng cách tối thiểu (0.2 m)
        depth_max_m: Khoảng cách tối đa (1.2 m)
        target_size: (H_target, W_target) - Tùy chọn resize
    Output:
        rgbd_tensor: [4, H, W] float32
    """
    # 1. Tiền xử lý RGB: Chuyển [H, W, 3] -> [3, H, W], chia 255 và chuẩn hóa ImageNet
    if not isinstance(rgb_raw, np.ndarray):
        rgb_raw = np.array(rgb_raw)

    rgb_float = torch.from_numpy(rgb_raw).permute(2, 0, 1).float() / 255.0 # [3, H, W]
    rgb_norm = (rgb_float - IMAGENET_MEAN) / IMAGENET_STD

    # 2. Tiền xử lý Depth:
    H, W = rgb_raw.shape[:2]
    if depth_raw is not None:
        if not isinstance(depth_raw, np.ndarray):
            depth_raw = np.array(depth_raw)

        # Nếu kiểu dữ liệu là uint16 (milimét theo chuẩn specification)
        if depth_raw.dtype == np.uint16 or depth_raw.max() > 20.0:
            depth_m = depth_raw.astype(np.float32) / 1000.0
        else:
            depth_m = depth_raw.astype(np.float32)

        # Clip khoảng giá trị mặt bàn thao tác [0.2m, 1.2m]
        depth_clipped = np.clip(depth_m, depth_min_m, depth_max_m)
        # Chuẩn hóa min-max về [0, 1]
        depth_norm_np = (depth_clipped - depth_min_m) / (depth_max_m - depth_min_m)
        depth_norm = torch.from_numpy(depth_norm_np).unsqueeze(0).float() # [1, H, W]
    else:
        # Fallback: Kênh depth trung tính (mặt phẳng bàn ~0.5m) nếu dataset chỉ có 3 kênh RGB
        default_val = (0.6 - depth_min_m) / (depth_max_m - depth_min_m)
        depth_norm = torch.full((1, H, W), fill_value=default_val, dtype=torch.float32)

    # 3. Ghép thành 4 kênh: [3 RGB, 1 Depth]
    rgbd_tensor = torch.cat([rgb_norm, depth_norm], dim=0) # [4, H, W]

    # 4. Tùy chọn Resize nếu target_size được chỉ định
    if target_size is not None and (H, W) != target_size:
        rgbd_tensor = F.interpolate(
            rgbd_tensor.unsqueeze(0),
            size=target_size,
            mode="bilinear",
            align_corners=False,
        ).squeeze(0)

    return rgbd_tensor
