#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Configuration parameters for ACT (Action Chunking with Transformers)
Bimanual OpenArm (16-DOF: 2 arms x 8 motors) + 01 Chest RGB-D Camera
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class ModelConfig:
    # Vision Backbone
    in_channels: int = 4            # RGB (3) + Depth (1)
    backbone_type: str = "resnet18" # ResNet-18 (Layers 1..4 frozen)
    freeze_backbone: bool = True    # Đóng băng ImageNet pretrained backbone
    img_height: int = 480
    img_width: int = 640
    
    # Dimensions
    d_model: int = 512              # Transformer hidden dimension
    nheads: int = 8                 # Multi-head attention heads
    dim_feedforward: int = 2048     # FFN intermediate dimension
    dropout: float = 0.1
    
    # CVAE
    cvae_layers: int = 2            # Số layer TransformerEncoder của CVAE
    latent_dim: int = 32            # Kích thước latent vector z
    
    # Policy Decoder
    decoder_layers: int = 4         # Số layer TransformerDecoder của Policy
    chunk_size: int = 50            # Action chunk size k=50 (1 giây tại 50Hz)
    action_dim: int = 16            # 16 khớp (8 tay trái: J1..J7 + gripper; 8 tay phải: J1..J7 + gripper)
    qpos_dim: int = 16              # 16 góc khớp hiện tại
    
    # Depth Preprocessing
    depth_min_m: float = 0.2        # Khoảng cách bàn tối thiểu (mét)
    depth_max_m: float = 1.2        # Khoảng cách bàn tối đa (mét)


@dataclass
class TrainConfig:
    # Paths
    dataset_dir: str = "dataset/real_towel_folding"
    output_dir: str = "checkpoints/act_openarm"
    stats_file: Optional[str] = None
    resume_checkpoint: Optional[str] = None
    
    # Training Loop
    epochs: int = 500
    batch_size: int = 16
    lr: float = 1e-4
    lr_backbone: float = 1e-5       # Learning rate nhỏ cho conv1 adapter
    weight_decay: float = 1e-4
    lr_scheduler: str = "cosine"    # "cosine", "step", "none"
    warmup_epochs: int = 10
    min_lr: float = 1e-6
    
    # Loss Weights
    kl_weight: float = 10.0         # Trọng số beta cho KL Divergence loss
    loss_type: str = "l1"           # "l1" hoặc "l2"
    
    # Hardware & Performance
    num_workers: int = 2
    use_amp: bool = True            # Automatic Mixed Precision (FP16)
    seed: int = 42
    device: str = "cuda"            # "cuda" hoặc "cpu"
    
    # Validation & Logging
    val_split: float = 0.15         # 15% validation data
    eval_every: int = 10            # Đánh giá validation mỗi N epochs
    save_every: int = 50            # Lưu checkpoint định kỳ mỗi N epochs
    log_every_steps: int = 10       # In progress mỗi N batches
    use_tensorboard: bool = True
    use_wandb: bool = False
    wandb_project: str = "act-openarm"
    wandb_entity: Optional[str] = None


@dataclass
class EvalConfig:
    checkpoint_path: str = "checkpoints/act_openarm/best_checkpoint.pth"
    dataset_dir: str = "dataset/real_towel_folding"
    output_dir: str = "evaluation_results"
    batch_size: int = 16
    num_workers: int = 2
    temporal_ensemble: bool = True
    ensemble_m: float = 0.01        # Hệ số suy giảm trọng số exp(-m * i)
    device: str = "cuda"
