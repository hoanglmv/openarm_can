#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Episodic Dataset Loader for ACT Bimanual OpenArm
Supports HDF5 files containing 16-DOF joint data and RGB-D camera feeds.
Handles future action chunking [50, 16], boundary padding with is_pad mask,
and Z-score normalization for actions and qpos.
"""

import os
import glob
from typing import Dict, List, Optional, Tuple, Union, Any
import numpy as np
import torch
from torch.utils.data import Dataset
import h5py

from act_pipeline.data.preprocess import preprocess_rgbd
from act_pipeline.data.normalization import normalize_data


class BimanualEpisodicDataset(Dataset):
    """
    Episodic Dataset cho ACT:
    - Đọc các file HDF5 đại diện cho từng Episode.
    - Cắt lát dữ liệu thành các mẫu: (image_t, qpos_t, actions_{t:t+k}, is_pad).
    - Quản lý padding mask chính xác khi gần cuối Episode.
    - Chuẩn hóa Z-score toàn bộ vector góc khớp và hành động.
    """
    def __init__(
        self,
        file_paths: List[str],
        stats: Dict[str, np.ndarray],
        chunk_size: int = 50,
        step_stride: int = 1,
        target_img_size: Optional[Tuple[int, int]] = (480, 640),
        depth_min_m: float = 0.2,
        depth_max_m: float = 1.2,
    ):
        super().__init__()
        self.file_paths = sorted(file_paths)
        if len(self.file_paths) == 0:
            raise ValueError("Không có file HDF5 nào được cung cấp cho Dataset!")

        self.stats = stats
        self.chunk_size = chunk_size
        self.step_stride = step_stride
        self.target_img_size = target_img_size
        self.depth_min_m = depth_min_m
        self.depth_max_m = depth_max_m

        self.samples: List[Tuple[int, int]] = [] # (file_idx, start_t)
        self.episode_lengths: List[int] = []

        self._build_index()

    def _build_index(self):
        """
        Quét nhanh độ dài từng episode để lập chỉ mục các lát cắt (file_idx, start_t).
        """
        total_steps = 0
        for file_idx, fpath in enumerate(self.file_paths):
            with h5py.File(fpath, "r") as f:
                T = len(f["action"])
                self.episode_lengths.append(T)
                total_steps += T
                # Lấy tất cả các thời điểm start_t với bước nhảy step_stride
                for start_t in range(0, T, self.step_stride):
                    self.samples.append((file_idx, start_t))

        print(f"[✓] Đã lập chỉ mục Dataset: {len(self.file_paths)} episodes, {total_steps} timesteps, {len(self.samples)} chunks (k={self.chunk_size}, stride={self.step_stride}).")

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        file_idx, start_t = self.samples[idx]
        fpath = self.file_paths[file_idx]

        with h5py.File(fpath, "r") as f:
            T = len(f["action"])

            # 1. ĐỌC VÀ TIỀN XỬ LÝ ẢNH RGB-D TẠI THỜI ĐIỂM start_t
            obs_img = f["observations/images"]
            rgb_raw = None
            depth_raw = None

            if "chest_rgb" in obs_img and "chest_depth" in obs_img:
                # Định dạng chuẩn theo DATA_FORMAT_SPECIFICATION.md
                rgb_raw = obs_img["chest_rgb"][start_t]
                depth_raw = obs_img["chest_depth"][start_t]
            elif "chest" in obs_img:
                raw_chest = obs_img["chest"][start_t]
                if raw_chest.shape[-1] == 4:
                    rgb_raw = raw_chest[..., :3]
                    depth_raw = raw_chest[..., 3]
                else:
                    rgb_raw = raw_chest
                    # Nếu có dataset chest_depth riêng bên ngoài
                    if "chest_depth" in obs_img:
                        depth_raw = obs_img["chest_depth"][start_t]
            else:
                # Thử tìm bất kỳ dataset ảnh nào trong images
                first_key = list(obs_img.keys())[0]
                rgb_raw = obs_img[first_key][start_t]

            # Chuyển đổi thành tensor 4 kênh RGB-D
            rgbd_tensor = preprocess_rgbd(
                rgb_raw=rgb_raw,
                depth_raw=depth_raw,
                depth_min_m=self.depth_min_m,
                depth_max_m=self.depth_max_m,
                target_size=self.target_img_size,
            )

            # 2. ĐỌC VÀ CHUẨN HÓA GÓC KHỚP qpos TẠI THỜI ĐIỂM start_t
            raw_qpos = f["observations/qpos"][start_t].astype(np.float32)
            norm_qpos = normalize_data(raw_qpos, self.stats["qpos_mean"], self.stats["qpos_std"])
            qpos_tensor = torch.tensor(norm_qpos, dtype=torch.float32)

            # 3. ĐỌC VÀ CHUẨN HÓA CHUỖI HÀNH ĐỘNG ACTION CHUNK [start_t : start_t + chunk_size]
            end_t = start_t + self.chunk_size
            is_pad = np.zeros(self.chunk_size, dtype=bool)

            if end_t <= T:
                # Đủ 50 bước tương lai
                raw_actions = f["action"][start_t:end_t].astype(np.float32)
            else:
                # Gần cuối episode: Lấy phần còn lại và lặp lại bước cuối cùng để làm đệm padding
                available = f["action"][start_t:T].astype(np.float32)
                num_available = len(available)
                pad_count = self.chunk_size - num_available
                last_act = available[-1:]
                padding = np.repeat(last_act, pad_count, axis=0)
                raw_actions = np.concatenate([available, padding], axis=0)
                is_pad[num_available:] = True

            norm_actions = normalize_data(raw_actions, self.stats["action_mean"], self.stats["action_std"])
            actions_tensor = torch.tensor(norm_actions, dtype=torch.float32)
            is_pad_tensor = torch.tensor(is_pad, dtype=torch.bool)

        return {
            "image": rgbd_tensor,      # [4, H, W]
            "qpos": qpos_tensor,        # [16]
            "actions": actions_tensor,  # [chunk_size, 16]
            "is_pad": is_pad_tensor,    # [chunk_size]
        }
