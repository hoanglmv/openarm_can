#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Temporal Ensembling for ACT (Action Chunking with Transformers)
Implements exponential smoothing over overlapping action chunks:
omega_i = exp(-m * i)
a_t = sum(omega_i * a_{t | t-i}) / sum(omega_i)
Eliminates jerky motions and guarantees smooth trajectory execution on 16 Damiao motors.
"""

from typing import Dict, List, Optional, Tuple, Union
import math
import numpy as np
import torch


class TemporalEnsemblePolicy:
    """
    Bộ ghép hành động theo thời gian (Temporal Ensembling):
    OpenArm điều khiển ở tần số 50Hz, mô hình ACT dự đoán một chunk 50 bước góc mỗi chu kỳ.
    Thay vì thực thi một chunk rời rạc gây giật robot tại biên giới các chunk,
    thuật toán này lấy trung bình trượt có trọng số hàm mũ của tất cả các dự đoán chồng lấn lên nhau.
    """
    def __init__(
        self,
        chunk_size: int = 50,
        action_dim: int = 16,
        ensemble_m: float = 0.01,
    ):
        self.chunk_size = chunk_size
        self.action_dim = action_dim
        self.ensemble_m = ensemble_m

        # Bảng trọng số suy giảm mũ theo thời gian trôi qua: omega_i = exp(-m * i)
        # i = 0 là dự đoán mới nhất, i lớn hơn là dự đoán từ các bước quá khứ
        self.weights = np.exp(-ensemble_m * np.arange(chunk_size)) # [chunk_size]

        # Bộ đệm lưu trữ các dự đoán cho từng bước thời gian
        # Mỗi phần tử trong buffer là list các dự đoán cho bước thời gian đó: [(pred_action, weight)]
        self.reset()

    def reset(self):
        """
        Xóa sạch bộ đệm khi bắt đầu một Episode mới.
        """
        self.action_buffer: List[List[Tuple[np.ndarray, float]]] = []
        self.current_step = 0

    def update(self, action_chunk: Union[np.ndarray, torch.Tensor]) -> np.ndarray:
        """
        Cập nhật chunk hành động mới [chunk_size, 16] và trả về hành động đã làm mượt cho bước hiện tại.
        """
        if isinstance(action_chunk, torch.Tensor):
            action_chunk = action_chunk.detach().cpu().numpy()

        if action_chunk.ndim == 3:
            # Nếu batch size = 1, squeeze batch
            action_chunk = action_chunk[0]

        assert action_chunk.shape == (self.chunk_size, self.action_dim), (
            f"Shape action chunk {action_chunk.shape} không khớp với ({self.chunk_size}, {self.action_dim})"
        )

        # Mở rộng buffer nếu cần thiết
        target_len = self.current_step + self.chunk_size
        while len(self.action_buffer) < target_len:
            self.action_buffer.append([])

        # Đẩy các dự đoán từ chunk mới vào buffer với trọng số tương ứng
        for offset in range(self.chunk_size):
            step = self.current_step + offset
            weight = self.weights[offset]
            action = action_chunk[offset]
            self.action_buffer[step].append((action, weight))

        # Tính trung bình trọng số cho bước thời gian hiện tại
        current_predictions = self.action_buffer[self.current_step]
        total_weight = 0.0
        weighted_action = np.zeros(self.action_dim, dtype=np.float32)

        for action, weight in current_predictions:
            weighted_action += action * weight
            total_weight += weight

        smoothed_action = weighted_action / max(total_weight, 1e-6)

        self.current_step += 1
        return smoothed_action
