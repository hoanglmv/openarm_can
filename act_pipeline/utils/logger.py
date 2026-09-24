#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Production Logging System for ACT Training & Evaluation
Provides:
1. Console Logger: Clean, structured terminal output with timestamps and status indicators.
2. File Logger: Comprehensive text log in log_dir/train.log.
3. JSONL Logger: Machine-readable metric tracking in log_dir/metrics.jsonl for plotting.
4. TensorBoard Logger: Real-time visual monitoring of loss curves, LR, and validation metrics.
5. Optional WandB Logger: Cloud tracking integration if enabled.
"""

import os
import sys
import time
import json
import logging
from typing import Dict, Any, Optional

try:
    from torch.utils.tensorboard import SummaryWriter
    TENSORBOARD_AVAILABLE = True
except ImportError:
    TENSORBOARD_AVAILABLE = False


class MetricLogger:
    """
    Trình quản lý ghi chép (Logger) toàn diện cho quá trình Huấn luyện & Đánh giá ACT.
    """
    def __init__(
        self,
        log_dir: str,
        exp_name: str = "act_experiment",
        use_tensorboard: bool = True,
        use_wandb: bool = False,
        wandb_project: str = "act-openarm",
        wandb_entity: Optional[str] = None,
        config_dict: Optional[Dict[str, Any]] = None,
    ):
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)

        self.exp_name = exp_name
        self.metrics_file = os.path.join(self.log_dir, "metrics.jsonl")
        self.log_file = os.path.join(self.log_dir, "train.log")

        # 1. Thiết lập File & Stream Logger
        self.logger = logging.getLogger(exp_name)
        self.logger.setLevel(logging.INFO)
        self.logger.handlers.clear()

        # File Handler
        file_handler = logging.FileHandler(self.log_file, mode="a", encoding="utf-8")
        file_formatter = logging.Formatter(
            "[%(asctime)s] [%(levelname)s] %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
        )
        file_handler.setFormatter(file_formatter)
        self.logger.addHandler(file_handler)

        # Console Stream Handler
        console_handler = logging.StreamHandler(sys.stdout)
        console_formatter = logging.Formatter("[%(asctime)s] %(message)s", datefmt="%H:%M:%S")
        console_handler.setFormatter(console_formatter)
        self.logger.addHandler(console_handler)

        # 2. Khởi tạo TensorBoard
        self.tb_writer = None
        if use_tensorboard and TENSORBOARD_AVAILABLE:
            tb_dir = os.path.join(self.log_dir, "tensorboard")
            self.tb_writer = SummaryWriter(log_dir=tb_dir)
            self.info(f"[✓] Đã khởi tạo TensorBoard tại: {tb_dir}")
        elif use_tensorboard and not TENSORBOARD_AVAILABLE:
            self.warning("[!] Cảnh báo: tensorboard chưa được cài đặt. Bỏ qua ghi đồ thị TensorBoard.")

        # 3. Khởi tạo WandB (nếu được yêu cầu và cài đặt)
        self.use_wandb = use_wandb
        if self.use_wandb:
            try:
                import wandb
                wandb.init(
                    project=wandb_project,
                    entity=wandb_entity,
                    name=exp_name,
                    config=config_dict,
                )
                self.info("[✓] Đã kết nối với Weights & Biases (WandB).")
            except Exception as e:
                self.warning(f"[!] Không thể khởi động WandB: {e}. Vô hiệu hóa WandB.")
                self.use_wandb = False

        # Lưu config dưới dạng JSON
        if config_dict is not None:
            config_path = os.path.join(self.log_dir, "config.json")
            with open(config_path, "w", encoding="utf-8") as f:
                json.dump(config_dict, f, indent=2, default=str)
            self.info(f"[✓] Đã lưu cấu hình tại: {config_path}")

    def info(self, msg: str):
        self.logger.info(msg)

    def warning(self, msg: str):
        self.logger.warning(msg)

    def error(self, msg: str):
        self.logger.error(msg)

    def log_metrics(self, metrics: Dict[str, float], step: int, prefix: str = ""):
        """
        Ghi nhận các độ đo metrics vào:
        1. JSONL file (máy đọc)
        2. TensorBoard (đồ thị trực quan)
        3. WandB (nếu có)
        """
        timestamp = time.time()
        record = {
            "step": step,
            "timestamp": timestamp,
            **{f"{prefix}{k}": v for k, v in metrics.items()}
        }

        # Ghi vào metrics.jsonl
        with open(self.metrics_file, "a", encoding="utf-8") as f:
            f.write(json.dumps(record) + "\n")

        # Ghi vào TensorBoard
        if self.tb_writer is not None:
            for k, v in metrics.items():
                tag = f"{prefix}{k}" if prefix else k
                self.tb_writer.add_scalar(tag, v, global_step=step)

        # Ghi vào WandB
        if self.use_wandb:
            try:
                import wandb
                wandb.log({f"{prefix}{k}": v for k, v in metrics.items()}, step=step)
            except Exception:
                pass

    def log_epoch_summary(
        self,
        epoch: int,
        total_epochs: int,
        train_metrics: Dict[str, float],
        val_metrics: Optional[Dict[str, float]] = None,
        lr: float = 0.0,
        elapsed_sec: float = 0.0,
    ):
        """
        In dòng tóm tắt Epoch đẹp mắt ra console và log file.
        """
        line = (
            f"Epoch [{epoch:4d}/{total_epochs:4d}] "
            f"| Train Loss: {train_metrics.get('loss', 0.0):.4f} "
            f"(L1: {train_metrics.get('l1', 0.0):.4f}, KL: {train_metrics.get('kl', 0.0):.4f}) "
        )

        if val_metrics is not None:
            line += (
                f"| Val Loss: {val_metrics.get('loss', 0.0):.4f} "
                f"(L1: {val_metrics.get('l1', 0.0):.4f}) "
            )

        line += f"| LR: {lr:.2e} | Time: {elapsed_sec:.1f}s"
        self.info(line)

    def close(self):
        if self.tb_writer is not None:
            self.tb_writer.close()
        if self.use_wandb:
            try:
                import wandb
                wandb.finish()
            except Exception:
                pass
