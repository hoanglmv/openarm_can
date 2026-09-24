#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robust Checkpoint Management for ACT
Handles:
1. Saving best_checkpoint.pth (based on lowest validation loss / L1 loss)
2. Saving latest_checkpoint.pth (continuous recovery point)
3. Periodic checkpoints: checkpoint_epoch_{N}.pth
4. Saving clean deployment weights: act_deploy_weights.pth
5. Resuming training seamlessly (restores model, optimizer, lr scheduler, scaler, stats, epoch)
"""

import os
import glob
from typing import Dict, Any, Optional, Tuple, Union
import torch
import torch.nn as nn


class CheckpointManager:
    """
    Quản lý lưu trữ và khôi phục checkpoint mô hình ACT.
    """
    def __init__(self, checkpoint_dir: str):
        self.checkpoint_dir = checkpoint_dir
        os.makedirs(self.checkpoint_dir, exist_ok=True)

        self.best_checkpoint_path = os.path.join(self.checkpoint_dir, "best_checkpoint.pth")
        self.latest_checkpoint_path = os.path.join(self.checkpoint_dir, "latest_checkpoint.pth")
        self.deploy_weights_path = os.path.join(self.checkpoint_dir, "act_deploy_weights.pth")

    def save_checkpoint(
        self,
        epoch: int,
        global_step: int,
        model: nn.Module,
        optimizer: torch.optim.Optimizer,
        scheduler: Optional[Any],
        scaler: Optional[Any],
        stats: Dict[str, Any],
        config: Dict[str, Any],
        metrics: Dict[str, float],
        is_best: bool = False,
        save_periodic: bool = False,
    ) -> Dict[str, str]:
        """
        Lưu trạng thái huấn luyện đầy đủ vào file checkpoint.
        """
        saved_paths = {}

        # Trích xuất state_dict của mô hình (xử lý DataParallel nếu có)
        model_state = model.module.state_dict() if hasattr(model, "module") else model.state_dict()

        checkpoint_data = {
            "epoch": epoch,
            "global_step": global_step,
            "model_state_dict": model_state,
            "optimizer_state_dict": optimizer.state_dict(),
            "scheduler_state_dict": scheduler.state_dict() if scheduler is not None else None,
            "scaler_state_dict": scaler.state_dict() if scaler is not None else None,
            "stats": stats,
            "config": config,
            "metrics": metrics,
        }

        # 1. Luôn lưu vào latest_checkpoint.pth để đảm bảo an toàn nếu gặp sự cố sập nguồn/ngắt kết nối
        torch.save(checkpoint_data, self.latest_checkpoint_path)
        saved_paths["latest"] = self.latest_checkpoint_path

        # 2. Lưu checkpoint tốt nhất nếu is_best = True
        if is_best:
            torch.save(checkpoint_data, self.best_checkpoint_path)
            saved_paths["best"] = self.best_checkpoint_path

            # Đồng thời lưu một bản weights rút gọn nhẹ phục vụ triển khai chạy thực tế trên robot (Deployment)
            deploy_data = {
                "model_state_dict": model_state,
                "stats": stats,
                "config": config,
                "best_epoch": epoch,
                "best_metrics": metrics,
            }
            torch.save(deploy_data, self.deploy_weights_path)
            saved_paths["deploy"] = self.deploy_weights_path

        # 3. Lưu định kỳ mỗi N epoch nếu yêu cầu
        if save_periodic:
            periodic_path = os.path.join(self.checkpoint_dir, f"checkpoint_epoch_{epoch:04d}.pth")
            torch.save(checkpoint_data, periodic_path)
            saved_paths["periodic"] = periodic_path

        return saved_paths

    def load_checkpoint(
        self,
        checkpoint_path: str,
        model: nn.Module,
        optimizer: Optional[torch.optim.Optimizer] = None,
        scheduler: Optional[Any] = None,
        scaler: Optional[Any] = None,
        device: Union[str, torch.device] = "cpu",
        strict: bool = True,
    ) -> Dict[str, Any]:
        """
        Khôi phục toàn bộ trạng thái từ một file checkpoint.
        """
        if not os.path.exists(checkpoint_path):
            raise FileNotFoundError(f"Không tìm thấy checkpoint tại: {checkpoint_path}")

        print(f"[*] Đang tải checkpoint từ: {checkpoint_path} lên {device}...")
        checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)

        # 1. Tải trọng số mô hình
        state_dict = checkpoint["model_state_dict"] if "model_state_dict" in checkpoint else checkpoint
        
        # Nếu model bọc trong DataParallel
        target_model = model.module if hasattr(model, "module") else model
        missing, unexpected = target_model.load_state_dict(state_dict, strict=strict)
        if len(missing) > 0:
            print(f"[!] Cảnh báo các keys bị thiếu khi load: {missing}")
        if len(unexpected) > 0:
            print(f"[!] Cảnh báo các keys thừa: {unexpected}")

        # 2. Khôi phục Optimizer nếu được cung cấp
        if optimizer is not None and "optimizer_state_dict" in checkpoint and checkpoint["optimizer_state_dict"] is not None:
            optimizer.load_state_dict(checkpoint["optimizer_state_dict"])
            print("[✓] Đã khôi phục trạng thái Optimizer.")

        # 3. Khôi phục Scheduler nếu được cung cấp
        if scheduler is not None and "scheduler_state_dict" in checkpoint and checkpoint["scheduler_state_dict"] is not None:
            scheduler.load_state_dict(checkpoint["scheduler_state_dict"])
            print("[✓] Đã khôi phục trạng thái Learning Rate Scheduler.")

        # 4. Khôi phục GradScaler nếu có
        if scaler is not None and "scaler_state_dict" in checkpoint and checkpoint["scaler_state_dict"] is not None:
            scaler.load_state_dict(checkpoint["scaler_state_dict"])
            print("[✓] Đã khôi phục trạng thái GradScaler (AMP).")

        epoch = checkpoint.get("epoch", 0)
        global_step = checkpoint.get("global_step", 0)
        stats = checkpoint.get("stats", None)
        config = checkpoint.get("config", {})
        metrics = checkpoint.get("metrics", {})

        print(f"[✓] Đã khôi phục thành công checkpoint tại Epoch {epoch}, Global Step {global_step}!")
        return {
            "epoch": epoch,
            "global_step": global_step,
            "stats": stats,
            "config": config,
            "metrics": metrics,
        }

    def find_latest_checkpoint(self) -> Optional[str]:
        """
        Tự động tìm checkpoint mới nhất trong thư mục để khôi phục (Resume).
        """
        if os.path.exists(self.latest_checkpoint_path):
            return self.latest_checkpoint_path

        periodic_files = sorted(glob.glob(os.path.join(self.checkpoint_dir, "checkpoint_epoch_*.pth")))
        if len(periodic_files) > 0:
            return periodic_files[-1]

        if os.path.exists(self.best_checkpoint_path):
            return self.best_checkpoint_path

        return None
