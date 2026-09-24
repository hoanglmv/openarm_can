#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Download and Convert Real-World Towel Folding Dataset from Hugging Face LeRobot
Dataset gốc: lerobot/aloha_static_towel (Stanford ALOHA Towel Folding, 50 episodes, 50 Hz)
Tự động chuyển đổi sang định dạng HDF5 chuẩn của OpenArm (1 Camera góc cao/ngực + 8 khớp)
"""

import os
import sys
import argparse
import h5py
import cv2
import pandas as pd
import numpy as np
from huggingface_hub import hf_hub_download

# Đảm bảo UTF-8
if sys.stdout.encoding != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def download_and_convert(output_dir="dataset/real_towel_folding", max_episodes=50):
    os.makedirs(output_dir, exist_ok=True)
    repo_id = "lerobot/aloha_static_towel"

    print("=" * 70)
    print(f"[*] ĐANG TẢI DỮ LIỆU GẤP KHĂN THẬT TỪ HUGGING FACE: {repo_id}")
    print("=" * 70)

    # 1. Tải bảng dữ liệu động học (1.7 MB)
    print("[1/3] Đang tải file dữ liệu khớp (Parquet)...")
    parquet_path = hf_hub_download(
        repo_id=repo_id,
        filename="data/chunk-000/file-000.parquet",
        repo_type="dataset",
    )
    df = pd.read_parquet(parquet_path)
    print(f"[✓] Đã tải xong dữ liệu động học: {len(df)} timesteps")

    # 2. Tải video camera góc cao (cam_high - tương đương camera ngực OpenArm, ~201 MB)
    print("[2/3] Đang tải video camera góc cao (cam_high, ~201 MB)...")
    video_path = hf_hub_download(
        repo_id=repo_id,
        filename="videos/observation.images.cam_high/chunk-000/file-000.mp4",
        repo_type="dataset",
    )
    print(f"[✓] Đã tải xong video: {video_path}")

    # 3. Mở video bằng OpenCV và trích xuất theo từng Episode
    print(f"[3/3] Đang chuyển đổi sang định dạng HDF5 OpenArm (Tối đa {max_episodes} episodes)...")
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Không thể mở file video: {video_path}")

    total_episodes = min(max_episodes, df["episode_index"].max() + 1)
    current_frame_idx = 0

    for ep_idx in range(total_episodes):
        ep_df = df[df["episode_index"] == ep_idx]
        num_frames = len(ep_df)

        images = []
        for _ in range(num_frames):
            ret, frame = cap.read()
            if not ret:
                print(f"[!] Cảnh báo: Video hết frame sớm tại episode {ep_idx}")
                break
            # OpenCV đọc BGR -> đổi sang RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            images.append(rgb_frame)

        if len(images) != num_frames:
            break

        images = np.array(images, dtype=np.uint8)

        # Lấy 14 chiều gốc của ALOHA:
        # Nhánh tay trái (6 khớp + 1 kẹp) + 1 khớp ảo để đủ 8 DOF của OpenArm
        # Hoặc lấy 7 khớp tay đầu tiên + 1 khớp kẹp
        raw_state = np.array(ep_df["observation.state"].tolist(), dtype=np.float32)
        raw_action = np.array(ep_df["action"].tolist(), dtype=np.float32)

        # Trích xuất 8 chiều cho OpenArm: [6 khớp tay trái, khớp xoay cổ tay, kẹp trái]
        # ALOHA có 7 DOF mỗi tay: waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate, gripper
        # OpenArm có 7 DOF tay + 1 Gripper (8 DOF)
        # Ta lấy 7 DOF tay trái (cột 0..6) và nhân bản khớp cổ tay xoay để khớp với 8 chiều của OpenArm:
        qpos_8d = np.zeros((num_frames, 8), dtype=np.float32)
        qpos_8d[:, :6] = raw_state[:, :6]       # 6 khớp tay
        qpos_8d[:, 6] = raw_state[:, 5]        # Khớp cổ tay xoay
        qpos_8d[:, 7] = raw_state[:, 6]        # Khớp kẹp Gripper

        action_8d = np.zeros((num_frames, 8), dtype=np.float32)
        action_8d[:, :6] = raw_action[:, :6]
        action_8d[:, 6] = raw_action[:, 5]
        action_8d[:, 7] = raw_action[:, 6]

        # Tính qvel xấp xỉ bằng vi phân
        qvel_8d = np.gradient(qpos_8d, axis=0) * 50.0

        # Ghi file HDF5 chuẩn
        out_file = os.path.join(output_dir, f"episode_{ep_idx}.hdf5")
        with h5py.File(out_file, "w") as root:
            root.attrs["sim"] = False
            root.attrs["frequency_hz"] = 50
            root.attrs["robot_type"] = "OpenArm_Converted_From_ALOHA_Towel"
            root.attrs["source_dataset"] = "lerobot/aloha_static_towel"

            obs = root.create_group("observations")
            imgs = obs.create_group("images")
            imgs.create_dataset("chest", data=images, chunks=(1, 480, 640, 3), compression="gzip")
            obs.create_dataset("qpos", data=qpos_8d)
            obs.create_dataset("qvel", data=qvel_8d.astype(np.float32))

            root.create_dataset("action", data=action_8d)

        print(f"  + [✓] Đã tạo episode_{ep_idx}.hdf5: {num_frames} frames, camera ngực RGB [480, 640, 3], 8 khớp")

    cap.release()
    print("=" * 70)
    print(f"[✓] HOÀN TẤT! Đã chuyển đổi thành công {total_episodes} episodes vào: {output_dir}")
    print("=" * 70)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tải và convert dataset gấp khăn từ Hugging Face LeRobot")
    parser.add_argument("--output_dir", type=str, default="dataset/real_towel_folding", help="Thư mục lưu dataset HDF5")
    parser.add_argument("--episodes", type=int, default=5, help="Số episodes muốn tải về convert (mặc định 5 episodes để test nhanh, tối đa 50)")
    args = parser.parse_args()

    download_and_convert(args.output_dir, args.episodes)
