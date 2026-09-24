#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate Mock Dataset for ACT Model Smoke-Testing
Sinh ra 3 tập dữ liệu giả lập (mock_episodes) chuẩn định dạng HDF5 để kiểm tra toàn bộ pipeline.
"""

import os
import h5py
import numpy as np

OUTPUT_DIR = "dataset/mock_episodes"
os.makedirs(OUTPUT_DIR, exist_ok=True)

NUM_EPISODES = 3
TIMESTEPS_PER_EPISODE = 200  # 4 giây ở tần số 50 Hz

print(f"[*] Đang sinh {NUM_EPISODES} file Mock HDF5 vào thư mục: {OUTPUT_DIR} ...")

for ep in range(NUM_EPISODES):
    filepath = os.path.join(OUTPUT_DIR, f"episode_{ep}.hdf5")
    
    # 1. Sinh quỹ đạo góc 8 khớp mượt mà (dạng sóng sin mô phỏng hành vi gắp)
    t = np.linspace(0, np.pi, TIMESTEPS_PER_EPISODE)
    
    # 7 khớp tay + 1 khớp kẹp
    qpos = np.zeros((TIMESTEPS_PER_EPISODE, 8), dtype=np.float32)
    qpos[:, 0] = 0.2 * np.sin(t)          # Khớp vai 1 (xoay ngang)
    qpos[:, 1] = -0.4 * np.sin(t)         # Khớp vai 2 (hạ tay)
    qpos[:, 2] = 0.1 * np.cos(t)          # Bắp tay 3
    qpos[:, 3] = 0.5 * np.sin(t)          # Khuỷu tay 4
    qpos[:, 4] = 0.0                      # Cổ tay 5
    qpos[:, 5] = -0.2 * np.sin(t)         # Cổ tay 6
    qpos[:, 6] = 0.0                      # Cổ tay 7
    # Kẹp mở (1.2 rad) ở nửa đầu, đóng (0.0 rad) ở nửa sau
    qpos[:100, 7] = 1.2
    qpos[100:, 7] = 0.02
    
    # Vận tốc = đạo hàm của góc
    qvel = np.gradient(qpos, axis=0) * 50.0  # rad/s ở 50Hz
    
    # Action = góc ở bước tiếp theo
    action = np.roll(qpos, -1, axis=0)
    action[-1] = qpos[-1]

    # 2. Sinh ảnh giả lập Camera ngực [T, 480, 640, 3]
    # Tạo mặt bàn màu xám và một khối màu đỏ di chuyển
    print(f"  + Tạo ảnh camera ngực cho episode_{ep}...")
    images = np.full((TIMESTEPS_PER_EPISODE, 480, 640, 3), fill_value=40, dtype=np.uint8)
    
    for i in range(TIMESTEPS_PER_EPISODE):
        # Vẽ mặt bàn màu xanh xám
        images[i, 200:, 100:540] = [60, 70, 80]
        # Vẽ một vật thể màu đỏ (khối lập phương) trên bàn
        pos_x = int(300 + 40 * np.sin(t[i]))
        pos_y = int(350 + 20 * np.cos(t[i]))
        images[i, pos_y-20:pos_y+20, pos_x-20:pos_x+20] = [220, 30, 30] # Màu đỏ RGB

    # 3. Ghi vào file HDF5
    with h5py.File(filepath, "w") as root:
        root.attrs["sim"] = True
        root.attrs["frequency_hz"] = 50
        root.attrs["robot_type"] = "OpenArm_7DOF_Gripper"

        obs = root.create_group("observations")
        img_grp = obs.create_group("images")
        
        # Lưu ảnh với chunks và nén gzip
        img_grp.create_dataset("chest", data=images, chunks=(1, 480, 640, 3), compression="gzip")
        obs.create_dataset("qpos", data=qpos)
        obs.create_dataset("qvel", data=qvel)
        root.create_dataset("action", data=action)

    print(f"[✓] Đã tạo thành công: {filepath} ({os.path.getsize(filepath) / (1024*1024):.1f} MB)")

print("\n[✓] HOÀN TẤT SINH DỮ LIỆU GIẢ LẬP!")
