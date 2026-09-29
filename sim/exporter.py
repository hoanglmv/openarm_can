#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
Standardized Multimodal Dataset Exporter & UDP Streamer (ACT / ALOHA format).
Logs 16-axis positions, velocities, torques, temperatures, AND synchronized
camera RGB-D frames (/observations/images/chest_rgb, chest_depth) into HDF5 (.hdf5)
compliant with robotic imitation learning (ALOHA, ACT, Robomimic) at 50 Hz.
Also writes companion CSV and broadcasts real-time UDP packets (port 9871).
"""

import json
import os
import socket
import threading
import time
from datetime import datetime
from typing import Optional, List

try:
    import h5py
    import numpy as np
    HAS_H5PY = True
except ImportError:
    HAS_H5PY = False

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    from .config import RECORDER_ALIGNMENT_HZ
except ImportError:
    try:
        from config import RECORDER_ALIGNMENT_HZ
    except ImportError:
        RECORDER_ALIGNMENT_HZ = 50.0

JOINT_NAMES = [
    "left_j1", "left_j2", "left_j3", "left_j4",
    "left_j5", "left_j6", "left_j7", "left_gripper",
    "right_j1", "right_j2", "right_j3", "right_j4",
    "right_j5", "right_j6", "right_j7", "right_gripper",
]


class JointStateExporter100Hz:
    """
    Standardized Multimodal Dataset Exporter (ACT / ALOHA HDF5 format):
    - Creates HDF5 datasets:
        /observations/images/chest_rgb    [T, 480, 640, 3] uint8
        /observations/images/chest_depth  [T, 480, 640]    uint16
        /observations/qpos                [T, 16]          float32
        /observations/qvel                [T, 16]          float32
        /observations/effort              [T, 16]          float32
        /observations/temperatures        [T, 16]          float32
        /action                           [T, 16]          float32
        /timestamp                        [T]              float64
        /timestamp_ns                     [T]              int64
        /rel_time_s                       [T]              float32
    - Sampling loop running at 50 Hz (20 ms interval, aligned with RECORDER_ALIGNMENT_HZ)
    - Broadcasts real-time UDP stream on port 9871 for ROS 2 / external consumers
    """

    def __init__(self, server, export_dir: str = "exports", udp_port: int = 9871, frequency_hz: float = RECORDER_ALIGNMENT_HZ):
        self.server = server
        self.export_dir = os.path.abspath(export_dir)
        os.makedirs(self.export_dir, exist_ok=True)
        self.udp_port = udp_port
        self.frequency_hz = frequency_hz
        self.running = False
        self.active = False

        # File handles & paths
        self.h5_file = None
        self.h5_datasets = {}
        self.csv_file = None
        self.file_path: Optional[str] = None       # Primary file (.hdf5)
        self.file_name: Optional[str] = None
        self.csv_path: Optional[str] = None

        # Buffers for high-speed batch append to HDF5
        self._buf_qpos: List[List[float]] = []
        self._buf_qvel: List[List[float]] = []
        self._buf_effort: List[List[float]] = []
        self._buf_temps: List[List[float]] = []
        self._buf_action: List[List[float]] = []
        self._buf_time: List[float] = []
        self._buf_time_ns: List[int] = []
        self._buf_rel_time: List[float] = []
        self._buf_rgb: List[np.ndarray] = []
        self._buf_depth: List[np.ndarray] = []

        self.samples = 0
        self.start_time = 0.0
        self.last_flush = 0.0
        self.hz = 0.0
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            pass

    def start_session(self, tag: str = "record"):
        """Begin a new HDF5 & CSV recording session with metadata."""
        with self.lock:
            if self.active and (self.h5_file or self.csv_file):
                self._close_session_locked()

            now = datetime.now()
            now_str = now.strftime("%Y%m%d_%H%M%S")

            # 1. HDF5 Primary File
            self.file_name = f"joint_states_{tag}_{now_str}.hdf5"
            self.file_path = os.path.join(self.export_dir, self.file_name)

            if HAS_H5PY:
                self.h5_file = h5py.File(self.file_path, "w", libver="latest")
                obs_grp = self.h5_file.create_group("observations")
                images_grp = obs_grp.create_group("images")
                self.h5_datasets = {
                    "qpos": obs_grp.create_dataset(
                        "qpos", shape=(0, 16), maxshape=(None, 16), chunks=(50, 16), dtype=np.float32
                    ),
                    "qvel": obs_grp.create_dataset(
                        "qvel", shape=(0, 16), maxshape=(None, 16), chunks=(50, 16), dtype=np.float32
                    ),
                    "effort": obs_grp.create_dataset(
                        "effort", shape=(0, 16), maxshape=(None, 16), chunks=(50, 16), dtype=np.float32
                    ),
                    "temperatures": obs_grp.create_dataset(
                        "temperatures", shape=(0, 16), maxshape=(None, 16), chunks=(50, 16), dtype=np.float32
                    ),
                    "chest_rgb": images_grp.create_dataset(
                        "chest_rgb", shape=(0, 480, 640, 3), maxshape=(None, 480, 640, 3), chunks=(1, 480, 640, 3), dtype=np.uint8
                    ),
                    "chest_depth": images_grp.create_dataset(
                        "chest_depth", shape=(0, 480, 640), maxshape=(None, 480, 640), chunks=(1, 480, 640), dtype=np.uint16
                    ),
                    "action": self.h5_file.create_dataset(
                        "action", shape=(0, 16), maxshape=(None, 16), chunks=(50, 16), dtype=np.float32
                    ),
                    "timestamp": self.h5_file.create_dataset(
                        "timestamp", shape=(0,), maxshape=(None,), chunks=(50,), dtype=np.float64
                    ),
                    "timestamp_ns": self.h5_file.create_dataset(
                        "timestamp_ns", shape=(0,), maxshape=(None,), chunks=(50,), dtype=np.int64
                    ),
                    "rel_time_s": self.h5_file.create_dataset(
                        "rel_time_s", shape=(0,), maxshape=(None,), chunks=(50,), dtype=np.float32
                    ),
                }
                # Attributes compliant with ACT / ALOHA imitation learning datasets
                is_sim = bool(getattr(self.server, "mode", "sim") == "sim")
                self.h5_file.attrs["sim"] = is_sim
                self.h5_file.attrs["frequency_hz"] = self.frequency_hz
                self.h5_file.attrs["robot_type"] = "OpenArm_Bimanual_16DOF"
                self.h5_file.attrs["num_joints"] = 16
                self.h5_file.attrs["created_at"] = now.isoformat()
                self.h5_file.attrs["joint_names"] = JOINT_NAMES
                self.h5_file.flush()

            # 2. Companion CSV File
            self.csv_name = f"joint_states_{tag}_{now_str}.csv"
            self.csv_path = os.path.join(self.export_dir, self.csv_name)
            self.csv_file = open(self.csv_path, "w", buffering=1024 * 64, newline="")
            header = ["timestamp", "rel_time_s"]
            for i in range(1, 17):
                header.append(f"q_{i}")
            for i in range(1, 17):
                header.append(f"dq_{i}")
            for i in range(1, 17):
                header.append(f"tau_{i}")
            for i in range(1, 17):
                header.append(f"t_mos_{i}")
            self.csv_file.write(",".join(header) + "\n")
            self.csv_file.flush()

            self._clear_buffers()
            self.samples = 0
            self.start_time = time.time()
            self.last_flush = time.time()
            self.active = True
            print(f"[Record {self.frequency_hz:g}Hz] 🔴 Bắt đầu Record Multimodal HDF5: {self.file_path} (Images: /observations/images/chest_rgb, chest_depth)")

    def _clear_buffers(self):
        self._buf_qpos.clear()
        self._buf_qvel.clear()
        self._buf_effort.clear()
        self._buf_temps.clear()
        self._buf_action.clear()
        self._buf_time.clear()
        self._buf_time_ns.clear()
        self._buf_rel_time.clear()
        self._buf_rgb.clear()
        self._buf_depth.clear()

    def _flush_h5_buffer_locked(self):
        if not HAS_H5PY or not self.h5_file or not self._buf_qpos:
            return
        n = len(self._buf_qpos)
        old_size = self.h5_datasets["qpos"].shape[0]
        new_size = old_size + n

        for key, buf in [
            ("qpos", self._buf_qpos),
            ("qvel", self._buf_qvel),
            ("effort", self._buf_effort),
            ("temperatures", self._buf_temps),
            ("action", self._buf_action),
        ]:
            ds = self.h5_datasets[key]
            ds.resize(new_size, axis=0)
            ds[old_size:new_size] = np.array(buf, dtype=np.float32)

        if "chest_rgb" in self.h5_datasets and self._buf_rgb:
            ds_rgb = self.h5_datasets["chest_rgb"]
            ds_rgb.resize(new_size, axis=0)
            ds_rgb[old_size:new_size] = np.stack(self._buf_rgb, axis=0)

        if "chest_depth" in self.h5_datasets and self._buf_depth:
            ds_depth = self.h5_datasets["chest_depth"]
            ds_depth.resize(new_size, axis=0)
            ds_depth[old_size:new_size] = np.stack(self._buf_depth, axis=0)

        for key, buf, dtype in [
            ("timestamp", self._buf_time, np.float64),
            ("timestamp_ns", self._buf_time_ns, np.int64),
            ("rel_time_s", self._buf_rel_time, np.float32),
        ]:
            ds = self.h5_datasets[key]
            ds.resize(new_size, axis=0)
            ds[old_size:new_size] = np.array(buf, dtype=dtype)

        self._clear_buffers()
        self.h5_file.flush()

    def _close_session_locked(self):
        # Flush & close HDF5
        if self.h5_file:
            try:
                self._flush_h5_buffer_locked()
                self.h5_file.flush()
                self.h5_file.close()
                print(f"[Record {self.frequency_hz:g}Hz] ⏹ Đã dừng Record và lưu file Multimodal HDF5 ({self.samples} mẫu): {self.file_path}")
            except Exception as e:
                print(f"[Record HDF5 Error]: {e}")
            self.h5_file = None

        # Flush & close CSV
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception as e:
                print(f"[Record CSV Error]: {e}")
            self.csv_file = None

        self._clear_buffers()
        self.active = False

    def close_session(self):
        """Safely close active recording session and flush remaining buffers."""
        with self.lock:
            self._close_session_locked()

    def get_latest_file(self) -> Optional[str]:
        """Return the filepath of the most recent recorded HDF5 (or CSV fallback)."""
        try:
            h5_files = [os.path.join(self.export_dir, f) for f in os.listdir(self.export_dir) if f.endswith('.hdf5')]
            if h5_files:
                h5_files.sort(key=os.path.getmtime, reverse=True)
                return h5_files[0]
            csv_files = [os.path.join(self.export_dir, f) for f in os.listdir(self.export_dir) if f.endswith('.csv')]
            if csv_files:
                csv_files.sort(key=os.path.getmtime, reverse=True)
                return csv_files[0]
        except Exception:
            pass
        return None

    def get_stats(self) -> dict:
        """Return real-time recording session statistics for UI cards."""
        now = time.time()
        dur = round(now - self.start_time, 1) if (self.active and self.start_time > 0) else 0.0
        file_size_kb = 0
        latest_file = self.get_latest_file()
        target_path = self.file_path or latest_file
        if target_path and os.path.exists(target_path):
            try:
                file_size_kb = round(os.path.getsize(target_path) / 1024, 1)
            except Exception:
                pass
        curr_name = self.file_name or (os.path.basename(latest_file) if latest_file else "")
        return {
            "active": self.active,
            "recording": self.active,
            "hz": round(self.hz, 1) if self.active else 0.0,
            "sample_rate_hz": round(self.hz, 1) if self.active else self.frequency_hz,
            "samples": self.samples,
            "samples_recorded": self.samples,
            "duration_s": dur,
            "elapsed_sec": dur,
            "file_name": curr_name,
            "filepath": curr_name,
            "file_path": target_path or "",
            "file_size_kb": file_size_kb,
            "format": "HDF5 Multimodal (.hdf5)",
            "udp_port": self.udp_port
        }

    def loop(self):
        """Standardized Multimodal Dataset sampling loop (ACT / ALOHA format)."""
        self.running = True
        interval = 1.0 / max(1.0, self.frequency_hz)  # 20ms = 50 Hz
        next_tick = time.perf_counter()
        last_t = time.perf_counter()

        while self.running:
            now = time.perf_counter()
            if now < next_tick:
                sleep_s = next_tick - now
                if sleep_s > 0.001:
                    time.sleep(sleep_s)
                continue
            next_tick += interval

            dt = now - last_t
            last_t = now
            if dt > 0:
                inst_hz = 1.0 / dt
                self.hz = self.hz * 0.95 + inst_hz * 0.05

            if not self.active or (not self.h5_file and not self.csv_file):
                continue

            cur_time = time.time()
            rel_t = cur_time - self.start_time

            positions = []
            velocities = []
            efforts = []
            temps = []
            actions = []

            with self.server.hw.lock:
                for mid in range(1, 17):
                    m = self.server.motors.get(mid)
                    if m:
                        if mid in (8, 16):
                            raw_ratio = max(0.0, min(1.0, abs(m.q) / 1.20))
                            ratio = (1.0 - raw_ratio) if getattr(m, 'invert', False) else raw_ratio
                            pos = ratio * 0.043
                            act = getattr(m, 'q_target', pos)
                        else:
                            pos = m.q
                            act = getattr(m, 'q_cmd', m.q)
                        positions.append(pos)
                        velocities.append(m.dq)
                        efforts.append(m.tau)
                        temps.append(m.t_mos)
                        actions.append(act)
                    else:
                        positions.append(0.0)
                        velocities.append(0.0)
                        efforts.append(0.0)
                        temps.append(0.0)
                        actions.append(0.0)

            # Multimodal frame acquisition (50 Hz alignment)
            rgb_f = None
            depth_f = None
            if hasattr(self.server, "inference_engine") and self.server.inference_engine:
                try:
                    rgb_f, depth_f = self.server.inference_engine.get_latest_rgbd()
                except Exception:
                    pass
            elif hasattr(self.server, "latest_rgbd"):
                rgb_f, depth_f = self.server.latest_rgbd

            # Standardized 480x640x3 uint8 RGB
            if rgb_f is not None and isinstance(rgb_f, np.ndarray):
                if rgb_f.shape != (480, 640, 3):
                    try:
                        import cv2
                        rgb_frame = cv2.resize(rgb_f, (640, 480), interpolation=cv2.INTER_LINEAR)
                    except Exception:
                        rgb_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                else:
                    rgb_frame = rgb_f.astype(np.uint8)
            else:
                rgb_frame = np.zeros((480, 640, 3), dtype=np.uint8)

            # Standardized 480x640 uint16 Depth
            if depth_f is not None and isinstance(depth_f, np.ndarray):
                if depth_f.shape != (480, 640):
                    try:
                        import cv2
                        depth_frame = cv2.resize(depth_f, (640, 480), interpolation=cv2.INTER_NEAREST)
                    except Exception:
                        depth_frame = np.zeros((480, 640), dtype=np.uint16)
                else:
                    depth_frame = depth_f.astype(np.uint16)
            else:
                depth_frame = np.zeros((480, 640), dtype=np.uint16)

            # 1. Append to in-memory buffers for HDF5 batch write
            with self.lock:
                if self.active:
                    self._buf_qpos.append(positions)
                    self._buf_qvel.append(velocities)
                    self._buf_effort.append(efforts)
                    self._buf_temps.append(temps)
                    self._buf_action.append(actions)
                    self._buf_rgb.append(rgb_frame)
                    self._buf_depth.append(depth_frame)
                    self._buf_time.append(cur_time)
                    self._buf_time_ns.append(int(cur_time * 1e9))
                    self._buf_rel_time.append(float(rel_t))
                    self.samples += 1

                    # Write CSV line
                    if self.csv_file:
                        row = [f"{cur_time:.6f}", f"{rel_t:.3f}"]
                        row.extend(f"{v:.5f}" for v in positions)
                        row.extend(f"{v:.4f}" for v in velocities)
                        row.extend(f"{v:.3f}" for v in efforts)
                        row.extend(f"{v:.1f}" for v in temps)
                        self.csv_file.write(",".join(row) + "\n")

                    # Batch flush to HDF5 & disk flush every 25 samples (0.5s @ 50Hz)
                    if len(self._buf_qpos) >= 25 or (cur_time - self.last_flush >= 1.0):
                        self._flush_h5_buffer_locked()
                        if self.csv_file:
                            self.csv_file.flush()
                        self.last_flush = cur_time

            # 2. Real-time UDP stream (port 9871)
            udp_payload = json.dumps({
                "seq": self.samples,
                "timestamp": round(cur_time, 4),
                "rel_time": round(rel_t, 3),
                "hz": round(self.hz, 1),
                "positions": [round(p, 5) for p in positions],
                "velocities": [round(v, 4) for v in velocities],
                "efforts": [round(e, 3) for e in efforts]
            }).encode('utf-8')
            try:
                self.sock.sendto(udp_payload, ("127.0.0.1", self.udp_port))
            except Exception:
                pass


MultimodalDatasetExporter = JointStateExporter100Hz
