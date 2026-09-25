#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
100Hz High-Precision Joint State Continuous Exporter & UDP Streamer.
Logs 16-axis positions, velocities, torques, and temperatures into HDF5 (.hdf5)
compliant with robotic imitation learning (ALOHA, ACT, Robomimic).
Also writes companion CSV and broadcasts real-time UDP packets.
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

JOINT_NAMES = [
    "left_j1", "left_j2", "left_j3", "left_j4",
    "left_j5", "left_j6", "left_j7", "left_gripper",
    "right_j1", "right_j2", "right_j3", "right_j4",
    "right_j5", "right_j6", "right_j7", "right_gripper",
]


class JointStateExporter100Hz:
    """
    100Hz Joint State Exporter into HDF5 (.hdf5) and CSV format:
    - Creates HDF5 datasets:
        /observations/qpos          [T, 16] float32
        /observations/qvel          [T, 16] float32
        /observations/effort        [T, 16] float32
        /observations/temperatures  [T, 16] float32
        /action                     [T, 16] float32
        /timestamp                  [T]     float64
        /timestamp_ns               [T]     int64
        /rel_time_s                 [T]     float32
    - Precision loop running at 100 Hz (10 ms interval via perf_counter)
    - Broadcasts real-time UDP stream on port 9871 for ROS 2 / external consumers
    """

    def __init__(self, server, export_dir: str = "exports", udp_port: int = 9871):
        self.server = server
        self.export_dir = os.path.abspath(export_dir)
        os.makedirs(self.export_dir, exist_ok=True)
        self.udp_port = udp_port
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
                self.h5_datasets = {
                    "qpos": obs_grp.create_dataset(
                        "qpos", shape=(0, 16), maxshape=(None, 16), chunks=(100, 16), dtype=np.float32
                    ),
                    "qvel": obs_grp.create_dataset(
                        "qvel", shape=(0, 16), maxshape=(None, 16), chunks=(100, 16), dtype=np.float32
                    ),
                    "effort": obs_grp.create_dataset(
                        "effort", shape=(0, 16), maxshape=(None, 16), chunks=(100, 16), dtype=np.float32
                    ),
                    "temperatures": obs_grp.create_dataset(
                        "temperatures", shape=(0, 16), maxshape=(None, 16), chunks=(100, 16), dtype=np.float32
                    ),
                    "action": self.h5_file.create_dataset(
                        "action", shape=(0, 16), maxshape=(None, 16), chunks=(100, 16), dtype=np.float32
                    ),
                    "timestamp": self.h5_file.create_dataset(
                        "timestamp", shape=(0,), maxshape=(None,), chunks=(100,), dtype=np.float64
                    ),
                    "timestamp_ns": self.h5_file.create_dataset(
                        "timestamp_ns", shape=(0,), maxshape=(None,), chunks=(100,), dtype=np.int64
                    ),
                    "rel_time_s": self.h5_file.create_dataset(
                        "rel_time_s", shape=(0,), maxshape=(None,), chunks=(100,), dtype=np.float32
                    ),
                }
                # Attributes
                is_sim = bool(getattr(self.server, "mode", "sim") == "sim")
                self.h5_file.attrs["sim"] = is_sim
                self.h5_file.attrs["frequency_hz"] = 100
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
            print(f"[Record 100Hz] 🔴 Bắt đầu Record dữ liệu HDF5 (.hdf5): {self.file_path} (UDP: {self.udp_port})")

    def _clear_buffers(self):
        self._buf_qpos.clear()
        self._buf_qvel.clear()
        self._buf_effort.clear()
        self._buf_temps.clear()
        self._buf_action.clear()
        self._buf_time.clear()
        self._buf_time_ns.clear()
        self._buf_rel_time.clear()

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
                print(f"[Record 100Hz] ⏹ Đã dừng Record và lưu file HDF5 ({self.samples} mẫu): {self.file_path}")
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
            "sample_rate_hz": round(self.hz, 1) if self.active else 100.0,
            "samples": self.samples,
            "samples_recorded": self.samples,
            "duration_s": dur,
            "elapsed_sec": dur,
            "file_name": curr_name,
            "filepath": curr_name,
            "file_path": target_path or "",
            "file_size_kb": file_size_kb,
            "format": "HDF5 (.hdf5)",
            "udp_port": self.udp_port
        }

    def loop(self):
        """High-precision 100Hz sampling loop using high-resolution monotonic timer."""
        self.running = True
        interval = 0.010  # 10ms = 100 Hz
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
                        if mid == 8:
                            raw_ratio = max(0.0, min(1.0, abs(m.q) / 1.20))
                            ratio = (1.0 - raw_ratio) if getattr(m, 'invert', False) else raw_ratio
                            pos = ratio * 0.043
                            act = getattr(m, 'q_target', pos)
                        elif mid == 16:
                            raw_ratio = max(0.0, min(1.0, abs(m.q) / 1.20))
                            ratio = raw_ratio if getattr(m, 'invert', False) else (1.0 - raw_ratio)
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

            # 1. Append to in-memory buffers for HDF5 batch write
            with self.lock:
                if self.active:
                    self._buf_qpos.append(positions)
                    self._buf_qvel.append(velocities)
                    self._buf_effort.append(efforts)
                    self._buf_temps.append(temps)
                    self._buf_action.append(actions)
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

                    # Batch flush to HDF5 & disk flush every 50 samples (0.5s)
                    if len(self._buf_qpos) >= 50 or (cur_time - self.last_flush >= 1.0):
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

