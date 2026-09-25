#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
100Hz High-Precision Joint State Continuous Exporter & UDP Streamer.
Logs 16-axis positions, velocities, torques, and temperatures to CSV.
Broadcasts low-latency JSON packets over UDP for ROS 2 / external telemetry.
"""

import json
import os
import socket
import threading
import time
from datetime import datetime
from typing import Optional


class JointStateExporter100Hz:
    """
    100Hz Joint State Continuous Exporter & Streamer:
    - Automatically activates upon connecting to the physical robot
    - Precision loop running at 100 Hz (10 ms interval via perf_counter)
    - Saves high-precision CSV file to `exports/joint_states_<tag>_<timestamp>.csv`
    - Broadcasts real-time UDP stream on port 9871 for ROS 2 / external consumers
    """

    def __init__(self, server, export_dir: str = "exports", udp_port: int = 9871):
        self.server = server
        self.export_dir = os.path.abspath(export_dir)
        os.makedirs(self.export_dir, exist_ok=True)
        self.udp_port = udp_port
        self.running = False
        self.active = False
        self.file = None
        self.file_path: Optional[str] = None
        self.file_name: Optional[str] = None
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
        """Begin a new CSV recording session with full timestamped header."""
        with self.lock:
            if self.active and self.file:
                self._close_session_locked()

            now = datetime.now()
            now_str = now.strftime("%Y%m%d_%H%M%S")
            self.file_name = f"joint_states_{tag}_{now_str}.csv"
            self.file_path = os.path.join(self.export_dir, self.file_name)
            self.file = open(self.file_path, "w", buffering=1024 * 64, newline="")
            self.samples = 0
            self.start_time = time.time()
            self.last_flush = time.time()
            self.active = True

            # CSV Header: timestamp, rel_time_s, q_1..16, dq_1..16, tau_1..16, t_mos_1..16
            header = ["timestamp", "rel_time_s"]
            for i in range(1, 17):
                header.append(f"q_{i}")
            for i in range(1, 17):
                header.append(f"dq_{i}")
            for i in range(1, 17):
                header.append(f"tau_{i}")
            for i in range(1, 17):
                header.append(f"t_mos_{i}")
            self.file.write(",".join(header) + "\n")
            self.file.flush()
            print(f"[Record 100Hz] 🔴 Bắt đầu Record dữ liệu Joint State: {self.file_path} (UDP: {self.udp_port})")

    def _close_session_locked(self):
        if self.file:
            try:
                self.file.flush()
                self.file.close()
                print(f"[Record 100Hz] ⏹ Đã dừng Record và lưu file ({self.samples} mẫu): {self.file_path}")
            except Exception as e:
                print(f"[Record Error]: {e}")
            self.file = None
        self.active = False

    def close_session(self):
        """Safely close active recording session and flush remaining buffers."""
        with self.lock:
            self._close_session_locked()

    def get_latest_file(self) -> Optional[str]:
        """Return the filepath of the most recent recorded CSV export."""
        try:
            files = [os.path.join(self.export_dir, f) for f in os.listdir(self.export_dir) if f.endswith('.csv')]
            if files:
                files.sort(key=os.path.getmtime, reverse=True)
                return files[0]
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

            if not self.active or not self.file:
                continue

            cur_time = time.time()
            rel_t = cur_time - self.start_time

            positions = []
            velocities = []
            efforts = []
            temps = []

            with self.server.hw.lock:
                for mid in range(1, 17):
                    m = self.server.motors.get(mid)
                    if m:
                        if mid == 8:
                            raw_ratio = max(0.0, min(1.0, abs(m.q) / 1.20))
                            ratio = (1.0 - raw_ratio) if getattr(m, 'invert', False) else raw_ratio
                            pos = ratio * 0.043
                        elif mid == 16:
                            raw_ratio = max(0.0, min(1.0, abs(m.q) / 1.20))
                            ratio = raw_ratio if getattr(m, 'invert', False) else (1.0 - raw_ratio)
                            pos = ratio * 0.043
                        else:
                            pos = m.q
                        positions.append(pos)
                        velocities.append(m.dq)
                        efforts.append(m.tau)
                        temps.append(m.t_mos)
                    else:
                        positions.append(0.0)
                        velocities.append(0.0)
                        efforts.append(0.0)
                        temps.append(0.0)

            # 1. Write CSV line
            row = [f"{cur_time:.6f}", f"{rel_t:.3f}"]
            row.extend(f"{v:.5f}" for v in positions)
            row.extend(f"{v:.4f}" for v in velocities)
            row.extend(f"{v:.3f}" for v in efforts)
            row.extend(f"{v:.1f}" for v in temps)
            with self.lock:
                if self.file:
                    self.file.write(",".join(row) + "\n")
                    self.samples += 1

                    # Periodic disk flush every 1 second (100 samples)
                    if cur_time - self.last_flush >= 1.0:
                        self.file.flush()
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
