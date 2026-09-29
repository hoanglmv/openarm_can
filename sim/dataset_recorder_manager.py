#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
Start/stop sim/data_recorder.py (ACT RGB-D HDF5 episode recorder) from the web dashboard.

Each recording runs data_recorder.py as a ROS 2 subprocess. Stopping sends SIGINT so the
recorder finalizes the episode (<episode>.partial.hdf5 -> <episode>.hdf5) in output_dir.
"""

import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from typing import Callable, Optional

_SAVED_RE = re.compile(r"\[Recorder\] Saved (\d+) samples to (\S+)")
_PROGRESS_RE = re.compile(r"Recorded (\d+) samples")
_NO_SAMPLES_RE = re.compile(r"\[Recorder\] No valid samples")


class DatasetRecorderManager:
    """Manage a single data_recorder.py subprocess and expose its status to the UI."""

    def __init__(self, output_dir: str, on_finished: Optional[Callable[[dict], None]] = None):
        self.output_dir = output_dir
        self.on_finished = on_finished
        self.script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data_recorder.py")
        self.lock = threading.Lock()
        self.process: Optional[subprocess.Popen] = None
        self.episode: Optional[str] = None
        self.start_time = 0.0
        self.samples = 0
        self.last_file: Optional[str] = None
        self.last_samples = 0
        self.last_error: Optional[str] = None

    @property
    def active(self) -> bool:
        with self.lock:
            return self.process is not None and self.process.poll() is None

    def _python(self) -> str:
        ros_python = os.environ.get("OPENARM_CAMERA_PYTHON")
        if ros_python and os.path.exists(ros_python):
            return ros_python
        return sys.executable or shutil.which("python3") or "/usr/bin/python3"

    def start(self) -> dict:
        """Launch a new episode recording. Returns {"ok": bool, "message": str}."""
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return {"ok": False, "message": f"Đang ghi episode {self.episode}"}

            os.makedirs(self.output_dir, exist_ok=True)
            episode = datetime.now().strftime("episode_%Y%m%d_%H%M%S")
            # --max-duration 0: no time limit, record until the user presses stop
            cmd = [
                self._python(), "-u", self.script,
                "--output-dir", self.output_dir,
                "--episode", episode,
                "--max-duration", "0",
            ]
            env = dict(os.environ, PYTHONUNBUFFERED="1")
            try:
                self.process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    env=env,
                )
            except Exception as error:
                self.process = None
                self.last_error = str(error)
                return {"ok": False, "message": f"Không khởi động được data_recorder.py: {error}"}

            self.episode = episode
            self.start_time = time.time()
            self.samples = 0
            self.last_error = None
            process = self.process

        print(f"[Dataset] Bắt đầu ghi episode {episode} -> {self.output_dir}")
        threading.Thread(target=self._reader_loop, args=(process, episode), daemon=True).start()
        return {"ok": True, "message": f"Bắt đầu ghi dataset: {episode}"}

    def stop(self, timeout: float = 10.0) -> dict:
        """Ask the recorder to finalize the episode (SIGINT), kill it if it hangs."""
        with self.lock:
            process = self.process
            if process is None or process.poll() is not None:
                return {"ok": False, "message": "Không có phiên ghi nào đang chạy"}
            process.send_signal(signal.SIGINT)

        def _watchdog():
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                print("[Dataset] data_recorder.py không phản hồi SIGINT, buộc dừng")
                process.kill()

        threading.Thread(target=_watchdog, daemon=True).start()
        return {"ok": True, "message": "Đang dừng và lưu episode..."}

    def _reader_loop(self, process: subprocess.Popen, episode: str):
        saved_file = None
        saved_samples = 0
        no_samples = False
        tail = []
        for line in process.stdout:
            line = line.rstrip()
            if not line:
                continue
            print(f"[data_recorder] {line}")
            tail = (tail + [line])[-5:]
            match = _SAVED_RE.search(line)
            if match:
                saved_samples = int(match.group(1))
                saved_file = match.group(2)
                continue
            match = _PROGRESS_RE.search(line)
            if match:
                with self.lock:
                    self.samples = int(match.group(1))
                continue
            if _NO_SAMPLES_RE.search(line):
                no_samples = True
        returncode = process.wait()

        with self.lock:
            if self.process is process:
                self.process = None
            if saved_file:
                self.last_file = saved_file
                self.last_samples = saved_samples
                self.last_error = None
            elif no_samples:
                self.last_error = "Không nhận được mẫu hợp lệ (kiểm tra camera /camera/act/* và /openarm/joint_states)"
            else:
                self.last_error = tail[-1] if tail else f"data_recorder.py thoát với mã {returncode}"
            result = {
                "episode": episode,
                "file": saved_file,
                "samples": saved_samples,
                "error": None if saved_file else self.last_error,
            }

        if self.on_finished:
            try:
                self.on_finished(result)
            except Exception as error:
                print(f"[Dataset] on_finished callback error: {error}")

    def get_stats(self) -> dict:
        with self.lock:
            recording = self.process is not None and self.process.poll() is None
            return {
                "recording": recording,
                "episode": self.episode if recording else None,
                "elapsed_sec": (time.time() - self.start_time) if recording else 0.0,
                "samples": self.samples if recording else 0,
                "output_dir": os.path.basename(self.output_dir.rstrip(os.sep)),
                "last_file": os.path.basename(self.last_file) if self.last_file else None,
                "last_samples": self.last_samples,
                "last_error": self.last_error,
            }
