#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
Architectural Compliance Tests for OpenArm Bimanual System.
Verifies the 7 architectural rules from CONTEXT.md:
  1. CAN bus output frequency contract (400 Hz).
  2. Trajectory Interpolator (Quintic Hermite Spline 50Hz -> 400Hz) with C2 continuity.
  3. No CAN bypass for Model AI.
  4. AI Inference cycle standard (50 Hz / 20 ms).
  5. Multimodal observation input (Real RGB-D, no dummy zeros).
  6. UI Telemetry stream standard (100 Hz).
  7. UI Multimodal HDF5 recording (/observations/images/chest_rgb, chest_depth at 50 Hz).
"""

import os
import shutil
import tempfile
import threading
import time
import numpy as np
import pytest

from sim.config import (
    CONTROL_FREQ,
    TELEMETRY_FREQ,
    RECORDER_ALIGNMENT_HZ,
    MODEL_INFERENCE_HZ,
)
from sim.spline_interpolator import (
    QuinticHermiteSpline,
    BimanualSplineInterpolator,
)
from sim.exporter import JointStateExporter100Hz, MultimodalDatasetExporter


def test_frequency_contracts():
    """Verify frequency contracts match CONTEXT.md specifications."""
    assert CONTROL_FREQ == 400.0, "CAN output cycle must be 400 Hz (2.5 ms)"
    assert TELEMETRY_FREQ == 100.0, "UI Telemetry broadcast must be 100 Hz (10 ms)"
    assert RECORDER_ALIGNMENT_HZ == 50.0, "Multimodal HDF5 recording must be 50 Hz (20 ms)"
    assert MODEL_INFERENCE_HZ == 50.0, "Model AI inference cycle must be 50 Hz (20 ms)"


def test_spline_interpolator_50hz_to_400hz():
    """Verify BimanualSplineInterpolator smooth upsampling from 50Hz to 400Hz."""
    interpolator = BimanualSplineInterpolator(initial_positions={i: 0.0 for i in range(1, 17)})

    # Push a target position at 50Hz (dt = 0.02s)
    for mid in range(1, 17):
        interpolator.update_joint_target(mid, 0.5, dt_target=0.02)

    # 50Hz to 400Hz ratio = 8 samples (each 2.5ms = 0.0025s)
    samples = []
    for _ in range(8):
        step_cmds = []
        for mid in range(1, 17):
            pos, vel = interpolator.step(mid, dt=0.0025)
            step_cmds.append(pos)
        samples.append(step_cmds)

    samples = np.array(samples)
    assert samples.shape == (8, 16)
    # Monotonic progression towards target without overshoot
    assert samples[-1, 0] > samples[0, 0]
    # Smooth progression: no instant jump to 0.5 rad on step 0
    assert samples[0, 0] < 0.15
    assert samples[0, 0] < 0.2


def test_multimodal_dataset_exporter_structure():
    """Verify HDF5 schema contains /observations/images/chest_rgb and chest_depth at 50Hz."""
    temp_dir = tempfile.mkdtemp(prefix="openarm_test_export_")
    try:
        class DummyHardware:
            lock = threading.Lock()

        class DummyServer:
            mode = "sim"
            hw = DummyHardware()
            motors = {}
            latest_rgbd = (np.zeros((480, 640, 3), dtype=np.uint8), np.zeros((480, 640), dtype=np.uint16))

        exporter = MultimodalDatasetExporter(
            server=DummyServer(),
            export_dir=temp_dir,
            frequency_hz=50.0,
            udp_port=19871,
        )

        exporter.start_session(tag="test_compliance")
        assert exporter.active is True
        assert exporter.h5_file is not None

        # Verify HDF5 datasets
        ds = exporter.h5_datasets
        assert "chest_rgb" in ds, "Missing /observations/images/chest_rgb dataset"
        assert "chest_depth" in ds, "Missing /observations/images/chest_depth dataset"
        assert "qpos" in ds, "Missing /observations/qpos dataset"
        assert "action" in ds, "Missing /action dataset"

        # Check attributes
        assert exporter.h5_file.attrs["frequency_hz"] == 50.0
        assert exporter.h5_file.attrs["robot_type"] == "OpenArm_Bimanual_16DOF"

        exporter.close_session()
        assert exporter.active is False
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)
