#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
Unit tests for the AI Model Inference Engine (ACT Policy & Future Action Trajectory).
"""

import time
import numpy as np
import pytest

from sim.model_inference_engine import (
    ModelInferenceEngine,
    SyntheticTrajectoryGenerator,
    TrajectorySmoother,
)


def test_trajectory_smoother_bounds():
    smoother = TrajectorySmoother(vel_scale=1.0, dt=0.02)
    q0 = np.zeros(16, dtype=np.float32)
    smoother.reset(q0)

    # Sudden jump step of 2.0 rad on all joints
    target = np.full(16, 2.0, dtype=np.float32)
    q_next = smoother.step(target)

    # Output must not jump straight to 2.0 rad
    max_step = np.max(np.abs(q_next - q0))
    assert max_step < 0.25, f"Step change too large: {max_step} rad"


def test_synthetic_trajectory_generator_continuity():
    gen = SyntheticTrajectoryGenerator(chunk_size=50)
    current_qpos = np.array([0.1 * i for i in range(16)], dtype=np.float32)

    horizon = gen.generate(current_qpos)
    assert horizon.shape == (50, 16), f"Unexpected horizon shape: {horizon.shape}"

    # First point should start smoothly near current_qpos
    first_diff = np.max(np.abs(horizon[0] - current_qpos))
    assert first_diff < 0.05, f"Discontinuity at start: {first_diff} rad"


def test_model_inference_engine_lifecycle():
    engine = ModelInferenceEngine(server=None)
    assert engine.target_hz == 50.0

    # 1. Load model (should succeed in preview/mock or pytorch mode)
    load_res = engine.load_model("checkpoints/act_deploy_weights.pth")
    assert load_res["success"] is True
    assert engine.is_loaded is True

    # 2. Single step inference in shadow mode (default)
    step_res = engine.single_step(control_mode="shadow")
    assert step_res["success"] is True
    assert len(step_res["future_actions"]) == 50
    assert len(step_res["future_actions"][0]) == 16

    # 3. Start background inference loop in shadow mode
    start_res = engine.start_inference(control_mode="shadow", vel_scale=1.0)
    assert start_res["success"] is True
    assert engine.running is True
    assert engine.control_mode == "shadow"

    # Allow background loop to process at least 2 ticks
    time.sleep(0.08)
    status = engine.get_status()
    assert status["running"] is True
    assert status["mode"] == "shadow"
    assert status["step_count"] > 0
    assert status["has_future_actions"] is True

    # 4. Stop inference
    stop_res = engine.stop_inference()
    assert stop_res["success"] is True
    assert engine.running is False


def test_model_inference_engine_shadow_mode_zero_motion():
    """Verify that shadow mode NEVER modifies joint target angles or dispatches commands."""
    class DummyMotor:
        def __init__(self, q=0.5):
            self.q = q
            self.q_target = q
            self.q_cmd = q
            self.joint_idx = 1
            self.last_update = time.time()

    class DummyHW:
        def __init__(self):
            import threading
            self.lock = threading.Lock()

    class DummyServer:
        def __init__(self):
            self.mode = "sim"
            self.hw = DummyHW()
            self.motors = {i: DummyMotor(0.5) for i in range(1, 17)}
            self.dispatched = []

        def _set_single_joint_target(self, motor_id, pos):
            self.dispatched.append((motor_id, pos))

    server = DummyServer()
    engine = ModelInferenceEngine(server=server)
    engine.load_model()

    # In shadow mode, run single step
    res = engine.single_step(control_mode="shadow")
    assert res["success"] is True
    assert len(res["future_actions"]) == 50

    # Verify all motors still have q_target == 0.5 and q == 0.5
    for m in server.motors.values():
        assert m.q_target == 0.5
        assert m.q == 0.5
    assert len(server.dispatched) == 0
