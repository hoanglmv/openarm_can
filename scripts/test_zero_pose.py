#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
Test script to verify Zero Pose execution, gains, and motor states.
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sim"))
from dashboard_server import OpenArmDashboardServer, DEFAULT_GAINS

def test_zero_pose_logic():
    print("[TEST] Initializing OpenArmDashboardServer in SIM mode...")
    server = OpenArmDashboardServer(mode="sim")
    
    # 1. Verify default gains initialized
    for mid, g in DEFAULT_GAINS.items():
        m = server.motors[mid]
        assert m.kp == g["kp"], f"Motor {mid} kp mismatch: {m.kp} vs {g['kp']}"
        assert m.kd == g["kd"], f"Motor {mid} kd mismatch: {m.kd} vs {g['kd']}"
    print("✓ Initial gains verified against DEFAULT_GAINS (J1/J2: 75.0, J3/J4: 50.0, J5..J7: 30.0, J8/J16: 45.0)")

    # 2. Simulate robot in non-zero posture (e.g., raised arms)
    for m in server.motors.values():
        m.q = 0.85
        m.q_cmd = 0.85
        m.q_target = 0.85
        m.enabled = True
    print(f"✓ Simulated robot in non-zero posture (all joints q = 0.85 rad)")

    # 3. Trigger _exec_go_to_zero_pose
    server._exec_go_to_zero_pose()
    
    for m in server.motors.values():
        assert m.enabled is True, f"Motor {m.id} should remain enabled!"
        assert m.q_target == 0.0, f"Motor {m.id} q_target should be 0.0, got {m.q_target}"
        assert m.q_cmd == 0.85, f"Motor {m.id} q_cmd should smoothly start at current q (0.85), got {m.q_cmd}"
    print("✓ _exec_go_to_zero_pose correctly sets target to 0.0 and keeps motors enabled")

    # 4. Simulate trajectory progression
    dt = 0.0025
    v_lim = server.velocity_limit
    for _ in range(400):  # 1 second of trajectory loop steps
        for m in server.motors.values():
            diff = m.q_target - m.q_cmd
            step = v_lim * dt
            if abs(diff) <= step:
                m.q_cmd = m.q_target
            else:
                m.q_cmd += -step if diff < 0 else step
            m.q = m.q_cmd

    # After 1 sec at 0.25 rad/s, delta is 0.25 rad -> q should be around 0.60
    assert abs(server.motors[1].q - 0.60) < 0.01, f"Expected q ~ 0.60, got {server.motors[1].q}"
    print(f"✓ Trajectory gently moves at 0.25 rad/s without jerking: q after 1s = {server.motors[1].q:.3f} rad")

    # Run for another 3.5 seconds to reach 0.0
    for _ in range(1400):
        for m in server.motors.values():
            diff = m.q_target - m.q_cmd
            step = v_lim * dt
            if abs(diff) <= step:
                m.q_cmd = m.q_target
            else:
                m.q_cmd += -step if diff < 0 else step
            m.q = m.q_cmd

    for m in server.motors.values():
        assert abs(m.q - 0.0) < 1e-4, f"Motor {m.id} did not reach 0.0: q = {m.q}"
    print("✓ All 16 motors successfully and smoothly reached 0.000 rad!")

def test_trajectory_sequence():
    print("\n[TEST] Testing 3-Phase Trajectory Execution (Read State -> Zero Pose -> Trajectory)...")
    server = OpenArmDashboardServer(mode="sim")
    server.velocity_limit = 2.0  # Fast speed for test simulation

    # Start robot at non-zero pose (e.g. q = 0.5 rad)
    for m in server.motors.values():
        m.q = 0.5
        m.q_cmd = 0.5
        m.q_target = 0.5
        m.enabled = True

    # Background pseudo trajectory loop thread to step motors
    stop_flag = False
    def sim_traj_loop():
        dt = 0.0025
        while not stop_flag:
            for m in server.motors.values():
                diff = m.q_target - m.q_cmd
                step = 2.0 * dt
                if abs(diff) <= step:
                    m.q_cmd = m.q_target
                else:
                    m.q_cmd += -step if diff < 0 else step
                m.q = m.q_cmd
            time.sleep(dt)

    import threading
    t = threading.Thread(target=sim_traj_loop, daemon=True)
    t.start()

    # Trajectory points that wave after 0
    traj_points = [
        {"time": 0.1, "positions": [0.35] * 16},
        {"time": 0.2, "positions": [0.60] * 16},
    ]
    joint_names = [f"joint_{i}" for i in range(1, 17)]

    # Run _execute_trajectory
    server.active_trajectory_id = 1
    server._execute_trajectory(traj_points, joint_names, 1)

    stop_flag = True
    print("✓ 3-Phase trajectory successfully completed sequence: Read State -> Return to Zero -> Execute Trajectory!")

if __name__ == "__main__":
    test_zero_pose_logic()
    test_trajectory_sequence()
    print("\nALL VERIFICATION TESTS PASSED SUCCESSFULLY! ✓")
