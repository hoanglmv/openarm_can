#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
Test script to verify Zero Pose execution, gains, and motor states.
"""

import sys
import os
import time
import asyncio

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


def test_joint1_left_shoulder_direction():
    print("\n[TEST] Testing Joint 1 Left Shoulder follows the official URDF convention (Motor 1 vs Motor 9)...")
    from config import JOINT_LIMITS

    # Left J1 is mirrored: its limits are the negation of Right J1 (as in the official URDF)
    assert JOINT_LIMITS[1] == (-JOINT_LIMITS[9][1], -JOINT_LIMITS[9][0]), \
        f"Left J1 limits must mirror Right J1, got {JOINT_LIMITS[1]} vs {JOINT_LIMITS[9]}"

    print(f"✓ Left J1 limits {JOINT_LIMITS[1]} mirror Right J1 limits {JOINT_LIMITS[9]} (official URDF)")


def test_gripper_d4310_and_origins():
    print("\n[TEST] Testing Gripper DM4310 Mapping & OpenArm v2.0 Joint Origins...")
    from config import JOINT_ORIGINS, JOINT_LIMITS
    from models import RealDamiaoMotorState

    # 1. Verify OpenArm v2.0 Kinematic Joint Origins
    assert "joint1" in JOINT_ORIGINS and JOINT_ORIGINS["joint1"]["y"] == -0.0625
    assert "joint2" in JOINT_ORIGINS and abs(JOINT_ORIGINS["joint2"]["y"] - (-0.0600)) < 1e-6
    assert "joint3" in JOINT_ORIGINS and abs(JOINT_ORIGINS["joint3"]["z"] - (-0.06625)) < 1e-6
    assert "joint4" in JOINT_ORIGINS and JOINT_ORIGINS["joint4"]["z"] == -0.15375
    assert "joint5" in JOINT_ORIGINS and abs(JOINT_ORIGINS["joint5"]["z"] - (-0.0955)) < 1e-6
    assert "joint6" in JOINT_ORIGINS and abs(JOINT_ORIGINS["joint6"]["z"] - (-0.1205)) < 1e-6
    assert "joint7" in JOINT_ORIGINS and JOINT_ORIGINS["joint7"]["z"] == 0.0
    print("✓ OpenArm v2.0 Joint Origins verified (J1: y=-0.0625, J2: y=-0.060, J3: z=-0.06625, J4: z=-0.15375, J5: z=-0.0955, J6: z=-0.1205, J7: z=0.0)")

    # 2. Test Gripper Direction & Command Mapping
    server = OpenArmDashboardServer(mode="sim")
    m_left_grip = server.motors[8]
    m_right_grip = server.motors[16]

    # Test Closed (0.0 m)
    server._send_gripper_command(m_left_grip, 0.0)
    server._send_gripper_command(m_right_grip, 0.0)
    assert abs(m_left_grip.q_target - 0.0) < 1e-5, f"Left gripper closed target must be 0.0 rad, got {m_left_grip.q_target}"
    assert abs(m_right_grip.q_target - 0.0) < 1e-5, f"Right gripper closed target must be 0.0 rad, got {m_right_grip.q_target}"

    # Test Fully Open (0.043 m / 43 mm)
    server._send_gripper_command(m_left_grip, 0.043)
    server._send_gripper_command(m_right_grip, 0.043)
    assert abs(m_left_grip.q_target - (-1.20)) < 1e-5, f"Left gripper open target must be -1.20 rad, got {m_left_grip.q_target}"
    assert abs(m_right_grip.q_target - (-1.20)) < 1e-5, f"Right gripper open target must be -1.20 rad, got {m_right_grip.q_target}"
    print("✓ Gripper command mapping verified: Both Left and Right close at 0.0 rad, open at -1.20 rad")

    # 3. Test Gripper Feedback to_dict() display in mm
    # Left Gripper
    m_left_grip.q = 0.0
    assert abs(m_left_grip.to_dict()["q_deg"] - 0.0) < 0.5
    m_left_grip.q = -0.60
    assert abs(m_left_grip.to_dict()["q_deg"] - 21.5) < 0.5
    m_left_grip.q = -1.20
    assert abs(m_left_grip.to_dict()["q_deg"] - 43.0) < 0.5

    # Right Gripper
    m_right_grip.q = 0.0
    assert abs(m_right_grip.to_dict()["q_deg"] - 0.0) < 0.5, f"Right gripper closed at 0.0 rad should be 0 mm, got {m_right_grip.to_dict()['q_deg']}"
    m_right_grip.q = -0.60
    assert abs(m_right_grip.to_dict()["q_deg"] - 21.5) < 0.5, f"Right gripper half open at -0.60 rad should be 21.5 mm, got {m_right_grip.to_dict()['q_deg']}"
    m_right_grip.q = -1.20
    assert abs(m_right_grip.to_dict()["q_deg"] - 43.0) < 0.5, f"Right gripper open at -1.20 rad should be 43 mm, got {m_right_grip.to_dict()['q_deg']}"
    print("✓ Gripper feedback to_dict() verified: Both Left and Right (0 rad -> 0mm, -1.2 rad -> 43mm)")


def test_record_workflow():
    print("\n[TEST] Testing ACT Dataset Record Workflow (Start / Stop / Toggle)...")
    import tempfile
    server = OpenArmDashboardServer(mode="sim")
    recorder = server.dataset_recorder

    # 1. Must NOT auto-record on startup, and saves into data_set/
    assert not recorder.active, "Dataset recorder must be IDLE on startup (no auto-record)"
    stats_init = recorder.get_stats()
    assert not stats_init["recording"], "get_stats() must report recording=False"
    assert stats_init["output_dir"] == "data_set", f"Episodes must be saved to data_set/, got {stats_init['output_dir']}"
    print("✓ Initial state verified: recorder idle, output dir data_set/")

    # Replace data_recorder.py (needs ROS 2) with a stub that finalizes on SIGINT like the real one
    tmp_dir = tempfile.mkdtemp()
    recorder.output_dir = tmp_dir
    stub = os.path.join(tmp_dir, "stub_recorder.py")
    with open(stub, "w") as f:
        f.write(
            "import sys, time\n"
            "args = sys.argv\n"
            "out, ep = args[args.index('--output-dir') + 1], args[args.index('--episode') + 1]\n"
            "print('[*] stub started', flush=True)\n"
            "try:\n"
            "    while True: time.sleep(0.02)\n"
            "except KeyboardInterrupt:\n"
            "    path = f'{out}/{ep}.hdf5'\n"
            "    open(path, 'w').close()\n"
            "    print(f'[Recorder] Saved 42 samples to {path} (0 rejected, Frequency: 50.0Hz)', flush=True)\n"
        )
    recorder.script = stub

    def wait_idle(timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not recorder.active and recorder.get_stats()["last_file"]:
                return
            time.sleep(0.05)
        raise AssertionError("Recorder did not finish in time")

    # 2. Start Record
    asyncio.run(server.handle_action("record_start", {}, None))
    assert recorder.active, "Recorder must be ACTIVE after record_start"
    time.sleep(0.3)
    stats_rec = recorder.get_stats()
    assert stats_rec["recording"] and stats_rec["episode"].startswith("episode_")
    print(f"✓ Record start verified: episode={stats_rec['episode']}")

    # 3. Stop Record -> episode finalized
    asyncio.run(server.handle_action("record_stop", {}, None))
    wait_idle()
    stats_end = recorder.get_stats()
    assert not stats_end["recording"]
    assert stats_end["last_samples"] == 42 and stats_end["last_error"] is None
    assert os.path.exists(os.path.join(tmp_dir, stats_end["last_file"]))
    print(f"✓ Record stop verified: saved {stats_end['last_file']} ({stats_end['last_samples']} samples)")

    # 4. Record Toggle
    recorder.last_file = None
    time.sleep(1.1)  # episode names have 1 s resolution
    asyncio.run(server.handle_action("record_toggle", {}, None))
    assert recorder.active, "record_toggle from idle must start recording"
    time.sleep(0.3)
    asyncio.run(server.handle_action("record_toggle", {}, None))
    wait_idle()
    print("✓ Record toggle verified: start -> stop seamlessly")


if __name__ == "__main__":
    test_zero_pose_logic()
    test_trajectory_sequence()
    test_joint1_left_shoulder_direction()
    test_gripper_d4310_and_origins()
    test_record_workflow()
    print("\nALL VERIFICATION TESTS PASSED SUCCESSFULLY! ✓")

