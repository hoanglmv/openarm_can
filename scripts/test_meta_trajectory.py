#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Meta Quest VR / ROS 2 Trajectory & Teleop Test Tool for OpenArm Bimanual Robot.

This tool demonstrates and validates motion control for the OpenArm robot:
1. Trajectory Mode: Publishes multi-waypoint JointTrajectory (trajectory_msgs/msg/JointTrajectory)
   to topic `/openarm/joint_trajectory`.
2. Teleop Stream Mode: Streams continuous 50Hz JointState (sensor_msgs/msg/JointState)
   to topic `/openarm/teleop/joint_commands` or `/meta/joint_states` (simulating Meta Quest VR hand/arm tracking).
3. Direct UDP Mode: Sends JSON trajectory / teleop packets directly over UDP to port 9870.

Usage examples:
  # Test multi-point trajectory playback via ROS 2
  python3 scripts/test_meta_trajectory.py --mode trajectory

  # Test continuous real-time teleop streaming (Meta Quest VR simulation)
  python3 scripts/test_meta_trajectory.py --mode teleop --rate 50

  # Test directly via UDP without ROS 2
  python3 scripts/test_meta_trajectory.py --mode udp-trajectory
  python3 scripts/test_meta_trajectory.py --mode udp-teleop
"""

import argparse
import json
import math
import socket
import sys
import time

JOINT_NAMES = [
    "left_j1", "left_j2", "left_j3", "left_j4",
    "left_j5", "left_j6", "left_j7", "left_gripper",
    "right_j1", "right_j2", "right_j3", "right_j4",
    "right_j5", "right_j6", "right_j7", "right_gripper",
]


def generate_sample_trajectory(duration: float = 6.0, steps: int = 24):
    """
    Generate a smooth, coordinated bimanual trajectory:
    - Left Arm waves and flexes elbow
    - Right Arm mirrored waving
    - Grippers open and close synchronously
    """
    points = []
    dt = duration / float(steps)

    for i in range(steps + 1):
        t = i * dt
        phase = 2.0 * math.pi * (t / duration)

        # Gentle smooth wave motion
        left_j1 = 0.35 * math.sin(phase)
        left_j2 = 0.40 * (1.0 - math.cos(phase))
        left_j3 = 0.20 * math.sin(phase)
        left_j4 = -0.50 * (1.0 - math.cos(phase))
        left_j5 = 0.15 * math.sin(phase)
        left_j6 = 0.25 * math.sin(phase * 2.0)
        left_j7 = 0.0
        # Gripper cycles open (0.040m) and closed (0.0m)
        left_grip = 0.020 * (1.0 - math.cos(phase * 2.0))

        # Mirrored right arm
        right_j1 = -0.35 * math.sin(phase)
        right_j2 = -0.40 * (1.0 - math.cos(phase))
        right_j3 = -0.20 * math.sin(phase)
        right_j4 = 0.50 * (1.0 - math.cos(phase))
        right_j5 = -0.15 * math.sin(phase)
        right_j6 = -0.25 * math.sin(phase * 2.0)
        right_j7 = 0.0
        right_grip = left_grip

        positions = [
            left_j1, left_j2, left_j3, left_j4,
            left_j5, left_j6, left_j7, left_grip,
            right_j1, right_j2, right_j3, right_j4,
            right_j5, right_j6, right_j7, right_grip,
        ]

        points.append({
            "time": round(t, 3),
            "positions": [round(p, 4) for p in positions]
        })

    return points


def run_ros2_trajectory(topic: str = "/openarm/joint_trajectory", duration: float = 6.0):
    """Publish a multi-point trajectory to ROS 2 topic."""
    try:
        import rclpy
        from rclpy.node import Node
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
    except ImportError:
        print("[ERROR] ROS 2 Python libraries (rclpy, trajectory_msgs) not found in current environment.")
        print("Please source your ROS 2 environment: source /opt/ros/<distro>/setup.bash")
        sys.exit(1)

    rclpy.init()
    node = Node("meta_trajectory_publisher")
    pub = node.create_publisher(JointTrajectory, topic, 10)
    print(f"[*] Waiting for subscribers on ROS 2 topic: {topic}...")
    t_start = time.time()
    while pub.get_subscription_count() == 0 and (time.time() - t_start < 2.5):
        rclpy.spin_once(node, timeout_sec=0.1)

    sub_count = pub.get_subscription_count()
    print(f"[*] Found {sub_count} subscriber(s). Publishing trajectory...")

    traj_msg = JointTrajectory()
    traj_msg.header.stamp = node.get_clock().now().to_msg()
    traj_msg.joint_names = JOINT_NAMES

    raw_points = generate_sample_trajectory(duration=duration, steps=30)
    for pt in raw_points:
        p_msg = JointTrajectoryPoint()
        p_msg.positions = [float(p) for p in pt["positions"]]
        sec = int(pt["time"])
        nanosec = int((pt["time"] - sec) * 1e9)
        p_msg.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj_msg.points.append(p_msg)

    pub.publish(traj_msg)
    # Spin to flush DDS transmission buffers
    for _ in range(15):
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"[✓] Published {len(traj_msg.points)} waypoints (duration {duration:.1f}s) successfully!")
    node.destroy_node()
    rclpy.shutdown()


def run_ros2_teleop(topic: str = "/meta/joint_states", rate_hz: float = 50.0, duration: float = 15.0):
    """Simulate streaming Meta Quest VR headset tracking to ROS 2 JointState topic."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError:
        print("[ERROR] ROS 2 Python libraries (rclpy, sensor_msgs) not found.")
        sys.exit(1)

    rclpy.init()
    node = Node("meta_quest_teleop_simulator")
    pub = node.create_publisher(JointState, topic, 10)
    print(f"[*] Streaming simulated Meta Quest VR teleop motions to {topic} at {rate_hz:.1f} Hz...")
    print(f"[*] Duration: {duration:.1f}s (Press Ctrl+C to stop)...")

    start_time = time.perf_counter()
    period = 1.0 / rate_hz

    try:
        while rclpy.ok():
            now_rel = time.perf_counter() - start_time
            if duration > 0 and now_rel >= duration:
                break

            phase = now_rel * 1.5

            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = JOINT_NAMES

            # Human-like teleop motion tracking
            left_j1 = 0.3 * math.sin(phase * 0.8)
            left_j2 = 0.35 * (1.0 - math.cos(phase * 0.6))
            left_j3 = 0.2 * math.sin(phase)
            left_j4 = -0.4 * (1.0 - math.cos(phase * 0.7))
            left_j5 = 0.15 * math.sin(phase * 1.2)
            left_j6 = 0.25 * math.sin(phase)
            left_j7 = 0.0
            left_grip = 0.020 * (1.0 + math.sin(phase * 1.5))

            right_j1 = -0.3 * math.sin(phase * 0.8)
            right_j2 = -0.35 * (1.0 - math.cos(phase * 0.6))
            right_j3 = -0.2 * math.sin(phase)
            right_j4 = 0.4 * (1.0 - math.cos(phase * 0.7))
            right_j5 = -0.15 * math.sin(phase * 1.2)
            right_j6 = -0.25 * math.sin(phase)
            right_j7 = 0.0
            right_grip = left_grip

            msg.position = [
                left_j1, left_j2, left_j3, left_j4,
                left_j5, left_j6, left_j7, left_grip,
                right_j1, right_j2, right_j3, right_j4,
                right_j5, right_j6, right_j7, right_grip,
            ]

            pub.publish(msg)
            time.sleep(period)

    except KeyboardInterrupt:
        pass
    finally:
        print("\n[✓] Teleop stream finished.")
        node.destroy_node()
        rclpy.shutdown()


def run_udp_trajectory(host: str = "127.0.0.1", port: int = 9870, duration: float = 6.0):
    """Send trajectory directly to OpenArm high-speed UDP engine."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    points = generate_sample_trajectory(duration=duration, steps=25)
    payload = {
        "source": "UDP Trajectory Test Tool",
        "timestamp": time.time(),
        "joint_names": JOINT_NAMES,
        "trajectory": points,
    }
    raw = json.dumps(payload).encode("utf-8")
    sock.sendto(raw, (host, port))
    print(f"[✓] Sent trajectory ({len(points)} waypoints, duration {duration:.1f}s) to UDP {host}:{port}")
    sock.close()


def run_udp_teleop(host: str = "127.0.0.1", port: int = 9870, rate_hz: float = 50.0, duration: float = 10.0):
    """Send real-time teleop frames directly to OpenArm high-speed UDP engine."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[*] Streaming simulated Meta Quest VR motions over UDP to {host}:{port} at {rate_hz:.1f} Hz...")

    start_time = time.perf_counter()
    period = 1.0 / rate_hz

    try:
        while True:
            now_rel = time.perf_counter() - start_time
            if duration > 0 and now_rel >= duration:
                break

            phase = now_rel * 1.5
            pos = [
                0.3 * math.sin(phase * 0.8),
                0.35 * (1.0 - math.cos(phase * 0.6)),
                0.2 * math.sin(phase),
                -0.4 * (1.0 - math.cos(phase * 0.7)),
                0.15 * math.sin(phase * 1.2),
                0.25 * math.sin(phase),
                0.0,
                0.020 * (1.0 + math.sin(phase * 1.5)),
                -0.3 * math.sin(phase * 0.8),
                -0.35 * (1.0 - math.cos(phase * 0.6)),
                -0.2 * math.sin(phase),
                0.4 * (1.0 - math.cos(phase * 0.7)),
                -0.15 * math.sin(phase * 1.2),
                -0.25 * math.sin(phase),
                0.0,
                0.020 * (1.0 + math.sin(phase * 1.5)),
            ]

            payload = {
                "source": "Meta Quest VR Teleop (UDP)",
                "names": JOINT_NAMES,
                "positions": [round(p, 4) for p in pos],
            }
            sock.sendto(json.dumps(payload).encode("utf-8"), (host, port))
            time.sleep(period)

    except KeyboardInterrupt:
        pass
    finally:
        print("\n[✓] Finished UDP stream.")
        sock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=["trajectory", "teleop", "udp-trajectory", "udp-teleop"],
        default="trajectory",
        help="Test mode: 'trajectory' (ROS 2), 'teleop' (ROS 2), 'udp-trajectory', 'udp-teleop'",
    )
    parser.add_argument("--topic", default=None, help="ROS 2 topic to publish to (auto-selected by default)")
    parser.add_argument("--host", default="127.0.0.1", help="UDP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9870, help="UDP target port (default: 9870)")
    parser.add_argument("--rate", type=float, default=50.0, help="Streaming rate in Hz (default: 50.0)")
    parser.add_argument("--duration", type=float, default=8.0, help="Duration in seconds (default: 8.0)")

    args = parser.parse_args()

    if args.mode == "trajectory":
        topic = args.topic or "/openarm/joint_trajectory"
        run_ros2_trajectory(topic=topic, duration=args.duration)
    elif args.mode == "teleop":
        topic = args.topic or "/meta/joint_states"
        run_ros2_teleop(topic=topic, rate_hz=args.rate, duration=args.duration)
    elif args.mode == "udp-trajectory":
        run_udp_trajectory(host=args.host, port=args.port, duration=args.duration)
    elif args.mode == "udp-teleop":
        run_udp_teleop(host=args.host, port=args.port, rate_hz=args.rate, duration=args.duration)


if __name__ == "__main__":
    main()
