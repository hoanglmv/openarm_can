#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Meta Quest VR / ROS 2 Streaming Joint Commands Test Tool for OpenArm Bimanual Robot.

Phương án 1 (Zero-Latency Streaming Joint Commands):
Kính Meta Quest VR stream trực tiếp JointState (danh sách Joint Command) vào topic
`/openarm/teleop/joint_commands` (hoặc `/meta/joint_states`).
Phía backend OpenArm đã có sẵn bộ nội suy làm mượt (400Hz S-curve interpolator)
xử lý tức thì với độ trễ xấp xỉ 0 (Zero-latency).

Các chế độ kiểm thử:
1. Waypoints Mode (List Joint Command):
   Stream danh sách góc khớp (waypoints) dạng JointState ở tần số 100Hz.
2. Teleop Stream Mode (Meta Quest VR Teleop):
   Stream cử động tay thời gian thực ở tần số 100Hz (sensor_msgs/msg/JointState).
3. Direct UDP Mode:
   Bắn gói tin JSON trực tiếp qua UDP port 9870 tới backend OpenArm.

Ví dụ sử dụng:
  # 1. Stream danh sách joint command (waypoints) 100Hz qua ROS 2:
  python3 tools/test_meta_trajectory.py --mode waypoints --rate 100

  # 2. Giả lập Meta Quest VR stream cử động trực tiếp 100Hz:
  python3 tools/test_meta_trajectory.py --mode teleop --rate 100

  # 3. Stream trực tiếp qua UDP (không cần ROS 2):
  python3 tools/test_meta_trajectory.py --mode udp-teleop --rate 100
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


def generate_sample_waypoints(duration: float = 6.0, rate_hz: float = 100.0):
    """
    Tạo danh sách các điểm đặt góc khớp (List Joint Commands / Waypoints) mượt mà:
    - Cánh tay trái vẫy và gập khuỷu
    - Cánh tay phải chuyển động đối xứng
    - Gripper đóng mở nhịp nhàng
    """
    total_steps = int(duration * rate_hz)
    dt = 1.0 / rate_hz
    waypoints = []

    for i in range(total_steps + 1):
        t = i * dt
        phase = 2.0 * math.pi * (t / duration)

        # Chuyển động sóng hài mượt mà
        left_j1 = 0.35 * math.sin(phase)
        left_j2 = 0.40 * (1.0 - math.cos(phase))
        left_j3 = 0.20 * math.sin(phase)
        left_j4 = -0.50 * (1.0 - math.cos(phase))
        left_j5 = 0.15 * math.sin(phase)
        left_j6 = 0.25 * math.sin(phase * 2.0)
        left_j7 = 0.0
        # Gripper đóng mở (0.0m -> 0.04m)
        left_grip = 0.020 * (1.0 - math.cos(phase * 2.0))

        # Đối xứng cánh tay phải
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

        waypoints.append({
            "time": round(t, 4),
            "positions": [round(p, 4) for p in positions],
        })

    return waypoints


def run_ros2_streaming_waypoints(
    topic: str = "/openarm/teleop/joint_commands",
    duration: float = 6.0,
    rate_hz: float = 100.0,
):
    """
    Stream danh sách Joint Commands (Waypoints) liên tục dạng JointState qua ROS 2.
    Phía backend OpenArm đón nhận và nội suy 400Hz mượt mà tức thì (Zero-latency).
    """
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import JointState
    except ImportError:
        print("[ERROR] ROS 2 Python libraries (rclpy, sensor_msgs) not found.")
        print("Please source your ROS 2 environment: source /opt/ros/<distro>/setup.bash")
        sys.exit(1)

    rclpy.init()
    node = Node("openarm_joint_command_streamer")

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    pub = node.create_publisher(JointState, topic, qos)

    print(f"[*] Streaming waypoint list to ROS 2 topic: {topic} at {rate_hz:.1f} Hz...")
    waypoints = generate_sample_waypoints(duration=duration, rate_hz=rate_hz)
    period = 1.0 / rate_hz

    start_time = time.perf_counter()
    count = 0

    try:
        for wp in waypoints:
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = [float(p) for p in wp["positions"]]

            pub.publish(msg)
            count += 1

            # Duy trì tần số phát chính xác
            elapsed = time.perf_counter() - start_time
            target_time = count * period
            sleep_time = target_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        total_time = time.perf_counter() - start_time
        actual_hz = count / total_time if total_time > 0 else 0
        print(f"\n[✓] Finished streaming {count} joint commands in {total_time:.2f}s ({actual_hz:.1f} Hz)")
        node.destroy_node()
        rclpy.shutdown()


def run_ros2_teleop(
    topic: str = "/openarm/teleop/joint_commands",
    rate_hz: float = 100.0,
    duration: float = 15.0,
):
    """
    Giả lập Meta Quest VR stream cử động trực tiếp tới ROS 2 JointState topic (100Hz).
    """
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import JointState
    except ImportError:
        print("[ERROR] ROS 2 Python libraries (rclpy, sensor_msgs) not found.")
        sys.exit(1)

    rclpy.init()
    node = Node("meta_quest_teleop_simulator")

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    pub = node.create_publisher(JointState, topic, qos)
    print(f"[*] Simulating Meta Quest VR teleop stream to {topic} at {rate_hz:.1f} Hz...")
    print(f"[*] Duration: {duration:.1f}s (Press Ctrl+C to stop)...")

    start_time = time.perf_counter()
    period = 1.0 / rate_hz
    count = 0

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
            count += 1

            elapsed = time.perf_counter() - start_time
            target_time = count * period
            sleep_time = target_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        total_time = time.perf_counter() - start_time
        actual_hz = count / total_time if total_time > 0 else 0
        print(f"\n[✓] Teleop stream finished ({count} frames in {total_time:.2f}s, {actual_hz:.1f} Hz).")
        node.destroy_node()
        rclpy.shutdown()


def run_udp_teleop(
    host: str = "127.0.0.1",
    port: int = 9870,
    rate_hz: float = 100.0,
    duration: float = 10.0,
):
    """
    Bắn gói tin JSON trực tiếp qua UDP port 9870 tới backend OpenArm.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[*] Streaming simulated Meta Quest VR motions over UDP to {host}:{port} at {rate_hz:.1f} Hz...")

    start_time = time.perf_counter()
    period = 1.0 / rate_hz
    count = 0

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
            count += 1

            elapsed = time.perf_counter() - start_time
            target_time = count * period
            sleep_time = target_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        total_time = time.perf_counter() - start_time
        actual_hz = count / total_time if total_time > 0 else 0
        print(f"\n[✓] Finished UDP stream ({count} packets in {total_time:.2f}s, {actual_hz:.1f} Hz).")
        sock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=["waypoints", "trajectory", "teleop", "udp-teleop"],
        default="waypoints",
        help="Test mode: 'waypoints' (100Hz JointState stream), 'teleop' (100Hz continuous stream), 'udp-teleop'",
    )
    parser.add_argument(
        "--topic",
        default="/openarm/teleop/joint_commands",
        help="ROS 2 topic to publish to (default: /openarm/teleop/joint_commands)",
    )
    parser.add_argument("--host", default="127.0.0.1", help="UDP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9870, help="UDP target port (default: 9870)")
    parser.add_argument("--rate", type=float, default=100.0, help="Streaming rate in Hz (default: 100.0)")
    parser.add_argument("--duration", type=float, default=6.0, help="Duration in seconds (default: 6.0)")

    args = parser.parse_args()

    if args.mode in ("waypoints", "trajectory"):
        if args.mode == "trajectory":
            print("[INFO] Phương án 1 được kích hoạt: Chuyển trajectory thành stream danh sách JointState commands (100Hz).")
        run_ros2_streaming_waypoints(topic=args.topic, duration=args.duration, rate_hz=args.rate)
    elif args.mode == "teleop":
        run_ros2_teleop(topic=args.topic, rate_hz=args.rate, duration=args.duration)
    elif args.mode == "udp-teleop":
        run_udp_teleop(host=args.host, port=args.port, rate_hz=args.rate, duration=args.duration)


if __name__ == "__main__":
    main()
