#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenArm ROS 2 Joint State Bridge
Subscribes to ROS 2 topics (sensor_msgs/JointState, trajectory_msgs/JointTrajectory)
and streams joint targets to the OpenArm controller via high-speed UDP (or WebSocket).
"""

import sys
import json
import socket
import argparse
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class OpenArmROS2BridgeNode(Node if ROS2_AVAILABLE else object):
    def __init__(self, topic: str = "/joint_states", udp_host: str = "127.0.0.1", udp_port: int = 9870):
        if not ROS2_AVAILABLE:
            raise RuntimeError("rclpy / ROS 2 is not installed in the current environment.")
        
        super().__init__('openarm_ros2_joint_bridge')
        
        self.udp_target = (udp_host, udp_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # QoS Profile: Best Effort for fast, low-latency joint state streaming
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.subscription = self.create_subscription(
            JointState,
            topic,
            self.joint_state_callback,
            qos
        )
        
        self.msg_count = 0
        self.last_log_time = time.time()
        self.get_logger().info(f"🚀 OpenArm ROS 2 Bridge Node Started!")
        self.get_logger().info(f"   Subscribed to Topic: {topic} (sensor_msgs/msg/JointState)")
        self.get_logger().info(f"   Forwarding UDP stream -> {udp_host}:{udp_port}")

    def joint_state_callback(self, msg: JointState):
        """Receive JointState message and forward as lightweight JSON via UDP"""
        if not msg.name or not msg.position:
            return

        payload = {
            "names": list(msg.name),
            "positions": [float(p) for p in msg.position],
            "timestamp": time.time()
        }

        try:
            data = json.dumps(payload).encode('utf-8')
            self.sock.sendto(data, self.udp_target)
            self.msg_count += 1
            
            now = time.time()
            if now - self.last_log_time >= 3.0:
                hz = self.msg_count / (now - self.last_log_time)
                self.get_logger().info(f"Streaming joint states: {hz:.1f} Hz | Last: {len(msg.name)} joints")
                self.msg_count = 0
                self.last_log_time = now
        except Exception as e:
            self.get_logger().warn(f"Failed to forward UDP packet: {e}")


def main():
    parser = argparse.ArgumentParser(description="OpenArm ROS 2 Joint State Bridge")
    parser.add_argument("--topic", default="/joint_states", help="ROS 2 topic to subscribe (mặc định: /joint_states)")
    parser.add_argument("--udp-ip", default="127.0.0.1", help="OpenArm server IP (mặc định: 127.0.0.1)")
    parser.add_argument("--udp-port", type=int, default=9870, help="OpenArm UDP port (mặc định: 9870)")
    args = parser.parse_args()

    if not ROS2_AVAILABLE:
        print("[LỖI] rclpy không tồn tại trong môi trường Python hiện tại.")
        print("Gợi ý:")
        print("  1. Chạy script này trong môi trường ROS 2 (source /opt/ros/.../setup.bash)")
        print("  2. Hoặc chạy bên trong Docker container ROS 2 (ví dụ: openarm_ros2)")
        print("  3. Nếu không có ROS 2, bạn có thể dùng script test thuần Python: python3 scripts/stream_joint_states.py")
        sys.exit(1)

    rclpy.init()
    node = OpenArmROS2BridgeNode(topic=args.topic, udp_host=args.udp_ip, udp_port=args.udp_port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Dừng bridge...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
