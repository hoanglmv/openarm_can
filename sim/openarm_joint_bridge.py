#!/usr/bin/env python3
"""
OpenArm ROS 2 Streaming Joint Commands Bridge:
1. Publishes OpenArm real/sim joint states & commanded actions to ROS 2 topics:
   - /openarm/joint_states (sensor_msgs/msg/JointState)
   - /openarm/joint_commands (sensor_msgs/msg/JointState)
2. Subscribes to real-time streaming joint commands (Meta Quest VR / Teleop):
   - /openarm/teleop/joint_commands (sensor_msgs/msg/JointState)
   - /teleop/joint_commands (sensor_msgs/msg/JointState - alias)
   - /meta/joint_states (sensor_msgs/msg/JointState - alias for Meta Quest teleop)
   Forwards received commands directly to the OpenArm high-speed UDP engine (port 9870),
   which are immediately smoothed by the 400Hz interpolator with zero buffering latency.
"""

import argparse
import json
import socket
import time
from typing import List

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
except ImportError:
    import sys
    print("[ROS Bridge] ROS 2 (rclpy/sensor_msgs) not found. ROS 2 bridge disabled.")
    sys.exit(0)

JOINT_NAMES = [
    "left_j1", "left_j2", "left_j3", "left_j4",
    "left_j5", "left_j6", "left_j7", "left_gripper",
    "right_j1", "right_j2", "right_j3", "right_j4",
    "right_j5", "right_j6", "right_j7", "right_gripper",
]


class OpenArmJointBridge(Node):
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8891,
        udp_target_host: str = "127.0.0.1",
        udp_target_port: int = 9870,
        state_topic: str = "/openarm/joint_states",
        command_topic: str = "/openarm/joint_commands",
        teleop_topic: str = "/openarm/teleop/joint_commands",
        meta_topic: str = "/meta/joint_states",
    ):
        super().__init__("openarm_joint_bridge")

        # QoS profiles: BEST_EFFORT for minimum latency real-time streaming
        qos_pub = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        qos_sub = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # 1. State Publishers (OpenArm -> ROS 2)
        self.state_pub = self.create_publisher(JointState, state_topic, qos_pub)
        self.command_pub = self.create_publisher(JointState, command_topic, qos_pub)

        # 2. Command Subscribers (Meta Quest VR / Teleop Streaming Joint Commands -> OpenArm)
        self.teleop_sub = self.create_subscription(
            JointState,
            teleop_topic,
            self._handle_teleop_joint_state,
            qos_sub,
        )
        self.teleop_short_sub = self.create_subscription(
            JointState,
            "/teleop/joint_commands",
            self._handle_teleop_joint_state,
            qos_sub,
        )
        self.meta_sub = self.create_subscription(
            JointState,
            meta_topic,
            self._handle_teleop_joint_state,
            qos_sub,
        )

        # UDP Sockets
        # Incoming state from OpenArm backend (port 8891)
        self.sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_in.bind((host, port))
        self.sock_in.setblocking(False)

        # Outgoing command to OpenArm backend (port 9870)
        self.udp_target = (udp_target_host, udp_target_port)
        self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.poll_timer = self.create_timer(0.002, self._poll_incoming_backend)

        # Statistics
        self.teleop_count = 0
        self.last_teleop_log = 0.0

        self.get_logger().info("==================================================================")
        self.get_logger().info("🚀 OpenArm ROS 2 Streaming Joint Commands Bridge Ready (Zero-Latency)")
        self.get_logger().info(f"   [PUB] State topic: {state_topic}")
        self.get_logger().info(f"   [PUB] Command topic: {command_topic}")
        self.get_logger().info(f"   [SUB] Teleop Commands: {teleop_topic} & /teleop/joint_commands")
        self.get_logger().info(f"   [SUB] Meta Quest VR: {meta_topic}")
        self.get_logger().info(f"   [UDP] Target OpenArm Engine: {udp_target_host}:{udp_target_port}")
        self.get_logger().info("==================================================================")

    @staticmethod
    def _make_message(timestamp_ns: int, positions, velocities=None, efforts=None):
        message = JointState()
        message.header.stamp.sec = timestamp_ns // 1_000_000_000
        message.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        message.name = JOINT_NAMES
        message.position = [float(value) for value in positions]
        if velocities is not None:
            message.velocity = [float(value) for value in velocities]
        if efforts is not None:
            message.effort = [float(value) for value in efforts]
        return message

    def _poll_incoming_backend(self):
        """Poll OpenArm backend state updates from UDP (port 8891) and publish to ROS 2."""
        latest = None
        while True:
            try:
                payload, _address = self.sock_in.recvfrom(8192)
                latest = payload
            except BlockingIOError:
                break
            except OSError:
                return

        if latest is None:
            return

        try:
            sample = json.loads(latest.decode("utf-8"))
            qpos = sample["qpos"]
            qvel = sample["qvel"]
            effort = sample["effort"]
            action = sample["action"]
            timestamp_ns = int(sample["timestamp_ns"])
            if any(len(values) != 16 for values in (qpos, qvel, effort, action)):
                raise ValueError(
                    "qpos, qvel, effort, and action must contain exactly 16 values"
                )
            self.state_pub.publish(
                self._make_message(timestamp_ns, qpos, qvel, effort)
            )
            self.command_pub.publish(self._make_message(timestamp_ns, action))
        except Exception as error:
            self.get_logger().warning(
                f"Rejected backend joint sample: {error}",
                throttle_duration_sec=5.0,
            )

    def _handle_teleop_joint_state(self, msg: JointState):
        """
        Receive real-time JointState from Meta Quest VR / Teleop controller
        and forward directly to OpenArm UDP engine (port 9870).
        """
        if not msg.position:
            return

        payload = {
            "source": "Meta Quest VR Teleop",
            "timestamp": time.time(),
        }

        # Case 1: Message contains joint names
        if msg.name and len(msg.name) == len(msg.position):
            payload["names"] = list(msg.name)
            payload["positions"] = [float(p) for p in msg.position]
        else:
            # Case 2: Direct list of positions
            payload["positions"] = [float(p) for p in msg.position]

        try:
            raw_bytes = json.dumps(payload).encode("utf-8")
            self.sock_out.sendto(raw_bytes, self.udp_target)
            self.teleop_count += 1
            now = time.time()
            if now - self.last_teleop_log >= 2.0:
                hz = self.teleop_count / (now - self.last_teleop_log)
                self.get_logger().info(
                    f"[Meta Teleop Stream] {hz:.1f} Hz | Received {len(msg.position)} joint targets -> Forwarded to OpenArm"
                )
                self.teleop_count = 0
                self.last_teleop_log = now
        except Exception as e:
            self.get_logger().warning(f"Error forwarding teleop stream: {e}", throttle_duration_sec=2.0)

    def destroy_node(self):
        self.sock_in.close()
        self.sock_out.close()
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8891)
    parser.add_argument("--udp-target-host", default="127.0.0.1")
    parser.add_argument("--udp-target-port", type=int, default=9870)
    parser.add_argument("--state-topic", default="/openarm/joint_states")
    parser.add_argument("--command-topic", default="/openarm/joint_commands")
    parser.add_argument("--teleop-topic", default="/openarm/teleop/joint_commands")
    parser.add_argument("--meta-topic", default="/meta/joint_states")
    args = parser.parse_args()

    rclpy.init()
    node = OpenArmJointBridge(
        host=args.host,
        port=args.port,
        udp_target_host=args.udp_target_host,
        udp_target_port=args.udp_target_port,
        state_topic=args.state_topic,
        command_topic=args.command_topic,
        teleop_topic=args.teleop_topic,
        meta_topic=args.meta_topic,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
