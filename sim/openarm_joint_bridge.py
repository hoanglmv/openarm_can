#!/usr/bin/env python3
"""
OpenArm ROS 2 Streaming Joint Commands Bridge:
1. Publishes OpenArm real/sim joint states & commanded actions to ROS 2 topics:
   - /openarm/joint_states (sensor_msgs/msg/JointState)
   - /openarm/joint_commands (sensor_msgs/msg/JointState)
2. Subscribes to real-time streaming joint commands (Meta Quest VR / Teleop):
   - /openarm/teleop/joint_commands (std_msgs/msg/Float64MultiArray - 14 arm joints:
     left j1..j7 then right j1..j7)
   - /openarm/teleop/left_gripper (std_msgs/msg/Float64MultiArray - data[0])
   - /openarm/teleop/right_gripper (std_msgs/msg/Float64MultiArray - data[0])
   - /teleop/joint_commands (sensor_msgs/msg/JointState - alias, arm joints only)
   - /meta/joint_states (sensor_msgs/msg/JointState - alias for Meta Quest teleop, arm joints only)
   All joint angles use the official OpenArm URDF convention (joint angle == motor angle).
   Arm and gripper targets are merged into the 16-joint order and forwarded to the
   OpenArm high-speed UDP engine (port 9870), which drives /openarm/joint_commands.
"""

import argparse
import json
import socket
import time
from typing import Dict, Optional, Sequence

from config import JOINT_NAME_TO_ID

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
except ImportError:
    import sys
    print("[ROS Bridge] ROS 2 (rclpy/sensor_msgs/std_msgs) not found. ROS 2 bridge disabled.")
    sys.exit(0)

JOINT_NAMES = [
    "left_j1", "left_j2", "left_j3", "left_j4",
    "left_j5", "left_j6", "left_j7", "left_gripper",
    "right_j1", "right_j2", "right_j3", "right_j4",
    "right_j5", "right_j6", "right_j7", "right_gripper",
]

# Motor IDs (1-based index into JOINT_NAMES)
ARM_JOINT_IDS = [1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15]
LEFT_GRIPPER_ID = 8
RIGHT_GRIPPER_ID = 16


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
        left_gripper_topic: str = "/openarm/teleop/left_gripper",
        right_gripper_topic: str = "/openarm/teleop/right_gripper",
    ):
        super().__init__("openarm_joint_bridge")

        # QoS profiles: BEST_EFFORT for minimum latency real-time streaming
        qos_pub = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        qos_state_pub = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        qos_sub = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # 1. State Publishers (OpenArm -> ROS 2)
        self.state_pub = self.create_publisher(JointState, state_topic, qos_state_pub)
        self.command_pub = self.create_publisher(JointState, command_topic, qos_pub)

        # 2. Command Subscribers (Meta Quest VR / Teleop Streaming Joint Commands -> OpenArm)
        # Latest teleop target per motor ID, merged from the arm and gripper topics
        self.teleop_targets: Dict[int, float] = {}

        self.teleop_sub = self.create_subscription(
            Float64MultiArray,
            teleop_topic,
            self._handle_teleop_arm_array,
            qos_sub,
        )
        self.teleop_short_sub = self.create_subscription(
            JointState,
            "/teleop/joint_commands",
            self._handle_teleop_arm_joint_state,
            qos_sub,
        )
        self.meta_sub = self.create_subscription(
            JointState,
            meta_topic,
            self._handle_teleop_arm_joint_state,
            qos_sub,
        )
        self.left_gripper_sub = self.create_subscription(
            Float64MultiArray,
            left_gripper_topic,
            lambda msg: self._handle_teleop_gripper(msg, LEFT_GRIPPER_ID),
            qos_sub,
        )
        self.right_gripper_sub = self.create_subscription(
            Float64MultiArray,
            right_gripper_topic,
            lambda msg: self._handle_teleop_gripper(msg, RIGHT_GRIPPER_ID),
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
        self.get_logger().info(f"   [SUB] Teleop Commands (Float64MultiArray, 14 arm joints): {teleop_topic}")
        self.get_logger().info(f"   [SUB] Teleop Grippers (Float64MultiArray): {left_gripper_topic} & {right_gripper_topic}")
        self.get_logger().info("   [SUB] Teleop Commands alias (JointState): /teleop/joint_commands")
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

    def _handle_teleop_arm_array(self, msg: Float64MultiArray):
        """Receive the 14 arm joint targets (left j1..j7, right j1..j7) as a Float64MultiArray."""
        self._apply_teleop_arm_positions(msg.data)

    def _handle_teleop_arm_joint_state(self, msg: JointState):
        """Receive arm joint targets as a JointState (alias topics). Named joints may be any subset."""
        names = msg.name if msg.name and len(msg.name) == len(msg.position) else None
        self._apply_teleop_arm_positions(msg.position, names)

    def _apply_teleop_arm_positions(self, positions: Sequence[float], names: Optional[Sequence[str]] = None):
        """
        Merge arm joint targets from the Meta Quest VR / Teleop controller.
        Gripper joints are ignored here; they come from the dedicated gripper topics.
        """
        if not positions:
            return

        if names:
            updates = {}
            for name, position in zip(names, positions):
                motor_id = JOINT_NAME_TO_ID.get(str(name).lower())
                if motor_id in ARM_JOINT_IDS:
                    updates[motor_id] = float(position)
        elif len(positions) == len(ARM_JOINT_IDS):
            updates = dict(zip(ARM_JOINT_IDS, (float(p) for p in positions)))
        else:
            self.get_logger().warning(
                f"Rejected teleop arm command: expected {len(ARM_JOINT_IDS)} positions "
                f"(or named joints), got {len(positions)}",
                throttle_duration_sec=2.0,
            )
            return

        if not updates:
            return
        self.teleop_targets.update(updates)
        self._forward_teleop_targets()

    def _handle_teleop_gripper(self, msg: Float64MultiArray, motor_id: int):
        """Receive a single gripper target (data[0]) from the teleop controller."""
        if not msg.data:
            return
        self.teleop_targets[motor_id] = float(msg.data[0])
        self._forward_teleop_targets()

    def _forward_teleop_targets(self):
        """Forward the merged arm + gripper targets, in JOINT_NAMES order, to the OpenArm UDP engine (port 9870)."""
        motor_ids = sorted(self.teleop_targets)
        payload = {
            "source": "Meta Quest VR Teleop",
            "timestamp": time.time(),
            "names": [JOINT_NAMES[motor_id - 1] for motor_id in motor_ids],
            "positions": [self.teleop_targets[motor_id] for motor_id in motor_ids],
        }

        try:
            raw_bytes = json.dumps(payload).encode("utf-8")
            self.sock_out.sendto(raw_bytes, self.udp_target)
            self.teleop_count += 1
            now = time.time()
            if now - self.last_teleop_log >= 2.0:
                hz = self.teleop_count / (now - self.last_teleop_log)
                self.get_logger().info(
                    f"[Meta Teleop Stream] {hz:.1f} Hz | Forwarded {len(motor_ids)} merged joint targets -> OpenArm"
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
    parser.add_argument("--left-gripper-topic", default="/openarm/teleop/left_gripper")
    parser.add_argument("--right-gripper-topic", default="/openarm/teleop/right_gripper")
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
        left_gripper_topic=args.left_gripper_topic,
        right_gripper_topic=args.right_gripper_topic,
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
