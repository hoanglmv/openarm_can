#!/usr/bin/env python3
"""Publish OpenArm backend joint samples received over local UDP to ROS 2."""

import argparse
import json
import socket

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    "left_j1", "left_j2", "left_j3", "left_j4",
    "left_j5", "left_j6", "left_j7", "left_gripper",
    "right_j1", "right_j2", "right_j3", "right_j4",
    "right_j5", "right_j6", "right_j7", "right_gripper",
]


class OpenArmJointBridge(Node):
    def __init__(self, host: str, port: int, state_topic: str, command_topic: str):
        super().__init__("openarm_joint_bridge")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.state_pub = self.create_publisher(JointState, state_topic, qos)
        self.command_pub = self.create_publisher(JointState, command_topic, qos)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.poll_timer = self.create_timer(0.002, self._poll)
        self.get_logger().info(
            f"Backend joint bridge listening on udp://{host}:{port}; "
            f"publishing {state_topic} and {command_topic}"
        )

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

    def _poll(self):
        latest = None
        while True:
            try:
                payload, _address = self.sock.recvfrom(8192)
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

    def destroy_node(self):
        self.sock.close()
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8891)
    parser.add_argument("--state-topic", default="/openarm/joint_states")
    parser.add_argument("--command-topic", default="/openarm/joint_commands")
    args = parser.parse_args()

    rclpy.init()
    node = OpenArmJointBridge(
        args.host,
        args.port,
        args.state_topic,
        args.command_topic,
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
