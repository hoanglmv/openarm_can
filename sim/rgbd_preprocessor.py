#!/usr/bin/env python3
"""Synchronize, resize, and rate-limit RealSense RGB-D frames for ACT."""

import argparse

import cv2
import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class RGBDPreprocessor(Node):
    def __init__(self, args):
        super().__init__("openarm_rgbd_preprocessor")
        self.bridge = CvBridge()
        self.width = args.width
        self.height = args.height
        self.period = 1.0 / args.rate
        self.latest_pair = None
        self.last_published_stamp = None
        self.pair_count = 0

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.rgb_pub = self.create_publisher(Image, args.rgb_output, output_qos)
        self.depth_pub = self.create_publisher(Image, args.depth_output, output_qos)

        self.rgb_sub = message_filters.Subscriber(
            self, Image, args.rgb_input, qos_profile=sensor_qos
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, args.depth_input, qos_profile=sensor_qos
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=8,
            slop=0.04,
        )
        self.sync.registerCallback(self._on_rgbd)
        self.publish_timer = self.create_timer(self.period, self._publish_latest)

        self.get_logger().info(
            f"RGB-D ACT output: {self.width}x{self.height} @ {args.rate:.1f} Hz | "
            f"{args.rgb_output} + {args.depth_output}"
        )

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image):
        self.latest_pair = (rgb_msg, depth_msg)

    def _publish_latest(self):
        if self.latest_pair is None:
            return

        rgb_msg, depth_msg = self.latest_pair
        stamp = (rgb_msg.header.stamp.sec, rgb_msg.header.stamp.nanosec)
        if stamp == self.last_published_stamp:
            return
        self.last_published_stamp = stamp

        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

            rgb_small = cv2.resize(
                rgb,
                (self.width, self.height),
                interpolation=cv2.INTER_AREA,
            )
            depth_small = cv2.resize(
                depth,
                (self.width, self.height),
                interpolation=cv2.INTER_NEAREST,
            )

            rgb_out = self.bridge.cv2_to_imgmsg(rgb_small, encoding="rgb8")
            depth_out = self.bridge.cv2_to_imgmsg(depth_small, encoding="16UC1")

            # One shared timestamp makes each RGB/depth pair unambiguous to ACT.
            rgb_out.header = rgb_msg.header
            depth_out.header = depth_msg.header
            depth_out.header.stamp = rgb_msg.header.stamp

            self.rgb_pub.publish(rgb_out)
            self.depth_pub.publish(depth_out)
            self.pair_count += 1
        except Exception as error:
            self.get_logger().warning(
                f"Could not preprocess RGB-D frame pair: {error}",
                throttle_duration_sec=5.0,
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rgb-input",
        default="/camera/camera/color/image_raw",
    )
    parser.add_argument(
        "--depth-input",
        default="/camera/camera/aligned_depth_to_color/image_raw",
    )
    parser.add_argument("--rgb-output", default="/camera/act/rgb")
    parser.add_argument("--depth-output", default="/camera/act/depth")
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=180)
    parser.add_argument("--rate", type=float, default=25.0)
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0 or args.rate <= 0:
        parser.error("width, height, and rate must be positive")

    rclpy.init()
    node = RGBDPreprocessor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
