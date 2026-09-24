#!/usr/bin/env python3
"""Expose a ROS 2 sensor_msgs/Image topic as an MJPEG HTTP stream."""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraFrameStore:
    def __init__(self):
        self.condition = threading.Condition()
        self.jpeg = None
        self.sequence = 0
        self.last_frame_at = 0.0

    def update(self, jpeg: bytes):
        with self.condition:
            self.jpeg = jpeg
            self.sequence += 1
            self.last_frame_at = time.monotonic()
            self.condition.notify_all()

    def wait_for_frame(self, after_sequence: int, timeout: float = 1.0):
        with self.condition:
            self.condition.wait_for(lambda: self.sequence > after_sequence, timeout=timeout)
            return self.jpeg, self.sequence

    def status(self):
        with self.condition:
            age = time.monotonic() - self.last_frame_at if self.last_frame_at else None
            return {
                "streaming": age is not None and age < 2.0,
                "frames": self.sequence,
                "frame_age_seconds": round(age, 3) if age is not None else None,
            }


class RosImageSubscriber(Node):
    def __init__(self, topic: str, frames: CameraFrameStore, max_width: int, quality: int):
        super().__init__("openarm_dashboard_camera_bridge")
        self.frames = frames
        self.bridge = CvBridge()
        self.max_width = max_width
        self.quality = quality
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.subscription = self.create_subscription(Image, topic, self._on_image, qos)
        self.get_logger().info(f"Waiting for camera frames on {topic}")

    def _on_image(self, message: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            height, width = frame.shape[:2]
            if self.max_width > 0 and width > self.max_width:
                scale = self.max_width / float(width)
                frame = cv2.resize(
                    frame,
                    (self.max_width, max(1, int(height * scale))),
                    interpolation=cv2.INTER_AREA,
                )
            ok, encoded = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
            )
            if ok:
                self.frames.update(encoded.tobytes())
        except Exception as error:
            self.get_logger().warning(f"Could not encode camera frame: {error}")


def make_handler(frames: CameraFrameStore, topic: str):
    class CameraHTTPHandler(BaseHTTPRequestHandler):
        def _common_headers(self):
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/status":
                payload = frames.status()
                payload["topic"] = topic
                body = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self._common_headers()
                self.end_headers()
                self.wfile.write(body)
                return

            if path not in ("/", "/stream.mjpg"):
                self.send_error(404)
                return

            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self._common_headers()
            self.end_headers()

            sequence = -1
            try:
                while True:
                    jpeg, next_sequence = frames.wait_for_frame(sequence)
                    if jpeg is None or next_sequence == sequence:
                        continue
                    sequence = next_sequence
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, _format, *_args):
            pass

    return CameraHTTPHandler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8890)
    parser.add_argument("--max-width", type=int, default=640)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    args = parser.parse_args()

    frames = CameraFrameStore()
    httpd = ThreadingHTTPServer((args.host, args.port), make_handler(frames, args.topic))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(
        f"[Camera] MJPEG stream: http://localhost:{args.port}/stream.mjpg "
        f"(ROS topic: {args.topic})",
        flush=True,
    )

    rclpy.init()
    node = RosImageSubscriber(
        args.topic,
        frames,
        max_width=max(0, args.max_width),
        quality=max(30, min(95, args.jpeg_quality)),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        httpd.shutdown()
        httpd.server_close()


if __name__ == "__main__":
    main()
