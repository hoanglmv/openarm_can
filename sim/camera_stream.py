#!/usr/bin/env python3
"""Expose ROS 2 sensor_msgs/Image topics as an MJPEG HTTP stream with active standby HUD."""

import argparse
import glob
import json
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
try:
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    class Node:
        pass
    class Image:
        pass


class CameraFrameStore:
    def __init__(self, topic: str, width: int, height: int, rate: float):
        self.condition = threading.Condition()
        self.topic = topic
        self.width = width
        self.height = height
        self.rate = rate
        self.jpeg = None
        self.sequence = 0
        self.last_frame_at = 0.0
        self.last_primary_at = 0.0
        self.feed_source = "none"

        # USB hardware detection cache
        self._last_usb_check = 0.0
        self._usb_realsense_status = "unknown"

    def _check_usb_realsense(self) -> str:
        now = time.monotonic()
        if now - self._last_usb_check < 3.0:
            return self._usb_realsense_status

        self._last_usb_check = now
        # 1. Check WSL2 Linux sysfs
        for path in glob.glob("/sys/bus/usb/devices/*/idVendor"):
            try:
                with open(path, "r") as f:
                    if f.read().strip() == "8086":
                        self._usb_realsense_status = "attached_wsl"
                        return self._usb_realsense_status
            except Exception:
                pass

        # 2. Check Windows host via usbipd (only Connected devices with valid X-Y BusId)
        try:
            res = subprocess.run(
                ["usbipd", "list"], capture_output=True, text=True, timeout=1.5
            )
            for line in res.stdout.splitlines():
                if "8086:0B3A" in line.upper() or "REALSENSE" in line.upper():
                    parts = line.split()
                    if parts and len(parts[0].split("-")) == 2 and all(p.isdigit() for p in parts[0].split("-")):
                        self._usb_realsense_status = f"windows_bus_{parts[0]}"
                        return self._usb_realsense_status
        except Exception:
            pass

        self._usb_realsense_status = "not_found"
        return self._usb_realsense_status

    def generate_standby_frame(self) -> bytes:
        """Render a crisp, high-tech standby monitor frame when camera feed is not publishing."""
        width = self.width
        height = self.height
        img = np.zeros((height, width, 3), dtype=np.uint8)

        # Subtle dark-slate gradient background
        for y in range(0, height, 4):
            val = int(10 + (y / height) * 14)
            img[y : y + 4, :] = (val + 6, val + 2, val)

        # Subtle tactical grid
        grid_step = 40
        for x in range(0, width, grid_step):
            cv2.line(img, (x, 0), (x, height), (32, 26, 20), 1)
        for y in range(0, height, grid_step):
            cv2.line(img, (0, y), (width, y), (32, 26, 20), 1)

        cx, cy = width // 2, height // 2

        # Reticle & Target Rings
        pulse = (np.sin(time.monotonic() * 3.0) + 1.0) * 0.5
        reticle_r = int(45 + pulse * 6)
        cv2.circle(img, (cx, cy), reticle_r, (70, 55, 40), 1)
        cv2.circle(img, (cx, cy), 18, (90, 75, 55), 1)
        cv2.circle(img, (cx, cy), 3, (120, 200, 255), -1)
        cv2.line(img, (cx - 65, cy), (cx - 24, cy), (70, 55, 40), 1)
        cv2.line(img, (cx + 24, cy), (cx + 65, cy), (70, 55, 40), 1)
        cv2.line(img, (cx, cy - 65), (cx, cy - 24), (70, 55, 40), 1)
        cv2.line(img, (cx, cy + 24), (cx, cy + 65), (70, 55, 40), 1)

        # Header Bar
        cv2.rectangle(img, (0, 0), (width, 36), (22, 16, 12), -1)
        cv2.line(img, (0, 36), (width, 36), (55, 42, 30), 1)
        cv2.putText(
            img,
            "OPENARM BIMANUAL AI - ACT CAMERA BRIDGE",
            (16, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (240, 195, 70),
            1,
            cv2.LINE_AA,
        )

        # Standby Status Box
        box_w, box_h = min(400, width - 24), min(95, height - 100)
        bx1 = cx - box_w // 2
        by1 = cy - 80
        cv2.rectangle(
            img,
            (bx1, by1),
            (bx1 + box_w, by1 + box_h),
            (18, 14, 10),
            -1,
        )
        cv2.rectangle(
            img,
            (bx1, by1),
            (bx1 + box_w, by1 + box_h),
            (60, 48, 32),
            1,
        )

        # Pulsing Amber Dot
        dot_color = (
            int(30 + pulse * 20),
            int(150 + pulse * 40),
            int(220 + pulse * 35),
        )
        cv2.circle(img, (bx1 + 24, by1 + 28), 6, dot_color, -1)
        cv2.putText(
            img,
            "STANDBY · WAITING FOR CAMERA STREAM",
            (bx1 + 38, by1 + 33),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        # Topic target line
        cv2.putText(
            img,
            f"ROS 2 Topic: {self.topic} ({self.width}x{self.height} @ {self.rate:g} Hz)",
            (bx1 + 38, by1 + 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (180, 180, 180),
            1,
            cv2.LINE_AA,
        )

        # Hardware connection hint based on real USB detection
        usb_status = self._check_usb_realsense()
        if usb_status == "attached_wsl":
            hint = "RealSense attached in WSL2 · Launching ROS 2 node..."
            hint_color = (100, 220, 120)
        elif usb_status.startswith("windows_bus_"):
            busid = usb_status.split("_")[-1]
            hint = f"Plugged into Windows! Attach via: usbipd attach --wsl --busid {busid}"
            hint_color = (70, 190, 255)
        else:
            hint = "Intel RealSense D435i not detected on USB. Plug in camera cable."
            hint_color = (140, 140, 140)

        cv2.putText(
            img,
            hint,
            (bx1 + 20, by1 + 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            hint_color,
            1,
            cv2.LINE_AA,
        )

        # Bottom Bar
        cv2.rectangle(img, (0, height - 32), (width, height), (18, 14, 10), -1)
        cv2.line(img, (0, height - 32), (width, height - 32), (45, 35, 25), 1)

        ts = time.strftime("%Y-%m-%d %H:%M:%S") + f".{int(time.time() * 10) % 10}"
        cv2.putText(
            img,
            f"LIVE: {ts}",
            (16, height - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (120, 220, 120),
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            img,
            "HTTP MJPEG: :8890/stream.mjpg",
            (width - 230, height - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (170, 170, 170),
            1,
            cv2.LINE_AA,
        )

        ok, encoded = cv2.imencode(
            ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        )
        return encoded.tobytes() if ok else b""

    def update(self, jpeg: bytes, is_primary: bool = True):
        now = time.monotonic()
        with self.condition:
            if is_primary:
                self.jpeg = jpeg
                self.sequence += 1
                self.last_frame_at = now
                self.last_primary_at = now
                self.feed_source = "act_rgb"
                self.condition.notify_all()
            else:
                # Fallback: only update if primary hasn't arrived recently
                if now - self.last_primary_at > 1.0:
                    self.jpeg = jpeg
                    self.sequence += 1
                    self.last_frame_at = now
                    self.feed_source = "color_raw"
                    self.condition.notify_all()

    def wait_for_frame(self, after_sequence: int, timeout: float = 0.12):
        with self.condition:
            now = time.monotonic()
            has_live = self.last_frame_at > 0.0 and (now - self.last_frame_at < 2.0)
            if has_live:
                self.condition.wait_for(
                    lambda: self.sequence > after_sequence, timeout=timeout
                )
                if self.sequence > after_sequence and self.jpeg is not None:
                    return self.jpeg, self.sequence, True

        # When no live frame is active or timed out, yield a standby frame
        time.sleep(0.08)  # ~12 FPS cadence for standby
        standby_bytes = self.generate_standby_frame()
        with self.condition:
            self.sequence += 1
            seq = self.sequence
        return standby_bytes, seq, False

    def status(self):
        with self.condition:
            age = (
                time.monotonic() - self.last_frame_at
                if self.last_frame_at > 0.0
                else None
            )
            has_live = age is not None and age < 2.0
            return {
                "streaming": True,
                "has_live_feed": has_live,
                "source": self.feed_source if has_live else "standby",
                "frames": self.sequence,
                "frame_age_seconds": round(age, 3) if age is not None else None,
                "topic": self.topic,
                "width": self.width,
                "height": self.height,
                "rate_hz": self.rate,
                "usb_realsense": self._check_usb_realsense(),
            }


class RosImageSubscriber(Node):
    def __init__(
        self,
        primary_topic: str,
        frames: CameraFrameStore,
        max_width: int,
        quality: int,
    ):
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

        # Primary subscription (e.g. /camera/act/rgb)
        self.sub_primary = self.create_subscription(
            Image, primary_topic, self._on_primary_image, qos
        )
        self.get_logger().info(f"Subscribed to primary camera topic: {primary_topic}")

        # Fallback subscriptions to raw RealSense color topics if primary is different
        fallback_topics = ["/camera/camera/color/image_raw", "/camera/color/image_raw"]
        self.sub_fallbacks = []
        for fb_topic in fallback_topics:
            if primary_topic != fb_topic:
                sub = self.create_subscription(
                    Image, fb_topic, self._on_fallback_image, qos
                )
                self.sub_fallbacks.append(sub)
                self.get_logger().info(f"Subscribed to raw fallback camera topic: {fb_topic}")

    def _process_frame(self, message: Image) -> bytes:
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
        return encoded.tobytes() if ok else None

    def _on_primary_image(self, message: Image):
        try:
            jpeg = self._process_frame(message)
            if jpeg:
                self.frames.update(jpeg, is_primary=True)
        except Exception as error:
            self.get_logger().warning(
                f"Could not encode primary camera frame: {error}"
            )

    def _on_fallback_image(self, message: Image):
        try:
            jpeg = self._process_frame(message)
            if jpeg:
                self.frames.update(jpeg, is_primary=False)
        except Exception as error:
            self.get_logger().warning(
                f"Could not encode fallback camera frame: {error}"
            )


def make_handler(frames: CameraFrameStore, topic: str):
    class CameraHTTPHandler(BaseHTTPRequestHandler):
        def _common_headers(self):
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/status":
                payload = frames.status()
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
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self._common_headers()
            self.end_headers()

            sequence = -1
            try:
                while True:
                    jpeg, next_sequence, _is_live = frames.wait_for_frame(sequence)
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


def find_realsense_v4l2_device():
    """Find the best V4L2 device index for RealSense RGB camera."""
    for dev_path in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            name_file = os.path.join(dev_path, "name")
            if os.path.exists(name_file):
                with open(name_file, "r") as f:
                    name = f.read().strip()
                if "realsense" in name.lower():
                    idx = int(os.path.basename(dev_path).replace("video", ""))
                    cap = cv2.VideoCapture(idx)
                    if cap.isOpened():
                        ret, frame = cap.read()
                        cap.release()
                        if ret and frame is not None and frame.size > 0:
                            return idx
        except Exception:
            pass
    for idx in [4, 6, 2, 0]:
        try:
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                ret, frame = cap.read()
                cap.release()
                if ret and frame is not None and frame.size > 0:
                    return idx
        except Exception:
            pass
    return None


def start_v4l2_worker(frames: CameraFrameStore, max_width: int, quality: int, rate: float):
    """Background worker capturing directly from V4L2 RealSense device when ROS 2 is not active."""
    def _worker():
        dev_idx = find_realsense_v4l2_device()
        if dev_idx is None:
            print("[Camera Stream] Notice: No V4L2 RealSense video device found for direct capture.", flush=True)
            return
        print(f"[Camera Stream] Direct V4L2 RealSense capture active on /dev/video{dev_idx} ({rate:g} FPS)", flush=True)
        cap = cv2.VideoCapture(dev_idx)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, rate)
        interval = 1.0 / max(1.0, rate)

        try:
            while True:
                now = time.monotonic()
                if frames.last_primary_at > 0.0 and (now - frames.last_primary_at < 2.0):
                    time.sleep(0.2)
                    continue

                ret, frame = cap.read()
                if not ret or frame is None:
                    time.sleep(0.05)
                    continue

                height, width = frame.shape[:2]
                if max_width > 0 and width > max_width:
                    scale = max_width / float(width)
                    frame = cv2.resize(
                        frame,
                        (max_width, max(1, int(height * scale))),
                        interpolation=cv2.INTER_AREA,
                    )

                ok, encoded = cv2.imencode(
                    ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
                )
                if ok:
                    frames.update(encoded.tobytes(), is_primary=False)
                time.sleep(interval)
        except Exception as e:
            print(f"[Camera Stream] V4L2 capture worker exception: {e}", flush=True)
        finally:
            cap.release()

    t = threading.Thread(target=_worker, daemon=True)
    t.start()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/camera/act/rgb")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8890)
    parser.add_argument("--width", type=int, default=424)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--rate", type=float, default=25.0)
    parser.add_argument("--max-width", type=int, default=424)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    args = parser.parse_args()

    if args.width <= 0 or args.height <= 0 or args.rate <= 0:
        parser.error("width, height, and rate must be positive")

    frames = CameraFrameStore(args.topic, args.width, args.height, args.rate)
    ThreadingHTTPServer.allow_reuse_address = True
    httpd = ThreadingHTTPServer(
        (args.host, args.port), make_handler(frames, args.topic)
    )
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(
        f"[Camera] MJPEG stream: http://localhost:{args.port}/stream.mjpg "
        f"(ROS topic: {args.topic})",
        flush=True,
    )

    start_v4l2_worker(
        frames,
        max_width=max(0, args.max_width),
        quality=max(30, min(95, args.jpeg_quality)),
        rate=args.rate,
    )

    if HAS_ROS2:
        try:
            rclpy.init()
            node = RosImageSubscriber(
                args.topic,
                frames,
                max_width=max(0, args.max_width),
                quality=max(30, min(95, args.jpeg_quality)),
            )
            try:
                rclpy.spin(node)
            except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
                pass
            finally:
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
        except Exception as error:
            print(f"[Camera Stream] ROS 2 error: {error}", flush=True)
    else:
        print("[Camera Stream] ROS 2 not installed: running with direct V4L2 capture.", flush=True)
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass

    httpd.shutdown()
    httpd.server_close()


if __name__ == "__main__":
    main()
