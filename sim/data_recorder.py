#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Record synchronized OpenArm RGB-D demonstrations in ACT HDF5 format.

Supports two synchronization architectures:
1. 50Hz Periodic Timer with Zero-Order Hold (ZOH) Frame Buffer (Default, Recommended):
   - Samples CAN bus motor joints (50Hz) and the latest RGB-D frame at exact 20ms intervals (dt=0.02s).
   - Eliminates camera FPS bottlenecks and guarantees strict 50Hz HDF5 compliance with DATA_FORMAT_SPECIFICATION.md.
2. Approximate Time Synchronizer (Legacy / High-FPS Camera):
   - 1:1 camera-triggered callback via message_filters.ApproximateTimeSynchronizer.
"""

import argparse
import json
import os
import sys
import threading
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any

import cv2
import h5py
import numpy as np

# Reconfigure stdout for Windows unicode support
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

# Optional ROS 2 imports for cross-platform portability
try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    import message_filters
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, JointState
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object
    ExternalShutdownException = Exception
    CvBridge = None
    Image = None
    JointState = None


DEFAULT_JOINT_NAMES = [
    "left_j1", "left_j2", "left_j3", "left_j4",
    "left_j5", "left_j6", "left_j7", "left_gripper",
    "right_j1", "right_j2", "right_j3", "right_j4",
    "right_j5", "right_j6", "right_j7", "right_gripper",
]


class EpisodeWriter:
    """
    Quản lý việc ghi dữ liệu HDF5 chuẩn ACT cho 1 Episode theo DATA_FORMAT_SPECIFICATION.md.
    """
    def __init__(
        self,
        output_dir: Path,
        episode_name: str,
        batch_size: int = 10,
        frequency_hz: float = 50.0,
        img_height: int = 480,
        img_width: int = 640,
        depth_scale: float = 0.001,
        depth_min_mm: int = 200,
        depth_max_mm: int = 1200,
        joint_names: Optional[List[str]] = None,
        is_sim: bool = False,
    ):
        output_dir.mkdir(parents=True, exist_ok=True)
        self.partial_path = output_dir / f"{episode_name}.partial.hdf5"
        self.final_path = output_dir / f"{episode_name}.hdf5"
        if self.partial_path.exists() or self.final_path.exists():
            raise FileExistsError(f"Episode already exists: {episode_name}")

        self.joint_names = joint_names or DEFAULT_JOINT_NAMES
        self.num_joints = len(self.joint_names)
        self.frequency_hz = float(frequency_hz)
        self.img_height = int(img_height)
        self.img_width = int(img_width)
        self.depth_scale = float(depth_scale)
        self.depth_min_mm = int(depth_min_mm)
        self.depth_max_mm = int(depth_max_mm)
        self.batch_size = int(batch_size)

        self.file = h5py.File(self.partial_path, "x", libver="latest")
        observations = self.file.create_group("observations")
        images = observations.create_group("images")

        self.datasets = {
            "chest_rgb": images.create_dataset(
                "chest_rgb",
                shape=(0, self.img_height, self.img_width, 3),
                maxshape=(None, self.img_height, self.img_width, 3),
                chunks=(1, self.img_height, self.img_width, 3),
                dtype=np.uint8,
                compression="lzf",
            ),
            "chest_depth": images.create_dataset(
                "chest_depth",
                shape=(0, self.img_height, self.img_width),
                maxshape=(None, self.img_height, self.img_width),
                chunks=(1, self.img_height, self.img_width),
                dtype=np.uint16,
                compression="lzf",
            ),
            "qpos": observations.create_dataset(
                "qpos",
                shape=(0, self.num_joints),
                maxshape=(None, self.num_joints),
                chunks=(self.batch_size, self.num_joints),
                dtype=np.float32,
            ),
            "qvel": observations.create_dataset(
                "qvel",
                shape=(0, self.num_joints),
                maxshape=(None, self.num_joints),
                chunks=(self.batch_size, self.num_joints),
                dtype=np.float32,
            ),
            "effort": observations.create_dataset(
                "effort",
                shape=(0, self.num_joints),
                maxshape=(None, self.num_joints),
                chunks=(self.batch_size, self.num_joints),
                dtype=np.float32,
            ),
            "action": self.file.create_dataset(
                "action",
                shape=(0, self.num_joints),
                maxshape=(None, self.num_joints),
                chunks=(self.batch_size, self.num_joints),
                dtype=np.float32,
            ),
            "timestamp_ns": self.file.create_dataset(
                "timestamp_ns",
                shape=(0,),
                maxshape=(None,),
                chunks=(self.batch_size,),
                dtype=np.int64,
            ),
        }

        # Metadata attributes chuẩn hóa theo DATA_FORMAT_SPECIFICATION.md
        self.file.attrs["sim"] = bool(is_sim)
        self.file.attrs["frequency_hz"] = self.frequency_hz
        self.file.attrs["control_dt"] = float(1.0 / self.frequency_hz)
        self.file.attrs["robot_type"] = "OpenArm_Bimanual_16DOF"
        self.file.attrs["num_joints"] = self.num_joints
        self.file.attrs["depth_scale"] = self.depth_scale
        self.file.attrs["depth_range_m"] = np.asarray(
            [self.depth_min_mm / 1000.0, self.depth_max_mm / 1000.0],
            dtype=np.float32,
        )
        self.file.attrs["action_representation"] = "absolute_joint_position"
        self.file.attrs["joint_names_json"] = json.dumps(self.joint_names)
        self.file.attrs["complete"] = False
        self.file.attrs["created_at"] = datetime.now().astimezone().isoformat()

        self.buffers = {name: [] for name in self.datasets}
        self.sample_count = 0
        self.first_timestamp_ns = None
        self.last_timestamp_ns = None

    def append(self, rgb, depth, qpos, qvel, effort, action, timestamp_ns):
        self.buffers["chest_rgb"].append(rgb)
        self.buffers["chest_depth"].append(depth)
        self.buffers["qpos"].append(qpos)
        self.buffers["qvel"].append(qvel)
        self.buffers["effort"].append(effort)
        self.buffers["action"].append(action)
        self.buffers["timestamp_ns"].append(timestamp_ns)
        self.sample_count += 1
        if self.first_timestamp_ns is None:
            self.first_timestamp_ns = timestamp_ns
        self.last_timestamp_ns = timestamp_ns
        if len(self.buffers["timestamp_ns"]) >= self.batch_size:
            self.flush()

    def flush(self):
        count = len(self.buffers["timestamp_ns"])
        if count == 0:
            return
        old_size = self.datasets["timestamp_ns"].shape[0]
        new_size = old_size + count
        for name, dataset in self.datasets.items():
            dataset.resize(new_size, axis=0)
            dataset[old_size:new_size] = np.asarray(self.buffers[name], dtype=dataset.dtype)
            self.buffers[name].clear()
        self.file.flush()

    def finalize(self):
        self.flush()
        if self.sample_count == 0:
            self.file.attrs["complete"] = False
            self.file.flush()
            self.file.close()
            return None

        duration_s = (self.last_timestamp_ns - self.first_timestamp_ns) / 1e9
        self.file.attrs["samples"] = self.sample_count
        self.file.attrs["duration_seconds"] = duration_s
        self.file.attrs["complete"] = True
        self.file.flush()
        self.file.close()
        os.replace(self.partial_path, self.final_path)
        return self.final_path


class ACTDataRecorder(Node):
    """
    ROS 2 Node thu thập dữ liệu OpenArm đồng bộ.
    Hỗ trợ 2 chế độ:
    - periodic_zoh (mặc định): Timer cố định 50Hz, đọc góc CAN bus tức thì và frame ảnh mới nhất từ bộ đệm (ZOH).
    - approximate: ApproximateTimeSynchronizer truyền thống chờ khung hình camera.
    """
    def __init__(self, args, writer: EpisodeWriter):
        if not HAS_ROS2:
            raise RuntimeError("ROS 2 (rclpy) chưa được cài đặt trong môi trường này.")

        super().__init__("openarm_act_data_recorder")
        self.args = args
        self.writer = writer
        self.bridge = CvBridge()
        self.rejected_samples = 0
        self.last_timestamp_ns = -1
        self.lock = threading.Lock()

        # Bộ đệm tin nhắn cảm biến
        self.latest_rgb_msg = None
        self.latest_depth_msg = None
        self.latest_state_msg = None
        self.latest_command_msg = None
        self.warmup_logged = False

        # Cache giải mã ảnh nhằm tối ưu hoá CPU trong ZOH 50Hz
        self._cached_rgb_msg = None
        self._cached_rgb = None
        self._cached_depth_msg = None
        self._cached_depth = None

        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        joint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        has_command = (
            bool(self.args.command_topic)
            and self.args.command_topic.strip().lower() not in ("none", "false")
            and not getattr(self.args, "use_state_as_action", False)
        )
        self.has_command = has_command

        if self.args.sync_mode == "periodic_zoh":
            # 1. Đăng ký nhận tin tức thời từ các cảm biến độc lập
            self.create_subscription(Image, args.rgb_topic, self._on_rgb, camera_qos)
            self.create_subscription(Image, args.depth_topic, self._on_depth, camera_qos)
            self.create_subscription(JointState, args.state_topic, self._on_state, joint_qos)
            if self.has_command:
                self.create_subscription(JointState, args.command_topic, self._on_command, joint_qos)

            # 2. Timer chu kỳ lấy mẫu cố định (50.0Hz -> 20.0ms)
            timer_period = 1.0 / self.args.frequency_hz
            self.timer = self.create_timer(timer_period, self._on_periodic_timer)
            self.get_logger().info(
                f"[*] Khởi động ACT Data Recorder chế độ Periodic ZOH tại {self.args.frequency_hz} Hz (chu kỳ {timer_period*1000:.1f}ms). "
                f"Nhấn Ctrl+C để kết thúc episode."
            )
        else:
            # Chế độ Approximate Time Synchronizer truyền thống
            subscribers = [
                message_filters.Subscriber(self, Image, args.rgb_topic, qos_profile=camera_qos),
                message_filters.Subscriber(self, Image, args.depth_topic, qos_profile=camera_qos),
                message_filters.Subscriber(self, JointState, args.state_topic, qos_profile=joint_qos),
            ]
            if self.has_command:
                subscribers.append(
                    message_filters.Subscriber(self, JointState, args.command_topic, qos_profile=joint_qos)
                )

            self.sync = message_filters.ApproximateTimeSynchronizer(
                subscribers,
                queue_size=25,
                slop=args.sync_tolerance_ms / 1000.0,
            )
            if self.has_command:
                self.sync.registerCallback(self._on_approx_sample_4)
            else:
                self.sync.registerCallback(self._on_approx_sample_3)

            self.get_logger().info(
                f"[*] Khởi động ACT Data Recorder chế độ Approximate Synchronizer. Nhấn Ctrl+C để kết thúc episode."
            )

    def _on_rgb(self, msg):
        with self.lock:
            self.latest_rgb_msg = msg

    def _on_depth(self, msg):
        with self.lock:
            self.latest_depth_msg = msg

    def _on_state(self, msg):
        with self.lock:
            self.latest_state_msg = msg

    def _on_command(self, msg):
        with self.lock:
            self.latest_command_msg = msg

    def _ordered_values(self, message: JointState, field: str) -> np.ndarray:
        raw_values = getattr(message, field, None)
        expected_joints = self.writer.joint_names
        expected_count = len(expected_joints)

        # Hỗ trợ trường hợp robot driver không gửi velocity hoặc effort
        if (raw_values is None or len(raw_values) == 0) and field in ("velocity", "effort"):
            return np.zeros(expected_count, dtype=np.float32)

        if len(raw_values) != expected_count:
            raise ValueError(
                f"expected {expected_count} {field} values, received {len(raw_values)}"
            )

        if not message.name:
            values = np.asarray(raw_values, dtype=np.float32)
        else:
            if len(message.name) != len(raw_values):
                raise ValueError(f"joint name and {field} lengths differ")
            by_name = dict(zip(message.name, raw_values))
            missing = [name for name in expected_joints if name not in by_name]
            if missing:
                raise ValueError(f"missing joints: {missing}")
            values = np.asarray([by_name[name] for name in expected_joints], dtype=np.float32)

        if not np.all(np.isfinite(values)):
            raise ValueError(f"joint {field} values contain NaN or infinity")
        return values

    def _reject(self, reason: str):
        self.rejected_samples += 1
        self.get_logger().warning(reason, throttle_duration_sec=5.0)

    def _decode_rgb(self, rgb_msg: Image) -> np.ndarray:
        if rgb_msg is self._cached_rgb_msg and self._cached_rgb is not None:
            return self._cached_rgb

        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"RGB image must have three channels, got {rgb.shape}")

        if rgb.shape[:2] != (self.writer.img_height, self.writer.img_width):
            rgb = cv2.resize(
                rgb,
                (self.writer.img_width, self.writer.img_height),
                interpolation=cv2.INTER_AREA,
            )

        rgb = np.asarray(rgb, dtype=np.uint8)
        self._cached_rgb_msg = rgb_msg
        self._cached_rgb = rgb
        return rgb

    def _decode_depth(self, depth_msg: Image) -> np.ndarray:
        if depth_msg is self._cached_depth_msg and self._cached_depth is not None:
            return self._cached_depth

        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if depth.ndim != 2:
            raise ValueError(f"depth image must be single-channel, got {depth.shape}")

        if depth.shape != (self.writer.img_height, self.writer.img_width):
            depth = cv2.resize(
                depth,
                (self.writer.img_width, self.writer.img_height),
                interpolation=cv2.INTER_NEAREST,
            )

        # Chuẩn hoá đơn vị depth về milimét (uint16)
        if depth.dtype == np.float32:
            scale = self.writer.depth_scale if self.writer.depth_scale > 0 else 0.001
            depth_mm = np.nan_to_num(depth, nan=0.0) / scale
        elif depth.dtype == np.uint16:
            depth_mm = depth.astype(np.float32)
        else:
            depth_mm = depth.astype(np.float32)

        # Giữ 0 cho pixel mất tín hiệu / rỗng, kẹp trong dải làm việc quy định
        depth = np.where(
            depth_mm == 0,
            0,
            np.clip(depth_mm, self.writer.depth_min_mm, self.writer.depth_max_mm),
        ).astype(np.uint16, copy=False)

        self._cached_depth_msg = depth_msg
        self._cached_depth = depth
        return depth

    def _process_and_record(
        self,
        rgb_msg: Image,
        depth_msg: Image,
        state_msg: JointState,
        command_msg: Optional[JointState],
        timestamp_ns: int,
    ):
        try:
            if timestamp_ns <= self.last_timestamp_ns:
                self._reject("Rejected duplicate or non-monotonic timestamp")
                return

            rgb = self._decode_rgb(rgb_msg)
            depth = self._decode_depth(depth_msg)

            qpos = self._ordered_values(state_msg, "position")
            qvel = self._ordered_values(state_msg, "velocity")
            effort = self._ordered_values(state_msg, "effort")

            if command_msg is not None and not getattr(self.args, "use_state_as_action", False):
                action = self._ordered_values(command_msg, "position")
            else:
                # Nếu không có command topic, sử dụng chính góc đo thực tế làm action
                action = qpos.copy()

            self.writer.append(
                rgb,
                depth,
                qpos,
                qvel,
                effort,
                action,
                timestamp_ns,
            )
            self.last_timestamp_ns = timestamp_ns

            if self.writer.sample_count % int(self.writer.frequency_hz * 5) == 0:
                seconds = self.writer.sample_count / self.writer.frequency_hz
                self.get_logger().info(
                    f"Recorded {self.writer.sample_count} samples ({seconds:.1f} s @ {self.writer.frequency_hz:.1f}Hz)"
                )

            if self.args.max_duration > 0:
                elapsed = self.writer.sample_count / self.writer.frequency_hz
                if elapsed >= self.args.max_duration:
                    self.get_logger().info("Maximum duration reached; stopping episode")
                    rclpy.shutdown()
        except Exception as error:
            self._reject(f"Rejected sample: {error}")

    def _on_periodic_timer(self):
        """Callback định kỳ kích hoạt chính xác theo chu kỳ frequency_hz (50Hz = 20ms)."""
        with self.lock:
            rgb_msg = self.latest_rgb_msg
            depth_msg = self.latest_depth_msg
            state_msg = self.latest_state_msg
            command_msg = self.latest_command_msg

        # Kiểm tra điều kiện khởi động luồng cảm biến (Warm-up check)
        need_command = self.has_command
        if rgb_msg is None or depth_msg is None or state_msg is None or (need_command and command_msg is None):
            waiting = []
            if rgb_msg is None: waiting.append("RGB Image")
            if depth_msg is None: waiting.append("Depth Image")
            if state_msg is None: waiting.append("Joint States")
            if need_command and command_msg is None: waiting.append("Joint Commands")
            self.get_logger().info(
                f"[*] Đang chờ luồng cảm biến xuất hiện: {', '.join(waiting)}...",
                throttle_duration_sec=2.0,
            )
            return

        if not self.warmup_logged:
            self.warmup_logged = True
            self.get_logger().info(f"[*] Tất cả luồng cảm biến đã sẵn sàng. Bắt đầu ghi dữ liệu tại {self.writer.frequency_hz} Hz...")

        timestamp_ns = self.get_clock().now().nanoseconds
        self._process_and_record(rgb_msg, depth_msg, state_msg, command_msg, timestamp_ns)

    def _on_approx_sample_4(self, rgb_msg, depth_msg, state_msg, command_msg):
        """Callback đồng bộ approximate khi có command topic."""
        timestamp_ns = rgb_msg.header.stamp.sec * 1_000_000_000 + rgb_msg.header.stamp.nanosec
        self._process_and_record(rgb_msg, depth_msg, state_msg, command_msg, timestamp_ns)

    def _on_approx_sample_3(self, rgb_msg, depth_msg, state_msg):
        """Callback đồng bộ approximate khi không có command topic."""
        timestamp_ns = rgb_msg.header.stamp.sec * 1_000_000_000 + rgb_msg.header.stamp.nanosec
        self._process_and_record(rgb_msg, depth_msg, state_msg, None, timestamp_ns)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="dataset", help="Thư mục lưu trữ HDF5")
    parser.add_argument("--episode", default=None, help="Tên episode (tùy chọn, mặc định sinh theo thời gian)")
    parser.add_argument("--frequency-hz", type=float, default=50.0, help="Tần số lấy mẫu ghi dữ liệu cố định (mặc định: 50.0 Hz)")
    parser.add_argument(
        "--sync-mode",
        type=str,
        default="periodic_zoh",
        choices=["periodic_zoh", "approximate"],
        help="Cơ chế đồng bộ: 'periodic_zoh' (khuyên dùng - 50Hz chuẩn ZOH) hoặc 'approximate' (chờ camera)",
    )
    parser.add_argument("--rgb-topic", default="/camera/act/rgb", help="ROS 2 topic ảnh RGB")
    parser.add_argument("--depth-topic", default="/camera/act/depth", help="ROS 2 topic ảnh Depth")
    parser.add_argument("--state-topic", default="/openarm/joint_states", help="ROS 2 topic trạng thái khớp thực tế")
    parser.add_argument(
        "--command-topic",
        default="/openarm/joint_commands",
        help="ROS 2 topic lệnh góc điều khiển (đặt 'none' nếu không sử dụng)",
    )
    parser.add_argument(
        "--use-state-as-action",
        action="store_true",
        help="Sử dụng trực tiếp qpos làm action (cho teleop/kinesthetic teaching không qua command topic)",
    )
    parser.add_argument("--sync-tolerance-ms", type=float, default=12.0, help="Dung sai đồng bộ cho mode approximate (ms)")
    parser.add_argument("--img-height", type=int, default=480, help="Chiều cao ảnh RGB-D (mặc định: 480)")
    parser.add_argument("--img-width", type=int, default=640, help="Chiều rộng ảnh RGB-D (mặc định: 640)")
    parser.add_argument("--depth-scale", type=float, default=0.001, help="Hệ số quy đổi depth (mặc định: 0.001)")
    parser.add_argument("--depth-min-mm", type=int, default=200, help="Khoảng cách depth tối thiểu hợp lệ mm (mặc định: 200)")
    parser.add_argument("--depth-max-mm", type=int, default=1200, help="Khoảng cách depth tối đa hợp lệ mm (mặc định: 1200)")
    parser.add_argument("--num-joints", type=int, default=16, help="Số lượng khớp robot (mặc định: 16)")
    parser.add_argument("--batch-size", type=int, default=10, help="HDF5 write batch size")
    parser.add_argument("--max-duration", type=float, default=25.0, help="Thời lượng ghi tối đa của 1 episode (giây, mặc định: 25.0s)")
    parser.add_argument("--sim", action="store_true", help="Đánh dấu episode là dữ liệu mô phỏng Simulation")

    args = parser.parse_args()

    if args.frequency_hz <= 0:
        parser.error("Tần số sampling frequency-hz phải > 0")
    if args.sync_tolerance_ms <= 0 or args.batch_size <= 0 or args.max_duration < 0:
        parser.error("invalid tolerance, batch size, or maximum duration")

    if not HAS_ROS2:
        print("[X] Lỗi: Cần môi trường ROS 2 (rclpy) để khởi động node thu thập dữ liệu trên Robot/IPC.", file=sys.stderr)
        sys.exit(1)

    episode_name = args.episode or datetime.now().strftime("episode_%Y%m%d_%H%M%S")
    if not episode_name.replace("-", "").replace("_", "").isalnum():
        parser.error("episode name may contain only letters, numbers, '-' and '_'")

    writer = EpisodeWriter(
        output_dir=Path(args.output_dir).expanduser(),
        episode_name=episode_name,
        batch_size=args.batch_size,
        frequency_hz=args.frequency_hz,
        img_height=args.img_height,
        img_width=args.img_width,
        depth_scale=args.depth_scale,
        depth_min_mm=args.depth_min_mm,
        depth_max_mm=args.depth_max_mm,
        is_sim=args.sim,
    )

    rclpy.init()
    node = ACTDataRecorder(args, writer)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        final_path = writer.finalize()
        if final_path:
            print(
                f"[Recorder] Saved {writer.sample_count} samples to {final_path} "
                f"({node.rejected_samples} rejected, Frequency: {args.frequency_hz}Hz)",
                flush=True,
            )
        else:
            print(
                f"[Recorder] No valid samples; incomplete file kept at {writer.partial_path}",
                flush=True,
            )


if __name__ == "__main__":
    main()
