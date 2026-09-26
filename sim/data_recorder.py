#!/usr/bin/env python3
"""Record OpenArm ROS 2 topics at a fixed 50 Hz in ACT HDF5 format."""

import argparse
import json
import os
from pathlib import Path
from datetime import datetime

import h5py
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState


JOINT_NAMES = [
    "left_j1", "left_j2", "left_j3", "left_j4",
    "left_j5", "left_j6", "left_j7", "left_gripper",
    "right_j1", "right_j2", "right_j3", "right_j4",
    "right_j5", "right_j6", "right_j7", "right_gripper",
]
IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640
FREQUENCY_HZ = 50
DEPTH_SCALE = 0.001
DEPTH_MIN_MM = 200
DEPTH_MAX_MM = 1200


class EpisodeWriter:
    def __init__(
        self,
        output_dir: Path,
        episode_name: str,
        batch_size: int,
        is_sim: bool = False,
    ):
        output_dir.mkdir(parents=True, exist_ok=True)
        self.partial_path = output_dir / f"{episode_name}.partial.hdf5"
        self.final_path = output_dir / f"{episode_name}.hdf5"
        if self.partial_path.exists() or self.final_path.exists():
            raise FileExistsError(f"Episode already exists: {episode_name}")

        self.file = h5py.File(self.partial_path, "x", libver="latest")
        observations = self.file.create_group("observations")
        images = observations.create_group("images")
        self.datasets = {
            "chest_rgb": images.create_dataset(
                "chest_rgb",
                shape=(0, IMAGE_HEIGHT, IMAGE_WIDTH, 3),
                maxshape=(None, IMAGE_HEIGHT, IMAGE_WIDTH, 3),
                chunks=(1, IMAGE_HEIGHT, IMAGE_WIDTH, 3),
                dtype=np.uint8,
                compression="lzf",
            ),
            "chest_depth": images.create_dataset(
                "chest_depth",
                shape=(0, IMAGE_HEIGHT, IMAGE_WIDTH),
                maxshape=(None, IMAGE_HEIGHT, IMAGE_WIDTH),
                chunks=(1, IMAGE_HEIGHT, IMAGE_WIDTH),
                dtype=np.uint16,
                compression="lzf",
            ),
            "qpos": observations.create_dataset(
                "qpos",
                shape=(0, 16),
                maxshape=(None, 16),
                chunks=(batch_size, 16),
                dtype=np.float32,
            ),
            "qvel": observations.create_dataset(
                "qvel",
                shape=(0, 16),
                maxshape=(None, 16),
                chunks=(batch_size, 16),
                dtype=np.float32,
            ),
            "effort": observations.create_dataset(
                "effort",
                shape=(0, 16),
                maxshape=(None, 16),
                chunks=(batch_size, 16),
                dtype=np.float32,
            ),
            "action": self.file.create_dataset(
                "action",
                shape=(0, 16),
                maxshape=(None, 16),
                chunks=(batch_size, 16),
                dtype=np.float32,
            ),
            "timestamp_ns": self.file.create_dataset(
                "timestamp_ns",
                shape=(0,),
                maxshape=(None,),
                chunks=(batch_size,),
                dtype=np.int64,
            ),
        }
        self.file.attrs["sim"] = bool(is_sim)
        self.file.attrs["frequency_hz"] = FREQUENCY_HZ
        self.file.attrs["robot_type"] = "OpenArm_Bimanual_16DOF"
        self.file.attrs["num_joints"] = 16
        self.file.attrs["depth_scale"] = DEPTH_SCALE
        self.file.attrs["depth_range_m"] = np.asarray([0.2, 1.2], dtype=np.float32)
        self.file.attrs["action_representation"] = "absolute_joint_position"
        self.file.attrs["joint_names_json"] = json.dumps(JOINT_NAMES)
        self.file.attrs["complete"] = False
        self.file.attrs["created_at"] = datetime.now().astimezone().isoformat()

        self.batch_size = batch_size
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
    def __init__(self, args, writer: EpisodeWriter):
        super().__init__("openarm_act_data_recorder")
        self.args = args
        self.writer = writer
        self.bridge = CvBridge()
        self.rejected_samples = 0
        self.recording_timestamp_ns = None

        # Every input comes exclusively from three ROS 2 subscriptions. Callbacks
        # cache a complete RGB-D pair and the latest joint state; the 50 Hz timer
        # samples those values. A 25 Hz camera frame is therefore intentionally
        # used for two dataset timesteps while the 100 Hz joint state is downsampled.
        self.latest_rgb = None
        self.latest_rgb_stamp_ns = None
        self.latest_depth = None
        self.latest_depth_stamp_ns = None
        self.latest_camera_pair = None
        self.latest_state = None

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
        self.rgb_sub = self.create_subscription(
            Image, args.rgb_topic, self._on_rgb, camera_qos
        )
        self.depth_sub = self.create_subscription(
            Image, args.depth_topic, self._on_depth, camera_qos
        )
        self.state_sub = self.create_subscription(
            JointState, args.state_topic, self._on_state, joint_qos
        )
        self.record_timer = self.create_timer(1.0 / FREQUENCY_HZ, self._record_latest)
        self.get_logger().info(
            "Subscribed to RGB, depth, and joint state ROS 2 topics; recording "
            "their latest complete samples at 50 Hz. "
            "Press Ctrl+C to finish the episode."
        )

    @staticmethod
    def _timestamp_ns(message):
        return (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )

    @staticmethod
    def _ordered_values(message: JointState, field: str):
        raw_values = getattr(message, field)
        if len(raw_values) != 16:
            raise ValueError(
                f"expected 16 {field} values, received {len(raw_values)}"
            )
        if not message.name:
            values = np.asarray(raw_values, dtype=np.float32)
        else:
            if len(message.name) != len(raw_values):
                raise ValueError(f"joint name and {field} lengths differ")
            by_name = dict(zip(message.name, raw_values))
            missing = [name for name in JOINT_NAMES if name not in by_name]
            if missing:
                raise ValueError(f"missing joints: {missing}")
            values = np.asarray([by_name[name] for name in JOINT_NAMES], dtype=np.float32)
        if not np.all(np.isfinite(values)):
            raise ValueError("joint values contain NaN or infinity")
        return values

    def _reject(self, reason):
        self.rejected_samples += 1
        self.get_logger().warning(reason, throttle_duration_sec=5.0)

    def _on_rgb(self, message):
        try:
            rgb = self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
            if rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError(f"RGB image must have three channels, got {rgb.shape}")
            if rgb.shape[:2] != (IMAGE_HEIGHT, IMAGE_WIDTH):
                rgb = cv2.resize(
                    rgb,
                    (IMAGE_WIDTH, IMAGE_HEIGHT),
                    interpolation=cv2.INTER_AREA,
                )
            self.latest_rgb = np.asarray(rgb, dtype=np.uint8).copy()
            self.latest_rgb_stamp_ns = self._timestamp_ns(message)
            self._update_camera_pair()
        except Exception as error:
            self._reject(f"Rejected RGB topic sample: {error}")

    def _on_depth(self, message):
        try:
            depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            if depth.ndim != 2:
                raise ValueError(f"depth image must be single-channel, got {depth.shape}")
            if depth.dtype != np.uint16:
                raise ValueError(f"depth dtype must be uint16, got {depth.dtype}")
            if depth.shape != (IMAGE_HEIGHT, IMAGE_WIDTH):
                depth = cv2.resize(
                    depth,
                    (IMAGE_WIDTH, IMAGE_HEIGHT),
                    interpolation=cv2.INTER_NEAREST,
                )
            self.latest_depth = np.where(
                depth == 0, 0, np.clip(depth, DEPTH_MIN_MM, DEPTH_MAX_MM)
            ).astype(np.uint16, copy=False)
            self.latest_depth_stamp_ns = self._timestamp_ns(message)
            self._update_camera_pair()
        except Exception as error:
            self._reject(f"Rejected depth topic sample: {error}")

    def _update_camera_pair(self):
        if (
            self.latest_rgb_stamp_ns is not None
            and self.latest_rgb_stamp_ns == self.latest_depth_stamp_ns
        ):
            self.latest_camera_pair = (self.latest_rgb, self.latest_depth)

    def _on_state(self, message):
        try:
            self.latest_state = (
                self._ordered_values(message, "position"),
                self._ordered_values(message, "velocity"),
                self._ordered_values(message, "effort"),
            )
        except Exception as error:
            self._reject(f"Rejected joint state topic sample: {error}")

    def _record_latest(self):
        if self.latest_camera_pair is None or self.latest_state is None:
            self.get_logger().warning(
                "Waiting for complete RGB-D and joint state topic samples",
                throttle_duration_sec=5.0,
            )
            return

        try:
            rgb, depth = self.latest_camera_pair
            qpos, qvel, effort = self.latest_state
            # This recorder intentionally has no command-topic input. Preserve the
            # ACT schema by using the observed absolute joint position as action.
            action = qpos.copy()
            period_ns = round(1_000_000_000 / FREQUENCY_HZ)
            if self.recording_timestamp_ns is None:
                now_ns = self.get_clock().now().nanoseconds
                self.recording_timestamp_ns = (now_ns // period_ns) * period_ns
            else:
                self.recording_timestamp_ns += period_ns

            self.writer.append(
                rgb,
                depth,
                qpos,
                qvel,
                effort,
                action,
                self.recording_timestamp_ns,
            )
            if self.writer.sample_count % 250 == 0:
                seconds = self.writer.sample_count / FREQUENCY_HZ
                self.get_logger().info(
                    f"Recorded {self.writer.sample_count} samples ({seconds:.1f} s)"
                )
            if self.args.max_duration > 0:
                elapsed = self.writer.sample_count / FREQUENCY_HZ
                if elapsed >= self.args.max_duration:
                    self.get_logger().info("Maximum duration reached; stopping episode")
                    rclpy.shutdown()
        except Exception as error:
            self._reject(f"Could not record latest ROS 2 topic samples: {error}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="dataset")
    parser.add_argument("--episode", default=None)
    parser.add_argument("--rgb-topic", default="/camera/act/rgb")
    parser.add_argument("--depth-topic", default="/camera/act/depth")
    parser.add_argument("--state-topic", default="/openarm/joint_states")
    parser.add_argument(
        "--batch-size",
        type=int,
        default=10,
        help="HDF5 write batch size; unrelated to the ACT action chunk size",
    )
    parser.add_argument("--max-duration", type=float, default=25.0)
    parser.add_argument(
        "--sim",
        action="store_true",
        help="mark the episode as simulation data",
    )
    args = parser.parse_args()
    if args.batch_size <= 0 or args.max_duration < 0:
        parser.error("invalid batch size or maximum duration")

    episode_name = args.episode or datetime.now().strftime("episode_%Y%m%d_%H%M%S")
    if not episode_name.replace("-", "").replace("_", "").isalnum():
        parser.error("episode name may contain only letters, numbers, '-' and '_'")

    writer = EpisodeWriter(
        Path(args.output_dir).expanduser(),
        episode_name,
        args.batch_size,
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
                f"({node.rejected_samples} rejected)",
                flush=True,
            )
        else:
            print(
                f"[Recorder] No valid samples; incomplete file kept at {writer.partial_path}",
                flush=True,
            )


if __name__ == "__main__":
    main()
