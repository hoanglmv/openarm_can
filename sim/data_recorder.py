#!/usr/bin/env python3
"""Record synchronized OpenArm RGB-D demonstrations in ACT HDF5 format."""

import argparse
import json
import os
from pathlib import Path
from datetime import datetime

import h5py
import message_filters
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
        self.last_timestamp_ns = -1

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
        subscribers = [
            message_filters.Subscriber(
                self, Image, args.rgb_topic, qos_profile=camera_qos
            ),
            message_filters.Subscriber(
                self, Image, args.depth_topic, qos_profile=camera_qos
            ),
            message_filters.Subscriber(
                self, JointState, args.state_topic, qos_profile=joint_qos
            ),
            message_filters.Subscriber(
                self, JointState, args.command_topic, qos_profile=joint_qos
            ),
        ]
        self.sync = message_filters.ApproximateTimeSynchronizer(
            subscribers,
            queue_size=25,
            slop=args.sync_tolerance_ms / 1000.0,
        )
        self.sync.registerCallback(self._on_sample)
        self.get_logger().info(
            "Recording synchronized RGB, depth, qpos, qvel, effort, and action. "
            "Press Ctrl+C to finish the episode."
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

    def _on_sample(self, rgb_msg, depth_msg, state_msg, command_msg):
        try:
            timestamp_ns = rgb_msg.header.stamp.sec * 1_000_000_000
            timestamp_ns += rgb_msg.header.stamp.nanosec
            if timestamp_ns <= self.last_timestamp_ns:
                self._reject("Rejected duplicate or non-monotonic camera timestamp")
                return

            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            expected_rgb_shape = (IMAGE_HEIGHT, IMAGE_WIDTH, 3)
            expected_depth_shape = (IMAGE_HEIGHT, IMAGE_WIDTH)
            if rgb.shape != expected_rgb_shape:
                raise ValueError(
                    f"RGB shape must be {expected_rgb_shape}, got {rgb.shape}"
                )
            if depth.shape != expected_depth_shape:
                raise ValueError(
                    f"depth shape must be {expected_depth_shape}, got {depth.shape}"
                )
            if depth.dtype != np.uint16:
                raise ValueError(f"depth dtype must be uint16, got {depth.dtype}")

            qpos = self._ordered_values(state_msg, "position")
            qvel = self._ordered_values(state_msg, "velocity")
            effort = self._ordered_values(state_msg, "effort")
            action = self._ordered_values(command_msg, "position")

            # Preserve missing depth as zero. Valid measurements are clipped to
            # the chest-camera working range defined by the dataset contract.
            depth = np.where(
                depth == 0,
                0,
                np.clip(depth, DEPTH_MIN_MM, DEPTH_MAX_MM),
            ).astype(np.uint16, copy=False)

            self.writer.append(
                np.asarray(rgb, dtype=np.uint8),
                depth,
                qpos,
                qvel,
                effort,
                action,
                timestamp_ns,
            )
            self.last_timestamp_ns = timestamp_ns
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
            self._reject(f"Rejected synchronized sample: {error}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="dataset")
    parser.add_argument("--episode", default=None)
    parser.add_argument("--rgb-topic", default="/camera/act/rgb")
    parser.add_argument("--depth-topic", default="/camera/act/depth")
    parser.add_argument("--state-topic", default="/openarm/joint_states")
    parser.add_argument("--command-topic", default="/openarm/joint_commands")
    parser.add_argument("--sync-tolerance-ms", type=float, default=12.0)
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
    if args.sync_tolerance_ms <= 0 or args.batch_size <= 0 or args.max_duration < 0:
        parser.error("invalid tolerance, batch size, or maximum duration")

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
