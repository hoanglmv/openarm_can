#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for sim/data_recorder.py and ACT HDF5 dataset compliance."""

import os
import shutil
import tempfile
from pathlib import Path
import numpy as np
import h5py
import pytest

from sim.data_recorder import EpisodeWriter, ACTDataRecorder, DEFAULT_JOINT_NAMES, HAS_ROS2
from act_pipeline.data.validate_dataset import validate_single_hdf5


def test_episode_writer_creates_valid_act_hdf5():
    """Verify EpisodeWriter outputs an HDF5 file 100% compliant with DATA_FORMAT_SPECIFICATION.md."""
    test_dir = tempfile.mkdtemp(prefix="test_recorder_")
    try:
        hz = 50.0
        dt_ns = int(1e9 / hz)
        writer = EpisodeWriter(
            output_dir=Path(test_dir),
            episode_name="episode_test",
            batch_size=10,
            frequency_hz=hz,
            img_height=480,
            img_width=640,
            depth_scale=0.001,
            depth_min_mm=200,
            depth_max_mm=1200,
        )

        # Record 100 steps (2.0 seconds @ 50 Hz, chunk size >= 50)
        start_ns = 1_000_000_000
        for step in range(100):
            rgb = np.full((480, 640, 3), fill_value=step % 256, dtype=np.uint8)
            depth = np.full((480, 640), fill_value=600, dtype=np.uint16)
            qpos = np.full(16, fill_value=0.05 * np.sin(step * 0.1), dtype=np.float32)
            qvel = np.zeros(16, dtype=np.float32)
            effort = np.zeros(16, dtype=np.float32)
            action = np.full(16, fill_value=0.05 * np.sin((step + 1) * 0.1), dtype=np.float32)
            t_ns = start_ns + step * dt_ns

            writer.append(rgb, depth, qpos, qvel, effort, action, t_ns)

        hdf5_path = writer.finalize()
        assert hdf5_path is not None
        assert os.path.exists(hdf5_path)
        assert not os.path.exists(writer.partial_path)

        # Validate with the strict DATA_FORMAT_SPECIFICATION validator
        val_result = validate_single_hdf5(str(hdf5_path), expected_hz=50.0)
        assert val_result.passed, f"Validation failed with errors: {val_result.errors}"
        assert len(val_result.errors) == 0
        assert val_result.info["timesteps"] == 100
        assert val_result.info["calculated_hz"] == 50.0
    finally:
        shutil.rmtree(test_dir, ignore_errors=True)


def test_episode_writer_prevent_overwrite():
    """Verify EpisodeWriter rejects creating an episode that already exists."""
    test_dir = tempfile.mkdtemp(prefix="test_recorder_")
    try:
        writer1 = EpisodeWriter(Path(test_dir), "episode_dup")
        writer1.append(
            np.zeros((480, 640, 3), dtype=np.uint8),
            np.zeros((480, 640), dtype=np.uint16),
            np.zeros(16, dtype=np.float32),
            np.zeros(16, dtype=np.float32),
            np.zeros(16, dtype=np.float32),
            np.zeros(16, dtype=np.float32),
            100,
        )
        writer1.finalize()

        with pytest.raises(FileExistsError):
            EpisodeWriter(Path(test_dir), "episode_dup")
    finally:
        shutil.rmtree(test_dir, ignore_errors=True)


@pytest.mark.skipif(not HAS_ROS2, reason="ROS 2 (rclpy) not available")
def test_act_data_recorder_joint_ordering_and_resizing():
    """Test ACTDataRecorder joint reordering, fallback zeros, and image resizing."""
    import rclpy
    from sensor_msgs.msg import JointState, Image
    from cv_bridge import CvBridge

    if not rclpy.ok():
        rclpy.init()

    test_dir = tempfile.mkdtemp(prefix="test_recorder_node_")
    bridge = CvBridge()

    try:
        class MockArgs:
            command_topic = "/openarm/joint_commands"
            use_state_as_action = False
            sync_mode = "periodic_zoh"
            rgb_topic = "/camera/act/rgb"
            depth_topic = "/camera/act/depth"
            state_topic = "/openarm/joint_states"
            frequency_hz = 50.0
            max_duration = 0.0

        writer = EpisodeWriter(
            output_dir=Path(test_dir),
            episode_name="episode_node_test",
            batch_size=5,
            frequency_hz=50.0,
        )
        node = ACTDataRecorder(MockArgs(), writer)

        # 1. Test _ordered_values with shuffled names
        state_msg = JointState()
        # Reverse joint names
        reversed_names = list(reversed(DEFAULT_JOINT_NAMES))
        state_msg.name = reversed_names
        state_msg.position = [float(i) for i in range(16)]
        state_msg.velocity = [] # Test empty velocity fallback
        state_msg.effort = []   # Test empty effort fallback

        positions = node._ordered_values(state_msg, "position")
        velocities = node._ordered_values(state_msg, "velocity")
        efforts = node._ordered_values(state_msg, "effort")

        assert len(positions) == 16
        # First joint in DEFAULT_JOINT_NAMES is left_j1, which was last in reversed_names (value = 15.0)
        assert positions[0] == 15.0
        assert np.all(velocities == 0.0)
        assert np.all(efforts == 0.0)

        # 2. Test _decode_rgb with resizing (input: 240x424 -> output: 480x640)
        rgb_raw = np.full((240, 424, 3), fill_value=200, dtype=np.uint8)
        rgb_msg = bridge.cv2_to_imgmsg(rgb_raw, encoding="rgb8")
        rgb_decoded = node._decode_rgb(rgb_msg)
        assert rgb_decoded.shape == (480, 640, 3)
        assert rgb_decoded.dtype == np.uint8

        # 3. Test _decode_depth with resizing & clipping (input: 240x424 -> output: 480x640)
        depth_raw = np.array([[0, 100, 600, 2000]], dtype=np.uint16)
        depth_raw = np.tile(depth_raw, (240, 106))[:, :424]
        depth_msg = bridge.cv2_to_imgmsg(depth_raw, encoding="passthrough")
        depth_decoded = node._decode_depth(depth_msg)
        assert depth_decoded.shape == (480, 640)
        assert depth_decoded.dtype == np.uint16
        # Check clipping bounds: 0 stays 0, 100 clips to 200, 600 stays 600, 2000 clips to 1200
        assert np.min(depth_decoded) == 0
        assert np.max(depth_decoded) == 1200

        # 4. Test _process_and_record with mock command
        cmd_msg = JointState()
        cmd_msg.name = DEFAULT_JOINT_NAMES
        cmd_msg.position = [0.1 * i for i in range(16)]

        node._process_and_record(rgb_msg, depth_msg, state_msg, cmd_msg, 10_000_000)
        assert writer.sample_count == 1
        assert node.rejected_samples == 0

        # Monotonicity check: duplicate timestamp rejected
        node._process_and_record(rgb_msg, depth_msg, state_msg, cmd_msg, 10_000_000)
        assert writer.sample_count == 1
        assert node.rejected_samples == 1

        node.destroy_node()
        writer.finalize()
    finally:
        shutil.rmtree(test_dir, ignore_errors=True)


@pytest.mark.skipif(not HAS_ROS2, reason="ROS 2 (rclpy) not available")
def test_act_data_recorder_live_spin_and_validation():
    """Verify live ROS 2 topic publisher and ACTDataRecorder interaction produces valid 50Hz HDF5."""
    import time
    import threading
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState, Image
    from cv_bridge import CvBridge

    if not rclpy.ok():
        rclpy.init()

    test_dir = tempfile.mkdtemp(prefix="test_recorder_live_")
    bridge = CvBridge()

    try:
        class LiveArgs:
            command_topic = "/test/joint_commands"
            use_state_as_action = False
            sync_mode = "periodic_zoh"
            rgb_topic = "/test/rgb"
            depth_topic = "/test/depth"
            state_topic = "/test/joint_states"
            frequency_hz = 50.0
            max_duration = 0.0

        writer = EpisodeWriter(
            output_dir=Path(test_dir),
            episode_name="episode_live",
            batch_size=10,
            frequency_hz=50.0,
        )

        recorder_node = ACTDataRecorder(LiveArgs(), writer)

        # Create mock publisher node
        pub_node = rclpy.create_node("mock_sensor_publisher")
        rgb_pub = pub_node.create_publisher(Image, "/test/rgb", 10)
        depth_pub = pub_node.create_publisher(Image, "/test/depth", 10)
        state_pub = pub_node.create_publisher(JointState, "/test/joint_states", 10)
        cmd_pub = pub_node.create_publisher(JointState, "/test/joint_commands", 10)

        rgb_raw = np.full((240, 424, 3), fill_value=120, dtype=np.uint8)
        depth_raw = np.full((240, 424), fill_value=500, dtype=np.uint16)

        stop_publishing = threading.Event()

        def publish_loop():
            step = 0
            while not stop_publishing.is_set():
                now_msg = pub_node.get_clock().now().to_msg()

                # Publish camera (25-30 Hz rate)
                if step % 2 == 0:
                    rgb_msg = bridge.cv2_to_imgmsg(rgb_raw, encoding="rgb8")
                    rgb_msg.header.stamp = now_msg
                    rgb_pub.publish(rgb_msg)

                    depth_msg = bridge.cv2_to_imgmsg(depth_raw, encoding="passthrough")
                    depth_msg.header.stamp = now_msg
                    depth_pub.publish(depth_msg)

                # Publish joints (100 Hz rate)
                state_msg = JointState()
                state_msg.header.stamp = now_msg
                state_msg.name = DEFAULT_JOINT_NAMES
                state_msg.position = [0.05 * np.sin(step * 0.05 + i * 0.1) for i in range(16)]
                state_msg.velocity = [0.0] * 16
                state_msg.effort = [0.0] * 16
                state_pub.publish(state_msg)

                cmd_msg = JointState()
                cmd_msg.header.stamp = now_msg
                cmd_msg.name = DEFAULT_JOINT_NAMES
                cmd_msg.position = [0.05 * np.sin((step + 1) * 0.05 + i * 0.1) for i in range(16)]
                cmd_pub.publish(cmd_msg)

                step += 1
                time.sleep(0.01)

        pub_thread = threading.Thread(target=publish_loop, daemon=True)
        pub_thread.start()

        executor = SingleThreadedExecutor()
        executor.add_node(recorder_node)
        exec_thread = threading.Thread(target=executor.spin, daemon=True)
        exec_thread.start()

        start_t = time.time()
        while time.time() - start_t < 2.0 and writer.sample_count < 60:
            time.sleep(0.05)

        stop_publishing.set()
        pub_thread.join(timeout=1.0)

        executor.shutdown()
        exec_thread.join(timeout=1.0)

        pub_node.destroy_node()
        recorder_node.destroy_node()
        final_file = writer.finalize()

        assert final_file is not None
        assert os.path.exists(final_file)
        assert writer.sample_count >= 50, f"Expected at least 50 samples, got {writer.sample_count}"

        # Validate with strict validator
        val_res = validate_single_hdf5(str(final_file), expected_hz=50.0)
        assert val_res.passed, f"Live recorded HDF5 validation errors: {val_res.errors}"
        assert len(val_res.errors) == 0
    finally:
        shutil.rmtree(test_dir, ignore_errors=True)

