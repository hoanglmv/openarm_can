#!/usr/bin/env python3
"""Validate one OpenArm RGB-D ACT HDF5 episode."""

import argparse
from pathlib import Path

import h5py
import numpy as np


REQUIRED_ATTRIBUTES = {
    "frequency_hz": 50,
    "robot_type": "OpenArm_Bimanual_16DOF",
    "num_joints": 16,
    "depth_scale": 0.001,
}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def validate_hdf5(filepath: Path, strict_duration: bool = True):
    print(f"[*] Validating: {filepath}")
    with h5py.File(filepath, "r") as episode:
        for key in ("sim", *REQUIRED_ATTRIBUTES, "depth_range_m"):
            _require(key in episode.attrs, f"missing root attribute: {key}")
        for key, expected in REQUIRED_ATTRIBUTES.items():
            actual = episode.attrs[key]
            if isinstance(expected, float):
                _require(np.isclose(actual, expected), f"attribute {key}={actual}")
            else:
                _require(actual == expected, f"attribute {key}={actual!r}")
        _require(
            np.allclose(episode.attrs["depth_range_m"], [0.2, 1.2]),
            "depth_range_m must be [0.2, 1.2]",
        )

        required_paths = (
            "observations/images/chest_rgb",
            "observations/images/chest_depth",
            "observations/qpos",
            "observations/qvel",
            "action",
            "timestamp_ns",
        )
        for path in required_paths:
            _require(path in episode, f"missing dataset: {path}")

        rgb = episode["observations/images/chest_rgb"]
        depth = episode["observations/images/chest_depth"]
        qpos = episode["observations/qpos"]
        qvel = episode["observations/qvel"]
        effort = episode.get("observations/effort")
        action = episode["action"]
        timestamps = episode["timestamp_ns"]
        timesteps = action.shape[0]

        expected = {
            "chest_rgb": (rgb, (timesteps, 480, 640, 3), np.dtype(np.uint8)),
            "chest_depth": (depth, (timesteps, 480, 640), np.dtype(np.uint16)),
            "qpos": (qpos, (timesteps, 16), np.dtype(np.float32)),
            "qvel": (qvel, (timesteps, 16), np.dtype(np.float32)),
            "action": (action, (timesteps, 16), np.dtype(np.float32)),
            "timestamp_ns": (timestamps, (timesteps,), np.dtype(np.int64)),
        }
        if effort is not None:
            expected["effort"] = (
                effort,
                (timesteps, 16),
                np.dtype(np.float32),
            )
        for name, (dataset, shape, dtype) in expected.items():
            _require(dataset.shape == shape, f"{name} shape {dataset.shape} != {shape}")
            _require(dataset.dtype == dtype, f"{name} dtype {dataset.dtype} != {dtype}")

        if strict_duration:
            _require(
                750 <= timesteps <= 1250,
                f"episode has {timesteps} samples; expected 750..1250",
            )
        for name, dataset in (("qpos", qpos), ("qvel", qvel), ("action", action)):
            _require(np.isfinite(dataset[:]).all(), f"{name} contains NaN/Inf")
        if effort is not None:
            _require(np.isfinite(effort[:]).all(), "effort contains NaN/Inf")
        if timesteps > 1:
            intervals_ns = np.diff(timestamps[:])
            timing_error_ns = np.abs(intervals_ns - 20_000_000)
            _require(
                np.all(timing_error_ns <= 1_000_000),
                "sample interval exceeds the required 20 ms +/- 1 ms",
            )

        sample_indices = np.linspace(0, max(0, timesteps - 1), min(timesteps, 10), dtype=int)
        valid_ratios = []
        for index in sample_indices:
            depth_frame = depth[index]
            valid = depth_frame != 0
            valid_ratios.append(float(np.count_nonzero(valid)) / valid.size)
            if np.any(valid):
                valid_depth = depth_frame[valid]
                _require(
                    valid_depth.min() >= 200 and valid_depth.max() <= 1200,
                    f"depth frame {index} contains values outside 200..1200 mm",
                )
        minimum_valid_ratio = min(valid_ratios, default=0.0)
        _require(
            minimum_valid_ratio > 0.85,
            f"depth valid-pixel ratio is only {minimum_valid_ratio * 100:.1f}%",
        )

        print(f"  + Timesteps : {timesteps} ({timesteps / 50.0:.2f} s)")
        print(f"  + RGB       : {rgb.shape} | {rgb.dtype}")
        print(f"  + Depth     : {depth.shape} | {depth.dtype}")
        print(f"  + qpos/qvel : {qpos.shape} / {qvel.shape}")
        print(f"  + action    : {action.shape}")
        if timesteps > 1:
            print(
                "  + Period    : "
                f"{np.mean(np.diff(timestamps[:])) / 1e6:.3f} ms mean"
            )
        print(f"  + Min valid depth ratio: {minimum_valid_ratio * 100:.1f}%")
    print("[OK] Episode matches the OpenArm RGB-D ACT dataset contract.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("filepath", type=Path)
    parser.add_argument(
        "--allow-short",
        action="store_true",
        help="skip the normal 15-25 second episode-length check",
    )
    args = parser.parse_args()
    validate_hdf5(args.filepath.expanduser(), strict_duration=not args.allow_short)


if __name__ == "__main__":
    main()
