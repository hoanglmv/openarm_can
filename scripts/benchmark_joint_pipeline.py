#!/usr/bin/env python3
"""Measure the simulated 100 Hz joint / 25 Hz camera timing contract."""

import argparse
import time


def run(duration_s: float):
    joint_period = 1.0 / 100.0
    camera_period = 1.0 / 25.0
    started = time.monotonic()
    next_joint = started
    next_camera = started
    last_joint_timestamp = None
    joint_timestamps = []
    camera_count = 0
    duplicate_count = 0
    dropped_count = 0

    while time.monotonic() - started < duration_s:
        now = time.monotonic()
        if now >= next_joint:
            timestamp = time.time_ns()
            if last_joint_timestamp == timestamp:
                duplicate_count += 1
            else:
                joint_timestamps.append(timestamp)
                last_joint_timestamp = timestamp
            next_joint += joint_period
            if now - next_joint > joint_period:
                dropped_count += int((now - next_joint) / joint_period)
                next_joint = now + joint_period
        if now >= next_camera:
            camera_count += 1
            next_camera += camera_period
        sleep_for = min(next_joint, next_camera) - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)

    elapsed = time.monotonic() - started
    intervals = [
        (right - left) / 1e6
        for left, right in zip(joint_timestamps, joint_timestamps[1:])
    ]
    mean_interval = sum(intervals) / len(intervals) if intervals else 0.0
    unique_rate = len(joint_timestamps) / elapsed if elapsed else 0.0
    print(f"duration_s={elapsed:.3f}")
    print(f"joint_unique_rate_hz={unique_rate:.3f}")
    print(f"camera_rate_hz={camera_count / elapsed:.3f}")
    print(f"raw_samples={len(joint_timestamps)}")
    print(f"duplicate_count={duplicate_count}")
    print(f"dropped_count={dropped_count}")
    print(f"mean_joint_interval_ms={mean_interval:.3f}")
    return unique_rate, duplicate_count, dropped_count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("duration must be positive")
    run(args.duration)


if __name__ == "__main__":
    main()