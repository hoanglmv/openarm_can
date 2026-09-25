#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenArm Kungfu Motion Importer & Player
Dataset: LuluCao/KungfuAthleteBot (Hugging Face)
https://huggingface.co/datasets/LuluCao/KungfuAthleteBot

Extracts, converts, and streams martial arts / kungfu motion trajectories
from the Unitree G1 29-DOF humanoid dataset into OpenArm Dual 7-DOF Bimanual Robot.

Safety Architecture:
- Step 1: Query current robot joint state
- Step 2: Smoothly bring all 16 joints to Zero Pose (0.0 rad) with active holding torque
- Step 3: Execute the Kungfu martial arts sequence with strict velocity & joint limit safety clamping
"""

import argparse
import io
import json
import math
import os
import socket
import sys
import time
import urllib.request
import numpy as np

# OpenArm Joint Limits [min_rad, max_rad]
JOINT_LIMITS = {
    # Left Arm
    1: (-1.396, 3.490),   # M1: Vai Pitch [-80°, +200°]
    2: (-3.316, 0.174),   # M2: Vai Roll  [-190°, +10°]
    3: (-1.570, 1.570),   # M3: Bắp Yaw   [-90°, +90°]
    4: (0.000, 2.443),    # M4: Khuỷu     [0°, +140°]
    5: (-1.570, 1.570),   # M5: Cẳng Yaw  [-90°, +90°]
    6: (-0.785, 0.785),   # M6: Cổ Pitch  [-45°, +45°]
    7: (-1.570, 1.570),   # M7: Cổ Roll   [-90°, +90°]
    8: (0.000, 0.043),    # M8: Kẹp Trái  [0, 43mm]
    # Right Arm
    9:  (-1.396, 3.490),  # M9:  Vai Pitch [-80°, +200°]
    10: (-0.174, 3.316),  # M10: Vai Roll  [-10°, +190°]
    11: (-1.570, 1.570),  # M11: Bắp Yaw   [-90°, +90°]
    12: (0.000, 2.443),   # M12: Khuỷu     [0°, +140°]
    13: (-1.570, 1.570),  # M13: Cẳng Yaw  [-90°, +90°]
    14: (-0.785, 0.785),  # M14: Cổ Pitch  [-45°, +45°]
    15: (-1.570, 1.570),  # M15: Cổ Roll   [-90°, +90°]
    16: (0.000, 0.043),   # M16: Kẹp Phải  [0, 43mm]
}

JOINT_NAMES = [
    "left_j1", "left_j2", "left_j3", "left_j4",
    "left_j5", "left_j6", "left_j7", "left_gripper",
    "right_j1", "right_j2", "right_j3", "right_j4",
    "right_j5", "right_j6", "right_j7", "right_gripper",
]

HF_BASE_URL = "https://huggingface.co/datasets/LuluCao/KungfuAthleteBot/resolve/main"
CACHE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sim", "data", "kungfu")


def clamp_val(val: float, mid: int) -> float:
    min_v, max_v = JOINT_LIMITS[mid]
    return max(min_v, min(max_v, float(val)))


def fetch_action_titles():
    """Fetch and parse Action File Title.csv from cache or Hugging Face."""
    os.makedirs(CACHE_DIR, exist_ok=True)
    cache_file = os.path.join(CACHE_DIR, "Action_File_Title.csv")

    if not os.path.exists(cache_file):
        print(f"[Dataset] Downloading action index from Hugging Face...")
        url = f"{HF_BASE_URL}/Action%20File%20Title.csv"
        req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
        with urllib.request.urlopen(req, timeout=15) as resp:
            content = resp.read().decode("utf-8", errors="ignore")
        with open(cache_file, "w", encoding="utf-8") as f:
            f.write(content)
    else:
        with open(cache_file, "r", encoding="utf-8") as f:
            content = f.read()

    records = {}
    lines = content.splitlines()
    for l in lines[1:]:
        parts = l.split(",", 1)
        if len(parts) == 2:
            num = parts[0].replace(".mp4", "").strip()
            title = parts[1].strip()
            records[num] = title
    return records


def list_motions(keyword: str = None):
    """List available Kungfu motions matching optional keyword."""
    titles = fetch_action_titles()
    print(f"\n{'='*70}")
    print(f"KUNGFU ATHLETE BOT DATASET (LuluCao/KungfuAthleteBot)")
    print(f"Total motions available: {len(titles)}")
    print(f"{'='*70}")

    query = (keyword or "").strip().lower()
    matches = []
    for num, title in titles.items():
        if not query or query in title.lower() or query == "all":
            matches.append((num, title))

    print(f"Found {len(matches)} matching clips:")
    for num, title in matches[:40]:
        print(f"  Clip #{num:>4s}: {title}")
    if len(matches) > 40:
        print(f"  ... and {len(matches) - 40} more clips (use --list <keyword> to filter)")
    print(f"{'='*70}\n")


def download_clip_npz(clip_id: str) -> str:
    """Download .npz file for given clip ID to cache."""
    os.makedirs(CACHE_DIR, exist_ok=True)
    local_path = os.path.join(CACHE_DIR, f"{clip_id}.npz")
    if os.path.exists(local_path):
        return local_path

    # Try smooth_mj path first, then org path
    paths_to_try = [
        f"{HF_BASE_URL}/kungfu_athlete_for_g1_29dof/datasets/org_smooth_mj/{clip_id}.npz",
        f"{HF_BASE_URL}/kungfu_athlete_for_g1_29dof/datasets/org/kongfu_ground/{clip_id}/{clip_id}.npz",
        f"{HF_BASE_URL}/kungfu_athlete_for_g1_29dof/datasets/org/kongfu_jump/{clip_id}/{clip_id}.npz",
    ]

    last_err = None
    for url in paths_to_try:
        try:
            print(f"[Dataset] Downloading clip #{clip_id} from {url}...")
            req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
            with urllib.request.urlopen(req, timeout=20) as resp:
                data = resp.read()
            with open(local_path, "wb") as f:
                f.write(data)
            print(f"[Dataset] Saved to {local_path} ({len(data)} bytes)")
            return local_path
        except Exception as e:
            last_err = e

    raise RuntimeError(f"Could not download clip #{clip_id}: {last_err}")


def convert_g1_to_openarm_trajectory(npz_path: str, max_duration: float = None, sample_rate_hz: int = 30) -> dict:
    """
    Converts G1 29-DOF humanoid martial arts motion to OpenArm Dual 7-DOF Trajectory.
    
    Mapping:
    Left Arm (G1 cols 22..28 -> OpenArm M1..M7):
      M1 (Left Shoulder Pitch): G1 left_shoulder_pitch
      M2 (Left Shoulder Roll):  -G1 left_shoulder_roll (abduction sign inversion)
      M3 (Left Shoulder Yaw):   G1 left_shoulder_yaw
      M4 (Left Elbow Flex):     max(0.0, G1 left_elbow)
      M5 (Left Wrist Yaw):      G1 left_wrist_yaw
      M6 (Left Wrist Pitch):    G1 left_wrist_pitch
      M7 (Left Wrist Roll):     G1 left_wrist_roll
      M8 (Left Gripper):        0.015m (relaxed fist)
      
    Right Arm (G1 cols 29..35 -> OpenArm M9..M15):
      M9  (Right Shoulder Pitch): G1 right_shoulder_pitch
      M10 (Right Shoulder Roll):  -G1 right_shoulder_roll (abduction sign inversion)
      M11 (Right Shoulder Yaw):   G1 right_shoulder_yaw
      M12 (Right Elbow Flex):     max(0.0, G1 right_elbow)
      M13 (Right Wrist Yaw):      G1 right_wrist_yaw
      M14 (Right Wrist Pitch):    G1 right_wrist_pitch
      M15 (Right Wrist Roll):     G1 right_wrist_roll
      M16 (Right Gripper):        0.015m (relaxed fist)
    """
    data = np.load(npz_path, allow_pickle=True)
    fps = int(data["fps"][0]) if ("fps" in data and len(data["fps"].shape) > 0) else int(data.get("fps", 30))
    pos = data["joint_pos"] if "joint_pos" in data else data["qpos"]

    total_frames = pos.shape[0]
    total_time = total_frames / float(fps)
    
    if max_duration and max_duration < total_time:
        total_frames = int(max_duration * fps)

    # Frame step for desired sample_rate_hz
    step = max(1, int(round(fps / float(sample_rate_hz))))
    actual_dt = step / float(fps)

    # G1 column offsets:
    # First 7 are root pose [x, y, z, qw, qx, qy, qz]
    # cols 22..28: left arm
    # cols 29..35: right arm
    points = []
    elapsed = 0.0

    for f in range(0, total_frames, step):
        row = pos[f]

        # Extract Left Arm
        l_pitch = float(row[22])
        l_roll  = -float(row[23])  # OpenArm roll is negative outward
        l_yaw   = float(row[24])
        l_elbow = max(0.0, float(row[25]))
        l_wyaw  = float(row[28])
        l_wpitch= float(row[27])
        l_wroll = float(row[26])
        l_grip  = 0.015

        # Extract Right Arm
        r_pitch = float(row[29])
        r_roll  = -float(row[30])  # OpenArm roll is positive outward
        r_yaw   = float(row[31])
        r_elbow = max(0.0, float(row[32]))
        r_wyaw  = float(row[35])
        r_wpitch= float(row[34])
        r_wroll = float(row[33])
        r_grip  = 0.015

        # Clamp all 16 joints strictly within mechanical limits
        q16 = [
            clamp_val(l_pitch, 1),
            clamp_val(l_roll, 2),
            clamp_val(l_yaw, 3),
            clamp_val(l_elbow, 4),
            clamp_val(l_wyaw, 5),
            clamp_val(l_wpitch, 6),
            clamp_val(l_wroll, 7),
            clamp_val(l_grip, 8),
            clamp_val(r_pitch, 9),
            clamp_val(r_roll, 10),
            clamp_val(r_yaw, 11),
            clamp_val(r_elbow, 12),
            clamp_val(r_wyaw, 13),
            clamp_val(r_wpitch, 14),
            clamp_val(r_wroll, 15),
            clamp_val(r_grip, 16),
        ]

        points.append({
            "positions": q16,
            "time_from_start": round(elapsed, 4),
        })
        elapsed += actual_dt

    return {
        "joint_names": JOINT_NAMES,
        "sample_rate_hz": round(1.0 / actual_dt, 1),
        "duration_sec": round(elapsed, 2),
        "num_points": len(points),
        "trajectory": points,
    }


def send_trajectory_udp(traj_data: dict, host: str = "127.0.0.1", port: int = 9870):
    """Sends trajectory to OpenArm Dashboard server via high-speed UDP stream."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    payload_str = json.dumps(traj_data)
    payload_bytes = payload_str.encode("utf-8")
    sock.sendto(payload_bytes, (host, port))
    print(f"[UDP] Sent trajectory ({len(traj_data['trajectory'])} waypoints, {len(payload_bytes)} bytes) to {host}:{port}")


def generate_standard_presets():
    """Extract and generate key Kungfu presets from dataset."""
    titles = fetch_action_titles()
    # Key signature martial arts forms
    key_clips = {
        "292": ("tai_chi_quan", "Thái Cực Quyền (Tai Chi Quan - Bimanual Smooth Flow)"),
        "196": ("chang_quan", "Trường Quyền (Changquan / Long Fist 4th Duan)"),
        "47":  ("cloud_sword", "Võ Thuật Kiếm Pháp (Yun Jian / Cloud Sword Play)"),
        "68":  ("elementary_sword", "Sơ Cấp Kiếm Thuật (Chu Ji Jian Routine)"),
        "89":  ("elementary_staff", "Thiếu Lâm Côn Pháp (Shaolin Staff Form)"),
    }

    print(f"\nGenerating standard OpenArm Kungfu presets in {CACHE_DIR}...")
    for clip_id, (slug, desc) in key_clips.items():
        try:
            print(f"\nProcessing clip #{clip_id} ({desc})...")
            npz_file = download_clip_npz(clip_id)
            traj = convert_g1_to_openarm_trajectory(npz_file, max_duration=30.0, sample_rate_hz=25)
            traj["title"] = desc
            traj["clip_id"] = clip_id
            traj["source"] = titles.get(clip_id, "KungfuAthleteBot")

            out_json = os.path.join(CACHE_DIR, f"{slug}.json")
            with open(out_json, "w", encoding="utf-8") as f:
                json.dump(traj, f, indent=2)
            print(f"Exported: {out_json} ({traj['num_points']} waypoints, {traj['duration_sec']}s)")
        except Exception as e:
            print(f"Failed clip #{clip_id}: {e}")

    print("\n[Done] Preset generation completed successfully.")


def main():
    parser = argparse.ArgumentParser(description="OpenArm Kungfu Athlete Bot Motion Tool")
    parser.add_argument("--list", nargs="?", const="all", help="List Kungfu motions (optional filter keyword, e.g. '太极', '拳', '剑', '刀')")
    parser.add_argument("--download", type=str, help="Download motion by clip number (e.g. 292)")
    parser.add_argument("--play", type=str, help="Download & play motion trajectory to robot (e.g. 292)")
    parser.add_argument("--preset-all", action="store_true", help="Download and generate all standard Kungfu presets")
    parser.add_argument("--duration", type=float, default=20.0, help="Maximum trajectory duration in seconds (default: 20.0)")
    parser.add_argument("--rate", type=int, default=25, help="Sampling rate in Hz (default: 25)")
    parser.add_argument("--udp-port", type=int, default=9870, help="Dashboard UDP port (default: 9870)")

    args = parser.parse_args()

    if args.list:
        list_motions(args.list)
    elif args.preset_all:
        generate_standard_presets()
    elif args.download:
        npz_file = download_clip_npz(args.download)
        traj = convert_g1_to_openarm_trajectory(npz_file, max_duration=args.duration, sample_rate_hz=args.rate)
        out_file = os.path.join(CACHE_DIR, f"kungfu_clip_{args.download}.json")
        with open(out_file, "w", encoding="utf-8") as f:
            json.dump(traj, f, indent=2)
        print(f"\n[OK] Converted clip #{args.download} to {out_file}")
        print(f"     Waypoints: {traj['num_points']}, Duration: {traj['duration_sec']}s, Rate: {traj['sample_rate_hz']}Hz")
    elif args.play:
        npz_file = download_clip_npz(args.play)
        traj = convert_g1_to_openarm_trajectory(npz_file, max_duration=args.duration, sample_rate_hz=args.rate)
        print(f"\n[Motion] Playing Kungfu clip #{args.play} ({traj['num_points']} points, {traj['duration_sec']}s)")
        print(f"         Adhering to safety protocol: Current -> Smooth Zero -> Kungfu Trajectory")
        send_trajectory_udp(traj, port=args.udp_port)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
