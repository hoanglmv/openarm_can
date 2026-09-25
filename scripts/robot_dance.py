#!/usr/bin/env python3
"""
OpenArm Robot Dance Choreographer (SDK Utility)
Performs smooth, synchronized bimanual dance routines designed for dancing along with the OpenArm robot.
All movements strictly adhere to OpenArm mechanical joint limits.

Usage Examples:
  # 1. Dance the classic Disco Wave at 100 BPM for 20 seconds:
  python3 scripts/robot_dance.py --dance disco --duration 20

  # 2. Dance Penguin Pop at 110 BPM:
  python3 scripts/robot_dance.py --dance penguin

  # 3. Tai Chi Flow at 60 BPM:
  python3 scripts/robot_dance.py --dance taichi --duration 30

  # 4. List all available dance routines:
  python3 scripts/robot_dance.py --list
"""

import sys
import os
import time
import math
import json
import socket
import argparse
import signal

UDP_IP = "127.0.0.1"
UDP_PORT = 9870

# OpenArm Mechanical Limits (radians & mm)
JOINT_LIMITS = {
    # Left Arm
    1: (-1.3963, 3.4907),  # J1 Shoulder Pitch
    2: (-3.3161, 0.1745),  # J2 Shoulder Roll
    3: (-1.5708, 1.5708),  # J3 Arm Twist
    4: (0.0000, 2.4435),   # J4 Elbow Pitch
    5: (-1.5708, 1.5708),  # J5 Forearm Twist
    6: (-0.7854, 0.7854),  # J6 Wrist Pitch
    7: (-1.5708, 1.5708),  # J7 Wrist Roll
    8: (0.0000, 0.0430),   # Left Gripper (meters)

    # Right Arm
    9:  (-1.3963, 3.4907), # J1 Shoulder Pitch
    10: (-0.1745, 3.3161), # J2 Shoulder Roll
    11: (-1.5708, 1.5708), # J3 Arm Twist
    12: (0.0000, 2.4435),  # J4 Elbow Pitch
    13: (-1.5708, 1.5708), # J5 Forearm Twist
    14: (-0.7854, 0.7854), # J6 Wrist Pitch
    15: (-1.5708, 1.5708), # J7 Wrist Roll
    16: (0.0000, 0.0430),  # Right Gripper (meters)
}

def clamp_joint(motor_id: int, val: float) -> float:
    lim = JOINT_LIMITS.get(motor_id)
    if not lim:
        return val
    return max(lim[0], min(lim[1], val))

# -------------------------------------------------------------------------
# Dance Mathematical Generators
# -------------------------------------------------------------------------
def dance_disco(t: float, bpm: float):
    omega = (2.0 * math.pi * bpm) / 60.0
    l_j1 = 0.50 + 0.32 * math.sin(omega * t)
    r_j1 = 0.50 - 0.32 * math.sin(omega * t)
    l_j2 = -0.30 + 0.12 * math.cos(omega * t * 0.5)
    r_j2 = 0.30 + 0.12 * math.cos(omega * t * 0.5)
    l_j3 = 0.18 * math.sin(omega * t * 0.5)
    r_j3 = -0.18 * math.sin(omega * t * 0.5)
    l_j4 = 1.15 + 0.28 * math.cos(omega * t)
    r_j4 = 1.15 - 0.28 * math.cos(omega * t)
    l_j5 = 0.0
    r_j5 = 0.0
    l_j6 = 0.20 * math.sin(omega * t * 2.0)
    r_j6 = -0.20 * math.sin(omega * t * 2.0)
    l_j7 = 0.35 * math.cos(omega * t)
    r_j7 = -0.35 * math.cos(omega * t)
    beat_pulse = max(0.0, math.sin(omega * t)) ** 4
    grip_l = 0.010 + 0.028 * beat_pulse
    grip_r = 0.010 + 0.028 * beat_pulse
    return [l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7, grip_l,
            r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7, grip_r]

def dance_penguin(t: float, bpm: float):
    omega = (2.0 * math.pi * bpm) / 60.0
    l_j4 = 1.50
    r_j4 = 1.50
    l_j1 = 0.22 + 0.10 * math.sin(omega * t * 0.5)
    r_j1 = 0.22 + 0.10 * math.sin(omega * t * 0.5)
    flap = math.sin(omega * t) ** 2
    l_j2 = -0.30 - 0.35 * flap
    r_j2 = 0.30 + 0.35 * flap
    l_j3 = 0.25 * math.sin(omega * t * 2.0)
    r_j3 = -0.25 * math.sin(omega * t * 2.0)
    l_j5 = 0.0
    r_j5 = 0.0
    l_j6 = 0.30 * math.sin(omega * t)
    r_j6 = 0.30 * math.sin(omega * t)
    l_j7 = 0.0
    r_j7 = 0.0
    grip_l = 0.005 + 0.035 * flap
    grip_r = 0.005 + 0.035 * flap
    return [l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7, grip_l,
            r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7, grip_r]

def dance_ocean(t: float, bpm: float):
    omega = (2.0 * math.pi * bpm) / 60.0
    l_j1 = 0.45 + 0.25 * math.sin(omega * t)
    l_j2 = -0.40 + 0.15 * math.cos(omega * t)
    l_j3 = 0.10 * math.sin(omega * t - 0.4)
    l_j4 = 1.05 + 0.35 * math.sin(omega * t - 0.6)
    l_j5 = 0.15 * math.cos(omega * t - 0.8)
    l_j6 = 0.30 * math.sin(omega * t - 1.0)
    l_j7 = 0.20 * math.cos(omega * t - 1.2)
    grip_l = 0.015 + 0.020 * (0.5 + 0.5 * math.sin(omega * t - 1.2))

    r_phase = omega * t - 2.0
    r_j1 = 0.45 + 0.25 * math.sin(r_phase)
    r_j2 = 0.40 + 0.15 * math.cos(r_phase)
    r_j3 = -0.10 * math.sin(r_phase - 0.4)
    r_j4 = 1.05 + 0.35 * math.sin(r_phase - 0.6)
    r_j5 = -0.15 * math.cos(r_phase - 0.8)
    r_j6 = 0.30 * math.sin(r_phase - 1.0)
    r_j7 = -0.20 * math.cos(r_phase - 1.2)
    grip_r = 0.015 + 0.020 * (0.5 + 0.5 * math.sin(r_phase - 1.2))
    return [l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7, grip_l,
            r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7, grip_r]

def dance_cheer(t: float, bpm: float):
    omega = (2.0 * math.pi * bpm) / 60.0
    l_j1 = 1.15 + 0.15 * math.sin(omega * t * 2.0)
    r_j1 = 1.15 + 0.15 * math.sin(omega * t * 2.0)
    sway = 0.25 * math.sin(omega * t)
    l_j2 = -0.55 + sway
    r_j2 = 0.55 + sway
    l_j3 = 0.0
    r_j3 = 0.0
    l_j4 = 0.70 + 0.20 * math.cos(omega * t * 2.0)
    r_j4 = 0.70 + 0.20 * math.cos(omega * t * 2.0)
    l_j5 = 0.0
    r_j5 = 0.0
    l_j6 = 0.25 * math.sin(omega * t * 4.0)
    r_j6 = -0.25 * math.sin(omega * t * 4.0)
    l_j7 = 0.40 * math.cos(omega * t * 2.0)
    r_j7 = -0.40 * math.cos(omega * t * 2.0)
    grip_l = 0.038 + 0.005 * math.sin(omega * t * 2.0)
    grip_r = 0.038 + 0.005 * math.sin(omega * t * 2.0)
    return [l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7, grip_l,
            r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7, grip_r]

def dance_taichi(t: float, bpm: float):
    omega = (2.0 * math.pi * bpm) / 60.0
    l_j1 = 0.55 + 0.25 * math.sin(omega * t)
    r_j1 = 0.55 - 0.25 * math.sin(omega * t)
    l_j2 = -0.25 - 0.20 * math.cos(omega * t)
    r_j2 = 0.25 + 0.20 * math.cos(omega * t)
    l_j3 = 0.20 * math.sin(omega * t)
    r_j3 = -0.20 * math.sin(omega * t)
    l_j4 = 1.25 - 0.35 * math.sin(omega * t)
    r_j4 = 1.25 + 0.35 * math.sin(omega * t)
    l_j5 = 0.0
    r_j5 = 0.0
    l_j6 = 0.20 * math.cos(omega * t)
    r_j6 = -0.20 * math.cos(omega * t)
    l_j7 = 0.45 * math.sin(omega * t)
    r_j7 = -0.45 * math.sin(omega * t)
    grip_l = 0.022
    grip_r = 0.022
    return [l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7, grip_l,
            r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7, grip_r]

DANCES = {
    "disco": {
        "name": "Disco Wave (Sóng Disco)",
        "default_bpm": 100,
        "func": dance_disco,
        "desc": "Đánh tay so le nhịp 4/4 sôi động, lắc cổ tay và búng kẹp"
    },
    "penguin": {
        "name": "Penguin Pop (Cánh Cụt)",
        "default_bpm": 110,
        "func": dance_penguin,
        "desc": "Khuỷu tay gập vuông, đập cánh nhún nhảy popping robot vui nhộn"
    },
    "ocean": {
        "name": "Ocean Flow (Biển Sóng)",
        "default_bpm": 80,
        "func": dance_ocean,
        "desc": "Chuyển động lượn sóng mềm mại dẻo dai từ trái sang phải"
    },
    "cheer": {
        "name": "Cheer Up (Vũ Điệu Cổ Vũ)",
        "default_bpm": 120,
        "func": dance_cheer,
        "desc": "Hai tay giơ cao chữ V vung lắc cổ vũ reo hò ăn mừng"
    },
    "taichi": {
        "name": "Tai Chi Flow (Thái Cực Quyền)",
        "default_bpm": 60,
        "func": dance_taichi,
        "desc": "Đẩy chưởng và ôm quả cầu thái cực nhu hòa thư thái dưỡng sinh"
    }
}

running = True

def sigint_handler(sig, frame):
    global running
    print("\n🛑 Nhận tín hiệu dừng (Ctrl+C). Đang đưa robot về vị trí nghỉ an toàn...")
    running = False

def main():
    parser = argparse.ArgumentParser(description="OpenArm Robot Dance Choreographer")
    parser.add_argument("--dance", default="disco", choices=list(DANCES.keys()), help="Dance routine to perform")
    parser.add_argument("--bpm", type=float, default=None, help="Tempo BPM (overrides routine default)")
    parser.add_argument("--duration", type=float, default=20.0, help="Duration in seconds (default: 20s, set 0 for infinite)")
    parser.add_argument("--host", default=UDP_IP, help="Target Host IP")
    parser.add_argument("--port", type=int, default=UDP_PORT, help="Target UDP Port")
    parser.add_argument("--list", action="store_true", help="List all available dance routines")

    args = parser.parse_args()

    if args.list:
        print("\n💃 DANH SÁCH CÁC ĐIỆU MÚA ROBOT CÓ SẴN (ROBOT DANCE PRESETS):")
        print("=" * 70)
        for key, d in DANCES.items():
            print(f" • [{key:7s}] {d['name']:32s} (Nhịp: {d['default_bpm']} BPM)")
            print(f"   Mô tả: {d['desc']}")
        print("=" * 70 + "\n")
        sys.exit(0)

    routine = DANCES.get(args.dance)
    bpm = args.bpm if args.bpm else routine["default_bpm"]

    print("\n" + "=" * 65)
    print(f"💃 BẮT ĐẦU VŨ ĐIỆU ROBOT: {routine['name']}")
    print(f"🎵 Nhịp điệu (Tempo): {bpm} BPM")
    print(f"⏱️ Thời lượng: {'Vô tận (bấm Ctrl+C để dừng)' if args.duration <= 0 else f'{args.duration:.1f} giây'}")
    print(f"🛡️ Kiểm soát an toàn: Đã kích hoạt giới hạn cơ học OpenArm")
    print("=" * 65)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    signal.signal(signal.SIGINT, sigint_handler)

    start_time = time.time()
    dt = 0.04  # 25 Hz control loop

    try:
        while running:
            elapsed = time.time() - start_time
            if args.duration > 0 and elapsed >= args.duration:
                print("\n✓ Đã hoàn thành bài múa theo thời lượng đã đặt!")
                break

            # 1. Compute dance pose
            raw_angles = routine["func"](elapsed, bpm)

            # 2. Strict limit clamping
            clamped = [clamp_joint(i + 1, val) for i, val in enumerate(raw_angles)]

            # 3. Transmit UDP frame
            payload = json.dumps({"positions": clamped}).encode('utf-8')
            sock.sendto(payload, (args.host, args.port))

            # 4. Console Beat Tracker
            current_beat = (int(elapsed * (bpm / 60.0)) % 4) + 1
            beat_stars = ["  ", "  ", "  ", "  "]
            beat_stars[current_beat - 1] = "⭐"
            beat_bar = f"[{beat_stars[0]} 1 ] [{beat_stars[1]} 2 ] [{beat_stars[2]} 3 ] [{beat_stars[3]} 4 ]"
            sys.stdout.write(f"\r⏱️ {elapsed:04.1f}s | {beat_bar} | Điệu: {routine['name'][:18]}...")
            sys.stdout.flush()

            time.sleep(dt)

    finally:
        # Gracefully return robot to Home pose
        print("\n\n🏠 Đang đưa các khớp về vị trí nghỉ an toàn (0.0 rad)...")
        home_payload = json.dumps({"positions": [0.0] * 16}).encode('utf-8')
        for _ in range(5):
            sock.sendto(home_payload, (args.host, args.port))
            time.sleep(0.04)
        sock.close()
        print("✓ Hoàn tất. Robot đã về trạng thái an toàn!\n")

if __name__ == "__main__":
    main()
