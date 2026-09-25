#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenArm Universal Joint State Streamer
Transmits joint states to the OpenArm backend via high-speed UDP (or HTTP/WebSocket).
Can be used by AI models, vision algorithms, teleoperation devices, or test scripts.
"""

import sys
import time
import math
import json
import socket
import argparse
import urllib.request

DEFAULT_JOINTS = {
    # Left Arm
    "openarm_left_joint1": 0.0,
    "openarm_left_joint2": 0.0,
    "openarm_left_joint3": 0.0,
    "openarm_left_joint4": 0.0,
    "openarm_left_joint5": 0.0,
    "openarm_left_joint6": 0.0,
    "openarm_left_joint7": 0.0,
    "left_gripper": 0.0,
    # Right Arm
    "openarm_right_joint1": 0.0,
    "openarm_right_joint2": 0.0,
    "openarm_right_joint3": 0.0,
    "openarm_right_joint4": 0.0,
    "openarm_right_joint5": 0.0,
    "openarm_right_joint6": 0.0,
    "openarm_right_joint7": 0.0,
    "right_gripper": 0.0,
}

def send_udp(payload: dict, host: str = "127.0.0.1", port: int = 9870):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data = json.dumps(payload).encode('utf-8')
    sock.sendto(data, (host, port))
    sock.close()

def send_http(payload: dict, url: str = "http://127.0.0.1:8888/api/joint_state"):
    data = json.dumps(payload).encode('utf-8')
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=1.0) as resp:
        return resp.read()

def run_sine_wave(host: str, port: int, freq_hz: float = 30.0, arm: str = "both"):
    print(f"[*] Đang phát luồng chuyển động hình sin (Sine Wave) ở tần số {freq_hz:.0f} Hz...")
    print(f"    Mục tiêu: {host}:{port} via UDP")
    print("    Bấm Ctrl+C để dừng.")

    dt = 1.0 / freq_hz
    t_start = time.time()
    while True:
        t = time.time() - t_start
        # Gentle smooth wave on shoulder pitch (J1) and elbow (J4)
        j1_angle = 0.3 * math.sin(0.8 * t)
        j4_angle = 0.4 + 0.3 * math.sin(0.8 * t + math.pi/4)
        grip_pos = 0.0215 + 0.020 * math.sin(1.2 * t)

        payload = {"joints": {}}
        if arm in ["left", "both"]:
            payload["joints"]["openarm_left_joint1"] = j1_angle
            payload["joints"]["openarm_left_joint4"] = j4_angle
            payload["joints"]["left_gripper"] = grip_pos

        if arm in ["right", "both"]:
            payload["joints"]["openarm_right_joint1"] = -j1_angle
            payload["joints"]["openarm_right_joint4"] = j4_angle
            payload["joints"]["right_gripper"] = grip_pos

        send_udp(payload, host, port)
        time.sleep(dt)

def run_wave_gesture(host: str, port: int):
    print("[*] Đang thực hiện cử chỉ vẫy tay chào (Wave Gesture)...")
    steps = [
        # Lift arm up
        {"openarm_left_joint1": 0.5, "openarm_left_joint4": 1.2, "openarm_left_joint6": 0.0, "left_gripper": 0.04},
        # Wave left
        {"openarm_left_joint6": -0.4, "left_gripper": 0.04},
        # Wave right
        {"openarm_left_joint6": 0.4, "left_gripper": 0.02},
        # Wave left
        {"openarm_left_joint6": -0.4, "left_gripper": 0.04},
        # Wave right
        {"openarm_left_joint6": 0.4, "left_gripper": 0.02},
        # Return to resting
        {"openarm_left_joint1": 0.0, "openarm_left_joint4": 0.0, "openarm_left_joint6": 0.0, "left_gripper": 0.0}
    ]
    for idx, target in enumerate(steps, 1):
        print(f"  -> Bước {idx}/{len(steps)}: {target}")
        send_udp({"joints": target}, host, port)
        time.sleep(1.2)
    print("[✓] Hoàn thành cử chỉ vẫy tay!")

def main():
    parser = argparse.ArgumentParser(description="OpenArm Universal Joint State Streamer")
    parser.add_argument("--mode", choices=["udp", "http"], default="udp", help="Phương thức gửi (mặc định: udp)")
    parser.add_argument("--ip", default="127.0.0.1", help="Địa chỉ IP của server OpenArm (mặc định: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9870, help="Cổng UDP (mặc định: 9870)")
    parser.add_argument("--demo", choices=["wave", "sine"], default=None, help="Chế độ demo tự động: 'sine' hoặc 'wave'")
    parser.add_argument("--arm", choices=["left", "right", "both"], default="both", help="Tay robot điều khiển")
    parser.add_argument("--json", default=None, help="Gửi một chuỗi JSON góc khớp tùy chỉnh")
    args = parser.parse_args()

    if args.demo == "sine":
        try:
            run_sine_wave(args.ip, args.port, arm=args.arm)
        except KeyboardInterrupt:
            print("\n[!] Dừng luồng sine.")
    elif args.demo == "wave":
        try:
            run_wave_gesture(args.ip, args.port)
        except KeyboardInterrupt:
            print("\n[!] Dừng.")
    elif args.json:
        try:
            payload = json.loads(args.json)
            if args.mode == "udp":
                send_udp(payload, args.ip, args.port)
                print(f"[✓] Đã gửi gói UDP tới {args.ip}:{args.port}")
            else:
                url = f"http://{args.ip}:8888/api/joint_state"
                res = send_http(payload, url)
                print(f"[✓] Đã gửi HTTP POST tới {url}, phản hồi: {res.decode('utf-8')}")
        except Exception as e:
            print(f"[LỖI] Không thể gửi dữ liệu: {e}")
    else:
        # Default: Send a neutral test pose
        print(f"[*] Gửi tư thế cơ bản (Neutral Pose) tới {args.ip}:{args.port} qua UDP...")
        send_udp({"joints": DEFAULT_JOINTS}, args.ip, args.port)
        print("[✓] Đã gửi thành công!")
        print("\nGợi ý sử dụng:")
        print("  python3 scripts/stream_joint_states.py --demo wave")
        print("  python3 scripts/stream_joint_states.py --demo sine --arm left")
        print("  python3 scripts/stream_joint_states.py --json '{\"joints\": {\"openarm_left_joint1\": 0.2, \"left_gripper\": 0.03}}'")

if __name__ == "__main__":
    main()
