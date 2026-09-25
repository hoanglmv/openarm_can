#!/usr/bin/env python3
"""
OpenArm CAN-FD Desired Joint State Sender (SDK Command Utility)
Sends desired target joint states to OpenArm via high-speed UDP or REST HTTP.

Usage Examples:
  # 1. Move Left Arm (in degrees) and close Gripper to 15mm:
  python3 scripts/send_joint_state.py --deg --left 0 -20 0 45 0 10 0 --left-gripper 15

  # 2. Move Both Arms into Ready Pose (in radians):
  python3 scripts/send_joint_state.py --left 0 -0.35 0 0.85 0 0 0 --right 0 0.35 0 0.85 0 0 0

  # 3. Send ROS 2 style JSON dict:
  python3 scripts/send_joint_state.py --json '{"openarm_left_joint1": 0.5, "openarm_left_joint4": 1.2}'

  # 4. Send from a JSON file:
  python3 scripts/send_joint_state.py --file pose.json
"""

import sys
import os
import json
import socket
import argparse
import math
import urllib.request
import urllib.error

UDP_IP = "127.0.0.1"
UDP_PORT = 9870
HTTP_URL = "http://127.0.0.1:8888/api/joint_state"

def deg2rad(val: float) -> float:
    return val * math.pi / 180.0

# OpenArm Mechanical Joint Limits (min, max, name)
JOINT_LIMITS_DEG = {
    # Left Arm
    "left_1": (-80.0, 200.0, "Left J1 (Vai Pitch)"),
    "left_2": (-190.0, 10.0, "Left J2 (Vai Roll)"),
    "left_3": (-90.0, 90.0, "Left J3 (Bắp Xoay)"),
    "left_4": (0.0, 140.0, "Left J4 (Khuỷu Pitch)"),
    "left_5": (-90.0, 90.0, "Left J5 (Cẳng Xoay)"),
    "left_6": (-45.0, 45.0, "Left J6 (Cổ Pitch)"),
    "left_7": (-90.0, 90.0, "Left J7 (Cổ Xoay)"),
    "left_8": (0.0, 43.0, "Left J8 (Kẹp Ngang - mm)"),

    # Right Arm
    "right_1": (-80.0, 200.0, "Right J1 (Vai Pitch)"),
    "right_2": (-10.0, 190.0, "Right J2 (Vai Roll)"),
    "right_3": (-90.0, 90.0, "Right J3 (Bắp Xoay)"),
    "right_4": (0.0, 140.0, "Right J4 (Khuỷu Pitch)"),
    "right_5": (-90.0, 90.0, "Right J5 (Cẳng Xoay)"),
    "right_6": (-45.0, 45.0, "Right J6 (Cổ Pitch)"),
    "right_7": (-90.0, 90.0, "Right J7 (Cổ Xoay)"),
    "right_8": (0.0, 43.0, "Right J8 (Kẹp Ngang - mm)"),
}

JOINT_LIMITS_RAD = {
    k: (deg2rad(v[0]), deg2rad(v[1]), v[2]) if not k.endswith("_8") else (v[0], v[1], v[2])
    for k, v in JOINT_LIMITS_DEG.items()
}

def clamp_and_check(val: float, min_val: float, max_val: float, name: str, unit: str = "rad") -> float:
    if val < min_val:
        print(f"⚠️  [Cảnh Báo Giới Hạn Khớp] {name}: Giá trị {val:.3f}{unit} nhỏ hơn giới hạn dưới [{min_val:.3f}{unit}]. Tự động kẹp an toàn về {min_val:.3f}{unit}!")
        return min_val
    elif val > max_val:
        print(f"⚠️  [Cảnh Báo Giới Hạn Khớp] {name}: Giá trị {val:.3f}{unit} vượt quá giới hạn trên [{max_val:.3f}{unit}]. Tự động kẹp an toàn về {max_val:.3f}{unit}!")
        return max_val
    return val

def validate_payload_limits(payload: dict) -> dict:
    # 1. Names + Positions format
    if "names" in payload and "positions" in payload:
        new_positions = []
        for name, pos in zip(payload["names"], payload["positions"]):
            key = str(name).lower()
            clamped = pos
            for jkey, lim in JOINT_LIMITS_RAD.items():
                arm, num = jkey.split("_")
                if f"{arm}_joint{num}" in key or f"{arm}_j{num}" in key or f"openarm_{arm}_joint{num}" in key or (num == "8" and "finger" in key and arm in key):
                    unit = " mm" if num == "8" else " rad"
                    clamped = clamp_and_check(float(pos), lim[0], lim[1], lim[2], unit=unit)
                    break
            new_positions.append(clamped)
        payload["positions"] = new_positions

    # 2. "left" and "right" lists
    if "left" in payload and isinstance(payload["left"], list):
        payload["left"] = [
            clamp_and_check(float(val), JOINT_LIMITS_RAD[f"left_{idx+1}"][0], JOINT_LIMITS_RAD[f"left_{idx+1}"][1], JOINT_LIMITS_RAD[f"left_{idx+1}"][2], unit=" rad")
            for idx, val in enumerate(payload["left"][:7])
        ]
    if "right" in payload and isinstance(payload["right"], list):
        payload["right"] = [
            clamp_and_check(float(val), JOINT_LIMITS_RAD[f"right_{idx+1}"][0], JOINT_LIMITS_RAD[f"right_{idx+1}"][1], JOINT_LIMITS_RAD[f"right_{idx+1}"][2], unit=" rad")
            for idx, val in enumerate(payload["right"][:7])
        ]

    # 3. Direct dictionary format (e.g. {"openarm_left_joint1": 0.5})
    for k, v in list(payload.items()):
        key = str(k).lower()
        for jkey, lim in JOINT_LIMITS_RAD.items():
            arm, num = jkey.split("_")
            if f"{arm}_joint{num}" in key or f"{arm}_j{num}" in key or f"openarm_{arm}_joint{num}" in key or (num == "8" and "finger" in key and arm in key):
                unit = " mm" if num == "8" else " rad"
                payload[k] = clamp_and_check(float(v), lim[0], lim[1], lim[2], unit=unit)
                break

    return payload

def main():
    parser = argparse.ArgumentParser(
        description="Send desired Joint State to OpenArm SDK with strict Joint Limit validation and safety clamping",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Giới hạn khớp cơ học an toàn (Mechanical Joint Limits):
  Tay Trái (Left Arm):
    J1: [-80.0° ~ +200.0°]  ([-1.40 ~ +3.49] rad)
    J2: [-190.0° ~ +10.0°]  ([-3.32 ~ +0.17] rad)
    J3: [-90.0° ~ +90.0°]   ([-1.57 ~ +1.57] rad)
    J4: [0.0° ~ +140.0°]    ([0.00 ~ +2.44] rad)
    J5: [-90.0° ~ +90.0°]   ([-1.57 ~ +1.57] rad)
    J6: [-45.0° ~ +45.0°]   ([-0.79 ~ +0.79] rad)
    J7: [-90.0° ~ +90.0°]   ([-1.57 ~ +1.57] rad)
    J8: [0.0 ~ 43.0 mm] (Kẹp)

  Tay Phải (Right Arm):
    J1: [-80.0° ~ +200.0°]  ([-1.40 ~ +3.49] rad)
    J2: [-10.0° ~ +190.0°]  ([-0.17 ~ +3.32] rad)
    J3: [-90.0° ~ +90.0°]   ([-1.57 ~ +1.57] rad)
    J4: [0.0° ~ +140.0°]    ([0.00 ~ +2.44] rad)
    J5: [-90.0° ~ +90.0°]   ([-1.57 ~ +1.57] rad)
    J6: [-45.0° ~ +45.0°]   ([-0.79 ~ +0.79] rad)
    J7: [-90.0° ~ +90.0°]   ([-1.57 ~ +1.57] rad)
    J8: [0.0 ~ 43.0 mm] (Kẹp)
        """
    )
    parser.add_argument("--host", default=UDP_IP, help="Target Host IP (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=UDP_PORT, help="UDP Port (default: 9870)")
    parser.add_argument("--http", action="store_true", help="Send via REST HTTP instead of UDP")
    parser.add_argument("--deg", action="store_true", help="Inputs are in degrees instead of radians")

    # Joint angle arrays
    parser.add_argument("--left", nargs="+", type=float, help="Left arm J1..J7 angles")
    parser.add_argument("--right", nargs="+", type=float, help="Right arm J1..J7 angles")
    parser.add_argument("--left-gripper", type=float, help="Left gripper stroke in mm (0.0 to 43.0)")
    parser.add_argument("--right-gripper", type=float, help="Right gripper stroke in mm (0.0 to 43.0)")

    # JSON input options
    parser.add_argument("--json", type=str, help="Raw JSON string of joint states")
    parser.add_argument("--file", type=str, help="Path to JSON file containing joint states")

    args = parser.parse_args()

    payload = {}

    if args.json:
        try:
            payload = json.loads(args.json)
            payload = validate_payload_limits(payload)
        except Exception as e:
            print(f"[Error] Invalid JSON string: {e}")
            sys.exit(1)
    elif args.file:
        try:
            with open(args.file, "r") as f:
                payload = json.load(f)
            payload = validate_payload_limits(payload)
        except Exception as e:
            print(f"[Error] Failed to read JSON file '{args.file}': {e}")
            sys.exit(1)
    else:
        if args.left:
            raw_angles = args.left
            clamped_rads = []
            for idx, raw in enumerate(raw_angles[:7]):
                jid = idx + 1
                lim_deg = JOINT_LIMITS_DEG[f"left_{jid}"]
                lim_rad = JOINT_LIMITS_RAD[f"left_{jid}"]
                if args.deg:
                    clamped_deg = clamp_and_check(raw, lim_deg[0], lim_deg[1], lim_deg[2], unit="°")
                    clamped_rads.append(deg2rad(clamped_deg))
                else:
                    clamped_r = clamp_and_check(raw, lim_rad[0], lim_rad[1], lim_rad[2], unit=" rad")
                    clamped_rads.append(clamped_r)
            payload["left"] = clamped_rads

        if args.right:
            raw_angles = args.right
            clamped_rads = []
            for idx, raw in enumerate(raw_angles[:7]):
                jid = idx + 1
                lim_deg = JOINT_LIMITS_DEG[f"right_{jid}"]
                lim_rad = JOINT_LIMITS_RAD[f"right_{jid}"]
                if args.deg:
                    clamped_deg = clamp_and_check(raw, lim_deg[0], lim_deg[1], lim_deg[2], unit="°")
                    clamped_rads.append(deg2rad(clamped_deg))
                else:
                    clamped_r = clamp_and_check(raw, lim_rad[0], lim_rad[1], lim_rad[2], unit=" rad")
                    clamped_rads.append(clamped_r)
            payload["right"] = clamped_rads

        if args.left_gripper is not None:
            # Gripper mm [0.0, 43.0]
            raw_mm = args.left_gripper if args.left_gripper > 0.043 else (args.left_gripper * 1000.0)
            clamped_mm = clamp_and_check(raw_mm, 0.0, 43.0, "Left Gripper", unit=" mm")
            payload["left_gripper"] = clamped_mm / 1000.0

        if args.right_gripper is not None:
            # Gripper mm [0.0, 43.0]
            raw_mm = args.right_gripper if args.right_gripper > 0.043 else (args.right_gripper * 1000.0)
            clamped_mm = clamp_and_check(raw_mm, 0.0, 43.0, "Right Gripper", unit=" mm")
            payload["right_gripper"] = clamped_mm / 1000.0

    if not payload:
        print("[Error] No joint state provided. Use --left, --right, --json, or --file.")
        parser.print_help()
        sys.exit(1)

    payload_bytes = json.dumps(payload).encode('utf-8')

    if args.http:
        url = f"http://{args.host}:8888/api/joint_state"
        print(f"[SDK Client] Sending desired joint state via HTTP POST to {url}...")
        req = urllib.request.Request(url, data=payload_bytes, headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=3.0) as resp:
                result = resp.read().decode('utf-8')
                print(f"[SDK Client] Server Response: {result}")
                print("✓ Tín hiệu SDK đã gửi thành công! Robot đang chuyển động đến đúng vị trí.")
        except Exception as e:
            print(f"[Error] HTTP request failed: {e}")
            sys.exit(1)
    else:
        print(f"[SDK Client] Sending desired joint state via UDP to {args.host}:{args.port}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(payload_bytes, (args.host, args.port))
            print("✓ Tín hiệu SDK đã gửi thành công qua UDP! Robot đang chuyển động đến đúng vị trí.")
        except Exception as e:
            print(f"[Error] UDP send failed: {e}")
            sys.exit(1)
        finally:
            sock.close()

if __name__ == "__main__":
    main()
