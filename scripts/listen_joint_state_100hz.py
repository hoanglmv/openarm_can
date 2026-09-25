#!/usr/bin/env python3
"""
OpenArm 100Hz Joint State Stream Listener
Subscribes to the live 100Hz Joint State stream published by OpenArm Server on UDP port 9871.

Usage:
  python3 scripts/listen_joint_state_100hz.py
  python3 scripts/listen_joint_state_100hz.py --port 9871 --hz
  python3 scripts/listen_joint_state_100hz.py --record my_recording.csv
"""

import sys
import os
import time
import json
import socket
import argparse

def main():
    parser = argparse.ArgumentParser(description="OpenArm 100Hz Real-Time Joint State Stream Listener")
    parser.add_argument("--port", type=int, default=9871, help="UDP listening port (default: 9871)")
    parser.add_argument("--host", default="0.0.0.0", help="Binding host IP (default: 0.0.0.0)")
    parser.add_argument("--hz", action="store_true", help="Display only frequency and sample count summary")
    parser.add_argument("--count", type=int, default=0, help="Stop after receiving N packets (0 = infinite)")
    parser.add_argument("--record", type=str, default=None, help="Save received stream to a custom CSV file")

    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        sock.bind((args.host, args.port))
    except Exception as e:
        print(f"[Error] Failed to bind to {args.host}:{args.port}: {e}")
        sys.exit(1)

    print("=" * 70)
    print(f"📡 ĐANG LẮNG NGHE LUỒNG JOINT STATE 100Hz TỪ ROBOT...")
    print(f"Cổng UDP: {args.port} | Địa chỉ: {args.host}")
    if args.count > 0:
        print(f"Giới hạn mẫu nhận: {args.count} gói tin")
    if args.record:
        print(f"Lưu vào file: {args.record}")
    print("=" * 70)

    record_file = None
    if args.record:
        record_file = open(args.record, "w", buffering=1024*64)
        header = ["timestamp", "seq", "rel_time"] + [f"q_{i}" for i in range(1, 17)] + [f"dq_{i}" for i in range(1, 17)]
        record_file.write(",".join(header) + "\n")

    count = 0
    t0 = time.perf_counter()
    last_print = time.perf_counter()
    calc_hz = 0.0

    try:
        while True:
            data, addr = sock.recvfrom(4096)
            now = time.perf_counter()
            count += 1

            dt = now - t0
            t0 = now
            if dt > 0:
                calc_hz = calc_hz * 0.9 + (1.0 / dt) * 0.1

            try:
                pkt = json.loads(data.decode('utf-8'))
            except Exception:
                continue

            if record_file:
                row = [str(pkt.get("timestamp", 0)), str(pkt.get("seq", 0)), str(pkt.get("rel_time", 0))]
                row.extend(str(x) for x in pkt.get("positions", []))
                row.extend(str(x) for x in pkt.get("velocities", []))
                record_file.write(",".join(row) + "\n")

            if now - last_print >= 0.2:  # update display at 5 Hz
                last_print = now
                pos = pkt.get("positions", [0]*16)
                l_q = [f"{x:+.2f}" for x in pos[:4]]
                r_q = [f"{x:+.2f}" for x in pos[8:12]]
                sys.stdout.write(
                    f"\r⚡ [100Hz STREAM] Tần số: {calc_hz:5.1f} Hz | Gói: {count:6d} | L1..4: {l_q} | R1..4: {r_q} "
                )
                sys.stdout.flush()

            if args.count > 0 and count >= args.count:
                print(f"\n✓ Đã nhận đủ {count} gói tin. Tần số đạt: {calc_hz:.1f} Hz.")
                break

    except KeyboardInterrupt:
        print("\n\n⏹ Đã dừng lắng nghe.")
    finally:
        if record_file:
            record_file.flush()
            record_file.close()
            print(f"✓ Đã lưu file: {args.record}")
        sock.close()

if __name__ == "__main__":
    main()
