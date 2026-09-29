#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenArm Gripper Control Script
Controls opening and closing of the OpenArm gripper (DM4310 motor at CAN ID 0x08 / 0x18).
"""

import sys
import time
import argparse
import openarm_can as oa

def main():
    parser = argparse.ArgumentParser(description="Điều khiển đóng / mở kẹp (Gripper) robot OpenArm")
    parser.add_argument("-i", "--interface", default="can0", help="CAN interface (mặc định: can0)")
    parser.add_argument("--action", choices=["test", "open", "close", "toggle", "interactive"], default="test",
                        help="Hành động: 'test' (chu kỳ đóng/mở tự động), 'open' (mở), 'close' (đóng), 'toggle', 'interactive'")
    parser.add_argument("--open-pos", type=float, default=0.043, help="Hành trình mở tối đa kẹp ngang (m, mặc định: 0.043 m ~ 43 mm)")
    parser.add_argument("--close-pos", type=float, default=0.0, help="Hành trình đóng kẹp ngang (m, mặc định: 0.0 m)")
    parser.add_argument("--speed", type=float, default=10.0, help="Vận tốc đóng mở tối đa (mặc định: 10.0)")
    parser.add_argument("--force", type=float, default=0.15, help="Giới hạn lực kẹp torque_pu (0.0 - 1.0, mặc định: 0.15 an toàn)")
    args = parser.parse_args()

    print("======================================================================")
    print("                OpenArm - Điều Khiển Kẹp Tay Robot (Gripper)          ")
    print("======================================================================")
    print(f"[*] Cổng kết nối   : {args.interface} (CAN-FD)")
    print(f"[*] Động cơ kẹp    : DM4310 (Send ID: 0x08, Recv ID: 0x18)")
    print(f"[*] Chế độ điều khiển: POS_FORCE (Vị trí kèm giới hạn lực)")
    print(f"[*] Giới hạn lực   : {args.force} pu (bảo vệ cơ khí chống kẹp quá tải)")
    print("----------------------------------------------------------------------")

    try:
        arm = oa.OpenArm(args.interface, True)
    except Exception as e:
        print(f"[LỖI] Không thể mở cổng SocketCAN '{args.interface}': {e}")
        print("Gợi ý: Kiểm tra xem adapter USB-CAN đã được gắn vào WSL và chạy 'sudo ip link set can0 up' chưa.")
        sys.exit(1)

    # Khởi tạo duy nhất motor kẹp (Joint 8 / Gripper)
    print("[1] Đang khởi tạo motor kẹp...")
    arm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18, oa.ControlMode.POS_FORCE)

    # Đọc trạng thái ban đầu
    arm.set_callback_mode_all(oa.CallbackMode.STATE)
    arm.refresh_all()
    arm.recv_all(500)

    gripper = arm.get_gripper()
    motor = gripper.get_motor()

    # Kích hoạt motor
    print("[2] Bật kích hoạt động cơ kẹp (enable_all)...")
    arm.enable_all()
    time.sleep(0.05)
    arm.recv_all(500)

    current_pos = motor.get_position()
    cur_mm = (abs(current_pos) / 1.20) * 43.0
    print(f"[i] Vị trí kẹp ban đầu: {cur_mm:.1f} mm ({current_pos:.4f} rad), Nhiệt độ: {motor.get_state_tmos():.1f} °C")

    def move_gripper(target_pos, desc=""):
        # Convert stroke in meters (0.0 .. 0.043) to motor target radians (0.0 .. 1.20 rad)
        if target_pos <= 0.043:
            rad = (target_pos / 0.043) * 1.20
            stroke_mm = target_pos * 1000.0
        else:
            rad = target_pos
            stroke_mm = (abs(target_pos) / 1.20) * 43.0

        print(f"\n--> {desc}: Đưa kẹp về {stroke_mm:.1f} mm ({rad:.3f} rad) (Tốc độ: {args.speed}, Lực: {args.force})...")
        gripper.set_position(rad, speed_rad_s=args.speed, torque_pu=args.force)
        
        # Theo dõi quá trình dịch chuyển trong 1.5 giây
        t_end = time.time() + 1.5
        while time.time() < t_end:
            arm.refresh_all()
            arm.recv_all(200)
            p = motor.get_position()
            v = motor.get_velocity()
            t = motor.get_torque()
            p_mm = (abs(p) / 1.20) * 43.0
            print(f"\r    Hành trình kẹp: {p_mm:>6.1f} mm ({p:>6.3f} rad) | Vận tốc: {v:>7.2f} | Lực phản hồi: {t:>6.2f} Nm", end="", flush=True)
            time.sleep(0.05)
        print(" [Hoàn thành]")

    try:
        if args.action == "test":
            print("\n[*] Bắt đầu bài test chu kỳ ĐÓNG - MỞ kẹp (3 chu kỳ):")
            for cycle in range(1, 4):
                print(f"\n===== Chu kỳ {cycle}/3 =====")
                move_gripper(args.open_pos, desc=f"[Chu kỳ {cycle}] MỞ KẸP (43 mm)")
                time.sleep(0.8)
                move_gripper(args.close_pos, desc=f"[Chu kỳ {cycle}] ĐÓNG KẸP (0 mm)")
                time.sleep(0.8)
            print("\n[✓] Hoàn thành bài test chu kỳ đóng mở kẹp thành công!")

        elif args.action == "open":
            move_gripper(args.open_pos, desc="MỞ KẸP (43 mm)")

        elif args.action == "close":
            move_gripper(args.close_pos, desc="ĐÓNG KẸP (0 mm)")

        elif args.action == "toggle":
            if abs(current_pos - args.close_pos) < 0.010:
                move_gripper(args.open_pos, desc="Chuyển sang MỞ KẸP (43 mm)")
            else:
                move_gripper(args.close_pos, desc="Chuyển sang ĐÓNG KẸP (0 mm)")

        elif args.action == "interactive":
            print("\nChế độ điều khiển bàn phím:")
            print("  - Nhập 'o' hoặc 'open'  : Mở kẹp (43 mm)")
            print("  - Nhập 'c' hoặc 'close' : Đóng kẹp (0 mm)")
            print("  - Nhập một số (mm hoặc m): Đưa kẹp đến hành trình tùy ý (ví dụ: 20 mm hoặc 0.02 m)")
            print("  - Nhập 'q' hoặc 'exit'  : Thoát")
            while True:
                cmd = input("\nLệnh kẹp (o/c/mm/q): ").strip().lower()
                if cmd in ["q", "exit"]:
                    break
                elif cmd in ["o", "open"]:
                    move_gripper(args.open_pos, desc="Mở kẹp")
                elif cmd in ["c", "close"]:
                    move_gripper(args.close_pos, desc="Đóng kẹp")
                else:
                    try:
                        raw_val = float(cmd.replace("mm", "").replace("m", "").strip())
                        val = raw_val / 1000.0 if raw_val > 0.043 else raw_val
                        move_gripper(val, desc=f"Vị trí {val*1000:.1f} mm")
                    except ValueError:
                        print("Lệnh không hợp lệ.")

    except KeyboardInterrupt:
        print("\n\n[!] Nhận tín hiệu dừng từ người dùng (Ctrl+C).")
    finally:
        print("\n[3] Đang tắt động cơ kẹp (disable_all) để bảo vệ động cơ...")
        arm.disable_all()
        arm.recv_all(500)
        print("[✓] Đã tắt động cơ an toàn.")

if __name__ == "__main__":
    main()
