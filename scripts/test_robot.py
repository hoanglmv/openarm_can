#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenArm Hardware Connection Test Script
Tests communication with Damiao motors (DM8009, DM4340, DM4310) over SocketCAN.
"""

import sys
import time
import argparse
import openarm_can as oa

def main():
    parser = argparse.ArgumentParser(description="Test OpenArm hardware connection via CAN")
    parser.add_argument("-i", "--interface", default="can0", help="CAN interface (default: can0)")
    parser.add_argument("--fd", action="store_true", default=True, help="Use CAN-FD (default: True)")
    parser.add_argument("--no-fd", dest="fd", action="store_false", help="Use Classic CAN")
    parser.add_argument("--side", choices=["right_arm", "left_arm"], default="right_arm", help="Arm side (default: right_arm)")
    parser.add_argument("--mode", choices=["read", "monitor", "gentle_hold"], default="read", 
                        help="Test mode: 'read' (query once), 'monitor' (live stream), 'gentle_hold' (test PD control)")
    args = parser.parse_args()

    print("======================================================================")
    print(f"      OpenArm Hardware Connection Test ({args.interface} | FD={args.fd})")
    print("======================================================================")
    print(f"Khởi tạo OpenArm trên interface: {args.interface} ...")

    try:
        arm = oa.OpenArm(args.interface, args.fd)
    except Exception as e:
        print(f"Lỗi khởi tạo CAN socket trên {args.interface}: {e}")
        print("Hãy đảm bảo interface đã được 'ip link set up' và adapter đã cắm vào WSL.")
        sys.exit(1)

    # Standard OpenArm 7-DOF + 1 Gripper motor mapping
    motor_types = [
        oa.MotorType.DM8009, oa.MotorType.DM8009,  # J1, J2 (Shoulder)
        oa.MotorType.DM4340, oa.MotorType.DM4340,  # J3, J4 (Elbow)
        oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310 # J5, J6, J7 (Wrist)
    ]
    send_ids = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
    recv_ids = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]

    print("Đăng ký cấu trúc động cơ (7 khớp tay + 1 gripper)...")
    arm.init_arm_motors(motor_types, send_ids, recv_ids)
    arm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18)

    print("Gửi lệnh đọc trạng thái (refresh)...")
    arm.set_callback_mode_all(oa.CallbackMode.STATE)
    arm.refresh_all()
    time.sleep(0.05)
    arm.recv_all(500)

    arm_motors = arm.get_arm().get_motors()
    gripper_motors = arm.get_gripper().get_motors()

    print("\n----------------------------------------------------------------------")
    print(" KẾT QUẢ ĐỌC TRẠNG THÁI HIỆN TẠI TỪ CÁC KHỚP:")
    print("----------------------------------------------------------------------")
    print(f"{'Khớp':<10} | {'Send ID':<8} | {'Recv ID':<8} | {'Vị trí (rad)':<15} | {'Vận tốc (rad/s)':<16} | {'Nhiệt độ MOS (°C)'}")
    print("-----------+----------+----------+-----------------+------------------+------------------")

    for idx, m in enumerate(arm_motors):
        pos = m.get_position()
        vel = m.get_velocity()
        temp = m.get_state_tmos()
        print(f"Joint {idx+1:<4} | 0x{m.get_send_can_id():02X}     | 0x{m.get_recv_can_id():02X}     | {pos:>12.4f} rad  | {vel:>12.4f} rad/s | {temp:>8.1f} °C")

    for idx, m in enumerate(gripper_motors):
        pos = m.get_position()
        vel = m.get_velocity()
        temp = m.get_state_tmos()
        print(f"Gripper    | 0x{m.get_send_can_id():02X}     | 0x{m.get_recv_can_id():02X}     | {pos:>12.4f} rad  | {vel:>12.4f} rad/s | {temp:>8.1f} °C")
    print("----------------------------------------------------------------------")

    if args.mode == "monitor":
        print("\nĐang chạy chế độ Live Monitor (Bấm Ctrl+C để dừng)...")
        try:
            while True:
                time.sleep(0.1)
                arm.refresh_all()
                arm.recv_all(100)
                positions = [f"J{i+1}:{m.get_position():+.2f}" for i, m in enumerate(arm_motors)]
                positions.append(f"Grip:{gripper_motors[0].get_position():+.2f}")
                print("\r" + " | ".join(positions), end="", flush=True)
        except KeyboardInterrupt:
            print("\nĐã dừng monitor.")

    elif args.mode == "gentle_hold":
        print("\n[CẢNH BÁO AN TOÀN] Kích hoạt chế độ giữ vị trí nhẹ (Gentle Hold PD Control, kp=20, kd=0.8)...")
        input("Nhấn ENTER để kích hoạt động cơ (đảm bảo không gian an toàn xung quanh robot): ")
        arm.enable_all()
        time.sleep(0.05)
        arm.recv_all(1000)

        initial_arm_q = [m.get_position() for m in arm_motors]
        initial_grip_q = gripper_motors[0].get_position()

        arm_params = [oa.MITParam(20.0, 0.8, q, 0.0, 0.0) for q in initial_arm_q]
        grip_params = [oa.MITParam(5.0, 0.5, initial_grip_q, 0.0, 0.0)]

        print("Đang giữ vị trí trong 5 giây...")
        start_time = time.time()
        try:
            while time.time() - start_time < 5.0:
                arm.get_arm().mit_control_all(arm_params)
                arm.get_gripper().mit_control_all(grip_params)
                arm.recv_all(200)
                time.sleep(0.01)
        finally:
            print("Đang tắt động cơ (disable_all) để đảm bảo an toàn...")
            arm.disable_all()
            arm.recv_all(500)
            print("Động cơ đã được tắt an toàn.")

if __name__ == "__main__":
    main()
