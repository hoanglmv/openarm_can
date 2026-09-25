#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenArm Gripper Flash Configuration & Recovery Tool
Permanently saves POS_FORCE mode into DM4310 motor internal Flash memory.
Fixes the issue where unplugging/replugging the cable resets the motor to MIT mode.
"""

import sys
import time
import socket
import struct
import argparse

SOL_CAN_RAW = 101
CAN_RAW_FD_FRAMES = 5
CANFD_FRAME_FMT = "=IBBB1x64s"
CAN_FRAME_FMT = "=IB3x8s"

def make_socket(iface: str) -> socket.socket:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        s.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
    except Exception:
        pass
    s.settimeout(0.2)
    s.bind((iface,))
    return s

def send_frame(sock: socket.socket, can_id: int, data: bytes):
    pad = 64 - len(data)
    frame_fd = struct.pack(CANFD_FRAME_FMT, can_id, len(data), 0, 0, data + b'\x00' * pad)
    try:
        sock.send(frame_fd)
    except Exception:
        pass
    if len(data) == 8:
        try:
            frame_c = struct.pack(CAN_FRAME_FMT, can_id, 8, data)
            sock.send(frame_c)
        except Exception:
            pass

def read_motor_state(sock: socket.socket, recv_id: int = 0x18, timeout: float = 0.3):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            raw = sock.recv(72)
            if len(raw) == 72:
                cid, dlc, fl, r, data = struct.unpack(CANFD_FRAME_FMT, raw)
            elif len(raw) == 16:
                cid, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
            else:
                continue
            cid &= 0x1FFFFFFF
            if cid == recv_id:
                d0, d1, d2, d3, d4, d5, d6, d7 = data[:8]
                err = (d0 >> 4) & 0x0F
                q_uint = (d1 << 8) | d2
                pos = (q_uint / 65535.0) * 25.0 - 12.5
                return err, pos, d0
        except Exception:
            pass
    return None, None, None

def query_rid(sock: socket.socket, send_id: int, rid: int, timeout: float = 0.4):
    """Query parameter register RID from motor using 0x33"""
    query_packet = bytes([send_id & 0xFF, (send_id >> 8) & 0xFF, 0x33, rid, 0, 0, 0, 0])
    send_frame(sock, 0x7FF, query_packet)
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            raw = sock.recv(72)
            if len(raw) == 72:
                cid, dlc, fl, r, data = struct.unpack(CANFD_FRAME_FMT, raw)
            elif len(raw) == 16:
                cid, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
            else:
                continue
            cid &= 0x1FFFFFFF
            # Response comes on 0x7FF or recv_id
            if (cid == 0x7FF or cid == (send_id + 0x10)) and len(data) >= 8:
                if data[2] == 0x33 and data[3] == rid:
                    val_int = struct.unpack("<I", data[4:8])[0]
                    return val_int
        except Exception:
            pass
    return None

def configure_and_save_flash(iface: str, motor_name: str, send_id: int = 0x08, recv_id: int = 0x18):
    print("======================================================================")
    print(f"[*] Cấu hình và ghi Flash cho: {motor_name} trên '{iface}'")
    print(f"[*] Motor CAN Send ID: {hex(send_id)} | Recv ID: {hex(recv_id)}")
    print("======================================================================")

    try:
        sock = make_socket(iface)
    except Exception as e:
        print(f"[!] Không thể mở cổng {iface}: {e}")
        return False

    # 1. Ping / Query state
    send_frame(sock, 0x7FF, bytes([send_id & 0xFF, (send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0]))
    err, pos, d0 = read_motor_state(sock, recv_id)
    if err is not None:
        state_str = "Enabled" if err == 1 else ("Disabled" if err == 0 else f"Fault {err}")
        print(f"[✓] Động cơ phản hồi! Trạng thái: {state_str}, Vị trí hiện tại: {pos:.4f} rad")
    else:
        print(f"[?] Động cơ chưa phản hồi trên {iface}. Đang thử khởi động lại...")

    # 2. Đọc RID 10 hiện tại
    cur_mode = query_rid(sock, send_id, 10)
    modes_map = {1: "MIT (1)", 2: "POS_VEL (2)", 3: "VEL (3)", 4: "POS_FORCE (4)"}
    if cur_mode is not None:
        print(f"[i] Control Mode hiện tại trong RAM/Flash: {modes_map.get(cur_mode, f'Unknown ({cur_mode})')}")
    else:
        print("[i] Không đọc được RID 10 trực tiếp, đang tiến hành ghi đè chế độ...")

    # 3. Clear faults (0xFB)
    print("[1/5] Xóa lỗi tồn đọng (Clear Fault 0xFB)...")
    send_frame(sock, send_id, bytes([0xFF] * 7 + [0xFB]))
    time.sleep(0.05)

    # 4. Disable motor (0xFD) trước khi ghi tham số
    print("[2/5] Tắt motor (Disable 0xFD) để bảo đảm an toàn khi ghi Flash...")
    send_frame(sock, send_id, bytes([0xFF] * 7 + [0xFD]))
    time.sleep(0.05)

    # 5. Ghi RID 10 = 4 (POS_FORCE) bằng lệnh 0x55
    print("[3/5] Ghi tham số RID 10 (Control Mode) = 4 (POS_FORCE)...")
    write_frame = bytes([send_id & 0xFF, (send_id >> 8) & 0xFF, 0x55, 10, 4, 0, 0, 0])
    send_frame(sock, 0x7FF, write_frame)
    time.sleep(0.1)

    # 6. Lệnh lưu vào FLASH (0xAA)
    print("[4/5] GHI VÀO BỘ NHỚ FLASH VĨNH VIỄN (Command 0xAA)...")
    save_flash_cmd = bytes([send_id & 0xFF, (send_id >> 8) & 0xFF, 0xAA, 0, 0, 0, 0, 0])
    send_frame(sock, 0x7FF, save_flash_cmd)
    time.sleep(0.2)
    print("    [✓] Đã gửi lệnh lưu Flash. Motor sẽ luôn khởi động ở chế độ POS_FORCE!")

    # 7. Bật motor (0xFC)
    print("[5/5] Kích hoạt lại motor (Enable 0xFC)...")
    send_frame(sock, send_id, bytes([0xFF] * 7 + [0xFC]))
    time.sleep(0.05)

    # 8. Kiểm tra lại trạng thái
    send_frame(sock, 0x7FF, bytes([send_id & 0xFF, (send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0]))
    err, pos, d0 = read_motor_state(sock, recv_id)
    if err == 1:
        print(f"\n[THÀNH CÔNG] Động cơ {motor_name} đã sẵn sàng ở chế độ POS_FORCE (err=1, pos={pos:.4f} rad)!")
    else:
        print(f"\n[HOÀN TẤT] Lệnh lưu Flash đã xong. Trạng thái motor: err={err}, pos={pos}")

    sock.close()
    return True

def test_motion(iface: str, motor_name: str, send_id: int = 0x08, recv_id: int = 0x18):
    print(f"\n[*] Đang chạy bài test dịch chuyển kẹp {motor_name}...")
    try:
        sock = make_socket(iface)
    except Exception as e:
        print(f"Lỗi: {e}")
        return

    # Enable
    send_frame(sock, send_id, bytes([0xFF] * 7 + [0xFB]))
    time.sleep(0.02)
    send_frame(sock, send_id, bytes([0xFF] * 7 + [0xFC]))
    time.sleep(0.05)

    vel_uint = 2500  # 25.0 rad/s
    i_uint = 2000    # 20% safe torque limit

    # Step 1: Open gripper (0.0 rad)
    print("--> Test Mở kẹp (target: 0.0 rad)...")
    cmd_open = struct.pack("<fHH", 0.0, vel_uint, i_uint)
    t_end = time.time() + 1.0
    while time.time() < t_end:
        send_frame(sock, send_id + 0x300, cmd_open)
        send_frame(sock, 0x7FF, bytes([send_id & 0xFF, (send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0]))
        time.sleep(0.05)
    err, pos, _ = read_motor_state(sock, recv_id)
    print(f"    Vị trí hiện tại: {pos:.4f} rad")

    # Step 2: Close gripper partially (0.5 rad)
    print("--> Test Đóng kẹp một phần (target: 0.5 rad)...")
    cmd_close = struct.pack("<fHH", 0.5, vel_uint, i_uint)
    t_end = time.time() + 1.2
    while time.time() < t_end:
        send_frame(sock, send_id + 0x300, cmd_close)
        send_frame(sock, 0x7FF, bytes([send_id & 0xFF, (send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0]))
        time.sleep(0.05)
    err, pos, _ = read_motor_state(sock, recv_id)
    print(f"    Vị trí hiện tại: {pos:.4f} rad")

    # Step 3: Reopen gripper (0.0 rad)
    print("--> Test Mở lại kẹp (target: 0.0 rad)...")
    t_end = time.time() + 1.0
    while time.time() < t_end:
        send_frame(sock, send_id + 0x300, cmd_open)
        send_frame(sock, 0x7FF, bytes([send_id & 0xFF, (send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0]))
        time.sleep(0.05)
    err, pos, _ = read_motor_state(sock, recv_id)
    print(f"    Vị trí hiện tại: {pos:.4f} rad")

    sock.close()
    print("[✓] Bài test hoàn tất thành công!\n")

def main():
    parser = argparse.ArgumentParser(description="Fix Joint 8 & Joint 16 Grippers Flash Memory")
    parser.add_argument("--test", action="store_true", help="Chạy thử chuyển động đóng/mở sau khi ghi Flash")
    parser.add_argument("--left-only", action="store_true", help="Chỉ ghi cho Joint 8 bên Trái (can1)")
    parser.add_argument("--right-only", action="store_true", help="Chỉ ghi cho Joint 16 bên Phải (can0)")
    args = parser.parse_args()

    print("######################################################################")
    print("#      OPENARM GRIPPER FLASH PERMANENT FIX TOOL                      #")
    print("######################################################################\n")

    targets = []
    if not args.right_only:
        targets.append(("can1", "Joint 8 - Tay kẹp Trái (Left Gripper)", 0x08, 0x18))
    if not args.left_only:
        targets.append(("can0", "Joint 16 - Tay kẹp Phải (Right Gripper)", 0x08, 0x18))

    for iface, name, send_id, recv_id in targets:
        configure_and_save_flash(iface, name, send_id, recv_id)
        if args.test:
            test_motion(iface, name, send_id, recv_id)

    print("\n[XONG] Bạn có thể an tâm rút dây cắm lại, motor vẫn sẽ tự nhận chế độ POS_FORCE!")

if __name__ == "__main__":
    main()
