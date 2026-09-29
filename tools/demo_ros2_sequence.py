#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenArm Bimanual Action Sequence Demo (ROS 2)
Chạy tự động một chuỗi các tác vụ bimanual hoàn chỉnh:
1. Home Pose (Vị trí ban đầu)
2. Ready Pose (Sẵn sàng)
3. Reach & Open Grippers (Vươn 2 tay tới trước & mở kẹp)
4. Grip Object (Đóng chặt 2 kẹp gắp vật thể)
5. Lift Up (Nâng vật thể lên trước ngực)
6. Wings Open (Dang rộng 2 cánh tay chào)
7. Release & Return Home (Nhả kẹp và trở về vị trí an toàn)
"""

import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import JointState
except ImportError:
    print("[ERROR] rclpy không tìm thấy. Hãy source ROS 2: source /opt/ros/<distro>/setup.bash")
    sys.exit(1)

# Danh sách các bước trong chuỗi hành động
ACTION_SEQUENCE = [
    {
        "step": 1,
        "name": "🏠 1. Reset Về Vị Trí Gốc (Home Pose)",
        "duration": 2.5,
        "joints": {
            "name": [
                "left_j1", "left_j2", "left_j3", "left_j4", "left_j5", "left_j6", "left_j7", "left_gripper",
                "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6", "right_j7", "right_gripper"
            ],
            "position": [
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            ]
        }
    },
    {
        "step": 2,
        "name": "⚡ 2. Tư Thế Sẵn Sàng (Ready Pose)",
        "duration": 2.5,
        "joints": {
            "name": [
                "left_j1", "left_j2", "left_j4", "left_gripper",
                "right_j1", "right_j2", "right_j4", "right_gripper"
            ],
            "position": [
                0.20, -0.20, 0.85, 0.025,
                0.20,  0.20, 0.85, 0.025
            ]
        }
    },
    {
        "step": 3,
        "name": "👐 3. Vươn 2 Tay Tới Trước & Mở Kẹp (Reach & Open 40mm)",
        "duration": 3.0,
        "joints": {
            "name": [
                "left_j1", "left_j2", "left_j4", "left_gripper",
                "right_j1", "right_j2", "right_j4", "right_gripper"
            ],
            "position": [
                0.65, -0.05, 0.45, 0.040,
                0.65,  0.05, 0.45, 0.040
            ]
        }
    },
    {
        "step": 4,
        "name": "✊ 4. Kẹp Chặt 2 Tay Lại (Grip / Pick 0mm)",
        "duration": 2.0,
        "joints": {
            "name": ["left_gripper", "right_gripper"],
            "position": [0.0, 0.0]
        }
    },
    {
        "step": 5,
        "name": "📦 5. Nâng Vật Thể Lên Trước Ngực (Lift & Carry Box)",
        "duration": 3.5,
        "joints": {
            "name": [
                "left_j1", "left_j2", "left_j4",
                "right_j1", "right_j2", "right_j4"
            ],
            "position": [
                0.35, -0.10, 1.15,
                0.35,  0.10, 1.15
            ]
        }
    },
    {
        "step": 6,
        "name": "🦅 6. Mở Rộng 2 Cánh Tay Sang Hai Bên (Wings Open / T-Pose)",
        "duration": 3.5,
        "joints": {
            "name": [
                "left_j1", "left_j2", "left_j4", "left_gripper",
                "right_j1", "right_j2", "right_j4", "right_gripper"
            ],
            "position": [
                0.0, -1.20, 0.25, 0.043,
                0.0,  1.20, 0.25, 0.043
            ]
        }
    },
    {
        "step": 7,
        "name": "🏡 7. Thu Tay Về Vị Trí Gốc Hoàn Thành (Return Home)",
        "duration": 3.0,
        "joints": {
            "name": [
                "left_j1", "left_j2", "left_j3", "left_j4", "left_j5", "left_j6", "left_j7", "left_gripper",
                "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6", "right_j7", "right_gripper"
            ],
            "position": [
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            ]
        }
    }
]


def run_sequence():
    rclpy.init()
    node = Node("openarm_demo_sequence_runner")

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    pub = node.create_publisher(JointState, "/openarm/teleop/joint_commands", qos)

    print("\n" + "=" * 65)
    print("🤖 KHỞI ĐỘNG CHUỖI HÀNH ĐỘNG DEMO OPENARM BIMANUAL (7 BƯỚC)")
    print("   Topic: /openarm/teleop/joint_commands (Zero-Latency)")
    print("=" * 65)

    time.sleep(1.0)

    try:
        for item in ACTION_SEQUENCE:
            print(f"\n▶ Đang thực thi {item['name']}...")
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = item["joints"]["name"]
            msg.position = [float(p) for p in item["joints"]["position"]]

            # Bắn lệnh tới robot
            pub.publish(msg)
            print(f"   ✓ Đã gửi {len(msg.position)} góc mục tiêu.")
            print(f"   ⏳ Giữ tư thế trong {item['duration']} giây để robot di chuyển mượt mà...")

            # Đợi với spin để xử lý event ROS 2
            t_start = time.time()
            while time.time() - t_start < item["duration"]:
                rclpy.spin_once(node, timeout_sec=0.05)

        print("\n" + "=" * 65)
        print("🎉 HOÀN THÀNH TOÀN BỘ CHUỖI HÀNH ĐỘNG DEMO THÀNH CÔNG! ✓")
        print("=" * 65 + "\n")

    except KeyboardInterrupt:
        print("\n[!] Người dùng đã dừng chương trình bằng Ctrl+C.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run_sequence()
