#!/usr/bin/env bash
# Copyright 2026 Enactic, Inc. / OpenArm Bimanual Demo Sequence
set -e

TOPIC="/openarm/teleop/joint_commands"

echo "================================================================="
echo "🤖 KHỞI ĐỘNG CHUỖI HÀNH ĐỘNG DEMO OPENARM BIMANUAL (7 BƯỚC)"
echo "   Topic: $TOPIC"
echo "================================================================="

echo ""
echo "▶ [Bước 1/7] 🏠 Reset Về Vị Trí Gốc (Home Pose)..."
ros2 topic pub --once "$TOPIC" sensor_msgs/msg/JointState "{name: ['left_j1', 'left_j2', 'left_j3', 'left_j4', 'left_j5', 'left_j6', 'left_j7', 'left_gripper', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6', 'right_j7', 'right_gripper'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
sleep 2.5

echo ""
echo "▶ [Bước 2/7] ⚡ Tư Thế Sẵn Sàng (Ready Pose)..."
ros2 topic pub --once "$TOPIC" sensor_msgs/msg/JointState "{name: ['left_j1', 'left_j2', 'left_j4', 'left_gripper', 'right_j1', 'right_j2', 'right_j4', 'right_gripper'], position: [0.20, -0.20, 0.85, 0.025, 0.20, 0.20, 0.85, 0.025]}"
sleep 2.5

echo ""
echo "▶ [Bước 3/7] 👐 Vươn 2 Tay Tới Trước & Mở Kẹp 40mm (Reach & Open)..."
ros2 topic pub --once "$TOPIC" sensor_msgs/msg/JointState "{name: ['left_j1', 'left_j2', 'left_j4', 'left_gripper', 'right_j1', 'right_j2', 'right_j4', 'right_gripper'], position: [0.65, -0.05, 0.45, 0.040, 0.65, 0.05, 0.45, 0.040]}"
sleep 3.0

echo ""
echo "▶ [Bước 4/7] ✊ Kẹp Chặt 2 Tay Lại Gắp Vật Thể (Grip / Pick)..."
ros2 topic pub --once "$TOPIC" sensor_msgs/msg/JointState "{name: ['left_gripper', 'right_gripper'], position: [0.0, 0.0]}"
sleep 2.0

echo ""
echo "▶ [Bước 5/7] 📦 Nâng Thùng Hàng Lên Trước Ngực (Lift & Carry Box)..."
ros2 topic pub --once "$TOPIC" sensor_msgs/msg/JointState "{name: ['left_j1', 'left_j2', 'left_j4', 'right_j1', 'right_j2', 'right_j4'], position: [0.35, -0.10, 1.15, 0.35, 0.10, 1.15]}"
sleep 3.5

echo ""
echo "▶ [Bước 6/7] 🦅 Mở Rộng 2 Cánh Tay Sang Hai Bên (Wings Open / T-Pose)..."
ros2 topic pub --once "$TOPIC" sensor_msgs/msg/JointState "{name: ['left_j1', 'left_j2', 'left_j4', 'left_gripper', 'right_j1', 'right_j2', 'right_j4', 'right_gripper'], position: [0.0, -1.20, 0.25, 0.043, 0.0, 1.20, 0.25, 0.043]}"
sleep 3.5

echo ""
echo "▶ [Bước 7/7] 🏡 Thu Tay Về Vị Trí Gốc Hoàn Thành (Return Home)..."
ros2 topic pub --once "$TOPIC" sensor_msgs/msg/JointState "{name: ['left_j1', 'left_j2', 'left_j3', 'left_j4', 'left_j5', 'left_j6', 'left_j7', 'left_gripper', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_gripper'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
sleep 2.5

echo ""
echo "================================================================="
echo "🎉 HOÀN THÀNH TOÀN BỘ CHUỖI HÀNH ĐỘNG DEMO THÀNH CÔNG! ✓"
echo "================================================================="
