#!/usr/bin/env bash
# ==============================================================================
# OpenArm Bimanual 7-DOF System Setup Script for Native Ubuntu (22.04 / 24.04 LTS)
# Authors: Enactic, Inc. & OpenArm Project Team
# ==============================================================================

set -euo pipefail

# ANSI Color Codes for Rich Terminal Output
BOLD="\033[1m"
GREEN="\033[0;32m"
BLUE="\033[0;34m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
CYAN="\033[0;36m"
NC="\033[0m" # No Color

echo -e "${BOLD}${BLUE}======================================================================${NC}"
echo -e "${BOLD}${CYAN}   🚀 OpenArm Dual 7-DOF System Setup - Native Ubuntu Linux          ${NC}"
echo -e "${BOLD}${BLUE}======================================================================${NC}"
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

# ------------------------------------------------------------------------------
# 1. Check OS Compatibility
# ------------------------------------------------------------------------------
echo -e "${BOLD}${CYAN}[1/7] Kiểm tra hệ điều hành...${NC}"
if [ -f /etc/os-release ]; then
    . /etc/os-release
    echo -e "  -> Phát hiện: ${GREEN}${PRETTY_NAME}${NC}"
    if [[ "$ID" != "ubuntu" && "$ID_LIKE" != *"ubuntu"* && "$ID_LIKE" != *"debian"* ]]; then
        echo -e "${YELLOW}  [Cảnh báo] Hệ điều hành không phải Ubuntu/Debian chính thức. Script sẽ tiếp tục nhưng có thể cần chỉnh sửa.${NC}"
    fi
else
    echo -e "${RED}  [Lỗi] Không tìm thấy /etc/os-release.${NC}"
    exit 1
fi

# Request sudo upfront
echo -e "${BOLD}${CYAN}[2/7] Yêu cầu quyền quản trị sudo để cài đặt gói hệ thống...${NC}"
sudo -v
# Keep-alive sudo until script finishes
while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done 2>/dev/null &

# ------------------------------------------------------------------------------
# 2. Install System Packages via APT
# ------------------------------------------------------------------------------
echo -e "${BOLD}${CYAN}[3/7] Cài đặt công cụ lập trình C++, Python & CAN Utilities...${NC}"
sudo apt-get update -y
sudo apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    pkg-config \
    libusb-1.0-0-dev \
    can-utils \
    iproute2 \
    net-tools \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    udev

echo -e "  -> ${GREEN}Cài đặt APT packages thành công!${NC}"

# ------------------------------------------------------------------------------
# 3. Configure CAN Kernel Modules & SocketCAN
# ------------------------------------------------------------------------------
echo -e "${BOLD}${CYAN}[4/7] Cấu hình Kernel Modules CAN & Virtual CAN (vcan0)...${NC}"

# Load modules immediately
for mod in can can_raw can_dev vcan peak_usb; do
    if sudo modprobe "$mod" 2>/dev/null; then
        echo -e "  -> Nạp module: ${GREEN}$mod${NC}"
    else
        echo -e "  -> ${YELLOW}Module $mod có thể đã tích hợp sẵn trong kernel.${NC}"
    fi
done

# Persist modules across system reboots
echo -e "# OpenArm CAN Kernel Modules\ncan\ncan_raw\ncan_dev\nvcan\npeak_usb" | sudo tee /etc/modules-load.d/openarm-can.conf >/dev/null
echo -e "  -> Đã lưu cấu hình tự nạp module vào: ${GREEN}/etc/modules-load.d/openarm-can.conf${NC}"

# Configure and bring up virtual CAN (vcan0) for safe 3D simulation
if ! ip link show vcan0 >/dev/null 2>&1; then
    sudo ip link add dev vcan0 type vcan
fi
sudo ip link set up vcan0
echo -e "  -> Giao diện ${GREEN}vcan0${NC} (Mô phỏng 3D) đã sẵn sàng: ${GREEN}UP${NC}"

# Configure sudoers rule so Web UI can hotplug/up/down CAN without password prompt
SUDOERS_FILE="/etc/sudoers.d/openarm-can"
if [ ! -f "$SUDOERS_FILE" ]; then
    echo -e "# Allow current user to control CAN network interfaces without password prompt for OpenArm Web UI\n$USER ALL=(ALL) NOPASSWD: /usr/sbin/ip link set can* *, /usr/sbin/ip link set vcan* *" | sudo tee "$SUDOERS_FILE" >/dev/null
    sudo chmod 0440 "$SUDOERS_FILE"
    echo -e "  -> Đã cấp quyền tự động chuyển đổi CAN 1-Click: ${GREEN}$SUDOERS_FILE${NC}"
fi

# ------------------------------------------------------------------------------
# 4. Setup Python Virtual Environment (.venv) & Dependencies
# ------------------------------------------------------------------------------
echo -e "${BOLD}${CYAN}[5/7] Thiết lập môi trường Python (.venv) & Cài đặt thư viện AI...${NC}"

if [ ! -d ".venv" ]; then
    echo -e "  -> Đang khởi tạo Python virtual environment tại ${GREEN}.venv${NC}..."
    python3 -m venv .venv
fi

# Activate virtualenv
# shellcheck disable=SC1091
source .venv/bin/activate
echo -e "  -> Đang sử dụng Python: ${GREEN}$(which python3)${NC}"

# Upgrade pip and packaging tools
pip install --upgrade pip setuptools wheel

# Check GPU availability for PyTorch
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    echo -e "  -> Phát hiện ${GREEN}NVIDIA GPU${NC}. Cài đặt PyTorch với hỗ trợ CUDA..."
    pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
else
    echo -e "  -> Không tìm thấy GPU NVIDIA. Cài đặt ${YELLOW}PyTorch CPU${NC} (tối ưu dung lượng và tốc độ)..."
    pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
fi

# Install other requirements
if [ -f "requirements.txt" ]; then
    echo -e "  -> Cài đặt thư viện từ requirements.txt..."
    pip install -r requirements.txt
fi

# Ensure websockets, pytest, and ament-lint are installed for testing & server
pip install websockets pytest

echo -e "  -> ${GREEN}Cài đặt môi trường Python hoàn tất!${NC}"

# ------------------------------------------------------------------------------
# 5. Build C++ Core Library & Install Python SDK (openarm_can)
# ------------------------------------------------------------------------------
echo -e "${BOLD}${CYAN}[6/7] Biên dịch Thư viện C++ & Cài đặt Python SDK (openarm_can)...${NC}"

mkdir -p build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -GNinja
cmake --build build -j"$(nproc)"
sudo cmake --install build
sudo ldconfig

echo -e "  -> Cài đặt Python bindings (openarm_can)..."
pip install ./python

echo -e "  -> ${GREEN}Biên dịch C++ và cài đặt openarm_can thành công!${NC}"

# ------------------------------------------------------------------------------
# 6. Verify Installation
# ------------------------------------------------------------------------------
echo -e "${BOLD}${CYAN}[7/7] Kiểm tra hoạt động hệ thống...${NC}"

python3 -c "
import openarm_can as oa
print(f'  [✓] openarm_can SDK: Sẵn sàng (File: {oa.__file__})')
"

python3 -c "
import torch
dev = 'CUDA (' + torch.cuda.get_device_name(0) + ')' if torch.cuda.is_available() else 'CPU'
print(f'  [✓] PyTorch: Sẵn sàng trên thiết bị {dev}')
"

echo -e "  [✓] Virtual CAN vcan0: $(ip link show vcan0 | grep -o 'state [A-Z]*')"

echo ""
echo -e "${BOLD}${GREEN}======================================================================${NC}"
echo -e "${BOLD}${GREEN}   🎉 THIẾT LẬP HỆ THỐNG OPENARM THÀNH CÔNG!                          ${NC}"
echo -e "${BOLD}${GREEN}======================================================================${NC}"
echo ""
echo -e "${BOLD}Để khởi động hệ thống OpenArm Digital Twin & Điều khiển Robot:${NC}"
echo -e "  1. Kích hoạt môi trường:  ${CYAN}source .venv/bin/activate${NC}"
echo -e "  2. Khởi chạy Dashboard:   ${CYAN}python3 sim/server.py${NC}"
echo -e "  3. Mở trình duyệt Web:    ${CYAN}http://localhost:8888${NC}"
echo ""
echo -e "${BOLD}Lưu ý khi cắm Robot thật qua PEAK CAN USB:${NC}"
echo -e "  - Bấm nút ${GREEN}'Connect USB Robot'${NC} trên thanh tiêu đề Web Dashboard để tự động kết nối."
echo -e "  - Hoặc chạy script: ${CYAN}./scripts/connect_robot.sh${NC}"
echo ""
