#!/usr/bin/env bash
# ==============================================================================
# OpenArm Bimanual 7-DOF System Setup Script for Windows WSL2 (Ubuntu Environment)
# Authors: Enactic, Inc. & OpenArm Project Team
# ==============================================================================

set -euo pipefail

# ANSI Color Codes
BOLD="\033[1m"
GREEN="\033[0;32m"
BLUE="\033[0;34m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
CYAN="\033[0;36m"
MAGENTA="\033[0;35m"
NC="\033[0m"

echo -e "${BOLD}${BLUE}======================================================================${NC}"
echo -e "${BOLD}${CYAN}   🚀 OpenArm Dual 7-DOF System Setup - Windows WSL2 (Ubuntu)        ${NC}"
echo -e "${BOLD}${BLUE}======================================================================${NC}"
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

# ------------------------------------------------------------------------------
# 1. Verify WSL2 Environment
# ------------------------------------------------------------------------------
echo -e "${BOLD}${CYAN}[1/7] Kiểm tra môi trường Windows Subsystem for Linux (WSL2)...${NC}"
if [ -f /proc/version ] && grep -qi "microsoft" /proc/version; then
    echo -e "  -> Phát hiện: ${GREEN}Môi trường WSL2 Linux Kernel $(uname -r)${NC}"
else
    echo -e "  -> ${YELLOW}Không phát hiện nhân Microsoft WSL2. Nếu bạn đang chạy Ubuntu thuần (Native Linux), vui lòng chạy: ./scripts/setup_ubuntu.sh${NC}"
    read -r -p "Bạn có muốn tiếp tục cài đặt trên môi trường hiện tại không? [y/N]: " choice
    if [[ "$choice" != "y" && "$choice" != "Y" ]]; then
        echo "Đã hủy cài đặt."
        exit 0
    fi
fi

# Request sudo upfront
echo -e "${BOLD}${CYAN}[2/7] Yêu cầu quyền quản trị sudo để cài đặt gói hệ thống...${NC}"
sudo -v
while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done 2>/dev/null &

# ------------------------------------------------------------------------------
# 2. Check usbipd-win on Windows Host
# ------------------------------------------------------------------------------
echo -e "${BOLD}${CYAN}[3/7] Kiểm tra công cụ chuyển tiếp phần cứng USB (usbipd-win)...${NC}"

USBIPD_PATH=""
if command -v usbipd.exe >/dev/null 2>&1; then
    USBIPD_PATH="$(command -v usbipd.exe)"
elif [ -f "/mnt/c/Program Files/usbipd-win/usbipd.exe" ]; then
    USBIPD_PATH="/mnt/c/Program Files/usbipd-win/usbipd.exe"
fi

if [ -n "$USBIPD_PATH" ]; then
    echo -e "  -> ${GREEN}[✓] Đã phát hiện usbipd-win tại: ${USBIPD_PATH}${NC}"
else
    echo -e "  -> ${YELLOW}[!] Chưa phát hiện usbipd-win trên Windows host.${NC}"
    echo -e "     Để kết nối phần cứng PEAK CAN USB hoặc Camera RealSense từ Windows vào WSL2:"
    echo -e "     Vui lòng mở ${BOLD}PowerShell (Run as Administrator)${NC} trên Windows và chạy:"
    echo -e "     ${MAGENTA}winget install --interactive --exact dorssel.usbipd-win${NC}"
    echo -e "     (Bạn vẫn có thể tiếp tục cài đặt để chạy chế độ Mô phỏng 3D bình thường)."
fi

# ------------------------------------------------------------------------------
# 3. Install System Packages via APT
# ------------------------------------------------------------------------------
echo -e "${BOLD}${CYAN}[4/7] Cài đặt công cụ lập trình C++, Python & CAN Utilities...${NC}"
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
    hwdata \
    udev

echo -e "  -> ${GREEN}Cài đặt APT packages thành công!${NC}"

# ------------------------------------------------------------------------------
# 4. Configure CAN Kernel Modules & SocketCAN for WSL2
# ------------------------------------------------------------------------------
echo -e "${BOLD}${CYAN}[5/7] Nạp Kernel Modules CAN & Virtual CAN (vcan0) trong WSL2...${NC}"

# Load modules required for USB-over-IP and SocketCAN
for mod in vhci-hcd can can_raw can_dev vcan peak_usb; do
    if sudo modprobe "$mod" 2>/dev/null; then
        echo -e "  -> Nạp module: ${GREEN}$mod${NC}"
    else
        echo -e "  -> ${YELLOW}Module $mod có thể đã tích hợp sẵn trong nhân WSL2.${NC}"
    fi
done

# Persist modules
echo -e "# OpenArm CAN Kernel Modules for WSL2\nvhci-hcd\ncan\ncan_raw\ncan_dev\nvcan\npeak_usb" | sudo tee /etc/modules-load.d/openarm-can.conf >/dev/null

# Configure vcan0 (Virtual CAN) for smooth 3D simulation preview
if ! ip link show vcan0 >/dev/null 2>&1; then
    sudo ip link add dev vcan0 type vcan
fi
sudo ip link set up vcan0
echo -e "  -> Giao diện ${GREEN}vcan0${NC} (Mô phỏng 3D) đã sẵn sàng: ${GREEN}UP${NC}"

# Configure sudoers rule so Web UI 1-Click "Connect USB Robot" button works without password
SUDOERS_FILE="/etc/sudoers.d/openarm-can"
if [ ! -f "$SUDOERS_FILE" ]; then
    echo -e "# Allow current user to control CAN network interfaces without password prompt for OpenArm Web UI\n$USER ALL=(ALL) NOPASSWD: /usr/sbin/ip link set can* *, /usr/sbin/ip link set vcan* *" | sudo tee "$SUDOERS_FILE" >/dev/null
    sudo chmod 0440 "$SUDOERS_FILE"
    echo -e "  -> Đã cấp quyền tự động gắn CAN 1-Click: ${GREEN}$SUDOERS_FILE${NC}"
fi

# ------------------------------------------------------------------------------
# 5. Setup Python Virtual Environment (.venv) & Dependencies
# ------------------------------------------------------------------------------
echo -e "${BOLD}${CYAN}[6/7] Thiết lập môi trường Python (.venv) & Cài đặt thư viện AI...${NC}"

if [ ! -d ".venv" ]; then
    echo -e "  -> Đang khởi tạo Python virtual environment tại ${GREEN}.venv${NC}..."
    python3 -m venv .venv
fi

# Activate virtualenv
# shellcheck disable=SC1091
source .venv/bin/activate
echo -e "  -> Đang sử dụng Python: ${GREEN}$(which python3)${NC}"

pip install --upgrade pip setuptools wheel

# Check GPU availability inside WSL2
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    echo -e "  -> Phát hiện ${GREEN}NVIDIA GPU qua WSL2 vGPU${NC}. Cài đặt PyTorch CUDA..."
    pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
else
    echo -e "  -> Chạy chế độ CPU. Cài đặt ${YELLOW}PyTorch CPU${NC} (tối ưu dung lượng và tốc độ)..."
    pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
fi

if [ -f "requirements.txt" ]; then
    echo -e "  -> Cài đặt thư viện từ requirements.txt..."
    pip install -r requirements.txt
fi

pip install websockets pytest

echo -e "  -> ${GREEN}Cài đặt môi trường Python hoàn tất!${NC}"

# ------------------------------------------------------------------------------
# 6. Build C++ Core Library & Install Python SDK (openarm_can)
# ------------------------------------------------------------------------------
echo -e "${BOLD}${CYAN}[7/7] Biên dịch Thư viện C++ & Cài đặt Python SDK (openarm_can)...${NC}"

mkdir -p build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -GNinja
cmake --build build -j"$(nproc)"
sudo cmake --install build
sudo ldconfig

echo -e "  -> Cài đặt Python bindings (openarm_can)..."
pip install ./python

echo -e "  -> ${GREEN}Biên dịch C++ và cài đặt openarm_can thành công!${NC}"

# Verify
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
echo -e "${BOLD}${GREEN}   🎉 THIẾT LẬP OPENARM TRÊN WSL2 THÀNH CÔNG!                         ${NC}"
echo -e "${BOLD}${GREEN}======================================================================${NC}"
echo ""
echo -e "${BOLD}Để khởi động hệ thống OpenArm Digital Twin & Điều khiển Robot:${NC}"
echo -e "  1. Kích hoạt môi trường:  ${CYAN}source .venv/bin/activate${NC}"
echo -e "  2. Khởi chạy Dashboard:   ${CYAN}python3 sim/server.py${NC}"
echo -e "  3. Mở trình duyệt Web:    ${CYAN}http://localhost:8888${NC}"
echo ""
echo -e "${BOLD}Hướng dẫn kết nối Robot thật trên Windows / WSL2:${NC}"
echo -e "  - Cắm dây USB PEAK CAN vào máy tính Windows."
echo -e "  - Mở giao diện Web ${CYAN}http://localhost:8888${NC}, bấm nút ${GREEN}'Connect USB Robot'${NC} trên góc phải."
echo -e "  - Hệ thống sẽ tự động gọi usbipd để gắn thiết bị vào WSL2 và kích hoạt can0/can1!"
echo ""
