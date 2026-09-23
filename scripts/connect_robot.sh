#!/bin/bash
# ==============================================================================
# OpenArm Robot USB & CAN Bridge for WSL2
# ==============================================================================

set -e

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

echo -e "${BOLD}${BLUE}======================================================================${NC}"
echo -e "${BOLD}${CYAN}          OpenArm Robot USB & CAN Connection Helper (WSL2)            ${NC}"
echo -e "${BOLD}${BLUE}======================================================================${NC}"

# Ensure kernel modules are loaded
echo -e "\n${YELLOW}[1/4] Ensuring Linux kernel modules are loaded...${NC}"
sudo modprobe vhci-hcd 2>/dev/null || true
sudo modprobe can 2>/dev/null || true
sudo modprobe can-raw 2>/dev/null || true
sudo modprobe can-dev 2>/dev/null || true
sudo modprobe gs_usb 2>/dev/null || true
sudo modprobe peak_usb 2>/dev/null || true
sudo modprobe slcan 2>/dev/null || true
echo -e "${GREEN}✓ Kernel modules (vhci-hcd, gs_usb, peak_usb, slcan) ready.${NC}"

# Check usbipd tool
if ! command -v usbipd &> /dev/null; then
    echo -e "${RED}Error: usbipd is not accessible.${NC}"
    exit 1
fi

echo -e "\n${YELLOW}[2/4] Scanning USB devices connected to Windows host...${NC}"
usbipd list

BUSID="${1:-}"

if [ -z "$BUSID" ]; then
    echo ""
    echo -e "${BOLD}Nhập BUSID của thiết bị USB CAN / Robot (ví dụ: 1-2, 1-3, ...):${NC}"
    read -p "BUSID: " BUSID
fi

if [ -z "$BUSID" ]; then
    echo -e "${RED}Không có BUSID được cung cấp. Đang thoát.${NC}"
    exit 1
fi

echo -e "\n${YELLOW}[3/4] Bridging USB device (BUSID: ${BUSID}) to WSL...${NC}"

# Check status of this BUSID
DEVICE_LINE=$(usbipd list | grep -E "^[[:space:]]*${BUSID}[[:space:]]" || true)

if echo "$DEVICE_LINE" | grep -qi "Not shared"; then
    echo -e "${CYAN}Thiết bị chưa được chia sẻ (Not shared). Yêu cầu cấp quyền Admin trên Windows...${NC}"
    echo -e "${YELLOW}Nếu có cửa sổ User Account Control (UAC) hiện lên trên màn hình Windows, hãy bấm YES/ĐỒNG Ý.${NC}"
    powershell.exe -Command "Start-Process 'C:\Program Files\usbipd-win\usbipd.exe' -ArgumentList 'bind --busid $BUSID' -Verb RunAs -Wait"
    sleep 1
fi

echo -e "${CYAN}Attaching device ${BUSID} to WSL...${NC}"
usbipd attach --wsl --busid "$BUSID" || {
    echo -e "${YELLOW}Nếu attach thất bại do chưa bind, thử bind lại bằng Admin...${NC}"
    powershell.exe -Command "Start-Process 'C:\Program Files\usbipd-win\usbipd.exe' -ArgumentList 'bind --busid $BUSID' -Verb RunAs -Wait"
    sleep 1
    usbipd attach --wsl --busid "$BUSID"
}

sleep 2
echo -e "\n${GREEN}✓ Thiết bị đã được kết nối vào WSL!${NC}"
echo -e "${CYAN}Danh sách USB trong WSL (lsusb):${NC}"
lsusb

echo -e "\n${YELLOW}[4/4] Cấu hình giao tiếp SocketCAN...${NC}"

# Detect CAN interfaces
CAN_IFS=$(ip -br link | grep -E "^can[0-9]" | awk '{print $1}')

if [ -z "$CAN_IFS" ]; then
    # Check if a serial device was created (e.g. slcan / ttyACM / ttyUSB)
    SERIAL_DEV=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n 1 || true)
    if [ -n "$SERIAL_DEV" ]; then
        echo -e "${CYAN}Phát hiện thiết bị Serial CAN tại ${SERIAL_DEV}. Đang kích hoạt slcand...${NC}"
        sudo slcand -o -c -s8 "$SERIAL_DEV" can0
        sleep 1
        CAN_IFS="can0"
    fi
fi

if [ -z "$CAN_IFS" ]; then
    echo -e "${YELLOW}Chưa thấy interface can0 xuất hiện tự động.${NC}"
    echo -e "${YELLOW}Kiểm tra lại xem adapter có yêu cầu driver đặc biệt hoặc nguồn robot đã bật chưa.${NC}"
    exit 0
fi

for IF in $CAN_IFS; do
    echo -e "${CYAN}Thiết lập ${IF} sang chuẩn CAN-FD (Nominal: 1Mbps, Data: 5Mbps)...${NC}"
    if command -v openarm-can-cli &> /dev/null; then
        sudo openarm-can-cli -i "$IF" can_configure || true
    else
        sudo ip link set "$IF" down 2>/dev/null || true
        if sudo ip link set "$IF" type can bitrate 1000000 sample-point 0.75 dbitrate 5000000 dsample-point 0.75 dsjw 2 fd on 2>/dev/null; then
            sudo ip link set "$IF" up
            echo -e "${GREEN}✓ ${IF} đã được kích hoạt thành công ở chế độ CAN-FD (1M/5M)!${NC}"
        else
            echo -e "${YELLOW}Không hỗ trợ CAN-FD, đang thử chế độ Classic CAN 1Mbps...${NC}"
            sudo ip link set "$IF" type can bitrate 1000000
            sudo ip link set "$IF" up
            echo -e "${GREEN}✓ ${IF} đã được kích hoạt ở chế độ Classic CAN 1Mbps!${NC}"
        fi
    fi
    ip -details link show "$IF"
done

echo -e "\n${BOLD}${GREEN}======================================================================${NC}"
echo -e "${BOLD}${GREEN}           KẾT NỐI VÀ CẤU HÌNH PHẦN CỨNG HOÀN TẤT!                    ${NC}"
echo -e "${BOLD}${GREEN}======================================================================${NC}"
echo -e "Bạn có thể kiểm tra motor bằng các lệnh sau:"
echo -e " 1. Quét tìm motor trên bus:      ${CYAN}openarm-can-cli -i can0 discover${NC}"
echo -e " 2. Theo dõi trạng thái motor:    ${CYAN}openarm-can-cli -i can0 monitor${NC}"
echo -e " 3. Chạy script test Python:       ${CYAN}python3 scripts/test_robot.py --interface can0${NC}"
echo -e " 4. Ngắt kết nối USB khi xong:     ${CYAN}usbipd detach --busid ${BUSID}${NC}"
