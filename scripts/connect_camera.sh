#!/bin/bash
# ==============================================================================
# OpenArm RealSense Camera USB Bridge for WSL2
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

echo -e "${BOLD}${CYAN}======================================================================${NC}"
echo -e "${BOLD}${CYAN}          OpenArm RealSense Camera Connection Helper (WSL2)           ${NC}"
echo -e "${BOLD}${CYAN}======================================================================${NC}"

# 1. Scan only connected devices that have a valid Bus ID (format: X-Y)
DEVICE_LINE=$(usbipd list 2>/dev/null | grep -E "^[0-9]+-[0-9]+" | grep -Ei "8086:0b3a|RealSense" | head -n 1 || true)

if [ -z "$DEVICE_LINE" ]; then
    # Check if device is in Persisted list (was known but currently unplugged)
    PERSISTED_CHECK=$(usbipd list 2>/dev/null | grep -Ei "8086:0b3a|RealSense" || true)
    echo -e "${RED}[ERROR] Không tìm thấy camera RealSense D435i đang cắm trên cổng USB!${NC}"
    if [ -n "$PERSISTED_CHECK" ]; then
        echo -e "${YELLOW}>> Thiết bị đã được cấp quyền trước đó, nhưng hiện tại cáp USB ĐANG RÚT hoặc cổng USB chưa nhận diện.${NC}"
    fi
    echo -e "${CYAN}>> Vui lòng cắm lại cáp USB của camera vào cổng USB 3.0 (hoặc Type-C) rồi chạy lại lệnh này.${NC}\n"
    echo -e "${BOLD}Danh sách thiết bị USB đang cắm trên máy tính:${NC}"
    usbipd list 2>/dev/null || true
    exit 1
fi

BUSID=$(echo "$DEVICE_LINE" | awk '{print $1}')
echo -e "${GREEN}✓ Phát hiện Intel RealSense D435i đang cắm tại Bus ID: ${BOLD}${BUSID}${NC}"

# 2. Check if already attached
if echo "$DEVICE_LINE" | grep -qi "Attached"; then
    echo -e "${GREEN}✓ Camera đã được gắn vào WSL2 rồi!${NC}"
    exit 0
fi

# 3. Bind if not shared
if echo "$DEVICE_LINE" | grep -qi "Not shared"; then
    echo -e "\n${CYAN}Thiết bị chưa được chia sẻ (Not shared). Đang yêu cầu quyền Admin trên Windows...${NC}"
    echo -e "${YELLOW}>>> Hãy bấm YES/ĐỒNG Ý nếu màn hình Windows hiện thông báo User Account Control (UAC)! <<<${NC}"
    powershell.exe -Command "Start-Process 'C:\Program Files\usbipd-win\usbipd.exe' -ArgumentList 'bind --busid $BUSID' -Verb RunAs -Wait"
    sleep 1
fi

# 4. Attach with auto-attach
echo -e "\n${CYAN}Gắn camera (Bus ID: ${BUSID}) vào WSL2 với chế độ auto-attach...${NC}"
usbipd attach --wsl --busid "$BUSID" --auto-attach || {
    echo -e "${YELLOW}Thử cấp quyền bind lại qua Admin...${NC}"
    powershell.exe -Command "Start-Process 'C:\Program Files\usbipd-win\usbipd.exe' -ArgumentList 'bind --busid $BUSID' -Verb RunAs -Wait"
    sleep 1
    usbipd attach --wsl --busid "$BUSID" --auto-attach
}

echo -e "\n${GREEN}======================================================================${NC}"
echo -e "${GREEN}✓ Camera RealSense D435i đã được kết nối thành công vào WSL2!${NC}"
echo -e "${CYAN}Kiểm tra thiết bị bên trong WSL2:${NC}"
lsusb | grep -Ei "8086|Intel|RealSense" || true
echo -e "${GREEN}======================================================================${NC}"
