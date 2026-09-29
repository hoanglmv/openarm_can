#!/usr/bin/env bash
# ==============================================================================
# Setup OpenArm Dashboard & ACT Model Auto-Start Service on Linux Boot
# ==============================================================================

set -euo pipefail

BOLD="\033[1m"
GREEN="\033[0;32m"
BLUE="\033[0;34m"
YELLOW="\033[1;33m"
CYAN="\033[0;36m"
NC="\033[0m"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SERVICE_SRC="$SCRIPT_DIR/openarm-dashboard.service"
SERVICE_DEST="/etc/systemd/system/openarm-dashboard.service"

echo -e "${BOLD}${BLUE}======================================================================${NC}"
echo -e "${BOLD}${CYAN}   🚀 Setup OpenArm Auto-Start Service (Tự Động Chạy Khi Khởi Động)   ${NC}"
echo -e "${BOLD}${BLUE}======================================================================${NC}"

if [ ! -f "$SERVICE_SRC" ]; then
    echo -e "${YELLOW}[!] Không tìm thấy file $SERVICE_SRC${NC}"
    exit 1
fi

ACTION="${1:-enable}"

case "$ACTION" in
    enable)
        echo -e "-> Đang sao chép file service vào ${GREEN}$SERVICE_DEST${NC}..."
        sudo cp "$SERVICE_SRC" "$SERVICE_DEST"
        sudo systemctl daemon-reload
        echo -e "-> Kích hoạt tự khởi động cùng hệ thống (enable)..."
        sudo systemctl enable openarm-dashboard.service
        echo -e "-> Khởi động service ngay bây giờ (start)..."
        sudo systemctl restart openarm-dashboard.service
        echo -e "${GREEN}✓ Hoàn tất! Dashboard server và Model ACT đã được kích hoạt tự động chạy khi khởi động máy tính.${NC}"
        echo -e "  Kiểm tra trạng thái: ${BOLD}systemctl status openarm-dashboard${NC}"
        echo -e "  Xem logs thời gian thực: ${BOLD}journalctl -u openarm-dashboard -f${NC}"
        ;;
    disable)
        echo -e "-> Tắt tự động khởi động (disable)..."
        sudo systemctl stop openarm-dashboard.service || true
        sudo systemctl disable openarm-dashboard.service || true
        if [ -f "$SERVICE_DEST" ]; then
            sudo rm -f "$SERVICE_DEST"
            sudo systemctl daemon-reload
        fi
        echo -e "${GREEN}✓ Đã hủy tự động khởi động OpenArm Dashboard.${NC}"
        ;;
    status)
        sudo systemctl status openarm-dashboard.service --no-pager || true
        ;;
    *)
        echo "Cách sử dụng: $0 [enable|disable|status]"
        exit 1
        ;;
esac
