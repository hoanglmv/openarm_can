#!/usr/bin/env bash
# ==============================================================================
# OpenArm - Universal UV Environment Setup Script
# Runs on any Linux / WSL machine to configure Python, dependencies, and ROS 2
# ==============================================================================

set -e

# ANSI Color Codes
BOLD="\033[1m"
GREEN="\033[0;32m"
BLUE="\033[0;34m"
YELLOW="\033[1;33m"
RED="\033[0;31m"
CYAN="\033[0;36m"
NC="\033[0m"

echo -e "${BOLD}${BLUE}======================================================================${NC}"
echo -e "${BOLD}${CYAN}   ⚡ OpenArm Universal UV Environment Setup                         ${NC}"
echo -e "${BOLD}${BLUE}======================================================================${NC}"
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

# 1. Check & Install uv if not present
echo -e "${BOLD}${CYAN}[1/5] Kiểm tra công cụ quản lý package 'uv'...${NC}"
if ! command -v uv &> /dev/null; then
    echo -e "  -> Không tìm thấy 'uv'. Đang tự động cài đặt uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
fi

if ! command -v uv &> /dev/null; then
    echo -e "${RED}[Lỗi] Không thể cài đặt uv. Vui lòng cài thủ công: curl -LsSf https://astral.sh/uv/install.sh | sh${NC}"
    exit 1
fi
echo -e "  -> ${GREEN}uv đã sẵn sàng:${NC} $(uv --version)"

# 2. Setup Python 3.12 via uv
echo -e "${BOLD}${CYAN}[2/5] Cài đặt Python 3.12 độc lập qua uv...${NC}"
uv python install 3.12
echo -e "  -> ${GREEN}Python 3.12 đã sẵn sàng!${NC}"

# 3. Detect and Source ROS 2 (if available on system)
echo -e "${BOLD}${CYAN}[3/5] Phát hiện môi trường ROS 2 hệ thống...${NC}"
ROS_DISTRO_FOUND=""
for distro in jazzy humble iron rolling foxy; do
    if [ -f "/opt/ros/$distro/setup.bash" ]; then
        ROS_DISTRO_FOUND="$distro"
        echo -e "  -> Phát hiện ROS 2 ${GREEN}$distro${NC}. Đang nạp setup.bash..."
        # shellcheck disable=SC1090
        source "/opt/ros/$distro/setup.bash"
        break
    fi
done

if [ -z "$ROS_DISTRO_FOUND" ]; then
    echo -e "  -> ${YELLOW}Không tìm thấy ROS 2 trong /opt/ros. Hệ thống vẫn chạy được Web Dashboard & Mô phỏng CAN ảo độc lập.${NC}"
fi

# 4. Create Virtual Environment with system-site-packages
echo -e "${BOLD}${CYAN}[4/5] Khởi tạo / Đồng bộ Virtual Environment (.venv)...${NC}"
if [ ! -d ".venv" ]; then
    echo -e "  -> Tạo .venv với cờ --system-site-packages (kết nối ROS 2)..."
    uv venv --system-site-packages --python 3.12 .venv
else
    echo -e "  -> .venv đã tồn tại. Đảm bảo cấu hình hệ thống..."
fi

# 5. Sync all libraries from pyproject.toml
echo -e "${BOLD}${CYAN}[5/5] Cài đặt tất cả thư viện (uv sync)...${NC}"
uv sync

echo ""
echo -e "${BOLD}${GREEN}======================================================================${NC}"
echo -e "${BOLD}${GREEN}   ✅ CÀI ĐẶT MÔI TRƯỜNG THÀNH CÔNG!                                  ${NC}"
echo -e "${BOLD}${GREEN}======================================================================${NC}"
echo -e "Bạn có thể sử dụng các lệnh sau:"
echo -e "  • Kích hoạt môi trường:  ${CYAN}source .venv/bin/activate${NC}"
echo -e "  • Chạy kiểm thử:         ${CYAN}PYTHONPATH=. uv run pytest -v tests/${NC}"
echo -e "  • Khởi động Dashboard:   ${CYAN}uv run python openarm/dashboard_server.py${NC}"
echo -e "  • Huấn luyện ACT:        ${CYAN}uv run python act_pipeline/train.py --help${NC}"
echo ""
