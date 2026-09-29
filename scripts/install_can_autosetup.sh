#!/usr/bin/env bash
# ==============================================================================
# OpenArm CAN FD Auto-Setup (Native Ubuntu)
# Cài một lần: udev sẽ tự cấu hình & bật can0/can1 (CAN FD 1M/5M) mỗi khi
# giao diện xuất hiện — lúc boot, khi cắm lại USB, hoặc sau khi reset USB.
#
#   sudo ./scripts/install_can_autosetup.sh            # cài đặt
#   sudo ./scripts/install_can_autosetup.sh --uninstall  # gỡ
# ==============================================================================

set -euo pipefail

HELPER="/usr/local/sbin/openarm-can-up"
RULE="/etc/udev/rules.d/80-openarm-can.rules"

if [ "$(id -u)" -ne 0 ]; then
    echo "Cần chạy với sudo: sudo $0 $*" >&2
    exit 1
fi

if [ "${1:-}" = "--uninstall" ]; then
    rm -f "$HELPER" "$RULE"
    udevadm control --reload-rules
    echo "Đã gỡ OpenArm CAN auto-setup."
    exit 0
fi

cat > "$HELPER" <<'EOF'
#!/bin/sh
# Called by udev with the interface name (e.g. can0). Configures CAN FD and brings it up.
IF="$1"
IP=/usr/sbin/ip
[ -n "$IF" ] || exit 1
$IP link set "$IF" down 2>/dev/null
$IP link set "$IF" type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
$IP link set "$IF" txqueuelen 1000
$IP link set "$IF" up
logger -t openarm-can "$IF configured: CAN FD 1M/5M, restart-ms 100, txqueuelen 1000, UP"
EOF
chmod 755 "$HELPER"

cat > "$RULE" <<EOF
# OpenArm: auto-configure CAN FD interfaces (can0, can1, ...) when they appear
ACTION=="add", SUBSYSTEM=="net", KERNEL=="can[0-9]*", RUN+="$HELPER %k"
EOF

udevadm control --reload-rules

# Apply now to interfaces that already exist
for path in /sys/class/net/can[0-9]*; do
    [ -e "$path" ] || continue
    "$HELPER" "$(basename "$path")"
done

echo "Đã cài OpenArm CAN auto-setup:"
echo "  - $HELPER"
echo "  - $RULE"
ip -br link show type can || true
