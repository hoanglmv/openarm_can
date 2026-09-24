// ==============================================================================
// OPENARM NETWORK & COMMUNICATIONS SUBSYSTEM
// Handles WebSocket client, USB connection controller, CLI runner, and Toasts
// ==============================================================================

let currentUsbState = false;
let hasInitialRobotSync = false;
let usbActionPending = false;
let usbPendingTimer = null;

// Initialize WebSocket Connection (Port 8889)
function initWebSocket() {
    const wsHost = window.location.hostname || "127.0.0.1";
    const wsUrl = `ws://${wsHost}:8889`;
    ws = new WebSocket(wsUrl);

    const badge = document.getElementById("ws-status-badge");

    ws.onopen = () => {
        if (badge) {
            badge.className = "badge badge-success";
            badge.innerHTML = "WebSocket: <strong>CONNECTED</strong>";
        }
    };

    ws.onmessage = (event) => {
        try {
            const msg = JSON.parse(event.data);
            if (msg.type === "telemetry") {
                const isReal = (msg.data.mode === "real");
                if (isReal && !hasInitialRobotSync) {
                    hasInitialRobotSync = true;
                    syncUiFromRobot(msg.data.motors, true);
                    showToast("success", "✓ Đã tự động đọc góc khớp thực từ Robot vật lý!");
                }
                if (typeof handleTelemetry === "function") {
                    handleTelemetry(msg.data);
                }
            } else if (msg.type === "traffic") {
                if (typeof handleTraffic === "function") {
                    handleTraffic(msg.data);
                }
            } else if (msg.type === "cli_output") {
                handleCliOutput(msg.data);
            } else if (msg.type === "notice") {
                showToast(msg.level || "info", msg.message || "");
                if (msg.mode) {
                    updateUsbUiState(msg.mode === "real");
                }
            }
        } catch (e) {
            console.error("WS Parse Error:", e);
        }
    };

    ws.onclose = () => {
        if (badge) {
            badge.className = "badge badge-danger";
            badge.innerHTML = "WebSocket: <strong>DISCONNECTED</strong>";
        }
        setTimeout(initWebSocket, 2000);
    };

    ws.onerror = () => {
        if (badge) {
            badge.className = "badge badge-danger";
            badge.innerHTML = "WebSocket: <strong>ERROR</strong>";
        }
    };
}

// Send JSON action packet to server via WebSocket
function sendAction(action, payload = {}) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ action, ...payload }));
    }
}

// Run CAN CLI Command
function runCliCommand(cmd) {
    const term = document.getElementById("term-output");
    if (term) term.textContent = `[Executing openarm-can-cli ${cmd} on vcan0...]\n`;
    sendAction("run_cli", { cmd });
}

function handleCliOutput(data) {
    const term = document.getElementById("term-output");
    if (term) term.textContent = data;
}

// Synchronize UI Sliders to match live angles from real robot
function syncUiFromRobot(motors, force = false) {
    if (!motors || motors.length === 0) return;
    motors.forEach(m => {
        const isLeft = (m.arm === "left" || m.id <= 8);
        const jointIndex = (m.joint_idx !== undefined ? m.joint_idx - 1 : ((m.id <= 8 ? m.id : m.id - 8) - 1));
        const armGroup = isLeft ? 'left' : 'right';
        const sliderKey = `${armGroup}-${jointIndex}`;

        const sliderEl = document.getElementById(`slider-${sliderKey}`);
        const dispEl = document.getElementById(`val-disp-${sliderKey}`);

        if (sliderEl && (force || document.activeElement !== sliderEl)) {
            sliderEl.value = m.q;
            if (dispEl) {
                if (jointIndex === 7) {
                    dispEl.textContent = formatGripperText(m.q);
                } else {
                    const deg = (m.q * 180 / Math.PI).toFixed(0);
                    dispEl.textContent = `${m.q.toFixed(2)} rad (${deg}°)`;
                }
            }
        }

        // Top Dual Grippers
        if (jointIndex === 7) {
            const topSlider = document.getElementById(isLeft ? "slider-gripper-left" : "slider-gripper-right");
            const topDisp = document.getElementById(isLeft ? "left-gripper-val-display" : "right-gripper-val-display");
            if (topSlider && (force || document.activeElement !== topSlider)) {
                topSlider.value = m.q;
                if (topDisp) topDisp.textContent = formatGripperText(m.q);
            }
        }
    });
}

// Update USB Connect/Disconnect State in UI
function updateUsbUiState(isReal) {
    currentUsbState = isReal;
    usbActionPending = false;
    if (usbPendingTimer) {
        clearTimeout(usbPendingTimer);
        usbPendingTimer = null;
    }

    if (!isReal) {
        hasInitialRobotSync = false;
    }

    const btnHeader = document.getElementById("btn-usb-toggle");
    const textHeader = document.getElementById("usb-btn-text");
    const btnMaster = document.getElementById("btn-master-usb");
    const ifaceBadge = document.getElementById("iface-name");

    if (btnHeader) btnHeader.classList.remove("btn-usb-busy");
    if (btnMaster) btnMaster.classList.remove("btn-usb-busy");

    if (isReal) {
        if (btnHeader) {
            btnHeader.className = "btn-usb-toggle btn-usb-connected";
            if (textHeader) textHeader.textContent = "Disconnect USB Robot";
            btnHeader.title = "Đang kết nối robot thật (can0/can1). Bấm để ngắt kết nối an toàn.";
        }
        if (btnMaster) {
            btnMaster.className = "btn btn-usb btn-usb-connected";
            btnMaster.textContent = "🔌 Disconnect USB Robot (can0/can1)";
            btnMaster.style.gridColumn = "span 2";
        }
        if (ifaceBadge) ifaceBadge.textContent = "can0 / can1 (REAL)";
    } else {
        if (btnHeader) {
            btnHeader.className = "btn-usb-toggle btn-usb-disconnected";
            if (textHeader) textHeader.textContent = "Connect USB Robot";
            btnHeader.title = "Đang ở chế độ mô phỏng (vcan0). Bấm để kết nối USB Robot thật.";
        }
        if (btnMaster) {
            btnMaster.className = "btn btn-usb btn-usb-disconnected";
            btnMaster.textContent = "⚡ Connect USB Robot (Physical)";
            btnMaster.style.gridColumn = "span 2";
        }
        if (ifaceBadge) ifaceBadge.textContent = "vcan0 (SIM)";
    }
}

// Toggle USB physical connection
function toggleUsbConnection() {
    if (usbActionPending) {
        console.log("USB action already pending, ignoring click");
        return;
    }

    const btnHeader = document.getElementById("btn-usb-toggle");
    const textHeader = document.getElementById("usb-btn-text");
    const btnMaster = document.getElementById("btn-master-usb");

    if (!ws || ws.readyState !== WebSocket.OPEN) {
        showToast("error", "Chưa kết nối được với máy chủ (WebSocket đang tắt). Vui lòng thử lại sau giây lát.");
        return;
    }

    usbActionPending = true;

    // Safety timeout: auto-reset busy state after 8 seconds if no response
    clearTimeout(usbPendingTimer);
    usbPendingTimer = setTimeout(() => {
        if (usbActionPending) {
            usbActionPending = false;
            updateUsbUiState(currentUsbState);
            showToast("warning", "Quá thời gian phản hồi từ USB Robot. Vui lòng kiểm tra lại cáp nối.");
        }
    }, 8000);

    if (currentUsbState) {
        // Currently connected -> Disconnect
        if (textHeader) textHeader.textContent = "Disconnecting USB...";
        if (btnMaster) btnMaster.textContent = "Disconnecting USB...";
        if (btnHeader) btnHeader.classList.add("btn-usb-busy");
        if (btnMaster) btnMaster.classList.add("btn-usb-busy");
        showToast("info", "Đang gửi lệnh ngắt kết nối USB và tắt torque an toàn...");
        sendAction("disconnect_usb");
    } else {
        // Currently disconnected -> Connect
        hasInitialRobotSync = false;
        if (textHeader) textHeader.textContent = "Connecting USB...";
        if (btnMaster) btnMaster.textContent = "Connecting USB...";
        if (btnHeader) btnHeader.classList.add("btn-usb-busy");
        if (btnMaster) btnMaster.classList.add("btn-usb-busy");
        showToast("info", "Đang quét cổng USB và gắn thiết bị vào WSL2...");
        sendAction("connect_usb");
    }
}

// Toast Notification popup
function showToast(level, message) {
    const container = document.getElementById("toast-container");
    if (!container) return;
    const toast = document.createElement("div");
    toast.className = `toast toast-${level}`;
    const icon = level === "success" ? "✓" : level === "error" ? "✕" : level === "warning" ? "⚠" : "ℹ";
    toast.innerHTML = `<span style="font-weight: 800; font-size: 13px;">${icon}</span> <span>${message}</span>`;
    container.appendChild(toast);
    setTimeout(() => {
        toast.style.opacity = "0";
        toast.style.transform = "translateY(-10px)";
        toast.style.transition = "all 0.3s ease";
        setTimeout(() => toast.remove(), 300);
    }, 4500);
}
