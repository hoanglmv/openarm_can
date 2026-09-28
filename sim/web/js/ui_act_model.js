// ==============================================================================
// OPENARM AI MODEL INFERENCE & FUTURE TRAJECTORY UI CONTROLLER (ACT)
// Wires the AI Model control panel, execution modes (3D Simulation Preview vs Real Hardware),
// live trajectory horizon streaming, and WebSocket command dispatch.
// ==============================================================================

let actInferenceState = {
    running: false,
    mode: "preview", // 'preview' (safe 3D sim) or 'hardware' (real CAN bus)
    isLoaded: false,
    isSynthetic: false,
    latencyMs: 0,
    hz: 0,
    stepCount: 0,
    maxDeltaQ: 0,
    futureActions: [],
};

function setupActModelUI() {
    // 1. Hook Model Load Button
    const btnLoad = document.getElementById("btn-act-load");
    const inputCkpt = document.getElementById("input-act-checkpoint");
    const selectDevice = document.getElementById("select-act-device");

    if (btnLoad) {
        btnLoad.addEventListener("click", () => {
            const ckpt = inputCkpt ? inputCkpt.value.trim() : "checkpoints/best_checkpoint.pth";
            const device = selectDevice ? selectDevice.value : "cuda";

            updateActStatusBadge("loading", "ĐANG NẠP...");
            showToast("info", `Đang nạp file checkpoint: ${ckpt}...`);

            sendWsMessage({
                action: "model_load",
                checkpoint: ckpt,
                device: device,
            });
        });
    }

    // 2. Hook Start / Stop / Step Buttons & Execution Mode Radios
    const btnStart = document.getElementById("btn-act-start");
    const btnStop = document.getElementById("btn-act-stop");
    const btnStep = document.getElementById("btn-act-step");
    const sliderSpeed = document.getElementById("slider-act-speed");

    // Dynamic active styling for the 3 mode labels
    const modeRadios = document.querySelectorAll('input[name="act-mode-radio"]');
    modeRadios.forEach((radio) => {
        radio.addEventListener("change", () => {
            document.querySelectorAll(".act-mode-option").forEach((opt) => opt.classList.remove("active"));
            const parentLabel = radio.closest(".act-mode-option");
            if (parentLabel) parentLabel.classList.add("active");

            // Sync with global execution mode button in left control panel
            const globalMode = radio.value === "preview" ? "sim" : "dual";
            if (typeof setGlobalExecutionMode === "function") {
                setGlobalExecutionMode(globalMode, false);
            }
        });
    });

    const getSelectedControlMode = () => {
        const checked = document.querySelector('input[name="act-mode-radio"]:checked');
        return checked ? checked.value : "preview";
    };

    if (btnStart) {
        btnStart.addEventListener("click", () => {
            const controlMode = getSelectedControlMode();
            const velScale = sliderSpeed ? parseFloat(sliderSpeed.value) : 1.0;
            const ckpt = inputCkpt ? inputCkpt.value.trim() : "";

            if (controlMode === "dual") {
                const confirmed = confirm(
                    "CẢNH BÁO AN TOÀN PHẦN CỨNG:\n" +
                    "Bạn đang chọn chế độ 'CHẠY CẢ 2 (MÔ PHỎNG & ROBOT THẬT)'.\n" +
                    "Hệ thống sẽ đồng thời mô phỏng 3D và bơm xung điều khiển xuống 16 động cơ CAN bus!\n" +
                    "Hãy chắc chắn không gian xung quanh robot an toàn. Tiếp tục?"
                );
                if (!confirmed) return;
            }
            // If controlMode === "preview" (Chỉ mô phỏng), runs immediately without warning prompt

            sendWsMessage({
                action: "model_start",
                control_mode: controlMode,
                vel_scale: velScale,
                ensemble_m: 0.01,
                checkpoint: ckpt,
            });
        });
    }

    if (btnStop) {
        btnStop.addEventListener("click", () => {
            sendWsMessage({ action: "model_stop" });
        });
    }

    if (btnStep) {
        btnStep.addEventListener("click", () => {
            const controlMode = getSelectedControlMode();
            sendWsMessage({
                action: "model_step",
                control_mode: controlMode,
            });
        });
    }

    // 3. Hook Speed Slider
    if (sliderSpeed) {
        const dispSpeed = document.getElementById("disp-act-speed");
        sliderSpeed.addEventListener("input", (e) => {
            if (dispSpeed) dispSpeed.textContent = `${parseFloat(e.target.value).toFixed(2)}x`;
        });
    }

    // 4. Hook 3D Path Checkboxes & Horizon
    const checkShowPath = document.getElementById("check-show-future-path");
    const checkLeftPath = document.getElementById("check-show-left-path");
    const checkRightPath = document.getElementById("check-show-right-path");
    const sliderHorizon = document.getElementById("slider-path-horizon");
    const dispHorizon = document.getElementById("disp-path-horizon");

    if (checkShowPath) {
        checkShowPath.addEventListener("change", (e) => {
            setFuturePathVisibility(e.target.checked);
            updateViewportPathBtn(e.target.checked);
        });
    }

    if (checkLeftPath) {
        checkLeftPath.addEventListener("change", (e) => {
            setLeftPathVisibility(e.target.checked);
        });
    }

    if (checkRightPath) {
        checkRightPath.addEventListener("change", (e) => {
            setRightPathVisibility(e.target.checked);
        });
    }

    if (sliderHorizon) {
        sliderHorizon.addEventListener("input", (e) => {
            const val = parseInt(e.target.value, 10);
            if (dispHorizon) dispHorizon.textContent = `${val} bước (Horizon ${(val * 0.02).toFixed(2)}s)`;
            setPathHorizonSteps(val);
        });
    }

    // 5. Hook 3D Viewport Toolbar Button
    const btnTogglePath = document.getElementById("btn-toggle-future-path");
    if (btnTogglePath) {
        btnTogglePath.addEventListener("click", () => {
            showFuturePath = !showFuturePath;
            setFuturePathVisibility(showFuturePath);
            updateViewportPathBtn(showFuturePath);
            if (checkShowPath) checkShowPath.checked = showFuturePath;
        });
    }

    // 6. Hook Presets / Suggested Checkpoint Chips
    document.querySelectorAll(".btn-ckpt-chip").forEach((chip) => {
        chip.addEventListener("click", () => {
            const path = chip.getAttribute("data-path");
            if (inputCkpt && path) {
                inputCkpt.value = path;
                showToast("info", `Đã chọn checkpoint: ${path}`);
            }
        });
    });

    console.log("[INIT] OpenArm ACT AI Model UI subsystem initialized.");
}

function updateViewportPathBtn(active) {
    const btn = document.getElementById("btn-toggle-future-path");
    if (btn) {
        btn.textContent = `AI Path: ${active ? "ON" : "OFF"}`;
        btn.classList.toggle("active", active);
    }
}

function updateActStatusBadge(state, label) {
    const badge = document.getElementById("act-model-status-badge");
    if (!badge) return;
    badge.className = `badge badge-${state}`;
    badge.textContent = label;
}

/**
 * Handle incoming telemetry packet with inference telemetry from WebSocket.
 */
function handleActInferenceTelemetry(infData) {
    if (!infData) return;

    actInferenceState.running = Boolean(infData.running);
    actInferenceState.mode = infData.mode || "preview";
    actInferenceState.isLoaded = Boolean(infData.is_loaded);
    actInferenceState.isSynthetic = Boolean(infData.is_synthetic);
    actInferenceState.latencyMs = infData.latency_ms || 0;
    actInferenceState.hz = infData.hz || 0;
    actInferenceState.stepCount = infData.step_count || 0;
    actInferenceState.maxDeltaQ = infData.max_delta_q || 0;

    // Update Status Badge & Cards
    if (actInferenceState.running) {
        if (actInferenceState.mode === "hardware_only") {
            updateActStatusBadge("warning", "CHỈ ROBOT THẬT (CAN)");
        } else if (actInferenceState.mode === "dual") {
            updateActStatusBadge("success", "CHẠY CẢ 2 (3D + ROBOT)");
        } else {
            updateActStatusBadge("info", "CHỈ MÔ PHỎNG 3D (AN TOÀN)");
        }
    } else if (actInferenceState.isLoaded) {
        updateActStatusBadge(
            actInferenceState.isSynthetic ? "accent" : "primary",
            actInferenceState.isSynthetic ? "PREVIEW (MOCK)" : "SẴN SÀNG (TORCH)"
        );
    } else {
        updateActStatusBadge("idle", "CHƯA NẠP");
    }

    // Update Telemetry Metrics Counters
    const elLatency = document.getElementById("disp-act-latency");
    const elHz = document.getElementById("disp-act-hz");
    const elSteps = document.getElementById("disp-act-steps");
    const elDelta = document.getElementById("disp-act-delta");

    if (elLatency) elLatency.textContent = `${actInferenceState.latencyMs.toFixed(1)} ms`;
    if (elHz) elHz.textContent = `${actInferenceState.hz.toFixed(1)} FPS`;
    if (elSteps) elSteps.textContent = `${actInferenceState.stepCount}`;
    if (elDelta) elDelta.textContent = `${actInferenceState.maxDeltaQ.toFixed(4)} rad`;

    // Metadata Display
    if (infData.metadata) {
        const elArch = document.getElementById("disp-act-arch");
        const elCkpt = document.getElementById("disp-act-ckpt");
        const elChunk = document.getElementById("disp-act-chunk");

        if (elArch) elArch.textContent = infData.metadata.architecture || "ACT CVAE";
        if (elCkpt) elCkpt.textContent = infData.metadata.checkpoint || "--";
        if (elChunk) elChunk.textContent = `${infData.metadata.chunk_size || 50} steps (1.0s)`;
    }

    // Update 3D Future Action Path
    if (infData.future_actions && Array.isArray(infData.future_actions)) {
        actInferenceState.futureActions = infData.future_actions;
        if (typeof updateFutureActionPath === "function") {
            updateFutureActionPath(infData.future_actions);
        }

        // If in 3D Preview simulation or Dual mode, preview motion on Three.js robot
        if (actInferenceState.running && (actInferenceState.mode === "preview" || actInferenceState.mode === "dual") && infData.future_actions.length > 0) {
            const step0 = infData.future_actions[0];
            if (step0 && step0.length >= 16) {
                for (let i = 0; i < 16; i++) {
                    const motorId = i + 1;
                    const motorVal = step0[i];
                    if (typeof updateOpenArmJointVisual === "function") {
                        updateOpenArmJointVisual({ id: motorId, q: motorVal });
                    }
                }
            }
        }
    }
}
