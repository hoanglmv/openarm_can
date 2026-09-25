// ==============================================================================
// OPENARM ROBOT DANCE STUDIO & MOTION PRESETS SUBSYSTEM
// Manages Dance Player State Machine, Beat Synchronizer, and Basic Poses
// ==============================================================================

let danceState = "idle"; // "idle" | "dancing" | "paused"
let currentDanceKey = "disco";
let currentDanceBpm = 100;
let isDanceLoop = true;
let danceTime = 0.0;
let danceTimer = null;

let activePreset = null;
let presetTimer = null;

function updateBeatVisualizer(beatIndex) {
    const beatDots = [
        document.getElementById("beat-dot-1"),
        document.getElementById("beat-dot-2"),
        document.getElementById("beat-dot-3"),
        document.getElementById("beat-dot-4")
    ];

    beatDots.forEach((dot, idx) => {
        if (!dot) return;
        if (idx === (beatIndex - 1)) {
            dot.classList.add("beat-active");
        } else {
            dot.classList.remove("beat-active");
        }
    });
}

function formatTimer(sec) {
    const m = Math.floor(sec / 60);
    const s = Math.floor(sec % 60);
    return `${m < 10 ? '0' : ''}${m}:${s < 10 ? '0' : ''}${s}`;
}

function tickDance() {
    danceTime += 0.05; // 50ms tick
    const routine = DANCE_ROUTINES[currentDanceKey] || DANCE_ROUTINES.disco;

    // 1. Calculate raw mathematical angles
    const rawPose = routine.calcPose(danceTime, currentDanceBpm);

    // 2. Strict safety clamping against OpenArm Mechanical Limits
    const safePositions = [];
    for (let i = 1; i <= 16; i++) {
        const rawVal = rawPose[i - 1];
        const lim = MECHANICAL_LIMITS[i];
        if (!lim) {
            safePositions.push(rawVal);
            continue;
        }
        if (lim.isGripper) {
            // Gripper stroke in meters: clamp between [0.0, 0.043 m]
            const clampedGrip = Math.max(0.0, Math.min(0.043, rawVal));
            safePositions.push(clampedGrip);
        } else {
            // Arm joints in radians: clamp between minRad and maxRad
            const clampedRad = Math.max(lim.minRad, Math.min(lim.maxRad, rawVal));
            safePositions.push(clampedRad);
        }
    }

    // 3. Dispatch safe joint target to backend & simulator via set_joint_state
    sendAction("set_joint_state", { positions: safePositions });

    // 4. Update Beat Indicator (4/4 measure)
    const currentBeat = (Math.floor(danceTime * (currentDanceBpm / 60)) % 4) + 1;
    updateBeatVisualizer(currentBeat);

    // 5. Update Timer & Step Display
    const danceTimerDisplay = document.getElementById("dance-timer-display");
    if (danceTimerDisplay) {
        danceTimerDisplay.textContent = formatTimer(danceTime);
    }
    const danceStepDisplay = document.getElementById("dance-step-display");
    if (danceStepDisplay) {
        const cycleNum = Math.floor(danceTime * (currentDanceBpm / 60) / 4) + 1;
        danceStepDisplay.textContent = `Chu kỳ ${cycleNum} • Phách ${currentBeat}/4 • ${currentDanceBpm} BPM • Giới hạn an toàn OK`;
    }
}

function startDance() {
    stopPresets(); // stop any other basic preset
    sendAction("enable_all");

    danceState = "dancing";
    const danceBadgeStatus = document.getElementById("dance-badge-status");
    if (danceBadgeStatus) {
        danceBadgeStatus.className = "dance-live-badge badge-dancing";
        danceBadgeStatus.textContent = "ĐANG MÚA";
    }
    const routine = DANCE_ROUTINES[currentDanceKey] || DANCE_ROUTINES.disco;
    const danceTitlePlaying = document.getElementById("dance-title-playing");
    if (danceTitlePlaying) {
        danceTitlePlaying.textContent = `Đang múa: ${routine.name} • Hãy nhảy cùng robot!`;
    }

    const btnDancePlay = document.getElementById("btn-dance-play");
    const btnDancePause = document.getElementById("btn-dance-pause");
    const btnDanceStop = document.getElementById("btn-dance-stop");
    if (btnDancePlay) btnDancePlay.disabled = true;
    if (btnDancePause) btnDancePause.disabled = false;
    if (btnDanceStop) btnDanceStop.disabled = false;

    if (danceTimer) clearInterval(danceTimer);
    danceTimer = setInterval(tickDance, 50);

    showToast("success", `💃 Bắt đầu bài múa: ${routine.name}!`);
}

function pauseDance() {
    if (danceTimer) {
        clearInterval(danceTimer);
        danceTimer = null;
    }
    danceState = "paused";
    const danceBadgeStatus = document.getElementById("dance-badge-status");
    if (danceBadgeStatus) {
        danceBadgeStatus.className = "dance-live-badge badge-paused";
        danceBadgeStatus.textContent = "TẠM DỪNG";
    }
    const btnDancePlay = document.getElementById("btn-dance-play");
    const btnDancePause = document.getElementById("btn-dance-pause");
    if (btnDancePlay) {
        btnDancePlay.disabled = false;
        btnDancePlay.textContent = "▶ Tiếp Tục Múa";
    }
    if (btnDancePause) btnDancePause.disabled = true;

    showToast("info", "⏸ Đã tạm dừng điệu múa!");
}

function stopDance(returnHome = true) {
    if (danceTimer) {
        clearInterval(danceTimer);
        danceTimer = null;
    }
    danceState = "idle";
    danceTime = 0.0;

    const danceBadgeStatus = document.getElementById("dance-badge-status");
    if (danceBadgeStatus) {
        danceBadgeStatus.className = "dance-live-badge badge-idle";
        danceBadgeStatus.textContent = "SẴN SÀNG";
    }
    const btnDancePlay = document.getElementById("btn-dance-play");
    const btnDancePause = document.getElementById("btn-dance-pause");
    if (btnDancePlay) {
        btnDancePlay.disabled = false;
        btnDancePlay.textContent = "▶ Bắt Đầu Múa";
    }
    if (btnDancePause) btnDancePause.disabled = true;

    const danceTimerDisplay = document.getElementById("dance-timer-display");
    if (danceTimerDisplay) danceTimerDisplay.textContent = "00:00";
    const danceStepDisplay = document.getElementById("dance-step-display");
    if (danceStepDisplay) danceStepDisplay.textContent = "Nhịp 4/4 • Tự động giữ khoảng an toàn cơ học OpenArm";

    updateBeatVisualizer(0); // clear active beat

    if (returnHome) {
        // Smoothly send robot to Home pose (0 rad)
        const homePose = new Array(16).fill(0.0);
        sendAction("set_joint_state", { positions: homePose });
        showToast("info", "⏹ Đã dừng bài múa. Robot trở về vị trí nghỉ an toàn.");
    }
}

// Initialize Dance Studio UI listeners
function initDanceEngine() {
    const btnDancePlay = document.getElementById("btn-dance-play");
    const btnDancePause = document.getElementById("btn-dance-pause");
    const btnDanceStop = document.getElementById("btn-dance-stop");
    const danceLoopCheckbox = document.getElementById("dance-loop-checkbox");

    if (btnDancePlay) btnDancePlay.addEventListener("click", startDance);
    if (btnDancePause) btnDancePause.addEventListener("click", pauseDance);
    if (btnDanceStop) btnDanceStop.addEventListener("click", () => stopDance(true));

    if (danceLoopCheckbox) {
        danceLoopCheckbox.addEventListener("change", (e) => {
            isDanceLoop = e.target.checked;
            showToast("info", isDanceLoop ? "🔁 Đã BẬT lặp lại bài múa liên tục" : "Đã TẮT lặp lại (múa 1 chu kỳ)");
        });
    }

    // Dance Cards Click Handlers
    document.querySelectorAll(".dance-card").forEach(card => {
        card.addEventListener("click", () => {
            const danceKey = card.dataset.dance;
            if (!DANCE_ROUTINES[danceKey]) return;

            currentDanceKey = danceKey;
            const routine = DANCE_ROUTINES[danceKey];

            // Update active card styling
            document.querySelectorAll(".dance-card").forEach(c => {
                c.classList.remove("active");
                const marker = c.querySelector(".dance-select-marker");
                if (marker) marker.textContent = "Chọn điệu này";
            });
            card.classList.add("active");
            const activeMarker = card.querySelector(".dance-select-marker");
            if (activeMarker) activeMarker.textContent = "✓ Đang chọn";

            const danceTitlePlaying = document.getElementById("dance-title-playing");
            if (danceTitlePlaying) {
                danceTitlePlaying.textContent = danceState === "dancing"
                    ? `Đang múa: ${routine.name} • Hãy nhảy cùng robot!`
                    : `Đã chọn: ${routine.name} (${routine.desc})`;
            }

            showToast("info", `🎵 Đã chọn: ${routine.name}`);
        });
    });

    // Tempo Buttons Click Handlers
    document.querySelectorAll(".btn-tempo-opt").forEach(btn => {
        btn.addEventListener("click", () => {
            const bpm = parseInt(btn.dataset.bpm) || 100;
            currentDanceBpm = bpm;

            document.querySelectorAll(".btn-tempo-opt").forEach(b => b.classList.remove("active"));
            btn.classList.add("active");

            showToast("info", `⏱️ Nhịp điệu: ${bpm} BPM`);
        });
    });

    // Sub-Tab Switcher (Dance Studio vs Basic Poses)
    const tabDanceMode = document.getElementById("tab-dance-mode");
    const tabPosesMode = document.getElementById("tab-poses-mode");
    const subDanceSection = document.getElementById("sub-dance-section");
    const subPosesSection = document.getElementById("sub-poses-section");

    if (tabDanceMode && tabPosesMode) {
        tabDanceMode.addEventListener("click", () => {
            tabDanceMode.classList.add("active");
            tabPosesMode.classList.remove("active");
            if (subDanceSection) subDanceSection.style.display = "flex";
            if (subPosesSection) subPosesSection.style.display = "none";
        });
        tabPosesMode.addEventListener("click", () => {
            tabPosesMode.classList.add("active");
            tabDanceMode.classList.remove("active");
            if (subDanceSection) subDanceSection.style.display = "none";
            if (subPosesSection) subPosesSection.style.display = "flex";
        });
    }
}

// ----------------------------------------------------
// BASIC BIMANUAL POSES & TRAJECTORIES
// ----------------------------------------------------
function stopPresets() {
    if (presetTimer) {
        clearInterval(presetTimer);
        presetTimer = null;
    }
    activePreset = null;
    document.querySelectorAll(".btn-preset").forEach(b => b.classList.remove("active"));
}

function triggerPreset(preset) {
    // If dance is active, stop it
    if (danceState !== "idle") {
        stopDance(false);
    }
    stopPresets();
    activePreset = preset;

    if (preset === "stop") {
        return;
    }

    sendAction("enable_all");

    if (preset === "home") {
        // Natural resting pose: arms hang down vertically beside pillar (0 rad)
        for (let id = 1; id <= 16; id++) {
            const isGripper = (id === 8 || id === 16);
            if (isGripper) {
                sendAction("set_gripper", { id, pos: 0.0 });
            } else {
                sendAction("set_mit", { id, q: 0.0, kp: 35.0, kd: 1.2, tau: 0.0 });
            }
        }
    } else if (preset === "ready") {
        // Ready stance: Both arms raised forward at chest height
        [1, 9].forEach(baseId => {
            const isLeftArm = (baseId === 1);
            sendAction("set_mit", { id: baseId + 0, q: 0.35 });
            sendAction("set_mit", { id: baseId + 1, q: isLeftArm ? -0.20 : 0.20 });
            sendAction("set_mit", { id: baseId + 2, q: 0.00 });
            sendAction("set_mit", { id: baseId + 3, q: 1.20 });
            sendAction("set_mit", { id: baseId + 4, q: 0.00 });
            sendAction("set_mit", { id: baseId + 5, q: -0.35 });
            sendAction("set_mit", { id: baseId + 6, q: 0.00 });
            sendAction("set_gripper", { id: baseId + 7, pos: 0.020 });
        });
    } else if (preset === "wave") {
        // Left arm raises up and waves, Right arm rests
        let t = 0;
        sendAction("set_mit", { id: 9, q: 0.0 });
        sendAction("set_mit", { id: 10, q: 0.0 });
        sendAction("set_mit", { id: 12, q: 0.0 });
        presetTimer = setInterval(() => {
            t += 0.05;
            const waveAngle = Math.sin(t * 3.5) * 0.40;
            sendAction("set_mit", { id: 1, q: 1.10 });
            sendAction("set_mit", { id: 2, q: -0.50 });
            sendAction("set_mit", { id: 3, q: 0.00 });
            sendAction("set_mit", { id: 4, q: 1.50 });
            sendAction("set_mit", { id: 5, q: 0.00 });
            sendAction("set_mit", { id: 6, q: waveAngle });
            sendAction("set_mit", { id: 7, q: 0.00 });
        }, 50);
    } else if (preset === "clap") {
        // Handshake / Grippers reach toward center
        [1, 9].forEach(baseId => {
            const isLeftArm = (baseId === 1);
            sendAction("set_mit", { id: baseId + 0, q: 0.50 });
            sendAction("set_mit", { id: baseId + 1, q: isLeftArm ? -0.15 : 0.15 });
            sendAction("set_mit", { id: baseId + 2, q: 0.00 });
            sendAction("set_mit", { id: baseId + 3, q: 1.30 });
            sendAction("set_mit", { id: baseId + 4, q: 0.00 });
            sendAction("set_mit", { id: baseId + 5, q: -0.20 });
            sendAction("set_mit", { id: baseId + 6, q: 0.00 });
            sendAction("set_gripper", { id: baseId + 7, pos: 0.035 });
        });
    } else if (preset === "carry") {
        // Dual-arm reach forward, grasp box, lift upwards
        let step = 0;
        presetTimer = setInterval(() => {
            step++;
            const phase = (step % 120);
            if (phase < 40) {
                // Reach forward & open grippers
                [1, 9].forEach(baseId => {
                    const isLeftArm = (baseId === 1);
                    sendAction("set_mit", { id: baseId + 0, q: 0.45 });
                    sendAction("set_mit", { id: baseId + 1, q: isLeftArm ? -0.25 : 0.25 });
                    sendAction("set_mit", { id: baseId + 3, q: 1.10 });
                    sendAction("set_mit", { id: baseId + 5, q: -0.30 });
                    sendAction("set_gripper", { id: baseId + 7, pos: 0.043 });
                });
            } else if (phase < 80) {
                // Grasp box
                [1, 9].forEach(baseId => {
                    sendAction("set_gripper", { id: baseId + 7, pos: 0.015 });
                });
            } else {
                // Lift box upward
                [1, 9].forEach(baseId => {
                    sendAction("set_mit", { id: baseId + 0, q: 0.80 });
                    sendAction("set_mit", { id: baseId + 3, q: 1.45 });
                    sendAction("set_mit", { id: baseId + 5, q: -0.45 });
                });
            }
        }, 60);
    } else if (preset === "sine") {
        let t = 0;
        presetTimer = setInterval(() => {
            t += 0.04;
            const q1 = Math.sin(t * 0.8) * 0.25 + 0.30;
            const q2 = Math.sin(t * 0.6) * 0.15 + 0.20;
            const q4 = Math.cos(t * 1.0) * 0.35 + 1.10;

            [1, 9].forEach(baseId => {
                const isLeftArm = (baseId === 1);
                sendAction("set_mit", { id: baseId + 0, q: q1 });
                sendAction("set_mit", { id: baseId + 1, q: isLeftArm ? -q2 : q2 });
                sendAction("set_mit", { id: baseId + 3, q: q4 });
            });
        }, 50);
    }
}
