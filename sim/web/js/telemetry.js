// ==============================================================================
// OPENARM TELEMETRY & TRAFFIC INSPECTOR & 100Hz EXPORT SUBSYSTEM
// Manages 16 motor cards, 3D kinematic rotation sync, CAN traffic table, and 100Hz telemetry export
// ==============================================================================

// Build Telemetry Cards for 16 Motors
function buildTelemetryCards() {
    const grid = document.getElementById("telemetry-grid");
    if (!grid) return;
    grid.innerHTML = "";

    ALL_JOINTS.forEach(j => {
        const isLeft = j.arm === "left";
        const card = document.createElement("div");
        card.className = `tel-card ${isLeft ? 'tel-card-left' : 'tel-card-right'}`;
        card.id = `tel-card-${j.id}`;
        card.dataset.arm = j.arm;

        const armTagHtml = isLeft
            ? `<span class="arm-tag arm-tag-left">L${j.idx === 7 ? 'G' : j.idx + 1}</span>`
            : `<span class="arm-tag arm-tag-right">R${j.idx === 7 ? 'G' : j.idx + 1}</span>`;

        card.innerHTML = `
            <div class="tel-card-head">
                <span class="tel-card-title">${armTagHtml} M${j.id} • ${j.type}</span>
                <span class="tel-chip chip-disabled" id="chip-${j.id}">OFF</span>
            </div>
            <div class="tel-metrics">
                <div><span class="metric-label">POS: </span><span class="metric-val" id="tel-q-${j.id}">0.000</span></div>
                <div><span class="metric-label">VEL: </span><span class="metric-val" id="tel-dq-${j.id}">0.000</span></div>
                <div><span class="metric-label">TAU: </span><span class="metric-val-tau" id="tel-tau-${j.id}">0.00</span></div>
                <div><span class="metric-label">MOS: </span><span class="metric-val" id="tel-tmos-${j.id}">30°C</span></div>
            </div>
        `;
        grid.appendChild(card);
    });

    applyTelemetryFilter();
}

function applyTelemetryFilter() {
    const cards = document.querySelectorAll(".tel-card");
    cards.forEach(card => {
        if (telemFilter === 'all') {
            card.style.display = 'block';
        } else if (card.dataset.arm === telemFilter) {
            card.style.display = 'block';
        } else {
            card.style.display = 'none';
        }
    });
}

// Telemetry & 3D Kinematics Synchronizer
function handleTelemetry(data) {
    const motors = data.motors || [];
    currentTelemetry = motors;

    const statRx = document.getElementById("stat-rx");
    const statTx = document.getElementById("stat-tx");
    if (statRx) statRx.textContent = data.frames_rx || 0;
    if (statTx) statTx.textContent = data.frames_tx || 0;

    const isRealMode = (data.mode === "real");
    updateUsbUiState(isRealMode);

    if (data.velocity_limit !== undefined && typeof window.syncSpeedControls === "function") {
        window.syncSpeedControls(data.velocity_limit, false);
    }

    motors.forEach(m => {
        const motorHasUsableState = (globalExecutionMode === "sim")
            || !isRealMode
            || (hasInitialRobotSync && m.has_sync === true && isOpenArmFeedbackFresh(m));
        // Update DOM Telemetry Card
        const chip = document.getElementById(`chip-${m.id}`);
        const qEl = document.getElementById(`tel-q-${m.id}`);
        const dqEl = document.getElementById(`tel-dq-${m.id}`);
        const tauEl = document.getElementById(`tel-tau-${m.id}`);
        const tmosEl = document.getElementById(`tel-tmos-${m.id}`);

        if (chip) {
            if (isRealMode && !isOpenArmFeedbackFresh(m)) {
                chip.className = "tel-chip chip-error";
                chip.textContent = "NO DATA";
            } else if (m.error_code > 1) {
                chip.className = "tel-chip chip-error";
                chip.textContent = `ERR ${m.error_code}`;
            } else if (m.enabled) {
                chip.className = "tel-chip chip-enabled";
                chip.textContent = "EN";
            } else {
                chip.className = "tel-chip chip-disabled";
                chip.textContent = "OFF";
            }
        }
        const isGripperMotor = (m.id === 8 || m.id === 16);
        if (qEl) {
            if (isGripperMotor) {
                qEl.textContent = `${(m.q * 1000).toFixed(1)} mm`;
            } else {
                qEl.textContent = `${m.q.toFixed(3)} (${m.q_deg}°)`;
            }
        }
        if (dqEl) dqEl.textContent = `${m.dq.toFixed(2)}`;
        if (tauEl) tauEl.textContent = `${m.tau.toFixed(2)} Nm`;
        if (tmosEl) tmosEl.textContent = `${m.t_mos.toFixed(1)}°C`;

        // Keep showing the commanded target after the slider is released. Live
        // feedback takes ownership again only after the real joint reaches it or
        // the user explicitly requests robot-state synchronization.
        const applyTelemetry = motorHasUsableState && (
            typeof shouldApplyOpenArmTelemetry !== "function"
            || shouldApplyOpenArmTelemetry(m)
        );
        if (applyTelemetry && typeof updateOpenArmJointVisual === "function") {
            updateOpenArmJointVisual(m);
        }

        const isLeft = m.id <= 8;
        const jointIndex = (m.id <= 8 ? m.id : m.id - 8) - 1;

        // Bi-directional Synchronization: Update UI Sliders to match live robot state
        const armGroup = isLeft ? 'left' : 'right';
        const sliderKey = `${armGroup}-${jointIndex}`;
        const sliderEl = document.getElementById(`slider-${sliderKey}`);
        const dispEl = document.getElementById(`val-disp-${sliderKey}`);

        if (applyTelemetry && sliderEl && document.activeElement !== sliderEl) {
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

        // Also sync top Dual Gripper sliders if not focused
        if (jointIndex === 7) {
            const topSlider = document.getElementById(isLeft ? "slider-gripper-left" : "slider-gripper-right");
            const topDisp = document.getElementById(isLeft ? "left-gripper-val-display" : "right-gripper-val-display");
            if (applyTelemetry && topSlider && document.activeElement !== topSlider) {
                topSlider.value = m.q;
                if (topDisp) topDisp.textContent = formatGripperText(m.q);
            }
        }
    });

    if (motors.length > 0 && typeof setOpenArmModelVisibility === "function") {
        setOpenArmModelVisibility(true);
    }

    // Check target tracking convergence
    if (activeMotionTracking && targetJointValues && motors.length >= 16) {
        let maxErr = 0.0;
        let maxErrJoint = 1;
        motors.forEach((m, idx) => {
            const target = targetJointValues[idx];
            if (target !== undefined) {
                const err = Math.abs(m.q - target);
                if (err > maxErr) {
                    maxErr = err;
                    maxErrJoint = m.id;
                }
            }
        });

        const statusDot = document.getElementById("target-status-dot");
        const statusText = document.getElementById("target-status-text");
        const statusSub = document.getElementById("target-status-sub");

        if (maxErr < 0.025) {
            activeMotionTracking = false;
            if (statusDot) statusDot.className = "status-indicator-dot dot-reached";
            if (statusText) statusText.textContent = "Đã chuyển động đến đúng vị trí mong muốn!";
            if (statusSub) statusSub.textContent = `Tất cả 16 khớp đã đến đích (Sai số < 0.025 rad)`;
            showToast("success", "Robot đã đến đúng vị trí góc khớp mong muốn!");
        } else {
            if (statusText) statusText.textContent = `Đang di chuyển: Sai số lớn nhất M${maxErrJoint} = ${maxErr.toFixed(3)} rad`;
            const pct = Math.max(0, Math.min(100, (1.0 - maxErr / 1.5) * 100));
            if (statusSub) statusSub.textContent = `Nội suy vận tốc an toàn 400Hz • Tiến trình: ${pct.toFixed(0)}%`;
        }
    }

    // ACT dataset recorder status update
    if (data.dataset_stats) {
        handleDatasetStats(data.dataset_stats);
    }
}

// ACT Dataset Record (data_set/) UI Updater: round button, top-left of the 3D robot viewport
function handleDatasetStats(stats) {
    const btn = document.getElementById("btn-dataset-record");
    const timer = document.getElementById("dataset-rec-timer");
    const dir = stats.output_dir || "data_set";

    if (stats.recording) {
        const elapsed = (stats.elapsed_sec || 0).toFixed(1);
        if (btn) {
            btn.classList.add("recording");
            btn.title = `Đang ghi ${dir}/${stats.episode} • Bấm để dừng và lưu`;
        }
        if (timer) {
            timer.classList.add("visible");
            timer.textContent = `REC ${elapsed}s`;
        }
    } else {
        if (btn) {
            btn.classList.remove("recording");
            if (stats.last_error) btn.title = `Ghi Data • Lần trước lỗi: ${stats.last_error}`;
            else if (stats.last_file) btn.title = `Ghi Data • Đã lưu: ${dir}/${stats.last_file} (${stats.last_samples} mẫu)`;
            else btn.title = `Ghi Data: ghi episode vào ${dir}/`;
        }
        if (timer) timer.classList.remove("visible");
    }
}

// CAN Traffic Inspector Table
function handleTraffic(packets) {
    if (!packets || packets.length === 0) return;
    const tbody = document.getElementById("traffic-tbody");
    if (!tbody) return;

    // Filter packets
    const filtered = packets.filter(p => {
        if (trafficFilter === 'all') return true;
        const canIdInt = parseInt(p.id, 16);
        const baseId = canIdInt & 0xFF;
        if (trafficFilter === 'left') {
            return (baseId >= 0x01 && baseId <= 0x18);
        } else {
            return (baseId >= 0x21 && baseId <= 0x38) || (baseId >= 0x09 && baseId <= 0x10);
        }
    });

    const recent = filtered.slice(-15).reverse();
    if (recent.length === 0) return;

    tbody.innerHTML = recent.map(p => {
        const dirClass = p.dir === 'RX' ? 'dir-rx' : 'dir-tx';
        const d = new Date(p.time * 1000);
        const timeStr = d.toTimeString().split(' ')[0] + '.' + String(d.getMilliseconds()).padStart(3, '0');
        return `
            <tr>
                <td style="color: var(--text-dim);">${timeStr}</td>
                <td><span class="${dirClass}">${p.dir}</span></td>
                <td><code>0x${p.id}</code></td>
                <td>${p.dlc}</td>
                <td><code>${p.data}</code></td>
            </tr>
        `;
    }).join("");
}
