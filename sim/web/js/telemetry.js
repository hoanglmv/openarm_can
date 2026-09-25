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

    motors.forEach(m => {
        // Update DOM Telemetry Card
        const chip = document.getElementById(`chip-${m.id}`);
        const qEl = document.getElementById(`tel-q-${m.id}`);
        const dqEl = document.getElementById(`tel-dq-${m.id}`);
        const tauEl = document.getElementById(`tel-tau-${m.id}`);
        const tmosEl = document.getElementById(`tel-tmos-${m.id}`);

        if (chip) {
            if (m.error_code > 1) {
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

        // Update 3D Robot Kinematics
        const isLeft = m.id <= 8;
        const armJoints = isLeft ? leftArmJoints : rightArmJoints;
        const gripperFingers = isLeft ? leftGripperFingers : rightGripperFingers;
        const jointIndex = (m.id <= 8 ? m.id : m.id - 8) - 1;

        if (jointIndex < 7) {
            const jEntry = armJoints[jointIndex];
            if (jEntry && jEntry.group) {
                const angle = m.q;
                if (jointIndex === 0) {
                    // J1: Shoulder Pitch (swings arm forward/backward around X)
                    jEntry.group.rotation.x = -angle;
                } else if (jointIndex === 1) {
                    // J2: Shoulder Roll / Abduction
                    jEntry.group.rotation.z = angle;
                } else if (jointIndex === 2) {
                    // J3: Arm Twist (humeral twist around Y axis)
                    jEntry.group.rotation.y = isLeft ? angle : -angle;
                } else if (jointIndex === 3) {
                    // J4: Elbow Pitch (flexion forward around X)
                    jEntry.group.rotation.x = -angle;
                } else if (jointIndex === 4) {
                    // J5: Forearm Twist (pronation/supination around Y)
                    jEntry.group.rotation.y = isLeft ? angle : -angle;
                } else if (jointIndex === 5) {
                    // J6: Wrist Pitch (tilts up/down around X)
                    jEntry.group.rotation.x = -angle;
                } else if (jointIndex === 6) {
                    // J7: Wrist Roll (gripper rotation around tool axis Y)
                    jEntry.group.rotation.y = isLeft ? angle : -angle;
                }
            }
        } else if (jointIndex === 7) {
            // Horizontal parallel linear gripper (stroke: 0.0m closed to 0.043m open)
            const strokeRatio = Math.max(0.0, Math.min(1.0, Math.abs(m.q) / 0.043));
            const fingerOffset = 0.010 + strokeRatio * 0.028;
            if (gripperFingers.left && gripperFingers.right) {
                gripperFingers.left.position.x = -fingerOffset;
                gripperFingers.right.position.x = fingerOffset;
            }
        }

        // Bi-directional Synchronization: Update UI Sliders to match live robot state
        const armGroup = isLeft ? 'left' : 'right';
        const sliderKey = `${armGroup}-${jointIndex}`;
        const sliderEl = document.getElementById(`slider-${sliderKey}`);
        const dispEl = document.getElementById(`val-disp-${sliderKey}`);

        if (sliderEl && document.activeElement !== sliderEl) {
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
            if (topSlider && document.activeElement !== topSlider) {
                topSlider.value = m.q;
                if (topDisp) topDisp.textContent = formatGripperText(m.q);
            }
        }
    });

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

    // 100Hz Continuous Joint State Exporter status update
    if (data.export_stats) {
        handleExportStats(data.export_stats);
    }
}

// Joint State Record (100Hz) UI Updater
function handleExportStats(stats) {
    if (!stats) return;
    const badgeText = document.getElementById("export-badge-text");
    const headerBadge = document.getElementById("export-status-badge");
    const dot = document.getElementById("export-dot");
    const subInfo = document.getElementById("export-sub-info");
    const samplesBadge = document.getElementById("export-samples-badge");
    const recordBtn = document.getElementById("btn-record-toggle") || document.getElementById("btn-export-toggle");
    const recordText = document.getElementById("btn-record-text");
    const recordIcon = document.getElementById("btn-record-icon");

    const hz = stats.sample_rate_hz > 0 ? stats.sample_rate_hz.toFixed(1) : "100.0";
    const samples = stats.samples_recorded !== undefined ? stats.samples_recorded : (stats.samples || 0);
    const elapsed = stats.elapsed_sec ? stats.elapsed_sec.toFixed(1) : (samples / 100).toFixed(1);
    const filename = stats.filepath || stats.file_name || "";

    if (stats.recording && stats.active) {
        // --- RECORDING STATE ---
        if (headerBadge) headerBadge.className = "badge badge-export recording";
        if (badgeText) badgeText.textContent = `REC (${samples.toLocaleString()})`;
        if (dot) dot.className = "export-status-dot recording";
        if (subInfo) subInfo.textContent = `🔴 Đang record: exports/${filename} • ${hz} Hz • UDP :${stats.udp_port || 9871}`;
        if (samplesBadge) {
            samplesBadge.className = "badge badge-danger";
            samplesBadge.textContent = `🔴 Đang Record: ${samples.toLocaleString()} mẫu (${elapsed}s)`;
        }
        if (recordBtn) {
            recordBtn.className = "btn-record-main recording";
            recordBtn.title = "Bấm để dừng và lưu file Record";
        }
        if (recordIcon) recordIcon.textContent = "■";
        if (recordText) recordText.textContent = "Dừng Record (End)";
    } else {
        // --- IDLE / STOPPED STATE ---
        if (headerBadge) headerBadge.className = "badge badge-export idle";
        if (badgeText) badgeText.textContent = "REC: Sẵn sàng";
        if (dot) dot.className = "export-status-dot idle";

        const dlBtn = document.getElementById("btn-export-download");
        const dlHeaderBtn = document.getElementById("btn-export-download-header");
        if (filename) {
            if (dlBtn) dlBtn.setAttribute("download", filename);
            if (dlHeaderBtn) dlHeaderBtn.setAttribute("download", filename);
        }

        if (samples > 0 && filename) {
            if (subInfo) subInfo.textContent = `✓ Đã lưu HDF5: exports/${filename} • Bấm Record để ghi phiên mới • UDP :${stats.udp_port || 9871}`;
            if (samplesBadge) {
                samplesBadge.className = "badge badge-success";
                samplesBadge.textContent = `✓ Đã lưu • ${samples.toLocaleString()} mẫu (${elapsed}s)`;
            }
        } else {
            if (subInfo) subInfo.textContent = `Chưa ghi • Bấm "Bắt đầu Record" để ghi dữ liệu HDF5 (.hdf5) • UDP :${stats.udp_port || 9871}`;
            if (samplesBadge) {
                samplesBadge.className = "badge badge-outline";
                samplesBadge.textContent = "Chờ bắt đầu";
            }
        }

        if (recordBtn) {
            recordBtn.className = "btn-record-main";
            recordBtn.title = "Bấm để bắt đầu Record dữ liệu góc khớp";
        }
        if (recordIcon) recordIcon.textContent = "●";
        if (recordText) recordText.textContent = "Bắt đầu Record";
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
