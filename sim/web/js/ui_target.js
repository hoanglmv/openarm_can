// ==============================================================================
// OPENARM TARGET JOINT STATE CONTROLLER (SDK MOTION EXECUTION)
// Handles Target Pose inputs, Rad/Deg conversions, Safety Clamping, and Trajectory Execution
// ==============================================================================

let activeMotionTracking = false;
let targetJointValues = null;
let targetUnit = 'deg'; // 'deg' or 'rad'

function setupTargetJointUI() {
    const btnUnitDeg = document.getElementById("btn-unit-deg");
    const btnUnitRad = document.getElementById("btn-unit-rad");
    const unitLabels = document.querySelectorAll(".target-arm-col .unit-label");

    function updateAllLimitDisplays() {
        for (let i = 1; i <= 16; i++) {
            const lim = MECHANICAL_LIMITS[i];
            if (!lim) continue;
            const inp = document.getElementById(`inp-target-${i}`);
            const lbl = document.getElementById(`limit-lbl-${i}`);

            if (lim.isGripper) {
                if (lbl) lbl.textContent = `[${lim.minMm} ~ ${lim.maxMm} mm]`;
                if (inp) {
                    inp.min = lim.minMm;
                    inp.max = lim.maxMm;
                    inp.step = 0.5;
                }
            } else {
                if (targetUnit === 'deg') {
                    if (lbl) lbl.textContent = `[${lim.minDeg > 0 ? '+' : ''}${lim.minDeg}° ~ +${lim.maxDeg}°]`;
                    if (inp) {
                        inp.min = lim.minDeg;
                        inp.max = lim.maxDeg;
                        inp.step = 0.5;
                    }
                } else {
                    if (lbl) lbl.textContent = `[${lim.minRad > 0 ? '+' : ''}${lim.minRad.toFixed(2)} ~ +${lim.maxRad.toFixed(2)}]`;
                    if (inp) {
                        inp.min = lim.minRad.toFixed(3);
                        inp.max = lim.maxRad.toFixed(3);
                        inp.step = 0.01;
                    }
                }
            }
        }
    }

    function setTargetUnit(newUnit) {
        if (newUnit === targetUnit) return;
        targetUnit = newUnit;

        if (newUnit === 'deg') {
            if (btnUnitDeg) btnUnitDeg.classList.add("active");
            if (btnUnitRad) btnUnitRad.classList.remove("active");
            unitLabels.forEach(lbl => {
                if (!lbl.closest(".highlight-gripper")) lbl.textContent = "°";
            });
            // Convert current input values from rad to deg
            for (let i = 1; i <= 16; i++) {
                if (i === 8 || i === 16) continue;
                const inp = document.getElementById(`inp-target-${i}`);
                if (inp) {
                    const r = parseFloat(inp.value) || 0.0;
                    inp.value = (r * 180 / Math.PI).toFixed(1);
                }
            }
        } else {
            if (btnUnitRad) btnUnitRad.classList.add("active");
            if (btnUnitDeg) btnUnitDeg.classList.remove("active");
            unitLabels.forEach(lbl => {
                if (!lbl.closest(".highlight-gripper")) lbl.textContent = "rad";
            });
            // Convert current input values from deg to rad
            for (let i = 1; i <= 16; i++) {
                if (i === 8 || i === 16) continue;
                const inp = document.getElementById(`inp-target-${i}`);
                if (inp) {
                    const d = parseFloat(inp.value) || 0.0;
                    inp.value = (d * Math.PI / 180).toFixed(3);
                }
            }
        }

        updateAllLimitDisplays();
    }

    if (btnUnitDeg) btnUnitDeg.addEventListener("click", () => setTargetUnit('deg'));
    if (btnUnitRad) btnUnitRad.addEventListener("click", () => setTargetUnit('rad'));

    // Realtime joint limits validation & boundary clamping
    for (let i = 1; i <= 16; i++) {
        const inp = document.getElementById(`inp-target-${i}`);
        const row = document.getElementById(`row-target-${i}`);
        const lbl = document.getElementById(`limit-lbl-${i}`);
        const lim = MECHANICAL_LIMITS[i];

        if (!inp || !lim) continue;

        inp.addEventListener("input", () => {
            const val = parseFloat(inp.value);
            if (isNaN(val)) return;

            let minBound, maxBound;
            if (lim.isGripper) {
                minBound = lim.minMm;
                maxBound = lim.maxMm;
            } else if (targetUnit === 'deg') {
                minBound = lim.minDeg;
                maxBound = lim.maxDeg;
            } else {
                minBound = lim.minRad;
                maxBound = lim.maxRad;
            }

            if (val < minBound || val > maxBound) {
                if (row) row.classList.add("input-exceeded");
                if (lbl) lbl.classList.add("limit-warning");
            } else {
                if (row) row.classList.remove("input-exceeded");
                if (lbl) lbl.classList.remove("limit-warning");
            }
        });

        inp.addEventListener("change", () => {
            let val = parseFloat(inp.value);
            if (isNaN(val)) val = 0.0;

            let minBound, maxBound, unitText;
            if (lim.isGripper) {
                minBound = lim.minMm;
                maxBound = lim.maxMm;
                unitText = "mm";
            } else if (targetUnit === 'deg') {
                minBound = lim.minDeg;
                maxBound = lim.maxDeg;
                unitText = "°";
            } else {
                minBound = lim.minRad;
                maxBound = lim.maxRad;
                unitText = " rad";
            }

            if (val < minBound) {
                inp.value = minBound;
                if (row) row.classList.remove("input-exceeded");
                if (lbl) lbl.classList.remove("limit-warning");
                showToast("warning", `⚠️ [Giới hạn Khớp] ${lim.name} không thể quay dưới ${minBound}${unitText}! Đã tự động giữ ở ngưỡng an toàn.`);
            } else if (val > maxBound) {
                inp.value = maxBound;
                if (row) row.classList.remove("input-exceeded");
                if (lbl) lbl.classList.remove("limit-warning");
                showToast("warning", `⚠️ [Giới hạn Khớp] ${lim.name} không thể quay quá ${maxBound}${unitText}! Đã tự động giữ ở ngưỡng an toàn.`);
            }
        });
    }

    // Input View Tab Selector (Table vs JSON)
    const tabTypeTable = document.getElementById("tab-type-table");
    const tabTypeJson = document.getElementById("tab-type-json");
    const viewTable = document.getElementById("target-view-table");
    const viewJson = document.getElementById("target-view-json");
    let currentInputType = 'table';

    if (tabTypeTable && tabTypeJson) {
        tabTypeTable.addEventListener("click", () => {
            currentInputType = 'table';
            tabTypeTable.classList.add("active");
            tabTypeJson.classList.remove("active");
            if (viewTable) viewTable.style.display = "flex";
            if (viewJson) viewJson.style.display = "none";
        });
        tabTypeJson.addEventListener("click", () => {
            currentInputType = 'json';
            tabTypeJson.classList.add("active");
            tabTypeTable.classList.remove("active");
            if (viewTable) viewTable.style.display = "none";
            if (viewJson) viewJson.style.display = "flex";
        });
    }

    // Copy Current Robot Pose into inputs
    const btnCopyCurrent = document.getElementById("btn-target-copy-current");
    if (btnCopyCurrent) {
        btnCopyCurrent.addEventListener("click", () => {
            if (!currentTelemetry || currentTelemetry.length === 0) {
                showToast("warning", "Chưa có dữ liệu từ robot, vui lòng thử lại sau giây lát.");
                return;
            }
            currentTelemetry.forEach(m => {
                const inp = document.getElementById(`inp-target-${m.id}`);
                const row = document.getElementById(`row-target-${m.id}`);
                const lbl = document.getElementById(`limit-lbl-${m.id}`);
                if (inp) {
                    if (m.id === 8 || m.id === 16) {
                        inp.value = (m.q * 1000).toFixed(1);
                    } else {
                        if (targetUnit === 'deg') {
                            inp.value = (m.q * 180 / Math.PI).toFixed(1);
                        } else {
                            inp.value = m.q.toFixed(3);
                        }
                    }
                    if (row) row.classList.remove("input-exceeded");
                    if (lbl) lbl.classList.remove("limit-warning");
                }
            });
            showToast("info", "✓ Đã đọc toàn bộ góc hiện tại vào bảng nhập mục tiêu!");
        });
    }

    // Zero all inputs (with limit clamping)
    const btnZeroInputs = document.getElementById("btn-target-zero");
    if (btnZeroInputs) {
        btnZeroInputs.addEventListener("click", () => {
            for (let i = 1; i <= 16; i++) {
                const inp = document.getElementById(`inp-target-${i}`);
                const row = document.getElementById(`row-target-${i}`);
                const lbl = document.getElementById(`limit-lbl-${i}`);
                if (inp) {
                    inp.value = "0.0";
                    if (row) row.classList.remove("input-exceeded");
                    if (lbl) lbl.classList.remove("limit-warning");
                }
            }
            showToast("info", "Đã đặt tất cả các góc nhập về 0.0");
        });
    }

    // Sample JSON payloads (guaranteed within mechanical limits)
    const samplePayloads = {
        ready: {
            "openarm_left_joint1": 0.0,
            "openarm_left_joint2": -0.35,
            "openarm_left_joint4": 0.85,
            "openarm_left_finger_joint": 0.02,
            "openarm_right_joint1": 0.0,
            "openarm_right_joint2": 0.35,
            "openarm_right_joint4": 0.85,
            "openarm_right_finger_joint": 0.02
        },
        wave: {
            "openarm_right_joint1": 0.35,
            "openarm_right_joint2": 1.20,
            "openarm_right_joint4": 1.57,
            "openarm_right_joint6": 0.40,
            "openarm_right_finger_joint": 0.04
        },
        reach: {
            "openarm_left_joint1": 0.70,
            "openarm_left_joint4": 0.30,
            "openarm_left_finger_joint": 0.043,
            "openarm_right_joint1": 0.70,
            "openarm_right_joint4": 0.30,
            "openarm_right_finger_joint": 0.043
        }
    };

    document.querySelectorAll(".btn-sample-chip").forEach(btn => {
        btn.addEventListener("click", () => {
            const key = btn.dataset.sample;
            const payload = samplePayloads[key];
            if (payload) {
                const ta = document.getElementById("target-json-textarea");
                if (ta) ta.value = JSON.stringify(payload, null, 2);
            }
        });
    });

    // Execute Target Pose Button with strictly enforced limit checking
    const btnExec = document.getElementById("btn-execute-target-pose");
    const statusDot = document.getElementById("target-status-dot");
    const statusText = document.getElementById("target-status-text");
    const statusSub = document.getElementById("target-status-sub");

    if (btnExec) {
        btnExec.addEventListener("click", () => {
            if (currentInputType === 'json') {
                const ta = document.getElementById("target-json-textarea");
                const raw = ta ? ta.value.trim() : "";
                if (!raw) {
                    showToast("error", "Vui lòng nhập JSON joint state trước khi gửi!");
                    return;
                }
                try {
                    const parsed = JSON.parse(raw);
                    let jsonClampedCount = 0;

                    // Helper to clamp a single joint
                    function checkAndClampJoint(mid, rawVal, keyName) {
                        if (!mid || !MECHANICAL_LIMITS[mid]) return rawVal;
                        const lim = MECHANICAL_LIMITS[mid];
                        let minB = lim.isGripper ? lim.minMm / 1000.0 : lim.minRad;
                        let maxB = lim.isGripper ? lim.maxMm / 1000.0 : lim.maxRad;
                        // Handle mm passed directly for gripper
                        if (lim.isGripper && rawVal > 0.043 && rawVal <= 43.0) {
                            rawVal = rawVal / 1000.0;
                        }
                        const clamped = Math.max(minB, Math.min(maxB, rawVal));
                        if (Math.abs(clamped - rawVal) > 1e-4) {
                            jsonClampedCount++;
                            const unit = lim.isGripper ? "m" : "rad";
                            showToast("warning", `⚠️ [An Toàn] ${lim.name} (${keyName}): Vượt giới hạn (${rawVal.toFixed(3)}${unit}), đã tự động kẹp về ${clamped.toFixed(3)}${unit}!`);
                            return clamped;
                        }
                        return rawVal;
                    }

                    // A: If "joints" dict
                    if (parsed.joints && typeof parsed.joints === 'object') {
                        Object.keys(parsed.joints).forEach(k => {
                            const mid = (typeof k === 'number' || !isNaN(parseInt(k))) ? parseInt(k) : (JOINT_NAME_TO_ID ? JOINT_NAME_TO_ID[k.toLowerCase()] : null);
                            parsed.joints[k] = checkAndClampJoint(mid, parseFloat(parsed.joints[k]), k);
                        });
                    }

                    // B: If "left" or "right" arrays
                    if (Array.isArray(parsed.left)) {
                        parsed.left = parsed.left.map((val, idx) => checkAndClampJoint(idx + 1, parseFloat(val), `Left J${idx + 1}`));
                    }
                    if (Array.isArray(parsed.right)) {
                        parsed.right = parsed.right.map((val, idx) => checkAndClampJoint(idx + 9, parseFloat(val), `Right J${idx + 1}`));
                    }
                    if (parsed.left_gripper !== undefined) {
                        parsed.left_gripper = checkAndClampJoint(8, parseFloat(parsed.left_gripper), "Left Gripper");
                    }
                    if (parsed.right_gripper !== undefined) {
                        parsed.right_gripper = checkAndClampJoint(16, parseFloat(parsed.right_gripper), "Right Gripper");
                    }

                    // C: If top-level keys
                    Object.keys(parsed).forEach(k => {
                        if (["joints", "left", "right", "positions", "names", "left_gripper", "right_gripper"].includes(k)) return;
                        const mid = (typeof k === 'number' || !isNaN(parseInt(k))) ? parseInt(k) : (JOINT_NAME_TO_ID ? JOINT_NAME_TO_ID[k.toLowerCase()] : null);
                        if (mid) {
                            parsed[k] = checkAndClampJoint(mid, parseFloat(parsed[k]), k);
                        }
                    });

                    // D: If "positions" list
                    if (Array.isArray(parsed.positions)) {
                        parsed.positions = parsed.positions.map((val, idx) => checkAndClampJoint(idx + 1, parseFloat(val), `Joint ${idx + 1}`));
                    }

                    if (jsonClampedCount > 0 && ta) {
                        ta.value = JSON.stringify(parsed, null, 2);
                    }

                    sendAction("set_joint_state", parsed);
                    activeMotionTracking = true;
                    if (statusDot) statusDot.className = "status-indicator-dot dot-moving";
                    if (statusText) statusText.textContent = "Đang gửi tín hiệu SDK & di chuyển robot...";
                    if (statusSub) statusSub.textContent = "Lệnh JSON an toàn đã gửi qua WebSocket / 400Hz MIT";
                    showToast("success", "🚀 Đã gửi tín hiệu SDK! Robot đang chuyển động theo JSON...");
                } catch (err) {
                    showToast("error", "Lỗi định dạng JSON: " + err.message);
                }
                return;
            }

            // Mode Table: build 16 values with strict safety clamping
            const targetPositions = [];
            let clampedCount = 0;

            for (let i = 1; i <= 16; i++) {
                const inp = document.getElementById(`inp-target-${i}`);
                const lim = MECHANICAL_LIMITS[i];
                let val = inp ? (parseFloat(inp.value) || 0.0) : 0.0;

                if (lim.isGripper) {
                    // Gripper is in mm -> clamp between [0, 43mm]
                    const clampedMm = Math.max(lim.minMm, Math.min(lim.maxMm, val));
                    if (clampedMm !== val) {
                        clampedCount++;
                        if (inp) inp.value = clampedMm;
                    }
                    targetPositions.push(clampedMm / 1000.0);
                } else {
                    let radVal = (targetUnit === 'deg') ? (val * Math.PI / 180) : val;
                    const clampedRad = Math.max(lim.minRad, Math.min(lim.maxRad, radVal));
                    if (clampedRad !== radVal) {
                        clampedCount++;
                        if (inp) {
                            inp.value = (targetUnit === 'deg') ? (clampedRad * 180 / Math.PI).toFixed(1) : clampedRad.toFixed(3);
                        }
                    }
                    targetPositions.push(clampedRad);
                }
            }

            if (clampedCount > 0) {
                showToast("warning", `⚠️ Đã tự động giới hạn ${clampedCount} khớp về ngưỡng cơ học an toàn để chống quay quá khớp!`);
            }

            targetJointValues = targetPositions;
            activeMotionTracking = true;

            // Send to backend via set_joint_state action
            sendAction("set_joint_state", { positions: targetPositions });

            if (statusDot) statusDot.className = "status-indicator-dot dot-moving";
            if (statusText) statusText.textContent = "Đang chuyển động đến vị trí mong muốn...";
            if (statusSub) statusSub.textContent = `Tốc độ: 0.25 rad/s • Kiểm soát giới hạn khớp an toàn`;
            showToast("success", "🚀 Đã gửi tín hiệu SDK! Robot đang chuyển động đến đúng vị trí...");
        });
    }

    // Emergency Stop for Target Mode
    const btnTargetStop = document.getElementById("btn-target-stop");
    if (btnTargetStop) {
        btnTargetStop.addEventListener("click", () => {
            activeMotionTracking = false;
            if (currentTelemetry && currentTelemetry.length > 0) {
                const freezePos = currentTelemetry.map(m => m.q);
                sendAction("set_joint_state", { positions: freezePos });
            }
            if (statusDot) statusDot.className = "status-indicator-dot dot-idle";
            if (statusText) statusText.textContent = "Đã dừng chuyển động khẩn cấp.";
            if (statusSub) statusSub.textContent = "Điểm đặt đã đóng băng tại vị trí hiện tại.";
            showToast("warning", "⏹ Đã dừng chuyển động của robot!");
        });
    }

    // Initial limit display setup
    updateAllLimitDisplays();
}
