// ==============================================================================
// OPENARM UI SLIDERS & MAIN EVENT HANDLERS
// Manages Joint Sliders, Safety Locks, Gripper Direct Controls, and Dashboard Tools
// ==============================================================================

// Build Sliders for Arm Joints & Gripper (J1-J7 + J8 Gripper)
function buildJointSliders() {
    const list = document.getElementById("sliders-list");
    if (!list) return;
    list.innerHTML = "";

    let itemsToRender = [];

    if (currentArmTab === 'both') {
        // Dual view: Show Left (J1..J8) and Right (J1..J8)
        itemsToRender = [
            ...LEFT_JOINTS.map(j => ({ ...j, group: 'left' })),
            ...RIGHT_JOINTS.map(j => ({ ...j, group: 'right' }))
        ];
    } else if (currentArmTab === 'right') {
        itemsToRender = RIGHT_JOINTS.map(j => ({ ...j, group: 'right' }));
    } else if (currentArmTab === 'sync') {
        itemsToRender = LEFT_JOINTS.map((j, idx) => ({ ...j, group: 'sync', syncIdx: idx }));
    } else {
        // Default left (J1..J8)
        itemsToRender = LEFT_JOINTS.map(j => ({ ...j, group: 'left' }));
    }

    itemsToRender.forEach((j) => {
        const card = document.createElement("div");
        const isLeft = j.group === 'left';
        const isRight = j.group === 'right';
        const isSync = j.group === 'sync';
        const isLocked = jointLockStates[j.id] === false;
        const isGripper = (j.idx === 7);

        card.className = `joint-slider-card ${isLeft ? 'left-arm-joint' : (isRight ? 'right-arm-joint' : 'sync-joint')} ${isLocked ? 'joint-locked' : 'joint-active'} ${isGripper ? 'gripper-joint-card' : ''}`;

        const uniqueKey = `${j.group}-${j.idx}`;
        const armTagClass = isSync ? "arm-tag-sync" : (isRight ? "arm-tag-right" : "arm-tag-left");
        const armTagText = isSync ? "L+R" : (isRight ? "R" : "L");
        const jNum = `J${j.idx + 1}`;
        const cleanName = JOINT_NAMES[j.idx] || `Joint ${j.idx + 1}`;
        const valUnitText = isGripper ? "0.0 mm (Đóng)" : "0.00 rad (0°)";

        card.innerHTML = `
            <div class="slider-header-compact">
                <div class="joint-title-row">
                    <button class="btn-joint-power ${isLocked ? 'power-off' : 'power-on'}" 
                            title="${isLocked ? 'Khóa an toàn: Đang KHÓA (Bấm để MỞ)' : 'Khớp đang BẬT sẵn sàng (Bấm để KHÓA)'}" 
                            onclick="toggleJointLock(${j.id}, '${uniqueKey}', '${j.group}', ${j.idx})">
                        ${isLocked ? 'OFF' : 'ON'}
                    </button>
                    <span class="arm-tag ${armTagClass}">${armTagText}</span>
                    <span class="joint-id-badge">${jNum}</span>
                    <span class="joint-name-text">${cleanName}</span>
                    <span class="joint-motor-tag">${j.type}</span>
                </div>
                <div class="joint-val-row">
                    <span class="joint-val" id="val-disp-${uniqueKey}">${valUnitText}</span>
                    <button class="btn-zero-single" title="Reset to 0" onclick="resetSingleJoint(${j.id}, '${uniqueKey}', '${j.group}', ${j.idx})" ${isLocked ? 'disabled' : ''}>0</button>
                </div>
            </div>
            <div class="slider-track-row">
                <span class="range-bound">${isGripper ? '0.0 mm' : j.min.toFixed(2)}</span>
                <input type="range" class="joint-slider-input" id="slider-${uniqueKey}" min="${j.min}" max="${j.max}" step="${isGripper ? '0.0005' : '0.005'}" value="0.0" ${isLocked ? 'disabled' : ''}>
                <span class="range-bound">+${isGripper ? '43.0 mm' : j.max.toFixed(2)}</span>
            </div>
        `;
        list.appendChild(card);

        const slider = card.querySelector(`#slider-${uniqueKey}`);
        slider.addEventListener("input", (e) => {
            if (jointLockStates[j.id] === false) return;
            const val = parseFloat(e.target.value);

            if (isGripper) {
                const dispText = formatGripperText(val);
                document.getElementById(`val-disp-${uniqueKey}`).textContent = dispText;

                if (isLeft) {
                    sendAction("set_gripper", { id: 8, arm: "left", pos: val });
                    const topSlider = document.getElementById("slider-gripper-left");
                    if (topSlider) topSlider.value = val;
                    const topDisp = document.getElementById("left-gripper-val-display");
                    if (topDisp) topDisp.textContent = dispText;
                } else if (isRight) {
                    sendAction("set_gripper", { id: 16, arm: "right", pos: val });
                    const topSlider = document.getElementById("slider-gripper-right");
                    if (topSlider) topSlider.value = val;
                    const topDisp = document.getElementById("right-gripper-val-display");
                    if (topDisp) topDisp.textContent = dispText;
                } else { // sync
                    sendAction("set_gripper", { id: 8, arm: "left", pos: val });
                    sendAction("set_gripper", { id: 16, arm: "right", pos: val });
                    const topL = document.getElementById("slider-gripper-left");
                    const topR = document.getElementById("slider-gripper-right");
                    if (topL) topL.value = val;
                    if (topR) topR.value = val;
                    const dispL = document.getElementById("left-gripper-val-display");
                    const dispR = document.getElementById("right-gripper-val-display");
                    if (dispL) dispL.textContent = dispText;
                    if (dispR) dispR.textContent = dispText;
                }
            } else {
                const deg = (val * 180 / Math.PI).toFixed(0);
                document.getElementById(`val-disp-${uniqueKey}`).textContent = `${val.toFixed(2)} rad (${deg}°)`;

                if (isLeft) {
                    sendAction("set_mit", { id: j.id, q: val, kp: 30.0, kd: 1.2, tau: 0.0 });
                } else if (isRight) {
                    sendAction("set_mit", { id: j.id, q: val, kp: 30.0, kd: 1.2, tau: 0.0 });
                } else { // sync
                    const leftMotorId = LEFT_JOINTS[j.idx].id;
                    const rightMotorId = RIGHT_JOINTS[j.idx].id;
                    // Mirrored roll/yaw/abduction for symmetrical bimanual gestures (J2, J3, J5, J7)
                    const mirrorSign = (j.idx === 1 || j.idx === 2 || j.idx === 4 || j.idx === 6) ? -1.0 : 1.0;

                    sendAction("set_mit", { id: leftMotorId, q: val, kp: 30.0, kd: 1.2, tau: 0.0 });
                    sendAction("set_mit", { id: rightMotorId, q: val * mirrorSign, kp: 30.0, kd: 1.2, tau: 0.0 });
                }
            }
        });
    });
}

// Arm tab switching
function setArmTab(tab, element) {
    currentArmTab = tab;
    document.querySelectorAll(".arm-tab-btn").forEach(btn => {
        btn.classList.remove("active", "tab-left", "tab-right", "tab-sync");
    });
    if (element) {
        element.classList.add("active");
        if (tab === 'left') element.classList.add("tab-left");
        else if (tab === 'right') element.classList.add("tab-right");
        else if (tab === 'sync') element.classList.add("tab-sync");
    }
    buildJointSliders();
}

// Toggle Safety Lock / Enable switch for individual joint
window.toggleJointLock = function(id, key, group, idx) {
    if (group === 'sync') {
        const leftId = LEFT_JOINTS[idx].id;
        const rightId = RIGHT_JOINTS[idx].id;
        const currentVal = jointLockStates[leftId] !== false;
        const newState = !currentVal;
        jointLockStates[leftId] = newState;
        jointLockStates[rightId] = newState;
        sendAction(newState ? "enable_motor" : "disable_motor", { id: leftId });
        sendAction(newState ? "enable_motor" : "disable_motor", { id: rightId });
    } else {
        const currentVal = jointLockStates[id] !== false;
        const newState = !currentVal;
        jointLockStates[id] = newState;
        sendAction(newState ? "enable_motor" : "disable_motor", { id });
    }
    buildJointSliders();
};

// Quick individual joint zeroing
window.resetSingleJoint = function(id, key, group, idx) {
    if (jointLockStates[id] === false) return; // Do not zero locked joints
    const slider = document.getElementById(`slider-${key}`);
    if (slider) slider.value = 0.0;
    const disp = document.getElementById(`val-disp-${key}`);
    const isGripper = (idx === 7);

    if (isGripper) {
        if (disp) disp.textContent = "0.0 mm (Đóng)";
        if (group === 'sync') {
            sendAction("set_gripper", { id: 8, arm: "left", pos: 0.0 });
            sendAction("set_gripper", { id: 16, arm: "right", pos: 0.0 });
            const topL = document.getElementById("slider-gripper-left");
            const topR = document.getElementById("slider-gripper-right");
            if (topL) topL.value = 0.0;
            if (topR) topR.value = 0.0;
            const dL = document.getElementById("left-gripper-val-display");
            const dR = document.getElementById("right-gripper-val-display");
            if (dL) dL.textContent = "0.0 mm (Đóng)";
            if (dR) dR.textContent = "0.0 mm (Đóng)";
        } else {
            const arm = (id === 8) ? "left" : "right";
            sendAction("set_gripper", { id, arm, pos: 0.0 });
            const topSlider = document.getElementById(arm === 'left' ? "slider-gripper-left" : "slider-gripper-right");
            if (topSlider) topSlider.value = 0.0;
            const topDisp = document.getElementById(arm === 'left' ? "left-gripper-val-display" : "right-gripper-val-display");
            if (topDisp) topDisp.textContent = "0.0 mm (Đóng)";
        }
    } else {
        if (disp) disp.textContent = "0.00 rad (0°)";
        if (group === 'sync') {
            const leftId = LEFT_JOINTS[idx].id;
            const rightId = RIGHT_JOINTS[idx].id;
            sendAction("set_mit", { id: leftId, q: 0.0, tau: 0.0 });
            sendAction("set_mit", { id: rightId, q: 0.0, tau: 0.0 });
        } else {
            sendAction("set_mit", { id, q: 0.0, tau: 0.0 });
        }
    }
};

// Quick gripper actions (Open / 50% / Close)
window.setGripperDirect = function(arm, val) {
    const isLeft = arm === 'left';
    const sliderId = isLeft ? 'slider-gripper-left' : 'slider-gripper-right';
    const dispId = isLeft ? 'left-gripper-val-display' : 'right-gripper-val-display';
    const motorId = isLeft ? 8 : 16;
    const text = formatGripperText(val);

    const slider = document.getElementById(sliderId);
    if (slider) slider.value = val;
    const disp = document.getElementById(dispId);
    if (disp) disp.textContent = text;

    // Also sync J8 slider in the joint sliders list
    const j8Key = isLeft ? "left-7" : "right-7";
    const j8Slider = document.getElementById(`slider-${j8Key}`);
    if (j8Slider) j8Slider.value = val;
    const j8Disp = document.getElementById(`val-disp-${j8Key}`);
    if (j8Disp) j8Disp.textContent = text;

    sendAction("set_gripper", { id: motorId, arm, pos: val });

    if (syncGrippers) {
        const otherSliderId = isLeft ? 'slider-gripper-right' : 'slider-gripper-left';
        const otherDispId = isLeft ? 'right-gripper-val-display' : 'left-gripper-val-display';
        const otherMotorId = isLeft ? 16 : 8;
        const otherArm = isLeft ? 'right' : 'left';

        const otherSlider = document.getElementById(otherSliderId);
        if (otherSlider) otherSlider.value = val;
        const otherDisp = document.getElementById(otherDispId);
        if (otherDisp) otherDisp.textContent = text;

        const otherJ8Key = isLeft ? "right-7" : "left-7";
        const otherJ8Slider = document.getElementById(`slider-${otherJ8Key}`);
        if (otherJ8Slider) otherJ8Slider.value = val;
        const otherJ8Disp = document.getElementById(`val-disp-${otherJ8Key}`);
        if (otherJ8Disp) otherJ8Disp.textContent = text;

        sendAction("set_gripper", { id: otherMotorId, arm: otherArm, pos: val });
    }
};

window.toggleGripperInvert = function(arm) {
    const motorId = (arm === 'left') ? 8 : 16;
    sendAction("toggle_gripper_invert", { id: motorId });
    showToast("info", `Đã đảo chiều quay cho kẹp ${arm === 'left' ? 'Tay Trái (Left Gripper)' : 'Tay Phải (Right Gripper)'}`);
};

// Setup Master Event Handlers
function setupEventHandlers() {
    // Theme toggle
    const btnTheme = document.getElementById("btn-theme-toggle");
    if (btnTheme) {
        btnTheme.addEventListener("click", () => {
            isLightTheme = !isLightTheme;
            if (isLightTheme) {
                document.body.classList.add("theme-light");
                btnTheme.textContent = "Studio Bright";
                if (floorMesh) floorMesh.material.color.setHex(0xeef2f6);
                if (scene) scene.background = new THREE.Color(0xf1f5f9);
            } else {
                document.body.classList.remove("theme-light");
                btnTheme.textContent = "Cyber Dark";
                if (floorMesh) floorMesh.material.color.setHex(0x111827);
                if (scene) scene.background = new THREE.Color(0x0f172a);
            }
        });
    }

    // Mode tabs switching (Khớp Tay, Tay Kẹp, Tư Thế & Tốc Độ, Bắn Joint State)
    const modeBtns = document.querySelectorAll(".mode-tab-btn");
    const modePanes = {
        joints: document.getElementById("pane-joints"),
        grippers: document.getElementById("pane-grippers"),
        presets: document.getElementById("pane-presets"),
        target: document.getElementById("pane-target")
    };

    function switchControlMode(mode) {
        modeBtns.forEach(b => {
            if (b.dataset.mode === mode) {
                b.classList.add("active");
            } else {
                b.classList.remove("active");
            }
        });

        Object.keys(modePanes).forEach(k => {
            if (modePanes[k]) {
                modePanes[k].style.display = (k === mode) ? "flex" : "none";
            }
        });
    }

    modeBtns.forEach(b => {
        b.addEventListener("click", () => {
            switchControlMode(b.dataset.mode);
        });
    });

    // Arm tab switching
    const tabL = document.getElementById("tab-arm-left");
    if (tabL) tabL.addEventListener("click", (e) => setArmTab("left", e.target));
    const tabR = document.getElementById("tab-arm-right");
    if (tabR) tabR.addEventListener("click", (e) => setArmTab("right", e.target));
    const tabB = document.getElementById("tab-arm-both");
    if (tabB) tabB.addEventListener("click", (e) => setArmTab("both", e.target));
    const tabS = document.getElementById("tab-arm-sync");
    if (tabS) tabS.addEventListener("click", (e) => setArmTab("sync", e.target));

    // USB Toggle button handlers
    const btnUsbToggle = document.getElementById("btn-usb-toggle");
    if (btnUsbToggle) btnUsbToggle.addEventListener("click", toggleUsbConnection);
    const btnMasterUsb = document.getElementById("btn-master-usb");
    if (btnMasterUsb) btnMasterUsb.addEventListener("click", toggleUsbConnection);

    const btnSyncState = document.getElementById("btn-sync-state");
    if (btnSyncState) {
        btnSyncState.addEventListener("click", () => {
            sendAction("sync_robot_state");
            syncUiFromRobot(currentTelemetry, true);
            showToast("success", "Đã đọc và đồng bộ trạng thái thực tế từ Robot vật lý!");
        });
    }

    // Master actions
    const btnEnableAll = document.getElementById("btn-enable-all");
    if (btnEnableAll) {
        btnEnableAll.addEventListener("click", () => {
            for (let i = 1; i <= 16; i++) jointLockStates[i] = true;
            sendAction("enable_all");
            buildJointSliders();
        });
    }

    const btnDisableAll = document.getElementById("btn-disable-all");
    if (btnDisableAll) {
        btnDisableAll.addEventListener("click", () => {
            if (typeof stopPresets === "function") stopPresets();
            for (let i = 1; i <= 16; i++) jointLockStates[i] = false;
            sendAction("disable_all");
            buildJointSliders();
        });
    }

    function triggerZeroPose() {
        if (typeof stopPresets === "function") stopPresets();
        document.querySelectorAll(".joint-slider-input").forEach(slider => {
            slider.value = 0.0;
        });
        document.querySelectorAll(".joint-val").forEach(disp => {
            if (disp.id.startsWith("val-disp-")) {
                if (disp.id.includes("-7")) {
                    disp.textContent = "0.0 mm (Đóng)";
                } else {
                    disp.textContent = "0.00 rad (0°)";
                }
            }
        });

        const leftG = document.getElementById("slider-gripper-left");
        const rightG = document.getElementById("slider-gripper-right");
        if (leftG) leftG.value = 0.0;
        if (rightG) rightG.value = 0.0;
        const leftGDisp = document.getElementById("left-gripper-val-display");
        const rightGDisp = document.getElementById("right-gripper-val-display");
        if (leftGDisp) leftGDisp.textContent = "0.0 mm (Đóng)";
        if (rightGDisp) rightGDisp.textContent = "0.0 mm (Đóng)";

        sendAction("go_to_zero_pose");
        showToast("info", "Đang đưa toàn bộ cánh tay về Zero Pose (0.0 rad) với lực giữ vững chắc...");
    }

    const btnGoZeroPose = document.getElementById("btn-go-zero-pose");
    if (btnGoZeroPose) {
        btnGoZeroPose.addEventListener("click", triggerZeroPose);
    }

    const btnZeroAll = document.getElementById("btn-zero-all");
    if (btnZeroAll) {
        btnZeroAll.addEventListener("click", triggerZeroPose);
    }

    const btnCalibrateHw = document.getElementById("btn-calibrate-hw-zero");
    if (btnCalibrateHw) {
        btnCalibrateHw.addEventListener("click", () => {
            const ok = confirm("CẢNH BÁO BẢO DƯỠNG CƠ KHÍ:\n\nThao tác này sẽ ghi đè vị trí hiện tại của các động cơ thành gốc 0 cơ khí (Lệnh 0xFE) trong bộ nhớ NVRAM.\n\nChỉ sử dụng khi bạn đã đặt robot lên đồ gá chuẩn căn chỉnh!\n\nBạn có chắc chắn muốn ghi đè gốc 0 cơ khí không?");
            if (ok) {
                sendAction("calibrate_mechanical_zero", { calibrate_hardware: true });
                showToast("warning", "Đã gửi lệnh căn chỉnh gốc 0 cơ khí 0xFE đến phần cứng!");
            }
        });
    }

    const btnClearErr = document.getElementById("btn-clear-err");
    if (btnClearErr) {
        btnClearErr.addEventListener("click", () => {
            sendAction("clear_error_all");
        });
    }

    // Motion Speed / Velocity Limit Controls
    const speedSlider = document.getElementById("slider-max-velocity");
    const speedDisp = document.getElementById("disp-speed-mode");
    const speedBtns = document.querySelectorAll(".btn-speed-preset");

    function applyVelocityLimit(val, activeBtn = null) {
        speedBtns.forEach(b => b.classList.remove("active"));
        if (activeBtn) activeBtn.classList.add("active");
        if (speedSlider) speedSlider.value = val;
        const degS = (val * 180 / Math.PI).toFixed(0);
        if (speedDisp) speedDisp.textContent = `${val.toFixed(2)} rad/s (${degS}°/s)`;
        sendAction("set_velocity_limit", { v_limit: val });
    }

    speedBtns.forEach(btn => {
        btn.addEventListener("click", () => {
            const spd = parseFloat(btn.dataset.speed);
            applyVelocityLimit(spd, btn);
        });
    });

    if (speedSlider) {
        speedSlider.addEventListener("input", (e) => {
            const spd = parseFloat(e.target.value);
            applyVelocityLimit(spd);
        });
    }

    // Grippers
    const syncCheck = document.getElementById("sync-grippers-check");
    if (syncCheck) {
        syncCheck.addEventListener("change", (e) => {
            syncGrippers = e.target.checked;
        });
    }

    const leftGripperSlider = document.getElementById("slider-gripper-left");
    if (leftGripperSlider) {
        leftGripperSlider.addEventListener("input", (e) => {
            const val = parseFloat(e.target.value);
            const text = formatGripperText(val);
            const disp = document.getElementById("left-gripper-val-display");
            if (disp) disp.textContent = text;
            const j8Slider = document.getElementById("slider-left-7");
            if (j8Slider) j8Slider.value = val;
            const j8Disp = document.getElementById("val-disp-left-7");
            if (j8Disp) j8Disp.textContent = text;
            sendAction("set_gripper", { id: 8, arm: "left", pos: val });

            if (syncGrippers) {
                const rightSlider = document.getElementById("slider-gripper-right");
                if (rightSlider) rightSlider.value = val;
                const rDisp = document.getElementById("right-gripper-val-display");
                if (rDisp) rDisp.textContent = text;
                const rJ8Slider = document.getElementById("slider-right-7");
                if (rJ8Slider) rJ8Slider.value = val;
                const rJ8Disp = document.getElementById("val-disp-right-7");
                if (rJ8Disp) rJ8Disp.textContent = text;
                sendAction("set_gripper", { id: 16, arm: "right", pos: val });
            }
        });
    }

    const rightGripperSlider = document.getElementById("slider-gripper-right");
    if (rightGripperSlider) {
        rightGripperSlider.addEventListener("input", (e) => {
            const val = parseFloat(e.target.value);
            const text = formatGripperText(val);
            const disp = document.getElementById("right-gripper-val-display");
            if (disp) disp.textContent = text;
            const j8Slider = document.getElementById("slider-right-7");
            if (j8Slider) j8Slider.value = val;
            const j8Disp = document.getElementById("val-disp-right-7");
            if (j8Disp) j8Disp.textContent = text;
            sendAction("set_gripper", { id: 16, arm: "right", pos: val });

            if (syncGrippers) {
                const leftSlider = document.getElementById("slider-gripper-left");
                if (leftSlider) leftSlider.value = val;
                const lDisp = document.getElementById("left-gripper-val-display");
                if (lDisp) lDisp.textContent = text;
                const lJ8Slider = document.getElementById("slider-left-7");
                if (lJ8Slider) lJ8Slider.value = val;
                const lJ8Disp = document.getElementById("val-disp-left-7");
                if (lJ8Disp) lJ8Disp.textContent = text;
                sendAction("set_gripper", { id: 8, arm: "left", pos: val });
            }
        });
    }

    // Preset buttons
    document.querySelectorAll(".btn-preset").forEach(btn => {
        btn.addEventListener("click", () => {
            const preset = btn.dataset.preset;
            document.querySelectorAll(".btn-preset").forEach(b => b.classList.remove("active"));
            if (preset !== "stop") btn.classList.add("active");
            if (typeof triggerPreset === "function") triggerPreset(preset);
        });
    });

    // Viewport tools
    const btnResetCam = document.getElementById("btn-reset-cam");
    if (btnResetCam) btnResetCam.addEventListener("click", resetCamera);
    const btnFrontView = document.getElementById("btn-front-view");
    if (btnFrontView) btnFrontView.addEventListener("click", setFrontView);
    const btnLeftView = document.getElementById("btn-left-view");
    if (btnLeftView) btnLeftView.addEventListener("click", setLeftArmView);
    const btnRightView = document.getElementById("btn-right-view");
    if (btnRightView) btnRightView.addEventListener("click", setRightArmView);
    const btnTopView = document.getElementById("btn-top-view");
    if (btnTopView) btnTopView.addEventListener("click", setTopView);

    const btnToggleGrid = document.getElementById("btn-toggle-grid");
    if (btnToggleGrid) {
        btnToggleGrid.addEventListener("click", (e) => {
            if (gridHelper) {
                gridHelper.visible = !gridHelper.visible;
                e.target.textContent = `Grid: ${gridHelper.visible ? 'ON' : 'OFF'}`;
            }
        });
    }

    const btnToggleAxes = document.getElementById("btn-toggle-axes");
    if (btnToggleAxes) {
        btnToggleAxes.addEventListener("click", (e) => {
            if (axesHelper) {
                axesHelper.visible = !axesHelper.visible;
                e.target.textContent = `Axes: ${axesHelper.visible ? 'ON' : 'OFF'}`;
            }
        });
    }

    // Inspector toggle (Terminal & CAN-FD Inspector)
    const btnInspector = document.getElementById("btn-toggle-inspector");
    if (btnInspector) {
        btnInspector.addEventListener("click", () => {
            const grid = document.querySelector(".dashboard-grid");
            const rightCol = document.querySelector(".right-col");
            if (grid && rightCol) {
                const isHidden = window.getComputedStyle(rightCol).display === "none";
                if (isHidden) {
                    rightCol.style.display = "flex";
                    grid.classList.remove("right-col-hidden");
                    btnInspector.classList.add("active");
                } else {
                    rightCol.style.display = "none";
                    grid.classList.add("right-col-hidden");
                    btnInspector.classList.remove("active");
                }
                setTimeout(onWindowResize, 50);
            }
        });
    }

    // Telemetry Panel Collapsible Toggle
    const telemPanel = document.getElementById("telemetry-panel");
    const telemGrid = document.getElementById("telemetry-grid");
    const telemTabs = document.getElementById("telem-tabs-group");
    const btnTelemExpand = document.getElementById("btn-toggle-telem-expand");
    const btnTelemHeader = document.getElementById("btn-toggle-telemetry-header");
    const telemToggleHeader = document.getElementById("telemetry-toggle-header");

    function toggleTelemetryPanel(forceOpen = null) {
        if (!telemPanel) return;
        const isCollapsed = telemPanel.classList.contains("collapsed");
        const willCollapse = (forceOpen !== null) ? !forceOpen : !isCollapsed;

        if (willCollapse) {
            telemPanel.classList.add("collapsed");
            if (telemGrid) telemGrid.style.display = "none";
            if (telemTabs) telemTabs.style.display = "none";
            if (btnTelemExpand) btnTelemExpand.textContent = "Mở rộng ▲";
            if (btnTelemHeader) btnTelemHeader.classList.remove("active");
        } else {
            telemPanel.classList.remove("collapsed");
            if (telemGrid) telemGrid.style.display = "grid";
            if (telemTabs) telemTabs.style.display = "flex";
            if (btnTelemExpand) btnTelemExpand.textContent = "Thu gọn ▼";
            if (btnTelemHeader) btnTelemHeader.classList.add("active");
        }
        setTimeout(onWindowResize, 50);
    }

    if (btnTelemExpand) {
        btnTelemExpand.addEventListener("click", (e) => {
            e.stopPropagation();
            toggleTelemetryPanel();
        });
    }

    if (telemToggleHeader) {
        telemToggleHeader.addEventListener("click", (e) => {
            if (e.target.closest(".telem-tabs")) return;
            toggleTelemetryPanel();
        });
    }

    if (btnTelemHeader) {
        btnTelemHeader.addEventListener("click", () => {
            toggleTelemetryPanel();
        });
    }

    // Telemetry filter tabs
    document.querySelectorAll(".telem-tab-btn").forEach(btn => {
        btn.addEventListener("click", () => {
            document.querySelectorAll(".telem-tab-btn").forEach(b => b.classList.remove("active"));
            btn.classList.add("active");
            telemFilter = btn.dataset.filter;
            if (typeof applyTelemetryFilter === "function") applyTelemetryFilter();
        });
    });

    // Traffic filter tabs
    document.querySelectorAll(".traffic-filter-btn").forEach(btn => {
        btn.addEventListener("click", () => {
            document.querySelectorAll(".traffic-filter-btn").forEach(b => b.classList.remove("active"));
            btn.classList.add("active");
            trafficFilter = btn.dataset.filter;
        });
    });

    // CLI actions
    document.querySelectorAll(".btn-cli").forEach(btn => {
        btn.addEventListener("click", () => {
            const cmd = btn.dataset.cmd;
            if (cmd === "clear_term") {
                const term = document.getElementById("term-output");
                if (term) term.textContent = "Terminal output cleared.";
                return;
            }
            runCliCommand(cmd);
        });
    });

    // 100Hz Joint State Continuous Exporter actions
    const btnExportNew = document.getElementById("btn-export-new");
    if (btnExportNew) {
        btnExportNew.addEventListener("click", () => {
            sendAction("export_new_session");
            showToast("info", "Đã khởi tạo file ghi CSV 100Hz mới!");
        });
    }

    const btnExportToggle = document.getElementById("btn-export-toggle");
    if (btnExportToggle) {
        btnExportToggle.addEventListener("click", () => {
            sendAction("export_toggle");
        });
    }
}
