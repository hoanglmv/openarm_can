// OpenArm Bimanual 7DOF CAN-FD Simulator & Digital Twin Controller
// Supports Dual 7-DOF Robotic Arms (Left & Right) + Dual Grippers (16 CAN Nodes)

const LEFT_JOINTS = [
    { id: 1, name: "L-Joint 1 (Shoulder Pitch)", arm: "left",  idx: 0, type: "DM8009", min: -1.396, max: 3.490, default: 0.0 },
    { id: 2, name: "L-Joint 2 (Shoulder Roll)",  arm: "left",  idx: 1, type: "DM8009", min: -0.174, max: 3.316, default: 0.0 },
    { id: 3, name: "L-Joint 3 (Arm Twist)",      arm: "left",  idx: 2, type: "DM4340", min: -1.570, max: 1.570, default: 0.0 },
    { id: 4, name: "L-Joint 4 (Elbow Pitch)",    arm: "left",  idx: 3, type: "DM4340", min:  0.000, max: 2.443, default: 0.0 },
    { id: 5, name: "L-Joint 5 (Forearm Twist)",  arm: "left",  idx: 4, type: "DM4310", min: -1.570, max: 1.570, default: 0.0 },
    { id: 6, name: "L-Joint 6 (Wrist Pitch)",    arm: "left",  idx: 5, type: "DM4310", min: -0.785, max: 0.785, default: 0.0 },
    { id: 7, name: "L-Joint 7 (Wrist Roll)",     arm: "left",  idx: 6, type: "DM4310", min: -1.570, max: 1.570, default: 0.0 },
    { id: 8, name: "L-Gripper (J8 Kẹp Ngang)",   arm: "left",  idx: 7, type: "DM4310", min:  0.000, max: 0.043, default: 0.0 }
];

const RIGHT_JOINTS = [
    { id: 9,  name: "R-Joint 1 (Shoulder Pitch)", arm: "right", idx: 0, type: "DM8009", min: -1.396, max: 3.490, default: 0.0 },
    { id: 10, name: "R-Joint 2 (Shoulder Roll)",  arm: "right", idx: 1, type: "DM8009", min: -0.174, max: 3.316, default: 0.0 },
    { id: 11, name: "R-Joint 3 (Arm Twist)",      arm: "right", idx: 2, type: "DM4340", min: -1.570, max: 1.570, default: 0.0 },
    { id: 12, name: "R-Joint 4 (Elbow Pitch)",    arm: "right", idx: 3, type: "DM4340", min:  0.000, max: 2.443, default: 0.0 },
    { id: 13, name: "R-Joint 5 (Forearm Twist)",  arm: "right", idx: 4, type: "DM4310", min: -1.570, max: 1.570, default: 0.0 },
    { id: 14, name: "R-Joint 6 (Wrist Pitch)",    arm: "right", idx: 5, type: "DM4310", min: -0.785, max: 0.785, default: 0.0 },
    { id: 15, name: "R-Joint 7 (Wrist Roll)",     arm: "right", idx: 6, type: "DM4310", min: -1.570, max: 1.570, default: 0.0 },
    { id: 16, name: "R-Gripper (J8 Kẹp Ngang)",   arm: "right", idx: 7, type: "DM4310", min:  0.000, max: 0.043, default: 0.0 }
];

const ALL_JOINTS = [...LEFT_JOINTS, ...RIGHT_JOINTS];

let ws = null;
let scene, camera, renderer, controls;

// Kinematics containers
let leftArmJoints = [];  // J0..J6
let rightArmJoints = []; // J0..J6
let leftGripperFingers = { left: null, right: null };
let rightGripperFingers = { left: null, right: null };

let gridHelper, axesHelper, floorMesh;
let activePreset = null;
let presetTimer = null;
let currentTelemetry = [];
let isLightTheme = true;

// Active slider tab: 'left', 'right', or 'sync'
let currentArmTab = 'left';
let syncGrippers = true;
let telemFilter = 'all';
let trafficFilter = 'all';

function formatGripperText(val) {
    const mm = (val * 1000).toFixed(1);
    const status = val < 0.002 ? 'Đóng' : (val >= 0.040 ? 'Mở tối đa' : `${mm} mm`);
    return `${mm} mm (${status})`;
}

// Initialize DOM and App
document.addEventListener("DOMContentLoaded", () => {
    document.body.classList.add("theme-light");

    buildJointSliders();
    buildTelemetryCards();
    initThreeJS();
    initWebSocket();
    setupEventHandlers();
});

let jointLockStates = {};
for (let i = 1; i <= 16; i++) {
    jointLockStates[i] = true;
}

// Build Sliders for Arm Joints & Gripper (J1-J7 + J8 Gripper) - Ultra Compact & Fitted with Joint Safety Locks
function buildJointSliders() {
    const list = document.getElementById("sliders-list");
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

    const JOINT_NAMES = [
        "Shoulder Pitch",
        "Shoulder Roll (Abduct)",
        "Arm Twist",
        "Elbow Pitch",
        "Forearm Twist",
        "Wrist Pitch",
        "Wrist Roll",
        "Gripper J8 (Thanh kẹp ngang)"
    ];

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
                    // Mirrored yaw/roll for symmetrical bimanual gestures
                    const mirrorSign = (j.idx === 0 || j.idx === 2 || j.idx === 4 || j.idx === 6) ? -1.0 : 1.0;

                    sendAction("set_mit", { id: leftMotorId, q: val, kp: 30.0, kd: 1.2, tau: 0.0 });
                    sendAction("set_mit", { id: rightMotorId, q: val * mirrorSign, kp: 30.0, kd: 1.2, tau: 0.0 });
                }
            }
        });
    });
}

// Build Telemetry Cards for 16 Motors
function buildTelemetryCards() {
    const grid = document.getElementById("telemetry-grid");
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

// Setup Event Handlers
function setupEventHandlers() {
    // Theme toggle
    const btnTheme = document.getElementById("btn-theme-toggle");
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

    // Arm tab switching
    document.getElementById("tab-arm-left").addEventListener("click", (e) => {
        setArmTab("left", e.target);
    });
    document.getElementById("tab-arm-right").addEventListener("click", (e) => {
        setArmTab("right", e.target);
    });
    document.getElementById("tab-arm-both").addEventListener("click", (e) => {
        setArmTab("both", e.target);
    });
    document.getElementById("tab-arm-sync").addEventListener("click", (e) => {
        setArmTab("sync", e.target);
    });

    // Master actions
    document.getElementById("btn-enable-all").addEventListener("click", () => {
        for (let i = 1; i <= 16; i++) jointLockStates[i] = true;
        sendAction("enable_all");
        buildJointSliders();
    });
    document.getElementById("btn-disable-all").addEventListener("click", () => {
        stopPresets();
        for (let i = 1; i <= 16; i++) jointLockStates[i] = false;
        sendAction("disable_all");
        buildJointSliders();
    });
    document.getElementById("btn-zero-all").addEventListener("click", () => {
        // 1. Reset all joint slider bars in the list to 0.0
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

        // 2. Reset top-level gripper sliders and displays
        const leftG = document.getElementById("slider-gripper-left");
        const rightG = document.getElementById("slider-gripper-right");
        if (leftG) leftG.value = 0.0;
        if (rightG) rightG.value = 0.0;
        const leftGDisp = document.getElementById("left-gripper-val-display");
        const rightGDisp = document.getElementById("right-gripper-val-display");
        if (leftGDisp) leftGDisp.textContent = "0.0 mm (Đóng)";
        if (rightGDisp) rightGDisp.textContent = "0.0 mm (Đóng)";

        // 3. Immediately reset 3D Digital Twin model to natural resting pose
        leftArmJoints.forEach(j => {
            if (j && j.group) j.group.rotation.set(0, 0, 0);
        });
        rightArmJoints.forEach(j => {
            if (j && j.group) j.group.rotation.set(0, 0, 0);
        });
        if (leftGripperFingers.left && leftGripperFingers.right) {
            leftGripperFingers.left.position.x = -0.010;
            leftGripperFingers.right.position.x = 0.010;
        }
        if (rightGripperFingers.left && rightGripperFingers.right) {
            rightGripperFingers.left.position.x = -0.010;
            rightGripperFingers.right.position.x = 0.010;
        }

        // 4. Send zero calibration sequence to server & physical motors
        sendAction("set_zero_all");
    });
    document.getElementById("btn-clear-err").addEventListener("click", () => {
        sendAction("clear_error_all");
    });

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
    syncCheck.addEventListener("change", (e) => {
        syncGrippers = e.target.checked;
    });

    const leftGripperSlider = document.getElementById("slider-gripper-left");
    leftGripperSlider.addEventListener("input", (e) => {
        const val = parseFloat(e.target.value);
        const text = formatGripperText(val);
        document.getElementById("left-gripper-val-display").textContent = text;
        const j8Slider = document.getElementById("slider-left-7");
        if (j8Slider) j8Slider.value = val;
        const j8Disp = document.getElementById("val-disp-left-7");
        if (j8Disp) j8Disp.textContent = text;
        sendAction("set_gripper", { id: 8, arm: "left", pos: val });

        if (syncGrippers) {
            const rightSlider = document.getElementById("slider-gripper-right");
            rightSlider.value = val;
            document.getElementById("right-gripper-val-display").textContent = text;
            const rJ8Slider = document.getElementById("slider-right-7");
            if (rJ8Slider) rJ8Slider.value = val;
            const rJ8Disp = document.getElementById("val-disp-right-7");
            if (rJ8Disp) rJ8Disp.textContent = text;
            sendAction("set_gripper", { id: 16, arm: "right", pos: val });
        }
    });

    const rightGripperSlider = document.getElementById("slider-gripper-right");
    rightGripperSlider.addEventListener("input", (e) => {
        const val = parseFloat(e.target.value);
        const text = formatGripperText(val);
        document.getElementById("right-gripper-val-display").textContent = text;
        const j8Slider = document.getElementById("slider-right-7");
        if (j8Slider) j8Slider.value = val;
        const j8Disp = document.getElementById("val-disp-right-7");
        if (j8Disp) j8Disp.textContent = text;
        sendAction("set_gripper", { id: 16, arm: "right", pos: val });

        if (syncGrippers) {
            const leftSlider = document.getElementById("slider-gripper-left");
            leftSlider.value = val;
            document.getElementById("left-gripper-val-display").textContent = text;
            const lJ8Slider = document.getElementById("slider-left-7");
            if (lJ8Slider) lJ8Slider.value = val;
            const lJ8Disp = document.getElementById("val-disp-left-7");
            if (lJ8Disp) lJ8Disp.textContent = text;
            sendAction("set_gripper", { id: 8, arm: "left", pos: val });
        }
    });

    // Preset buttons
    document.querySelectorAll(".btn-preset").forEach(btn => {
        btn.addEventListener("click", () => {
            const preset = btn.dataset.preset;
            document.querySelectorAll(".btn-preset").forEach(b => b.classList.remove("active"));
            if (preset !== "stop") btn.classList.add("active");
            triggerPreset(preset);
        });
    });

    // Viewport tools
    document.getElementById("btn-reset-cam").addEventListener("click", resetCamera);
    document.getElementById("btn-front-view").addEventListener("click", setFrontView);
    const btnLeftView = document.getElementById("btn-left-view");
    if (btnLeftView) btnLeftView.addEventListener("click", setLeftArmView);
    const btnRightView = document.getElementById("btn-right-view");
    if (btnRightView) btnRightView.addEventListener("click", setRightArmView);
    document.getElementById("btn-top-view").addEventListener("click", setTopView);
    document.getElementById("btn-toggle-grid").addEventListener("click", (e) => {
        gridHelper.visible = !gridHelper.visible;
        e.target.textContent = `Grid: ${gridHelper.visible ? 'ON' : 'OFF'}`;
    });
    document.getElementById("btn-toggle-axes").addEventListener("click", (e) => {
        axesHelper.visible = !axesHelper.visible;
        e.target.textContent = `Axes: ${axesHelper.visible ? 'ON' : 'OFF'}`;
    });

    // Inspector toggle
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
                    btnInspector.classList.remove("active");
                } else {
                    rightCol.style.display = "none";
                    grid.classList.add("right-col-hidden");
                    btnInspector.classList.add("active");
                }
                setTimeout(onWindowResize, 50);
            }
        });
    }

    // Telemetry filters
    document.querySelectorAll(".telem-tab-btn").forEach(btn => {
        btn.addEventListener("click", () => {
            document.querySelectorAll(".telem-tab-btn").forEach(b => b.classList.remove("active"));
            btn.classList.add("active");
            telemFilter = btn.dataset.filter;
            applyTelemetryFilter();
        });
    });

    // Traffic filters
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
                document.getElementById("term-output").textContent = "Terminal output cleared.";
                return;
            }
            runCliCommand(cmd);
        });
    });
}

function setArmTab(tab, element) {
    currentArmTab = tab;
    document.querySelectorAll(".arm-tab-btn").forEach(btn => {
        btn.classList.remove("active", "tab-left", "tab-right", "tab-sync");
    });
    element.classList.add("active");
    if (tab === 'left') element.classList.add("tab-left");
    else if (tab === 'right') element.classList.add("tab-right");
    else if (tab === 'sync') element.classList.add("tab-sync");
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
        if (disp) disp.textContent = "0.00 rad (Open)";
        if (group === 'sync') {
            sendAction("set_zero_single", { id: 8 });
            sendAction("set_zero_single", { id: 16 });
            sendAction("set_gripper", { id: 8, arm: "left", pos: 0.0 });
            sendAction("set_gripper", { id: 16, arm: "right", pos: 0.0 });
            const topL = document.getElementById("slider-gripper-left");
            const topR = document.getElementById("slider-gripper-right");
            if (topL) topL.value = 0.0;
            if (topR) topR.value = 0.0;
            const dL = document.getElementById("left-gripper-val-display");
            const dR = document.getElementById("right-gripper-val-display");
            if (dL) dL.textContent = "0.00 rad (Open)";
            if (dR) dR.textContent = "0.00 rad (Open)";
        } else {
            const arm = (id === 8) ? "left" : "right";
            sendAction("set_zero_single", { id });
            sendAction("set_gripper", { id, arm, pos: 0.0 });
            const topSlider = document.getElementById(arm === 'left' ? "slider-gripper-left" : "slider-gripper-right");
            if (topSlider) topSlider.value = 0.0;
            const topDisp = document.getElementById(arm === 'left' ? "left-gripper-val-display" : "right-gripper-val-display");
            if (topDisp) topDisp.textContent = "0.00 rad (Open)";
        }
    } else {
        if (disp) disp.textContent = "0.00 rad (0°)";
        if (group === 'sync') {
            const leftId = LEFT_JOINTS[idx].id;
            const rightId = RIGHT_JOINTS[idx].id;
            sendAction("set_zero_single", { id: leftId });
            sendAction("set_zero_single", { id: rightId });
            sendAction("set_mit", { id: leftId, q: 0.0, kp: 30.0, kd: 1.2, tau: 0.0 });
            sendAction("set_mit", { id: rightId, q: 0.0, kp: 30.0, kd: 1.2, tau: 0.0 });
        } else {
            sendAction("set_zero_single", { id });
            sendAction("set_mit", { id, q: 0.0, kp: 30.0, kd: 1.2, tau: 0.0 });
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

// WebSocket Connection
function initWebSocket() {
    const wsHost = window.location.hostname || "127.0.0.1";
    const wsUrl = `ws://${wsHost}:8889`;
    ws = new WebSocket(wsUrl);

    const badge = document.getElementById("ws-status-badge");

    ws.onopen = () => {
        badge.className = "badge badge-success";
        badge.innerHTML = "WebSocket: <strong>CONNECTED</strong>";
    };

    ws.onmessage = (event) => {
        try {
            const msg = JSON.parse(event.data);
            if (msg.type === "telemetry") {
                handleTelemetry(msg.data);
            } else if (msg.type === "traffic") {
                handleTraffic(msg.data);
            } else if (msg.type === "cli_output") {
                handleCliOutput(msg.data);
            }
        } catch (e) {
            console.error("WS Parse Error:", e);
        }
    };

    ws.onclose = () => {
        badge.className = "badge badge-danger";
        badge.innerHTML = "WebSocket: <strong>DISCONNECTED</strong>";
        setTimeout(initWebSocket, 2000);
    };

    ws.onerror = () => {
        badge.className = "badge badge-danger";
        badge.innerHTML = "WebSocket: <strong>ERROR</strong>";
    };
}

function sendAction(action, payload = {}) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ action, ...payload }));
    }
}

function runCliCommand(cmd) {
    const term = document.getElementById("term-output");
    term.textContent = `[Executing openarm-can-cli ${cmd} on vcan0...]\n`;
    sendAction("run_cli", { cmd });
}

function handleCliOutput(data) {
    const term = document.getElementById("term-output");
    term.textContent = data;
}

// Telemetry & 3D Kinematics Synchronizer
function handleTelemetry(data) {
    const motors = data.motors || [];
    currentTelemetry = motors;

    document.getElementById("stat-rx").textContent = data.frames_rx || 0;
    document.getElementById("stat-tx").textContent = data.frames_tx || 0;

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
                    // J2: Shoulder Roll / Abduction (swings arm outward away from torso around Z)
                    // Left arm swings outward to -X (+Z rotation)
                    // Right arm swings outward to +X (-Z rotation)
                    jEntry.group.rotation.z = isLeft ? angle : -angle;
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

        // Bi-directional Synchronization: Update UI Sliders to match live robot state in real time
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
}

// CAN Traffic Inspector Table
function handleTraffic(packets) {
    if (!packets || packets.length === 0) return;
    const tbody = document.getElementById("traffic-tbody");

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
                <td class="${dirClass}">${p.dir}</td>
                <td><strong>${p.id}</strong></td>
                <td>${p.dlc}B</td>
                <td style="letter-spacing: 0.5px;">${p.hex}</td>
            </tr>
        `;
    }).join("");
}

// ----------------------------------------------------
// THREE.JS 3D BIMANUAL MODEL BUILDER (MATCHING OFFICIAL OPENARM)
// ----------------------------------------------------
function initThreeJS() {
    const container = document.getElementById("three-container");
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf1f5f9);

    camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.05, 50);
    resetCamera();

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.15;
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.06;
    controls.target.set(0, 0.40, 0);

    // Studio Lighting Setup
    const ambientLight = new THREE.AmbientLight(0xffffff, 1.3);
    scene.add(ambientLight);

    const mainKeyLight = new THREE.DirectionalLight(0xffffff, 1.7);
    mainKeyLight.position.set(2.0, 3.5, 2.5);
    mainKeyLight.castShadow = true;
    mainKeyLight.shadow.mapSize.width = 2048;
    mainKeyLight.shadow.mapSize.height = 2048;
    mainKeyLight.shadow.bias = -0.0001;
    scene.add(mainKeyLight);

    const fillLight = new THREE.DirectionalLight(0xbfdbfe, 0.85);
    fillLight.position.set(-2.5, 2.5, 1.5);
    scene.add(fillLight);

    const rimLight = new THREE.DirectionalLight(0xffffff, 0.8);
    rimLight.position.set(0, 2.5, -2.5);
    scene.add(rimLight);

    // Ground Floor & Grid
    const floorGeo = new THREE.PlaneGeometry(16, 16);
    const floorMat = new THREE.MeshStandardMaterial({
        color: 0xeef2f6,
        roughness: 0.65,
        metalness: 0.1
    });
    floorMesh = new THREE.Mesh(floorGeo, floorMat);
    floorMesh.rotation.x = -Math.PI / 2;
    floorMesh.receiveShadow = true;
    scene.add(floorMesh);

    gridHelper = new THREE.GridHelper(8, 32, 0x0284c7, 0x94a3b8);
    gridHelper.position.y = 0.001;
    scene.add(gridHelper);

    axesHelper = new THREE.AxesHelper(0.35);
    axesHelper.position.y = 0.01;
    scene.add(axesHelper);

    // Build the complete Bimanual Robot (Pillar, Torso, Left Arm, Right Arm)
    buildBimanualOpenArm();

    window.addEventListener("resize", onWindowResize);
    animate();
}

function resetCamera() {
    camera.position.set(0.0, 0.46, 1.25);
    if (controls) controls.target.set(0, 0.40, 0);
}

function setFrontView() {
    camera.position.set(0, 0.42, 1.20);
    if (controls) controls.target.set(0, 0.40, 0);
}

function setLeftArmView() {
    camera.position.set(-0.28, 0.38, 0.65);
    if (controls) controls.target.set(-0.11, 0.35, 0);
}

function setRightArmView() {
    camera.position.set(0.28, 0.38, 0.65);
    if (controls) controls.target.set(0.11, 0.35, 0);
}

function setTopView() {
    camera.position.set(0, 1.35, 0.05);
    if (controls) controls.target.set(0, 0.40, 0);
}

function onWindowResize() {
    const container = document.getElementById("three-container");
    if (!container) return;
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
}

// Construct Bimanual Robot Assembly Matching Reference Image
function buildBimanualOpenArm() {
    // High-definition Materials (Matte Black, Machined Silver Aluminum, Motor Casings)
    const matteBlackMat = new THREE.MeshStandardMaterial({
        color: 0x1d1f25,
        roughness: 0.38,
        metalness: 0.18
    });
    const motorCasingMat = new THREE.MeshStandardMaterial({
        color: 0x131417,
        roughness: 0.28,
        metalness: 0.55
    });
    const silverMetalMat = new THREE.MeshStandardMaterial({
        color: 0xdde2ea,
        roughness: 0.18,
        metalness: 0.90
    });
    const darkExtrusionMat = new THREE.MeshStandardMaterial({
        color: 0x22252c,
        roughness: 0.45,
        metalness: 0.65
    });
    const basePlateMat = new THREE.MeshStandardMaterial({
        color: 0xbac0ca,
        roughness: 0.22,
        metalness: 0.85
    });
    const cableMat = new THREE.MeshStandardMaterial({
        color: 0x0a0b0d,
        roughness: 0.85,
        metalness: 0.05
    });
    const labelMat = new THREE.MeshBasicMaterial({
        color: 0xf3f4f6
    });

    const rootGroup = new THREE.Group();
    scene.add(rootGroup);

    // 1. Heavy Machined Base Plate (Silver aluminum plate as in photo)
    const basePlate = new THREE.Mesh(new THREE.BoxGeometry(0.36, 0.02, 0.40), basePlateMat);
    basePlate.position.y = 0.01;
    basePlate.castShadow = true;
    basePlate.receiveShadow = true;
    rootGroup.add(basePlate);

    // Base Mounting Holes & Counter-sunk Bolts
    [[-0.15, -0.17], [-0.15, 0.17], [0.15, -0.17], [0.15, 0.17]].forEach(([bx, bz]) => {
        const holeRing = new THREE.Mesh(new THREE.CylinderGeometry(0.011, 0.011, 0.021, 16), darkExtrusionMat);
        holeRing.position.set(bx, 0.01, bz);
        rootGroup.add(holeRing);
    });

    // Optical Breadboard Grid of Tapped Holes on Silver Base Plate (as in photo)
    const breadboardHoleMat = new THREE.MeshBasicMaterial({ color: 0x22262e });
    for (let hx = -0.15; hx <= 0.151; hx += 0.05) {
        for (let hz = -0.16; hz <= 0.161; hz += 0.04) {
            if (Math.abs(hx) < 0.055 && Math.abs(hz) < 0.065) continue; // under pillar
            const holeDot = new THREE.Mesh(new THREE.CircleGeometry(0.0035, 12), breadboardHoleMat);
            holeDot.rotation.x = -Math.PI / 2;
            holeDot.position.set(hx, 0.0205, hz);
            rootGroup.add(holeDot);
        }
    }

    // Front Machined Silver Curved Mounting Bracket (as in photo)
    const frontBracket = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.065, 0.008), silverMetalMat);
    frontBracket.position.set(0, 0.052, 0.044);
    rootGroup.add(frontBracket);

    const frontFoot = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.008, 0.040), silverMetalMat);
    frontFoot.position.set(0, 0.024, 0.060);
    rootGroup.add(frontFoot);

    // Socket screws on front foot
    [-0.012, 0.012].forEach(fx => {
        const scr = new THREE.Mesh(new THREE.CylinderGeometry(0.0035, 0.0035, 0.006, 12), darkExtrusionMat);
        scr.position.set(fx, 0.0285, 0.065);
        rootGroup.add(scr);
    });

    // Triangular Gusset Corner Brackets (Holding column to base plate)
    [-0.065, 0.065].forEach(gx => {
        const gussetGeo = new THREE.BufferGeometry();
        const verts = new Float32Array([
            gx, 0.02, -0.045,
            gx, 0.02,  0.045,
            gx * 0.72, 0.15, -0.045,
            gx * 0.72, 0.15, -0.045,
            gx, 0.02,  0.045,
            gx * 0.72, 0.15,  0.045,
        ]);
        gussetGeo.setAttribute('position', new THREE.BufferAttribute(verts, 3));
        gussetGeo.computeVertexNormals();
        const gussetMesh = new THREE.Mesh(gussetGeo, silverMetalMat);
        rootGroup.add(gussetMesh);

        // Mounting bolt head on gusset
        const gBolt = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.007, 0.015, 12), silverMetalMat);
        gBolt.rotation.z = Math.PI / 2;
        gBolt.position.set(gx * 1.05, 0.05, 0);
        rootGroup.add(gBolt);
    });

    // Main Power / CAN Cable coming from base to the left (as in photo)
    const baseCable = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.35, 12), cableMat);
    baseCable.rotation.z = Math.PI / 2;
    baseCable.position.set(-0.25, 0.015, -0.05);
    rootGroup.add(baseCable);

    // 2. Central Pillar (Heavy Black T-slot Aluminum Extrusion Profile)
    const pillarHeight = 0.68;
    const pillar = new THREE.Mesh(new THREE.BoxGeometry(0.08, pillarHeight, 0.08), darkExtrusionMat);
    pillar.position.y = 0.02 + pillarHeight / 2;
    pillar.castShadow = true;
    rootGroup.add(pillar);

    // Extrusion T-slot Grooves on all 4 faces
    const slotMat = new THREE.MeshBasicMaterial({ color: 0x0c0e12 });
    [-0.041, 0.041].forEach(x => {
        const slot = new THREE.Mesh(new THREE.BoxGeometry(0.003, pillarHeight, 0.014), slotMat);
        slot.position.set(x, 0.02 + pillarHeight / 2, 0);
        rootGroup.add(slot);
    });
    [-0.041, 0.041].forEach(z => {
        const slot = new THREE.Mesh(new THREE.BoxGeometry(0.014, pillarHeight, 0.003), slotMat);
        slot.position.set(0, 0.02 + pillarHeight / 2, z);
        rootGroup.add(slot);
    });

    // Black Pillar Cable Strap (around mid-height, as in photo)
    const strap = new THREE.Mesh(new THREE.BoxGeometry(0.084, 0.025, 0.084), cableMat);
    strap.position.y = 0.02 + pillarHeight * 0.45;
    rootGroup.add(strap);

    // 3. Central Torso Unit (At top of column)
    const torsoY = 0.02 + pillarHeight;
    const torso = new THREE.Group();
    torso.position.y = torsoY;
    rootGroup.add(torso);

    // Main Torso Housing - Sleek sculpted matte black cowl (matching photo width)
    const torsoBody = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.13, 0.14), matteBlackMat);
    torsoBody.position.y = 0.05;
    torsoBody.castShadow = true;
    torso.add(torsoBody);

    // Torso Beveled Top Cap (Trapezoidal crown)
    const torsoCrown = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.09, 0.045, 32), matteBlackMat);
    torsoCrown.scale.set(1.0, 1.0, 0.85);
    torsoCrown.position.y = 0.13;
    torsoCrown.castShadow = true;
    torso.add(torsoCrown);

    // Center Front Lower Lip / Chin Notch
    const torsoChin = new THREE.Mesh(new THREE.BoxGeometry(0.055, 0.035, 0.015), matteBlackMat);
    torsoChin.position.set(0, -0.01, 0.072);
    torso.add(torsoChin);

    // Torso Front Logo (Authentic OpenArm 3-Node Connected Network Emblem as in photo)
    const logoGroup = new THREE.Group();
    logoGroup.position.set(0, 0.065, 0.072);
    torso.add(logoGroup);

    const logoMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const n1 = new THREE.Mesh(new THREE.CircleGeometry(0.005, 16), logoMat);
    n1.position.set(-0.014, 0.012, 0.001);
    logoGroup.add(n1);

    const n2 = new THREE.Mesh(new THREE.CircleGeometry(0.005, 16), logoMat);
    n2.position.set(0.000, -0.010, 0.001);
    logoGroup.add(n2);

    const n3 = new THREE.Mesh(new THREE.CircleGeometry(0.005, 16), logoMat);
    n3.position.set(0.014, 0.012, 0.001);
    logoGroup.add(n3);

    // Connecting bars
    function makeBar(p1, p2) {
        const dx = p2.x - p1.x;
        const dy = p2.y - p1.y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        const bar = new THREE.Mesh(new THREE.PlaneGeometry(0.002, dist), logoMat);
        bar.position.set((p1.x + p2.x) / 2, (p1.y + p2.y) / 2, 0.0008);
        bar.rotation.z = Math.atan2(dy, dx) - Math.PI / 2;
        return bar;
    }
    logoGroup.add(makeBar({x: -0.014, y: 0.012}, {x: 0, y: -0.010}));
    logoGroup.add(makeBar({x: 0, y: -0.010}, {x: 0.014, y: 0.012}));

    // 4. Build Left Arm and Right Arm - Tightly and gracefully flanking the column
    buildSingleArm(torso, "left",  -0.075, 0.045, 0, matteBlackMat, motorCasingMat, silverMetalMat, cableMat, labelMat);
    buildSingleArm(torso, "right",  0.075, 0.045, 0, matteBlackMat, motorCasingMat, silverMetalMat, cableMat, labelMat);
}

// Single 7-DOF Arm + 2-Finger Gripper Kinematic Assembly
function buildSingleArm(parent, side, offsetX, offsetY, offsetZ, blackMat, motorMat, silverMat, cableMat, labelMat) {
    const isLeft = side === "left";
    const sign = isLeft ? -1 : 1;
    const armJoints = isLeft ? leftArmJoints : rightArmJoints;
    const gripperFingers = isLeft ? leftGripperFingers : rightGripperFingers;

    // Shoulder Base Collar on Torso
    const shoulderMount = new THREE.Group();
    shoulderMount.position.set(offsetX, offsetY, offsetZ);
    parent.add(shoulderMount);

    // Torso side collar socket
    const collar = new THREE.Mesh(new THREE.CylinderGeometry(0.052, 0.052, 0.020, 28), blackMat);
    collar.rotation.z = Math.PI / 2;
    shoulderMount.add(collar);

    // Inner Silver Accent Ring between Torso & Shoulder Pod
    const innerSilverRing = new THREE.Mesh(new THREE.TorusGeometry(0.0525, 0.003, 16, 32), silverMat);
    innerSilverRing.rotation.y = Math.PI / 2;
    innerSilverRing.position.x = -sign * 0.003;
    shoulderMount.add(innerSilverRing);

    // ====================================================
    // Joint 1: Shoulder Pitch (rotates around X axis)
    // Swings the entire arm forward / backward
    // ====================================================
    const j1 = new THREE.Group();
    shoulderMount.add(j1);
    armJoints[0] = { group: j1, axis: 'x' };

    // Lateral Shoulder Motor Pod (horizontal cylinder pointing outwards)
    const shoulderPod = new THREE.Mesh(new THREE.CylinderGeometry(0.050, 0.050, 0.042, 28), motorMat);
    shoulderPod.rotation.z = Math.PI / 2;
    shoulderPod.position.x = sign * 0.020;
    shoulderPod.castShadow = true;
    j1.add(shoulderPod);

    // Second Silver Accent / Bearing Ring on outer shoulder pod
    const outerSilverRing = new THREE.Mesh(new THREE.TorusGeometry(0.0505, 0.003, 16, 32), silverMat);
    outerSilverRing.rotation.y = Math.PI / 2;
    outerSilverRing.position.x = sign * 0.038;
    j1.add(outerSilverRing);

    // Outer Shoulder Cap: Spherical dome with recessed circular hub
    const shoulderDome = new THREE.Mesh(new THREE.SphereGeometry(0.050, 24, 16, 0, Math.PI), blackMat);
    shoulderDome.rotation.y = sign > 0 ? 0 : Math.PI;
    shoulderDome.position.x = sign * 0.040;
    shoulderDome.scale.set(0.35, 1.0, 1.0);
    j1.add(shoulderDome);

    const shoulderCenterCap = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.004, 20), silverMat);
    shoulderCenterCap.rotation.z = Math.PI / 2;
    shoulderCenterCap.position.x = sign * 0.048;
    j1.add(shoulderCenterCap);

    // ====================================================
    // Joint 2: Shoulder Roll / Abduction (rotates around Z axis)
    // Hinge located at outer shoulder pod, swings arm outward away from torso
    // ====================================================
    const j2 = new THREE.Group();
    j2.position.set(sign * 0.036, 0, 0);
    j1.add(j2);
    armJoints[1] = { group: j2, axis: 'z' };

    // J2 Pitch Hinge Hub
    const j2Hub = new THREE.Mesh(new THREE.CylinderGeometry(0.050, 0.050, 0.065, 24), motorMat);
    j2Hub.rotation.z = Math.PI / 2;
    j2Hub.castShadow = true;
    j2.add(j2Hub);

    // Upper Arm Link: Ergonomic curved sculpted black casing (~0.23m long)
    const upperArmLength = 0.23;
    const upperArmBody = new THREE.Mesh(new THREE.CylinderGeometry(0.042, 0.036, upperArmLength, 24), blackMat);
    upperArmBody.position.y = -upperArmLength / 2;
    upperArmBody.castShadow = true;
    j2.add(upperArmBody);

    // Distinctive Machined Silver Aluminum Side Plate (as in photo)
    // Runs down the lateral outer side of the upper arm with visible socket screws
    const silverPlate = new THREE.Mesh(new THREE.BoxGeometry(0.005, upperArmLength * 0.88, 0.042), silverMat);
    silverPlate.position.set(sign * 0.038, -upperArmLength * 0.48, 0);
    silverPlate.castShadow = true;
    j2.add(silverPlate);

    // 4 Rivet / Socket Screws along the silver plate
    [-0.07, -0.02, 0.03, 0.08].forEach(py => {
        const screw = new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.007, 12), silverMat);
        screw.rotation.z = Math.PI / 2;
        screw.position.set(sign * 0.042, -upperArmLength * 0.48 + py, 0);
        j2.add(screw);
    });

    // Cable harness loop running behind the shoulder/arm
    const shoulderCable = new THREE.Mesh(new THREE.TorusGeometry(0.045, 0.006, 12, 20, Math.PI), cableMat);
    shoulderCable.rotation.x = Math.PI / 2;
    shoulderCable.position.set(0, -0.03, -0.035);
    j2.add(shoulderCable);

    // ====================================================
    // Joint 3: Elbow Roll (rotates around Y axis)
    // ====================================================
    const j3 = new THREE.Group();
    j3.position.y = -upperArmLength;
    j2.add(j3);
    armJoints[2] = { group: j3, axis: 'y' };

    // In-line roll actuator cylinder with silver accent ring
    const j3Actuator = new THREE.Mesh(new THREE.CylinderGeometry(0.037, 0.037, 0.032, 24), motorMat);
    j3Actuator.position.y = -0.016;
    j3Actuator.castShadow = true;
    j3.add(j3Actuator);

    const j3SilverRing = new THREE.Mesh(new THREE.TorusGeometry(0.0375, 0.003, 16, 28), silverMat);
    j3SilverRing.rotation.x = Math.PI / 2;
    j3SilverRing.position.y = -0.026;
    j3.add(j3SilverRing);

    // ====================================================
    // Joint 4: Elbow Pitch (rotates around X axis)
    // Features authentic DM4340 motor with circular bolt pattern & spec label
    // ====================================================
    const j4 = new THREE.Group();
    j4.position.y = -0.045;
    j3.add(j4);
    armJoints[3] = { group: j4, axis: 'x' };

    // DM4340 Motor Body (horizontal cylinder)
    const dm4340Body = new THREE.Mesh(new THREE.CylinderGeometry(0.040, 0.040, 0.068, 24), motorMat);
    dm4340Body.rotation.z = Math.PI / 2;
    dm4340Body.castShadow = true;
    j4.add(dm4340Body);

    // DM4340 Outer Silver Faceplate with bolt circle pattern (as in photo)
    const dm4340Face = new THREE.Mesh(new THREE.CylinderGeometry(0.039, 0.039, 0.006, 24), silverMat);
    dm4340Face.rotation.z = Math.PI / 2;
    dm4340Face.position.x = sign * 0.036;
    j4.add(dm4340Face);

    // 6 Hex Screws on bolt circle
    for (let i = 0; i < 6; i++) {
        const angle = (i * Math.PI) / 3;
        const sRad = 0.026;
        const bY = Math.cos(angle) * sRad;
        const bZ = Math.sin(angle) * sRad;
        const bolt = new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.008, 8), motorMat);
        bolt.rotation.z = Math.PI / 2;
        bolt.position.set(sign * 0.039, bY, bZ);
        j4.add(bolt);
    }

    // Center Bearing Hub
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.009, 16), motorMat);
    hub.rotation.z = Math.PI / 2;
    hub.position.x = sign * 0.038;
    j4.add(hub);

    // DM4340 Specification Label (crisp white label on black body)
    const labelBand = new THREE.Mesh(new THREE.CylinderGeometry(0.0402, 0.0402, 0.024, 24, 1, false, 0, Math.PI * 0.8), labelMat);
    labelBand.rotation.z = Math.PI / 2;
    labelBand.rotation.y = -Math.PI / 2;
    j4.add(labelBand);

    // Forearm Link: Sleek black link (~0.19m long)
    const forearmLength = 0.19;
    const forearmBody = new THREE.Mesh(new THREE.CylinderGeometry(0.034, 0.030, forearmLength, 24), blackMat);
    forearmBody.position.y = -forearmLength / 2;
    forearmBody.castShadow = true;
    j4.add(forearmBody);

    // Forearm Silver Bracket Accent & Cable Clip (as in photo)
    const forearmBracket = new THREE.Mesh(new THREE.BoxGeometry(0.004, 0.08, 0.034), silverMat);
    forearmBracket.position.set(sign * 0.031, -forearmLength * 0.45, 0);
    j4.add(forearmBracket);

    // Elbow under-joint cable loop
    const elbowCable = new THREE.Mesh(new THREE.TorusGeometry(0.038, 0.005, 12, 20, Math.PI * 0.8), cableMat);
    elbowCable.rotation.z = Math.PI / 2;
    elbowCable.position.set(0, -0.03, 0.032);
    j4.add(elbowCable);

    // ====================================================
    // Joint 5: Wrist Roll (rotates around Y axis)
    // ====================================================
    const j5 = new THREE.Group();
    j5.position.y = -forearmLength;
    j4.add(j5);
    armJoints[4] = { group: j5, axis: 'y' };

    const j5Collar = new THREE.Mesh(new THREE.CylinderGeometry(0.030, 0.030, 0.026, 24), motorMat);
    j5Collar.position.y = -0.013;
    j5.add(j5Collar);

    const j5SilverRing = new THREE.Mesh(new THREE.TorusGeometry(0.0305, 0.003, 16, 24), silverMat);
    j5SilverRing.rotation.x = Math.PI / 2;
    j5SilverRing.position.y = -0.022;
    j5.add(j5SilverRing);

    // ====================================================
    // Joint 6: Wrist Pitch (rotates around X axis)
    // Features signature Machined Silver Clevis / Dual-Prong Fork Bracket
    // ====================================================
    const j6 = new THREE.Group();
    j6.position.y = -0.036;
    j5.add(j6);
    armJoints[5] = { group: j6, axis: 'x' };

    // Machined Silver Clevis Fork Bracket (U-shaped dual-prong, as in photo)
    const clevisGroup = new THREE.Group();
    j6.add(clevisGroup);

    // Horizontal bridge
    const clevisBridge = new THREE.Mesh(new THREE.BoxGeometry(0.046, 0.010, 0.036), silverMat);
    clevisBridge.position.y = -0.005;
    clevisGroup.add(clevisBridge);

    // Left & Right vertical prongs
    [-0.022, 0.022].forEach(px => {
        const prong = new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.036, 0.034), silverMat);
        prong.position.set(px, -0.022, 0);
        clevisGroup.add(prong);

        // Circular pivot bolt cap
        const boltCap = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.007, 0.008, 16), silverMat);
        boltCap.rotation.z = Math.PI / 2;
        boltCap.position.set(px + (px > 0 ? 0.003 : -0.003), -0.022, 0);
        clevisGroup.add(boltCap);
    });

    // Flexible cable conduit through clevis
    const clevisCable = new THREE.Mesh(new THREE.TorusGeometry(0.022, 0.004, 12, 16, Math.PI * 0.7), cableMat);
    clevisCable.position.set(0, -0.025, 0.022);
    clevisGroup.add(clevisCable);

    // ====================================================
    // Joint 7: Wrist Yaw / End-Effector Roll (rotates around Y axis)
    // Features vertical black DM4310 motor cylinder with silver flanges
    // ====================================================
    const j7 = new THREE.Group();
    j7.position.y = -0.040;
    j6.add(j7);
    armJoints[6] = { group: j7, axis: 'y' };

    // Top silver mounting flange
    const j7TopFlange = new THREE.Mesh(new THREE.CylinderGeometry(0.028, 0.028, 0.005, 24), silverMat);
    j7TopFlange.position.y = -0.003;
    j7.add(j7TopFlange);

    // DM4310 Vertical Black Motor Can
    const dm4310Can = new THREE.Mesh(new THREE.CylinderGeometry(0.026, 0.026, 0.044, 24), motorMat);
    dm4310Can.position.y = -0.025;
    dm4310Can.castShadow = true;
    j7.add(dm4310Can);

    // White/silver spec label on DM4310
    const dm4310Label = new THREE.Mesh(new THREE.CylinderGeometry(0.0262, 0.0262, 0.020, 24, 1, false, 0, Math.PI * 0.85), labelMat);
    dm4310Label.position.y = -0.025;
    dm4310Label.rotation.y = -Math.PI / 2;
    j7.add(dm4310Label);

    // Bottom silver output collar
    const j7BottomFlange = new THREE.Mesh(new THREE.CylinderGeometry(0.027, 0.027, 0.006, 24), silverMat);
    j7BottomFlange.position.y = -0.050;
    j7.add(j7BottomFlange);

    // ====================================================
    // Joint 8: End-Effector Gripper (OpenArm Parallel Slider with Angled Beak Claws)
    // Matching official OpenArm photo: horizontal silver guide rail & angled black beak claws
    // ====================================================
    const gripperAssembly = new THREE.Group();
    gripperAssembly.position.y = -0.054;
    j7.add(gripperAssembly);

    // Horizontal Silver Guide Rail / Slider Crossbar (~96mm wide)
    const guideRail = new THREE.Mesh(new THREE.BoxGeometry(0.096, 0.008, 0.016), silverMat);
    guideRail.position.y = -0.004;
    guideRail.castShadow = true;
    gripperAssembly.add(guideRail);

    // Rail End Stops (Silver tabs on ends of crossbar)
    [-0.048, 0.048].forEach(rx => {
        const stop = new THREE.Mesh(new THREE.BoxGeometry(0.004, 0.014, 0.018), silverMat);
        stop.position.set(rx, -0.004, 0);
        gripperAssembly.add(stop);
    });

    // Helper to create the angled matte black wedge beak claw finger
    function createBeakClawFinger(isLeftFinger) {
        const clawGroup = new THREE.Group();

        // 1. Machined Silver Slider Carriage Block
        const carriage = new THREE.Mesh(new THREE.BoxGeometry(0.016, 0.012, 0.022), silverMat);
        carriage.position.y = -0.006;
        carriage.castShadow = true;
        clawGroup.add(carriage);

        // 2. Angled Matte Black Beak Claw (Wedge-shaped, tapering down to a sharp tip)
        const clawLength = 0.055;
        const signClaw = isLeftFinger ? -1 : 1;

        // Custom sculpted wedge claw geometry
        const clawGeo = new THREE.BufferGeometry();
        const wTop = 0.014;
        const depth = 0.018;

        const verts = new Float32Array([
            // Front face
            -wTop/2, 0, depth/2,
             wTop/2, 0, depth/2,
            -signClaw * 0.008, -clawLength, depth * 0.3,

             wTop/2, 0, depth/2,
            -signClaw * 0.008, -clawLength, depth * 0.3,
             signClaw * 0.002, -clawLength, depth * 0.3,

            // Back face
            -wTop/2, 0, -depth/2,
             wTop/2, 0, -depth/2,
            -signClaw * 0.008, -clawLength, -depth * 0.3,

             wTop/2, 0, -depth/2,
            -signClaw * 0.008, -clawLength, -depth * 0.3,
             signClaw * 0.002, -clawLength, -depth * 0.3,

            // Outer bevel face
            -signClaw * wTop/2, 0, -depth/2,
            -signClaw * wTop/2, 0,  depth/2,
            -signClaw * 0.008, -clawLength,  depth * 0.3,

            -signClaw * wTop/2, 0, -depth/2,
            -signClaw * 0.008, -clawLength,  depth * 0.3,
            -signClaw * 0.008, -clawLength, -depth * 0.3,

            // Inner gripping contact face
             signClaw * wTop/2, 0, -depth/2,
             signClaw * wTop/2, 0,  depth/2,
             signClaw * 0.002, -clawLength,  depth * 0.3,

             signClaw * wTop/2, 0, -depth/2,
             signClaw * 0.002, -clawLength,  depth * 0.3,
             signClaw * 0.002, -clawLength, -depth * 0.3,
        ]);

        clawGeo.setAttribute('position', new THREE.BufferAttribute(verts, 3));
        clawGeo.computeVertexNormals();

        const clawMesh = new THREE.Mesh(clawGeo, blackMat);
        clawMesh.position.y = -0.012;
        clawMesh.castShadow = true;
        clawGroup.add(clawMesh);

        // Inner high-friction gripping pad (dark textured strip)
        const pad = new THREE.Mesh(new THREE.BoxGeometry(0.002, clawLength * 0.75, depth * 0.6), motorMat);
        pad.position.set(signClaw * 0.003, -0.034, 0);
        clawGroup.add(pad);

        return clawGroup;
    }

    // Left Finger Assembly
    const fingerLeft = createBeakClawFinger(true);
    fingerLeft.position.set(-0.018, 0, 0);
    gripperAssembly.add(fingerLeft);

    // Right Finger Assembly
    const fingerRight = createBeakClawFinger(false);
    fingerRight.position.set(0.018, 0, 0);
    gripperAssembly.add(fingerRight);

    gripperFingers.left = fingerLeft;
    gripperFingers.right = fingerRight;
}

// ----------------------------------------------------
// AUTOMATED BIMANUAL TRAJECTORIES
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
    stopPresets();
    activePreset = preset;

    if (preset === "stop") {
        return;
    }

    sendAction("enable_all");

    if (preset === "home") {
        // Natural resting pose: arms hang down vertically beside pillar (exact photo pose)
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
        // J1(Pitch)=0.35, J2(Roll)=0.20, J3=0, J4(Elbow)=1.20, J5=0, J6(Wrist)=-0.35, J7=0
        [1, 9].forEach(baseId => {
            sendAction("set_mit", { id: baseId + 0, q: 0.35 });
            sendAction("set_mit", { id: baseId + 1, q: 0.20 });
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
            sendAction("set_mit", { id: 2, q: 0.50 });
            sendAction("set_mit", { id: 3, q: 0.00 });
            sendAction("set_mit", { id: 4, q: 1.50 });
            sendAction("set_mit", { id: 5, q: 0.00 });
            sendAction("set_mit", { id: 6, q: waveAngle });
            sendAction("set_mit", { id: 7, q: 0.00 });
        }, 50);
    } else if (preset === "clap") {
        // Handshake / Grippers reach toward center
        [1, 9].forEach(baseId => {
            sendAction("set_mit", { id: baseId + 0, q: 0.50 });
            sendAction("set_mit", { id: baseId + 1, q: 0.15 });
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
                    sendAction("set_mit", { id: baseId + 0, q: 0.45 });
                    sendAction("set_mit", { id: baseId + 1, q: 0.25 });
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
            const q6 = -Math.sin(t * 0.8) * 0.20 - 0.25;

            [1, 9].forEach(baseId => {
                sendAction("set_mit", { id: baseId + 0, q: q1 });
                sendAction("set_mit", { id: baseId + 1, q: q2 });
                sendAction("set_mit", { id: baseId + 3, q: q4 });
            });
        }, 50);
    }
}

// Render Animation Loop
function animate() {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    renderer.render(scene, camera);
}
