// OpenArm Bimanual 7DOF CAN-FD Simulator & Digital Twin Controller
// Supports Dual 7-DOF Robotic Arms (Left & Right) + Dual Grippers (16 CAN Nodes)

const LEFT_JOINTS = [
    { id: 1, name: "L-Joint 1 (Shoulder Yaw)",   arm: "left",  idx: 0, type: "DM8009", min: -3.14, max: 3.14, default: 0.0 },
    { id: 2, name: "L-Joint 2 (Shoulder Pitch)", arm: "left",  idx: 1, type: "DM8009", min: -2.00, max: 2.00, default: 0.0 },
    { id: 3, name: "L-Joint 3 (Elbow Roll)",     arm: "left",  idx: 2, type: "DM4340", min: -3.14, max: 3.14, default: 0.0 },
    { id: 4, name: "L-Joint 4 (Elbow Pitch)",    arm: "left",  idx: 3, type: "DM4340", min: -2.20, max: 2.20, default: 0.0 },
    { id: 5, name: "L-Joint 5 (Wrist Roll)",     arm: "left",  idx: 4, type: "DM4310", min: -3.14, max: 3.14, default: 0.0 },
    { id: 6, name: "L-Joint 6 (Wrist Pitch)",    arm: "left",  idx: 5, type: "DM4310", min: -1.80, max: 1.80, default: 0.0 },
    { id: 7, name: "L-Joint 7 (Wrist Yaw)",      arm: "left",  idx: 6, type: "DM4310", min: -3.14, max: 3.14, default: 0.0 },
    { id: 8, name: "L-Gripper (Linear Finger)",  arm: "left",  idx: 7, type: "DM4310", min: 0.0,   max: 1.0,  default: 0.0 }
];

const RIGHT_JOINTS = [
    { id: 9,  name: "R-Joint 1 (Shoulder Yaw)",   arm: "right", idx: 0, type: "DM8009", min: -3.14, max: 3.14, default: 0.0 },
    { id: 10, name: "R-Joint 2 (Shoulder Pitch)", arm: "right", idx: 1, type: "DM8009", min: -2.00, max: 2.00, default: 0.0 },
    { id: 11, name: "R-Joint 3 (Elbow Roll)",     arm: "right", idx: 2, type: "DM4340", min: -3.14, max: 3.14, default: 0.0 },
    { id: 12, name: "R-Joint 4 (Elbow Pitch)",    arm: "right", idx: 3, type: "DM4340", min: -2.20, max: 2.20, default: 0.0 },
    { id: 13, name: "R-Joint 5 (Wrist Roll)",     arm: "right", idx: 4, type: "DM4310", min: -3.14, max: 3.14, default: 0.0 },
    { id: 14, name: "R-Joint 6 (Wrist Pitch)",    arm: "right", idx: 5, type: "DM4310", min: -1.80, max: 1.80, default: 0.0 },
    { id: 15, name: "R-Joint 7 (Wrist Yaw)",      arm: "right", idx: 6, type: "DM4310", min: -3.14, max: 3.14, default: 0.0 },
    { id: 16, name: "R-Gripper (Linear Finger)",  arm: "right", idx: 7, type: "DM4310", min: 0.0,   max: 1.0,  default: 0.0 }
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

// Initialize DOM and App
document.addEventListener("DOMContentLoaded", () => {
    document.body.classList.add("theme-light");

    buildJointSliders();
    buildTelemetryCards();
    initThreeJS();
    initWebSocket();
    setupEventHandlers();
});

// Build Sliders for Arm Joints
function buildJointSliders() {
    const list = document.getElementById("sliders-list");
    list.innerHTML = "";

    const activeList = currentArmTab === 'right' ? RIGHT_JOINTS.slice(0, 7) : LEFT_JOINTS.slice(0, 7);

    activeList.forEach((j, index) => {
        const card = document.createElement("div");
        card.className = "joint-slider-card";

        const labelPrefix = currentArmTab === 'sync' ? "Dual J" + (index + 1) : (currentArmTab === 'right' ? "R-J" + (index + 1) : "L-J" + (index + 1));
        const subName = j.name.split("(")[1] ? "(" + j.name.split("(")[1] : "";

        card.innerHTML = `
            <div class="slider-header">
                <span class="joint-title">
                    <span class="arm-tag ${currentArmTab === 'right' ? 'arm-tag-right' : 'arm-tag-left'}">${currentArmTab === 'sync' ? 'L+R' : (currentArmTab === 'right' ? 'R' : 'L')}</span>
                    ${labelPrefix} ${subName} • ${j.type}
                </span>
                <span class="joint-val" id="val-disp-j${index}">0.00 rad (0°)</span>
            </div>
            <input type="range" id="slider-j${index}" min="${j.min}" max="${j.max}" step="0.01" value="0.0">
            <div class="gripper-tags">
                <span>${j.min.toFixed(1)} rad</span>
                <span>${j.max.toFixed(1)} rad</span>
            </div>
        `;
        list.appendChild(card);

        const slider = card.querySelector(`#slider-j${index}`);
        slider.addEventListener("input", (e) => {
            const val = parseFloat(e.target.value);
            const deg = (val * 180 / Math.PI).toFixed(0);
            document.getElementById(`val-disp-j${index}`).textContent = `${val.toFixed(2)} rad (${deg}°)`;

            if (currentArmTab === 'left') {
                sendAction("set_mit", { id: j.id, q: val, kp: 30.0, kd: 1.2, tau: 0.0 });
            } else if (currentArmTab === 'right') {
                sendAction("set_mit", { id: j.id, q: val, kp: 30.0, kd: 1.2, tau: 0.0 });
            } else { // sync
                // Left arm joint
                const leftMotorId = LEFT_JOINTS[index].id;
                // Right arm joint (mirrored yaw/roll joints for symmetrical motion)
                const rightMotorId = RIGHT_JOINTS[index].id;
                const mirrorSign = (index === 0 || index === 2 || index === 4 || index === 6) ? -1.0 : 1.0;

                sendAction("set_mit", { id: leftMotorId, q: val, kp: 30.0, kd: 1.2, tau: 0.0 });
                sendAction("set_mit", { id: rightMotorId, q: val * mirrorSign, kp: 30.0, kd: 1.2, tau: 0.0 });
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
            btnTheme.textContent = "☀️ Studio Bright";
            if (floorMesh) floorMesh.material.color.setHex(0xeef2f6);
            if (scene) scene.background = new THREE.Color(0xf1f5f9);
        } else {
            document.body.classList.remove("theme-light");
            btnTheme.textContent = "🌙 Cyber Dark";
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
    document.getElementById("tab-arm-sync").addEventListener("click", (e) => {
        setArmTab("sync", e.target);
    });

    // Master actions
    document.getElementById("btn-enable-all").addEventListener("click", () => {
        sendAction("enable_all");
    });
    document.getElementById("btn-disable-all").addEventListener("click", () => {
        stopPresets();
        sendAction("disable_all");
    });
    document.getElementById("btn-zero-all").addEventListener("click", () => {
        sendAction("set_zero_all");
    });
    document.getElementById("btn-clear-err").addEventListener("click", () => {
        sendAction("clear_error_all");
    });

    // Grippers
    const syncCheck = document.getElementById("sync-grippers-check");
    syncCheck.addEventListener("change", (e) => {
        syncGrippers = e.target.checked;
    });

    const leftGripperSlider = document.getElementById("slider-gripper-left");
    leftGripperSlider.addEventListener("input", (e) => {
        const val = parseFloat(e.target.value);
        document.getElementById("left-gripper-val-display").textContent = `${val.toFixed(2)} rad (${val > 0.4 ? 'Grasping' : 'Open'})`;
        sendAction("set_gripper", { id: 8, arm: "left", pos: val });

        if (syncGrippers) {
            const rightSlider = document.getElementById("slider-gripper-right");
            rightSlider.value = val;
            document.getElementById("right-gripper-val-display").textContent = `${val.toFixed(2)} rad (${val > 0.4 ? 'Grasping' : 'Open'})`;
            sendAction("set_gripper", { id: 16, arm: "right", pos: val });
        }
    });

    const rightGripperSlider = document.getElementById("slider-gripper-right");
    rightGripperSlider.addEventListener("input", (e) => {
        const val = parseFloat(e.target.value);
        document.getElementById("right-gripper-val-display").textContent = `${val.toFixed(2)} rad (${val > 0.4 ? 'Grasping' : 'Open'})`;
        sendAction("set_gripper", { id: 16, arm: "right", pos: val });

        if (syncGrippers) {
            const leftSlider = document.getElementById("slider-gripper-left");
            leftSlider.value = val;
            document.getElementById("left-gripper-val-display").textContent = `${val.toFixed(2)} rad (${val > 0.4 ? 'Grasping' : 'Open'})`;
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
    document.getElementById("btn-front-view").addEventListener("click", () => {
        camera.position.set(0, 0.70, 1.45);
        if (controls) controls.target.set(0, 0.55, 0);
    });
    document.getElementById("btn-toggle-grid").addEventListener("click", (e) => {
        gridHelper.visible = !gridHelper.visible;
        e.target.textContent = `Grid: ${gridHelper.visible ? 'ON' : 'OFF'}`;
    });
    document.getElementById("btn-toggle-axes").addEventListener("click", (e) => {
        axesHelper.visible = !axesHelper.visible;
        e.target.textContent = `Axes: ${axesHelper.visible ? 'ON' : 'OFF'}`;
    });

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
    document.querySelectorAll(".arm-tab-btn").forEach(btn => btn.classList.remove("active"));
    element.classList.add("active");
    buildJointSliders();
}

// WebSocket Connection
function initWebSocket() {
    const wsUrl = `ws://${window.location.hostname}:8889`;
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
        if (qEl) qEl.textContent = `${m.q.toFixed(3)} (${m.q_deg}°)`;
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
                if (jEntry.axis === 'y') {
                    jEntry.group.rotation.y = angle;
                } else if (jEntry.axis === 'z') {
                    jEntry.group.rotation.z = angle;
                } else if (jEntry.axis === 'x') {
                    jEntry.group.rotation.x = angle;
                }
            }
        } else if (jointIndex === 7) {
            // Gripper opening/closing (0.0: fully open, 1.0: grasp)
            const stroke = Math.max(0.0, Math.min(1.0, m.q));
            const fingerOffset = 0.038 - stroke * 0.024;
            if (gripperFingers.left && gripperFingers.right) {
                gripperFingers.left.position.x = -fingerOffset;
                gripperFingers.right.position.x = fingerOffset;
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
    controls.target.set(0, 0.55, 0);

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
    camera.position.set(0.85, 0.85, 1.45);
    if (controls) controls.target.set(0, 0.55, 0);
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
    // Premium Industrial Materials (Stealth Matte Black + Metallic Silver Rings)
    const matteBlackMat = new THREE.MeshStandardMaterial({
        color: 0x18191d,
        roughness: 0.35,
        metalness: 0.25
    });
    const silverMetalMat = new THREE.MeshStandardMaterial({
        color: 0xd8d8de,
        roughness: 0.18,
        metalness: 0.92
    });
    const darkExtrusionMat = new THREE.MeshStandardMaterial({
        color: 0x22242a,
        roughness: 0.45,
        metalness: 0.7
    });
    const basePlateMat = new THREE.MeshStandardMaterial({
        color: 0xb5b9c0,
        roughness: 0.25,
        metalness: 0.85
    });
    const cyanGlowMat = new THREE.MeshStandardMaterial({
        color: 0x0284c7,
        emissive: 0x00f0ff,
        emissiveIntensity: 0.45,
        roughness: 0.2
    });

    const rootGroup = new THREE.Group();
    scene.add(rootGroup);

    // 1. Heavy Machined Base Plate
    const basePlate = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.018, 0.38), basePlateMat);
    basePlate.position.y = 0.009;
    basePlate.castShadow = true;
    basePlate.receiveShadow = true;
    rootGroup.add(basePlate);

    // Base Mounting Bolts
    [-0.14, 0.14].forEach(x => {
        [-0.16, 0.16].forEach(z => {
            const bolt = new THREE.Mesh(new THREE.CylinderGeometry(0.01, 0.01, 0.015, 12), silverMetalMat);
            bolt.position.set(x, 0.02, z);
            rootGroup.add(bolt);
        });
    });

    // 2. Central Vertical Pillar (Black Anodized Extrusion Profile)
    const pillarHeight = 0.70;
    const pillar = new THREE.Mesh(new THREE.BoxGeometry(0.08, pillarHeight, 0.08), darkExtrusionMat);
    pillar.position.y = 0.018 + pillarHeight / 2;
    pillar.castShadow = true;
    rootGroup.add(pillar);

    // Column T-slot Grooves
    const slotMat = new THREE.MeshBasicMaterial({ color: 0x0f1115 });
    [-0.041, 0.041].forEach(x => {
        const slot = new THREE.Mesh(new THREE.BoxGeometry(0.004, pillarHeight, 0.016), slotMat);
        slot.position.set(x, 0.018 + pillarHeight / 2, 0);
        rootGroup.add(slot);
    });

    // Base Support Triangular Gussets / Brackets
    [-0.065, 0.065].forEach(x => {
        const gussetGeo = new THREE.BufferGeometry();
        // Triangle shape
        const vertices = new Float32Array([
            x, 0.018, -0.04,
            x, 0.018,  0.04,
            x * 0.7, 0.14, -0.04,
            x * 0.7, 0.14, -0.04,
            x, 0.018,  0.04,
            x * 0.7, 0.14,  0.04,
        ]);
        gussetGeo.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
        gussetGeo.computeVertexNormals();
        const gussetMesh = new THREE.Mesh(gussetGeo, silverMetalMat);
        rootGroup.add(gussetMesh);
    });

    // 3. Central Torso / Head Unit (at top of pillar)
    const torsoY = 0.018 + pillarHeight;
    const torso = new THREE.Group();
    torso.position.y = torsoY;
    rootGroup.add(torso);

    const torsoBody = new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.13, 0.16), matteBlackMat);
    torsoBody.position.y = 0.04;
    torsoBody.castShadow = true;
    torso.add(torsoBody);

    // Torso Chamfered Top Shell
    const torsoTop = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.12, 0.04, 32), matteBlackMat);
    torsoTop.position.y = 0.12;
    torso.add(torsoTop);

    // OpenArm Distinctive Chest Emblem Logo (white 3-node connected glyph)
    const logoGroup = new THREE.Group();
    logoGroup.position.set(0, 0.05, 0.082);

    const nodeMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const n1 = new THREE.Mesh(new THREE.CircleGeometry(0.008, 16), nodeMat);
    n1.position.set(-0.015, 0, 0);
    const n2 = new THREE.Mesh(new THREE.CircleGeometry(0.008, 16), nodeMat);
    n2.position.set(0.015, 0, 0);
    const n3 = new THREE.Mesh(new THREE.CircleGeometry(0.007, 16), nodeMat);
    n3.position.set(0, 0.014, 0);
    logoGroup.add(n1, n2, n3);
    torso.add(logoGroup);

    // 4. Build Left Arm and Right Arm
    buildSingleArm(torso, "left",  -0.145, 0.04, 0, matteBlackMat, silverMetalMat, cyanGlowMat);
    buildSingleArm(torso, "right",  0.145, 0.04, 0, matteBlackMat, silverMetalMat, cyanGlowMat);
}

// Single 7-DOF Arm + 2-Finger Gripper Kinematic Assembly
function buildSingleArm(parent, side, offsetX, offsetY, offsetZ, blackMat, silverMat, glowMat) {
    const isLeft = side === "left";
    const armJoints = isLeft ? leftArmJoints : rightArmJoints;
    const gripperFingers = isLeft ? leftGripperFingers : rightGripperFingers;

    // Shoulder Base Mount on Torso
    const shoulderMount = new THREE.Group();
    shoulderMount.position.set(offsetX, offsetY, offsetZ);
    parent.add(shoulderMount);

    // Lateral Shoulder Motor Pod
    const shoulderPod = new THREE.Mesh(new THREE.CylinderGeometry(0.052, 0.052, 0.06, 28), blackMat);
    shoulderPod.rotation.z = Math.PI / 2;
    shoulderPod.castShadow = true;
    shoulderMount.add(shoulderPod);

    const shoulderRing = new THREE.Mesh(new THREE.TorusGeometry(0.053, 0.004, 16, 32), silverMat);
    shoulderRing.rotation.y = Math.PI / 2;
    shoulderMount.add(shoulderRing);

    // ----------------------------------------------------
    // Joint 1: Shoulder Yaw (rotates around Y)
    // ----------------------------------------------------
    const j1 = new THREE.Group();
    shoulderMount.add(j1);
    armJoints[0] = { group: j1, axis: 'y' };

    // ----------------------------------------------------
    // Joint 2: Shoulder Pitch (rotates around Z or X)
    // In natural rest/home pose, arm hangs straight down along -Y
    // ----------------------------------------------------
    const j2 = new THREE.Group();
    j1.add(j2);
    armJoints[1] = { group: j2, axis: 'x' };

    // J2 Motor Housing with Silver Metallic Bezel Ring
    const j2Housing = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.08, 28), blackMat);
    j2Housing.rotation.z = Math.PI / 2;
    j2Housing.castShadow = true;
    j2.add(j2Housing);

    const j2Ring = new THREE.Mesh(new THREE.TorusGeometry(0.051, 0.005, 16, 32), silverMat);
    j2Ring.rotation.y = Math.PI / 2;
    j2.add(j2Ring);

    // Link 1: Upper Arm (slender sculpted black casing, ~220mm long downwards)
    const upperArmLength = 0.22;
    const upperArmMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.042, 0.038, upperArmLength, 24), blackMat);
    upperArmMesh.position.y = -upperArmLength / 2;
    upperArmMesh.castShadow = true;
    j2.add(upperArmMesh);

    // Upper arm decorative silver accent groove
    const upperArmGroove = new THREE.Mesh(new THREE.TorusGeometry(0.041, 0.003, 16, 28), silverMat);
    upperArmGroove.rotation.x = Math.PI / 2;
    upperArmGroove.position.y = -upperArmLength * 0.45;
    j2.add(upperArmGroove);

    // ----------------------------------------------------
    // Joint 3: Elbow Roll (rotates around Y)
    // ----------------------------------------------------
    const j3 = new THREE.Group();
    j3.position.y = -upperArmLength;
    j2.add(j3);
    armJoints[2] = { group: j3, axis: 'y' };

    const j3Motor = new THREE.Mesh(new THREE.CylinderGeometry(0.038, 0.038, 0.05, 24), silverMat);
    j3Motor.position.y = -0.025;
    j3Motor.castShadow = true;
    j3.add(j3Motor);

    // ----------------------------------------------------
    // Joint 4: Elbow Pitch (rotates around X)
    // ----------------------------------------------------
    const j4 = new THREE.Group();
    j4.position.y = -0.045;
    j3.add(j4);
    armJoints[3] = { group: j4, axis: 'x' };

    const j4Housing = new THREE.Mesh(new THREE.CylinderGeometry(0.042, 0.042, 0.075, 24), blackMat);
    j4Housing.rotation.z = Math.PI / 2;
    j4Housing.castShadow = true;
    j4.add(j4Housing);

    const j4Ring = new THREE.Mesh(new THREE.TorusGeometry(0.043, 0.004, 16, 28), silverMat);
    j4Ring.rotation.y = Math.PI / 2;
    j4.add(j4Ring);

    // Link 2: Forearm (sleek black body, ~200mm long downwards)
    const forearmLength = 0.20;
    const forearmMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.036, 0.032, forearmLength, 24), blackMat);
    forearmMesh.position.y = -forearmLength / 2;
    forearmMesh.castShadow = true;
    j4.add(forearmMesh);

    // ----------------------------------------------------
    // Joint 5: Wrist Roll (rotates around Y)
    // ----------------------------------------------------
    const j5 = new THREE.Group();
    j5.position.y = -forearmLength;
    j4.add(j5);
    armJoints[4] = { group: j5, axis: 'y' };

    const j5Motor = new THREE.Mesh(new THREE.CylinderGeometry(0.032, 0.032, 0.04, 24), silverMat);
    j5Motor.position.y = -0.02;
    j5Motor.castShadow = true;
    j5.add(j5Motor);

    // ----------------------------------------------------
    // Joint 6: Wrist Pitch (rotates around X)
    // ----------------------------------------------------
    const j6 = new THREE.Group();
    j6.position.y = -0.038;
    j5.add(j6);
    armJoints[5] = { group: j6, axis: 'x' };

    const j6Housing = new THREE.Mesh(new THREE.CylinderGeometry(0.032, 0.032, 0.06, 24), blackMat);
    j6Housing.rotation.z = Math.PI / 2;
    j6Housing.castShadow = true;
    j6.add(j6Housing);

    // ----------------------------------------------------
    // Joint 7: Wrist Flange / End-Effector Roll (rotates around Y)
    // ----------------------------------------------------
    const j7 = new THREE.Group();
    j7.position.y = -0.035;
    j6.add(j7);
    armJoints[6] = { group: j7, axis: 'y' };

    const j7Flange = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.032, 0.025, 24), silverMat);
    j7Flange.position.y = -0.012;
    j7Flange.castShadow = true;
    j7.add(j7Flange);

    // ----------------------------------------------------
    // End-Effector: 2-Finger Parallel/Angular Gripper
    // Matching official OpenArm with machined silver links & black pads
    // ----------------------------------------------------
    const gripperGroup = new THREE.Group();
    gripperGroup.position.y = -0.025;
    j7.add(gripperGroup);

    // Gripper Base / Actuator Housing
    const palm = new THREE.Mesh(new THREE.BoxGeometry(0.075, 0.025, 0.045), blackMat);
    palm.position.y = -0.012;
    palm.castShadow = true;
    gripperGroup.add(palm);

    // Left Finger Assembly
    const fingerLeftGroup = new THREE.Group();
    fingerLeftGroup.position.set(-0.028, -0.025, 0);
    gripperGroup.add(fingerLeftGroup);

    const fingerLeftLink = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.045, 0.02), silverMat);
    fingerLeftLink.position.y = -0.022;
    fingerLeftLink.castShadow = true;
    fingerLeftGroup.add(fingerLeftLink);

    const fingerLeftPad = new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.035, 0.018), blackMat);
    fingerLeftPad.position.set(0.006, -0.025, 0);
    fingerLeftGroup.add(fingerLeftPad);

    // Right Finger Assembly
    const fingerRightGroup = new THREE.Group();
    fingerRightGroup.position.set(0.028, -0.025, 0);
    gripperGroup.add(fingerRightGroup);

    const fingerRightLink = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.045, 0.02), silverMat);
    fingerRightLink.position.y = -0.022;
    fingerRightLink.castShadow = true;
    fingerRightGroup.add(fingerRightLink);

    const fingerRightPad = new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.035, 0.018), blackMat);
    fingerRightPad.position.set(-0.006, -0.025, 0);
    fingerRightGroup.add(fingerRightPad);

    gripperFingers.left = fingerLeftGroup;
    gripperFingers.right = fingerRightGroup;
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
        // L: J1=0.2, J2=0.7, J3=0.0, J4=-1.1, J5=0.0, J6=0.4, J7=0.0
        // R: J1=-0.2, J2=0.7, J3=0.0, J4=-1.1, J5=0.0, J6=0.4, J7=0.0
        sendAction("set_mit", { id: 1, q: 0.25 });
        sendAction("set_mit", { id: 2, q: 0.75 });
        sendAction("set_mit", { id: 3, q: 0.0 });
        sendAction("set_mit", { id: 4, q: -1.05 });
        sendAction("set_mit", { id: 5, q: 0.0 });
        sendAction("set_mit", { id: 6, q: 0.35 });
        sendAction("set_mit", { id: 7, q: 0.0 });
        sendAction("set_gripper", { id: 8, pos: 0.2 });

        sendAction("set_mit", { id: 9,  q: -0.25 });
        sendAction("set_mit", { id: 10, q: 0.75 });
        sendAction("set_mit", { id: 11, q: 0.0 });
        sendAction("set_mit", { id: 12, q: -1.05 });
        sendAction("set_mit", { id: 13, q: 0.0 });
        sendAction("set_mit", { id: 14, q: 0.35 });
        sendAction("set_mit", { id: 15, q: 0.0 });
        sendAction("set_gripper", { id: 16, pos: 0.2 });
    } else if (preset === "wave") {
        // Bimanual alternating waving motion
        let t = 0;
        presetTimer = setInterval(() => {
            t += 0.05;
            // Left arm wave
            const leftWave = Math.sin(t * 3.5) * 0.45;
            sendAction("set_mit", { id: 1, q: 0.4 });
            sendAction("set_mit", { id: 2, q: 1.1 });
            sendAction("set_mit", { id: 4, q: -1.2 });
            sendAction("set_mit", { id: 6, q: leftWave });

            // Right arm wave with phase offset
            const rightWave = Math.sin(t * 3.5 + Math.PI) * 0.45;
            sendAction("set_mit", { id: 9,  q: -0.4 });
            sendAction("set_mit", { id: 10, q: 1.1 });
            sendAction("set_mit", { id: 12, q: -1.2 });
            sendAction("set_mit", { id: 14, q: rightWave });
        }, 50);
    } else if (preset === "clap") {
        // Handshake / Grippers meet at center
        sendAction("set_mit", { id: 1, q: 0.65 });
        sendAction("set_mit", { id: 2, q: 0.70 });
        sendAction("set_mit", { id: 3, q: 0.0 });
        sendAction("set_mit", { id: 4, q: -1.15 });
        sendAction("set_mit", { id: 5, q: 0.8 });
        sendAction("set_mit", { id: 6, q: 0.2 });
        sendAction("set_gripper", { id: 8, pos: 0.6 });

        sendAction("set_mit", { id: 9,  q: -0.65 });
        sendAction("set_mit", { id: 10, q: 0.70 });
        sendAction("set_mit", { id: 11, q: 0.0 });
        sendAction("set_mit", { id: 12, q: -1.15 });
        sendAction("set_mit", { id: 13, q: -0.8 });
        sendAction("set_mit", { id: 14, q: 0.2 });
        sendAction("set_gripper", { id: 16, pos: 0.6 });
    } else if (preset === "carry") {
        // Dual-arm reach forward, grasp box, lift upwards
        let step = 0;
        presetTimer = setInterval(() => {
            step++;
            const phase = (step % 120);
            if (phase < 40) {
                // Reach forward & open grippers
                sendAction("set_mit", { id: 2, q: 0.6 });
                sendAction("set_mit", { id: 4, q: -0.7 });
                sendAction("set_mit", { id: 10, q: 0.6 });
                sendAction("set_mit", { id: 12, q: -0.7 });
                sendAction("set_gripper", { arm: "both", pos: 0.0 });
            } else if (phase < 80) {
                // Grasp
                sendAction("set_gripper", { arm: "both", pos: 0.85 });
            } else {
                // Lift
                sendAction("set_mit", { id: 2, q: 0.95 });
                sendAction("set_mit", { id: 4, q: -1.2 });
                sendAction("set_mit", { id: 10, q: 0.95 });
                sendAction("set_mit", { id: 12, q: -1.2 });
            }
        }, 60);
    } else if (preset === "sine") {
        let t = 0;
        presetTimer = setInterval(() => {
            t += 0.04;
            const q2 = Math.sin(t) * 0.4 + 0.3;
            const q4 = -Math.cos(t * 1.2) * 0.5 - 0.6;
            const q6 = Math.sin(t * 1.5) * 0.35;

            // Coordinated mirrored sine wave for both arms
            sendAction("set_mit", { id: 2,  q: q2 });
            sendAction("set_mit", { id: 4,  q: q4 });
            sendAction("set_mit", { id: 6,  q: q6 });

            sendAction("set_mit", { id: 10, q: q2 });
            sendAction("set_mit", { id: 12, q: q4 });
            sendAction("set_mit", { id: 14, q: q6 });
        }, 40);
    }
}

// Render Animation Loop
function animate() {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    renderer.render(scene, camera);
}
