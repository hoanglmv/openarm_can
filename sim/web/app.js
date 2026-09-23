// OpenArm CAN-FD Simulator & Digital Twin Controller

// Joint configuration: name, limits, motor type
const JOINTS = [
    { id: 1, name: "Joint 1 (Base Yaw)",       type: "DM8009", min: -3.14, max: 3.14, default: 0.0 },
    { id: 2, name: "Joint 2 (Shoulder Pitch)", type: "DM8009", min: -2.00, max: 2.00, default: 0.0 },
    { id: 3, name: "Joint 3 (Elbow Roll)",     type: "DM4340", min: -3.14, max: 3.14, default: 0.0 },
    { id: 4, name: "Joint 4 (Elbow Pitch)",    type: "DM4340", min: -2.20, max: 2.20, default: 0.0 },
    { id: 5, name: "Joint 5 (Wrist Roll)",     type: "DM4310", min: -3.14, max: 3.14, default: 0.0 },
    { id: 6, name: "Joint 6 (Wrist Pitch)",    type: "DM4310", min: -1.80, max: 1.80, default: 0.0 },
    { id: 7, name: "Joint 7 (Wrist Yaw)",      type: "DM4310", min: -3.14, max: 3.14, default: 0.0 },
    { id: 8, name: "Gripper (Linear Finger)",  type: "DM4310", min: 0.0,   max: 1.0,  default: 0.0 }
];

let ws = null;
let scene, camera, renderer, controls;
let robotJoints = [];
let leftFinger, rightFinger;
let gridHelper, axesHelper, floorMesh;
let activePreset = null;
let currentTelemetry = [];
let isLightTheme = true; // Default to clean, bright studio theme!

// Initialize DOM and App
document.addEventListener("DOMContentLoaded", () => {
    // Apply default light studio theme
    document.body.classList.add("theme-light");

    buildJointSliders();
    buildTelemetryCards();
    initThreeJS();
    initWebSocket();
    setupEventHandlers();
});

// Build Sliders for Arm Joints 1 to 7
function buildJointSliders() {
    const list = document.getElementById("sliders-list");
    list.innerHTML = "";

    JOINTS.slice(0, 7).forEach(j => {
        const card = document.createElement("div");
        card.className = "joint-slider-card";
        card.innerHTML = `
            <div class="slider-header">
                <span class="joint-title">${j.name}</span>
                <span class="joint-val" id="val-disp-${j.id}">0.00 rad (0°)</span>
            </div>
            <input type="range" id="slider-${j.id}" min="${j.min}" max="${j.max}" step="0.01" value="${j.default}">
            <div class="gripper-tags">
                <span>${j.min.toFixed(1)} rad</span>
                <span>${j.max.toFixed(1)} rad</span>
            </div>
        `;
        list.appendChild(card);

        const slider = card.querySelector(`#slider-${j.id}`);
        slider.addEventListener("input", (e) => {
            const val = parseFloat(e.target.value);
            const deg = (val * 180 / Math.PI).toFixed(0);
            document.getElementById(`val-disp-${j.id}`).textContent = `${val.toFixed(2)} rad (${deg}°)`;
            sendAction("set_mit", { id: j.id, q: val, kp: 30.0, kd: 1.2, tau: 0.0 });
        });
    });

    // Gripper slider
    const gripperSlider = document.getElementById("slider-gripper");
    gripperSlider.addEventListener("input", (e) => {
        const val = parseFloat(e.target.value);
        document.getElementById("gripper-val-display").textContent = `${val.toFixed(2)} rad (${val > 0.5 ? 'Grasping' : 'Open'})`;
        sendAction("set_gripper", { pos: val });
    });
}

// Build Telemetry Cards for 8 Motors
function buildTelemetryCards() {
    const grid = document.getElementById("telemetry-grid");
    grid.innerHTML = "";

    JOINTS.forEach(j => {
        const card = document.createElement("div");
        card.className = "tel-card";
        card.id = `tel-card-${j.id}`;
        card.innerHTML = `
            <div class="tel-card-head">
                <span class="tel-card-title">M${j.id} • ${j.type}</span>
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
}

// Setup Event Handlers
function setupEventHandlers() {
    // Theme Toggle
    const themeBtn = document.getElementById("btn-theme-toggle");
    themeBtn.addEventListener("click", () => {
        isLightTheme = !isLightTheme;
        if (isLightTheme) {
            document.body.classList.add("theme-light");
            themeBtn.textContent = "🌙 Dark Mode";
            updateThreeTheme(true);
        } else {
            document.body.classList.remove("theme-light");
            themeBtn.textContent = "☀️ Studio Bright";
            updateThreeTheme(false);
        }
    });

    document.getElementById("btn-enable-all").addEventListener("click", () => sendAction("enable_all"));
    document.getElementById("btn-disable-all").addEventListener("click", () => {
        stopPreset();
        sendAction("disable_all");
    });
    document.getElementById("btn-zero-all").addEventListener("click", () => {
        sendAction("set_zero_all");
        JOINTS.slice(0, 7).forEach(j => {
            const s = document.getElementById(`slider-${j.id}`);
            if (s) { s.value = 0; document.getElementById(`val-disp-${j.id}`).textContent = "0.00 rad (0°)"; }
        });
    });
    document.getElementById("btn-clear-err").addEventListener("click", () => sendAction("clear_error_all"));

    // Trajectory Presets
    document.querySelectorAll(".btn-preset").forEach(btn => {
        btn.addEventListener("click", (e) => {
            const preset = btn.dataset.preset;
            if (preset === "stop") {
                stopPreset();
            } else {
                startPreset(preset);
            }
        });
    });

    // Viewport tools
    document.getElementById("btn-reset-cam").addEventListener("click", resetCamera);
    document.getElementById("btn-toggle-grid").addEventListener("click", (e) => {
        gridHelper.visible = !gridHelper.visible;
        e.target.textContent = `Grid: ${gridHelper.visible ? 'ON' : 'OFF'}`;
    });
    document.getElementById("btn-toggle-axes").addEventListener("click", (e) => {
        axesHelper.visible = !axesHelper.visible;
        e.target.textContent = `Axes: ${axesHelper.visible ? 'ON' : 'OFF'}`;
    });

    // CLI actions
    document.querySelectorAll(".btn-cli").forEach(btn => {
        btn.addEventListener("click", () => {
            const cmd = btn.dataset.cmd;
            if (cmd === "clear_term") {
                document.getElementById("term-output").textContent = "Terminal cleared.";
            } else {
                runCliCommand(cmd);
            }
        });
    });
}

function updateThreeTheme(isLight) {
    if (!scene) return;
    if (isLight) {
        scene.background = new THREE.Color(0xdde5ed); // Soft clean studio background
        scene.fog.color = new THREE.Color(0xdde5ed);
        floorMesh.material.color.set(0xeef2f6);
        gridHelper.material.color.set(0x94a3b8);
    } else {
        scene.background = new THREE.Color(0x1e293b); // Slate 800
        scene.fog.color = new THREE.Color(0x1e293b);
        floorMesh.material.color.set(0x141e2e);
        gridHelper.material.color.set(0x00f0ff);
    }
}

// Three.js 3D OpenArm Scene Construction
function initThreeJS() {
    const container = document.getElementById("three-container");
    const width = container.clientWidth;
    const height = container.clientHeight;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xdde5ed); // Default Bright Studio
    scene.fog = new THREE.FogExp2(0xdde5ed, 0.15);

    camera = new THREE.PerspectiveCamera(38, width / height, 0.05, 50);
    resetCamera();

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.1;
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.06;
    controls.target.set(0, 0.35, 0);

    // Bright Studio Lighting Setup
    const ambientLight = new THREE.AmbientLight(0xffffff, 1.4);
    scene.add(ambientLight);

    const mainKeyLight = new THREE.DirectionalLight(0xffffff, 1.8);
    mainKeyLight.position.set(1.5, 3.5, 2.5);
    mainKeyLight.castShadow = true;
    mainKeyLight.shadow.mapSize.width = 2048;
    mainKeyLight.shadow.mapSize.height = 2048;
    mainKeyLight.shadow.bias = -0.0001;
    scene.add(mainKeyLight);

    const fillLight = new THREE.DirectionalLight(0xb0c4de, 0.9);
    fillLight.position.set(-2, 2, -1.5);
    scene.add(fillLight);

    const rimLight = new THREE.DirectionalLight(0xffffff, 0.7);
    rimLight.position.set(0, 2, -2.5);
    scene.add(rimLight);

    // Studio Floor & Grid
    const floorGeo = new THREE.PlaneGeometry(16, 16);
    const floorMat = new THREE.MeshStandardMaterial({
        color: 0xeef2f6,
        roughness: 0.6,
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

    // Build OpenArm Kinematic Hierarchy
    buildOpenArmModel();

    window.addEventListener("resize", onWindowResize);
    animate();
}

function resetCamera() {
    // Closer, dynamic 3D angle so robot is clearly visible
    camera.position.set(0.70, 0.55, 0.85);
    if (controls) controls.target.set(0, 0.35, 0);
}

function onWindowResize() {
    const container = document.getElementById("three-container");
    if (!container) return;
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
}

// OpenArm 7 DOF Kinematic Mesh Constructor (High Visibility & Realistic Styling)
function buildOpenArmModel() {
    // High-Contrast Robotic Materials (Clean Porcelain White Shell + Metallic Joint Motors)
    const whiteShellMat = new THREE.MeshStandardMaterial({
        color: 0xffffff,
        roughness: 0.25,
        metalness: 0.15
    });
    const darkMotorMat = new THREE.MeshStandardMaterial({
        color: 0x334155,
        roughness: 0.3,
        metalness: 0.8
    });
    const cyanRingMat = new THREE.MeshStandardMaterial({
        color: 0x0284c7,
        emissive: 0x00f0ff,
        emissiveIntensity: 0.4,
        roughness: 0.2
    });
    const orangeGripperMat = new THREE.MeshStandardMaterial({
        color: 0xf97316,
        roughness: 0.35,
        metalness: 0.2
    });
    const padMat = new THREE.MeshStandardMaterial({
        color: 0x1e293b,
        roughness: 0.9
    });

    // Base Pedestal
    const baseGroup = new THREE.Group();
    const pedestal = new THREE.Mesh(new THREE.CylinderGeometry(0.14, 0.17, 0.09, 36), darkMotorMat);
    pedestal.position.y = 0.045;
    pedestal.castShadow = true;
    baseGroup.add(pedestal);

    const baseRing = new THREE.Mesh(new THREE.TorusGeometry(0.14, 0.01, 16, 48), cyanRingMat);
    baseRing.rotation.x = Math.PI / 2;
    baseRing.position.y = 0.09;
    baseGroup.add(baseRing);
    scene.add(baseGroup);

    // Joint 1: Base Yaw (rotates around Y)
    const j1 = new THREE.Group();
    j1.position.y = 0.09;
    const j1Mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.10, 0.10, 0.13, 32), whiteShellMat);
    j1Mesh.position.y = 0.065;
    j1Mesh.castShadow = true;
    j1.add(j1Mesh);
    baseGroup.add(j1);
    robotJoints[0] = { group: j1, axis: 'y' };

    // Joint 2: Shoulder Pitch (rotates around Z)
    const j2 = new THREE.Group();
    j2.position.y = 0.13;
    const j2Motor = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.16, 32), darkMotorMat);
    j2Motor.rotation.x = Math.PI / 2;
    j2Motor.castShadow = true;
    j2.add(j2Motor);
    j1.add(j2);
    robotJoints[1] = { group: j2, axis: 'z' };

    // Link 1: Upper Arm (Strong white bone with chamfered geometry)
    const upperLink = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.28, 0.09), whiteShellMat);
    upperLink.position.y = 0.14;
    upperLink.castShadow = true;
    j2.add(upperLink);

    // Joint 3: Elbow Roll (rotates around Y)
    const j3 = new THREE.Group();
    j3.position.y = 0.28;
    const j3Motor = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.075, 0.10, 28), darkMotorMat);
    j3Motor.position.y = 0.05;
    j3Motor.castShadow = true;
    j3.add(j3Motor);
    j2.add(j3);
    robotJoints[2] = { group: j3, axis: 'y' };

    // Joint 4: Elbow Pitch (rotates around Z)
    const j4 = new THREE.Group();
    j4.position.y = 0.10;
    const j4Motor = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.075, 0.14, 28), darkMotorMat);
    j4Motor.rotation.x = Math.PI / 2;
    j4Motor.castShadow = true;
    j4.add(j4Motor);
    j3.add(j4);
    robotJoints[3] = { group: j4, axis: 'z' };

    // Link 2: Forearm
    const foreLink = new THREE.Mesh(new THREE.BoxGeometry(0.075, 0.24, 0.075), whiteShellMat);
    foreLink.position.y = 0.12;
    foreLink.castShadow = true;
    j4.add(foreLink);

    // Joint 5: Wrist Roll (rotates around Y)
    const j5 = new THREE.Group();
    j5.position.y = 0.24;
    const j5Motor = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.08, 24), darkMotorMat);
    j5Motor.position.y = 0.04;
    j5Motor.castShadow = true;
    j5.add(j5Motor);
    j4.add(j5);
    robotJoints[4] = { group: j5, axis: 'y' };

    // Joint 6: Wrist Pitch (rotates around Z)
    const j6 = new THREE.Group();
    j6.position.y = 0.08;
    const j6Motor = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.11, 24), whiteShellMat);
    j6Motor.rotation.x = Math.PI / 2;
    j6Motor.castShadow = true;
    j6.add(j6Motor);
    j5.add(j6);
    robotJoints[5] = { group: j6, axis: 'z' };

    // Joint 7: Wrist Yaw (rotates around Y / tool flange)
    const j7 = new THREE.Group();
    j7.position.y = 0.08;
    const j7Flange = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.052, 0.05, 24), darkMotorMat);
    j7Flange.position.y = 0.025;
    j7Flange.castShadow = true;
    j7.add(j7Flange);
    j6.add(j7);
    robotJoints[6] = { group: j7, axis: 'y' };

    // Gripper Assembly (High Visibility Safety Orange)
    const gripperGroup = new THREE.Group();
    gripperGroup.position.y = 0.05;

    const gripperPalm = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.03, 0.06), darkMotorMat);
    gripperPalm.position.y = 0.015;
    gripperPalm.castShadow = true;
    gripperGroup.add(gripperPalm);

    // 2 Pinch Fingers
    const fingerGeo = new THREE.BoxGeometry(0.018, 0.09, 0.028);

    leftFinger = new THREE.Mesh(fingerGeo, orangeGripperMat);
    leftFinger.position.set(-0.045, 0.06, 0);
    leftFinger.castShadow = true;

    // Rubber contact pad on left finger
    const leftPad = new THREE.Mesh(new THREE.BoxGeometry(0.005, 0.08, 0.025), padMat);
    leftPad.position.x = 0.01;
    leftFinger.add(leftPad);
    gripperGroup.add(leftFinger);

    rightFinger = new THREE.Mesh(fingerGeo, orangeGripperMat);
    rightFinger.position.set(0.045, 0.06, 0);
    rightFinger.castShadow = true;

    // Rubber contact pad on right finger
    const rightPad = new THREE.Mesh(new THREE.BoxGeometry(0.005, 0.08, 0.025), padMat);
    rightPad.position.x = -0.01;
    rightFinger.add(rightPad);
    gripperGroup.add(rightFinger);

    j7.add(gripperGroup);
}

// Animation loop
function animate() {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    renderer.render(scene, camera);
}

// WebSocket client connection
function initWebSocket() {
    const wsProto = window.location.protocol === "https:" ? "wss:" : "ws:";
    const wsHost = window.location.hostname || "127.0.0.1";
    const wsUrl = `${wsProto}//${wsHost}:8889`;

    const badge = document.getElementById("ws-status-badge");
    badge.innerHTML = `WebSocket: <strong style="color:var(--amber);">CONNECTING</strong>`;

    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        badge.innerHTML = `WebSocket: <strong style="color:var(--emerald);">ONLINE</strong>`;
    };

    ws.onmessage = (evt) => {
        try {
            const msg = JSON.parse(evt.data);
            if (msg.type === "telemetry") {
                handleTelemetry(msg.data);
            } else if (msg.type === "traffic") {
                handleTraffic(msg.data);
            } else if (msg.type === "cli_output") {
                appendTerminal(msg.data);
            }
        } catch (e) {
            console.error("WS Parse error:", e);
        }
    };

    ws.onclose = () => {
        badge.innerHTML = `WebSocket: <strong style="color:var(--rose);">RETRYING</strong>`;
        setTimeout(initWebSocket, 2000);
    };

    ws.onerror = () => {
        badge.innerHTML = `WebSocket: <strong style="color:var(--rose);">ERROR</strong>`;
    };
}

function sendAction(action, payload = {}) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ action, ...payload }));
    }
}

// Update 3D Joints & UI Cards from Telemetry
function handleTelemetry(data) {
    if (!data || !data.motors) return;
    currentTelemetry = data.motors;

    // Update Header counters
    document.getElementById("stat-rx").textContent = data.frames_rx || 0;
    document.getElementById("stat-tx").textContent = data.frames_tx || 0;

    data.motors.forEach(m => {
        // Update 3D Arm Joints
        if (m.id >= 1 && m.id <= 7 && robotJoints[m.id - 1]) {
            const joint = robotJoints[m.id - 1];
            joint.group.rotation[joint.axis] = m.q;
        }

        // Update Gripper in 3D
        if (m.id === 8 && leftFinger && rightFinger) {
            // m.q in [0, 1]
            const fingerOffset = 0.045 - (m.q * 0.032);
            leftFinger.position.x = -fingerOffset;
            rightFinger.position.x = fingerOffset;
        }

        // Update Telemetry Card
        const chip = document.getElementById(`chip-${m.id}`);
        if (chip) {
            chip.className = m.enabled ? "tel-chip chip-enabled" : "tel-chip chip-disabled";
            chip.textContent = m.enabled ? "ARMED" : "OFF";
        }

        const qElem = document.getElementById(`tel-q-${m.id}`);
        if (qElem) qElem.textContent = m.q.toFixed(3);

        const dqElem = document.getElementById(`tel-dq-${m.id}`);
        if (dqElem) dqElem.textContent = m.dq.toFixed(2);

        const tauElem = document.getElementById(`tel-tau-${m.id}`);
        if (tauElem) tauElem.textContent = m.tau.toFixed(2);

        const mosElem = document.getElementById(`tel-tmos-${m.id}`);
        if (mosElem) mosElem.textContent = `${m.t_mos.toFixed(0)}°C`;
    });
}

// Traffic table update
function handleTraffic(frames) {
    if (!frames || !frames.length) return;
    const tbody = document.getElementById("traffic-tbody");
    tbody.innerHTML = "";

    const recent = frames.slice(-18).reverse();
    recent.forEach(f => {
        const tr = document.createElement("tr");
        const dirClass = f.dir === "RX" ? "dir-rx" : "dir-tx";
        
        let typeName = "CAN_FRAME";
        let typeClass = "can-type-state";
        if (f.id === "0x7ff") {
            typeName = "MGMT (RID/REF)";
            typeClass = "can-type-mgmt";
        } else if (f.hex.startsWith("FFFFFFFFFFFFFF")) {
            typeName = "CMD (STATE)";
            typeClass = "can-type-mgmt";
        } else if (f.dir === "TX" && f.id.startsWith("0x1")) {
            typeName = "STATE_TELEM";
            typeClass = "can-type-state";
        } else if (f.dir === "RX") {
            typeName = "MIT_CONTROL";
            typeClass = "can-type-mit";
        }

        tr.innerHTML = `
            <td class="${dirClass}"><strong>${f.dir}</strong></td>
            <td><code>${f.id}</code></td>
            <td>${f.dlc}</td>
            <td><code>${f.hex.substring(0, 16)}...</code></td>
            <td class="${typeClass}"><strong>${typeName}</strong></td>
        `;
        tbody.appendChild(tr);
    });
}

// Trajectory Generator (Home, Wave, Reach, Sine)
let trajectoryInterval = null;
let trajTime = 0;

function startPreset(name) {
    stopPreset();
    activePreset = name;

    document.querySelectorAll(".btn-preset").forEach(b => {
        if (b.dataset.preset === name) b.classList.add("active");
    });

    sendAction("enable_all");
    trajTime = 0;

    trajectoryInterval = setInterval(() => {
        trajTime += 0.02; // 50 Hz

        if (name === "home") {
            for (let id = 1; id <= 7; id++) {
                sendAction("set_mit", { id, q: 0.0, kp: 35.0, kd: 1.5, tau: 0.0 });
            }
            sendAction("set_gripper", { pos: 0.0 });
        }
        else if (name === "wave") {
            const j2 = -0.6;
            const j4 = -1.2;
            const j6 = Math.sin(trajTime * 4.0) * 0.7;
            const j7 = Math.cos(trajTime * 4.0) * 0.4;
            sendAction("set_mit", { id: 2, q: j2, kp: 35.0, kd: 1.5, tau: 0.0 });
            sendAction("set_mit", { id: 4, q: j4, kp: 30.0, kd: 1.2, tau: 0.0 });
            sendAction("set_mit", { id: 6, q: j6, kp: 25.0, kd: 1.0, tau: 0.0 });
            sendAction("set_mit", { id: 7, q: j7, kp: 20.0, kd: 1.0, tau: 0.0 });
        }
        else if (name === "reach") {
            const phase = Math.sin(trajTime * 1.5);
            const j1 = phase * 0.4;
            const j2 = -0.5 - phase * 0.2;
            const j4 = -0.8 - phase * 0.3;
            const grip = phase > 0 ? 0.9 : 0.0;
            sendAction("set_mit", { id: 1, q: j1, kp: 35.0, kd: 1.5, tau: 0.0 });
            sendAction("set_mit", { id: 2, q: j2, kp: 35.0, kd: 1.5, tau: 0.0 });
            sendAction("set_mit", { id: 4, q: j4, kp: 30.0, kd: 1.2, tau: 0.0 });
            sendAction("set_gripper", { pos: grip });
        }
        else if (name === "sine") {
            for (let id = 1; id <= 7; id++) {
                const targetQ = Math.sin(trajTime * 2.0 + id * 0.8) * 0.5;
                sendAction("set_mit", { id, q: targetQ, kp: 28.0, kd: 1.2, tau: 0.0 });
            }
        }
    }, 20);
}

function stopPreset() {
    if (trajectoryInterval) {
        clearInterval(trajectoryInterval);
        trajectoryInterval = null;
    }
    activePreset = null;
    document.querySelectorAll(".btn-preset").forEach(b => b.classList.remove("active"));
}

// Terminal Output
function runCliCommand(subcommand) {
    const term = document.getElementById("term-output");
    term.textContent = `\n$ openarm-can-cli -i vcan0 ${subcommand}\nExecuting command over SocketCAN... please wait...\n`;
    sendAction("run_cli", { cmd: subcommand });
}

function appendTerminal(output) {
    const term = document.getElementById("term-output");
    term.textContent = output;
    term.scrollTop = term.scrollHeight;
}
