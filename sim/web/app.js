// ==============================================================================
// OPENARM BIMANUAL 7-DOF CAN DASHBOARD & DIGITAL TWIN
// Master Application Bootstrap & Lifecycle Coordinator
// Modular Architecture:
//   - js/config.js         : Joint maps, mechanical limits, shared constants
//   - js/network.js        : WebSocket client, USB connection, CLI, Toasts
//   - js/three_scene.js    : Three.js viewport, camera controls, render loop
//   - js/three_model.js    : 3D procedural robot CAD models & parallel grippers
//   - js/ui_sliders.js     : Joint sliders, safety locks, dashboard controls
//   - js/ui_target.js      : Target Joint State input, units, trajectory execution
//   - js/telemetry.js      : 16 motor cards, 3D kinematics sync, 100Hz exporter UI
//   - js/dance_routines.js : Dance choreography mathematical routines
//   - js/dance_engine.js   : Dance player state machine, tempo, basic presets
// ==============================================================================

document.addEventListener("DOMContentLoaded", () => {
    // 1. Set default bright studio theme
    document.body.classList.add("theme-light");

    // 2. Build UI sliders and telemetry cards
    buildJointSliders();
    buildTelemetryCards();

    // 3. Initialize Three.js 3D Digital Twin Viewport
    initThreeJS();

    // 4. Connect to backend WebSocket (Telemetry & CAN-FD)
    initWebSocket();

    // 5. Initialize RealSense Camera Feed & ROS 2 Preview
    initCameraFeed();

    // 6. Wire UI event listeners & tools
    setupEventHandlers();
    setupTargetJointUI();

    // 7. Initialize Robot Dance Studio Subsystem
    initDanceEngine();

    console.log("[INIT] OpenArm Modular Dashboard initialized successfully.");
});

function initCameraFeed() {
    const image = document.getElementById("camera-feed");
    const status = document.getElementById("camera-feed-status");
    const placeholder = document.getElementById("camera-feed-placeholder");
    if (!image || !status || !placeholder) return;

    const cameraHost = window.location.hostname || "127.0.0.1";
    const cameraBase = `http://${cameraHost}:8890`;
    let retryTimer = null;
    let streamRequested = false;

    const setStatus = (state, label) => {
        status.className = `camera-feed-status ${state}`;
        status.innerHTML = `<span class="camera-status-dot"></span>${label}`;
    };

    const connectStream = () => {
        clearTimeout(retryTimer);
        streamRequested = true;
        setStatus("waiting", "WAITING");
        image.src = `${cameraBase}/stream.mjpg?t=${Date.now()}`;
    };

    image.addEventListener("load", () => {
        image.classList.add("is-live");
        placeholder.style.display = "none";
    });

    image.addEventListener("error", () => {
        streamRequested = false;
        image.classList.remove("is-live");
        placeholder.style.display = "flex";
        setStatus("offline", "OFFLINE");
        retryTimer = setTimeout(connectStream, 3000);
    });

    const refreshStatus = async () => {
        try {
            const response = await fetch(`${cameraBase}/status?t=${Date.now()}`, {
                cache: "no-store",
            });
            if (!response.ok) throw new Error(`HTTP ${response.status}`);
            const data = await response.json();
            if (data.streaming) {
                image.classList.add("is-live");
                placeholder.style.display = "none";
                if (data.has_live_feed) {
                    setStatus("live", "LIVE");
                } else {
                    setStatus("waiting", "STANDBY");
                }
                if (!streamRequested) connectStream();
            } else {
                image.classList.remove("is-live");
                placeholder.style.display = "flex";
                setStatus("offline", "OFFLINE");
            }
        } catch (_error) {
            image.classList.remove("is-live");
            placeholder.style.display = "flex";
            setStatus("offline", "OFFLINE");
        }
    };

    connectStream();
    refreshStatus();
    window.setInterval(refreshStatus, 2500);
}
