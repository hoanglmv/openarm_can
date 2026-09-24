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

    // 5. Wire UI event listeners & tools
    setupEventHandlers();
    setupTargetJointUI();

    // 6. Initialize Robot Dance Studio Subsystem
    initDanceEngine();

    console.log("✓ OpenArm Modular Dashboard initialized successfully.");
});
