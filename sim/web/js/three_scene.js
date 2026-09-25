// ==============================================================================
// OPENARM 3D VIEWPORT & SCENE MANAGER (THREE.JS)
// Handles scene setup, lighting, camera controls, viewpoint presets, and render loop
// ==============================================================================

let scene, camera, renderer, controls;
let gridHelper, axesHelper, floorMesh;

// Kinematics containers for both arms
let leftArmJoints = [];  // J0..J6 groups
let rightArmJoints = []; // J0..J6 groups
let leftGripperFingers = { left: null, right: null };
let rightGripperFingers = { left: null, right: null };

function initThreeJS() {
    const container = document.getElementById("three-container");
    if (!container) return;

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
    if (typeof buildBimanualOpenArm === "function") {
        buildBimanualOpenArm();
    }

    window.addEventListener("resize", onWindowResize);
    animate();
}

function resetCamera() {
    if (!camera) return;
    camera.position.set(0.0, 0.46, 1.25);
    if (controls) controls.target.set(0, 0.40, 0);
}

function setFrontView() {
    if (!camera) return;
    camera.position.set(0, 0.42, 1.20);
    if (controls) controls.target.set(0, 0.40, 0);
}

function setLeftArmView() {
    if (!camera) return;
    camera.position.set(-0.28, 0.38, 0.65);
    if (controls) controls.target.set(-0.11, 0.35, 0);
}

function setRightArmView() {
    if (!camera) return;
    camera.position.set(0.28, 0.38, 0.65);
    if (controls) controls.target.set(0.11, 0.35, 0);
}

function setTopView() {
    if (!camera) return;
    camera.position.set(0, 1.35, 0.05);
    if (controls) controls.target.set(0, 0.40, 0);
}

function onWindowResize() {
    const container = document.getElementById("three-container");
    if (!container || !camera || !renderer) return;
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
}

// Render Animation Loop
function animate() {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    if (renderer && scene && camera) {
        renderer.render(scene, camera);
    }
}
