// ==============================================================================
// OPENARM 3D FUTURE ACTION PATH VISUALIZER (ACT INFERENCE HORIZON)
// Renders the predicted 50-step (1.0s) future motion trajectories for Left & Right arms:
//   - Left TCP Trajectory: Neon Cyan (#00e5ff) spline & waypoints
//   - Right TCP Trajectory: Neon Orange (#ff6d00) spline & waypoints
//   - Forward Kinematics (FK) projection in real-time Three.js scene
//   - Target Pose Goal Markers at t=50 with pulsing glow
// ==============================================================================

let futurePathGroup = null;
let leftPathLine = null;
let rightPathLine = null;
let leftPointsObj = null;
let rightPointsObj = null;
let leftGoalMarker = null;
let rightGoalMarker = null;

// Display settings
let showFuturePath = true;
let showLeftPath = true;
let showRightPath = true;
let showWaypoints = true;
let pathHorizonSteps = 50;

// Temporary math buffers for ultra-low GC allocation
const _vLeft = new THREE.Vector3();
const _vRight = new THREE.Vector3();

function initFuturePathVisualizer() {
    if (!scene) return;
    if (futurePathGroup) return; // already initialized

    futurePathGroup = new THREE.Group();
    futurePathGroup.name = "FutureActionPathGroup";

    const MAX_STEPS = 60;

    // 1. LEFT ARM PATH (Neon Cyan)
    const leftPositions = new Float32Array(MAX_STEPS * 3);
    const leftGeo = new THREE.BufferGeometry();
    leftGeo.setAttribute("position", new THREE.BufferAttribute(leftPositions, 3));
    leftGeo.setDrawRange(0, 0);

    const leftLineMat = new THREE.LineBasicMaterial({
        color: 0x00e5ff,
        linewidth: 3,
        transparent: true,
        opacity: 0.95,
        depthTest: true,
    });
    leftPathLine = new THREE.Line(leftGeo, leftLineMat);
    futurePathGroup.add(leftPathLine);

    // Left Arm Waypoint Dots
    const leftPointsMat = new THREE.PointsMaterial({
        color: 0x00e5ff,
        size: 0.012,
        transparent: true,
        opacity: 0.85,
    });
    leftPointsObj = new THREE.Points(leftGeo, leftPointsMat);
    futurePathGroup.add(leftPointsObj);

    // Left Arm End Goal Sphere (t=50)
    const goalSphereGeo = new THREE.SphereGeometry(0.016, 16, 16);
    const leftGoalMat = new THREE.MeshBasicMaterial({
        color: 0x00e5ff,
        wireframe: true,
        transparent: true,
        opacity: 0.9,
    });
    leftGoalMarker = new THREE.Mesh(goalSphereGeo, leftGoalMat);
    leftGoalMarker.visible = false;
    futurePathGroup.add(leftGoalMarker);

    // 2. RIGHT ARM PATH (Neon Orange)
    const rightPositions = new Float32Array(MAX_STEPS * 3);
    const rightGeo = new THREE.BufferGeometry();
    rightGeo.setAttribute("position", new THREE.BufferAttribute(rightPositions, 3));
    rightGeo.setDrawRange(0, 0);

    const rightLineMat = new THREE.LineBasicMaterial({
        color: 0xff6d00,
        linewidth: 3,
        transparent: true,
        opacity: 0.95,
        depthTest: true,
    });
    rightPathLine = new THREE.Line(rightGeo, rightLineMat);
    futurePathGroup.add(rightPathLine);

    // Right Arm Waypoint Dots
    const rightPointsMat = new THREE.PointsMaterial({
        color: 0xff6d00,
        size: 0.012,
        transparent: true,
        opacity: 0.85,
    });
    rightPointsObj = new THREE.Points(rightGeo, rightPointsMat);
    futurePathGroup.add(rightPointsObj);

    // Right Arm End Goal Sphere (t=50)
    const rightGoalMat = new THREE.MeshBasicMaterial({
        color: 0xff6d00,
        wireframe: true,
        transparent: true,
        opacity: 0.9,
    });
    rightGoalMarker = new THREE.Mesh(goalSphereGeo, rightGoalMat);
    rightGoalMarker.visible = false;
    futurePathGroup.add(rightGoalMarker);

    scene.add(futurePathGroup);
    console.log("[3D] Future Action Path Visualizer initialized in Three.js scene.");
}

/**
 * Procedural Forward Kinematics (FK) calculation for OpenArm 7-DOF kinematics.
 * Computes world space Cartesian position (x, y, z) of the End-Effector TCP.
 */
function computeProceduralArmFK(isLeft, q1, q2, q3, q4, q5, q6, q7) {
    // OpenArm Torso Geometry
    const shoulderX = isLeft ? -0.165 : 0.165;
    const shoulderY = 0.70;
    const shoulderZ = 0.0;

    // Segment lengths
    const L_upper = 0.250; // J1 -> J3
    const L_lower = 0.220; // J3 -> J5
    const L_wrist = 0.136; // J5 -> TCP

    // Direction sign conventions
    const sign = isLeft ? -1.0 : 1.0;

    // Transformation angles
    const yaw = sign * q1;
    const pitch1 = q2;
    const roll1 = sign * q3;
    const pitch2 = q4;

    // Cumulative pitch
    const totalPitch = pitch1 + pitch2;

    const dx = Math.sin(yaw) * (L_upper * Math.sin(pitch1) + L_lower * Math.sin(totalPitch) + L_wrist * Math.sin(totalPitch + q6));
    const dy = - (L_upper * Math.cos(pitch1) + L_lower * Math.cos(totalPitch) + L_wrist * Math.cos(totalPitch + q6));
    const dz = Math.cos(yaw) * (L_upper * Math.sin(pitch1) + L_lower * Math.sin(totalPitch) + L_wrist * Math.sin(totalPitch + q6));

    return new THREE.Vector3(
        shoulderX + dx,
        shoulderY + dy,
        shoulderZ + dz + 0.18
    );
}

/**
 * Computes Tool Center Point (TCP) positions for an action vector [16].
 * Uses official URDF robot when available, with procedural kinematics fallback.
 */
function computeTCPPositions(action16) {
    if (!action16 || action16.length < 16) {
        return { left: null, right: null };
    }

    if (openArmUrdfRobot && openArmModelMode === "urdf") {
        // 1. Save current robot joint states
        const savedJointValues = {};
        for (let motorId = 1; motorId <= 16; motorId++) {
            const jointName = motorToUrdfJoint[motorId];
            if (openArmUrdfRobot.joints && openArmUrdfRobot.joints[jointName]) {
                savedJointValues[jointName] = openArmUrdfRobot.joints[jointName].jointValue;
            }
        }

        // 2. Set action vector joint angles
        for (let motorId = 1; motorId <= 16; motorId++) {
            const jointName = motorToUrdfJoint[motorId];
            let val = action16[motorId - 1];
            if (motorId === 1) val = -val;
            if (motorId === 8 || motorId === 16) val = Math.max(0.0, Math.min(0.043, Math.abs(val)));

            if (openArmUrdfRobot.joints && openArmUrdfRobot.joints[jointName]) {
                openArmUrdfRobot.setJointValue(jointName, val);
            }
        }

        // 3. Update world transformation matrices
        openArmUrdfRobot.updateMatrixWorld(true);

        const leftTcpLink =
            (openArmUrdfRobot.links && openArmUrdfRobot.links["openarm_left_hand_tcp"]) ||
            openArmUrdfRobot.getObjectByName("openarm_left_hand_tcp") ||
            (openArmUrdfRobot.links && openArmUrdfRobot.links["openarm_left_hand"]) ||
            openArmUrdfRobot.getObjectByName("openarm_left_hand");

        const rightTcpLink =
            (openArmUrdfRobot.links && openArmUrdfRobot.links["openarm_right_hand_tcp"]) ||
            openArmUrdfRobot.getObjectByName("openarm_right_hand_tcp") ||
            (openArmUrdfRobot.links && openArmUrdfRobot.links["openarm_right_hand"]) ||
            openArmUrdfRobot.getObjectByName("openarm_right_hand");

        const leftPos = new THREE.Vector3();
        const rightPos = new THREE.Vector3();

        if (leftTcpLink) leftTcpLink.getWorldPosition(leftPos);
        if (rightTcpLink) rightTcpLink.getWorldPosition(rightPos);

        // 4. Restore original robot joint states
        for (const [jName, jVal] of Object.entries(savedJointValues)) {
            if (openArmUrdfRobot.joints && openArmUrdfRobot.joints[jName]) {
                openArmUrdfRobot.setJointValue(jName, jVal);
            }
        }
        openArmUrdfRobot.updateMatrixWorld(true);

        return { left: leftPos, right: rightPos };
    }

    // Procedural Fallback
    const leftPos = computeProceduralArmFK(
        true,
        action16[0], action16[1], action16[2], action16[3], action16[4], action16[5], action16[6]
    );
    const rightPos = computeProceduralArmFK(
        false,
        action16[8], action16[9], action16[10], action16[11], action16[12], action16[13], action16[14]
    );
    return { left: leftPos, right: rightPos };
}

/**
 * Updates 3D visual lines and waypoints from the predicted future_actions (50 x 16 array).
 */
function updateFutureActionPath(futureActions) {
    if (!futurePathGroup) initFuturePathVisualizer();
    if (!futurePathGroup) return;

    if (!showFuturePath || !futureActions || !Array.isArray(futureActions) || futureActions.length === 0) {
        if (leftPathLine) leftPathLine.geometry.setDrawRange(0, 0);
        if (rightPathLine) rightPathLine.geometry.setDrawRange(0, 0);
        if (leftGoalMarker) leftGoalMarker.visible = false;
        if (rightGoalMarker) rightGoalMarker.visible = false;
        return;
    }

    const steps = Math.min(futureActions.length, pathHorizonSteps);

    // Buffers for Left and Right Arm coordinates
    const leftPositions = leftPathLine.geometry.attributes.position.array;
    const rightPositions = rightPathLine.geometry.attributes.position.array;

    let validLeftSteps = 0;
    let validRightSteps = 0;

    let lastLeftPt = null;
    let lastRightPt = null;

    for (let i = 0; i < steps; i++) {
        const action = futureActions[i];
        if (!action || action.length < 16) continue;

        const { left, right } = computeTCPPositions(action);

        if (left && showLeftPath) {
            leftPositions[validLeftSteps * 3 + 0] = left.x;
            leftPositions[validLeftSteps * 3 + 1] = left.y;
            leftPositions[validLeftSteps * 3 + 2] = left.z;
            lastLeftPt = left;
            validLeftSteps++;
        }

        if (right && showRightPath) {
            rightPositions[validRightSteps * 3 + 0] = right.x;
            rightPositions[validRightSteps * 3 + 1] = right.y;
            rightPositions[validRightSteps * 3 + 2] = right.z;
            lastRightPt = right;
            validRightSteps++;
        }
    }

    // 1. Update Left Arm Visuals
    if (leftPathLine) {
        leftPathLine.geometry.attributes.position.needsUpdate = true;
        leftPathLine.geometry.setDrawRange(0, validLeftSteps);
        leftPathLine.visible = showLeftPath && validLeftSteps > 1;
    }
    if (leftPointsObj) {
        leftPointsObj.visible = showLeftPath && showWaypoints && validLeftSteps > 0;
    }
    if (leftGoalMarker) {
        if (lastLeftPt && showLeftPath) {
            leftGoalMarker.position.copy(lastLeftPt);
            leftGoalMarker.visible = true;
        } else {
            leftGoalMarker.visible = false;
        }
    }

    // 2. Update Right Arm Visuals
    if (rightPathLine) {
        rightPathLine.geometry.attributes.position.needsUpdate = true;
        rightPathLine.geometry.setDrawRange(0, validRightSteps);
        rightPathLine.visible = showRightPath && validRightSteps > 1;
    }
    if (rightPointsObj) {
        rightPointsObj.visible = showRightPath && showWaypoints && validRightSteps > 0;
    }
    if (rightGoalMarker) {
        if (lastRightPt && showRightPath) {
            rightGoalMarker.position.copy(lastRightPt);
            rightGoalMarker.visible = true;
        } else {
            rightGoalMarker.visible = false;
        }
    }
}

/**
 * Toggle Visibility APIs for UI Buttons and Checkboxes
 */
function setFuturePathVisibility(visible) {
    showFuturePath = Boolean(visible);
    if (futurePathGroup) futurePathGroup.visible = showFuturePath;
}

function setLeftPathVisibility(visible) {
    showLeftPath = Boolean(visible);
}

function setRightPathVisibility(visible) {
    showRightPath = Boolean(visible);
}

function setPathHorizonSteps(numSteps) {
    pathHorizonSteps = Math.max(5, Math.min(60, Number(numSteps) || 50));
}
