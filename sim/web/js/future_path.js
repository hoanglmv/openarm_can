// ==============================================================================
// OPENARM 3D FUTURE ACTION PATH VISUALIZER (ACT INFERENCE 50Hz HORIZON)
// Conforms strictly to KE_HOACH_REALTIME_VISUAL_50HZ.md:
//   1. 50-step (1.0s @ 50Hz) Future Action Horizon (Left: Neon Cyan, Right: Neon Orange)
//   2. 6-DOF End-Effector TCP Pose Kinematics (Forward Kinematics SE(3))
//   3. Standard 3-Axis Triad Gizmo:
//      - 🔵 Z-Axis (Blue #0088ff): Tool Forward / Approach Vector (pointing toward target)
//      - 🔴 X-Axis (Red #ff3333): Lateral Vector (parallel to gripper jaws J8/J16)
//      - 🟢 Y-Axis (Green #33cc33): Normal Vector (u_y = u_z x u_x)
//   4. Destination step t=50: Large 3-axis Triad (0.05m) + Ghost Gripper wireframe
//   5. Intermediate waypoints (t=10, 20, 30, 40): Secondary sub-triads (0.025m)
// ==============================================================================

let futurePathGroup = null;
let leftPathLine = null;
let rightPathLine = null;
let leftPointsObj = null;
let rightPointsObj = null;
let leftGoalMarker = null;
let rightGoalMarker = null;

// Triad Gizmos & Ghost Grippers
let leftTriadGroup = null;
let rightTriadGroup = null;
let leftGhostGripper = null;
let rightGhostGripper = null;

// Display settings
let showFuturePath = true;
let showLeftPath = true;
let showRightPath = true;
let showWaypoints = true;
let showTriads = true;
let showGhostGripper = true;
let pathHorizonSteps = 50;
let triadSize = 0.05; // 0.05m = 5cm
let triadStride = 10; // Draw triad every 10 steps (t=10, 20, 30, 40, 50)

// Pooled arrow helpers for zero-allocation performance
const leftTriadPool = [];
const rightTriadPool = [];
const MAX_TRIADS_PER_ARM = 12;

function createTriadInstance() {
    const group = new THREE.Group();
    // Z-Axis (Blue): Tool Forward
    const arrowZ = new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, 1),
        new THREE.Vector3(0, 0, 0),
        triadSize,
        0x0088ff,
        0.016,
        0.009
    );
    // X-Axis (Red): Lateral Gripper Jaws
    const arrowX = new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0, 0, 0),
        triadSize,
        0xff3333,
        0.016,
        0.009
    );
    // Y-Axis (Green): Normal Vector
    const arrowY = new THREE.ArrowHelper(
        new THREE.Vector3(0, 1, 0),
        new THREE.Vector3(0, 0, 0),
        triadSize,
        0x33cc33,
        0.016,
        0.009
    );

    group.add(arrowZ);
    group.add(arrowX);
    group.add(arrowY);
    group.visible = false;

    return {
        group: group,
        arrowZ: arrowZ,
        arrowX: arrowX,
        arrowY: arrowY,
    };
}

function createGhostGripper(colorHex) {
    // Builds a wireframe representation of the 2-jaw parallel gripper:
    // Base palm bar + 2 forward-pointing finger jaws
    const geom = new THREE.BufferGeometry();
    // 6 lines = 12 vertices (x, y, z)
    const positions = new Float32Array(12 * 3);
    geom.setAttribute("position", new THREE.BufferAttribute(positions, 3));

    const mat = new THREE.LineBasicMaterial({
        color: colorHex,
        linewidth: 2,
        transparent: true,
        opacity: 0.85,
        depthTest: true,
    });

    const lines = new THREE.LineSegments(geom, mat);
    lines.visible = false;
    return lines;
}

function updateGhostGripperGeometry(gripperObj, pos, forward, jaw, normal, strokeMeters) {
    if (!gripperObj) return;
    const halfWidth = Math.max(0.008, Math.min(0.05, (strokeMeters || 0.02) * 0.5 + 0.008));
    const fingerLen = 0.040; // 4cm finger length forward
    const baseOffset = 0.015; // 1.5cm palm bracket

    const p = pos;
    const f = forward;
    const j = jaw;
    const n = normal;

    // Palm base center (shifted slightly backwards along -forward)
    const baseC = p.clone().addScaledVector(f, -baseOffset);
    // Palm left corner
    const baseL = baseC.clone().addScaledVector(j, -halfWidth);
    // Palm right corner
    const baseR = baseC.clone().addScaledVector(j, halfWidth);
    // Left finger tip
    const tipL = baseL.clone().addScaledVector(f, fingerLen);
    // Right finger tip
    const tipR = baseR.clone().addScaledVector(f, fingerLen);
    // Palm mounting stub
    const mountBack = baseC.clone().addScaledVector(f, -0.02);

    const arr = gripperObj.geometry.attributes.position.array;
    let idx = 0;

    function addLine(p1, p2) {
        arr[idx++] = p1.x; arr[idx++] = p1.y; arr[idx++] = p1.z;
        arr[idx++] = p2.x; arr[idx++] = p2.y; arr[idx++] = p2.z;
    }

    // Line 1: Palm cross bar (between left and right jaw base)
    addLine(baseL, baseR);
    // Line 2: Left finger jaw (from base to tip)
    addLine(baseL, tipL);
    // Line 3: Right finger jaw (from base to tip)
    addLine(baseR, tipR);
    // Line 4: Palm mount backward stub
    addLine(baseC, mountBack);
    // Line 5: Left fingertip hook inwards
    addLine(tipL, tipL.clone().addScaledVector(j, 0.006));
    // Line 6: Right fingertip hook inwards
    addLine(tipR, tipR.clone().addScaledVector(j, -0.006));

    gripperObj.geometry.attributes.position.needsUpdate = true;
    gripperObj.visible = true;
}

function initFuturePathVisualizer() {
    if (!scene) return;
    if (futurePathGroup) return; // already initialized

    futurePathGroup = new THREE.Group();
    futurePathGroup.name = "FutureActionPathGroup";

    const MAX_STEPS = 65;

    // 1. LEFT ARM PATH (Neon Cyan #00e5ff)
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
        size: 0.010,
        transparent: true,
        opacity: 0.85,
    });
    leftPointsObj = new THREE.Points(leftGeo, leftPointsMat);
    futurePathGroup.add(leftPointsObj);

    // Left Arm Goal Marker Sphere (t=50)
    const goalSphereGeo = new THREE.SphereGeometry(0.012, 16, 16);
    const leftGoalMat = new THREE.MeshBasicMaterial({
        color: 0x00e5ff,
        wireframe: true,
        transparent: true,
        opacity: 0.85,
    });
    leftGoalMarker = new THREE.Mesh(goalSphereGeo, leftGoalMat);
    leftGoalMarker.visible = false;
    futurePathGroup.add(leftGoalMarker);

    // 2. RIGHT ARM PATH (Neon Orange #ff6d00)
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
        size: 0.010,
        transparent: true,
        opacity: 0.85,
    });
    rightPointsObj = new THREE.Points(rightGeo, rightPointsMat);
    futurePathGroup.add(rightPointsObj);

    // Right Arm Goal Marker Sphere (t=50)
    const rightGoalMat = new THREE.MeshBasicMaterial({
        color: 0xff6d00,
        wireframe: true,
        transparent: true,
        opacity: 0.85,
    });
    rightGoalMarker = new THREE.Mesh(goalSphereGeo, rightGoalMat);
    rightGoalMarker.visible = false;
    futurePathGroup.add(rightGoalMarker);

    // 3. TRIAD GIZMOS (Left & Right Pools)
    leftTriadGroup = new THREE.Group();
    leftTriadGroup.name = "LeftTriadGroup";
    rightTriadGroup = new THREE.Group();
    rightTriadGroup.name = "RightTriadGroup";

    for (let i = 0; i < MAX_TRIADS_PER_ARM; i++) {
        const itemL = createTriadInstance();
        leftTriadPool.push(itemL);
        leftTriadGroup.add(itemL.group);

        const itemR = createTriadInstance();
        rightTriadPool.push(itemR);
        rightTriadGroup.add(itemR.group);
    }

    futurePathGroup.add(leftTriadGroup);
    futurePathGroup.add(rightTriadGroup);

    // 4. GHOST GRIPPERS (at step t=50)
    leftGhostGripper = createGhostGripper(0x00e5ff);
    rightGhostGripper = createGhostGripper(0xff6d00);
    futurePathGroup.add(leftGhostGripper);
    futurePathGroup.add(rightGhostGripper);

    scene.add(futurePathGroup);
    console.log("[3D] Future Action Path Visualizer & 3-Axis Triad Gizmos (Z-Forward, X-Jaw, Y-Normal) initialized.");
}

/**
 * 6-DOF Forward Kinematics (FK) for OpenArm 7-DOF arm kinematics.
 * Computes world Cartesian position (x, y, z) and orientation triad:
 *   - Z-axis (Blue): Tool forward approach vector pointing towards target
 *   - X-axis (Red): Lateral vector parallel to gripper jaws
 *   - Y-axis (Green): Normal vector completing right-hand triad (u_y = u_z x u_x)
 */
function computeArmPoseFK(isLeft, q1, q2, q3, q4, q5, q6, q7, gripperVal = 0.02) {
    const sign = isLeft ? -1.0 : 1.0;
    const shoulderX = isLeft ? -0.165 : 0.165;
    const shoulderY = 0.70;
    const shoulderZ = 0.0;

    // Kinematic link lengths (meters)
    const L_upper = 0.250; // J1 -> J3
    const L_lower = 0.220; // J3 -> J5
    const L_wrist = 0.136; // J5 -> TCP tip

    // Construct homogeneous transformation chain SE(3)
    const T = new THREE.Matrix4();
    T.makeTranslation(shoulderX, shoulderY, shoulderZ + 0.02);

    // 1. Joint 1: Shoulder Yaw
    const mY1 = new THREE.Matrix4().makeRotationY(sign * q1);
    T.multiply(mY1);

    // 2. Joint 2: Shoulder Pitch (pitches forward/down)
    const mX2 = new THREE.Matrix4().makeRotationX(q2);
    T.multiply(mX2);

    // Upper arm translation along local -Y
    const mT_upper = new THREE.Matrix4().makeTranslation(0, -L_upper, 0);
    T.multiply(mT_upper);

    // 3. Joint 3: Upper Arm Axial Roll
    const mY3 = new THREE.Matrix4().makeRotationY(sign * q3);
    T.multiply(mY3);

    // 4. Joint 4: Elbow Pitch
    const mX4 = new THREE.Matrix4().makeRotationX(q4);
    T.multiply(mX4);

    // Forearm translation along local -Y
    const mT_lower = new THREE.Matrix4().makeTranslation(0, -L_lower, 0);
    T.multiply(mT_lower);

    // 5. Joint 5: Forearm Axial Roll
    const mY5 = new THREE.Matrix4().makeRotationY(sign * q5);
    T.multiply(mY5);

    // 6. Joint 6: Wrist Pitch
    const mX6 = new THREE.Matrix4().makeRotationX(q6);
    T.multiply(mX6);

    // Wrist translation along local -Y to hand/gripper
    const mT_wrist = new THREE.Matrix4().makeTranslation(0, -L_wrist, 0);
    T.multiply(mT_wrist);

    // 7. Joint 7: Wrist Roll / Yaw
    const mY7 = new THREE.Matrix4().makeRotationY(sign * q7);
    T.multiply(mY7);

    // Tool Frame Alignment according to KE_HOACH_REALTIME_VISUAL_50HZ.md:
    // Z-axis: Tool forward approach vector (points out of palm toward target) -> map local -Y to +Z
    // X-axis: Lateral vector parallel to gripper jaws -> map local X to +X
    // Y-axis: Normal vector (perpendicular to jaws) -> u_y = u_z x u_x -> map local +Z to +Y
    const mToolAlign = new THREE.Matrix4().set(
        1,  0,  0, 0,
        0,  0,  1, 0,
        0, -1,  0, 0,
        0,  0,  0, 1
    );
    T.multiply(mToolAlign);

    const pos = new THREE.Vector3().setFromMatrixPosition(T);
    const quat = new THREE.Quaternion().setFromRotationMatrix(T);

    // Extract unit basis vectors in world space:
    const forward = new THREE.Vector3(0, 0, 1).applyQuaternion(quat).normalize();
    const jaw = new THREE.Vector3(1, 0, 0).applyQuaternion(quat).normalize();
    const normal = new THREE.Vector3(0, 1, 0).applyQuaternion(quat).normalize();

    return {
        position: pos,
        quaternion: quat,
        forward: forward,
        jaw: jaw,
        normal: normal,
        gripperStroke: Math.max(0.0, Math.min(0.043, Number(gripperVal) || 0.02)),
    };
}

/**
 * Procedural Forward Kinematics backwards-compatibility shim.
 */
function computeProceduralArmFK(isLeft, q1, q2, q3, q4, q5, q6, q7) {
    const pose = computeArmPoseFK(isLeft, q1, q2, q3, q4, q5, q6, q7, 0.02);
    return pose.position;
}

/**
 * Computes Tool Center Point (TCP) 6-DOF poses for an action vector [16].
 */
function computeTCPPositions(action16) {
    if (!action16 || action16.length < 16) {
        return { left: null, right: null };
    }

    const leftPose = computeArmPoseFK(
        true,
        action16[0], action16[1], action16[2], action16[3], action16[4], action16[5], action16[6],
        action16[7]
    );
    const rightPose = computeArmPoseFK(
        false,
        action16[8], action16[9], action16[10], action16[11], action16[12], action16[13], action16[14],
        action16[15]
    );
    return { left: leftPose, right: rightPose };
}

/**
 * Renders or updates a single Triad Gizmo (Z-Forward, X-Jaw, Y-Normal).
 */
function applyTriadGizmo(triadItem, pos, forward, jaw, normal, sizeScale = 1.0, opacity = 1.0) {
    if (!triadItem) return;
    const len = triadSize * sizeScale;
    const headLen = Math.max(0.006, len * 0.32);
    const headWidth = Math.max(0.003, len * 0.18);

    // Z-axis: Tool Forward (Blue #0088ff)
    triadItem.arrowZ.position.copy(pos);
    triadItem.arrowZ.setDirection(forward);
    triadItem.arrowZ.setLength(len, headLen, headWidth);
    if (triadItem.arrowZ.line && triadItem.arrowZ.line.material) {
        triadItem.arrowZ.line.material.transparent = opacity < 1.0;
        triadItem.arrowZ.line.material.opacity = opacity;
    }

    // X-axis: Lateral Jaw Vector (Red #ff3333)
    triadItem.arrowX.position.copy(pos);
    triadItem.arrowX.setDirection(jaw);
    triadItem.arrowX.setLength(len, headLen, headWidth);
    if (triadItem.arrowX.line && triadItem.arrowX.line.material) {
        triadItem.arrowX.line.material.transparent = opacity < 1.0;
        triadItem.arrowX.line.material.opacity = opacity;
    }

    // Y-axis: Normal Vector (Green #33cc33)
    triadItem.arrowY.position.copy(pos);
    triadItem.arrowY.setDirection(normal);
    triadItem.arrowY.setLength(len, headLen, headWidth);
    if (triadItem.arrowY.line && triadItem.arrowY.line.material) {
        triadItem.arrowY.line.material.transparent = opacity < 1.0;
        triadItem.arrowY.line.material.opacity = opacity;
    }

    triadItem.group.visible = true;
}

/**
 * Gets the current physical world pose of the robot's gripper (End-Effector / TCP)
 * directly from the active Three.js 3D scene (URDF robot or procedural fallback).
 */
function getCurrentGripperWorldPose(isLeft) {
    // 1. Check Official OpenArm URDF Model
    if (typeof openArmUrdfRobot !== "undefined" && openArmUrdfRobot && openArmUrdfRobot.links) {
        const prefix = isLeft ? "openarm_left_" : "openarm_right_";
        const fingerL = openArmUrdfRobot.links[prefix + "left_finger"];
        const fingerR = openArmUrdfRobot.links[prefix + "right_finger"];
        const tcpLink = openArmUrdfRobot.links[prefix + "hand_tcp"] || openArmUrdfRobot.links[prefix + "link7"];

        let pos = null;
        const quat = new THREE.Quaternion();

        if (fingerL && fingerR) {
            const pL = new THREE.Vector3();
            const pR = new THREE.Vector3();
            fingerL.getWorldPosition(pL);
            fingerR.getWorldPosition(pR);
            pos = new THREE.Vector3().addVectors(pL, pR).multiplyScalar(0.5);
        } else if (tcpLink) {
            pos = new THREE.Vector3();
            tcpLink.getWorldPosition(pos);
        }

        if (pos) {
            if (tcpLink) {
                tcpLink.getWorldQuaternion(quat);
            } else if (fingerL) {
                fingerL.getWorldQuaternion(quat);
            }
            const forward = new THREE.Vector3(0, 0, 1).applyQuaternion(quat).normalize();
            const jaw = new THREE.Vector3(1, 0, 0).applyQuaternion(quat).normalize();
            const normal = new THREE.Vector3(0, 1, 0).applyQuaternion(quat).normalize();

            return { position: pos, quaternion: quat, forward, jaw, normal, gripperStroke: 0.02 };
        }
    }

    // 2. Check Procedural Fallback Model
    const gripperFingers = isLeft
        ? (typeof leftGripperFingers !== "undefined" ? leftGripperFingers : null)
        : (typeof rightGripperFingers !== "undefined" ? rightGripperFingers : null);

    if (gripperFingers && gripperFingers.left && gripperFingers.right) {
        const pL = new THREE.Vector3();
        const pR = new THREE.Vector3();
        gripperFingers.left.getWorldPosition(pL);
        gripperFingers.right.getWorldPosition(pR);
        const pos = new THREE.Vector3().addVectors(pL, pR).multiplyScalar(0.5);

        const quat = new THREE.Quaternion();
        gripperFingers.left.getWorldQuaternion(quat);
        const forward = new THREE.Vector3(0, 0, 1).applyQuaternion(quat).normalize();
        const jaw = new THREE.Vector3(1, 0, 0).applyQuaternion(quat).normalize();
        const normal = new THREE.Vector3(0, 1, 0).applyQuaternion(quat).normalize();

        return { position: pos, quaternion: quat, forward, jaw, normal, gripperStroke: 0.02 };
    }

    return null;
}

/**
 * Updates 3D visual lines, waypoints, 6-DOF Triad Gizmos, and Ghost Grippers
 * from predicted future_actions (50 x 16 array).
 * Ensures the trajectory is strictly anchored at the robot's current gripper jaws (t=0).
 */
function updateFutureActionPath(futureActions) {
    if (!futurePathGroup) initFuturePathVisualizer();
    if (!futurePathGroup) return;

    if (!showFuturePath || !futureActions || !Array.isArray(futureActions) || futureActions.length === 0) {
        if (leftPathLine) leftPathLine.geometry.setDrawRange(0, 0);
        if (rightPathLine) rightPathLine.geometry.setDrawRange(0, 0);
        if (leftGoalMarker) leftGoalMarker.visible = false;
        if (rightGoalMarker) rightGoalMarker.visible = false;
        if (leftGhostGripper) leftGhostGripper.visible = false;
        if (rightGhostGripper) rightGhostGripper.visible = false;
        leftTriadPool.forEach(t => t.group.visible = false);
        rightTriadPool.forEach(t => t.group.visible = false);
        return;
    }

    const steps = Math.min(futureActions.length, pathHorizonSteps);

    const leftPositions = leftPathLine.geometry.attributes.position.array;
    const rightPositions = rightPathLine.geometry.attributes.position.array;

    let validLeftSteps = 0;
    let validRightSteps = 0;

    let lastLeftPose = null;
    let lastRightPose = null;

    let leftTriadIdx = 0;
    let rightTriadIdx = 0;

    // Reset all pooled triads
    leftTriadPool.forEach(t => t.group.visible = false);
    rightTriadPool.forEach(t => t.group.visible = false);

    // 1. Anchor step t=0 directly to the current physical gripper pose in 3D scene
    const curLeftGripper = getCurrentGripperWorldPose(true);
    const curRightGripper = getCurrentGripperWorldPose(false);

    if (curLeftGripper && showLeftPath) {
        leftPositions[0] = curLeftGripper.position.x;
        leftPositions[1] = curLeftGripper.position.y;
        leftPositions[2] = curLeftGripper.position.z;
        validLeftSteps = 1;
        lastLeftPose = curLeftGripper;
    }

    if (curRightGripper && showRightPath) {
        rightPositions[0] = curRightGripper.position.x;
        rightPositions[1] = curRightGripper.position.y;
        rightPositions[2] = curRightGripper.position.z;
        validRightSteps = 1;
        lastRightPose = curRightGripper;
    }

    // Pre-calculate step 0 FK baseline for delta anchoring
    let baseLeftFK = null;
    let baseRightFK = null;
    if (futureActions.length > 0 && futureActions[0] && futureActions[0].length >= 16) {
        const baseTCP = computeTCPPositions(futureActions[0]);
        baseLeftFK = baseTCP.left ? baseTCP.left.position.clone() : null;
        baseRightFK = baseTCP.right ? baseTCP.right.position.clone() : null;
    }

    for (let i = 0; i < steps; i++) {
        const action = futureActions[i];
        if (!action || action.length < 16) continue;

        const { left, right } = computeTCPPositions(action);
        const stepNum = i + 1; // 1-indexed (1..50)
        const isDestination = (i === steps - 1);
        const isStrideStep = (stepNum % triadStride === 0);

        // --- Left Arm ---
        if (left && showLeftPath) {
            // Anchor relative to current physical gripper if available
            let renderPos = left.position;
            if (curLeftGripper && baseLeftFK) {
                const delta = left.position.clone().sub(baseLeftFK);
                renderPos = curLeftGripper.position.clone().add(delta);
            }

            leftPositions[validLeftSteps * 3 + 0] = renderPos.x;
            leftPositions[validLeftSteps * 3 + 1] = renderPos.y;
            leftPositions[validLeftSteps * 3 + 2] = renderPos.z;

            const stepPose = {
                position: renderPos,
                forward: left.forward,
                jaw: left.jaw,
                normal: left.normal,
                gripperStroke: left.gripperStroke
            };
            lastLeftPose = stepPose;
            validLeftSteps++;

            // Render 3-axis Triad Gizmo at destination step or intermediate stride
            if (showTriads && leftTriadIdx < MAX_TRIADS_PER_ARM) {
                if (isDestination) {
                    // Full-scale triad at t=50 (1.0x scale, 1.0 opacity)
                    applyTriadGizmo(leftTriadPool[leftTriadIdx++], renderPos, left.forward, left.jaw, left.normal, 1.0, 1.0);
                } else if (isStrideStep) {
                    // Intermediate secondary sub-triad (0.55x scale, 0.65 opacity)
                    applyTriadGizmo(leftTriadPool[leftTriadIdx++], renderPos, left.forward, left.jaw, left.normal, 0.55, 0.65);
                }
            }
        }

        // --- Right Arm ---
        if (right && showRightPath) {
            let renderPos = right.position;
            if (curRightGripper && baseRightFK) {
                const delta = right.position.clone().sub(baseRightFK);
                renderPos = curRightGripper.position.clone().add(delta);
            }

            rightPositions[validRightSteps * 3 + 0] = renderPos.x;
            rightPositions[validRightSteps * 3 + 1] = renderPos.y;
            rightPositions[validRightSteps * 3 + 2] = renderPos.z;

            const stepPose = {
                position: renderPos,
                forward: right.forward,
                jaw: right.jaw,
                normal: right.normal,
                gripperStroke: right.gripperStroke
            };
            lastRightPose = stepPose;
            validRightSteps++;

            if (showTriads && rightTriadIdx < MAX_TRIADS_PER_ARM) {
                if (isDestination) {
                    applyTriadGizmo(rightTriadPool[rightTriadIdx++], renderPos, right.forward, right.jaw, right.normal, 1.0, 1.0);
                } else if (isStrideStep) {
                    applyTriadGizmo(rightTriadPool[rightTriadIdx++], renderPos, right.forward, right.jaw, right.normal, 0.55, 0.65);
                }
            }
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
        if (lastLeftPose && showLeftPath) {
            leftGoalMarker.position.copy(lastLeftPose.position);
            leftGoalMarker.visible = true;
        } else {
            leftGoalMarker.visible = false;
        }
    }
    if (leftGhostGripper) {
        if (lastLeftPose && showLeftPath && showGhostGripper) {
            updateGhostGripperGeometry(
                leftGhostGripper,
                lastLeftPose.position,
                lastLeftPose.forward,
                lastLeftPose.jaw,
                lastLeftPose.normal,
                lastLeftPose.gripperStroke
            );
        } else {
            leftGhostGripper.visible = false;
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
        if (lastRightPose && showRightPath) {
            rightGoalMarker.position.copy(lastRightPose.position);
            rightGoalMarker.visible = true;
        } else {
            rightGoalMarker.visible = false;
        }
    }
    if (rightGhostGripper) {
        if (lastRightPose && showRightPath && showGhostGripper) {
            updateGhostGripperGeometry(
                rightGhostGripper,
                lastRightPose.position,
                lastRightPose.forward,
                lastRightPose.jaw,
                lastRightPose.normal,
                lastRightPose.gripperStroke
            );
        } else {
            rightGhostGripper.visible = false;
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

function setShowTriads(visible) {
    showTriads = Boolean(visible);
    if (leftTriadGroup) leftTriadGroup.visible = showTriads;
    if (rightTriadGroup) rightTriadGroup.visible = showTriads;
}

function setTriadSize(sizeMeters) {
    triadSize = Math.max(0.015, Math.min(0.12, Number(sizeMeters) || 0.05));
}

function setTriadStride(stride) {
    triadStride = Math.max(1, Math.min(50, parseInt(stride, 10) || 10));
}

function setShowGhostGripper(visible) {
    showGhostGripper = Boolean(visible);
    if (leftGhostGripper) leftGhostGripper.visible = showGhostGripper;
    if (rightGhostGripper) rightGhostGripper.visible = showGhostGripper;
}
