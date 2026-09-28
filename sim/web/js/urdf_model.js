// ==============================================================================
// OPENARM V1.0 OFFICIAL URDF MODEL
// Loads the Apache-2.0 openarm_description URDF and visual meshes in-browser.
// ==============================================================================

const OPENARM_V1_URDF =
    "assets/openarm_v1/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf";
const OPENARM_V1_PACKAGE_ROOT = "assets/openarm_v1/openarm_description";

let openArmUrdfRobot = null;
let openArmModelMode = "loading";
let openArmModelVisibleRequested = false;
const pendingUrdfJointValues = new Map();

function setOpenArmModelVisibility(visible) {
    openArmModelVisibleRequested = Boolean(visible);
    if (openArmUrdfRobot) openArmUrdfRobot.visible = openArmModelVisibleRequested;
}

const motorToUrdfJoint = {
    1: "openarm_left_joint1",
    2: "openarm_left_joint2",
    3: "openarm_left_joint3",
    4: "openarm_left_joint4",
    5: "openarm_left_joint5",
    6: "openarm_left_joint6",
    7: "openarm_left_joint7",
    8: "openarm_left_finger_joint1",
    9: "openarm_right_joint1",
    10: "openarm_right_joint2",
    11: "openarm_right_joint3",
    12: "openarm_right_joint4",
    13: "openarm_right_joint5",
    14: "openarm_right_joint6",
    15: "openarm_right_joint7",
    16: "openarm_right_finger_joint1",
};

function createStableCadMaterial(source) {
    const color = source && source.color && source.color.isColor
        ? source.color.clone()
        : new THREE.Color(0xcccccc);
    const opacity = source && Number.isFinite(source.opacity) ? source.opacity : 1.0;

    return new THREE.MeshBasicMaterial({
        name: source ? source.name : "openarm-cad-material",
        color,
        map: source ? source.map : null,
        alphaMap: source ? source.alphaMap : null,
        transparent: opacity < 1.0,
        opacity,
        alphaTest: source ? source.alphaTest : 0,
        side: THREE.DoubleSide,
        depthTest: true,
        depthWrite: opacity >= 1.0,
        vertexColors: Boolean(source && source.vertexColors),
        toneMapped: false,
    });
}

function stabilizeOpenArmMeshAppearance(mesh) {
    if (!mesh || !mesh.isMesh) return;
    const sourceMaterials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
    const stableMaterials = sourceMaterials.map(createStableCadMaterial);
    mesh.material = Array.isArray(mesh.material) ? stableMaterials : stableMaterials[0];

    // Mirrored DAE meshes use a negative scale. Rendering both faces prevents a
    // link from disappearing when its winding becomes reversed during rotation.
    mesh.frustumCulled = false;
    mesh.castShadow = true;
    mesh.receiveShadow = false;
}

function fallbackToProceduralModel(error) {
    console.error("[3D] Could not load the official OpenArm v1 URDF:", error);
    openArmModelMode = "procedural";
    if (typeof buildProceduralBimanualOpenArm === "function") {
        buildProceduralBimanualOpenArm();
    }
}

function buildBimanualOpenArm() {
    if (typeof URDFLoader !== "function") {
        fallbackToProceduralModel(new Error("URDFLoader is unavailable"));
        return;
    }

    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loader.packages = {
        openarm_description: OPENARM_V1_PACKAGE_ROOT,
    };
    loader.parseVisual = true;
    loader.parseCollision = false;

    manager.onLoad = () => {
        if (!openArmUrdfRobot) return;
        openArmUrdfRobot.traverse(object => {
            if (object.isMesh) {
                stabilizeOpenArmMeshAppearance(object);
            }
        });
        console.info("[3D] Official OpenArm v1 URDF and visual meshes loaded");
    };

    loader.load(
        OPENARM_V1_URDF,
        robot => {
            openArmUrdfRobot = robot;
            openArmModelMode = "urdf";

            // Feedback may temporarily be outside nominal URDF limits when torque
            // is off and gravity moves the arm. Render measured angles exactly;
            // command limits remain enforced by the UI and backend.
            Object.values(motorToUrdfJoint).forEach(name => {
                if (robot.joints[name]) robot.joints[name].ignoreLimits = true;
            });

            // ROS URDF is Z-up; the dashboard Three.js scene is Y-up.
            robot.rotation.x = -Math.PI / 2;
            robot.position.set(0, 0, 0);
            robot.visible = openArmModelVisibleRequested;
            scene.add(robot);

            pendingUrdfJointValues.forEach((value, name) => {
                robot.setJointValue(name, value);
            });
            pendingUrdfJointValues.clear();
        },
        undefined,
        fallbackToProceduralModel,
    );
}

function updateOpenArmUrdfJoint(motor) {
    if (!motor || !(motor.id in motorToUrdfJoint)) return false;
    if (openArmModelMode === "procedural") return false;

    const name = motorToUrdfJoint[motor.id];
    let value = Number(motor.q);
    if (!Number.isFinite(value)) return true;

    // The backend normalizes the mirrored physical left shoulder direction.
    // OpenArm v1 URDF retains the original left-joint coordinate convention.
    if (motor.id === 1) value = -value;

    // Gripper state is already expressed as a 0..0.043 metre stroke.
    if (motor.id === 8 || motor.id === 16) {
        value = Math.max(0.0, Math.min(0.044, Math.abs(value)));
    }

    if (openArmUrdfRobot) {
        openArmUrdfRobot.setJointValue(name, value);
    } else {
        pendingUrdfJointValues.set(name, value);
    }
    return true;
}

// Update whichever 3D model is active. This is shared by live telemetry and by
// local command preview, so dragging a slider remains responsive even before the
// backend sends the next telemetry packet.
function updateOpenArmJointVisual(motor) {
    if (!motor || !Number.isFinite(Number(motor.id)) || !Number.isFinite(Number(motor.q))) {
        return false;
    }

    if (updateOpenArmUrdfJoint(motor)) return true;

    const id = Number(motor.id);
    const angle = Number(motor.q);
    const isLeft = id <= 8;
    const jointIndex = (isLeft ? id : id - 8) - 1;
    const armJoints = isLeft ? leftArmJoints : rightArmJoints;
    const gripperFingers = isLeft ? leftGripperFingers : rightGripperFingers;

    if (jointIndex >= 0 && jointIndex < 7) {
        const jEntry = armJoints[jointIndex];
        if (!jEntry || !jEntry.group) return false;

        if (jointIndex === 0) jEntry.group.rotation.x = -angle;
        else if (jointIndex === 1) jEntry.group.rotation.z = angle;
        else if (jointIndex === 2) jEntry.group.rotation.y = isLeft ? angle : -angle;
        else if (jointIndex === 3) jEntry.group.rotation.x = -angle;
        else if (jointIndex === 4) jEntry.group.rotation.y = isLeft ? angle : -angle;
        else if (jointIndex === 5) jEntry.group.rotation.x = -angle;
        else if (jointIndex === 6) jEntry.group.rotation.y = isLeft ? angle : -angle;
        return true;
    }

    if (jointIndex === 7 && gripperFingers.left && gripperFingers.right) {
        const strokeRatio = Math.max(0.0, Math.min(1.0, Math.abs(angle) / 0.043));
        const fingerOffset = 0.010 + strokeRatio * 0.028;
        gripperFingers.left.position.x = -fingerOffset;
        gripperFingers.right.position.x = fingerOffset;
        return true;
    }

    return false;
}
