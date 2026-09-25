// ==============================================================================
// OPENARM 3D KINEMATIC MODEL BUILDER (THREE.JS)
// Procedural high-definition CAD geometry for Dual 7-DOF Arms & Parallel Grippers
// ==============================================================================

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

    // 4. Build Left Arm and Right Arm - Positioned at exact OpenArm v2.0 Joint 1 Origins (y = +/-0.0625m)
    buildSingleArm(torso, "left",  -0.0625, 0.045, 0, matteBlackMat, motorCasingMat, silverMetalMat, cableMat, labelMat);
    buildSingleArm(torso, "right",  0.0625, 0.045, 0, matteBlackMat, motorCasingMat, silverMetalMat, cableMat, labelMat);
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
    // Joint 2: Shoulder Roll / Abduction (rotates around Z axis)
    // Lateral offset from J1: 0.060m (y: -0.0600 in OpenArm v2.0 URDF)
    // ====================================================
    const j2 = new THREE.Group();
    j2.position.set(sign * 0.060, 0, 0);
    j1.add(j2);
    armJoints[1] = { group: j2, axis: 'z' };

    // J2 Pitch Hinge Hub
    const j2Hub = new THREE.Mesh(new THREE.CylinderGeometry(0.050, 0.050, 0.065, 24), motorMat);
    j2Hub.rotation.z = Math.PI / 2;
    j2Hub.castShadow = true;
    j2.add(j2Hub);

    // Upper Arm Link: Ergonomic curved sculpted black casing (0.220m total from J2 to J4)
    const upperArmLength = 0.220;
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
    // Offset from J2: -0.06625m (z: -0.06625 in OpenArm v2.0 URDF)
    // ====================================================
    const j3 = new THREE.Group();
    j3.position.y = -0.06625;
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
    // Offset from J3: -0.15375m (z: -0.15375 in OpenArm v2.0 URDF)
    // Features authentic DM4340 motor with circular bolt pattern & spec label
    // ====================================================
    const j4 = new THREE.Group();
    j4.position.y = -0.15375;
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

    // Forearm Link: Sleek black link (0.216m total from J4 to J6)
    const forearmLength = 0.216;
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
    // Offset from J4: -0.0955m (z: -0.0955 in OpenArm v2.0 URDF)
    // ====================================================
    const j5 = new THREE.Group();
    j5.position.y = -0.0955;
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
    // Offset from J5: -0.1205m (z: -0.1205 in OpenArm v2.0 URDF)
    // Features signature Machined Silver Clevis / Dual-Prong Fork Bracket
    // ====================================================
    const j6 = new THREE.Group();
    j6.position.y = -0.1205;
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
    // Offset from J6: 0.0m (z: 0.0 in OpenArm v2.0 URDF)
    // Features vertical black DM4310 motor cylinder with silver flanges
    // ====================================================
    const j7 = new THREE.Group();
    j7.position.y = 0.0;
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
