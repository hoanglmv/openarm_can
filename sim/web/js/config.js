// ==============================================================================
// OPENARM CONFIGURATION & SHARED CONSTANTS
// Supports Dual 7-DOF Robotic Arms (Left & Right) + Dual Grippers (16 CAN Nodes)
// ==============================================================================

const LEFT_JOINTS = [
    { id: 1, name: "L-Joint 1 (Shoulder Pitch)", arm: "left",  idx: 0, type: "DM8009", min: -1.3963, max: 3.4907, default: 0.0 },
    { id: 2, name: "L-Joint 2 (Shoulder Roll)",  arm: "left",  idx: 1, type: "DM8009", min: -3.3161, max: 0.1745, default: 0.0 },
    { id: 3, name: "L-Joint 3 (Arm Twist)",      arm: "left",  idx: 2, type: "DM4340", min: -1.5708, max: 1.5708, default: 0.0 },
    { id: 4, name: "L-Joint 4 (Elbow Pitch)",    arm: "left",  idx: 3, type: "DM4340", min:  0.0000, max: 2.4435, default: 0.0 },
    { id: 5, name: "L-Joint 5 (Forearm Twist)",  arm: "left",  idx: 4, type: "DM4310", min: -1.5708, max: 1.5708, default: 0.0 },
    { id: 6, name: "L-Joint 6 (Wrist Pitch)",    arm: "left",  idx: 5, type: "DM4310", min: -0.7854, max: 0.7854, default: 0.0 },
    { id: 7, name: "L-Joint 7 (Wrist Roll)",     arm: "left",  idx: 6, type: "DM4310", min: -1.5708, max: 1.5708, default: 0.0 },
    { id: 8, name: "L-Gripper (J8 Kẹp Ngang)",   arm: "left",  idx: 7, type: "DM4310", min:  0.0000, max: 0.0430, default: 0.0 }
];

const RIGHT_JOINTS = [
    { id: 9,  name: "R-Joint 1 (Shoulder Pitch)", arm: "right", idx: 0, type: "DM8009", min: -1.3963, max: 3.4907, default: 0.0 },
    { id: 10, name: "R-Joint 2 (Shoulder Roll)",  arm: "right", idx: 1, type: "DM8009", min: -0.1745, max: 3.3161, default: 0.0 },
    { id: 11, name: "R-Joint 3 (Arm Twist)",      arm: "right", idx: 2, type: "DM4340", min: -1.5708, max: 1.5708, default: 0.0 },
    { id: 12, name: "R-Joint 4 (Elbow Pitch)",    arm: "right", idx: 3, type: "DM4340", min:  0.0000, max: 2.4435, default: 0.0 },
    { id: 13, name: "R-Joint 5 (Forearm Twist)",  arm: "right", idx: 4, type: "DM4310", min: -1.5708, max: 1.5708, default: 0.0 },
    { id: 14, name: "R-Joint 6 (Wrist Pitch)",    arm: "right", idx: 5, type: "DM4310", min: -0.7854, max: 0.7854, default: 0.0 },
    { id: 15, name: "R-Joint 7 (Wrist Roll)",     arm: "right", idx: 6, type: "DM4310", min: -1.5708, max: 1.5708, default: 0.0 },
    { id: 16, name: "R-Gripper (J8 Kẹp Ngang)",   arm: "right", idx: 7, type: "DM4310", min:  0.0000, max: 0.0430, default: 0.0 }
];

const ALL_JOINTS = [...LEFT_JOINTS, ...RIGHT_JOINTS];

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

const MECHANICAL_LIMITS = {
    1:  { name: "L-J1 Vai Pitch", minRad: -1.3963, maxRad: 3.4907,  minDeg: -80.0,  maxDeg: 200.0,  isGripper: false },
    2:  { name: "L-J2 Vai Roll",  minRad: -3.3161, maxRad: 0.1745,  minDeg: -190.0, maxDeg: 10.0,   isGripper: false },
    3:  { name: "L-J3 Bắp Xoay",  minRad: -1.5708, maxRad: 1.5708,  minDeg: -90.0,  maxDeg: 90.0,   isGripper: false },
    4:  { name: "L-J4 Khuỷu Pitch",minRad: 0.0000, maxRad: 2.4435,  minDeg: 0.0,    maxDeg: 140.0,  isGripper: false },
    5:  { name: "L-J5 Cẳng Xoay", minRad: -1.5708, maxRad: 1.5708,  minDeg: -90.0,  maxDeg: 90.0,   isGripper: false },
    6:  { name: "L-J6 Cổ Pitch",  minRad: -0.7854, maxRad: 0.7854,  minDeg: -45.0,  maxDeg: 45.0,   isGripper: false },
    7:  { name: "L-J7 Cổ Xoay",   minRad: -1.5708, maxRad: 1.5708,  minDeg: -90.0,  maxDeg: 90.0,   isGripper: false },
    8:  { name: "L-J8 Kẹp Ngang", minRad: 0.0,     maxRad: 0.043,   minMm: 0.0,     maxMm: 43.0,    isGripper: true },

    9:  { name: "R-J1 Vai Pitch", minRad: -1.3963, maxRad: 3.4907,  minDeg: -80.0,  maxDeg: 200.0,  isGripper: false },
    10: { name: "R-J2 Vai Roll",  minRad: -0.1745, maxRad: 3.3161,  minDeg: -10.0,  maxDeg: 190.0,  isGripper: false },
    11: { name: "R-J3 Bắp Xoay",  minRad: -1.5708, maxRad: 1.5708,  minDeg: -90.0,  maxDeg: 90.0,   isGripper: false },
    12: { name: "R-J4 Khuỷu Pitch",minRad: 0.0000, maxRad: 2.4435,  minDeg: 0.0,    maxDeg: 140.0,  isGripper: false },
    13: { name: "R-J5 Cẳng Xoay", minRad: -1.5708, maxRad: 1.5708,  minDeg: -90.0,  maxDeg: 90.0,   isGripper: false },
    14: { name: "R-J6 Cổ Pitch",  minRad: -0.7854, maxRad: 0.7854,  minDeg: -45.0,  maxDeg: 45.0,   isGripper: false },
    15: { name: "R-J7 Cổ Xoay",   minRad: -1.5708, maxRad: 1.5708,  minDeg: -90.0,  maxDeg: 90.0,   isGripper: false },
    16: { name: "R-J8 Kẹp Ngang", minRad: 0.0,     maxRad: 0.043,   minMm: 0.0,     maxMm: 43.0,    isGripper: true }
};

const JOINT_NAME_TO_ID = {
    "openarm_left_joint1": 1,
    "openarm_left_joint2": 2,
    "openarm_left_joint3": 3,
    "openarm_left_joint4": 4,
    "openarm_left_joint5": 5,
    "openarm_left_joint6": 6,
    "openarm_left_joint7": 7,
    "openarm_left_finger_joint": 8,
    "left_gripper": 8,
    "openarm_right_joint1": 9,
    "openarm_right_joint2": 10,
    "openarm_right_joint3": 11,
    "openarm_right_joint4": 12,
    "openarm_right_joint5": 13,
    "openarm_right_joint6": 14,
    "openarm_right_joint7": 15,
    "openarm_right_finger_joint": 16,
    "right_gripper": 16
};

// Global App State
let ws = null;
let currentTelemetry = [];
let isLightTheme = true;
let currentArmTab = 'left';  // 'left' | 'right' | 'both' | 'sync'
let syncGrippers = true;
let telemFilter = 'all';
let trafficFilter = 'all';

let jointLockStates = {};
for (let i = 1; i <= 16; i++) {
    jointLockStates[i] = true;
}

// Utility formatting
function formatGripperText(val) {
    const mm = (val * 1000).toFixed(1);
    const status = val < 0.002 ? 'Đóng' : (val >= 0.040 ? 'Mở tối đa' : `${mm} mm`);
    return `${mm} mm (${status})`;
}
