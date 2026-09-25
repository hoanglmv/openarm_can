// ==============================================================================
// OPENARM ROBOT DANCE ROUTINES (CHOREOGRAPHY MATHEMATICAL ENGINE)
// Strictly bounded within OpenArm Mechanical Limits & Safe Accelerations
// ==============================================================================

const DANCE_ROUTINES = {
    disco: {
        name: "Disco Wave (Sóng Disco)",
        defaultBpm: 100,
        desc: "Đánh tay so le nhịp 4/4 sôi động",
        calcPose: function(t, bpm) {
            const omega = (2 * Math.PI * bpm) / 60;
            // J1: Vai Pitch đánh sóng so le lên xuống
            const l_j1 = 0.50 + 0.32 * Math.sin(omega * t);
            const r_j1 = 0.50 - 0.32 * Math.sin(omega * t);
            // J2: Vai Roll nhún nhẹ hai bên
            const l_j2 = -0.30 + 0.12 * Math.cos(omega * t * 0.5);
            const r_j2 = 0.30 + 0.12 * Math.cos(omega * t * 0.5);
            // J3: Bắp xoay theo phách
            const l_j3 = 0.18 * Math.sin(omega * t * 0.5);
            const r_j3 = -0.18 * Math.sin(omega * t * 0.5);
            // J4: Khuỷu gập duỗi so le
            const l_j4 = 1.15 + 0.28 * Math.cos(omega * t);
            const r_j4 = 1.15 - 0.28 * Math.cos(omega * t);
            // J5: Cẳng xoay
            const l_j5 = 0.0;
            const r_j5 = 0.0;
            // J6: Cổ tay vẫy theo nhịp
            const l_j6 = 0.20 * Math.sin(omega * t * 2);
            const r_j6 = -0.20 * Math.sin(omega * t * 2);
            // J7: Cổ xoay lắc dập dìu
            const l_j7 = 0.35 * Math.cos(omega * t);
            const r_j7 = -0.35 * Math.cos(omega * t);
            // J8: Kẹp búng ngón tay theo phách 2 và 4
            const beatPulse = Math.pow(Math.max(0, Math.sin(omega * t)), 4);
            const grip_l = 0.010 + 0.028 * beatPulse;
            const grip_r = 0.010 + 0.028 * beatPulse;

            return [
                l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7, grip_l,
                r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7, grip_r
            ];
        }
    },
    penguin: {
        name: "Penguin Pop (Cánh Cụt)",
        defaultBpm: 110,
        desc: "Đập cánh nhún nhảy popping hóm hỉnh",
        calcPose: function(t, bpm) {
            const omega = (2 * Math.PI * bpm) / 60;
            // Elbows bent 90 degrees
            const l_j4 = 1.50;
            const r_j4 = 1.50;
            // J1: Hướng cánh hơi chúi về phía trước
            const l_j1 = 0.22 + 0.10 * Math.sin(omega * t * 0.5);
            const r_j1 = 0.22 + 0.10 * Math.sin(omega * t * 0.5);
            // J2: Đập cánh (flapping wings out & in)
            const flap = Math.pow(Math.sin(omega * t), 2);
            const l_j2 = -0.30 - 0.35 * flap; // ranges -0.30 to -0.65 rad
            const r_j2 = 0.30 + 0.35 * flap;   // ranges +0.30 to +0.65 rad
            // J3: Giật bắp tay popping robot
            const l_j3 = 0.25 * Math.sin(omega * t * 2);
            const r_j3 = -0.25 * Math.sin(omega * t * 2);
            // J5: Forearm twist
            const l_j5 = 0.0;
            const r_j5 = 0.0;
            // J6: Cổ tay vẫy lên xuống theo cánh
            const l_j6 = 0.30 * Math.sin(omega * t);
            const r_j6 = 0.30 * Math.sin(omega * t);
            // J7: Cổ xoay
            const l_j7 = 0.0;
            const r_j7 = 0.0;
            // Grippers: vỗ kẹp nhịp nhàng
            const grip_l = 0.005 + 0.035 * flap;
            const grip_r = 0.005 + 0.035 * flap;

            return [
                l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7, grip_l,
                r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7, grip_r
            ];
        }
    },
    ocean: {
        name: "Ocean Flow (Biển Sóng)",
        defaultBpm: 80,
        desc: "Sóng lượn dẻo dai từ trái sang phải",
        calcPose: function(t, bpm) {
            const omega = (2 * Math.PI * bpm) / 60;
            // Continuous traveling wave with phase delay from left arm to right arm
            const l_j1 = 0.45 + 0.25 * Math.sin(omega * t);
            const l_j2 = -0.40 + 0.15 * Math.cos(omega * t);
            const l_j3 = 0.10 * Math.sin(omega * t - 0.4);
            const l_j4 = 1.05 + 0.35 * Math.sin(omega * t - 0.6);
            const l_j5 = 0.15 * Math.cos(omega * t - 0.8);
            const l_j6 = 0.30 * Math.sin(omega * t - 1.0);
            const l_j7 = 0.20 * Math.cos(omega * t - 1.2);
            const grip_l = 0.015 + 0.020 * (0.5 + 0.5 * Math.sin(omega * t - 1.2));

            // Right arm follows with continuous phase delay
            const r_phase = omega * t - 2.0;
            const r_j1 = 0.45 + 0.25 * Math.sin(r_phase);
            const r_j2 = 0.40 + 0.15 * Math.cos(r_phase);
            const r_j3 = -0.10 * Math.sin(r_phase - 0.4);
            const r_j4 = 1.05 + 0.35 * Math.sin(r_phase - 0.6);
            const r_j5 = -0.15 * Math.cos(r_phase - 0.8);
            const r_j6 = 0.30 * Math.sin(r_phase - 1.0);
            const r_j7 = -0.20 * Math.cos(r_phase - 1.2);
            const grip_r = 0.015 + 0.020 * (0.5 + 0.5 * Math.sin(r_phase - 1.2));

            return [
                l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7, grip_l,
                r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7, grip_r
            ];
        }
    },
    cheer: {
        name: "Cheer Up (Vũ Điệu Cổ Vũ)",
        defaultBpm: 120,
        desc: "Hai tay chữ V lắc lư reo hò ăn mừng",
        calcPose: function(t, bpm) {
            const omega = (2 * Math.PI * bpm) / 60;
            // High V celebration stance
            const l_j1 = 1.15 + 0.15 * Math.sin(omega * t * 2);
            const r_j1 = 1.15 + 0.15 * Math.sin(omega * t * 2);
            // Swaying together left and right
            const sway = 0.25 * Math.sin(omega * t);
            const l_j2 = -0.55 + sway;
            const r_j2 = 0.55 + sway;
            // Twist
            const l_j3 = 0.0;
            const r_j3 = 0.0;
            // Elbow bounce
            const l_j4 = 0.70 + 0.20 * Math.cos(omega * t * 2);
            const r_j4 = 0.70 + 0.20 * Math.cos(omega * t * 2);
            const l_j5 = 0.0;
            const r_j5 = 0.0;
            // Wrist cheer shaking
            const l_j6 = 0.25 * Math.sin(omega * t * 4);
            const r_j6 = -0.25 * Math.sin(omega * t * 4);
            const l_j7 = 0.40 * Math.cos(omega * t * 2);
            const r_j7 = -0.40 * Math.cos(omega * t * 2);
            // Wide open celebration grippers
            const grip_l = 0.038 + 0.005 * Math.sin(omega * t * 2);
            const grip_r = 0.038 + 0.005 * Math.sin(omega * t * 2);

            return [
                l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7, grip_l,
                r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7, grip_r
            ];
        }
    },
    taichi: {
        name: "Tai Chi Flow (Thái Cực Quyền)",
        defaultBpm: 60,
        desc: "Đẩy chưởng và ôm cầu dưỡng sinh nhu hòa",
        calcPose: function(t, bpm) {
            const omega = (2 * Math.PI * bpm) / 60;
            // Slow majestic circular push-pull
            const l_j1 = 0.55 + 0.25 * Math.sin(omega * t);
            const r_j1 = 0.55 - 0.25 * Math.sin(omega * t);
            const l_j2 = -0.25 - 0.20 * Math.cos(omega * t);
            const r_j2 = 0.25 + 0.20 * Math.cos(omega * t);
            const l_j3 = 0.20 * Math.sin(omega * t);
            const r_j3 = -0.20 * Math.sin(omega * t);
            const l_j4 = 1.25 - 0.35 * Math.sin(omega * t);
            const r_j4 = 1.25 + 0.35 * Math.sin(omega * t);
            const l_j5 = 0.0;
            const r_j5 = 0.0;
            const l_j6 = 0.20 * Math.cos(omega * t);
            const r_j6 = -0.20 * Math.cos(omega * t);
            const l_j7 = 0.45 * Math.sin(omega * t);
            const r_j7 = -0.45 * Math.sin(omega * t);
            // Relaxed hand
            const grip_l = 0.022;
            const grip_r = 0.022;

            return [
                l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7, grip_l,
                r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7, grip_r
            ];
        }
    },
    changquan: {
        name: "Changquan Strike (Trường Quyền)",
        defaultBpm: 75,
        desc: "Đấm xuất quyền, thu thủ thủ thế dứt khoát Wushu",
        calcPose: function(t, bpm) {
            const omega = (2 * Math.PI * bpm) / 60;
            const strikePhase = Math.sin(omega * t);
            // Right arm thrust punch forward, Left arm guard back
            const r_j1 = 0.65 + 0.45 * strikePhase;
            const l_j1 = 0.65 - 0.45 * strikePhase;
            const r_j2 = 0.20 - 0.15 * Math.abs(strikePhase);
            const l_j2 = -0.20 + 0.15 * Math.abs(strikePhase);
            const r_j3 = 0.25 * strikePhase;
            const l_j3 = -0.25 * strikePhase;
            // Elbow extension during punch, bent in chamber/guard
            const r_j4 = 0.85 - 0.60 * strikePhase;
            const l_j4 = 0.85 + 0.60 * strikePhase;
            const r_j5 = 0.15 * strikePhase;
            const l_j5 = -0.15 * strikePhase;
            const r_j6 = 0.10 * Math.cos(omega * t);
            const l_j6 = -0.10 * Math.cos(omega * t);
            const r_j7 = 0.30 * strikePhase;
            const l_j7 = -0.30 * strikePhase;
            // Solid martial fist (closed gripper)
            const grip_l = 0.005;
            const grip_r = 0.005;

            return [
                l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7, grip_l,
                r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7, grip_r
            ];
        }
    },
    sword: {
        name: "Cloud Sword (Kiếm Pháp Wushu)",
        defaultBpm: 65,
        desc: "Múa kiếm vạch mây, song thủ biến ảo uyển chuyển",
        calcPose: function(t, bpm) {
            const omega = (2 * Math.PI * bpm) / 60;
            // Figure-8 sword waving pattern
            const l_j1 = 0.50 + 0.35 * Math.sin(omega * t);
            const r_j1 = 0.70 + 0.40 * Math.cos(omega * t);
            const l_j2 = -0.40 + 0.25 * Math.cos(omega * t);
            const r_j2 = 0.45 + 0.25 * Math.sin(omega * t);
            const l_j3 = 0.30 * Math.sin(omega * t * 0.5);
            const r_j3 = -0.30 * Math.sin(omega * t * 0.5);
            const l_j4 = 1.10 + 0.40 * Math.cos(omega * t);
            const r_j4 = 0.90 + 0.35 * Math.sin(omega * t);
            const l_j5 = 0.20 * Math.sin(omega * t);
            const r_j5 = -0.20 * Math.sin(omega * t);
            const l_j6 = 0.25 * Math.cos(omega * t * 2);
            const r_j6 = 0.30 * Math.sin(omega * t * 2);
            const l_j7 = 0.50 * Math.sin(omega * t);
            const r_j7 = -0.50 * Math.sin(omega * t);
            // Holding sword hilt / sword finger
            const grip_l = 0.015;
            const grip_r = 0.012;

            return [
                l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7, grip_l,
                r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7, grip_r
            ];
        }
    },
    staff: {
        name: "Shaolin Staff (Thiếu Lâm Côn Pháp)",
        defaultBpm: 80,
        desc: "Loan hoa côn, quét ngang đỡ gạt dũng mãnh",
        calcPose: function(t, bpm) {
            const omega = (2 * Math.PI * bpm) / 60;
            // Coordinated two-hand staff twirl
            const twirl = Math.sin(omega * t);
            const l_j1 = 0.60 + 0.30 * twirl;
            const r_j1 = 0.60 - 0.30 * twirl;
            const l_j2 = -0.35 + 0.15 * Math.cos(omega * t);
            const r_j2 = 0.35 + 0.15 * Math.cos(omega * t);
            const l_j3 = 0.15 * Math.cos(omega * t);
            const r_j3 = -0.15 * Math.cos(omega * t);
            const l_j4 = 1.30 - 0.40 * Math.abs(twirl);
            const r_j4 = 1.30 - 0.40 * Math.abs(twirl);
            const l_j5 = 0.10 * twirl;
            const r_j5 = -0.10 * twirl;
            const l_j6 = 0.20 * Math.sin(omega * t * 2);
            const r_j6 = -0.20 * Math.sin(omega * t * 2);
            const l_j7 = 0.35 * Math.cos(omega * t);
            const r_j7 = -0.35 * Math.cos(omega * t);
            // Firm staff grip
            const grip_l = 0.010;
            const grip_r = 0.010;

            return [
                l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7, grip_l,
                r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7, grip_r
            ];
        }
    }
};
