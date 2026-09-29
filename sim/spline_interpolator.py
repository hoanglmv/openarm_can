#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2026 VinRobotics / OpenArm Robotics
"""
Cubic / Quintic Spline Interpolator for OpenArm Trajectory Smoothing.
Interpolates 50Hz (20ms) discrete joint targets from Model AI / Teleop
into continuous, jerk-free 400Hz (2.5ms) motor position and velocity commands
complying with the 7-layer architecture specified in CONTEXT.md.
"""

import math
from typing import Dict, List, Optional, Tuple, Union
import numpy as np


class QuinticHermiteSpline:
    """
    Quintic polynomial segment q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    Satisfies boundary conditions:
      q(0)  = q0,   q(T)  = q1
      q'(0) = v0,   q'(T) = v1
      q''(0)= a0_acc, q''(T)= a1_acc
    Guarantees C^2 continuity (continuous acceleration, smooth jerk).
    """

    def __init__(
        self,
        q0: float,
        v0: float,
        a0_acc: float,
        q1: float,
        v1: float,
        a1_acc: float,
        duration: float,
    ):
        self.T = max(1e-4, float(duration))
        self.q0 = float(q0)
        self.v0 = float(v0)
        self.a0_acc = float(a0_acc)
        self.q1 = float(q1)
        self.v1 = float(v1)
        self.a1_acc = float(a1_acc)

        T = self.T
        T2 = T * T
        T3 = T2 * T
        T4 = T3 * T
        T5 = T4 * T

        self.c0 = self.q0
        self.c1 = self.v0
        self.c2 = 0.5 * self.a0_acc

        delta_q = self.q1 - self.q0
        self.c3 = (
            20.0 * delta_q
            - (8.0 * self.v1 + 12.0 * self.v0) * T
            - (3.0 * self.a0_acc - self.a1_acc) * T2
        ) / (2.0 * T3)

        self.c4 = (
            30.0 * (-delta_q)
            + (14.0 * self.v1 + 16.0 * self.v0) * T
            + (3.0 * self.a0_acc - 2.0 * self.a1_acc) * T2
        ) / (2.0 * T4)

        self.c5 = (
            12.0 * delta_q
            - 6.0 * (self.v1 + self.v0) * T
            - (self.a1_acc - self.a0_acc) * T2
        ) / (2.0 * T5)

    def evaluate(self, t: float) -> Tuple[float, float, float]:
        """
        Evaluate (position, velocity, acceleration) at time t in [0, T].
        Clamped beyond bounds.
        """
        t = min(max(t, 0.0), self.T)
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t

        pos = (
            self.c0
            + self.c1 * t
            + self.c2 * t2
            + self.c3 * t3
            + self.c4 * t4
            + self.c5 * t5
        )
        vel = (
            self.c1
            + 2.0 * self.c2 * t
            + 3.0 * self.c3 * t2
            + 4.0 * self.c4 * t3
            + 5.0 * self.c5 * t4
        )
        acc = (
            2.0 * self.c2
            + 6.0 * self.c3 * t
            + 12.0 * self.c4 * t2
            + 20.0 * self.c5 * t3
        )
        return pos, vel, acc


class SingleJointSplineInterpolator:
    """
    Online Quintic Spline Interpolator for a single joint.
    Receives target updates at 50Hz (every 20ms) and generates 400Hz steps (every 2.5ms).
    """

    def __init__(
        self,
        v_max: float = 2.0,
        a_max: float = 6.0,
        initial_q: float = 0.0,
    ):
        self.v_max = abs(float(v_max))
        self.a_max = abs(float(a_max))

        self.current_q = float(initial_q)
        self.current_v = 0.0
        self.current_a = 0.0

        self.target_q = float(initial_q)
        self.spline: Optional[QuinticHermiteSpline] = None
        self.segment_time: float = 0.0
        self.segment_duration: float = 0.02  # default 50Hz (20ms)

    def set_state(self, q: float, v: float = 0.0, a: float = 0.0):
        """Force internal state (e.g. on physical sync or enable)."""
        self.current_q = float(q)
        self.current_v = float(v)
        self.current_a = float(a)
        self.target_q = float(q)
        self.spline = None
        self.segment_time = 0.0

    def update_target(self, target_q: float, dt_target: float = 0.02):
        """
        Called when a new target arrives (typically at 50Hz from AI / Teleop).
        Plans a smooth Quintic Spline from current state to the target.
        """
        self.target_q = float(target_q)
        duration = max(0.005, float(dt_target))

        # Check if duration needs extension based on kinematic limits
        diff = abs(self.target_q - self.current_q)
        min_time_vel = diff / max(self.v_max, 1e-3)
        min_time_acc = math.sqrt(2.0 * diff / max(self.a_max, 1e-3))
        planned_duration = max(duration, min_time_vel, min_time_acc)

        # Plan next segment from current position, velocity, and acceleration
        self.spline = QuinticHermiteSpline(
            q0=self.current_q,
            v0=self.current_v,
            a0_acc=self.current_a,
            q1=self.target_q,
            v1=0.0,
            a1_acc=0.0,
            duration=planned_duration,
        )
        self.segment_time = 0.0
        self.segment_duration = planned_duration

    def step(self, dt: float = 0.0025) -> Tuple[float, float]:
        """
        Called at 400Hz (dt = 0.0025s) to sample the interpolated position and velocity.
        Returns (q_cmd, v_cmd).
        """
        if self.spline is None:
            diff = self.target_q - self.current_q
            if abs(diff) < 1e-6:
                self.current_q = self.target_q
                self.current_v = 0.0
                self.current_a = 0.0
                return self.current_q, 0.0
            self.update_target(self.target_q, dt_target=0.02)

        self.segment_time += dt
        pos, vel, acc = self.spline.evaluate(self.segment_time)

        # Clamping to limits for safety
        if abs(vel) > self.v_max:
            vel = math.copysign(self.v_max, vel)
        if abs(acc) > self.a_max:
            acc = math.copysign(self.a_max, acc)

        self.current_q = pos
        self.current_v = vel
        self.current_a = acc

        # If reached end of spline segment, hold target
        if self.segment_time >= self.segment_duration:
            self.current_q = self.target_q
            self.current_v = 0.0
            self.current_a = 0.0

        return self.current_q, self.current_v


class BimanualSplineInterpolator:
    """
    16-DOF Spline Interpolator for OpenArm Bimanual robot.
    Applies per-joint kinematic limits (Shoulder, Elbow, Wrist, Gripper)
    and interpolates 50Hz action chunks into 400Hz CAN commands.
    """

    # Kinematic limits per joint type according to CONTEXT.md Section 6.3:
    # J1, J2 (Shoulder DM8009): vmax = 1.2 rad/s, amax = 3.5 rad/s^2
    # J3, J4 (Elbow DM4340):    vmax = 2.0 rad/s, amax = 6.0 rad/s^2
    # J5..J7 (Wrist DM4310):    vmax = 3.0 rad/s, amax = 10.0 rad/s^2
    # J8 (Gripper DM4310):      vmax = 2.5 rad/s, amax = 10.0 rad/s^2
    KINEMATIC_LIMITS = {
        1: (1.2, 3.5),   # J1 Base Yaw
        2: (1.2, 3.5),   # J2 Shoulder Pitch
        3: (2.0, 6.0),   # J3 Elbow Roll
        4: (2.0, 6.0),   # J4 Elbow Pitch
        5: (3.0, 10.0),  # J5 Wrist Roll
        6: (3.0, 10.0),  # J6 Wrist Pitch
        7: (3.0, 10.0),  # J7 Wrist Yaw
        8: (2.5, 10.0),  # J8 Gripper
    }

    def __init__(self, initial_positions: Optional[Dict[int, float]] = None):
        self.interpolators: Dict[int, SingleJointSplineInterpolator] = {}
        for motor_id in range(1, 17):
            joint_idx = ((motor_id - 1) % 8) + 1
            v_max, a_max = self.KINEMATIC_LIMITS.get(joint_idx, (2.0, 6.0))
            init_q = initial_positions.get(motor_id, 0.0) if initial_positions else 0.0
            self.interpolators[motor_id] = SingleJointSplineInterpolator(
                v_max=v_max, a_max=a_max, initial_q=init_q
            )

    def set_joint_state(self, motor_id: int, q: float, v: float = 0.0):
        if motor_id in self.interpolators:
            self.interpolators[motor_id].set_state(q, v)

    def update_joint_target(self, motor_id: int, target_q: float, dt_target: float = 0.02):
        if motor_id in self.interpolators:
            self.interpolators[motor_id].update_target(target_q, dt_target)

    def step(self, motor_id: int, dt: float = 0.0025) -> Tuple[float, float]:
        if motor_id in self.interpolators:
            return self.interpolators[motor_id].step(dt)
        return 0.0, 0.0
