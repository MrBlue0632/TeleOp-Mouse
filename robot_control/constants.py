"""Shared constants for xArm6 control."""

import os

import numpy as np

# Home position from TeleOp-Mouse project
HOME_JOINTS_DEG = [14.1, -8.0, -24.7, 196.9, 62.3, -8.8]
GRIPPER_OPEN = 840.0
N_JOINTS = 6

# URDF path
_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
XARM6_URDF = os.path.join(_PKG_DIR, os.pardir, "assets", "urdf", "xarm6", "xarm6.urdf")

# PyBullet revolute joint indices (joint0=fixed world, 1-6=revolute, 7=fixed eef)
PB_JOINT_INDICES = [1, 2, 3, 4, 5, 6]

# Per-joint dead-zone (Nm) — filters sensor noise and static friction
TORQUE_DEAD_ZONE = np.array([1.5, 1.5, 1.0, 0.8, 0.5, 0.3])

# Per-joint damping scaling (relative to base damping value)
DAMPING_SCALE = np.array([2.0, 2.5, 1.8, 1.0, 0.8, 0.5])

# Per-joint stiffness scaling (relative to base stiffness value)
STIFFNESS_SCALE = np.array([1.5, 2.0, 1.5, 1.0, 0.6, 0.3])

# Joint limits (degrees)
JOINT_LIMITS_LOW = np.array([-360, -118, -225, -360, -97, -360], dtype=np.float64)
JOINT_LIMITS_HIGH = np.array([360, 120, 11, 360, 180, 360], dtype=np.float64)
LIMIT_BUFFER_DEG = 10.0
