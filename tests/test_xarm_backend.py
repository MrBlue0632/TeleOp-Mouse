import unittest

import numpy as np

from dynamics.backends.xarm import XArmBackend


class FakeArm:
    def __init__(self):
        self.modes = []
        self.states = []
        self.servo_angle_calls = []
        self.servo_angle_j_calls = []

    def set_mode(self, mode):
        self.modes.append(mode)
        return 0

    def set_state(self, state):
        self.states.append(state)
        return 0

    def set_servo_angle(self, **kwargs):
        self.servo_angle_calls.append(kwargs)
        return 0

    def set_servo_angle_j(self, **kwargs):
        self.servo_angle_j_calls.append(kwargs)
        return 0


class XArmBackendReplayTests(unittest.TestCase):
    def test_timestamped_replay_uses_servo_motion_mode(self):
        arm = FakeArm()
        backend = XArmBackend(ip="127.0.0.1", joint_count=2)
        backend.arm = arm

        backend.replay_joint_positions(
            np.array([[0.1, 0.2], [0.2, 0.3]], dtype=np.float64),
            timestamps=np.array([0.0, 0.0], dtype=np.float64),
            speed_deg_s=15.0,
            acc_deg_s2=100.0,
        )

        self.assertEqual(arm.modes[0], 1)
        self.assertEqual(arm.states[0], 0)
        self.assertEqual(len(arm.servo_angle_j_calls), 2)
        self.assertEqual(arm.servo_angle_calls, [])
        self.assertTrue(arm.servo_angle_j_calls[0]["is_radian"])
        self.assertEqual(arm.servo_angle_j_calls[0]["mvtime"], 0)

    def test_untimed_replay_keeps_position_mode_compatibility(self):
        arm = FakeArm()
        backend = XArmBackend(ip="127.0.0.1", joint_count=2)
        backend.arm = arm

        backend.replay_joint_positions(np.array([[np.pi / 2, 0.0]], dtype=np.float64))

        self.assertEqual(arm.modes[0], 0)
        self.assertEqual(len(arm.servo_angle_calls), 1)
        self.assertEqual(arm.servo_angle_j_calls, [])
        self.assertAlmostEqual(arm.servo_angle_calls[0]["angle"][0], 90.0)


if __name__ == "__main__":
    unittest.main()
