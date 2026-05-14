import pathlib
import unittest

import numpy as np

from dynamics.config import load_config
from dynamics.resolver import resolve_robot, write_payload_augmented_urdf


ROOT = pathlib.Path(__file__).resolve().parents[1]


class ResolverTests(unittest.TestCase):
    def test_resolves_xarm6_urdf_and_payload_metadata(self):
        config = load_config(robot="xarm6")
        robot = resolve_robot(config["urdf_path"], name=config["robot_name"], payload=config["payload"])

        self.assertEqual(robot.name, "xarm6")
        self.assertEqual(robot.joint_count, 6)
        self.assertEqual(robot.joint_names, ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        self.assertIsNotNone(robot.payload)
        self.assertEqual(robot.payload.name, "xarm_gripper_g2")
        self.assertFalse(robot.payload.apply_to_model)
        np.testing.assert_allclose(robot.payload.com_xyz_m, [0.12, -0.04, 0.158])

    def test_can_write_augmented_payload_urdf_when_requested(self):
        config = load_config(robot="xarm6")
        config["payload"]["mode"] = "augment_urdf"
        robot = resolve_robot(config["urdf_path"], name=config["robot_name"], payload=config["payload"])

        path = write_payload_augmented_urdf(robot)
        try:
            text = path.read_text(encoding="utf-8")
            self.assertIn("xarm_gripper_g2_payload", text)
            self.assertIn("xarm_gripper_g2_payload_joint", text)
        finally:
            path.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
