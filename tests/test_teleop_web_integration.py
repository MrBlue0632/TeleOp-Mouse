
import pathlib
import subprocess
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class TeleopWebIntegrationTests(unittest.TestCase):
    def test_root_main_self_check_accepts_dashboard_cli_flags(self):
        cmd = [
            sys.executable,
            str(ROOT / "main.py"),
            "--self-check",
            "--web-dashboard",
            "--web-host",
            "127.0.0.1",
            "--web-port",
            "8765",
            "--web-fps",
            "10",
        ]
        proc = subprocess.run(cmd, cwd=str(ROOT), text=True, capture_output=True, timeout=10)
        self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
        self.assertIn("SELF_CHECK_OK", proc.stdout)

    def test_record_collect_demo_exposes_dashboard_flags(self):
        content = (ROOT / "record" / "collect_demo.sh").read_text(encoding="utf-8")
        self.assertIn("--web-dashboard", content)
        self.assertIn("--web-host", content)
        self.assertIn("--web-port", content)
        self.assertIn("--web-fps", content)
        self.assertIn('cd "${ROOT_DIR}"', content)
        self.assertIn('"${ROOT_DIR}/main.py"', content)

    def test_root_main_delegates_to_record_orchestration(self):
        content = (ROOT / "main.py").read_text(encoding="utf-8")
        self.assertIn("from record.main import main", content)
        self.assertIn('if __name__ == "__main__"', content)

    def test_record_flow_uses_approved_top_level_modules(self):
        teleop = (ROOT / "record" / "teleop.py").read_text(encoding="utf-8")
        self.assertIn("from record.control.torque_estimation import FirmwareBiasTorqueEstimator", teleop)
        self.assertNotIn("dynamics.torque_estimation", teleop)
        self.assertNotIn("robot_control", teleop)
        self.assertFalse((ROOT / "teleop_web").exists())
        self.assertFalse((ROOT / "robot_control").exists())
        self.assertFalse((ROOT / "scripts").exists())
        self.assertFalse((ROOT / "scripts" / "teleop.py").exists())
        self.assertFalse((ROOT / "scripts" / "collect_demo.sh").exists())
        self.assertFalse((ROOT / "dynamics" / "robot_constants.py").exists())
        self.assertFalse((ROOT / "dynamics" / "robot_model.py").exists())
        self.assertFalse((ROOT / "dynamics" / "torque_estimation.py").exists())
        self.assertFalse((ROOT / "dynamics" / "reset.py").exists())
        self.assertFalse((ROOT / "record" / "reset.py").exists())

    def test_tools_folder_owns_standalone_reset_script(self):
        reset_tool = ROOT / "tools" / "reset.py"

        self.assertTrue(reset_tool.is_file())
        content = reset_tool.read_text(encoding="utf-8")
        self.assertIn("from dynamics.backends import create_backend", content)
        self.assertIn("backend.reset_home", content)

    def test_record_control_urdf_path_points_to_assets(self):
        from record.control.constants import XARM6_URDF

        self.assertTrue(pathlib.Path(XARM6_URDF).is_file(), XARM6_URDF)
        self.assertIn("assets/urdf", pathlib.Path(XARM6_URDF).as_posix())


if __name__ == "__main__":
    unittest.main()
