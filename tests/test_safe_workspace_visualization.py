import pathlib
import subprocess
import sys
import unittest

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from dynamics.config import load_config
from record import safe_workspace_visualization as runner


class SafeWorkspaceVisualizationTests(unittest.TestCase):
    def test_motion_plan_stays_inside_joint_envelope_and_returns_home(self):
        args = runner.parse_args([
            "--self-check",
            "--motion-duration-s",
            "2.0",
            "--motion-hz",
            "20",
            "--amplitude-deg",
            "2,3,4,5,6,7",
            "--speed-deg-s",
            "12",
            "--accel-deg-s2",
            "45",
            "--hold-s",
            "0.1",
            "--waypoints",
            "2",
        ])
        config = load_config(robot="xarm6")
        plan = runner.build_motion_plan(args, config)

        q_deg = np.rad2deg(plan.q_traj_rad)
        home = np.asarray(plan.home_joints_deg)
        amp = np.asarray(plan.amplitude_deg)
        self.assertGreater(len(q_deg), 1)
        self.assertTrue(np.all(np.abs(q_deg - home) <= amp + 1e-6))
        np.testing.assert_allclose(q_deg[-1], home, atol=1e-6)

    def test_self_check_runs_without_importing_xarm_runtime(self):
        proc = subprocess.run(
            [
                sys.executable,
                str(ROOT / "record" / "safe_workspace_visualization.py"),
                "--self-check",
                "--motion-duration-s",
                "2.0",
                "--motion-hz",
                "20",
                "--waypoints",
                "2",
                "--hold-s",
                "0.1",
            ],
            cwd=str(ROOT),
            text=True,
            capture_output=True,
            timeout=10,
        )
        self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
        self.assertIn("SELF_CHECK_OK", proc.stdout)
        self.assertIn("EEF Force HUD", proc.stdout)

    def test_shell_launcher_exposes_dashboard_and_stale_process_controls(self):
        content = (ROOT / "record" / "run_safe_workspace_visualization.sh").read_text(encoding="utf-8")

        self.assertIn("safe_workspace_visualization.py", content)
        self.assertIn("TELEOP_TORQUE_COMP_MODEL", content)
        self.assertIn("STOP_STALE_TELEOP", content)
        self.assertIn("--record-dataset", content)
        self.assertIn("live 3D URDF dashboard", content)


if __name__ == "__main__":
    unittest.main()
