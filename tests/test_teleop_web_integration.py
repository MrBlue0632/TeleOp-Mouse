
import pathlib
import subprocess
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class TeleopWebIntegrationTests(unittest.TestCase):
    def test_teleop_self_check_accepts_dashboard_cli_flags(self):
        cmd = [
            sys.executable,
            str(ROOT / "scripts" / "teleop.py"),
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
        missing_runtime = "ModuleNotFoundError" in proc.stderr and any(name in proc.stderr for name in ("cv2", "lerobot", "xarm"))
        if missing_runtime:
            self.skipTest(proc.stderr.strip().splitlines()[-1])
        self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
        self.assertIn("SELF_CHECK_OK", proc.stdout)

    def test_collect_demo_exposes_dashboard_flags(self):
        content = (ROOT / "scripts" / "collect_demo.sh").read_text(encoding="utf-8")
        self.assertIn("--web-dashboard", content)
        self.assertIn("--web-host", content)
        self.assertIn("--web-port", content)
        self.assertIn("--web-fps", content)


if __name__ == "__main__":
    unittest.main()
