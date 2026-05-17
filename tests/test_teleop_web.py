
import tempfile
import time
import unittest
from urllib.request import urlopen
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from teleop_web.model import inspect_robot_model
from teleop_web.server import ConfigStore, DashboardServer, _mock_state
from teleop_web.telemetry import DashboardState, build_app_snapshot, torque_bar


class FakeCamera:
    def __init__(self, frame=None, ts=None, count=0):
        self.frame = frame
        self.ts = time.monotonic() if ts is None else ts
        self.count = count

    def get(self):
        return self.frame, self.ts, self.count


class TeleopWebTelemetryTests(unittest.TestCase):
    def test_torque_bar_uses_absolute_value_clamp_and_thresholds(self):
        self.assertEqual(torque_bar(-1.5)["level"], "low")
        self.assertAlmostEqual(torque_bar(-1.5)["value"], 1.5)
        self.assertEqual(torque_bar(2.0)["level"], "medium")
        self.assertEqual(torque_bar(5.0)["level"], "high")
        self.assertAlmostEqual(torque_bar(12.0)["ratio"], 1.0)

    def test_build_app_snapshot_handles_empty_dataset_stale_camera_and_fallback(self):
        stale_ts = time.monotonic() - 4.0
        app = SimpleNamespace(
            running=True,
            robot_ip="192.168.1.199",
            use_tool_coord=True,
            display_camera="wrist",
            task_description="keyboard_mouse_teleop",
            last_robot_state=1,
            last_robot_err=0,
            last_velocity_cmd=[1, 2, 3, 4, 5, 6],
            gripper_pos=321.0,
            last_state={
                "ts": 123.0,
                "joints_deg": [1, 2, 3, 4, 5, 6],
                "pose_xyzrpy_deg": [10, 20, 30, 40, 50, 60],
                "torques": [0, -1, 2, -3, 4, -5, 6],
                "torque_external": [0.5, -2.5, 5.5, -11.0, 1.0, -1.0],
                "torque_model": [0, 0, 0, 0, 0, 0],
                "torque_motion_comp": [0, 0, 0, 0, 0, 0],
                "torque_static_bias": [0, 0, 0, 0, 0, 0],
                "torque_firmware_bias": [0, 0, 0, 0, 0, 0],
                "torque_lambda": 0.25,
                "ee_force": [1, 2, 2, 0, 0, 0],
            },
            lerobot_dataset=SimpleNamespace(root="/tmp/data", num_episodes=2, meta=SimpleNamespace(total_frames=42), episode_buffer=None),
            camera=FakeCamera(np.zeros((4, 4, 3), dtype=np.uint8), stale_ts, 1),
            base_camera=None,
            target_camera=FakeCamera(None, 0.0, 0),
            saving_evt=SimpleNamespace(is_set=lambda: False),
            dashboard_torque_mode="firmware_bias_fallback",
            dashboard_torque_note="no checkpoint",
        )

        snapshot = build_app_snapshot(app)

        self.assertEqual(snapshot["robot"]["ip"], "192.168.1.199")
        self.assertEqual(snapshot["dataset"]["episode_buffer_size"], 0)
        self.assertEqual(snapshot["camera"]["wrist"]["status"], "stale")
        self.assertEqual(snapshot["camera"]["base"]["status"], "missing")
        self.assertEqual(snapshot["torque"]["external_mode"], "firmware_bias_fallback")
        self.assertEqual(snapshot["torque"]["external_bars"][2]["level"], "high")
        self.assertEqual(snapshot["action"]["velocity_cmd"], [1, 2, 3, 4, 5, 6])

    def test_mock_state_includes_external_torque_bars(self):
        snapshot = _mock_state().snapshot()
        self.assertEqual(len(snapshot["torque"]["external_bars"]), 6)
        self.assertEqual(snapshot["torque"]["external_bars"][3]["level"], "high")

    def test_dashboard_state_reset_rejects_without_callback_and_during_save(self):
        state = DashboardState()
        code, payload = state.reset_home()
        self.assertEqual(code, 409)
        self.assertFalse(payload["ok"])

        called = []
        state = DashboardState(reset_callback=lambda: called.append(True), saving_provider=lambda: True)
        code, payload = state.reset_home()
        self.assertEqual(code, 409)
        self.assertEqual(called, [])

        state = DashboardState(reset_callback=lambda: called.append(True), saving_provider=lambda: False)
        code, payload = state.reset_home()
        self.assertEqual(code, 202)
        self.assertTrue(payload["ok"])
        self.assertEqual(called, [True])


class TeleopWebConfigTests(unittest.TestCase):
    def test_config_store_reads_validates_writes_and_backs_up(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "dynamics" / "config").mkdir(parents=True)
            global_path = root / "dynamics" / "config.yaml"
            robot_path = root / "dynamics" / "config" / "xarm6.yaml"
            global_path.write_text("robot_name: xarm6\nconnection:\n  ip: 127.0.0.1\n", encoding="utf-8")
            robot_path.write_text("robot_name: xarm6\njoint_count: 6\n", encoding="utf-8")
            store = ConfigStore(root)

            read = store.read("global")
            self.assertIn("robot_name: xarm6", read["text"])
            self.assertTrue(store.validate_text("robot_name: xarm6\n")["ok"])
            self.assertFalse(store.validate_text("robot_name: [\n")["ok"])

            written = store.write("robot", "robot_name: xarm6\njoint_count: 6\nhome_joints_deg: [0, 0, 0, 0, 0, 0]\n")
            self.assertTrue(written["ok"])
            self.assertIn("home_joints_deg", robot_path.read_text(encoding="utf-8"))
            backups = list(robot_path.parent.glob("xarm6.yaml.*.bak"))
            self.assertEqual(len(backups), 1)


class TeleopWebRobotModelTests(unittest.TestCase):
    def test_inspects_current_urdf_and_reports_mesh_fallback(self):
        repo_root = Path(__file__).resolve().parents[1]

        info = inspect_robot_model(repo_root)

        self.assertTrue(info["ok"])
        self.assertTrue(info["urdf_path"].endswith("assets/urdf/xarm6/xarm6/xarm6.urdf"))
        self.assertGreaterEqual(info["joint_count"], 6)
        self.assertEqual(info["mesh_count"], 0)
        self.assertEqual(info["render_mode"], "primitive")
        self.assertIn("cylinder", info["geometry_types"])

    def test_server_serves_robot_model_contract(self):
        repo_root = Path(__file__).resolve().parents[1]
        server = DashboardServer(_mock_state(), host="127.0.0.1", port=0, repo_root=repo_root)
        port = server.httpd.server_address[1]
        server.start()
        try:
            with urlopen(f"http://127.0.0.1:{port}/api/robot-model", timeout=3) as response:
                body = response.read().decode("utf-8")
        finally:
            server.stop()

        self.assertIn('"ok":true', body)
        self.assertIn('"render_mode":"primitive"', body)


class TeleopWebStaticLayoutTests(unittest.TestCase):
    def test_static_layout_prioritizes_vision_and_compacts_robot_view(self):
        repo_root = Path(__file__).resolve().parents[1]
        html = (repo_root / "teleop_web" / "static" / "index.html").read_text(encoding="utf-8")
        css = (repo_root / "teleop_web" / "static" / "styles.css").read_text(encoding="utf-8")

        self.assertIn('class="panel vision-panel primary-vision"', html)
        self.assertIn('class="panel robot-panel compact-robot"', html)
        self.assertIn('"vision vision robot"', css)
        self.assertIn('"vision vision dataset"', css)
        self.assertIn('"torque config hotkeys"', css)
        self.assertIn(".primary-vision .camera-frame", css)
        self.assertIn(".compact-robot .robot-canvas-wrap", css)

    def test_static_theme_uses_solid_black_gold_without_gradients(self):
        repo_root = Path(__file__).resolve().parents[1]
        css = (repo_root / "teleop_web" / "static" / "styles.css").read_text(encoding="utf-8")

        self.assertNotIn("gradient(", css)
        self.assertIn("--bg: #050403;", css)
        self.assertIn("--panel: #11100d;", css)
        self.assertIn("--gold: #d7a735;", css)


if __name__ == "__main__":
    unittest.main()
