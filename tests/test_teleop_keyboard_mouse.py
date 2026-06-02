import pathlib
import sys
import time
import types
import unittest
from unittest import mock

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

try:
    from record import teleop
except ModuleNotFoundError as exc:
    if (exc.name or "").split(".")[0] in {"cv2", "lerobot", "xarm"}:
        raise unittest.SkipTest(f"{exc.name} is not installed")
    raise


class _FakeCap:
    def __init__(self, opened=True):
        self._opened = opened

    def isOpened(self):
        return self._opened

    def release(self):
        pass


class _FakeStream:
    plans = {}

    def __init__(self, source, width=640, height=480):
        plan = self.plans.get(source, {"opened": False})
        self.cap = _FakeCap(plan.get("opened", True))
        self._frame = plan.get("frame")
        self._ts = 0.0
        self._count = 0

    def start(self):
        if self._frame is not None:
            self._count = 1
            self._ts = time.monotonic()

    def get(self):
        return self._frame, self._ts, self._count

    def close(self):
        pass


class TeleopLocalTests(unittest.TestCase):
    def _make_app(self):
        app = teleop.TeleopApp.__new__(teleop.TeleopApp)
        app.linear_vel_mm_s = 60.0
        app.angular_vel_deg_s = 33.75
        app.mouse_vel_gain_deg_s_per_px = 1.35
        app.camera_id = 4
        app.camera_dev = None
        app.camera_probe_timeout_s = 0.01
        app.strict_camera_dev = False
        app.camera_source = None
        app.base_camera_source = None
        app.target_camera_source = None
        app.last_camera_retry_ts = 0.0
        app.camera = None
        app.base_camera = None
        app.target_camera = None
        app.base_camera_id = None
        app.base_camera_dev = None
        app.target_camera_id = None
        app.target_camera_dev = None
        app.display_camera = "wrist"
        app.use_tool_coord = True
        app.video_window = mock.Mock()
        app.video_window_scale = 1.0
        app.request_stop = mock.Mock()
        app.Key = types.SimpleNamespace(
            esc=object(),
            space=object(),
            shift=object(),
            shift_l=object(),
            shift_r=object(),
            enter=object(),
            tab=object(),
        )
        app.keys_down = set()
        app.mouse_dx = 0.0
        app.mouse_dy = 0.0
        app.gripper_dir = 0
        app.pending_gripper_pos = None
        app.end_episode_requested = False
        app.lock = teleop.threading.Lock()
        app.window_name = "Teleop Video+Torques"
        app.window_inited = False
        app.last_window_maintain_ts = 0.0
        app.screen_center = None
        app.current_alignment_ok = True
        app.filtered_curr = np.full((7,), np.nan, dtype=np.float64)
        app.curr_valid = np.zeros((7,), dtype=bool)
        app.med_buf = [teleop.collections.deque(maxlen=5) for _ in range(7)]
        app.alpha_rise = 0.35
        app.alpha_fall = 0.12
        app.velocity_mode_ready = True
        app.velocity_cmd_duration_s = 0.05
        app.velocity_send_lock = teleop.threading.Lock()
        app.last_velocity_cmd = [0.0] * 6
        app.diag_send_cnt = 0
        app.diag_send_fail = 0
        app.diag_send_ms_sum = 0.0
        app.diag_send_ms_max = 0.0
        app.diag_fail_streak = 0
        app.diag_last_recover_ts = 0.0
        return app

    def test_compose_velocity_cmd_uses_current_mapping(self):
        app = self._make_app()
        vel = app.compose_velocity_cmd({"w", "a", "space"}, -20.0, 10.0, 1.0)

        np.testing.assert_allclose(
            vel,
            [60.0, 60.0, 60.0, 0.0, 13.5, 27.0],
            atol=1e-6,
        )

    def test_qe_uses_vrz_slot(self):
        app = self._make_app()

        vel_q = app.compose_velocity_cmd({"q"}, 0.0, 0.0, 1.0)
        vel_e = app.compose_velocity_cmd({"e"}, 0.0, 0.0, 1.0)

        np.testing.assert_allclose(vel_q, [0.0, 0.0, 0.0, 0.0, 0.0, -33.75], atol=1e-6)
        np.testing.assert_allclose(vel_e, [0.0, 0.0, 0.0, 0.0, 0.0, 33.75], atol=1e-6)

    def test_consume_events_does_not_clear_mouse_deltas(self):
        app = self._make_app()
        app.mouse_dx = 12.0
        app.mouse_dy = -8.0
        app.pending_gripper_pos = 300.0
        app.end_episode_requested = True

        gripper_dir, gripper_pos, end = app.consume_events()

        self.assertEqual(gripper_dir, 0)
        self.assertEqual(gripper_pos, 300.0)
        self.assertTrue(end)
        self.assertEqual(app.mouse_dx, 12.0)
        self.assertEqual(app.mouse_dy, -8.0)

    def test_send_velocity_recovers_mode_before_command(self):
        app = self._make_app()
        app.arm = mock.Mock()
        app.arm.mode = 0
        app.arm.state = 1
        app.arm.vc_set_cartesian_velocity.return_value = 0

        def _recover():
            app.velocity_mode_ready = True
            app.arm.mode = 5
            app.arm.state = 1
            return True

        app.enter_realtime_velocity_mode = mock.Mock(side_effect=_recover)

        app.send_velocity([1, 2, 3, 4, 5, 6])

        app.enter_realtime_velocity_mode.assert_called_once()
        app.arm.vc_set_cartesian_velocity.assert_called_once_with(
            [1, 2, 3, 4, 5, 6],
            is_radian=False,
            is_tool_coord=True,
            duration=0.05,
            check_mode=False,
        )

    def test_key_release_sends_immediate_stop_after_last_motion_key(self):
        app = self._make_app()
        app.keys_down = {"w"}
        app.mouse_dx = 12.0
        app.mouse_dy = -8.0
        app.last_velocity_cmd = [60.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        app.stop_motion_now = mock.Mock()

        app.on_key_release(types.SimpleNamespace(char="w"))

        self.assertEqual(app.keys_down, set())
        self.assertEqual(app.mouse_dx, 0.0)
        self.assertEqual(app.mouse_dy, 0.0)
        self.assertEqual(app.last_velocity_cmd, [0.0] * 6)
        app.stop_motion_now.assert_called_once()

    def test_key_release_keeps_motion_when_another_motion_key_is_held(self):
        app = self._make_app()
        app.keys_down = {"w", "a"}
        app.stop_motion_now = mock.Mock()

        app.on_key_release(types.SimpleNamespace(char="w"))

        self.assertEqual(app.keys_down, {"a"})
        app.stop_motion_now.assert_not_called()

    def test_ensure_velocity_runtime_ready_allows_state_two(self):
        app = self._make_app()
        app.arm = mock.Mock()
        app.arm.mode = 5
        app.arm.state = 2
        app.velocity_mode_ready = True
        app.enter_realtime_velocity_mode = mock.Mock(return_value=False)

        self.assertTrue(app.ensure_velocity_runtime_ready())
        app.enter_realtime_velocity_mode.assert_not_called()

    def test_reset_to_home_on_start_waits_for_servo_motion_to_finish(self):
        app = self._make_app()
        app.arm = mock.Mock()
        app.gripper_speed = 2400.0
        app.arm.get_state.side_effect = [(0, 2), (0, 2), (0, 1)]

        app.reset_to_home_on_start()

        app.arm.set_servo_angle.assert_called_once()
        self.assertTrue(app.arm.set_servo_angle.call_args.kwargs["wait"])
        self.assertGreaterEqual(app.arm.get_state.call_count, 1)

    def test_open_camera_with_probe_falls_back_to_next_source(self):
        app = self._make_app()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        _FakeStream.plans = {
            "/dev/video4": {"opened": False},
            "/dev/video6": {"opened": True, "frame": frame},
        }

        with mock.patch.object(teleop, "CameraStream", _FakeStream):
            stream = app.open_camera_with_probe(4, "/dev/video4")

        self.assertIsNotNone(stream)
        self.assertEqual(app.camera_source, "/dev/video6")
        got_frame, _ts, count = stream.get()
        self.assertEqual(count, 1)
        self.assertEqual(got_frame.shape, (480, 640, 3))

    def test_show_video_panel_reopens_stalled_camera(self):
        app = self._make_app()
        stalled = mock.Mock()
        stalled.get.return_value = (None, 0.0, 0)
        stalled.close.return_value = None
        app.camera = stalled

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        replacement = mock.Mock()
        replacement.get.return_value = (frame, time.monotonic(), 1)

        with mock.patch.object(app, "open_camera_with_probe", return_value=replacement) as reopen:
            with mock.patch.object(app, "draw_current_panel"):
                with mock.patch.object(teleop.cv2, "namedWindow"), \
                     mock.patch.object(teleop.cv2, "imshow"), \
                     mock.patch.object(teleop.cv2, "waitKey", return_value=-1), \
                     mock.patch.object(teleop.cv2, "resizeWindow"), \
                     mock.patch.object(teleop.cv2, "moveWindow"), \
                     mock.patch.object(teleop.cv2, "setWindowProperty"):
                    app.show_video_panel()

        stalled.close.assert_called_once()
        reopen.assert_called_once_with(app.camera_id, app.camera_dev)
        self.assertIs(app.camera, replacement)


    def test_capture_episode_observation_records_torque_estimator_metadata(self):
        app = self._make_app()
        app.dashboard_state = None
        app.task_description = "test"
        app.lerobot_dataset = mock.Mock()

        state = {
            "joints_deg": [1, 2, 3, 4, 5, 6],
            "joint_velocities_deg_s": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            "gripper_pos": 123.0,
            "torques": [0, 1, 2, 3, 4, 5, 6],
            "torque_external": [1, 1, 1, 1, 1, 1],
            "torque_external_direct": [2, 2, 2, 2, 2, 2],
            "torque_external_observer": [3, 3, 3, 3, 3, 3],
            "torque_estimator_confidence": 0.25,
            "torque_valid_no_contact": False,
            "torque_stop_event_id": 7,
        }

        app.capture_episode_observation(state)

        frame = app.lerobot_dataset.add_frame.call_args.args[0]
        np.testing.assert_allclose(frame["observation.torque_external_direct"], np.full(6, 2.0))
        np.testing.assert_allclose(frame["observation.torque_external_observer"], np.full(6, 3.0))
        np.testing.assert_allclose(frame["observation.torque_estimator_confidence"], [0.25])
        np.testing.assert_allclose(frame["observation.torque_valid_no_contact"], [0.0])
        np.testing.assert_array_equal(frame["observation.torque_stop_event_id"], np.array([7], dtype=np.int64))

    def test_lerobot_features_include_torque_estimator_metadata(self):
        app = self._make_app()
        features = app._build_lerobot_features()

        self.assertEqual(features["observation.torque_external_direct"]["dtype"], "float32")
        self.assertEqual(features["observation.torque_external_observer"]["shape"], (6,))
        self.assertEqual(features["observation.torque_estimator_confidence"]["shape"], (1,))
        self.assertEqual(features["observation.torque_valid_no_contact"]["dtype"], "float32")
        self.assertEqual(features["observation.torque_stop_event_id"]["dtype"], "int64")


if __name__ == "__main__":
    unittest.main()
