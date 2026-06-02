"""Thread-safe telemetry state for the TeleOp web dashboard."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path
from threading import Lock
from typing import Any, Callable
import json
import math
import time


CameraRole = str


def _clean_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(out) or math.isinf(out):
        return default
    return out


def _clean_list(values: Any, size: int | None = None, default: float = 0.0) -> list[float]:
    if values is None:
        values = []
    try:
        result = [_clean_float(v, default) for v in list(values)]
    except TypeError:
        result = []
    if size is not None:
        if len(result) < size:
            result.extend([default] * (size - len(result)))
        result = result[:size]
    return result


def _finite_vector(values: Any, size: int) -> list[float] | None:
    if values is None:
        return None
    try:
        raw = list(values)
    except TypeError:
        return None
    if len(raw) < size:
        return None
    out: list[float] = []
    for value in raw[:size]:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number):
            return None
        out.append(number)
    return out


def _rpy_deg_to_rotmat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> list[list[float]]:
    roll = math.radians(float(roll_deg))
    pitch = math.radians(float(pitch_deg))
    yaw = math.radians(float(yaw_deg))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _rotmat_transpose_vec(rot: list[list[float]], vec: list[float]) -> list[float]:
    return [
        rot[0][0] * vec[0] + rot[1][0] * vec[1] + rot[2][0] * vec[2],
        rot[0][1] * vec[0] + rot[1][1] * vec[1] + rot[2][1] * vec[2],
        rot[0][2] * vec[0] + rot[1][2] * vec[1] + rot[2][2] * vec[2],
    ]


def transform_wrench_base_to_eef(wrench_base: Any, pose_xyzrpy_deg: Any) -> tuple[list[float], bool]:
    """Rotate a BASE-frame wrench into EEF coordinates using pose XYZ/RPY degrees."""
    wrench = _finite_vector(wrench_base, 6)
    pose = _finite_vector(pose_xyzrpy_deg, 6)
    if wrench is None or pose is None:
        return [0.0] * 6, False

    rot_base_eef = _rpy_deg_to_rotmat(pose[3], pose[4], pose[5])
    force_eef = _rotmat_transpose_vec(rot_base_eef, wrench[:3])
    torque_eef = _rotmat_transpose_vec(rot_base_eef, wrench[3:6])
    wrench_eef = force_eef + torque_eef
    if not all(math.isfinite(value) for value in wrench_eef):
        return [0.0] * 6, False
    return wrench_eef, True


def torque_bar(value: Any, *, limit_nm: float = 10.0) -> dict[str, Any]:
    """Return UI-ready absolute torque bar data for an external torque value."""
    magnitude = abs(_clean_float(value))
    clamped = min(magnitude, float(limit_nm))
    if magnitude < 2.0:
        level = "low"
        color = "#d7a735"
    elif magnitude < 5.0:
        level = "medium"
        color = "#f08a24"
    else:
        level = "high"
        color = "#ff5738"
    return {
        "value": magnitude,
        "clamped": clamped,
        "limit": float(limit_nm),
        "ratio": 0.0 if limit_nm <= 0 else clamped / float(limit_nm),
        "level": level,
        "color": color,
    }


def _dataset_snapshot(dataset: Any) -> dict[str, Any]:
    if dataset is None:
        return {
            "root": None,
            "num_episodes": 0,
            "total_frames": 0,
            "episode_index": None,
            "episode_buffer_size": 0,
        }
    episode_buffer = getattr(dataset, "episode_buffer", None) or {}
    meta = getattr(dataset, "meta", None)
    return {
        "root": str(getattr(dataset, "root", "")),
        "num_episodes": int(getattr(dataset, "num_episodes", 0) or 0),
        "total_frames": int(getattr(meta, "total_frames", 0) or 0),
        "episode_index": episode_buffer.get("episode_index"),
        "episode_buffer_size": int(episode_buffer.get("size", 0) or 0),
    }


def _camera_status(camera: Any) -> dict[str, Any]:
    if camera is None:
        return {"status": "missing", "age_ms": None, "frames": 0}
    try:
        frame, ts, count = camera.get()
    except Exception as exc:  # pragma: no cover - defensive against SDK/camera objects
        return {"status": "error", "error": str(exc), "age_ms": None, "frames": 0}
    now = time.monotonic()
    age_ms = ((now - ts) * 1000.0) if ts else None
    if frame is None or not count:
        status = "empty"
    elif age_ms is not None and age_ms > 1200.0:
        status = "stale"
    else:
        status = "ok"
    shape = list(getattr(frame, "shape", []) or [])
    return {"status": status, "age_ms": age_ms, "frames": int(count or 0), "shape": shape}


def _torque_mode(app: Any) -> tuple[str, str]:
    explicit_mode = getattr(app, "dashboard_torque_mode", None)
    explicit_note = getattr(app, "dashboard_torque_note", None)
    if explicit_mode:
        return str(explicit_mode), str(explicit_note or "")
    estimator = getattr(app, "torque_estimator", None)
    if estimator is None:
        return "unavailable", "torque estimator is not initialized"
    if getattr(estimator, "hybrid", None) is not None:
        return "compensation_model", "loaded compensation checkpoint"
    error = getattr(estimator, "hybrid_error", None)
    if error:
        return "firmware_bias_fallback", f"compensation load failed: {error}"
    return "firmware_bias_fallback", "no compensation checkpoint; using model plus firmware-bias fallback"


def build_app_snapshot(app: Any) -> dict[str, Any]:
    """Build a JSON-safe telemetry snapshot from a TeleopApp-like object."""
    state = dict(getattr(app, "last_state", {}) or {})
    torques = _clean_list(state.get("torques"), 7)
    tau_external = _clean_list(state.get("torque_external"), 6)
    tau_model = _clean_list(state.get("torque_model"), 6)
    tau_static = _clean_list(state.get("torque_static_bias"), 6)
    tau_motion = _clean_list(state.get("torque_motion_comp"), 6)
    tau_firmware = _clean_list(state.get("torque_firmware_bias"), 6)
    ee_force = _clean_list(state.get("ee_force"), 6)
    pose_xyzrpy_deg = _clean_list(state.get("pose_xyzrpy_deg"), 6)
    ee_force_base = _clean_list(state.get("ee_force_base", ee_force), 6)
    transformed_force_eef, transform_ok = transform_wrench_base_to_eef(ee_force_base, state.get("pose_xyzrpy_deg"))
    if state.get("ee_force_eef") is None:
        ee_force_eef = transformed_force_eef
    else:
        ee_force_eef = _clean_list(state.get("ee_force_eef"), 6)
        transform_ok = bool(state.get("ee_force_transform_ok", True))
    ee_force_display = _clean_list(state.get("ee_force_display", ee_force_eef), 6)
    ee_force_display_frame = "EEF"
    force_mag = math.sqrt(sum(v * v for v in ee_force_display[:3]))
    torque_mode, torque_note = _torque_mode(app)
    saving_evt = getattr(app, "saving_evt", None)
    try:
        saving = bool(saving_evt.is_set()) if saving_evt is not None else False
    except Exception:
        saving = False

    diagnostics = dict(getattr(app, "last_diag", {}) or {})
    diagnostics["ee_force_transform_ok"] = bool(transform_ok)
    diagnostics["ee_force_display_frame"] = ee_force_display_frame
    transform_note = state.get("ee_force_transform_note")
    if transform_note:
        diagnostics["ee_force_transform_note"] = str(transform_note)
    elif not transform_ok:
        diagnostics["ee_force_transform_note"] = "pose unavailable or non-finite; display force is zeroed"

    return {
        "schema_version": 1,
        "server_time": time.time(),
        "robot": {
            "ip": str(getattr(app, "robot_ip", "")),
            "running": bool(getattr(app, "running", False)),
            "coord_frame": "TOOL" if bool(getattr(app, "use_tool_coord", False)) else "BASE",
            "state": getattr(app, "last_robot_state", None),
            "error": getattr(app, "last_robot_err", None),
            "saving_episode": saving,
            "reset_enabled": bool(getattr(app, "running", False)) and not saving,
        },
        "dataset": {
            **_dataset_snapshot(getattr(app, "lerobot_dataset", None)),
            "repo_id": str(getattr(app, "repo_id", "")),
            "task": str(getattr(app, "task_description", "")),
            "rate_hz": _clean_float(getattr(app, "rate_hz", 0.0)),
        },
        "camera": {
            "display": str(getattr(app, "display_camera", "wrist")),
            "wrist": _camera_status(getattr(app, "camera", None)),
            "base": _camera_status(getattr(app, "base_camera", None)),
            "target": _camera_status(getattr(app, "target_camera", None)),
        },
        "state": {
            "timestamp": _clean_float(state.get("ts"), time.time()),
            "joints_deg": _clean_list(state.get("joints_deg"), 6),
            "pose_xyzrpy_deg": pose_xyzrpy_deg,
            "gripper_pos": _clean_float(state.get("gripper_pos", getattr(app, "gripper_pos", 0.0))),
            "ee_force": ee_force,
            "ee_force_base": ee_force_base,
            "ee_force_eef": ee_force_eef,
            "ee_force_display": ee_force_display,
            "ee_force_display_frame": ee_force_display_frame,
            "ee_force_magnitude": force_mag,
        },
        "action": {
            "velocity_cmd": _clean_list(state.get("action_velocity_cmd", getattr(app, "last_velocity_cmd", [])), 6),
            "gripper": _clean_float(state.get("action_gripper", getattr(app, "gripper_pos", 0.0))),
        },
        "torque": {
            "api": torques,
            "model": tau_model,
            "external": tau_external,
            "external_bars": [torque_bar(v) for v in tau_external],
            "static_bias": tau_static,
            "motion_comp": tau_motion,
            "firmware_bias": tau_firmware,
            "bias_lambda": _clean_float(state.get("torque_lambda")),
            "external_mode": torque_mode,
            "external_note": torque_note,
        },
        "diagnostics": diagnostics,
        "config_status": {
            "restart_required_fields": ["connection", "urdf_path", "payload"],
            "message": "Connection, URDF, and payload edits apply on next dashboard/teleop start.",
        },
    }


@dataclass
class DashboardState:
    """Shared state between TeleopApp and the HTTP dashboard."""

    reset_callback: Callable[[], None] | None = None
    saving_provider: Callable[[], bool] | None = None
    web_fps: float = 10.0
    _snapshot: dict[str, Any] = field(default_factory=dict)
    _jpeg: dict[CameraRole, bytes] = field(default_factory=dict)
    _jpeg_ts: dict[CameraRole, float] = field(default_factory=dict)
    _lock: Lock = field(default_factory=Lock)

    def publish_snapshot(self, snapshot: dict[str, Any]) -> None:
        with self._lock:
            self._snapshot = deepcopy(snapshot)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return deepcopy(self._snapshot)

    def publish_frame(self, role: CameraRole, frame: Any, *, rgb: bool = True) -> None:
        now = time.monotonic()
        min_dt = 0.0 if self.web_fps <= 0 else 1.0 / float(self.web_fps)
        with self._lock:
            if now - self._jpeg_ts.get(role, 0.0) < min_dt:
                return
        jpeg = _encode_jpeg(frame, rgb=rgb)
        if jpeg is None:
            return
        with self._lock:
            self._jpeg[role] = jpeg
            self._jpeg_ts[role] = now

    def get_jpeg(self, role: CameraRole) -> bytes | None:
        with self._lock:
            return self._jpeg.get(role)

    def reset_home(self) -> tuple[int, dict[str, Any]]:
        if self.reset_callback is None:
            return 409, {"ok": False, "error": "reset is unavailable outside a running teleop session"}
        if self.saving_provider is not None and self.saving_provider():
            return 409, {"ok": False, "error": "cannot reset while an episode is being saved"}
        try:
            self.reset_callback()
        except Exception as exc:  # pragma: no cover - defensive around robot SDK
            return 500, {"ok": False, "error": str(exc)}
        return 202, {"ok": True, "message": "reset requested"}


def _encode_jpeg(frame: Any, *, rgb: bool) -> bytes | None:
    if frame is None:
        return None
    try:
        import cv2
    except Exception:
        return None
    try:
        image = frame
        if rgb:
            image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ok, data = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
        if not ok:
            return None
        return data.tobytes()
    except Exception:
        return None


def json_dumps(data: Any) -> bytes:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


REPO_ROOT = Path(__file__).resolve().parents[1]
