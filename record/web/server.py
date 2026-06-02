"""Standard-library HTTP server for the record dashboard.

Inputs: a ``DashboardState`` plus repository root and HTTP bind settings.
Returns: a background HTTP server exposing static UI, SSE telemetry, MJPEG, config,
reset, and robot-model endpoints.
"""

from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Thread
from urllib.parse import parse_qs, unquote, urlparse
import argparse
import json
import mimetypes
import os
import shutil
import tempfile
import time

import yaml

from .model import inspect_robot_model
from .telemetry import DashboardState, json_dumps, torque_bar


REPO_ROOT = Path(__file__).resolve().parents[2]
STATIC_ROOT = Path(__file__).resolve().parent / "static"
CONFIG_FILES = {
    "global": Path("dynamics/config.yaml"),
    "robot": Path("dynamics/config/xarm6.yaml"),
}


class ConfigStore:
    def __init__(self, repo_root: str | os.PathLike[str] = REPO_ROOT):
        self.repo_root = Path(repo_root).resolve()

    def _path_for(self, key: str) -> Path:
        rel = CONFIG_FILES.get(key)
        if rel is None:
            raise ValueError(f"unknown config file: {key}")
        path = (self.repo_root / rel).resolve()
        if self.repo_root not in path.parents and path != self.repo_root:
            raise ValueError("config path escapes repository root")
        return path

    def read(self, key: str) -> dict[str, object]:
        path = self._path_for(key)
        return {
            "ok": True,
            "file": key,
            "path": str(path),
            "text": path.read_text(encoding="utf-8"),
            "restart_required": True,
        }

    def validate_text(self, text: str) -> dict[str, object]:
        try:
            loaded = yaml.safe_load(text) if text.strip() else {}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
        if loaded is not None and not isinstance(loaded, dict):
            return {"ok": False, "error": "top-level YAML value must be a mapping"}
        return {"ok": True, "parsed_type": type(loaded or {}).__name__}

    def write(self, key: str, text: str) -> dict[str, object]:
        validation = self.validate_text(text)
        if not validation.get("ok"):
            return validation
        path = self._path_for(key)
        path.parent.mkdir(parents=True, exist_ok=True)
        backup = None
        if path.exists():
            stamp = time.strftime("%Y%m%d_%H%M%S")
            backup = path.with_name(f"{path.name}.{stamp}.bak")
            shutil.copy2(path, backup)
        fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                f.write(text)
                if text and not text.endswith("\n"):
                    f.write("\n")
            os.replace(tmp_name, path)
        finally:
            if os.path.exists(tmp_name):
                os.unlink(tmp_name)
        return {
            "ok": True,
            "file": key,
            "path": str(path),
            "backup": str(backup) if backup else None,
            "restart_required": True,
        }


class DashboardServer:
    def __init__(
        self,
        state: DashboardState,
        *,
        host: str = "127.0.0.1",
        port: int = 8765,
        repo_root: str | os.PathLike[str] = REPO_ROOT,
        static_root: str | os.PathLike[str] = STATIC_ROOT,
    ):
        self.state = state
        self.host = host
        self.port = int(port)
        self.repo_root = Path(repo_root).resolve()
        self.static_root = Path(static_root).resolve()
        self.config_store = ConfigStore(self.repo_root)
        handler_cls = self._make_handler()
        self.httpd = ThreadingHTTPServer((self.host, self.port), handler_cls)
        self.thread: Thread | None = None

    def _make_handler(self):
        server_state = self.state
        config_store = self.config_store
        repo_root = self.repo_root
        static_root = self.static_root

        class Handler(BaseHTTPRequestHandler):
            server_version = "TeleOpDashboard/1.0"

            def log_message(self, fmt, *args):  # noqa: D401 - BaseHTTPRequestHandler hook
                print(f"[WEB] {self.address_string()} - {fmt % args}")

            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == "/":
                    return self._serve_file(static_root / "index.html")
                if parsed.path.startswith("/static/"):
                    return self._serve_static(parsed.path.removeprefix("/static/"), static_root)
                if parsed.path.startswith("/assets/"):
                    return self._serve_static(parsed.path.removeprefix("/assets/"), repo_root / "assets")
                if parsed.path == "/api/telemetry":
                    return self._serve_sse(server_state)
                if parsed.path == "/api/robot-model":
                    return self._json(HTTPStatus.OK, inspect_robot_model(repo_root))
                if parsed.path.startswith("/stream/") and parsed.path.endswith(".mjpg"):
                    role = parsed.path.removeprefix("/stream/").removesuffix(".mjpg")
                    return self._serve_mjpeg(server_state, role)
                if parsed.path == "/api/config":
                    key = parse_qs(parsed.query).get("file", ["global"])[0]
                    try:
                        return self._json(HTTPStatus.OK, config_store.read(key))
                    except Exception as exc:
                        return self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
                return self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})

            def do_POST(self):
                parsed = urlparse(self.path)
                if parsed.path == "/api/config/validate":
                    payload = self._read_json()
                    return self._json(HTTPStatus.OK, config_store.validate_text(str(payload.get("text", ""))))
                if parsed.path == "/api/reset-home":
                    code, payload = server_state.reset_home()
                    return self._json(HTTPStatus(code), payload)
                return self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})

            def do_PUT(self):
                parsed = urlparse(self.path)
                if parsed.path == "/api/config":
                    key = parse_qs(parsed.query).get("file", ["global"])[0]
                    payload = self._read_json()
                    try:
                        result = config_store.write(key, str(payload.get("text", "")))
                    except Exception as exc:
                        result = {"ok": False, "error": str(exc)}
                    status = HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST
                    return self._json(status, result)
                return self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})

            def _read_json(self) -> dict:
                length = int(self.headers.get("Content-Length", "0") or 0)
                if length <= 0:
                    return {}
                raw = self.rfile.read(length).decode("utf-8")
                try:
                    return json.loads(raw)
                except json.JSONDecodeError:
                    return {}

            def _json(self, status: HTTPStatus, payload: dict):
                body = json_dumps(payload)
                self.send_response(int(status))
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_file(self, path: Path):
                if not path.exists() or not path.is_file():
                    return self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "file not found"})
                body = path.read_bytes()
                ctype = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_static(self, rel_url: str, root: Path):
                rel = unquote(rel_url).lstrip("/")
                path = (root / rel).resolve()
                if root not in path.parents and path != root:
                    return self._json(HTTPStatus.FORBIDDEN, {"ok": False, "error": "forbidden"})
                return self._serve_file(path)

            def _serve_sse(self, state: DashboardState):
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                while True:
                    payload = json_dumps(state.snapshot())
                    try:
                        self.wfile.write(b"event: telemetry\n")
                        self.wfile.write(b"data: " + payload + b"\n\n")
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        break
                    time.sleep(0.2)

            def _serve_mjpeg(self, state: DashboardState, role: str):
                if role not in {"wrist", "base", "target"}:
                    return self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "unknown stream"})
                boundary = "teleopframe"
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                while True:
                    frame = state.get_jpeg(role)
                    if frame is None:
                        time.sleep(0.2)
                        continue
                    try:
                        self.wfile.write(f"--{boundary}\r\n".encode("ascii"))
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        break
                    time.sleep(0.1)

        return Handler

    def start(self) -> None:
        if self.thread is not None:
            return
        self.thread = Thread(target=self.httpd.serve_forever, name="teleop-web-dashboard", daemon=True)
        self.thread.start()
        print(f"[WEB] dashboard listening on http://{self.host}:{self.port}/")

    def stop(self) -> None:
        self.httpd.shutdown()
        self.httpd.server_close()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None


def start_dashboard_server(
    state: DashboardState,
    *,
    host: str = "127.0.0.1",
    port: int = 8765,
    repo_root: str | os.PathLike[str] = REPO_ROOT,
) -> DashboardServer:
    server = DashboardServer(state, host=host, port=port, repo_root=repo_root)
    server.start()
    return server


def _mock_state() -> DashboardState:
    state = DashboardState()
    state.publish_snapshot(
        {
            "schema_version": 1,
            "server_time": time.time(),
            "robot": {"ip": "mock", "running": True, "coord_frame": "BASE", "state": 1, "error": 0, "saving_episode": False, "reset_enabled": False},
            "dataset": {"root": "data/lerobot_dataset", "repo_id": "teleop/xarm_demo", "task": "mock", "num_episodes": 3, "total_frames": 980, "episode_buffer_size": 42, "episode_index": 3, "rate_hz": 30},
            "camera": {"display": "wrist", "wrist": {"status": "mock"}, "base": {"status": "mock"}, "target": {"status": "mock"}},
            "state": {"joints_deg": [14, -8, -25, 197, 62, -9], "pose_xyzrpy_deg": [320, 12, 180, 179, 1, -8], "gripper_pos": 620, "ee_force": [1, 2, 2, 0, 0, 0], "ee_force_base": [1, 2, 2, 0, 0, 0], "ee_force_eef": [1, 2, 2, 0, 0, 0], "ee_force_display": [1, 2, 2, 0, 0, 0], "ee_force_display_frame": "EEF", "ee_force_magnitude": 3},
            "action": {"velocity_cmd": [0, 0, 0, 0, 0, 0], "gripper": 620},
            "torque": {"api": [0.6, -1.2, 2.8, 5.2, -7.0, 0.3, 0], "model": [0, 0, 0, 0, 0, 0], "external": [0.6, -1.2, 2.8, 5.2, -7.0, 0.3], "external_bars": [torque_bar(v) for v in [0.6, -1.2, 2.8, 5.2, -7.0, 0.3]], "static_bias": [0, 0, 0, 0, 0, 0], "motion_comp": [0, 0, 0, 0, 0, 0], "firmware_bias": [0, 0, 0, 0, 0, 0], "bias_lambda": 0.1, "external_mode": "mock", "external_note": "mock dashboard"},
            "diagnostics": {"ctrl_hz": 120.0, "main_hz": 30.0, "video_hz": 10.0, "ee_force_transform_ok": True, "ee_force_display_frame": "EEF"},
            "config_status": {"restart_required_fields": ["connection", "urdf_path", "payload"], "message": "mock"},
        }
    )
    return state


def main() -> int:
    parser = argparse.ArgumentParser(description="TeleOp web dashboard server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    state = _mock_state() if args.mock else DashboardState()
    server = start_dashboard_server(state, host=args.host, port=args.port)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
