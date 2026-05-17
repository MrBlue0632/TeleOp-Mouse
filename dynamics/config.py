"""Configuration loading for dynamics command-line tools."""

from __future__ import annotations

from pathlib import Path
from typing import Any
import copy
import os

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT_CONFIG_PATH = Path(__file__).resolve().parent / "config.yaml"
DEFAULT_CONFIG_DIR = Path(__file__).resolve().parent / "config"


def deep_update(base: dict[str, Any], updates: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_update(result[key], value)
        elif value is not None:
            result[key] = value
    return result


def default_config_path(robot: str) -> Path:
    path = DEFAULT_CONFIG_DIR / f"{robot}.yaml"
    if not path.exists():
        raise FileNotFoundError(f"no default config for robot '{robot}' at {path}")
    return path


def _read_yaml(path: Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_config(path: str | os.PathLike[str] | None = None, *, robot: str = "xarm6", overrides: dict[str, Any] | None = None) -> dict[str, Any]:
    config: dict[str, Any] = {}
    defaults_path = DEFAULT_INPUT_CONFIG_PATH
    if defaults_path.exists():
        config = deep_update(config, _read_yaml(defaults_path))

    config_path = Path(path).expanduser() if path else default_config_path(robot)
    config = deep_update(config, _read_yaml(config_path))
    config = _resolve_paths(config, config_path.parent)
    if overrides:
        config = deep_update(config, overrides)
        config = _resolve_paths(config, config_path.parent)
    config["_config_path"] = str(config_path.resolve())
    if defaults_path.exists():
        config["_defaults_path"] = str(defaults_path.resolve())
    return config


def _resolve_paths(config: dict[str, Any], base_dir: Path) -> dict[str, Any]:
    result = copy.deepcopy(config)
    for key in ("urdf_path",):
        value = result.get(key)
        if isinstance(value, str):
            path = Path(value).expanduser()
            if not path.is_absolute():
                path = (REPO_ROOT / path).resolve()
            result[key] = str(path)
    for key, value in list(result.items()):
        if isinstance(value, dict):
            result[key] = _resolve_paths(value, base_dir)
    return result


def build_cli_overrides(args: Any) -> dict[str, Any]:
    connection: dict[str, Any] = {}
    if getattr(args, "ip", None):
        connection["ip"] = args.ip
    if getattr(args, "robot_port", None) is not None:
        connection["robot_port"] = args.robot_port
    if getattr(args, "report_port_normal", None) is not None:
        connection["report_port_normal"] = args.report_port_normal
    if getattr(args, "report_port_rich", None) is not None:
        connection["report_port_rich"] = args.report_port_rich
    if getattr(args, "report_port_real", None) is not None:
        connection["report_port_real"] = args.report_port_real

    overrides: dict[str, Any] = {}
    if connection:
        overrides["connection"] = connection
    if getattr(args, "urdf", None):
        overrides["urdf_path"] = args.urdf
    if getattr(args, "hz", None) is not None:
        overrides["sampling_hz"] = args.hz
    return overrides
