"""Embodiment fingerprinting for dynamics calibration checkpoints."""

from __future__ import annotations

from pathlib import Path
from typing import Any
import hashlib

import numpy as np

from .resolver import PayloadSpec, ResolvedRobot, resolve_robot


UNITS = {
    "q": "rad",
    "qd": "rad/s",
    "qdd": "rad/s^2",
    "torque": "Nm",
}


class EmbodimentMismatchError(ValueError):
    """Raised when a checkpoint is loaded for a different robot embodiment."""


def file_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with open(Path(path).expanduser(), "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _payload_metadata(config: dict[str, Any], payload: PayloadSpec | None) -> dict[str, Any]:
    payload_config = config.get("payload") or {}
    out: dict[str, Any] = {
        "enabled": bool(payload_config.get("enabled", False)),
        "profile": str(payload_config.get("profile", "none")),
        "mode": str(payload_config.get("mode", "metadata_only")),
    }
    if payload is None:
        return out
    out.update(
        {
            "name": payload.name,
            "mass_kg": payload.mass_kg,
            "com_xyz_m": np.asarray(payload.com_xyz_m, dtype=np.float64).astype(float).tolist(),
            "inertia_kg_m2": np.asarray(payload.inertia_kg_m2, dtype=np.float64).astype(float).tolist(),
            "attach_link": payload.attach_link,
            "joint_origin_xyz_m": np.asarray(payload.joint_origin_xyz_m, dtype=np.float64).astype(float).tolist(),
            "joint_origin_rpy_rad": np.asarray(payload.joint_origin_rpy_rad, dtype=np.float64).astype(float).tolist(),
            "apply_to_model": bool(payload.apply_to_model),
        }
    )
    return out


def build_embodiment_metadata(config: dict[str, Any], robot: ResolvedRobot | None = None) -> dict[str, Any]:
    """Build the metadata used to pair compensation checkpoints to a robot."""
    if robot is None:
        robot = resolve_robot(
            config["urdf_path"],
            name=str(config.get("robot_name", "robot")),
            payload=config.get("payload"),
        )
    payload = _payload_metadata(config, robot.payload)
    embodiment_id = str(config.get("embodiment_id") or f"{robot.name}_{payload['profile']}_{payload['mode']}")
    return {
        "id": embodiment_id,
        "robot_name": robot.name,
        "joint_count": robot.joint_count,
        "joint_names": robot.joint_names,
        "urdf_sha256": file_sha256(robot.urdf_path),
        "payload": payload,
        "units": dict(UNITS),
    }


def assert_embodiment_matches(checkpoint_embodiment: dict[str, Any] | None, expected: dict[str, Any]) -> None:
    """Validate a checkpoint embodiment against the current config fingerprint."""
    if checkpoint_embodiment is None:
        raise EmbodimentMismatchError("checkpoint missing embodiment metadata")
    keys = ("id", "robot_name", "joint_count", "joint_names", "urdf_sha256", "payload", "units")
    mismatches = [key for key in keys if checkpoint_embodiment.get(key) != expected.get(key)]
    if mismatches:
        joined = ", ".join(mismatches)
        raise EmbodimentMismatchError(f"embodiment mismatch: {joined}")
