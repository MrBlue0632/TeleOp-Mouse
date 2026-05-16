"""Real-time torque monitor mode."""

from __future__ import annotations

from pathlib import Path
import sys
import time

import numpy as np

from dynamics.backends import create_backend
from dynamics.model import TheoreticalModel
from dynamics.resolver import resolve_robot

from .compensation import load_compensation
from .io import LowLatencyDifferentiator
from .runtime import estimate_torque_sample


def _fmt_vec(name: str, vec: np.ndarray, width: int = 7, precision: int = 2) -> str:
    values = " ".join(f"{v:{width}.{precision}f}" for v in vec)
    return f"{name}:[{values}]"


def monitor(
    config: dict,
    *,
    model_path: str | Path | None = None,
    hz: float | None = None,
) -> None:
    robot_name = str(config.get("robot_name", "robot"))
    joint_count = int(config.get("joint_count", 6))
    sample_hz = float(hz or config.get("sampling_hz", 100))
    dt = 1.0 / sample_hz
    robot = resolve_robot(config["urdf_path"], name=robot_name, payload=config.get("payload"))
    comp = load_compensation(model_path) if model_path else None
    backend = create_backend(config)
    diff = LowLatencyDifferentiator(joint_count)

    backend.connect()
    try:
        with TheoreticalModel(robot) as model:
            print("[MONITOR] press Ctrl+C to stop")
            next_t = time.monotonic()
            while True:
                sample = backend.read_sample()
                qd, qdd = diff.update(sample.timestamp, sample.q, sample.qd)
                estimate = estimate_torque_sample(
                    sample=sample,
                    qd=qd,
                    qdd=qdd,
                    joint_count=joint_count,
                    model=model,
                    compensator=comp,
                )
                line = " ".join(
                    [
                        _fmt_vec("q", sample.q, precision=3),
                        _fmt_vec("qd", qd, precision=3),
                        _fmt_vec("qdd", qdd, precision=2),
                        _fmt_vec("api", estimate.tau_api),
                        _fmt_vec("model", estimate.tau_theory),
                        _fmt_vec("static", estimate.tau_static_bias),
                        _fmt_vec("fw", estimate.tau_firmware_bias),
                        _fmt_vec("comp", estimate.tau_comp),
                        _fmt_vec("err", estimate.tau_external),
                    ]
                )
                sys.stdout.write("\r" + line[:240])
                sys.stdout.flush()
                next_t += dt
                delay = next_t - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_t = time.monotonic()
    except KeyboardInterrupt:
        print("\n[MONITOR] stopped")
    finally:
        backend.close()
