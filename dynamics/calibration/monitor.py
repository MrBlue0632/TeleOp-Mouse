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
                tau_api = sample.tau_api if sample.tau_api is not None else np.zeros(joint_count)
                tau_theory = model.estimate_joint_torque(sample.q, qd, qdd)
                tau_static = np.zeros(joint_count)
                tau_firmware = np.zeros(joint_count)
                if comp is None:
                    tau_comp = np.zeros(joint_count)
                    tau_error = tau_api - tau_theory
                elif hasattr(comp, "update"):
                    estimate = comp.update(sample.q, qd, qdd, tau_api, tau_theory, timestamp=sample.timestamp)
                    tau_comp = estimate.tau_comp
                    tau_static = estimate.tau_static_bias
                    tau_firmware = estimate.tau_firmware_bias
                    tau_error = estimate.tau_external
                else:
                    tau_comp = comp.predict_compensation(sample.q, qd, qdd, tau_theory=tau_theory)
                    tau_error = tau_api - tau_theory - tau_comp
                line = " ".join(
                    [
                        _fmt_vec("q", sample.q, precision=3),
                        _fmt_vec("qd", qd, precision=3),
                        _fmt_vec("qdd", qdd, precision=2),
                        _fmt_vec("api", tau_api),
                        _fmt_vec("model", tau_theory),
                        _fmt_vec("static", tau_static),
                        _fmt_vec("fw", tau_firmware),
                        _fmt_vec("comp", tau_comp),
                        _fmt_vec("err", tau_error),
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
