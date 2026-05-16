# Dynamics

This directory contains the standalone dynamics, torque calibration, and torque compensation pipeline for the robot.
The current implementation is mainly built around xArm6.

## What It Does

- Parses robot URDF and optional payload metadata.
- Builds a PyBullet-based theoretical rigid-body dynamics model.
- Connects to the real xArm through the xArm SDK backend.
- Records hand-guided joint trajectories in teach mode.
- Replays trajectories and logs measured/API torque against theoretical torque.
- Trains torque compensation models from logged data.
- Runs a real-time torque monitor with optional compensation.

All public inputs in the new `dynamics/` package use SI units:

- `q`: radians
- `qd`: radians/second
- `qdd`: radians/second^2
- torque: Nm

## Main CLI

Use:

```bash
python -m dynamics.main --mode <traj|torque|train|monitor>
```

For Python callers, use the public facade:

```python
from dynamics.api import load_dynamics_config, run_mode

config = load_dynamics_config(robot="xarm6")
run_mode("traj", config, duration_s=10.0)
```

`main.py` owns the workflow orchestration:

```text
CLI args
  -> main.parse_args()
  -> main.run_cli_args()
  -> config.load_config()
  -> main.run_mode()
  -> calibration record / torque / train / monitor
  -> parquet, checkpoint, or terminal stream
```

Common options:

| Option | Meaning |
| --- | --- |
| `--robot` | Robot config name, default `xarm6` |
| `--config` | Custom YAML config path |
| `--ip` | Override robot IP |
| `--urdf` | Override URDF path |
| `--hz` | Override sampling rate |
| `--model-path` | Compensation checkpoint path |

## Typical Workflow

1. Record a teach-mode trajectory:

```bash
python -m dynamics.main --mode traj --robot xarm6 --ip 192.168.1.199
```

Output: a parquet file under `dynamics/calibration/traj/`.

2. Replay the trajectory and collect torque data:

```bash
python -m dynamics.main --mode torque --traj dynamics/calibration/traj/example.parquet
```

Output: a parquet file under `dynamics/calibration/torque/`.

3. Train a baseline compensation model:

```bash
python -m dynamics.main --mode train \
  --model-kind baseline \
  --data dynamics/calibration/torque/example.parquet \
  --model-path dynamics/calibration/compensation/compensation.pt
```

Output: a `.pt` checkpoint.

4. Monitor real-time torque:

```bash
python -m dynamics.main --mode monitor \
  --model-path dynamics/calibration/compensation/compensation.pt
```

Output: live terminal values for joint state, API torque, model torque, compensation, and residual error.

## Core Files

| File | Purpose | Main Input | Main Output |
| --- | --- | --- | --- |
| `api.py` | Public Python facade over config/model/workflow helpers | Loaded config, mode args | Delegates to `main.py` or builds model helpers |
| `main.py` | CLI entrypoint and workflow orchestrator | CLI args or loaded config | Runs one mode |
| `config.py` | Load YAML config and CLI overrides | Config path, robot name, overrides | `dict` config |
| `resolver.py` | Parse URDF and payload metadata | URDF path, payload config | `ResolvedRobot` |
| `model.py` | Theoretical dynamics through PyBullet | `ResolvedRobot`, `q/qd/qdd` | mass matrix, gravity, Coriolis, inverse dynamics, estimated torque |
| `backends/base.py` | Robot backend interface | None | `RobotSample`, `RobotBackend` protocol |
| `backends/xarm.py` | xArm SDK backend | xArm connection config | Samples, teach mode, trajectory replay |
| `backends/registry.py` | Backend factory | Config or robot name | Robot backend instance |

## Calibration Files

| File | Purpose | Main Input | Main Output |
| --- | --- | --- | --- |
| `calibration/io.py` | Parquet I/O and online differentiation | Records, DataFrame, `q/qd` | parquet files, matrices, `qd/qdd` |
| `calibration/runtime.py` | Shared per-sample torque estimation | `RobotSample`, model, optional compensator | Torque estimate fields |
| `calibration/record.py` | Record teach-mode trajectory | Config, duration, Hz | Trajectory parquet path |
| `calibration/torque.py` | Replay trajectory and log torque | Config, trajectory parquet, optional model | Torque parquet path |
| `calibration/train.py` | Train compensation checkpoint | Torque parquet, training args | `.pt` model path |
| `calibration/monitor.py` | Real-time torque monitor | Config, optional model path | Terminal stream |

## Compensation Models

| File | Purpose | Main Input | Main Output |
| --- | --- | --- | --- |
| `calibration/compensation/mlp.py` | Baseline MLP compensation | `q`, `qd`, `qdd`, `tau_api`, `tau_theory` | `CompensationBundle`, `.pt` |
| `calibration/compensation/hybrid.py` | Hybrid compensation model | Motion/static/stop torque data | `HybridTorqueCompensator`, `.pt` |
| `calibration/compensation/static_bias.py` | Position-dependent static bias | Static `q`, residual torque | Static bias prediction |
| `calibration/compensation/firmware_state.py` | Firmware/current-bias state tracker | Residual, velocity, timestamps | Firmware bias and stop-state estimates |
| `calibration/compensation/per_joint_mlp.py` | Per-joint motion residual MLPs | Motion features and targets | Motion compensation prediction |
| `calibration/compensation/__init__.py` | Unified checkpoint loader | `.pt` path | Baseline or hybrid model |

## Data Columns

Trajectory records usually contain:

- `timestamp`
- `robot`
- `mode`
- `q_1 ... q_N`
- `qd_1 ... qd_N`
- `qdd_1 ... qdd_N`

Torque records add:

- `tau_api_1 ... tau_api_N`
- `tau_theory_1 ... tau_theory_N`
- `tau_comp_1 ... tau_comp_N`
- `tau_error_1 ... tau_error_N`
- optional hybrid fields such as `tau_static_bias`, `tau_motion_comp`, `tau_firmware_bias`, `tau_external`, `time_since_stop`, and `firmware_state`

## Current Limitations

- Only the xArm backend is implemented.
- UR, Franka, and Kinova currently have capability notes only.
- `pybullet`, `torch`, `pandas`, and the xArm SDK are required for the full pipeline.
- `robot_control/dynamics.py` is a separate older runtime module that uses degree-based inputs; do not mix its API with the SI-unit `dynamics/` package without converting units.
