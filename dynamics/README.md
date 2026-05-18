# Dynamics

This directory contains the standalone dynamics, torque calibration, and torque compensation pipeline for the robot.
The current implementation is mainly built around xArm6.

## What It Does

- Parses robot URDF and optional payload metadata.
- Builds a PyBullet-based theoretical rigid-body dynamics model.
- Connects to the real xArm through the xArm SDK backend.
- Records hand-guided joint trajectories in teach mode.
- Generates bounded random workspace trajectories from hand-guided workspace samples.
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

## Configuration Inputs

`dynamics/config.yaml` is the complete default input sheet for the current dynamics pipeline. Robot-specific files such as `dynamics/config/xarm6.yaml` are loaded on top of it, and CLI/API arguments override both.

The config is grouped by responsibility:

| Section | Inputs |
| --- | --- |
| Robot identity | `robot_name`, `embodiment_id`, `sdk_backend`, `urdf_path`, `joint_count`, `home_joints_deg`, `gripper_open` |
| Connection | `connection.ip`, `connection.robot_port`, report ports, SDK timeout |
| Payload | `payload.enabled`, `payload.profile`, `payload.mode` |
| Paths | trajectory output dir, torque output dir, compensation checkpoint path |
| Trajectory | sample rate, mode, duration, teach sensitivity, workspace point count, margin, speed cap, seed |
| Torque replay | trajectory path override, optional compensation model path, replay speed/acceleration |
| Training | data paths, model kind, target, epochs, learning rate, hidden width, seed |
| Hybrid compensation | static ridge alpha, stop speed threshold, motion history window, delayed-jump fitting constants |
| Monitor | optional compensation model path |

Python functions keep fallback defaults so direct module calls remain usable, but normal workflow defaults should be changed in `dynamics/config.yaml`.

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
| `--traj-kind` | Trajectory submode: `drag`, `workspace`, or default `all` |
| `--traj` | Trajectory parquet for torque mode; repeat to replay multiple files |
| `--epochs`, `--lr`, `--hidden-dim`, `--seed` | Training overrides |
| `--static-alpha`, `--speed-threshold-deg-s`, `--motion-history-steps` | Hybrid compensation overrides |

## Typical Workflow

1. Record trajectories:

```bash
python -m dynamics.main --mode traj --robot xarm6 --ip 192.168.1.199
```

Output: by default two parquet files under `dynamics/calibration/traj/`:

- `*_traj_drag_*.parquet`: the direct hand-guided trajectory.
- `*_traj_workspace_*.parquet`: a random joint-space trajectory generated inside the sampled safe workspace.

To run only one trajectory submode:

```bash
python -m dynamics.main --mode traj --traj-kind drag
python -m dynamics.main --mode traj --traj-kind workspace
```

2. Replay the trajectories and collect torque data:

```bash
python -m dynamics.main --mode torque
```

Without `--traj`, torque mode replays the latest `drag` and `workspace` trajectory pair. To choose explicit files:

```bash
python -m dynamics.main --mode torque \
  --traj dynamics/calibration/traj/xarm6_traj_drag_example.parquet \
  --traj dynamics/calibration/traj/xarm6_traj_workspace_example.parquet
```

Output: one parquet file under `dynamics/calibration/torque/`, with `source_file` marking which trajectory produced each row.

3. Train a kinematic-history compensation model:

Calibration torque data must be collected without external contact or human-applied force. The arm should only be under its own motion, gravity, and the configured gripper/payload so `tau_api - tau_model` represents internal residual torque.

```bash
python -m dynamics.main --mode train \
  --model-kind kinematic_history \
  --data dynamics/calibration/torque/example.parquet \
  --model-path dynamics/calibration/compensation/history_q_qd.pt
```

Output: a `.pt` checkpoint. The default input is `q/qd` history shaped `(12, 20)` for xArm6; set `compensation.kinematic_history.channels: q_qd_qdd` to train the `(18, 20)` `q/qd/qdd` variant.

4. Monitor real-time torque:

```bash
python -m dynamics.main --mode monitor \
  --model-path dynamics/calibration/compensation/history_q_qd.pt
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
| `embodiment.py` | Build and check robot/checkpoint fingerprints | Dynamics config, resolved robot | Embodiment metadata and mismatch errors |
| `backends/base.py` | Robot backend interface | None | `RobotSample`, `RobotBackend` protocol |
| `backends/xarm.py` | xArm SDK backend | xArm connection config | Samples, teach mode, trajectory replay |
| `backends/registry.py` | Backend factory | Config or robot name | Robot backend instance |

## Calibration Files

| File | Purpose | Main Input | Main Output |
| --- | --- | --- | --- |
| `calibration/io.py` | Parquet I/O and online differentiation | Records, DataFrame, `q/qd` | parquet files, matrices, `qd/qdd` |
| `calibration/runtime.py` | Shared per-sample torque estimation | `RobotSample`, model, optional compensator | Torque estimate fields |
| `calibration/record.py` | Record drag/workspace trajectory modes | Config, duration, Hz, trajectory kind | One or more trajectory parquet paths |
| `calibration/workspace.py` | Estimate sampled joint bounds and generate speed-limited random workspace trajectories | Sampled `q`, point count, speed cap | Generated `q` trajectory and timestamps |
| `calibration/torque.py` | Replay one or more trajectories and log torque | Config, trajectory parquet paths, optional model | Torque parquet path |
| `calibration/train.py` | Train compensation checkpoint | Torque parquet, training args | `.pt` model path |
| `calibration/monitor.py` | Real-time torque monitor | Config, optional model path | Terminal stream |
| `calibration/validation.py` | Compare staged compensation terms | `tau_api`, `tau_model`, component predictions | RMSE metrics by segment |

## Compensation Models

| File | Purpose | Main Input | Main Output |
| --- | --- | --- | --- |
| `calibration/compensation/history.py` | Kinematic-history temporal CNN | `q/qd` or `q/qd/qdd` history plus residual targets | `KinematicHistoryCompensator`, `.pt` |
| `calibration/compensation/mlp.py` | Baseline MLP compensation | `q`, `qd`, `qdd`, `tau_api`, `tau_model` | `CompensationBundle`, `.pt` |
| `calibration/compensation/hybrid.py` | Hybrid compensation model | Motion/static/stop torque data | `HybridTorqueCompensator`, `.pt` |
| `calibration/compensation/static_bias.py` | Position-dependent static bias | Static `q`, residual torque | Static bias prediction |
| `calibration/compensation/firmware_state.py` | Delayed jump state and fitted levels | Residuals for fitting, kinematics for runtime | Jump bias and stop-state estimates |
| `calibration/compensation/per_joint_mlp.py` | Per-joint motion residual MLPs | Current and historical `q/qd/qdd` features | Motion compensation prediction |
| `calibration/compensation/__init__.py` | Unified checkpoint loader | `.pt` path | Kinematic-history, baseline, or hybrid model |

Kinematic-history checkpoints are trained to predict `tau_api - tau_model` from no-force data. Runtime prediction uses only kinematic history: the default B variant uses `q/qd` with shape `(12, 20)` for xArm6, while the A variant uses `q/qd/qdd` with shape `(18, 20)`. `tau_api` is used only to compute the final residual:

```text
tau_external = tau_api - tau_model - tau_comp
```

Compensation checkpoints saved through the training workflow include an `embodiment` fingerprint with robot name, active joint names, URDF hash, payload profile, and SI-unit conventions. Runtime torque collection and monitoring load checkpoints through the config-aware loader, so a checkpoint trained for another embodiment is rejected before use.

## Data Columns

Trajectory records usually contain:

- `timestamp`
- `robot`
- `mode`
- `traj_kind`
- `q_1 ... q_N`
- `qd_1 ... qd_N`
- `qdd_1 ... qdd_N`

Torque records add:

- `tau_api_1 ... tau_api_N`
- `tau_model_1 ... tau_model_N`
- `tau_theory_1 ... tau_theory_N`
- `tau_comp_1 ... tau_comp_N`
- `tau_error_1 ... tau_error_N`
- optional hybrid fields such as `tau_static_bias`, `tau_motion_comp`, `tau_firmware_bias`, `tau_external`, `time_since_stop`, and `firmware_state`

## Current Limitations

- Only the xArm backend is implemented.
- UR, Franka, and Kinova currently have capability notes only.
- `pybullet`, `torch`, `pandas`, and the xArm SDK are required for the full pipeline.
- `record/control/dynamics.py` is the degree-based runtime module used by recording; do not mix its API with the SI-unit dynamics APIs without converting units.
