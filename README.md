# TeleOp-Mouse

Keyboard & mouse teleoperation for xArm 6, with LeRobot v3.0 dataset recording.

## Setup

**Dependencies:**
```bash
# Activate conda environment
source ~/anaconda3/etc/profile.d/conda.sh && conda activate base

# Ensure lerobot is available
pip install lerobot
# Or use local source:
export PYTHONPATH=~/lerobot/src:$PYTHONPATH
```

**Required packages:** `pynput`, `xarm-python-sdk`, `numpy`, `opencv-python`, `lerobot`, `pybullet`

## Quick Start

```bash
# Collect demo dataset (recommended)
./record/collect_demo.sh

# Custom task description
./record/collect_demo.sh --task "pick_red_cube"

# Without video window (headless)
./record/collect_demo.sh --no-video

# Custom data directory
./record/collect_demo.sh --data-dir ~/my_dataset
```

## Controls

| Key | Function |
|-----|----------|
| W/S | Forward / Backward |
| A/D | Left / Right |
| Space/Shift | Up / Down |
| Q/E | Rotate Z-axis |
| Mouse move | Horizontal yaw RZ, vertical pitch RY |
| Left click | Toggle gripper |
| Right click | Open gripper (gradual) |
| Tab | Toggle BASE / TOOL coordinate frame |
| 1/2/3 | Speed 0.5x / 1x / 2x |
| R | Return to home position |
| **Enter** | **Save current episode** |
| ESC / Ctrl+C | Exit |

## Dataset Format

Data is saved in **LeRobot v3.0** format, directly compatible with LeRobot training pipelines.

```
data/lerobot_dataset/
├── data/chunk-000/file-000.parquet    # Observations + actions
├── videos/
│   ├── observation.images.wrist/     # Wrist camera video
│   └── observation.images.base/      # Base camera video
└── meta/
    ├── info.json                     # Dataset metadata
    ├── stats.json                    # Feature statistics
    ├── tasks.parquet                 # Task definitions
    └── episodes/                     # Episode metadata
```

**Features recorded:**

| Feature | Shape | Description |
|---------|-------|-------------|
| `observation.state` | (14,) | 6 joints + gripper + 7 raw torques |
| `action` | (7,) | 6 velocity + gripper action |
| `observation.images.wrist` | (480,640,3) | Wrist camera (video) |
| `observation.images.base` | (480,640,3) | Base camera (video) |
| `observation.joints_deg` | (6,) | Joint angles in degrees |
| `observation.joint_velocities_deg_s` | (6,) | Joint velocity `qd` in deg/s |
| `observation.pose_xyzrpy_deg` | (6,) | End-effector pose |
| `observation.torques` | (7,) | Raw API joint torques |
| `observation.torques_filtered` | (7,) | Filtered joint torques |
| `observation.torque_external` | (6,) | Admittance-control external joint torques after dead-zone + EMA filter |
| `observation.torque_model` | (6,) | Gravity + Coriolis model torque |
| `observation.torque_static_bias` | (6,) | Position-dependent static residual compensation |
| `observation.torque_motion_comp` | (6,) | Motion residual compensation |
| `observation.torque_firmware_bias` | (6,) | Firmware-bias compensation term |
| `observation.torque_bias_lambda` | (1,) | Motion/static blend used for firmware-bias compensation |
| `observation.torque_time_since_stop` | (6,) | Per-joint stop duration used by hybrid compensation |
| `observation.torque_firmware_state` | (6,) | Per-joint detected firmware bias level |
| `observation.ee_force` | (6,) | Estimated end-effector wrench `[Fx,Fy,Fz,Tx,Ty,Tz]` |

## Loading Dataset

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset("teleop/xarm_demo", root="data/lerobot_dataset")
print(f"{dataset.num_episodes} episodes, {dataset.num_frames} frames")
print(dataset[0])  # First frame
```

## Defaults

- Robot IP: `192.168.1.199`
- Control frequency: 120 Hz
- Data capture: 30 Hz
- Camera: index 1, fallback enabled
- Video codec: libsvtav1 (streaming encoding)

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBOT_IP` | `192.168.1.199` | xArm IP address |
| `DATA_DIR` | `./data` | Dataset output directory |
| `REPO_ID` | `teleop/xarm_demo` | LeRobot dataset repo ID |
| `TASK` | `keyboard_mouse_teleop` | Task description |
| `RATE_HZ` | `30` | Data capture frequency |
| `VCODEC` | `libsvtav1` | Video codec |
| `CAMERA_ID` | `1` | Camera device index |

## Torque Compensation

Compensation training data must be collected with no external contact or human-applied force. During these calibration trajectories the arm should only be affected by its own motion, gravity, and the configured gripper/payload. Under that assumption, `tau_api - tau_model` is treated as an internal residual/bias target, not as real external force.

Train the default kinematic-history compensator. This uses the recommended B variant: `q/qd` only, 20 points from the current sample back through the 1.9 s history at 0.1 s spacing, and a `(6,)` joint-torque compensation output.

```bash
python -m dynamics.main --mode train \
  --model-kind kinematic_history \
  --data dynamics/calibration/torque/example.parquet \
  --model-path dynamics/calibration/compensation/history_q_qd.pt
```

To evaluate the A variant, set `compensation.kinematic_history.channels: q_qd_qdd` in the dynamics config before training. The A variant adds `qdd` history but is more sensitive to differentiation noise.

Legacy models remain available for comparison:

```bash
python -m dynamics.main --mode train --model-kind baseline --data dynamics/calibration/torque/example.parquet
python -m dynamics.main --mode train --model-kind hybrid --data dynamics/calibration/torque/motion.parquet
```

Use a validated checkpoint during teleop only when its calibration workspace matches the task workspace. Runtime compensation is guarded; if the model output exceeds `TELEOP_TORQUE_COMP_MAX_ABS_NM` (default 25 Nm), teleop falls back to the direct admittance external-torque estimator. `record/collect_demo.sh` auto-enables `dynamics/calibration/compensation/history_q_qd.pt` when that file exists; pass `--no-torque-comp` to disable it.

```bash
TELEOP_TORQUE_COMP_MODEL=dynamics/calibration/compensation/history_q_qd.pt ./record/collect_demo.sh
```
