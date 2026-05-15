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
./scripts/collect_demo.sh

# Custom task description
./scripts/collect_demo.sh --task "pick_red_cube"

# Without video window (headless)
./scripts/collect_demo.sh --no-video

# Custom data directory
./scripts/collect_demo.sh --data-dir ~/my_dataset
```

## Controls

| Key | Function |
|-----|----------|
| W/S | Forward / Backward |
| A/D | Left / Right |
| Space/Shift | Down / Up |
| Q/E | Rotate Z-axis |
| Mouse move | Rotate X/Y-axis (pitch/yaw) |
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
| `observation.pose_xyzrpy_deg` | (6,) | End-effector pose |
| `observation.torques` | (7,) | Raw API joint torques |
| `observation.torques_filtered` | (7,) | Filtered joint torques |
| `observation.torque_external` | (6,) | Estimated external joint torques |
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

## Hybrid Torque Compensation

Train the legacy baseline MLP:

```bash
python -m dynamics.main --mode train --model-kind baseline --data dynamics/calibration/torque/example.parquet
```

Train the hybrid per-joint compensator:

```bash
python -m dynamics.main --mode train --model-kind hybrid \
  --data dynamics/calibration/torque/motion.parquet \
  --static-data dynamics/calibration/torque/static.parquet \
  --stop-data dynamics/calibration/torque/stop.parquet \
  --model-path dynamics/calibration/compensation/hybrid.pt
```

Use a hybrid checkpoint during teleop:

```bash
TELEOP_TORQUE_COMP_MODEL=dynamics/calibration/compensation/hybrid.pt ./scripts/collect_demo.sh
```
