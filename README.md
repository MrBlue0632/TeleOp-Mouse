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

**Required packages:** `pynput`, `xarm-python-sdk`, `numpy`, `opencv-python`, `lerobot`

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
| `observation.state` | (14,) | 6 joints + gripper + 7 currents |
| `action` | (7,) | 6 velocity + gripper action |
| `observation.images.wrist` | (480,640,3) | Wrist camera (video) |
| `observation.images.base` | (480,640,3) | Base camera (video) |
| `observation.joints_deg` | (6,) | Joint angles in degrees |
| `observation.pose_xyzrpy_deg` | (6,) | End-effector pose |
| `observation.currents` | (7,) | Raw joint currents |
| `observation.currents_filtered` | (7,) | Filtered currents |

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
