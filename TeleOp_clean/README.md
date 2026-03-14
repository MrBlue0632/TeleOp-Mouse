# TeleOp_clean

`TeleOp_clean` is the minimal local-only teleoperation bundle for this host.

It keeps only the files needed to run keyboard and mouse teleop on the same
machine as the operator. SSH tunnel launchers, 4060 integration, runtime
monitors, and legacy sources are intentionally excluded.

## Layout

- `scripts/teleop_keyboard_mouse.py`: core local teleop runtime
- `scripts/run_local_teleop.sh`: local launcher with dependency check
- `tests/test_teleop_keyboard_mouse.py`: small regression suite for input and camera logic

## Run

```bash
cd /home/zyl5090/Projects/Teleop/TeleOp_clean
./scripts/run_local_teleop.sh
```

Common overrides:

```bash
./scripts/run_local_teleop.sh --robot-ip 192.168.1.199
./scripts/run_local_teleop.sh --camera-dev /dev/video6
./scripts/run_local_teleop.sh --camera-id 1
./scripts/run_local_teleop.sh --no-fps-mouse
./scripts/run_local_teleop.sh --no-video
```

## Defaults

- robot IP: `192.168.1.199`
- control port: `502`
- report ports: `30001`, `30002`, `30003`
- camera: `/dev/video4`, fallback enabled
- episode output: `TeleOp_clean/data`

If local forwarded xArm ports already exist on `127.0.0.1:1502/13001/13002/13003`,
the launcher reuses them automatically. It does not create SSH tunnels itself.

## Verify

```bash
python3 scripts/teleop_keyboard_mouse.py --self-check
python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 -m py_compile scripts/teleop_keyboard_mouse.py
```
