4060-side source import

This directory contains code imported from the 4060 host, primarily from:

- `~/ros_ws_zyl/src/xarm_sys/scripts/xarm_joycon`
- `~/ros_ws_zyl/scripts/replay_episode_5090.py`

Import policy:

- Included: source files, reusable scripts, examples, meshes, XML assets, vendored helper libraries.
- Excluded: build outputs, `devel/`, `data/`, `logs/`, `__pycache__/`, compiled Python extension artifacts, and git metadata.

Intent:

- Preserve the original 4060-side mouse/joycon teleop codebase in one repository with the 5090-side runtime.
- Keep the current 5090 teleop entrypoints under `scripts/`.
- Keep the imported 4060 code isolated under `legacy_4060/` until it is selectively refactored or merged.
