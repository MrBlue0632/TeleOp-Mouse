# Tools

Small one-shot maintenance scripts that are useful outside the main recording
flow live here. Keep long-running data collection code in `record/`.

Current tools:

- `reset.py`: reset a configured robot arm to its home position.

Example:

```bash
python3 tools/reset.py --robot xarm6
```
