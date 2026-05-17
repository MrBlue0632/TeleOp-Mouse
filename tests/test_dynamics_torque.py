import tempfile
import unittest
from pathlib import Path

import numpy as np

from dynamics.calibration.io import make_record, write_parquet
from dynamics.calibration.torque import resolve_traj_paths


class TorqueTrajectoryResolutionTests(unittest.TestCase):
    def write_traj(self, directory: str, filename: str) -> Path:
        return write_parquet(
            [
                make_record(
                    timestamp=1.0,
                    robot="xarm6",
                    joint_count=2,
                    mode="traj",
                    q=np.array([0.1, 0.2]),
                    qd=np.array([0.0, 0.0]),
                    qdd=np.array([0.0, 0.0]),
                )
            ],
            Path(directory) / filename,
        )

    def test_explicit_multiple_trajectory_paths_are_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            drag = self.write_traj(tmp, "drag.parquet")
            workspace = self.write_traj(tmp, "workspace.parquet")

            paths = resolve_traj_paths([str(drag), str(workspace)], tmp)

        self.assertEqual(paths, [drag, workspace])

    def test_default_resolution_returns_latest_drag_and_workspace_pair(self):
        with tempfile.TemporaryDirectory() as tmp:
            old_drag = self.write_traj(tmp, "xarm6_traj_drag_20260101_000000.parquet")
            drag = self.write_traj(tmp, "xarm6_traj_drag_20260101_000001.parquet")
            workspace = self.write_traj(tmp, "xarm6_traj_workspace_20260101_000002.parquet")
            old_drag.touch()
            drag.touch()
            workspace.touch()

            paths = resolve_traj_paths(None, tmp)

        self.assertEqual(paths, [drag, workspace])


if __name__ == "__main__":
    unittest.main()
