"""Record package for TeleOp data collection.

Inputs: CLI commands, robot/camera streams, LeRobot dataset configuration, and
dashboard telemetry. Returns construction and CLI entrypoints without importing
heavy robot runtime dependencies during package import.
"""

__all__ = ["TeleopApp", "main", "self_check"]


def __getattr__(name):
    if name == "TeleopApp":
        from .teleop import TeleopApp

        return TeleopApp
    if name in {"main", "self_check"}:
        from .main import main, self_check

        return {"main": main, "self_check": self_check}[name]
    raise AttributeError(name)
