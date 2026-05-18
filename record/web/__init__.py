"""Web dashboard helpers for the record data-collection subsystem.

Inputs: telemetry snapshots, camera frames, config requests, and optional reset
callbacks from ``record.teleop``. Returns importable server/state helpers for the
record package and compatibility shims.
"""

__all__ = [
    "ConfigStore",
    "DashboardServer",
    "DashboardState",
    "build_app_snapshot",
    "start_dashboard_server",
    "torque_bar",
]


def __getattr__(name):
    if name in {"ConfigStore", "DashboardServer", "start_dashboard_server"}:
        from .server import ConfigStore, DashboardServer, start_dashboard_server

        values = {
            "ConfigStore": ConfigStore,
            "DashboardServer": DashboardServer,
            "start_dashboard_server": start_dashboard_server,
        }
        return values[name]
    if name in {"DashboardState", "build_app_snapshot", "torque_bar"}:
        from .telemetry import DashboardState, build_app_snapshot, torque_bar

        values = {
            "DashboardState": DashboardState,
            "build_app_snapshot": build_app_snapshot,
            "torque_bar": torque_bar,
        }
        return values[name]
    raise AttributeError(name)
