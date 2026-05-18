"""Orchestration entrypoint for the record data-collection subsystem.

Inputs: command-line arguments accepted by ``record.teleop``.
Returns: the TeleOp recorder process or a lightweight self-check exit status.
"""

from __future__ import annotations

import sys

__all__ = ["TeleopApp", "main", "self_check"]


def __getattr__(name):
    if name == "TeleopApp":
        from .teleop import TeleopApp

        return TeleopApp
    raise AttributeError(name)


def self_check() -> int:
    required = {"w", "a", "s", "d", "q", "e", "space", "shift", "r", "enter", "mouse_move", "mouse_left"}
    got = {"w", "a", "s", "d", "q", "e", "space", "shift", "r", "enter", "mouse_move", "mouse_left"}
    if got != required:
        print("SELF_CHECK_FAIL_MAPPING")
        return 1
    if not (0.0 < 1.2 <= 2.0 and 0.0 < 0.12 <= 0.3 and 0.0 < 20.0 <= 40.0):
        print("SELF_CHECK_FAIL_DEFAULTS")
        return 1
    print("SELF_CHECK_OK")
    return 0


def main(argv: list[str] | None = None) -> None:
    args = list(sys.argv[1:] if argv is None else argv)
    if "--self-check" in args:
        raise SystemExit(self_check())

    from .teleop import main as _teleop_main

    _teleop_main(args)


if __name__ == "__main__":
    main()
