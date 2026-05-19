#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import sys
from pathlib import Path


SCRIPTS = [
    "c_firmware_arming_acceptance.py",
    "c_firmware_joystick_modes_acceptance.py",
    "c_firmware_passthrough_acceptance.py",
    "c_firmware_waypoint_acceptance.py",
]


def main() -> int:
    root = Path(__file__).resolve().parent
    for script in SCRIPTS:
        print(f"=== {script} ===", flush=True)
        result = subprocess.run([sys.executable, str(root / script)])
        if result.returncode != 0:
            return result.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
