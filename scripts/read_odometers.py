#!/usr/bin/env python3
"""read_odometers.py - Print the console's odometer readings and exit.

Connects to the console over USB CDC and prints both odometers:

  system odometer - total minutes the console has been powered on
  laser odometer  - total LSYNC laser pulses fired (~40 per second of scan)

Note: on firmware affected by
OpenwaterHealth/openmotion-console-fw#50, simply disconnecting at the end
of this script adds phantom pulses to the laser odometer (the firmware
re-counts the previous scan's pulses on every port close), so back-to-back
runs will show the laser value creeping up on its own.

Usage:
    python scripts/read_odometers.py
"""

from __future__ import annotations

import sys
import time

from omotion import MotionInterface

CONNECT_TIMEOUT_S = 12.0


def main() -> int:
    iface = MotionInterface()
    iface.start(wait=False)
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while time.monotonic() < deadline and not iface.console.is_connected():
        time.sleep(0.2)
    if not iface.console.is_connected():
        iface.stop()
        print(f"ERROR: console not connected within {CONNECT_TIMEOUT_S:.0f}s")
        return 1

    try:
        system_min = iface.console.get_system_odometer_minutes()
        laser_pulses = iface.console.get_laser_odometer_pulses()
    finally:
        iface.stop()

    if system_min is None or laser_pulses is None:
        print("ERROR: firmware NAK'd an odometer read (build predates the feature?)")
        return 1

    print(f"system odometer: {system_min:,} min  ({system_min // 60}h {system_min % 60:02d}m)")
    print(f"laser odometer:  {laser_pulses:,} pulses  "
          f"(~{laser_pulses / 40:,.0f} s of scan at 40 Hz)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
