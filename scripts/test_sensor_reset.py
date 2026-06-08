#!/usr/bin/env python3
"""
test_sensor_reset.py

Send a soft reset command to connected MOTION sensors.

Usage
-----
    python test_sensor_reset.py [--no-confirm]
"""

import argparse
import time
import sys

from omotion.MotionInterface import MotionInterface


_CONNECT_TIMEOUT = 5.0


def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a soft reset command to connected MOTION sensors."
    )
    parser.add_argument(
        "--no-confirm",
        action="store_true",
        help="Skip the interactive confirmation before each soft reset.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_cli()

    print("[*] Acquiring MOTION interface …")
    iface = MotionInterface()
    iface.start(wait=True, wait_timeout=_CONNECT_TIMEOUT)

    # Poll for connection
    def _await(handle, label):
        deadline = time.monotonic() + _CONNECT_TIMEOUT
        while time.monotonic() < deadline:
            if handle.is_connected():
                return True
            time.sleep(0.1)
        print(f"❌  {label} not connected after {_CONNECT_TIMEOUT:.0f}s.")
        return False

    left_connected = _await(iface.left, "Left sensor")
    right_connected = _await(iface.right, "Right sensor")

    if not left_connected and not right_connected:
        print("No sensors are connected.")
        iface.stop()
        return 1

    def _maybe_reset(sensor, label):
        if not sensor.is_connected():
            return True

        if not args.no_confirm:
            answer = input(
                f"Do you want to soft reset the {label}? (y/N): "
            ).strip().lower()
            if answer != "y":
                print(f"Aborted {label} reset.")
                return True

        print(f"[+] Sending soft reset to {label} …")
        try:
            ok = sensor.soft_reset()
        except Exception as exc:
            print(f"   ❌  Exception while resetting {label}: {exc}")
            return False

        if ok:
            print(f"   ✅  Soft reset sent successfully to {label}.")
            return True

        print(f"   ❌  {label} did not report success.")
        return False

    success = True
    success = _maybe_reset(iface.left, "left sensor") and success
    success = _maybe_reset(iface.right, "right sensor") and success

    try:
        return 0 if success else 1
    finally:
        iface.stop()


if __name__ == "__main__":
    sys.exit(main())
