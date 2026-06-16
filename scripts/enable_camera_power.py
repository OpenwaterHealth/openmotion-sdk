#!/usr/bin/env python3
"""Enable power to selected cameras on all connected sensors.

Usage
-----
    python scripts/enable_camera_power.py --mask 0xFF
"""

import argparse
import sys

from omotion import MotionInterface


def parse_args():
    parser = argparse.ArgumentParser(description="Enable power to selected cameras on all connected sensors")
    parser.add_argument(
        "--mask",
        type=lambda x: int(x, 0),  # supports hex (e.g., 0xFF) or decimal
        default=0xFF,
        help="Camera bitmask to power on (default 0xFF for all cameras)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    interface = MotionInterface()
    interface.start()

    try:
        console_connected, left_connected, right_connected = interface.is_device_connected()

        if console_connected and left_connected and right_connected:
            print("MOTION System fully connected.")
        else:
            print(
                f"MOTION System NOT Fully Connected. CONSOLE: {console_connected}, "
                f"SENSOR (LEFT,RIGHT): {left_connected}, {right_connected}"
            )

        if not left_connected and not right_connected:
            print("Sensor Module not connected.")
            return 1

        print(f"Enabling camera power with mask {args.mask:#04x} on all connected sensors...")
        results = interface.run_on_sensors("enable_camera_power", args.mask)

        any_success = False
        for side, success in results.items():
            if success is True:
                any_success = True
                print(f"{side.capitalize()}: ✅ Power enabled")
            elif success is False:
                print(f"{side.capitalize()}: ❌ Failed to enable power")
            else:
                print(f"{side.capitalize()}: ⚠️ No result (possibly disconnected)")

        return 0 if any_success else 1
    finally:
        interface.stop()


if __name__ == "__main__":
    sys.exit(main())
