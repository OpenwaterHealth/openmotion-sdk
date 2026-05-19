#!/usr/bin/env python3
"""
enter_dfu.py

Put a chosen MOTION device (console or left/right sensor) into DFU mode.

Usage
-----
    python enter_dfu.py <device> [--no-confirm] [--timeout SECONDS]

    device:  console | left | right
"""

import argparse
import sys
import time

from omotion import MotionInterface


def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Put a MOTION device (console or sensor) into DFU mode."
    )
    parser.add_argument(
        "device",
        choices=("console", "left", "right"),
        help="Which device to put into DFU mode: console, left sensor, or right sensor.",
    )
    parser.add_argument(
        "--no-confirm",
        action="store_true",
        help="Skip the interactive confirmation before entering DFU mode.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the target device to connect (default: 10).",
    )
    return parser.parse_args()


def _wait_for_handle(handle, timeout: float) -> bool:
    """Poll the specific handle until it reaches CONNECTED or timeout expires."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if handle.is_connected():
            return True
        time.sleep(0.1)
    return handle.is_connected()


def main() -> int:
    args = parse_cli()

    print("[*] Starting MOTION interface …")
    interface = MotionInterface()
    interface.start()

    try:
        if args.device == "console":
            target = interface.console
            label = "console module"
        elif args.device == "left":
            target = interface.left
            label = "left sensor"
        else:  # right
            target = interface.right
            label = "right sensor"

        if not _wait_for_handle(target, args.timeout):
            print(f"❌  {label} not connected (waited {args.timeout:.1f}s).")
            return 1

        if not args.no_confirm:
            answer = input(
                f"Do you want to put the {label} into DFU mode? (y/N): "
            ).strip().lower()
            if answer != "y":
                print("Aborted.")
                return 0

        print(f"[+] Requesting DFU mode from {label} …")
        try:
            ok = target.enter_dfu()
        except Exception as exc:
            print(f"   ❌  Exception: {exc}")
            return 1

        if ok:
            print("   ✅  DFU mode requested successfully. Device should re-enumerate as DFU.")
            return 0
        print("   ❌  Device did not report success.")
        return 1
    finally:
        interface.stop()


if __name__ == "__main__":
    sys.exit(main())
