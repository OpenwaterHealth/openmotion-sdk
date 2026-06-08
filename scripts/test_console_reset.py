#!/usr/bin/env python3
"""
test_console_reset.py

Send a soft reset command to the connected MOTION console.

Usage
-----
    python test_console_reset.py [--no-confirm]
"""

import argparse
import time
import sys

from omotion.MotionInterface import MotionInterface


_CONNECT_TIMEOUT = 5.0


def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a soft reset command to the connected MOTION console."
    )
    parser.add_argument(
        "--no-confirm",
        action="store_true",
        help="Skip the interactive confirmation before soft reset.",
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

    console_connected = _await(iface.console, "Console")

    if not console_connected:
        iface.stop()
        return 1

    if not args.no_confirm:
        answer = input(
            "Do you want to soft reset the console? (y/N): "
        ).strip().lower()
        if answer != "y":
            print("Aborted.")
            iface.stop()
            return 0

    # Stop the background telemetry poller before the reset so it exits
    # cleanly while the device is still alive.  If we reset first and then
    # stop, the poller is mid-poll on a dead serial port and logs a cascade
    # of ClearCommError / tec_status / _read_all errors before it notices.
    iface.console.telemetry.stop()

    print("[+] Sending soft reset to console …")
    try:
        ok = iface.console.soft_reset()
    except Exception as exc:
        print(f"   ❌  Exception: {exc}")
        iface.stop()
        return 1

    try:
        if ok:
            print("   ✅  Soft reset sent successfully. Console should reboot.")
            return 0
        print("   ❌  Console did not report success.")
        return 1
    finally:
        iface.stop()


if __name__ == "__main__":
    sys.exit(main())
