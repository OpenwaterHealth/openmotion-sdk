#!/usr/bin/env python3
"""
check_odometer.py

Read the console's lifetime usage counters (system uptime + cumulative
laser pulses) and print them. Optionally reset one or both.

The odometer values live in console flash and are exposed via the
``OW_CTRL_GET_SYSTEM_ODO`` / ``GET_LASER_ODO`` / ``RESET_ODO`` opcodes
(0x26 / 0x27 / 0x28). Older firmware that predates the feature will NAK
these opcodes — the script prints a clear message and exits non-zero in
that case so callers (CI, ops scripts) can branch on it.

Usage
-----
    python scripts/check_odometer.py
    python scripts/check_odometer.py --reset system   [--no-confirm]
    python scripts/check_odometer.py --reset laser    [--no-confirm]
    python scripts/check_odometer.py --reset both     [--no-confirm]
"""

import argparse
import sys
import time

from omotion import MotionInterface


# Maps the CLI --reset names to the firmware's OdoResetTarget enum.
_RESET_TARGETS = {"system": 0, "laser": 1, "both": 2}


def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read (and optionally reset) the console odometer.",
    )
    parser.add_argument(
        "--reset",
        choices=tuple(_RESET_TARGETS.keys()),
        default=None,
        help=(
            "Reset one or both odometers to zero after reading. "
            "Persists the cleared state to console flash."
        ),
    )
    parser.add_argument(
        "--no-confirm",
        action="store_true",
        help="Skip the interactive confirmation before resetting.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the console to connect (default: 10).",
    )
    return parser.parse_args()


def _wait_for_console(interface: MotionInterface, timeout: float) -> bool:
    """Poll until the console handle reaches CONNECTED or timeout expires."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if interface.console.is_connected():
            return True
        time.sleep(0.1)
    return interface.console.is_connected()


def _format_minutes(minutes: int | None) -> str:
    if minutes is None:
        return "?"
    h = minutes // 60
    m = minutes % 60
    d = h // 24
    if d > 0:
        return f"{minutes:,} min  ({d}d {h % 24}h {m:02d}m)"
    return f"{minutes:,} min  ({h}h {m:02d}m)"


def main() -> int:
    args = parse_cli()

    print("[*] Starting MOTION interface …")
    interface = MotionInterface()
    interface.start()

    try:
        if not _wait_for_console(interface, args.timeout):
            print(f"❌  console not connected (waited {args.timeout:.1f}s).")
            return 1

        console = interface.console

        # Read the firmware version too so the output is self-explanatory when
        # the odometer opcodes NAK on older builds.
        try:
            fw = console.get_version()
        except Exception:
            fw = "?"
        print(f"[*] Console firmware: {fw}")

        print("[*] Reading odometer …")
        system_min = console.get_system_odometer_minutes()
        laser_pulses = console.get_laser_odometer_pulses()

        if system_min is None and laser_pulses is None:
            print(
                "❌  Console did not return odometer data. This firmware "
                "predates the odometer feature (or both opcodes failed)."
            )
            return 2

        print()
        print("  System uptime: " + _format_minutes(system_min))
        print(
            "  Laser pulses : "
            + ("?" if laser_pulses is None else f"{laser_pulses:,}")
        )
        print()

        if args.reset is None:
            return 0

        target_name = args.reset
        target_code = _RESET_TARGETS[target_name]
        if not args.no_confirm:
            answer = input(
                f"⚠️  Reset {target_name} odometer to zero? (y/N): "
            ).strip().lower()
            if answer != "y":
                print("Aborted.")
                return 0

        print(f"[+] Resetting {target_name} odometer …")
        ok = console.reset_odometer(target_code)
        if not ok:
            print(f"❌  Reset failed (firmware reported error).")
            return 3

        # Read back so the user sees the post-reset state in one log.
        system_min = console.get_system_odometer_minutes()
        laser_pulses = console.get_laser_odometer_pulses()
        print()
        print("[+] Reset complete. New values:")
        print("    System uptime: " + _format_minutes(system_min))
        print(
            "    Laser pulses : "
            + ("?" if laser_pulses is None else f"{laser_pulses:,}")
        )
        return 0

    finally:
        interface.stop()


if __name__ == "__main__":
    sys.exit(main())
