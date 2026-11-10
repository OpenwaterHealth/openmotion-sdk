#!/usr/bin/env python3
"""
enter_dfu.py

Put a chosen MOTION device (console or left/right sensor) into DFU mode and
report what it is running there: our bootloader and its version, or the STM32
ROM loader of a unit without a bootloader.

Usage
-----
    python enter_dfu.py <device> [--no-confirm] [--timeout SECONDS]

    device:  console | left | right
"""

import argparse
import sys
import time

from omotion import MotionInterface
from omotion.dfu import STM32DFU, DFUError

# How long the unit gets to re-enumerate as a DFU device (0483:DF11).
DFU_APPEAR_TIMEOUT_S = 20.0


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


def _report_dfu_device() -> None:
    """Say what is on the other end of the DFU device."""
    try:
        if not STM32DFU.wait_for_device(DFU_APPEAR_TIMEOUT_S, present=True):
            print(f"   [WARN]  No DFU device appeared within {DFU_APPEAR_TIMEOUT_S:.0f}s.")
            return
        with STM32DFU() as dfu:
            dfu.connect()
            mode, alts = dfu.detect_mode()
            print(f"   DFU mode   : {mode}")
            for alt in alts:
                print(f"     alt      : {alt}")
            if mode == "bootloader":
                print(f"   Bootloader : {dfu.read_version()}")
            elif mode == "rom":
                print("   Bootloader : none (STM32 ROM loader; bare-metal unit)")
    except DFUError as exc:
        print(f"   [WARN]  Could not read the DFU device: {exc}")


def main() -> int:
    args = parse_cli()

    print("[*] Starting MOTION interface ...")
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
            print(f"[FAIL]  {label} not connected (waited {args.timeout:.1f}s).")
            return 1

        if not args.no_confirm:
            answer = input(
                f"Do you want to put the {label} into DFU mode? (y/N): "
            ).strip().lower()
            if answer != "y":
                print("Aborted.")
                return 0

        print(f"[+] Requesting DFU mode from {label} ...")
        try:
            ok = target.enter_dfu()
        except Exception as exc:
            print(f"   [FAIL]  Exception: {exc}")
            return 1

        if not ok:
            print("   [FAIL]  Device did not report success.")
            return 1

        print("   [OK]  DFU mode requested. Waiting for the DFU device ...")
        _report_dfu_device()
        return 0
    finally:
        interface.stop()


if __name__ == "__main__":
    sys.exit(main())
