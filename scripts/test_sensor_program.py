#!/usr/bin/env python3
"""Update the firmware of a MOTION sensor module over USB DFU.

Uses :class:`omotion.FirmwareUpdater`, which works out what the sensor is
running and flashes accordingly: a signed image into the bootloader's slot, or
a bare-metal image on a unit without a bootloader.

Installing or replacing the bootloader is irreversible over USB and is only
done with --acknowledge-irreversible plus the image the unit needs
(--updater for a bootloader older than 1.2.0, --production for a unit without
one). Bench units only.

Usage
-----
    python scripts/test_sensor_program.py <path_to_bin_file> [--sensor left|right]
"""

import argparse
import sys
import time
from pathlib import Path

from omotion import (
    BootloaderChangeRequired,
    FirmwareKind,
    FirmwareUpdateError,
    FirmwareUpdater,
    MotionInterface,
)
from omotion.dfu import DFUProgress

# Sensors enumerate asynchronously after MotionInterface.start(); how long to
# wait before declaring them absent.
STARTUP_TIMEOUT_S = 20.0
# How long the connection monitor gets to reconnect the sensor once update()
# has seen it back on USB.
REENUM_TIMEOUT_S = 30.0

# update() argument -> the flag that supplies it.
_FLAG_FOR = {
    "updater_image": "--updater",
    "production_image": "--production",
    "acknowledge_irreversible": "--acknowledge-irreversible",
}


class _LiveStatus:
    def __init__(self, *, enabled: bool = True):
        self.enabled = enabled
        self._spinner = "|/-\\"
        self._spinner_index = 0
        self._last_render = 0.0
        self._phase = "working"
        self._percent: int | None = None

    def clear(self) -> None:
        if not self.enabled:
            return
        sys.stderr.write("\r" + (" " * 80) + "\r")
        sys.stderr.flush()

    def update(self, p: DFUProgress) -> None:
        if p.percent is None and p.message:
            # A step announcement or an operator instruction: always shown.
            self.clear()
            print("   [i] ", p.message)
            self._last_render = 0.0
            self._percent = None

        if not self.enabled:
            return

        if p.phase == "erase":
            self._phase = "erasing"
        elif p.phase == "download":
            self._phase = "downloading"
        elif p.phase in ("wait", "power_cycle"):
            self._phase = "waiting"

        if p.percent is not None:
            self._percent = p.percent

        now = time.monotonic()
        if now - self._last_render < 0.1:
            return

        ch = self._spinner[self._spinner_index % len(self._spinner)]
        self._spinner_index += 1
        pct = f" {self._percent:3d}%" if self._percent is not None else ""
        sys.stderr.write(f"\r   ... {self._phase} {ch}{pct}  ({p.elapsed_s:0.1f}s)")
        sys.stderr.flush()
        self._last_render = now


def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Update the firmware of a MOTION sensor module over USB DFU."
    )
    parser.add_argument(
        "bin_file",
        type=Path,
        help="Application image: a signed .bin for a unit with the bootloader, "
             "a bare-metal .bin for a unit without one.",
    )
    parser.add_argument(
        "--no-confirm",
        action="store_true",
        help="Skip the interactive confirmation before entering DFU mode.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for DFU device to appear.",
    )
    parser.add_argument(
        "--updater",
        type=Path,
        default=None,
        help="Signed bootloader-updater image, for a sensor whose bootloader is "
             "older than 1.2.0. Used only with --acknowledge-irreversible.",
    )
    parser.add_argument(
        "--production",
        type=Path,
        default=None,
        help="Production image (bootloader + application), for a sensor without "
             "a bootloader. Used only with --acknowledge-irreversible.",
    )
    parser.add_argument(
        "--acknowledge-irreversible",
        action="store_true",
        help="Allow installing or replacing the bootloader. Cannot be undone over "
             "USB; the unit must not lose power while it runs.",
    )
    parser.add_argument(
        "--no-spinner",
        action="store_true",
        help="Disable live status line.",
    )
    parser.add_argument(
        "--sensor",
        choices=("left", "right"),
        default=None,
        help="Select which sensor to target (left or right). If omitted, the first present sensor is used.",
    )
    return parser.parse_args()


def _wait_for_sensor(interface, side: str | None, timeout_s: float) -> bool:
    """Block until the requested sensor side is connected, or ``timeout_s``.

    ``side`` is "left", "right", or None for either side. Unlike
    ``MotionInterface.wait_for_ready(sensors=1)``, a specific side is not
    satisfied by the other module being present.
    """
    deadline = time.monotonic() + timeout_s
    while True:
        _console, left, right = interface.is_device_connected()
        if side == "left":
            if left:
                return True
        elif side == "right":
            if right:
                return True
        elif left or right:
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.2)


def main() -> int:
    args = parse_cli()

    print("[*] Starting MOTION interface ...")
    interface = MotionInterface()
    interface.start()

    try:
        # Enumeration is asynchronous - checking straight after start() races
        # the connection monitor and reports healthy sensors as absent.
        want = f"the {args.sensor.upper()} sensor" if args.sensor else "a sensor module"
        print(f"[*] Waiting up to {STARTUP_TIMEOUT_S:.0f}s for {want} to connect ...")
        _wait_for_sensor(interface, args.sensor, STARTUP_TIMEOUT_S)

        _console_connected, left_connected, right_connected = interface.is_device_connected()

        # Ensure at least one sensor module is present.
        if not (left_connected or right_connected):
            print("[FAIL]  No sensor modules connected - cannot continue.")
            return 1

        selected_sensor = None
        selected_side = None
        # If the user requested a specific side, honor it (fail if not present).
        if args.sensor == "left":
            if not left_connected:
                print("[FAIL]  LEFT sensor not connected - cannot continue.")
                return 1
            print("Running firmware update on LEFT sensor")
            selected_sensor, selected_side = interface.left, "left"
        elif args.sensor == "right":
            if not right_connected:
                print("[FAIL]  RIGHT sensor not connected - cannot continue.")
                return 1
            print("Running firmware update on RIGHT sensor")
            selected_sensor, selected_side = interface.right, "right"
        else:
            # Auto-select: prefer left if present, otherwise right.
            if left_connected:
                print("Running firmware update on LEFT sensor (auto-selected)")
                selected_sensor, selected_side = interface.left, "left"
            elif right_connected:
                print("Running firmware update on RIGHT sensor (auto-selected)")
                selected_sensor, selected_side = interface.right, "right"

        if selected_sensor is None:
            print("[FAIL]  Sensor module not connected - cannot continue.")
            return 1

        status = _LiveStatus(enabled=not args.no_spinner)

        if not args.no_confirm:
            answer = input("Do you really want to put the board into DFU mode? (y/N): ").strip().lower()
            if answer != "y":
                print("Aborted by user.")
                return 0

        print("\n[+] Updating the Sensor module ...")
        updater = FirmwareUpdater(dfu_wait_timeout_s=args.timeout)
        try:
            result = updater.update(
                selected_sensor,
                args.bin_file,
                status.update,
                kind=FirmwareKind.SENSOR,
                updater_image=args.updater,
                production_image=args.production,
                acknowledge_irreversible=args.acknowledge_irreversible,
            )
        except BootloaderChangeRequired as exc:
            status.clear()
            print(f"[FAIL]  {exc}")
            print("        Missing: " + ", ".join(_FLAG_FOR.get(n, n) for n in exc.needs))
            return 1
        except FirmwareUpdateError as exc:
            status.clear()
            print(f"[FAIL]  {exc}")
            return 1

        status.clear()
        print(f"   [OK]  {result.message}.")
        if updater.last_bootloader_version:
            print(f"   [i]  Bootloader {updater.last_bootloader_version}.")

        side_label = selected_side.upper()
        if result.needs_power_cycle:
            print(f"   [WARN]  The {side_label} sensor did not come back on USB.")
            print("           Power-cycle the DUT mains supply (known STM32 ROM-loader")
            print("           quirk), then check the device re-appears.")
            return 0

        print(f"\n[+] Waiting up to {REENUM_TIMEOUT_S:.0f}s for the {side_label} sensor to reconnect ...")
        if _wait_for_sensor(interface, selected_side, REENUM_TIMEOUT_S):
            try:
                version = selected_sensor.get_version()
                print(f"   [OK]  {side_label} sensor is back on USB, firmware version {version}.")
            except Exception as exc:
                print(f"   [WARN]  {side_label} sensor reconnected but get_version() failed: {exc}")
        else:
            print(f"   [WARN]  {side_label} sensor did not reconnect within {REENUM_TIMEOUT_S:.0f}s.")

        print("\n[OK]  All done! The STM32 should now be running the newly-flashed firmware.\n")
        return 0
    finally:
        interface.stop()


if __name__ == "__main__":
    sys.exit(main())
