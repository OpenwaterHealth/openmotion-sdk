#!/usr/bin/env python3
"""Update the firmware of one MOTION device (console, left or right sensor).

Uses :class:`omotion.FirmwareUpdater`, which works out what the device is
running once it is in DFU and flashes accordingly:

    bootloader 1.2.0 or newer    the signed application
    older bootloader             the bootloader updater, then the signed
                                 application (needs --acknowledge-irreversible)
    no bootloader (bare metal)   the production image: bootloader + application
                                 (needs --acknowledge-irreversible)

The images come from a release folder, or are named one by one:

    python scripts/update_firmware.py console --release-dir D:/release
    python scripts/update_firmware.py right --release-dir D:/release --acknowledge-irreversible
    python scripts/update_firmware.py left --app sensor-signed.bin --updater sensor-updater.bin

Without --acknowledge-irreversible only the application is installed; a device
that needs its bootloader installed or replaced is left as it was and the
script says what is missing. Replacing or installing a bootloader cannot be
undone over USB, and the device must stay powered and plugged in while it
runs. Bench units only.
"""

import argparse
import logging
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
from omotion.dfu import STM32DFU, DFUProgress

# Devices enumerate asynchronously after MotionInterface.start(); how long to
# wait before declaring the target absent.
STARTUP_TIMEOUT_S = 20.0
# How long the connection monitor gets to reconnect the device once update()
# has seen it back on USB.
RECONNECT_TIMEOUT_S = 45.0

# What each image is called in a release folder: (role, word in the file name).
_ROLES = (("app", "signed"), ("updater", "updater"), ("production", "production"))

# update() argument -> the flag that supplies it.
_FLAG_FOR = {
    "updater_image": "--updater (or a release folder that has one)",
    "production_image": "--production (or a release folder that has one)",
    "acknowledge_irreversible": "--acknowledge-irreversible",
}

# ST-LINK debug probes (VID 0483). One wired to the board can disturb a
# bootloader rewrite, so the irreversible paths refuse to start with one on USB.
_STLINK_PIDS = frozenset(
    {0x3744, 0x3748, 0x374A, 0x374B, 0x374D, 0x374E, 0x374F, 0x3752, 0x3753, 0x3754, 0x3757}
)


def product_of(device: str) -> str:
    """ "console" for the console, "sensor" for either sensor."""
    return "console" if device == "console" else "sensor"


def find_release_images(release_dir: Path, product: str) -> dict:
    """The images for ``product`` in ``release_dir``: role -> path.

    A file belongs to a role when its name carries the product and the role's
    word (signed / updater / production). Raises ``ValueError`` when a role
    matches more than one file: guessing between two images is not acceptable
    for a step that cannot be undone.
    """
    found = {}
    for role, word in _ROLES:
        matches = sorted(
            p for p in Path(release_dir).glob("*.bin")
            if product in p.name.lower() and word in p.name.lower()
            # An updater is a signed image too; it is not the application.
            and not (role == "app" and "updater" in p.name.lower())
        )
        if len(matches) > 1:
            raise ValueError(
                f"{release_dir} holds {len(matches)} {product} {word} images: "
                + ", ".join(p.name for p in matches)
            )
        if matches:
            found[role] = matches[0]
    return found


def stlink_probes() -> int:
    """How many ST-LINK probes are on USB (0 if USB cannot be enumerated)."""
    try:
        import usb.core

        from omotion.usb_backend import get_libusb1_backend

        return sum(
            1 for d in usb.core.find(find_all=True, idVendor=0x0483,
                                     backend=get_libusb1_backend()) or []
            if d.idProduct in _STLINK_PIDS
        )
    except Exception:
        return 0


class _Status:
    """Prints step announcements in full and progress in 25% steps."""

    def __init__(self, label: str):
        self._label = label
        self._last = None

    def update(self, p: DFUProgress) -> None:
        if p.percent is None:
            if p.message:
                print(f"   [{p.elapsed_s:6.1f}s]  {p.message}", flush=True)
            return
        key = (p.phase, p.percent // 25)
        if key != self._last:
            self._last = key
            print(f"   [{p.elapsed_s:6.1f}s]  {self._label}: {p.phase} {p.percent}%", flush=True)


def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Update the firmware of one MOTION device over USB DFU."
    )
    parser.add_argument(
        "device",
        choices=("console", "left", "right"),
        help="Which device to update: console, left sensor, or right sensor.",
    )
    parser.add_argument(
        "--release-dir",
        type=Path,
        default=None,
        help="Folder holding the release images. The signed application, the "
             "bootloader updater and the production image for the device are "
             "picked by name (product plus signed / updater / production).",
    )
    parser.add_argument(
        "--app",
        type=Path,
        default=None,
        help="Application image: a signed .bin for a device with the bootloader, "
             "a bare-metal .bin for one without. Overrides --release-dir.",
    )
    parser.add_argument(
        "--updater",
        type=Path,
        default=None,
        help="Signed bootloader-updater image, for a device whose bootloader is "
             "older than 1.2.0. Overrides --release-dir.",
    )
    parser.add_argument(
        "--production",
        type=Path,
        default=None,
        help="Production image (bootloader + application), for a device without "
             "a bootloader. Overrides --release-dir.",
    )
    parser.add_argument(
        "--acknowledge-irreversible",
        action="store_true",
        help="Allow installing or replacing the bootloader. Cannot be undone over "
             "USB; the device must not lose power while it runs.",
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
        help="Seconds to wait for the DFU device to appear.",
    )
    return parser.parse_args()


def _wait_connected(handle, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if handle.is_connected():
            return True
        time.sleep(0.25)
    return handle.is_connected()


def _describe(handle) -> str:
    try:
        return f"firmware {handle.get_version()}, boot mode {handle.get_boot_mode().value}"
    except Exception as exc:
        return f"could not be read ({exc})"


def main() -> int:
    args = parse_cli()
    product = product_of(args.device)
    label = "console" if args.device == "console" else f"{args.device} sensor"

    images = {}
    if args.release_dir is not None:
        try:
            images = find_release_images(args.release_dir, product)
        except ValueError as exc:
            print(f"[FAIL]  {exc}")
            return 1
    for role in ("app", "updater", "production"):
        if getattr(args, role) is not None:
            images[role] = getattr(args, role)
    if "app" not in images:
        print("[FAIL]  No application image: give --app, or a --release-dir that holds "
              f"a {product} signed image.")
        return 1

    print(f"[*] Updating the {label}")
    for role, _word in _ROLES:
        print(f"      {role:10s}: {images.get(role, '-')}")
    print(f"      bootloader changes allowed: {'YES' if args.acknowledge_irreversible else 'no'}")

    if args.acknowledge_irreversible and stlink_probes():
        print("[FAIL]  An ST-LINK probe is on USB. Unplug it before a bootloader can be "
              "installed or replaced.")
        return 1
    try:
        if STM32DFU.list_devices():
            print("[FAIL]  A DFU device is already on USB, so the device to update would "
                  "be ambiguous. Unplug or power-cycle it first.")
            return 1
    except Exception as exc:
        print(f"[FAIL]  Cannot enumerate USB: {exc}")
        return 1

    # The connection monitor reports every device that drops off USB while the
    # target (and, for the console, both sensors with it) restarts.
    logging.getLogger().setLevel(logging.ERROR)

    print("[*] Starting MOTION interface ...")
    interface = MotionInterface()
    interface.start()
    try:
        handle = getattr(interface, args.device)
        print(f"[*] Waiting up to {STARTUP_TIMEOUT_S:.0f}s for the {label} to connect ...")
        if not _wait_connected(handle, STARTUP_TIMEOUT_S):
            print(f"[FAIL]  The {label} is not connected - cannot continue.")
            return 1
        print(f"   [i]  Before: {_describe(handle)}")

        if not args.no_confirm:
            question = f"Put the {label} into DFU mode and update it? (y/N): "
            if input(question).strip().lower() != "y":
                print("Aborted by user.")
                return 0

        updater = FirmwareUpdater(dfu_wait_timeout_s=args.timeout)
        started = time.monotonic()
        try:
            result = updater.update(
                handle,
                images["app"],
                _Status(label).update,
                kind=FirmwareKind(product),
                updater_image=images.get("updater"),
                production_image=images.get("production"),
                acknowledge_irreversible=args.acknowledge_irreversible,
            )
        except BootloaderChangeRequired as exc:
            print(f"[FAIL]  {exc}")
            print("        Missing: " + ", ".join(_FLAG_FOR.get(n, n) for n in exc.needs))
            return 1
        except FirmwareUpdateError as exc:
            print(f"[FAIL]  {exc}")
            if updater.last_boot_mode is not None:
                print(f"        Device was found in {updater.last_boot_mode.value} mode"
                      + (f", bootloader {updater.last_bootloader_version}"
                         if updater.last_bootloader_version else "") + ".")
            return 1

        print(f"   [OK]  {result.message} ({time.monotonic() - started:.0f}s).")
        if result.bootloader_version_before or result.bootloader_version_after:
            print(f"   [i]  Bootloader: {result.bootloader_version_before or 'none'} -> "
                  f"{result.bootloader_version_after or 'installed'}")
        if result.needs_power_cycle:
            print(f"   [WARN]  The {label} did not come back on USB. Power-cycle it, then")
            print("           check that it re-appears.")
            return 0

        print(f"[*] Waiting up to {RECONNECT_TIMEOUT_S:.0f}s for the {label} to reconnect ...")
        if _wait_connected(handle, RECONNECT_TIMEOUT_S):
            time.sleep(1.0)
            print(f"   [i]  After: {_describe(handle)}")
        else:
            print(f"   [WARN]  The {label} did not reconnect within {RECONNECT_TIMEOUT_S:.0f}s.")
        return 0
    finally:
        interface.stop()


if __name__ == "__main__":
    sys.exit(main())
