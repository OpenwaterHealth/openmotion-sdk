"""The flash sequences for each state a unit can be found in.

Ported from open-motion-console-bl ``py-tools/migrate.py`` and the downgrade
pre-check of ``py-tools/flash_firmware.py`` (tag 1.2.0, commit 34178cd), the
tooling the bootloader 1.2.0 migration was bench-tested with on 2026-10-08. See
PROVENANCE.txt. The step order, erase ranges and timeouts are the tested ones;
do not change them without a bench run.

Starting states:

  bootloader 1.0.x / 1.1.x (old key)  erase slot -> flash the signed UPDATER image
                                      (old key) -> the updater rewrites sector 0
                                      with bootloader 1.2.0 and resets into its
                                      DFU -> flash the signed APP (new key).
  bootloader 1.2.x                    flash the signed APP only.
  STM32 ROM loader (bare-metal unit)  erase sectors 0-5, write the production
                                      image (bootloader + app) at 0x08000000.

The bare-metal application update (:func:`install_bare_metal_app`) is the SDK's
own, not part of migrate.py. It verifies the write before it leaves DFU.

These functions only drive an already-connected :class:`STM32DFU`. Deciding
which one applies, verifying the images and asking for consent is
:class:`omotion.firmware_update.FirmwareUpdater`'s job.
"""

from __future__ import annotations

import re
import time

from omotion.dfu.errors import BootloaderVersionError
from omotion.dfu.image import ImageVerificationError, read_header_version
from omotion.dfu.progress import ProgressCallback, progress_adapter

__all__ = [
    "SLOT_ADDR",
    "BOOTLOADER_LEN",
    "FLASH_BASE",
    "ROM_ERASE_END",
    "MIN_BOOTLOADER",
    "T_DFU_APPEAR",
    "T_UPDATER_LEAVE",
    "T_UPDATER_CYCLE",
    "T_APP_APPEAR",
    "T_POWER_CYCLE",
    "parse_bl_version",
    "check_production_image",
    "installed_fw_version",
    "install_production_via_rom",
    "install_bare_metal_app",
    "write_bare_metal_app",
    "leave_rom",
    "run_updater",
    "wait_for_new_bootloader",
    "install_signed_app",
]

SLOT_ADDR      = 0x08020000
BOOTLOADER_LEN = 0x00020000           # sector 0
FLASH_BASE     = 0x08000000
ROM_ERASE_END  = 0x080C0000           # sectors 0..5: bootloader, slot, floor. Never sector 15 (user config).

#: First bootloader that accepts new-key images and needs no updater.
MIN_BOOTLOADER = (1, 2, 0)

T_DFU_APPEAR     = 20.0               # app -> bootloader DFU after OW_CMD_DFU
T_UPDATER_LEAVE  = 15.0               # old bootloader resets into the updater after the download
T_UPDATER_CYCLE  = 150.0              # 1.0.0 verifies updater, updater rewrites sector 0, 1.2.0 first boot applies
                                      # option bytes (reset) and enumerates DFU
T_APP_APPEAR     = 90.0               # bootloader verifies the app, launches it, the app enumerates
T_POWER_CYCLE    = 300.0              # ROM path on a sensor module: operator power-cycles the unit by hand

_VER_RE = re.compile(r"^v?(\d+)\.(\d+)\.(\d+)")


def parse_bl_version(s: str):
    """'1.2.0-rc.1-3-gabc-dirty' -> (1, 2, 0); None if unparsable."""
    m = _VER_RE.match((s or "").strip())
    return tuple(int(x) for x in m.groups()) if m else None


def check_production_image(production: bytes, signed: bytes | None = None,
                           updater: bytes | None = None) -> bytes:
    """Sanity-check a production image (bootloader + signed app); returns its
    bootloader part.

    The bootloader part carries no signature the host can check, so this is a
    consistency check: the image has the right shape, it contains ``signed`` at
    the slot offset, and it embeds the same bootloader as ``updater``. Raises
    :class:`ImageVerificationError`.
    """
    if len(production) <= BOOTLOADER_LEN:
        raise ImageVerificationError("production image is not longer than the bootloader sector")
    if len(production) > ROM_ERASE_END - FLASH_BASE:
        # install_production_via_rom() erases FLASH_BASE..ROM_ERASE_END only.
        raise ImageVerificationError(
            f"production image is {len(production)} bytes, larger than the "
            f"{ROM_ERASE_END - FLASH_BASE} bytes erased for it"
        )
    if signed is not None and production[BOOTLOADER_LEN:BOOTLOADER_LEN + len(signed)] != signed:
        raise ImageVerificationError(
            "production image does not contain the given signed application at 0x08020000"
        )
    bl = production[:BOOTLOADER_LEN].rstrip(b"\xff")
    sp, rv = int.from_bytes(bl[0:4], "little"), int.from_bytes(bl[4:8], "little")
    if not (0x20000000 <= sp <= 0x20020000 or 0x24000000 <= sp <= 0x24080000) \
            or not (FLASH_BASE < rv < FLASH_BASE + BOOTLOADER_LEN):
        raise ImageVerificationError(
            f"production image bootloader part looks wrong (sp={sp:#x} reset={rv:#x})"
        )
    if updater is not None and updater.find(bl) < 0:
        raise ImageVerificationError(
            "the updater does not embed the same bootloader as the production image"
        )
    return bl


def installed_fw_version(dfu) -> int:
    """Raw FwVersion of the image in the slot, or 0 if the slot has no header.

    Downgrade pre-flight: the bootloader destroys a downloaded image that is
    older than the one it replaced, so the caller refuses before the erase.
    """
    return read_header_version(dfu.upload(SLOT_ADDR, 8))


def install_production_via_rom(dfu, production: bytes,
                               progress: ProgressCallback | None = None) -> None:
    """ROM loader: write the production image (bootloader + application)."""
    t0 = time.monotonic()
    dfu.set_rom_mode()
    dfu.erase_range(FLASH_BASE, ROM_ERASE_END, progress_cb=progress_adapter(progress, "erase", t0=t0))
    dfu.download(FLASH_BASE, production, progress_cb=progress_adapter(progress, "download", t0=t0),
                 pre_erased=True)
    # The zero-length DNLOAD that ends download() is the DfuSe manifestation; the
    # ROM loader jumps to the image right there (seen on the bench 2026-10-08), so
    # the device is normally gone before this explicit leave. Keep it for a ROM
    # that waits for one, tolerate its absence.
    try:
        dfu.leave_dfu()
    except Exception:
        pass


def write_bare_metal_app(dfu, image: bytes,
                         progress: ProgressCallback | None = None) -> None:
    """ROM loader: write a bare-metal application at the base of flash and read
    it back. Only the sectors the image covers are erased.

    Does not leave DFU, whether it returns or raises. The erase destroys the
    image in sector 0, the unit has no other way back into DFU, and the ROM
    loader jumps into flash as soon as it manifests. So nothing may start the
    image until the read-back matches. A failed write leaves the unit in the
    ROM loader, where the write can be redone. Raises :class:`DFUVerifyError`
    when the read-back differs, :class:`DFUError` for anything else.
    """
    t0 = time.monotonic()
    dfu.set_rom_mode()
    dfu.download(FLASH_BASE, image, progress_cb=progress_adapter(progress, "download", t0=t0),
                 manifest=False)
    dfu.verify(FLASH_BASE, image, progress_cb=progress_adapter(progress, "verify", t0=t0))


def leave_rom(dfu) -> None:
    """ROM loader: start the image at the base of flash. Best effort: a unit
    that does not hear the request stays in DFU."""
    try:
        dfu.leave_dfu()
    except Exception:
        pass


def install_bare_metal_app(dfu, image: bytes,
                           progress: ProgressCallback | None = None) -> None:
    """ROM loader: write a bare-metal application at the base of flash, verify
    it, then start it.

    Not part of migrate.py: this is the SDK's long-standing update for units
    without a bootloader, formerly ``dfu-util -s 0x08000000:leave -D``. A failed
    write or read-back raises with the unit still in the ROM loader; see
    :func:`write_bare_metal_app`. :class:`omotion.firmware_update.FirmwareUpdater`
    calls the two halves itself so that it can redo the write.
    """
    write_bare_metal_app(dfu, image, progress)
    leave_rom(dfu)


def run_updater(dfu, updater: bytes, progress: ProgressCallback | None = None) -> None:
    """Old bootloader: install the bootloader-updater image in the slot.

    The unit resets and runs the updater, which rewrites sector 0. It must not
    lose power or USB until :func:`wait_for_new_bootloader` returns.
    """
    t0 = time.monotonic()
    # 1.0.0 does not clean stale tails, so the whole slot is erased first.
    dfu.erase_all(progress_cb=progress_adapter(progress, "erase", t0=t0))
    dfu.download(SLOT_ADDR, updater, progress_cb=progress_adapter(progress, "download", t0=t0),
                 pre_erased=True)


def wait_for_new_bootloader(wait_for_device, serial=None) -> bool:
    """After :func:`run_updater`: wait for the unit to leave, then for the new
    bootloader's DFU to enumerate. ``wait_for_device`` is
    :meth:`STM32DFU.wait_for_device`. Returns False if DFU never came back."""
    wait_for_device(T_UPDATER_LEAVE, present=False, serial=serial)
    return wait_for_device(T_UPDATER_CYCLE, present=True, serial=serial)


def install_signed_app(dfu, signed: bytes, progress: ProgressCallback | None = None) -> str:
    """Bootloader 1.2.0 or newer: install the signed application (new key).

    Returns the bootloader version string. Raises
    :class:`BootloaderVersionError` if the bootloader is older, and lets the
    :class:`DFUError` of a refused download through: the bootloader checks the
    signed header before writing (wrong key, or FwVersion below the header still
    in the slot).
    """
    ver_str = dfu.read_version()
    ver = parse_bl_version(ver_str)
    if ver is None or ver < MIN_BOOTLOADER:
        raise BootloaderVersionError(
            f"bootloader reports {ver_str!r}, not 1.2.0 or newer", version=ver_str
        )
    dfu.download(SLOT_ADDR, signed, progress_cb=progress_adapter(progress, "download"))
    return ver_str
