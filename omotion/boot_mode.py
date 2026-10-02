"""Which bootloader is a device running?

An openmotion board in DFU is either sitting in the STM32 ROM loader (no custom
bootloader installed — a "bare metal" unit) or in openmotion-bl / open-motion-
console-bl (a converted unit). Both enumerate as **0483:df11**, so VID/PID tells
you nothing. The DFU alt-setting layout does:

  ROM loader      ROM-only alts — Option Bytes, OTP Memory, Device Feature —
                  alongside a fully-writable flash descriptor (``16*128Kg``).
  openmotion-bl   one alt, whose descriptor carries read-only runs
                  (``01*128Ka,04*128Kg,11*128Ka``; the console build uses
                  ``01*128Ka,08*128Kg,07*128Ka``) because everything outside the
                  application slot is deliberately locked.

How many alts the ROM loader actually reports varies — bench consoles have shown
two, not four, and a listing can carry the Internal Flash alt alone. So the
ROM-only alts are checked first but are not required: failing that, the flash
descriptor decides, since only the ROM loader leaves the whole of flash
writable. Both signals are positive evidence; neither infers a mode from the
*absence* of the other.

That difference decides where a firmware update is written: ``0x08000000`` for a
bare-metal image, ``0x08020000`` for a signed image in the bootloader's slot.
Writing one to the other's address produces a brick, so when the evidence is
ambiguous this module reports :data:`BootMode.UNKNOWN` and callers must refuse to
flash rather than guess.
"""

from __future__ import annotations

import re
import struct
from enum import Enum

__all__ = ["BootMode", "parse_boot_mode", "parse_boot_info", "flash_address_for"]


class BootMode(Enum):
    BARE_METAL = "bare_metal"
    """STM32 ROM loader: no custom bootloader installed."""

    BOOTLOADER = "bootloader"
    """openmotion-bl / open-motion-console-bl."""

    UNKNOWN = "unknown"
    """Could not tell. Never flash on this."""


#: Bare-metal images link and flash at the base of flash.
BARE_METAL_FLASH_ADDRESS = "0x08000000"

#: Signed images live in the bootloader's active slot (vectors at +0x400).
BOOTLOADER_SLOT_ADDRESS = "0x08020000"

_FLASH_ADDRESS = {
    BootMode.BARE_METAL: BARE_METAL_FLASH_ADDRESS,
    BootMode.BOOTLOADER: BOOTLOADER_SLOT_ADDRESS,
}

# Runtime vector-table base (SCB->VTOR) each build reports via OW_CMD_BOOT_INFO.
# These are the VECT_TAB_BASE_ADDRESS values system_stm32h7xx.c sets per build.
_VTOR_BARE_METAL = 0x08000000
_VTOR_BOOTLOADER_SLOT = 0x08020400   # slot base 0x08020000 + 0x400 image header

_BOOT_MODE_BY_VTOR = {
    _VTOR_BARE_METAL: BootMode.BARE_METAL,
    _VTOR_BOOTLOADER_SLOT: BootMode.BOOTLOADER,
}

# OW_CMD_BOOT_INFO reply: u8 struct_version; u8 reserved[3]; u32 vtor; u32 flash_base
_BOOT_INFO = struct.Struct("<B3sII")


def parse_boot_info(payload) -> BootMode:
    """Classify an ``OW_CMD_BOOT_INFO`` reply by its reported ``vtor``.

    Never raises. Returns :data:`BootMode.UNKNOWN` for a missing/short/garbled
    reply, or one whose vtor is neither the bare-metal base nor the slot vectors
    — which is also what old firmware effectively yields, since it answers
    OW_UNKNOWN (no payload) to command 0x09. The struct version is not enforced,
    so a future reply that only appends fields still classifies.
    """
    if not payload or len(payload) < _BOOT_INFO.size:
        return BootMode.UNKNOWN
    _ver, _rsvd, vtor, _flash_base = _BOOT_INFO.unpack_from(payload)
    return _BOOT_MODE_BY_VTOR.get(vtor, BootMode.UNKNOWN)

# `Found DFU: [0483:df11] ver=..., ..., alt=0, name="@Internal Flash/...", serial="..."`
_ALT_RE = re.compile(r"\balt=(\d+)\b")
_NAME_RE = re.compile(r'\bname="([^"]*)"')

# DfuSe sector spec: <count>*<size><multiplier><access>, access = 'a'+bitmask
# where bit0=readable, bit1=erasable, bit2=writable. So 'a' is read-only and
# 'g' is read/erase/write.
_SECTOR_RE = re.compile(r"\d+\s*\*\s*\d+\s*[KMB]?\s*([a-g])")

# Alt settings only the ST ROM loader exposes. Presence of any of these is
# decisive: a listing containing them is the ROM loader regardless of what the
# flash descriptor looks like.
_ROM_ONLY_ALTS = ("option bytes", "otp memory", "device feature")

# The main flash alt, present in every mode. Anchored to 0x08000000 so a
# writable descriptor for some other memory can never be read as bare metal.
_INTERNAL_FLASH_RE = re.compile(r"@\s*internal\s+flash\s*/\s*0x08000000\s*/", re.I)


def _alt_entries(dfu_util_output: str) -> list[tuple[int, str]]:
    """Extract ``(alt, name)`` for each `Found DFU:` line."""
    entries: list[tuple[int, str]] = []
    for line in dfu_util_output.splitlines():
        if "found dfu" not in line.lower():
            continue
        alt_m = _ALT_RE.search(line)
        name_m = _NAME_RE.search(line)
        if alt_m is None or name_m is None:
            continue
        entries.append((int(alt_m.group(1)), name_m.group(1)))
    return entries


def parse_boot_mode(dfu_util_output: str) -> BootMode:
    """Classify a ``dfu-util -l`` listing. Never raises."""
    entries = _alt_entries(dfu_util_output or "")
    if not entries:
        return BootMode.UNKNOWN

    if any(marker in name.lower() for _, name in entries for marker in _ROM_ONLY_ALTS):
        return BootMode.BARE_METAL

    # No ROM-only alt in the listing — which happens whenever dfu-util reports
    # just the Internal Flash alt. Fall back to the flash descriptor's access
    # letters, which are decisive on their own: every openmotion bootloader
    # hard-codes read-only runs around its app slot (sector 0 holds the
    # bootloader; the reserved/config sectors hold the anti-rollback floor), so
    #
    #   openmotion-bl           01*128Ka,04*128Kg,11*128Ka
    #   open-motion-console-bl  01*128Ka,08*128Kg,07*128Ka
    #
    # A descriptor with no read-only run at all therefore cannot be a
    # bootloader, and the ROM loader is the only other thing it can be. Before
    # this, such a listing was UNKNOWN and the unit could not be flashed at all
    # (openmotion-sdk#197).
    for _, name in entries:
        if not _INTERNAL_FLASH_RE.search(name):
            continue
        access = _SECTOR_RE.findall(name)
        if not access:
            continue
        if any(a == "a" for a in access):
            return BootMode.BOOTLOADER
        return BootMode.BARE_METAL

    return BootMode.UNKNOWN


def flash_address_for(mode: BootMode) -> str:
    """Address a normal firmware update writes to under ``mode``.

    Raises ``ValueError`` for :data:`BootMode.UNKNOWN` — there is no safe
    default, and picking one is how a device gets bricked.
    """
    try:
        return _FLASH_ADDRESS[mode]
    except KeyError:
        raise ValueError(
            f"cannot choose a flash address for boot mode {mode.value!r}; "
            "refusing to guess"
        ) from None
