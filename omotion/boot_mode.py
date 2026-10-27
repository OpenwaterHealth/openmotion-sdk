"""Which bootloader is a device running?

An openmotion board in DFU is either sitting in the STM32 ROM loader (no custom
bootloader installed — a "bare metal" unit) or in openmotion-bl / open-motion-
console-bl (a converted unit). Both enumerate as **0483:df11**, so VID/PID tells
you nothing. The DFU alt-setting layout does:

  ROM loader      four alts — Internal Flash, Option Bytes, OTP Memory, Device
                  Feature — and a fully-writable flash descriptor (``16*128Kg``).
  openmotion-bl   one alt, whose descriptor carries read-only runs
                  (``01*128Ka,04*128Kg,11*128Ka``) because everything outside the
                  application slot is deliberately locked.

That difference decides where a firmware update is written: ``0x08000000`` for a
bare-metal image, ``0x08020000`` for a signed image in the bootloader's slot.
Writing one to the other's address produces a brick, so when the evidence is
ambiguous this module reports :data:`BootMode.UNKNOWN` and callers must refuse to
flash rather than guess.
"""

from __future__ import annotations

import struct
from enum import Enum

__all__ = ["BootMode", "boot_mode_from_dfu", "parse_boot_info", "flash_address_for"]


class BootMode(Enum):
    BARE_METAL = "bare_metal"
    """STM32 ROM loader: no custom bootloader installed."""

    BOOTLOADER = "bootloader"
    """openmotion-bl / open-motion-console-bl."""

    UNKNOWN = "unknown"
    """Could not tell. Never flash on this."""


#: Bare-metal images link and flash at the base of flash.
BARE_METAL_FLASH_ADDRESS = 0x08000000

#: Signed images live in the bootloader's active slot (vectors at +0x400).
BOOTLOADER_SLOT_ADDRESS = 0x08020000

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

# omotion.dfu.classify_alt_names() reads the alt-setting strings of a connected
# DFU device and answers in the bootloader tooling's vocabulary.
_BOOT_MODE_BY_DFU = {
    "rom": BootMode.BARE_METAL,
    "bootloader": BootMode.BOOTLOADER,
}


def boot_mode_from_dfu(dfu_mode: str) -> BootMode:
    """Map :meth:`omotion.dfu.STM32DFU.detect_mode`'s answer to a
    :class:`BootMode`. Anything unrecognised is :data:`BootMode.UNKNOWN`."""
    return _BOOT_MODE_BY_DFU.get(dfu_mode, BootMode.UNKNOWN)


def flash_address_for(mode: BootMode) -> int:
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
