"""Boot-mode detection from a DFU device's alt-setting strings.

Both the ST ROM loader and openmotion-bl enumerate as 0483:df11, so VID/PID
cannot tell them apart. The DFU alt-setting layout can:

  * ROM loader  -> four alts (Internal Flash, Option Bytes, OTP, Device Feature)
                   and a fully-writable flash descriptor ("16*128Kg")
  * openmotion-bl -> one alt, with read-only runs ("01*128Ka,04*128Kg,11*128Ka")
                     because everything outside the app slot is locked
"""

import pytest

from omotion.boot_mode import BootMode, boot_mode_from_dfu
from omotion.dfu import classify_alt_names


ROM_ALTS = [
    "@Internal Flash /0x08000000/16*128Kg",
    "@Option Bytes /0x5200201C/01*128 e",
    "@OTP Memory /0x08FFF000/01*1024 e",
    "@Device Feature/0xFFFF0000/01*004 e",
]

BL_ALTS = ["@Internal Flash/0x08000000/01*128Ka,04*128Kg,11*128Ka"]


def _mode(alts):
    return boot_mode_from_dfu(classify_alt_names(alts))


def test_rom_loader_alts_are_bare_metal():
    assert _mode(ROM_ALTS) is BootMode.BARE_METAL


def test_openmotion_bl_alt_is_bootloader():
    assert _mode(BL_ALTS) is BootMode.BOOTLOADER


def test_no_alt_settings_is_unknown():
    assert _mode([]) is BootMode.UNKNOWN
    assert _mode(None) is BootMode.UNKNOWN


def test_unreadable_alt_strings_are_unknown():
    """A device whose string descriptors cannot be read yields empty names."""
    assert _mode(["", "", "", ""]) is BootMode.UNKNOWN


def test_option_bytes_alt_wins_even_if_flash_alt_looks_locked():
    """A ROM listing must never be read as the custom bootloader. The
    ROM-only alt settings are the decisive signal, not the sector layout."""
    weird = [ROM_ALTS[0].replace("16*128Kg", "01*128Ka,15*128Kg")] + ROM_ALTS[1:]
    assert _mode(weird) is BootMode.BARE_METAL


def test_single_fully_writable_alt_is_unknown_not_bootloader():
    """One alt with no read-only run is not openmotion-bl. Refuse to guess
    rather than mis-detect and write to the wrong address."""
    assert _mode(["@Internal Flash/0x08000000/16*128Kg"]) is BootMode.UNKNOWN


def test_garbage_alt_names_are_unknown():
    assert _mode(["wharrgarbl", "not a dfu descriptor at all"]) is BootMode.UNKNOWN


def test_unrecognised_dfu_mode_string_is_unknown():
    assert boot_mode_from_dfu("something else") is BootMode.UNKNOWN


@pytest.mark.parametrize("mode,expected", [
    (BootMode.BARE_METAL, 0x08000000),
    (BootMode.BOOTLOADER, 0x08020000),
])
def test_flash_address_for_mode(mode, expected):
    """The whole point of detection: which address a normal update writes to."""
    from omotion.boot_mode import flash_address_for

    assert flash_address_for(mode) == expected


def test_flash_address_for_unknown_mode_raises():
    from omotion.boot_mode import flash_address_for

    with pytest.raises(ValueError):
        flash_address_for(BootMode.UNKNOWN)


# ---------------------------------------------------------------------------
# parse_boot_info: OW_CMD_BOOT_INFO reply -> BootMode, over normal comms
# (no DFU cycle). Payload is packed little-endian:
#   u8 struct_version=1; u8 reserved[3]; u32 vtor; u32 flash_base
# ---------------------------------------------------------------------------
import struct

from omotion.boot_mode import parse_boot_info


def _payload(vtor, *, version=1, flash_base=None):
    return struct.pack("<B3sII", version, b"\x00\x00\x00", vtor,
                       vtor if flash_base is None else flash_base)


def test_boot_info_vtor_at_flash_base_is_bare_metal():
    assert parse_boot_info(_payload(0x08000000)) is BootMode.BARE_METAL


def test_boot_info_vtor_at_slot_vectors_is_bootloader():
    assert parse_boot_info(_payload(0x08020400)) is BootMode.BOOTLOADER


def test_boot_info_unrecognised_vtor_is_unknown():
    assert parse_boot_info(_payload(0x24000000)) is BootMode.UNKNOWN  # RAM_D1, nonsense


def test_boot_info_too_short_is_unknown():
    assert parse_boot_info(b"\x01\x00\x00") is BootMode.UNKNOWN


def test_boot_info_empty_is_unknown():
    assert parse_boot_info(b"") is BootMode.UNKNOWN
    assert parse_boot_info(None) is BootMode.UNKNOWN


def test_boot_info_ignores_flash_base_and_uses_vtor():
    # A future firmware might report a distinct flash_base; classification is on vtor.
    assert parse_boot_info(_payload(0x08000000, flash_base=0x08000000)) is BootMode.BARE_METAL
    assert parse_boot_info(_payload(0x08020400, flash_base=0x08020000)) is BootMode.BOOTLOADER


def test_boot_info_unknown_struct_version_still_classifies_by_vtor():
    # Be lenient on the struct version so a v2 that only appends fields still works.
    assert parse_boot_info(_payload(0x08020400, version=2)) is BootMode.BOOTLOADER
