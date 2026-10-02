"""Boot-mode detection from `dfu-util -l` output.

Both the ST ROM loader and openmotion-bl enumerate as 0483:df11, so VID/PID
cannot tell them apart. The DFU alt-setting layout can:

  * ROM loader  -> four alts (Internal Flash, Option Bytes, OTP, Device Feature)
                   and a fully-writable flash descriptor ("16*128Kg")
  * openmotion-bl -> one alt, with read-only runs ("01*128Ka,04*128Kg,11*128Ka")
                     because everything outside the app slot is locked
"""

import pytest

from omotion.boot_mode import BootMode, parse_boot_mode


ROM_LISTING = """dfu-util 0.11

Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
Copyright 2010-2021 Tormod Volden and Stefan Schmidt

Found DFU: [0483:df11] ver=2200, devnum=27, cfg=1, intf=0, path="1-4", alt=3, name="@Device Feature/0xFFFF0000/01*004 e", serial="200364500000"
Found DFU: [0483:df11] ver=2200, devnum=27, cfg=1, intf=0, path="1-4", alt=2, name="@OTP Memory /0x08FFF000/01*1024 e", serial="200364500000"
Found DFU: [0483:df11] ver=2200, devnum=27, cfg=1, intf=0, path="1-4", alt=1, name="@Option Bytes /0x5200201C/01*128 e", serial="200364500000"
Found DFU: [0483:df11] ver=2200, devnum=27, cfg=1, intf=0, path="1-4", alt=0, name="@Internal Flash /0x08000000/16*128Kg", serial="200364500000"
"""

BL_LISTING = """dfu-util 0.11

Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.

Found DFU: [0483:df11] ver=0200, devnum=31, cfg=1, intf=0, path="1-4", alt=0, name="@Internal Flash/0x08000000/01*128Ka,04*128Kg,11*128Ka", serial="OWSENSORBL"
"""

NO_DEVICE = """dfu-util 0.11

Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.

No DFU capable USB device available
"""


def test_rom_loader_listing_is_bare_metal():
    assert parse_boot_mode(ROM_LISTING) is BootMode.BARE_METAL


def test_openmotion_bl_listing_is_bootloader():
    assert parse_boot_mode(BL_LISTING) is BootMode.BOOTLOADER


def test_no_dfu_device_is_unknown():
    assert parse_boot_mode(NO_DEVICE) is BootMode.UNKNOWN


def test_empty_output_is_unknown():
    assert parse_boot_mode("") is BootMode.UNKNOWN


def test_option_bytes_alt_wins_even_if_flash_alt_looks_locked():
    """A ROM listing must never be read as the custom bootloader. The
    ROM-only alt settings are the decisive signal, not the sector layout."""
    weird = ROM_LISTING.replace("16*128Kg", "01*128Ka,15*128Kg")
    assert parse_boot_mode(weird) is BootMode.BARE_METAL


def test_single_fully_writable_alt_is_bare_metal():
    """A fully-writable flash descriptor is decisive for the ROM loader.

    Both bootloaders hard-code their descriptor and both always mark sector 0
    (the bootloader itself) plus the reserved/config region read-only — that
    clamp is the security property they exist to enforce:

        openmotion-bl        01*128Ka,04*128Kg,11*128Ka
        open-motion-console-bl  01*128Ka,08*128Kg,07*128Ka

    So an Internal Flash alt at 0x08000000 with *no* read-only run cannot be an
    openmotion bootloader. This used to report UNKNOWN, which made a bare-metal
    unit unflashable whenever dfu-util listed only the Internal Flash alt
    (openmotion-sdk#197).
    """
    listing = BL_LISTING.replace("01*128Ka,04*128Kg,11*128Ka", "16*128Kg")
    assert parse_boot_mode(listing) is BootMode.BARE_METAL


def test_console_bootloader_listing_is_bootloader():
    """The console bootloader has a bigger app slot than the sensor one, but the
    same read-only clamp, so it must still classify as BOOTLOADER."""
    listing = BL_LISTING.replace("01*128Ka,04*128Kg,11*128Ka",
                                 "01*128Ka,08*128Kg,07*128Ka")
    assert parse_boot_mode(listing) is BootMode.BOOTLOADER


# Exactly what a bench console in the ST ROM loader produced (2 alts, not the 4
# the ROM loader is often documented as exposing) — the case from #197.
ROM_TWO_ALT_LISTING = """dfu-util 0.11

Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.

Found DFU: [0483:df11] ver=0200, devnum=54, cfg=1, intf=0, path="2-1.1.1", alt=1, name="@Option Bytes   /0x5200201C/01*128 e", serial="200364500000"
Found DFU: [0483:df11] ver=0200, devnum=54, cfg=1, intf=0, path="2-1.1.1", alt=0, name="@Internal Flash   /0x08000000/16*128Kg", serial="200364500000"
"""


def test_observed_two_alt_rom_listing_is_bare_metal():
    assert parse_boot_mode(ROM_TWO_ALT_LISTING) is BootMode.BARE_METAL


def test_internal_flash_alt_alone_is_bare_metal():
    """The regression from #197: drop the Option Bytes alt and the remaining
    Internal Flash alt must still classify, or the unit cannot be updated."""
    only_flash = "\n".join(
        line for line in ROM_TWO_ALT_LISTING.splitlines()
        if "Option Bytes" not in line
    )
    assert parse_boot_mode(only_flash) is BootMode.BARE_METAL


def test_internal_flash_alt_with_no_parseable_runs_is_unknown():
    """No access letters at all is still not evidence of anything."""
    listing = BL_LISTING.replace("01*128Ka,04*128Kg,11*128Ka", "wharrgarbl")
    assert parse_boot_mode(listing) is BootMode.UNKNOWN


def test_writable_alt_at_a_different_address_is_unknown():
    """The bare-metal call is anchored to Internal Flash at 0x08000000. A
    writable descriptor somewhere else is not a device we recognise."""
    listing = BL_LISTING.replace("0x08000000/01*128Ka,04*128Kg,11*128Ka",
                                 "0x90000000/16*128Kg")
    assert parse_boot_mode(listing) is BootMode.UNKNOWN


def test_garbage_output_is_unknown():
    assert parse_boot_mode("wharrgarbl\nnot a dfu listing at all\n") is BootMode.UNKNOWN


@pytest.mark.parametrize("mode,expected", [
    (BootMode.BARE_METAL, "0x08000000"),
    (BootMode.BOOTLOADER, "0x08020000"),
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
