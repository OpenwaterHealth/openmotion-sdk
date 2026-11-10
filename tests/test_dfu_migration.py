"""The migration helpers: bootloader version parsing and production-image
consistency. Version vectors are the ones in the bootloader repository's
py-tools/tests/test_migrate_tools.py. Needs no hardware.
"""

import struct

import pytest

from _dfu_fakes import make_key, make_production, sign_image
from omotion.dfu.image import ImageVerificationError
from omotion.dfu.migration import (
    BOOTLOADER_LEN,
    FLASH_BASE,
    MIN_BOOTLOADER,
    ROM_ERASE_END,
    check_production_image,
    parse_bl_version,
)

pytestmark = pytest.mark.unit


def test_bootloader_versions_parse_to_tuples():
    assert parse_bl_version("1.0.0") == (1, 0, 0)
    assert parse_bl_version("1.2.0-rc.1") == (1, 2, 0)
    assert parse_bl_version("v1.2.0-3-gabcdef-dirty") == (1, 2, 0)
    assert parse_bl_version("abcdef1") is None
    assert parse_bl_version("") is None
    assert parse_bl_version("1.0.0") < MIN_BOOTLOADER
    assert parse_bl_version("1.1.0") < MIN_BOOTLOADER
    assert not parse_bl_version("1.2.0-rc.1") < MIN_BOOTLOADER


@pytest.fixture(scope="module")
def signed():
    priv, _ = make_key()
    return sign_image(priv)


def test_consistent_production_image_passes_and_returns_the_bootloader(signed):
    production = make_production(signed)
    updater = b"\x00" * 64 + production[:BOOTLOADER_LEN].rstrip(b"\xff") + b"\x00" * 64

    bl = check_production_image(production, signed=signed, updater=updater)

    assert bl == production[:BOOTLOADER_LEN].rstrip(b"\xff")


def test_production_image_without_an_application_is_rejected():
    with pytest.raises(ImageVerificationError):
        check_production_image(b"\x00" * BOOTLOADER_LEN)


def test_production_image_larger_than_the_erased_range_is_rejected(signed):
    """Upstream erases sectors 0-5 and then writes without erasing: anything
    beyond would be programmed over stale flash."""
    production = make_production(signed)
    production += b"\xff" * (ROM_ERASE_END - FLASH_BASE - len(production) + 1)
    with pytest.raises(ImageVerificationError, match="larger"):
        check_production_image(production)


def test_production_image_with_a_different_application_is_rejected(signed):
    priv, _ = make_key()
    other = sign_image(priv, body=b"\x77" * 2048)
    with pytest.raises(ImageVerificationError, match="signed application"):
        check_production_image(make_production(signed), signed=other)


@pytest.mark.parametrize("sp, reset", [
    (0x00000000, 0x08000401),       # no stack
    (0x24080000, 0x08020401),       # reset vector in the application slot
    (0x24080000, 0xFFFFFFFF),       # erased
])
def test_production_image_with_an_implausible_vector_table_is_rejected(signed, sp, reset):
    bootloader = struct.pack("<II", sp, reset) + b"\xb1" * 256
    with pytest.raises(ImageVerificationError, match="looks wrong"):
        check_production_image(make_production(signed, bootloader=bootloader))


def test_updater_embedding_another_bootloader_is_rejected(signed):
    with pytest.raises(ImageVerificationError, match="updater"):
        check_production_image(make_production(signed), updater=b"\x00" * 4096)
