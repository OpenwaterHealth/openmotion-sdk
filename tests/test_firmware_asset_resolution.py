"""Which release asset gets flashed, for a given device mode and release era.

CI renamed every firmware asset on 2026-07-09 when the bootloader-slot build was
added. Releases before that carry a single `motion-{sensor,console}-fw.bin`;
releases after carry separate bare-metal, signed and production images. Both eras
must keep working.
"""

import pytest

from omotion.boot_mode import BootMode
from omotion.firmware_update import (
    FirmwareKind,
    UnsupportedReleaseError,
    candidate_assets,
    production_asset,
    resolve_asset,
)

LEGACY_SENSOR = ["motion-sensor-fw.bin", "motion-sensor-fw-raw.bin"]
LEGACY_CONSOLE = ["motion-console-fw.bin"]

NEW_SENSOR = [
    "motion-sensor-fw-baremetal.bin",
    "motion-sensor-fw-baremetal-fpga.bin",
    "motion-sensor-fw-signed.bin",
    "motion-sensor-production.bin",
]
NEW_CONSOLE = [
    "motion-console-fw-baremetal.bin",
    "motion-console-fw-signed.bin",
    "motion-console-production.bin",
]


@pytest.mark.parametrize("kind,mode,names,expected", [
    # Bare metal, new-style release. Sensor takes the FPGA-merged image so the
    # bitstream always matches the firmware, matching pre-rename behaviour.
    (FirmwareKind.SENSOR, BootMode.BARE_METAL, NEW_SENSOR,
     "motion-sensor-fw-baremetal-fpga.bin"),
    # Console has no merged variant.
    (FirmwareKind.CONSOLE, BootMode.BARE_METAL, NEW_CONSOLE,
     "motion-console-fw-baremetal.bin"),
    # Bare metal, legacy release.
    (FirmwareKind.SENSOR, BootMode.BARE_METAL, LEGACY_SENSOR, "motion-sensor-fw.bin"),
    (FirmwareKind.CONSOLE, BootMode.BARE_METAL, LEGACY_CONSOLE, "motion-console-fw.bin"),
    # Bootloader units take the signed slot image.
    (FirmwareKind.SENSOR, BootMode.BOOTLOADER, NEW_SENSOR, "motion-sensor-fw-signed.bin"),
    (FirmwareKind.CONSOLE, BootMode.BOOTLOADER, NEW_CONSOLE, "motion-console-fw-signed.bin"),
])
def test_resolve_asset(kind, mode, names, expected):
    assert resolve_asset(kind, mode, names) == expected


@pytest.mark.parametrize("kind,names,minimum", [
    (FirmwareKind.SENSOR, LEGACY_SENSOR, "1.8.2"),
    (FirmwareKind.CONSOLE, LEGACY_CONSOLE, "1.8.1"),
])
def test_bootloader_unit_rejects_prebootloader_release(kind, names, minimum):
    """The refusal must lead with the device state — the active bootloader is
    why the rollback is impossible, the missing signed asset is only the
    mechanism (#225). Keep naming the minimum version the unit can take."""
    with pytest.raises(UnsupportedReleaseError) as exc:
        resolve_asset(kind, BootMode.BOOTLOADER, names)
    assert "bootloader is active" in str(exc.value)
    assert minimum in str(exc.value)


def test_unknown_mode_refuses_to_resolve():
    with pytest.raises(ValueError):
        resolve_asset(FirmwareKind.SENSOR, BootMode.UNKNOWN, NEW_SENSOR)


def test_missing_expected_asset_raises():
    with pytest.raises(UnsupportedReleaseError):
        resolve_asset(FirmwareKind.SENSOR, BootMode.BARE_METAL, ["something-else.bin"])


@pytest.mark.parametrize("kind,expected", [
    (FirmwareKind.SENSOR, "motion-sensor-production.bin"),
    (FirmwareKind.CONSOLE, "motion-console-production.bin"),
])
def test_production_asset(kind, expected):
    assert production_asset(kind) == expected


def test_candidate_assets_covers_every_mode():
    """download_firmware() fetches all mode-candidates up front, because the
    device's mode is not known until it is already in DFU."""
    names = candidate_assets(FirmwareKind.SENSOR, NEW_SENSOR)
    assert "motion-sensor-fw-baremetal-fpga.bin" in names
    assert "motion-sensor-fw-signed.bin" in names
    # The production image converts a device; it is never part of an update.
    assert "motion-sensor-production.bin" not in names


def test_candidate_assets_on_legacy_release_is_the_single_image():
    assert candidate_assets(FirmwareKind.SENSOR, LEGACY_SENSOR) == ["motion-sensor-fw.bin"]
