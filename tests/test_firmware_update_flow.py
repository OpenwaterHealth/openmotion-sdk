"""FirmwareUpdater picks the asset and address from the device's boot mode.

The device is only classifiable once it is already in DFU, so the flow is:
enter DFU -> detect -> choose asset + address -> flash.
"""

from pathlib import Path

import pytest

from _dfu_fakes import FakeDfuUnit, FakeHandle, TestKeys
from omotion import firmware_update
from omotion.boot_mode import BootMode
from omotion.firmware_update import (
    BootloaderChangeRequired,
    FirmwareKind,
    FirmwareUpdateError,
    FirmwareUpdater,
    UnsupportedReleaseError,
    register_download,
)

pytestmark = pytest.mark.unit

BASE = 0x08000000
SLOT = 0x08020000
BARE_METAL_IMAGE = b"\x00\x00\x08\x24\x01\x04\x00\x08" + b"\xa5" * 56


@pytest.fixture(scope="module")
def keys():
    return TestKeys()


@pytest.fixture(autouse=True)
def _test_keys(monkeypatch, keys):
    monkeypatch.setattr(firmware_update, "public_key_pem", keys.pem)


def _updater(unit):
    return FirmwareUpdater(dfu=unit, app_present=unit.app_present)


def _update(unit, path, **kwargs):
    return _updater(unit).update(FakeHandle(unit), path, **kwargs)


@pytest.fixture
def release_dir(tmp_path, keys):
    """A download directory holding every candidate for a new-style release."""
    (tmp_path / "motion-sensor-fw-baremetal-fpga.bin").write_bytes(BARE_METAL_IMAGE)
    (tmp_path / "motion-sensor-fw-signed.bin").write_bytes(keys.signed("sensor"))
    return tmp_path


def _legacy_release(tmp_path):
    legacy = tmp_path / "motion-sensor-fw.bin"
    legacy.write_bytes(BARE_METAL_IMAGE)
    register_download(legacy, FirmwareKind.SENSOR, "1.8.1")
    return legacy


def test_bare_metal_device_flashes_baremetal_image_at_flash_base(release_dir):
    unit = FakeDfuUnit(mode="rom")
    primary = release_dir / "motion-sensor-fw-baremetal-fpga.bin"
    register_download(primary, FirmwareKind.SENSOR, "1.8.2")

    _update(unit, primary)

    assert unit.downloads == [(BASE, BARE_METAL_IMAGE)]


def test_bootloader_device_flashes_signed_image_into_the_slot(release_dir):
    """Same starting path as the bare-metal case — the detected mode, not the
    caller, decides which sibling is used. Both files are registered, as
    download_firmware() registers every asset it fetches; only registered
    same-release files are sibling candidates."""
    unit = FakeDfuUnit(mode="bootloader")
    primary = release_dir / "motion-sensor-fw-baremetal-fpga.bin"
    signed = release_dir / "motion-sensor-fw-signed.bin"
    register_download(primary, FirmwareKind.SENSOR, "1.8.2")
    register_download(signed, FirmwareKind.SENSOR, "1.8.2")

    _update(unit, primary)

    assert unit.downloads == [(SLOT, signed.read_bytes())]


def test_unknown_mode_refuses_to_flash_anything(release_dir):
    unit = FakeDfuUnit(mode="unknown")
    primary = release_dir / "motion-sensor-fw-baremetal-fpga.bin"
    register_download(primary, FirmwareKind.SENSOR, "1.8.2")

    with pytest.raises(FirmwareUpdateError):
        _update(unit, primary)
    assert unit.calls == []


def test_bootloader_device_with_legacy_release_refuses(tmp_path):
    legacy = _legacy_release(tmp_path)
    unit = FakeDfuUnit(mode="bootloader")

    with pytest.raises(UnsupportedReleaseError) as exc:
        _update(unit, legacy)
    assert "1.8.2" in str(exc.value)
    assert unit.downloads == []


def test_legacy_release_still_flashes_on_a_bare_metal_device(tmp_path):
    legacy = _legacy_release(tmp_path)
    unit = FakeDfuUnit(mode="rom")

    _update(unit, legacy)

    assert unit.downloads == [(BASE, BARE_METAL_IMAGE)]


def test_stale_higher_preference_file_from_another_release_is_not_selected(tmp_path):
    """Regression for #218: the downloads dir is shared across releases, so a
    leftover registered for a newer release must not shadow the release the
    caller actually chose."""
    legacy = _legacy_release(tmp_path)
    stale = tmp_path / "motion-sensor-fw-baremetal-fpga.bin"
    stale.write_bytes(b"\x01" * 16)
    register_download(stale, FirmwareKind.SENSOR, "1.10.0")
    unit = FakeDfuUnit(mode="rom")

    _update(unit, legacy)

    assert unit.downloads == [(BASE, BARE_METAL_IMAGE)]


def test_stale_signed_image_does_not_bypass_the_legacy_release_refusal(tmp_path, keys):
    """A bootloader unit on a pre-bootloader release must refuse — not flash a
    signed image left over from some other release (#218)."""
    legacy = _legacy_release(tmp_path)
    stale = tmp_path / "motion-sensor-fw-signed.bin"
    stale.write_bytes(keys.signed("sensor"))
    register_download(stale, FirmwareKind.SENSOR, "1.10.0")
    unit = FakeDfuUnit(mode="bootloader")

    with pytest.raises(UnsupportedReleaseError):
        _update(unit, legacy)
    assert unit.downloads == []


def test_unregistered_files_in_the_directory_are_not_candidates(tmp_path):
    """A file from a previous session (no provenance in this process) is
    invisible to sibling selection even when its name would win on preference
    (#218)."""
    legacy = _legacy_release(tmp_path)
    (tmp_path / "motion-sensor-fw-baremetal-fpga.bin").write_bytes(b"\x01" * 16)
    unit = FakeDfuUnit(mode="rom")

    _update(unit, legacy)

    assert unit.downloads == [(BASE, BARE_METAL_IMAGE)]


def test_device_refusing_dfu_raises_before_flashing():
    unit = FakeDfuUnit()
    with pytest.raises(FirmwareUpdateError):
        _updater(unit).update(FakeHandle(unit, accepts_dfu=False), Path("x.bin"))
    assert unit.calls == []


def test_dfu_device_not_appearing_raises(release_dir):
    unit = FakeDfuUnit(appears=False)
    primary = release_dir / "motion-sensor-fw-baremetal-fpga.bin"
    with pytest.raises(FirmwareUpdateError):
        _update(unit, primary)
    assert unit.calls == []


def test_unregistered_signed_image_refused_on_bare_metal_device(release_dir):
    """The 'Upload File...' path: an arbitrary file the SDK never downloaded.
    A signed slot image at 0x08000000 would land on the bootloader sector."""
    unit = FakeDfuUnit(mode="rom")

    with pytest.raises(BootloaderChangeRequired):
        _update(unit, release_dir / "motion-sensor-fw-signed.bin", kind=FirmwareKind.SENSOR)
    assert unit.downloads == []


def test_signed_image_under_another_name_is_still_refused_on_bare_metal(tmp_path, keys):
    """The name is a hint; the 'SFU1' header is the fact."""
    stray = tmp_path / "custom-build.bin"
    stray.write_bytes(keys.signed("sensor"))
    unit = FakeDfuUnit(mode="rom")

    with pytest.raises(BootloaderChangeRequired):
        _update(unit, stray, kind=FirmwareKind.SENSOR)
    assert unit.downloads == []


def test_unregistered_baremetal_image_refused_on_bootloader_device(release_dir):
    unit = FakeDfuUnit(mode="bootloader")

    with pytest.raises(FirmwareUpdateError):
        _update(unit, release_dir / "motion-sensor-fw-baremetal-fpga.bin",
                kind=FirmwareKind.SENSOR)
    assert unit.downloads == []


def test_unregistered_production_image_refused_by_update(tmp_path):
    """A production image converts a device to bootloader mode. That is never an
    update, and passing one as the application must not be a back door to it."""
    stray = tmp_path / "motion-sensor-production.bin"
    stray.write_bytes(BARE_METAL_IMAGE)
    unit = FakeDfuUnit(mode="rom")

    with pytest.raises(FirmwareUpdateError) as exc:
        _update(unit, stray)
    assert "production_image" in str(exc.value)
    assert unit.downloads == []


def test_unregistered_unrecognisable_file_flashes_at_the_mode_address(tmp_path, keys):
    """A file we cannot classify by name is taken at face value and written to
    whatever the detected mode calls for."""
    stray = tmp_path / "custom-build.bin"
    stray.write_bytes(keys.signed("sensor"))
    unit = FakeDfuUnit(mode="bootloader")

    _update(unit, stray, kind=FirmwareKind.SENSOR)

    assert unit.downloads == [(SLOT, stray.read_bytes())]


def test_signed_image_without_a_known_product_is_refused(tmp_path, keys):
    """The keys differ per product, so the pre-flight cannot run without one."""
    stray = tmp_path / "custom-build.bin"
    stray.write_bytes(keys.signed("sensor"))
    unit = FakeDfuUnit(mode="bootloader")

    with pytest.raises(FirmwareUpdateError, match="kind="):
        _update(unit, stray)
    assert unit.downloads == []


def test_updater_records_the_mode_it_detected(release_dir):
    """UIs need to show what the device turned out to be, and the only moment
    it is observable is inside update() while the device sits in DFU."""
    unit = FakeDfuUnit(mode="bootloader", version="1.2.0")
    updater = _updater(unit)
    primary = release_dir / "motion-sensor-fw-baremetal-fpga.bin"
    register_download(primary, FirmwareKind.SENSOR, "1.8.2")
    register_download(release_dir / "motion-sensor-fw-signed.bin", FirmwareKind.SENSOR, "1.8.2")

    assert updater.last_boot_mode is None
    updater.update(FakeHandle(unit), primary)
    assert updater.last_boot_mode is BootMode.BOOTLOADER
    assert updater.last_bootloader_version == "1.2.0"


def test_updater_records_mode_even_when_the_flash_is_refused(release_dir):
    """A refusal is still information about the device — don't discard it.

    The new-style files from the fixture stay on disk, unregistered: they must
    not shadow the legacy release's refusal (#218)."""
    legacy = _legacy_release(release_dir)
    unit = FakeDfuUnit(mode="bootloader")
    updater = _updater(unit)

    with pytest.raises(UnsupportedReleaseError):
        updater.update(FakeHandle(unit), legacy)
    assert updater.last_boot_mode is BootMode.BOOTLOADER
