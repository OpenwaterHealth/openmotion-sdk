"""FirmwareUpdater picks the asset and address from the device's boot mode.

The device is only classifiable once it is already in DFU, so the flow is:
enter DFU -> detect -> choose asset + address -> flash.
"""

from pathlib import Path

import pytest

from omotion.boot_mode import BootMode
from omotion.DFUProgrammer import DFUResult
from omotion.firmware_update import (
    FirmwareKind,
    FirmwareUpdateError,
    FirmwareUpdater,
    UnsupportedReleaseError,
    register_download,
)


class FakeHandle:
    def __init__(self, accepts_dfu=True):
        self.accepts_dfu = accepts_dfu
        self.enter_dfu_calls = 0

    def enter_dfu(self):
        self.enter_dfu_calls += 1
        return self.accepts_dfu


class FakeProgrammer:
    """Stands in for DFUProgrammer; records what would have been flashed."""

    def __init__(self, mode=BootMode.BARE_METAL, appears=True):
        self._mode = mode
        self._appears = appears
        self.flashed = []          # list of (path, address)

    def wait_for_dfu_device(self, *, timeout_s=30.0):
        return self._appears

    def detect_boot_mode(self):
        return self._mode

    def flash_bin(self, bin_path, *, address, progress=None):
        self.flashed.append((Path(bin_path), address))
        return DFUResult(command=[], returncode=0, stdout="", success=True)


@pytest.fixture
def release_dir(tmp_path):
    """A download directory holding every candidate for a new-style release."""
    for name in (
        "motion-sensor-fw-baremetal-fpga.bin",
        "motion-sensor-fw-signed.bin",
    ):
        (tmp_path / name).write_bytes(b"\x00" * 16)
    return tmp_path


def test_bare_metal_device_flashes_baremetal_image_at_flash_base(release_dir):
    prog = FakeProgrammer(mode=BootMode.BARE_METAL)
    updater = FirmwareUpdater(programmer=prog)
    primary = release_dir / "motion-sensor-fw-baremetal-fpga.bin"
    register_download(primary, FirmwareKind.SENSOR, "1.8.2")

    updater.update(FakeHandle(), primary)

    assert prog.flashed == [(primary, "0x08000000")]


def test_bootloader_device_flashes_signed_image_into_the_slot(release_dir):
    """Same starting path as the bare-metal case — the detected mode, not the
    caller, decides which sibling is used. Both files are registered, as
    download_firmware() registers every asset it fetches; only registered
    same-release files are sibling candidates."""
    prog = FakeProgrammer(mode=BootMode.BOOTLOADER)
    updater = FirmwareUpdater(programmer=prog)
    primary = release_dir / "motion-sensor-fw-baremetal-fpga.bin"
    register_download(primary, FirmwareKind.SENSOR, "1.8.2")
    register_download(release_dir / "motion-sensor-fw-signed.bin", FirmwareKind.SENSOR, "1.8.2")

    updater.update(FakeHandle(), primary)

    assert prog.flashed == [(release_dir / "motion-sensor-fw-signed.bin", "0x08020000")]


def test_unknown_mode_falls_back_to_bare_metal(release_dir):
    """An unclassifiable device is flashed as bare metal rather than refused.

    This is fail-safe in hardware, which is what makes it an acceptable default:
    both bootloaders clamp their DFU write window to the application slot and
    mark sector 0 read-only, so a bare-metal write at 0x08000000 against a
    bootloader unit is rejected by the bootloader itself. The device fails the
    flash loudly instead of being bricked. Defaulting the *other* way would not
    be safe -- a signed image at 0x08020000 lands happily in the middle of a
    bare-metal device's flash and produces a unit that will not boot.
    """
    prog = FakeProgrammer(mode=BootMode.UNKNOWN)
    updater = FirmwareUpdater(programmer=prog)
    primary = release_dir / "motion-sensor-fw-baremetal-fpga.bin"
    register_download(primary, FirmwareKind.SENSOR, "1.8.2")

    updater.update(FakeHandle(), primary)

    assert prog.flashed == [(primary, "0x08000000")]


def test_unknown_mode_still_reports_what_was_actually_detected(release_dir):
    """The fallback must not launder a guess into an observation: callers (and
    the lock icon in the apps) read last_boot_mode, so it stays UNKNOWN."""
    prog = FakeProgrammer(mode=BootMode.UNKNOWN)
    updater = FirmwareUpdater(programmer=prog)
    primary = release_dir / "motion-sensor-fw-baremetal-fpga.bin"
    register_download(primary, FirmwareKind.SENSOR, "1.8.2")

    updater.update(FakeHandle(), primary)

    assert updater.last_boot_mode is BootMode.UNKNOWN


def test_unknown_mode_never_picks_the_signed_image(release_dir):
    """The dangerous outcome, asserted directly: whatever else happens on an
    unclassifiable device, it must never be given a slot image."""
    prog = FakeProgrammer(mode=BootMode.UNKNOWN)
    updater = FirmwareUpdater(programmer=prog)
    primary = release_dir / "motion-sensor-fw-baremetal-fpga.bin"
    register_download(primary, FirmwareKind.SENSOR, "1.8.2")

    updater.update(FakeHandle(), primary)

    (flashed, address), = prog.flashed
    assert "signed" not in flashed.name
    assert address != "0x08020000"


def test_bootloader_device_with_legacy_release_refuses(tmp_path):
    legacy = tmp_path / "motion-sensor-fw.bin"
    legacy.write_bytes(b"\x00" * 16)
    register_download(legacy, FirmwareKind.SENSOR, "1.8.1")
    prog = FakeProgrammer(mode=BootMode.BOOTLOADER)

    with pytest.raises(UnsupportedReleaseError) as exc:
        FirmwareUpdater(programmer=prog).update(FakeHandle(), legacy)
    assert "1.8.2" in str(exc.value)
    assert prog.flashed == []


def test_legacy_release_still_flashes_on_a_bare_metal_device(tmp_path):
    legacy = tmp_path / "motion-sensor-fw.bin"
    legacy.write_bytes(b"\x00" * 16)
    register_download(legacy, FirmwareKind.SENSOR, "1.8.1")
    prog = FakeProgrammer(mode=BootMode.BARE_METAL)

    FirmwareUpdater(programmer=prog).update(FakeHandle(), legacy)

    assert prog.flashed == [(legacy, "0x08000000")]


def test_stale_higher_preference_file_from_another_release_is_not_selected(tmp_path):
    """Regression for #218: the downloads dir is shared across releases, so a
    leftover registered for a newer release must not shadow the release the
    caller actually chose."""
    legacy = tmp_path / "motion-sensor-fw.bin"
    legacy.write_bytes(b"\x00" * 16)
    stale = tmp_path / "motion-sensor-fw-baremetal-fpga.bin"
    stale.write_bytes(b"\x01" * 16)
    register_download(legacy, FirmwareKind.SENSOR, "1.8.1")
    register_download(stale, FirmwareKind.SENSOR, "1.10.0")
    prog = FakeProgrammer(mode=BootMode.BARE_METAL)

    FirmwareUpdater(programmer=prog).update(FakeHandle(), legacy)

    assert prog.flashed == [(legacy, "0x08000000")]


def test_stale_signed_image_does_not_bypass_the_legacy_release_refusal(tmp_path):
    """A bootloader unit on a pre-bootloader release must refuse — not flash a
    signed image left over from some other release (#218)."""
    legacy = tmp_path / "motion-sensor-fw.bin"
    legacy.write_bytes(b"\x00" * 16)
    stale = tmp_path / "motion-sensor-fw-signed.bin"
    stale.write_bytes(b"\x01" * 16)
    register_download(legacy, FirmwareKind.SENSOR, "1.8.1")
    register_download(stale, FirmwareKind.SENSOR, "1.10.0")
    prog = FakeProgrammer(mode=BootMode.BOOTLOADER)

    with pytest.raises(UnsupportedReleaseError):
        FirmwareUpdater(programmer=prog).update(FakeHandle(), legacy)
    assert prog.flashed == []


def test_unregistered_files_in_the_directory_are_not_candidates(tmp_path):
    """A file from a previous session (no provenance in this process) is
    invisible to sibling selection even when its name would win on preference
    (#218)."""
    legacy = tmp_path / "motion-sensor-fw.bin"
    legacy.write_bytes(b"\x00" * 16)
    (tmp_path / "motion-sensor-fw-baremetal-fpga.bin").write_bytes(b"\x01" * 16)
    register_download(legacy, FirmwareKind.SENSOR, "1.8.1")
    prog = FakeProgrammer(mode=BootMode.BARE_METAL)

    FirmwareUpdater(programmer=prog).update(FakeHandle(), legacy)

    assert prog.flashed == [(legacy, "0x08000000")]


def test_device_refusing_dfu_raises_before_flashing():
    prog = FakeProgrammer()
    with pytest.raises(FirmwareUpdateError):
        FirmwareUpdater(programmer=prog).update(FakeHandle(accepts_dfu=False), Path("x.bin"))
    assert prog.flashed == []


def test_dfu_device_not_appearing_raises(release_dir):
    prog = FakeProgrammer(appears=False)
    primary = release_dir / "motion-sensor-fw-baremetal-fpga.bin"
    with pytest.raises(FirmwareUpdateError):
        FirmwareUpdater(programmer=prog).update(FakeHandle(), primary)
    assert prog.flashed == []


def test_unregistered_signed_image_refused_on_bare_metal_device(tmp_path):
    """The 'Upload File...' path: an arbitrary file the SDK never downloaded.
    A signed slot image at 0x08000000 would land on the bootloader sector."""
    stray = tmp_path / "motion-sensor-fw-signed.bin"
    stray.write_bytes(b"\x00" * 16)
    prog = FakeProgrammer(mode=BootMode.BARE_METAL)

    with pytest.raises(FirmwareUpdateError):
        FirmwareUpdater(programmer=prog).update(FakeHandle(), stray)
    assert prog.flashed == []


def test_unregistered_baremetal_image_refused_on_bootloader_device(tmp_path):
    stray = tmp_path / "motion-sensor-fw-baremetal-fpga.bin"
    stray.write_bytes(b"\x00" * 16)
    prog = FakeProgrammer(mode=BootMode.BOOTLOADER)

    with pytest.raises(FirmwareUpdateError):
        FirmwareUpdater(programmer=prog).update(FakeHandle(), stray)
    assert prog.flashed == []


def test_unregistered_production_image_refused_by_update(tmp_path):
    """A production image converts a device to bootloader mode. That is never an
    update, and update() must not be a back door to it."""
    stray = tmp_path / "motion-sensor-production.bin"
    stray.write_bytes(b"\x00" * 16)
    prog = FakeProgrammer(mode=BootMode.BARE_METAL)

    with pytest.raises(FirmwareUpdateError) as exc:
        FirmwareUpdater(programmer=prog).update(FakeHandle(), stray)
    assert "install_bootloader" in str(exc.value)
    assert prog.flashed == []


def test_unregistered_unrecognisable_file_flashes_at_the_mode_address(tmp_path):
    """A file we cannot classify by name is taken at face value and written to
    whatever the detected mode calls for."""
    stray = tmp_path / "custom-build.bin"
    stray.write_bytes(b"\x00" * 16)
    prog = FakeProgrammer(mode=BootMode.BOOTLOADER)

    FirmwareUpdater(programmer=prog).update(FakeHandle(), stray)

    assert prog.flashed == [(stray, "0x08020000")]


def test_updater_records_the_mode_it_detected(release_dir):
    """UIs need to show what the device turned out to be, and the only moment
    it is observable is inside update() while the device sits in DFU."""
    prog = FakeProgrammer(mode=BootMode.BOOTLOADER)
    updater = FirmwareUpdater(programmer=prog)
    primary = release_dir / "motion-sensor-fw-baremetal-fpga.bin"
    register_download(primary, FirmwareKind.SENSOR, "1.8.2")
    register_download(release_dir / "motion-sensor-fw-signed.bin", FirmwareKind.SENSOR, "1.8.2")

    assert updater.last_boot_mode is None
    updater.update(FakeHandle(), primary)
    assert updater.last_boot_mode is BootMode.BOOTLOADER


def test_updater_records_mode_even_when_the_flash_is_refused(release_dir):
    """A refusal is still information about the device — don't discard it.

    The new-style files from the fixture stay on disk, unregistered: they must
    not shadow the legacy release's refusal (#218)."""
    legacy = release_dir / "motion-sensor-fw.bin"
    legacy.write_bytes(b"\x00" * 16)
    register_download(legacy, FirmwareKind.SENSOR, "1.8.1")
    updater = FirmwareUpdater(programmer=FakeProgrammer(mode=BootMode.BOOTLOADER))

    with pytest.raises(UnsupportedReleaseError):
        updater.update(FakeHandle(), legacy)
    assert updater.last_boot_mode is BootMode.BOOTLOADER


# ---------------------------------------------------------------------------
# End-to-end classification (openmotion-sdk#197)
#
# FakeProgrammer above stubs detect_boot_mode() outright, so those tests can
# never catch a *classification* bug — they assume the mode is already known.
# These drive a real `dfu-util -l` listing through the actual
# DFUProgrammer.detect_boot_mode -> parse_boot_mode chain, which is the path the
# apps take: the test app hands a FirmwareUpdater to its flash thread and lets
# the SDK decide the asset and address.
# ---------------------------------------------------------------------------

from omotion.DFUProgrammer import DFUProgrammer


def _listing(alt_lines: str) -> str:
    return (
        "dfu-util 0.11\n\n"
        "Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.\n\n"
        f"{alt_lines}\n"
    )


# A bare-metal unit whose listing carries the Internal Flash alt and nothing
# else — no ROM-only alt to key off. This is what made old consoles unflashable.
ROM_FLASH_ALT_ONLY = _listing(
    'Found DFU: [0483:df11] ver=0200, devnum=54, cfg=1, intf=0, path="2-1.1.1", '
    'alt=0, name="@Internal Flash   /0x08000000/16*128Kg", serial="200364500000"'
)

# A converted console: same single-alt shape, but read-only runs around the slot.
CONSOLE_BL_LISTING = _listing(
    'Found DFU: [0483:df11] ver=0200, devnum=31, cfg=1, intf=0, path="2-1.1.1", '
    'alt=0, name="@Internal Flash/0x08000000/01*128Ka,08*128Kg,07*128Ka", '
    'serial="OWCONSOLEBL"'
)


class ListingProgrammer(FakeProgrammer):
    """FakeProgrammer, but boot mode comes from a real listing via real parsing."""

    def __init__(self, listing):
        super().__init__()
        self._listing = listing

    def list_devices(self):
        return self._listing

    # Deliberately the genuine implementation, not a stub.
    detect_boot_mode = DFUProgrammer.detect_boot_mode


def test_update_flashes_bare_metal_when_only_the_flash_alt_is_listed(release_dir):
    """The #197 regression: a bare-metal unit whose listing has no ROM-only alt
    used to raise "could not tell whether this device has the bootloader
    installed" and could not be updated at all."""
    prog = ListingProgrammer(ROM_FLASH_ALT_ONLY)
    updater = FirmwareUpdater(programmer=prog)
    primary = release_dir / "motion-sensor-fw-baremetal-fpga.bin"
    register_download(primary, FirmwareKind.SENSOR, "1.8.2")

    updater.update(FakeHandle(), primary)

    assert updater.last_boot_mode is BootMode.BARE_METAL
    assert prog.flashed == [(primary, "0x08000000")]


def test_update_still_routes_a_converted_console_to_the_signed_slot(release_dir):
    """The other half of the guarantee: relaxing bare-metal detection must not
    let a bootloader unit get a bare-metal image at the base of flash."""
    for name in ("motion-console-fw-baremetal.bin", "motion-console-fw-signed.bin"):
        (release_dir / name).write_bytes(b"\x00" * 16)
    prog = ListingProgrammer(CONSOLE_BL_LISTING)
    updater = FirmwareUpdater(programmer=prog)
    primary = release_dir / "motion-console-fw-baremetal.bin"
    register_download(primary, FirmwareKind.CONSOLE, "1.8.1-rc.2")

    updater.update(FakeHandle(), primary)

    assert updater.last_boot_mode is BootMode.BOOTLOADER
    assert prog.flashed == [
        (release_dir / "motion-console-fw-signed.bin", "0x08020000")
    ]
