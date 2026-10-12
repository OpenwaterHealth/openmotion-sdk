"""STM32DFU, the pure-Python DfuSe port, against a fake USB device.

The classification vectors are the ones in the bootloader repository's
py-tools/tests/test_migrate_tools.py. Needs no hardware.
"""

import pytest

from _dfu_fakes import FakeUsbDevice, install_fake_usb, usb_error
from omotion.dfu import (
    STM32DFU,
    DFUAmbiguousDeviceError,
    DFUDeviceNotFoundError,
    DFUError,
    DFUTimeoutError,
    DFUVerifyError,
    classify_alt_names,
)

pytestmark = pytest.mark.unit

ROM = ["@Internal Flash  /0x08000000/16*128Kg",
       "@Option Bytes  /0x5200201C/01*128 e",
       "@OTP Memory /0x1FF0F000/01*1024 e",
       "@Device Feature/0xFFFF0000/01*004 e"]
BL = ["@Internal Flash/0x08000000/01*128Ka,04*128Kg,11*128Ka"]

SLOT = 0x08020000


def test_rom_loader_alts_classify_as_rom():
    assert classify_alt_names(ROM) == "rom"


def test_bootloader_alt_classifies_as_bootloader():
    assert classify_alt_names(BL) == "bootloader"


def test_anything_else_is_unknown():
    assert classify_alt_names([]) == "unknown"
    assert classify_alt_names(["@Internal Flash/0x08000000/16*128Kg"]) == "unknown"
    assert classify_alt_names(["junk", "more junk"]) == "unknown"


@pytest.fixture
def connected(monkeypatch):
    dev = FakeUsbDevice()
    install_fake_usb(monkeypatch, [dev])
    dfu = STM32DFU()
    dfu.connect()
    yield dfu, dev
    dfu.disconnect()


def test_connect_without_a_device_raises_not_found(monkeypatch):
    install_fake_usb(monkeypatch, [])
    with pytest.raises(DFUDeviceNotFoundError):
        STM32DFU().connect()


def test_connect_refuses_to_pick_between_two_devices(monkeypatch):
    """Flashing the wrong unit is not recoverable for a bootloader change."""
    install_fake_usb(monkeypatch, [FakeUsbDevice(serial="A"), FakeUsbDevice(serial="B")])
    with pytest.raises(DFUAmbiguousDeviceError):
        STM32DFU().connect()


def test_connect_by_serial_selects_among_several(monkeypatch):
    a, b = FakeUsbDevice(serial="A"), FakeUsbDevice(serial="B")
    install_fake_usb(monkeypatch, [a, b])
    dfu = STM32DFU()
    dfu.connect(serial="B")
    assert dfu._dev is b


def test_connect_by_unknown_serial_raises_not_found(monkeypatch):
    install_fake_usb(monkeypatch, [FakeUsbDevice(serial="A")])
    with pytest.raises(DFUDeviceNotFoundError):
        STM32DFU().connect(serial="Z")


def test_reopen_after_reset_finds_the_same_unit(monkeypatch):
    a = FakeUsbDevice(serial="A", port=(1, (1,)))
    b = FakeUsbDevice(serial="B", port=(1, (2,)))
    devices = [b]
    install_fake_usb(monkeypatch, devices)
    dfu = STM32DFU()
    dfu.connect()
    devices.insert(0, a)                      # another unit shows up first in the list

    assert dfu._same_device() is b


def test_download_erases_the_covered_sectors_and_writes_the_image(connected):
    dfu, dev = connected
    image = bytes(range(256)) * 600           # 150 KB: crosses into a second sector

    dfu.download(SLOT, image)

    assert dev.erased == [SLOT, SLOT + 0x20000]
    assert dev.read(SLOT, len(image)) == image
    assert dev.manifested


def test_download_pre_erased_does_not_erase(connected):
    dfu, dev = connected
    dfu.download(SLOT, b"\x01" * 64, pre_erased=True)
    assert dev.erased == []


def test_download_outside_the_slot_is_refused_before_any_transfer(connected):
    dfu, dev = connected
    with pytest.raises(DFUError):
        dfu.download(0x08000000, b"\x01" * 64)
    assert dev.erased == [] and not dev.manifested


def test_rom_mode_widens_the_window_to_the_base_of_flash(connected):
    dfu, dev = connected
    dfu.set_rom_mode()
    dfu.download(0x08000000, b"\x02" * 64)
    assert dev.erased == [0x08000000]
    assert dev.read(0x08000000, 64) == b"\x02" * 64


def test_erase_all_covers_exactly_the_application_slot(connected):
    dfu, dev = connected
    dfu.erase_all(progress_cb=lambda *a: None)
    assert dev.erased == [0x08020000, 0x08040000, 0x08060000, 0x08080000]


def test_read_version_returns_the_bootloader_string(connected):
    dfu, dev = connected
    dev.version = "1.2.0-rc.1"
    assert dfu.read_version() == "1.2.0-rc.1"


def test_verify_passes_on_matching_flash(connected):
    dfu, dev = connected
    dfu.download(SLOT, b"\x03" * 2048)
    assert dfu.verify(SLOT, b"\x03" * 2048) is True


def test_verify_reports_the_first_mismatch(connected):
    dfu, dev = connected
    dfu.download(SLOT, b"\x03" * 2048)
    with pytest.raises(DFUVerifyError):
        dfu.verify(SLOT, b"\x03" * 2047 + b"\x04")


def test_verify_fails_on_a_short_read_back(connected):
    """A truncated read-back used to compare equal on its prefix."""
    dfu, dev = connected
    dfu.download(SLOT, b"\x03" * 512)
    dev.short_upload = True
    with pytest.raises(DFUVerifyError):
        dfu.verify(SLOT, b"\x03" * 512)


def test_usb_errors_surface_as_dfu_errors(connected):
    dfu, dev = connected
    dev.fail_with = usb_error()
    with pytest.raises(DFUError):
        dfu.set_address(SLOT)


def test_a_device_error_status_names_the_status(connected):
    """errVENDOR is how bootloader 1.2.0 refuses a signed header."""
    dfu, dev = connected
    dev.status = 0x0B
    with pytest.raises(DFUError, match="errVENDOR"):
        dfu.set_address(SLOT)


def test_poll_timeout_is_a_dfu_error(connected):
    dfu, dev = connected
    dev.stuck_busy = True
    with pytest.raises(DFUTimeoutError):
        dfu._poll_idle(timeout_s=0.05)


def test_requests_without_a_connection_raise_dfu_error():
    with pytest.raises(DFUError):
        STM32DFU().get_status()
