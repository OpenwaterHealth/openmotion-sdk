"""Exceptions raised by :mod:`omotion.dfu`."""

from __future__ import annotations

__all__ = [
    "DFUError",
    "DFUDeviceNotFoundError",
    "DFUAmbiguousDeviceError",
    "DFUBackendError",
    "DFUTimeoutError",
    "DFUVerifyError",
    "BootloaderVersionError",
]


class DFUError(RuntimeError):
    """A DFU operation failed. Base class for everything this package raises
    about the device or the USB link."""


class DFUDeviceNotFoundError(DFUError):
    """No DFU device (0483:DF11) matched."""


class DFUAmbiguousDeviceError(DFUError):
    """Several DFU devices are attached and none was selected by serial."""


class DFUBackendError(DFUError):
    """libusb could not be loaded, or the device is not bound to WinUSB."""


class DFUTimeoutError(DFUError):
    """The device did not reach the expected DFU state in time."""


class DFUVerifyError(DFUError):
    """Read-back did not match what was written."""


class BootloaderVersionError(DFUError):
    """The bootloader reports a version that cannot take the requested image."""

    def __init__(self, message: str, version: str | None = None):
        super().__init__(message)
        self.version = version
