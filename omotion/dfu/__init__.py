"""STM32 DFU in pure Python: the DfuSe protocol, signed-image pre-flight and the
bootloader migration sequences.

Ported from the bootloader repository's ``py-tools`` (see PROVENANCE.txt). Most
callers want :class:`omotion.firmware_update.FirmwareUpdater`, which drives
these pieces; they are public for scripts and bench work.
"""

from omotion.dfu.errors import (
    BootloaderVersionError,
    DFUAmbiguousDeviceError,
    DFUBackendError,
    DFUDeviceNotFoundError,
    DFUError,
    DFUTimeoutError,
    DFUVerifyError,
)
from omotion.dfu.image import (
    ImageInfo,
    ImageVerificationError,
    read_header_version,
    verify_image,
)
from omotion.dfu.keys import public_key_pem
from omotion.dfu.migration import MIN_BOOTLOADER, parse_bl_version
from omotion.dfu.progress import DFUProgress, ProgressCallback
from omotion.dfu.stm32dfu import STM32DFU, DFUState, DFUStatus, classify_alt_names

__all__ = [
    "STM32DFU",
    "DFUState",
    "DFUStatus",
    "classify_alt_names",
    "DFUProgress",
    "ProgressCallback",
    "DFUError",
    "DFUDeviceNotFoundError",
    "DFUAmbiguousDeviceError",
    "DFUBackendError",
    "DFUTimeoutError",
    "DFUVerifyError",
    "BootloaderVersionError",
    "ImageInfo",
    "ImageVerificationError",
    "read_header_version",
    "verify_image",
    "public_key_pem",
    "MIN_BOOTLOADER",
    "parse_bl_version",
]
