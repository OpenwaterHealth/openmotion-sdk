"""Shared fakes for the DFU and firmware-update unit tests (no hardware).

Imported with an absolute import (``from _dfu_fakes import ...``); pytest puts
``tests/`` on ``sys.path`` (rootdir conftest).
"""

import hashlib
import struct

import usb.core

from omotion.dfu.image import IMAGE_OFFSET

_AUTH_HEADER_FMT = "<4sHHIII32s32s16s28s"


# ---------------------------------------------------------------------------
# Signed images
# ---------------------------------------------------------------------------

def make_key():
    """A throwaway P-256 key: ``(private_key, public_pem)``."""
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric import ec

    key = ec.generate_private_key(ec.SECP256R1())
    pem = key.public_key().public_bytes(
        serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo
    )
    return key, pem


def sign_image(key, *, body=None, fw_version=10801, fw_size=None, partial_size=0) -> bytes:
    """Build a signed slot image the way py-tools/sign_firmware.py lays it out:
    128-byte signed header, 64-byte raw R||S, state + fingerprint, 0xFF padding
    to 0x400, then the firmware body."""
    from cryptography.hazmat.primitives import hashes
    from cryptography.hazmat.primitives.asymmetric import ec
    from cryptography.hazmat.primitives.asymmetric.utils import decode_dss_signature

    if body is None:
        body = bytes(range(256)) * 8
    tag = hashlib.sha256(body).digest()
    header = struct.pack(
        _AUTH_HEADER_FMT, b"SFU1", 1, fw_version,
        len(body) if fw_size is None else fw_size, 0, partial_size,
        tag, tag, b"\x00" * 16, b"\x00" * 28,
    )
    r, s = decode_dss_signature(key.sign(header, ec.ECDSA(hashes.SHA256())))
    head = header + r.to_bytes(32, "big") + s.to_bytes(32, "big") + b"\xff" * 96 + b"\x00" * 32
    return head + b"\xff" * (IMAGE_OFFSET - len(head)) + body


def make_production(signed: bytes, *, bootloader: bytes | None = None) -> bytes:
    """Bootloader sector (plausible vector table) followed by ``signed``."""
    if bootloader is None:
        bootloader = struct.pack("<II", 0x24080000, 0x08000401) + b"\xb1" * 1024
    return bootloader + b"\xff" * (0x20000 - len(bootloader)) + signed


# ---------------------------------------------------------------------------
# A DfuSe device at the USB control-transfer level, for STM32DFU itself
# ---------------------------------------------------------------------------

FLASH_BASE = 0x08000000
FLASH_SIZE = 0x00200000
SECTOR = 128 * 1024
VERSION_ADDR = 0xFFFFFF00

_IDLE, _DNLOAD_IDLE, _MANIFEST_SYNC, _MANIFEST, _UPLOAD_IDLE = 2, 5, 6, 7, 9

_REQUEST_NAMES = {1: "DNLOAD", 2: "UPLOAD", 3: "GETSTATUS", 4: "CLRSTATUS",
                  5: "GETSTATE", 6: "ABORT"}


class FakeUsbDevice:
    """Enough of a pyusb device, speaking enough DfuSe, to drive STM32DFU.

    Manifestation follows the Open-Motion bootloaders' DFU class, which is
    manifestation-tolerant: the zero-length DNLOAD moves to dfuMANIFEST-SYNC,
    the next GETSTATUS answers dfuMANIFEST and completes manifestation
    (DFU_Leave), and the bootloader's main loop then reboots on its own (sdk#315).
    """

    idVendor = 0x0483
    idProduct = 0xDF11
    iSerialNumber = 0
    iProduct = 0
    address = 5

    def __init__(self, *, serial=None, port=(1, (4,)), version="1.2.0", xfer=1024):
        self.serial = serial
        self.bus, self.port_numbers = port
        self.version = version
        self.xfer = xfer
        self.flash = bytearray(b"\xff" * FLASH_SIZE)
        self.pointer = FLASH_BASE
        self.state = _IDLE
        self.status = 0
        self.erased = []            # sector addresses, in order
        self.manifested = False     # the zero-length DNLOAD arrived
        self.manifest_complete = False
        self.image_written = False  # a data block was programmed
        self.requests = []          # ("DNLOAD", block, length), ("GETSTATUS",), ("reset",), ...
        self.short_upload = False   # return one byte less than asked
        self.fail_with = None       # exception to raise from every transfer
        self.stuck_busy = False     # never leave dfuDNLOAD-BUSY
        self.gone_after_manifest = None  # exception for every transfer after the leave

    # -- pyusb surface STM32DFU touches ------------------------------------
    def set_configuration(self):
        pass

    def is_kernel_driver_active(self, _iface):
        return False

    def get_active_configuration(self):
        return []

    def reset(self):
        self.requests.append(("reset",))

    def ctrl_transfer(self, request_type, request, value, index, data_or_length):
        name = _REQUEST_NAMES.get(request, request)
        if request in (1, 2):
            length = data_or_length if isinstance(data_or_length, int) else len(data_or_length or b"")
            self.requests.append((name, value, length))
        else:
            self.requests.append((name,))
        if self.fail_with is not None:
            raise self.fail_with
        if self.manifested and self.gone_after_manifest is not None:
            raise self.gone_after_manifest
        if request == 3:                                    # GETSTATUS
            if self.state == _MANIFEST_SYNC and not self.manifest_complete:
                # Answers dfuMANIFEST; once that status is sent, DFU_Leave()
                # marks manifestation complete and returns to dfuMANIFEST-SYNC.
                self.manifest_complete = True
                return bytes([self.status, 1, 0, 0, _MANIFEST, 0])
            if self.state == _MANIFEST_SYNC:
                self.state = _IDLE                          # manifestation-tolerant
            state = 4 if self.stuck_busy else self.state
            return bytes([self.status, 0, 0, 0, state, 0])
        if request == 5:                                    # GETSTATE
            return bytes([self.state])
        if request in (4, 6):                               # CLRSTATUS / ABORT
            self.state, self.status = _IDLE, 0
            return 0
        if request == 1:                                    # DNLOAD
            return self._dnload(value, bytes(data_or_length or b""))
        if request == 2:                                    # UPLOAD
            return self._upload(value, data_or_length)
        raise AssertionError(f"unexpected DFU request {request}")

    # -- DfuSe -------------------------------------------------------------
    def _dnload(self, block, payload):
        self.state = _DNLOAD_IDLE
        if block == 0:
            if not payload:
                self.manifested = True
                self.manifest_complete = False
                self.state = _MANIFEST_SYNC
            elif payload[0] == 0x21:
                self.pointer = struct.unpack_from("<I", payload, 1)[0]
            elif payload[0] == 0x41:
                addr = struct.unpack_from("<I", payload, 1)[0]
                self.erased.append(addr)
                off = addr - FLASH_BASE
                self.flash[off:off + SECTOR] = b"\xff" * SECTOR
            return len(payload)
        off = self.pointer - FLASH_BASE + (block - 2) * self.xfer
        self.flash[off:off + len(payload)] = payload
        self.image_written = True
        return len(payload)

    def _upload(self, block, length):
        self.state = _UPLOAD_IDLE
        if self.short_upload:
            length -= 1
        if self.pointer == VERSION_ADDR:
            return self.version.encode().ljust(64, b"\x00")[:length]
        off = self.pointer - FLASH_BASE + (block - 2) * self.xfer
        return bytes(self.flash[off:off + length])

    def read(self, address, length):
        off = address - FLASH_BASE
        return bytes(self.flash[off:off + length])

    @property
    def bootloader_reboots(self):
        """Would the bootloader's main loop reboot into the new image now?

        Mirrors DFU_ImageDownloadComplete() in the bootloaders' usbd_dfu_if.c
        (1.0.0 to 1.2.0): an image was written, manifestation is complete, and
        the class sits in dfuMANIFEST-SYNC or dfuIDLE.
        """
        return (self.image_written and self.manifest_complete
                and self.state in (_MANIFEST_SYNC, _IDLE))

    def requests_after_first_write(self):
        """Every request from the first data block on."""
        for i, req in enumerate(self.requests):
            if req[0] == "DNLOAD" and req[1] >= 2:
                return self.requests[i:]
        return []


def install_fake_usb(monkeypatch, devices):
    """Make STM32DFU see ``devices`` (a list, read on every enumeration)."""
    import usb.util

    from omotion.dfu import stm32dfu

    monkeypatch.setattr(stm32dfu, "_find_devices", lambda vid, pid: list(devices))
    monkeypatch.setattr(stm32dfu, "_serial_of", lambda dev: dev.serial)
    monkeypatch.setattr(usb.util, "claim_interface", lambda dev, iface: None)
    monkeypatch.setattr(usb.util, "release_interface", lambda dev, iface: None)
    monkeypatch.setattr(usb.util, "dispose_resources", lambda dev: None)


def usb_error(msg="pipe error"):
    return usb.core.USBError(msg)


# ---------------------------------------------------------------------------
# A unit at the STM32DFU API level, for FirmwareUpdater
# ---------------------------------------------------------------------------

SLOT_ADDR = 0x08020000


class FakeDfuUnit:
    """Stands in for the STM32DFU class *and* the device behind it.

    FirmwareUpdater calls it to get a device object and asks it
    ``wait_for_device``; both are this one object, which also plays the unit:
    it resets after a download, runs the updater, and boots the application.
    Everything that would touch flash is recorded in ``calls``.
    """

    def __init__(self, mode="bootloader", version="1.2.0", *, installed_fw_version=0,
                 appears=True, in_dfu=False, refuse_download=False, app_boots=True,
                 updater=None, updater_outcome="upgraded"):
        self.mode = mode
        self.version = version
        self.installed_fw_version = installed_fw_version
        self.appears = appears
        self.in_dfu = in_dfu
        self.refuse_download = refuse_download
        self.app_boots = app_boots
        self.updater = updater                  # bytes that count as "the updater"
        self.updater_outcome = updater_outcome  # upgraded | refused | dead | unchanged
        self.app_running = not in_dfu
        self.calls = []

    # -- the STM32DFU class surface ----------------------------------------
    def __call__(self):
        return self

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.disconnect()

    def wait_for_device(self, timeout_s, present=True, serial=None):
        return self.in_dfu == present

    # -- the STM32DFU instance surface -------------------------------------
    def connect(self, serial=None):
        from omotion.dfu import DFUDeviceNotFoundError

        if not self.in_dfu:
            raise DFUDeviceNotFoundError("No DFU device found")

    def disconnect(self):
        pass

    def detect_mode(self):
        return self.mode, []

    def read_version(self):
        from omotion.dfu import DFUError

        if self.mode != "bootloader":
            raise DFUError("the STM32 ROM loader has no version string")
        return self.version

    def upload(self, address, length):
        if address == SLOT_ADDR and self.installed_fw_version:
            return (b"SFU1" + struct.pack("<HH", 1, self.installed_fw_version))[:length]
        return b"\xff" * length

    def set_rom_mode(self):
        self.calls.append(("set_rom_mode",))

    def erase_all(self, progress_cb=None):
        self.calls.append(("erase_all",))
        if progress_cb:
            progress_cb(4, 4)

    def erase_range(self, start, end, progress_cb=None):
        self.calls.append(("erase_range", start, end))
        if progress_cb:
            progress_cb(1, 1)

    def download(self, address, data, progress_cb=None, pre_erased=False):
        from omotion.dfu import DFUError

        if self.refuse_download:
            raise DFUError("Device error: errVENDOR in state dfuERROR")
        self.calls.append(("download", address, bytes(data), pre_erased))
        if progress_cb:
            progress_cb(len(data), len(data))
        self.in_dfu = False                     # the unit resets
        if self.updater is not None and bytes(data) == self.updater:
            if self.updater_outcome == "upgraded":
                self.version, self.in_dfu = "1.2.0", True
            elif self.updater_outcome == "unchanged":
                self.in_dfu = True
            elif self.updater_outcome == "refused":
                self.app_running = True
        else:
            self.app_running = self.app_boots
            self.in_dfu = self.mode == "bootloader" and not self.app_boots

    def leave_dfu(self):
        self.calls.append(("leave",))
        if self.in_dfu:                         # else it already reset: nobody to hear it
            self.in_dfu = False
            self.app_running = True

    # -- for assertions ----------------------------------------------------
    def app_present(self, kind, handle):
        return self.app_running

    @property
    def downloads(self):
        return [(c[1], c[2]) for c in self.calls if c[0] == "download"]


class FakeHandle:
    """A connected console or sensor that can be asked into DFU."""

    def __init__(self, unit=None, accepts_dfu=True):
        self.unit = unit
        self.accepts_dfu = accepts_dfu
        self.enter_dfu_calls = 0

    def enter_dfu(self):
        self.enter_dfu_calls += 1
        if self.accepts_dfu and self.unit is not None and self.unit.appears:
            self.unit.in_dfu = True
            self.unit.app_running = False
        return self.accepts_dfu


class TestKeys:
    """Throwaway stand-ins for the four bootloader keys."""

    __test__ = False

    def __init__(self):
        self._keys = {(p, legacy): make_key()
                      for p in ("console", "sensor") for legacy in (False, True)}

    def private(self, product, legacy=False):
        return self._keys[(product, legacy)][0]

    def pem(self, kind, *, legacy=False):
        return self._keys[(getattr(kind, "value", kind), bool(legacy))][1]

    def signed(self, product="sensor", *, legacy=False, **kwargs):
        return sign_image(self.private(product, legacy), **kwargs)
