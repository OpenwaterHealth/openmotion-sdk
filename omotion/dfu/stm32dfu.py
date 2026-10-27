"""
stm32dfu.py — Pure-Python USB DFU class for the Open-Motion bootloaders and the
STM32 ROM loader.

Ported from open-motion-console-bl ``py-tools/stm32dfu.py`` (tag 1.2.0, commit
34178cd), the implementation the bootloader 1.2.0 migration was bench-tested
with. Local changes are numbered in PROVENANCE.txt; keep that list current when
re-syncing with the bootloader repository.

Implements the STM32 DFU sub-protocol (AN3156) over pyusb:
  - Sector erase + block download (flash write)
  - Block upload (flash read)
  - Download verification (read-back compare)
  - Device reset / leave-DFU

Windows driver note:
    pyusb needs the DFU device (0483:DF11) bound to WinUSB. The SDK's
    ``winusb-driver`` package installs that binding; Zadig does the same by hand.
"""

from __future__ import annotations

import logging
import re
import struct
import sys
import time

import usb.core
import usb.util

from omotion.dfu.errors import (
    DFUAmbiguousDeviceError,
    DFUBackendError,
    DFUDeviceNotFoundError,
    DFUError,
    DFUTimeoutError,
    DFUVerifyError,
)
from omotion.usb_backend import get_libusb1_backend

logger = logging.getLogger(__name__)

_WINUSB_HINT = (
    "On Windows the DFU device (0483:DF11) must be bound to the WinUSB driver; "
    "install the Open-Motion WinUSB driver package (or use Zadig)."
)


# ── DFU state / status constants ────────────────────────────────────────────

class DFUState:
    APP_IDLE              = 0
    APP_DETACH            = 1
    DFU_IDLE              = 2
    DFU_DNLOAD_SYNC       = 3
    DFU_DNLOAD_BUSY       = 4
    DFU_DNLOAD_IDLE       = 5
    DFU_MANIFEST_SYNC     = 6
    DFU_MANIFEST          = 7
    DFU_MANIFEST_WAIT_RST = 8
    DFU_UPLOAD_IDLE       = 9
    DFU_ERROR             = 10

    NAMES = {
        0: "appIDLE",           1: "appDETACH",
        2: "dfuIDLE",           3: "dfuDNLOAD-SYNC",
        4: "dfuDNLOAD-BUSY",    5: "dfuDNLOAD-IDLE",
        6: "dfuMANIFEST-SYNC",  7: "dfuMANIFEST",
        8: "dfuMANIFEST-WAIT-RESET",
        9: "dfuUPLOAD-IDLE",   10: "dfuERROR",
    }

    @classmethod
    def name(cls, state):
        return cls.NAMES.get(state, f"UNKNOWN({state})")


class DFUStatus:
    OK                = 0x00
    ERROR_TARGET      = 0x01
    ERROR_FILE        = 0x02
    ERROR_WRITE       = 0x03
    ERROR_ERASE       = 0x04
    ERROR_CHECK_ERASED= 0x05
    ERROR_PROG        = 0x06
    ERROR_VERIFY      = 0x07
    ERROR_ADDRESS     = 0x08
    ERROR_NOTDONE     = 0x09
    ERROR_FIRMWARE    = 0x0A
    ERROR_VENDOR      = 0x0B
    ERROR_USB         = 0x0C
    ERROR_POR         = 0x0D
    ERROR_UNKNOWN     = 0x0E
    ERROR_STALLEDPKT  = 0x0F

    NAMES = {
        0x00: "OK",            0x01: "errTARGET",  0x02: "errFILE",
        0x03: "errWRITE",      0x04: "errERASE",   0x05: "errCHECK_ERASED",
        0x06: "errPROG",       0x07: "errVERIFY",  0x08: "errADDRESS",
        0x09: "errNOTDONE",    0x0A: "errFIRMWARE",0x0B: "errVENDOR",
        0x0C: "errUSB",        0x0D: "errPOR",     0x0E: "errUNKNOWN",
        0x0F: "errSTALLEDPKT",
    }

    @classmethod
    def name(cls, status):
        return cls.NAMES.get(status, f"UNKNOWN({status:#04x})")


# ── DFU flavour detection ────────────────────────────────────────────────────
# A unit in DFU is either our secure bootloader (one alt setting whose DfuSe
# descriptor has read-only 'a' runs) or the STM32 ROM loader of a bare-metal unit
# (four alts: Internal Flash, Option Bytes, OTP Memory, Device Feature). Same
# VID/PID, so only the alt-setting strings tell them apart.
ROM_ONLY_ALTS = ("option bytes", "otp memory", "device feature")
ROM_FLASH_START = 0x08000000
ROM_FLASH_END   = 0x08200000
_SECTOR_RUN_RE  = re.compile(r"\d+\s*\*\s*\d+\s*[KMB]?\s*([a-g])")


def classify_alt_names(names):
    """'bootloader' | 'rom' | 'unknown' from the interface-0 alt-setting strings."""
    names = [n for n in (names or []) if n]
    if not names:
        return "unknown"
    if any(m in n.lower() for n in names for m in ROM_ONLY_ALTS):
        return "rom"
    if len(names) == 1 and any(a == "a" for a in _SECTOR_RUN_RE.findall(names[0])):
        return "bootloader"
    return "unknown"


# ── libusb backend and device discovery ──────────────────────────────────────

def _backend():
    """The SDK's libusb-1.0 backend (the vendored DLL on Windows)."""
    try:
        backend = get_libusb1_backend()
    except OSError as e:
        raise DFUBackendError(f"libusb-1.0 could not be loaded: {e}") from e
    if backend is None:
        raise DFUBackendError("libusb-1.0 could not be loaded")
    return backend


def _find_devices(vid, pid):
    """Every attached device with this VID/PID."""
    try:
        return list(usb.core.find(idVendor=vid, idProduct=pid,
                                  find_all=True, backend=_backend()) or [])
    except usb.core.USBError as e:
        raise DFUError(f"USB enumeration failed: {e}") from e


def _serial_of(dev):
    try:
        return usb.util.get_string(dev, dev.iSerialNumber) if dev.iSerialNumber else None
    except Exception:
        return None


def _port_path(dev):
    """(bus, port chain): where the device is plugged in. Survives a bus reset."""
    try:
        return (dev.bus, tuple(dev.port_numbers or ()))
    except Exception:
        return None


# ── Main class ───────────────────────────────────────────────────────────────

class STM32DFU:
    """
    USB DFU interface for the Open-Motion bootloaders and the STM32 ROM loader.

    Typical usage::

        with STM32DFU() as dfu:
            dfu.connect()
            dfu.download(0x08020000, open("app-signed.bin","rb").read())
    """

    STM32_VID = 0x0483
    STM32_PID = 0xDF11

    # DFU class request codes
    _REQ_DETACH    = 0
    _REQ_DNLOAD    = 1
    _REQ_UPLOAD    = 2
    _REQ_GETSTATUS = 3
    _REQ_CLRSTATUS = 4
    _REQ_GETSTATE  = 5
    _REQ_ABORT     = 6

    # STM32 special commands (wBlockNum=0 DNLOAD payload)
    _CMD_GETCOMMANDS      = 0x00
    _CMD_SETADDRESSPOINTER= 0x21
    _CMD_ERASE            = 0x41

    # bmRequestType constants
    _H2D = 0x21   # Class | Interface | Host→Device
    _D2H = 0xA1   # Class | Interface | Device→Host

    # STM32H743 flash geometry
    FLASH_SECTOR_SIZE    = 128 * 1024   # 128 KB per sector, both banks

    # Writable region exposed by the secure bootloader's DFU interface
    # (usbd_dfu_if.c APP_FLASH_BASE .. FLASH_END_ADDR; see Core/Inc/memory_map.h).
    # Clamped to the SBSFU active slot only: the bootloader (sector 0), reserved
    # sectors, the anti-rollback floor and the user-config sector are all
    # read-only over DFU.
    APP_FLASH_START = 0x08020000   # SBSFU active slot start (signed image goes here)
    APP_FLASH_END   = 0x080A0000   # exclusive end of writable region (active-slot end + 1)

    # Virtual UPLOAD address that returns the bootloader version string
    # (usbd_dfu_if.c DFU_VERSION_VIRT_ADDR / DFU_VERSION_READ_LEN).
    DFU_VERSION_ADDR = 0xFFFFFF00
    DFU_VERSION_LEN  = 64

    # Virtual DNLOAD address that asks the bootloader to reset without flashing
    # (usbd_dfu_if.c DFU_RESET_VIRT_ADDR). The DFU class is manifestation-
    # tolerant, so a plain DfuSe "leave" (zero-length DNLOAD) no longer resets
    # the device by itself; the bootloader main loop resets on this request.
    # Bootloader 1.2.0 and newer only: 1.0.0 and 1.1.0 do not implement it.
    DFU_RESET_ADDR   = 0xFFFFFF08

    def __init__(self, vid=None, pid=None, xfer_size=1024):
        """
        Args:
            vid:        USB Vendor ID  (default 0x0483)
            pid:        USB Product ID (default 0xDF11)
            xfer_size:  DFU transfer size — must be <= USBD_DFU_XFER_SIZE in the
                        bootloader firmware (1024). This is a fallback; connect()
                        auto-detects wTransferSize from the device's DFU
                        functional descriptor.
        """
        self.vid       = vid or self.STM32_VID
        self.pid       = pid or self.STM32_PID
        self.xfer_size = xfer_size
        self._dev      = None
        self._iface    = 0
        # Identity of the connected device, so a reopen after a bus reset lands
        # on the same unit.
        self._serial   = None
        self._port     = None
        # Writable window. Defaults to the secure bootloader's slot; set_rom_mode()
        # widens it for the STM32 ROM loader (bare-metal units).
        self.flash_start = self.APP_FLASH_START
        self.flash_end   = self.APP_FLASH_END
        self.rom_mode    = False

    # ── context manager ──────────────────────────────────────────────────────

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.disconnect()

    # ── device discovery ─────────────────────────────────────────────────────

    @classmethod
    def list_devices(cls):
        """
        Return a list of dicts describing connected DFU devices::

            [{"vid": 0x0483, "pid": 0xDF11, "serial": "316C347D3437",
              "bus": 1, "address": 3, "product": "STM32 DownLoad Firmware Update"}]
        """
        result = []
        for dev in _find_devices(cls.STM32_VID, cls.STM32_PID):
            def _str(idx):
                try:
                    return usb.util.get_string(dev, idx) if idx else None
                except Exception:
                    return None
            result.append({
                "vid":     dev.idVendor,
                "pid":     dev.idProduct,
                "serial":  _str(dev.iSerialNumber),
                "product": _str(dev.iProduct),
                "bus":     dev.bus,
                "address": dev.address,
            })
        return result

    @classmethod
    def wait_for_device(cls, timeout_s, present=True, serial=None):
        """Poll until a DFU device is present (or absent). Returns True on success."""
        deadline = time.monotonic() + timeout_s
        while True:
            devs = _find_devices(cls.STM32_VID, cls.STM32_PID)
            if serial:
                devs = [d for d in devs if _serial_of(d) == serial]
            if bool(devs) == present:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.25)

    def alt_setting_names(self):
        """Interface-0 alt-setting strings of the connected device."""
        names = []
        try:
            cfg = self._dev.get_active_configuration()
        except Exception:
            return names
        for intf in cfg:
            if intf.bInterfaceNumber != self._iface:
                continue
            try:
                names.append(usb.util.get_string(self._dev, intf.iInterface) if intf.iInterface else "")
            except Exception:
                names.append("")
        return names

    def detect_mode(self):
        """('bootloader' | 'rom' | 'unknown', alt-setting names) for the connected device."""
        names = self.alt_setting_names()
        return classify_alt_names(names), names

    def set_rom_mode(self):
        """Address the STM32 ROM loader: alt 0 (Internal Flash), whole-flash window.

        Only valid after detect_mode() returned 'rom'.
        """
        try:
            self._dev.set_interface_altsetting(interface=self._iface, alternate_setting=0)
        except Exception:
            pass
        self.flash_start = ROM_FLASH_START
        self.flash_end   = ROM_FLASH_END
        self.rom_mode    = True

    def erase_range(self, start, end, progress_cb=None):
        """Erase every 128 KB sector in [start, end) within the writable window."""
        if start < self.flash_start or end > self.flash_end or start >= end:
            raise DFUError(f"Erase range {start:#010x}..{end:#010x} outside the writable window "
                           f"{self.flash_start:#010x}..{self.flash_end:#010x}")
        sectors = list(range((start // self.FLASH_SECTOR_SIZE) * self.FLASH_SECTOR_SIZE, end,
                             self.FLASH_SECTOR_SIZE))
        for i, sec in enumerate(sectors):
            msg = f"Erasing sector {i + 1}/{len(sectors)} @ {sec:#010x}"
            if progress_cb:
                progress_cb(i, len(sectors), msg)
            else:
                logger.info(msg)
            self.erase_sector(sec)
        if progress_cb:
            progress_cb(len(sectors), len(sectors))

    # ── connect / disconnect ─────────────────────────────────────────────────

    def connect(self, serial=None):
        """
        Open the DFU device.

        Args:
            serial: Optional USB serial-number string to select a specific
                    device when multiple are attached.

        Raises:
            DFUDeviceNotFoundError:  no matching device.
            DFUAmbiguousDeviceError: several devices and no ``serial``.
        """
        devs = _find_devices(self.vid, self.pid)
        if serial:
            devs = [d for d in devs if _serial_of(d) == serial]
            if not devs:
                raise DFUDeviceNotFoundError(f"No DFU device with serial {serial!r} found")
        elif not devs:
            raise DFUDeviceNotFoundError(
                f"No DFU device found (VID={self.vid:#06x} PID={self.pid:#06x})")
        elif len(devs) > 1:
            raise DFUAmbiguousDeviceError(
                f"{len(devs)} DFU devices are attached; connect exactly one unit, "
                "or select one by serial number")
        dev = devs[0]

        self._open(dev)
        self._serial = serial or _serial_of(dev)
        self._port   = _port_path(dev)

        # Auto-detect the DFU transfer size from the functional descriptor so we
        # never send blocks larger than the device accepts (1024).
        detected = self._read_dfu_transfer_size(dev)
        if detected:
            self.xfer_size = detected

        # Recover from any leftover error state
        self._ensure_idle()

        # A refused out-of-window UPLOAD can leave the device's DFU state
        # machine in a state that neither CLRSTATUS nor ABORT clears. A USB bus
        # reset re-initialises the DFU class (no device reset, nothing flashed),
        # so do that once and reopen the device.
        try:
            _, _, state, _ = self.get_status()
        except Exception:
            state = None
        for attempt in range(2):
            if state in (DFUState.DFU_IDLE, DFUState.DFU_DNLOAD_IDLE,
                         DFUState.DFU_UPLOAD_IDLE):
                return
            try:
                self._dev.reset()
            except Exception:
                pass
            self.disconnect()
            time.sleep(2.5)
            state = self._reopen_after_reset()
        if state not in (DFUState.DFU_IDLE, DFUState.DFU_DNLOAD_IDLE,
                         DFUState.DFU_UPLOAD_IDLE):
            raise DFUError(f"DFU device stuck in state {DFUState.name(state)} after a USB reset")

    def _open(self, dev):
        """Configure and claim ``dev``."""
        try:
            dev.set_configuration()

            if sys.platform != "win32":
                try:
                    if dev.is_kernel_driver_active(self._iface):
                        dev.detach_kernel_driver(self._iface)
                except Exception:
                    pass

            usb.util.claim_interface(dev, self._iface)
        except NotImplementedError as e:
            # libusb reports NOT_SUPPORTED when Windows has no WinUSB binding.
            raise DFUBackendError(f"cannot open the DFU device: {e}. {_WINUSB_HINT}") from e
        except usb.core.USBError as e:
            raise DFUError(f"cannot open the DFU device: {e}. {_WINUSB_HINT}") from e
        self._dev = dev

    def _same_device(self):
        """The device connect() opened, found again after a bus reset, or None."""
        devs = _find_devices(self.vid, self.pid)
        if self._serial:
            devs = [d for d in devs if _serial_of(d) == self._serial]
        elif self._port is not None:
            devs = [d for d in devs if _port_path(d) == self._port]
        return devs[0] if len(devs) == 1 else None

    def _reopen_after_reset(self):
        """Reopen the device after a bus reset; returns the DFU state seen."""
        dev = None
        for _ in range(10):
            dev = self._same_device()
            if dev is not None:
                break
            time.sleep(0.5)
        if dev is None:
            raise DFUError("DFU device did not come back after a USB reset")
        self._open(dev)
        self._ensure_idle()
        try:
            _, _, state, _ = self.get_status()
        except Exception:
            state = None
        return state

    @staticmethod
    def _read_dfu_transfer_size(dev):
        """Parse wTransferSize (offset 5-6) from the DFU functional descriptor
        (bDescriptorType 0x21) in the active configuration. Returns None on
        failure, in which case the constructor default is kept."""
        try:
            cfg = dev.get_active_configuration()
            for intf in cfg:
                extra = bytes(getattr(intf, "extra_descriptors", b"") or b"")
                i = 0
                while i + 2 <= len(extra):
                    blen = extra[i]
                    if blen < 2 or i + blen > len(extra):
                        break
                    if extra[i + 1] == 0x21 and blen >= 7:   # DFU functional descriptor
                        return extra[i + 5] | (extra[i + 6] << 8)
                    i += blen
        except Exception:
            pass
        return None

    def disconnect(self):
        """Release the USB interface."""
        if self._dev is not None:
            try:
                usb.util.release_interface(self._dev, self._iface)
                usb.util.dispose_resources(self._dev)
            except Exception:
                pass
            self._dev = None

    # ── low-level DFU requests ───────────────────────────────────────────────

    def _ctrl(self, request_type, request, value, data_or_length):
        """One DFU class control transfer; USB failures surface as DFUError."""
        if self._dev is None:
            raise DFUError("not connected to a DFU device")
        try:
            return self._dev.ctrl_transfer(
                request_type, request, value, self._iface, data_or_length)
        except usb.core.USBError as e:
            raise DFUError(f"USB transfer failed: {e}") from e

    def get_status(self):
        """
        DFU_GETSTATUS → (bStatus, bwPollTimeout_ms, bState, iString)
        """
        data = self._ctrl(self._D2H, self._REQ_GETSTATUS, 0, 6)
        status      = data[0]
        poll_ms     = data[1] | (data[2] << 8) | (data[3] << 16)
        state       = data[4]
        istring     = data[5]
        return status, poll_ms, state, istring

    def get_state(self):
        """DFU_GETSTATE → state byte."""
        data = self._ctrl(self._D2H, self._REQ_GETSTATE, 0, 1)
        return data[0]

    def clear_status(self):
        """DFU_CLRSTATUS — clears error, returns to dfuIDLE."""
        self._ctrl(self._H2D, self._REQ_CLRSTATUS, 0, None)

    def abort(self):
        """DFU_ABORT — abort current op, return to dfuIDLE."""
        self._ctrl(self._H2D, self._REQ_ABORT, 0, None)

    # ── STM32 special commands ───────────────────────────────────────────────

    def set_address(self, address):
        """Send STM32 Set Address Pointer command."""
        payload = bytes([self._CMD_SETADDRESSPOINTER]) + struct.pack("<I", address)
        self._dnload(0, payload)
        self._poll_idle()

    def erase_sector(self, address):
        """
        Erase the 128 KB flash sector whose start address is `address`.
        `address` must be sector-aligned (multiple of 128 KB).
        """
        if address % self.FLASH_SECTOR_SIZE:
            # round down to sector start
            address = (address // self.FLASH_SECTOR_SIZE) * self.FLASH_SECTOR_SIZE
        payload = bytes([self._CMD_ERASE]) + struct.pack("<I", address)
        self._dnload(0, payload)
        self._poll_idle(timeout_s=30.0)   # erase can take up to ~500 ms per sector

    def erase_all(self, progress_cb=None):
        """
        Erase every sector of the current writable window: the application slot
        (0x08020000–0x0809FFFF) on a bootloader unit.

        Sector 0 (0x08000000–0x0801FFFF) holds the read-only bootloader and is
        intentionally skipped.

        Args:
            progress_cb: Optional callable(done: int, total: int, msg: str="").
                         ``done`` counts sectors completed; ``total`` is the
                         total number of sectors to erase.
        """
        sector = self.flash_start
        sectors = []
        while sector < self.flash_end:
            sectors.append(sector)
            sector += self.FLASH_SECTOR_SIZE

        total = len(sectors)
        for i, sec in enumerate(sectors):
            msg = f"Erasing sector {i+1}/{total} @ {sec:#010x}"
            if progress_cb:
                progress_cb(i, total, msg)
            else:
                logger.info(msg)
            self.erase_sector(sec)

        if progress_cb:
            progress_cb(total, total)
        else:
            logger.info("Erase complete: %d sectors (%d KB) erased.",
                        total, total * self.FLASH_SECTOR_SIZE // 1024)

    # ── high-level operations ────────────────────────────────────────────────

    def download(self, address, data, progress_cb=None, pre_erased=False):
        """
        Erase affected flash sectors and write `data` starting at `address`.

        Args:
            address:     Target flash address. For the secure bootloader this is
                         the SBSFU active-slot start, 0x08020000 (APP_FLASH_START),
                         and `data` must be a SIGNED image.
            data:        Bytes-like firmware image (signed .bin for secure boot).
            progress_cb: Optional callable(done: int, total: int, msg: str="").
                         Called after each erased sector and each written block.

        After the download completes the device issues NVIC_SystemReset(); the
        bootloader then verifies the signature and launches the app if valid.
        """
        data  = bytes(data)
        total = len(data)
        if total == 0:
            raise ValueError("Empty firmware image")

        if address < self.flash_start or (address + total) > self.flash_end:
            raise DFUError(
                f"Target range {address:#010x}..{address + total:#010x} is outside the "
                f"writable region {self.flash_start:#010x}.."
                f"{self.flash_end:#010x} (sector 0 holds the read-only bootloader).")

        # ── erase affected sectors (unless the caller already did) ──────────
        sector_start = (address // self.FLASH_SECTOR_SIZE) * self.FLASH_SECTOR_SIZE
        end_address  = address + total
        sectors = []
        s = sector_start
        while s < end_address and not pre_erased:
            sectors.append(s)
            s += self.FLASH_SECTOR_SIZE

        for i, sec in enumerate(sectors):
            msg = f"Erasing sector {i+1}/{len(sectors)} @ {sec:#010x}"
            if progress_cb:
                progress_cb(0, total, msg)
            self.erase_sector(sec)

        # ── set address pointer ─────────────────────────────────────────────
        self.set_address(address)

        # ── write blocks ────────────────────────────────────────────────────
        block_num = 2       # DFU data blocks start at wBlockNum=2
        offset    = 0

        while offset < total:
            chunk = data[offset : offset + self.xfer_size]

            # Pad to 32-byte flash word boundary (STM32H7 requirement)
            remainder = len(chunk) % 32
            if remainder:
                chunk = chunk + b'\xff' * (32 - remainder)

            self._dnload(block_num, chunk)
            self._poll_idle()

            offset    += self.xfer_size
            block_num += 1

            if progress_cb:
                progress_cb(min(offset, total), total)

        # ── trigger manifestation (zero-length DNLOAD) ──────────────────────
        # This sends the device into MANIFEST → NVIC_SystemReset()
        self._dnload(0, b'')
        try:
            self.get_status()   # may time out / fail as device resets
        except Exception:
            pass

    def upload(self, address, length):
        """
        Read `length` bytes from device flash starting at `address`.

        Returns:
            bytes
        """
        self.set_address(address)
        # DfuSe leaves us in dfuDNLOAD-IDLE after Set Address Pointer; UPLOAD is
        # only served from dfuIDLE/dfuUPLOAD-IDLE, so abort back to dfuIDLE first.
        self.abort()

        result    = bytearray()
        block_num = 2
        remaining = length

        while remaining > 0:
            chunk_len = min(remaining, self.xfer_size)
            chunk = bytes(self._ctrl(self._D2H, self._REQ_UPLOAD, block_num, chunk_len))
            result.extend(chunk)
            remaining -= chunk_len
            block_num += 1

        self.abort()    # return to dfuIDLE without resetting
        return bytes(result)

    def read_version(self):
        """
        Read the bootloader version string (FW_VERSION) via the virtual DFU
        version address. Returns the decoded string with null padding stripped.
        """
        if self.rom_mode:
            raise DFUError("the STM32 ROM loader has no version string (bare-metal unit)")
        raw = self.upload(self.DFU_VERSION_ADDR, self.DFU_VERSION_LEN)
        return raw.split(b"\x00", 1)[0].decode("ascii", errors="replace").strip()

    def verify(self, address, data, progress_cb=None):
        """
        Read back the programmed region and compare with `data`.

        Returns:
            True if every byte matches.

        Raises:
            DFUVerifyError: on a short read-back or the first mismatch
                            (includes offset and values).
        """
        data     = bytes(data)
        readback = self.upload(address, len(data))

        if len(readback) != len(data):
            raise DFUVerifyError(
                f"Verify read back {len(readback)} of {len(data)} bytes at {address:#010x}")

        for i, (expected, actual) in enumerate(zip(data, readback)):
            if expected != actual:
                raise DFUVerifyError(
                    f"Verify mismatch at {address + i:#010x}: "
                    f"expected {expected:#04x}, got {actual:#04x}")

        return True

    def leave_dfu(self):
        """
        Reset the device without downloading firmware.
        The bootloader will verify and launch the app if a valid signed image
        exists in the active slot.
        """
        if self.rom_mode:
            # DfuSe manifestation as dfu-util's ":leave" does it: address pointer at the
            # image base, then a zero-length DNLOAD; the ROM loader jumps to the image.
            self.set_address(ROM_FLASH_START)
            self._dnload(0, b'')
            try:
                self.get_status()
            except Exception:
                pass
            return
        self.set_address(self.DFU_RESET_ADDR)
        self._dnload(2, b'\x00' * 32)      # any payload: the address is the request
        try:
            self._poll_idle()
        except Exception:
            pass

    # ── internal helpers ─────────────────────────────────────────────────────

    def _dnload(self, block_num, data):
        self._ctrl(self._H2D, self._REQ_DNLOAD, block_num, bytes(data))

    def _poll_idle(self, timeout_s=10.0):
        """
        Poll GETSTATUS until the state is dfuDNLOAD-IDLE or dfuIDLE.
        Respects the bwPollTimeout from each status response.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status, poll_ms, state, _ = self.get_status()
            if status != DFUStatus.OK:
                raise DFUError(
                    f"Device error: {DFUStatus.name(status)} "
                    f"in state {DFUState.name(state)}")
            if state in (DFUState.DFU_DNLOAD_IDLE, DFUState.DFU_IDLE):
                return
            if state == DFUState.DFU_ERROR:
                raise DFUError(
                    f"dfuERROR: {DFUStatus.name(status)}")
            time.sleep(max(poll_ms / 1000.0, 0.001))
        raise DFUTimeoutError(f"Timed out after {timeout_s:.0f}s waiting for dfuIDLE")

    def _ensure_idle(self):
        """Recover to dfuIDLE regardless of current state."""
        try:
            _, _, state, _ = self.get_status()
        except Exception:
            return
        if state == DFUState.DFU_ERROR:
            self.clear_status()
        elif state not in (DFUState.DFU_IDLE, DFUState.DFU_DNLOAD_IDLE,
                           DFUState.DFU_UPLOAD_IDLE):
            self.abort()
