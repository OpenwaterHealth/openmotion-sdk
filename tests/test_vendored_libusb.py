"""Every libusb copy the package ships is libusb 1.0.30 or newer (#305).

1.0.30 fixes denial-of-service bugs a malicious USB device can trigger with
malformed descriptors. The package carries libusb in exactly four DLLs: the
pyusb backend's (``_vendor``) and the one the Windows ``dfu-util.exe`` loads
from its own directory, each for x64 and x86. Static libraries, statically
linked tools and the macOS dylib that dfu-util never loaded were removed
instead of kept current, so nothing else may reappear.

Reads the files in this checkout, not the installed ``omotion``. Needs no
hardware.
"""

import struct
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

PKG = Path(__file__).resolve().parents[1] / "omotion"
MIN_LIBUSB = (1, 0, 30)
SHIPPED_DLLS = {
    "_vendor/libusb/windows/x64/libusb-1.0.dll",
    "_vendor/libusb/windows/x86/libusb-1.0.dll",
    "dfu-util/win64/libusb-1.0.dll",
    "dfu-util/win32/libusb-1.0.dll",
}


def _pe_file_version(path: Path) -> tuple[int, int, int, int]:
    """FileVersion from the PE's VS_FIXEDFILEINFO (works for any arch)."""
    data = path.read_bytes()
    i = data.find(struct.pack("<I", 0xFEEF04BD))
    assert i >= 0, f"{path} has no version resource"
    ms, ls = struct.unpack_from("<II", data, i + 8)
    return ms >> 16, ms & 0xFFFF, ls >> 16, ls & 0xFFFF


def test_shipped_libusb_dlls_are_at_least_1_0_30():
    dlls = {p.relative_to(PKG).as_posix(): p for p in PKG.rglob("libusb-1.0.dll")}
    assert set(dlls) == SHIPPED_DLLS
    for rel, path in sorted(dlls.items()):
        version = _pe_file_version(path)
        assert version[:3] >= MIN_LIBUSB, f"{rel} is libusb {version}"


def test_no_other_libusb_binaries_ship():
    stale = sorted(
        p.relative_to(PKG).as_posix()
        for p in PKG.rglob("*")
        if p.is_file()
        and (
            (p.name.startswith("libusb-1.0") and p.name != "libusb-1.0.dll")
            or "-static" in p.name
        )
    )
    assert stale == []
