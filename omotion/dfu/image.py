"""Check a signed SBSFU image on the host BEFORE it is flashed.

Ported from open-motion-console-bl ``py-tools/verify_firmware.py`` (tag 1.2.0,
commit 34178cd); see PROVENANCE.txt. The header constants and the FwVersion
encoding are copied from ``py-tools/sign_firmware.py`` of the same commit, which
owns the layout.

The bootloader has a single application slot: a DFU download erases the installed
image first and SBSFU only verifies the new one on the next boot. A bad file is
therefore refused too late to keep the old firmware. :func:`verify_image` runs
the same checks the bootloader will run, on the file, so a bad image is caught
while the installed one is intact.

Only the ECDSA *public* key is needed: the active slot holds the firmware in
clear, so FwTag can be recomputed from the file.

This is a pre-flight, not a security boundary: the bootloader still verifies
every image at boot and that check cannot be skipped.
"""

from __future__ import annotations

import hashlib
import struct
from dataclasses import dataclass

__all__ = [
    "SFUMAGIC",
    "PROTOCOL_VERSION",
    "HEADER_AUTH_LEN",
    "HEADER_SIGN_LEN",
    "IMAGE_OFFSET",
    "SLOT_SIZE",
    "ImageInfo",
    "ImageVerificationError",
    "read_header_version",
    "unpack_semver",
    "verify_image",
]

# ---------------------------------------------------------------------------
# SBSFU header constants (sign_firmware.py)
# ---------------------------------------------------------------------------
SFUMAGIC          = b"SFU1"
PROTOCOL_VERSION  = 1
HEADER_AUTH_LEN   = 128    # bytes signed by ECDSA
HEADER_SIGN_LEN   = 64     # ECDSA P-256 raw R||S

# Offset, from the slot start, at which SBSFU expects the firmware binary. MUST
# match SFU_IMG_IMAGE_OFFSET in SBSFU/App/Inc/sfu_fwimg_regions.h.
IMAGE_OFFSET      = 0x400

# FwVersion = major * 10000 + minor * 100 + patch (1.8.1 -> 10801). Fielded units
# carry a monotonic anti-rollback floor in this encoding; it must never change.
FWVER_MAJOR_MULT  = 10000
FWVER_MINOR_MULT  = 100
FWVER_MAX         = 0xFFFF   # 6.55.35

# Active slot size. MUST match SLOT_ACTIVE_1 in Linker/mapping_fwimg.ld
# (0x08020000-0x0809FFFF, the same on the console and sensor bootloaders).
SLOT_SIZE = 0x00080000

# Same struct as sign_firmware.py, Step 4.
_AUTH_HEADER_FMT = "<4sHHIII32s32s16s28s"


def unpack_semver(fw_version: int) -> tuple[int, int, int]:
    """Decode the 16-bit FwVersion field to ``(major, minor, patch)``."""
    if not (0 <= fw_version <= FWVER_MAX):
        raise ValueError(f"FwVersion must fit in 16 bits, got {fw_version}")
    major = fw_version // FWVER_MAJOR_MULT
    minor = (fw_version // FWVER_MINOR_MULT) % 100
    patch = fw_version % 100
    return major, minor, patch


class ImageVerificationError(Exception):
    """The image would be rejected by the bootloader."""


# FwVersion is compared as the raw uint16, exactly as the bootloader does (plain
# unsigned `<`); the dotted form is for display only.
@dataclass(frozen=True)
class ImageInfo:
    fw_version: int      # raw uint16 FwVersion from the signed header
    fw_size: int         # firmware body size in bytes
    fw_tag: bytes        # SHA-256 of the firmware body

    @property
    def version_str(self) -> str:
        """'10801 (1.8.1)' — raw value first, since that is what the floor stores."""
        return "{} ({}.{}.{})".format(self.fw_version, *unpack_semver(self.fw_version))


def read_header_version(header: bytes) -> int:
    """
    FwVersion from the start of a slot/image, or 0 if there is no "SFU1" header.
    Not authenticated — use it only to read what is currently installed.
    """
    if len(header) < 8 or header[:4] != SFUMAGIC:
        return 0
    return struct.unpack_from("<H", header, 6)[0]


def verify_image(
    image: bytes,
    public_key_pem: bytes,
    slot_size: int = SLOT_SIZE,
    min_version: int = 0,
) -> ImageInfo:
    """
    Verify a signed image the way the bootloader will. Raises
    ImageVerificationError naming the first check that fails.
    """
    # Imported here so that importing omotion does not load cryptography (and
    # its OpenSSL) for applications that never update firmware.
    from cryptography.exceptions import InvalidSignature
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import ec
    from cryptography.hazmat.primitives.asymmetric.utils import encode_dss_signature

    if len(image) < IMAGE_OFFSET:
        raise ImageVerificationError(
            f"file is {len(image)} bytes, shorter than the 0x{IMAGE_OFFSET:X}-byte header region"
        )

    auth_header = image[:HEADER_AUTH_LEN]
    (magic, protocol, fw_version, fw_size, partial_offset, partial_size,
     fw_tag, partial_tag, _iv, _reserved) = struct.unpack(_AUTH_HEADER_FMT, auth_header)

    if magic != SFUMAGIC:
        raise ImageVerificationError(
            "no 'SFU1' magic: this is not a signed image (raw application .bin?)"
        )
    if protocol != PROTOCOL_VERSION:
        raise ImageVerificationError(f"unsupported ProtocolVersion {protocol}")

    # Signature first: no other header field is trusted until this passes.
    public_key = serialization.load_pem_public_key(public_key_pem)
    raw_sig = image[HEADER_AUTH_LEN:HEADER_AUTH_LEN + HEADER_SIGN_LEN]
    der_sig = encode_dss_signature(
        int.from_bytes(raw_sig[:32], "big"), int.from_bytes(raw_sig[32:], "big")
    )
    try:
        public_key.verify(der_sig, auth_header, ec.ECDSA(hashes.SHA256()))
    except InvalidSignature:
        raise ImageVerificationError(
            "header signature is invalid: the header was modified, or the image "
            "was signed with a different key than this bootloader trusts"
        ) from None

    if fw_version == 0:
        raise ImageVerificationError("FwVersion 0 is invalid")
    if fw_version < min_version:
        raise ImageVerificationError(
            f"FwVersion {fw_version} is older than {min_version}: the bootloader "
            "refuses downgrades"
        )

    if partial_offset != 0 or partial_size != 0 or partial_tag != fw_tag:
        raise ImageVerificationError("header describes a partial image; only full images are supported")
    if fw_size == 0 or fw_size > slot_size - IMAGE_OFFSET:
        raise ImageVerificationError(
            f"FwSize {fw_size} does not fit the {slot_size // 1024} KB application slot"
        )
    body_end = IMAGE_OFFSET + fw_size
    if len(image) < body_end:
        raise ImageVerificationError(
            f"file is truncated: header declares {fw_size} firmware bytes, "
            f"file holds {len(image) - IMAGE_OFFSET}"
        )

    if hashlib.sha256(image[IMAGE_OFFSET:body_end]).digest() != fw_tag:
        raise ImageVerificationError(
            "firmware body does not match FwTag: the file is corrupted or was modified after signing"
        )

    # VerifySlot (sfu_fwimg_common.c) accepts only 0x00 or 0xFF beyond the body.
    if image[body_end:].strip(b"\x00\xFF"):
        raise ImageVerificationError("unexpected data after the firmware body")

    return ImageInfo(fw_version=fw_version, fw_size=fw_size, fw_tag=fw_tag)
