"""Host pre-flight of a signed image: the checks the bootloader will run.

Images are signed in the test with a throwaway key. Needs no hardware.
"""

import pytest

from _dfu_fakes import make_key, sign_image
from omotion.dfu.image import (
    IMAGE_OFFSET,
    ImageVerificationError,
    read_header_version,
    unpack_semver,
    verify_image,
)

pytestmark = pytest.mark.unit


@pytest.fixture(scope="module")
def key():
    return make_key()


def test_good_image_verifies_and_reports_its_header(key):
    priv, pem = key
    body = b"\x5a" * 4096
    info = verify_image(sign_image(priv, body=body, fw_version=10801), pem)
    assert (info.fw_version, info.fw_size) == (10801, 4096)
    assert info.version_str == "10801 (1.8.1)"


def test_trailing_erased_flash_is_accepted(key):
    """A production image carries the signed app followed by 0xFF padding."""
    priv, pem = key
    verify_image(sign_image(priv) + b"\xff" * 512 + b"\x00" * 32, pem)


def test_flipped_header_byte_fails_the_signature(key):
    priv, pem = key
    image = bytearray(sign_image(priv))
    image[6] ^= 0x01                                   # FwVersion
    with pytest.raises(ImageVerificationError, match="signature"):
        verify_image(bytes(image), pem)


def test_flipped_body_byte_fails_the_tag(key):
    priv, pem = key
    image = bytearray(sign_image(priv))
    image[IMAGE_OFFSET + 10] ^= 0x01
    with pytest.raises(ImageVerificationError, match="FwTag"):
        verify_image(bytes(image), pem)


def test_image_signed_with_another_key_is_rejected(key):
    priv, _ = key
    _, other_pem = make_key()
    with pytest.raises(ImageVerificationError, match="signature"):
        verify_image(sign_image(priv), other_pem)


def test_unsigned_binary_is_rejected(key):
    _, pem = key
    with pytest.raises(ImageVerificationError, match="SFU1"):
        verify_image(b"\x00" * 4096, pem)


def test_short_file_is_rejected(key):
    _, pem = key
    with pytest.raises(ImageVerificationError):
        verify_image(b"SFU1" + b"\x00" * 16, pem)


def test_fw_version_zero_is_rejected(key):
    priv, pem = key
    with pytest.raises(ImageVerificationError, match="FwVersion 0"):
        verify_image(sign_image(priv, fw_version=0), pem)


def test_image_older_than_the_minimum_is_a_downgrade(key):
    priv, pem = key
    with pytest.raises(ImageVerificationError, match="downgrade"):
        verify_image(sign_image(priv, fw_version=10801), pem, min_version=10900)


def test_partial_image_header_is_rejected(key):
    priv, pem = key
    with pytest.raises(ImageVerificationError, match="partial"):
        verify_image(sign_image(priv, partial_size=64), pem)


def test_truncated_body_is_rejected(key):
    priv, pem = key
    with pytest.raises(ImageVerificationError, match="truncated"):
        verify_image(sign_image(priv)[:-100], pem)


def test_image_larger_than_the_slot_is_rejected(key):
    priv, pem = key
    with pytest.raises(ImageVerificationError, match="slot"):
        verify_image(sign_image(priv, fw_size=0x00080000), pem)


def test_data_after_the_body_is_rejected(key):
    priv, pem = key
    with pytest.raises(ImageVerificationError, match="after the firmware body"):
        verify_image(sign_image(priv) + b"\x01", pem)


def test_read_header_version_reads_a_slot_header():
    priv, _ = make_key()
    assert read_header_version(sign_image(priv, fw_version=10899)[:8]) == 10899


def test_read_header_version_is_zero_for_an_empty_slot():
    assert read_header_version(b"\xff" * 8) == 0
    assert read_header_version(b"") == 0


def test_unpack_semver_matches_the_signing_encoding():
    assert unpack_semver(10801) == (1, 8, 1)
    assert unpack_semver(10899) == (1, 8, 99)
