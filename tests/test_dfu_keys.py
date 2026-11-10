"""The bootloader public keys the package ships. Needs no hardware."""

import hashlib

import pytest

from omotion.dfu.keys import public_key_pem
from omotion.firmware_update import FirmwareKind

pytestmark = pytest.mark.unit

# SHA-256 of each PEM as committed in the bootloader repositories (see
# omotion/dfu/PROVENANCE.txt). A changed hash means a different trust anchor.
EXPECTED = {
    ("console", False): "6a5c192f97009714df50ccfdd7130e18c32f5a99be31d00bf56b8742059a919c",
    ("console", True): "b50fa2ea068dc41f95baef5773f87322c5b417e7c0b33267ff1785870df36d4c",
    ("sensor", False): "3f9ddfd47302357f61a8148f80cabe904e12689e0ca1893e19b57aed0453c056",
    ("sensor", True): "2c0c991f11cb04fcee45ba42cc0d97519eb1fbf4ffde5a13bd1b33c18599f9ca",
}


@pytest.mark.parametrize("product, legacy", sorted(EXPECTED))
def test_shipped_key_is_the_one_the_bootloader_repository_committed(product, legacy):
    pem = public_key_pem(product, legacy=legacy)
    assert hashlib.sha256(pem).hexdigest() == EXPECTED[(product, legacy)]


@pytest.mark.parametrize("product, legacy", sorted(EXPECTED))
def test_shipped_key_is_a_p256_public_key(product, legacy):
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric import ec

    key = serialization.load_pem_public_key(public_key_pem(product, legacy=legacy))
    assert isinstance(key.curve, ec.SECP256R1)


def test_the_four_keys_are_distinct():
    """A console image must not verify as a sensor image, nor a legacy-key
    image as a current one."""
    assert len({public_key_pem(p, legacy=l) for p, l in EXPECTED}) == 4


def test_firmware_kind_selects_the_product():
    assert public_key_pem(FirmwareKind.SENSOR) == public_key_pem("sensor")
    assert public_key_pem(FirmwareKind.CONSOLE, legacy=True) == public_key_pem("console", legacy=True)


def test_unknown_product_is_an_error():
    with pytest.raises(ValueError):
        public_key_pem("toaster")
