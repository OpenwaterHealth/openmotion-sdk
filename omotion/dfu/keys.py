"""The ECDSA P-256 *public* keys the Open-Motion bootloaders trust.

Each bootloader build has one key compiled in. Bootloader 1.2.0 moved both
products to new keys (held in Cloud KMS), so there are two generations per
product:

  current   what bootloader 1.2.0 and newer accept
  legacy    what the fielded bootloaders accept (console 1.0.0; sensor 1.0.0 and
            1.1.0). The bootloader-updater image is signed with this one, which
            is how an old bootloader can be made to run it.

Sources are recorded in PROVENANCE.txt. These are public keys: they let the host
predict what a bootloader will refuse, nothing more.
"""

from __future__ import annotations

from importlib import resources

__all__ = ["public_key_pem"]

_KEY_FILES = {
    ("console", False): "console_bl_1.2.pem",
    ("console", True): "console_bl_legacy_1.0.0.pem",
    ("sensor", False): "sensor_bl_1.2.pem",
    ("sensor", True): "sensor_bl_legacy_1.1.0.pem",
}


def public_key_pem(kind, *, legacy: bool = False) -> bytes:
    """PEM of the key ``kind``'s bootloader trusts.

    ``kind`` is a :class:`omotion.firmware_update.FirmwareKind` or its value
    (``"console"`` / ``"sensor"``).
    """
    product = getattr(kind, "value", kind)
    try:
        name = _KEY_FILES[(product, bool(legacy))]
    except KeyError:
        raise ValueError(f"no bootloader key for product {product!r}") from None
    return resources.files("omotion.dfu").joinpath("keys", name).read_bytes()
