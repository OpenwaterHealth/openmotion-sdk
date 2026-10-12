"""Picking one update's images out of a release folder and describing them.

Shared by scripts/update_firmware.py and the test app: what the operator is
shown before confirming a step that cannot be undone. Needs no hardware.
"""

import pytest

from _dfu_fakes import TestKeys, make_production
from omotion import firmware_update
from omotion.firmware_update import (
    FirmwareKind,
    FirmwareUpdateError,
    ImageRejectedError,
    describe_images,
    find_release_images,
)

pytestmark = pytest.mark.unit


@pytest.fixture(scope="module")
def keys():
    return TestKeys()


@pytest.fixture(autouse=True)
def _test_keys(monkeypatch, keys):
    monkeypatch.setattr(firmware_update, "public_key_pem", keys.pem)


@pytest.fixture
def release(tmp_path, keys):
    """A complete release folder for both products, named as the 2026-10-12
    release was."""
    paths = {}
    for product in ("console", "sensor"):
        signed = keys.signed(product, fw_version=10802)
        production = make_production(signed)
        bootloader = production[:0x20000].rstrip(b"\xff")
        files = {
            f"openmotion-{product}-fw-signed.bin": signed,
            f"openmotion-{product}-fw-production.bin": production,
            f"open-motion-{product}-bl-updater.bin": keys.signed(
                product, legacy=True, fw_version=10897,
                body=b"\x00" * 64 + bootloader + b"\x00" * 64),
        }
        for name, data in files.items():
            (tmp_path / name).write_bytes(data)
        paths[product] = find_release_images(tmp_path, product)
    paths["dir"] = tmp_path
    return paths


def test_images_are_picked_per_product(release):
    assert {k: v.name for k, v in release["sensor"].items()} == {
        "app": "openmotion-sensor-fw-signed.bin",
        "updater": "open-motion-sensor-bl-updater.bin",
        "production": "openmotion-sensor-fw-production.bin",
    }
    assert find_release_images(release["dir"], FirmwareKind.CONSOLE) == release["console"]


def test_unknown_product_is_an_error(release):
    with pytest.raises(ValueError):
        find_release_images(release["dir"], "toaster")


def test_a_complete_release_is_described(release):
    described = describe_images("console", **release["console"])

    assert set(described) == {"app", "updater", "production"}
    assert "10802 (1.8.2)" in described["app"] and "1.2.0 or newer" in described["app"]
    assert "10897 (1.8.97)" in described["updater"] and "older than 1.2.0" in described["updater"]
    assert "bootloader" in described["production"] and "1.8.2" in described["production"]


def test_only_the_images_given_are_described(release):
    assert set(describe_images("sensor", release["sensor"]["app"])) == {"app"}


def test_an_application_for_the_other_product_is_rejected_and_named(release):
    with pytest.raises(ImageRejectedError, match="console bootloader 1.2.0 or newer"):
        describe_images("sensor", release["console"]["app"])


def test_an_updater_signed_for_the_new_bootloader_is_rejected(release):
    """The old bootloader has to accept it, or the slot is erased for nothing."""
    with pytest.raises(ImageRejectedError, match="updater"):
        describe_images("console", release["console"]["app"],
                        updater=release["console"]["app"])


def test_a_production_image_for_another_application_is_rejected(release, tmp_path, keys):
    other = tmp_path / "other" / "openmotion-console-fw-production.bin"
    other.parent.mkdir()
    other.write_bytes(make_production(keys.signed("console", body=b"\x42" * 512)))
    with pytest.raises(ImageRejectedError, match="signed application"):
        describe_images("console", release["console"]["app"], production=other)


def test_a_legacy_key_application_is_described_as_such(tmp_path, keys):
    app = tmp_path / "old-signed.bin"
    app.write_bytes(keys.signed("sensor", legacy=True, fw_version=10801))
    assert "older than 1.2.0" in describe_images("sensor", app)["app"]


def test_a_bare_metal_application_is_described_not_rejected(tmp_path):
    app = tmp_path / "fw-baremetal.bin"
    app.write_bytes(b"\x00\x00\x08\x24" + b"\xa5" * 60)
    assert "bare-metal" in describe_images("sensor", app)["app"]


def test_a_missing_file_is_an_error(tmp_path):
    with pytest.raises(FirmwareUpdateError, match="not found"):
        describe_images("sensor", tmp_path / "nope.bin")
