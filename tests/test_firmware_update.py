import pytest

from omotion.firmware_update import parse_version, is_update_available


@pytest.mark.parametrize("s,expected", [
    ("1.2.3", (1, 2, 3)),
    ("v1.2.3", (1, 2, 3)),
    ("1.5.4-dev.1", (1, 5, 4)),
    ("v2.0.0-rc.2", (2, 0, 0)),
    ("1.5.4+build.7", (1, 5, 4)),
    ("1.2.3-5-g1234abc-dirty", (1, 2, 3)),
])
def test_parse_version_ok(s, expected):
    assert parse_version(s) == expected


@pytest.mark.parametrize("s", ["", "unknown", "vX.Y.Z", "1.2"])
def test_parse_version_rejects(s):
    with pytest.raises(ValueError):
        parse_version(s)


def test_parse_version_none_raises_typeerror():
    with pytest.raises(TypeError):
        parse_version(None)


@pytest.mark.parametrize("installed,latest,expected", [
    ("1.2.3", "1.2.4", True),
    ("1.2.3", "1.3.0", True),
    ("1.2.3", "2.0.0", True),
    ("1.2.3", "1.2.3", False),
    ("1.2.4", "1.2.3", False),
    ("v1.2.3", "v1.2.4", True),
    ("1.2.3-dev.1", "1.2.3", False),   # suffixes ignored: same core, not newer
    ("", "1.2.3", False),               # unparseable installed -> no update
    ("1.2.3", "garbage", False),        # unparseable latest -> no update
])
def test_is_update_available(installed, latest, expected):
    assert is_update_available(installed, latest) is expected


# ---------------------------------------------------------------------------
# Task 3: FirmwareKind + check_latest
# ---------------------------------------------------------------------------
from unittest.mock import MagicMock

from omotion.firmware_update import FirmwareKind, LatestInfo, check_latest


def _fake_gh(latest_release=None, raises=False):
    gh = MagicMock()
    if raises:
        gh.get_latest_release.side_effect = RuntimeError("network down")
    else:
        gh.get_latest_release.return_value = latest_release
        gh.get_asset_list.return_value = [{"name": "motion-sensor-fw.bin"}]
    return gh


def test_check_latest_returns_info():
    gh = _fake_gh({"tag_name": "1.4.0", "published_at": "2026-01-01"})
    info = check_latest(FirmwareKind.SENSOR, releases=gh)
    assert info == LatestInfo(
        kind=FirmwareKind.SENSOR, tag="1.4.0",
        asset_name="motion-sensor-fw.bin", published_at="2026-01-01",
    )


def test_check_latest_none_on_network_error():
    assert check_latest(FirmwareKind.SENSOR, releases=_fake_gh(raises=True)) is None


def test_check_latest_none_when_no_bin_asset():
    gh = _fake_gh({"tag_name": "1.4.0"})
    gh.get_asset_list.return_value = []
    assert check_latest(FirmwareKind.SENSOR, releases=gh) is None


def test_check_latest_none_when_no_tag():
    gh = _fake_gh({"published_at": "x"})  # no tag_name
    assert check_latest(FirmwareKind.CONSOLE, releases=gh) is None


def test_check_latest_falls_back_to_first_bin_when_canonical_missing():
    gh = _fake_gh({"tag_name": "1.4.0", "published_at": "2026-01-01"})
    gh.get_asset_list.return_value = [{"name": "motion-sensor-fw-v1.4.0.bin"}]
    info = check_latest(FirmwareKind.SENSOR, releases=gh)
    assert info is not None
    assert info.asset_name == "motion-sensor-fw-v1.4.0.bin"


# ---------------------------------------------------------------------------
# Task 4: download_firmware + FirmwareUpdater
# ---------------------------------------------------------------------------
from pathlib import Path

from omotion.firmware_update import (
    FirmwareUpdateError, FirmwareUpdater, download_firmware,
)


def test_download_firmware_uses_release_by_tag(tmp_path):
    gh = MagicMock()
    gh.get_release_by_tag.return_value = {"tag_name": "1.4.0"}
    gh.download_asset.return_value = tmp_path / "motion-sensor-fw.bin"
    info = LatestInfo(FirmwareKind.SENSOR, "1.4.0", "motion-sensor-fw.bin")

    out = download_firmware(info, tmp_path, releases=gh)

    gh.get_release_by_tag.assert_called_once_with("1.4.0")
    gh.download_asset.assert_called_once_with(
        {"tag_name": "1.4.0"}, "motion-sensor-fw.bin", output_dir=tmp_path)
    assert out == tmp_path / "motion-sensor-fw.bin"


def test_updater_happy_path(tmp_path):
    handle = MagicMock()
    handle.enter_dfu.return_value = True
    dfu = MagicMock()
    dfu.wait_for_dfu_device.return_value = True
    dfu.flash_bin.return_value = MagicMock(success=True)

    updater = FirmwareUpdater(programmer=dfu)
    result = updater.update(handle, tmp_path / "fw.bin")

    assert result.success is True
    handle.enter_dfu.assert_called_once()
    dfu.flash_bin.assert_called_once()


def test_updater_raises_if_enter_dfu_rejected(tmp_path):
    handle = MagicMock()
    handle.enter_dfu.return_value = False
    updater = FirmwareUpdater(programmer=MagicMock())
    with pytest.raises(FirmwareUpdateError):
        updater.update(handle, tmp_path / "fw.bin")


def test_updater_raises_if_dfu_never_appears(tmp_path):
    handle = MagicMock()
    handle.enter_dfu.return_value = True
    dfu = MagicMock()
    dfu.wait_for_dfu_device.return_value = False
    updater = FirmwareUpdater(programmer=dfu)
    with pytest.raises(FirmwareUpdateError):
        updater.update(handle, tmp_path / "fw.bin")
