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
