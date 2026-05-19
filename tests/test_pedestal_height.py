"""
Unit tests for sensor firmware version parsing used by pedestal selection.

These exercise the pure helper ``_parse_firmware_version`` so they do not
require any connected hardware.
"""

from unittest.mock import MagicMock

import pytest

import omotion.MotionProcessing as _mp
from omotion.MotionSensor import MotionSensor, _parse_firmware_version


@pytest.mark.parametrize(
    "raw, expected",
    [
        # Plain release-style versions
        ("v1.5.2", (1, 5, 2)),
        ("1.5.2", (1, 5, 2)),
        ("v1.5.3", (1, 5, 3)),
        ("1.5.4", (1, 5, 4)),
        # Pre-release / dev suffixes (the original bug)
        ("1.5.4-dev", (1, 5, 4)),
        ("v1.5.4-dev", (1, 5, 4)),
        ("1.5.4-dev.0", (1, 5, 4)),
        ("1.5.4-rc.1", (1, 5, 4)),
        # git-describe output: <tag>-<n>-g<sha>[-dirty]
        ("1.5.4-5-g1234abc", (1, 5, 4)),
        ("1.5.4-dev.0-5-g1234abc", (1, 5, 4)),
        ("1.5.4-dirty", (1, 5, 4)),
        ("1.5.4-5-g1234abc-dirty", (1, 5, 4)),
        # Build-metadata form (+build.N)
        ("1.5.4+build.7", (1, 5, 4)),
    ],
)
def test_parse_firmware_version_valid(raw, expected):
    assert _parse_firmware_version(raw) == expected


@pytest.mark.parametrize(
    "raw",
    [
        "",
        "unknown",
        "v",
        "vunknown",
        "not-a-version",
        None,
    ],
)
def test_parse_firmware_version_invalid(raw):
    """Strings with no leading numeric component must raise ValueError."""
    with pytest.raises((ValueError, TypeError)):
        _parse_firmware_version(raw)


def _make_sensor_stub(version: str) -> MotionSensor:
    """Construct a MotionSensor without going through __init__/USB enumeration."""
    s = MotionSensor.__new__(MotionSensor)
    s.get_version = MagicMock(return_value=version)
    return s


@pytest.mark.parametrize(
    "version, expected_pedestal",
    [
        ("v1.5.1", 64.0),
        ("v1.5.2", 64.0),
        ("v1.5.3", 128.0),
        ("v1.5.4", 128.0),
        # Regression: dev / git-describe builds were silently leaving the
        # module-level default of 64 in place. Anything that parses as
        # >(1,5,2) must select 128.
        ("1.5.4-dev", 128.0),
        ("1.5.4-dev.0-5-g1234abc", 128.0),
        ("1.5.4-dirty", 128.0),
        ("v2.0.0", 128.0),
    ],
)
def test_refresh_pedestal_height_selects_correct_value(version, expected_pedestal):
    saved = _mp.PEDESTAL_HEIGHT
    try:
        _mp.PEDESTAL_HEIGHT = -1.0  # poison value so we can see whether it was set
        sensor = _make_sensor_stub(version)
        sensor._refresh_pedestal_height()
        assert _mp.PEDESTAL_HEIGHT == expected_pedestal, (
            f"version={version!r} expected pedestal {expected_pedestal} "
            f"but got {_mp.PEDESTAL_HEIGHT}"
        )
    finally:
        _mp.PEDESTAL_HEIGHT = saved


def test_refresh_pedestal_height_unknown_version_keeps_default(caplog):
    """Unparseable versions must leave PEDESTAL_HEIGHT untouched and warn."""
    saved = _mp.PEDESTAL_HEIGHT
    try:
        _mp.PEDESTAL_HEIGHT = 99.0
        sensor = _make_sensor_stub("unknown")
        with caplog.at_level("WARNING"):
            sensor._refresh_pedestal_height()
        assert _mp.PEDESTAL_HEIGHT == 99.0
        assert any(
            "pedestal" in rec.message.lower() for rec in caplog.records
        ), "expected a warning mentioning pedestal selection"
    finally:
        _mp.PEDESTAL_HEIGHT = saved
