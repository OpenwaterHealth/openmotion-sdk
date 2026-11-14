"""Tests for omotion/exposure/firmware.py: which consoles take the extended limits.

Pure-software. The gate fails closed: only an exact release tag known to
predate console-fw 1.8.2-rc.3's safety envelope passes.
"""

import pytest

from omotion.exposure.firmware import (
    ExtendedLimitsRefused,
    extended_limits_problem,
    require_extended_limits,
)

pytestmark = pytest.mark.unit

# Every console-fw tag, as of 1.8.2-rc.4, checked for Core/Inc/safety_envelope.h.
PRE_ENVELOPE_TAGS = [
    "1.2.1", "1.5.4-rc1", "1.5.8-rc.2", "1.7.0", "1.8.0-dev.0", "1.8.0", "1.8.1-rc.2",
    "1.8.1", "1.8.2-dev.1", "1.8.2-dev.3", "1.8.2-dev.4", "1.8.2-dev.5", "1.8.2-rc.1",
    "1.8.2-rc.2",
]
ENVELOPE_TAGS = ["1.8.2-rc.3", "1.8.2-rc.4"]


@pytest.mark.parametrize("version", PRE_ENVELOPE_TAGS + ["v1.8.1", " 1.8.1\n"])
def test_pre_envelope_tags_are_allowed(version):
    assert extended_limits_problem(version) is None
    require_extended_limits(version)


@pytest.mark.parametrize("version", ENVELOPE_TAGS + ["v1.8.2-rc.4"])
def test_envelope_tags_are_refused(version):
    problem = extended_limits_problem(version)
    assert problem and "safety envelope" in problem and "1.8.1" in problem
    with pytest.raises(ExtendedLimitsRefused, match="safety envelope"):
        require_extended_limits(version)


@pytest.mark.parametrize("version", [
    "1.8.2",            # the release the envelope ships in
    "1.8.2-rc.5",       # later pre-releases
    "1.8.2-dev.2",      # never tagged, so never checked
    "1.8.2-dev.6",
    "1.8.3",
    "1.9.0",
    "2.0.0",
])
def test_later_or_unchecked_versions_are_refused(version):
    assert "safety envelope" in extended_limits_problem(version)


@pytest.mark.parametrize("version", [
    "1.8.1-12-gabc1234",        # a build after the tag: may carry anything
    "1.8.1-12-gabc1234-dirty",
    "1.8.1-dirty",
    "1.8.2-rc.1-3-g0123abc",
])
def test_builds_between_tags_are_refused(version):
    assert "between release tags" in extended_limits_problem(version)


@pytest.mark.parametrize("version", [None, "", "  ", "unknown", "v0.0.0", "0.0.0"])
def test_no_reported_version_is_refused(version):
    assert "did not report a firmware version" in extended_limits_problem(version)


@pytest.mark.parametrize("version", ["abc1234", "1.8", "1.8.1.1", "release-1.8.1", "1.8.1 beta"])
def test_unparseable_versions_are_refused(version):
    assert "not a release tag" in extended_limits_problem(version)


def test_refusal_is_a_runtime_error():
    assert issubclass(ExtendedLimitsRefused, RuntimeError)
