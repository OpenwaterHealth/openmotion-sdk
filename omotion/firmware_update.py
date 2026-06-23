"""Firmware update helpers: version comparison, GitHub "latest" lookup,
download, and a thin DFU flash orchestrator. UI-agnostic (no Qt)."""
from __future__ import annotations

import re

# Matches a leading MAJOR.MINOR.PATCH, tolerating a leading "v" and any
# pre-release/build/git-describe suffix. Mirrors MotionSensor._VERSION_RE.
_VERSION_RE = re.compile(r"^v?(\d+)\.(\d+)\.(\d+)")


def parse_version(version_str: str) -> tuple[int, int, int]:
    """Parse a firmware version into ``(major, minor, patch)``.

    Tolerates a leading ``v`` and any pre-release/build suffix. Raises
    ``TypeError`` for ``None`` and ``ValueError`` for a string with no leading
    numeric ``MAJOR.MINOR.PATCH`` component.
    """
    if version_str is None:
        raise TypeError("version_str must be a string, got None")
    m = _VERSION_RE.match(version_str)
    if not m:
        raise ValueError(f"unparseable firmware version {version_str!r}")
    return (int(m.group(1)), int(m.group(2)), int(m.group(3)))


def is_update_available(installed: str, latest: str) -> bool:
    """True iff ``latest`` parses to a strictly greater (maj, min, patch) than
    ``installed``. Pre-release suffixes are ignored. Returns ``False`` if either
    string is empty/unparseable (fail-safe: never offer an unreasoned update)."""
    try:
        return parse_version(latest) > parse_version(installed)
    except (ValueError, TypeError):
        return False
