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


# ---------------------------------------------------------------------------
# Task 3: FirmwareKind, LatestInfo, check_latest
# ---------------------------------------------------------------------------
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from omotion.GitHubReleases import GitHubReleases


class FirmwareKind(Enum):
    CONSOLE = "console"
    SENSOR = "sensor"


_REPO = {
    FirmwareKind.CONSOLE: ("OpenwaterHealth", "openmotion-console-fw"),
    FirmwareKind.SENSOR: ("OpenwaterHealth", "openmotion-sensor-fw"),
}
_ASSET = {
    FirmwareKind.CONSOLE: "motion-console-fw.bin",
    FirmwareKind.SENSOR: "motion-sensor-fw.bin",
}


@dataclass(frozen=True)
class LatestInfo:
    kind: FirmwareKind
    tag: str
    asset_name: str
    published_at: Optional[str] = None


def check_latest(
    kind: FirmwareKind,
    *,
    include_prerelease: bool = False,
    releases: GitHubReleases | None = None,
) -> Optional[LatestInfo]:
    """Newest release of ``kind``'s firmware repo, or ``None`` on any
    network/parse failure or if no matching ``.bin`` asset exists. Never raises:
    callers treat ``None`` as "couldn't determine, show nothing"."""
    owner, repo = _REPO[kind]
    gh = releases or GitHubReleases(owner, repo)
    try:
        rel = gh.get_latest_release(include_prerelease=include_prerelease)
        tag = rel.get("tag_name")
        if not tag:
            return None
        names = [a.get("name", "") for a in gh.get_asset_list(release=rel, extension=".bin")]
        if _ASSET[kind] in names:
            asset_name = _ASSET[kind]
        elif names:
            asset_name = names[0]
        else:
            return None
        return LatestInfo(kind=kind, tag=tag, asset_name=asset_name,
                          published_at=rel.get("published_at"))
    except Exception:
        return None
