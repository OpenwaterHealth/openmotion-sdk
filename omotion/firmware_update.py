"""Firmware update helpers: version comparison, GitHub "latest" lookup,
download, and a thin DFU flash orchestrator. UI-agnostic (no Qt)."""
from __future__ import annotations

import re
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable

from omotion.GitHubReleases import GitHubReleases
from omotion.DFUProgrammer import DFUProgrammer, DFUProgress, DFUResult

# ---------------------------------------------------------------------------
# Version parsing
# ---------------------------------------------------------------------------

# Matches a leading MAJOR.MINOR.PATCH, tolerating a leading "v" and any
# pre-release/build/git-describe suffix. Shared with MotionSensor (which
# re-imports parse_version).
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
# FirmwareKind, LatestInfo, check_latest
# ---------------------------------------------------------------------------

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
    published_at: str | None = None


def check_latest(
    kind: FirmwareKind,
    *,
    include_prerelease: bool = False,
    releases: GitHubReleases | None = None,
) -> LatestInfo | None:
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


# ---------------------------------------------------------------------------
# download_firmware + FirmwareUpdater
# ---------------------------------------------------------------------------

_STM32_DFU_VIDPID = "0483:df11"


class FirmwareUpdateError(RuntimeError):
    """Raised when a firmware flash cannot proceed (DFU entry/enumeration)."""


def download_firmware(
    info: LatestInfo,
    dest_dir: Path,
    *,
    releases: GitHubReleases | None = None,
) -> Path:
    """Download ``info``'s ``.bin`` asset into ``dest_dir``; returns the path."""
    owner, repo = _REPO[info.kind]
    gh = releases or GitHubReleases(owner, repo)
    rel = gh.get_release_by_tag(info.tag)
    return gh.download_asset(rel, info.asset_name, output_dir=Path(dest_dir))


class FirmwareUpdater:
    """Flash one STM32 firmware ``.bin`` onto a handle that supports ``enter_dfu``.

    Pure SDK: enters DFU, waits for the ROM bootloader (PID df11) to
    re-enumerate, and flashes with the bundled dfu-util. Does NOT manage any
    connection monitor — the caller pauses reconnection logic around this call.
    """

    def __init__(
        self,
        *,
        programmer: DFUProgrammer | None = None,
        dfu_wait_timeout_s: float = 30.0,
    ):
        self._dfu = programmer or DFUProgrammer(vidpid=_STM32_DFU_VIDPID)
        self._wait_timeout_s = dfu_wait_timeout_s

    def update(
        self,
        handle,
        bin_path: Path,
        progress_cb: Callable[[DFUProgress], None] | None = None,
    ) -> DFUResult:
        if not handle.enter_dfu():
            raise FirmwareUpdateError("device did not accept enter_dfu()")
        if not self._dfu.wait_for_dfu_device(timeout_s=self._wait_timeout_s):
            raise FirmwareUpdateError("DFU device did not appear after enter_dfu()")
        return self._dfu.flash_bin(Path(bin_path), progress=progress_cb)
