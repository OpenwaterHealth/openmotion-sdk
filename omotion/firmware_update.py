"""Firmware update helpers: version comparison, GitHub "latest" lookup,
download, and a thin DFU flash orchestrator. UI-agnostic (no Qt)."""
from __future__ import annotations

import logging
import re
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Iterable

from omotion import _log_root
from omotion.GitHubReleases import GitHubReleases
from omotion.DFUProgrammer import DFUProgrammer, DFUProgress, DFUResult
from omotion.boot_mode import BootMode, flash_address_for

logger = logging.getLogger(
    f"{_log_root}.firmware_update" if _log_root else "firmware_update"
)

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


_GIT_DESCRIBE_TAIL = re.compile(r"-\d+-g[0-9a-f]+$", re.IGNORECASE)


def _release_tag(version: str) -> str:
    """Reduce a version/tag to its base release tag: strip a leading 'v', a
    trailing '-dirty', and a git-describe tail ('-<N>-g<sha>'). Used by the
    beta (most-recently-published) update check, which compares release
    identity rather than semver precedence."""
    if not version:
        return ""
    v = version.strip()
    if v[:1] in ("v", "V"):
        v = v[1:]
    if v.endswith("-dirty"):
        v = v[: -len("-dirty")]
    # Assumes real release tags never end in the git-describe pattern
    # '-<N>-g<sha>'; that suffix only appears in describe output for commits
    # AFTER a tag, so stripping it here recovers the base release tag.
    return _GIT_DESCRIBE_TAIL.sub("", v)


def is_update_available(installed: str, latest: str, *, prerelease: bool = False) -> bool:
    """Whether ``latest`` should be offered over ``installed``.

    prerelease=False (stable): True iff ``latest``'s (major, minor, patch) is
    strictly greater than ``installed``'s. prerelease=True (beta): True iff the
    device's base release tag differs from ``latest``'s — i.e. the device is
    not already on the most-recently-published release. Fail-safe ``False`` on
    empty/unparseable input."""
    if prerelease:
        inst = _release_tag(installed)
        lat = _release_tag(latest)
        if not inst or not lat:
            return False
        return inst != lat
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

# ---------------------------------------------------------------------------
# Release assets
#
# CI renamed every firmware asset on 2026-07-09, when the bootloader-slot build
# was added alongside the bare-metal one. Releases before that carry a single
# `motion-{sensor,console}-fw.bin`; releases after carry separate bare-metal,
# signed and production images. Both eras have to keep working, so each slot
# below is a preference list: newest name first, legacy name last.
# ---------------------------------------------------------------------------

# Bare-metal units. Sensor prefers the FPGA-merged image so the bitstream always
# matches the firmware it shipped with — that is what the legacy single asset
# was, so behaviour is unchanged for existing callers. Console has no merged
# variant.
_BARE_METAL_ASSETS = {
    FirmwareKind.SENSOR: ("motion-sensor-fw-baremetal-fpga.bin", "motion-sensor-fw.bin"),
    FirmwareKind.CONSOLE: ("motion-console-fw-baremetal.bin", "motion-console-fw.bin"),
}

# Bootloader units. Signed slot image only — there is no legacy equivalent,
# which is why a pre-bootloader release cannot be installed on one.
_SIGNED_ASSETS = {
    FirmwareKind.SENSOR: ("motion-sensor-fw-signed.bin",),
    FirmwareKind.CONSOLE: ("motion-console-fw-signed.bin",),
}

# bootloader + signed app, for converting a bare-metal unit. Never part of an
# ordinary update — see omotion.bootloader_install.
_PRODUCTION_ASSETS = {
    FirmwareKind.SENSOR: "motion-sensor-production.bin",
    FirmwareKind.CONSOLE: "motion-console-production.bin",
}

# First release of each firmware that ships a signed slot image.
_MIN_BOOTLOADER_TAG = {
    FirmwareKind.SENSOR: "1.8.2",
    FirmwareKind.CONSOLE: "1.8.1",
}

_MODE_ASSETS = {
    BootMode.BARE_METAL: _BARE_METAL_ASSETS,
    BootMode.BOOTLOADER: _SIGNED_ASSETS,
}


class UnsupportedReleaseError(RuntimeError):
    """The selected release has no asset that suits the device's boot mode."""


def _first_present(preferences, available) -> str | None:
    return next((name for name in preferences if name in available), None)


def resolve_asset(
    kind: FirmwareKind,
    mode: BootMode,
    asset_names: Iterable[str],
) -> str:
    """Pick the asset to flash onto a ``mode`` device from ``asset_names``.

    Raises ``ValueError`` for :data:`BootMode.UNKNOWN` (guessing bricks devices)
    and ``UnsupportedReleaseError`` when the release predates what the device
    needs.
    """
    try:
        preferences = _MODE_ASSETS[mode][kind]
    except KeyError:
        raise ValueError(
            f"cannot choose a firmware asset for boot mode {mode.value!r}; "
            "refusing to guess"
        ) from None

    available = set(asset_names)
    name = _first_present(preferences, available)
    if name is not None:
        return name

    if mode is BootMode.BOOTLOADER:
        # Lead with the device state, not the release: an operator picking an
        # old release is trying to roll back, and the real answer is that a
        # converted unit cannot leave the bootloader over USB (#225).
        raise UnsupportedReleaseError(
            f"the bootloader is active on this {kind.value}, and rolling back to "
            f"a pre-bootloader release is not possible over USB: the bootloader "
            f"only accepts signed images, and this release has none "
            f"({preferences[0]}). Use {_MIN_BOOTLOADER_TAG[kind]} or newer; "
            "removing the bootloader requires SWD access to the board"
        )
    raise UnsupportedReleaseError(
        f"this release has none of {', '.join(preferences)}; cannot update a "
        f"bare-metal {kind.value}"
    )


def production_asset(kind: FirmwareKind) -> str:
    """Name of the bootloader + signed app image used to convert a device."""
    return _PRODUCTION_ASSETS[kind]


def is_production_asset(name: str) -> bool:
    """Does this filename look like a bootloader + signed app image?"""
    return "production" in Path(name).name.lower()


def classify_asset_name(name: str) -> BootMode | None:
    """Boot mode an asset *filename* implies, or ``None`` if unrecognisable.

    Used to sanity-check files the SDK did not download itself (the test app's
    "Upload File..." path). A name we do not recognise yields ``None`` — the
    caller then takes the file at face value rather than refusing outright.
    Production images are not a boot mode; see :func:`is_production_asset`.
    """
    stem = Path(name).name.lower()
    if "signed" in stem:
        return BootMode.BOOTLOADER
    if "baremetal" in stem:
        return BootMode.BARE_METAL
    # Legacy single-asset era: bare metal was the only thing that existed.
    if stem in {"motion-sensor-fw.bin", "motion-console-fw.bin",
                "motion-sensor-fw-raw.bin", "motion-console-fw-raw.bin"}:
        return BootMode.BARE_METAL
    return None


def candidate_assets(kind: FirmwareKind, asset_names: Iterable[str]) -> list[str]:
    """Every asset that might be flashed for an update, in any boot mode.

    Boot mode cannot be known until the device is already in DFU, and by then
    downloading is inconvenient — so the whole candidate set is fetched up front
    and the right one picked at flash time. Excludes the production image.
    """
    available = set(asset_names)
    names: list[str] = []
    for table in (_BARE_METAL_ASSETS, _SIGNED_ASSETS):
        name = _first_present(table[kind], available)
        if name is not None and name not in names:
            names.append(name)
    return names


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
        if include_prerelease:
            rels = gh.get_all_releases(include_prerelease=True)
            if not rels:
                return None
            # Most recently PUBLISHED wins (timestamp), even if its version is
            # "lower" semver — a dev released after an rc is the one to flash.
            # published_at is ISO-8601, so lexicographic max == chronological.
            # On a tie / missing published_at, Python's max keeps the FIRST
            # maximal element and GitHub returns releases newest-first, so the
            # API-newest release wins.
            rel = max(rels, key=lambda r: (r.get("published_at") or ""))
        else:
            rel = gh.get_latest_release(include_prerelease=False)
        tag = rel.get("tag_name")
        if not tag:
            return None
        names = [a.get("name", "") for a in gh.get_asset_list(release=rel, extension=".bin")]
        if not names:
            return None
        # The bare-metal image is the primary: it is what the legacy single
        # asset was, so this keeps LatestInfo.asset_name meaning what it always
        # meant. download_firmware() fetches the signed sibling alongside it,
        # and update() swaps to that if the device turns out to have the
        # bootloader. An unrecognised release falls back to its first .bin.
        try:
            asset_name = resolve_asset(kind, BootMode.BARE_METAL, names)
        except UnsupportedReleaseError:
            asset_name = names[0]
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


# Downloaded-file provenance, so update() can find a sibling asset for whatever
# boot mode the device turns out to be in. In-process only: callers download and
# flash within one session. Also the authority on which files count as siblings
# — the download directory may be shared across releases, so a directory
# listing could offer a leftover from a different version. A stale entry whose
# file is gone is skipped at lookup time.
_DOWNLOADS: dict[str, tuple[FirmwareKind, str]] = {}


def register_download(path: Path, kind: FirmwareKind, tag: str) -> None:
    """Record that ``path`` came from ``kind``'s release ``tag``."""
    _DOWNLOADS[str(Path(path).resolve())] = (kind, tag)


def _provenance(path: Path) -> tuple[FirmwareKind, str] | None:
    return _DOWNLOADS.get(str(Path(path).resolve()))


def download_firmware(
    info: LatestInfo,
    dest_dir: Path,
    *,
    releases: GitHubReleases | None = None,
) -> Path:
    """Download ``info``'s release into ``dest_dir``; returns the primary path.

    Fetches **every** asset that could be flashed in any boot mode, not just
    ``info.asset_name``. The device's mode is unknowable until it is already in
    DFU, by which point downloading is inconvenient — so the candidate set comes
    down up front and :meth:`FirmwareUpdater.update` picks from it.

    The return value is unchanged (the primary asset), so existing callers are
    unaffected.
    """
    owner, repo = _REPO[info.kind]
    gh = releases or GitHubReleases(owner, repo)
    rel = gh.get_release_by_tag(info.tag)
    dest_dir = Path(dest_dir)

    available = [a.get("name", "") for a in gh.get_asset_list(release=rel, extension=".bin")]
    wanted = candidate_assets(info.kind, available)
    if info.asset_name not in wanted:
        wanted.insert(0, info.asset_name)

    primary: Path | None = None
    for name in wanted:
        try:
            path = Path(gh.download_asset(rel, name, output_dir=dest_dir))
        except Exception:
            # A missing sibling is not fatal: only the primary has to arrive.
            # update() re-checks what is actually on disk before flashing.
            continue
        register_download(path, info.kind, info.tag)
        if name == info.asset_name:
            primary = path

    if primary is None:
        raise FirmwareUpdateError(
            f"could not download {info.asset_name} from {info.kind.value} release {info.tag}"
        )
    return primary


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
        #: Boot mode observed by the most recent :meth:`update` call, or None
        #: if it has not run (or never got as far as detecting). A device is
        #: only classifiable while it sits in DFU, so this is the one chance a
        #: UI gets to learn what it is talking to.
        self.last_boot_mode: BootMode | None = None

    def update(
        self,
        handle,
        bin_path: Path,
        progress_cb: Callable[[DFUProgress], None] | None = None,
    ) -> DFUResult:
        """Flash ``bin_path``, or its sibling suited to the device's boot mode.

        Enters DFU, classifies the device, then flashes the right image at the
        right address. Never installs a bootloader — see
        :func:`omotion.bootloader_install.install_bootloader`.
        """
        bin_path = Path(bin_path)
        if not handle.enter_dfu():
            raise FirmwareUpdateError("device did not accept enter_dfu()")
        if not self._dfu.wait_for_dfu_device(timeout_s=self._wait_timeout_s):
            raise FirmwareUpdateError("DFU device did not appear after enter_dfu()")

        mode = self._dfu.detect_boot_mode()
        # Report what was actually observed, not what we fall back to below --
        # callers (and the apps' lock indicator) must not read a guess as fact.
        self.last_boot_mode = mode

        effective = mode
        if mode is BootMode.UNKNOWN:
            # Refusing here strands any device we cannot classify, and a device
            # that cannot be flashed is worse than one flashed on a safe
            # assumption. Bare metal IS the safe assumption, because it is
            # enforced by the hardware rather than by this guess: both
            # bootloaders clamp their DFU write window to the application slot
            # and mark sector 0 read-only, so a bare-metal write at 0x08000000
            # against a bootloader unit is rejected by the bootloader and fails
            # loudly. The opposite default has no such backstop -- a signed
            # image at 0x08020000 writes happily into the middle of a bare-metal
            # device and leaves it unbootable.
            effective = BootMode.BARE_METAL
            logger.warning(
                "could not classify this device from its DFU alt settings; "
                "proceeding as bare metal (%s). If it does have the bootloader "
                "installed, the write is rejected rather than applied.",
                flash_address_for(effective),
            )

        target = self._select_image(bin_path, effective)
        return self._dfu.flash_bin(
            target, address=flash_address_for(effective), progress=progress_cb
        )

    def _select_image(self, bin_path: Path, mode: BootMode) -> Path:
        if is_production_asset(bin_path.name):
            raise FirmwareUpdateError(
                f"{bin_path.name} is a production image (bootloader + signed app). "
                "Updating cannot install a bootloader — use install_bootloader() "
                "if that is what you want."
            )

        provenance = _provenance(bin_path)
        if provenance is not None:
            kind, tag = provenance
            # Only files downloaded from the same release may stand in for
            # bin_path. The directory itself is no authority: callers reuse one
            # downloads/ folder across releases, and a listing would let a
            # higher-preference leftover from a different version win (#218).
            parent = bin_path.parent.resolve()
            siblings = [
                Path(p).name
                for p, prov in _DOWNLOADS.items()
                if prov == (kind, tag) and Path(p).parent == parent and Path(p).is_file()
            ]
            # Raises UnsupportedReleaseError if this release has nothing for
            # the mode — e.g. a bootloader unit pointed at a legacy release.
            return bin_path.parent / resolve_asset(kind, mode, siblings)

        # Not something we downloaded (the test app's "Upload File..." path).
        # We cannot look up siblings, but we can refuse an image whose name says
        # it belongs at a different address.
        implied = classify_asset_name(bin_path.name)
        if implied is not None and implied is not mode:
            raise FirmwareUpdateError(
                f"{bin_path.name} looks like a {implied.value} image, but this "
                f"device is in {mode.value} mode. Flashing it at "
                f"{flash_address_for(mode)} would corrupt the device."
            )
        return bin_path
