"""Firmware update helpers: version comparison, GitHub "latest" lookup,
download, and the DFU update orchestrator. UI-agnostic (no Qt)."""
from __future__ import annotations

import logging
import re
import time
from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Callable, Iterable

from omotion.GitHubReleases import GitHubReleases
from omotion.boot_mode import BootMode, boot_mode_from_dfu, flash_address_for
from omotion.config import CONSOLE_MODULE_PID, SENSOR_MODULE_PID
from omotion.dfu import migration
from omotion.dfu.errors import BootloaderVersionError, DFUError
from omotion.dfu.image import SFUMAGIC, ImageVerificationError, verify_image
from omotion.dfu.keys import public_key_pem
from omotion.dfu.progress import DFUProgress, progress_adapter
from omotion.dfu.stm32dfu import STM32DFU

_log = logging.getLogger(__name__)

_ST_VID = 0x0483

#: Bare-metal writes (erase, write, read-back) before update() gives up with
#: UpdateInterruptedError. SDK-local, not from the bootloader tooling.
ROM_WRITE_ATTEMPTS = 3
#: How long a ROM loader that dropped off USB during a write gets to come back
#: before the next attempt.
T_ROM_REATTACH = 10.0

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
# ordinary update: FirmwareUpdater.update() writes it only when the caller
# passes it as production_image with acknowledge_irreversible=True.
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

class FirmwareUpdateError(RuntimeError):
    """Raised when a firmware update cannot proceed or did not complete."""


class BootloaderChangeRequired(FirmwareUpdateError):
    """The unit cannot take this firmware until its bootloader is installed or
    replaced, and the caller did not authorise that. Nothing was written.

    ``needs`` names the ``update()`` arguments that were missing:
    ``"updater_image"`` or ``"production_image"``, and
    ``"acknowledge_irreversible"``.
    """

    def __init__(self, message: str, *, boot_mode: BootMode,
                 bootloader_version: str | None, needs: tuple[str, ...]):
        super().__init__(message)
        self.boot_mode = boot_mode
        self.bootloader_version = bootloader_version
        self.needs = needs


class ImageRejectedError(FirmwareUpdateError):
    """An image failed the host pre-flight, or the bootloader refused it."""


class UpdaterRefusedError(FirmwareUpdateError):
    """The bootloader updater ran (or was refused) and the old bootloader is
    still in place."""


class UpdateInterruptedError(FirmwareUpdateError):
    """A bare-metal write failed after the erase, and every retry failed too.

    The unit's flash is incomplete, and a unit without a bootloader can only get
    back into DFU through its own application. While it stays powered it sits
    in the STM32 ROM loader and can still be rewritten, but a power cycle starts
    the half-written image, and then it needs bench recovery. Tell the operator
    not to power off or unplug it, and offer to retry::

        FirmwareUpdater().update(None, e.image_path, kind=e.kind,
                                 dfu_serial=e.dfu_serial)

    ``in_dfu`` says whether the unit was still on USB in the ROM loader when
    update() gave up. If not, it dropped off USB: if it is still powered,
    reconnecting its USB cable brings the ROM loader back.
    """

    def __init__(self, message: str, *, image_path: Path, kind: FirmwareKind | None,
                 dfu_serial: str | None, in_dfu: bool, attempts: int):
        super().__init__(message)
        #: The bare-metal image that was being written; retry with this one.
        self.image_path = image_path
        self.kind = kind
        #: USB serial of the ROM loader, to retry on this unit and no other.
        self.dfu_serial = dfu_serial
        self.in_dfu = in_dfu
        self.attempts = attempts


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


class UpdateAction(Enum):
    """What :meth:`FirmwareUpdater.update` did to the unit."""

    APP_ONLY = "app_only"
    """Signed application installed in the bootloader's slot."""

    BARE_METAL_APP = "bare_metal_app"
    """Bare-metal application written at the base of flash (no bootloader)."""

    BOOTLOADER_UPGRADED = "bootloader_upgraded"
    """Old bootloader replaced through the updater, then the application."""

    BOOTLOADER_INSTALLED = "bootloader_installed"
    """Production image (bootloader + application) written to a bare-metal unit."""


@dataclass(frozen=True)
class FirmwareUpdateResult:
    success: bool
    action: UpdateAction
    boot_mode: BootMode
    #: Bootloader version strings; None on a unit without a bootloader, and
    #: ``after`` is None when the bootloader was just installed from a
    #: production image (it is not read back).
    bootloader_version_before: str | None
    bootloader_version_after: str | None
    #: The application was seen on USB again before update() returned.
    app_reenumerated: bool
    #: The image is written but the unit never came back: it must be
    #: power-cycled by hand (the STM32 ROM loader's jump into a new image hangs
    #: on sensor modules).
    needs_power_cycle: bool
    message: str


def _strip_dfu_suffix(data: bytes) -> bytes:
    """Drop a 16-byte DFU file suffix ('UFD' signature) if the image has one."""
    if len(data) >= 16 and data[-8:-5] == b"UFD" and data[-5] == 16:
        return data[:-16]
    return data


def _app_present(kind: FirmwareKind | None, handle) -> bool:
    """Is the application on USB? Presence only: nothing is opened, so this
    cannot fight the connection monitor for the port."""
    try:
        if kind in (None, FirmwareKind.CONSOLE):
            import serial.tools.list_ports

            if any(
                getattr(p, "vid", None) == _ST_VID
                and getattr(p, "pid", None) == CONSOLE_MODULE_PID
                for p in serial.tools.list_ports.comports()
            ):
                return True
        if kind in (None, FirmwareKind.SENSOR):
            import usb.core

            from omotion.usb_backend import get_libusb1_backend

            # Left and right sensors share a VID/PID; the hub port tells them apart.
            suffix = getattr(handle, "_port_suffix", None)
            for dev in usb.core.find(find_all=True, idVendor=_ST_VID,
                                     idProduct=SENSOR_MODULE_PID,
                                     backend=get_libusb1_backend()) or []:
                ports = getattr(dev, "port_numbers", None) or ()
                if suffix is None or (ports and ports[-1] == suffix):
                    return True
    except Exception:
        return False
    return False


@dataclass
class _Job:
    """One update() call's inputs, after host pre-flight."""

    handle: object
    bin_path: Path
    kind: FirmwareKind | None
    updater: bytes | None
    production: bytes | None
    ack: bool
    serial: str | None
    progress_cb: Callable[[DFUProgress], None] | None
    t0: float
    #: Set just before the first erase or write. Until then a refusal can still
    #: send the unit back to the application it was running.
    wrote: bool = False

    def say(self, phase: str, message: str) -> None:
        _log.info("%s", message)
        if self.progress_cb is not None:
            self.progress_cb(DFUProgress(phase=phase, percent=None, bytes_written=None,
                                         message=message,
                                         elapsed_s=time.monotonic() - self.t0))


class FirmwareUpdater:
    """Update the firmware of a console or sensor, whatever it is running.

    Enters DFU, works out what the unit has, and follows the sequence the
    bootloader tooling was bench-tested with (:mod:`omotion.dfu.migration`):

    ==========================  ==================================================
    Unit has                    update() does
    ==========================  ==================================================
    bootloader 1.2.0 or newer   installs the signed application
    older bootloader            installs a signed application the old bootloader
                                accepts (legacy key). A current application needs
                                the bootloader replaced first: with
                                ``updater_image`` and ``acknowledge_irreversible``
                                it runs the updater, then installs the application
    no bootloader (ROM loader)  writes the bare-metal application. With
                                ``production_image`` and
                                ``acknowledge_irreversible`` it installs the
                                bootloader and application instead
    ==========================  ==================================================

    Replacing or installing a bootloader cannot be undone over USB and a power
    loss while sector 0 is rewritten needs bench recovery (SWD), so it never
    happens unless the caller says ``acknowledge_irreversible=True``. Without
    it update() raises :class:`BootloaderChangeRequired` naming what the unit
    needs.

    Every image is verified on the host before anything is erased. Does NOT
    manage any connection monitor: the caller pauses reconnection logic around
    this call. update() blocks until the application is back on USB, which can
    take minutes on the bootloader paths; run it on a worker thread.
    """

    def __init__(
        self,
        *,
        dfu=None,
        dfu_wait_timeout_s: float = 30.0,
        app_present: Callable[[FirmwareKind | None, object], bool] | None = None,
    ):
        # ``dfu`` is the STM32DFU class, or a stand-in for it in tests: called
        # to get a device object, and asked ``wait_for_device(...)`` directly.
        self._dfu = dfu or STM32DFU
        self._wait_timeout_s = dfu_wait_timeout_s
        self._app_present = app_present or _app_present
        #: How long the application gets to enumerate after the last download.
        self.app_wait_timeout_s = migration.T_APP_APPEAR
        #: How long the operator gets to power-cycle a unit that left USB.
        self.power_cycle_timeout_s = migration.T_POWER_CYCLE
        #: Bare-metal write attempts before UpdateInterruptedError.
        self.rom_write_attempts = ROM_WRITE_ATTEMPTS
        #: How long a ROM loader that left USB gets to come back between attempts.
        self.rom_reattach_timeout_s = T_ROM_REATTACH
        #: Boot mode observed by the most recent :meth:`update` call, or None
        #: if it has not run (or never got as far as detecting). A device is
        #: only classifiable while it sits in DFU, so this is the one chance a
        #: UI gets to learn what it is talking to.
        self.last_boot_mode: BootMode | None = None
        #: Bootloader version string read by the most recent :meth:`update`
        #: call; None for a unit without a bootloader.
        self.last_bootloader_version: str | None = None

    # ------------------------------------------------------------------ API

    def update(
        self,
        handle,
        bin_path: Path,
        progress_cb: Callable[[DFUProgress], None] | None = None,
        *,
        kind: FirmwareKind | None = None,
        updater_image: Path | None = None,
        production_image: Path | None = None,
        acknowledge_irreversible: bool = False,
        dfu_serial: str | None = None,
    ) -> FirmwareUpdateResult:
        """Install the application ``bin_path``, or its sibling suited to what
        the unit turns out to be running.

        ``handle`` is a connected console or sensor; ``None`` means the unit is
        already in DFU (resuming after an interrupted run). ``kind`` says which
        product the images are for and selects the keys they are checked
        against; it is inferred from a downloaded file or the handle when
        omitted.

        ``updater_image`` is the signed bootloader-updater for an old
        bootloader, ``production_image`` the bootloader + application image for
        a unit without one. Neither is used unless ``acknowledge_irreversible``
        is True and the unit needs it.

        Raises :class:`FirmwareUpdateError` (or a subclass) on any refusal or
        failure.

        On a unit without a bootloader the application is written, read back,
        and started only once it matches; a failed write is redone while the
        unit stays in the ROM loader. If every attempt fails, the unit is left
        in the ROM loader and :class:`UpdateInterruptedError` is raised: it
        must not be powered off. Retry by calling ``update(None,
        e.image_path, kind=e.kind, dfu_serial=e.dfu_serial)``. That is the
        same resume-from-DFU call as after any other interrupted run.
        """
        if acknowledge_irreversible is not True and acknowledge_irreversible is not False:
            raise FirmwareUpdateError(
                "acknowledge_irreversible must be exactly True or False, got "
                f"{acknowledge_irreversible!r}"
            )
        bin_path = Path(bin_path)
        try:
            job = self._preflight(handle, bin_path, progress_cb, kind, updater_image,
                                  production_image, acknowledge_irreversible, dfu_serial)
            self._enter_dfu(job)
            return self._run(job)
        except (DFUError, ImageVerificationError) as e:
            raise FirmwareUpdateError(str(e)) from e

    # ------------------------------------------------------------ internals

    def _preflight(self, handle, bin_path, progress_cb, kind, updater_image,
                   production_image, ack, serial) -> _Job:
        """Checks that need no device: done while the application still runs."""
        kind = self._resolve_kind(kind, handle, bin_path)
        updater = production = None

        if updater_image is not None:
            updater = _read_image(updater_image, "updater image")
            # The updater has to get past the OLD bootloader, so it carries the
            # legacy key's signature.
            self._verify(updater, kind, legacy=True, what="updater image")

        if production_image is not None:
            production = _read_image(production_image, "production image")
            signed = None
            if bin_path.is_file():
                head = bin_path.read_bytes()
                signed = head if head[:4] == SFUMAGIC else None
            migration.check_production_image(production, signed=signed, updater=updater)
            # The application part is what bootloader 1.2.0 will verify at boot.
            self._verify(production[migration.BOOTLOADER_LEN:], kind, legacy=False,
                         what="application in the production image")

        t0 = time.monotonic()
        timed_cb = None
        if progress_cb is not None:
            # Each flash step times itself; callers want one clock for the
            # whole update.
            def timed_cb(p: DFUProgress) -> None:
                progress_cb(replace(p, elapsed_s=time.monotonic() - t0))

        return _Job(handle=handle, bin_path=bin_path, kind=kind, updater=updater,
                    production=production, ack=ack, serial=serial,
                    progress_cb=timed_cb, t0=t0)

    @staticmethod
    def _resolve_kind(kind, handle, bin_path) -> FirmwareKind | None:
        if kind is not None:
            return FirmwareKind(kind)
        provenance = _provenance(bin_path)
        if provenance is not None:
            return provenance[0]
        # Imported here: both modules import this one.
        from omotion.MotionConsole import MotionConsole
        from omotion.MotionSensor import MotionSensor

        if isinstance(handle, MotionConsole):
            return FirmwareKind.CONSOLE
        if isinstance(handle, MotionSensor):
            return FirmwareKind.SENSOR
        return None

    def _verify(self, image: bytes, kind: FirmwareKind | None, *, legacy: bool, what: str):
        """Verify ``image`` against the key ``kind``'s bootloader trusts."""
        if kind is None:
            raise FirmwareUpdateError(
                f"cannot check the {what}: is this a console or a sensor? Pass kind="
            )
        try:
            return verify_image(image, public_key_pem(kind, legacy=legacy))
        except ImageVerificationError as e:
            hint = self._signed_for(image, exclude=(kind, legacy))
            raise ImageRejectedError(
                f"{what} rejected on the host, nothing was flashed: {e}"
                + (f" (it is signed for {hint})" if hint else "")
            ) from e

    @staticmethod
    def _signed_for(image: bytes, exclude=None) -> str | None:
        """Which bootloader accepts ``image``, in words; None if none we know."""
        for k in FirmwareKind:
            for legacy in (False, True):
                if (k, legacy) == exclude:
                    continue
                try:
                    verify_image(image, public_key_pem(k, legacy=legacy))
                except ImageVerificationError:
                    continue
                return f"the {k.value} bootloader " + (
                    "older than 1.2.0" if legacy else "1.2.0 or newer"
                )
        return None

    def _enter_dfu(self, job: _Job) -> None:
        if self._dfu.wait_for_device(0.5, present=True, serial=job.serial):
            if job.handle is not None:
                raise FirmwareUpdateError(
                    "a DFU device is already attached, so the unit to update would be "
                    "ambiguous. Unplug it, or pass handle=None to update the unit that "
                    "is already in DFU"
                )
            return
        if job.handle is None:
            raise FirmwareUpdateError("no DFU device found, and no device handle to ask for one")
        if not job.handle.enter_dfu():
            raise FirmwareUpdateError("device did not accept enter_dfu()")
        if not self._dfu.wait_for_device(self._wait_timeout_s, present=True, serial=job.serial):
            raise FirmwareUpdateError("DFU device did not appear after enter_dfu()")

    def _run(self, job: _Job) -> FirmwareUpdateResult:
        version_before = version_after = None
        with self._dfu() as dfu:
            dfu.connect(serial=job.serial)
            mode = boot_mode_from_dfu(dfu.detect_mode()[0])
            self.last_boot_mode = mode
            self.last_bootloader_version = None
            if mode is BootMode.UNKNOWN:
                raise FirmwareUpdateError(
                    "could not tell whether this device has the bootloader installed; "
                    "refusing to flash, because the wrong address would brick it"
                )
            can_leave = mode is BootMode.BARE_METAL
            try:
                if mode is BootMode.BARE_METAL:
                    action = self._update_rom(dfu, job)
                else:
                    version_before = dfu.read_version()
                    self.last_bootloader_version = version_before
                    parsed = migration.parse_bl_version(version_before)
                    if parsed is None:
                        raise FirmwareUpdateError(
                            f"cannot parse bootloader version {version_before!r}; refusing "
                            "to guess which images it accepts"
                        )
                    can_leave = parsed >= migration.MIN_BOOTLOADER
                    action, version_after = self._update_bootloader(
                        dfu, job, version_before, parsed)
            except BaseException:
                if not job.wrote:
                    self._back_to_application(dfu, mode, can_leave)
                raise

        reenumerated, needs_power_cycle = self._await_application(job, mode)
        if needs_power_cycle:
            message = ("firmware written, but the unit did not come back on USB: "
                       "power-cycle it to start the new firmware")
        else:
            message = {
                UpdateAction.APP_ONLY: "application installed",
                UpdateAction.BARE_METAL_APP: "bare-metal application installed",
                UpdateAction.BOOTLOADER_UPGRADED:
                    f"bootloader upgraded from {version_before} to {version_after}, "
                    "application installed",
                UpdateAction.BOOTLOADER_INSTALLED: "bootloader and application installed",
            }[action]
        return FirmwareUpdateResult(
            success=True, action=action, boot_mode=mode,
            bootloader_version_before=version_before,
            bootloader_version_after=version_after,
            app_reenumerated=reenumerated, needs_power_cycle=needs_power_cycle,
            message=message,
        )

    @staticmethod
    def _back_to_application(dfu, mode: BootMode, can_leave: bool) -> None:
        """Nothing was written: restart the application the unit already has.

        Best effort. Bootloaders older than 1.2.0 have no reset request, so
        those units stay in DFU until they are power-cycled.
        """
        if not can_leave:
            return
        try:
            if mode is BootMode.BARE_METAL:
                dfu.set_rom_mode()
            dfu.leave_dfu()
        except Exception:
            pass

    # -- unit without a bootloader ------------------------------------------

    def _update_rom(self, dfu, job: _Job) -> UpdateAction:
        if job.ack and job.production is not None:
            job.say("erase", "No bootloader on this unit: writing the production image "
                             "(bootloader + application). Do not power off or unplug.")
            job.wrote = True
            migration.install_production_via_rom(dfu, job.production, job.progress_cb)
            return UpdateAction.BOOTLOADER_INSTALLED

        needs = tuple(name for name, have in (
            ("production_image", job.production is not None),
            ("acknowledge_irreversible", job.ack),
        ) if not have)
        if _provenance(job.bin_path) is None and (
            classify_asset_name(job.bin_path.name) is BootMode.BOOTLOADER
            or _starts_with_sfu_magic(job.bin_path)
        ):
            raise BootloaderChangeRequired(
                f"{job.bin_path.name} is a signed image for the bootloader, and this unit "
                "has no bootloader. Installing one needs the production image "
                "(bootloader + application) and acknowledge_irreversible=True; "
                "nothing was written",
                boot_mode=BootMode.BARE_METAL, bootloader_version=None, needs=needs,
            )

        target = self._select_image(job.bin_path, BootMode.BARE_METAL)
        image = _strip_dfu_suffix(_read_image(target, "firmware image"))
        if image[:4] == SFUMAGIC:
            raise BootloaderChangeRequired(
                f"{target.name} is a signed image for the bootloader, and this unit has "
                "no bootloader; nothing was written",
                boot_mode=BootMode.BARE_METAL, bootloader_version=None, needs=needs,
            )
        job.wrote = True
        self._write_bare_metal(dfu, job, target, image)
        return UpdateAction.BARE_METAL_APP

    def _write_bare_metal(self, dfu, job: _Job, target: Path, image: bytes) -> None:
        """Write ``image``, read it back, and start it only once it matches.

        The first erase destroys the image in sector 0. From then on a power
        cycle starts whatever is in flash, so leaving DFU before the read-back
        matches would brick the unit. A failed attempt is redone while the
        unit is still in the ROM loader. When none succeeds the unit is left
        there and UpdateInterruptedError says so.
        """
        serial = job.serial or getattr(dfu, "serial", None)
        attempts = self.rom_write_attempts
        last: DFUError | None = None
        for attempt in range(1, attempts + 1):
            try:
                if attempt > 1:
                    job.say("retry", f"The firmware write failed ({last}). Retrying "
                                     f"({attempt}/{attempts}): do not power off or unplug "
                                     "the unit.")
                    if not self._reattach_rom(dfu, serial):
                        raise self._interrupted(job, target, serial, last, attempt - 1,
                                                in_dfu=False)
                migration.write_bare_metal_app(dfu, image, job.progress_cb)
            except DFUError as e:
                last = e
                _log.warning("bare-metal write attempt %d/%d failed: %s", attempt, attempts, e)
                continue
            migration.leave_rom(dfu)
            return
        # Released first: reading the serial of a device we still hold open can
        # fail on Windows, which would make the unit look gone.
        dfu.disconnect()
        in_dfu = self._dfu.wait_for_device(0.5, present=True, serial=serial)
        raise self._interrupted(job, target, serial, last, attempts, in_dfu=in_dfu)

    def _reattach_rom(self, dfu, serial: str | None) -> bool:
        """Reopen the ROM loader for another attempt; False if it is gone from
        USB. A reopen that fails, or finds something other than the ROM loader,
        counts as a failed attempt (DFUError)."""
        dfu.disconnect()
        if not self._dfu.wait_for_device(self.rom_reattach_timeout_s, present=True,
                                         serial=serial):
            return False
        dfu.connect(serial=serial)
        if boot_mode_from_dfu(dfu.detect_mode()[0]) is not BootMode.BARE_METAL:
            raise DFUError("the DFU device that came back is not the STM32 ROM loader")
        return True

    @staticmethod
    def _interrupted(job: _Job, target: Path, serial: str | None, last, attempts: int,
                     *, in_dfu: bool) -> UpdateInterruptedError:
        if in_dfu:
            message = (
                f"the firmware write failed {attempts} time(s) ({last}). The unit is still "
                "in update mode with incomplete firmware: do NOT power it off or unplug "
                "it. Retry the update to finish it"
            )
        else:
            message = (
                f"the unit left update mode during the firmware write ({last}), so its "
                "firmware is incomplete. If it is still powered, do NOT power it off: "
                "reconnect its USB cable and retry the update. A unit that lost power "
                "during the write needs bench recovery"
            )
        _log.error("bare-metal update interrupted: %s", message)
        return UpdateInterruptedError(message, image_path=target, kind=job.kind,
                                      dfu_serial=serial, in_dfu=in_dfu, attempts=attempts)

    # -- unit with a bootloader ---------------------------------------------

    def _update_bootloader(self, dfu, job: _Job, version: str,
                           parsed: tuple) -> tuple[UpdateAction, str]:
        if job.production is not None and is_production_asset(job.bin_path.name):
            # An "install the bootloader" request (production image only) that
            # found one already there.
            raise FirmwareUpdateError(
                f"this unit already has a bootloader ({version}); nothing was written. "
                "To update it, pass the signed application image"
            )
        target = self._select_image(job.bin_path, BootMode.BOOTLOADER)
        signed = _read_image(target, "firmware image")

        if parsed >= migration.MIN_BOOTLOADER:
            info = self._verify(signed, job.kind, legacy=False, what=target.name)
            self._refuse_downgrade(dfu, info, target)
            job.wrote = True
            self._install_signed(dfu, job, signed)
            return UpdateAction.APP_ONLY, version

        # Bootloader 1.0.x / 1.1.x: it trusts the legacy key only.
        if job.kind is None:
            raise FirmwareUpdateError(
                f"cannot check {target.name}: is this a console or a sensor? Pass kind="
            )
        try:
            info = verify_image(signed, public_key_pem(job.kind, legacy=True))
        except ImageVerificationError:
            info = None
        if info is not None:
            self._refuse_downgrade(dfu, info, target)
            job.wrote = True
            try:
                dfu.download(migration.SLOT_ADDR, signed,
                             progress_cb=progress_adapter(job.progress_cb, "download"))
            except DFUError as e:
                raise ImageRejectedError(f"the bootloader refused the download: {e}") from e
            return UpdateAction.APP_ONLY, version

        # Not for the old bootloader. Is it a good image for the new one?
        self._verify(signed, job.kind, legacy=False, what=target.name)
        if not (job.ack and job.updater is not None):
            needs = tuple(name for name, have in (
                ("updater_image", job.updater is not None),
                ("acknowledge_irreversible", job.ack),
            ) if not have)
            raise BootloaderChangeRequired(
                f"this unit runs bootloader {version}, which does not accept "
                f"{target.name}: the bootloader has to be replaced with 1.2.0 or newer "
                "first. That needs the bootloader-updater image and "
                "acknowledge_irreversible=True. Nothing was written; power-cycle the "
                "unit to return to its application",
                boot_mode=BootMode.BOOTLOADER, bootloader_version=version, needs=needs,
            )

        job.say("erase", f"Bootloader {version}: installing the bootloader updater. "
                         "DO NOT POWER OFF OR UNPLUG until the update has finished.")
        job.wrote = True
        migration.run_updater(dfu, job.updater, job.progress_cb)
        dfu.disconnect()

        job.say("wait", "The unit is replacing its bootloader. DO NOT POWER OFF OR UNPLUG "
                        f"(up to {migration.T_UPDATER_CYCLE:.0f}s).")
        if not migration.wait_for_new_bootloader(self._dfu.wait_for_device, serial=job.serial):
            if self._app_present(job.kind, job.handle):
                raise UpdaterRefusedError(
                    "the unit came back as an application, not in DFU: the old bootloader "
                    "refused the updater. The bootloader was not changed"
                )
            raise FirmwareUpdateError(
                "no DFU device after the updater ran. If all three LEDs blink, programming "
                "the bootloader failed and the unit needs bench recovery (SWD)"
            )

        with self._dfu() as dfu2:
            dfu2.connect(serial=job.serial)
            try:
                version_after = self._install_signed(dfu2, job, signed)
            except BootloaderVersionError as e:
                raise UpdaterRefusedError(
                    f"the bootloader still reports {e.version!r}: the updater did not "
                    "replace it (it refuses when the option bytes are not RDP 0 with no "
                    "WRP and no PCROP). The unit is still usable with images for the old "
                    "bootloader"
                ) from e
        self.last_bootloader_version = version_after
        return UpdateAction.BOOTLOADER_UPGRADED, version_after

    @staticmethod
    def _refuse_downgrade(dfu, info, target: Path) -> None:
        """The bootloader destroys a downloaded image older than the one it
        replaced, so refuse before the erase instead."""
        installed = migration.installed_fw_version(dfu)
        if info.fw_version < installed:
            raise ImageRejectedError(
                f"{target.name} has FwVersion {info.fw_version}, older than the installed "
                f"{installed}; the bootloader refuses downgrades. Nothing was flashed"
            )

    @staticmethod
    def _install_signed(dfu, job: _Job, signed: bytes) -> str:
        try:
            return migration.install_signed_app(dfu, signed, job.progress_cb)
        except BootloaderVersionError:
            raise
        except DFUError as e:
            raise ImageRejectedError(
                f"the bootloader refused the download: {e}. It checks the signed header "
                "before writing: wrong key, or a FwVersion below the header still in the "
                "slot"
            ) from e

    # -- afterwards ---------------------------------------------------------

    def _wait_app(self, job: _Job, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while True:
            if self._app_present(job.kind, job.handle):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.5)

    def _await_application(self, job: _Job, mode: BootMode) -> tuple[bool, bool]:
        """``(app_reenumerated, needs_power_cycle)`` once the unit restarts."""
        job.say("wait", "Waiting for the application "
                        f"(up to {self.app_wait_timeout_s:.0f}s).")
        if self._wait_app(job, self.app_wait_timeout_s):
            return True, False

        in_dfu = self._dfu.wait_for_device(0.5, present=True, serial=job.serial)
        if mode is BootMode.BARE_METAL and not in_dfu:
            # Flash is complete, but the unit is gone from USB: the STM32 ROM
            # loader's jump into a freshly written image hangs on the sensor
            # modules until a real power cycle (seen on the bench 2026-10-08).
            job.say("power_cycle",
                    "The unit left USB without restarting. The firmware is written: "
                    "POWER-CYCLE THE UNIT NOW (a console power cycle restarts both sensors).")
            if self._wait_app(job, self.power_cycle_timeout_s):
                return True, False
            return False, True

        raise FirmwareUpdateError(
            "the application did not start after the update"
            + (": the unit is still in DFU, so the bootloader refused the image"
               if in_dfu else "")
        )

    def _select_image(self, bin_path: Path, mode: BootMode) -> Path:
        if is_production_asset(bin_path.name):
            raise FirmwareUpdateError(
                f"{bin_path.name} is a production image (bootloader + signed app), not "
                "an application. To install a bootloader pass it as production_image "
                "with acknowledge_irreversible=True."
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
                f"{flash_address_for(mode):#010x} would corrupt the device."
            )
        return bin_path


def _read_image(path: Path, what: str) -> bytes:
    path = Path(path)
    if not path.is_file():
        raise FirmwareUpdateError(f"{what} not found: {path}")
    return path.read_bytes()


def _starts_with_sfu_magic(path: Path) -> bool:
    try:
        with open(path, "rb") as f:
            return f.read(4) == SFUMAGIC
    except OSError:
        return False


# ---------------------------------------------------------------------------
# Release folders
#
# The images for one update, picked out of a folder and checked on the host.
# Shared by scripts/update_firmware.py and the test app, so that both pick the
# same files and say the same things about them before anything is flashed.
# ---------------------------------------------------------------------------

# What each image is called in a release folder: (role, word in the file name).
_RELEASE_ROLES = (("app", "signed"), ("updater", "updater"), ("production", "production"))

# ST-LINK debug probes (VID 0483). One wired to the board can disturb a
# bootloader rewrite.
_STLINK_PIDS = frozenset(
    {0x3744, 0x3748, 0x374A, 0x374B, 0x374D, 0x374E, 0x374F, 0x3752, 0x3753, 0x3754, 0x3757}
)


def find_release_images(release_dir: Path, kind) -> dict[str, Path]:
    """The images for ``kind`` in ``release_dir``, as ``{"app": ..., "updater":
    ..., "production": ...}`` with absent roles left out.

    A ``.bin`` belongs to a role when its name carries the product (console /
    sensor) and the role's word: ``signed`` for the application, ``updater``
    for the bootloader updater, ``production`` for the bootloader + application
    image. Raises ``ValueError`` when a role matches more than one file:
    guessing between two images is not acceptable for a step that cannot be
    undone.
    """
    product = FirmwareKind(getattr(kind, "value", kind)).value
    found: dict[str, Path] = {}
    for role, word in _RELEASE_ROLES:
        matches = sorted(
            p for p in Path(release_dir).glob("*.bin")
            if product in p.name.lower() and word in p.name.lower()
            # An updater is a signed image too; it is not the application.
            and not (role == "app" and "updater" in p.name.lower())
        )
        if len(matches) > 1:
            raise ValueError(
                f"{release_dir} holds {len(matches)} {product} {word} images: "
                + ", ".join(p.name for p in matches)
            )
        if matches:
            found[role] = matches[0]
    return found


def describe_images(kind, app: Path, updater: Path | None = None,
                    production: Path | None = None) -> dict[str, str]:
    """Check one update's images on the host and say what each one is.

    Returns ``{role: description}`` for the images given, e.g. ``"FwVersion
    10802 (1.8.2), signed for bootloader 1.2.0 or newer"``. Raises
    :class:`ImageRejectedError` for an image that is for another product, is
    signed for the wrong bootloader generation, is damaged, or does not belong
    with the others. Touches no device: this is what a UI shows the operator
    before asking for confirmation. :meth:`FirmwareUpdater.update` repeats the
    checks before it writes.
    """
    kind = FirmwareKind(getattr(kind, "value", kind))
    out: dict[str, str] = {}

    def verified(image: bytes, legacy: bool, what: str):
        try:
            return verify_image(image, public_key_pem(kind, legacy=legacy))
        except ImageVerificationError as e:
            hint = FirmwareUpdater._signed_for(image, exclude=(kind, legacy))
            raise ImageRejectedError(
                f"{what} rejected: {e}" + (f" (it is signed for {hint})" if hint else "")
            ) from e

    app_bytes = _read_image(app, "application image")
    signed = app_bytes if app_bytes[:4] == SFUMAGIC else None
    if signed is None:
        out["app"] = "bare-metal image (not signed; only for a unit without a bootloader)"
    else:
        try:
            info = verify_image(signed, public_key_pem(kind, legacy=True))
            out["app"] = (f"FwVersion {info.version_str}, signed for a bootloader older "
                          "than 1.2.0")
        except ImageVerificationError:
            # Not an old-bootloader image, so it has to be a good current one;
            # its failure (wrong key, damaged body, ...) is the one to report.
            info = verified(signed, False, Path(app).name)
            out["app"] = f"FwVersion {info.version_str}, signed for bootloader 1.2.0 or newer"

    updater_bytes = None
    if updater is not None:
        updater_bytes = _read_image(updater, "updater image")
        info = verified(updater_bytes, True, f"updater image {Path(updater).name}")
        out["updater"] = (f"FwVersion {info.version_str}, signed for a bootloader older "
                          "than 1.2.0 (which runs it)")

    if production is not None:
        production_bytes = _read_image(production, "production image")
        try:
            bootloader = migration.check_production_image(
                production_bytes, signed=signed, updater=updater_bytes)
        except ImageVerificationError as e:
            raise ImageRejectedError(f"{Path(production).name} rejected: {e}") from e
        info = verified(production_bytes[migration.BOOTLOADER_LEN:], False,
                        f"application in {Path(production).name}")
        out["production"] = (f"bootloader ({len(bootloader)} B) + application FwVersion "
                             f"{info.version_str}")
    return out


def stlink_probes() -> int:
    """How many ST-LINK debug probes are on USB; 0 if USB cannot be enumerated.

    A probe wired to the board must be unplugged while a bootloader is
    installed or replaced. This cannot tell which board a probe is wired to, so
    callers refuse on any.
    """
    try:
        import usb.core

        from omotion.usb_backend import get_libusb1_backend

        return sum(
            1 for d in usb.core.find(find_all=True, idVendor=_ST_VID,
                                     backend=get_libusb1_backend()) or []
            if d.idProduct in _STLINK_PIDS
        )
    except Exception:
        return 0
