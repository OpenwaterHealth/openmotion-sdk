from __future__ import annotations
from importlib.metadata import PackageNotFoundError, version as _pkg_version

# Global log root variable - can be set via set_log_root()
_log_root = "openmotion.sdk"


def set_log_root(root: str):
    """
    Set a global log root prefix that will be prepended to all logger names.

    Args:
        root: The prefix to prepend to all logger names (e.g., "MyApp" will make
              loggers like "MyApp.Console", "MyApp.Sensor", etc.)
    """
    global _log_root
    _log_root = root


# Resolve __version__ before any submodule that does `from omotion import
# __version__` is loaded.
try:
    # works when installed (wheel/sdist) — uses dist-info METADATA.
    # Must match the [project] name in pyproject.toml (renamed from
    # "openmotion-pylib" → "openmotion-sdk" in commit 0f7bb52).
    __version__ = _pkg_version("openmotion-sdk")
except PackageNotFoundError:
    # running from source (no dist-info)? try pyproject.toml first
    try:
        import tomllib  # Python 3.11+
        from pathlib import Path

        pyproject = (
            Path(__file__).resolve().parents[1] / "pyproject.toml"
        ).read_bytes()
        __version__ = tomllib.loads(pyproject)["project"]["version"]
    except Exception:
        # fall back to setuptools_scm if tomllib or key lookup fails
        try:
            from setuptools_scm import get_version

            __version__ = get_version(root="..", relative_to=__file__)
        except Exception:
            __version__ = "0+unknown"

from .config import *
from .MotionUart import MotionUart
from .MotionSignal import MotionSignal
from .MotionComposite import MotionComposite
from .USBInterfaceBase import USBInterfaceBase
from .MotionConfig import MotionConfig
from .ScanDatabase import ScanDatabase
# Submodules, not names: db_open.connect() / db_schema.upgrade() read
# unambiguously where a top-level connect() or upgrade() would not.
from . import db_key, db_migrate, db_open, db_schema
from .SessionPlayback import materialize_corrected_csv
from .Calibration import Calibration
from .CalibrationWorkflow import (
    CalibrationOutcome,
    CalibrationRequest,
    CalibrationResult,
    CalibrationResultRow,
    CalibrationThresholds,
    factory_calibration_thresholds,
    ungated_cameras,
)
from .connection_state import ConnectionState
from .boot_mode import BootInfo, BootMode, parse_boot_info, parse_boot_info_details
from .fpga_attest import FpgaAttestReport, parse_attest_report
from .firmware_update import (
    BootloaderChangeRequired,
    FirmwareKind,
    FirmwareUpdater,
    FirmwareUpdateError,
    FirmwareUpdateResult,
    LatestInfo,
    UpdateAction,
    check_latest,
    download_firmware,
    UnsupportedReleaseError,
    UpdateInterruptedError,
    is_update_available,
    parse_version,
    production_asset,
)
# NOTE: installing or replacing a bootloader is irreversible over USB. It is a
# path inside FirmwareUpdater.update(), taken only with
# acknowledge_irreversible=True. See tests/test_firmware_update_states.py.
# Top-level handles + interface (deferred until after the leaf modules above
# so MotionConsole/MotionSensor can do `from omotion import _log_root` during
# their own module load without hitting the partially-loaded package).
from .MotionConsole import MotionConsole
from .MotionSensor import MotionSensor
from .MotionInterface import MotionInterface

__all__ = [
    "__version__",
    "set_log_root",
    "MotionInterface",
    "MotionConsole",
    "MotionSensor",
    "MotionUart",
    "MotionSignal",
    "MotionComposite",
    "USBInterfaceBase",
    "MotionConfig",
    "ScanDatabase",
    "db_key",
    "db_migrate",
    "db_open",
    "db_schema",
    "materialize_corrected_csv",
    "Calibration",
    "CalibrationOutcome",
    "CalibrationRequest",
    "CalibrationResult",
    "CalibrationResultRow",
    "CalibrationThresholds",
    "factory_calibration_thresholds",
    "ungated_cameras",
    "ConnectionState",
    "FirmwareKind",
    "FirmwareUpdater",
    "FirmwareUpdateError",
    "FirmwareUpdateResult",
    "BootloaderChangeRequired",
    "UpdateInterruptedError",
    "UpdateAction",
    "LatestInfo",
    "check_latest",
    "download_firmware",
    "BootMode",
    "BootInfo",
    "parse_boot_info",
    "parse_boot_info_details",
    "FpgaAttestReport",
    "parse_attest_report",
    "UnsupportedReleaseError",
    "is_update_available",
    "parse_version",
    "production_asset",
]
