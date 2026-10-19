"""Calibration procedure orchestrator.

Submits two short scans through ScanWorkflow: computes (2, 8)
calibration arrays from scan #1, gates on scan #1's mean/contrast,
applies the proposed calibration to the SDK's in-memory cache only,
runs validation scan #2 against it, writes a per-camera CSV with
mean/contrast/BFI/BVI plus pass/fail vs caller-supplied thresholds,
and returns a CalibrationResult.

If any camera misses any threshold the whole run FAILS and the console
EEPROM is never touched — the write happens only after a fully-passing
validation. There is no operator override and no rollback: nothing to
roll back, because nothing was written.

The workflow does not talk to USB/UART directly. It calls into the
existing ScanWorkflow and processes the raw-histogram CSVs ScanWorkflow
produces.
"""
from __future__ import annotations

import csv
import datetime
import enum
import json
import logging
import os
import platform
import socket
import sys
import threading
import time
import dataclasses
from dataclasses import dataclass
from typing import Callable, Optional, TYPE_CHECKING

import numpy as np

from omotion import _log_root
from omotion.Calibration import Calibration
from omotion.config import (
    CALIBRATION_DEFAULT_MAX_DURATION_SEC,
    CALIBRATION_DEFAULT_SCAN_DELAY_SEC,
    CALIBRATION_I_MAX_MULTIPLIER,
    CAMS_PER_MODULE,
    CAPTURE_HZ,
)

if TYPE_CHECKING:
    from omotion.MotionInterface import MotionInterface

logger = logging.getLogger(
    f"{_log_root}.CalibrationWorkflow" if _log_root else "CalibrationWorkflow"
)


@dataclass
class CalibrationThresholds:
    """Per-camera bounds (length 8, indexed by cam_id 0..7), applied
    symmetrically to left and right modules.

    Mean and contrast are tested as lower bounds only (must be >= min).
    BFI and BVI support both lower and optional upper bounds — for
    target-based criteria like ``BFI = 0 ± 0.1`` the caller sets
    ``min_bfi = -0.1`` and ``max_bfi = +0.1``. When a ``max_*`` field
    is ``None`` (or shorter than 8) the upper-bound check is skipped
    for those positions.
    """
    min_mean_per_camera: list[float]
    min_contrast_per_camera: list[float]
    min_bfi_per_camera: list[float]
    min_bvi_per_camera: list[float]
    max_bfi_per_camera: Optional[list[float]] = None
    max_bvi_per_camera: Optional[list[float]] = None
    max_dark_per_camera: Optional[list[float]] = None


def factory_calibration_thresholds() -> CalibrationThresholds:
    """The canonical WI-00015 / SPEC-69 factory acceptance thresholds.

    Single source of truth shared by the WI-15 measurement-calibration
    runner and the apps' ``ft_*`` config defaults — callers that used to
    hardcode copies of these values import this instead. Mean minimums
    are per-position (corner cameras 1/8 sit farther from the source, so
    40 vs the inner cameras' 80); contrast is the absolute speckle floor;
    BFI/BVI are the SPEC-69 target bands (BFI 0 ± 0.5 must straddle zero
    — a static phantom legitimately reads slightly negative); dark is the
    ambient-light ceiling.

    Returns a fresh instance each call: the fields are mutable lists, so
    a shared module-level constant could be corrupted by one caller
    editing its thresholds in place.
    """
    return CalibrationThresholds(
        min_mean_per_camera=[40.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 40.0],
        min_contrast_per_camera=[0.25] * 8,
        min_bfi_per_camera=[-0.5] * 8,
        max_bfi_per_camera=[0.5] * 8,
        min_bvi_per_camera=[4.5] * 8,
        max_bvi_per_camera=[5.5] * 8,
        max_dark_per_camera=[3.0] * 8,
    )


@dataclass
class CalibrationRequest:
    operator_id: str
    output_dir: str
    left_camera_mask: int
    right_camera_mask: int
    thresholds: CalibrationThresholds
    duration_sec: int  # required; caller supplies from config
    scan_delay_sec: int = CALIBRATION_DEFAULT_SCAN_DELAY_SEC
    max_duration_sec: int = CALIBRATION_DEFAULT_MAX_DURATION_SEC
    # Averaging window of the validation scan (phase 4), in seconds. The
    # approved WI-00015 process runs a 15-second calibration scan and a
    # 2-second validation scan: validation only reads back the just-written
    # calibration, so a stable BFI/BVI average (80 frames at 40 Hz) is
    # enough. The leading scan_delay_sec skip applies to both sub-scans.
    validation_duration_sec: int = 2
    # Trigger config dict (matches the JSON payload expected by
    # console.set_trigger_json). When non-None the workflow re-sends
    # this to the console firmware before each sub-scan, which resets
    # the firmware-side ``fsync_counter`` so the dark schedule starts
    # fresh and aligned. The bloodflow app's CQ flow does this every
    # time it sets up a scan; the calibration flow must do the same to
    # avoid the off-by-one symptom that comes from stale firmware
    # state inherited from a previous scan.
    #
    # Standard payload (matching pages/BloodFlow.qml):
    #   {
    #     "TriggerStatus": 2,                # 2=ON, 1=OFF
    #     "TriggerFrequencyHz": 40,
    #     "TriggerPulseWidthUsec": 500,
    #     "LaserPulseDelayUsec": 100,
    #     "LaserPulseWidthUsec": 500,
    #     "LaserPulseSkipInterval": 600,
    #     "LaserPulseSkipDelayUsec": 1800,
    #     "EnableSyncOut": True,
    #     "EnableTaTrigger": True,
    #   }
    trigger_config: Optional[dict] = None
    notes: str = ""
    # Prepended verbatim to the calibration-<ts>.csv/.json artifact names
    # (e.g. "CS0123-" so files sort by console serial). The caller supplies
    # a filesystem-safe value; empty keeps the historical names.
    artifact_prefix: str = ""
    average_full_scan: bool = False
    # Explicit opt-in to run with thresholds that cannot fail the
    # pre-write gate (mean/contrast missing or <= 0 for an active
    # camera). Without it start_calibration() refuses such a request:
    # zero thresholds turn the entire #199 protection chain (gate,
    # never-write, rollback, PASS verdict) into a no-op, which is how a
    # far-below-spec calibration once reached a console EEPROM and
    # reported PASSED (#256). Bench/plumbing callers that genuinely
    # want an ungated run say so here, loudly, instead of encoding it
    # in threshold values.
    allow_ungated: bool = False


@dataclass
class CalibrationResultRow:
    camera_index: int
    side: str
    cam_id: int
    mean: float
    avg_contrast: float
    bfi: float
    bvi: float
    dark: float
    mean_test: str
    contrast_test: str
    bfi_test: str
    bvi_test: str
    dark_test: str
    security_id: str
    hwid: str


class CalibrationOutcome(str, enum.Enum):
    """Single authoritative terminal state of a calibration / test-scan
    procedure. Replaces consumer-side guessing from the ok/passed/
    canceled boolean triple (which allowed 16 combinations, ~5 of them
    meaningful, and could not distinguish a watchdog timeout from an
    operator cancel)."""
    PASSED = "passed"        # ran end-to-end, all cameras met thresholds
    FAILED = "failed"        # ran to a verdict, >=1 camera missed a threshold
                             # (at the pre-write gate or at validation);
                             # nothing was written to the console
    CANCELED = "canceled"    # cancel_calibration() stopped it
    TIMED_OUT = "timed_out"  # max_duration_sec watchdog stopped it
    ERROR = "error"          # broke before completing (flash, USB, degenerate data, ...)
    OVERRIDDEN = "overridden"  # written to the console under explicit operator
                               # override (start_calibration on_override_fn);
                               # >=1 camera missed a threshold. Never "passed".


def _resolve_outcome(
    *, ok: bool, passed: bool, canceled: bool, timed_out: bool,
    overridden: bool = False,
) -> CalibrationOutcome:
    if timed_out:
        return CalibrationOutcome.TIMED_OUT
    if canceled:
        return CalibrationOutcome.CANCELED
    if not ok:
        return CalibrationOutcome.ERROR
    if overridden:
        return CalibrationOutcome.OVERRIDDEN
    return CalibrationOutcome.PASSED if passed else CalibrationOutcome.FAILED


@dataclass
class CalibrationResult:
    ok: bool
    passed: bool
    canceled: bool
    error: str
    csv_path: str
    json_path: str
    calibration: Optional[Calibration]
    rows: list[CalibrationResultRow]
    calibration_scan_left_path: str
    calibration_scan_right_path: str
    validation_scan_left_path: str
    validation_scan_right_path: str
    started_timestamp: str
    outcome: Optional[CalibrationOutcome] = None
    # True only when this run wrote the console EEPROM — i.e. every
    # camera cleared every threshold and the post-validation write
    # succeeded. Any other outcome leaves the console untouched: the
    # proposed calibration only ever existed in the SDK's in-memory
    # cache during the validation scan.
    calibration_written: bool = False
    # Operator override (start_calibration(on_override_fn=...)): True when
    # the operator consented at the pre-write gate, plus the justification
    # they gave. ``outcome`` is OVERRIDDEN only when that consent led to a
    # write.
    override_granted: bool = False
    override_justification: str = ""


@dataclass
class TestScanResult:
    """Outcome of a stand-alone Test scan — phase 1 only, no calibration
    write, no validation scan. Shape mirrors ``CalibrationResult`` so the
    bloodflow-app's QML layer can re-use the row formatting code, but the
    fields are scoped to what a test scan actually produces (no
    ``calibration`` field — Test scans don't write to console EEPROM, no
    ``validation_scan_*_path`` — there's no validation scan).
    """
    ok: bool
    passed: bool
    canceled: bool
    error: str
    csv_path: str
    json_path: str
    rows: list[CalibrationResultRow]
    test_scan_left_path: str
    test_scan_right_path: str
    started_timestamp: str
    mode: str = "test"
    outcome: Optional[CalibrationOutcome] = None


# ---------------------------------------------------------------------------
# Pure compute helpers — no hardware, no UART. Tested in
# tests/test_calibration_workflow_compute.py.
# ---------------------------------------------------------------------------

from omotion.MotionProcessing import Sample


class DegenerateCalibrationError(RuntimeError):
    """Raised when an active camera's calibration scan produces unusable
    data (zero / negative aggregates), making BFI/BVI math impossible."""


def _camera_active(mask: int, cam_id: int) -> bool:
    return bool(mask & (1 << cam_id))


def _compute_calibration_from_samples(
    samples: list[Sample],
    *,
    left_camera_mask: int,
    right_camera_mask: int,
    baseline: Optional[Calibration] = None,
) -> Calibration:
    """Core calibration math: aggregate dark-corrected Samples into a
    ``(MODULES, CAMS_PER_MODULE)`` Calibration.

    Pure function — no I/O. Caller pre-filters ``samples`` to the
    averaging window. Each input Sample should be from the science
    pipeline's corrected stream (``is_corrected=True``): ``mean``
    is dark-baseline-subtracted, ``std_dev`` has shot-noise removed,
    ``contrast = std_dev / mean`` is physical speckle contrast.

    Inactive cameras (cam_id whose bit is not set in the per-side
    mask) inherit their value from ``baseline`` if provided, otherwise
    from :meth:`Calibration.default`. This is what makes a target-
    restricted calibration (e.g. ``left_camera_mask=0xFF, right_camera_mask=0``)
    safe: with the live console calibration passed in as ``baseline``,
    the un-targeted module's row carries forward instead of being
    clobbered with SDK defaults at write time. See bloodflow-app
    issue #117.
    """
    if baseline is None:
        baseline = Calibration.default()
    c_max = baseline.c_max.copy()
    i_max = baseline.i_max.copy()
    c_min = baseline.c_min.copy()
    i_min = baseline.i_min.copy()

    masks = (left_camera_mask, right_camera_mask)

    for module_idx, side in enumerate(("left", "right")):
        mask = masks[module_idx]
        for cam_id in range(CAMS_PER_MODULE):
            if not _camera_active(mask, cam_id):
                continue  # inactive — keep default value
            cam_samples = [
                s for s in samples
                if s.side == side and s.cam_id == cam_id
            ]
            if not cam_samples:
                raise DegenerateCalibrationError(
                    f"active camera ({side}, cam={cam_id + 1}) produced "
                    f"no corrected samples; calibration aborted."
                )
            # C_max is a contrast averaged over time, so it uses
            # average-of-ratios — the mean of each frame's std/mean —
            # never ratio-of-averages (mean(std)/mean(mean)). The two
            # disagree whenever the per-frame mean varies across the
            # window. Average-of-ratios is the project-wide standard for
            # any time-averaged contrast and matches the validation
            # scan's avg_contrast, so the two are directly comparable
            # (issue #148).
            mean_avg = float(np.mean([s.mean for s in cam_samples]))
            std_avg = float(np.mean([s.std_dev for s in cam_samples]))
            new_c_max = float(np.mean([s.contrast for s in cam_samples]))
            new_i_max = CALIBRATION_I_MAX_MULTIPLIER * mean_avg
            # Ratio-of-averages is logged alongside for diagnosis: a
            # large divergence between the two estimators is the smoking
            # gun for a flaky / partially-occluded camera.
            ratio_of_averages = (std_avg / mean_avg) if mean_avg > 0.0 else 0.0
            logger.info(
                "  cam (%s, cam=%d): n=%d  mean=%.2f  std=%.2f  "
                "C_max(avg-of-ratios)=%.4f  ratio-of-averages=%.4f  "
                "I_max=%.2f",
                side, cam_id + 1, len(cam_samples),
                mean_avg, std_avg, new_c_max, ratio_of_averages,
                new_i_max,
            )
            if new_c_max <= 0.0 or new_i_max <= 0.0:
                raise DegenerateCalibrationError(
                    f"active camera ({side}, cam={cam_id + 1}) produced "
                    f"zero or negative aggregate (C_max={new_c_max:.4f}, "
                    f"I_max={new_i_max:.4f}); calibration aborted."
                )
            c_max[module_idx, cam_id] = new_c_max
            i_max[module_idx, cam_id] = new_i_max

    return Calibration(
        c_min=c_min, c_max=c_max,
        i_min=i_min, i_max=i_max,
        source="console",
    )


def _format_calibration(cal: Calibration) -> str:
    """Return a multi-line human-readable dump of a Calibration's arrays.
    Cameras are labeled 1..8 (not 0..7)."""
    header = "  " + " " * 21 + "  ".join(f"{cam:>8d}" for cam in range(1, CAMS_PER_MODULE + 1))

    def _row(label: str, arr: np.ndarray) -> str:
        rows = []
        for module_idx, side in enumerate(("left ", "right")):
            vals = "  ".join(f"{v:>8.4f}" for v in arr[module_idx])
            rows.append(f"  {label} {side} (m={module_idx}): {vals}")
        return "\n".join(rows)

    return (
        f"Calibration(source={cal.source!r}):\n"
        f"{header}    (cam #)\n"
        f"{_row('C_min', cal.c_min)}\n"
        f"{_row('C_max', cal.c_max)}\n"
        f"{_row('I_min', cal.i_min)}\n"
        f"{_row('I_max', cal.i_max)}"
    )


def _threshold_test(value: float, thresholds: list[float], cam_id: int) -> str:
    """PASS if the threshold list doesn't cover this cam_id, or value
    >= threshold."""
    if cam_id >= len(thresholds):
        return "PASS"
    t = thresholds[cam_id]
    if t is None or not isinstance(t, (int, float)):
        return "PASS"
    return "PASS" if value >= float(t) else "FAIL"


def _threshold_max_test(
    value: float,
    maxs: Optional[list[float]],
    cam_id: int,
) -> str:
    """PASS if the upper-bound list is None / shorter than cam_id /
    has a non-numeric entry, or value <= max[cam_id]."""
    if maxs is None or cam_id >= len(maxs):
        return "PASS"
    m = maxs[cam_id]
    if m is None or not isinstance(m, (int, float)):
        return "PASS"
    return "PASS" if value <= float(m) else "FAIL"


def _combined_test(*results: str) -> str:
    """Combine multiple PASS/FAIL labels — overall PASS only if every
    sub-test is PASS."""
    return "PASS" if all(r == "PASS" for r in results) else "FAIL"


def _build_result_rows_from_samples(
    samples: list[Sample],
    *,
    dark_samples: Optional[list[Sample]] = None,
    left_camera_mask: int,
    right_camera_mask: int,
    thresholds: CalibrationThresholds,
    sensor_left,
    sensor_right,
) -> list[CalibrationResultRow]:
    """Core row aggregation: per-camera mean/contrast/BFI/BVI averages
    and threshold pass/fail. Pure function — caller pre-filters.

    ``dark_samples`` is the leading + trailing out-of-window samples
    from the validation scan (laser off; per-camera mean is the
    ambient-light reading). When supplied alongside
    ``thresholds.max_dark_per_camera`` the row builder also evaluates
    the dark gate (#122). When either is absent each row's
    ``dark_test`` is ``"NA"`` and ``dark`` is the measured mean (NaN
    if no dark samples were captured for that camera).
    """
    rows: list[CalibrationResultRow] = []
    masks = (left_camera_mask, right_camera_mask)
    sensors = (sensor_left, sensor_right)
    dark_samples = dark_samples or []

    for module_idx, side in enumerate(("left", "right")):
        mask = masks[module_idx]
        sensor = sensors[module_idx]
        for cam_id in range(CAMS_PER_MODULE):
            if not _camera_active(mask, cam_id):
                continue
            cam_samples = [
                s for s in samples
                if s.side == side and s.cam_id == cam_id
            ]
            if not cam_samples:
                continue   # silently drop — no data for this active cam

            mean_val = float(np.mean([s.mean for s in cam_samples]))
            contrast_val = float(np.mean([s.contrast for s in cam_samples]))
            bfi_val = float(np.mean([s.bfi for s in cam_samples]))
            bvi_val = float(np.mean([s.bvi for s in cam_samples]))

            cam_dark_samples = [
                s for s in dark_samples
                if s.side == side and s.cam_id == cam_id
            ]
            if cam_dark_samples:
                dark_val = float(np.mean([s.mean for s in cam_dark_samples]))
            else:
                dark_val = float("nan")

            if thresholds.max_dark_per_camera is None:
                dark_test = "NA"
            elif cam_id >= len(thresholds.max_dark_per_camera):
                dark_test = "NA"
            elif not cam_dark_samples:
                # Active camera but zero dark frames captured — surface
                # as FAIL rather than silently passing.
                dark_test = "FAIL"
            else:
                cap = thresholds.max_dark_per_camera[cam_id]
                dark_test = "PASS" if dark_val <= float(cap) else "FAIL"

            security_id = ""
            hwid = ""
            if sensor is not None and hasattr(sensor, "get_cached_camera_security_uid"):
                try:
                    security_id = str(sensor.get_cached_camera_security_uid(cam_id) or "")
                except Exception:
                    security_id = ""
                try:
                    hwid = str(sensor.get_cached_hardware_id() or "")
                except Exception:
                    hwid = ""

            # BFI / BVI support an optional upper bound for target-style
            # criteria (e.g. BFI = 0 ± 0.1 → min=-0.1, max=+0.1).
            bfi_test = _combined_test(
                _threshold_test(bfi_val, thresholds.min_bfi_per_camera, cam_id),
                _threshold_max_test(bfi_val, thresholds.max_bfi_per_camera, cam_id),
            )
            bvi_test = _combined_test(
                _threshold_test(bvi_val, thresholds.min_bvi_per_camera, cam_id),
                _threshold_max_test(bvi_val, thresholds.max_bvi_per_camera, cam_id),
            )
            rows.append(CalibrationResultRow(
                camera_index=len(rows),
                side=side,
                cam_id=cam_id,
                mean=mean_val,
                avg_contrast=contrast_val,
                bfi=bfi_val,
                bvi=bvi_val,
                dark=dark_val,
                mean_test=_threshold_test(mean_val, thresholds.min_mean_per_camera, cam_id),
                contrast_test=_threshold_test(contrast_val, thresholds.min_contrast_per_camera, cam_id),
                bfi_test=bfi_test,
                bvi_test=bvi_test,
                dark_test=dark_test,
                security_id=security_id,
                hwid=hwid,
            ))

    return rows


def evaluate_passed(rows: list[CalibrationResultRow]) -> bool:
    if not rows:
        return False
    return all(
        r.mean_test == "PASS"
        and r.contrast_test == "PASS"
        and r.bfi_test == "PASS"
        and r.bvi_test == "PASS"
        and r.dark_test != "FAIL"
        for r in rows
    )


def evaluate_gate_passed(rows: list[CalibrationResultRow]) -> bool:
    """Pre-write gate (#199): mean + contrast only.

    Evaluated on the *calibration* scan, before anything is written to the
    console. Both quantities are calibration-independent — mean is the raw
    pixel average and contrast is speckle std/mean — so applying the newly
    computed calibration cannot change them. That is what makes it sound to
    judge them one scan early: a camera that is too dim here will still be
    too dim in the validation scan.

    BFI/BVI are deliberately excluded — they *are* the calibrated
    quantities, so they only become meaningful after the calibration is
    applied, which is what the validation scan is for. Ambient-dark is
    excluded too: phase 1's dark frames are captured but the ambient
    criterion is defined against the validation scan (#122).
    """
    if not rows:
        return False
    return all(
        r.mean_test == "PASS" and r.contrast_test == "PASS"
        for r in rows
    )


def ungated_cameras(
    thresholds: CalibrationThresholds,
    left_camera_mask: int,
    right_camera_mask: int,
) -> list[str]:
    """Active cameras whose pre-write gate is a no-op, as ``L1``..``R8``
    labels (empty list = the gate can fail, i.e. it actually gates).

    The gate judges mean and contrast, both non-negative quantities, so
    ``_threshold_test`` can only ever FAIL a camera whose threshold is a
    number > 0 at an index the list covers. A camera is reported here
    when either of its two gate thresholds is missing (list ``None`` or
    too short), non-numeric, NaN, or <= 0 — for that camera
    ``evaluate_gate_passed`` is unconditionally PASS and the #199
    protections cannot trigger.
    """
    def _effective(t_list: Optional[list], cam_id: int) -> bool:
        if t_list is None or cam_id >= len(t_list):
            return False
        t = t_list[cam_id]
        if t is None or not isinstance(t, (int, float)):
            return False
        return float(t) > 0  # NaN compares False -> ineffective

    labels: list[str] = []
    for prefix, mask in (("L", left_camera_mask), ("R", right_camera_mask)):
        for cam_id in range(CAMS_PER_MODULE):
            if not _camera_active(mask, cam_id):
                continue
            if not (
                _effective(thresholds.min_mean_per_camera, cam_id)
                and _effective(thresholds.min_contrast_per_camera, cam_id)
            ):
                labels.append(f"{prefix}{cam_id + 1}")
    return labels


_CSV_FIELDS = [
    "camera_index", "side", "cam",
    "mean", "avg_contrast", "bfi", "bvi", "dark",
    "mean_test", "contrast_test", "bfi_test", "bvi_test", "dark_test",
    "security_id", "hwid", "sensor_serial", "console_serial",
]


def write_result_csv(
    path: str,
    rows: list[CalibrationResultRow],
    *,
    console_serial: str = "",
    left_sensor_serial: str = "",
    right_sensor_serial: str = "",
) -> None:
    """Write CalibrationResultRow list to ``path`` in the canonical
    column order. Creates parent directories if needed.

    The ``cam`` column is 1-indexed (1..8), matching how cameras are
    physically labeled. Internally ``CalibrationResultRow.cam_id`` is
    still 0-indexed (so it can be used to lookup into the per-camera
    threshold arrays).

    ``sensor_serial`` is the programmed serial of the module each row's
    camera belongs to (picked by ``row.side``); ``console_serial`` is the
    console EEPROM serial, repeated per row so the CSV stays traceable to
    the physical unit on its own. Both are "" when unprogrammed/unread.
    """
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=_CSV_FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({
                "camera_index": r.camera_index,
                "side": r.side,
                "cam": r.cam_id + 1,
                "mean": f"{r.mean:.4f}",
                "avg_contrast": f"{r.avg_contrast:.6f}",
                "bfi": f"{r.bfi:.4f}",
                "bvi": f"{r.bvi:.4f}",
                "dark": f"{r.dark:.4f}",
                "mean_test": r.mean_test,
                "contrast_test": r.contrast_test,
                "bfi_test": r.bfi_test,
                "bvi_test": r.bvi_test,
                "dark_test": r.dark_test,
                "security_id": r.security_id,
                "hwid": r.hwid,
                "sensor_serial": (
                    left_sensor_serial if r.side == "left"
                    else right_sensor_serial
                ),
                "console_serial": console_serial,
            })


# ---------------------------------------------------------------------------
# JSON manifest — full record of a calibration run, including every device
# identity that produced the data so the file is self-describing on its own
# (no need to cross-reference logs to know which firmware / hardware
# generated a given calibration).
# ---------------------------------------------------------------------------

_JSON_SCHEMA_VERSION = 1


def _safe_call(fn: Callable[[], object], default: object = "") -> object:
    """Call ``fn()`` and swallow any exception, returning ``default``.

    Device-info reads can fail (disconnect, transient bus error). Manifest
    writing must never abort the calibration result, so each field is
    pulled defensively.
    """
    try:
        return fn()
    except Exception:
        return default


def _read_device_serial(device) -> str:
    """Best-effort read of a device's programmed serial number (console
    EEPROM or sensor module). Returns "" when the device is absent, the
    serial is unprogrammed, or the read fails — identity reads must never
    abort report writing.
    """
    if device is None:
        return ""
    return str(_safe_call(lambda: device.read_serial_number(), "") or "")


def _collect_host_info() -> dict:
    return {
        "hostname": _safe_call(socket.gethostname, ""),
        "platform": _safe_call(platform.platform, ""),
        "python": sys.version.split()[0],
    }


def _collect_sdk_info() -> dict:
    try:
        from omotion import __version__ as sdk_version
    except Exception:
        sdk_version = ""
    return {"version": sdk_version}


def _collect_console_info(console) -> dict:
    if console is None:
        return {"serial": "", "hwid": "", "firmware_version": ""}
    return {
        "serial": _read_device_serial(console),
        "hwid": str(_safe_call(console.get_hardware_id, "") or ""),
        "firmware_version": str(_safe_call(console.get_version, "") or ""),
    }


def _collect_sensor_info(sensor, camera_mask: int) -> dict:
    if sensor is None:
        return {
            "connected": False,
            "serial": "",
            "hwid": "",
            "firmware_version": "",
            "camera_mask": f"0x{camera_mask:02X}",
        }
    hwid = _safe_call(sensor.get_cached_hardware_id, "") or _safe_call(sensor.get_hardware_id, "")
    return {
        "connected": True,
        "serial": _read_device_serial(sensor),
        "hwid": str(hwid or ""),
        "firmware_version": str(_safe_call(sensor.get_version, "") or ""),
        "camera_mask": f"0x{camera_mask:02X}",
    }


def _calibration_to_dict(cal: Optional[Calibration]) -> Optional[dict]:
    if cal is None:
        return None
    return {
        "source": cal.source,
        "c_min": cal.c_min.tolist(),
        "c_max": cal.c_max.tolist(),
        "i_min": cal.i_min.tolist(),
        "i_max": cal.i_max.tolist(),
    }


def _row_with_thresholds(
    r: CalibrationResultRow, thresholds: CalibrationThresholds
) -> dict:
    """Per-camera record matching the log table — measurement, the
    threshold values it was tested against, and PASS/FAIL."""
    def _get(arr, idx):
        if arr is None or idx >= len(arr):
            return None
        v = arr[idx]
        return float(v) if isinstance(v, (int, float)) else None

    return {
        "camera_index": r.camera_index,
        "side": r.side,
        "cam": r.cam_id + 1,
        "security_id": r.security_id,
        "sensor_hwid": r.hwid,
        "mean": r.mean,
        "min_mean": _get(thresholds.min_mean_per_camera, r.cam_id),
        "mean_test": r.mean_test,
        "avg_contrast": r.avg_contrast,
        "min_contrast": _get(thresholds.min_contrast_per_camera, r.cam_id),
        "contrast_test": r.contrast_test,
        "bfi": r.bfi,
        "min_bfi": _get(thresholds.min_bfi_per_camera, r.cam_id),
        "max_bfi": _get(thresholds.max_bfi_per_camera, r.cam_id),
        "bfi_test": r.bfi_test,
        "bvi": r.bvi,
        "min_bvi": _get(thresholds.min_bvi_per_camera, r.cam_id),
        "max_bvi": _get(thresholds.max_bvi_per_camera, r.cam_id),
        "bvi_test": r.bvi_test,
        "dark": r.dark,
        "max_dark": _get(thresholds.max_dark_per_camera, r.cam_id),
        "dark_test": r.dark_test,
    }


def write_result_json(
    path: str,
    *,
    started_timestamp: str,
    passed: bool,
    canceled: bool,
    error: str,
    request: CalibrationRequest,
    rows: list[CalibrationResultRow],
    calibration: Optional[Calibration],
    scan_paths: dict,
    interface,
    mode: str = "calibrate",
    outcome: str = "",
    calibration_written: bool = False,
    override_granted: bool = False,
    override_justification: str = "",
) -> None:
    """Write a self-describing JSON manifest of the calibration run.

    Includes the per-camera result table, the proposed calibration arrays
    (``calibration_written`` records whether this run put them on the
    console EEPROM), the camera/sensor/console identities
    (serial numbers, security UIDs, HWIDs, firmware versions), and host
    info — so the file is enough on its own to trace a run back to the
    exact hardware + firmware that produced it.
    """
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)

    started_iso = ""
    try:
        started_iso = datetime.datetime.strptime(
            started_timestamp, "%Y%m%d_%H%M%S"
        ).astimezone().isoformat()
    except Exception:
        pass

    manifest = {
        "schema_version": _JSON_SCHEMA_VERSION,
        "mode": mode,
        "started_timestamp": started_timestamp,
        "started_iso": started_iso,
        "passed": passed,
        "canceled": canceled,
        "error": error,
        "outcome": outcome,
        # True only when this run wrote the console EEPROM (every camera
        # cleared every threshold). The "calibration" arrays below are the
        # proposed values either way — written on a pass, discarded (never
        # on the console) otherwise.
        "calibration_written": calibration_written,
        # Operator override at the pre-write gate (bloodflow-app#482): the
        # console was written although >=1 camera missed a threshold.
        "override_granted": override_granted,
        "override_justification": override_justification,
        "operator_id": request.operator_id,
        "notes": request.notes,
        "host": _collect_host_info(),
        "sdk": _collect_sdk_info(),
        "console": _collect_console_info(getattr(interface, "console", None)),
        "sensors": {
            "left": _collect_sensor_info(
                getattr(interface, "left", None), request.left_camera_mask,
            ),
            "right": _collect_sensor_info(
                getattr(interface, "right", None), request.right_camera_mask,
            ),
        },
        "request": {
            "duration_sec": request.duration_sec,
            "scan_delay_sec": request.scan_delay_sec,
            "validation_duration_sec": request.validation_duration_sec,
            "max_duration_sec": request.max_duration_sec,
            "left_camera_mask": request.left_camera_mask,
            "right_camera_mask": request.right_camera_mask,
        },
        "thresholds": {
            "min_mean_per_camera": list(request.thresholds.min_mean_per_camera),
            "min_contrast_per_camera": list(request.thresholds.min_contrast_per_camera),
            "min_bfi_per_camera": list(request.thresholds.min_bfi_per_camera),
            "max_bfi_per_camera": (
                list(request.thresholds.max_bfi_per_camera)
                if request.thresholds.max_bfi_per_camera is not None else None
            ),
            "min_bvi_per_camera": list(request.thresholds.min_bvi_per_camera),
            "max_bvi_per_camera": (
                list(request.thresholds.max_bvi_per_camera)
                if request.thresholds.max_bvi_per_camera is not None else None
            ),
            "max_dark_per_camera": (
                list(request.thresholds.max_dark_per_camera)
                if request.thresholds.max_dark_per_camera is not None else None
            ),
        },
        "calibration": _calibration_to_dict(calibration),
        "scan_paths": dict(scan_paths),
        "cameras": [_row_with_thresholds(r, request.thresholds) for r in rows],
    }

    with open(path, "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=2, sort_keys=False)


def _format_result_rows_table(
    rows: list[CalibrationResultRow],
    thresholds: CalibrationThresholds,
) -> str:
    """Multi-line per-camera table for the log: actual measurement,
    threshold(s), and PASS/FAIL for mean / contrast / BFI / BVI / dark."""
    def _t(arr: Optional[list[float]], i: int, fmt: str = "{:>7.3f}") -> str:
        if arr is None or i >= len(arr):
            return "   —   "
        v = arr[i]
        if not isinstance(v, (int, float)):
            return "   —   "
        return fmt.format(float(v))

    # Map PASS/FAIL/NA to a single character so the per-camera row stays
    # narrow. NA renders as a dash so masked-out cameras (or runs where
    # the threshold wasn't configured) don't look like failures.
    def pf(s: str) -> str:
        if s == "PASS":
            return "P"
        if s == "NA":
            return "-"
        return "F"

    header = (
        "| cam | side  |   mean   |   min    | M |   contrast |    min   | C |    bfi   |    min   |    max   | B |    bvi   |    min   |    max   | V |   dark   |    max   | D |"
    )
    sep = (
        "|-----|-------|----------|----------|---|------------|----------|---|----------|----------|----------|---|----------|----------|----------|---|----------|----------|---|"
    )
    lines = [header, sep]
    for r in rows:
        cam_id = r.cam_id  # 0..7 internally; display 1..8
        lines.append(
            "| {cam:>3d} | {side:<5} | {mean:>8.3f} | {mean_min} | {mean_pf} "
            "| {contrast:>10.5f} | {contrast_min} | {contrast_pf} "
            "| {bfi:>+8.3f} | {bfi_min} | {bfi_max} | {bfi_pf} "
            "| {bvi:>+8.3f} | {bvi_min} | {bvi_max} | {bvi_pf} "
            "| {dark:>8.3f} | {dark_max} | {dark_pf} |"
            .format(
                cam=cam_id + 1, side=r.side,
                mean=r.mean,
                mean_min=_t(thresholds.min_mean_per_camera, cam_id, "{:>8.2f}"),
                mean_pf=pf(r.mean_test),
                contrast=r.avg_contrast,
                contrast_min=_t(thresholds.min_contrast_per_camera, cam_id, "{:>8.4f}"),
                contrast_pf=pf(r.contrast_test),
                bfi=r.bfi,
                bfi_min=_t(thresholds.min_bfi_per_camera, cam_id, "{:>+8.3f}"),
                bfi_max=_t(thresholds.max_bfi_per_camera, cam_id, "{:>+8.3f}"),
                bfi_pf=pf(r.bfi_test),
                bvi=r.bvi,
                bvi_min=_t(thresholds.min_bvi_per_camera, cam_id, "{:>+8.3f}"),
                bvi_max=_t(thresholds.max_bvi_per_camera, cam_id, "{:>+8.3f}"),
                bvi_pf=pf(r.bvi_test),
                dark=r.dark,
                dark_max=_t(thresholds.max_dark_per_camera, cam_id, "{:>8.3f}"),
                dark_pf=pf(r.dark_test),
            )
        )
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Calibration collector sink
# ---------------------------------------------------------------------------


class _CalibrationCollectorSink:
    """Internal: collects corrected light samples + dark samples for the
    calibration math.

    Subscribes to two channels:

    * ``"final"``  — each payload is an ``EnrichedCorrectedInterval`` of
      light frames produced by DarkCorrectionStage after a dark interval
      closes. The sink slices each frame down to a legacy ``Sample``-shaped
      object (mean, std_dev, contrast, BFI, BVI) so the existing math
      functions (``_compute_calibration_from_samples``, ``_build_result_rows_from_samples``)
      keep working unchanged. Synthetic ``quality == "nan_filled"``
      placeholder frames (gap fill for frames lost in transit) are
      skipped — they are not measurements (#270).

    * ``"live"``   — each payload is a per-frame ``FrameBatch``. The sink
      picks out rows where ``frame_type == "dark"`` and emits a Sample
      whose ``mean`` is the pedestal-subtracted ``subtracted_mean`` (matching
      the legacy "u1 - PEDESTAL_HEIGHT" semantics used by the FT
      ambient-light gate).

    After the scan completes, the workflow drains ``corrected_samples`` and
    ``dark_samples`` and applies the existing frame-id windowing on the
    light samples (skip leading + frame_window_count cap).
    """

    channels: set = frozenset({"final", "live"})

    def __init__(self) -> None:
        self.corrected_samples: list[Sample] = []
        self.dark_samples: list[Sample] = []
        # Keep raw EnrichedCorrectedInterval payloads around for any future
        # consumer that wants more than the legacy Sample fields.
        self.batches: list = []

    def on_scan_start(self, meta) -> None:  # noqa: D401
        self.corrected_samples.clear()
        self.dark_samples.clear()
        self.batches.clear()

    def consume(self, channel: str, payload) -> None:
        if channel == "final":
            self.batches.append(payload)
            for f in payload.frames:
                # nan_filled frames are synthetic placeholders inserted by
                # TimestampRepairStage for frames that never arrived; one of
                # them NaNs the whole per-camera np.mean aggregate and fails
                # the run (#270). Same skip policy as side_avg.py /
                # batch.iter_rows. Real frames whose derived stats are NaN
                # (e.g. zero-light contrast) must still flow through so the
                # gate fails on them loudly — do not relax this into
                # NaN-aware averaging downstream.
                if str(getattr(f, "quality", "ok")) == "nan_filled":
                    continue
                self.corrected_samples.append(Sample(
                    side=f.side,
                    cam_id=f.cam_id,
                    frame_id=int(f.abs_frame_id) & 0xFF,
                    absolute_frame_id=int(f.abs_frame_id),
                    timestamp_s=float(f.t),
                    row_sum=0,
                    temperature_c=float("nan"),
                    mean=float(f.mean),
                    std_dev=float(f.std),
                    contrast=float(f.contrast),
                    bfi=float(f.bfi),
                    bvi=float(f.bvi),
                    is_corrected=True,
                    is_dark=False,
                ))
        elif channel == "live":
            # payload is a FrameBatch — pick out dark frames and emit Samples
            # with pedestal-subtracted mean (subtracted_mean).
            if payload.frame_type is None or payload.subtracted_mean is None:
                return
            n = payload.subtracted_mean.shape[0]
            for i in range(n):
                if str(payload.frame_type[i]) != "dark":
                    continue
                abs_id = (
                    int(payload.abs_frame_ids[i])
                    if payload.abs_frame_ids is not None
                    else int(payload.frame_ids[i])
                )
                ts = float(payload.timestamp_s[i])
                for side_idx, side in enumerate(("left", "right")):
                    for cam_id in range(8):
                        m = float(payload.subtracted_mean[i, side_idx, cam_id])
                        if not np.isfinite(m):
                            continue
                        temp = (
                            float(payload.temperature_c[i, side_idx, cam_id])
                            if payload.temperature_c is not None
                            else float("nan")
                        )
                        self.dark_samples.append(Sample(
                            side=side,
                            cam_id=cam_id,
                            frame_id=int(payload.frame_ids[i]),
                            absolute_frame_id=abs_id,
                            timestamp_s=ts,
                            row_sum=0,
                            temperature_c=temp,
                            mean=m,
                            std_dev=0.0,
                            contrast=0.0,
                            bfi=0.0,
                            bvi=0.0,
                            is_corrected=False,
                            is_dark=True,
                        ))

    def on_complete(self) -> None:
        pass


# ---------------------------------------------------------------------------
# Orchestration class
# ---------------------------------------------------------------------------

from omotion.ScanWorkflow import run_collection_scan


def _run_subscan_capture(
    interface,
    request: CalibrationRequest,
    *,
    subject_id: str,
    duration_sec: int,
    skip_leading_frames: int,
    frame_window_count: int,
    stop_evt: threading.Event,
) -> tuple[str, str, list[Sample], list[Sample]]:
    """Submit a ScanRequest and capture corrected samples in-memory as
    the science pipeline emits them.

    The scan still writes its raw histogram CSV to disk (`write_raw_csv=True`)
    so operators retain the artifact for later verification, but we
    don't re-parse it — corrected samples are captured live via
    ``on_corrected_batch_fn``. This avoids running the science pipeline
    twice on the same data.

    Returns ``(left_path, right_path, captured_samples, dark_samples)``.
    ``captured_samples`` is the in-window averaging set (laser-on
    corrected Samples, the historical return). ``dark_samples`` is
    the laser-off frames the science pipeline emits via
    ``on_dark_frame_fn``: each sample has ``is_dark=True,
    is_corrected=False`` and ``mean = u1 - PEDESTAL_HEIGHT`` —
    pedestal-subtracted ambient DN, same convention the CQ ambient
    gate uses (see motion_connector.py's _on_dark_frame). Used by the
    FT calibration's #122 ambient-light gate. Raises ``RuntimeError``
    on scan failure. Honors ``stop_evt`` by calling ``cancel_scan``
    and returning empty paths + empty lists.
    """
    collector = _CalibrationCollectorSink()

    # Shared short-scan engine (ScanWorkflow.run_collection_scan), the same one
    # the contact-quality check uses. Cancellable via stop_evt; raises on a
    # scan-level failure so the caller aborts calibration cleanly rather than
    # running the math on empty data.
    completed = run_collection_scan(
        interface.scan_workflow,
        collector,
        subject_id=subject_id,
        duration_sec=duration_sec,
        left_camera_mask=request.left_camera_mask,
        right_camera_mask=request.right_camera_mask,
        stop_evt=stop_evt,
        raise_on_error=True,
    )
    if not completed:  # canceled (mid-scan stop_evt or last_scan_canceled)
        return "", "", [], []

    # Apply the legacy windowing on light samples here so the sink's
    # collection logic stays uncoupled from the calibration-specific frame
    # window. Dark samples bypass windowing — every dark frame in the
    # schedule is a valid ambient reading (#122).
    upper_bound = skip_leading_frames + int(frame_window_count)
    captured: list[Sample] = [
        s for s in collector.corrected_samples
        if skip_leading_frames <= s.absolute_frame_id < upper_bound
    ]
    dark = list(collector.dark_samples)
    captured.sort(key=lambda s: (s.side, s.cam_id, s.absolute_frame_id))
    dark.sort(key=lambda s: (s.side, s.cam_id, s.absolute_frame_id))

    # Raw CSV paths are written by the SDK's default CsvSink (skip_default_storage=False
    # would attach it automatically; here we set skip_default_storage=True so we
    # don't double-write). The legacy contract returned scan paths for the
    # CalibrationResult; pass empty strings — calibration JSON is the
    # authoritative artifact, and CSVs aren't produced by this scan path.
    return "", "", captured, dark


class CalibrationWorkflow:
    def __init__(self, interface: "MotionInterface"):
        self._interface = interface
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()
        self._running = False

    @property
    def running(self) -> bool:
        with self._lock:
            return self._running

    def start_calibration(
        self,
        request: CalibrationRequest,
        *,
        on_log_fn: Optional[Callable[[str], None]] = None,
        on_progress_fn: Optional[Callable[[str], None]] = None,
        on_complete_fn: Optional[Callable[[CalibrationResult], None]] = None,
        on_override_fn: Optional[
            Callable[[list[CalibrationResultRow]], object]
        ] = None,
    ) -> bool:
        """Run the calibration procedure on a worker thread.

        **If any camera misses any threshold, the whole run FAILS and the
        console EEPROM is never written.** The pre-write gate (#199)
        judges the calibration scan's mean/contrast; the proposed
        calibration is then applied to the SDK's in-memory cache only, the
        validation scan measures BFI/BVI/dark against it, and the EEPROM
        write happens after — and only after — every camera clears every
        threshold. There is no rollback: a failed run has nothing to undo.
        (Deliberate ungated bench runs gate-disable via
        ``CalibrationRequest.allow_ungated`` instead.)

        ``on_override_fn`` is the engineering-only operator override
        (bloodflow-app#482, ``omotion.calibration.override``); the clinical
        app never passes it. When given, a pre-write gate failure calls it
        on the worker thread with the gate rows instead of failing. A truthy
        return (``bool``, or an object with ``accepted``/``justification``)
        lets the run continue to validation, after which the console is
        written with outcome ``OVERRIDDEN`` — unless any camera fails the
        ambient-dark check, which is never overridable and leaves the
        console untouched. A falsy return is the usual FAILED path.

        **A side that is not being calibrated keeps its stored values.**
        The written block covers both modules, so a one-side run (mask 0x00
        on the other side) carries that side's row forward. Its baseline is
        read fresh from the console before the calibration scan — never
        taken from the SDK's in-memory cache, which starts as SDK defaults
        and is only populated when the host calls ``log_console_info``
        (#281: the WI-15 script runs each side in a fresh process). If that
        read fails the run ends as ERROR before scanning and nothing is
        written. A console with no calibration block reads as SDK defaults,
        and those are what gets carried forward.

        Returns False without starting when a run is already in flight, or
        when the request's thresholds cannot fail the pre-write gate and
        ``request.allow_ungated`` is not set (#256) — the refusal reason is
        logged and sent through ``on_log_fn``.
        """
        ungated = ungated_cameras(
            request.thresholds,
            request.left_camera_mask,
            request.right_camera_mask,
        )
        if ungated and not request.allow_ungated:
            msg = (
                "Calibration refused: the pre-write gate cannot fail for "
                f"{', '.join(ungated)} (min mean/contrast threshold missing "
                "or <= 0), so a below-spec calibration would be written to "
                "the console EEPROM and reported PASSED. Supply real "
                "thresholds (factory_calibration_thresholds()) or set "
                "CalibrationRequest.allow_ungated=True for a deliberate "
                "ungated engineering run."
            )
            logger.error(msg)
            if on_log_fn:
                on_log_fn(msg)
            return False

        with self._lock:
            if self._running:
                logger.warning("start_calibration refused: already running.")
                return False
            self._running = True
        self._stop_evt = threading.Event()

        def _emit_log(msg: str) -> None:
            logger.info(msg)
            if on_log_fn:
                on_log_fn(msg)

        def _emit_progress(stage: str) -> None:
            if on_progress_fn:
                on_progress_fn(stage)

        def _worker() -> None:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            cal_left = cal_right = ""
            val_left = val_right = ""
            cal_obj: Optional[Calibration] = None
            csv_path = ""
            json_path = ""
            rows: list[CalibrationResultRow] = []
            ok = False
            passed = False
            error = ""
            canceled = False
            timed_out = False
            prior_cal: Optional[Calibration] = None
            baseline: Optional[Calibration] = None
            wrote_calibration = False
            applied_override = False
            override_granted = False
            override_justification = ""
            below = ""

            logger.info(
                "Calibration: starting procedure (operator=%s, output_dir=%s, "
                "masks=(0x%02X, 0x%02X), duration_sec=%d, scan_delay_sec=%d, "
                "max_duration_sec=%d, ts=%s)",
                request.operator_id, request.output_dir,
                request.left_camera_mask, request.right_camera_mask,
                request.duration_sec, request.scan_delay_sec,
                request.max_duration_sec, ts,
            )

            def _watchdog() -> None:
                nonlocal timed_out
                timed_out = True
                self._stop_evt.set()
                logger.warning(
                    "Calibration watchdog fired after %d sec; aborting.",
                    request.max_duration_sec,
                )
                try:
                    self._interface.scan_workflow.cancel_scan()
                except Exception:
                    logger.exception("Calibration watchdog: cancel_scan raised.")
            wd = threading.Timer(request.max_duration_sec, _watchdog)
            wd.daemon = True
            wd.start()

            skip_frames = int(round(request.scan_delay_sec * CAPTURE_HZ))
            # Bound the trailing edge to keep the firmware's terminal
            # dark frame (and any laser ramp-down) out of the average.
            window_frames = int(round(request.duration_sec * CAPTURE_HZ))
            # Phase 1 (calibration scan) widens its averaging window
            # to swallow every laser-on corrected sample after the
            # leading scan_delay_sec skip when average_full_scan is set
            # (#132 — "all of the corrected data ... averaged, not just
            # the rolling average numbers"). Dark frames flow through
            # on_dark_frame_fn, not on_corrected_batch, so they're not
            # affected by this widening. Phase 4 (validation scan) uses
            # its own validation_duration_sec window.
            phase1_window_frames = (
                10 ** 9 if request.average_full_scan else window_frames
            )

            def _flash_sensors() -> tuple[bool, str]:
                """Re-flash the FPGA bitstream and reinitialize the
                camera sensors. Equivalent to FlashSensorsTask in the
                bloodflow app's QML scan chain. Resets the sensor's
                frame counter, clears any residual COMM endpoint
                state, and puts the FPGA + cameras into a known-good
                configuration.

                Returns ``(ok, error_message)``. Synchronous from the
                worker's perspective — blocks on a local event until
                ``start_configure_camera_sensors`` fires its
                ``on_complete_fn``.
                """
                from omotion.ScanWorkflow import ConfigureRequest, ConfigureResult

                cfg_req = ConfigureRequest(
                    left_camera_mask=request.left_camera_mask,
                    right_camera_mask=request.right_camera_mask,
                    power_off_unused_cameras=False,
                )
                evt = threading.Event()
                holder: dict[str, ConfigureResult] = {}

                def _on_done(r: ConfigureResult) -> None:
                    holder["r"] = r
                    evt.set()

                def _on_log(msg: str) -> None:
                    logger.info("Calibration flash: %s", msg)

                started = self._interface.start_configure_camera_sensors(
                    cfg_req,
                    on_log_fn=_on_log,
                    on_complete_fn=_on_done,
                )
                if not started:
                    return False, (
                        "start_configure_camera_sensors refused "
                        "(another configure already running?)"
                    )

                while not evt.wait(timeout=0.2):
                    if self._stop_evt.is_set():
                        return False, "canceled during flash"
                res = holder.get("r")
                if res is None:
                    return False, "flash completed with no result"
                return bool(res.ok), str(res.error or "")

            def _reset_firmware_trigger(phase_label: str) -> None:
                """Send the trigger config to the firmware before each
                sub-scan. Resets the firmware's ``fsync_counter`` to 1
                so the dark schedule starts fresh.

                Single attempt — flash already put the firmware into a
                known state, so timeouts shouldn't happen here. If
                this does fail, the dark-integrity monitor catches any
                resulting schedule misalignment.
                """
                # Resolve to (interface default ⊕ request override).
                # Per-request fields win; absent fields fall through
                # to the SDK / app-level default. Always populated, so
                # we no longer need a None check / skip-the-reset
                # branch — the trigger is always reset before each
                # phase, which is what we want for the firmware
                # fsync_counter alignment guarantee anyway.
                trigger_cfg = self._interface.resolve_trigger_config(
                    request.trigger_config
                )
                try:
                    self._interface.console.set_trigger_json(
                        data=trigger_cfg,
                    )
                    logger.info(
                        "Calibration %s: trigger reset OK "
                        "(firmware fsync_counter=1).", phase_label,
                    )
                except Exception as e:
                    logger.error(
                        "Calibration %s: trigger reset failed: %s. "
                        "Continuing — dark-integrity monitor will catch "
                        "any schedule misalignment.",
                        phase_label, e,
                    )

            try:
                _emit_progress("flash_sensors")
                _emit_log("Calibration: flashing sensors / FPGA…")
                logger.info(
                    "Calibration phase 0: re-flash sensors so frame "
                    "counters and COMM endpoints start in a known state."
                )
                flash_ok, flash_err = _flash_sensors()
                if not flash_ok:
                    error = f"flash phase failed: {flash_err}"
                    canceled = self._stop_evt.is_set()   # was: "canceled" in flash_err
                    return
                logger.info("Calibration phase 0 done: sensors flashed.")
                if self._stop_evt.is_set():
                    canceled = True
                    error = "canceled after flash"
                    return

                # ── Phase 0.5: read the console's current calibration ────
                # The side not being calibrated (mask 0x00) keeps its stored
                # row: the block written in phase 6 carries it forward from
                # this read. Read the console, never the SDK cache: the
                # cache starts as SDK defaults and is only populated when
                # the host calls log_console_info(), so a right-only run in
                # a fresh process (the WI-15 script) overwrote the left
                # module's stored calibration with defaults (#281). A
                # failed read raises out of the worker — outcome ERROR,
                # nothing scanned, nothing written — rather than guessing.
                _emit_log("Calibration: reading current console calibration…")
                baseline = self._interface.refresh_calibration()
                logger.info(
                    "Calibration phase 0.5 done: console calibration read "
                    "(source=%s).", baseline.source,
                )

                _emit_progress("calibration_scan")
                _emit_log("Calibration: starting calibration scan…")
                logger.info(
                    "Calibration phase 1: calibration scan, "
                    "duration=%d sec (= %d duration + %d delay)",
                    request.duration_sec + request.scan_delay_sec,
                    request.duration_sec, request.scan_delay_sec,
                )
                if request.average_full_scan:
                    logger.info(
                        "Calibration phase 1: average_full_scan=True — "
                        "averaging every laser-on corrected sample after "
                        "the %d-frame leading skip (no upper-bound window).",
                        skip_frames,
                    )
                _reset_firmware_trigger("phase 1 (pre-scan)")
                # cal_dark_samples feeds the pre-write gate's row build
                # (#199) so its table can show an ambient column. The
                # ambient *criterion* still gates on validation-scan darks
                # (#122) — evaluate_gate_passed ignores dark_test — so the
                # pass/fail semantics are unchanged; these frames are only
                # here so the operator sees a complete table.
                cal_left, cal_right, cal_samples, cal_dark_samples = _run_subscan_capture(
                    self._interface, request,
                    subject_id=f"calib1_{request.operator_id}",
                    duration_sec=request.duration_sec + request.scan_delay_sec,
                    skip_leading_frames=skip_frames,
                    frame_window_count=phase1_window_frames,
                    stop_evt=self._stop_evt,
                )
                logger.info(
                    "Calibration phase 1 done: %d corrected samples captured "
                    "live; raw CSVs: left=%s  right=%s",
                    len(cal_samples),
                    cal_left or "(none)", cal_right or "(none)",
                )
                if self._stop_evt.is_set():
                    canceled = True
                    error = "canceled during calibration scan"
                    return

                _emit_progress("compute_calibration")
                _emit_log("Calibration: computing arrays…")
                logger.info("Calibration phase 2: computing (2, 8) arrays.")
                # Issue #117: inactive cameras (those excluded by a
                # left-only / right-only mask) keep their on-device values
                # instead of falling back to SDK defaults at write time.
                # ``baseline`` is the fresh console read from phase 0.5,
                # not the SDK cache.
                try:
                    cal_obj = _compute_calibration_from_samples(
                        cal_samples,
                        left_camera_mask=request.left_camera_mask,
                        right_camera_mask=request.right_camera_mask,
                        baseline=baseline,
                    )
                except DegenerateCalibrationError as e:
                    error = str(e)
                    return
                logger.info(
                    "Calibration phase 2 done — proposed calibration:\n%s",
                    _format_calibration(cal_obj),
                )

                # ── Phase 2.5: pre-write gate (#199) ──────────────────────
                # If any camera misses its mean/contrast bar the whole run
                # FAILS here, nothing written — unless the caller supplied
                # the engineering-only on_override_fn and the operator
                # consents (bloodflow-app#482). Both quantities are
                # calibration-independent, so judging them on the
                # calibration scan is sound (see evaluate_gate_passed).
                _emit_progress("gate")
                logger.info(
                    "Calibration phase 2.5: pre-write gate on calibration-"
                    "scan mean/contrast."
                )
                gate_rows = _build_result_rows_from_samples(
                    cal_samples,
                    dark_samples=cal_dark_samples,
                    left_camera_mask=request.left_camera_mask,
                    right_camera_mask=request.right_camera_mask,
                    thresholds=request.thresholds,
                    sensor_left=getattr(self._interface, "left", None),
                    sensor_right=getattr(self._interface, "right", None),
                )
                gate_passed = evaluate_gate_passed(gate_rows)
                below = ", ".join(
                    f"{'L' if r.side == 'left' else 'R'}{r.cam_id + 1}"
                    for r in gate_rows
                    if r.mean_test == "FAIL" or r.contrast_test == "FAIL"
                )
                if not gate_passed and on_override_fn is not None:
                    _emit_progress("override")
                    logger.warning(
                        "Calibration phase 2.5: gate FAIL — below threshold "
                        "on %s. Asking the operator for an override (console "
                        "EEPROM not written yet).", below,
                    )
                    _emit_log(
                        "Calibration: scan mean/contrast below threshold on "
                        f"{below}. Nothing has been written; asking the "
                        "operator whether to continue under override…"
                    )
                    try:
                        decision = on_override_fn(list(gate_rows))
                    except Exception:
                        logger.exception(
                            "on_override_fn raised; treating as declined."
                        )
                        decision = False
                    override_granted = bool(
                        getattr(decision, "accepted", decision)
                    )
                    override_justification = str(
                        getattr(decision, "justification", "") or ""
                    )
                    if override_granted:
                        logger.warning(
                            "Calibration phase 2.5: operator OVERRIDE granted "
                            "(%s). Continuing to validation; the console will "
                            "be written under override unless the ambient-"
                            "dark check fails.", override_justification,
                        )
                        _emit_log(
                            "Calibration: override accepted — continuing to "
                            "validation. The console will be written under "
                            "override unless the ambient-dark check fails."
                        )
                    else:
                        _emit_log("Calibration: override declined.")
                if not gate_passed and not override_granted:
                    logger.warning(
                        "Calibration phase 2.5: gate FAIL — below threshold "
                        "on %s. Run FAILED; console EEPROM untouched.", below,
                    )
                    _emit_log(
                        "Calibration: FAILED — scan mean/contrast below "
                        f"threshold on {below}. Nothing was written to "
                        "the console."
                    )
                    # The gate rows are this run's result table (there is
                    # no validation scan to build one from); the CSV is
                    # written so the failure leaves the same evidence trail
                    # as a validation-stage failure.
                    rows = gate_rows
                    csv_path = os.path.join(
                        request.output_dir,
                        f"{request.artifact_prefix}calibration-{ts}.csv",
                    )
                    write_result_csv(
                        csv_path, rows,
                        console_serial=_read_device_serial(
                            getattr(self._interface, "console", None)
                        ),
                        left_sensor_serial=_read_device_serial(
                            getattr(self._interface, "left", None)
                        ),
                        right_sensor_serial=_read_device_serial(
                            getattr(self._interface, "right", None)
                        ),
                    )
                    error = (
                        "calibration scan below threshold on "
                        f"{below}; nothing written"
                    )
                    ok = True          # ran to an honest FAILED verdict
                    return
                logger.info(
                    "Calibration phase 2.5 done: %s — proceeding to validation.",
                    "gate PASS" if gate_passed else "operator override granted",
                )

                # ── Phase 3: apply proposed calibration IN MEMORY ─────────
                # The validation scan must measure BFI/BVI with the new
                # calibration applied, but the console EEPROM must stay
                # untouched until the verdict is in. set_realtime_calibration
                # installs the proposed arrays into the same SDK cache the
                # corrected pipeline reads at scan start; the finally block
                # restores the console's own calibration unless this run
                # ends in the post-validation write.
                _emit_log(
                    "Calibration: applying proposed calibration for "
                    "validation (console not written yet)…"
                )
                logger.info(
                    "Calibration phase 3: proposed calibration applied to "
                    "the in-memory cache only (console EEPROM untouched)."
                )
                prior_cal = self._interface.get_calibration()
                self._interface.scan_workflow.set_realtime_calibration(
                    cal_obj.c_min, cal_obj.c_max,
                    cal_obj.i_min, cal_obj.i_max,
                )
                applied_override = True

                _emit_progress("validation_scan")
                _emit_log("Calibration: starting validation scan…")
                validation_window_frames = int(
                    round(request.validation_duration_sec * CAPTURE_HZ))
                logger.info(
                    "Calibration phase 4: validation scan, "
                    "duration=%d sec (= %d validation + %d delay)",
                    request.validation_duration_sec + request.scan_delay_sec,
                    request.validation_duration_sec, request.scan_delay_sec,
                )
                _reset_firmware_trigger("phase 4 (pre-scan)")
                val_left, val_right, val_samples, val_dark_samples = _run_subscan_capture(
                    self._interface, request,
                    subject_id=f"calib2_{request.operator_id}",
                    duration_sec=request.validation_duration_sec
                    + request.scan_delay_sec,
                    skip_leading_frames=skip_frames,
                    frame_window_count=validation_window_frames,
                    stop_evt=self._stop_evt,
                )
                logger.info(
                    "Calibration phase 4 done: %d corrected samples captured "
                    "live; raw CSVs: left=%s  right=%s",
                    len(val_samples),
                    val_left or "(none)", val_right or "(none)",
                )
                if self._stop_evt.is_set():
                    canceled = True
                    error = "canceled during validation scan"
                    return

                _emit_progress("evaluate")
                _emit_log("Calibration: evaluating…")
                logger.info("Calibration phase 5: aggregating per-camera rows + thresholds.")
                rows = _build_result_rows_from_samples(
                    val_samples,
                    dark_samples=val_dark_samples,
                    left_camera_mask=request.left_camera_mask,
                    right_camera_mask=request.right_camera_mask,
                    thresholds=request.thresholds,
                    sensor_left=getattr(self._interface, "left", None),
                    sensor_right=getattr(self._interface, "right", None),
                )
                csv_path = os.path.join(
                    request.output_dir,
                    f"{request.artifact_prefix}calibration-{ts}.csv",
                )
                write_result_csv(
                    csv_path, rows,
                    console_serial=_read_device_serial(
                        getattr(self._interface, "console", None)
                    ),
                    left_sensor_serial=_read_device_serial(
                        getattr(self._interface, "left", None)
                    ),
                    right_sensor_serial=_read_device_serial(
                        getattr(self._interface, "right", None)
                    ),
                )
                passed = evaluate_passed(rows)
                pass_count = sum(
                    1 for r in rows
                    if r.mean_test == "PASS" and r.contrast_test == "PASS"
                    and r.bfi_test == "PASS" and r.bvi_test == "PASS"
                )
                logger.info(
                    "Calibration result table:\n%s",
                    _format_result_rows_table(rows, request.thresholds),
                )
                logger.info(
                    "Calibration phase 5 done: %d/%d cameras PASS, "
                    "overall=%s. CSV: %s",
                    pass_count, len(rows), "PASS" if passed else "FAIL",
                    csv_path,
                )

                dark_failed = ", ".join(
                    f"{'L' if r.side == 'left' else 'R'}{r.cam_id + 1}"
                    for r in rows if r.dark_test == "FAIL"
                )
                if passed or (override_granted and not dark_failed):
                    # ── Phase 6: every camera cleared every threshold — or
                    # the operator consented at the gate and no camera failed
                    # the ambient-dark check — only now does the console
                    # EEPROM get written. The returned object is the console
                    # read-back (cache refreshed, source="console"), so no
                    # override restore is needed.
                    _emit_progress("write_calibration")
                    if passed:
                        _emit_log(
                            "Calibration: all cameras within limits — writing "
                            "to console…"
                        )
                        logger.info(
                            "Calibration phase 6: writing validated calibration "
                            "to console EEPROM."
                        )
                    else:
                        _emit_log(
                            "Calibration: writing to console UNDER OPERATOR "
                            f"OVERRIDE (below threshold on {below})…"
                        )
                        logger.warning(
                            "Calibration phase 6: writing calibration to "
                            "console EEPROM under operator override (%s).",
                            override_justification,
                        )
                    cal_obj = self._interface.write_calibration(
                        cal_obj.c_min, cal_obj.c_max,
                        cal_obj.i_min, cal_obj.i_max,
                    )
                    wrote_calibration = True
                    logger.info(
                        "Calibration phase 6 done — calibration written and "
                        "cached (source=%s).", cal_obj.source,
                    )
                elif override_granted:
                    # Ambient-dark failure is a data-integrity fault (room
                    # light leaking in), never a dim laser: an override
                    # cannot write past it.
                    error = (
                        f"ambient-dark check failed on {dark_failed}; the "
                        "override cannot write this calibration; nothing "
                        "written"
                    )
                    _emit_log(
                        "Calibration: FAILED — ambient-dark check failed on "
                        f"{dark_failed}. An override cannot write this "
                        "calibration. Nothing was written to the console."
                    )
                    logger.warning(
                        "Calibration: ambient-dark FAIL on %s under override "
                        "— console EEPROM untouched.", dark_failed,
                    )
                else:
                    _emit_log(
                        "Calibration: FAILED — nothing was written to "
                        "the console."
                    )
                    logger.info(
                        "Calibration: validation FAILED — console EEPROM "
                        "untouched."
                    )
                ok = True
            except Exception as e:
                logger.exception("Calibration worker failed.")
                if not error:
                    error = f"{type(e).__name__}: {e}"
            finally:
                wd.cancel()
                # The watchdog is authoritative: if it fired, this run is a
                # timeout regardless of which phase-boundary check (flash /
                # scan) already stamped a generic "canceled ..." message —
                # those messages describe a symptom of the stop_evt the
                # watchdog itself set, not an independent cancel. Only fall
                # back to the earlier finally-block guessing (any
                # unaccounted-for stop_evt is a plain cancel) when the
                # watchdog did not fire.
                if timed_out:
                    canceled = True
                    error = (
                        f"calibration exceeded max_duration_sec="
                        f"{request.max_duration_sec}"
                    )
                elif self._stop_evt.is_set() and not canceled:
                    canceled = True
                    if not error:
                        error = "canceled"
                outcome = _resolve_outcome(
                    ok=ok, passed=passed, canceled=canceled, timed_out=timed_out,
                    overridden=override_granted and wrote_calibration,
                )

                if applied_override and not wrote_calibration:
                    # The proposed calibration only ever lived in the SDK's
                    # in-memory cache (phase 3). Put the console's own
                    # calibration back so later scans — and the next run's
                    # #117 baseline — read what the EEPROM actually holds.
                    # Pure cache operation: nothing was written to the
                    # console, so there is nothing to undo on it.
                    if prior_cal is not None:
                        self._interface.scan_workflow._install_calibration(
                            prior_cal
                        )
                        logger.info(
                            "Calibration: in-memory cache restored to the "
                            "console calibration (source=%s); EEPROM was "
                            "never written.", prior_cal.source,
                        )

                if cal_obj is not None:
                    logger.info(
                        "Calibration: %s:\n%s",
                        "final calibration on console" if wrote_calibration
                        else "proposed calibration (NOT written)",
                        _format_calibration(cal_obj),
                    )

                # Self-describing JSON manifest — emitted unconditionally
                # so failed/canceled runs still leave a record for triage.
                try:
                    json_path = os.path.join(
                        request.output_dir,
                        f"{request.artifact_prefix}calibration-{ts}.json",
                    )
                    write_result_json(
                        json_path,
                        started_timestamp=ts,
                        passed=passed,
                        canceled=canceled,
                        error=error,
                        outcome=outcome.value,
                        request=request,
                        rows=rows,
                        calibration=cal_obj,
                        scan_paths={
                            "calibration_left": cal_left,
                            "calibration_right": cal_right,
                            "validation_left": val_left,
                            "validation_right": val_right,
                        },
                        interface=self._interface,
                        calibration_written=wrote_calibration,
                        override_granted=override_granted,
                        override_justification=override_justification,
                    )
                    logger.info("Calibration manifest written: %s", json_path)
                except Exception:
                    logger.exception("Failed to write calibration JSON manifest.")
                    json_path = ""

                logger.info(
                    "Calibration: procedure complete (ok=%s, passed=%s, "
                    "canceled=%s, error=%r, outcome=%s)",
                    ok, passed, canceled, error, outcome.value,
                )

                result = CalibrationResult(
                    ok=ok, passed=passed, canceled=canceled, error=error,
                    csv_path=csv_path, json_path=json_path,
                    calibration=cal_obj, rows=rows,
                    calibration_scan_left_path=cal_left,
                    calibration_scan_right_path=cal_right,
                    validation_scan_left_path=val_left,
                    validation_scan_right_path=val_right,
                    started_timestamp=ts,
                    outcome=outcome,
                    calibration_written=wrote_calibration,
                    override_granted=override_granted,
                    override_justification=override_justification,
                )
                with self._lock:
                    self._running = False
                if on_complete_fn:
                    try:
                        on_complete_fn(result)
                    except Exception:
                        logger.exception("on_complete_fn raised.")

        self._thread = threading.Thread(
            target=_worker, name="CalibrationWorker", daemon=True,
        )
        self._thread.start()
        return True

    def start_test_scan(
        self,
        request: CalibrationRequest,
        *,
        on_log_fn: Optional[Callable[[str], None]] = None,
        on_progress_fn: Optional[Callable[[str], None]] = None,
        on_complete_fn: Optional[Callable[["TestScanResult"], None]] = None,
    ) -> bool:
        """Run just the calibration scan (CalibrationWorkflow phase 1)
        as a stand-alone diagnostic. No calibration write, no validation
        scan. Returns False if a calibration or test scan is already in
        flight. Forces ``request.average_full_scan = True`` so the Test
        results reflect the same averaging the calibration math would
        use (#132).
        """
        with self._lock:
            if self._running:
                logger.warning("start_test_scan refused: already running.")
                return False
            self._running = True
        self._stop_evt = threading.Event()

        # Test scans always average all laser-on samples — single source
        # of truth so the connector doesn't have to remember to set this.
        request = dataclasses.replace(request, average_full_scan=True)

        def _emit_log(msg: str) -> None:
            logger.info(msg)
            if on_log_fn:
                on_log_fn(msg)

        def _emit_progress(stage: str) -> None:
            if on_progress_fn:
                on_progress_fn(stage)

        def _worker() -> None:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            test_left = test_right = ""
            csv_path = ""
            json_path = ""
            rows: list[CalibrationResultRow] = []
            ok = False
            passed = False
            error = ""
            canceled = False
            timed_out = False

            logger.info(
                "Test scan: starting (operator=%s, output_dir=%s, "
                "masks=(0x%02X, 0x%02X), duration_sec=%d, scan_delay_sec=%d, "
                "max_duration_sec=%d, ts=%s)",
                request.operator_id, request.output_dir,
                request.left_camera_mask, request.right_camera_mask,
                request.duration_sec, request.scan_delay_sec,
                request.max_duration_sec, ts,
            )

            def _watchdog() -> None:
                nonlocal timed_out
                timed_out = True
                self._stop_evt.set()
                logger.warning(
                    "Test-scan watchdog fired after %d sec; aborting.",
                    request.max_duration_sec,
                )
                try:
                    self._interface.scan_workflow.cancel_scan()
                except Exception:
                    logger.exception("Test-scan watchdog: cancel_scan raised.")

            wd = threading.Timer(request.max_duration_sec, _watchdog)
            wd.daemon = True
            wd.start()

            skip_frames = int(round(request.scan_delay_sec * CAPTURE_HZ))
            window_frames = int(round(request.duration_sec * CAPTURE_HZ))
            phase1_window_frames = (
                10 ** 9 if request.average_full_scan else window_frames
            )

            # Inner helpers — duplicate the calibration worker's shape
            # rather than refactor, so this method ships as a single
            # contained change. The two flash/trigger helpers below are
            # textually identical to the calibration worker's; consider
            # extracting later if a third caller appears.
            def _flash_sensors() -> tuple[bool, str]:
                from omotion.ScanWorkflow import ConfigureRequest, ConfigureResult

                cfg_req = ConfigureRequest(
                    left_camera_mask=request.left_camera_mask,
                    right_camera_mask=request.right_camera_mask,
                    power_off_unused_cameras=False,
                )
                evt = threading.Event()
                holder: dict[str, ConfigureResult] = {}

                def _on_done(r: ConfigureResult) -> None:
                    holder["r"] = r
                    evt.set()

                def _on_log(msg: str) -> None:
                    logger.info("Test-scan flash: %s", msg)

                started = self._interface.start_configure_camera_sensors(
                    cfg_req,
                    on_log_fn=_on_log,
                    on_complete_fn=_on_done,
                )
                if not started:
                    return False, (
                        "start_configure_camera_sensors refused "
                        "(another configure already running?)"
                    )

                while not evt.wait(timeout=0.2):
                    if self._stop_evt.is_set():
                        return False, "canceled during flash"
                res = holder.get("r")
                if res is None:
                    return False, "flash completed with no result"
                return bool(res.ok), str(res.error or "")

            def _reset_firmware_trigger(phase_label: str) -> None:
                trigger_cfg = self._interface.resolve_trigger_config(
                    request.trigger_config
                )
                try:
                    self._interface.console.set_trigger_json(data=trigger_cfg)
                    logger.info(
                        "Test scan %s: trigger reset OK "
                        "(firmware fsync_counter=1).", phase_label,
                    )
                except Exception as e:
                    logger.error(
                        "Test scan %s: trigger reset failed: %s. "
                        "Continuing — dark-integrity monitor will catch "
                        "any schedule misalignment.",
                        phase_label, e,
                    )

            try:
                _emit_progress("flash_sensors")
                _emit_log("Test scan: flashing sensors / FPGA…")
                flash_ok, flash_err = _flash_sensors()
                if not flash_ok:
                    error = f"flash phase failed: {flash_err}"
                    canceled = self._stop_evt.is_set()   # was: "canceled" in flash_err
                    return
                if self._stop_evt.is_set():
                    canceled = True
                    error = "canceled after flash"
                    return

                _emit_progress("test_scan")
                _emit_log("Test scan: starting…")
                _reset_firmware_trigger("test (pre-scan)")
                test_left, test_right, test_samples, test_dark_samples = _run_subscan_capture(
                    self._interface, request,
                    subject_id=f"test_{request.operator_id}",
                    duration_sec=request.duration_sec + request.scan_delay_sec,
                    skip_leading_frames=skip_frames,
                    frame_window_count=phase1_window_frames,
                    stop_evt=self._stop_evt,
                )
                logger.info(
                    "Test scan done: %d corrected samples captured live; "
                    "raw CSVs: left=%s  right=%s",
                    len(test_samples),
                    test_left or "(none)", test_right or "(none)",
                )
                if self._stop_evt.is_set():
                    canceled = True
                    error = "canceled during test scan"
                    return

                _emit_progress("evaluate")
                _emit_log("Test scan: evaluating…")
                rows = _build_result_rows_from_samples(
                    test_samples,
                    dark_samples=test_dark_samples,
                    left_camera_mask=request.left_camera_mask,
                    right_camera_mask=request.right_camera_mask,
                    thresholds=request.thresholds,
                    sensor_left=getattr(self._interface, "left", None),
                    sensor_right=getattr(self._interface, "right", None),
                )
                csv_path = os.path.join(
                    request.output_dir, f"test-{ts}.csv"
                )
                write_result_csv(
                    csv_path, rows,
                    console_serial=_read_device_serial(
                        getattr(self._interface, "console", None)
                    ),
                    left_sensor_serial=_read_device_serial(
                        getattr(self._interface, "left", None)
                    ),
                    right_sensor_serial=_read_device_serial(
                        getattr(self._interface, "right", None)
                    ),
                )
                # Test "passed" uses the same gate as calibration but
                # without BFI/BVI participating — Test acceptance is
                # mean + contrast + dark only (see spec R5/R6).
                passed = bool(rows) and all(
                    r.mean_test == "PASS"
                    and r.contrast_test == "PASS"
                    and r.dark_test != "FAIL"
                    for r in rows
                )
                pass_count = sum(
                    1 for r in rows
                    if r.mean_test == "PASS"
                    and r.contrast_test == "PASS"
                    and r.dark_test != "FAIL"
                )
                logger.info(
                    "Test scan result table:\n%s",
                    _format_result_rows_table(rows, request.thresholds),
                )
                logger.info(
                    "Test scan done: %d/%d cameras PASS, overall=%s. CSV: %s",
                    pass_count, len(rows), "PASS" if passed else "FAIL",
                    csv_path,
                )
                ok = True
            except Exception as e:
                logger.exception("Test scan worker failed.")
                if not error:
                    error = f"{type(e).__name__}: {e}"
            finally:
                wd.cancel()
                # See the calibration worker's identical comment: the
                # watchdog is authoritative — if it fired, this run is a
                # timeout regardless of which phase-boundary check already
                # stamped a generic "canceled ..." message.
                if timed_out:
                    canceled = True
                    error = (
                        f"test scan exceeded max_duration_sec="
                        f"{request.max_duration_sec}"
                    )
                elif self._stop_evt.is_set() and not canceled:
                    canceled = True
                    if not error:
                        error = "canceled"
                outcome = _resolve_outcome(
                    ok=ok, passed=passed, canceled=canceled, timed_out=timed_out,
                )

                try:
                    json_path = os.path.join(
                        request.output_dir, f"test-{ts}.json"
                    )
                    write_result_json(
                        json_path,
                        started_timestamp=ts,
                        passed=passed,
                        canceled=canceled,
                        error=error,
                        outcome=outcome.value,
                        request=request,
                        rows=rows,
                        calibration=None,
                        scan_paths={
                            "test_left": test_left,
                            "test_right": test_right,
                        },
                        interface=self._interface,
                        mode="test",
                    )
                    logger.info("Test scan manifest written: %s", json_path)
                except Exception:
                    logger.exception("Failed to write test scan JSON manifest.")
                    json_path = ""

                logger.info(
                    "Test scan: procedure complete (ok=%s, passed=%s, "
                    "canceled=%s, error=%r, outcome=%s)",
                    ok, passed, canceled, error, outcome.value,
                )

                result = TestScanResult(
                    ok=ok, passed=passed, canceled=canceled, error=error,
                    csv_path=csv_path, json_path=json_path,
                    rows=rows,
                    test_scan_left_path=test_left,
                    test_scan_right_path=test_right,
                    started_timestamp=ts,
                    outcome=outcome,
                )
                with self._lock:
                    self._running = False
                if on_complete_fn:
                    try:
                        on_complete_fn(result)
                    except Exception:
                        logger.exception("on_complete_fn raised.")

        self._thread = threading.Thread(
            target=_worker, name="TestScanWorker", daemon=True,
        )
        self._thread.start()
        return True

    def cancel_calibration(self, *, join_timeout: float = 10.0) -> None:
        if not self.running:
            return
        self._stop_evt.set()
        try:
            self._interface.scan_workflow.cancel_scan()
        except Exception:
            logger.warning("cancel_calibration: cancel_scan raised; ignoring.")
        if self._thread is not None:
            self._thread.join(timeout=join_timeout)
