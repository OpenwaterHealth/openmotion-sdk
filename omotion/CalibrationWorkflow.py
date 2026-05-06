"""Calibration procedure orchestrator.

Submits two short scans through ScanWorkflow, computes (2, 8)
calibration arrays from scan #1, writes them to the console (which
auto-refreshes the SDK cache), runs scan #2 with the freshly-written
calibration, writes a per-camera CSV with mean/contrast/BFI/BVI plus
pass/fail vs caller-supplied thresholds, and returns a
CalibrationResult.

The workflow does not talk to USB/UART directly. It calls into the
existing ScanWorkflow and processes the raw-histogram CSVs ScanWorkflow
produces.
"""
from __future__ import annotations

import csv
import datetime
import json
import logging
import os
import platform
import socket
import sys
import threading
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


@dataclass
class CalibrationResultRow:
    camera_index: int
    side: str
    cam_id: int
    mean: float
    avg_contrast: float
    bfi: float
    bvi: float
    mean_test: str
    contrast_test: str
    bfi_test: str
    bvi_test: str
    security_id: str
    hwid: str


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


# ---------------------------------------------------------------------------
# Pure compute helpers — no hardware, no UART. Tested in
# tests/test_calibration_workflow_compute.py.
# ---------------------------------------------------------------------------

from omotion.MotionProcessing import (
    CorrectedBatch,
    Sample,
)


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
            # Ratio-of-means contrast, not mean-of-ratios.
            # mean(std/mean) is statistically biased upward whenever
            # per-frame mean varies; mean(std)/mean(mean) matches the
            # live UI's "speckle contrast" and stays bounded by 1 for a
            # well-conditioned signal.
            mean_avg = float(np.mean([s.mean for s in cam_samples]))
            std_avg = float(np.mean([s.std_dev for s in cam_samples]))
            new_c_max = (std_avg / mean_avg) if mean_avg > 0.0 else 0.0
            new_i_max = CALIBRATION_I_MAX_MULTIPLIER * mean_avg
            # The biased per-frame average is logged for diagnosis: a
            # large divergence between the two estimators is the smoking
            # gun for a flaky / partially-occluded camera.
            per_frame_contrast_avg = float(
                np.mean([s.contrast for s in cam_samples])
            )
            logger.info(
                "  cam (%s, cam=%d): n=%d  mean=%.2f  std=%.2f  "
                "C_max(ratio-of-means)=%.4f  C_max(mean-of-ratios)=%.4f  "
                "I_max=%.2f",
                side, cam_id + 1, len(cam_samples),
                mean_avg, std_avg, new_c_max, per_frame_contrast_avg,
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
    left_camera_mask: int,
    right_camera_mask: int,
    thresholds: CalibrationThresholds,
    sensor_left,
    sensor_right,
) -> list[CalibrationResultRow]:
    """Core row aggregation: per-camera mean/contrast/BFI/BVI averages
    and threshold pass/fail. Pure function — caller pre-filters.
    """
    rows: list[CalibrationResultRow] = []
    masks = (left_camera_mask, right_camera_mask)
    sensors = (sensor_left, sensor_right)

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
                mean_test=_threshold_test(mean_val, thresholds.min_mean_per_camera, cam_id),
                contrast_test=_threshold_test(contrast_val, thresholds.min_contrast_per_camera, cam_id),
                bfi_test=bfi_test,
                bvi_test=bvi_test,
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
        for r in rows
    )


_CSV_FIELDS = [
    "camera_index", "side", "cam",
    "mean", "avg_contrast", "bfi", "bvi",
    "mean_test", "contrast_test", "bfi_test", "bvi_test",
    "security_id", "hwid",
]


def write_result_csv(path: str, rows: list[CalibrationResultRow]) -> None:
    """Write CalibrationResultRow list to ``path`` in the canonical
    column order. Creates parent directories if needed.

    The ``cam`` column is 1-indexed (1..8), matching how cameras are
    physically labeled. Internally ``CalibrationResultRow.cam_id`` is
    still 0-indexed (so it can be used to lookup into the per-camera
    threshold arrays).
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
                "mean_test": r.mean_test,
                "contrast_test": r.contrast_test,
                "bfi_test": r.bfi_test,
                "bvi_test": r.bvi_test,
                "security_id": r.security_id,
                "hwid": r.hwid,
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
        return {"hwid": "", "firmware_version": ""}
    return {
        "hwid": str(_safe_call(console.get_hardware_id, "") or ""),
        "firmware_version": str(_safe_call(console.get_version, "") or ""),
    }


def _collect_sensor_info(sensor, camera_mask: int) -> dict:
    if sensor is None:
        return {
            "connected": False,
            "hwid": "",
            "firmware_version": "",
            "camera_mask": f"0x{camera_mask:02X}",
        }
    hwid = _safe_call(sensor.get_cached_hardware_id, "") or _safe_call(sensor.get_hardware_id, "")
    return {
        "connected": True,
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
) -> None:
    """Write a self-describing JSON manifest of the calibration run.

    Includes the per-camera result table, the calibration arrays that
    were written to the console, the camera/sensor/console identities
    (security UIDs, HWIDs, firmware versions), and host info — so the
    file is enough on its own to trace a run back to the exact hardware
    + firmware that produced it.
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
        "started_timestamp": started_timestamp,
        "started_iso": started_iso,
        "passed": passed,
        "canceled": canceled,
        "error": error,
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
    threshold(s), and PASS/FAIL for mean / contrast / BFI / BVI."""
    def _t(arr: Optional[list[float]], i: int, fmt: str = "{:>7.3f}") -> str:
        if arr is None or i >= len(arr):
            return "   —   "
        v = arr[i]
        if not isinstance(v, (int, float)):
            return "   —   "
        return fmt.format(float(v))

    pf = lambda s: "P" if s == "PASS" else "F"

    header = (
        "| cam | side  |   mean   |   min    | M |   contrast |    min   | C |    bfi   |    min   |    max   | B |    bvi   |    min   |    max   | V |"
    )
    sep = (
        "|-----|-------|----------|----------|---|------------|----------|---|----------|----------|----------|---|----------|----------|----------|---|"
    )
    lines = [header, sep]
    for r in rows:
        cam_id = r.cam_id  # 0..7 internally; display 1..8
        lines.append(
            "| {cam:>3d} | {side:<5} | {mean:>8.3f} | {mean_min} | {mean_pf} "
            "| {contrast:>10.5f} | {contrast_min} | {contrast_pf} "
            "| {bfi:>+8.3f} | {bfi_min} | {bfi_max} | {bfi_pf} "
            "| {bvi:>+8.3f} | {bvi_min} | {bvi_max} | {bvi_pf} |"
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
            )
        )
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Orchestration class
# ---------------------------------------------------------------------------

from omotion.ScanWorkflow import ScanRequest, ScanResult


def _run_subscan_capture(
    interface,
    request: CalibrationRequest,
    *,
    subject_id: str,
    duration_sec: int,
    skip_leading_frames: int,
    frame_window_count: int,
    stop_evt: threading.Event,
) -> tuple[str, str, list[Sample]]:
    """Submit a ScanRequest and capture corrected samples in-memory as
    the science pipeline emits them.

    The scan still writes its raw histogram CSV to disk (`write_raw_csv=True`)
    so operators retain the artifact for later verification, but we
    don't re-parse it — corrected samples are captured live via
    ``on_corrected_batch_fn``. This avoids running the science pipeline
    twice on the same data.

    Returns ``(left_path, right_path, captured_samples)``. Raises
    ``RuntimeError`` on scan failure. Honors ``stop_evt`` by calling
    ``cancel_scan`` and returning empty paths + empty list.
    """
    scan_req = ScanRequest(
        subject_id=subject_id,
        duration_sec=duration_sec,
        left_camera_mask=request.left_camera_mask,
        right_camera_mask=request.right_camera_mask,
        data_dir=request.output_dir,
        disable_laser=False,
        write_raw_csv=True,         # keep the raw artifact for audit
        write_corrected_csv=False,
        write_telemetry_csv=False,
        reduced_mode=False,
    )

    upper_bound = skip_leading_frames + int(frame_window_count)
    captured: list[Sample] = []

    def _on_corrected_batch(batch: CorrectedBatch) -> None:
        for s in batch.samples:
            if s.absolute_frame_id < skip_leading_frames:
                continue
            if s.absolute_frame_id >= upper_bound:
                continue
            captured.append(s)

    evt = threading.Event()
    holder: dict[str, ScanResult] = {}

    def _on_complete(r: ScanResult) -> None:
        holder["r"] = r
        evt.set()

    started = interface.scan_workflow.start_scan(
        scan_req,
        on_corrected_batch_fn=_on_corrected_batch,
        on_complete_fn=_on_complete,
        log_dark_endpoints=True,
    )
    if not started:
        raise RuntimeError("ScanWorkflow refused start_scan.")

    while not evt.wait(timeout=0.1):
        if stop_evt.is_set():
            try:
                interface.scan_workflow.cancel_scan()
            except Exception:
                pass
            evt.wait(timeout=5.0)
            return "", "", []
    res = holder.get("r")
    if res is None or not res.ok:
        raise RuntimeError(
            f"sub-scan failed: {(res.error if res else 'no result')}"
        )
    if res.canceled:
        return "", "", []
    # Loud failure: if the science pipeline detected dark-frame
    # schedule/measurement disagreement, abort the calibration. The
    # interpolated dark baseline would be polluted and the resulting
    # C_max/I_max would be wrong.
    if res.dark_integrity_warnings:
        details = " | ".join(res.dark_integrity_warnings[:3])
        if len(res.dark_integrity_warnings) > 3:
            details += f" (+{len(res.dark_integrity_warnings) - 3} more)"
        raise RuntimeError(
            f"sub-scan dark integrity check failed ({len(res.dark_integrity_warnings)} "
            f"warnings): {details}"
        )
    captured.sort(key=lambda s: (s.side, s.cam_id, s.absolute_frame_id))
    return res.left_path or "", res.right_path or "", captured


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
    ) -> bool:
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
                self._stop_evt.set()
                logger.warning(
                    "Calibration watchdog fired after %d sec; aborting.",
                    request.max_duration_sec,
                )
                try:
                    self._interface.scan_workflow.cancel_scan()
                except Exception:
                    pass
            wd = threading.Timer(request.max_duration_sec, _watchdog)
            wd.daemon = True
            wd.start()

            skip_frames = int(round(request.scan_delay_sec * CAPTURE_HZ))
            # Bound the trailing edge to keep the firmware's terminal
            # dark frame (and any laser ramp-down) out of the average.
            window_frames = int(round(request.duration_sec * CAPTURE_HZ))

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
                    if "canceled" in flash_err:
                        canceled = True
                    return
                logger.info("Calibration phase 0 done: sensors flashed.")
                if self._stop_evt.is_set():
                    canceled = True
                    error = "canceled after flash"
                    return

                _emit_progress("calibration_scan")
                _emit_log("Calibration: starting calibration scan…")
                logger.info(
                    "Calibration phase 1: calibration scan, "
                    "duration=%d sec (= %d duration + %d delay)",
                    request.duration_sec + request.scan_delay_sec,
                    request.duration_sec, request.scan_delay_sec,
                )
                _reset_firmware_trigger("phase 1 (pre-scan)")
                cal_left, cal_right, cal_samples = _run_subscan_capture(
                    self._interface, request,
                    subject_id=f"calib1_{request.operator_id}",
                    duration_sec=request.duration_sec + request.scan_delay_sec,
                    skip_leading_frames=skip_frames,
                    frame_window_count=window_frames,
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
                # Issue #117: pass the currently-cached calibration as
                # ``baseline`` so inactive cameras (those excluded by a
                # left-only / right-only mask) keep their on-device
                # values instead of falling back to SDK defaults at
                # write time. The cache is refreshed after every
                # write_calibration, so it reflects what's actually on
                # the console EEPROM.
                try:
                    cal_obj = _compute_calibration_from_samples(
                        cal_samples,
                        left_camera_mask=request.left_camera_mask,
                        right_camera_mask=request.right_camera_mask,
                        baseline=self._interface.get_calibration(),
                    )
                except DegenerateCalibrationError as e:
                    error = str(e)
                    return
                logger.info(
                    "Calibration phase 2 done — proposed calibration:\n%s",
                    _format_calibration(cal_obj),
                )

                _emit_progress("write_calibration")
                _emit_log("Calibration: writing to console…")
                logger.info("Calibration phase 3: writing to console EEPROM.")
                cal_obj = self._interface.write_calibration(
                    cal_obj.c_min, cal_obj.c_max,
                    cal_obj.i_min, cal_obj.i_max,
                )
                logger.info(
                    "Calibration phase 3 done — calibration written and "
                    "cached (source=%s).", cal_obj.source,
                )

                if self._stop_evt.is_set():
                    canceled = True
                    error = "canceled after calibration write"
                    return

                _emit_progress("validation_scan")
                _emit_log("Calibration: starting validation scan…")
                logger.info(
                    "Calibration phase 4: validation scan, "
                    "duration=%d sec (= %d duration + %d delay)",
                    request.duration_sec + request.scan_delay_sec,
                    request.duration_sec, request.scan_delay_sec,
                )
                _reset_firmware_trigger("phase 4 (pre-scan)")
                val_left, val_right, val_samples = _run_subscan_capture(
                    self._interface, request,
                    subject_id=f"calib2_{request.operator_id}",
                    duration_sec=request.duration_sec + request.scan_delay_sec,
                    skip_leading_frames=skip_frames,
                    frame_window_count=window_frames,
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
                    left_camera_mask=request.left_camera_mask,
                    right_camera_mask=request.right_camera_mask,
                    thresholds=request.thresholds,
                    sensor_left=getattr(self._interface, "left", None),
                    sensor_right=getattr(self._interface, "right", None),
                )
                csv_path = os.path.join(
                    request.output_dir, f"calibration-{ts}.csv"
                )
                write_result_csv(csv_path, rows)
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
                ok = True
            except Exception as e:
                logger.exception("Calibration worker failed.")
                if not error:
                    error = f"{type(e).__name__}: {e}"
            finally:
                wd.cancel()
                if self._stop_evt.is_set() and not canceled:
                    canceled = True
                    if not error:
                        error = (
                            f"calibration exceeded max_duration_sec="
                            f"{request.max_duration_sec}"
                        )

                if cal_obj is not None:
                    logger.info(
                        "Calibration: final calibration on console:\n%s",
                        _format_calibration(cal_obj),
                    )

                # Self-describing JSON manifest — emitted unconditionally
                # so failed/canceled runs still leave a record for triage.
                try:
                    json_path = os.path.join(
                        request.output_dir, f"calibration-{ts}.json"
                    )
                    write_result_json(
                        json_path,
                        started_timestamp=ts,
                        passed=passed,
                        canceled=canceled,
                        error=error,
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
                    )
                    logger.info("Calibration manifest written: %s", json_path)
                except Exception:
                    logger.exception("Failed to write calibration JSON manifest.")
                    json_path = ""

                logger.info(
                    "Calibration: procedure complete (ok=%s, passed=%s, "
                    "canceled=%s, error=%r)",
                    ok, passed, canceled, error,
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
