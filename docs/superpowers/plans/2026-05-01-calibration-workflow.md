# CalibrationWorkflow — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an SDK-side `CalibrationWorkflow` that runs two short scans against a connected console, computes `(2, 8)` calibration arrays from scan #1, writes them to the console, runs scan #2 with the new calibration, writes a per-camera CSV with mean/contrast/BFI/BVI plus pass/fail vs config thresholds, returns a `CalibrationResult`, and exposes the procedure through a "Run Calibration" button on the bloodflow app's Settings page with a status indicator.

**Architecture:** New `omotion/CalibrationWorkflow.py` parallel to `ScanWorkflow.py`. Workflow has its own worker thread, stop event, and watchdog timer. Submits two `ScanRequest`s through the existing `ScanWorkflow` (does not talk to USB/UART directly), then aggregates the resulting raw-histogram CSVs through the existing `feed_pipeline_from_csv` / `create_science_pipeline` helpers. SDK constants (`MODULES`, `CAMS_PER_MODULE`, `CAPTURE_HZ`, `CALIBRATION_I_MAX_MULTIPLIER`, `CALIBRATION_DEFAULT_SCAN_DELAY_SEC`, `CALIBRATION_DEFAULT_MAX_DURATION_SEC`) live in `omotion/config.py`.

**Tech Stack:** Python 3.12, numpy, pytest, PyQt6 + QML 6.0 (app side). No new dependencies.

---

## Spec

`docs/superpowers/specs/2026-05-01-calibration-workflow-design.md`

## File Structure

**Create:**
- `openmotion-sdk/omotion/CalibrationWorkflow.py` — request/result/thresholds/row dataclasses, pure compute functions, workflow class. ~700 lines.
- `openmotion-sdk/tests/test_calibration_workflow_compute.py` — pure unit tests for `compute_calibration_from_csvs`, `build_result_rows`, threshold evaluation.
- `openmotion-sdk/tests/test_calibration_workflow.py` — integration tests for the workflow class with mocked `ScanWorkflow.start_scan`.

**Modify:**
- `openmotion-sdk/omotion/config.py` — add SDK-wide calibration constants.
- `openmotion-sdk/omotion/Calibration.py` — replace `_EXPECTED_SHAPE = (2, 8)` with `(MODULES, CAMS_PER_MODULE)` from config.
- `openmotion-sdk/omotion/__init__.py` — export the new dataclasses + workflow.
- `openmotion-sdk/omotion/MotionInterface.py` — add `calibration_workflow` property + `start_calibration` / `cancel_calibration` facade methods.
- `openmotion-bloodflow-app/config/app_config.json` — add four new keys.
- `openmotion-bloodflow-app/motion_connector.py` — read keys, expose Qt properties, add `runCalibration` slot, marshal completion.
- `openmotion-bloodflow-app/pages/Settings.qml` — add the Calibration row.

---

## Task 1: Add SDK-wide constants to `config.py`; refactor `Calibration.py`

**Files:**
- Modify: `openmotion-sdk/omotion/config.py`
- Modify: `openmotion-sdk/omotion/Calibration.py`
- Test: `openmotion-sdk/tests/test_calibration.py` (existing — verify still green)

- [ ] **Step 1: Add constants to `config.py`**

Modify `openmotion-sdk/omotion/config.py`. Find `XO2_FLASH_PAGE_SIZE: int = 16` (around line 201) and insert before the `# Erase mode bitmap` block:

```python
# ---------------------------------------------------------------------------
# Hardware geometry — shared by Calibration, ScanWorkflow, CalibrationWorkflow.
# ---------------------------------------------------------------------------
MODULES: int = 2
"""Number of sensor modules per device (left + right)."""

CAMS_PER_MODULE: int = 8
"""Cameras per sensor module (OV2312 array)."""

CAPTURE_HZ: float = 40.0
"""Histogram capture rate per camera, in Hz."""

# ---------------------------------------------------------------------------
# CalibrationWorkflow defaults.
# ---------------------------------------------------------------------------
CALIBRATION_I_MAX_MULTIPLIER: float = 2.0
"""Multiplier applied to the average light-frame mean to derive I_max."""

CALIBRATION_DEFAULT_SCAN_DELAY_SEC: int = 1
"""Default lead-in skip per sub-scan, in seconds."""

CALIBRATION_DEFAULT_MAX_DURATION_SEC: int = 600
"""Default watchdog timeout for the whole calibration procedure, in seconds."""

```

- [ ] **Step 2: Refactor `Calibration.py` to use the new constants**

Modify `openmotion-sdk/omotion/Calibration.py`. Find the import block and add `from omotion.config import ...`:

```python
from omotion import _log_root
from omotion.config import MODULES, CAMS_PER_MODULE
```

Find:
```python
# Required shape — (modules, cams_per_module).
_EXPECTED_SHAPE = (2, 8)
```

Replace with:
```python
# Required shape — (modules, cams_per_module).
_EXPECTED_SHAPE = (MODULES, CAMS_PER_MODULE)
```

- [ ] **Step 3: Verify all existing calibration tests still pass**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration.py tests/test_calibration_console.py -v`
Expected: 37 passed.

- [ ] **Step 4: Commit**

```bash
git add openmotion-sdk/omotion/config.py openmotion-sdk/omotion/Calibration.py
git commit -m "refactor(sdk): centralize hardware geometry + calibration defaults in config"
```

---

## Task 2: `CalibrationWorkflow.py` skeleton — dataclasses

**Files:**
- Create: `openmotion-sdk/omotion/CalibrationWorkflow.py`
- Create: `openmotion-sdk/tests/test_calibration_workflow_compute.py`

- [ ] **Step 1: Write failing tests for the dataclass shape**

Create `openmotion-sdk/tests/test_calibration_workflow_compute.py`:

```python
"""Unit tests for CalibrationWorkflow pure helpers (no hardware)."""

import numpy as np
import pytest

from omotion.CalibrationWorkflow import (
    CalibrationRequest,
    CalibrationResult,
    CalibrationResultRow,
    CalibrationThresholds,
)


def _thresholds():
    return CalibrationThresholds(
        min_mean_per_camera=[100.0]*8,
        min_contrast_per_camera=[0.2]*8,
        min_bfi_per_camera=[3.0]*8,
        min_bvi_per_camera=[3.0]*8,
    )


def test_request_requires_duration_sec():
    with pytest.raises(TypeError):
        # duration_sec is required, no default.
        CalibrationRequest(
            operator_id="op",
            output_dir="/tmp/x",
            left_camera_mask=0xFF,
            right_camera_mask=0xFF,
            thresholds=_thresholds(),
        )


def test_request_defaults():
    req = CalibrationRequest(
        operator_id="op",
        output_dir="/tmp/x",
        left_camera_mask=0xFF,
        right_camera_mask=0xFF,
        thresholds=_thresholds(),
        duration_sec=5,
    )
    assert req.scan_delay_sec == 1
    assert req.max_duration_sec == 600
    assert req.notes == ""


def test_thresholds_lengths_are_eight():
    t = _thresholds()
    assert len(t.min_mean_per_camera) == 8
    assert len(t.min_contrast_per_camera) == 8
    assert len(t.min_bfi_per_camera) == 8
    assert len(t.min_bvi_per_camera) == 8


def test_result_default_state_is_failed():
    r = CalibrationResult(
        ok=False, passed=False, canceled=False, error="",
        csv_path="", calibration=None, rows=[],
        calibration_scan_left_path="", calibration_scan_right_path="",
        validation_scan_left_path="", validation_scan_right_path="",
        started_timestamp="",
    )
    assert r.ok is False
    assert r.passed is False
```

- [ ] **Step 2: Verify failure**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_workflow_compute.py -v`
Expected: ImportError on `omotion.CalibrationWorkflow`.

- [ ] **Step 3: Create the workflow module skeleton**

Create `openmotion-sdk/omotion/CalibrationWorkflow.py`:

```python
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
import logging
import os
import threading
from dataclasses import dataclass, field
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
    MODULES,
)

if TYPE_CHECKING:
    from omotion.MotionInterface import MotionInterface

logger = logging.getLogger(
    f"{_log_root}.CalibrationWorkflow" if _log_root else "CalibrationWorkflow"
)


@dataclass
class CalibrationThresholds:
    """Per-camera lower bounds (length 8, indexed by cam_id 0..7).
    Applied symmetrically to left and right modules."""
    min_mean_per_camera: list[float]
    min_contrast_per_camera: list[float]
    min_bfi_per_camera: list[float]
    min_bvi_per_camera: list[float]


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
    calibration: Optional[Calibration]
    rows: list[CalibrationResultRow]
    calibration_scan_left_path: str
    calibration_scan_right_path: str
    validation_scan_left_path: str
    validation_scan_right_path: str
    started_timestamp: str
```

- [ ] **Step 4: Run dataclass tests**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_workflow_compute.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add openmotion-sdk/omotion/CalibrationWorkflow.py openmotion-sdk/tests/test_calibration_workflow_compute.py
git commit -m "feat(sdk): CalibrationWorkflow dataclasses (request/result/thresholds/row)"
```

---

## Task 3: `_collect_samples_from_csvs` — pure helper to load Sample lists

**Files:**
- Modify: `openmotion-sdk/omotion/CalibrationWorkflow.py`
- Modify: `openmotion-sdk/tests/test_calibration_workflow_compute.py`
- Test fixtures: reuse `openmotion-sdk/tests/fixtures/scan_owC18EHALL_20251217_160949_left_maskFF.csv` (existing).

This helper takes left/right CSV paths and returns a list of `Sample` objects ordered by absolute frame id. It runs the science pipeline using `Calibration.default()` (the BFI/BVI values will be wrong but the `mean` and `contrast` fields are upstream of the calibration math, so they'll match the values we want for `compute_calibration_from_csvs`).

- [ ] **Step 1: Write failing test**

Append to `openmotion-sdk/tests/test_calibration_workflow_compute.py`:

```python
import os

from omotion.CalibrationWorkflow import _collect_samples_from_csvs


_FIXTURE_DIR = os.path.join(
    os.path.dirname(__file__), "fixtures",
)


def test_collect_samples_loads_left_csv():
    left = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
    if not os.path.exists(left):
        pytest.skip("fixture CSV missing")
    samples = _collect_samples_from_csvs(
        left_csv=left, right_csv=None,
        skip_leading_frames=0,
    )
    assert len(samples) > 0
    assert all(s.side == "left" for s in samples)
    assert all(0 <= s.cam_id < 8 for s in samples)


def test_collect_samples_skips_leading_frames():
    left = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
    if not os.path.exists(left):
        pytest.skip("fixture CSV missing")
    full = _collect_samples_from_csvs(left_csv=left, right_csv=None, skip_leading_frames=0)
    skipped = _collect_samples_from_csvs(left_csv=left, right_csv=None, skip_leading_frames=40)

    full_min_frame = min(s.absolute_frame_id for s in full)
    skipped_min_frame = min(s.absolute_frame_id for s in skipped)
    assert skipped_min_frame >= full_min_frame + 40
```

- [ ] **Step 2: Verify failure**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_workflow_compute.py -v -k collect`
Expected: ImportError on `_collect_samples_from_csvs`.

- [ ] **Step 3: Implement `_collect_samples_from_csvs`**

Append to `openmotion-sdk/omotion/CalibrationWorkflow.py`:

```python
from omotion.MotionProcessing import (
    Sample,
    create_science_pipeline,
    feed_pipeline_from_csv,
)


def _collect_samples_from_csvs(
    *,
    left_csv: Optional[str],
    right_csv: Optional[str],
    skip_leading_frames: int,
    left_camera_mask: int = 0xFF,
    right_camera_mask: int = 0xFF,
    calibration: Optional[Calibration] = None,
) -> list[Sample]:
    """Run the science pipeline against raw histogram CSVs and return all
    light-frame Samples whose absolute_frame_id is at or beyond
    ``skip_leading_frames``.

    Dark frames are skipped by virtue of how create_science_pipeline emits
    samples: ``on_uncorrected_fn`` only fires for non-dark frames.

    Calibration defaults to ``Calibration.default()``; for phase 2 the
    actual values don't matter (only ``mean`` and ``contrast`` are read,
    which are computed upstream of the calibration math). For phase 5 the
    caller passes the freshly-written console calibration so BFI/BVI on
    the returned Samples reflect it.
    """
    cal = calibration or Calibration.default()
    samples: list[Sample] = []

    def _on_sample(s: Sample) -> None:
        if s.absolute_frame_id >= skip_leading_frames:
            samples.append(s)

    pipeline = create_science_pipeline(
        left_camera_mask=left_camera_mask,
        right_camera_mask=right_camera_mask,
        bfi_c_min=cal.c_min,
        bfi_c_max=cal.c_max,
        bfi_i_min=cal.i_min,
        bfi_i_max=cal.i_max,
        on_uncorrected_fn=_on_sample,
    )
    pipeline.start()
    try:
        if left_csv:
            feed_pipeline_from_csv(left_csv, "left", pipeline)
        if right_csv:
            feed_pipeline_from_csv(right_csv, "right", pipeline)
        pipeline.flush()
    finally:
        pipeline.stop()
    samples.sort(key=lambda s: (s.side, s.cam_id, s.absolute_frame_id))
    return samples
```

- [ ] **Step 4: Run tests**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_workflow_compute.py -v -k collect`
Expected: 2 passed.

- [ ] **Step 5: Commit**

```bash
git add openmotion-sdk/omotion/CalibrationWorkflow.py openmotion-sdk/tests/test_calibration_workflow_compute.py
git commit -m "feat(sdk): _collect_samples_from_csvs helper for CalibrationWorkflow"
```

---

## Task 4: `compute_calibration_from_csvs` — phase 2 logic

**Files:**
- Modify: `openmotion-sdk/omotion/CalibrationWorkflow.py`
- Modify: `openmotion-sdk/tests/test_calibration_workflow_compute.py`

- [ ] **Step 1: Write failing tests**

Append to `openmotion-sdk/tests/test_calibration_workflow_compute.py`:

```python
from omotion.CalibrationWorkflow import (
    DegenerateCalibrationError,
    compute_calibration_from_csvs,
)


def test_compute_calibration_against_fixture():
    left = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
    right = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_right_maskFF.csv")
    if not (os.path.exists(left) and os.path.exists(right)):
        pytest.skip("fixture CSVs missing")
    cal = compute_calibration_from_csvs(
        left_csv=left, right_csv=right,
        left_camera_mask=0xFF, right_camera_mask=0xFF,
        skip_leading_frames=0,
    )
    assert cal.c_min.shape == (2, 8)
    assert cal.c_max.shape == (2, 8)
    assert np.all(cal.c_min == 0.0)
    assert np.all(cal.i_min == 0.0)
    # All cameras active and non-degenerate.
    assert np.all(cal.c_max > 0)
    assert np.all(cal.i_max > 0)
    # I_max should be 2x the average light-frame mean — rough sanity check.
    assert cal.source == "console"


def test_compute_calibration_inactive_cameras_use_defaults():
    """Inactive cameras (mask bit clear) should fall back to defaults so
    the (2, 8) array always satisfies monotonicity."""
    left = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
    if not os.path.exists(left):
        pytest.skip("fixture CSV missing")
    cal = compute_calibration_from_csvs(
        left_csv=left, right_csv=None,
        left_camera_mask=0x0F,        # only cams 0..3 active on left
        right_camera_mask=0x00,       # right entirely inactive
        skip_leading_frames=0,
    )
    # Defaults match Calibration.default(); we re-derive them.
    defaults = Calibration.default()
    # Right module: all 8 should match defaults.
    np.testing.assert_array_equal(cal.c_max[1], defaults.c_max[1])
    np.testing.assert_array_equal(cal.i_max[1], defaults.i_max[1])
    # Left module cams 4..7 should match defaults.
    np.testing.assert_array_equal(cal.c_max[0, 4:], defaults.c_max[0, 4:])
    np.testing.assert_array_equal(cal.i_max[0, 4:], defaults.i_max[0, 4:])


def test_compute_calibration_degenerate_active_cam_raises():
    """If an active camera produces zero contrast / zero mean, abort."""
    # Synthesize: write a tiny CSV with cam_id=0, mean=0, contrast=0
    # (See Step 3 implementation for the helper that builds the CSV.)
    pass  # Filled below once the impl & helper exist.
```

- [ ] **Step 2: Verify failure**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_workflow_compute.py -v -k compute_calibration`
Expected: ImportError on `compute_calibration_from_csvs`.

- [ ] **Step 3: Implement `compute_calibration_from_csvs`**

Append to `openmotion-sdk/omotion/CalibrationWorkflow.py`:

```python
class DegenerateCalibrationError(RuntimeError):
    """Raised when an active camera's calibration scan produces unusable
    data (zero / negative aggregates), making BFI/BVI math impossible."""


def _camera_active(mask: int, cam_id: int) -> bool:
    return bool(mask & (1 << cam_id))


def compute_calibration_from_csvs(
    *,
    left_csv: Optional[str],
    right_csv: Optional[str],
    left_camera_mask: int,
    right_camera_mask: int,
    skip_leading_frames: int,
) -> Calibration:
    """Compute (2, 8) C_max and I_max arrays from raw histogram CSVs.

    C_min and I_min are zero. C_max[m, c] is the average light-frame
    contrast for camera (m, c); I_max[m, c] is
    ``CALIBRATION_I_MAX_MULTIPLIER * average light-frame mean``.

    Inactive cameras (mask bit clear) get ``Calibration.default()`` values
    so monotonicity always holds. Active cameras with zero/negative
    aggregates raise :class:`DegenerateCalibrationError`.

    Returns ``Calibration(source="console")`` so callers can pass it
    straight to :meth:`omotion.MotionInterface.write_calibration` (which
    re-wraps anyway).
    """
    samples = _collect_samples_from_csvs(
        left_csv=left_csv, right_csv=right_csv,
        skip_leading_frames=skip_leading_frames,
        left_camera_mask=left_camera_mask,
        right_camera_mask=right_camera_mask,
    )

    defaults = Calibration.default()
    c_max = defaults.c_max.copy()
    i_max = defaults.i_max.copy()
    c_min = np.zeros_like(c_max)
    i_min = np.zeros_like(i_max)

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
                    f"active camera ({side}, cam_id={cam_id}) produced "
                    f"no light-frame samples after skip_leading_frames="
                    f"{skip_leading_frames}; calibration aborted."
                )
            mean_avg = float(np.mean([s.mean for s in cam_samples]))
            contrast_avg = float(np.mean([s.contrast for s in cam_samples]))
            new_c_max = contrast_avg
            new_i_max = CALIBRATION_I_MAX_MULTIPLIER * mean_avg
            if new_c_max <= 0.0 or new_i_max <= 0.0:
                raise DegenerateCalibrationError(
                    f"active camera ({side}, cam_id={cam_id}) produced "
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
```

- [ ] **Step 4: Add the degenerate-cam test**

Replace the `pass  # Filled below ...` placeholder in
`test_compute_calibration_degenerate_active_cam_raises` with:

```python
def test_compute_calibration_degenerate_active_cam_raises(tmp_path):
    """An active camera with no samples → DegenerateCalibrationError."""
    # The fixture has all 8 cams; we synthesise the failure by passing a
    # CSV that exists but excluding cam_id 0 from it. Easiest path:
    # left_camera_mask=0x01 against a non-existent CSV path → no samples
    # → raise.
    bad = str(tmp_path / "nonexistent.csv")
    with pytest.raises(DegenerateCalibrationError):
        compute_calibration_from_csvs(
            left_csv=None, right_csv=None,
            left_camera_mask=0x01,        # cam 0 active
            right_camera_mask=0x00,
            skip_leading_frames=0,
        )
```

- [ ] **Step 5: Run tests**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_workflow_compute.py -v -k compute_calibration`
Expected: 3 passed.

- [ ] **Step 6: Commit**

```bash
git add openmotion-sdk/omotion/CalibrationWorkflow.py openmotion-sdk/tests/test_calibration_workflow_compute.py
git commit -m "feat(sdk): compute_calibration_from_csvs (phase 2 calibration math)"
```

---

## Task 5: `build_result_rows` — phase 5 row aggregation + threshold evaluation

**Files:**
- Modify: `openmotion-sdk/omotion/CalibrationWorkflow.py`
- Modify: `openmotion-sdk/tests/test_calibration_workflow_compute.py`

- [ ] **Step 1: Write failing tests**

Append to `openmotion-sdk/tests/test_calibration_workflow_compute.py`:

```python
from omotion.CalibrationWorkflow import (
    build_result_rows,
    evaluate_passed,
)


def test_build_result_rows_pass_when_all_metrics_above_threshold():
    left = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
    right = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_right_maskFF.csv")
    if not (os.path.exists(left) and os.path.exists(right)):
        pytest.skip("fixture CSVs missing")
    # Very low thresholds so everything passes.
    thr = CalibrationThresholds(
        min_mean_per_camera=[1.0]*8,
        min_contrast_per_camera=[0.0]*8,
        min_bfi_per_camera=[-100.0]*8,
        min_bvi_per_camera=[-100.0]*8,
    )
    rows = build_result_rows(
        left_csv=left, right_csv=right,
        left_camera_mask=0xFF, right_camera_mask=0xFF,
        skip_leading_frames=0,
        thresholds=thr,
        sensor_left=None, sensor_right=None,
        calibration=None,  # uses defaults
    )
    assert len(rows) == 16
    assert all(r.mean_test == "PASS" for r in rows)
    assert all(r.contrast_test == "PASS" for r in rows)
    assert evaluate_passed(rows) is True


def test_build_result_rows_fail_when_mean_below_threshold():
    left = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
    if not os.path.exists(left):
        pytest.skip("fixture CSV missing")
    # Impossible-to-satisfy mean threshold.
    thr = CalibrationThresholds(
        min_mean_per_camera=[1e9]*8,
        min_contrast_per_camera=[0.0]*8,
        min_bfi_per_camera=[-100.0]*8,
        min_bvi_per_camera=[-100.0]*8,
    )
    rows = build_result_rows(
        left_csv=left, right_csv=None,
        left_camera_mask=0xFF, right_camera_mask=0x00,
        skip_leading_frames=0,
        thresholds=thr,
        sensor_left=None, sensor_right=None,
        calibration=None,
    )
    assert len(rows) == 8
    assert all(r.mean_test == "FAIL" for r in rows)
    assert evaluate_passed(rows) is False


def test_build_result_rows_short_threshold_list_treated_as_pass():
    """Threshold list shorter than 8 → missing entries default to PASS.
    Matches existing _log_scan_image_stats semantics."""
    left = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
    if not os.path.exists(left):
        pytest.skip("fixture CSV missing")
    thr = CalibrationThresholds(
        min_mean_per_camera=[1e9, 1e9],   # only first two cams have a real bound
        min_contrast_per_camera=[],
        min_bfi_per_camera=[],
        min_bvi_per_camera=[],
    )
    rows = build_result_rows(
        left_csv=left, right_csv=None,
        left_camera_mask=0xFF, right_camera_mask=0x00,
        skip_leading_frames=0,
        thresholds=thr,
        sensor_left=None, sensor_right=None,
        calibration=None,
    )
    # Cams 0 and 1 fail mean_test (impossible threshold); cams 2..7 PASS.
    rows_by_cam = {r.cam_id: r for r in rows}
    assert rows_by_cam[0].mean_test == "FAIL"
    assert rows_by_cam[1].mean_test == "FAIL"
    for cam in range(2, 8):
        assert rows_by_cam[cam].mean_test == "PASS"


def test_evaluate_passed_empty_rows_returns_false():
    assert evaluate_passed([]) is False
```

- [ ] **Step 2: Verify failure**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_workflow_compute.py -v -k build_result_rows`
Expected: ImportError on `build_result_rows` / `evaluate_passed`.

- [ ] **Step 3: Implement `build_result_rows` and `evaluate_passed`**

Append to `openmotion-sdk/omotion/CalibrationWorkflow.py`:

```python
def _threshold_test(value: float, thresholds: list[float], cam_id: int) -> str:
    """PASS if the threshold list doesn't cover this cam_id, or value
    >= threshold."""
    if cam_id >= len(thresholds):
        return "PASS"
    t = thresholds[cam_id]
    if t is None or not isinstance(t, (int, float)):
        return "PASS"
    return "PASS" if value >= float(t) else "FAIL"


def build_result_rows(
    *,
    left_csv: Optional[str],
    right_csv: Optional[str],
    left_camera_mask: int,
    right_camera_mask: int,
    skip_leading_frames: int,
    thresholds: CalibrationThresholds,
    sensor_left,            # MotionSensor or None — for cached IDs
    sensor_right,           # MotionSensor or None
    calibration: Optional[Calibration],
) -> list[CalibrationResultRow]:
    """Aggregate per-camera mean/contrast/BFI/BVI from validation-scan
    CSVs and apply pass/fail thresholds."""
    samples = _collect_samples_from_csvs(
        left_csv=left_csv, right_csv=right_csv,
        skip_leading_frames=skip_leading_frames,
        left_camera_mask=left_camera_mask,
        right_camera_mask=right_camera_mask,
        calibration=calibration,
    )

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
                bfi_test=_threshold_test(bfi_val, thresholds.min_bfi_per_camera, cam_id),
                bvi_test=_threshold_test(bvi_val, thresholds.min_bvi_per_camera, cam_id),
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
```

- [ ] **Step 4: Run tests**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_workflow_compute.py -v`
Expected: all tests pass (dataclass + collect + compute + build_result_rows).

- [ ] **Step 5: Commit**

```bash
git add openmotion-sdk/omotion/CalibrationWorkflow.py openmotion-sdk/tests/test_calibration_workflow_compute.py
git commit -m "feat(sdk): build_result_rows + evaluate_passed for phase 5"
```

---

## Task 6: `write_result_csv` helper

**Files:**
- Modify: `openmotion-sdk/omotion/CalibrationWorkflow.py`
- Modify: `openmotion-sdk/tests/test_calibration_workflow_compute.py`

- [ ] **Step 1: Write failing test**

Append:

```python
from omotion.CalibrationWorkflow import write_result_csv


def test_write_result_csv_round_trip(tmp_path):
    rows = [
        CalibrationResultRow(
            camera_index=0, side="left", cam_id=0,
            mean=200.0, avg_contrast=0.4, bfi=5.0, bvi=5.5,
            mean_test="PASS", contrast_test="PASS",
            bfi_test="PASS", bvi_test="FAIL",
            security_id="sec-0", hwid="hw-x",
        ),
    ]
    out = tmp_path / "calibration-test.csv"
    write_result_csv(str(out), rows)
    assert out.exists()
    content = out.read_text(encoding="utf-8").splitlines()
    # Header + 1 row.
    assert len(content) == 2
    header = content[0].split(",")
    assert header == [
        "camera_index", "side", "cam_id",
        "mean", "avg_contrast", "bfi", "bvi",
        "mean_test", "contrast_test", "bfi_test", "bvi_test",
        "security_id", "hwid",
    ]
    assert "left" in content[1]
    assert "FAIL" in content[1]
```

- [ ] **Step 2: Verify failure**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_workflow_compute.py -v -k write_result_csv`
Expected: ImportError.

- [ ] **Step 3: Implement**

Append to `openmotion-sdk/omotion/CalibrationWorkflow.py`:

```python
_CSV_FIELDS = [
    "camera_index", "side", "cam_id",
    "mean", "avg_contrast", "bfi", "bvi",
    "mean_test", "contrast_test", "bfi_test", "bvi_test",
    "security_id", "hwid",
]


def write_result_csv(path: str, rows: list[CalibrationResultRow]) -> None:
    """Write CalibrationResultRow list to ``path`` in the canonical
    column order. Creates parent directories if needed."""
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
                "cam_id": r.cam_id,
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
```

- [ ] **Step 4: Run tests**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_workflow_compute.py -v`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add openmotion-sdk/omotion/CalibrationWorkflow.py openmotion-sdk/tests/test_calibration_workflow_compute.py
git commit -m "feat(sdk): write_result_csv helper"
```

---

## Task 7: `CalibrationWorkflow` class — orchestration

**Files:**
- Modify: `openmotion-sdk/omotion/CalibrationWorkflow.py`
- Create: `openmotion-sdk/tests/test_calibration_workflow.py`

- [ ] **Step 1: Write failing tests for the workflow class (mocked ScanWorkflow)**

Create `openmotion-sdk/tests/test_calibration_workflow.py`:

```python
"""Integration tests for CalibrationWorkflow with mocked ScanWorkflow.

Patches MotionInterface.scan_workflow.start_scan to immediately invoke
on_complete_fn with a fake ScanResult pointing at fixture CSVs.
Patches MotionInterface.write_calibration to record arguments without
hitting hardware.
"""
import os
import threading
import time
from unittest.mock import MagicMock

import numpy as np
import pytest

from omotion import MotionInterface
from omotion.Calibration import Calibration
from omotion.CalibrationWorkflow import (
    CalibrationRequest,
    CalibrationResult,
    CalibrationThresholds,
)


_FIXTURE_DIR = os.path.join(os.path.dirname(__file__), "fixtures")
_LEFT  = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
_RIGHT = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_right_maskFF.csv")


def _have_fixtures() -> bool:
    return os.path.exists(_LEFT) and os.path.exists(_RIGHT)


@pytest.fixture
def thresholds():
    return CalibrationThresholds(
        min_mean_per_camera=[1.0]*8,
        min_contrast_per_camera=[0.0]*8,
        min_bfi_per_camera=[-100.0]*8,
        min_bvi_per_camera=[-100.0]*8,
    )


@pytest.fixture
def request_obj(tmp_path, thresholds):
    return CalibrationRequest(
        operator_id="opX",
        output_dir=str(tmp_path),
        left_camera_mask=0xFF,
        right_camera_mask=0xFF,
        thresholds=thresholds,
        duration_sec=2,
        scan_delay_sec=0,
        max_duration_sec=30,
    )


@pytest.fixture
def interface():
    iface = MotionInterface(demo_mode=True)
    return iface


def _make_fake_scan(left, right):
    """Return a function that mimics scan_workflow.start_scan: kicks off
    a thread that invokes on_complete_fn with a ScanResult pointing at
    the given paths."""
    from omotion.ScanWorkflow import ScanResult

    def _fake(req, *, on_complete_fn=None, on_log_fn=None, **kw):
        def _run():
            time.sleep(0.05)
            res = ScanResult(
                ok=True, error="", left_path=left, right_path=right,
                canceled=False, scan_timestamp="20260501_000000",
            )
            if on_complete_fn:
                on_complete_fn(res)
        threading.Thread(target=_run, daemon=True).start()
        return True
    return _fake


def test_happy_path_produces_csv_and_passes(interface, request_obj):
    if not _have_fixtures():
        pytest.skip("fixture CSVs missing")

    interface.scan_workflow.start_scan = _make_fake_scan(_LEFT, _RIGHT)
    interface.write_calibration = MagicMock(
        return_value=Calibration.default()._replace_(source="console")
        if hasattr(Calibration, "_replace_") else Calibration(
            c_min=np.zeros((2, 8)), c_max=np.full((2, 8), 0.5),
            i_min=np.zeros((2, 8)), i_max=np.full((2, 8), 200.0),
            source="console",
        )
    )

    done = threading.Event()
    result_box: list[CalibrationResult] = []
    interface.start_calibration(
        request_obj,
        on_complete_fn=lambda r: (result_box.append(r), done.set()),
    )
    assert done.wait(timeout=20.0), "calibration didn't complete"
    r = result_box[0]
    assert r.ok is True
    assert r.passed is True
    assert r.canceled is False
    assert os.path.exists(r.csv_path)
    assert r.calibration is not None
    interface.write_calibration.assert_called_once()


def test_cancel_during_phase_1(interface, request_obj):
    if not _have_fixtures():
        pytest.skip("fixture CSVs missing")

    started = threading.Event()
    cancel_called = threading.Event()

    def _slow_scan(req, *, on_complete_fn=None, **kw):
        def _run():
            started.set()
            cancel_called.wait(timeout=5.0)
            from omotion.ScanWorkflow import ScanResult
            if on_complete_fn:
                on_complete_fn(ScanResult(
                    ok=False, error="canceled", left_path="", right_path="",
                    canceled=True, scan_timestamp="x",
                ))
        threading.Thread(target=_run, daemon=True).start()
        return True

    interface.scan_workflow.start_scan = _slow_scan
    interface.scan_workflow.cancel_scan = MagicMock(
        side_effect=lambda **kw: cancel_called.set()
    )
    interface.write_calibration = MagicMock()

    done = threading.Event()
    box: list[CalibrationResult] = []
    interface.start_calibration(
        request_obj,
        on_complete_fn=lambda r: (box.append(r), done.set()),
    )
    assert started.wait(timeout=5.0)
    interface.cancel_calibration()
    assert done.wait(timeout=10.0)
    r = box[0]
    assert r.ok is False
    assert r.canceled is True
    assert r.csv_path == ""
    interface.write_calibration.assert_not_called()


def test_phase1_scan_failure_aborts_before_write(interface, request_obj):
    def _fail_scan(req, *, on_complete_fn=None, **kw):
        from omotion.ScanWorkflow import ScanResult
        threading.Thread(
            target=lambda: on_complete_fn(ScanResult(
                ok=False, error="USB lost", left_path="", right_path="",
                canceled=False, scan_timestamp="x",
            )),
            daemon=True,
        ).start()
        return True

    interface.scan_workflow.start_scan = _fail_scan
    interface.write_calibration = MagicMock()

    done = threading.Event()
    box: list[CalibrationResult] = []
    interface.start_calibration(
        request_obj,
        on_complete_fn=lambda r: (box.append(r), done.set()),
    )
    assert done.wait(timeout=5.0)
    r = box[0]
    assert r.ok is False
    assert "USB lost" in r.error
    interface.write_calibration.assert_not_called()
```

(The `Calibration._replace_` line is a no-op safety; the real path
constructs the Calibration directly.)

- [ ] **Step 2: Verify failure**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_workflow.py -v`
Expected: AttributeError on `interface.start_calibration`.

- [ ] **Step 3: Implement the workflow class**

Append to `openmotion-sdk/omotion/CalibrationWorkflow.py`:

```python
from omotion.ScanWorkflow import ScanRequest, ScanResult


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
            rows: list[CalibrationResultRow] = []
            ok = False
            passed = False
            error = ""
            canceled = False

            # Watchdog timer
            def _watchdog() -> None:
                self._stop_evt.set()
                logger.warning(
                    "Calibration watchdog fired after %d sec; aborting.",
                    request.max_duration_sec,
                )
                # Best-effort cancel of any active sub-scan
                try:
                    self._interface.scan_workflow.cancel_scan()
                except Exception:
                    pass
            wd = threading.Timer(request.max_duration_sec, _watchdog)
            wd.daemon = True
            wd.start()

            try:
                _emit_progress("calibration_scan")
                _emit_log("Calibration: starting calibration scan…")
                cal_left, cal_right = _run_subscan(
                    self._interface, request,
                    subject_id=f"calib1_{request.operator_id}",
                    duration_sec=request.duration_sec + request.scan_delay_sec,
                    stop_evt=self._stop_evt,
                )
                if self._stop_evt.is_set():
                    canceled = True
                    error = "canceled during calibration scan"
                    return

                _emit_progress("compute_calibration")
                _emit_log("Calibration: computing arrays…")
                skip_frames = int(round(request.scan_delay_sec * CAPTURE_HZ))
                try:
                    cal_obj = compute_calibration_from_csvs(
                        left_csv=cal_left, right_csv=cal_right,
                        left_camera_mask=request.left_camera_mask,
                        right_camera_mask=request.right_camera_mask,
                        skip_leading_frames=skip_frames,
                    )
                except DegenerateCalibrationError as e:
                    error = str(e)
                    return

                _emit_progress("write_calibration")
                _emit_log("Calibration: writing to console…")
                cal_obj = self._interface.write_calibration(
                    cal_obj.c_min, cal_obj.c_max,
                    cal_obj.i_min, cal_obj.i_max,
                )

                if self._stop_evt.is_set():
                    canceled = True
                    error = "canceled after calibration write"
                    return

                _emit_progress("validation_scan")
                _emit_log("Calibration: starting validation scan…")
                val_left, val_right = _run_subscan(
                    self._interface, request,
                    subject_id=f"calib2_{request.operator_id}",
                    duration_sec=request.duration_sec + request.scan_delay_sec,
                    stop_evt=self._stop_evt,
                )
                if self._stop_evt.is_set():
                    canceled = True
                    error = "canceled during validation scan"
                    return

                _emit_progress("evaluate")
                _emit_log("Calibration: evaluating…")
                rows = build_result_rows(
                    left_csv=val_left, right_csv=val_right,
                    left_camera_mask=request.left_camera_mask,
                    right_camera_mask=request.right_camera_mask,
                    skip_leading_frames=skip_frames,
                    thresholds=request.thresholds,
                    sensor_left=getattr(self._interface, "left", None),
                    sensor_right=getattr(self._interface, "right", None),
                    calibration=cal_obj,
                )
                csv_path = os.path.join(
                    request.output_dir, f"calibration-{ts}.csv"
                )
                write_result_csv(csv_path, rows)
                passed = evaluate_passed(rows)
                ok = True
            except Exception as e:
                logger.exception("Calibration worker failed.")
                error = error or f"{type(e).__name__}: {e}"
            finally:
                wd.cancel()
                if self._stop_evt.is_set() and not canceled:
                    canceled = True
                    if not error:
                        error = (
                            f"calibration exceeded max_duration_sec="
                            f"{request.max_duration_sec}"
                        )

                result = CalibrationResult(
                    ok=ok, passed=passed, canceled=canceled, error=error,
                    csv_path=csv_path, calibration=cal_obj, rows=rows,
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


def _run_subscan(
    interface,
    request: CalibrationRequest,
    *,
    subject_id: str,
    duration_sec: int,
    stop_evt: threading.Event,
) -> tuple[str, str]:
    """Submit a ScanRequest and block until it completes, returning
    (left_path, right_path). Raises RuntimeError on scan failure.
    Honors stop_evt by calling cancel_scan and returning empty paths.
    """
    scan_req = ScanRequest(
        subject_id=subject_id,
        duration_sec=duration_sec,
        left_camera_mask=request.left_camera_mask,
        right_camera_mask=request.right_camera_mask,
        data_dir=request.output_dir,
        disable_laser=False,
        write_raw_csv=True,
        write_corrected_csv=False,
        write_telemetry_csv=False,
        reduced_mode=False,
    )
    evt = threading.Event()
    holder: dict[str, ScanResult] = {}

    def _on_complete(r: ScanResult) -> None:
        holder["r"] = r
        evt.set()

    started = interface.scan_workflow.start_scan(
        scan_req, on_complete_fn=_on_complete,
    )
    if not started:
        raise RuntimeError("ScanWorkflow refused start_scan.")

    while not evt.wait(timeout=0.1):
        if stop_evt.is_set():
            try:
                interface.scan_workflow.cancel_scan()
            except Exception:
                pass
            # Wait one more cycle for the on_complete callback.
            evt.wait(timeout=5.0)
            return "", ""
    res = holder.get("r")
    if res is None or not res.ok:
        raise RuntimeError(
            f"sub-scan failed: {(res.error if res else 'no result')}"
        )
    if res.canceled:
        return "", ""
    return res.left_path or "", res.right_path or ""
```

- [ ] **Step 4: Run integration tests**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_workflow.py -v`
Expected: 3 passed (happy path, cancel, scan failure).

- [ ] **Step 5: Commit**

```bash
git add openmotion-sdk/omotion/CalibrationWorkflow.py openmotion-sdk/tests/test_calibration_workflow.py
git commit -m "feat(sdk): CalibrationWorkflow orchestration with watchdog + cancel"
```

---

## Task 8: `MotionInterface` facade hooks

**Files:**
- Modify: `openmotion-sdk/omotion/MotionInterface.py`
- Modify: `openmotion-sdk/omotion/__init__.py`

- [ ] **Step 1: Add facade methods**

Modify `openmotion-sdk/omotion/MotionInterface.py`. Find the existing
`scan_workflow` property (around lines 244-251):

```python
    @property
    def scan_workflow(self):
        if self._scan_workflow is None:
            from omotion.ScanWorkflow import ScanWorkflow

            self._scan_workflow = ScanWorkflow(self)
        return self._scan_workflow
```

Add right below it:

```python
    @property
    def calibration_workflow(self):
        if self._calibration_workflow is None:
            from omotion.CalibrationWorkflow import CalibrationWorkflow

            self._calibration_workflow = CalibrationWorkflow(self)
        return self._calibration_workflow

    @property
    def calibration_running(self) -> bool:
        return (
            self._calibration_workflow is not None
            and self._calibration_workflow.running
        )

    def start_calibration(self, request, **kwargs) -> bool:
        return self.calibration_workflow.start_calibration(request, **kwargs)

    def cancel_calibration(self, **kwargs) -> None:
        if self._calibration_workflow is not None:
            self._calibration_workflow.cancel_calibration(**kwargs)
```

Find `__init__` (around line 36-65) and add the lazy field next to
`self._scan_workflow = None`:

```python
        self._scan_workflow = None
        self._calibration_workflow = None
```

- [ ] **Step 2: Export from package**

Modify `openmotion-sdk/omotion/__init__.py`. Add after `from .Calibration import Calibration`:

```python
from .CalibrationWorkflow import (
    CalibrationRequest,
    CalibrationResult,
    CalibrationResultRow,
    CalibrationThresholds,
)
```

Find `__all__` and extend:

```python
__all__ = [
    ...,
    "Calibration",
    "CalibrationRequest",
    "CalibrationResult",
    "CalibrationResultRow",
    "CalibrationThresholds",
    ...,
]
```

- [ ] **Step 3: Re-run all calibration tests**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration.py tests/test_calibration_console.py tests/test_calibration_workflow_compute.py tests/test_calibration_workflow.py -v`
Expected: all pass.

- [ ] **Step 4: Smoke-test the import path**

Run: `cd openmotion-sdk && python -c "from omotion import MotionInterface, CalibrationRequest, CalibrationThresholds; print('ok')"`
Expected: `ok`.

- [ ] **Step 5: Commit**

```bash
git add openmotion-sdk/omotion/MotionInterface.py openmotion-sdk/omotion/__init__.py
git commit -m "feat(sdk): MotionInterface.start_calibration / cancel_calibration facade"
```

---

## Task 9: Add app_config keys for calibration

**Files:**
- Modify: `openmotion-bloodflow-app/config/app_config.json`

- [ ] **Step 1: Add the four new keys**

Modify `openmotion-bloodflow-app/config/app_config.json`. Find the
existing `eol_min_contrast_per_camera` array. Insert after its closing
bracket (and the comma):

```json
"eol_min_bfi_per_camera":  [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0],
"eol_min_bvi_per_camera":  [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0],
"max_calibration_time_sec": 600,
"calibration_scan_duration_sec": 5,
"calibration_scan_delay_sec": 1,
```

(The 3.0 placeholder values are conservative starting bounds for BFI/BVI;
adjust to real production thresholds before shipping.)

- [ ] **Step 2: Verify JSON parses**

Run: `cd openmotion-bloodflow-app && python -c "import json; print(json.load(open('config/app_config.json'))['max_calibration_time_sec'])"`
Expected: `600`.

- [ ] **Step 3: Commit**

```bash
git add openmotion-bloodflow-app/config/app_config.json
git commit -m "config: add calibration thresholds + duration/delay/timeout keys"
```

---

## Task 10: Bloodflow connector — read keys, expose properties, add slot

**Files:**
- Modify: `openmotion-bloodflow-app/motion_connector.py`

- [ ] **Step 1: Read the new config keys**

Find the existing config-loading block around `set_eol_thresholds`
(lines 391-406 region). Locate where `eol_min_mean_per_camera` and
`eol_min_contrast_per_camera` are currently loaded from
`app_config.json`. Add reads for:

- `_eol_min_bfi_per_camera`
- `_eol_min_bvi_per_camera`
- `_max_calibration_time_sec` (default `600` if missing)
- `_calibration_scan_duration_sec` (default `5`)
- `_calibration_scan_delay_sec` (default `1`)

Concrete patch — find the existing reads and add the new ones in the
same block:

```python
self._eol_min_bfi_per_camera = cfg.get(
    "eol_min_bfi_per_camera", []) or []
self._eol_min_bvi_per_camera = cfg.get(
    "eol_min_bvi_per_camera", []) or []
self._max_calibration_time_sec = int(cfg.get(
    "max_calibration_time_sec", 600))
self._calibration_scan_duration_sec = int(cfg.get(
    "calibration_scan_duration_sec", 5))
self._calibration_scan_delay_sec = int(cfg.get(
    "calibration_scan_delay_sec", 1))
```

(The exact insertion point depends on how the existing config block is
structured — match the surrounding style.)

- [ ] **Step 2: Add Qt properties + signal**

Near the top of `MotionConnector` class (next to other `pyqtProperty`
declarations), add:

```python
calibrationStateChanged = pyqtSignal()

@pyqtProperty(bool, notify=calibrationStateChanged)
def calibrationRunning(self) -> bool:
    return self._calibration_status == "running"

@pyqtProperty(str, notify=calibrationStateChanged)
def calibrationStatus(self) -> str:
    return self._calibration_status

@pyqtProperty(int, notify=calibrationStateChanged)
def maxCalibrationTimeSec(self) -> int:
    return self._max_calibration_time_sec
```

In `__init__`, initialize:

```python
self._calibration_status = ""   # "", "running", "passed", "failed", "aborted"
```

(Find an existing `pyqtSignal` declaration in the file to confirm the
import path is `from PyQt6.QtCore import pyqtSignal, pyqtProperty,
pyqtSlot`. If not yet imported, add to the existing import line.)

- [ ] **Step 3: Add a `runCalibration` slot**

Next to other `@pyqtSlot()` methods in the connector, add:

```python
@pyqtSlot()
def runCalibration(self):
    """Kick off the SDK calibration procedure. Idempotent if already
    running. Marshals the worker-thread completion back onto the Qt
    event loop via _calibrationCompleteSignal.
    """
    from omotion import (
        CalibrationRequest, CalibrationThresholds,
    )

    if self._calibration_status == "running":
        return

    if not self._consoleConnected:
        self.captureLog.emit("⚠️ Cannot calibrate: console not connected.")
        return

    # Reuse existing camera-mask state for the active sensors.
    left_mask  = int(getattr(self, "_active_left_mask",  0xFF))
    right_mask = int(getattr(self, "_active_right_mask", 0xFF))
    if (left_mask | right_mask) == 0:
        self.captureLog.emit("⚠️ Cannot calibrate: no active cameras.")
        return

    thresholds = CalibrationThresholds(
        min_mean_per_camera=list(self._eol_min_mean_per_camera or [0]*8),
        min_contrast_per_camera=list(self._eol_min_contrast_per_camera or [0]*8),
        min_bfi_per_camera=list(self._eol_min_bfi_per_camera or [0]*8),
        min_bvi_per_camera=list(self._eol_min_bvi_per_camera or [0]*8),
    )
    output_dir = os.path.join(self._output_base, "app-logs", "calibrations")
    os.makedirs(output_dir, exist_ok=True)
    req = CalibrationRequest(
        operator_id="bloodflow-app",
        output_dir=output_dir,
        left_camera_mask=left_mask,
        right_camera_mask=right_mask,
        thresholds=thresholds,
        duration_sec=self._calibration_scan_duration_sec,
        scan_delay_sec=self._calibration_scan_delay_sec,
        max_duration_sec=self._max_calibration_time_sec,
    )

    self._calibration_status = "running"
    self.calibrationStateChanged.emit()
    self.captureLog.emit("Calibration: starting…")

    started = self._interface.start_calibration(
        req,
        on_log_fn=lambda msg: self.captureLog.emit(msg),
        on_complete_fn=self._calibrationCompleteSignal.emit,
    )
    if not started:
        self._calibration_status = ""
        self.calibrationStateChanged.emit()
        self.captureLog.emit("⚠️ Calibration failed to start.")
```

- [ ] **Step 4: Wire the worker-to-main-thread signal**

`on_complete_fn` runs on the SDK worker thread; we need it on the Qt
main thread before mutating Qt state. Add a private signal carrying the
result:

```python
_calibrationCompleteSignal = pyqtSignal(object)
```

In `__init__` (or wherever existing signals are connected), connect it:

```python
self._calibrationCompleteSignal.connect(self._on_calibration_complete)
```

Add the slot:

```python
@pyqtSlot(object)
def _on_calibration_complete(self, result):
    """Runs on the Qt main thread (queued from worker via signal)."""
    if result.canceled:
        self._calibration_status = "aborted"
        self.captureLog.emit(
            f"⚠️ Calibration aborted: {result.error or 'canceled'}"
        )
    elif not result.ok:
        self._calibration_status = "aborted"
        self.captureLog.emit(
            f"⚠️ Calibration aborted: {result.error or 'unknown error'}"
        )
    elif result.passed:
        self._calibration_status = "passed"
        self.captureLog.emit(f"✅ Calibration: PASS  (CSV: {result.csv_path})")
    else:
        self._calibration_status = "failed"
        self.captureLog.emit(f"❌ Calibration: FAIL  (CSV: {result.csv_path})")
    self.calibrationStateChanged.emit()
```

- [ ] **Step 5: Verify the app still imports**

Run: `cd openmotion-bloodflow-app && python -c "import motion_connector; print('ok')"`
Expected: `ok`.

- [ ] **Step 6: Commit**

```bash
git add openmotion-bloodflow-app/motion_connector.py
git commit -m "feat(bloodflow-app): runCalibration slot + Qt properties for status"
```

---

## Task 11: Settings.qml — button + indicator + status text + Timer

**Files:**
- Modify: `openmotion-bloodflow-app/pages/Settings.qml`

- [ ] **Step 1: Add the Calibration row**

Locate a sensible insertion point in `pages/Settings.qml` — alongside
existing device-action rows like fan-control or TEC voltage. Add:

```qml
RowLayout {
    spacing: 12
    Layout.fillWidth: true

    Button {
        id: runCalibrationButton
        text: "Run Calibration"
        enabled: motion.console_connected && !motion.calibrationRunning
        onClicked: motion.runCalibration()
    }

    // Indicator light
    Rectangle {
        id: calibLight
        width: 14
        height: 14
        radius: 7
        border.width: 1
        border.color: "#444"
        color: {
            switch (motion.calibrationStatus) {
            case "running": return "#2196F3"  // blue
            case "passed":  return "#4CAF50"  // green
            case "failed":  return "#F44336"  // red
            case "aborted": return "#FF9800"  // orange
            default:        return "#9E9E9E"  // gray
            }
        }
    }

    // Status text
    Label {
        id: calibStatusLabel
        text: {
            switch (motion.calibrationStatus) {
            case "running":
                return "Calibrating... (" + calibTimer.elapsedSec
                       + "s / " + motion.maxCalibrationTimeSec + "s)"
            case "passed":  return "Calibration Passed"
            case "failed":  return "Calibration Failed"
            case "aborted": return "Calibration Aborted"
            default:        return ""
            }
        }
    }

    // 1 Hz tick driving the elapsed counter while running.
    Timer {
        id: calibTimer
        property int elapsedSec: 0
        interval: 1000
        repeat: true
        running: motion.calibrationRunning
        onTriggered: elapsedSec += 1
    }

    Connections {
        target: motion
        function onCalibrationStateChanged() {
            if (motion.calibrationStatus === "running") {
                calibTimer.elapsedSec = 0
            }
        }
    }

    Item { Layout.fillWidth: true }   // push everything left
}
```

- [ ] **Step 2: Verify the app launches**

Run: `cd openmotion-bloodflow-app && python -c "
from PyQt6.QtCore import QCoreApplication
import main  # import only — don't execute the GUI loop
print('ok')
"` (this is a smoke check; if `main.py` runs the event loop on import,
adapt accordingly — at minimum confirm `python main.py` can be launched
in a developer environment and the Settings page renders without QML
parse errors).

For headless validation use:
```bash
cd openmotion-bloodflow-app
python -c "
from PyQt6.QtQml import QQmlApplicationEngine
from PyQt6.QtWidgets import QApplication
import sys
app = QApplication(sys.argv)
engine = QQmlApplicationEngine()
engine.load('pages/Settings.qml')
print('roots:', len(engine.rootObjects()))
"
```

(QML may need additional context properties to load standalone; if it
fails for missing `motion`, that's expected — manually launch the app
in step 4 of Task 12 to check rendering.)

- [ ] **Step 3: Commit**

```bash
git add openmotion-bloodflow-app/pages/Settings.qml
git commit -m "feat(bloodflow-app): Settings page Run Calibration button + status indicator"
```

---

## Task 12: End-to-end manual smoke test

**Files:** None (manual hardware verification).

Run only with a console + at least one sensor attached.

- [ ] **Step 1: Confirm SDK happy path against hardware**

```bash
cd openmotion-sdk
python -c "
import os, time
from omotion import (
    MotionInterface, CalibrationRequest, CalibrationThresholds,
)

mi = MotionInterface()
mi.start(wait=True, wait_timeout=5.0)
time.sleep(1)

thresholds = CalibrationThresholds(
    min_mean_per_camera=[1.0]*8,
    min_contrast_per_camera=[0.0]*8,
    min_bfi_per_camera=[-100.0]*8,
    min_bvi_per_camera=[-100.0]*8,
)
out = os.path.abspath('./calib-out')
os.makedirs(out, exist_ok=True)
req = CalibrationRequest(
    operator_id='manual',
    output_dir=out,
    left_camera_mask=0xFF, right_camera_mask=0xFF,
    thresholds=thresholds,
    duration_sec=3,
    scan_delay_sec=1,
    max_duration_sec=120,
)

import threading
done = threading.Event()
result_box = []
mi.start_calibration(req, on_complete_fn=lambda r: (result_box.append(r), done.set()))
done.wait(timeout=180)
r = result_box[0]
print('ok=', r.ok, 'passed=', r.passed, 'csv=', r.csv_path)
print('calibration source =', r.calibration.source if r.calibration else None)
print('cache source       =', mi.get_calibration().source)
mi.stop()
"
```

Expected:
- `ok=True`, `passed=True` (assuming hardware nominal; if specific
  cameras are below the relaxed thresholds we set above, that's a real
  hardware issue — investigate before declaring the procedure broken).
- CSV file exists at the printed path with 16 rows (one per camera) and
  the correct header.
- `calibration source = console` and `cache source = console` after the
  procedure.

- [ ] **Step 2: Confirm app-side flow**

Launch the bloodflow app:
```bash
cd openmotion-bloodflow-app
python main.py
```
Navigate to the Settings page. Click **Run Calibration**.
Observe:
- Indicator turns blue.
- Status text reads `"Calibrating... (Ns / 600s)"` with `N`
  incrementing once per second.
- captureLog shows phase progression messages from the SDK.
- After both scans complete, indicator turns green and label reads
  `"Calibration Passed"` (or red / `"Calibration Failed"` if
  thresholds were violated).
- A `calibration-{ts}.csv` file lives at
  `{output_base}/app-logs/calibrations/`.

- [ ] **Step 3: Confirm watchdog**

Set `max_calibration_time_sec` to a low value (e.g. `5`) in
`app_config.json`, restart the app, click Run Calibration, and confirm
the procedure aborts within ~5 seconds with the indicator turning
orange and the label reading `"Calibration Aborted"`. Restore the
default afterward.

---

## Spec Coverage Self-Review

| Spec section | Implementing task |
|---|---|
| `omotion/CalibrationWorkflow.py` dataclasses | Task 2 |
| Constants in `omotion/config.py` | Task 1 |
| Light-frame collection helper | Task 3 |
| `compute_calibration_from_csvs` calibration math | Task 4 |
| Inactive-cam fallback | Task 4 |
| Degenerate-cam abort | Task 4 |
| `build_result_rows` validation aggregation | Task 5 |
| Threshold evaluation (per-cam, fallback to PASS for missing) | Task 5 |
| `evaluate_passed` aggregation | Task 5 |
| `write_result_csv` (canonical column order) | Task 6 |
| `CalibrationWorkflow` orchestration (phases 1-6) | Task 7 |
| Watchdog timeout + cancel semantics | Task 7 |
| Sub-scan submission via `ScanWorkflow.start_scan` | Task 7 |
| `MotionInterface.start_calibration` / `cancel_calibration` / `calibration_workflow` / `calibration_running` | Task 8 |
| Package exports (`CalibrationRequest`, etc.) | Task 8 |
| `app_config.json` new keys | Task 9 |
| Connector reads + Qt properties + `runCalibration` slot | Task 10 |
| Worker-thread → Qt-main-thread marshalling | Task 10 |
| Settings.qml button + indicator + status text + Timer | Task 11 |
| Hardware smoke test | Task 12 |

