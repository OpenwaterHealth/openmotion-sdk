# EFT Timestamp Repair Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Detect EMI-induced timestamp corruption in the omotion science pipeline, repair the time axis via re-anchoring interpolation, NaN-fill missing frames, and log corrections — without changing raw CSV output.

**Architecture:** A new `TimestampRepairStage` sits between `FrameClassificationStage` and the science chain. It detects divergent frames via timestamp deviation (2ms) and in-packet frame_id disagreement, buffers bad runs, interpolates corrected timestamps between good anchors, and inserts synthetic NaN-fill rows for missing frames. A `quality` field threads through the entire pipeline to the corrected CSV/DB.

**Tech Stack:** Python 3.12+, NumPy, pytest. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-06-05-eft-timestamp-repair-design.md`

**Branch:** `feature/eft-timestamp-repair` (already created off `next`)

**Test data:**
- Clean scans: `C:\Users\ethan\Projects\eft-testing\scans\` (owYWB8TN, owYZ7T66 13:04)
- Degraded scans: `C:\Users\ethan\Projects\eft-testing\scans\` (owM8T7HS, owSPZMD1, owF0JJO2 ×2, owYZ7T66 13:05)
- Severe degraded: `C:\Users\ethan\Projects\eft-testing\final_tests-20260603T234709Z-3-001\final_tests\` (owEFTTEST1 ×3)

---

## File Map

| File | Action | Responsibility |
|------|--------|---------------|
| `omotion/pipeline/stages/timestamp_repair.py` | **Create** | TimestampRepairStage: detection, buffer, interpolation, NaN-fill, quality flags, logging |
| `omotion/pipeline/batch.py` | Modify | Add `quality: Optional[np.ndarray]` field to FrameBatch |
| `omotion/pipeline/stages/dark.py` | Modify | Add `quality: str` to `_LightSample`, `CorrectedFrame`, `EnrichedCorrectedFrame`; thread through |
| `omotion/pipeline/sinks.py` | Modify | Add `quality` column to corrected CSV headers + row writing |
| `omotion/pipeline/factory.py` | Modify | Reorder: Tee("raw") before TimestampRepairStage; insert new stage |
| `omotion/ScanDatabase.py` | Modify | Add `quality` column to `session_data` table |
| `tests/test_pipeline/test_timestamp_repair_stage.py` | **Create** | Unit tests for TimestampRepairStage |
| `tests/test_pipeline/test_eft_regression.py` | **Create** | Integration: clean-scan golden baseline regression |
| `tests/test_pipeline/test_eft_correction.py` | **Create** | Integration: degraded-scan structural assertions |
| `data-processing/check_csv.py` | Modify | Add post-repair invariant checks |

---

### Task 1: Add `quality` field to FrameBatch

**Files:**
- Modify: `omotion/pipeline/batch.py:120-273` (FrameBatch dataclass)
- Test: `tests/test_pipeline/test_batch.py`

- [ ] **Step 1: Write the failing test**

In `tests/test_pipeline/test_batch.py`, add:

```python
def test_framebatch_has_quality_field():
    """FrameBatch must expose an optional quality array (spec §5.1)."""
    import numpy as np
    from omotion.pipeline.batch import FrameBatch

    batch = FrameBatch(
        cam_ids=np.array([0], dtype=np.int8),
        frame_ids=np.array([1], dtype=np.uint8),
        raw_histograms=np.zeros((1, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((1, 2, 8), dtype=np.float32),
        timestamp_s=np.array([0.0], dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )
    assert batch.quality is None

    batch.quality = np.array(["ok"], dtype="<U14")
    assert batch.quality[0] == "ok"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_batch.py::test_framebatch_has_quality_field -v`
Expected: FAIL — `FrameBatch.__init__() got an unexpected keyword argument` or `has no attribute 'quality'`

- [ ] **Step 3: Add the quality field to FrameBatch**

In `omotion/pipeline/batch.py`, add after the `bvi_live` field (around line 263, before the `events` field):

```python
    # ── TimestampRepairStage output ──────────────────────────────────────

    # (N,) str — per-frame quality flag. Set by TimestampRepairStage.
    # "ok" = device timestamp passed through unchanged
    # "ts_corrected" = timestamp replaced by re-anchoring interpolation
    # "nan_filled" = synthetic row for a missing frame (zero histogram)
    quality:        Optional[np.ndarray] = None
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_pipeline/test_batch.py::test_framebatch_has_quality_field -v`
Expected: PASS

- [ ] **Step 5: Commit**

```
git add omotion/pipeline/batch.py tests/test_pipeline/test_batch.py
git commit -m "feat(pipeline): add quality field to FrameBatch"
```

---

### Task 2: Add `quality` to dark.py data types

**Files:**
- Modify: `omotion/pipeline/stages/dark.py:144-224` (_LightSample, CorrectedFrame, EnrichedCorrectedFrame)
- Modify: `omotion/pipeline/stages/bfi_bvi.py:98` (EnrichedCorrectedFrame constructor)
- Modify: `omotion/pipeline/stages/dark_frame_hold.py:167` (EnrichedCorrectedFrame constructor)
- Test: `tests/test_pipeline/test_dark_correction_stage.py`

- [ ] **Step 1: Write the failing test**

In `tests/test_pipeline/test_dark_correction_stage.py`, add:

```python
def test_corrected_frame_has_quality_field():
    from omotion.pipeline.stages.dark import (
        CorrectedFrame, EnrichedCorrectedFrame, _LightSample,
    )
    lf = _LightSample(abs_frame_id=10, t=0.25, u1=100.0, u2=10100.0, quality="ts_corrected")
    assert lf.quality == "ts_corrected"

    cf = CorrectedFrame(
        abs_frame_id=10, t=0.25, side="left", cam_id=0,
        mean=36.0, std=5.0, raw_u1=100.0, raw_var=25.0, dark_var=1.0,
        quality="ts_corrected",
    )
    assert cf.quality == "ts_corrected"

    ef = EnrichedCorrectedFrame(
        abs_frame_id=10, t=0.25, side="left", cam_id=0,
        mean=36.0, std=4.8, contrast=0.13, bfi=5.0, bvi=5.0,
        quality="nan_filled",
    )
    assert ef.quality == "nan_filled"


def test_corrected_frame_quality_defaults_to_ok():
    from omotion.pipeline.stages.dark import CorrectedFrame, EnrichedCorrectedFrame
    cf = CorrectedFrame(
        abs_frame_id=10, t=0.25, side="left", cam_id=0,
        mean=36.0, std=5.0, raw_u1=100.0, raw_var=25.0, dark_var=1.0,
    )
    assert cf.quality == "ok"
    ef = EnrichedCorrectedFrame(
        abs_frame_id=10, t=0.25, side="left", cam_id=0,
        mean=36.0, std=4.8, contrast=0.13, bfi=5.0, bvi=5.0,
    )
    assert ef.quality == "ok"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_pipeline/test_dark_correction_stage.py::test_corrected_frame_has_quality_field tests/test_pipeline/test_dark_correction_stage.py::test_corrected_frame_quality_defaults_to_ok -v`
Expected: FAIL — `unexpected keyword argument 'quality'`

- [ ] **Step 3: Add quality field to all three dataclasses**

In `omotion/pipeline/stages/dark.py`:

`_LightSample` (line ~144):
```python
@dataclass
class _LightSample:
    abs_frame_id: int
    t: float
    u1: float
    u2: float
    quality: str = "ok"
```

`CorrectedFrame` (line ~174):
```python
@dataclass
class CorrectedFrame:
    abs_frame_id: int
    t:             float
    side:          str
    cam_id:        int
    mean:          float
    std:           float
    raw_u1:        float
    raw_var:       float
    dark_var:      float
    contrast:      Optional[float] = None
    quality:       str = "ok"
```

`EnrichedCorrectedFrame` (line ~204):
```python
@dataclass
class EnrichedCorrectedFrame:
    abs_frame_id: int
    t:        float
    side:     str
    cam_id:   int
    mean:     float
    std:      float
    contrast: float
    bfi:      float
    bvi:      float
    quality:  str = "ok"
```

- [ ] **Step 4: Thread quality through the correction pipeline**

In `DarkCorrectionStage.process()` (line ~463, the `else: # light` branch), change `pi.add_light` to pass quality:

```python
                    pi.add_light(
                        abs_frame_id=abs_id, t=t, u1=u1, u2=u2,
                        quality=str(batch.quality[i]) if batch.quality is not None else "ok",
                    )
```

In `PendingInterval.add_light` (line ~239):
```python
    def add_light(self, *, abs_frame_id: int, t: float, u1: float, u2: float,
                  quality: str = "ok") -> None:
        self._light.append(_LightSample(
            abs_frame_id=int(abs_frame_id), t=float(t), u1=float(u1), u2=float(u2),
            quality=quality,
        ))
```

In `LinearInterpolation.correct_interval` (line ~304), pass quality through to CorrectedFrame:

```python
            corrected_frames.append(CorrectedFrame(
                abs_frame_id=lf.abs_frame_id, t=lf.t,
                side=side, cam_id=cam_id,
                mean=mean, std=std,
                raw_u1=lf.u1, raw_var=raw_var, dark_var=baseline_var,
                quality=lf.quality,
            ))
```

The two places that construct `EnrichedCorrectedFrame` also need `quality=`:

In `omotion/pipeline/stages/bfi_bvi.py` (line ~98), add `quality=f.quality`:

```python
                enriched_frames.append(EnrichedCorrectedFrame(
                    abs_frame_id=f.abs_frame_id, t=f.t,
                    side=f.side, cam_id=f.cam_id,
                    mean=float(f.mean), std=float(f.std),
                    contrast=float(contrast),
                    bfi=float(bfi), bvi=float(bvi),
                    quality=f.quality,
                ))
```

In `omotion/pipeline/stages/dark_frame_hold.py` (line ~167), add `quality="ok"` (dark frame stencil — real frame, not corrected):

```python
        return EnrichedCorrectedFrame(
            abs_frame_id=d_prev_abs,
            t=d_prev_t,
            side=side,
            cam_id=cam_id,
            mean=_interp("mean"),
            std=_interp("std"),
            contrast=_interp("contrast"),
            bfi=_interp("bfi"),
            bvi=_interp("bvi"),
            quality="ok",
        )
```

- [ ] **Step 5: Run full dark correction test suite**

Run: `pytest tests/test_pipeline/test_dark_correction_stage.py -v`
Expected: ALL PASS

- [ ] **Step 6: Commit**

```
git add omotion/pipeline/stages/dark.py tests/test_pipeline/test_dark_correction_stage.py
git commit -m "feat(pipeline): add quality field to dark correction data types"
```

---

### Task 3: TimestampRepairStage — core implementation

**Files:**
- Create: `omotion/pipeline/stages/timestamp_repair.py`
- Create: `tests/test_pipeline/test_timestamp_repair_stage.py`

This is the largest task. It implements the full stage in TDD steps.

- [ ] **Step 1: Write test for clean passthrough (no correction needed)**

Create `tests/test_pipeline/test_timestamp_repair_stage.py`:

```python
"""TimestampRepairStage — divergence detection, re-anchoring, NaN-fill."""

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.timestamp_repair import TimestampRepairStage


def _make_batch(cam_ids, frame_ids, side_ids, timestamps, abs_frame_ids=None,
                frame_types=None):
    """Build a minimal FrameBatch for testing the repair stage."""
    n = len(cam_ids)
    batch = FrameBatch(
        cam_ids=np.array(cam_ids, dtype=np.int8),
        frame_ids=np.array(frame_ids, dtype=np.uint8),
        side_ids=np.array(side_ids, dtype=np.int8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.array(timestamps, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )
    if abs_frame_ids is not None:
        batch.abs_frame_ids = np.array(abs_frame_ids, dtype=np.int64)
    if frame_types is not None:
        batch.frame_type = np.array(frame_types, dtype="<U14")
    return batch


def test_clean_passthrough():
    """Clean frames: timestamps unchanged, all quality='ok', batch size unchanged."""
    stage = TimestampRepairStage(tolerance_s=0.002, max_buffer_frames=16)
    # 4 clean frames from cam 0, side 0, 25ms apart
    ts = [0.025, 0.050, 0.075, 0.100]
    batch = _make_batch(
        cam_ids=[0, 0, 0, 0],
        frame_ids=[11, 12, 13, 14],
        side_ids=[0, 0, 0, 0],
        timestamps=ts,
        abs_frame_ids=[11, 12, 13, 14],
        frame_types=["light", "light", "light", "light"],
    )
    result = stage.process(batch)
    np.testing.assert_allclose(result.timestamp_s, ts, atol=1e-9)
    np.testing.assert_array_equal(result.quality, ["ok", "ok", "ok", "ok"])
    assert len(result.cam_ids) == 4
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_timestamp_repair_stage.py::test_clean_passthrough -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'omotion.pipeline.stages.timestamp_repair'`

- [ ] **Step 3: Create the stage with clean passthrough logic**

Create `omotion/pipeline/stages/timestamp_repair.py`:

```python
"""TimestampRepairStage — EMI timestamp correction + NaN-fill.

Detects EMI-induced timestamp misalignment via two conditions:
  1. Timestamp deviation: |actual_Δt - expected_Δt| > tolerance
  2. In-packet frame_id disagreement: cameras at the same timestamp
     report different frame_ids

Repairs bad runs by linear interpolation re-anchored between the last
and next good device timestamps. Inserts synthetic NaN-fill rows for
missing abs_frame_id gaps. Logs one WARNING per misalignment window
plus an end-of-scan summary.

See docs/superpowers/specs/2026-06-05-eft-timestamp-repair-design.md.
"""

from __future__ import annotations

import logging
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..batch import FrameBatch


logger = logging.getLogger("openmotion.sdk.pipeline.stages.timestamp_repair")

_INITIAL_NOMINAL_PERIOD_S = 0.025  # 25 ms (40 Hz)
_EMA_ALPHA = 0.01  # slow convergence for nominal period


@dataclass
class _BufferedFrame:
    """One frame held in the look-ahead buffer during a bad run."""
    idx: int
    cam_id: int
    side_idx: int
    abs_frame_id: int
    frame_id: int
    original_ts: float
    frame_type: str


@dataclass
class _WindowStats:
    """Tracks one misalignment window for coalesced logging."""
    onset_fid: int
    onset_t: float
    end_fid: int = 0
    end_t: float = 0.0
    n_corrected: int = 0
    n_nan: int = 0


class TimestampRepairStage:
    """Detect and correct EMI-induced timestamp corruption.

    Placed after FrameClassificationStage, before the science chain.
    Rewrites batch.timestamp_s in place for corrected frames. Sets
    batch.quality for every frame. May expand the batch by inserting
    synthetic NaN-fill rows for missing abs_frame_id gaps.
    """

    name = "timestamp_repair"

    def __init__(self, *, tolerance_s: float = 0.002,
                 max_buffer_frames: int = 16):
        self._tolerance = float(tolerance_s)
        self._max_buffer = int(max_buffer_frames)
        self._reset_state()

    def _reset_state(self) -> None:
        self._nominal_period = _INITIAL_NOMINAL_PERIOD_S
        # Per (side, cam): last known good (abs_frame_id, timestamp_s)
        self._last_good: dict[tuple[int, int], tuple[int, float]] = {}
        self._buffer: list[_BufferedFrame] = []
        self._in_bad_run = False
        self._window_onset: Optional[_WindowStats] = None
        # Scan-wide summary
        self._scan_windows: list[_WindowStats] = []
        self._total_frames_seen = 0

    def process(self, batch: FrameBatch) -> FrameBatch:
        if batch.abs_frame_ids is None or batch.frame_type is None:
            return batch

        n = len(batch.cam_ids)
        if n == 0:
            batch.quality = np.empty(0, dtype="<U14")
            return batch

        self._total_frames_seen += n

        # --- Condition 2: in-packet frame_id disagreement ---
        bad_by_cond2 = self._detect_frame_id_disagreement(batch)

        # --- Per-row processing: condition 1 + routing ---
        quality = np.full(n, "ok", dtype="<U14")
        output_indices = []
        nan_fill_inserts = []

        for i in range(n):
            ftype = str(batch.frame_type[i])
            if ftype in ("warmup", "stale"):
                output_indices.append(i)
                continue

            cam_id = int(batch.cam_ids[i])
            side_idx = int(batch.side_ids[i])
            abs_fid = int(batch.abs_frame_ids[i])
            ts = float(batch.timestamp_s[i])
            key = (side_idx, cam_id)

            is_bad = i in bad_by_cond2

            # Condition 1: timestamp deviation
            if not is_bad and key in self._last_good:
                prev_fid, prev_ts = self._last_good[key]
                fid_gap = abs_fid - prev_fid
                if fid_gap > 0:
                    expected_dt = fid_gap * self._nominal_period
                    actual_dt = ts - prev_ts
                    if abs(actual_dt - expected_dt) > self._tolerance:
                        is_bad = True

            if is_bad:
                if not self._in_bad_run:
                    self._in_bad_run = True
                    self._window_onset = _WindowStats(
                        onset_fid=abs_fid, onset_t=ts,
                    )
                self._buffer.append(_BufferedFrame(
                    idx=i, cam_id=cam_id, side_idx=side_idx,
                    abs_frame_id=abs_fid, frame_id=int(batch.frame_ids[i]),
                    original_ts=ts, frame_type=ftype,
                ))
                if len(self._buffer) >= self._max_buffer:
                    flushed = self._force_flush_buffer(batch, quality)
                    output_indices.extend(flushed)
            else:
                if self._in_bad_run:
                    # Re-anchor: this good frame closes the bad run
                    flushed = self._reanchor_flush(
                        batch, quality, anchor_fid=abs_fid, anchor_ts=ts,
                    )
                    output_indices.extend(flushed)

                # Check for NaN-fill gaps
                if key in self._last_good:
                    prev_fid, prev_ts = self._last_good[key]
                    gap = abs_fid - prev_fid
                    if gap > 1:
                        fills = self._make_nan_fills(
                            side_idx=side_idx, cam_id=cam_id,
                            start_fid=prev_fid + 1, end_fid=abs_fid,
                            start_ts=prev_ts, end_ts=ts,
                            batch=batch,
                        )
                        nan_fill_inserts.append((len(output_indices), fills))

                # Update nominal period from good single-step frames
                if key in self._last_good:
                    prev_fid, prev_ts = self._last_good[key]
                    if abs_fid - prev_fid == 1:
                        measured_dt = ts - prev_ts
                        if measured_dt > 0:
                            self._nominal_period = (
                                (1 - _EMA_ALPHA) * self._nominal_period
                                + _EMA_ALPHA * measured_dt
                            )

                self._last_good[key] = (abs_fid, ts)
                quality[i] = "ok"
                output_indices.append(i)

        batch.quality = quality

        # If we have NaN fills or buffer flushes that changed things,
        # rebuild the batch with insertions
        if nan_fill_inserts:
            batch = self._rebuild_batch_with_fills(batch, output_indices, nan_fill_inserts)

        return batch

    def _detect_frame_id_disagreement(self, batch: FrameBatch) -> set[int]:
        """Condition 2: find rows where cameras at the same timestamp disagree on frame_id."""
        bad_indices: set[int] = set()
        ts_groups: dict[float, list[int]] = defaultdict(list)
        n = len(batch.cam_ids)
        for i in range(n):
            ftype = str(batch.frame_type[i])
            if ftype in ("warmup", "stale"):
                continue
            ts_groups[float(batch.timestamp_s[i])].append(i)

        for ts_val, indices in ts_groups.items():
            if len(indices) < 2:
                continue
            fids = set()
            for idx in indices:
                fids.add(int(batch.frame_ids[idx]))
            if len(fids) > 1:
                bad_indices.update(indices)

        return bad_indices

    def _reanchor_flush(self, batch: FrameBatch, quality: np.ndarray, *,
                        anchor_fid: int, anchor_ts: float) -> list[int]:
        """Interpolate buffered frames between last good and the re-anchor, then flush."""
        flushed_indices = []
        if not self._buffer:
            self._in_bad_run = False
            return flushed_indices

        # Find the left anchor from the buffer's first frame's camera
        first = self._buffer[0]
        key = (first.side_idx, first.cam_id)
        if key in self._last_good:
            left_fid, left_ts = self._last_good[key]
        else:
            left_fid = first.abs_frame_id - 1
            left_ts = first.original_ts - self._nominal_period

        fid_span = anchor_fid - left_fid
        ts_span = anchor_ts - left_ts

        for bf in self._buffer:
            if fid_span > 0:
                corrected_ts = left_ts + (bf.abs_frame_id - left_fid) / fid_span * ts_span
            else:
                corrected_ts = bf.original_ts
            batch.timestamp_s[bf.idx] = corrected_ts
            quality[bf.idx] = "ts_corrected"
            flushed_indices.append(bf.idx)

        # Log the window
        if self._window_onset is not None:
            self._window_onset.end_fid = self._buffer[-1].abs_frame_id
            self._window_onset.end_t = self._buffer[-1].original_ts
            self._window_onset.n_corrected = len(self._buffer)
            self._scan_windows.append(self._window_onset)
            logger.warning(
                "Misalignment window: frames %d–%d (t=%.2f–%.2fs), "
                "%d frames re-timestamped, %d frames NaN-filled",
                self._window_onset.onset_fid, self._window_onset.end_fid,
                self._window_onset.onset_t, self._window_onset.end_t,
                self._window_onset.n_corrected, self._window_onset.n_nan,
            )

        # Update last_good for all cameras that were in the buffer
        for bf in self._buffer:
            bk = (bf.side_idx, bf.cam_id)
            self._last_good[bk] = (bf.abs_frame_id, float(batch.timestamp_s[bf.idx]))

        self._buffer.clear()
        self._in_bad_run = False
        self._window_onset = None
        return flushed_indices

    def _force_flush_buffer(self, batch: FrameBatch, quality: np.ndarray) -> list[int]:
        """Buffer full with no re-anchor: interpolate using nominal period."""
        flushed_indices = []
        if not self._buffer:
            return flushed_indices

        first = self._buffer[0]
        key = (first.side_idx, first.cam_id)
        if key in self._last_good:
            left_fid, left_ts = self._last_good[key]
        else:
            left_fid = first.abs_frame_id - 1
            left_ts = first.original_ts - self._nominal_period

        for bf in self._buffer:
            fid_delta = bf.abs_frame_id - left_fid
            corrected_ts = left_ts + fid_delta * self._nominal_period
            batch.timestamp_s[bf.idx] = corrected_ts
            quality[bf.idx] = "ts_corrected"
            flushed_indices.append(bf.idx)

        if self._window_onset is not None:
            self._window_onset.end_fid = self._buffer[-1].abs_frame_id
            self._window_onset.end_t = self._buffer[-1].original_ts
            self._window_onset.n_corrected = len(self._buffer)
            self._scan_windows.append(self._window_onset)
            logger.warning(
                "Misalignment window (forced flush): frames %d–%d "
                "(t=%.2f–%.2fs), %d frames re-timestamped, %d frames NaN-filled",
                self._window_onset.onset_fid, self._window_onset.end_fid,
                self._window_onset.onset_t, self._window_onset.end_t,
                self._window_onset.n_corrected, self._window_onset.n_nan,
            )

        for bf in self._buffer:
            bk = (bf.side_idx, bf.cam_id)
            self._last_good[bk] = (bf.abs_frame_id, float(batch.timestamp_s[bf.idx]))

        self._buffer.clear()
        self._in_bad_run = False
        self._window_onset = None
        return flushed_indices

    def _make_nan_fills(self, *, side_idx: int, cam_id: int,
                        start_fid: int, end_fid: int,
                        start_ts: float, end_ts: float,
                        batch: FrameBatch) -> list[dict]:
        """Create synthetic NaN-fill frame descriptors for missing abs_frame_ids."""
        fills = []
        fid_span = end_fid - (start_fid - 1)
        ts_span = end_ts - start_ts
        for fid in range(start_fid, end_fid):
            frac = (fid - (start_fid - 1)) / fid_span if fid_span > 0 else 0
            interp_ts = start_ts + frac * ts_span
            fills.append({
                "cam_id": cam_id,
                "frame_id": fid & 0xFF,
                "side_idx": side_idx,
                "abs_frame_id": fid,
                "timestamp_s": interp_ts,
                "frame_type": "light",
                "quality": "nan_filled",
            })
        if self._window_onset is not None:
            self._window_onset.n_nan += len(fills)
        return fills

    def _rebuild_batch_with_fills(self, batch: FrameBatch,
                                  output_indices: list[int],
                                  nan_fill_inserts: list[tuple[int, list[dict]]]) -> FrameBatch:
        """Rebuild the batch, inserting NaN-fill rows at the right positions."""
        # Build ordered list of (position, fill_dicts) sorted by position descending
        # so inserting doesn't shift indices
        inserts_by_pos = sorted(nan_fill_inserts, key=lambda x: x[0])

        # Count total fills
        total_fills = sum(len(fills) for _, fills in inserts_by_pos)
        if total_fills == 0:
            return batch

        n_orig = len(batch.cam_ids)
        n_new = n_orig + total_fills

        # Create new arrays
        new_cam_ids = np.zeros(n_new, dtype=np.int8)
        new_frame_ids = np.zeros(n_new, dtype=np.uint8)
        new_side_ids = np.zeros(n_new, dtype=np.int8)
        new_abs_frame_ids = np.zeros(n_new, dtype=np.int64)
        new_timestamp_s = np.zeros(n_new, dtype=np.float64)
        new_frame_type = np.empty(n_new, dtype="<U14")
        new_quality = np.empty(n_new, dtype="<U14")
        new_raw_histograms = np.zeros((n_new, 2, 8, 1024), dtype=np.uint32)
        new_temperature_c = np.zeros((n_new, 2, 8), dtype=np.float32)

        # Build an ordered sequence of (source_type, data) entries
        # source_type = "orig" with index, or "fill" with fill dict
        entries = []
        fill_iter = iter(inserts_by_pos)
        next_fill = next(fill_iter, None)

        for pos, orig_idx in enumerate(output_indices):
            while next_fill is not None and next_fill[0] == pos:
                for fd in next_fill[1]:
                    entries.append(("fill", fd))
                next_fill = next(fill_iter, None)
            entries.append(("orig", orig_idx))

        # Drain remaining fills
        while next_fill is not None:
            for fd in next_fill[1]:
                entries.append(("fill", fd))
            next_fill = next(fill_iter, None)

        for out_i, (src_type, data) in enumerate(entries):
            if src_type == "orig":
                orig_i = data
                new_cam_ids[out_i] = batch.cam_ids[orig_i]
                new_frame_ids[out_i] = batch.frame_ids[orig_i]
                new_side_ids[out_i] = batch.side_ids[orig_i]
                new_abs_frame_ids[out_i] = batch.abs_frame_ids[orig_i]
                new_timestamp_s[out_i] = batch.timestamp_s[orig_i]
                new_frame_type[out_i] = str(batch.frame_type[orig_i])
                new_quality[out_i] = str(batch.quality[orig_i])
                new_raw_histograms[out_i] = batch.raw_histograms[orig_i]
                new_temperature_c[out_i] = batch.temperature_c[orig_i]
            else:
                fd = data
                new_cam_ids[out_i] = fd["cam_id"]
                new_frame_ids[out_i] = fd["frame_id"]
                new_side_ids[out_i] = fd["side_idx"]
                new_abs_frame_ids[out_i] = fd["abs_frame_id"]
                new_timestamp_s[out_i] = fd["timestamp_s"]
                new_frame_type[out_i] = fd["frame_type"]
                new_quality[out_i] = fd["quality"]
                # raw_histograms and temperature_c stay zero

        new_batch = FrameBatch(
            cam_ids=new_cam_ids,
            frame_ids=new_frame_ids,
            side_ids=new_side_ids,
            raw_histograms=new_raw_histograms,
            temperature_c=new_temperature_c,
            timestamp_s=new_timestamp_s,
            pdc=None, tcm=None, tcl=None,
        )
        new_batch.abs_frame_ids = new_abs_frame_ids
        new_batch.frame_type = new_frame_type
        new_batch.quality = new_quality
        new_batch.events = batch.events
        return new_batch

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """Flush remaining buffer + emit end-of-scan summary."""
        if self._buffer:
            if batch.quality is None:
                batch.quality = np.empty(0, dtype="<U14")
            self._force_flush_buffer(batch, batch.quality)

        if self._scan_windows:
            total_corrected = sum(w.n_corrected for w in self._scan_windows)
            total_nan = sum(w.n_nan for w in self._scan_windows)
            pct = (total_corrected + total_nan) / max(1, self._total_frames_seen) * 100
            logger.warning(
                "Scan summary: %d misalignment window(s), %d frames re-timestamped, "
                "%d frames NaN-filled (%.1f%% of scan affected)",
                len(self._scan_windows), total_corrected, total_nan, pct,
            )

    def reset(self) -> None:
        self._reset_state()
```

- [ ] **Step 4: Run the clean passthrough test**

Run: `pytest tests/test_pipeline/test_timestamp_repair_stage.py::test_clean_passthrough -v`
Expected: PASS

- [ ] **Step 5: Write test for condition 1 detection + re-anchoring**

Add to `tests/test_pipeline/test_timestamp_repair_stage.py`:

```python
def test_condition1_bad_timestamp_gets_corrected():
    """A frame with timestamp off by >2ms is corrected via re-anchoring."""
    stage = TimestampRepairStage(tolerance_s=0.002, max_buffer_frames=16)
    # Frame 12 has a bad timestamp (jumped 50ms instead of 25ms)
    # Frame 13 is good and re-anchors
    ts = [0.025, 0.050, 0.100, 0.100]  # frame 12 @100ms is wrong (should be ~75ms)
    batch = _make_batch(
        cam_ids=[0, 0, 0, 0],
        frame_ids=[11, 12, 13, 14],
        side_ids=[0, 0, 0, 0],
        timestamps=ts,
        abs_frame_ids=[11, 12, 13, 14],
        frame_types=["light", "light", "light", "light"],
    )
    result = stage.process(batch)
    # Frame 11 passes through (first frame, no prior reference)
    assert result.quality[0] == "ok"
    # Frame 12 was bad (jumped from 50ms to 100ms = 50ms delta, expected 25ms, off by 25ms > 2ms)
    assert result.quality[1] == "ts_corrected"
    # Frame 13 re-anchored
    assert result.quality[2] == "ok"
    # Corrected timestamp for frame 12 should be interpolated between frame 11 (50ms) and frame 13 (100ms)
    # frame 12 is 1/2 of the way from frame 11 to frame 13 by frame_id
    expected_ts_12 = 0.050 + (12 - 11) / (13 - 11) * (0.100 - 0.050)  # = 0.075
    assert abs(result.timestamp_s[1] - expected_ts_12) < 1e-9
```

- [ ] **Step 6: Run test to verify it passes**

Run: `pytest tests/test_pipeline/test_timestamp_repair_stage.py::test_condition1_bad_timestamp_gets_corrected -v`
Expected: PASS

- [ ] **Step 7: Write test for condition 2 detection**

```python
def test_condition2_frame_id_disagreement():
    """Cameras at the same timestamp with different frame_ids are flagged bad."""
    stage = TimestampRepairStage(tolerance_s=0.002, max_buffer_frames=16)
    # Two cameras at t=0.050 disagree: cam0 says frame 12, cam1 says frame 13
    # Then a good frame at t=0.075 re-anchors
    batch = _make_batch(
        cam_ids=[0, 0, 1, 0],
        frame_ids=[11, 12, 13, 14],
        side_ids=[0, 0, 0, 0],
        timestamps=[0.025, 0.050, 0.050, 0.075],
        abs_frame_ids=[11, 12, 13, 14],
        frame_types=["light", "light", "light", "light"],
    )
    result = stage.process(batch)
    # Frames at t=0.050 should be flagged as bad (frame_id disagreement)
    assert result.quality[1] == "ts_corrected"
    assert result.quality[2] == "ts_corrected"
```

- [ ] **Step 8: Run test**

Run: `pytest tests/test_pipeline/test_timestamp_repair_stage.py::test_condition2_frame_id_disagreement -v`
Expected: PASS

- [ ] **Step 9: Write test for NaN-fill insertion**

```python
def test_nan_fill_for_missing_frames():
    """Missing abs_frame_ids get synthetic NaN-fill rows inserted."""
    stage = TimestampRepairStage(tolerance_s=0.002, max_buffer_frames=16)
    # Frame 11 then frame 14 — frames 12 and 13 are missing
    batch = _make_batch(
        cam_ids=[0, 0],
        frame_ids=[11, 14],
        side_ids=[0, 0],
        timestamps=[0.025, 0.100],
        abs_frame_ids=[11, 14],
        frame_types=["light", "light"],
    )
    result = stage.process(batch)
    # Should have 4 rows: original 2 + 2 NaN fills
    assert len(result.cam_ids) == 4
    # Check quality flags
    assert result.quality[0] == "ok"       # frame 11
    assert result.quality[1] == "nan_filled"  # frame 12 (synthetic)
    assert result.quality[2] == "nan_filled"  # frame 13 (synthetic)
    assert result.quality[3] == "ok"       # frame 14
    # Synthetic rows have zero histograms
    assert np.all(result.raw_histograms[1] == 0)
    assert np.all(result.raw_histograms[2] == 0)
    # Timestamps are interpolated
    assert result.timestamp_s[0] == pytest.approx(0.025)
    assert result.timestamp_s[3] == pytest.approx(0.100)
    assert result.timestamp_s[1] > 0.025
    assert result.timestamp_s[2] > result.timestamp_s[1]
    assert result.timestamp_s[2] < 0.100
```

- [ ] **Step 10: Run test**

Run: `pytest tests/test_pipeline/test_timestamp_repair_stage.py::test_nan_fill_for_missing_frames -v`
Expected: PASS

- [ ] **Step 11: Write test for buffer force flush**

```python
def test_buffer_force_flush_at_max():
    """When buffer fills without a re-anchor, force-flush using nominal period."""
    stage = TimestampRepairStage(tolerance_s=0.002, max_buffer_frames=4)
    # 1 good frame, then 5 bad frames (buffer size 4, so force-flush at frame 4)
    ts = [0.025]
    fids = [11]
    # Bad frames: all have timestamp 0.050 (way off from expected 50ms, 75ms, 100ms, 125ms, 150ms)
    for i in range(5):
        ts.append(0.050)
        fids.append(12 + i)
    batch = _make_batch(
        cam_ids=[0] * 6,
        frame_ids=fids,
        side_ids=[0] * 6,
        timestamps=ts,
        abs_frame_ids=fids,
        frame_types=["light"] * 6,
    )
    result = stage.process(batch)
    # The first 4 bad frames should be force-flushed as ts_corrected
    corrected_count = sum(1 for q in result.quality if q == "ts_corrected")
    assert corrected_count >= 4
```

- [ ] **Step 12: Run test**

Run: `pytest tests/test_pipeline/test_timestamp_repair_stage.py::test_buffer_force_flush_at_max -v`
Expected: PASS

- [ ] **Step 13: Write test for coalesced logging**

```python
def test_logging_one_warning_per_window(caplog):
    """One WARNING per misalignment window, not per frame."""
    import logging
    stage = TimestampRepairStage(tolerance_s=0.002, max_buffer_frames=16)
    ts = [0.025, 0.050, 0.100, 0.125]  # frame 12 is bad (50ms jump)
    batch = _make_batch(
        cam_ids=[0, 0, 0, 0],
        frame_ids=[11, 12, 13, 14],
        side_ids=[0, 0, 0, 0],
        timestamps=ts,
        abs_frame_ids=[11, 12, 13, 14],
        frame_types=["light", "light", "light", "light"],
    )
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.pipeline.stages.timestamp_repair"):
        stage.process(batch)
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 1
    assert "Misalignment window" in warnings[0].message


def test_logging_scan_summary(caplog):
    """End-of-scan summary emitted at on_scan_stop."""
    import logging
    stage = TimestampRepairStage(tolerance_s=0.002, max_buffer_frames=16)
    ts = [0.025, 0.050, 0.100, 0.125]
    batch = _make_batch(
        cam_ids=[0, 0, 0, 0],
        frame_ids=[11, 12, 13, 14],
        side_ids=[0, 0, 0, 0],
        timestamps=ts,
        abs_frame_ids=[11, 12, 13, 14],
        frame_types=["light", "light", "light", "light"],
    )
    stage.process(batch)

    flush_batch = _make_batch([], [], [], [], abs_frame_ids=[], frame_types=[])
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.pipeline.stages.timestamp_repair"):
        stage.on_scan_stop(flush_batch)
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    summary = [w for w in warnings if "Scan summary" in w.message]
    assert len(summary) == 1


def test_no_logging_on_clean_scan(caplog):
    """Clean scan produces zero log output."""
    import logging
    stage = TimestampRepairStage(tolerance_s=0.002, max_buffer_frames=16)
    ts = [0.025, 0.050, 0.075, 0.100]
    batch = _make_batch(
        cam_ids=[0, 0, 0, 0],
        frame_ids=[11, 12, 13, 14],
        side_ids=[0, 0, 0, 0],
        timestamps=ts,
        abs_frame_ids=[11, 12, 13, 14],
        frame_types=["light", "light", "light", "light"],
    )
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.pipeline.stages.timestamp_repair"):
        stage.process(batch)
        flush_batch = _make_batch([], [], [], [], abs_frame_ids=[], frame_types=[])
        stage.on_scan_stop(flush_batch)
    warnings = [r for r in caplog.records if r.levelno >= logging.WARNING]
    assert len(warnings) == 0
```

- [ ] **Step 14: Run all logging tests**

Run: `pytest tests/test_pipeline/test_timestamp_repair_stage.py -k "logging" -v`
Expected: ALL PASS

- [ ] **Step 15: Write test for warmup/stale passthrough**

```python
def test_warmup_and_stale_frames_pass_through_untouched():
    """Warmup and stale frames are not subject to divergence detection."""
    stage = TimestampRepairStage(tolerance_s=0.002, max_buffer_frames=16)
    batch = _make_batch(
        cam_ids=[0, 0, 0],
        frame_ids=[1, 2, 10],
        side_ids=[0, 0, 0],
        timestamps=[0.0, 0.025, 0.250],
        abs_frame_ids=[1, 2, 10],
        frame_types=["warmup", "warmup", "dark"],
    )
    result = stage.process(batch)
    assert len(result.cam_ids) == 3
    np.testing.assert_array_equal(result.quality, ["ok", "ok", "ok"])
```

- [ ] **Step 16: Run full stage test suite**

Run: `pytest tests/test_pipeline/test_timestamp_repair_stage.py -v`
Expected: ALL PASS

- [ ] **Step 17: Commit**

```
git add omotion/pipeline/stages/timestamp_repair.py tests/test_pipeline/test_timestamp_repair_stage.py
git commit -m "feat(pipeline): add TimestampRepairStage with detection, re-anchoring, NaN-fill, logging"
```

---

### Task 4: Wire TimestampRepairStage into the pipeline factory

**Files:**
- Modify: `omotion/pipeline/factory.py`
- Test: `tests/test_pipeline/test_factory.py`

- [ ] **Step 1: Write the failing test**

Add to `tests/test_pipeline/test_factory.py`:

```python
def test_pipeline_order_raw_tee_before_timestamp_repair():
    """Tee('raw') must come before TimestampRepairStage in the pipeline."""
    from omotion.pipeline.factory import default_pipeline
    from omotion.pipeline.sinks import ScanMetadata
    from omotion.pipeline.pedestal import SensorPedestals
    from omotion.pipeline.stages.timestamp_repair import TimestampRepairStage
    from omotion.pipeline.tee import Tee

    meta = ScanMetadata(
        scan_id="test", subject_id="s", operator="op",
        started_at_iso="2026-01-01T00:00:00", duration_sec=60,
        left_camera_mask=0x03, right_camera_mask=0x03,
        reduced_mode=False,
    )

    class _FakeCalibration:
        c_min = c_max = i_min = i_max = None

    pipeline = default_pipeline(
        metadata=meta,
        calibration=_FakeCalibration(),
        pedestals=SensorPedestals(left=64.0, right=64.0),
    )

    stage_names = [s.name if hasattr(s, 'name') else type(s).__name__ for s in pipeline.stages]
    # Find positions
    raw_tee_idx = None
    repair_idx = None
    for i, s in enumerate(pipeline.stages):
        if isinstance(s, Tee) and getattr(s, '_channel', None) == "raw":
            raw_tee_idx = i
        if isinstance(s, TimestampRepairStage):
            repair_idx = i

    assert raw_tee_idx is not None, "Tee('raw') not found in pipeline"
    assert repair_idx is not None, "TimestampRepairStage not found in pipeline"
    assert raw_tee_idx < repair_idx, (
        f"Tee('raw') at index {raw_tee_idx} must come before "
        f"TimestampRepairStage at index {repair_idx}"
    )
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_factory.py::test_pipeline_order_raw_tee_before_timestamp_repair -v`
Expected: FAIL — TimestampRepairStage not found

- [ ] **Step 3: Update factory.py**

In `omotion/pipeline/factory.py`, add the import:

```python
from .stages.timestamp_repair import TimestampRepairStage
```

Then reorder the stages list. Replace the current stage assembly (lines ~46-88) with:

```python
    stages: list = [
        FrameClassificationStage(discard_count=discard_count, dark_interval=dark_interval),
    ]

    # Raw tee BEFORE timestamp repair — raw CSV sees untouched device timestamps
    if raw_save_max_duration_s is None or raw_save_max_duration_s > 0:
        stages.append(
            Tee("raw", filter=lambda ft: ft != "stale", max_duration_s=raw_save_max_duration_s)
        )

    stages.append(
        TimestampRepairStage(tolerance_s=0.002, max_buffer_frames=16)
    )

    not_warmup_or_stale = lambda ft: ft != "warmup" and ft != "stale"

    stages.extend([
        NoiseFloorStage(threshold=noise_floor_threshold),
        MomentsStage(),
        PedestalSubtractionStage(pedestals=pedestals),

        DarkCorrectionStage(
            realtime_estimator=HybridRealtimePredictor(),
            batch_estimator=LinearInterpolation(),
            pedestals=pedestals,
            realtime_history_size=realtime_dark_history_size,
        ),

        ShotNoiseCorrectionStage(pedestals=pedestals, camera_gain_map=CAMERA_GAIN_MAP),
        BfiBviStage(calibration=calibration),
        DarkFrameHoldStage(),
        SideAverageStage(
            enabled=metadata.reduced_mode,
            left_camera_mask=metadata.left_camera_mask,
            right_camera_mask=metadata.right_camera_mask,
        ),
        Tee("live", filter=not_warmup_or_stale),
    ])
```

- [ ] **Step 4: Run test**

Run: `pytest tests/test_pipeline/test_factory.py::test_pipeline_order_raw_tee_before_timestamp_repair -v`
Expected: PASS

- [ ] **Step 5: Run full factory + pipeline test suite**

Run: `pytest tests/test_pipeline/test_factory.py tests/test_pipeline/test_pipeline_class.py -v`
Expected: ALL PASS

- [ ] **Step 6: Commit**

```
git add omotion/pipeline/factory.py tests/test_pipeline/test_factory.py
git commit -m "feat(pipeline): wire TimestampRepairStage into factory, reorder raw tee"
```

---

### Task 5: Add `quality` column to CsvSink

**Files:**
- Modify: `omotion/pipeline/sinks.py:77-89,332-429`
- Test: `tests/test_pipeline/test_csv_sink.py`

- [ ] **Step 1: Write the failing test**

Add to `tests/test_pipeline/test_csv_sink.py`:

```python
def test_corrected_csv_has_quality_column(tmp_path):
    """Corrected CSV must include a quality column."""
    from omotion.pipeline.sinks import CsvSink, ScanMetadata, _corrected_headers_normal
    headers = _corrected_headers_normal()
    assert "quality" in headers
    assert headers[-1] == "quality"


def test_corrected_csv_quality_column_reduced(tmp_path):
    """Reduced-mode corrected CSV also has quality."""
    from omotion.pipeline.sinks import _corrected_headers_reduced
    headers = _corrected_headers_reduced()
    assert "quality" in headers
    assert headers[-1] == "quality"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_pipeline/test_csv_sink.py::test_corrected_csv_has_quality_column tests/test_pipeline/test_csv_sink.py::test_corrected_csv_quality_column_reduced -v`
Expected: FAIL — `"quality" not in headers`

- [ ] **Step 3: Add quality to CSV headers**

In `omotion/pipeline/sinks.py`, modify `_corrected_headers_normal()` (line ~77):

```python
def _corrected_headers_normal() -> list[str]:
    """83-column corrected CSV header (82 legacy + quality)."""
    cols = ["frame_id", "timestamp_s"]
    for metric in ("bfi", "bvi", "mean", "contrast", "temp"):
        for side in ("l", "r"):
            for cam in range(1, 9):
                cols.append(f"{metric}_{side}{cam}")
    cols.append("quality")
    return cols
```

Modify `_corrected_headers_reduced()` (line ~87):

```python
def _corrected_headers_reduced() -> list[str]:
    """7-column reduced corrected CSV header."""
    return ["frame_id", "timestamp_s", "bfi_left", "bfi_right", "bvi_left", "bvi_right", "quality"]
```

- [ ] **Step 4: Thread quality through row accumulation**

In `_consume_final` (line ~332), after populating metrics for the row, add quality tracking. In the `_corrected_acc[abs_id]` entry initialization, add a `"quality"` key:

```python
            if abs_id not in acc:
                row = [""] * self._corrected_n_cols
                acc[abs_id] = {"t": float(frame.t), "row": row, "quality": "ok"}
```

After the metric population block (both normal and reduced mode), update quality:

```python
            frame_quality = getattr(frame, "quality", "ok")
            entry = acc[abs_id]
            # Worst quality wins: nan_filled > ts_corrected > ok
            _QUALITY_RANK = {"ok": 0, "ts_corrected": 1, "nan_filled": 2}
            if _QUALITY_RANK.get(frame_quality, 0) > _QUALITY_RANK.get(entry["quality"], 0):
                entry["quality"] = frame_quality
```

In `_maybe_flush_row`, when writing the row, set the quality column (last column):

```python
                row[0] = abs_id
                row[1] = round(t, 9)
                row[-1] = entry.get("quality", "ok")
                self._write_corrected_row(row)
```

Do the same in `on_complete`'s partial-row flush (line ~252):

```python
                row[0] = abs_id
                row[1] = round(entry["t"], 9)
                row[-1] = entry.get("quality", "ok")
                self._write_corrected_row(row)
```

- [ ] **Step 5: Run CSV sink tests**

Run: `pytest tests/test_pipeline/test_csv_sink.py -v`
Expected: ALL PASS

- [ ] **Step 6: Commit**

```
git add omotion/pipeline/sinks.py tests/test_pipeline/test_csv_sink.py
git commit -m "feat(pipeline): add quality column to corrected CSV"
```

---

### Task 6: Add `quality` column to ScanDBSink

**Files:**
- Modify: `omotion/ScanDatabase.py:115-129,497-527`
- Modify: `omotion/pipeline/sinks.py` (ScanDBSink._consume_live, _consume_side)
- Test: `tests/test_pipeline/test_scan_db_sink.py`

- [ ] **Step 1: Write the failing test**

Add to `tests/test_pipeline/test_scan_db_sink.py`:

```python
def test_session_data_has_quality_column(tmp_path):
    """session_data table must include a quality column."""
    from omotion.ScanDatabase import ScanDatabase
    db = ScanDatabase(db_path=str(tmp_path / "test.db"))
    conn = db._connection()
    cursor = conn.execute("PRAGMA table_info(session_data)")
    columns = {row[1] for row in cursor.fetchall()}
    assert "quality" in columns
    db.close()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_scan_db_sink.py::test_session_data_has_quality_column -v`
Expected: FAIL — `"quality" not in columns`

- [ ] **Step 3: Add quality to session_data schema**

In `omotion/ScanDatabase.py`, modify the `CREATE TABLE session_data` block (line ~115) to add:

```sql
                mean             REAL,
                quality          TEXT DEFAULT 'ok'
```

In `insert_session_data_rows` (line ~497), add quality to the INSERT:

```python
            params.append(
                (
                    row["session_id"],
                    row.get("session_raw_id"),
                    row["cam_id"],
                    row["side"],
                    int(row.get("frame_id", -1)),
                    row["timestamp_s"],
                    row.get("bfi"),
                    row.get("bvi"),
                    row.get("contrast"),
                    row.get("mean"),
                    row.get("quality", "ok"),
                )
            )

        self._connection().executemany(
            """
            INSERT INTO session_data (
                session_id, session_raw_id, cam_id, side,
                frame_id, timestamp_s, bfi, bvi, contrast, mean, quality
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            params,
        )
```

- [ ] **Step 4: Add quality to ScanDBSink row building**

In `omotion/pipeline/sinks.py`, in `ScanDBSink._consume_live` (line ~574), add quality to the row dict:

```python
            rows.append({
                ...existing fields...
                "quality": str(batch.quality[i]) if batch.quality is not None else "ok",
            })
```

In `ScanDBSink._consume_side` (line ~643), add quality to the side buffer entry:

```python
        self._side_buffer.append({
            ...existing fields...
            "quality": getattr(sample, "quality", "ok") or "ok",
        })
```

- [ ] **Step 5: Add ALTER TABLE migration for existing DBs**

In `ScanDatabase._init_schema` or a separate migration method, add after the existing CREATE TABLE statements:

```python
        try:
            conn.execute("ALTER TABLE session_data ADD COLUMN quality TEXT DEFAULT 'ok'")
        except sqlite3.OperationalError:
            pass  # column already exists
```

- [ ] **Step 6: Run tests**

Run: `pytest tests/test_pipeline/test_scan_db_sink.py -v`
Expected: ALL PASS

- [ ] **Step 7: Commit**

```
git add omotion/ScanDatabase.py omotion/pipeline/sinks.py tests/test_pipeline/test_scan_db_sink.py
git commit -m "feat(pipeline): add quality column to ScanDBSink + schema migration"
```

---

### Task 7: Integration test — clean-scan regression

**Files:**
- Create: `tests/test_pipeline/test_eft_regression.py`

- [ ] **Step 1: Write the clean-scan regression test**

Create `tests/test_pipeline/test_eft_regression.py`:

```python
"""EFT clean-scan regression — corrected output must not change for clean scans.

These tests replay clean scans through the pipeline and verify the corrected
CSV is numerically identical to current output (excluding the new quality column).
"""

import csv
from pathlib import Path

import numpy as np
import pytest

from omotion.pipeline.factory import default_pipeline
from omotion.pipeline.pedestal import SensorPedestals
from omotion.pipeline.runner import ScanRunner
from omotion.pipeline.sinks import CsvSink, ScanMetadata
from omotion.pipeline.sources import CsvReplaySource


SCANS_DIR = Path(r"C:\Users\ethan\Projects\eft-testing\scans")

CLEAN_SCANS = [
    {
        "name": "owYWB8TN",
        "left_raw": SCANS_DIR / "20260602_135759_owYWB8TN_left_mask66_raw.csv",
        "right_raw": SCANS_DIR / "20260602_135759_owYWB8TN_right_mask66_raw.csv",
        "baseline_corrected": SCANS_DIR / "20260602_135759_owYWB8TN.csv",
        "left_mask": 0x66,
        "right_mask": 0x66,
    },
    {
        "name": "owYZ7T66_clean",
        "left_raw": SCANS_DIR / "20260603_130423_owYZ7T66_left_maskC3_raw.csv",
        "right_raw": SCANS_DIR / "20260603_130423_owYZ7T66_right_maskC3_raw.csv",
        "baseline_corrected": SCANS_DIR / "20260603_130423_owYZ7T66.csv",
        "left_mask": 0xC3,
        "right_mask": 0xC3,
    },
]


class _NullCalibration:
    """Identity calibration — BFI = contrast * 10, BVI = mean * 10."""
    c_min = np.zeros((2, 8))
    c_max = np.zeros((2, 8))
    i_min = np.zeros((2, 8))
    i_max = np.zeros((2, 8))


def _replay_scan(scan_info, output_dir):
    """Replay a scan through the full pipeline and return the corrected CSV path."""
    meta = ScanMetadata(
        scan_id=scan_info["name"],
        subject_id="test",
        operator="regression",
        started_at_iso="2026-01-01T00:00:00",
        duration_sec=600,
        left_camera_mask=scan_info["left_mask"],
        right_camera_mask=scan_info["right_mask"],
        reduced_mode=False,
    )
    source = CsvReplaySource(
        raw_csv_left=scan_info["left_raw"],
        raw_csv_right=scan_info["right_raw"],
        metadata=meta,
        batch_size_frames=100,
    )
    pipeline = default_pipeline(
        metadata=meta,
        calibration=_NullCalibration(),
        pedestals=SensorPedestals(left=64.0, right=64.0),
    )
    sink = CsvSink(output_dir=str(output_dir))
    runner = ScanRunner(source=source, pipeline=pipeline, sinks=[sink])
    runner.run()
    # Find the corrected CSV in output_dir
    csvs = list(Path(output_dir).glob("*.csv"))
    corrected = [c for c in csvs if "_raw" not in c.name]
    assert len(corrected) == 1, f"Expected 1 corrected CSV, found {corrected}"
    return corrected[0]


def _compare_corrected_csvs(new_csv: Path, baseline_csv: Path):
    """Compare corrected CSVs: all pre-existing columns must match numerically."""
    with open(baseline_csv) as f:
        baseline_reader = csv.DictReader(f)
        baseline_rows = list(baseline_reader)
        baseline_fields = baseline_reader.fieldnames

    with open(new_csv) as f:
        new_reader = csv.DictReader(f)
        new_rows = list(new_reader)
        new_fields = new_reader.fieldnames

    # All baseline fields must exist in new (new may have extra like "quality")
    for field in baseline_fields:
        assert field in new_fields, f"Baseline field '{field}' missing from new CSV"

    assert len(new_rows) == len(baseline_rows), (
        f"Row count mismatch: baseline={len(baseline_rows)}, new={len(new_rows)}"
    )

    for i, (new_row, base_row) in enumerate(zip(new_rows, baseline_rows)):
        for field in baseline_fields:
            new_val = new_row[field]
            base_val = base_row[field]
            if new_val == base_val:
                continue
            # Try numeric comparison
            try:
                assert float(new_val) == pytest.approx(float(base_val), abs=1e-6), (
                    f"Row {i}, field '{field}': new={new_val}, baseline={base_val}"
                )
            except (ValueError, TypeError):
                assert new_val == base_val, (
                    f"Row {i}, field '{field}': new={new_val!r}, baseline={base_val!r}"
                )

    # Quality column must be all "ok"
    if "quality" in new_fields:
        for i, row in enumerate(new_rows):
            assert row["quality"] == "ok", (
                f"Row {i}: expected quality='ok', got '{row['quality']}'"
            )


@pytest.mark.slow
@pytest.mark.parametrize("scan_info", CLEAN_SCANS, ids=[s["name"] for s in CLEAN_SCANS])
def test_clean_scan_regression(scan_info, tmp_path):
    """Corrected CSV from clean scan must match baseline (excluding quality column)."""
    if not scan_info["left_raw"].exists():
        pytest.skip(f"Test data not found: {scan_info['left_raw']}")
    new_csv = _replay_scan(scan_info, tmp_path)
    _compare_corrected_csvs(new_csv, scan_info["baseline_corrected"])
```

- [ ] **Step 2: Run one clean-scan test**

Run: `pytest tests/test_pipeline/test_eft_regression.py::test_clean_scan_regression[owYWB8TN] -v --timeout=120`
Expected: PASS (may take ~30s for the 60MB raw CSVs)

- [ ] **Step 3: Commit**

```
git add tests/test_pipeline/test_eft_regression.py
git commit -m "test: add clean-scan regression tests for EFT timestamp repair"
```

---

### Task 8: Integration test — degraded-scan structural assertions

**Files:**
- Create: `tests/test_pipeline/test_eft_correction.py`

- [ ] **Step 1: Write the degraded-scan test**

Create `tests/test_pipeline/test_eft_correction.py`:

```python
"""EFT degraded-scan structural assertions.

Replays scans affected by EMI stimulation through the pipeline and verifies:
- Gap-free abs_frame_id grid in corrected CSV
- NaN where data was absent
- Monotonic non-decreasing timestamp_s
- Quality column populated correctly
- Coalesced logging (warnings present, no per-frame spam)
"""

import csv
import logging
from pathlib import Path

import numpy as np
import pytest

from omotion.pipeline.factory import default_pipeline
from omotion.pipeline.pedestal import SensorPedestals
from omotion.pipeline.runner import ScanRunner
from omotion.pipeline.sinks import CsvSink, ScanMetadata
from omotion.pipeline.sources import CsvReplaySource


SCANS_DIR = Path(r"C:\Users\ethan\Projects\eft-testing\scans")
FINAL_TESTS_DIR = Path(
    r"C:\Users\ethan\Projects\eft-testing\final_tests-20260603T234709Z-3-001\final_tests"
)

DEGRADED_SCANS = [
    {
        "name": "owEFTTEST1_1607",
        "left_raw": FINAL_TESTS_DIR / "20260603_160750_owEFTTEST1_left_maskC3_raw.csv",
        "right_raw": FINAL_TESTS_DIR / "20260603_160750_owEFTTEST1_right_maskC3_raw.csv",
        "left_mask": 0xC3,
        "right_mask": 0xC3,
    },
    {
        "name": "owEFTTEST1_1618",
        "left_raw": FINAL_TESTS_DIR / "20260603_161850_owEFTTEST1_left_maskC3_raw.csv",
        "right_raw": FINAL_TESTS_DIR / "20260603_161850_owEFTTEST1_right_maskC3_raw.csv",
        "left_mask": 0xC3,
        "right_mask": 0xC3,
    },
    {
        "name": "owEFTTEST1_1630",
        "left_raw": FINAL_TESTS_DIR / "20260603_163020_owEFTTEST1_left_maskC3_raw.csv",
        "right_raw": FINAL_TESTS_DIR / "20260603_163020_owEFTTEST1_right_maskC3_raw.csv",
        "left_mask": 0xC3,
        "right_mask": 0xC3,
    },
    {
        "name": "owM8T7HS",
        "left_raw": SCANS_DIR / "20260602_135343_owM8T7HS_left_mask66_raw.csv",
        "right_raw": SCANS_DIR / "20260602_135343_owM8T7HS_right_mask66_raw.csv",
        "left_mask": 0x66,
        "right_mask": 0x66,
    },
    {
        "name": "owSPZMD1",
        "left_raw": SCANS_DIR / "20260602_140215_owSPZMD1_left_mask66_raw.csv",
        "right_raw": SCANS_DIR / "20260602_140215_owSPZMD1_right_mask66_raw.csv",
        "left_mask": 0x66,
        "right_mask": 0x66,
    },
]


class _NullCalibration:
    c_min = np.zeros((2, 8))
    c_max = np.zeros((2, 8))
    i_min = np.zeros((2, 8))
    i_max = np.zeros((2, 8))


def _replay_degraded(scan_info, output_dir):
    meta = ScanMetadata(
        scan_id=scan_info["name"], subject_id="test", operator="eft",
        started_at_iso="2026-01-01T00:00:00", duration_sec=600,
        left_camera_mask=scan_info["left_mask"],
        right_camera_mask=scan_info["right_mask"],
        reduced_mode=False,
    )
    source = CsvReplaySource(
        raw_csv_left=scan_info["left_raw"],
        raw_csv_right=scan_info["right_raw"],
        metadata=meta, batch_size_frames=100,
    )
    pipeline = default_pipeline(
        metadata=meta, calibration=_NullCalibration(),
        pedestals=SensorPedestals(left=64.0, right=64.0),
    )
    sink = CsvSink(output_dir=str(output_dir))
    runner = ScanRunner(source=source, pipeline=pipeline, sinks=[sink])
    runner.run()
    csvs = list(Path(output_dir).glob("*.csv"))
    corrected = [c for c in csvs if "_raw" not in c.name]
    assert len(corrected) == 1
    return corrected[0]


@pytest.mark.slow
@pytest.mark.parametrize("scan_info", DEGRADED_SCANS, ids=[s["name"] for s in DEGRADED_SCANS])
def test_degraded_monotonic_timestamps(scan_info, tmp_path):
    """Corrected CSV timestamps must be monotonic non-decreasing."""
    if not scan_info["left_raw"].exists():
        pytest.skip(f"Test data not found: {scan_info['left_raw']}")
    csv_path = _replay_degraded(scan_info, tmp_path)
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        timestamps = [float(row["timestamp_s"]) for row in reader]
    for i in range(1, len(timestamps)):
        assert timestamps[i] >= timestamps[i - 1], (
            f"Non-monotonic at row {i}: {timestamps[i-1]} > {timestamps[i]}"
        )


@pytest.mark.slow
@pytest.mark.parametrize("scan_info", DEGRADED_SCANS, ids=[s["name"] for s in DEGRADED_SCANS])
def test_degraded_has_quality_column(scan_info, tmp_path):
    """Corrected CSV must have quality column with at least some non-ok values."""
    if not scan_info["left_raw"].exists():
        pytest.skip(f"Test data not found: {scan_info['left_raw']}")
    csv_path = _replay_degraded(scan_info, tmp_path)
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        qualities = [row["quality"] for row in reader]
    assert "quality" in reader.fieldnames
    non_ok = [q for q in qualities if q != "ok"]
    assert len(non_ok) > 0, "Degraded scan should have at least some corrected/NaN-filled frames"
    for q in qualities:
        assert q in ("ok", "ts_corrected", "nan_filled"), f"Unexpected quality value: {q!r}"


@pytest.mark.slow
@pytest.mark.parametrize("scan_info", DEGRADED_SCANS, ids=[s["name"] for s in DEGRADED_SCANS])
def test_degraded_logging_coalesced(scan_info, tmp_path, caplog):
    """Logging must be coalesced: one WARNING per window + one summary."""
    if not scan_info["left_raw"].exists():
        pytest.skip(f"Test data not found: {scan_info['left_raw']}")
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.pipeline.stages.timestamp_repair"):
        _replay_degraded(scan_info, tmp_path)
    warnings = [r for r in caplog.records
                if r.name == "openmotion.sdk.pipeline.stages.timestamp_repair"
                and r.levelno == logging.WARNING]
    window_msgs = [w for w in warnings if "Misalignment window" in w.message]
    summary_msgs = [w for w in warnings if "Scan summary" in w.message]
    assert len(window_msgs) >= 1, "Should have at least one misalignment window"
    assert len(summary_msgs) == 1, "Should have exactly one scan summary"
```

- [ ] **Step 2: Run one degraded test to sanity check**

Run: `pytest tests/test_pipeline/test_eft_correction.py::test_degraded_has_quality_column[owM8T7HS] -v --timeout=300`
Expected: PASS (may take ~60s)

- [ ] **Step 3: Commit**

```
git add tests/test_pipeline/test_eft_correction.py
git commit -m "test: add degraded-scan structural assertion tests for EFT correction"
```

---

### Task 9: Extend check_csv.py with post-repair invariant checks

**Files:**
- Modify: `data-processing/check_csv.py`

- [ ] **Step 1: Add post-repair checks**

Add to the end of `check_csv_integrity()` in `data-processing/check_csv.py`, before the summary block:

```python
    # --- Post-repair invariant checks (quality column) ---
    if "quality" in df.columns:
        print("\n[INFO] Post-repair invariant checks:")
        valid_qualities = {"ok", "ts_corrected", "nan_filled"}
        invalid = df[~df["quality"].isin(valid_qualities)]
        if not invalid.empty:
            print(f"  [ERROR] {len(invalid)} rows with invalid quality values")
            errors_found = True
        else:
            print(f"  Quality values: {dict(df['quality'].value_counts())}")

        # Check monotonic timestamps
        if "timestamp_s" in df.columns:
            ts = df["timestamp_s"].values
            non_mono = np.sum(np.diff(ts) < 0)
            if non_mono > 0:
                print(f"  [ERROR] {non_mono} non-monotonic timestamp transitions")
                errors_found = True
            else:
                print("  Timestamps: monotonic non-decreasing")
```

- [ ] **Step 2: Verify it runs on an existing CSV**

Run: `python data-processing/check_csv.py --csv C:\Users\ethan\Projects\eft-testing\scans\20260602_135759_owYWB8TN.csv`
Expected: Runs without error. Quality section either shows valid values or is skipped (quality column not yet present in that CSV).

- [ ] **Step 3: Commit**

```
git add data-processing/check_csv.py
git commit -m "feat(check_csv): add post-repair invariant checks for quality column"
```

---

### Task 10: Run full test suite and verify no regressions

- [ ] **Step 1: Run all pipeline unit tests**

Run: `pytest tests/test_pipeline/ -v --timeout=60 -m "not slow"`
Expected: ALL PASS. No regressions in existing tests.

- [ ] **Step 2: Run clean-scan regression tests**

Run: `pytest tests/test_pipeline/test_eft_regression.py -v --timeout=300`
Expected: ALL PASS (both clean scans produce identical corrected output).

- [ ] **Step 3: Run degraded-scan tests (EFTTEST1 scans)**

Run: `pytest tests/test_pipeline/test_eft_correction.py -v --timeout=600 -k "owEFTTEST1"`
Expected: ALL PASS (monotonic timestamps, quality column populated, coalesced logging).

- [ ] **Step 4: Run remaining degraded-scan tests**

Run: `pytest tests/test_pipeline/test_eft_correction.py -v --timeout=600`
Expected: ALL PASS.

- [ ] **Step 5: Final commit if any fixups were needed**

```
git add -A
git commit -m "fix: address test failures from full EFT integration run"
```

(Skip if no fixes were needed.)
