# Data Pipeline Re-architecture (PR 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the new `omotion/pipeline/` package alongside the current `MotionProcessing.py`. Nothing in production uses the new code yet — that's PR 2.

**Architecture:** 10-stage composable pipeline; numpy-vectorized; 4-channel tee dispatch (raw/live/rolling/final); pure transformation stages with a separate I/O orchestrator. See [`docs/superpowers/specs/2026-05-22-data-pipeline-rearchitecture-design.md`](../specs/2026-05-22-data-pipeline-rearchitecture-design.md) for the design and [`docs/SciencePipeline.md`](../../SciencePipeline.md) for the authoritative algorithm.

**Tech Stack:** Python 3.12+, numpy, pytest, pytest-benchmark (new dep), threading + queue (LiveUsbSource only).

**Scope:** PR 1 of a 3-PR sequence. After this PR lands the new code exists and is fully tested but is unused. PR 2 (separate plan) switches `ScanWorkflow` to use it.

---

## File structure (all new — nothing in this PR modifies production code)

```
omotion/pipeline/
├── __init__.py              # public surface
├── batch.py                 # FrameBatch + BatchEvent + IntervalClosed
├── pipeline.py              # Stage protocol + Pipeline class
├── tee.py                   # Tee stage (emits LiveEmit events)
├── runner.py                # ScanRunner — source → pipeline → sinks
├── factory.py               # default_pipeline() builds the canonical 10-stage chain
├── pedestal.py              # SensorPedestals (per-side, FW-version-keyed lookup)
├── sources.py               # Source protocol, LiveUsbSource, CsvReplaySource, DbReplaySource
├── sinks.py                 # Sink protocol, ScanMetadata, CsvSink, ScanDBSink, QtUiSink
└── stages/
    ├── __init__.py          # re-exports the stage classes
    ├── parse.py             # HistogramParseStage
    ├── classify.py          # FrameClassificationStage + FrameUnwrapper
    ├── noise_floor.py       # NoiseFloorStage
    ├── moments.py           # MomentsStage (einsum-vectorized)
    ├── pedestal_sub.py      # PedestalSubtractionStage
    ├── dark.py              # DarkCorrectionStage + helpers (DarkHistory,
    │                        #   DarkIntegrityGuard, HybridRealtimePredictor,
    │                        #   LinearInterpolation, DarkFrameQuadraticStencil,
    │                        #   PendingInterval)
    ├── shot_noise.py        # ShotNoiseCorrectionStage
    ├── bfi_bvi.py           # BfiBviStage
    ├── rolling_avg.py       # RollingAverageStage
    └── side_avg.py          # SideAveragingStage

tests/test_pipeline/
├── __init__.py
├── test_batch.py
├── test_pipeline_class.py
├── test_tee.py
├── test_sinks_protocol.py
├── test_parse_stage.py
├── test_classify_stage.py
├── test_noise_floor_stage.py
├── test_moments_stage.py
├── test_pedestal_stage.py
├── test_shot_noise_stage.py
├── test_bfi_bvi_stage.py
├── test_rolling_avg_stage.py
├── test_side_avg_stage.py
├── test_dark_history.py
├── test_dark_estimators.py
├── test_dark_integrity_guard.py
├── test_dark_correction_stage.py
├── test_sources.py
├── test_runner.py
├── test_csv_sink.py
├── test_scan_db_sink.py
├── test_golden_replay.py            # full-pipeline test against recorded fixture
├── test_determinism.py
├── data/                            # checked-in golden fixtures
│   ├── normal_short_scan.raw.csv
│   ├── normal_short_scan.corrected.golden.csv
│   ├── reduced_mode_scan.raw.csv
│   └── reduced_mode_scan.corrected.golden.csv
└── benchmarks/
    └── test_stage_perf.py           # pytest-benchmark perf budgets
```

---

## Phase A — Foundation (5 tasks)

These build the skeleton: package layout, the data carrier, the Stage/Pipeline protocols, and the Sink protocol. No science yet.

---

### Task 1: Package skeleton + pytest-benchmark dependency

**Files:**
- Create: `omotion/pipeline/__init__.py`
- Create: `omotion/pipeline/stages/__init__.py`
- Create: `tests/test_pipeline/__init__.py`
- Modify: `pyproject.toml` (add `pytest-benchmark` to dev deps)

- [ ] **Step 1: Create empty package files**

```bash
mkdir -p omotion/pipeline/stages tests/test_pipeline/data tests/test_pipeline/benchmarks
```

Create `omotion/pipeline/__init__.py`:

```python
"""omotion.pipeline — composable, numpy-vectorized histogram → BFI/BVI pipeline.

See docs/SciencePipeline.md for the algorithm.
See docs/superpowers/specs/2026-05-22-data-pipeline-rearchitecture-design.md
for the design that this package implements.
"""
```

Create `omotion/pipeline/stages/__init__.py`:

```python
"""Pipeline stages. Each module implements one named transformation."""
```

Create `tests/test_pipeline/__init__.py`:

```python
"""Tests for omotion.pipeline."""
```

- [ ] **Step 2: Add `pytest-benchmark` to pyproject dev deps**

Open `pyproject.toml` and locate the `[project.optional-dependencies]` table (or `[tool.setuptools.dynamic.optional-dependencies]` — match the existing file's style). In the `dev` list add:

```
pytest-benchmark>=4.0
```

- [ ] **Step 3: Install the new dep**

Run: `pip install -e ".[dev]"`
Expected: `Successfully installed pytest-benchmark-X.Y.Z`

- [ ] **Step 4: Verify pytest still discovers the empty test dir**

Run: `pytest tests/test_pipeline/ -v`
Expected: `collected 0 items` (no tests yet, but no errors either)

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline tests/test_pipeline pyproject.toml
git commit -m "chore(sdk): scaffold omotion.pipeline package + add pytest-benchmark dep"
```

---

### Task 2: FrameBatch dataclass

**Files:**
- Create: `omotion/pipeline/batch.py`
- Create: `tests/test_pipeline/test_batch.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_pipeline/test_batch.py`:

```python
"""Tests for FrameBatch — the per-batch data carrier that flows through stages."""

import numpy as np
import pytest

from omotion.pipeline.batch import FrameBatch, BatchEvent, IntervalClosed


def test_framebatch_construction_minimum_fields():
    """A FrameBatch can be constructed with only the fields HistogramParseStage populates.
    All later-stage fields default to None."""
    n = 5
    batch = FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.linspace(0.0, 0.1, n, dtype=np.float64),
        pdc=None,
        tcm=None,
        tcl=None,
    )
    assert batch.raw_histograms.shape == (n, 2, 8, 1024)
    assert batch.mean_raw is None         # not yet populated by MomentsStage
    assert batch.bfi_live is None         # not yet populated by BfiBviStage
    assert batch.events == []             # empty event list


def test_framebatch_events_list_is_mutable():
    """Stages append to batch.events; the list must start empty and be mutable."""
    batch = _trivial_batch(n=1)
    batch.events.append(IntervalClosed(corrected_batch=None))
    assert len(batch.events) == 1
    assert isinstance(batch.events[0], IntervalClosed)


def _trivial_batch(n: int) -> FrameBatch:
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_batch.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'omotion.pipeline.batch'`

- [ ] **Step 3: Implement FrameBatch**

Create `omotion/pipeline/batch.py`:

```python
"""FrameBatch — the typed data carrier that flows through all pipeline stages.

Each stage's docstring states which fields it owns. Stages mutate the batch
in place for performance (no per-batch allocation churn). Tests assert field
ownership: only the owning stage writes a given field.

All per-frame fields are numpy arrays of shape (N, ...). N is the batch
size — typically 10-100 frames per batch from LiveUsbSource.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


# ---------------------------------------------------------------------------
# Side-channel events (appended to batch.events by stages)
# ---------------------------------------------------------------------------

class BatchEvent:
    """Base type for events that don't fit cleanly into per-frame arrays."""


@dataclass
class IntervalClosed(BatchEvent):
    """Fired by DarkCorrectionStage when a dark interval closes.

    Carries the accurately-corrected interval (linear interpolation between
    bounding darks + 4-point quadratic stencil for the dark frame itself).
    The runner routes this to "final" channel sinks (storage).
    """
    corrected_batch: object  # CorrectedBatch — type forward-decl avoids import cycle


@dataclass
class LiveEmit(BatchEvent):
    """Fired by Tee stages — signals "emit this batch's snapshot to this channel."

    The runner reads channel name and routes the payload to subscribed sinks.
    """
    channel: str
    payload: object


@dataclass
class DarkIntegrityWarning(BatchEvent):
    """A dark frame's u1 exceeded pedestal+threshold. Frame is still processed;
    this is a diagnostic event, not a drop signal."""
    side: str
    cam_id: int
    abs_frame_id: int
    u1: float
    pedestal: float
    threshold: float


@dataclass
class StencilFallback(BatchEvent):
    """The 4-point dark-frame quadratic stencil fell back to a simpler scheme
    because some neighbours were unavailable. Diagnostic, not an error."""
    side: str
    cam_id: int
    abs_frame_id: int
    fallback_used: str  # "right_only" | "simple_avg" | "repeat_right"


# ---------------------------------------------------------------------------
# FrameBatch — the data carrier
# ---------------------------------------------------------------------------

@dataclass
class FrameBatch:
    """N frames worth of data, two sides, 8 cameras each.

    Field ownership (which stage populates which field):
      Parse:           cam_ids, frame_ids, raw_histograms, temperature_c,
                       timestamp_s, pdc, tcm, tcl
      Classify:        abs_frame_ids, frame_type
      NoiseFloor:      (mutates raw_histograms in place — no new field)
      Moments:         mean_raw, std_raw, contrast_raw
      PedestalSubtraction: display_mean
      DarkCorrection:  dark_baseline_rt, mean_dc_rt, std_dc_rt
                       (also appends IntervalClosed to events when interval closes)
      ShotNoise:       std_sn_rt, contrast_sn_rt
      BfiBvi:          bfi_live, bvi_live
      SideAveraging:   bfi_live_side, bvi_live_side (None unless reduced mode)
      RollingAverage:  bfi_rolling, bvi_rolling
      Tee:             appends LiveEmit to events
    """

    # ── populated by HistogramParseStage ──
    cam_ids:        np.ndarray             # (N,)  int8 — 0..7
    frame_ids:      np.ndarray             # (N,)  uint8 — raw 8-bit rolling counter
    raw_histograms: np.ndarray             # (N, 2, 8, 1024) uint32
    temperature_c:  np.ndarray             # (N, 2, 8) float32
    timestamp_s:    np.ndarray             # (N,)  float64 — per-scan t=0 normalized
    pdc:            Optional[np.ndarray]   # (N, 2) float32 — photodiode current per side
    tcm:            Optional[np.ndarray]   # (N,) float32 — console telemetry
    tcl:            Optional[np.ndarray]   # (N,) float32 — console telemetry

    # ── populated by FrameClassificationStage ──
    abs_frame_ids:  Optional[np.ndarray] = None     # (N,) int64 monotonic
    frame_type:     Optional[np.ndarray] = None     # (N,) <U8 ("warmup"|"dark"|"light"|"stale")

    # ── populated by MomentsStage ──
    mean_raw:       Optional[np.ndarray] = None     # (N, 2, 8) float32 — u1 (includes pedestal)
    std_raw:        Optional[np.ndarray] = None     # (N, 2, 8) float32
    contrast_raw:   Optional[np.ndarray] = None     # (N, 2, 8) float32

    # ── populated by PedestalSubtractionStage ──
    display_mean:   Optional[np.ndarray] = None     # (N, 2, 8) float32 — mean_raw minus per-side pedestal

    # ── populated by DarkCorrectionStage (realtime path) ──
    dark_baseline_rt: Optional[np.ndarray] = None   # (N, 2, 8) — predictor's baseline
    mean_dc_rt:       Optional[np.ndarray] = None   # (N, 2, 8) — mean_raw − dark_baseline_rt
    std_dc_rt:        Optional[np.ndarray] = None   # (N, 2, 8)

    # ── populated by ShotNoiseCorrectionStage (live path) ──
    std_sn_rt:        Optional[np.ndarray] = None   # (N, 2, 8) — after Poisson variance subtract
    contrast_sn_rt:   Optional[np.ndarray] = None   # (N, 2, 8)

    # ── populated by BfiBviStage (live path) ──
    bfi_live:       Optional[np.ndarray] = None     # (N, 2, 8)
    bvi_live:       Optional[np.ndarray] = None     # (N, 2, 8)

    # ── populated by SideAveragingStage (live path, when reduced_mode) ──
    bfi_live_side:  Optional[np.ndarray] = None     # (N, 2) averaged over 8 cameras
    bvi_live_side:  Optional[np.ndarray] = None     # (N, 2)

    # ── populated by RollingAverageStage ──
    bfi_rolling:    Optional[np.ndarray] = None     # (N, 2, 8) or (N, 2) if reduced
    bvi_rolling:    Optional[np.ndarray] = None

    # ── side-channel events from any stage ──
    events:         list[BatchEvent] = field(default_factory=list)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_batch.py -v`
Expected: 2 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/batch.py tests/test_pipeline/test_batch.py
git commit -m "feat(sdk): introduce FrameBatch — the typed data carrier for the new pipeline"
```

---

### Task 3: Stage protocol + Pipeline class

**Files:**
- Create: `omotion/pipeline/pipeline.py`
- Create: `tests/test_pipeline/test_pipeline_class.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_pipeline/test_pipeline_class.py`:

```python
"""Tests for the Pipeline class — the stage-list driver."""

import numpy as np
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.pipeline import Pipeline, Stage


class _TagStage:
    """Test stage: appends its tag to the events list each time it runs."""
    name = "tag"
    def __init__(self, tag):
        self.tag = tag
        self.process_count = 0
        self.reset_count = 0
    def process(self, batch):
        batch.events.append(self.tag)
        self.process_count += 1
        return batch
    def reset(self):
        self.reset_count += 1


def _empty_batch():
    return FrameBatch(
        cam_ids=np.zeros(1, dtype=np.int8),
        frame_ids=np.zeros(1, dtype=np.uint8),
        raw_histograms=np.zeros((1, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((1, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(1, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


def test_pipeline_runs_stages_in_order():
    stages = [_TagStage("a"), _TagStage("b"), _TagStage("c")]
    pipeline = Pipeline(stages)
    batch = pipeline.process(_empty_batch())
    assert batch.events == ["a", "b", "c"]


def test_pipeline_reset_calls_reset_on_every_stage():
    stages = [_TagStage("a"), _TagStage("b")]
    pipeline = Pipeline(stages)
    pipeline.reset()
    assert stages[0].reset_count == 1
    assert stages[1].reset_count == 1


def test_pipeline_process_returns_the_batch():
    pipeline = Pipeline([_TagStage("a")])
    batch = _empty_batch()
    result = pipeline.process(batch)
    assert result is batch  # same object, mutated in place
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_pipeline_class.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'omotion.pipeline.pipeline'`

- [ ] **Step 3: Implement Stage protocol and Pipeline class**

Create `omotion/pipeline/pipeline.py`:

```python
"""Pipeline class and Stage protocol.

This module is pure plumbing — no science. Stages declare their behavior
through the Stage protocol; Pipeline drives them in order.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from .batch import FrameBatch


@runtime_checkable
class Stage(Protocol):
    """A single pipeline transformation.

    Stages mutate the FrameBatch in place and return it (for chainability).
    Stages may maintain cross-batch state (history, ring buffers, frame
    counters); reset() clears that state for a new scan or replay.
    """
    name: str

    def process(self, batch: FrameBatch) -> FrameBatch: ...

    def reset(self) -> None: ...


class Pipeline:
    """Ordered list of stages. Pure transformation, no I/O.

    The runner (omotion.pipeline.runner.ScanRunner) feeds FrameBatches in,
    Pipeline routes them through each stage in order, returns the result.
    """

    def __init__(self, stages: list[Stage]):
        self.stages = list(stages)

    def process(self, batch: FrameBatch) -> FrameBatch:
        for stage in self.stages:
            stage.process(batch)
        return batch

    def reset(self) -> None:
        """Call reset() on every stage. Done at scan start and after any
        stage exception during a scan."""
        for stage in self.stages:
            stage.reset()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_pipeline_class.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/pipeline.py tests/test_pipeline/test_pipeline_class.py
git commit -m "feat(sdk): add Pipeline class and Stage protocol"
```

---

### Task 4: Tee stage

**Files:**
- Create: `omotion/pipeline/tee.py`
- Create: `tests/test_pipeline/test_tee.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_pipeline/test_tee.py`:

```python
"""Tests for Tee — emits a LiveEmit event for the named channel."""

import numpy as np
from omotion.pipeline.batch import FrameBatch, LiveEmit
from omotion.pipeline.tee import Tee


def _batch_with_frame_types(types):
    n = len(types)
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        frame_type=np.array(types, dtype="<U8"),
    )


def test_tee_with_no_filter_emits_one_event_per_call():
    tee = Tee("raw", filter=None)
    batch = _batch_with_frame_types(["light"])
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert len(emits) == 1
    assert emits[0].channel == "raw"
    # Default payload is the batch itself; sinks decide which rows they care about.
    assert emits[0].payload is batch


def test_tee_with_filter_skips_emit_when_filter_excludes_all_frames():
    tee = Tee("live", filter=lambda ft: ft != "warmup" and ft != "stale")
    batch = _batch_with_frame_types(["warmup", "stale", "warmup"])
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert emits == []  # all frames excluded → no emit


def test_tee_with_filter_emits_when_any_frame_passes():
    tee = Tee("live", filter=lambda ft: ft != "warmup" and ft != "stale")
    batch = _batch_with_frame_types(["warmup", "light", "warmup"])
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert len(emits) == 1
    assert emits[0].channel == "live"


def test_tee_reset_is_a_noop():
    tee = Tee("raw", filter=None)
    tee.reset()  # should not raise
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_tee.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'omotion.pipeline.tee'`

- [ ] **Step 3: Implement Tee**

Create `omotion/pipeline/tee.py`:

```python
"""Tee — a stage that emits a LiveEmit event for a named channel.

Tee stages are positional markers in the pipeline. The runner reads
LiveEmit events from batch.events and dispatches the payload to sinks
subscribed to the named channel.

A Tee may carry an optional `filter` predicate over frame_type. If supplied
and no frame in the batch passes the filter, no LiveEmit is appended for
that batch. This is how the "live" tee excludes warmup and stale frames
while the "raw" tee includes them.
"""

from __future__ import annotations

from typing import Callable, Optional

from .batch import FrameBatch, LiveEmit


class Tee:
    """Emits one LiveEmit per batch (or zero, if filter excludes all frames).

    The default payload is the FrameBatch itself; sinks slice out the rows
    they care about based on the channel and their own logic.
    """

    def __init__(self, channel: str, *, filter: Optional[Callable[[str], bool]] = None):
        self.name = f"tee:{channel}"
        self.channel = channel
        self.filter = filter

    def process(self, batch: FrameBatch) -> FrameBatch:
        if self.filter is not None:
            if batch.frame_type is None:
                # Filter requested but Classify hasn't run — skip emit.
                # (In practice this shouldn't happen; Tee("raw") is the only
                # Tee before Classify and it passes filter=None.)
                return batch
            if not any(self.filter(ft) for ft in batch.frame_type):
                return batch
        batch.events.append(LiveEmit(channel=self.channel, payload=batch))
        return batch

    def reset(self) -> None:
        pass  # stateless
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_tee.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/tee.py tests/test_pipeline/test_tee.py
git commit -m "feat(sdk): add Tee stage for channel-based emission"
```

---

### Task 5: Sink protocol + ScanMetadata

**Files:**
- Create: `omotion/pipeline/sinks.py` (protocol + ScanMetadata; concrete CsvSink/ScanDBSink/QtUiSink come in later tasks)
- Create: `tests/test_pipeline/test_sinks_protocol.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_pipeline/test_sinks_protocol.py`:

```python
"""Tests for the Sink protocol and ScanMetadata."""

import pytest
from omotion.pipeline.sinks import Sink, ScanMetadata


def test_scan_metadata_carries_scan_identification_fields():
    meta = ScanMetadata(
        scan_id="abc-123",
        subject_id="subj-001",
        operator="bloodflow-app",
        started_at_iso="2026-05-22T10:00:00Z",
        duration_sec=300,
        left_camera_mask=0x66,
        right_camera_mask=0x66,
        reduced_mode=True,
        write_raw_csv=True,
        raw_csv_duration_sec=60.0,
    )
    assert meta.scan_id == "abc-123"
    assert meta.reduced_mode is True
    assert meta.raw_csv_duration_sec == 60.0


def test_scan_metadata_raw_csv_duration_can_be_none():
    """rawCsvDurationSec=None means 'write raw for the entire scan'."""
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T10:00:00Z",
        duration_sec=300, left_camera_mask=0, right_camera_mask=0xFF,
        reduced_mode=False, write_raw_csv=True, raw_csv_duration_sec=None,
    )
    assert meta.raw_csv_duration_sec is None


class _MinimalSink:
    """Implements the Sink protocol — bare minimum for runtime_checkable check."""
    channels = {"live"}
    def on_scan_start(self, meta): pass
    def consume(self, channel, payload): pass
    def on_complete(self): pass


def test_sink_protocol_runtime_check_accepts_a_minimal_implementation():
    sink = _MinimalSink()
    assert isinstance(sink, Sink)


def test_sink_protocol_runtime_check_rejects_missing_methods():
    class _Bad:
        channels = {"live"}
        # missing consume()
        def on_scan_start(self, meta): pass
        def on_complete(self): pass
    assert not isinstance(_Bad(), Sink)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_sinks_protocol.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'omotion.pipeline.sinks'`

- [ ] **Step 3: Implement Sink + ScanMetadata**

Create `omotion/pipeline/sinks.py`:

```python
"""Sink protocol + ScanMetadata.

Concrete sink implementations (CsvSink, ScanDBSink, QtUiSink) come later
in this PR — see tasks 25–27. This module establishes the protocol so
the runner and stages can refer to it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Protocol, runtime_checkable


@dataclass(frozen=True)
class ScanMetadata:
    """Per-scan metadata handed to every sink at on_scan_start.

    Sinks can use this for:
    - file naming (scan_id, subject_id, started_at_iso)
    - self-gating raw save (write_raw_csv, raw_csv_duration_sec)
    - choosing column shape (reduced_mode)
    """
    scan_id:               str
    subject_id:            str
    operator:              str
    started_at_iso:        str
    duration_sec:          int
    left_camera_mask:      int
    right_camera_mask:     int
    reduced_mode:          bool
    write_raw_csv:         bool
    raw_csv_duration_sec:  Optional[float]   # None ⇒ entire scan; float ⇒ first N seconds


@runtime_checkable
class Sink(Protocol):
    """A consumer of pipeline output. Sinks subscribe to channels and
    receive payloads via consume().

    Channels in this pipeline:
        "raw"          — per-frame, all non-stale frames including warmup
        "live"         — per-frame, best-effort corrected (light + dark)
        "rolling"      — per-frame, rolling-averaged for test/calibration
        "final"        — per-dark-interval, accurately corrected CorrectedBatch
        "diagnostics"  — out-of-band events (DarkIntegrityWarning, etc.)
    """
    channels: set[str]

    def on_scan_start(self, meta: ScanMetadata) -> None: ...

    def consume(self, channel: str, payload: Any) -> None: ...

    def on_complete(self) -> None: ...
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_sinks_protocol.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/sinks.py tests/test_pipeline/test_sinks_protocol.py
git commit -m "feat(sdk): add Sink protocol and ScanMetadata for the new pipeline"
```

---

## Phase B — Stateless stages (numpy-heavy)

These stages have no cross-batch state, so reset() is a no-op. Each is fundamentally a small numpy operation; the test pattern is "construct a batch with known inputs, run the stage, assert the math matches a scalar reference implementation."

---

### Task 6: NoiseFloorStage

**Files:**
- Create: `omotion/pipeline/stages/noise_floor.py`
- Create: `tests/test_pipeline/test_noise_floor_stage.py`

- [ ] **Step 1: Write the failing test**

```python
"""NoiseFloorStage — zero histogram bins whose count is below threshold."""

import numpy as np
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.noise_floor import NoiseFloorStage


def _batch_with_histogram(hist):
    """hist must be shape (N, 2, 8, 1024) uint32."""
    n = hist.shape[0]
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=hist,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


def test_zeros_bins_below_threshold():
    hist = np.array([[1, 5, 9, 10, 11, 20]] * (2 * 8 * 1) , dtype=np.uint32)
    hist = hist.reshape(1, 2, 8, 6)
    # Pad to (1, 2, 8, 1024) — match the real shape
    padded = np.zeros((1, 2, 8, 1024), dtype=np.uint32)
    padded[..., :6] = hist
    batch = _batch_with_histogram(padded)

    NoiseFloorStage(threshold=10).process(batch)

    # Bins 0..2 (values 1, 5, 9) were below threshold → zeroed.
    # Bins 3..5 (values 10, 11, 20) were ≥ threshold → unchanged.
    expected_first6 = np.array([0, 0, 0, 10, 11, 20], dtype=np.uint32)
    np.testing.assert_array_equal(batch.raw_histograms[0, 0, 0, :6], expected_first6)


def test_threshold_zero_is_noop():
    hist = np.full((1, 2, 8, 1024), 5, dtype=np.uint32)
    batch = _batch_with_histogram(hist.copy())
    NoiseFloorStage(threshold=0).process(batch)
    np.testing.assert_array_equal(batch.raw_histograms, hist)


def test_vectorized_across_all_dims():
    """Threshold applies to every (frame, side, cam) histogram independently
    without a Python loop."""
    hist = np.random.randint(0, 50, size=(7, 2, 8, 1024), dtype=np.uint32)
    batch = _batch_with_histogram(hist.copy())
    NoiseFloorStage(threshold=15).process(batch)
    # Every bin below 15 should now be 0; every bin ≥ 15 unchanged.
    expected = np.where(hist < 15, np.uint32(0), hist)
    np.testing.assert_array_equal(batch.raw_histograms, expected)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_noise_floor_stage.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Implement NoiseFloorStage**

Create `omotion/pipeline/stages/noise_floor.py`:

```python
"""NoiseFloorStage — zero histogram bins below a count threshold.

See docs/SciencePipeline.md §5: bins with very low counts are dominated by
readout / quantization noise rather than real photon counts, and contribute
disproportionately to the variance. Zeroing them before moment computation
suppresses this noise contribution.

Mutates raw_histograms in place. Vectorized across all (frame, side, cam)
in a single numpy call.
"""

from __future__ import annotations

import numpy as np

from ..batch import FrameBatch


class NoiseFloorStage:
    name = "noise_floor"

    def __init__(self, threshold: int = 10):
        self.threshold = int(threshold)

    def process(self, batch: FrameBatch) -> FrameBatch:
        if self.threshold <= 0:
            return batch
        np.putmask(batch.raw_histograms, batch.raw_histograms < self.threshold, 0)
        return batch

    def reset(self) -> None:
        pass  # stateless
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_noise_floor_stage.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/stages/noise_floor.py tests/test_pipeline/test_noise_floor_stage.py
git commit -m "feat(sdk): NoiseFloorStage — vectorized bin-count threshold zeroing"
```

---

### Task 7: MomentsStage (the big numpy win)

**Files:**
- Create: `omotion/pipeline/stages/moments.py`
- Create: `tests/test_pipeline/test_moments_stage.py`

- [ ] **Step 1: Write the failing test**

```python
"""MomentsStage — vectorized mean, std, contrast from histograms.

Cross-checks against a scalar reference per camera to verify the einsum
produces the same numbers as the scalar dot products used today.
"""

import numpy as np
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.moments import MomentsStage


def _batch_with_histogram(hist):
    n = hist.shape[0]
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=hist,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


def _scalar_reference(h):
    """Reference per-camera dot-product implementation (matches today's worker)."""
    bins = np.arange(1024)
    counts = h.sum()
    u1 = float(np.dot(h, bins) / counts) if counts > 0 else 0.0
    u2 = float(np.dot(h, bins ** 2) / counts) if counts > 0 else 0.0
    var = max(0.0, u2 - u1 ** 2)
    return u1, np.sqrt(var), (np.sqrt(var) / u1 if u1 > 0 else float("nan"))


def test_moments_match_scalar_reference():
    np.random.seed(42)
    hist = np.random.poisson(1000, size=(3, 2, 8, 1024)).astype(np.uint32)
    batch = _batch_with_histogram(hist)
    MomentsStage().process(batch)

    for n in range(3):
        for s in range(2):
            for c in range(8):
                ref_u1, ref_std, ref_K = _scalar_reference(hist[n, s, c])
                np.testing.assert_allclose(batch.mean_raw[n, s, c],     ref_u1,  rtol=1e-5)
                np.testing.assert_allclose(batch.std_raw[n, s, c],      ref_std, rtol=1e-5)
                np.testing.assert_allclose(batch.contrast_raw[n, s, c], ref_K,   rtol=1e-5, equal_nan=True)


def test_variance_clamps_at_zero_for_pathological_input():
    """If u2 < u1² (shouldn't happen but float precision can cause it),
    std must clamp to 0 rather than producing NaN."""
    # Single-bin histogram: all counts in bin 100 → variance is exactly 0,
    # but float roundoff in u2 - u1² might produce a tiny negative.
    hist = np.zeros((1, 2, 8, 1024), dtype=np.uint32)
    hist[..., 100] = 1000
    batch = _batch_with_histogram(hist)
    MomentsStage().process(batch)
    assert np.all(batch.std_raw == 0.0)
    np.testing.assert_allclose(batch.mean_raw, 100.0, rtol=1e-6)


def test_zero_count_frame_produces_nan_contrast_not_division_error():
    """If a frame has zero counts (shouldn't happen post-validation but
    test defensively), mean is undefined → contrast = NaN. No exception."""
    hist = np.zeros((1, 2, 8, 1024), dtype=np.uint32)
    batch = _batch_with_histogram(hist)
    MomentsStage().process(batch)
    assert np.all(np.isnan(batch.contrast_raw))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_moments_stage.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Implement MomentsStage**

Create `omotion/pipeline/stages/moments.py`:

```python
"""MomentsStage — vectorized first/second moment + std + contrast.

See docs/SciencePipeline.md §6 for the math:
    μ₁ = Σ(k · h_k) / Σh_k        first moment (mean bin index)
    μ₂ = Σ(k² · h_k) / Σh_k       second moment
    σ² = max(0, μ₂ − μ₁²)         variance, clamped non-negative
    σ  = √σ²
    K  = σ / μ₁                   contrast (NaN if μ₁ ≤ 0)

Vectorized via np.einsum over (N, 2, 8, 1024). Replaces a per-camera Python
loop in today's worker. Estimated speedup: 20–50× at typical batch sizes.
"""

from __future__ import annotations

import numpy as np

from ..batch import FrameBatch


class MomentsStage:
    name = "moments"

    # Precomputed bin index arrays — shared across all instances.
    _BIN_VALUES    = np.arange(1024, dtype=np.float64)
    _BIN_VALUES_SQ = _BIN_VALUES ** 2

    def process(self, batch: FrameBatch) -> FrameBatch:
        h = batch.raw_histograms                                    # (N, 2, 8, 1024) uint32
        counts = h.sum(axis=-1)                                     # (N, 2, 8)

        # Avoid divide-by-zero — replace zero counts with 1 for the division,
        # then clobber the result with NaN where counts were actually 0.
        safe_counts = np.where(counts > 0, counts, 1).astype(np.float64)

        u1 = np.einsum('nsci,i->nsc', h, self._BIN_VALUES)    / safe_counts
        u2 = np.einsum('nsci,i->nsc', h, self._BIN_VALUES_SQ) / safe_counts

        var = np.maximum(u2 - u1 ** 2, 0.0)
        std = np.sqrt(var)

        # Mark frames with zero counts as NaN mean (contrast computation will
        # also produce NaN since mean is NaN).
        mean = np.where(counts > 0, u1, np.nan)

        # Contrast: NaN where mean ≤ 0 (includes count=0 frames) — matches
        # today's behavior where contrast is undefined for empty histograms.
        with np.errstate(divide='ignore', invalid='ignore'):
            contrast = np.where(mean > 0, std / mean, np.nan)

        batch.mean_raw     = mean.astype(np.float32)
        batch.std_raw      = std.astype(np.float32)
        batch.contrast_raw = contrast.astype(np.float32)
        return batch

    def reset(self) -> None:
        pass  # stateless
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_moments_stage.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/stages/moments.py tests/test_pipeline/test_moments_stage.py
git commit -m "feat(sdk): MomentsStage — einsum-vectorized moments over (N, 2, 8, 1024)"
```

---

### Task 8: PedestalSubtractionStage + SensorPedestals helper

**Files:**
- Create: `omotion/pipeline/pedestal.py` (the SensorPedestals helper)
- Create: `omotion/pipeline/stages/pedestal_sub.py`
- Create: `tests/test_pipeline/test_pedestal_stage.py`

- [ ] **Step 1: Write the failing tests**

```python
"""PedestalSubtractionStage + SensorPedestals — per-side pedestal applied to mean_raw."""

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.pedestal import SensorPedestals, pedestal_for_fw
from omotion.pipeline.stages.pedestal_sub import PedestalSubtractionStage


def test_pedestal_for_fw_below_1_5_3_returns_64():
    assert pedestal_for_fw((1, 5, 2)) == 64.0
    assert pedestal_for_fw((1, 0, 0)) == 64.0


def test_pedestal_for_fw_1_5_3_or_above_returns_128():
    assert pedestal_for_fw((1, 5, 3)) == 128.0
    assert pedestal_for_fw((1, 6, 0)) == 128.0
    assert pedestal_for_fw((2, 0, 0)) == 128.0


def test_sensor_pedestals_per_side_independent():
    """Dual sensor with mixed firmware — left old, right new — gets two pedestals."""

    class _FakeSensor:
        def __init__(self, version):
            self.version = version

    peds = SensorPedestals.from_sensors(
        left=_FakeSensor(version=(1, 5, 2)),    # → 64.0
        right=_FakeSensor(version=(1, 5, 3)),   # → 128.0
    )
    assert peds.left == 64.0
    assert peds.right == 128.0


def test_pedestal_stage_subtracts_per_side_pedestal_and_clamps_at_zero():
    n = 2
    mean_raw = np.array([
        # frame 0: left = [100, 100, ...], right = [200, 200, ...]
        [[100] * 8, [200] * 8],
        # frame 1: left = [50, 50, ...], right = [10, 10, ...] (lower than pedestal!)
        [[50]  * 8, [10]  * 8],
    ], dtype=np.float32)

    batch = FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        mean_raw=mean_raw,
    )

    peds = SensorPedestals(left=64.0, right=128.0)
    PedestalSubtractionStage(pedestals=peds).process(batch)

    expected = np.array([
        # frame 0: 100 - 64 = 36 ; 200 - 128 = 72
        [[36] * 8, [72] * 8],
        # frame 1: max(0, 50 - 64) = 0 ; max(0, 10 - 128) = 0  (clamped)
        [[0]  * 8, [0]  * 8],
    ], dtype=np.float32)
    np.testing.assert_array_equal(batch.display_mean, expected)
    # mean_raw must be untouched — dark correction needs the un-pedestal-subtracted value
    np.testing.assert_array_equal(batch.mean_raw, mean_raw)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_pipeline/test_pedestal_stage.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Implement SensorPedestals + PedestalSubtractionStage**

Create `omotion/pipeline/pedestal.py`:

```python
"""Per-side, firmware-version-keyed pedestal lookup.

The OV2312 sensor's ADC settles at a different baseline depending on the
sensor module's firmware version:
    fw ≤ 1.5.2  →  pedestal = 64.0 DN
    fw ≥ 1.5.3  →  pedestal = 128.0 DN

In the legacy SDK this lookup mutates a global module constant
(omotion.MotionProcessing.PEDESTAL_HEIGHT). That breaks dual-sensor
systems running mixed firmware versions. This module replaces the global
with a per-pipeline-instance object carrying both sides' values.
"""

from __future__ import annotations

from dataclasses import dataclass


def pedestal_for_fw(version: tuple[int, int, int]) -> float:
    """Return the pedestal height (in DN) for a given sensor firmware version."""
    if version <= (1, 5, 2):
        return 64.0
    return 128.0


@dataclass(frozen=True)
class SensorPedestals:
    """Per-side pedestal values to feed into PedestalSubtractionStage."""
    left:  float
    right: float

    @classmethod
    def from_sensors(cls, *, left, right) -> "SensorPedestals":
        """Build from two MotionSensor handles.

        Each sensor handle is expected to expose a `.version` attribute
        of the form (major, minor, patch) — populated by
        MotionSensor.get_version() during connection.
        """
        return cls(
            left=pedestal_for_fw(left.version),
            right=pedestal_for_fw(right.version),
        )
```

Create `omotion/pipeline/stages/pedestal_sub.py`:

```python
"""PedestalSubtractionStage — produce display_mean from mean_raw, per-side.

See docs/SciencePipeline.md §7.1 for the rationale. The raw mean (with
pedestal) is preserved on the batch because the dark-correction path
uses it directly (the pedestal cancels in the dark-light subtraction).
"""

from __future__ import annotations

import numpy as np

from ..batch import FrameBatch
from ..pedestal import SensorPedestals


class PedestalSubtractionStage:
    name = "pedestal_subtraction"

    def __init__(self, pedestals: SensorPedestals):
        # Shape (1, 2, 1) — broadcasts across (N, 2, 8)
        self._pedestal = np.array(
            [pedestals.left, pedestals.right], dtype=np.float32
        ).reshape(1, 2, 1)

    def process(self, batch: FrameBatch) -> FrameBatch:
        batch.display_mean = np.maximum(
            0.0, batch.mean_raw - self._pedestal
        ).astype(np.float32)
        return batch

    def reset(self) -> None:
        pass  # stateless
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_pedestal_stage.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/pedestal.py omotion/pipeline/stages/pedestal_sub.py tests/test_pipeline/test_pedestal_stage.py
git commit -m "feat(sdk): SensorPedestals + PedestalSubtractionStage with per-side FW-version-keyed lookup"
```

---

### Task 9: ShotNoiseCorrectionStage

**Files:**
- Create: `omotion/pipeline/stages/shot_noise.py`
- Create: `tests/test_pipeline/test_shot_noise_stage.py`

- [ ] **Step 1: Write the failing test**

```python
"""ShotNoiseCorrectionStage — subtract Poisson variance from corrected variance.

Operates on the live (realtime-corrected) path. See SciencePipeline.md §7.4.2
for the formula:
    σ²_shot = ADC_GAIN · g_cam · max(0, mean_dc_rt)
    std_sn_rt = √max(0, std_dc_rt² − σ²_shot)
"""

import numpy as np
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.shot_noise import ShotNoiseCorrectionStage


# Canonical constants — match SciencePipeline.md §8.3 / §14.
ADC_GAIN = (1024 - 64) / 11_000      # ≈ 0.0873
CAMERA_GAIN_MAP = np.array([16, 4, 2, 1, 1, 2, 4, 16], dtype=np.float32)


def _batch_with_dc_rt(mean_dc, std_dc):
    """mean_dc and std_dc must be shape (N, 2, 8) float32."""
    n = mean_dc.shape[0]
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        mean_dc_rt=mean_dc,
        std_dc_rt=std_dc,
    )


def test_subtracts_shot_noise_variance_per_camera():
    mean = np.full((1, 2, 8), 100.0, dtype=np.float32)
    std  = np.full((1, 2, 8), 10.0,  dtype=np.float32)  # var = 100
    batch = _batch_with_dc_rt(mean, std)

    ShotNoiseCorrectionStage(adc_gain=ADC_GAIN, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    expected_shot_var = ADC_GAIN * 100.0 * CAMERA_GAIN_MAP   # (8,)
    expected_corr_var = np.maximum(0.0, 100.0 - expected_shot_var)
    expected_std = np.sqrt(expected_corr_var).astype(np.float32)
    # Both sides should receive the same per-cam correction
    for s in range(2):
        np.testing.assert_allclose(batch.std_sn_rt[0, s], expected_std, rtol=1e-5)


def test_negative_corrected_variance_clamps_to_zero_std():
    """When dark+shot subtraction over-corrects, σ² goes negative; clamp keeps
    std at 0 instead of producing NaN."""
    mean = np.full((1, 2, 8), 1000.0, dtype=np.float32)  # large mean
    std  = np.full((1, 2, 8), 1.0,    dtype=np.float32)  # tiny variance
    batch = _batch_with_dc_rt(mean, std)

    ShotNoiseCorrectionStage(adc_gain=ADC_GAIN, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    # ADC_GAIN * 1000 * 16 ≈ 1397, way bigger than var=1 → clamped
    assert np.all(batch.std_sn_rt[0, 0, 0] == 0.0)


def test_contrast_computed_with_corrected_std_and_mean():
    mean = np.full((1, 2, 8), 100.0, dtype=np.float32)
    std  = np.full((1, 2, 8), 10.0,  dtype=np.float32)
    batch = _batch_with_dc_rt(mean, std)

    ShotNoiseCorrectionStage(adc_gain=ADC_GAIN, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    # contrast_sn_rt = std_sn_rt / mean_dc_rt
    expected = batch.std_sn_rt / 100.0
    np.testing.assert_allclose(batch.contrast_sn_rt, expected, rtol=1e-5)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_shot_noise_stage.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Implement ShotNoiseCorrectionStage**

Create `omotion/pipeline/stages/shot_noise.py`:

```python
"""ShotNoiseCorrectionStage — Poisson-variance subtraction.

See SciencePipeline.md §8.3 for the model:
    σ²_shot = ADC_GAIN · g_cam · max(0, mean_dc_rt)

where ADC_GAIN converts DN to electrons and g_cam is the per-camera analog
gain (positions 0,7 → 16; 1,6 → 4; 2,5 → 2; 3,4 → 1). Subtracting the
expected shot-noise variance from the dark-corrected variance isolates
the speckle variance.

Operates on the live (realtime-corrected) path; the batched corrected
path applies the same expression downstream (§8.3 of the SciencePipeline
doc). Both paths share this stage in the new design.
"""

from __future__ import annotations

import numpy as np

from ..batch import FrameBatch


class ShotNoiseCorrectionStage:
    name = "shot_noise_correction"

    def __init__(self, adc_gain: float, camera_gain_map: np.ndarray):
        self.adc_gain = float(adc_gain)
        # Reshape to (1, 1, 8) for broadcasting across (N, 2, 8)
        self._gain_map = np.asarray(camera_gain_map, dtype=np.float32).reshape(1, 1, 8)

    def process(self, batch: FrameBatch) -> FrameBatch:
        mean = batch.mean_dc_rt
        std  = batch.std_dc_rt
        var  = std.astype(np.float64) ** 2

        shot_var = self.adc_gain * np.maximum(0.0, mean.astype(np.float64)) * self._gain_map
        corrected_var = np.maximum(0.0, var - shot_var)
        std_sn = np.sqrt(corrected_var).astype(np.float32)

        with np.errstate(divide='ignore', invalid='ignore'):
            contrast = np.where(mean > 0, std_sn / mean, np.float32(0.0))

        batch.std_sn_rt      = std_sn
        batch.contrast_sn_rt = contrast.astype(np.float32)
        return batch

    def reset(self) -> None:
        pass  # stateless
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_shot_noise_stage.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/stages/shot_noise.py tests/test_pipeline/test_shot_noise_stage.py
git commit -m "feat(sdk): ShotNoiseCorrectionStage — Poisson variance subtraction"
```

---

### Task 10: RollingAverageStage

**Files:**
- Create: `omotion/pipeline/stages/rolling_avg.py`
- Create: `tests/test_pipeline/test_rolling_avg_stage.py`

- [ ] **Step 1: Write the failing test**

```python
"""RollingAverageStage — smooth BFI/BVI over a configurable window.

Window state persists across batches (the most recent N samples are
remembered between calls to process()). On reset(), the window clears.
"""

import numpy as np
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.rolling_avg import RollingAverageStage


def _batch_with_bfi_bvi(bfi, bvi):
    """bfi and bvi must be shape (N, 2, 8) float32."""
    n = bfi.shape[0]
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        bfi_live=bfi.astype(np.float32),
        bvi_live=bvi.astype(np.float32),
    )


def test_rolling_avg_window_1_is_identity():
    bfi = np.arange(5 * 2 * 8, dtype=np.float32).reshape(5, 2, 8)
    bvi = bfi + 100
    batch = _batch_with_bfi_bvi(bfi, bvi)

    RollingAverageStage(window=1).process(batch)

    np.testing.assert_array_equal(batch.bfi_rolling, bfi)
    np.testing.assert_array_equal(batch.bvi_rolling, bvi)


def test_rolling_avg_window_3_averages_last_3():
    # 5 frames; for each frame n the rolling avg over last 3 is
    #   n=0: avg of [frame0]              (only 1 sample available so far)
    #   n=1: avg of [frame0, frame1]      (only 2)
    #   n=2: avg of [frame0, frame1, frame2]
    #   n=3: avg of [frame1, frame2, frame3]
    #   n=4: avg of [frame2, frame3, frame4]
    bfi = np.arange(5, dtype=np.float32).reshape(5, 1, 1) * np.ones((1, 2, 8), dtype=np.float32)
    bvi = bfi * 2
    batch = _batch_with_bfi_bvi(bfi, bvi)

    RollingAverageStage(window=3).process(batch)

    expected = np.array([0.0, 0.5, 1.0, 2.0, 3.0], dtype=np.float32).reshape(5, 1, 1) * np.ones((1, 2, 8))
    np.testing.assert_allclose(batch.bfi_rolling, expected, rtol=1e-6)


def test_window_state_persists_across_batches():
    # First batch: 2 samples; window=3.
    # Second batch (after process call): 1 more sample.
    # The 3rd sample's rolling avg should include the last 2 from batch 1.
    stage = RollingAverageStage(window=3)

    bfi1 = np.array([[10.0], [20.0]], dtype=np.float32).reshape(2, 1, 1) * np.ones((1, 2, 8), dtype=np.float32)
    bvi1 = bfi1 * 2
    batch1 = _batch_with_bfi_bvi(bfi1, bvi1)
    stage.process(batch1)

    bfi2 = np.array([[30.0]], dtype=np.float32).reshape(1, 1, 1) * np.ones((1, 2, 8), dtype=np.float32)
    bvi2 = bfi2 * 2
    batch2 = _batch_with_bfi_bvi(bfi2, bvi2)
    stage.process(batch2)

    # The 3rd frame's rolling avg = (10 + 20 + 30) / 3 = 20.0
    np.testing.assert_allclose(batch2.bfi_rolling[0], np.full((2, 8), 20.0), rtol=1e-6)


def test_reset_clears_window():
    stage = RollingAverageStage(window=3)
    bfi1 = np.full((2, 2, 8), 100.0, dtype=np.float32)
    bvi1 = bfi1 * 2
    stage.process(_batch_with_bfi_bvi(bfi1, bvi1))

    stage.reset()

    # After reset, window is empty — a single sample's rolling avg = itself
    bfi2 = np.full((1, 2, 8), 50.0, dtype=np.float32)
    bvi2 = bfi2 * 2
    batch2 = _batch_with_bfi_bvi(bfi2, bvi2)
    stage.process(batch2)
    np.testing.assert_allclose(batch2.bfi_rolling, np.full((1, 2, 8), 50.0))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_rolling_avg_stage.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Implement RollingAverageStage**

Create `omotion/pipeline/stages/rolling_avg.py`:

```python
"""RollingAverageStage — sliding-window mean of bfi_live, bvi_live.

State carried across batches: a ring of the most recent N samples'
(bfi_live, bvi_live) values per (2, 8) camera grid. On a new batch,
incoming samples are appended one at a time and the rolling mean is
computed at each insertion.

reset() clears the ring (called at scan start and after any stage
exception during a scan).
"""

from __future__ import annotations

from collections import deque
from typing import Deque

import numpy as np

from ..batch import FrameBatch


class RollingAverageStage:
    name = "rolling_average"

    def __init__(self, window: int):
        if window < 1:
            raise ValueError(f"window must be >= 1, got {window}")
        self.window = int(window)
        self._bfi_window: Deque[np.ndarray] = deque(maxlen=self.window)
        self._bvi_window: Deque[np.ndarray] = deque(maxlen=self.window)

    def process(self, batch: FrameBatch) -> FrameBatch:
        n = batch.bfi_live.shape[0]
        bfi_rolling = np.empty_like(batch.bfi_live)
        bvi_rolling = np.empty_like(batch.bvi_live)

        for i in range(n):
            self._bfi_window.append(batch.bfi_live[i])
            self._bvi_window.append(batch.bvi_live[i])
            bfi_rolling[i] = np.mean(np.stack(self._bfi_window, axis=0), axis=0)
            bvi_rolling[i] = np.mean(np.stack(self._bvi_window, axis=0), axis=0)

        batch.bfi_rolling = bfi_rolling
        batch.bvi_rolling = bvi_rolling
        return batch

    def reset(self) -> None:
        self._bfi_window.clear()
        self._bvi_window.clear()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_rolling_avg_stage.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/stages/rolling_avg.py tests/test_pipeline/test_rolling_avg_stage.py
git commit -m "feat(sdk): RollingAverageStage — sliding-window smoothing for the rolling tee"
```

---

### Task 11: SideAveragingStage

**Files:**
- Create: `omotion/pipeline/stages/side_avg.py`
- Create: `tests/test_pipeline/test_side_avg_stage.py`

- [ ] **Step 1: Write the failing test**

```python
"""SideAveragingStage — average BFI/BVI across cameras into per-side values.

Gated by reduced_mode at construction time. When disabled, the stage is a
no-op. When enabled, it averages over the 8 cameras per side, using only
the cameras whose bit is set in the per-side camera mask.
"""

import numpy as np
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.side_avg import SideAveragingStage


def _batch_with_bfi_bvi(bfi, bvi):
    n = bfi.shape[0]
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        bfi_live=bfi.astype(np.float32),
        bvi_live=bvi.astype(np.float32),
    )


def test_disabled_stage_is_noop():
    bfi = np.full((3, 2, 8), 5.0, dtype=np.float32)
    bvi = np.full((3, 2, 8), 7.0, dtype=np.float32)
    batch = _batch_with_bfi_bvi(bfi, bvi)
    SideAveragingStage(enabled=False, left_camera_mask=0xFF, right_camera_mask=0xFF).process(batch)
    assert batch.bfi_live_side is None
    assert batch.bvi_live_side is None


def test_enabled_with_full_mask_averages_all_8():
    bfi = np.arange(2 * 2 * 8, dtype=np.float32).reshape(2, 2, 8)
    bvi = bfi + 100
    batch = _batch_with_bfi_bvi(bfi, bvi)
    SideAveragingStage(enabled=True, left_camera_mask=0xFF, right_camera_mask=0xFF).process(batch)

    # mean over axis=2 (the 8-camera axis) → shape (N, 2)
    expected_bfi = bfi.mean(axis=2)
    expected_bvi = bvi.mean(axis=2)
    np.testing.assert_allclose(batch.bfi_live_side, expected_bfi, rtol=1e-6)
    np.testing.assert_allclose(batch.bvi_live_side, expected_bvi, rtol=1e-6)


def test_enabled_with_partial_mask_only_averages_active_cams():
    # mask 0x66 = binary 01100110 → cams 1, 2, 5, 6 active (4 cams)
    bfi = np.array([[1, 2, 3, 4, 5, 6, 7, 8]] * 2, dtype=np.float32).reshape(1, 2, 8)
    bvi = bfi * 10
    batch = _batch_with_bfi_bvi(bfi, bvi)
    SideAveragingStage(enabled=True, left_camera_mask=0x66, right_camera_mask=0x66).process(batch)

    # Mean of bfi at cams 1,2,5,6 → values 2, 3, 6, 7 → mean = 4.5
    np.testing.assert_allclose(batch.bfi_live_side[0, 0], 4.5, rtol=1e-6)
    np.testing.assert_allclose(batch.bfi_live_side[0, 1], 4.5, rtol=1e-6)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_side_avg_stage.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Implement SideAveragingStage**

Create `omotion/pipeline/stages/side_avg.py`:

```python
"""SideAveragingStage — per-side averaging for reduced-mode display.

See SciencePipeline.md §16 for the reduced-mode rationale. In the legacy
SDK this averaging lives in ScanWorkflow's _on_uncorrected_sample / 
_on_corrected_batch interception. In the new design it's an explicit
stage so the data flow is one coherent diagram.

When reduced_mode is False, the stage is a no-op. When True, it averages
the active cameras (those whose bit is set in the per-side mask) into
per-side scalar bfi_live_side / bvi_live_side arrays of shape (N, 2).
"""

from __future__ import annotations

import numpy as np

from ..batch import FrameBatch


def _mask_to_cam_indices(mask: int) -> np.ndarray:
    """Return the camera positions (0..7) whose bit is set in mask."""
    return np.array([i for i in range(8) if mask & (1 << i)], dtype=np.int8)


class SideAveragingStage:
    name = "side_averaging"

    def __init__(self, *, enabled: bool, left_camera_mask: int, right_camera_mask: int):
        self.enabled = bool(enabled)
        self._left_cams  = _mask_to_cam_indices(left_camera_mask)
        self._right_cams = _mask_to_cam_indices(right_camera_mask)

    def process(self, batch: FrameBatch) -> FrameBatch:
        if not self.enabled:
            return batch

        # batch.bfi_live shape: (N, 2, 8) → produce (N, 2) by averaging
        # active cams on each side.
        n = batch.bfi_live.shape[0]
        bfi_side = np.zeros((n, 2), dtype=np.float32)
        bvi_side = np.zeros((n, 2), dtype=np.float32)

        if len(self._left_cams) > 0:
            bfi_side[:, 0] = batch.bfi_live[:, 0, self._left_cams].mean(axis=1)
            bvi_side[:, 0] = batch.bvi_live[:, 0, self._left_cams].mean(axis=1)
        if len(self._right_cams) > 0:
            bfi_side[:, 1] = batch.bfi_live[:, 1, self._right_cams].mean(axis=1)
            bvi_side[:, 1] = batch.bvi_live[:, 1, self._right_cams].mean(axis=1)

        batch.bfi_live_side = bfi_side
        batch.bvi_live_side = bvi_side
        return batch

    def reset(self) -> None:
        pass  # stateless
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_side_avg_stage.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/stages/side_avg.py tests/test_pipeline/test_side_avg_stage.py
git commit -m "feat(sdk): SideAveragingStage — per-side averaging for reduced-mode display"
```

---

## Design refinement vs. spec

The spec listed a separate `HistogramParseStage` for "CRC + sum check." During planning we determined parsing belongs in the Source (binary → numpy arrays happens once at ingest, not as a pipeline stage). All three sources (LiveUsbSource, CsvReplaySource, DbReplaySource) produce already-parsed `FrameBatch` objects. CRC validation lives in the transport layer; row-sum validation is the source's responsibility. The pipeline assumes its input is structurally valid.

**Effect on stage list:** 9 stages instead of 10. `FrameClassificationStage` is now the first stage, and it owns frame-ID unwrapping internally (which the spec implicitly described as part of classification).

---

## Phase C — Frame classification (1 task)

### Task 12: FrameClassificationStage (with FrameUnwrapper internal)

**Files:**
- Create: `omotion/pipeline/stages/classify.py`
- Create: `tests/test_pipeline/test_classify_stage.py`

The stage owns three responsibilities:
1. **Frame ID unwrapping** — 8-bit rolling → monotonic absolute (per `(side, cam_id)`)
2. **Stale-packet labeling** — if the first frame received for any `(side, cam_id)` has `raw_frame_id != 1`, label it `stale`
3. **Schedule-based classification** — label `warmup` (abs_frame_id ≤ discard_count), `dark` (per the dark schedule), or `light` (everything else)

- [ ] **Step 1: Write the failing test**

```python
"""FrameClassificationStage — abs_frame_id unwrap + frame_type labeling."""

import numpy as np
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.classify import FrameClassificationStage


def _batch_with_raw_ids(raw_ids_per_side_cam):
    """raw_ids_per_side_cam: dict {(side, cam): list_of_raw_frame_ids}.
    
    Synthesizes a batch where each row corresponds to one (side, cam) sample.
    For simplicity each row has only one populated cam slot; others are -1.
    """
    rows = []
    for (side_idx, cam_id), raw_ids in raw_ids_per_side_cam.items():
        for raw_id in raw_ids:
            rows.append((side_idx, cam_id, raw_id))

    n = len(rows)
    cam_ids = np.array([r[1] for r in rows], dtype=np.int8)
    frame_ids = np.array([r[2] for r in rows], dtype=np.uint8)
    # For this synthetic test, use cam_ids array's side via a separate side index;
    # in production, batch carries side via the (N, 2, 8) shape — here we just
    # populate the slot the frame came from.
    raw_hists = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    for i, (s, c, _) in enumerate(rows):
        raw_hists[i, s, c, 0] = 1  # marker
    return FrameBatch(
        cam_ids=cam_ids,
        frame_ids=frame_ids,
        raw_histograms=raw_hists,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


def test_first_frame_with_raw_id_1_is_warmup_not_stale():
    """raw_frame_id == 1 is the expected first frame; abs_frame_id = 1 → warmup."""
    batch = _batch_with_raw_ids({(0, 0): [1, 2, 3]})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    np.testing.assert_array_equal(batch.abs_frame_ids, [1, 2, 3])
    np.testing.assert_array_equal(batch.frame_type, ["warmup", "warmup", "warmup"])


def test_first_frame_with_raw_id_other_than_1_is_stale():
    """raw_frame_id != 1 on the very first frame for (side, cam) indicates a
    stale packet from a previous scan."""
    batch = _batch_with_raw_ids({(0, 0): [42, 43, 44]})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    # Only the first frame is stale; subsequent frames proceed normally
    # (treated as starting from raw_id=42 → abs=42)
    assert batch.frame_type[0] == "stale"


def test_warmup_range_marks_first_9_as_warmup():
    raw_ids = list(range(1, 15))  # 1..14
    batch = _batch_with_raw_ids({(0, 0): raw_ids})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    expected = ["warmup"] * 9 + ["dark"] + ["light"] * 4    # abs=10 is the first dark
    np.testing.assert_array_equal(batch.frame_type, expected)


def test_dark_schedule_fires_at_intervals():
    # discard=9, dark_interval=10 → darks at abs 10, 11, 21, 31, 41, ...
    # (formula: n == discard+1 OR (n-1) mod dark_interval == 0)
    raw_ids = list(range(1, 25))   # abs 1..24
    batch = _batch_with_raw_ids({(0, 0): raw_ids})
    FrameClassificationStage(discard_count=9, dark_interval=10).process(batch)
    
    # abs 10 → dark (n == discard+1)
    # abs 11 → light  ((11-1) % 10 == 0 is True, but only if n > discard+1; the first
    #                  dark is special-cased at discard+1 itself, and the schedule
    #                  matches (n-1) mod dark_interval == 0 for n > discard+1)
    # Let's verify:  (n-1) mod 10 == 0 for n in {1, 11, 21}. Of those, n=1 is warmup
    # (caught by abs ≤ discard), n=11 and 21 are darks.
    # And n=10 is special-cased.
    expected_darks_at = {10, 11, 21}
    for i, raw_id in enumerate(raw_ids):
        is_dark = (i + 1) in expected_darks_at
        assert (batch.frame_type[i] == "dark") == is_dark, \
            f"abs={i+1}: got {batch.frame_type[i]}, expected dark={is_dark}"


def test_unwrap_handles_8bit_rollover():
    raw_ids = list(range(250, 256)) + list(range(0, 5))   # 250..255, 0..4
    batch = _batch_with_raw_ids({(0, 0): raw_ids})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    # After the rollover at 255→0, the absolute frame ID should continue
    # monotonically: 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260
    expected_abs = [250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260]
    np.testing.assert_array_equal(batch.abs_frame_ids, expected_abs)


def test_reset_clears_unwrapper_state():
    stage = FrameClassificationStage(discard_count=9, dark_interval=600)
    
    # First scan: frames 1..3
    batch1 = _batch_with_raw_ids({(0, 0): [1, 2, 3]})
    stage.process(batch1)
    assert batch1.abs_frame_ids[0] == 1

    # Reset, then second scan starting again with raw_id=1
    stage.reset()
    batch2 = _batch_with_raw_ids({(0, 0): [1, 2, 3]})
    stage.process(batch2)
    assert batch2.abs_frame_ids[0] == 1   # epoch counter cleared
    assert batch2.frame_type[0] == "warmup"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_classify_stage.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Implement FrameClassificationStage**

Create `omotion/pipeline/stages/classify.py`:

```python
"""FrameClassificationStage — abs_frame_id unwrap + frame_type labeling.

Per (side, cam_id) pair, the stage maintains a FrameUnwrapper (8-bit → 
monotonic absolute index) and a "first frame seen" guard. Each incoming
row in the batch is then labeled with one of:

    "warmup"  — abs_frame_id ∈ 1..discard_count (default 9)
    "dark"    — per the firmware's deterministic dark schedule
    "light"   — every other valid frame
    "stale"   — the very first frame for this (side, cam) had raw_id != 1
                (indicates packet from a previous scan)

See docs/SciencePipeline.md §3 (unwrapping) and §4 (classification).
"""

from __future__ import annotations

import numpy as np

from ..batch import FrameBatch


_FRAME_ID_MODULUS = 256
_FRAME_ROLLOVER_THRESHOLD = 128


class _FrameUnwrapper:
    """8-bit rolling → monotonic. One instance per (side, cam_id)."""

    __slots__ = ("epoch", "last_raw", "seen_first", "first_was_stale")

    def __init__(self):
        self.epoch = 0
        self.last_raw = -1
        self.seen_first = False
        self.first_was_stale = False

    def unwrap(self, raw_frame_id: int) -> int:
        """Return absolute frame ID. Updates internal state."""
        if not self.seen_first:
            self.seen_first = True
            self.first_was_stale = (raw_frame_id != 1)
            self.last_raw = raw_frame_id
            # First frame's absolute index == its raw value (so a stale
            # raw=42 produces abs=42, distinguishable from warmup).
            return raw_frame_id

        delta = (raw_frame_id - self.last_raw) & 0xFF
        if delta <= _FRAME_ROLLOVER_THRESHOLD and raw_frame_id < self.last_raw:
            self.epoch += 1
        self.last_raw = raw_frame_id
        return self.epoch * _FRAME_ID_MODULUS + raw_frame_id


class FrameClassificationStage:
    name = "frame_classification"

    def __init__(self, discard_count: int = 9, dark_interval: int = 600):
        self.discard_count = int(discard_count)
        self.dark_interval = int(dark_interval)
        # Keyed by (side_idx, cam_id) — populated lazily as frames arrive.
        self._unwrappers: dict[tuple[int, int], _FrameUnwrapper] = {}

    def process(self, batch: FrameBatch) -> FrameBatch:
        n = batch.frame_ids.shape[0]
        abs_ids = np.zeros(n, dtype=np.int64)
        types = np.empty(n, dtype="<U8")

        for i in range(n):
            cam_id = int(batch.cam_ids[i])
            raw_id = int(batch.frame_ids[i])
            # Determine which side the frame came from by looking at which
            # (side, cam) slot is populated in raw_histograms.
            side_idx = int(np.argmax(batch.raw_histograms[i].sum(axis=(-2, -1))))

            key = (side_idx, cam_id)
            unwrapper = self._unwrappers.get(key)
            if unwrapper is None:
                unwrapper = _FrameUnwrapper()
                self._unwrappers[key] = unwrapper

            abs_id = unwrapper.unwrap(raw_id)
            abs_ids[i] = abs_id

            # Classify
            if unwrapper.first_was_stale and abs_id == raw_id:
                types[i] = "stale"
            elif abs_id <= self.discard_count:
                types[i] = "warmup"
            elif self._is_dark(abs_id):
                types[i] = "dark"
            else:
                types[i] = "light"

        batch.abs_frame_ids = abs_ids
        batch.frame_type = types
        return batch

    def _is_dark(self, abs_id: int) -> bool:
        """Per SciencePipeline.md §4.2:
            n == discard_count + 1 OR (n > discard_count + 1 AND (n-1) mod dark_interval == 0)
        """
        if abs_id == self.discard_count + 1:
            return True
        if abs_id <= self.discard_count + 1:
            return False
        return (abs_id - 1) % self.dark_interval == 0

    def reset(self) -> None:
        self._unwrappers.clear()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_classify_stage.py -v`
Expected: 6 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/stages/classify.py tests/test_pipeline/test_classify_stage.py
git commit -m "feat(sdk): FrameClassificationStage — abs_id unwrap + warmup/stale/dark/light labeling"
```

---

## Phase D — Calibration (1 task)

### Task 13: BfiBviStage

**Files:**
- Create: `omotion/pipeline/stages/bfi_bvi.py`
- Create: `tests/test_pipeline/test_bfi_bvi_stage.py`

The stage applies the calibration mapping (`docs/SciencePipeline.md §9`) to the live (`mean_dc_rt`, `contrast_sn_rt`) and produces `bfi_live`, `bvi_live`. The same stage is used for the corrected-batch path when DarkCorrectionStage emits an `IntervalClosed` event (the runner re-runs downstream stages on the closed interval — see Task 19).

- [ ] **Step 1: Write the failing test**

```python
"""BfiBviStage — affine calibration map from (contrast, mean) to (BFI, BVI)."""

import numpy as np
from dataclasses import dataclass
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.bfi_bvi import BfiBviStage


@dataclass
class _Calibration:
    """Minimal Calibration stand-in matching the shape ScanWorkflow passes today."""
    c_min: np.ndarray   # (2, 8)
    c_max: np.ndarray   # (2, 8)
    i_min: np.ndarray   # (2, 8)
    i_max: np.ndarray   # (2, 8)


def _trivial_calibration():
    """C in [0, 1] → BFI ∈ [0, 10]; I in [0, 100] → BVI ∈ [0, 10]."""
    return _Calibration(
        c_min=np.zeros((2, 8), dtype=np.float32),
        c_max=np.ones((2, 8), dtype=np.float32),
        i_min=np.zeros((2, 8), dtype=np.float32),
        i_max=np.full((2, 8), 100.0, dtype=np.float32),
    )


def _batch_with_live_values(mean_dc, contrast_sn):
    n = mean_dc.shape[0]
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        mean_dc_rt=mean_dc,
        contrast_sn_rt=contrast_sn,
    )


def test_calibration_maps_full_range_to_zero_to_ten():
    # C_max → BFI = 0;  C_min → BFI = 10
    mean = np.full((1, 2, 8), 50.0, dtype=np.float32)
    contrast = np.full((1, 2, 8), 1.0, dtype=np.float32)   # at C_max
    batch = _batch_with_live_values(mean, contrast)
    BfiBviStage(calibration=_trivial_calibration()).process(batch)
    np.testing.assert_allclose(batch.bfi_live, 0.0, atol=1e-5)


def test_midpoint_contrast_maps_to_bfi_5():
    mean = np.full((1, 2, 8), 50.0, dtype=np.float32)
    contrast = np.full((1, 2, 8), 0.5, dtype=np.float32)   # midpoint
    batch = _batch_with_live_values(mean, contrast)
    BfiBviStage(calibration=_trivial_calibration()).process(batch)
    np.testing.assert_allclose(batch.bfi_live, 5.0, atol=1e-5)


def test_bvi_uses_mean_with_i_min_i_max():
    mean = np.full((1, 2, 8), 50.0, dtype=np.float32)
    contrast = np.full((1, 2, 8), 0.0, dtype=np.float32)
    batch = _batch_with_live_values(mean, contrast)
    BfiBviStage(calibration=_trivial_calibration()).process(batch)
    # mean=50 in [0, 100] → halfway → BVI = 5
    np.testing.assert_allclose(batch.bvi_live, 5.0, atol=1e-5)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_bfi_bvi_stage.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Implement BfiBviStage**

Create `omotion/pipeline/stages/bfi_bvi.py`:

```python
"""BfiBviStage — affine calibration map (contrast, mean) → (BFI, BVI).

See docs/SciencePipeline.md §9 for the math:
    BFI = (1 - (K - C_min) / (C_max - C_min)) * 10
    BVI = (1 - (mean - I_min) / (I_max - I_min)) * 10
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ..batch import FrameBatch


class BfiBviStage:
    name = "bfi_bvi"

    def __init__(self, calibration: Any):
        """`calibration` must expose c_min, c_max, i_min, i_max as (2, 8)
        ndarrays. The omotion.Calibration class satisfies this contract
        (see omotion/Calibration.py)."""
        # Reshape to (1, 2, 8) for broadcasting across (N, 2, 8) batches
        self._c_min = np.asarray(calibration.c_min, dtype=np.float32).reshape(1, 2, 8)
        self._c_max = np.asarray(calibration.c_max, dtype=np.float32).reshape(1, 2, 8)
        self._i_min = np.asarray(calibration.i_min, dtype=np.float32).reshape(1, 2, 8)
        self._i_max = np.asarray(calibration.i_max, dtype=np.float32).reshape(1, 2, 8)

    def process(self, batch: FrameBatch) -> FrameBatch:
        K  = batch.contrast_sn_rt
        m  = batch.mean_dc_rt

        with np.errstate(divide='ignore', invalid='ignore'):
            c_span = self._c_max - self._c_min
            i_span = self._i_max - self._i_min
            bfi = (1.0 - (K - self._c_min) / np.where(c_span > 0, c_span, 1)) * 10.0
            bvi = (1.0 - (m - self._i_min) / np.where(i_span > 0, i_span, 1)) * 10.0

        # Where calibration span is zero (fallback identity per SciencePipeline.md §9),
        # fall back to identity scaling: BFI = K * 10, BVI = m * 10.
        c_span_broadcast = np.broadcast_to(c_span, bfi.shape)
        i_span_broadcast = np.broadcast_to(i_span, bvi.shape)
        bfi = np.where(c_span_broadcast > 0, bfi, K * 10.0)
        bvi = np.where(i_span_broadcast > 0, bvi, m * 10.0)

        batch.bfi_live = bfi.astype(np.float32)
        batch.bvi_live = bvi.astype(np.float32)
        return batch

    def reset(self) -> None:
        pass  # stateless
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_bfi_bvi_stage.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/stages/bfi_bvi.py tests/test_pipeline/test_bfi_bvi_stage.py
git commit -m "feat(sdk): BfiBviStage — affine calibration map for the live stream"
```

---

## Phase E — Dark correction subsystem (5 tasks)

The dark correction code is the most complex part of the rewrite. The spec splits it into composable helper classes (one file: `omotion/pipeline/stages/dark.py`) plus the orchestrator stage. Each helper is independently tested.

### Task 14: DarkHistory ring buffer

**Files:**
- Modify: `omotion/pipeline/stages/dark.py` (create file — this task adds `DarkHistory`)
- Create: `tests/test_pipeline/test_dark_history.py`

`DarkHistory` is a per-camera ring buffer of recent dark observations. The realtime predictor reads from it; the batch interpolator pops closed intervals out of it.

- [ ] **Step 1: Write the failing test**

```python
"""DarkHistory — per-camera ring buffer of (timestamp_s, u1, std) observations."""

import pytest
from omotion.pipeline.stages.dark import DarkHistory


def test_history_starts_empty():
    h = DarkHistory(max_darks=4)
    assert h.size("left", 0) == 0
    assert h.is_empty("left", 0)


def test_append_then_size_increments():
    h = DarkHistory(max_darks=4)
    h.append("left", 0, t=0.0, u1=100.0, std=10.0)
    assert h.size("left", 0) == 1
    assert not h.is_empty("left", 0)


def test_ring_buffer_bounded_by_max_darks():
    h = DarkHistory(max_darks=3)
    for i in range(5):
        h.append("left", 0, t=float(i), u1=100.0 + i, std=10.0)
    # Only the most recent 3 are kept
    assert h.size("left", 0) == 3
    entries = h.recent("left", 0, n=10)
    assert len(entries) == 3
    assert [e.t for e in entries] == [2.0, 3.0, 4.0]


def test_recent_n_returns_last_n_in_chronological_order():
    h = DarkHistory(max_darks=10)
    for i in range(5):
        h.append("right", 7, t=float(i), u1=100.0 + i, std=10.0)
    last3 = h.recent("right", 7, n=3)
    assert [e.u1 for e in last3] == [102.0, 103.0, 104.0]


def test_separate_cameras_have_independent_histories():
    h = DarkHistory(max_darks=4)
    h.append("left", 0, t=0.0, u1=100.0, std=10.0)
    h.append("right", 0, t=0.0, u1=200.0, std=20.0)
    assert h.recent("left", 0, n=1)[0].u1 == 100.0
    assert h.recent("right", 0, n=1)[0].u1 == 200.0


def test_clear_empties_all_cameras():
    h = DarkHistory(max_darks=4)
    h.append("left", 0, t=0.0, u1=100.0, std=10.0)
    h.append("right", 5, t=1.0, u1=200.0, std=20.0)
    h.clear()
    assert h.size("left", 0) == 0
    assert h.size("right", 5) == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_dark_history.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Implement DarkHistory**

Create `omotion/pipeline/stages/dark.py` (this is the first piece — more added in later tasks):

```python
"""Dark correction subsystem.

This module is built up across Tasks 14–18 in the implementation plan:
    Task 14: DarkHistory          — ring buffer of recent dark observations
    Task 15: DarkIntegrityGuard   — u1 > pedestal+threshold check
    Task 16: HybridRealtimePredictor — avg-of-3 u1 + linear-extrap std + ZOH
    Task 17: PendingInterval, LinearInterpolation, DarkFrameQuadraticStencil
    Task 18: DarkCorrectionStage  — orchestrator that ties everything together
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque


@dataclass(frozen=True)
class DarkObservation:
    """One dark-frame measurement for one camera."""
    t:   float    # timestamp_s
    u1:  float    # raw mean (not pedestal-subtracted)
    std: float    # raw std


class DarkHistory:
    """Per-(side, cam) ring buffer of recent DarkObservations.

    Two reasons to keep this separate from DarkCorrectionStage:
      - testability: history mechanics can be unit-tested in isolation
      - predictors take it as an argument, which lets us swap strategies
        without coupling them to the stage's internal state
    """

    def __init__(self, max_darks: int = 4):
        if max_darks < 1:
            raise ValueError(f"max_darks must be >= 1, got {max_darks}")
        self._max = int(max_darks)
        self._rings: dict[tuple[str, int], Deque[DarkObservation]] = {}

    def append(self, side: str, cam_id: int, *, t: float, u1: float, std: float) -> None:
        key = (side, int(cam_id))
        ring = self._rings.get(key)
        if ring is None:
            ring = deque(maxlen=self._max)
            self._rings[key] = ring
        ring.append(DarkObservation(t=float(t), u1=float(u1), std=float(std)))

    def recent(self, side: str, cam_id: int, n: int) -> list[DarkObservation]:
        """Return up to the most recent n entries in chronological order."""
        ring = self._rings.get((side, int(cam_id)))
        if ring is None:
            return []
        if n >= len(ring):
            return list(ring)
        return list(ring)[-n:]

    def size(self, side: str, cam_id: int) -> int:
        ring = self._rings.get((side, int(cam_id)))
        return 0 if ring is None else len(ring)

    def is_empty(self, side: str, cam_id: int) -> bool:
        return self.size(side, cam_id) == 0

    def clear(self) -> None:
        self._rings.clear()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_dark_history.py -v`
Expected: 6 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/stages/dark.py tests/test_pipeline/test_dark_history.py
git commit -m "feat(sdk): DarkHistory — per-camera ring buffer of dark observations"
```

---

### Task 15: DarkIntegrityGuard

**Files:**
- Modify: `omotion/pipeline/stages/dark.py` (append `DarkIntegrityGuard`)
- Create: `tests/test_pipeline/test_dark_integrity_guard.py`

The guard checks whether a dark frame's u1 exceeds `pedestal + threshold` (default 30). If so, it appends a `DarkIntegrityWarning` to `batch.events`. The frame still gets processed; this is diagnostic, not a drop signal.

- [ ] **Step 1: Write the failing test**

```python
"""DarkIntegrityGuard — flag dark frames whose u1 looks too bright."""

from omotion.pipeline.batch import DarkIntegrityWarning
from omotion.pipeline.stages.dark import DarkIntegrityGuard


def test_passes_silently_when_u1_within_range():
    guard = DarkIntegrityGuard(max_above_pedestal=30.0)
    events = []
    ok = guard.check(side="left", cam_id=0, abs_frame_id=10, u1=80.0,
                     pedestal=64.0, events=events)
    assert ok is True
    assert events == []


def test_appends_warning_when_u1_exceeds_pedestal_plus_threshold():
    guard = DarkIntegrityGuard(max_above_pedestal=30.0)
    events = []
    # u1=200 with pedestal=64 → 200 > 64+30=94 → fail
    ok = guard.check(side="left", cam_id=3, abs_frame_id=10, u1=200.0,
                     pedestal=64.0, events=events)
    assert ok is False
    assert len(events) == 1
    w = events[0]
    assert isinstance(w, DarkIntegrityWarning)
    assert w.side == "left"
    assert w.cam_id == 3
    assert w.u1 == 200.0


def test_threshold_is_configurable():
    guard = DarkIntegrityGuard(max_above_pedestal=50.0)
    events = []
    # u1=100 with pedestal=64 → 100 < 64+50=114 → ok
    ok = guard.check(side="left", cam_id=0, abs_frame_id=10, u1=100.0,
                     pedestal=64.0, events=events)
    assert ok is True
    assert events == []
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_dark_integrity_guard.py -v`
Expected: FAIL — `ImportError: cannot import name 'DarkIntegrityGuard'`

- [ ] **Step 3: Append DarkIntegrityGuard to `dark.py`**

Append to `omotion/pipeline/stages/dark.py`:

```python
from ..batch import DarkIntegrityWarning


class DarkIntegrityGuard:
    """Flag dark frames whose u1 looks suspiciously bright.

    A genuine dark frame should have u1 within ~30 DN of the sensor pedestal
    (the ambient + dark-current floor). If u1 is much higher, the laser
    probably wasn't actually off for that frame — most often a firmware
    off-by-one in NUM_DARK_FRAMES_AT_START or an unwrapper alignment quirk.

    The guard does not drop the frame; it just emits a DarkIntegrityWarning
    diagnostic event. Dark-frame interpolation will still happen, but the
    operator (and any audit log) sees a clear marker that the corrected
    stream may be polluted for this interval.

    See docs/SciencePipeline.md §11 (input validation rails).
    """

    def __init__(self, max_above_pedestal: float = 30.0):
        self.max_above_pedestal = float(max_above_pedestal)

    def check(self, *, side: str, cam_id: int, abs_frame_id: int,
              u1: float, pedestal: float, events: list) -> bool:
        """Return True if the dark frame passes; False if it failed (warning emitted)."""
        threshold = pedestal + self.max_above_pedestal
        if u1 > threshold:
            events.append(DarkIntegrityWarning(
                side=side, cam_id=int(cam_id), abs_frame_id=int(abs_frame_id),
                u1=float(u1), pedestal=float(pedestal),
                threshold=float(self.max_above_pedestal),
            ))
            return False
        return True
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_dark_integrity_guard.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/stages/dark.py tests/test_pipeline/test_dark_integrity_guard.py
git commit -m "feat(sdk): DarkIntegrityGuard — flag dark frames with u1 > pedestal+threshold"
```

---

### Task 16: HybridRealtimePredictor

**Files:**
- Modify: `omotion/pipeline/stages/dark.py` (append `HybridRealtimePredictor`)
- Create: `tests/test_pipeline/test_dark_estimators.py`

The predictor implements the hybrid algorithm from `SciencePipeline.md §7.4.1`: `u1 ← avg of last 3 darks`, `std ← linear extrapolation through last 2`, ZOH fallback when only 1 dark.

- [ ] **Step 1: Write the failing test**

```python
"""HybridRealtimePredictor — avg-of-3 u1 + linear-extrap std + ZOH fallback."""

import pytest
from omotion.pipeline.stages.dark import (
    DarkHistory, HybridRealtimePredictor,
)


def _hist_with(entries):
    """entries: list of (t, u1, std)"""
    h = DarkHistory(max_darks=10)
    for t, u1, std in entries:
        h.append("left", 0, t=t, u1=u1, std=std)
    return h


def test_zoh_when_only_one_dark():
    """Warmup case: single dark → both u1 and std are zero-order-hold."""
    h = _hist_with([(0.0, 100.0, 10.0)])
    pred = HybridRealtimePredictor()
    u1, std = pred.predict("left", 0, history=h, target_t=15.0)
    assert u1 == 100.0
    assert std == 10.0


def test_avg_of_2_when_only_two_darks():
    h = _hist_with([(0.0, 100.0, 10.0), (15.0, 200.0, 20.0)])
    pred = HybridRealtimePredictor()
    # u1 = mean of last 2 = 150
    # std = linear extrap from (0.0, 10.0) → (15.0, 20.0) at t=30:
    #       slope = (20-10) / (15-0) = 2/3, σ̂ = 20 + (2/3)(30-15) = 30
    u1, std = pred.predict("left", 0, history=h, target_t=30.0)
    assert u1 == pytest.approx(150.0)
    assert std == pytest.approx(30.0)


def test_avg_of_3_when_three_or_more_darks():
    h = _hist_with([
        (0.0,  100.0, 10.0),
        (15.0, 110.0, 12.0),
        (30.0, 120.0, 14.0),
        (45.0, 130.0, 16.0),
    ])
    pred = HybridRealtimePredictor()
    # u1 = mean of last 3 = (110+120+130)/3 ≈ 120
    # std = linear extrap from (30, 14) → (45, 16) at t=60:
    #       slope = (16-14)/(45-30) = 2/15; σ̂ = 16 + (2/15)(60-45) = 18
    u1, std = pred.predict("left", 0, history=h, target_t=60.0)
    assert u1 == pytest.approx(120.0)
    assert std == pytest.approx(18.0)


def test_returns_none_when_history_is_empty():
    h = DarkHistory(max_darks=4)
    pred = HybridRealtimePredictor()
    result = pred.predict("left", 0, history=h, target_t=15.0)
    assert result is None


def test_zoh_std_when_two_darks_have_same_timestamp():
    """Degenerate Δt=0 falls back to ZOH for std (avoids divide by zero)."""
    h = _hist_with([(0.0, 100.0, 10.0), (0.0, 200.0, 20.0)])
    pred = HybridRealtimePredictor()
    u1, std = pred.predict("left", 0, history=h, target_t=15.0)
    assert std == 20.0   # ZOH from most recent
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_dark_estimators.py -v`
Expected: FAIL — `ImportError`

- [ ] **Step 3: Append HybridRealtimePredictor to `dark.py`**

Append to `omotion/pipeline/stages/dark.py`:

```python
from typing import Optional


class HybridRealtimePredictor:
    """Realtime dark-baseline predictor.

    Predicts (û₁, σ̂) for a light frame at time `target_t`, given the
    rolling DarkHistory:

        u1   ← average of last 3 dark observations (truncated to whatever
               is available; with one dark this collapses to ZOH)
        std  ← linear extrapolation through the last two (t, std) points,
               with zero-order-hold fallback when only one dark is available
               or when the two darks have identical timestamps

    Returns None when no darks have been observed yet — caller should skip
    realtime emission for that frame (warmup window).

    See docs/SciencePipeline.md §7.4.1 and SciencePipeline.py:_emit_realtime_corrected.
    """

    def predict(self, side: str, cam_id: int, *, history: DarkHistory,
                target_t: float) -> Optional[tuple[float, float]]:
        recent = history.recent(side, cam_id, n=3)
        if not recent:
            return None

        # u1: average of last min(3, available) entries
        u1_pred = sum(o.u1 for o in recent) / len(recent)

        # std: linear extrapolation through last 2; ZOH with 1
        if len(recent) < 2 or history.size(side, cam_id) < 2:
            std_pred = recent[-1].std
        else:
            last2 = history.recent(side, cam_id, n=2)
            a, b = last2
            dt = b.t - a.t
            if dt <= 0:
                std_pred = b.std
            else:
                slope = (b.std - a.std) / dt
                std_pred = b.std + slope * (target_t - b.t)

        return (float(u1_pred), float(std_pred))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_dark_estimators.py -v`
Expected: 5 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/stages/dark.py tests/test_pipeline/test_dark_estimators.py
git commit -m "feat(sdk): HybridRealtimePredictor — avg-of-3 u1 + linear-extrap std + ZOH fallback"
```

---

### Task 17: PendingInterval + LinearInterpolation + DarkFrameQuadraticStencil

**Files:**
- Modify: `omotion/pipeline/stages/dark.py` (append the three classes)
- Add tests for each to `tests/test_pipeline/test_dark_estimators.py` (appending more cases)

These three together implement the batched (lagged but accurate) corrected-stream computation from `SciencePipeline.md §8`:
- `PendingInterval` buffers non-dark frames between two bounding darks
- `LinearInterpolation` computes the dark-baseline interpolation across that interval
- `DarkFrameQuadraticStencil` fills in the corrected value for the dark frame itself using a 4-point stencil with documented fallbacks

- [ ] **Step 1: Append failing tests**

Append to `tests/test_pipeline/test_dark_estimators.py`:

```python
import numpy as np
from omotion.pipeline.stages.dark import (
    PendingInterval, LinearInterpolation, DarkFrameQuadraticStencil,
    DarkObservation,
)


def test_pending_interval_collects_frames_between_darks():
    pi = PendingInterval()
    # Open interval with first dark
    pi.set_left_dark(DarkObservation(t=0.0, u1=100.0, std=10.0), abs_frame_id=10)
    # Add 3 non-dark frames
    pi.add_light(abs_frame_id=11, t=0.025, u1=500.0, u2=260_000.0)
    pi.add_light(abs_frame_id=12, t=0.050, u1=510.0, u2=265_000.0)
    pi.add_light(abs_frame_id=13, t=0.075, u1=520.0, u2=275_000.0)
    assert not pi.is_closed()

    # Close with second dark
    pi.set_right_dark(DarkObservation(t=0.100, u1=105.0, std=11.0), abs_frame_id=14)
    assert pi.is_closed()
    interval = pi.flush()
    assert interval.left_abs == 10
    assert interval.right_abs == 14
    assert len(interval.light_frames) == 3


def test_linear_interpolation_dark_baseline_across_interval():
    # Interval: left dark at t=0 with u1=100, std=10
    #           right dark at t=10 with u1=200, std=20
    # Light frame at t=5 → interpolated baseline u1=150, std=15
    pi = PendingInterval()
    pi.set_left_dark(DarkObservation(t=0.0, u1=100.0, std=10.0), abs_frame_id=10)
    pi.add_light(abs_frame_id=11, t=5.0, u1=500.0, u2=260_000.0)
    pi.set_right_dark(DarkObservation(t=10.0, u1=200.0, std=20.0), abs_frame_id=12)

    interval = pi.flush()
    interp = LinearInterpolation()
    corrected = interp.correct_interval(interval)
    # corrected.frames[0] is the light at t=5
    f = corrected.frames[0]
    assert f.abs_frame_id == 11
    # mean = u1 - baseline_u1 = 500 - 150 = 350
    assert f.mean == pytest.approx(350.0)
    # variance from u1, u2, baseline_std
    raw_var = 260_000.0 - 500.0 ** 2     # 10000
    expected_var = max(0.0, raw_var - 15.0 ** 2)   # 10000 - 225 = 9775
    assert f.std == pytest.approx(np.sqrt(expected_var))


def test_stencil_full_4_point_when_all_neighbours_present():
    stencil = DarkFrameQuadraticStencil()
    # 4 corrected bfi values: v(-2), v(-1), v(+1), v(+2) = 1.0, 2.0, 4.0, 5.0
    # Stencil: (-1/6)*1 + (2/3)*2 + (2/3)*4 + (-1/6)*5 = -1/6 + 4/3 + 8/3 - 5/6 = 3.0
    v = stencil.interpolate_dark_value(
        v_minus_2=1.0, v_minus_1=2.0, v_plus_1=4.0, v_plus_2=5.0,
    )
    assert v == pytest.approx(3.0)


def test_stencil_falls_back_to_right_only_when_no_left_neighbours():
    stencil = DarkFrameQuadraticStencil()
    v = stencil.interpolate_dark_value(
        v_minus_2=None, v_minus_1=None, v_plus_1=4.0, v_plus_2=5.0,
    )
    assert v == pytest.approx(4.5)   # (4+5)/2


def test_stencil_falls_back_to_simple_avg_when_only_immediate_neighbours():
    stencil = DarkFrameQuadraticStencil()
    v = stencil.interpolate_dark_value(
        v_minus_2=None, v_minus_1=2.0, v_plus_1=4.0, v_plus_2=None,
    )
    assert v == pytest.approx(3.0)   # (2+4)/2


def test_stencil_falls_back_to_repeat_right_when_only_right_available():
    stencil = DarkFrameQuadraticStencil()
    v = stencil.interpolate_dark_value(
        v_minus_2=None, v_minus_1=None, v_plus_1=4.0, v_plus_2=None,
    )
    assert v == pytest.approx(4.0)   # repeat right
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_pipeline/test_dark_estimators.py -v`
Expected: 5 PASS (from task 16), 6 FAIL (new ones)

- [ ] **Step 3: Append the three classes to `dark.py`**

```python
from typing import Optional


@dataclass
class _LightSample:
    abs_frame_id: int
    t: float
    u1: float
    u2: float


@dataclass
class _DarkBoundary:
    obs: DarkObservation
    abs_frame_id: int


@dataclass
class Interval:
    """A closed dark-bounded interval, ready for batch correction."""
    left:         _DarkBoundary
    right:        _DarkBoundary
    light_frames: list[_LightSample]

    @property
    def left_abs(self) -> int:
        return self.left.abs_frame_id

    @property
    def right_abs(self) -> int:
        return self.right.abs_frame_id


@dataclass
class CorrectedFrame:
    """One corrected sample, output of batch correction."""
    abs_frame_id: int
    t:        float
    mean:     float
    std:      float


@dataclass
class CorrectedInterval:
    """Output of batch correction: corrected values for every frame in
    the interval including the dark frame."""
    left_abs:  int
    right_abs: int
    frames:    list[CorrectedFrame]


class PendingInterval:
    """Buffers non-dark frames between two bounding darks."""

    def __init__(self):
        self._left:  Optional[_DarkBoundary] = None
        self._right: Optional[_DarkBoundary] = None
        self._light: list[_LightSample] = []

    def set_left_dark(self, obs: DarkObservation, *, abs_frame_id: int) -> None:
        self._left = _DarkBoundary(obs=obs, abs_frame_id=int(abs_frame_id))
        self._light = []
        self._right = None

    def add_light(self, *, abs_frame_id: int, t: float, u1: float, u2: float) -> None:
        self._light.append(_LightSample(
            abs_frame_id=int(abs_frame_id), t=float(t), u1=float(u1), u2=float(u2),
        ))

    def set_right_dark(self, obs: DarkObservation, *, abs_frame_id: int) -> None:
        self._right = _DarkBoundary(obs=obs, abs_frame_id=int(abs_frame_id))

    def is_closed(self) -> bool:
        return self._left is not None and self._right is not None

    def flush(self) -> Interval:
        """Return the closed interval; reset state so a new interval can begin."""
        assert self.is_closed(), "flush() called on non-closed interval"
        interval = Interval(
            left=self._left,
            right=self._right,
            light_frames=list(self._light),
        )
        # Roll the right dark into the new left for the next interval
        self._left = self._right
        self._right = None
        self._light = []
        return interval


class LinearInterpolation:
    """Compute corrected values for a closed dark-bounded interval.

    See docs/SciencePipeline.md §8.1–§8.3.
    """

    def correct_interval(self, interval: Interval) -> CorrectedInterval:
        d_prev = interval.left.obs
        d_next = interval.right.obs
        dt = d_next.t - d_prev.t

        corrected_frames: list[CorrectedFrame] = []
        for lf in interval.light_frames:
            if dt > 0:
                t_frac = (lf.t - d_prev.t) / dt
            else:
                t_frac = 0.0
            baseline_u1  = d_prev.u1 + t_frac * (d_next.u1 - d_prev.u1)
            baseline_std = d_prev.std + t_frac * (d_next.std - d_prev.std)

            mean = lf.u1 - baseline_u1
            raw_var = max(0.0, lf.u2 - lf.u1 ** 2)
            corrected_var = max(0.0, raw_var - baseline_std ** 2)
            std = float(corrected_var ** 0.5)

            corrected_frames.append(CorrectedFrame(
                abs_frame_id=lf.abs_frame_id, t=lf.t, mean=mean, std=std,
            ))

        return CorrectedInterval(
            left_abs=interval.left_abs, right_abs=interval.right_abs,
            frames=corrected_frames,
        )


class DarkFrameQuadraticStencil:
    """4-point quadratic interpolation for the dark frame's own corrected value.

    Stencil:
        v(D) = (-1/6) v(D-2) + (2/3) v(D-1) + (2/3) v(D+1) + (-1/6) v(D+2)

    Fallback chain when neighbours are missing (see SciencePipeline.md §8.4):
        full      — all four neighbours present
        right_only — left missing, right ≥2 present → (v(+1) + v(+2)) / 2
        simple_avg — only v(-1) and v(+1) → (v(-1) + v(+1)) / 2
        repeat_right — only v(+1) → v(+1)
    """

    def interpolate_dark_value(self, *,
                               v_minus_2: Optional[float],
                               v_minus_1: Optional[float],
                               v_plus_1: Optional[float],
                               v_plus_2: Optional[float]) -> float:
        # Full 4-point case
        if all(v is not None for v in (v_minus_2, v_minus_1, v_plus_1, v_plus_2)):
            return (-1/6) * v_minus_2 + (2/3) * v_minus_1 \
                + (2/3) * v_plus_1 + (-1/6) * v_plus_2

        # Right-only (≥2 right available, left missing)
        if v_minus_1 is None and v_plus_1 is not None and v_plus_2 is not None:
            return (v_plus_1 + v_plus_2) / 2

        # Simple-avg (v(-1) and v(+1) only)
        if v_minus_1 is not None and v_plus_1 is not None:
            return (v_minus_1 + v_plus_1) / 2

        # Repeat-right
        if v_plus_1 is not None:
            return v_plus_1

        # Should not happen — caller must provide at least v_plus_1
        raise ValueError("DarkFrameQuadraticStencil needs at least v_plus_1")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_dark_estimators.py -v`
Expected: 11 passed (5 from task 16 + 6 new).

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/stages/dark.py tests/test_pipeline/test_dark_estimators.py
git commit -m "feat(sdk): PendingInterval + LinearInterpolation + DarkFrameQuadraticStencil"
```

---

### Task 18: DarkCorrectionStage (orchestrator)

**Files:**
- Modify: `omotion/pipeline/stages/dark.py` (append `DarkCorrectionStage`)
- Create: `tests/test_pipeline/test_dark_correction_stage.py`

The stage ties everything together: runs the realtime predictor for light frames, records darks (with integrity check), tracks pending intervals, and emits `IntervalClosed` events when the second dark of an interval arrives. Also implements `on_scan_stop()` for the terminal dark flush.

- [ ] **Step 1: Write the failing test**

```python
"""DarkCorrectionStage — orchestrator: dual-output realtime + batch."""

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch, IntervalClosed
from omotion.pipeline.stages.dark import (
    DarkCorrectionStage, HybridRealtimePredictor, LinearInterpolation,
)


def _batch(n_frames, frame_types, abs_ids, *, mean_raw, std_raw, u2):
    """Build a minimal FrameBatch shaped (N, 2, 8) with only side=0 cam=0 populated."""
    n = n_frames
    raw_hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    raw_hist[:, 0, 0, 0] = 1   # marker for the only active camera

    batch = FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=raw_hist,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64) * 0.025,
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.array(abs_ids, dtype=np.int64),
        frame_type=np.array(frame_types, dtype="<U8"),
        mean_raw=mean_raw, std_raw=std_raw,
    )
    # Stash u2 onto the batch for the test stage — production stages compute
    # u2 in the moments path; here we pass it explicitly for testability.
    batch._test_u2 = u2
    return batch


def test_no_realtime_emit_before_first_dark():
    """Light frames before the first dark have no baseline to subtract."""
    mean = np.full((3, 2, 8), 500.0, dtype=np.float32)
    std  = np.full((3, 2, 8), 10.0,  dtype=np.float32)
    batch = _batch(3, ["light", "light", "light"], [11, 12, 13],
                   mean_raw=mean, std_raw=std, u2=np.full((3, 2, 8), 260_000.0))

    stage = DarkCorrectionStage(
        realtime_estimator=HybridRealtimePredictor(),
        batch_estimator=LinearInterpolation(),
    )
    stage.process(batch)
    # No realtime baseline → mean_dc_rt[0, 0, 0] should be NaN (no prediction available)
    assert np.isnan(batch.mean_dc_rt[0, 0, 0])


def test_emits_interval_closed_event_when_two_darks_bracket_lights():
    n = 4
    mean = np.array([[100.0], [500.0], [510.0], [105.0]], dtype=np.float32).reshape(4, 1, 1) * np.ones((1, 2, 8))
    std  = np.array([[10.0],  [20.0],  [22.0],  [11.0]], dtype=np.float32).reshape(4, 1, 1) * np.ones((1, 2, 8))
    u2   = mean ** 2 + std ** 2
    batch = _batch(n, ["dark", "light", "light", "dark"], [10, 11, 12, 14],
                   mean_raw=mean, std_raw=std, u2=u2)

    stage = DarkCorrectionStage(
        realtime_estimator=HybridRealtimePredictor(),
        batch_estimator=LinearInterpolation(),
    )
    stage.process(batch)

    events = [e for e in batch.events if isinstance(e, IntervalClosed)]
    assert len(events) > 0


def test_reset_clears_dark_history_and_pending():
    stage = DarkCorrectionStage(
        realtime_estimator=HybridRealtimePredictor(),
        batch_estimator=LinearInterpolation(),
    )
    # Process some darks
    mean = np.full((1, 2, 8), 100.0, dtype=np.float32)
    std  = np.full((1, 2, 8), 10.0,  dtype=np.float32)
    u2   = mean ** 2 + std ** 2
    batch1 = _batch(1, ["dark"], [10], mean_raw=mean, std_raw=std, u2=u2)
    stage.process(batch1)

    stage.reset()

    # New scan starts — no history
    batch2 = _batch(1, ["light"], [11], mean_raw=np.full((1, 2, 8), 500.0, dtype=np.float32),
                    std_raw=np.full((1, 2, 8), 20.0, dtype=np.float32),
                    u2=np.full((1, 2, 8), 260_000.0, dtype=np.float32))
    stage.process(batch2)
    assert np.isnan(batch2.mean_dc_rt[0, 0, 0])   # no baseline → no realtime
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_dark_correction_stage.py -v`
Expected: FAIL — `ImportError`

- [ ] **Step 3: Append `DarkCorrectionStage` to `dark.py`**

```python
from ..batch import FrameBatch, IntervalClosed
from ..pedestal import SensorPedestals


class DarkCorrectionStage:
    """Orchestrates the dual-output dark correction.

    Per non-dark frame: populates batch.dark_baseline_rt, batch.mean_dc_rt,
    batch.std_dc_rt using HybridRealtimePredictor (NaN where no prediction
    is available — the warmup window).

    Per dark frame: runs the integrity guard, appends to DarkHistory + the
    appropriate PendingInterval, and emits an IntervalClosed event if a
    full interval has bookended.

    See docs/SciencePipeline.md §7.4 (realtime) and §8 (batched).
    """
    name = "dark_correction"

    SIDE_NAMES = ("left", "right")

    def __init__(self, *,
                 realtime_estimator: HybridRealtimePredictor,
                 batch_estimator: LinearInterpolation,
                 pedestals: Optional[SensorPedestals] = None,
                 realtime_history_size: int = 4,
                 integrity_max_above_pedestal: float = 30.0):
        self._realtime = realtime_estimator
        self._batch = batch_estimator
        self._pedestals = pedestals or SensorPedestals(left=64.0, right=64.0)
        self._history = DarkHistory(max_darks=realtime_history_size)
        # Per (side, cam) PendingInterval
        self._pending: dict[tuple[str, int], PendingInterval] = {}
        self._guard = DarkIntegrityGuard(
            max_above_pedestal=integrity_max_above_pedestal
        )

    def process(self, batch: FrameBatch) -> FrameBatch:
        n = batch.frame_ids.shape[0]
        baseline_rt = np.full((n, 2, 8), np.nan, dtype=np.float32)
        mean_dc_rt  = np.full((n, 2, 8), np.nan, dtype=np.float32)
        std_dc_rt   = np.full((n, 2, 8), np.nan, dtype=np.float32)

        for i in range(n):
            ftype = str(batch.frame_type[i])
            if ftype not in ("light", "dark"):
                continue
            cam_id  = int(batch.cam_ids[i])
            abs_id  = int(batch.abs_frame_ids[i])
            t       = float(batch.timestamp_s[i])
            # Determine side from raw_histograms population
            side_idx = int(np.argmax(batch.raw_histograms[i].sum(axis=(-2, -1))))
            side = self.SIDE_NAMES[side_idx]

            u1 = float(batch.mean_raw[i, side_idx, cam_id])
            std = float(batch.std_raw[i, side_idx, cam_id])

            if ftype == "dark":
                pedestal = (self._pedestals.left if side == "left"
                            else self._pedestals.right)
                self._guard.check(
                    side=side, cam_id=cam_id, abs_frame_id=abs_id,
                    u1=u1, pedestal=pedestal, events=batch.events,
                )
                # Add to history regardless of integrity (warning is diagnostic)
                self._history.append(side, cam_id, t=t, u1=u1, std=std)

                # Update PendingInterval for this (side, cam)
                pi = self._pending.get((side, cam_id))
                if pi is None:
                    pi = PendingInterval()
                    self._pending[(side, cam_id)] = pi
                    pi.set_left_dark(DarkObservation(t=t, u1=u1, std=std),
                                     abs_frame_id=abs_id)
                else:
                    pi.set_right_dark(DarkObservation(t=t, u1=u1, std=std),
                                      abs_frame_id=abs_id)
                    if pi.is_closed():
                        interval = pi.flush()
                        # Use the just-flushed right dark as the new left
                        pi.set_left_dark(DarkObservation(t=t, u1=u1, std=std),
                                         abs_frame_id=abs_id)
                        corrected = self._batch.correct_interval(interval)
                        batch.events.append(IntervalClosed(corrected_batch=corrected))

            else:  # light
                pred = self._realtime.predict(
                    side, cam_id, history=self._history, target_t=t,
                )
                if pred is not None:
                    u1_hat, std_hat = pred
                    baseline_rt[i, side_idx, cam_id] = np.float32(u1_hat)
                    mean_dc_rt[i, side_idx, cam_id]  = np.float32(u1 - u1_hat)
                    raw_var = max(0.0, batch.std_raw[i, side_idx, cam_id] ** 2)
                    corr_var = max(0.0, raw_var - std_hat ** 2)
                    std_dc_rt[i, side_idx, cam_id] = np.float32(corr_var ** 0.5)

                    # Also append to the active PendingInterval for batch correction
                    pi = self._pending.get((side, cam_id))
                    if pi is not None:
                        # u2 = std² + u1²
                        u2 = std ** 2 + u1 ** 2
                        pi.add_light(abs_frame_id=abs_id, t=t, u1=u1, u2=u2)

        batch.dark_baseline_rt = baseline_rt
        batch.mean_dc_rt = mean_dc_rt
        batch.std_dc_rt = std_dc_rt
        return batch

    def reset(self) -> None:
        self._history.clear()
        self._pending.clear()

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """Terminal dark flush — for short scans that end before a second
        scheduled dark, synthesize a dark from the last buffered moment
        so a corrected batch can still be emitted. See SciencePipeline.md §8.6.
        """
        for (side, cam_id), pi in self._pending.items():
            if not pi._light:    # no buffered light frames → nothing to correct
                continue
            if self._history.size(side, cam_id) < 1:
                continue   # no opening dark either — skip with WARNING in production
            # Promote the last buffered light as synthetic terminal dark
            last = pi._light[-1]
            terminal = DarkObservation(t=last.t, u1=last.u1,
                                       std=(last.u2 - last.u1 ** 2) ** 0.5)
            pi.set_right_dark(terminal, abs_frame_id=last.abs_frame_id)
            interval = pi.flush()
            corrected = self._batch.correct_interval(interval)
            batch.events.append(IntervalClosed(corrected_batch=corrected))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_dark_correction_stage.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/stages/dark.py tests/test_pipeline/test_dark_correction_stage.py
git commit -m "feat(sdk): DarkCorrectionStage — orchestrator with terminal-flush + IntervalClosed event"
```

---

## Phase F — Sources (4 tasks)

### Task 19: Source protocol

**Files:**
- Create: `omotion/pipeline/sources.py` (this task adds the protocol + a base class)
- Create: `tests/test_pipeline/test_sources.py`

- [ ] **Step 1: Write the failing test**

```python
"""Source protocol — defines the FrameBatch iterator contract."""

import pytest
from omotion.pipeline.sources import Source, _BaseSource
from omotion.pipeline.sinks import ScanMetadata


def test_source_protocol_is_runtime_checkable():
    class _Mock:
        metadata = ScanMetadata(
            scan_id="x", subject_id="y", operator="z",
            started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
            left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
            write_raw_csv=False, raw_csv_duration_sec=None,
        )
        def __iter__(self): yield from []
        def close(self): pass
    assert isinstance(_Mock(), Source)


def test_base_source_close_is_noop_by_default():
    class _Sub(_BaseSource):
        def __iter__(self): yield from []

    src = _Sub(metadata=None)
    src.close()   # should not raise
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_sources.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Implement Source protocol + _BaseSource**

Create `omotion/pipeline/sources.py`:

```python
"""Source protocol + abstract base.

Concrete sources (LiveUsbSource, CsvReplaySource, DbReplaySource) come in
tasks 20–22. A Source produces an iterator of FrameBatch and carries the
ScanMetadata for the scan.
"""

from __future__ import annotations

from typing import Iterator, Protocol, runtime_checkable

from .batch import FrameBatch
from .sinks import ScanMetadata


@runtime_checkable
class Source(Protocol):
    metadata: ScanMetadata

    def __iter__(self) -> Iterator[FrameBatch]: ...

    def close(self) -> None: ...


class _BaseSource:
    """Shared scaffolding for concrete sources."""

    def __init__(self, *, metadata: ScanMetadata):
        self.metadata = metadata

    def close(self) -> None:
        pass
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_sources.py -v`
Expected: 2 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/sources.py tests/test_pipeline/test_sources.py
git commit -m "feat(sdk): Source protocol + abstract base"
```

---

### Task 20: CsvReplaySource

**Files:**
- Modify: `omotion/pipeline/sources.py` (append `CsvReplaySource`)
- Modify: `tests/test_pipeline/test_sources.py` (append CSV replay tests)

`CsvReplaySource` reads a raw histogram CSV produced by `CsvSink` (the new sink with the `type` column) and emits `FrameBatch` objects.

- [ ] **Step 1: Append failing tests**

```python
import csv
import io
import numpy as np
from omotion.pipeline.sources import CsvReplaySource
from omotion.pipeline.sinks import ScanMetadata


def _meta():
    return ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0x01, right_camera_mask=0, reduced_mode=False,
        write_raw_csv=False, raw_csv_duration_sec=None,
    )


def _write_raw_csv(tmp_path, rows):
    """Write a raw CSV in the new schema: cam_id, frame_id, timestamp_s, type, 0..1023, temperature, sum, tcm, tcl, pdc."""
    path = tmp_path / "raw.csv"
    header = ["cam_id", "frame_id", "timestamp_s", "type"] + \
             [str(i) for i in range(1024)] + \
             ["temperature", "sum", "tcm", "tcl", "pdc"]
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        for row in rows:
            w.writerow(row)
    return path


def test_csv_replay_yields_one_batch_per_chunk(tmp_path):
    bins = [0] * 1024
    bins[100] = 2_457_606
    rows = [
        # cam_id, frame_id, timestamp_s, type, *bins, temperature, sum, tcm, tcl, pdc
        [0, 1, 0.000, "warmup"] + bins + [27.0, 2_457_606, 0.0, 0.0, 0.0],
        [0, 2, 0.025, "warmup"] + bins + [27.0, 2_457_606, 0.0, 0.0, 0.0],
    ]
    path = _write_raw_csv(tmp_path, rows)

    src = CsvReplaySource(
        raw_csv_left=path, raw_csv_right=None,
        batch_size_frames=10, metadata=_meta(),
    )
    batches = list(src)
    assert len(batches) >= 1
    first = batches[0]
    assert first.raw_histograms.shape[-1] == 1024
    assert first.frame_ids.tolist()[:2] == [1, 2]
    assert first.raw_histograms[0, 0, 0, 100] == 2_457_606
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_pipeline/test_sources.py -v`
Expected: FAIL — `ImportError: cannot import name 'CsvReplaySource'`

- [ ] **Step 3: Append CsvReplaySource to `sources.py`**

```python
import csv
from pathlib import Path
from typing import Iterator, Optional

import numpy as np

from .batch import FrameBatch


class CsvReplaySource(_BaseSource):
    """Replays a raw-histogram CSV produced by CsvSink.

    The CSV schema (see SciencePipeline.md §12 / spec §12) is:
        cam_id, frame_id, timestamp_s, type, 0..1023, temperature, sum, tcm, tcl, pdc

    Accepts up to two CSVs (one per side); if a side is None, only the
    other side is replayed (mask reflects this).
    """

    def __init__(self, *,
                 raw_csv_left: Optional[Path],
                 raw_csv_right: Optional[Path],
                 batch_size_frames: int = 100,
                 metadata: ScanMetadata):
        super().__init__(metadata=metadata)
        self._paths = {"left": raw_csv_left, "right": raw_csv_right}
        self._batch_size = int(batch_size_frames)

    def __iter__(self) -> Iterator[FrameBatch]:
        # Open both files concurrently and step through them. For simplicity
        # in PR 1 we read each side independently and emit one batch per
        # chunk of `batch_size_frames` rows from whichever side has data.
        # PR 2 can add proper time-aligned merging.
        for side_name, path in self._paths.items():
            if path is None:
                continue
            yield from self._iter_side(side_name, path)

    def _iter_side(self, side_name: str, path) -> Iterator[FrameBatch]:
        side_idx = 0 if side_name == "left" else 1
        rows_buf: list[dict] = []
        with open(path, "r", newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                rows_buf.append(row)
                if len(rows_buf) >= self._batch_size:
                    yield self._rows_to_batch(side_idx, rows_buf)
                    rows_buf = []
            if rows_buf:
                yield self._rows_to_batch(side_idx, rows_buf)

    def _rows_to_batch(self, side_idx: int, rows: list[dict]) -> FrameBatch:
        n = len(rows)
        cam_ids       = np.array([int(r["cam_id"])       for r in rows], dtype=np.int8)
        frame_ids     = np.array([int(r["frame_id"])     for r in rows], dtype=np.uint8)
        timestamp_s   = np.array([float(r["timestamp_s"]) for r in rows], dtype=np.float64)
        temperatures  = np.array([float(r["temperature"]) for r in rows], dtype=np.float32)
        types         = np.array([r["type"] for r in rows], dtype="<U8")

        raw_hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
        temp_arr = np.zeros((n, 2, 8), dtype=np.float32)
        for i, row in enumerate(rows):
            cam = int(row["cam_id"])
            for b in range(1024):
                raw_hist[i, side_idx, cam, b] = int(row[str(b)])
            temp_arr[i, side_idx, cam] = float(row["temperature"])

        return FrameBatch(
            cam_ids=cam_ids,
            frame_ids=frame_ids,
            raw_histograms=raw_hist,
            temperature_c=temp_arr,
            timestamp_s=timestamp_s,
            pdc=None, tcm=None, tcl=None,
            # frame_type is populated by FrameClassificationStage during replay,
            # not pre-loaded from the CSV — so that the replay path is identical
            # to the live path.
        )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_sources.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/sources.py tests/test_pipeline/test_sources.py
git commit -m "feat(sdk): CsvReplaySource — read raw histogram CSVs into FrameBatches"
```

---

### Task 21: DbReplaySource

**Files:**
- Modify: `omotion/pipeline/sources.py` (append `DbReplaySource`)
- Modify: `tests/test_pipeline/test_sources.py` (append DB replay tests)

Reads from `session_raw` rows of a scan-DB session.

- [ ] **Step 1: Write a failing test** (tests assume the existing `omotion.ScanDatabase` API)

```python
def test_db_replay_yields_batches_from_session_raw(tmp_path):
    # Construct a minimal scan DB with a few rows in session_raw
    from omotion.ScanDatabase import ScanDatabase
    db_path = tmp_path / "scan.db"
    db = ScanDatabase(str(db_path))
    db.create_schema()
    session_id = db.insert_session(meta={"scan_id": "x"})

    bins = bytes(1024 * 4)   # zeroed-out histogram blob
    for fid in range(1, 4):
        db.insert_session_raw(
            session_id=session_id, side="left", cam_id=0,
            frame_id=fid, timestamp_s=fid * 0.025,
            histogram_blob=bins, temperature=27.0, row_sum=2_457_606,
        )
    db.close()

    from omotion.pipeline.sources import DbReplaySource
    src = DbReplaySource(db_path=str(db_path), session_id=session_id,
                          batch_size_frames=10, metadata=_meta())
    batches = list(src)
    assert len(batches) >= 1
    assert batches[0].frame_ids.tolist() == [1, 2, 3]
```

(Note: if `ScanDatabase.insert_session_raw` has a different signature, adjust the test setup. Source of truth is `omotion/ScanDatabase.py`.)

- [ ] **Step 2-3: Implement DbReplaySource**

Append to `omotion/pipeline/sources.py`:

```python
import sqlite3
import struct


class DbReplaySource(_BaseSource):
    """Replays a scan-DB session by reading rows out of session_raw.

    Assumes the table layout used by ScanDBSink (see omotion/ScanDatabase.py).
    """

    def __init__(self, *, db_path: str, session_id: int,
                 batch_size_frames: int = 100,
                 metadata: ScanMetadata):
        super().__init__(metadata=metadata)
        self._db_path = db_path
        self._session_id = int(session_id)
        self._batch_size = int(batch_size_frames)

    def __iter__(self) -> Iterator[FrameBatch]:
        conn = sqlite3.connect(self._db_path)
        try:
            cur = conn.execute(
                "SELECT side, cam_id, frame_id, timestamp_s, histogram_blob, "
                "       temperature, row_sum "
                "FROM session_raw WHERE session_id = ? "
                "ORDER BY timestamp_s, side, cam_id",
                (self._session_id,),
            )
            buf: list[tuple] = []
            for row in cur:
                buf.append(row)
                if len(buf) >= self._batch_size:
                    yield self._rows_to_batch(buf)
                    buf = []
            if buf:
                yield self._rows_to_batch(buf)
        finally:
            conn.close()

    def _rows_to_batch(self, rows: list) -> FrameBatch:
        n = len(rows)
        cam_ids       = np.zeros(n, dtype=np.int8)
        frame_ids     = np.zeros(n, dtype=np.uint8)
        timestamp_s   = np.zeros(n, dtype=np.float64)
        raw_hist      = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
        temp_arr      = np.zeros((n, 2, 8), dtype=np.float32)

        for i, (side, cam_id, frame_id, t, blob, temp, _row_sum) in enumerate(rows):
            side_idx = 0 if side == "left" else 1
            cam_ids[i] = int(cam_id)
            frame_ids[i] = int(frame_id)
            timestamp_s[i] = float(t)
            # blob is 4096 bytes = 1024 × uint32 LE
            hist_arr = np.frombuffer(blob, dtype=np.uint32, count=1024)
            raw_hist[i, side_idx, int(cam_id)] = hist_arr
            temp_arr[i, side_idx, int(cam_id)] = float(temp)

        return FrameBatch(
            cam_ids=cam_ids, frame_ids=frame_ids,
            raw_histograms=raw_hist, temperature_c=temp_arr,
            timestamp_s=timestamp_s,
            pdc=None, tcm=None, tcl=None,
        )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_sources.py -v`
Expected: 4 passed (3 from previous + 1 new).

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/sources.py tests/test_pipeline/test_sources.py
git commit -m "feat(sdk): DbReplaySource — read scan-DB session_raw rows into FrameBatches"
```

---

### Task 22: LiveUsbSource

**Files:**
- Modify: `omotion/pipeline/sources.py` (append `LiveUsbSource`)
- (Tests for `LiveUsbSource` require hardware — skip in CI, mark as `@pytest.mark.sensor`)

`LiveUsbSource` runs USB reads on background threads and pushes `FrameBatch` objects onto a queue. It's the only Source with real threading. Tests are hardware-marked.

- [ ] **Step 1: Write the (hardware-marked) test**

Append to `tests/test_pipeline/test_sources.py`:

```python
@pytest.mark.sensor
def test_live_usb_source_smoke():
    """Smoke test on real hardware — iterates a few batches and verifies shape.
    Skipped in CI; run manually against a connected sensor."""
    from omotion import MOTIONInterface
    from omotion.pipeline.sources import LiveUsbSource

    motion = MOTIONInterface()
    motion.start()
    try:
        src = LiveUsbSource(
            console=motion.console, left=motion.left, right=motion.right,
            batch_size_frames=10, metadata=_meta(),
        )
        batches_seen = 0
        for batch in src:
            batches_seen += 1
            assert batch.raw_histograms.shape[-1] == 1024
            if batches_seen >= 3:
                src.close()
                break
        assert batches_seen >= 1
    finally:
        motion.stop()
```

- [ ] **Step 2: Append LiveUsbSource to `sources.py`**

```python
import queue
import threading
from typing import Any


class LiveUsbSource(_BaseSource):
    """Reads histogram packets from USB on background threads, batches them
    into FrameBatch objects, hands them to the runner via a queue.

    One reader thread per side. Each thread parses incoming binary packets
    via the existing `parse_histogram_packet_structured()` in MotionProcessing
    (PR 2 may move that helper into pipeline/). Parsed HistogramSamples
    accumulate until either `batch_size_frames` is reached or `flush_interval_s`
    elapses; then a FrameBatch is built and pushed to the output queue.

    The output iterator pops FrameBatch objects from the queue until the
    runner is done (via `close()`).
    """

    def __init__(self, *,
                 console: Any, left: Any, right: Any,
                 batch_size_frames: int = 10,
                 flush_interval_s: float = 0.25,
                 queue_size: int = 4,
                 metadata: ScanMetadata):
        super().__init__(metadata=metadata)
        self._console = console
        self._left = left
        self._right = right
        self._batch_size = int(batch_size_frames)
        self._flush_interval = float(flush_interval_s)
        self._queue: queue.Queue = queue.Queue(maxsize=queue_size)
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    def __iter__(self) -> Iterator[FrameBatch]:
        # Start reader threads
        for side_name, sensor in (("left", self._left), ("right", self._right)):
            if sensor is None:
                continue
            t = threading.Thread(
                target=self._reader_loop, args=(side_name, sensor),
                name=f"LiveUsbSource-{side_name}", daemon=True,
            )
            t.start()
            self._threads.append(t)

        # Yield batches as they arrive
        while not self._stop.is_set():
            try:
                batch = self._queue.get(timeout=1.0)
            except queue.Empty:
                continue
            if batch is None:   # sentinel
                break
            yield batch

    def close(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2.0)

    def _reader_loop(self, side_name: str, sensor: Any) -> None:
        """Per-side reader: parse incoming USB packets into HistogramSamples,
        accumulate into FrameBatches, push to queue.

        Detailed implementation deferred to first integration: the parsing
        helpers (parse_histogram_packet_structured) live in MotionProcessing.py
        today; we call them here. PR 2 will refactor that path.
        """
        from omotion.MotionProcessing import parse_histogram_packet_structured
        # Implementation outline:
        #   while not self._stop.is_set():
        #       bytes = sensor.stream.read(timeout=...)
        #       sample = parse_histogram_packet_structured(bytes)
        #       buffer.append(sample)
        #       if len(buffer) >= self._batch_size or elapsed > flush_interval:
        #           batch = self._build_batch(side_name, buffer)
        #           self._queue.put(batch)
        #           buffer.clear()
        raise NotImplementedError(
            "LiveUsbSource reader loop — see PR 2 task list for the full "
            "integration with omotion.StreamInterface."
        )
```

- [ ] **Step 3-5: Commit (no automated test run for hardware path)**

```bash
git add omotion/pipeline/sources.py tests/test_pipeline/test_sources.py
git commit -m "feat(sdk): LiveUsbSource skeleton (reader loop deferred to PR 2)"
```

> **Note:** The reader loop is intentionally `NotImplementedError` in PR 1. PR 2's first task is to wire it up against the existing `omotion.StreamInterface`. Tests for live USB are hardware-marked and not required in CI.

---

## Phase G — Runner + Sinks (5 tasks)

### Task 23: ScanRunner

**Files:**
- Create: `omotion/pipeline/runner.py`
- Create: `tests/test_pipeline/test_runner.py`

- [ ] **Step 1: Write the failing test**

```python
"""ScanRunner — Source → Pipeline → Sinks. The only I/O orchestrator."""

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch, LiveEmit, IntervalClosed
from omotion.pipeline.pipeline import Pipeline
from omotion.pipeline.runner import ScanRunner
from omotion.pipeline.sinks import ScanMetadata


class _FakeSource:
    def __init__(self, batches, metadata):
        self._batches = batches
        self.metadata = metadata
    def __iter__(self):
        yield from self._batches
    def close(self):
        pass


class _RecordingSink:
    """Records every (channel, payload) it consumes for inspection."""
    def __init__(self, channels):
        self.channels = set(channels)
        self.consumed = []
        self.on_start_calls = 0
        self.on_complete_calls = 0
    def on_scan_start(self, meta):
        self.on_start_calls += 1
    def consume(self, channel, payload):
        self.consumed.append((channel, payload))
    def on_complete(self):
        self.on_complete_calls += 1


class _EmitTagsStage:
    """Test stage: appends LiveEmit events with the given channels."""
    name = "emit_tags"
    def __init__(self, channels):
        self.channels = channels
    def process(self, batch):
        for ch in self.channels:
            batch.events.append(LiveEmit(channel=ch, payload=batch))
        return batch
    def reset(self):
        pass


def _meta():
    return ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
        write_raw_csv=False, raw_csv_duration_sec=None,
    )


def _empty_batch():
    return FrameBatch(
        cam_ids=np.zeros(1, dtype=np.int8),
        frame_ids=np.zeros(1, dtype=np.uint8),
        raw_histograms=np.zeros((1, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((1, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(1, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


def test_runner_lifecycle_calls_on_start_and_on_complete():
    sink = _RecordingSink(channels={"live"})
    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_EmitTagsStage(["live"])]),
        sinks=[sink],
    )
    runner.run()
    assert sink.on_start_calls == 1
    assert sink.on_complete_calls == 1


def test_runner_routes_live_events_to_subscribed_sinks_only():
    live_sink = _RecordingSink(channels={"live"})
    raw_sink  = _RecordingSink(channels={"raw"})
    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_EmitTagsStage(["live", "raw"])]),
        sinks=[live_sink, raw_sink],
    )
    runner.run()
    assert [c for c, _ in live_sink.consumed] == ["live"]
    assert [c for c, _ in raw_sink.consumed]  == ["raw"]


def test_runner_routes_interval_closed_to_final_sinks():
    final_sink = _RecordingSink(channels={"final"})

    class _IntervalStage:
        name = "interval"
        def process(self, batch):
            batch.events.append(IntervalClosed(corrected_batch="payload_x"))
            return batch
        def reset(self): pass

    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_IntervalStage()]),
        sinks=[final_sink],
    )
    runner.run()
    assert final_sink.consumed == [("final", "payload_x")]


def test_runner_isolates_sink_exceptions():
    """A misbehaving sink can't crash the scan."""
    class _CrashingSink:
        channels = {"live"}
        def on_scan_start(self, m): pass
        def consume(self, ch, p): raise RuntimeError("boom")
        def on_complete(self): pass

    good_sink = _RecordingSink(channels={"live"})
    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_EmitTagsStage(["live"])]),
        sinks=[_CrashingSink(), good_sink],
    )
    runner.run()   # should not raise
    assert good_sink.consumed == [("live", good_sink.consumed[0][1])]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_runner.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: Implement ScanRunner**

Create `omotion/pipeline/runner.py`:

```python
"""ScanRunner — the one I/O orchestrator. Pulls batches from a Source,
runs them through the Pipeline, dispatches batch events to the right Sinks."""

from __future__ import annotations

import logging

from .batch import FrameBatch, LiveEmit, IntervalClosed, BatchEvent
from .pipeline import Pipeline
from .sinks import Sink
from .sources import Source


logger = logging.getLogger("omotion.pipeline.runner")


class ScanRunner:
    def __init__(self, *, source: Source, pipeline: Pipeline, sinks: list[Sink]):
        self.source = source
        self.pipeline = pipeline
        self.sinks = list(sinks)

    def _sinks_for(self, channel: str) -> list[Sink]:
        return [s for s in self.sinks if channel in getattr(s, "channels", set())]

    def _safe_consume(self, sink: Sink, channel: str, payload) -> None:
        try:
            sink.consume(channel, payload)
        except Exception:
            logger.exception("sink %r raised on channel %s; continuing",
                             type(sink).__name__, channel)

    def run(self) -> None:
        for sink in self.sinks:
            try:
                sink.on_scan_start(self.source.metadata)
            except Exception:
                logger.exception("sink %r raised in on_scan_start", type(sink).__name__)

        try:
            for batch in self.source:
                try:
                    result = self.pipeline.process(batch)
                except Exception:
                    logger.exception("pipeline.process raised — resetting and continuing")
                    self.pipeline.reset()
                    continue
                self._dispatch(result)
        finally:
            for sink in self.sinks:
                try:
                    sink.on_complete()
                except Exception:
                    logger.exception("sink %r raised in on_complete", type(sink).__name__)

    def _dispatch(self, batch: FrameBatch) -> None:
        for event in batch.events:
            if isinstance(event, LiveEmit):
                for sink in self._sinks_for(event.channel):
                    self._safe_consume(sink, event.channel, event.payload)
            elif isinstance(event, IntervalClosed):
                for sink in self._sinks_for("final"):
                    self._safe_consume(sink, "final", event.corrected_batch)
            elif isinstance(event, BatchEvent):
                for sink in self._sinks_for("diagnostics"):
                    self._safe_consume(sink, "diagnostics", event)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_runner.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/runner.py tests/test_pipeline/test_runner.py
git commit -m "feat(sdk): ScanRunner — channel-based dispatch with sink isolation"
```

---

### Task 24: CsvSink (new, against the channel-based protocol)

**Files:**
- Modify: `omotion/pipeline/sinks.py` (append `CsvSink`)
- Create: `tests/test_pipeline/test_csv_sink.py`

The new `CsvSink` lives in `omotion.pipeline.sinks` (alongside the protocol). It writes raw + corrected CSVs in the new schema (with `type` column), self-gating raw writes on `write_raw_csv` and `raw_csv_duration_sec` from the `ScanMetadata`.

- [ ] **Step 1: Write the failing test**

```python
"""New CsvSink — channel-based, with the 'type' column in raw output."""

import csv
import pytest
from omotion.pipeline.sinks import CsvSink, ScanMetadata


def _meta_with_raw(write_raw, duration):
    return ScanMetadata(
        scan_id="abc", subject_id="subj", operator="op",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=300,
        left_camera_mask=0x01, right_camera_mask=0, reduced_mode=False,
        write_raw_csv=write_raw, raw_csv_duration_sec=duration,
    )


def test_csv_sink_skips_raw_when_write_raw_csv_is_false(tmp_path):
    sink = CsvSink(output_dir=tmp_path)
    sink.on_scan_start(_meta_with_raw(write_raw=False, duration=None))
    # Try to consume on the raw channel — sink should NOT create any raw CSV
    sink.consume("raw", _dummy_batch_payload())
    sink.on_complete()
    raw_files = list(tmp_path.glob("*raw*.csv"))
    assert raw_files == []


def test_csv_sink_writes_type_column_when_raw_enabled(tmp_path):
    sink = CsvSink(output_dir=tmp_path)
    sink.on_scan_start(_meta_with_raw(write_raw=True, duration=None))
    sink.consume("raw", _dummy_batch_payload())
    sink.on_complete()
    raw_files = list(tmp_path.glob("*raw*.csv"))
    assert len(raw_files) == 1
    with open(raw_files[0]) as fh:
        header = next(csv.reader(fh))
    assert "type" in header
    # New schema: type column comes right after timestamp_s
    assert header.index("type") == 3
```

(The `_dummy_batch_payload()` helper would build a minimal FrameBatch with one row, matching what a Tee("raw") emit would carry. Full test body deferred to implementation time.)

- [ ] **Step 2-5: Implement, test, commit**

Append `CsvSink` to `omotion/pipeline/sinks.py` (skeleton — the full row-writing detail follows the existing `omotion/CsvSink.py` pattern; only the schema and channel dispatch differ). The key changes vs the legacy CsvSink:

1. `channels = {"raw", "final"}` declared at class level
2. Single `consume(channel, payload)` method that switches on channel
3. New raw header includes `type` as column 4
4. Self-gates raw writes on `meta.write_raw_csv` + elapsed time vs `meta.raw_csv_duration_sec`

Commit:

```bash
git commit -m "feat(sdk): CsvSink (pipeline) — channel-based with 'type' column in raw CSV"
```

---

### Task 25: ScanDBSink (new, against the channel-based protocol)

Mirror of Task 24 but writing to the scan DB. Adapter wraps the existing `ScanDatabase` API (in `omotion/ScanDatabase.py`); `consume("raw", ...)` writes `session_raw`, `consume("final", ...)` writes `session_data`. Self-gates raw inserts the same way.

Commit:

```bash
git commit -m "feat(sdk): ScanDBSink (pipeline) — channel-based; same raw-gating contract"
```

---

### Task 26: QtUiSink skeleton

**Files:**
- Modify: `omotion/pipeline/sinks.py` (append `QtUiSink`)
- Create: `tests/test_pipeline/test_qt_ui_sink.py`

A minimal sink that emits Qt signals from `consume("live", ...)`. Skeleton in PR 1 (full integration with `motion_connector.py` is PR 3 work).

- [ ] **Implementation outline (full code in PR 3)**

```python
class QtUiSink:
    """Forwards live-channel batches as Qt signals for the app's plot widget.

    PR 1 ships a stub that records calls (for testability). PR 3 wires it
    against PyQt6 signals in motion_connector.py.
    """
    channels = {"live"}
    def __init__(self):
        self.live_batches = []
    def on_scan_start(self, meta): pass
    def consume(self, channel, payload):
        if channel == "live":
            self.live_batches.append(payload)
    def on_complete(self): pass
```

Commit:

```bash
git commit -m "feat(sdk): QtUiSink stub for the live channel"
```

---

### Task 27: `default_pipeline()` factory

**Files:**
- Create: `omotion/pipeline/factory.py`
- Create: `tests/test_pipeline/test_factory.py`

A single function that assembles the canonical 9-stage pipeline plus the 3 positional Tees, configured from a `ScanMetadata` + `Calibration` + sensor versions.

- [ ] **Step 1: Write the failing test**

```python
def test_default_pipeline_has_expected_stages():
    from omotion.pipeline.factory import default_pipeline
    from omotion.pipeline.sinks import ScanMetadata
    from omotion.pipeline.pedestal import SensorPedestals

    cal = _trivial_calibration()  # from earlier tests
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
        write_raw_csv=True, raw_csv_duration_sec=None,
    )
    pipeline = default_pipeline(
        metadata=meta, calibration=cal,
        pedestals=SensorPedestals(left=64.0, right=64.0),
    )
    names = [stage.name for stage in pipeline.stages]
    assert names == [
        "frame_classification",
        "tee:raw",
        "noise_floor", "moments", "pedestal_subtraction",
        "dark_correction", "shot_noise_correction", "bfi_bvi",
        "side_averaging",
        "tee:live",
        "rolling_average",
        "tee:rolling",
    ]
```

- [ ] **Step 2-5: Implement, test, commit**

Create `omotion/pipeline/factory.py`:

```python
"""default_pipeline() — assembles the canonical 9-stage + 3-Tee chain."""

from __future__ import annotations

from typing import Any

import numpy as np

from .pipeline import Pipeline
from .pedestal import SensorPedestals
from .sinks import ScanMetadata
from .stages.classify import FrameClassificationStage
from .stages.noise_floor import NoiseFloorStage
from .stages.moments import MomentsStage
from .stages.pedestal_sub import PedestalSubtractionStage
from .stages.dark import (
    DarkCorrectionStage, HybridRealtimePredictor, LinearInterpolation,
)
from .stages.shot_noise import ShotNoiseCorrectionStage
from .stages.bfi_bvi import BfiBviStage
from .stages.side_avg import SideAveragingStage
from .stages.rolling_avg import RollingAverageStage
from .tee import Tee


ADC_GAIN = (1024 - 64) / 11_000
CAMERA_GAIN_MAP = np.array([16, 4, 2, 1, 1, 2, 4, 16], dtype=np.float32)


def default_pipeline(*,
                    metadata: ScanMetadata,
                    calibration: Any,
                    pedestals: SensorPedestals,
                    noise_floor_threshold: int = 10,
                    rolling_avg_window: int = 10,
                    discard_count: int = 9,
                    dark_interval: int = 600,
                    realtime_dark_history_size: int = 4) -> Pipeline:
    """Build the canonical pipeline. See SciencePipeline.md for the algorithm."""

    not_warmup_or_stale = lambda ft: ft != "warmup" and ft != "stale"

    return Pipeline([
        FrameClassificationStage(discard_count=discard_count, dark_interval=dark_interval),
        Tee("raw", filter=None),

        NoiseFloorStage(threshold=noise_floor_threshold),
        MomentsStage(),
        PedestalSubtractionStage(pedestals=pedestals),

        DarkCorrectionStage(
            realtime_estimator=HybridRealtimePredictor(),
            batch_estimator=LinearInterpolation(),
            pedestals=pedestals,
            realtime_history_size=realtime_dark_history_size,
        ),

        ShotNoiseCorrectionStage(adc_gain=ADC_GAIN, camera_gain_map=CAMERA_GAIN_MAP),
        BfiBviStage(calibration=calibration),

        SideAveragingStage(
            enabled=metadata.reduced_mode,
            left_camera_mask=metadata.left_camera_mask,
            right_camera_mask=metadata.right_camera_mask,
        ),
        Tee("live", filter=not_warmup_or_stale),

        RollingAverageStage(window=rolling_avg_window),
        Tee("rolling", filter=not_warmup_or_stale),
    ])
```

Commit:

```bash
git add omotion/pipeline/factory.py tests/test_pipeline/test_factory.py
git commit -m "feat(sdk): default_pipeline() factory — assembles the canonical 9-stage + 3-Tee chain"
```

---

## Phase H — Integration tests (3 tasks)

### Task 28: Golden-replay test + fixture infrastructure

**Files:**
- Create: `tests/test_pipeline/data/normal_short_scan.raw.csv` (~50 KB; ~20 frames per side)
- Create: `tests/test_pipeline/data/normal_short_scan.corrected.golden.csv`
- Create: `tests/test_pipeline/data/reduced_mode_scan.raw.csv`
- Create: `tests/test_pipeline/data/reduced_mode_scan.corrected.golden.csv`
- Create: `tests/test_pipeline/test_golden_replay.py`

The golden tests are the audit-evidence backbone: a recorded raw CSV → run the pipeline → assert output equals the checked-in corrected CSV byte-for-byte.

- [ ] **Step 1: Generate fixtures from current SDK behavior**

Use the legacy `SciencePipeline` (still in `MotionProcessing.py`) to generate the golden fixtures so they reflect today's known-good behavior. Add a one-shot helper script:

Create `tests/test_pipeline/data/regenerate_goldens.py`:

```python
"""Regenerate golden fixtures from the legacy SciencePipeline.

Run this ONCE to create the goldens; re-run only if the algorithm itself
changes (and document why in a commit message). After PR 2 lands, this
script can also use the new pipeline as the reference — the two pipelines
must agree on these fixtures.
"""

# Use omotion.MotionProcessing.feed_pipeline_from_csv against a synthetic
# raw CSV constructed from known histograms (use tests/fixtures/generate_fixtures.py
# patterns). Save both raw and corrected outputs to data/.
```

- [ ] **Step 2: Write the golden test**

Create `tests/test_pipeline/test_golden_replay.py`:

```python
"""Golden-replay: run the new pipeline on a recorded raw CSV and assert
the corrected output matches the checked-in golden byte-for-byte.

These tests are the acceptance evidence — they prove the new pipeline
produces the same numbers as the legacy SciencePipeline for known scans.
"""

import csv
import pathlib
import pytest
import numpy as np

from omotion.pipeline.factory import default_pipeline
from omotion.pipeline.runner import ScanRunner
from omotion.pipeline.sources import CsvReplaySource
from omotion.pipeline.sinks import CsvSink, ScanMetadata
from omotion.pipeline.pedestal import SensorPedestals


HERE = pathlib.Path(__file__).parent / "data"


def _run_replay(raw_csv_path, output_dir, meta):
    """Replay a raw CSV through the new pipeline; CsvSink writes corrected CSV."""
    pipeline = default_pipeline(
        metadata=meta,
        calibration=_trivial_calibration(),
        pedestals=SensorPedestals(left=64.0, right=64.0),
    )
    source = CsvReplaySource(
        raw_csv_left=raw_csv_path, raw_csv_right=None,
        batch_size_frames=20, metadata=meta,
    )
    sink = CsvSink(output_dir=output_dir)
    runner = ScanRunner(source=source, pipeline=pipeline, sinks=[sink])
    runner.run()
    return next(output_dir.glob("*corrected*.csv"))


def _read_csv_rows(path):
    with open(path) as fh:
        return list(csv.reader(fh))


def _trivial_calibration():
    # Same calibration used to generate the goldens. Must match
    # regenerate_goldens.py to keep tests reproducible.
    from dataclasses import dataclass
    @dataclass
    class _Cal:
        c_min: np.ndarray
        c_max: np.ndarray
        i_min: np.ndarray
        i_max: np.ndarray
    return _Cal(
        c_min=np.zeros((2, 8), dtype=np.float32),
        c_max=np.ones((2, 8), dtype=np.float32),
        i_min=np.zeros((2, 8), dtype=np.float32),
        i_max=np.full((2, 8), 500.0, dtype=np.float32),
    )


def test_normal_short_scan_matches_golden(tmp_path):
    meta = ScanMetadata(
        scan_id="golden_normal", subject_id="x", operator="x",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=10,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
        write_raw_csv=False, raw_csv_duration_sec=None,
    )
    actual = _run_replay(HERE / "normal_short_scan.raw.csv", tmp_path, meta)
    golden_rows = _read_csv_rows(HERE / "normal_short_scan.corrected.golden.csv")
    actual_rows = _read_csv_rows(actual)
    assert actual_rows == golden_rows


def test_reduced_mode_scan_matches_golden(tmp_path):
    meta = ScanMetadata(
        scan_id="golden_reduced", subject_id="x", operator="x",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=10,
        left_camera_mask=0x66, right_camera_mask=0x66, reduced_mode=True,
        write_raw_csv=False, raw_csv_duration_sec=None,
    )
    actual = _run_replay(HERE / "reduced_mode_scan.raw.csv", tmp_path, meta)
    golden_rows = _read_csv_rows(HERE / "reduced_mode_scan.corrected.golden.csv")
    actual_rows = _read_csv_rows(actual)
    assert actual_rows == golden_rows
```

- [ ] **Step 3: Run regenerate_goldens.py once** to create the fixture files. Commit the fixtures.

- [ ] **Step 4: Run the golden tests**

Run: `pytest tests/test_pipeline/test_golden_replay.py -v`
Expected: 2 passed. If they fail, the new pipeline's output deviates from the legacy — investigate the diff (numeric vs. ordering vs. CSV-format).

- [ ] **Step 5: Commit**

```bash
git add tests/test_pipeline/data/ tests/test_pipeline/test_golden_replay.py
git commit -m "test(sdk): golden-replay tests + recorded fixtures (audit evidence)"
```

---

### Task 29: Determinism test

**Files:**
- Create: `tests/test_pipeline/test_determinism.py`

Two replays of the same source through the same pipeline must produce byte-identical output. Catches accidental non-determinism (clock reads, random sampling, mutable shared state).

- [ ] **Step 1-5: Write, run, commit**

Create `tests/test_pipeline/test_determinism.py`:

```python
"""Determinism: same source twice through the same pipeline = same output."""

import pathlib
import pytest

from omotion.pipeline.factory import default_pipeline
from omotion.pipeline.runner import ScanRunner
from omotion.pipeline.sources import CsvReplaySource
from omotion.pipeline.sinks import CsvSink, ScanMetadata
from omotion.pipeline.pedestal import SensorPedestals
from tests.test_pipeline.test_golden_replay import _trivial_calibration, _read_csv_rows


HERE = pathlib.Path(__file__).parent / "data"


def _replay_once(raw_csv, tmp_dir):
    meta = ScanMetadata(
        scan_id="det", subject_id="x", operator="x",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=10,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
        write_raw_csv=False, raw_csv_duration_sec=None,
    )
    pipeline = default_pipeline(
        metadata=meta, calibration=_trivial_calibration(),
        pedestals=SensorPedestals(left=64.0, right=64.0),
    )
    source = CsvReplaySource(
        raw_csv_left=raw_csv, raw_csv_right=None,
        batch_size_frames=20, metadata=meta,
    )
    sink = CsvSink(output_dir=tmp_dir)
    ScanRunner(source=source, pipeline=pipeline, sinks=[sink]).run()
    return next(tmp_dir.glob("*corrected*.csv"))


def test_two_replays_byte_identical(tmp_path):
    raw = HERE / "normal_short_scan.raw.csv"
    out1 = _replay_once(raw, tmp_path / "run1")
    out2 = _replay_once(raw, tmp_path / "run2")
    assert _read_csv_rows(out1) == _read_csv_rows(out2)
```

Run: `pytest tests/test_pipeline/test_determinism.py -v`
Expected: PASS.

```bash
git add tests/test_pipeline/test_determinism.py
git commit -m "test(sdk): replay-determinism test — same source twice = byte-identical output"
```

---

### Task 30: Per-stage perf benchmark

**Files:**
- Create: `tests/test_pipeline/benchmarks/test_stage_perf.py`

`pytest-benchmark` budgets per stage. CI flags >20 % regression.

- [ ] **Step 1-5: Write, run, commit**

```python
"""Perf budgets per stage. Run with: pytest tests/test_pipeline/benchmarks/ --benchmark-only

Budgets calibrated to a representative dev laptop (M1 / Ryzen 5). Adjust
in this file when hardware changes — flag any drift > 20 % in CI.
"""

import numpy as np
import pytest

from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.moments import MomentsStage
from omotion.pipeline.stages.noise_floor import NoiseFloorStage
from omotion.pipeline.stages.shot_noise import ShotNoiseCorrectionStage


N = 100   # batch size — representative of a 2.5 s window at 40 Hz


def _full_batch(n):
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.random.randint(0, 1000, (n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.linspace(0.0, 2.5, n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


def test_moments_stage_under_5ms(benchmark):
    stage = MomentsStage()
    batch = _full_batch(N)
    result = benchmark(stage.process, batch)
    # Budget: 5 ms / batch-of-100. (Compare to ~150 ms for the Python-loop legacy path.)
    assert benchmark.stats["mean"] < 0.005


def test_noise_floor_stage_under_1ms(benchmark):
    stage = NoiseFloorStage(threshold=10)
    batch = _full_batch(N)
    benchmark(stage.process, batch)
    assert benchmark.stats["mean"] < 0.001


def test_shot_noise_stage_under_1ms(benchmark):
    stage = ShotNoiseCorrectionStage(
        adc_gain=0.0873,
        camera_gain_map=np.array([16, 4, 2, 1, 1, 2, 4, 16], dtype=np.float32),
    )
    batch = _full_batch(N)
    batch.mean_dc_rt = np.full((N, 2, 8), 500.0, dtype=np.float32)
    batch.std_dc_rt  = np.full((N, 2, 8), 20.0,  dtype=np.float32)
    benchmark(stage.process, batch)
    assert benchmark.stats["mean"] < 0.001
```

Run: `pytest tests/test_pipeline/benchmarks/ --benchmark-only -v`
Expected: 3 passed (with budget assertions met on dev hardware). On slower CI runners loosen the budgets — these are guard rails, not hard SLAs.

```bash
git add tests/test_pipeline/benchmarks/
git commit -m "test(sdk): perf budgets per stage via pytest-benchmark"
```

---

## Phase I — Public API + finalization (1 task)

### Task 31: Public API surface in `omotion/pipeline/__init__.py`

**Files:**
- Modify: `omotion/pipeline/__init__.py`
- Create: `tests/test_pipeline/test_public_api.py`

Re-export the public types so downstream consumers can write `from omotion.pipeline import default_pipeline, ScanRunner, CsvSink, ...` without reaching into submodules.

- [ ] **Step 1: Write the failing test**

```python
"""Public API surface — symbols documented as importable from omotion.pipeline."""


def test_public_api_symbols_importable():
    from omotion.pipeline import (
        Pipeline, Stage,
        FrameBatch, BatchEvent, IntervalClosed, LiveEmit,
        DarkIntegrityWarning, StencilFallback,
        ScanRunner,
        Source, LiveUsbSource, CsvReplaySource, DbReplaySource,
        Sink, ScanMetadata,
        CsvSink, ScanDBSink, QtUiSink,
        Tee, default_pipeline,
        SensorPedestals,
    )
    # Smoke check: each is non-None
    for sym in (
        Pipeline, Stage, FrameBatch, BatchEvent, IntervalClosed, LiveEmit,
        DarkIntegrityWarning, StencilFallback, ScanRunner,
        Source, LiveUsbSource, CsvReplaySource, DbReplaySource,
        Sink, ScanMetadata, CsvSink, ScanDBSink, QtUiSink,
        Tee, default_pipeline, SensorPedestals,
    ):
        assert sym is not None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_public_api.py -v`
Expected: FAIL — `ImportError: cannot import name ...`

- [ ] **Step 3: Implement the public surface**

Replace contents of `omotion/pipeline/__init__.py`:

```python
"""omotion.pipeline — composable, numpy-vectorized histogram → BFI/BVI pipeline.

See docs/SciencePipeline.md for the algorithm.
See docs/superpowers/specs/2026-05-22-data-pipeline-rearchitecture-design.md
for the design.

Typical use::

    from omotion.pipeline import (
        default_pipeline, ScanRunner,
        CsvReplaySource, CsvSink, ScanMetadata, SensorPedestals,
    )

    meta = ScanMetadata(...)
    pipeline = default_pipeline(metadata=meta, calibration=cal,
                                pedestals=SensorPedestals.from_sensors(left, right))
    source = CsvReplaySource(raw_csv_left=..., raw_csv_right=..., metadata=meta)
    sinks  = [CsvSink(output_dir="./out")]
    ScanRunner(source=source, pipeline=pipeline, sinks=sinks).run()
"""

from .batch import (
    FrameBatch,
    BatchEvent,
    IntervalClosed,
    LiveEmit,
    DarkIntegrityWarning,
    StencilFallback,
)
from .pipeline import Pipeline, Stage
from .tee import Tee
from .runner import ScanRunner
from .factory import default_pipeline
from .pedestal import SensorPedestals
from .sinks import Sink, ScanMetadata, CsvSink, ScanDBSink, QtUiSink
from .sources import Source, LiveUsbSource, CsvReplaySource, DbReplaySource


__all__ = [
    "FrameBatch", "BatchEvent", "IntervalClosed", "LiveEmit",
    "DarkIntegrityWarning", "StencilFallback",
    "Pipeline", "Stage", "Tee",
    "ScanRunner",
    "default_pipeline",
    "SensorPedestals",
    "Sink", "ScanMetadata",
    "CsvSink", "ScanDBSink", "QtUiSink",
    "Source", "LiveUsbSource", "CsvReplaySource", "DbReplaySource",
]
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_public_api.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/__init__.py tests/test_pipeline/test_public_api.py
git commit -m "feat(sdk): consolidate omotion.pipeline public API surface in __init__.py"
```

---

### Final step: run the full test suite

After Task 31, run the entire pipeline test suite end-to-end:

```bash
pytest tests/test_pipeline/ -v -m "not sensor"
```

Expected: all tests pass except those marked `@pytest.mark.sensor` (hardware-required, skipped in CI). Counts roughly: 60+ unit tests, 2 golden tests, 1 determinism test, 3 perf benchmarks.

If any tests fail, the corresponding task should be revisited before declaring the plan complete.

---

## Plan complete — 31 tasks, 9 phases, 1 spec, 1 PR

Summary of what PR 1 produces:

- New `omotion/pipeline/` package with 9 stages, FrameBatch carrier, Pipeline + Runner + Sink protocol, Sources (CSV + DB replay + LiveUsb skeleton), new CsvSink + ScanDBSink + QtUiSink stub, and `default_pipeline()` factory
- ~60 unit tests, 2 golden-replay tests, 1 determinism test, 3 perf benchmarks
- `MotionProcessing.py` untouched — nothing in production uses the new code yet
- `LiveUsbSource` reader loop is intentionally `NotImplementedError`; PR 2 wires it up

After PR 1 lands, PR 2's plan (separate document) will: wire LiveUsbSource, switch `ScanWorkflow.start_scan()` to use the new ScanRunner, turn `MotionProcessing.py` into a deprecation shim, and add deprecation warnings to the legacy `on_*_fn` kwargs.
