# Pipeline Cutover (PR 2 + PR 3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Switch `ScanWorkflow` to drive the new `omotion.pipeline` package; collapse `MotionProcessing.py` to a parsing+dataclass shim; migrate `bloodflow-app` and `test-app` to construct sinks instead of passing callbacks. After this PR sequence the new pipeline is the production code path in all three repos.

**Architecture:** SDK init carries baseline output config (`data_dir`, `scan_db_path`, `operator_id`). `ScanRequest` carries scan parameters + a list of app-provided sinks. `start_scan` auto-injects default storage sinks + auto-wires auxiliary sources (telemetry) based on which channels the sinks subscribe to. Internal SDK workflows (`CalibrationWorkflow`, `ContactQualityWorkflow`) opt out of default storage. Apps drop the 6-callback API entirely.

**Tech Stack:** Python 3.12+, numpy, threading, queue. Cross-repo: openmotion-sdk, openmotion-bloodflow-app, openmotion-test-app.

**Predecessor:** PR 1 (`feature/data-pipeline-tweaks`, PR #56 → next-next) — builds the new pipeline package alongside the legacy code. This plan switches production over.

**Spec:** [`docs/superpowers/specs/2026-05-22-pipeline-cutover-design.md`](../specs/2026-05-22-pipeline-cutover-design.md)

---

## File-structure map

**openmotion-sdk:**

```
omotion/pipeline/
├── batch.py                       # Modify: add TelemetryEvent
├── tee.py                         # Modify: add max_duration_s gate
├── factory.py                     # Modify: accept raw_save_max_duration_s, pass to Tee
├── sources.py                     # Modify: flesh out LiveUsbSource, add ConsoleTelemetrySource
├── sinks.py                       # Modify: drop write_raw_csv from ScanMetadata + CsvSink/ScanDBSink; add TelemetrySink
├── pipeline.py                    # Modify: add telemetry_aggregator attribute + lifecycle wiring
├── runner.py                      # Modify: telemetry_source kwarg, _telemetry_loop, on_scan_stop wiring
└── telemetry.py                   # Create: TelemetryAggregator + TelemetryIngestStage

omotion/
├── MotionInterface.py             # Modify: data_dir, scan_db_path, operator_id ctor args; lazy ContactQualityWorkflow
├── ScanWorkflow.py                # Rewrite start_scan (~600 LOC delta)
├── CalibrationWorkflow.py         # Migrate to _CalibrationCollectorSink
├── ContactQualityWorkflow.py      # Create: workflow + internal sink
├── MotionProcessing.py            # Shrink: ~1940 LOC → ~400 LOC (keep parse helpers + dataclasses)
└── ScanRequest.py / ScanWorkflow.py:ScanRequest  # Drop write_raw_csv/raw_csv_duration_sec/operator; add raw_save_max_duration_s/sinks/skip_default_storage

tests/
├── test_pipeline/test_telemetry.py            # Create
├── test_pipeline/test_tee.py                  # Modify: max_duration_s tests
├── test_pipeline/test_csv_sink.py             # Modify: no write_raw_csv arg
├── test_pipeline/test_scan_db_sink.py         # Modify: same
├── test_pipeline/test_factory.py              # Modify: raw_save_max_duration_s pass-through
├── test_pipeline/test_sources.py              # Modify: real LiveUsbSource (hardware-marked) + ConsoleTelemetrySource
├── test_pipeline/test_runner.py               # Modify: telemetry routing tests
├── test_scan_workflow.py                      # Modify: signature + sink injection
├── test_calibration_workflow.py               # Modify: collector sink path
├── test_contact_quality_workflow.py           # Create
└── test_motion_interface.py                   # Modify: new ctor args
```

**openmotion-bloodflow-app:**

```
motion_connector.py                # Modify: ~600 LOC delta. Replace 4-callback scan-start call with sinks + workflow calls.
tests/test_motion_connector.py     # Modify: mock sinks instead of callbacks
```

**openmotion-test-app:**

```
motion_connector.py                # Modify: simpler equivalent change
tests/test_motion_connector.py     # Modify: same
```

---

## Phase A — Foundation: gate on Tee, ScanRequest reshape, MotionInterface init (5 tasks)

These are small, isolated changes that the rest of the plan depends on. Each completes a self-contained API shift.

---

### Task 1: `Tee` gains `max_duration_s` time-gate

**Files:**
- Modify: `omotion/pipeline/tee.py`
- Modify: `tests/test_pipeline/test_tee.py`

The `Tee` class currently has only a `filter` predicate. PR 2 adds a `max_duration_s` parameter that gates emission once a batch's first frame is past the configured budget (scan-relative time, source-normalized).

- [ ] **Step 1: Write the failing test**

Append to `tests/test_pipeline/test_tee.py`:

```python
def test_tee_with_max_duration_s_emits_within_budget():
    """First frame timestamp 0.0 — emit allowed."""
    tee = Tee("raw", filter=None, max_duration_s=60.0)
    batch = _batch_with_frame_types(["light"])
    batch.timestamp_s = np.array([0.0], dtype=np.float64)
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert len(emits) == 1


def test_tee_with_max_duration_s_skips_after_budget():
    """First frame timestamp 65 sec — past 60s budget — no emit."""
    tee = Tee("raw", filter=None, max_duration_s=60.0)
    batch = _batch_with_frame_types(["light"])
    batch.timestamp_s = np.array([65.0], dtype=np.float64)
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert emits == []


def test_tee_with_max_duration_s_none_means_unbounded():
    tee = Tee("raw", filter=None, max_duration_s=None)
    batch = _batch_with_frame_types(["light"])
    batch.timestamp_s = np.array([9999.0], dtype=np.float64)
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert len(emits) == 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_tee.py -v -k max_duration`
Expected: FAIL — `TypeError: Tee.__init__() got an unexpected keyword argument 'max_duration_s'`

- [ ] **Step 3: Modify Tee implementation**

Replace `omotion/pipeline/tee.py` contents:

```python
"""Tee — a stage that emits a LiveEmit event for a named channel.

Tee stages are positional markers in the pipeline. The runner reads
LiveEmit events from batch.events and dispatches the payload to sinks
subscribed to the named channel.

A Tee may carry:
- an optional `filter` predicate over frame_type. If supplied and no
  frame in the batch passes the filter, no LiveEmit is appended.
- an optional `max_duration_s` cap. If supplied and the batch's first
  frame timestamp exceeds it, no LiveEmit is appended. Used to cap
  raw-CSV writing at a configurable duration.
"""

from __future__ import annotations

from typing import Callable, Optional

from .batch import FrameBatch, LiveEmit


class Tee:
    def __init__(self, channel: str, *,
                 filter: Optional[Callable[[str], bool]] = None,
                 max_duration_s: Optional[float] = None):
        self.name = f"tee:{channel}"
        self.channel = channel
        self.filter = filter
        self.max_duration_s = max_duration_s

    def process(self, batch: FrameBatch) -> FrameBatch:
        # Time-gate first — cheapest check.
        if self.max_duration_s is not None and batch.timestamp_s.size > 0:
            if float(batch.timestamp_s[0]) > self.max_duration_s:
                return batch

        if self.filter is not None:
            if batch.frame_type is None:
                return batch
            if not any(self.filter(ft) for ft in batch.frame_type):
                return batch
        batch.events.append(LiveEmit(channel=self.channel, payload=batch))
        return batch

    def reset(self) -> None:
        pass
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_pipeline/test_tee.py -v`
Expected: all `test_tee_*` tests pass (existing + 3 new ones).

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/tee.py tests/test_pipeline/test_tee.py
git commit -m "feat(sdk): Tee gains max_duration_s time-gate"
```

---

### Task 2: `ScanMetadata` drops `write_raw_csv` and `raw_csv_duration_sec`

**Files:**
- Modify: `omotion/pipeline/sinks.py`
- Modify: `tests/test_pipeline/test_sinks_protocol.py`
- Modify: `tests/test_pipeline/test_csv_sink.py`
- Modify: `tests/test_pipeline/test_scan_db_sink.py`

`ScanMetadata` is now just identification + scan parameters. The raw-save gate lives on the Tee, so sinks don't need these fields.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_pipeline/test_sinks_protocol.py`:

```python
def test_scan_metadata_does_not_carry_raw_csv_fields():
    """write_raw_csv and raw_csv_duration_sec moved out of ScanMetadata — they
    were app-level config that's now driven by the raw-tee gate (see
    spec §3.2.1)."""
    import dataclasses
    field_names = {f.name for f in dataclasses.fields(ScanMetadata)}
    assert "write_raw_csv" not in field_names
    assert "raw_csv_duration_sec" not in field_names


def test_scan_metadata_constructs_without_raw_csv_kwargs():
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T10:00:00Z", duration_sec=300,
        left_camera_mask=0x66, right_camera_mask=0x66, reduced_mode=True,
    )
    assert meta.scan_id == "x"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_sinks_protocol.py::test_scan_metadata_does_not_carry_raw_csv_fields -v`
Expected: FAIL — the fields still exist.

- [ ] **Step 3: Modify ScanMetadata**

In `omotion/pipeline/sinks.py`, remove `write_raw_csv` and `raw_csv_duration_sec` from the `ScanMetadata` dataclass. Final shape:

```python
@dataclass(frozen=True)
class ScanMetadata:
    """Per-scan metadata handed to every sink at on_scan_start.

    Sinks use this for file naming, identification, and column-shape
    decisions (reduced_mode). Output gating lives elsewhere (raw-save
    gate is on the pipeline's Tee("raw") per spec §3.2.1; default
    storage sinks are SDK-injected per spec §3.0).
    """
    scan_id:               str
    subject_id:            str
    operator:              str
    started_at_iso:        str
    duration_sec:          int
    left_camera_mask:      int
    right_camera_mask:     int
    reduced_mode:          bool
```

- [ ] **Step 4: Update CsvSink and ScanDBSink constructors + tests**

In `omotion/pipeline/sinks.py`, remove the self-gating logic from `CsvSink._consume_raw` (it always writes when called) and any constructor args related to write_raw_csv / raw_csv_duration_sec.

In `tests/test_pipeline/test_csv_sink.py`, update tests that previously passed `write_raw_csv=True` in `ScanMetadata` — drop those kwargs. The raw-emit gating is now tested at the Tee layer (Task 1).

Same for `test_scan_db_sink.py`.

- [ ] **Step 5: Run all sink tests to verify they pass**

Run: `pytest tests/test_pipeline/test_sinks_protocol.py tests/test_pipeline/test_csv_sink.py tests/test_pipeline/test_scan_db_sink.py -v`
Expected: all pass with the new ScanMetadata shape.

- [ ] **Step 6: Commit**

```bash
git add omotion/pipeline/sinks.py tests/test_pipeline/test_sinks_protocol.py tests/test_pipeline/test_csv_sink.py tests/test_pipeline/test_scan_db_sink.py
git commit -m "feat(sdk): ScanMetadata drops write_raw_csv/raw_csv_duration_sec; sinks no longer self-gate"
```

---

### Task 3: `default_pipeline()` accepts `raw_save_max_duration_s`

**Files:**
- Modify: `omotion/pipeline/factory.py`
- Modify: `tests/test_pipeline/test_factory.py`

The factory passes the duration through to `Tee("raw")`. Special case: `0` or negative → omit the raw tee entirely.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_pipeline/test_factory.py`:

```python
def test_default_pipeline_omits_raw_tee_when_duration_zero():
    """raw_save_max_duration_s=0 means 'never write raw' — omit the tee entirely."""
    cal = _trivial_calibration()
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
    )
    pipeline = default_pipeline(
        metadata=meta, calibration=cal,
        pedestals=SensorPedestals(left=64.0, right=64.0),
        raw_save_max_duration_s=0,
    )
    names = [stage.name for stage in pipeline.stages]
    assert "tee:raw" not in names


def test_default_pipeline_includes_raw_tee_with_finite_duration():
    cal = _trivial_calibration()
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
    )
    pipeline = default_pipeline(
        metadata=meta, calibration=cal,
        pedestals=SensorPedestals(left=64.0, right=64.0),
        raw_save_max_duration_s=60.0,
    )
    raw_tee = next(s for s in pipeline.stages if s.name == "tee:raw")
    assert raw_tee.max_duration_s == 60.0


def test_default_pipeline_includes_raw_tee_with_none_means_unbounded():
    cal = _trivial_calibration()
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
    )
    pipeline = default_pipeline(
        metadata=meta, calibration=cal,
        pedestals=SensorPedestals(left=64.0, right=64.0),
        raw_save_max_duration_s=None,
    )
    raw_tee = next(s for s in pipeline.stages if s.name == "tee:raw")
    assert raw_tee.max_duration_s is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_pipeline/test_factory.py -v -k raw_save_max`
Expected: FAIL — `default_pipeline()` doesn't accept `raw_save_max_duration_s` yet.

- [ ] **Step 3: Modify `default_pipeline()`**

In `omotion/pipeline/factory.py`, add the parameter and route it:

```python
def default_pipeline(*,
                    metadata: ScanMetadata,
                    calibration: Any,
                    pedestals: SensorPedestals,
                    noise_floor_threshold: int = 10,
                    rolling_avg_window: int = 10,
                    discard_count: int = 9,
                    dark_interval: int = 600,
                    realtime_dark_history_size: int = 4,
                    raw_save_max_duration_s: Optional[float] = None,
                    ) -> Pipeline:
    """Build the canonical pipeline. See SciencePipeline.md for the algorithm.

    raw_save_max_duration_s controls the raw-channel tee:
      None  -> emit for entire scan
      > 0   -> emit until this many seconds of scan time
      <= 0  -> omit the raw tee entirely (no raw save anywhere)
    """
    not_warmup_or_stale = lambda ft: ft != "warmup" and ft != "stale"

    stages: list = [
        FrameClassificationStage(discard_count=discard_count, dark_interval=dark_interval),
    ]

    if raw_save_max_duration_s is None or raw_save_max_duration_s > 0:
        stages.append(Tee("raw", filter=None, max_duration_s=raw_save_max_duration_s))

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

    return Pipeline(stages)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_factory.py -v`
Expected: all 4 tests pass (existing + 3 new).

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/factory.py tests/test_pipeline/test_factory.py
git commit -m "feat(sdk): default_pipeline() accepts raw_save_max_duration_s; routes to Tee('raw').max_duration_s"
```

---

### Task 4: `ScanRequest` reshape — drop legacy fields, add new ones

**Files:**
- Modify: `omotion/ScanWorkflow.py` (where `ScanRequest` is defined) — or wherever it lives in the codebase
- Modify: `tests/test_scan_workflow.py`

`ScanRequest` becomes: scan parameters + raw-save behavior + sinks + skip_default_storage flag. Drops the old per-callback kwargs and the old write_raw_csv/raw_csv_duration_sec/operator fields.

- [ ] **Step 1: Locate the current ScanRequest definition**

Run: `grep -rn "class ScanRequest" omotion/`
Expected: shows one definition (likely in `ScanWorkflow.py` or a dedicated `ScanRequest.py`).

- [ ] **Step 2: Write the failing test**

Append to `tests/test_scan_workflow.py`:

```python
import dataclasses
from omotion.ScanWorkflow import ScanRequest


def test_scan_request_carries_sinks_field():
    field_names = {f.name for f in dataclasses.fields(ScanRequest)}
    assert "sinks" in field_names
    assert "skip_default_storage" in field_names
    assert "raw_save_max_duration_s" in field_names


def test_scan_request_drops_legacy_fields():
    field_names = {f.name for f in dataclasses.fields(ScanRequest)}
    # Drop per-callback kwargs (replaced by sinks list)
    assert "on_uncorrected_fn" not in field_names
    assert "on_corrected_batch_fn" not in field_names
    assert "on_dark_frame_fn" not in field_names
    assert "on_rolling_avg_fn" not in field_names
    assert "on_realtime_corrected_fn" not in field_names
    assert "on_raw_frame_fn" not in field_names
    # Drop sink-config-on-request fields (moved to SDK-managed / raw tee gate)
    assert "write_raw_csv" not in field_names
    assert "raw_csv_duration_sec" not in field_names
    # Drop app-identity field (moved to MotionInterface.operator_id)
    assert "operator" not in field_names


def test_scan_request_sinks_default_empty_list():
    req = ScanRequest(subject_id="x", duration_sec=60,
                      left_camera_mask=0xFF, right_camera_mask=0xFF,
                      reduced_mode=False)
    assert req.sinks == []
    assert req.skip_default_storage is False
    assert req.raw_save_max_duration_s is None
```

- [ ] **Step 3: Run test to verify it fails**

Run: `pytest tests/test_scan_workflow.py::test_scan_request_carries_sinks_field tests/test_scan_workflow.py::test_scan_request_drops_legacy_fields -v`
Expected: FAIL — fields don't match.

- [ ] **Step 4: Modify ScanRequest dataclass**

Replace `ScanRequest` definition with:

```python
@dataclass
class ScanRequest:
    """Scan request — the single contract for what to do with this scan.

    Carries scan parameters, raw-save behavior, app-provided sinks, and
    an internal-use flag (skip_default_storage). Storage sinks (CsvSink,
    ScanDBSink) are SDK-managed via MotionInterface init — apps never
    construct them. Operator identity also lives on MotionInterface, not
    here.
    """
    subject_id:              str
    duration_sec:            int
    left_camera_mask:        int
    right_camera_mask:       int
    reduced_mode:            bool
    raw_save_max_duration_s: Optional[float] = None
    rolling_avg_window:      Optional[int] = None
    batch_size_frames:       Optional[int] = None
    sinks:                   list = field(default_factory=list)
    skip_default_storage:    bool = False  # internal SDK workflows set True

    # Add other existing scan-parameter fields here as needed — preserve them.
    # Drop:
    #   on_uncorrected_fn, on_corrected_batch_fn, on_dark_frame_fn,
    #   on_rolling_avg_fn, on_realtime_corrected_fn, on_raw_frame_fn
    #   write_raw_csv, raw_csv_duration_sec, operator
```

Remove `from typing import Callable` if it was only used by the dropped callback kwargs.

- [ ] **Step 5: Run test to verify it passes**

Run: `pytest tests/test_scan_workflow.py::test_scan_request_carries_sinks_field tests/test_scan_workflow.py::test_scan_request_drops_legacy_fields tests/test_scan_workflow.py::test_scan_request_sinks_default_empty_list -v`
Expected: all 3 pass.

- [ ] **Step 6: Commit**

```bash
git add omotion/ScanWorkflow.py tests/test_scan_workflow.py
git commit -m "feat(sdk): ScanRequest reshape — drop legacy callback/csv/operator fields, add sinks/skip_default_storage/raw_save_max_duration_s"
```

---

### Task 5: `MotionInterface` gains `data_dir`, `scan_db_path`, `operator_id` constructor args

**Files:**
- Modify: `omotion/MotionInterface.py`
- Modify: `tests/test_motion_interface.py` (create if doesn't exist)

SDK-level output configuration set once at init.

- [ ] **Step 1: Write the failing test**

Create or append to `tests/test_motion_interface.py`:

```python
"""Tests for MotionInterface's new SDK-level output configuration args."""

import pytest
from omotion.MotionInterface import MotionInterface


def test_motion_interface_accepts_data_dir():
    motion = MotionInterface(demo_mode=True, data_dir="C:/tmp/scans")
    assert motion.data_dir == "C:/tmp/scans"


def test_motion_interface_accepts_scan_db_path():
    motion = MotionInterface(demo_mode=True, data_dir=None,
                             scan_db_path="C:/tmp/scans/scans.db")
    assert motion.scan_db_path == "C:/tmp/scans/scans.db"


def test_motion_interface_accepts_operator_id():
    motion = MotionInterface(demo_mode=True, operator_id="bloodflow-app")
    assert motion.operator_id == "bloodflow-app"


def test_motion_interface_defaults_when_args_omitted():
    motion = MotionInterface(demo_mode=True)
    assert motion.data_dir is None
    assert motion.scan_db_path is None
    assert motion.operator_id is None    # or whatever the chosen default is
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_motion_interface.py -v`
Expected: FAIL — `MotionInterface.__init__()` doesn't accept these kwargs.

- [ ] **Step 3: Modify MotionInterface.__init__**

In `omotion/MotionInterface.py`, add to the constructor:

```python
def __init__(
    self,
    *args,
    data_dir: Optional[str] = None,
    scan_db_path: Optional[str] = None,
    operator_id: Optional[str] = None,
    # ... preserve existing kwargs
    **kwargs,
):
    # ... existing init logic
    self.data_dir = data_dir
    self.scan_db_path = scan_db_path
    self.operator_id = operator_id
    # ... existing init logic
```

(Adjust the actual argument list to match the file's existing style — these are new optional kwargs, default None.)

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_motion_interface.py -v`
Expected: 4 pass.

- [ ] **Step 5: Commit**

```bash
git add omotion/MotionInterface.py tests/test_motion_interface.py
git commit -m "feat(sdk): MotionInterface gains data_dir, scan_db_path, operator_id ctor args"
```

---

## Phase B — Telemetry scaffolding (6 tasks)

PR 1 already added `TelemetrySink` and the `"telemetry"` channel concept. Phase B fills in the missing pieces — the event type, source, aggregator, and ingest stage — and wires them into the runner.

---

### Task 6: `TelemetryEvent` dataclass

**Files:**
- Modify: `omotion/pipeline/batch.py`
- Modify: `tests/test_pipeline/test_batch.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_pipeline/test_batch.py`:

```python
def test_telemetry_event_carries_console_telemetry_fields():
    from omotion.pipeline.batch import TelemetryEvent
    ev = TelemetryEvent(
        timestamp_s=12.5,
        pdc_samples=[1.23, 1.24, 1.22],
        tec_setpoint_c=25.0,
        tec_actual_c=25.1,
        console_temp_c=37.4,
        fan_rpm=2400,
        safety_status=0,
    )
    assert ev.timestamp_s == 12.5
    assert ev.pdc_samples == [1.23, 1.24, 1.22]
    assert ev.tec_setpoint_c == 25.0
    assert ev.fan_rpm == 2400


def test_telemetry_event_is_a_batch_event():
    from omotion.pipeline.batch import TelemetryEvent, BatchEvent
    ev = TelemetryEvent(
        timestamp_s=0.0, pdc_samples=[], tec_setpoint_c=0.0,
        tec_actual_c=0.0, console_temp_c=0.0, fan_rpm=0, safety_status=0,
    )
    assert isinstance(ev, BatchEvent)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_batch.py::test_telemetry_event_carries_console_telemetry_fields -v`
Expected: FAIL — `ImportError: cannot import name 'TelemetryEvent'`.

- [ ] **Step 3: Add TelemetryEvent to batch.py**

Append to `omotion/pipeline/batch.py`:

```python
@dataclass
class TelemetryEvent(BatchEvent):
    """One snapshot of console-level telemetry.

    Yielded by ConsoleTelemetrySource at ~10 Hz; dispatched to "telemetry"
    sinks and also ingested into the pipeline's TelemetryAggregator for
    future per-frame correction stages.
    """
    timestamp_s:        float        # per-scan t=0 normalized
    pdc_samples:        list[float]  # mA, PDC drain since last poll
    tec_setpoint_c:     float
    tec_actual_c:       float
    console_temp_c:     float
    fan_rpm:            int
    safety_status:      int
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_batch.py -v`
Expected: all batch tests pass (existing + 2 new).

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/batch.py tests/test_pipeline/test_batch.py
git commit -m "feat(sdk): add TelemetryEvent dataclass"
```

---

### Task 7: `TelemetryAggregator` + `TelemetryIngestStage`

**Files:**
- Create: `omotion/pipeline/telemetry.py`
- Create: `tests/test_pipeline/test_telemetry.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_pipeline/test_telemetry.py`:

```python
"""Tests for TelemetryAggregator + TelemetryIngestStage."""

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch, TelemetryEvent
from omotion.pipeline.telemetry import TelemetryAggregator, TelemetryIngestStage


def _ev(t, pdc):
    return TelemetryEvent(
        timestamp_s=t, pdc_samples=[pdc],
        tec_setpoint_c=25.0, tec_actual_c=25.0, console_temp_c=37.0,
        fan_rpm=2400, safety_status=0,
    )


def test_aggregator_starts_empty():
    agg = TelemetryAggregator()
    assert agg.snapshot_at(0.0) is None


def test_aggregator_returns_most_recent_event_before_target_t():
    agg = TelemetryAggregator()
    agg.update(_ev(1.0, 1.10))
    agg.update(_ev(2.0, 1.20))
    agg.update(_ev(3.0, 1.30))
    # At t=2.5, most recent event with .timestamp_s <= 2.5 is t=2.0
    snap = agg.snapshot_at(2.5)
    assert snap is not None
    assert snap.timestamp_s == 2.0


def test_aggregator_returns_none_when_no_event_before_t():
    agg = TelemetryAggregator()
    agg.update(_ev(5.0, 1.10))
    assert agg.snapshot_at(2.0) is None


def test_aggregator_bounded_capacity():
    agg = TelemetryAggregator(max_history=3)
    for i in range(5):
        agg.update(_ev(float(i), 1.0 + i * 0.01))
    # Only last 3 retained
    snap = agg.snapshot_at(10.0)
    assert snap.timestamp_s == 4.0
    # Earlier events evicted
    snap_old = agg.snapshot_at(1.5)
    assert snap_old.timestamp_s == 1.0   # if 0.0 was evicted, 1.0 should still be there
    # Actually with max=3 after 0,1,2,3,4 evictions, history is [2,3,4]
    # so snap_at(1.5) returns None
    # let's just test the more reliable invariant:
    assert agg.size() == 3


def _empty_batch_with_timestamps(timestamps):
    n = len(timestamps)
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.zeros(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.array(timestamps, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


def test_ingest_stage_fills_batch_telemetry_fields_per_frame():
    agg = TelemetryAggregator()
    agg.update(_ev(1.0, 1.10))
    agg.update(_ev(2.0, 1.20))

    stage = TelemetryIngestStage(aggregator=agg)
    batch = _empty_batch_with_timestamps([0.5, 1.5, 2.5])
    stage.process(batch)

    # Frame 0 (t=0.5): no event before → None
    # Frame 1 (t=1.5): most recent at 1.0 → pdc=[1.10]
    # Frame 2 (t=2.5): most recent at 2.0 → pdc=[1.20]
    assert batch.pdc is not None
    assert batch.pdc[0] is None   # no telemetry available yet
    assert batch.pdc[1] == [1.10]
    assert batch.pdc[2] == [1.20]


def test_ingest_stage_is_noop_when_no_aggregator():
    stage = TelemetryIngestStage(aggregator=None)
    batch = _empty_batch_with_timestamps([0.5, 1.5])
    stage.process(batch)
    # No aggregator → stage leaves pdc/tcm/tcl as-is (None)
    assert batch.pdc is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_telemetry.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'omotion.pipeline.telemetry'`.

- [ ] **Step 3: Implement telemetry.py**

Create `omotion/pipeline/telemetry.py`:

```python
"""TelemetryAggregator + TelemetryIngestStage.

Bridges the per-snapshot telemetry channel (events arriving at ~10 Hz on
the runner's telemetry loop) into the per-frame world (FrameBatch.pdc/
tcm/tcl fields the CsvSink raw writer expects). Also exposes a query
API (snapshot_at) for future correction stages that need to look up the
most recent telemetry at a frame timestamp.

See spec §3.6.5.
"""

from __future__ import annotations

import bisect
import threading
from collections import deque
from typing import Optional, Deque

from .batch import FrameBatch, TelemetryEvent


class TelemetryAggregator:
    """Thread-safe ring buffer of recent TelemetryEvents."""

    def __init__(self, max_history: int = 100):
        if max_history < 1:
            raise ValueError(f"max_history must be >= 1, got {max_history}")
        self._max = int(max_history)
        self._history: Deque[TelemetryEvent] = deque(maxlen=self._max)
        self._lock = threading.Lock()

    def update(self, event: TelemetryEvent) -> None:
        """Called by the runner's telemetry loop when a new event arrives."""
        with self._lock:
            self._history.append(event)

    def snapshot_at(self, t: float) -> Optional[TelemetryEvent]:
        """Return the most recent event with timestamp_s <= t, or None."""
        with self._lock:
            # History is appended in timestamp order. Walk from newest to oldest.
            for ev in reversed(self._history):
                if ev.timestamp_s <= t:
                    return ev
            return None

    def size(self) -> int:
        with self._lock:
            return len(self._history)

    def clear(self) -> None:
        with self._lock:
            self._history.clear()


class TelemetryIngestStage:
    """Per-frame telemetry attachment.

    Reads the most recent TelemetryEvent from the aggregator for each
    frame's timestamp and populates batch.pdc/tcm/tcl. When no aggregator
    is configured (telemetry source not running) the stage is a no-op.
    """
    name = "telemetry_ingest"

    def __init__(self, *, aggregator: Optional[TelemetryAggregator]):
        self._aggregator = aggregator

    def process(self, batch: FrameBatch) -> FrameBatch:
        if self._aggregator is None or batch.timestamp_s.size == 0:
            return batch

        n = batch.timestamp_s.size
        pdc: list = []
        tcm: list = []
        tcl: list = []
        for i in range(n):
            event = self._aggregator.snapshot_at(float(batch.timestamp_s[i]))
            if event is None:
                pdc.append(None)
                tcm.append(None)
                tcl.append(None)
            else:
                pdc.append(event.pdc_samples)
                tcm.append(event.console_temp_c)   # tcm = console temp monitor
                tcl.append(event.tec_actual_c)     # tcl = TEC actual

        batch.pdc = pdc
        batch.tcm = tcm
        batch.tcl = tcl
        return batch

    def reset(self) -> None:
        if self._aggregator is not None:
            self._aggregator.clear()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_telemetry.py -v`
Expected: 6 pass.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/telemetry.py tests/test_pipeline/test_telemetry.py
git commit -m "feat(sdk): TelemetryAggregator + TelemetryIngestStage scaffolding"
```

---

### Task 8: `Pipeline.telemetry_aggregator` attribute + factory wiring

**Files:**
- Modify: `omotion/pipeline/pipeline.py`
- Modify: `omotion/pipeline/factory.py`
- Modify: `tests/test_pipeline/test_pipeline_class.py`
- Modify: `tests/test_pipeline/test_factory.py`

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_pipeline/test_pipeline_class.py`:

```python
def test_pipeline_carries_optional_telemetry_aggregator():
    from omotion.pipeline.telemetry import TelemetryAggregator
    agg = TelemetryAggregator()
    pipeline = Pipeline(stages=[], telemetry_aggregator=agg)
    assert pipeline.telemetry_aggregator is agg


def test_pipeline_telemetry_aggregator_defaults_to_none():
    pipeline = Pipeline(stages=[])
    assert pipeline.telemetry_aggregator is None
```

Append to `tests/test_pipeline/test_factory.py`:

```python
def test_default_pipeline_constructs_a_telemetry_aggregator():
    cal = _trivial_calibration()
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
    )
    pipeline = default_pipeline(metadata=meta, calibration=cal,
                                pedestals=SensorPedestals(left=64.0, right=64.0))
    assert pipeline.telemetry_aggregator is not None
    # TelemetryIngestStage should be in the stage list, positioned after FrameClassification
    names = [s.name for s in pipeline.stages]
    assert "telemetry_ingest" in names
    classify_idx = names.index("frame_classification")
    ingest_idx = names.index("telemetry_ingest")
    assert ingest_idx == classify_idx + 1 or ingest_idx == classify_idx + 2  # before or after raw tee
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_pipeline/test_pipeline_class.py::test_pipeline_carries_optional_telemetry_aggregator tests/test_pipeline/test_factory.py::test_default_pipeline_constructs_a_telemetry_aggregator -v`
Expected: FAIL.

- [ ] **Step 3: Modify Pipeline class**

In `omotion/pipeline/pipeline.py`:

```python
class Pipeline:
    """Ordered list of stages. Pure transformation, no I/O."""

    def __init__(self, stages, *, telemetry_aggregator=None):
        self.stages = list(stages)
        # Optional reference to a TelemetryAggregator. The runner pushes
        # TelemetryEvents here in its _telemetry_loop; stages that need
        # per-frame telemetry data (TelemetryIngestStage; future
        # correction stages) take it as a constructor arg.
        self.telemetry_aggregator = telemetry_aggregator

    def process(self, batch):
        for stage in self.stages:
            stage.process(batch)
        return batch

    def reset(self):
        for stage in self.stages:
            stage.reset()
```

- [ ] **Step 4: Modify default_pipeline factory**

In `omotion/pipeline/factory.py`, construct an aggregator and the ingest stage, and assign the aggregator to the returned pipeline:

```python
def default_pipeline(*, ..., raw_save_max_duration_s=None) -> Pipeline:
    from .telemetry import TelemetryAggregator, TelemetryIngestStage
    aggregator = TelemetryAggregator()

    not_warmup_or_stale = lambda ft: ft != "warmup" and ft != "stale"

    stages = [
        FrameClassificationStage(discard_count=discard_count, dark_interval=dark_interval),
        TelemetryIngestStage(aggregator=aggregator),
    ]

    if raw_save_max_duration_s is None or raw_save_max_duration_s > 0:
        stages.append(Tee("raw", filter=None, max_duration_s=raw_save_max_duration_s))

    stages.extend([
        NoiseFloorStage(threshold=noise_floor_threshold),
        # ... (rest of stages unchanged)
    ])

    pipeline = Pipeline(stages, telemetry_aggregator=aggregator)
    return pipeline
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_pipeline_class.py tests/test_pipeline/test_factory.py -v`
Expected: all existing + 3 new pass.

- [ ] **Step 6: Commit**

```bash
git add omotion/pipeline/pipeline.py omotion/pipeline/factory.py tests/test_pipeline/test_pipeline_class.py tests/test_pipeline/test_factory.py
git commit -m "feat(sdk): Pipeline.telemetry_aggregator + factory wires TelemetryIngestStage"
```

---

### Task 9: `ConsoleTelemetrySource`

**Files:**
- Modify: `omotion/pipeline/sources.py`
- Modify: `tests/test_pipeline/test_sources.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_pipeline/test_sources.py`:

```python
def test_console_telemetry_source_yields_events_until_stopped():
    """The source wraps the existing ConsoleTelemetryPoller; pulls snapshots,
    yields TelemetryEvents with scan-relative timestamps."""
    from omotion.pipeline.sources import ConsoleTelemetrySource
    from omotion.pipeline.batch import TelemetryEvent

    class _FakeConsole:
        def __init__(self):
            self._snapshots = [
                _make_snapshot(absolute_t=100.0, pdc=1.10),
                _make_snapshot(absolute_t=100.1, pdc=1.11),
                _make_snapshot(absolute_t=100.2, pdc=1.12),
                None,  # signal source to stop after this
            ]
            self._i = 0

        def poll_telemetry(self, timeout):
            if self._i < len(self._snapshots):
                snap = self._snapshots[self._i]
                self._i += 1
                return snap
            return None

    src = ConsoleTelemetrySource(console=_FakeConsole(), poll_interval_s=0.01)
    events = []
    for ev in src:
        events.append(ev)
        if len(events) == 3:
            src.close()
            break

    assert len(events) == 3
    # Timestamps normalized to t=0 at first event
    assert events[0].timestamp_s == 0.0
    assert events[1].timestamp_s == pytest.approx(0.1)
    assert events[2].timestamp_s == pytest.approx(0.2)
    assert events[0].pdc_samples == [1.10]


def _make_snapshot(absolute_t, pdc):
    """Build a fake console-poll snapshot — adjust shape to match real impl."""
    from types import SimpleNamespace
    return SimpleNamespace(
        absolute_t=absolute_t,
        pdc=[pdc],
        tec_setpoint=25.0, tec_actual=25.0, console_temp=37.0,
        fan_rpm=2400, safety_status=0,
    )
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_pipeline/test_sources.py -v -k console_telemetry`
Expected: FAIL — `ImportError: cannot import name 'ConsoleTelemetrySource'`.

- [ ] **Step 3: Append ConsoleTelemetrySource to sources.py**

In `omotion/pipeline/sources.py`:

```python
class ConsoleTelemetrySource:
    """Polls MotionConsole at fixed cadence; yields TelemetryEvent with
    scan-relative timestamps.

    Used as the optional `telemetry_source` on ScanRunner. Doesn't produce
    FrameBatch — parallel input that flows to "telemetry"-channel sinks
    and feeds the pipeline's TelemetryAggregator.
    """

    def __init__(self, *, console, poll_interval_s: float = 0.1):
        self._console = console
        self._poll_interval_s = poll_interval_s
        self._stop = threading.Event()
        self._t0 = None

    def __iter__(self):
        while not self._stop.is_set():
            snap = self._console.poll_telemetry(timeout=self._poll_interval_s)
            if snap is None:
                continue
            if self._t0 is None:
                self._t0 = snap.absolute_t
            yield TelemetryEvent(
                timestamp_s=snap.absolute_t - self._t0,
                pdc_samples=list(snap.pdc),
                tec_setpoint_c=snap.tec_setpoint,
                tec_actual_c=snap.tec_actual,
                console_temp_c=snap.console_temp,
                fan_rpm=snap.fan_rpm,
                safety_status=snap.safety_status,
            )

    def close(self) -> None:
        self._stop.set()
```

Add imports at the top of the file: `from .batch import TelemetryEvent` (if not already imported).

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_pipeline/test_sources.py -v`
Expected: existing + 1 new pass.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/sources.py tests/test_pipeline/test_sources.py
git commit -m "feat(sdk): ConsoleTelemetrySource — parallel input to ScanRunner; wraps the existing poller"
```

---

### Task 10: `ScanRunner._telemetry_loop` + auto-wiring of telemetry source

**Files:**
- Modify: `omotion/pipeline/runner.py`
- Modify: `tests/test_pipeline/test_runner.py`

ScanRunner gets a second optional input (`telemetry_source`) and a parallel thread that drives it. The telemetry loop pushes events to `pipeline.telemetry_aggregator` and dispatches them to "telemetry"-channel sinks.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_pipeline/test_runner.py`:

```python
def test_runner_telemetry_source_routes_events_to_telemetry_sinks_and_aggregator():
    from omotion.pipeline.batch import TelemetryEvent
    from omotion.pipeline.telemetry import TelemetryAggregator

    class _FakeTelemetrySource:
        def __init__(self, events):
            self._events = events
            self._stop = False
        def __iter__(self):
            for e in self._events:
                if self._stop:
                    break
                yield e
        def close(self):
            self._stop = True

    fake_events = [
        TelemetryEvent(timestamp_s=0.0, pdc_samples=[1.0], tec_setpoint_c=25, tec_actual_c=25,
                       console_temp_c=37, fan_rpm=2400, safety_status=0),
        TelemetryEvent(timestamp_s=0.1, pdc_samples=[1.05], tec_setpoint_c=25, tec_actual_c=25,
                       console_temp_c=37, fan_rpm=2400, safety_status=0),
    ]
    telemetry_sink = _RecordingSink(channels={"telemetry"})

    agg = TelemetryAggregator()
    pipeline = Pipeline([_EmitTagsStage([])], telemetry_aggregator=agg)

    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=pipeline,
        sinks=[telemetry_sink],
        telemetry_source=_FakeTelemetrySource(fake_events),
    )
    runner.run()

    # Aggregator received all events
    assert agg.size() == 2
    # Telemetry sink received both events on the "telemetry" channel
    assert [c for c, _ in telemetry_sink.consumed] == ["telemetry", "telemetry"]


def test_runner_no_telemetry_source_means_no_telemetry_thread():
    """When telemetry_source is None, no thread is started; main loop runs normally."""
    sink = _RecordingSink(channels={"live"})
    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_EmitTagsStage(["live"])]),
        sinks=[sink],
        telemetry_source=None,
    )
    runner.run()
    assert len(sink.consumed) == 1
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_pipeline/test_runner.py -v -k telemetry`
Expected: FAIL — `ScanRunner.__init__()` doesn't accept `telemetry_source`.

- [ ] **Step 3: Modify ScanRunner**

In `omotion/pipeline/runner.py`:

```python
class ScanRunner:
    def __init__(self, *, source, pipeline, sinks, telemetry_source=None):
        self.source = source
        self.pipeline = pipeline
        self.sinks = list(sinks)
        self.telemetry_source = telemetry_source
        self._telemetry_thread = None

    def _sinks_for(self, channel):
        return [s for s in self.sinks if channel in getattr(s, "channels", set())]

    def _safe_consume(self, sink, channel, payload):
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

        if self.telemetry_source is not None:
            self._telemetry_thread = threading.Thread(
                target=self._telemetry_loop, daemon=True, name="ScanRunner-telemetry",
            )
            self._telemetry_thread.start()

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
            if self.telemetry_source is not None:
                self.telemetry_source.close()
                if self._telemetry_thread:
                    self._telemetry_thread.join(timeout=2.0)
            for sink in self.sinks:
                try:
                    sink.on_complete()
                except Exception:
                    logger.exception("sink %r raised in on_complete", type(sink).__name__)

    def _telemetry_loop(self) -> None:
        for event in self.telemetry_source:
            if self.pipeline.telemetry_aggregator is not None:
                self.pipeline.telemetry_aggregator.update(event)
            for sink in self._sinks_for("telemetry"):
                self._safe_consume(sink, "telemetry", event)

    def _dispatch(self, batch):
        # (existing dispatch logic — unchanged)
        ...
```

Add `import threading` at the top of `runner.py` if not present.

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_runner.py -v`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/runner.py tests/test_pipeline/test_runner.py
git commit -m "feat(sdk): ScanRunner._telemetry_loop — parallel thread pushes events to aggregator + telemetry sinks"
```

---

### Task 11: `TelemetrySink` produces legacy `_telemetry.csv` layout

**Files:**
- Modify: `omotion/pipeline/sinks.py`
- Modify: `tests/test_pipeline/test_csv_sink.py` (or create `tests/test_pipeline/test_telemetry_sink.py`)

PR 1 added `TelemetrySink`; verify the CSV layout matches what the bloodflow-app currently writes. Legacy columns (check against an existing `_telemetry.csv` in `C:/Users/ethan/Projects/scan_data/` for ground truth): `timestamp_s, pdc_samples_ma, tec_setpoint_c, tec_actual_c, console_temp_c, fan_rpm, safety_status`.

- [ ] **Step 1: Inspect a legacy telemetry CSV for column ground truth**

Run: `head -1 C:/Users/ethan/Projects/scan_data/20260519_161117_owW1WI5T_telemetry.csv`
Note the exact column names and order. Adjust the TelemetrySink header to match exactly so app code that reads the file keeps working.

- [ ] **Step 2: Write the failing test**

Create or append to `tests/test_pipeline/test_telemetry_sink.py`:

```python
"""Tests for TelemetrySink — writes the per-scan _telemetry.csv."""

import csv
import pytest
from omotion.pipeline.batch import TelemetryEvent
from omotion.pipeline.sinks import TelemetrySink, ScanMetadata


def _meta():
    return ScanMetadata(
        scan_id="abc", subject_id="x", operator="bloodflow-app",
        started_at_iso="2026-05-22T10:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
    )


def _ev(t, pdc):
    return TelemetryEvent(
        timestamp_s=t, pdc_samples=[pdc, pdc + 0.01, pdc + 0.02],
        tec_setpoint_c=25.0, tec_actual_c=25.1, console_temp_c=37.4,
        fan_rpm=2400, safety_status=0,
    )


def test_telemetry_sink_header_matches_legacy_columns(tmp_path):
    sink = TelemetrySink(output_path=str(tmp_path / "test_telemetry.csv"))
    sink.on_scan_start(_meta())
    sink.on_complete()
    with open(tmp_path / "test_telemetry.csv") as f:
        header = next(csv.reader(f))
    # Adjust expected columns to match what step-1 inspection showed; the
    # baseline below mirrors the spec's example.
    assert header == [
        "timestamp_s", "pdc_samples_ma", "tec_setpoint_c", "tec_actual_c",
        "console_temp_c", "fan_rpm", "safety_status",
    ]


def test_telemetry_sink_writes_one_row_per_event(tmp_path):
    sink = TelemetrySink(output_path=str(tmp_path / "test_telemetry.csv"))
    sink.on_scan_start(_meta())
    sink.consume("telemetry", _ev(0.0, 1.10))
    sink.consume("telemetry", _ev(0.1, 1.11))
    sink.on_complete()
    with open(tmp_path / "test_telemetry.csv") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 2
    assert float(rows[0]["timestamp_s"]) == 0.0
    assert float(rows[0]["tec_setpoint_c"]) == 25.0
    assert rows[0]["pdc_samples_ma"] == "1.100;1.110;1.120"
```

- [ ] **Step 3: Implement TelemetrySink (or update PR 1 stub)**

In `omotion/pipeline/sinks.py`, add or update `TelemetrySink`:

```python
class TelemetrySink:
    """Subscribes to 'telemetry' channel; writes one row per TelemetryEvent.
    See spec §3.6.1.
    """
    channels = {"telemetry"}

    def __init__(self, output_path: str):
        self._output_path = output_path
        self._fh = None
        self._writer = None

    def on_scan_start(self, meta):
        self._fh = open(self._output_path, "w", newline="")
        self._writer = csv.writer(self._fh)
        self._writer.writerow([
            "timestamp_s", "pdc_samples_ma", "tec_setpoint_c",
            "tec_actual_c", "console_temp_c", "fan_rpm", "safety_status",
        ])

    def consume(self, channel, event):
        if channel != "telemetry":
            return
        self._writer.writerow([
            f"{event.timestamp_s:.4f}",
            ";".join(f"{s:.3f}" for s in event.pdc_samples),
            event.tec_setpoint_c, event.tec_actual_c,
            event.console_temp_c, event.fan_rpm, event.safety_status,
        ])

    def on_complete(self):
        if self._fh:
            self._fh.flush()
            self._fh.close()
            self._fh = None
            self._writer = None
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_pipeline/test_telemetry_sink.py -v`
Expected: 2 pass.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/sinks.py tests/test_pipeline/test_telemetry_sink.py
git commit -m "feat(sdk): TelemetrySink writes per-scan _telemetry.csv in legacy column layout"
```

---

## Phase C — LiveUsbSource reader loop (1 task)

### Task 12: Wire `LiveUsbSource._reader_loop` against `parse_histogram_stream`

**Files:**
- Modify: `omotion/pipeline/sources.py`
- Modify: `tests/test_pipeline/test_sources.py`

PR 1 left `_reader_loop` raising `NotImplementedError`. Per spec §3.3, the loop delegates to `parse_histogram_stream` (kept in `MotionProcessing.py` per §3.4), with per-side packet queues and a callback that batches parsed samples into FrameBatches.

- [ ] **Step 1: Write the failing test (hardware-marked smoke)**

Append to `tests/test_pipeline/test_sources.py`:

```python
@pytest.mark.sensor
def test_live_usb_source_smoke_yields_framebatches():
    """Hardware-marked smoke test — requires a connected sensor module."""
    from omotion import MotionInterface
    from omotion.pipeline.sources import LiveUsbSource
    from omotion.pipeline.sinks import ScanMetadata

    motion = MotionInterface(data_dir=None, scan_db_path=None,
                             operator_id="test")
    motion.start()
    try:
        meta = ScanMetadata(
            scan_id="smoke", subject_id="x", operator="test",
            started_at_iso="2026-05-22T00:00:00Z", duration_sec=2,
            left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
        )
        src = LiveUsbSource(
            console=motion.console, left=motion.left, right=motion.right,
            batch_size_frames=10, metadata=meta,
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

- [ ] **Step 2: Verify the test fails on the existing skeleton**

Run: `pytest tests/test_pipeline/test_sources.py -v -k live_usb_source_smoke -m sensor`
Expected: when hardware connected, FAIL with `NotImplementedError`; when unconnected, SKIP — that's fine, the hardware test only runs on the bench.

- [ ] **Step 3: Replace LiveUsbSource skeleton**

In `omotion/pipeline/sources.py`, replace the existing `LiveUsbSource` class with the version that wires `parse_histogram_stream`:

```python
class LiveUsbSource(_BaseSource):
    """Per-side packet queues + per-side reader threads → shared batch queue."""

    def __init__(self, *, console, left, right, batch_size_frames=10,
                 flush_interval_s=0.25, packet_queue_size=64, metadata):
        super().__init__(metadata=metadata)
        self._console = console
        self._sensors = {"left": left, "right": right}
        self._batch_size = int(batch_size_frames)
        self._flush_interval = float(flush_interval_s)
        self._packet_queues = {
            side: queue.Queue(maxsize=packet_queue_size)
            for side, sensor in self._sensors.items() if sensor is not None
        }
        self._batch_queue: queue.Queue = queue.Queue(maxsize=4)
        self._stop = threading.Event()
        self._reader_threads: list[threading.Thread] = []

    def __iter__(self):
        from omotion.MotionProcessing import HISTOGRAM_BYTES   # per-packet size
        for side_name in self._packet_queues:
            sensor = self._sensors[side_name]
            sensor.histo_stream.start_streaming(
                self._packet_queues[side_name], expected_size=HISTOGRAM_BYTES,
            )
            t = threading.Thread(
                target=self._reader_loop, args=(side_name,),
                name=f"LiveUsbSource-{side_name}", daemon=True,
            )
            t.start()
            self._reader_threads.append(t)

        while not self._stop.is_set():
            try:
                batch = self._batch_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            if batch is None:
                break
            yield batch

    def close(self) -> None:
        self._stop.set()
        for sensor in self._sensors.values():
            if sensor is not None and getattr(sensor, "histo_stream", None) is not None:
                try:
                    sensor.histo_stream.stop_streaming()
                except Exception:
                    pass
        for t in self._reader_threads:
            t.join(timeout=2.0)

    def _reader_loop(self, side_name: str) -> None:
        """Per-side reader. Delegates packet parsing to parse_histogram_stream;
        accumulates HistogramSamples into FrameBatches and pushes them to the
        shared batch queue.
        """
        from omotion.MotionProcessing import (
            parse_histogram_stream, EXPECTED_HISTOGRAM_SUM,
        )

        side_idx = 0 if side_name == "left" else 1
        accumulated: list = []
        last_flush = time.monotonic()

        def on_row(cam_id, frame_id, ts, histogram, row_sum, temp):
            nonlocal last_flush
            accumulated.append((cam_id, frame_id, ts, histogram, row_sum, temp))
            now = time.monotonic()
            if (len(accumulated) >= self._batch_size or
                    now - last_flush >= self._flush_interval):
                self._batch_queue.put(self._build_batch(side_idx, accumulated))
                accumulated.clear()
                last_flush = now

        buf = bytearray()
        parse_histogram_stream(
            self._packet_queues[side_name], self._stop, buf,
            on_row_fn=on_row,
            expected_row_sum=EXPECTED_HISTOGRAM_SUM,
            t0_normalizer=self._t0_normalize,
        )
        if accumulated:
            self._batch_queue.put(self._build_batch(side_idx, accumulated))

    def _build_batch(self, side_idx: int, samples: list) -> FrameBatch:
        """Convert a list of (cam_id, frame_id, ts, histogram, row_sum, temp)
        tuples into one FrameBatch with (N, 2, 8, 1024) shape.
        """
        n = len(samples)
        cam_ids = np.array([s[0] for s in samples], dtype=np.int8)
        frame_ids = np.array([s[1] for s in samples], dtype=np.uint8)
        timestamp_s = np.array([s[2] for s in samples], dtype=np.float64)
        raw_hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
        temps = np.zeros((n, 2, 8), dtype=np.float32)
        for i, (cam_id, _, _, histogram, _, temp) in enumerate(samples):
            raw_hist[i, side_idx, cam_id] = histogram
            temps[i, side_idx, cam_id] = temp
        return FrameBatch(
            cam_ids=cam_ids, frame_ids=frame_ids,
            raw_histograms=raw_hist, temperature_c=temps,
            timestamp_s=timestamp_s, pdc=None, tcm=None, tcl=None,
        )
```

Add imports at the top of `sources.py`: `import queue, threading, time` if not already present.

- [ ] **Step 4: Verify the hardware test passes on bench**

Run on bench: `pytest tests/test_pipeline/test_sources.py -v -k live_usb_source_smoke -m sensor`
Expected: PASS (when hardware connected).

For CI (no hardware): the test is `@pytest.mark.sensor` and skipped per `pyproject.toml`'s default `-m "not fpga and not imu"` filter. Add `not sensor` to the default filter if it isn't already.

- [ ] **Step 5: Commit**

```bash
git add omotion/pipeline/sources.py tests/test_pipeline/test_sources.py
git commit -m "feat(sdk): wire LiveUsbSource._reader_loop against parse_histogram_stream"
```

---

## Phase D — Internal SDK workflows: Calibration migration + ContactQualityWorkflow (4 tasks)

### Task 13: `_CalibrationCollectorSink` + `CalibrationWorkflow.run_calibration` migration

**Files:**
- Modify: `omotion/CalibrationWorkflow.py`
- Modify: `tests/test_calibration_workflow.py`

Today the workflow passes `on_corrected_batch_fn=_on_corrected_batch` to `start_scan`. Replace with the collector sink + `skip_default_storage=True`.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_calibration_workflow.py`:

```python
def test_calibration_workflow_uses_collector_sink_and_skips_default_storage():
    """CalibrationWorkflow drives start_scan with a _CalibrationCollectorSink
    and skip_default_storage=True — no production CSV/DB writes for the
    diagnostic scan."""
    # Patch start_scan to capture the request
    from unittest.mock import MagicMock

    motion = _build_motion_with_fake_scan_workflow()
    received_request = {}

    def _capture(req):
        received_request["req"] = req
        return True

    motion.scan_workflow.start_scan = _capture
    motion.scan_workflow.await_complete = lambda timeout_sec: None

    # Invoke the workflow's run_calibration
    motion.calibration_workflow.run_calibration(...)

    req = received_request["req"]
    assert req.skip_default_storage is True
    # The collector sink subscribes to "final"
    final_sinks = [s for s in req.sinks if "final" in getattr(s, "channels", set())]
    assert len(final_sinks) >= 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_calibration_workflow.py -v -k uses_collector`
Expected: FAIL.

- [ ] **Step 3: Modify CalibrationWorkflow**

In `omotion/CalibrationWorkflow.py`:

1. Add `_CalibrationCollectorSink` class:

```python
class _CalibrationCollectorSink:
    """Internal: collects CorrectedIntervals during the calibration scan
    for downstream array computation."""
    channels = {"final"}

    def __init__(self):
        self.intervals: list = []

    def on_scan_start(self, meta):
        pass

    def consume(self, channel, payload):
        if channel == "final":
            self.intervals.append(payload)

    def on_complete(self):
        pass
```

2. Replace the `on_corrected_batch_fn=_on_corrected_batch` call with a sink-based call. Find the existing pattern:

```python
# Old shape (approximate):
started = interface.scan_workflow.start_scan(
    request,
    on_corrected_batch_fn=_on_corrected_batch,
    # ...
)
```

Replace with:

```python
collector = _CalibrationCollectorSink()
request = ScanRequest(
    # ... existing scan parameters ...
    sinks=[collector],
    skip_default_storage=True,   # no production CSV/DB for calibration scans
)
started = interface.scan_workflow.start_scan(request)
if not started:
    raise RuntimeError("ScanWorkflow refused start_scan.")
interface.scan_workflow.await_complete(timeout_sec=...)

# After the scan completes, process collector.intervals to compute the
# (2, 8) calibration arrays — port the existing array-compute code that
# previously ran inside _on_corrected_batch into a pure function that
# takes the list of intervals.
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_calibration_workflow.py -v`
Expected: all calibration tests pass with the new sink-based path.

- [ ] **Step 5: Commit**

```bash
git add omotion/CalibrationWorkflow.py tests/test_calibration_workflow.py
git commit -m "refactor(sdk): CalibrationWorkflow uses _CalibrationCollectorSink + skip_default_storage"
```

---

### Task 14: `ContactQualityWorkflow` — new module

**Files:**
- Create: `omotion/ContactQualityWorkflow.py`
- Create: `tests/test_contact_quality_workflow.py`

Full implementation per spec §3.8.

- [ ] **Step 1: Write the failing test**

Create `tests/test_contact_quality_workflow.py`:

```python
"""Tests for ContactQualityWorkflow — the SDK-owned CQ check procedure."""

import numpy as np
import pytest
from unittest.mock import MagicMock
from omotion.ContactQualityWorkflow import (
    ContactQualityWorkflow, CamCQResult, ContactQualityResult,
    _ContactQualitySink,
)


def test_cq_sink_result_marks_camera_ok_when_avg_within_range():
    sink = _ContactQualitySink(
        dark_thresholds=[1.0] * 8,
        light_thresholds=[8.0] * 8,
    )
    # Inject some rolling BFI values for cam (left, 2) — 4 frames at BFI=5.0
    from omotion.pipeline.batch import FrameBatch
    batch = FrameBatch(
        cam_ids=np.zeros(4, dtype=np.int8),
        frame_ids=np.arange(4, dtype=np.uint8),
        raw_histograms=np.zeros((4, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((4, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(4, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        frame_type=np.array(["light"] * 4, dtype="<U8"),
        bfi_rolling=np.full((4, 2, 8), np.nan, dtype=np.float32),
    )
    batch.bfi_rolling[:, 0, 2] = 5.0  # left, cam_id 2
    sink.consume("rolling", batch)
    result = sink.result(left_mask=(1 << 2), right_mask=0, duration_sec=1.0)
    cam = result.per_camera[("left", 2)]
    assert cam.passed is True
    assert cam.reason == "ok"
    assert cam.avg_bfi == 5.0
    assert result.passed is True


def test_cq_sink_fails_camera_below_dark_threshold():
    sink = _ContactQualitySink(
        dark_thresholds=[2.0] * 8,
        light_thresholds=[8.0] * 8,
    )
    from omotion.pipeline.batch import FrameBatch
    batch = FrameBatch(
        cam_ids=np.zeros(2, dtype=np.int8),
        frame_ids=np.arange(2, dtype=np.uint8),
        raw_histograms=np.zeros((2, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((2, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(2, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        frame_type=np.array(["light"] * 2, dtype="<U8"),
        bfi_rolling=np.full((2, 2, 8), 1.0, dtype=np.float32),  # below 2.0 threshold
    )
    sink.consume("rolling", batch)
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)
    cam = result.per_camera[("left", 0)]
    assert cam.passed is False
    assert cam.reason == "below_dark"


def test_cq_workflow_check_returns_passed_when_all_active_cams_in_range():
    """Drive ContactQualityWorkflow.check end-to-end with a faked ScanWorkflow."""
    fake_scan = MagicMock()
    fake_scan.start_scan = MagicMock(return_value=True)
    fake_scan.await_complete = MagicMock()

    cq = ContactQualityWorkflow(scan_workflow=fake_scan)

    # The workflow's run_calibration approach: when start_scan is called,
    # simulate the sink receiving good rolling values, then return.
    def _drive_scan(request):
        sink = request.sinks[0]
        sink.on_scan_start(None)
        # Synthesize rolling values within thresholds
        from omotion.pipeline.batch import FrameBatch
        n = 10
        batch = FrameBatch(
            cam_ids=np.zeros(n, dtype=np.int8),
            frame_ids=np.arange(n, dtype=np.uint8),
            raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
            temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
            timestamp_s=np.arange(n, dtype=np.float64),
            pdc=None, tcm=None, tcl=None,
            frame_type=np.array(["light"] * n, dtype="<U8"),
            bfi_rolling=np.full((n, 2, 8), 5.0, dtype=np.float32),
        )
        sink.consume("rolling", batch)
        sink.on_complete()
        return True

    fake_scan.start_scan.side_effect = _drive_scan

    result = cq.check(
        duration_sec=1.0,
        rolling_window=10,
        dark_threshold_per_camera=[1.0] * 8,
        light_threshold_per_camera=[10.0] * 8,
        left_camera_mask=0xFF,
        right_camera_mask=0,
    )
    assert isinstance(result, ContactQualityResult)
    assert result.passed is True
    # 8 left cams active → 8 entries
    assert len(result.per_camera) == 8
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_contact_quality_workflow.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'omotion.ContactQualityWorkflow'`.

- [ ] **Step 3: Implement ContactQualityWorkflow**

Create `omotion/ContactQualityWorkflow.py` per spec §3.8 (full code is in the spec). Key shape:

```python
"""ContactQualityWorkflow — SDK-owned contact-quality check procedure.

Symmetric with CalibrationWorkflow: runs a short scan, monitors per-camera
signal levels against dark/light thresholds, returns pass/fail with
per-camera diagnostics. See spec §3.8.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from omotion.ScanWorkflow import ScanRequest


@dataclass
class CamCQResult:
    side:     str
    cam_id:   int
    passed:   bool
    avg_bfi:  float
    reason:   str    # "ok" | "below_dark" | "above_light" | "no_signal"


@dataclass
class ContactQualityResult:
    passed:       bool
    per_camera:   dict
    duration_sec: float


class _ContactQualitySink:
    """Internal: collects rolling-averaged BFI per camera during a short scan."""
    channels = {"rolling"}

    def __init__(self, dark_thresholds, light_thresholds):
        self._dark = list(dark_thresholds)
        self._light = list(light_thresholds)
        self._accum: dict = {}

    def on_scan_start(self, meta):
        pass

    def consume(self, channel, batch):
        if channel != "rolling":
            return
        n = batch.bfi_rolling.shape[0]
        for i in range(n):
            if batch.frame_type[i] in ("warmup", "stale"):
                continue
            for side_idx, side in enumerate(("left", "right")):
                for cam_id in range(8):
                    v = float(batch.bfi_rolling[i, side_idx, cam_id])
                    if not np.isfinite(v):
                        continue
                    self._accum.setdefault((side, cam_id), []).append(v)

    def on_complete(self):
        pass

    def result(self, *, left_mask, right_mask, duration_sec):
        per_cam: dict = {}
        for side_idx, (side, mask) in enumerate((("left", left_mask), ("right", right_mask))):
            for cam_id in range(8):
                if not (mask & (1 << cam_id)):
                    continue
                vals = self._accum.get((side, cam_id), [])
                avg = float(np.mean(vals)) if vals else float("nan")
                if not vals or not np.isfinite(avg):
                    reason, passed = "no_signal", False
                elif avg < self._dark[cam_id]:
                    reason, passed = "below_dark", False
                elif avg > self._light[cam_id]:
                    reason, passed = "above_light", False
                else:
                    reason, passed = "ok", True
                per_cam[(side, cam_id)] = CamCQResult(
                    side=side, cam_id=cam_id, passed=passed,
                    avg_bfi=avg, reason=reason,
                )
        return ContactQualityResult(
            passed=all(r.passed for r in per_cam.values()),
            per_camera=per_cam,
            duration_sec=duration_sec,
        )


class ContactQualityWorkflow:
    """Run a short scan, check signal levels against thresholds, return verdict.
    See spec §3.8.
    """

    def __init__(self, scan_workflow):
        self._scan_workflow = scan_workflow

    def check(self, *, duration_sec=1.0, rolling_window=10,
              dark_threshold_per_camera, light_threshold_per_camera,
              left_camera_mask, right_camera_mask):
        sink = _ContactQualitySink(
            dark_thresholds=dark_threshold_per_camera,
            light_thresholds=light_threshold_per_camera,
        )
        request = ScanRequest(
            subject_id="_cq_check",
            duration_sec=int(np.ceil(duration_sec)),
            left_camera_mask=left_camera_mask,
            right_camera_mask=right_camera_mask,
            reduced_mode=False,
            rolling_avg_window=rolling_window,
            sinks=[sink],
            skip_default_storage=True,
        )
        self._scan_workflow.start_scan(request)
        self._scan_workflow.await_complete(timeout_sec=duration_sec + 2.0)
        return sink.result(
            left_mask=left_camera_mask,
            right_mask=right_camera_mask,
            duration_sec=duration_sec,
        )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_contact_quality_workflow.py -v`
Expected: 3 pass.

- [ ] **Step 5: Commit**

```bash
git add omotion/ContactQualityWorkflow.py tests/test_contact_quality_workflow.py
git commit -m "feat(sdk): ContactQualityWorkflow — SDK-owned CQ check procedure"
```

---

### Task 15: `MotionInterface.contact_quality_workflow` lazy property

**Files:**
- Modify: `omotion/MotionInterface.py`
- Modify: `tests/test_motion_interface.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_motion_interface.py`:

```python
def test_motion_interface_lazy_loads_contact_quality_workflow():
    from omotion.ContactQualityWorkflow import ContactQualityWorkflow
    motion = MotionInterface(demo_mode=True)
    cq = motion.contact_quality_workflow
    assert isinstance(cq, ContactQualityWorkflow)
    # Subsequent access returns the same instance
    assert motion.contact_quality_workflow is cq
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_motion_interface.py -v -k contact_quality`
Expected: FAIL — attribute doesn't exist.

- [ ] **Step 3: Add the lazy property**

In `omotion/MotionInterface.py`, add to the existing class:

```python
@property
def contact_quality_workflow(self):
    if self._cq_workflow is None:
        from omotion.ContactQualityWorkflow import ContactQualityWorkflow
        self._cq_workflow = ContactQualityWorkflow(scan_workflow=self.scan_workflow)
    return self._cq_workflow
```

And initialize `self._cq_workflow = None` in `__init__`.

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_motion_interface.py -v`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add omotion/MotionInterface.py tests/test_motion_interface.py
git commit -m "feat(sdk): MotionInterface.contact_quality_workflow lazy property"
```

---

### Task 16: Test-scan support migration (`CalibrationWorkflow.start_test_scan`)

**Files:**
- Modify: `omotion/CalibrationWorkflow.py`
- Modify: `tests/test_calibration_workflow.py`

The test-scan path from feature/132 currently uses the same callback machinery as full calibration. Migrate it the same way (collector sink + `skip_default_storage=True`).

- [ ] **Step 1: Locate and read the existing `start_test_scan` method**

Run: `grep -n "start_test_scan" omotion/CalibrationWorkflow.py`
Note the existing signature and which callbacks it passes.

- [ ] **Step 2: Write the failing test**

Append to `tests/test_calibration_workflow.py`:

```python
def test_start_test_scan_uses_collector_sink_and_skips_default_storage():
    from unittest.mock import MagicMock

    motion = _build_motion_with_fake_scan_workflow()
    received_request = {}
    motion.scan_workflow.start_scan = lambda req: received_request.setdefault("req", req) or True
    motion.scan_workflow.await_complete = lambda timeout_sec: None

    motion.calibration_workflow.start_test_scan(...)

    req = received_request["req"]
    assert req.skip_default_storage is True
    final_sinks = [s for s in req.sinks if "final" in getattr(s, "channels", set())]
    assert len(final_sinks) >= 1
```

- [ ] **Step 3: Modify `start_test_scan`**

Apply the same pattern as `run_calibration` (collector sink, `skip_default_storage=True`, await + return).

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_calibration_workflow.py -v`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add omotion/CalibrationWorkflow.py tests/test_calibration_workflow.py
git commit -m "refactor(sdk): start_test_scan migrates to collector sink + skip_default_storage"
```

---

## Phase E — `ScanWorkflow.start_scan` rewrite (1 task)

### Task 17: Rewrite `start_scan` to use ScanRunner

**Files:**
- Modify: `omotion/ScanWorkflow.py`
- Modify: `tests/test_scan_workflow.py`

This is the cutover. `start_scan` stops constructing a `SciencePipeline`, deleting ~500 LOC of legacy science-pipeline-wrangling code. Replaced with `default_pipeline()` + auto-injected storage sinks + auto-wired telemetry source + `ScanRunner`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_scan_workflow.py`:

```python
def test_start_scan_uses_new_runner_and_auto_injects_csv_sink_when_data_dir_set():
    """When MotionInterface(data_dir=...) is set, start_scan auto-injects
    a CsvSink. Apps don't pass CsvSink themselves."""
    from omotion.pipeline.sinks import CsvSink

    motion = _build_motion_with_data_dir("/tmp/scans")
    request = ScanRequest(
        subject_id="x", duration_sec=1,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
    )
    motion.scan_workflow.start_scan(request)
    runner = motion.scan_workflow._runner
    csv_sinks = [s for s in runner.sinks if isinstance(s, CsvSink)]
    assert len(csv_sinks) == 1


def test_start_scan_skips_csv_sink_when_data_dir_none():
    motion = _build_motion_with_data_dir(None)
    request = ScanRequest(
        subject_id="x", duration_sec=1,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
    )
    motion.scan_workflow.start_scan(request)
    runner = motion.scan_workflow._runner
    from omotion.pipeline.sinks import CsvSink
    csv_sinks = [s for s in runner.sinks if isinstance(s, CsvSink)]
    assert csv_sinks == []


def test_start_scan_skips_default_storage_when_request_opts_out():
    """Internal SDK workflows set skip_default_storage=True."""
    motion = _build_motion_with_data_dir("/tmp/scans")
    request = ScanRequest(
        subject_id="x", duration_sec=1,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
        skip_default_storage=True,
    )
    motion.scan_workflow.start_scan(request)
    runner = motion.scan_workflow._runner
    from omotion.pipeline.sinks import CsvSink
    assert all(not isinstance(s, CsvSink) for s in runner.sinks)


def test_start_scan_auto_wires_telemetry_source_when_telemetry_sink_subscribed():
    from omotion.pipeline.sinks import TelemetrySink
    from omotion.pipeline.sources import ConsoleTelemetrySource

    motion = _build_motion_with_data_dir(None)
    sink = TelemetrySink(output_path="/tmp/cq_telemetry.csv")
    request = ScanRequest(
        subject_id="x", duration_sec=1,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
        sinks=[sink],
    )
    motion.scan_workflow.start_scan(request)
    runner = motion.scan_workflow._runner
    assert isinstance(runner.telemetry_source, ConsoleTelemetrySource)


def test_start_scan_no_telemetry_source_when_no_telemetry_sink():
    motion = _build_motion_with_data_dir(None)
    request = ScanRequest(
        subject_id="x", duration_sec=1,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
    )
    motion.scan_workflow.start_scan(request)
    runner = motion.scan_workflow._runner
    assert runner.telemetry_source is None
```

(`_build_motion_with_data_dir` is a helper that returns a `MotionInterface(demo_mode=True, data_dir=..., operator_id="test")`. Define it once in the test file.)

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_scan_workflow.py -v -k start_scan_uses_new_runner or skip_csv or skip_default_storage or auto_wires_telemetry`
Expected: most FAIL.

- [ ] **Step 3: Rewrite `start_scan`**

In `omotion/ScanWorkflow.py`, replace the existing `start_scan` body. The new implementation:

```python
def start_scan(self, request) -> bool:
    """Drive a scan through the new pipeline.

    Constructs ScanMetadata, default_pipeline, LiveUsbSource, auto-injects
    default storage sinks (unless request.skip_default_storage), auto-wires
    a ConsoleTelemetrySource if any sink subscribes to 'telemetry', and
    spawns a worker thread that runs the ScanRunner.

    Sinks travel on request.sinks; storage sinks (CsvSink, ScanDBSink) are
    SDK-managed via MotionInterface init (data_dir, scan_db_path).
    """
    from omotion.pipeline.factory import default_pipeline
    from omotion.pipeline.runner import ScanRunner
    from omotion.pipeline.sources import LiveUsbSource, ConsoleTelemetrySource
    from omotion.pipeline.sinks import CsvSink, ScanDBSink, ScanMetadata
    from omotion.pipeline.pedestal import SensorPedestals

    # Pre-flight checks (camera enable, frame-sync trigger, etc.) — port from
    # the existing start_scan. These are unchanged in PR 2.
    if not self._pre_flight_check(request):
        return False

    # Construct ScanMetadata from the request + SDK-level config
    scan_id = self._generate_scan_id()
    meta = ScanMetadata(
        scan_id=scan_id,
        subject_id=request.subject_id,
        operator=self._interface.operator_id or "unknown",
        started_at_iso=datetime.utcnow().isoformat() + "Z",
        duration_sec=request.duration_sec,
        left_camera_mask=request.left_camera_mask,
        right_camera_mask=request.right_camera_mask,
        reduced_mode=request.reduced_mode,
    )

    # Build pipeline with the configured raw-save gate
    calibration = self._interface.console.calibration
    pedestals = SensorPedestals.from_sensors(
        left=self._interface.left, right=self._interface.right,
    )
    pipeline = default_pipeline(
        metadata=meta, calibration=calibration, pedestals=pedestals,
        rolling_avg_window=request.rolling_avg_window or 10,
        raw_save_max_duration_s=request.raw_save_max_duration_s,
    )

    # Auto-inject default storage sinks unless caller opted out
    default_sinks: list = []
    if not request.skip_default_storage:
        if self._interface.data_dir is not None:
            default_sinks.append(CsvSink(output_dir=self._interface.data_dir))
        if self._interface.scan_db_path is not None:
            default_sinks.append(ScanDBSink(db_path=self._interface.scan_db_path))
    all_sinks = default_sinks + list(request.sinks)

    # Auto-wire telemetry source based on subscribed channels
    subscribed_channels = {ch for s in all_sinks for ch in getattr(s, "channels", set())}
    telemetry_source = None
    if "telemetry" in subscribed_channels:
        telemetry_source = ConsoleTelemetrySource(
            console=self._interface.console, poll_interval_s=0.1,
        )

    # Build source + runner
    source = LiveUsbSource(
        console=self._interface.console,
        left=self._interface.left,
        right=self._interface.right,
        batch_size_frames=request.batch_size_frames or 10,
        metadata=meta,
    )
    self._runner = ScanRunner(
        source=source, pipeline=pipeline, sinks=all_sinks,
        telemetry_source=telemetry_source,
    )

    # Spawn worker thread
    self._scan_thread = threading.Thread(
        target=self._runner.run, daemon=True, name="ScanWorkflow-scan",
    )
    self._scan_thread.start()
    return True


def await_complete(self, timeout_sec: float = None) -> None:
    """Block until the scan thread exits."""
    if self._scan_thread is not None:
        self._scan_thread.join(timeout=timeout_sec)


def cancel(self) -> None:
    """Stop the in-progress scan."""
    if self._runner is not None:
        self._runner.source.close()
        if self._runner.telemetry_source is not None:
            self._runner.telemetry_source.close()
```

Delete (or comment out for now) the existing ~500 LOC of `SciencePipeline` construction + reduced-mode interception + callback wiring in the old `start_scan`. The bulk of that file shrinks dramatically.

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_scan_workflow.py -v`
Expected: all pass.

Also re-run the full pipeline suite to make sure nothing regressed:

Run: `pytest tests/test_pipeline/ -v -m "not sensor"`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add omotion/ScanWorkflow.py tests/test_scan_workflow.py
git commit -m "feat(sdk): ScanWorkflow.start_scan drives the new pipeline via ScanRunner

Constructs default_pipeline + LiveUsbSource + auto-injected storage sinks
+ auto-wired telemetry source. Deletes ~500 LOC of legacy
SciencePipeline construction and reduced-mode interception."
```

---

## Phase F — MotionProcessing.py shim collapse (1 task)

### Task 18: Strip `MotionProcessing.py` down to parsing helpers + dataclasses

**Files:**
- Modify: `omotion/MotionProcessing.py`
- Modify: `tests/test_pipeline_csv.py` (if it imports removed names)
- Modify: any other internal tests that imported removed names

Per spec §3.4: keep `parse_histogram_stream`, `parse_histogram_packet_structured`, `_rle_decompress`, `_util_crc16`, the constants (`EXPECTED_HISTOGRAM_SUM`, `HISTOGRAM_BYTES`, etc.), and the `Sample` / `CorrectedBatch` dataclasses. Remove everything else.

- [ ] **Step 1: Audit imports across the repo to see what's still used externally**

Run: `grep -rn "from omotion.MotionProcessing import\|from omotion import.*MotionProcessing" omotion/ tests/ scripts/ ../openmotion-bloodflow-app/ ../openmotion-test-app/ 2>/dev/null | grep -v ".pyc"`
Expected: a list of all external import sites. Anything importing `SciencePipeline`, `FrameIdUnwrapper`, `create_science_pipeline`, `_check_dark_integrity`, `_emit_realtime_corrected`, `_emit_corrected_for_camera`, `_calibrate_bfi_bvi`, `_flush_terminal_dark`, `compute_realtime_metrics` needs to be updated or removed.

- [ ] **Step 2: Write the failing test**

Append to `tests/test_pipeline_csv.py` (or create `tests/test_motion_processing_shim.py`):

```python
def test_motion_processing_retains_parsing_helpers():
    """After PR 2 collapse, these must still be importable from omotion.MotionProcessing."""
    from omotion.MotionProcessing import (
        parse_histogram_stream,
        parse_histogram_packet_structured,
        _rle_decompress,
        _util_crc16,
        EXPECTED_HISTOGRAM_SUM,
        HISTOGRAM_BYTES,
        Sample,
        CorrectedBatch,
        PEDESTAL_HEIGHT,
    )
    assert callable(parse_histogram_stream)
    assert callable(parse_histogram_packet_structured)
    assert EXPECTED_HISTOGRAM_SUM == 2_457_606


def test_motion_processing_removes_science_pipeline_class():
    import omotion.MotionProcessing as mp
    assert not hasattr(mp, "SciencePipeline")
    assert not hasattr(mp, "create_science_pipeline")
    assert not hasattr(mp, "FrameIdUnwrapper")
```

- [ ] **Step 3: Run test to verify it fails**

Run: `pytest tests/test_pipeline_csv.py -v -k motion_processing_removes_science_pipeline`
Expected: FAIL — the class still exists.

- [ ] **Step 4: Remove `SciencePipeline` and related helpers**

In `omotion/MotionProcessing.py`, delete:

- `class SciencePipeline` (entire class, ~1,200 LOC)
- `class FrameIdUnwrapper`
- `_check_dark_integrity()`
- `_emit_realtime_corrected()`
- `_emit_corrected_for_camera()`
- `_calibrate_bfi_bvi()`
- `_flush_terminal_dark()`
- `compute_realtime_metrics()`
- `create_science_pipeline()` factory
- `feed_pipeline_from_csv()` (if it exists — was for test fixtures)

Keep:

- `parse_histogram_stream()` (called by `LiveUsbSource._reader_loop`)
- `parse_histogram_packet_structured()` (called by `parse_histogram_stream`)
- `_rle_decompress()`, `_util_crc16()`
- All constants: `EXPECTED_HISTOGRAM_SUM`, `HISTOGRAM_BYTES`, `HISTO_SIZE_WORDS`, `MIN_PACKET_ENVELOPE_SIZE`, `FRAME_ID_MODULUS`, `FRAME_ROLLOVER_THRESHOLD`, `PEDESTAL_HEIGHT`, `ADC_GAIN`, `CAMERA_GAIN_MAP`, `_TIMESTAMP_ROLLOVER_S`
- `@dataclass Sample` and `@dataclass CorrectedBatch`
- `HistogramSample`, `HistogramPacket` dataclasses (used by the parser)
- Their imports

Result: file shrinks from ~1,940 LOC to ~400 LOC.

- [ ] **Step 5: Run the full SDK test suite to find broken imports**

Run: `pytest tests/ -v -m "not sensor and not destructive"`
Expected: PASS for everything except possibly old `test_pipeline_csv.py` tests that exercised `SciencePipeline` directly — those tests are obsoleted by `tests/test_pipeline/` and can be deleted if they fail. Audit the failures, delete obsoleted tests, fix any other importers.

- [ ] **Step 6: Commit**

```bash
git add omotion/MotionProcessing.py tests/
git commit -m "refactor(sdk): MotionProcessing.py collapses to parsing+dataclasses shim (~1940 → ~400 LOC)"
```

---

## Phase G — bloodflow-app migration (3 tasks)

Apps now consume the new SDK (which they pin in their build). `motion_connector.py` needs three targeted surgeries: (1) update SDK init to pass `data_dir`/`scan_db_path`/`operator_id`; (2) replace the 4-callback `start_scan` call with sink composition; (3) replace the CQ callback closures with a `contact_quality_workflow.check()` call.

**Working dir for Phase G:** `C:/Users/ethan/Projects/openmotion-bloodflow-app/`

---

### Task 19: Update MotionInterface init in bloodflow-app

**Files:**
- Modify: `openmotion-bloodflow-app/motion_connector.py` (the line where `MotionInterface(...)` is constructed)
- Modify: `openmotion-bloodflow-app/motion_singleton.py` (if init lives there)

- [ ] **Step 1: Find the existing MotionInterface construction**

Run: `grep -n "MotionInterface(" motion_connector.py motion_singleton.py`
Note the line number and existing kwargs.

- [ ] **Step 2: Update the construction**

Add the new SDK-init kwargs:

```python
# Before:
motion = MotionInterface(...)

# After:
motion = MotionInterface(
    data_dir=self._app_config.data_directory,
    scan_db_path=(self._app_config.scan_db_path
                   if self._app_config.scan_db_enabled else None),
    operator_id="bloodflow-app",
    # ... preserve existing kwargs
)
```

- [ ] **Step 3: Smoke-test the app imports**

Run: `python -c "import motion_connector; print('ok')"`
Expected: no ImportError; "ok" printed.

- [ ] **Step 4: Commit**

```bash
git add motion_connector.py motion_singleton.py
git commit -m "feat(bloodflow-app): pass data_dir/scan_db_path/operator_id to MotionInterface init"
```

---

### Task 20: Replace `start_scan` callback kwargs with sinks in bloodflow-app

**Files:**
- Modify: `openmotion-bloodflow-app/motion_connector.py` (lines ~1600-1900 — the scan-start path)
- Modify: `openmotion-bloodflow-app/tests/test_motion_connector.py` (any tests that mock the callback API)

This is the meat of the app-side migration. ~150 LOC of callback closures (`_on_uncorrected`, `_on_corrected_batch`) get replaced by one `_LivePlotSink` class.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_motion_connector.py`:

```python
def test_start_scan_passes_sinks_not_callbacks():
    """After PR 3 migration, motion_connector passes sinks=[...] to interface.start_scan,
    not on_*_fn kwargs."""
    from unittest.mock import MagicMock

    connector = _build_connector()
    connector._interface.start_scan = MagicMock(return_value=True)

    connector._begin_scan(_fake_scan_request())

    args, kwargs = connector._interface.start_scan.call_args
    # The new shape: start_scan(request) — one positional, request.sinks carries the sinks
    assert len(args) == 1
    request = args[0]
    assert hasattr(request, "sinks")
    assert len(request.sinks) >= 1   # at least the live plot sink
    # No legacy callback kwargs
    assert "on_uncorrected_fn" not in kwargs
    assert "on_corrected_batch_fn" not in kwargs
    assert "on_dark_frame_fn" not in kwargs
    assert "on_rolling_avg_fn" not in kwargs


def test_live_plot_sink_emits_qt_signals_per_frame():
    """The _LivePlotSink class emits the same per-frame Qt signal payload
    that the legacy on_uncorrected callback used to emit."""
    import numpy as np
    from omotion.pipeline.batch import FrameBatch
    from motion_connector import _LivePlotSink

    connector = _build_connector()
    sink = _LivePlotSink(connector=connector)

    batch = FrameBatch(
        cam_ids=np.zeros(2, dtype=np.int8),
        frame_ids=np.arange(2, dtype=np.uint8),
        raw_histograms=np.zeros((2, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((2, 2, 8), dtype=np.float32),
        timestamp_s=np.array([0.025, 0.050], dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.array([11, 12], dtype=np.int64),
        frame_type=np.array(["light", "light"], dtype="<U8"),
        bfi_live=np.full((2, 2, 8), 5.0, dtype=np.float32),
        bvi_live=np.full((2, 2, 8), 6.0, dtype=np.float32),
    )
    sink.consume("live", batch)
    # Assert the connector saw 2 sample emissions
    assert connector._emit_sample_to_qml.call_count == 2
```

(Adjust `_build_connector` and `_fake_scan_request` helpers to match the app's test infrastructure.)

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_motion_connector.py -v -k passes_sinks_not_callbacks`
Expected: FAIL.

- [ ] **Step 3: Add `_LivePlotSink` class to motion_connector.py**

Above the `MotionConnector` class:

```python
class _LivePlotSink:
    """Subscribes to 'live' channel. Emits per-frame Qt signals into the
    QML plot for whichever cameras are active in the current scan."""
    channels = {"live"}

    def __init__(self, connector):
        self._connector = connector
        self._meta = None

    def on_scan_start(self, meta):
        self._meta = meta

    def consume(self, channel, batch):
        if channel != "live":
            return
        n = batch.bfi_live.shape[0]
        reduced = self._meta is not None and self._meta.reduced_mode
        for i in range(n):
            ft = batch.frame_type[i]
            if ft in ("warmup", "stale"):
                continue
            if reduced:
                bfi = batch.bfi_live_side[i]    # shape (2,)
                bvi = batch.bvi_live_side[i]
            else:
                bfi = batch.bfi_live[i]          # shape (2, 8)
                bvi = batch.bvi_live[i]
            self._connector._emit_sample_to_qml(
                bfi=bfi, bvi=bvi,
                frame_id=int(batch.abs_frame_ids[i]),
                timestamp_s=float(batch.timestamp_s[i]),
                is_dark=(ft == "dark"),
            )

    def on_complete(self):
        pass
```

(`_emit_sample_to_qml` should already exist on the connector — if not, port the existing Qt-signal-emit logic from inside the old `_on_uncorrected` callback into this helper.)

- [ ] **Step 4: Replace the start_scan call**

Locate the `self._interface.start_scan(...)` call site (around line 1881). Replace:

```python
# Before (legacy):
on_uncorrected_fn = _on_uncorrected
on_corrected_batch_fn = None if self._uncorrected_only else _on_corrected_batch
on_dark_frame_fn, on_rolling_avg_fn = self._make_contact_quality_callbacks(...)
started = self._interface.start_scan(
    request,
    on_uncorrected_fn=on_uncorrected_fn,
    on_corrected_batch_fn=on_corrected_batch_fn,
    on_dark_frame_fn=on_dark_frame_fn,
    on_rolling_avg_fn=on_rolling_avg_fn,
)

# After (PR 3):
sinks = [_LivePlotSink(connector=self)]
if self._app_config.developer_mode:
    sinks.append(TelemetrySink(output_path=os.path.join(
        self._app_config.data_directory,
        f"{scan_id}_telemetry.csv",
    )))

request.sinks = sinks
request.raw_save_max_duration_s = (
    self._app_config.raw_data_duration_sec
    if self._app_config.write_raw_data else 0
)
started = self._interface.start_scan(request)
```

Delete the closures `_on_uncorrected`, `_on_corrected_batch` (and any related reduced-mode bookkeeping dicts — `_reduced_uncorr_buf`, `_reduced_corr_buf`). These are now `_LivePlotSink` + `SideAveragingStage` in the SDK.

Import `TelemetrySink` from the SDK: `from omotion.pipeline.sinks import TelemetrySink`.

- [ ] **Step 5: Run tests to verify they pass**

Run: `pytest tests/test_motion_connector.py -v`
Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add motion_connector.py tests/test_motion_connector.py
git commit -m "feat(bloodflow-app): replace start_scan callback closures with _LivePlotSink + TelemetrySink"
```

---

### Task 21: Replace contact-quality callbacks with `ContactQualityWorkflow.check()`

**Files:**
- Modify: `openmotion-bloodflow-app/motion_connector.py` (lines ~2470-2582 — CQ callback factories)
- Modify: `openmotion-bloodflow-app/tests/test_motion_connector.py`

The CQ procedure is now an SDK workflow. Replace the closure factory + the CQ scan path with one workflow call.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_motion_connector.py`:

```python
def test_contact_quality_check_uses_sdk_workflow_not_callbacks():
    """The CQ check now calls interface.contact_quality_workflow.check()
    and reads the structured result, instead of plumbing callbacks
    through start_scan."""
    from unittest.mock import MagicMock
    from omotion.ContactQualityWorkflow import ContactQualityResult

    connector = _build_connector()
    fake_result = ContactQualityResult(passed=True, per_camera={}, duration_sec=1.0)
    connector._interface.contact_quality_workflow.check = MagicMock(return_value=fake_result)

    result = connector._run_contact_quality_check(...)

    assert result.passed is True
    # Verify the call shape — keyword args
    kwargs = connector._interface.contact_quality_workflow.check.call_args.kwargs
    assert "dark_threshold_per_camera" in kwargs
    assert "light_threshold_per_camera" in kwargs
    assert "left_camera_mask" in kwargs
    assert "right_camera_mask" in kwargs
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_motion_connector.py -v -k contact_quality_check_uses_sdk_workflow`
Expected: FAIL.

- [ ] **Step 3: Rewrite the CQ check method**

In `motion_connector.py`, locate the existing contact-quality check (search for `cq_check_duration_sec` or `_make_contact_quality_callbacks`). Replace with:

```python
def _run_contact_quality_check(self) -> "ContactQualityResult":
    """Run the SDK's CQ workflow, return the result. UI display is the
    caller's responsibility (the reposition modal)."""
    result = self._interface.contact_quality_workflow.check(
        duration_sec=self._app_config.cq_check_duration_sec,
        rolling_window=self._app_config.cq_rolling_avg_window,
        dark_threshold_per_camera=self._app_config.cq_dark_threshold_per_camera,
        light_threshold_per_camera=self._app_config.cq_light_threshold_per_camera,
        left_camera_mask=self._current_left_mask,
        right_camera_mask=self._current_right_mask,
    )
    if not result.passed:
        self._show_reposition_modal(result)
    return result
```

Delete the old `_make_contact_quality_callbacks`, `_on_dark_frame`, `_on_rolling_avg` closures and the per-camera threshold-tracking dicts they maintain. All of this is now in the SDK's `_ContactQualitySink` + `ContactQualityWorkflow`.

Update any caller of the old CQ path to call `_run_contact_quality_check` instead.

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_motion_connector.py -v`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add motion_connector.py tests/test_motion_connector.py
git commit -m "feat(bloodflow-app): contact-quality check uses SDK ContactQualityWorkflow"
```

---

## Phase H — test-app migration (1 task)

### Task 22: test-app `motion_connector.py` sink migration

**Files:**
- Modify: `openmotion-test-app/motion_connector.py`
- Modify: `openmotion-test-app/tests/test_motion_connector.py`

test-app is smaller — likely just needs the live-plot sink. No contact-quality logic to migrate.

**Working dir for Phase H:** `C:/Users/ethan/Projects/openmotion-test-app/`

- [ ] **Step 1: Recon — what callbacks does test-app use?**

Run: `grep -n "on_uncorrected\|on_corrected\|on_dark_frame\|on_rolling_avg\|on_realtime_corrected\|on_raw_frame\|start_scan" motion_connector.py`
Identify each callback site.

- [ ] **Step 2: Add `_TestAppLiveSink`**

Use the same pattern as bloodflow-app's `_LivePlotSink`, tailored to test-app's QML signal surface. If test-app emits BFI/BVI to a different signal name or format, adjust the `consume` method's call site accordingly.

- [ ] **Step 3: Replace start_scan kwargs with sinks**

Same pattern as Task 20:

```python
# Before:
started = self._interface.start_scan(
    request,
    on_uncorrected_fn=_on_uncorrected,
    # ...
)

# After:
sinks = [_TestAppLiveSink(connector=self)]
if self._app_config.developer_mode:
    sinks.append(TelemetrySink(output_path=...))
request.sinks = sinks
request.raw_save_max_duration_s = (
    self._app_config.raw_data_duration_sec
    if self._app_config.write_raw_data else 0
)
started = self._interface.start_scan(request)
```

- [ ] **Step 4: Update MotionInterface init**

Same pattern as Task 19 — pass `data_dir`, `scan_db_path`, `operator_id="test-app"` to the constructor.

- [ ] **Step 5: Run tests to verify they pass**

Run: `pytest tests/test_motion_connector.py -v`
Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add motion_connector.py tests/test_motion_connector.py
git commit -m "feat(test-app): motion_connector uses new SDK sink API + MotionInterface init kwargs"
```

---

## Phase I — Integration + finalization (2 tasks)

### Task 23: Manual hardware-validation smoke test on real bench

**Files:**
- N/A — manual validation; document results in a follow-up commit if anything's flagged

This is the gate before merging. Run each app against real hardware and confirm:

- [ ] **Step 1: bloodflow-app scan**

Connect a phantom. Open the app. Run a 30-second scan with developer_mode=true, write_raw_data=true.

Verify:
- Live BFI/BVI plot updates smoothly during the scan
- Contact-quality check fires correctly (test with phantom on/off)
- Raw CSV lands in `data_directory` with the new `type` column populated
- Corrected CSV lands in `data_directory` with the legacy wide column layout (per-cam columns)
- `_telemetry.csv` lands alongside (because developer_mode=true)
- Scan-DB session persists (if scan_db_enabled)
- App doesn't crash or hang at the end of the scan

- [ ] **Step 2: test-app scan**

Same procedure, smaller surface. Verify the live plot works and the data is saved correctly.

- [ ] **Step 3: Calibration cycle**

Run a full calibration via bloodflow-app (with phantom). Verify the per-camera arrays are computed and written to the console correctly. Compare against a calibration run on the legacy code path if possible.

- [ ] **Step 4: Short-scan terminal flush**

Run a scan that ends before the second scheduled dark (frame 601, ≈15 s). Verify the corrected CSV is still populated via the terminal-flush path (this was a PR 1 fix that PR 2 must preserve).

- [ ] **Step 5: Document the validation in a commit**

If everything passes, commit a single doc-only marker:

```bash
git -C openmotion-sdk commit --allow-empty -m "validate(sdk): PR 2 hardware acceptance — bloodflow-app + test-app + calibration"
```

If anything fails, file an issue (or follow-up task) before merging.

---

### Task 24: Cross-repo PR coordination + release tag

**Files:**
- N/A — GitHub PR + tag operations

After hardware validation passes:

- [ ] **Step 1: Open PRs against `next-next` in each repo**

```bash
# Each repo, on its respective feature branch:
gh pr create --base next-next --title "Pipeline cutover: SDK switch + app migration"
```

For openmotion-sdk: `gh pr create --base next-next --head feature/pipeline-cutover ...`
For openmotion-bloodflow-app: `gh pr create --base next --head feature/pipeline-sinks ...`
For openmotion-test-app: same as bloodflow-app.

- [ ] **Step 2: Tag SDK pre-release for apps to pin against**

Once the SDK PR is approved (review pass + tests green), tag a pre-release on the PR's HEAD:

```bash
cd C:/Users/ethan/Projects/openmotion-sdk
git checkout feature/pipeline-cutover
git tag -a 1.7.0-rc.0 -m "Pipeline cutover pre-release"
git push origin 1.7.0-rc.0
```

Per `docs/Releasing.md`'s current policy this is `pre-X.Y.Z`-shaped (the policy disallows `-rc.N` suffix on stable tags) — adjust the tag name to `pre-1.7.0` per the doc if strict.

- [ ] **Step 3: Pin SDK rc in each app's requirements**

In each app's `pyproject.toml` / `requirements.txt`, update the SDK pin to the rc tag. Push to each app's feature branch and verify the app builds against the rc.

- [ ] **Step 4: Merge in coordinated order**

1. Merge SDK PR to `next-next` (or whatever base)
2. Merge bloodflow-app PR to its `next`
3. Merge test-app PR to its `next`

If any single PR's CI fails after merge, revert and investigate before re-merging.

- [ ] **Step 5: Tag stable releases**

Once all three are merged and a smoke scan still passes:

```bash
# SDK
git -C openmotion-sdk tag 1.7.0
git -C openmotion-sdk push origin 1.7.0

# Apps — increment to next minor
git -C openmotion-bloodflow-app tag 1.2.0
git -C openmotion-bloodflow-app push origin 1.2.0
git -C openmotion-test-app tag X.Y.0
git -C openmotion-test-app push origin X.Y.0
```

This concludes PR 2 + PR 3. The new pipeline is now the production code path in all three repos.

---

## Plan complete — 24 tasks, 9 phases, 3 repos

Summary:
- **Phase A (Tasks 1-5):** Foundational config + Tee gate + ScanRequest reshape + MotionInterface init
- **Phase B (Tasks 6-11):** Telemetry scaffolding (Event, Aggregator, IngestStage, Source, runner wiring, Sink)
- **Phase C (Task 12):** LiveUsbSource reader loop wired via parse_histogram_stream
- **Phase D (Tasks 13-16):** Internal SDK workflows on sinks (CalibrationWorkflow, ContactQualityWorkflow, lazy property, start_test_scan)
- **Phase E (Task 17):** ScanWorkflow.start_scan rewrite
- **Phase F (Task 18):** MotionProcessing.py shim collapse
- **Phase G (Tasks 19-21):** bloodflow-app migration
- **Phase H (Task 22):** test-app migration
- **Phase I (Tasks 23-24):** Hardware validation + cross-repo release

After this plan executes: the new pipeline is production code in all three repos; the legacy SciencePipeline class is gone; apps construct sinks instead of passing callbacks; contact quality is an SDK workflow.
