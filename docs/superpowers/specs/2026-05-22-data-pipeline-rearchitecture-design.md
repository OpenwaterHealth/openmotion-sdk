# Data Pipeline Re-architecture — Design

**Date:** 2026-05-22
**Branch:** `feature/data-pipeline-tweaks`
**Status:** Author-approved; pending review
**Authoritative algorithm reference:** [`docs/SciencePipeline.md`](../../SciencePipeline.md) — the math described there is preserved verbatim. This document covers the *organization* of the code that implements it, not the algorithm itself.

---

## 1. Summary

Replace the monolithic ~1,200 LOC `SciencePipeline` worker class in `omotion/MotionProcessing.py` with a composable chain of small, named, audit-friendly Stages under a new `omotion/pipeline/` package.

**Goals:**

- **Readable by scientists and auditors.** Each science step is a single file; the math is at the top; the pipeline list is the algorithm.
- **Composable + extensible.** Adding a correction or swapping an estimator is local to one file.
- **Numpy-vectorized.** Per-camera Python loops become numpy operations on `(N, 2, 8)` arrays. Estimated 20-50× speedup on `MomentsStage` alone.
- **Single coherent dataflow.** Live, raw, rolling-averaged, and corrected outputs flow through one pipeline with explicit tee points.
- **Reproducible.** A pure pipeline running on a recorded raw CSV or DB session produces byte-identical corrected output to a live scan.

## 2. Audience for the new code

In declining design weight:

1. **Auditors** — must be able to read the algorithm without untangling threads, callbacks, or per-camera state dicts. The stage list is the spec.
2. **Future contributors** — adding a correction or estimator must be local to one file, not a 6-place change.
3. **Scientists writing analysis code on top of the SDK** — subscribe to corrected samples, replay sessions, plug in custom processing.
4. **Code reviewers verifying the math** — open `moments.py` / `dark.py` / `shot_noise.py` and see the formulas at the top of the file.

## 3. Non-goals

- The BFI/BVI / dark-correction / shot-noise / calibration math itself. `docs/SciencePipeline.md` remains authoritative; the new stages implement that doc verbatim.
- Changes to the wire protocol or firmware.
- New data types or output formats beyond what's described here (raw / live / rolling / corrected).
- Migrating apps in the same PR sequence as the SDK rework — apps move on their own schedule using the deprecation path in §13.

## 4. Pain points being addressed

Catalogued from a survey of `MotionProcessing.py` on the current branch:

| Pain point | Manifestation today |
|---|---|
| State explosion | `SciencePipeline` has 11 instance dicts keyed by `(side, cam_id)` (`_unwrappers`, `_pending_moments`, `_dark_history`, `_last_uncorrected`, `_last_corrected`, etc.). Adding a new per-camera tracked quantity touches 6 of them. |
| Callback spaghetti | 6 callbacks (`on_uncorrected_fn`, `on_corrected_batch_fn`, `on_raw_frame_fn`, `on_realtime_corrected_fn`, `on_dark_frame_fn`, `on_rolling_avg_fn`) woven through one ~270-line worker function. |
| Duplicate parsing | `parse_histogram_stream()` and the final-accumulator flush share ~130 lines of near-duplicate logic. |
| Underused numpy | Moments computed frame-by-frame as scalar dot products in a Python loop. |
| Hidden preprocessing | Noise-floor zeroing, pedestal subtraction, dark integrity check, dark-frame UI masking, 4-point quadratic stencil, terminal dark flush are all inline in the worker — not visible from the API surface. |
| Coupled live + persistence | Reduced-mode side-averaging lives in `ScanWorkflow`, not in the pipeline; raw CSV and corrected CSV writers are both fed by separate callback paths. |
| Global pedestal | `PEDESTAL_HEIGHT` is a module global mutated at sensor-connect time, preventing dual-sensor systems running mixed firmware versions from having independent pedestals. |

## 5. Architecture overview

### 5.1 Package layout

```
omotion/
├── pipeline/
│   ├── __init__.py            # public API: Pipeline, Stage, FrameBatch, default_pipeline()
│   ├── batch.py               # FrameBatch — the typed data carrier
│   ├── pipeline.py            # Pipeline class + drive loop; no science
│   ├── tee.py                 # Tee stage; channel routing
│   ├── runner.py              # ScanRunner — wires source + pipeline + sinks
│   ├── stages/
│   │   ├── __init__.py
│   │   ├── parse.py           # HistogramParseStage (CRC + sum-check)
│   │   ├── classify.py        # FrameClassificationStage (warmup/stale/dark schedule)
│   │   ├── noise_floor.py     # NoiseFloorStage
│   │   ├── moments.py         # MomentsStage (vectorized einsum)
│   │   ├── pedestal.py        # PedestalSubtractionStage (per-side, FW-version-keyed)
│   │   ├── dark.py            # DarkCorrectionStage + DarkEstimator strategies
│   │   ├── shot_noise.py      # ShotNoiseCorrectionStage
│   │   ├── bfi_bvi.py         # BfiBviStage (calibration application)
│   │   ├── rolling_avg.py     # RollingAverageStage
│   │   └── side_avg.py        # SideAveragingStage (reduced-mode averaging)
│   ├── sources.py             # Source protocol: LiveUsbSource, CsvReplaySource, DbReplaySource
│   └── sinks.py               # Sink protocol; CsvSink, ScanDBSink, QtUiSink stubs
├── MotionProcessing.py        # DEPRECATED: shim re-exporting Sample, CorrectedBatch, etc. from pipeline/
├── ScanWorkflow.py            # thin wrapper that constructs a ScanRunner and runs it
└── ...
```

**Principle:** one concept per file, science separated from orchestration separated from I/O.

- Each `stages/<x>.py` is small (≤300 LOC, mostly math + docstring). An auditor opens `moments.py`, sees the variance/contrast formulas verbatim, done.
- `pipeline.py` contains zero science. It owns the drive loop, the stage list, and emission logic.
- `sources.py` and `sinks.py` use the same protocols, so live and replay differ only by the source object — the pipeline is identical.
- `runner.py` is what `ScanWorkflow.start_scan()` ends up calling — a thin tie-it-all-together.

### 5.2 The Stage list (10 stages)

```python
default_pipeline = Pipeline([
    # ── ingestion + validation ──
    HistogramParseStage(expected_row_sum=2_457_606),
    FrameClassificationStage(discard_count=9, dark_interval=600),

    Tee("raw", filter=None),                        # ── tee #1: per-frame raw, ALL frame types ──

    # ── histogram preprocessing ──
    NoiseFloorStage(threshold=NoiseFloor.from_config()),
    MomentsStage(),
    PedestalSubtractionStage(
        pedestals=SensorPedestals.from_sensors(left, right),
    ),

    # ── dark correction (dual-output: realtime predictor + batch interpolation) ──
    DarkCorrectionStage(
        realtime_estimator=HybridRealtimePredictor(),
        batch_estimator=LinearInterpolation(),
        realtime_history_size=4,
        integrity_max_above_pedestal=30.0,
    ),

    # ── shot-noise correction — applied to BOTH live & batch paths ──
    ShotNoiseCorrectionStage(adc_gain=ADC_GAIN, camera_gain_map=CAMERA_GAIN_MAP),

    # ── calibration → BFI/BVI ──
    BfiBviStage(calibration=calib),

    # ── side averaging (gated by reduced_mode; runs on live + batch) ──
    SideAveragingStage(enabled=reduced_mode),

    Tee("live", filter=lambda f: f.frame_type != "warmup" and f.frame_type != "stale"),
                                                     # ── tee #2: per-frame UI stream (incl. dark, see §9.1) ──

    # ── display-side smoothing for test/calibration consumers ──
    RollingAverageStage(window=rolling_avg_window),   # `rolling_avg_window` preserves today's tunable

    Tee("rolling", filter=lambda f: f.frame_type != "warmup" and f.frame_type != "stale"),
                                                     # ── tee #3: per-frame smoothed stream ──

    # ── Tee #4 "final" fires asynchronously from DarkCorrection IntervalClosed events;
    #    not a positional stage. See §7.3 below. ──
])
```

Auditor reads top-to-bottom and sees the entire algorithm in one screen.

## 6. FrameBatch — the data carrier

A single dataclass that flows through every stage. Each stage populates specific fields (its `process()` docstring states which fields it owns); later stages read them. Mutated **in place** for performance (no per-batch allocation churn).

```python
@dataclass
class FrameBatch:
    """N frames worth of data, two sides, 8 cameras each. Stages fill fields in order."""

    # ── populated by HistogramParseStage ──
    cam_ids:        np.ndarray              # (N,) int8 — 0..7
    frame_ids:      np.ndarray              # (N,) uint8  — raw 8-bit rolling counter
    raw_histograms: np.ndarray              # (N, 2, 8, 1024) uint32
    temperature_c:  np.ndarray              # (N, 2, 8) float32
    timestamp_s:    np.ndarray              # (N,) float64 — per-scan t=0 normalized
    pdc:            np.ndarray | None       # (N, 2) photodiode current per side
    tcm, tcl:       np.ndarray | None       # other telemetry

    # ── populated by FrameClassificationStage ──
    abs_frame_ids:  np.ndarray              # (N,) int64 monotonic
    frame_type:     np.ndarray              # (N,) categorical: "warmup" | "dark" | "light" | "stale"

    # ── populated by NoiseFloorStage (in place on raw_histograms) ──
    # (no new fields; mutates raw_histograms)

    # ── populated by MomentsStage ──
    mean_raw:       np.ndarray | None       # (N, 2, 8) float32 — u1 from raw histogram (incl pedestal)
    std_raw:        np.ndarray | None       # (N, 2, 8) float32
    contrast_raw:   np.ndarray | None       # (N, 2, 8) float32

    # ── populated by PedestalSubtractionStage ──
    display_mean:   np.ndarray | None       # (N, 2, 8) — mean_raw minus per-side pedestal, clamped ≥0

    # ── populated by DarkCorrectionStage (realtime path) ──
    dark_baseline_rt: np.ndarray | None     # (N, 2, 8) — predictor's baseline for this frame
    mean_dc_rt:       np.ndarray | None     # mean_raw − dark_baseline_rt
    std_dc_rt:        np.ndarray | None

    # ── populated by ShotNoiseCorrectionStage (live path) ──
    std_sn_rt:        np.ndarray | None     # std after Poisson variance subtraction
    contrast_sn_rt:   np.ndarray | None     # std_sn_rt / mean_dc_rt

    # ── populated by BfiBviStage (live path) ──
    bfi_live:         np.ndarray | None     # (N, 2, 8)
    bvi_live:         np.ndarray | None     # (N, 2, 8)

    # ── populated by SideAveragingStage (live path, when reduced_mode) ──
    bfi_live_side:    np.ndarray | None     # (N, 2) averaged over 8 cameras
    bvi_live_side:    np.ndarray | None

    # ── populated by RollingAverageStage ──
    bfi_rolling:      np.ndarray | None     # (N, 2, 8) or (N, 2) if reduced
    bvi_rolling:      np.ndarray | None

    # ── side-channel events from any stage ──
    events:         list[BatchEvent]        # IntervalClosed | DarkIntegrityWarning | StencilFallback | ...
```

**Rules:**

- **In-place mutation** is the default. Each stage's docstring lists fields it owns; tests assert that only the owning stage writes them.
- **All per-frame fields are numpy arrays** of shape `(N, …)` — not Python lists or dataclasses of scalars. That's the vectorization foundation; every stage operates on whole arrays.
- **`events` is the multi-output channel.** When `DarkCorrectionStage` closes an interval, it appends `IntervalClosed(...)` to `events`. Sinks read events alongside per-frame fields.

## 7. Pipeline driver, Runner, and tee routing

### 7.1 Pipeline

```python
class Pipeline:
    """Pure transformation. Knows nothing about I/O."""

    def __init__(self, stages: list[Stage]):
        self.stages = stages

    def process(self, batch: FrameBatch) -> FrameBatch:
        for stage in self.stages:
            stage.process(batch)
        return batch

    def reset(self) -> None:
        for stage in self.stages:
            stage.reset()
```

### 7.2 Stage protocol

```python
class Stage(Protocol):
    name: str

    def process(self, batch: FrameBatch) -> FrameBatch:
        """Mutate batch in place; return it for chainability."""

    def reset(self) -> None:
        """Clear any cross-batch state (history, ring buffers, frame counters)."""
```

`Tee` is a tiny Stage that appends a `LiveEmit(channel, snapshot)` to `batch.events`. The snapshot is a shallow view of the batch fields relevant to that channel (the Tee doesn't deep-copy the histogram blob unless needed).

### 7.3 ScanRunner and four tees

```python
class ScanRunner:
    """Source → Pipeline → Sinks. The only I/O orchestrator."""

    def __init__(self, source: Source, pipeline: Pipeline, sinks: list[Sink]):
        self.source = source
        self.pipeline = pipeline
        self.sinks = sinks

    def run(self) -> None:
        for sink in self.sinks: sink.on_scan_start(self.source.metadata)
        try:
            for batch in self.source:
                result = self.pipeline.process(batch)
                for event in result.events:
                    if isinstance(event, LiveEmit):
                        for sink in self.sinks_for(event.channel):
                            self._safe_consume(sink, event.channel, event.payload)
                    elif isinstance(event, IntervalClosed):
                        for sink in self.sinks_for("final"):
                            self._safe_consume(sink, "final", event.corrected_batch)
                    # Other event types (DarkIntegrityWarning, …) go to "diagnostics"
                    else:
                        for sink in self.sinks_for("diagnostics"):
                            self._safe_consume(sink, "diagnostics", event)
        finally:
            for sink in self.sinks: sink.on_complete()
```

Four channels:

| Channel | Fired when | Payload | Typical consumers |
|---|---|---|---|
| `raw` | Per-frame, immediately after `FrameClassificationStage` | All frames (incl. warmup, stale, dark, light) with original histograms, sum, telemetry | `CsvSink` raw writer, `ScanDBSink` session_raw |
| `live` | Per-frame, after `SideAveragingStage` | Best-effort corrected BFI/BVI/mean/contrast (predictor + shot-noise + calibration) | `QtUiSink` (live plot) |
| `rolling` | Per-frame, after `RollingAverageStage` | Same as `live` but smoothed over window of N frames | Calibration thresholds, test scripts |
| `final` | Async, when `DarkCorrectionStage` closes a dark interval | Accurate corrected stream computed from closed interval (linear interpolation between dark endpoints, 4-point stencil for the dark frame itself) | `CsvSink` corrected writer, `ScanDBSink` session_data |

### 7.4 Sink protocol

```python
class Sink(Protocol):
    channels: set[str]  # which channels this sink wants

    def on_scan_start(self, meta: ScanMetadata) -> None: ...
    def consume(self, channel: str, payload: Any) -> None: ...
    def on_complete(self) -> None: ...
```

`Sink.consume()` is the single dispatch method — sinks switch on `channel` to decide what to do with each payload. Today's 6-callback API (`on_uncorrected_fn`, `on_corrected_batch_fn`, etc.) collapses to one method per sink.

**Sink failures are isolated.** `ScanRunner._safe_consume()` wraps each sink call in `try/except`, logging exceptions and continuing. A misbehaving sink can't take down the scan.

### 7.5 Threading

- `LiveUsbSource` does USB reads on a background thread and pushes `FrameBatch` objects into a `queue.Queue` (max size 4) that the runner pulls from. One reader thread per side.
- `CsvReplaySource` / `DbReplaySource` are synchronous iterators — no thread.
- The runner itself is single-threaded. Stages are pure transforms with no thread management.

This replaces today's "2 raw parser threads + 1 science thread + 6 callbacks weaving across them" with one driver thread that pulls from the source. Order of operations is the order of the stage list.

## 8. Sources

```python
class Source(Protocol):
    metadata: ScanMetadata
    def __iter__(self) -> Iterator[FrameBatch]: ...
    def close(self) -> None: ...

class LiveUsbSource:
    """Reads USB bulk on background threads, batches frames, hands to runner via queue."""

class CsvReplaySource:
    """Reads raw histogram CSV(s) produced by CsvSink. One FrameBatch per chunk."""

class DbReplaySource:
    """Reads from session_raw rows in a scan-DB session."""
```

### 8.1 Offline replay determinism

Any stage's `process()` must be a pure function of `(batch, stage state at time-of-call)`. No reading the clock, no random, no environment. A determinism test asserts that running the same source through the same pipeline twice produces byte-identical output.

This enables: *"try a different dark estimator on a recorded session, diff the CSVs."* Same input bytes, two pipelines, see the difference. Critical for validating estimator changes without re-running hardware scans.

## 9. The dark-correction stage in detail

`DarkCorrectionStage` is the most complex stage; the rest of the design hinges on it making sense.

### 9.1 What happens on dark frames in the live stream

Per `SciencePipeline.md §7.3`, dark frames must still appear on the live tee but with **masked metric values** — copied from the immediately preceding bright frame — so the live plot doesn't drop and recover on every dark frame.

In the new design, `DarkCorrectionStage` keeps a `last_bright_live` cache per `(side, cam)` and, when it encounters a dark frame in `batch`, populates the live fields (`mean_dc_rt`, `std_dc_rt`, eventually `bfi_live`, `bvi_live`) for that frame by copying from the cache. The frame still appears on the live tee (its `frame_type == "dark"` so it passes the live tee filter); only its science values are masked.

The actual dark frame's raw moments still go into `DarkHistory` for the batch correction path — masking is purely a live-display concern.

If a dark frame arrives before any bright frame (very first frame in a scan), no masking is possible; the dark frame is **not** emitted to live (downstream consumers see no sample for that frame). This is the same as today's behavior.



```python
class DarkEstimator(Protocol):
    """Given a history of dark observations, predict the baseline for a target frame."""
    def predict(self, history: DarkHistory, target_frame_ids: np.ndarray) -> DarkBaseline: ...

class DarkCorrectionStage:
    def __init__(self, *,
                 realtime_estimator: DarkEstimator,
                 batch_estimator: DarkEstimator,
                 realtime_history_size: int = 4,
                 integrity_max_above_pedestal: float = 30.0):
        self.realtime = realtime_estimator
        self.batch = batch_estimator
        self.history = DarkHistory(max_darks=realtime_history_size)
        self.pending = PendingInterval()
        self.guard = DarkIntegrityGuard(max_above_pedestal=integrity_max_above_pedestal)
        self.stencil = DarkFrameQuadraticStencil()

    def process(self, batch: FrameBatch) -> FrameBatch:
        # 1. Realtime path — every light frame gets a predicted baseline NOW
        light_mask = batch.frame_type == "light"
        if light_mask.any():
            baseline_rt = self.realtime.predict(self.history, batch.abs_frame_ids[light_mask])
            batch.dark_baseline_rt = baseline_rt
            batch.mean_dc_rt = batch.mean_raw[light_mask] - baseline_rt
            batch.std_dc_rt  = np.sqrt(np.maximum(batch.std_raw[light_mask]**2 - baseline_rt.std**2, 0))

        # 2. Dark frames close an interval
        dark_mask = batch.frame_type == "dark"
        for dark_idx in np.where(dark_mask)[0]:
            dark_obs = self.guard.check_and_record(batch, dark_idx)
            self.history.append(dark_obs)
            self.pending.append(dark_obs)
            if self.pending.is_closed():        # two darks bookending light frames
                interval = self.pending.flush()
                corrected = self.batch.correct_interval(interval)
                corrected_for_dark = self.stencil.interpolate(corrected, dark_obs)
                batch.events.append(IntervalClosed(corrected_batch=corrected))

        return batch

    def reset(self) -> None:
        self.history.clear()
        self.pending.clear()

    def on_scan_stop(self) -> None:
        """Terminal dark flush — synthesize a dark from the last pending moment so
        short scans still produce a CorrectedBatch."""
        if self.pending.has_data() and self.history.has_at_least_one_dark():
            synthetic_dark = self.pending.last_moment_as_synthetic_dark()
            corrected = self.batch.correct_interval(self.pending.flush_with_terminal(synthetic_dark))
            # emit via a special TerminalFlush event picked up by the runner
            ...
```

Sub-modules inside `dark.py`:

- `DarkHistory` — ring buffer of recent dark observations, vectorized over `(2, 8)`
- `DarkIntegrityGuard` — checks u1 > pedestal+30, emits warning event
- `DarkEstimator` strategies: `ZohEstimator`, `HybridRealtimePredictor`, `LinearInterpolation`, `QuadraticPerCameraEstimator`
- `DarkFrameQuadraticStencil` — 4-point interpolation with documented fallback chain

`HybridRealtimePredictor` and `LinearInterpolation` together preserve the realtime + batch dual-output behavior shipped in commits `86539f7` (real-time dark correction) and `1dcb588` (1-dark warmup relaxation).

`HybridRealtimePredictor` is the realtime estimator and is genuinely hybrid — it applies a different algorithm to each of the two quantities it predicts:

```
u1 (mean):  average of the most recent 3 dark observations
std:        linear extrapolation through the last 2 dark observations
fallback:   zero-order hold (most recent dark, applied to BOTH u1 and std)
            when fewer than 2 darks are available — covers the warmup
            window right after scan start
```

The single-class form (rather than two separate u1 + std estimator classes) is deliberate: both halves consume the same `DarkHistory`, share the same ZOH fallback condition, and don't usefully vary independently in practice. Documented in one place; auditor reads `HybridRealtimePredictor` and sees the whole realtime-prediction policy.

## 10. Per-stage details and numpy vectorization

### 10.1 `MomentsStage` — vectorized

```python
class MomentsStage:
    BIN_VALUES    = np.arange(1024, dtype=np.float64)
    BIN_VALUES_SQ = BIN_VALUES ** 2

    def process(self, batch: FrameBatch) -> FrameBatch:
        h = batch.raw_histograms                  # (N, 2, 8, 1024) uint32
        counts = h.sum(axis=-1)                   # (N, 2, 8)
        u1 = np.einsum('nsci,i->nsc', h, self.BIN_VALUES) / counts
        u2 = np.einsum('nsci,i->nsc', h, self.BIN_VALUES_SQ) / counts
        var = u2 - u1 ** 2
        batch.mean_raw     = u1.astype(np.float32)
        batch.std_raw      = np.sqrt(np.maximum(var, 0.0)).astype(np.float32)
        batch.contrast_raw = batch.std_raw / np.where(batch.mean_raw > 0, batch.mean_raw, np.nan)
        return batch
```

Replaces ~30 lines of per-frame Python loop. Estimated speedup: **20-50× on this stage** at typical batch sizes.

### 10.2 Other vectorization wins

| Stage | Today | New |
|---|---|---|
| `MomentsStage` | Per-frame Python dot product, 640 iterations/sec | One `np.einsum` per batch |
| `NoiseFloorStage` | Per-frame conditional | `np.putmask(h, h < threshold, 0)` |
| `ShotNoiseCorrectionStage` | Per-frame scalar subtract | `var -= ADC_GAIN * mean * CAMERA_GAIN_MAP[None, None, :]` (broadcast) |
| `DarkCorrection.realtime_predict` | Per-camera history lookup | `history.u1[-3:, :, :].mean(axis=0)` |
| `BfiBviStage` | Nested `(side, cam)` loops | `np.clip((contrast - C_min) / (C_max - C_min), 0, 1) * 10` |
| Batch dark interpolation | Per-frame `np.interp` per cam | One `np.interp` vectorized via reshape |

### 10.3 Batch size knob

`LiveUsbSource.batch_size_frames=10` (~250 ms at 40 Hz) is the tunable. Smaller = lower latency for live UI; larger = better numpy throughput. Current code is effectively batch_size=1; moving to 10 captures most of the win without affecting live UX.

Memory budget: at N=10, raw_histograms is `10 × 2 × 8 × 1024 × 4 B = 640 KB` per batch. Batches recycle (in-place mutation) so no GC churn.

## 11. Pedestal lookup — per-sensor, FW-version-keyed

The existing `MotionSensor._refresh_pedestal_height()` already implements the lookup logic (≤1.5.2 → 64.0, ≥1.5.3 → 128.0) but mutates a global module constant. New design moves the value onto a per-pipeline-instance object:

```python
@dataclass(frozen=True)
class SensorPedestals:
    left: float
    right: float

    @classmethod
    def from_sensors(cls, left_sensor, right_sensor) -> "SensorPedestals":
        return cls(
            left=_pedestal_for_fw(left_sensor.version),
            right=_pedestal_for_fw(right_sensor.version),
        )

def _pedestal_for_fw(version: tuple[int, int, int]) -> float:
    if version <= (1, 5, 2):
        return 64.0
    return 128.0

class PedestalSubtractionStage:
    def __init__(self, pedestals: SensorPedestals):
        self.pedestals = np.array([pedestals.left, pedestals.right])[None, :, None]  # (1, 2, 1) broadcast

    def process(self, batch: FrameBatch) -> FrameBatch:
        batch.display_mean = np.maximum(0.0, batch.mean_raw - self.pedestals).astype(np.float32)
        return batch
```

Result: dual-sensor systems with mixed firmware versions get correct per-side pedestals. The global `omotion.MotionProcessing.PEDESTAL_HEIGHT` is removed.

## 12. Raw CSV schema

Minimal change to today's `_RAW_CSV_HEADERS` — add one column, `type`, immediately after `timestamp_s`:

**Today:**

```
cam_id, frame_id, timestamp_s, 0, 1, ..., 1023, temperature, sum, tcm, tcl, pdc
```

**New:**

```
cam_id, frame_id, timestamp_s, type, 0, 1, ..., 1023, temperature, sum, tcm, tcl, pdc
```

`type` values: `warmup` | `dark` | `light` | `stale`.

**Stale-packet handling changes:** today the first-frame staleness check drops stale packets entirely. New design *labels* them as `type=stale` and lets them reach the raw tee, so forensics has a complete record of what came off the wire. Science stages and downstream tees (live, rolling, final) filter them out via `frame_type != "stale"`.

**Backward compat:** any reader using `pandas.read_csv` with the default header-based parsing just works. Any reader using positional indexing (`row[3]`) would now hit `type` instead of bin 0. Migration step: grep `scripts/view_corrected_scan.py` and similar tools, update any positional readers.

**Config-driven save behavior** (unchanged from today's semantics):

| `writeRawCsv` | `rawCsvDurationSec` | Behavior |
|---|---|---|
| `false` | (any) | Raw sinks ignore the `raw` channel — no rows written |
| `true` | `null` | Raw sinks write all frames for the entire scan |
| `true` | `60.0` | Raw sinks write the first 60 s of frames, then stop |

The pipeline always emits to the raw tee; sinks self-gate based on config + elapsed scan time. Same gate works for CSV and DB independently.

## 13. Error handling

Three categories with different policies:

| Category | Examples | Policy |
|---|---|---|
| **Transport-level (input is bad)** | CRC failure, histogram sum ≠ 2,457,606, USB read error, malformed CSV row in replay | Drop the offending frame, log `WARNING`, continue. The pipeline never sees the bad frame. Matches today. |
| **Domain observations (not errors)** | Dark integrity failure (u1 > pedestal+30), missing left-neighbor for stencil (fallback used), calibration array index out of bounds (identity fallback) | Append a `BatchEvent` (e.g., `DarkIntegrityWarning`, `StencilFallback`, `CalibrationFallback`) to `batch.events`. The frame still processes normally. Sinks subscribed to a `"diagnostics"` channel can record these — gives auditors a trail of what fell back when. |
| **Stage bug (shouldn't happen)** | `process()` raises unexpected exception | Runner catches, logs `ERROR` with stage name + batch summary, calls `pipeline.reset()`, **continues the scan**. Tests assert no stage raises on valid input. |

**Sink failures are isolated** — `ScanRunner._safe_consume()` wraps each sink call in try/except, logging exceptions and continuing. Sinks have weak contract (no exceptions allowed; errors via logging).

**Pipeline state on reset:** stages must implement `reset()`. Called on `on_scan_start()` for fresh state, and after any stage exception. Replay calls `reset()` before iteration.

## 14. Testing strategy

Three layers:

1. **Unit tests per stage.** `tests/test_pipeline/test_moments_stage.py`, `test_dark_correction_stage.py`, etc. Each tests one stage in isolation against synthetic `FrameBatch` inputs. Cross-checks against scalar reference implementations of the math. Properties asserted: variance ≥ 0 after clamp; contrast = std/mean; pedestal subtraction is invertible; field ownership respected.

2. **Golden replay tests.** A handful of recorded raw CSVs from past scans, checked into `tests/data/`, each paired with the expected corrected CSV. Test: run the pipeline on the raw CSV, assert output equals the golden corrected CSV byte-for-byte. These are the acceptance evidence for audits — every science change is validated against known-good outputs.

3. **End-to-end determinism tests.** Same raw source through the pipeline twice = byte-identical output. Catches accidental non-determinism (clock reads, random sampling, etc.).

**Perf regression budget.** A `pytest-benchmark` test per stage with a documented budget (e.g., `MomentsStage < 5 ms / batch of 100 frames`). CI fails on >20% regression.

**Coverage target:** unit tests cover every stage's `process()` and `reset()`; golden tests cover the full pipeline against recorded scans for: normal mode, reduced mode, short scan triggering terminal flush, scan containing a dark integrity failure.

Pure-software test markers stay the same — no hardware needed for unit + golden + replay layers.

## 15. Migration plan

Three-PR sequence on top of `feature/data-pipeline-tweaks`, designed so apps keep working at every step.

### 15.1 PR 1 — Build the new pipeline alongside (SDK only)

- Create `omotion/pipeline/` package with all stages, `FrameBatch`, `Pipeline`, `ScanRunner`, sources, sinks (CsvSink/ScanDBSink moved here)
- Full unit + golden test coverage from day one (`tests/test_pipeline/`)
- `MotionProcessing.py` untouched, still in use
- Nothing in production uses the new code yet

Reviewable as a self-contained, well-tested addition.

### 15.2 PR 2 — Switch the SDK to use it

- `ScanWorkflow.start_scan()` now constructs a `ScanRunner` instead of a `SciencePipeline`
- `MotionProcessing.py` becomes a thin shim re-exporting `Sample`, `CorrectedBatch`, `parse_histogram_packet_structured`, etc. from `omotion.pipeline.*` for any external script
- Old `SciencePipeline` class removed
- Old 6-callback `on_*_fn` kwargs on `ScanWorkflow.start_scan()` still work — proxied to new channel-based sinks under the hood — but emit `DeprecationWarning` pointing to the new sink API
- Apps work unchanged in this PR (legacy kwargs still resolve)

Hardware tests + golden replay tests gate the merge. Live pipeline behavior must be byte-identical to before — golden tests against recordings from before-the-switch are the proof.

### 15.3 PR 3 — Apps move to the new API (bloodflow-app + test-app)

- `motion_connector.py` constructs sinks instead of passing callbacks
- Drops the deprecation warnings
- One SDK release after this, the shim can be deleted

### 15.4 Release sequencing (illustrative — actual version numbers TBD at release time)

- **First SDK minor release after this work** = PR 1 + PR 2 (new pipeline in place; legacy kwargs work with warning)
- **First app release after SDK lands** = PR 3 (apps on new API; deprecation warnings drop)
- **Next SDK minor release** = remove deprecated kwarg path

### 15.5 Risk mitigation per step

- **After PR 1:** nothing changes in production. New code, fully tested
- **After PR 2:** live pipeline behavior must be byte-identical to before. Golden tests against recordings from before-the-switch are the proof. If a golden diff appears, that's the bug to fix
- **After PR 3:** apps use the new sinks; raw + corrected CSVs must be byte-identical to PR-2 outputs (same pipeline, just consumed differently)

## 16. Out-of-scope follow-ups (post-rework)

These are explicitly **not** in this rework but become easier afterward:

- Pluggable dark estimator strategies in production (the design accommodates them; user chose to defer activation)
- Shot-noise estimator variants beyond ADC_GAIN × mean × camera_gain
- New `DarkEstimator` strategies (quadratic per-camera, etc.) — drop in by passing a different object to `DarkCorrectionStage`
- Contact quality scoring as a stage — already on the roadmap

## 17. References

- [`docs/SciencePipeline.md`](../../SciencePipeline.md) — authoritative algorithm spec (every step in this design implements that doc verbatim)
- [`docs/Architecture.md`](../../Architecture.md) — current SDK module layout
- [`docs/ScanDatabase.md`](../../ScanDatabase.md) — `ScanDBSink` schema
- [`docs/superpowers/specs/2026-04-14-scan-db-sink-design.md`](2026-04-14-scan-db-sink-design.md) — prior sink protocol design
- [`docs/superpowers/specs/2026-05-20-per-frame-pdc-telemetry-design.md`](2026-05-20-per-frame-pdc-telemetry-design.md) — PDC telemetry (concurrent work on this branch)
