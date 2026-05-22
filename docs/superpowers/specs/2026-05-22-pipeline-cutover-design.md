# Pipeline Cutover (PR 2 + PR 3) — Design

**Date:** 2026-05-22
**Branch:** off `feature/data-pipeline-tweaks` (or whatever lands first via PR #56)
**Status:** Approved by author; pending review
**Predecessor:** [`2026-05-22-data-pipeline-rearchitecture-design.md`](2026-05-22-data-pipeline-rearchitecture-design.md) (PR 1 built the new pipeline alongside the legacy code)

---

## 1. Summary

PR 2 + PR 3 of the 3-PR sequence: switch `ScanWorkflow` to use the new `omotion.pipeline` package and migrate both apps (`bloodflow-app`, `test-app`) to the new sink API in the same release. The legacy `SciencePipeline` class is removed; `MotionProcessing.py` becomes a thin compat shim retaining only the packet-parsing helpers and the `Sample` / `CorrectedBatch` dataclasses.

**Clean break, no deprecation bridge.** Apps in this release pin against the SDK with the new interface; older apps cannot run against the new SDK and vice versa.

## 2. Scope

**In scope (3 repos, 1 release event):**

- **openmotion-sdk:** `ScanWorkflow.start_scan()` rewritten to construct a `ScanRunner`; `LiveUsbSource._reader_loop` wired up; `MotionProcessing.py` collapsed to a shim; `CalibrationWorkflow` migrated to sinks
- **openmotion-bloodflow-app:** `motion_connector.py` rewritten to construct sinks instead of passing callbacks
- **openmotion-test-app:** same as bloodflow-app, smaller scope

**Out of scope:**

- Algorithmic changes to BFI/BVI / dark-correction (PR 1's work is the algorithm, unchanged)
- Wire-protocol or firmware changes
- New science features (PDC plumbing, contact-quality model changes, etc.)
- Backward-compat with apps that pin an older SDK

## 3. SDK-side cutover

### 3.1 `ScanWorkflow.start_scan()` signature change

**Before:**
```python
def start_scan(
    self,
    request: ScanRequest,
    on_uncorrected_fn: Callable | None = None,
    on_corrected_batch_fn: Callable | None = None,
    on_dark_frame_fn: Callable | None = None,
    on_rolling_avg_fn: Callable | None = None,
    on_realtime_corrected_fn: Callable | None = None,
    on_raw_frame_fn: Callable | None = None,
    ...
) -> bool: ...
```

**After:**
```python
def start_scan(
    self,
    request: ScanRequest,
    sinks: list[Sink] | None = None,    # app-provided UI / CQ / custom sinks
    ...
) -> bool: ...
```

The 6 legacy `on_*_fn` kwargs are removed entirely.

### 3.2 Sink composition inside `start_scan()`

The SDK constructs only the Source and the Pipeline. **All sinks** (UI, contact-quality, storage) are app-provided via the `sinks=` kwarg:

```python
def start_scan(self, request, sinks=None, ...):
    meta = ScanMetadata(
        scan_id=request.scan_id, subject_id=request.subject_id,
        operator=request.operator, started_at_iso=...,
        duration_sec=request.duration_sec,
        left_camera_mask=request.left_camera_mask,
        right_camera_mask=request.right_camera_mask,
        reduced_mode=request.reduced_mode,
        # ScanMetadata no longer carries write_raw_csv / raw_csv_duration_sec —
        # those are app-level config, not scan-request parameters. The raw-tee
        # time gate is configured on the pipeline (see below); the sinks
        # themselves are dumb consumers.
    )

    # Calibration is loaded from console EEPROM at connection time and lives on
    # MotionConsole as `console.calibration` per omotion/MotionConsole.py. If
    # the field name differs at implementation time, update accordingly.
    calibration = self._interface.console.calibration
    pedestals = SensorPedestals.from_sensors(
        left=self._interface.left, right=self._interface.right,
    )

    # raw_save_max_duration_s comes from the request (which the app populated
    # from its own config). None ⇒ raw tee emits for entire scan; positive
    # float ⇒ stops emitting after that many seconds of scan time; 0 or
    # negative ⇒ raw tee is dropped from the pipeline entirely (no raw save).
    pipeline = default_pipeline(
        metadata=meta,
        calibration=calibration,
        pedestals=pedestals,
        rolling_avg_window=request.rolling_avg_window or 10,
        raw_save_max_duration_s=request.raw_save_max_duration_s,
    )

    source = LiveUsbSource(
        console=self._interface.console,
        left=self._interface.left,
        right=self._interface.right,
        batch_size_frames=request.batch_size_frames or 10,
        metadata=meta,
    )

    user_sinks = sinks or []
    self._runner = ScanRunner(source=source, pipeline=pipeline, sinks=user_sinks)
    self._scan_thread = threading.Thread(target=self._runner.run, daemon=True)
    self._scan_thread.start()
    return True
```

**Apps construct their own storage sinks**, including `CsvSink` and `ScanDBSink`. Sinks are dumb consumers; they write everything they receive on subscribed channels. The raw-save *gate* lives on the pipeline's `Tee("raw")` (see §3.2.1).

### 3.2.1 Raw-save gating moves from sinks to the Tee

PR 1 had each sink self-gate on `meta.write_raw_csv` + `meta.raw_csv_duration_sec`. That created two problems: duplicate config per sink, and only CSV-specific semantics (CsvSink had the gate; ScanDBSink had its own copy). PR 2 lifts the gate to the pipeline's raw tee so **every** sink subscribed to `"raw"` honors the same setting uniformly.

**Changes:**

1. **`Tee` class** gains an optional `max_duration_s: float | None = None` constructor parameter. When set, the Tee checks the first frame's `batch.timestamp_s[0]` (per PR 1's source-normalized scan-relative time, t=0 at scan start) against `max_duration_s` and skips emission for any batch whose first frame is past the budget.

2. **`default_pipeline()` factory** gains a `raw_save_max_duration_s: float | None = None` parameter. The factory:
   - `None` → adds `Tee("raw", filter=None, max_duration_s=None)` (emit for entire scan)
   - `> 0` → adds `Tee("raw", filter=None, max_duration_s=value)` (emit until cap)
   - `0 or negative` → omits the `Tee("raw")` from the pipeline entirely (no raw save anywhere)

3. **`ScanMetadata`** drops `write_raw_csv` and `raw_csv_duration_sec` fields. The Sink protocol still receives `ScanMetadata` at `on_scan_start`, but the dataclass is now just identification + scan parameters.

4. **`CsvSink`** drops `write_raw_csv` / `raw_csv_duration_sec` from its self-gating logic. Constructor becomes `CsvSink(output_dir)`. Inside `consume("raw", batch)` it always writes — the gate is upstream.

5. **`ScanDBSink`** same — constructor becomes `ScanDBSink(db_path)` with no save flags.

6. **`ScanRequest`** gains one new field: `raw_save_max_duration_s: float | None` (semantics above). It drops the two old `write_raw_csv` / `raw_csv_duration_sec` fields. The app computes this single field from its `appConfig.writeRawData` and `appConfig.rawDataDurationSec`:
   ```python
   raw_save_max_duration_s = (
       None if not appConfig.writeRawData and appConfig.rawDataDurationSec is None
       else 0 if not appConfig.writeRawData
       else appConfig.rawDataDurationSec   # None or positive float
   )
   ```
   *(In practice apps will likely just pass `appConfig.rawDataDurationSec if appConfig.writeRawData else 0`.)*

**Edge case explicitly out of scope:** different save rules per sink (e.g., CSV writes raw forever but DB only first 60s). PR 2 doesn't support this — the gate is unified at the Tee. If needed later, a custom filter sink can add per-sink filtering on top.

### 3.3 LiveUsbSource wiring

The PR-1 skeleton's `_reader_loop` raises `NotImplementedError`. Fill it in by consuming the existing `StreamInterface` packet queue:

```python
class LiveUsbSource(_BaseSource):
    def __init__(self, *, console, left, right, batch_size_frames=10,
                 flush_interval_s=0.25, queue_size=64, metadata):
        super().__init__(metadata=metadata)
        self._console = console
        self._left = left
        self._right = right
        self._batch_size = batch_size_frames
        self._flush_interval = flush_interval_s
        self._packet_queue: queue.Queue = queue.Queue(maxsize=queue_size)
        self._batch_queue: queue.Queue = queue.Queue(maxsize=4)
        self._stop = threading.Event()
        self._reader_threads: list[threading.Thread] = []

    def __iter__(self) -> Iterator[FrameBatch]:
        # Start per-side StreamInterface streaming into _packet_queue
        for side_name, sensor in (("left", self._left), ("right", self._right)):
            if sensor is None:
                continue
            sensor.histo_stream.start_streaming(
                self._packet_queue, expected_size=HISTO_PACKET_BYTES,
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

    def _reader_loop(self, side_name: str) -> None:
        """Pull parsed HistogramSamples from _packet_queue, accumulate, yield FrameBatches."""
        from omotion.MotionProcessing import parse_histogram_packet_structured

        accumulated: list[HistogramSample] = []
        last_flush = time.monotonic()
        while not self._stop.is_set():
            try:
                raw_bytes = self._packet_queue.get(timeout=self._flush_interval)
            except queue.Empty:
                if accumulated:
                    self._batch_queue.put(self._build_batch(side_name, accumulated))
                    accumulated.clear()
                    last_flush = time.monotonic()
                continue

            sample = parse_histogram_packet_structured(raw_bytes)
            if sample is None:
                continue   # corrupt packet; logged by parser
            accumulated.append(sample)

            now = time.monotonic()
            if (len(accumulated) >= self._batch_size or
                    now - last_flush >= self._flush_interval):
                self._batch_queue.put(self._build_batch(side_name, accumulated))
                accumulated.clear()
                last_flush = now

    def _build_batch(self, side_name: str, samples: list) -> FrameBatch:
        """Convert N HistogramSamples into one FrameBatch.

        Each sample populates one (frame, side, cam) slot in the (N, 2, 8, 1024)
        raw_histograms array. The other side's slot is left zero — downstream
        stages don't care because frame_type / classification keys off the
        populated slot via the argmax trick the FrameClassificationStage uses.
        For batches mixing both sides (in practice rare since two reader
        threads each push their own batches), each row picks its own side.
        """
        n = len(samples)
        cam_ids = np.array([s.cam_id for s in samples], dtype=np.int8)
        frame_ids = np.array([s.frame_id for s in samples], dtype=np.uint8)
        timestamp_s = np.array([s.timestamp_s for s in samples], dtype=np.float64)
        side_idx = 0 if side_name == "left" else 1

        raw_hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
        temps = np.zeros((n, 2, 8), dtype=np.float32)
        for i, s in enumerate(samples):
            raw_hist[i, side_idx, s.cam_id] = s.histogram
            temps[i, side_idx, s.cam_id] = s.temperature_c
        return FrameBatch(
            cam_ids=cam_ids, frame_ids=frame_ids,
            raw_histograms=raw_hist, temperature_c=temps,
            timestamp_s=timestamp_s, pdc=None, tcm=None, tcl=None,
        )
```

Two reader threads (one per side) push parsed samples into a shared `_batch_queue`; the iterator yields from that.

**ConsoleTelemetry integration:** an optional `ConsoleTelemetryPoller` runs in the background polling PDC/TEC at 10 Hz. The `LiveUsbSource` exposes a small `update_telemetry(tcm, tcl, pdc)` callback; the poller calls it. When building the next batch, the source attaches the most recent telemetry values to each frame's `pdc`/`tcm`/`tcl` fields.

### 3.4 `MotionProcessing.py` becomes a shim

**Removed (now lives in `omotion.pipeline`):**
- `SciencePipeline` class (1,200 LOC)
- `FrameIdUnwrapper` (replaced by `_FrameUnwrapper` in `classify.py`)
- `_check_dark_integrity` (now `DarkIntegrityGuard`)
- `_emit_realtime_corrected` (now `HybridRealtimePredictor`)
- `_emit_corrected_for_camera` (now `LinearInterpolation`)
- `_calibrate_bfi_bvi` (now `BfiBviStage`)
- `_flush_terminal_dark` (now `DarkCorrectionStage.on_scan_stop`)
- `create_science_pipeline` factory
- `parse_histogram_stream` (per-stream parser/queue glue — not needed; sources do this now)

**Retained (consumed by `omotion.pipeline.sources.LiveUsbSource`):**
- `parse_histogram_packet_structured()` — the per-packet parser
- `_rle_decompress()`, `_util_crc16()` — packet validation helpers
- `EXPECTED_HISTOGRAM_SUM`, `HISTO_SIZE_WORDS`, `HISTOGRAM_BYTES` constants
- `Sample`, `CorrectedBatch` dataclasses (legacy emit shapes; new pipeline still uses these in places, and external scripts may import them)
- `PEDESTAL_HEIGHT` constant (legacy module global; replaced functionally by `SensorPedestals` but kept for any external dependent code reading the value)

**Final shape:** ~400 LOC of parsing + dataclasses. File can be renamed in a later cleanup if desired, but keep the name `MotionProcessing.py` here to preserve external import compat.

### 3.5 `CalibrationWorkflow` migration

`run_calibration()` currently calls `interface.scan_workflow.start_scan(..., on_corrected_batch_fn=_on_corrected_batch, ...)`. Migrate to:

```python
class _CalibrationCollectorSink:
    """Buffers CorrectedIntervals from a calibration scan for downstream
    array computation."""
    channels = {"final"}

    def __init__(self):
        self.intervals: list[EnrichedCorrectedInterval] = []

    def on_scan_start(self, meta): pass
    def consume(self, channel, payload):
        if channel == "final":
            self.intervals.append(payload)
    def on_complete(self): pass


# In run_calibration:
collector = _CalibrationCollectorSink()
interface.scan_workflow.start_scan(request, sinks=[collector])
self._await_scan_complete()
# Use collector.intervals to compute the (2, 8) calibration arrays
```

Same pattern for any other internal SDK callers (none expected beyond `CalibrationWorkflow`).

### 3.6 Test-scan support (from feature/132)

`CalibrationWorkflow.start_test_scan()` (added in PR #53 on `next`) uses the same callback machinery. Migrate the same way — replace its `on_corrected_batch_fn=...` with a collector sink.

## 4. App-side cutover (PR 3 portion)

### 4.1 bloodflow-app

`motion_connector.py` constructs **two specialized sinks per scan**:

```python
class _LivePlotSink:
    """'live' channel: per-frame best-effort corrected BFI/BVI for the QML plot."""
    channels = {"live"}

    def __init__(self, connector):
        self._connector = connector

    def on_scan_start(self, meta):
        self._meta = meta

    def consume(self, channel, batch):
        # In reduced mode, batch.bfi_live_side is populated (shape (N, 2));
        # otherwise read batch.bfi_live (shape (N, 2, 8)) and slice active cams.
        n = batch.bfi_live.shape[0]
        for i in range(n):
            if batch.frame_type[i] in ("warmup", "stale"):
                continue
            if self._meta.reduced_mode:
                bfi = batch.bfi_live_side[i]
                bvi = batch.bvi_live_side[i]
            else:
                bfi = batch.bfi_live[i]
                bvi = batch.bvi_live[i]
            self._connector._emit_sample_to_qml(
                bfi=bfi, bvi=bvi,
                frame_id=int(batch.abs_frame_ids[i]),
                timestamp_s=float(batch.timestamp_s[i]),
            )

    def on_complete(self): pass


class _ContactQualityCheckSink:
    """'rolling' channel for smoothed BFI; 'diagnostics' for DarkIntegrityWarning events."""
    channels = {"rolling", "diagnostics"}

    def __init__(self, connector):
        self._connector = connector

    def consume(self, channel, payload):
        if channel == "rolling":
            # rolling-averaged BFI used for contact-quality threshold checks
            ...
        elif channel == "diagnostics":
            if isinstance(payload, DarkIntegrityWarning):
                self._connector._on_dark_integrity_warning(payload)


# In start_scan call site (replaces the 4-callback kwarg call):
# The app constructs storage sinks itself with destination paths from config.
sinks = [
    _LivePlotSink(connector=self),
    _ContactQualityCheckSink(connector=self),
    CsvSink(output_dir=self._app_config.data_directory),
]
if self._app_config.scan_db_enabled:
    sinks.append(ScanDBSink(db_path=self._app_config.scan_db_path))

# Raw-save gate is computed from app config and lives on the request — the
# SDK passes it to default_pipeline so the Tee("raw") enforces it uniformly
# across CsvSink, ScanDBSink, and any other sink subscribed to "raw".
request.raw_save_max_duration_s = (
    self._app_config.raw_data_duration_sec
    if self._app_config.write_raw_data else 0
)

self._interface.start_scan(request, sinks=sinks)
```

**Code removed from `motion_connector.py`:**
- The reduced-mode `_reduced_uncorr_buf` / `_reduced_corr_buf` accumulator dicts (~80 LOC) — SideAveragingStage does this in the pipeline now
- The callback closure factories `_make_contact_quality_callbacks()`, `_on_uncorrected`, `_on_corrected_batch` (~200 LOC) — replaced by the two sink classes (~150 LOC)
- Net change: ~130 LOC removed from `motion_connector.py`

### 4.2 test-app

Smaller scope — likely just one `_TestAppLiveSink` subscribing to `"live"` for the test UI's display. The test-app doesn't do contact-quality.

`motion_connector.py` in test-app gets a similar surgery: one sink class replaces the callback closures.

### 4.3 App-side test updates

App tests that mock the callback API need to be updated to mock the sink protocol instead. Each app has 5-10 such tests.

## 5. Tests

### 5.1 SDK tests

**Existing PR-1 tests:** all 116 should continue to pass. PR 2 doesn't change the pipeline internals.

**New SDK tests:**
- `tests/test_scan_workflow_pipeline.py` — verify `ScanWorkflow.start_scan` constructs the right runner with the right sinks based on request fields
- `tests/test_live_usb_source.py` — hardware-marked smoke test that LiveUsbSource yields FrameBatches when sensors are connected
- `tests/test_calibration_workflow_pipeline.py` — verify CalibrationWorkflow's collector sink receives the corrected intervals it needs

**Updated tests:** the existing `tests/test_pipeline_csv.py` and `tests/test_calibration_workflow_compute.py` that exercised the legacy `SciencePipeline` directly — rewrite to use the new pipeline.

### 5.2 App-side tests

- Update mock fixtures that previously mocked `on_*_fn` callbacks to mock `Sink.consume(channel, payload)` instead
- Smoke test that the sink classes produce the correct Qt signal sequence for synthetic FrameBatches

### 5.3 Hardware-validation acceptance criteria

Before merging, run a real scan with the new pipeline against a phantom and confirm:
- Live BFI/BVI plot updates smoothly during the scan
- Raw + corrected CSVs land in `dataDirectory` with the same column layout as today
- Contact-quality check still works during the prelude
- The scan-DB session persists correctly (if enabled)

This is manual hardware validation, not an automated test.

## 6. Migration & release ordering

### 6.1 Branch / PR structure

Three branches across three repos, three PRs, ideally tagged in one release:

| Repo | Branch (off respective `next`) | PR title |
|---|---|---|
| openmotion-sdk | `feature/pipeline-cutover` | Pipeline cutover: ScanWorkflow uses new pipeline; remove legacy SciencePipeline |
| openmotion-bloodflow-app | `feature/pipeline-sinks` | Migrate motion_connector to new pipeline sinks |
| openmotion-test-app | `feature/pipeline-sinks` | Migrate motion_connector to new pipeline sinks |

The SDK branch can develop and test on its own (with mocked sinks). The app branches need a pre-release SDK tag to pin against — once the SDK PR is review-approved, tag a pre-release version (e.g., `1.7.0-rc.0`) for the apps to consume, then merge all three PRs together.

### 6.2 Release order

1. **SDK PR opens** — full test suite passes (no apps yet)
2. **SDK PR reviewed + approved**
3. **Tag SDK `1.7.0-rc.0`** (on the PR's HEAD, not yet merged)
4. **App PRs open** pinning the rc — each app builds against the new SDK, tests pass
5. **App PRs reviewed + approved**
6. **Merge all three PRs to their respective `next` branches in a coordinated push**
7. **Tag SDK `1.7.0` stable, app `X.Y.Z` releases** as the followup

### 6.3 Rollback story

If the post-merge hardware validation finds a regression, the rollback is:
1. Revert all three PRs from their respective `next` branches
2. Re-tag prior SDK + app versions if needed

Because PR 1 (the pipeline package) doesn't get touched by the rollback, the new code lives on but is unused — clean state for retry.

## 7. Risk catalog

| Risk | Likelihood | Severity | Mitigation |
|---|---|---|---|
| LiveUsbSource doesn't keep up with USB throughput | Medium | High | Benchmark against recorded scan + 40 Hz playback; queue sizes (64 / 4) tuned for headroom |
| App's Qt-signal emit rate differs subtly, UI feels different | Medium | Medium | Manual UX validation; option to revert to callback-style if needed |
| `MotionProcessing.py` shim breaks an external script we forgot about | Low | Low | grep for `from omotion.MotionProcessing import` across all repos before merging |
| CalibrationWorkflow's `_CalibrationCollectorSink` semantics diverge from the legacy callback timing | Medium | Medium | Hardware test: run a full calibration cycle; compare per-camera arrays against legacy |
| First-interval dark-frame stencil fallback diverges slightly from legacy (we observed 0.016 max diff on darks during PR 1 functional test) | Low | Low | Already documented; if the user-facing dark-frame BFI looks off, follow up with a stencil-fallback fix |

## 8. Non-goals (explicit deferrals)

- Renaming `MotionProcessing.py` — keeps external import paths working; rename in a separate housekeeping PR if desired
- Removing `Sample` / `CorrectedBatch` dataclasses from `MotionProcessing.py` — apps may still consume them via Sink payloads
- Improving the dark-frame stencil first-interval fallback — minor numerical issue, fix later
- ConsoleTelemetry refactor — kept as-is; `LiveUsbSource` consumes from it via a thin hook
- Adding new science features (PDC corrections, new estimators, etc.)

## 9. References

- [PR 1 design doc (predecessor)](2026-05-22-data-pipeline-rearchitecture-design.md)
- [`docs/SciencePipeline.md`](../../SciencePipeline.md) — algorithm reference
- [`docs/Architecture.md`](../../Architecture.md) — current SDK layout
