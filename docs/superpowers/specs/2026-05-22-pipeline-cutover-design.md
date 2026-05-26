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

### 3.0 `MotionInterface` gains baseline output configuration

CSV and scan-DB writing are SDK defaults, not app-level decisions. The SDK constructor takes the output paths once; every scan automatically writes to them. Apps never construct `CsvSink` or `ScanDBSink`.

```python
motion = MotionInterface(
    data_dir="C:/Users/ethan/Projects/scan_data",   # CSV output dir; None = no CSV
    scan_db_path=None,                                # optional; None = no DB
    operator_id="bloodflow-app",                      # app identity; written into manifests/CSVs
    # ... existing kwargs
)
```

`operator_id` is the app's identity (e.g. `"bloodflow-app"`, `"test-app"`) and was previously a per-scan field on `ScanRequest`/`ScanMetadata`. It's constant per app, so the right home for it is SDK init. `start_scan` reads it when building `ScanMetadata`; manifests, CSVs, and scan-DB sessions all carry the same value as today. Note: this is *app* identity, not a clinician/user identifier — a per-scan user tag would be a new, separate field added later if needed.

`ScanWorkflow.start_scan()` auto-injects the matching storage sinks:

```python
default_sinks: list[Sink] = []
if self._interface.data_dir is not None:
    default_sinks.append(CsvSink(output_dir=self._interface.data_dir))
if self._interface.scan_db_path is not None:
    default_sinks.append(ScanDBSink(db_path=self._interface.scan_db_path))
all_sinks = default_sinks + list(request.sinks)
```

Internal SDK workflows that don't want default storage set `request.skip_default_storage=True` (see §3.1.2).

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
def start_scan(self, request: ScanRequest) -> bool: ...
```

The 6 legacy `on_*_fn` kwargs are removed entirely. Sinks now travel **on the `ScanRequest` itself** — see §3.1.1 below.

### 3.1.1 `ScanRequest` carries only sinks (sources are SDK-managed)

`ScanRequest` gains a `sinks` field. The single dataclass captures the "what to do with this scan" contract: scan parameters + raw-save behavior + where the output goes. **Sources are SDK infrastructure** — `start_scan` builds them, including auxiliary probes like the telemetry source, based on which channels the sinks subscribe to (see §3.6):

```python
@dataclass
class ScanRequest:
    subject_id:          str
    duration_sec:        int
    left_camera_mask:    int
    right_camera_mask:   int
    reduced_mode:        bool
    raw_save_max_duration_s: Optional[float] = None  # see §3.2.1
    rolling_avg_window:  Optional[int] = None
    batch_size_frames:   Optional[int] = None
    sinks:               list[Sink] = field(default_factory=list)
    # ... other scan parameters
```

If `sinks` is empty, no sinks subscribe to any channel — the scan still runs but nothing is written or surfaced to the UI. (That's an explicit consequence of "no implicit defaults"; the SDK never silently constructs sinks the app didn't ask for.)

Call shape:
```python
req = ScanRequest(
    subject_id="...",
    duration_sec=300,
    ...,
    sinks=[
        _LivePlotSink(connector=self),
    ],
)
self._interface.start_scan(req)
```

Storage sinks (`CsvSink`, `ScanDBSink`) are **SDK-managed**, not app-constructed — see §3.0. Contact-quality is its own workflow, not a sink — see §3.8.

### 3.1.2 `skip_default_storage` flag (internal SDK use)

```python
@dataclass
class ScanRequest:
    ...
    skip_default_storage: bool = False   # internal SDK workflows set True
```

When `True`, `start_scan` skips auto-injection of the default `CsvSink` / `ScanDBSink`. Used by internal SDK workflows (`ContactQualityWorkflow`, `CalibrationWorkflow`) whose short diagnostic scans shouldn't write production CSVs. Apps never set this.

### 3.2 Pipeline construction inside `start_scan()`

The SDK constructs only the Source and the Pipeline; sinks travel on `request.sinks` (see §3.1.1):

```python
def start_scan(self, request):
    meta = ScanMetadata(
        scan_id=request.scan_id, subject_id=request.subject_id,
        operator=self._interface.operator_id, started_at_iso=...,
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

    # Auto-inject default storage sinks unless the caller opted out (internal
    # SDK workflows do this for diagnostic scans — see §3.0 / §3.1.2).
    default_sinks: list[Sink] = []
    if not request.skip_default_storage:
        if self._interface.data_dir is not None:
            default_sinks.append(CsvSink(output_dir=self._interface.data_dir))
        if self._interface.scan_db_path is not None:
            default_sinks.append(ScanDBSink(db_path=self._interface.scan_db_path))
    all_sinks = default_sinks + list(request.sinks)

    # Auxiliary sources are auto-wired based on which channels the sinks subscribe to.
    # Today: telemetry. Future: IMU, ambient light, etc. — same pattern.
    subscribed_channels = {ch for s in all_sinks for ch in s.channels}
    telemetry_source = None
    if "telemetry" in subscribed_channels:
        telemetry_source = ConsoleTelemetrySource(
            console=self._interface.console, poll_interval_s=0.1,
        )

    self._runner = ScanRunner(
        source=source, pipeline=pipeline, sinks=all_sinks,
        telemetry_source=telemetry_source,
    )
    self._scan_thread = threading.Thread(target=self._runner.run, daemon=True)
    self._scan_thread.start()
    return True
```

Storage sinks (`CsvSink`, `ScanDBSink`) are SDK-managed and auto-injected per §3.0 — apps never construct them. Apps only contribute their own UI / dev-mode sinks via `request.sinks`. Sinks are dumb consumers; they write everything they receive on subscribed channels. The raw-save *gate* lives on the pipeline's `Tee("raw")` (see §3.2.1).

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

The PR-1 skeleton's `_reader_loop` raises `NotImplementedError`. Fill it in by delegating to the legacy `parse_histogram_stream` helper (which we keep in `MotionProcessing.py` — see §3.4): the helper handles byte-buffer accumulation, multi-sample packets, 32-bit timestamp rollover, and row-sum validation. The source's only job is to convert the per-row callback into per-side `FrameBatch` objects.

```python
class LiveUsbSource(_BaseSource):
    """Per-side packet queues + per-side reader threads → shared batch queue."""

    def __init__(self, *, console, left, right, batch_size_frames=10,
                 flush_interval_s=0.25, packet_queue_size=64, metadata):
        super().__init__(metadata=metadata)
        self._console = console
        self._sensors = {"left": left, "right": right}
        self._batch_size = batch_size_frames
        self._flush_interval = flush_interval_s

        # One packet queue per active side. StreamInterface pushes raw bytes
        # in; the per-side reader_loop pulls them out.
        self._packet_queues = {
            side: queue.Queue(maxsize=packet_queue_size)
            for side, sensor in self._sensors.items() if sensor is not None
        }
        # Shared FrameBatch queue — reader threads push, __iter__ pulls.
        # Kept shallow because batches are bulky (10 frames × 2 × 8 × 1024 × 4B ≈ 640 KB).
        self._batch_queue: queue.Queue = queue.Queue(maxsize=4)
        self._stop = threading.Event()
        self._reader_threads: list[threading.Thread] = []

    def __iter__(self) -> Iterator[FrameBatch]:
        for side_name in self._packet_queues:
            self._sensors[side_name].histo_stream.start_streaming(
                self._packet_queues[side_name],
                expected_size=HISTO_PACKET_BYTES,
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
        """Per-side reader: delegates packet parsing to parse_histogram_stream,
        batches the resulting samples, and pushes FrameBatches to the shared
        batch queue.
        """
        from omotion.MotionProcessing import parse_histogram_stream

        side_idx = 0 if side_name == "left" else 1
        accumulated: list = []  # list of (cam_id, frame_id, ts, histogram, row_sum, temp)
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
            t0_normalizer=self._t0_normalize,  # inherited from _BaseSource
        )
        # When parse_histogram_stream returns (stop set + queue drained),
        # flush any remaining accumulated samples as a final batch.
        if accumulated:
            self._batch_queue.put(self._build_batch(side_idx, accumulated))

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

**Telemetry integration:** `LiveUsbSource` is now pure histogram. Per-frame `batch.pdc/tcm/tcl` fields are populated downstream by `TelemetryIngestStage` from the `TelemetryAggregator`, not by the source. See §3.6.5 for the full telemetry-correction scaffolding.

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

**Retained (consumed by `omotion.pipeline.sources.LiveUsbSource`):**
- `parse_histogram_stream()` — per-stream parser+buffer accumulator. `LiveUsbSource._reader_loop` delegates to this so it inherits byte-buffer accumulation, multi-sample packet handling, 32-bit timestamp rollover unwrapping, row-sum validation, and the `t0_normalizer` hook for scan-relative time (see §3.3).
- `parse_histogram_packet_structured()` — the per-packet parser (called internally by `parse_histogram_stream`)
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
request = ScanRequest(
    ...,
    sinks=[collector],
    skip_default_storage=True,   # diagnostic scan; no production CSV/DB
)
interface.scan_workflow.start_scan(request)
self._await_scan_complete()
# Use collector.intervals to compute the (2, 8) calibration arrays
```

The `skip_default_storage=True` opts out of the SDK's auto-injected `CsvSink` / `ScanDBSink` — calibration scans are diagnostic and shouldn't write production data-of-record CSVs. Same opt-out for `ContactQualityWorkflow` (§3.8).

### 3.6 Telemetry channel (developer-mode `_telemetry.csv`)

The legacy bloodflow-app writes a separate per-scan `<scan_id>_telemetry.csv` file in developer mode, captured by `ConsoleTelemetryPoller` at ~10 Hz. It carries PDC samples, TEC setpoint/actual, console temperature, fan speed, and safety chip status — all console-level data that's independent of the per-frame histogram stream.

PR 2 brings this into the pipeline model as a separate channel so the entire scan output flows through one diagram:

#### 3.6.1 New types

**`TelemetryEvent`** dataclass in `omotion/pipeline/batch.py`:
```python
@dataclass
class TelemetryEvent(BatchEvent):
    timestamp_s:        float        # per-scan t=0 normalized
    pdc_samples:        list[float]  # mA, PDC drain since last poll
    tec_setpoint_c:     float
    tec_actual_c:       float
    console_temp_c:     float
    fan_rpm:            int
    safety_status:      int
    # ... full surface determined at implementation time from MotionConsole's APIs
```

**`ConsoleTelemetrySource`** in `omotion/pipeline/sources.py` — a Source-shaped iterator that wraps the existing `ConsoleTelemetryPoller`:
```python
class ConsoleTelemetrySource:
    """Polls MotionConsole at fixed cadence; yields TelemetryEvents with
    scan-relative timestamps. Parallel input to ScanRunner; doesn't
    produce FrameBatch, doesn't go through the pipeline stages.
    """
    def __init__(self, *, console: "MotionConsole", poll_interval_s: float = 0.1):
        self._console = console
        self._poll_interval_s = poll_interval_s
        self._stop = threading.Event()
        self._t0: float | None = None

    def __iter__(self) -> Iterator[TelemetryEvent]:
        while not self._stop.is_set():
            snapshot = self._console.poll_telemetry(timeout=self._poll_interval_s)
            if snapshot is None:
                continue
            if self._t0 is None:
                self._t0 = snapshot.absolute_t
            yield TelemetryEvent(
                timestamp_s=snapshot.absolute_t - self._t0,
                pdc_samples=snapshot.pdc,
                tec_setpoint_c=snapshot.tec_setpoint,
                tec_actual_c=snapshot.tec_actual,
                console_temp_c=snapshot.console_temp,
                fan_rpm=snapshot.fan_rpm,
                safety_status=snapshot.safety_status,
            )

    def close(self) -> None:
        self._stop.set()
```

**`TelemetrySink`** in `omotion/pipeline/sinks.py`:
```python
class TelemetrySink:
    """Subscribes to the 'telemetry' channel; writes events to a CSV.
    Apps compose conditionally on developer_mode.
    """
    channels = {"telemetry"}

    def __init__(self, output_path: str):
        self._output_path = output_path
        self._fh = None
        self._writer: Any = None

    def on_scan_start(self, meta):
        self._fh = open(self._output_path, "w", newline="")
        self._writer = csv.writer(self._fh)
        self._writer.writerow([
            "timestamp_s", "pdc_samples_ma", "tec_setpoint_c",
            "tec_actual_c", "console_temp_c", "fan_rpm", "safety_status",
        ])

    def consume(self, channel, event: TelemetryEvent):
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
```

#### 3.6.2 ScanRunner gains a parallel telemetry thread

```python
class ScanRunner:
    def __init__(self, *, source, pipeline, sinks, telemetry_source=None):
        self.source = source
        self.pipeline = pipeline
        self.sinks = list(sinks)
        self.telemetry_source = telemetry_source
        self._telemetry_thread: threading.Thread | None = None

    def run(self) -> None:
        for sink in self.sinks: sink.on_scan_start(self.source.metadata)

        if self.telemetry_source is not None:
            self._telemetry_thread = threading.Thread(
                target=self._telemetry_loop, daemon=True,
                name="ScanRunner-telemetry",
            )
            self._telemetry_thread.start()

        try:
            for batch in self.source:
                # ... existing pipeline.process + dispatch loop ...
                pass
        finally:
            if self.telemetry_source is not None:
                self.telemetry_source.close()
                if self._telemetry_thread:
                    self._telemetry_thread.join(timeout=2.0)
            for sink in self.sinks: sink.on_complete()

    def _telemetry_loop(self) -> None:
        for event in self.telemetry_source:
            for sink in self._sinks_for("telemetry"):
                self._safe_consume(sink, "telemetry", event)
```

Same `_safe_consume` exception isolation as the main dispatch path.

#### 3.6.3 App composition (gated on developer_mode)

The app's contract is: **declare what you want via sinks**. The SDK auto-wires the matching source based on which channels are subscribed.

```python
# In bloodflow-app motion_connector.py.
# Storage sinks (CsvSink, ScanDBSink) are SDK-managed — apps never construct
# them; MotionInterface(data_dir=..., scan_db_path=...) wires them once at
# init time (see §3.0).
sinks = [_LivePlotSink(connector=self)]
if app_config.developer_mode:
    sinks.append(TelemetrySink(output_path=os.path.join(
        app_config.data_directory, f"{scan_id}_telemetry.csv",
    )))

request = ScanRequest(
    subject_id="...", duration_sec=300, ...,
    raw_save_max_duration_s=raw_max,
    sinks=sinks,
)
self._interface.start_scan(request)
```

**Auto-wiring inside `start_scan`:**

```python
subscribed_channels = {ch for s in request.sinks for ch in s.channels}
telemetry_source = None
if "telemetry" in subscribed_channels:
    telemetry_source = ConsoleTelemetrySource(console=self._interface.console)
```

When no `TelemetrySink` is in the request, no source runs — no USB chatter, no thread, no work. The app can't accidentally enable the source without a consumer (because it doesn't construct the source at all) and can't accidentally add a `TelemetrySink` without enabling the source (because the source is auto-wired).

#### 3.6.4 Why a separate source instead of folding into LiveUsbSource

The telemetry stream is **not** synchronized to the histogram stream — it's polled by the SDK over UART at a different cadence and has its own packet format. Coupling them into one source would force one cadence on both. Keeping them as parallel inputs to `ScanRunner` preserves their independence; the only shared concept is per-scan t=0 normalization (each source maintains its own `_t0`).

This generalizes for future probes: when an IMU source or ambient-light source lands, it follows the same pattern. The IMU source's sink subscribes to `"imu"`; `ScanWorkflow.start_scan` detects the channel and auto-wires the source. No changes to `ScanRequest`'s public shape.

#### 3.6.5 Telemetry-based correction (future) — scaffolding only

We anticipate a future where pipeline stages will need to correct BFI/BVI based on device telemetry (laser PDC drift, sensor temperature, etc.). PR 2 adds the minimum scaffolding so this can land cleanly later, without implementing any correction stage today.

**Three small pieces:**

1. **`TelemetryAggregator`** — thread-safe ring buffer of recent `TelemetryEvent` (default capacity 100 ≈ 10 s at 10 Hz). Exposes `snapshot_at(t: float) -> Optional[TelemetryEvent]` for stages to query the most recent telemetry at a given frame timestamp. Lives in `omotion/pipeline/telemetry.py`.

2. **`TelemetryIngestStage`** — early pipeline stage (positioned after `FrameClassificationStage`) that reads from the aggregator and populates `batch.pdc / batch.tcm / batch.tcl` for each frame in the batch. This is the bridge between the per-snapshot world (TelemetryEvent on the channel) and the per-frame world (FrameBatch fields the CsvSink raw writer expects). When no aggregator is present (telemetry source not running), the stage is a no-op and the per-frame fields stay `None`.

3. **`Pipeline.telemetry_aggregator: TelemetryAggregator | None`** — optional attribute on the Pipeline. `default_pipeline()` factory constructs one and assigns it; the factory also adds the `TelemetryIngestStage` to the stage list. Future correction stages take `aggregator` as a constructor arg.

**ScanRunner wires events to the aggregator inside `_telemetry_loop`:**

```python
def _telemetry_loop(self):
    for event in self.telemetry_source:
        if self.pipeline.telemetry_aggregator is not None:
            self.pipeline.telemetry_aggregator.update(event)
        for sink in self._sinks_for("telemetry"):
            self._safe_consume(sink, "telemetry", event)
```

One extra line in the runner. Free at runtime if no aggregator is present.

**LiveUsbSource simplification:** the earlier text in §3.3 mentioned LiveUsbSource pumping telemetry into FrameBatch via a poller hook. **Drop that.** With `TelemetryIngestStage` in the pipeline, LiveUsbSource is now pure histogram — it leaves `batch.pdc/tcm/tcl` as `None`, and the stage fills them downstream from the aggregator. One responsibility per piece.

**When a future correction stage lands** (e.g., `PdcCorrectionStage`):

```python
class PdcCorrectionStage:
    name = "pdc_correction"
    def __init__(self, *, aggregator: TelemetryAggregator):
        self._aggregator = aggregator

    def process(self, batch):
        for i in range(batch.bfi_live.shape[0]):
            event = self._aggregator.snapshot_at(batch.timestamp_s[i])
            if event is not None:
                # Apply PDC-based correction using event.pdc_samples
                ...
        return batch

    def reset(self): pass
```

`default_pipeline()` constructs it with `aggregator=pipeline.telemetry_aggregator` and inserts it at the right point in the stage list. No plumbing changes elsewhere. The aggregator is just a dependency passed in via constructor.

**What we are NOT designing today:** which corrections to apply, how to scale BFI/std from PDC drift, whether to invalidate frames during safety-chip errors, etc. That's a separate design when the science is ready.

### 3.8 ContactQualityWorkflow — moves CQ from app to SDK

The legacy bloodflow-app owns contact-quality (CQ) check logic: a short scan that monitors per-camera signal levels against dark/light thresholds, returning pass/fail. This is a clinical *procedure*, not a UI concern — the GUI's only job is to display the result. PR 2 moves it to the SDK, symmetric with `CalibrationWorkflow`.

**New module: `omotion/ContactQualityWorkflow.py`** (~150 LOC including the internal sink).

```python
@dataclass
class CamCQResult:
    side:     str
    cam_id:   int
    passed:   bool
    avg_bfi:  float
    reason:   str   # "ok" | "below_dark" | "above_light" | "no_signal"


@dataclass
class ContactQualityResult:
    passed:      bool                                    # all active cams passed
    per_camera:  dict[tuple[str, int], CamCQResult]
    duration_sec: float


class _ContactQualitySink:
    """Internal: collects rolling-averaged BFI per camera during a short scan.
    Not exposed to apps."""
    channels = {"rolling"}

    def __init__(self, dark_thresholds, light_thresholds):
        self._dark = dark_thresholds   # list[float], length 8
        self._light = light_thresholds
        self._accum: dict[tuple[str, int], list[float]] = {}

    def on_scan_start(self, meta): pass

    def consume(self, channel, batch):
        # batch.bfi_rolling has shape (N, 2, 8); accumulate per-cam
        for i in range(batch.bfi_rolling.shape[0]):
            if batch.frame_type[i] in ("warmup", "stale"):
                continue
            for side_idx, side in enumerate(("left", "right")):
                for cam_id in range(8):
                    v = float(batch.bfi_rolling[i, side_idx, cam_id])
                    if not np.isfinite(v):
                        continue
                    self._accum.setdefault((side, cam_id), []).append(v)

    def on_complete(self): pass

    def result(self, *, left_mask: int, right_mask: int,
               duration_sec: float) -> ContactQualityResult:
        per_cam: dict[tuple[str, int], CamCQResult] = {}
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
            per_camera=per_cam, duration_sec=duration_sec,
        )


class ContactQualityWorkflow:
    """Run a short scan, check signal levels against thresholds, return verdict.

    Symmetric with CalibrationWorkflow: uses an internal sink to collect the
    diagnostic data, skips default storage (no CSVs for CQ checks), returns
    a structured result the app displays via its modal.
    """
    def __init__(self, scan_workflow):
        self._scan_workflow = scan_workflow

    def check(self, *, duration_sec: float = 1.0,
              rolling_window: int = 10,
              dark_threshold_per_camera: list[float],
              light_threshold_per_camera: list[float],
              left_camera_mask: int, right_camera_mask: int) -> ContactQualityResult:
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
            skip_default_storage=True,   # diagnostic scan; no CSV/DB
        )
        self._scan_workflow.start_scan(request)
        self._scan_workflow.await_complete(timeout_sec=duration_sec + 2.0)
        return sink.result(
            left_mask=left_camera_mask,
            right_mask=right_camera_mask,
            duration_sec=duration_sec,
        )
```

**`MotionInterface` lazy-loads it:**

```python
@property
def contact_quality_workflow(self) -> ContactQualityWorkflow:
    if self._cq_workflow is None:
        self._cq_workflow = ContactQualityWorkflow(scan_workflow=self.scan_workflow)
    return self._cq_workflow
```

**App side becomes one call:**

```python
result = self._interface.contact_quality_workflow.check(
    duration_sec=app_config.cq_check_duration_sec,
    dark_threshold_per_camera=app_config.cq_dark_threshold_per_camera,
    light_threshold_per_camera=app_config.cq_light_threshold_per_camera,
    left_camera_mask=request.left_camera_mask,
    right_camera_mask=request.right_camera_mask,
)
if not result.passed:
    self._show_reposition_modal(result)
```

**App-side changes for this:**

- Delete the CQ callback closures (`_make_contact_quality_callbacks`, `_on_dark_frame`, `_on_rolling_avg` — about 100 LOC in `motion_connector.py:2470-2582`)
- Replace with the one-line `contact_quality_workflow.check(...)` call
- Keep the modal display logic (UI is still app's job — only the procedure moves)

### 3.9 Test-scan support (from feature/132)

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


# In start_scan call site (replaces the 4-callback kwarg call):
# Storage sinks (CsvSink, ScanDBSink) are SDK-managed; the app never constructs
# them — MotionInterface init carries data_dir / scan_db_path (see §3.0).
# Contact quality is its own SDK workflow, not a sink (see §3.8).
sinks = [_LivePlotSink(connector=self)]
if self._app_config.developer_mode:
    sinks.append(TelemetrySink(output_path=os.path.join(
        self._app_config.data_directory, f"{scan_id}_telemetry.csv",
    )))

# Raw-save gate is computed from app config and lives on the request — the
# SDK passes it to default_pipeline so the Tee("raw") enforces it uniformly
# across the auto-injected CsvSink, ScanDBSink, and any other sink subscribed
# to "raw".
raw_max = (
    self._app_config.raw_data_duration_sec
    if self._app_config.write_raw_data else 0
)

request = ScanRequest(
    subject_id="...", duration_sec=300, ...,
    raw_save_max_duration_s=raw_max,
    sinks=sinks,
)
self._interface.start_scan(request)

# Contact quality runs as a separate SDK workflow (§3.8):
cq = self._interface.contact_quality_workflow.check(
    duration_sec=self._app_config.cq_check_duration_sec,
    rolling_window=self._app_config.cq_rolling_avg_window,
    dark_threshold_per_camera=self._app_config.cq_dark_threshold_per_camera,
    light_threshold_per_camera=self._app_config.cq_light_threshold_per_camera,
    left_camera_mask=request.left_camera_mask,
    right_camera_mask=request.right_camera_mask,
)
if not cq.passed:
    self._show_reposition_modal(cq)
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
