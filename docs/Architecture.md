# Open-Motion SDK — Software Architecture

## Overview

The Open-Motion SDK is a Python library for controlling optical speckle imaging hardware. It manages two classes of physical device: a **console module** (connected via USB virtual COM port / UART) and up to two **sensor modules** (connected via composite USB bulk-transfer interfaces). The SDK handles device discovery, connection lifecycle, command/response communication, high-speed histogram streaming, science computation, and firmware programming. A thin signal abstraction makes the same SDK usable in both PyQt6 desktop applications and headless Python scripts.

---

## Layer diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Application / QML UI                         │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ signals / callbacks
┌──────────────────────────────▼──────────────────────────────────────┐
│                         MotionInterface                              │
│   console · left · right (stable handles) · scan_workflow            │
│   ConnectionMonitor (single daemon thread; OS hotplug + 200ms poll)  │
└──────────────┬──────────────────────────────┬───────────────────────┘
               │                              │
               │  Console path                │  Sensor path (×2: left, right)
               │  (USB VCP / pyserial)        │  (USB bulk transfer)
               │                              │
┌──────────────▼──────────────┐  ┌────────────▼──────────────────────┐
│        MotionConsole        │  │      MotionSensor  (per side)      │
│   + ConsoleTelemetryPoller  │  │   state machine:                   │
│                             │  │   DISCONNECTED → CONNECTING        │
│                             │  │              → CONNECTED            │
└──────────────┬──────────────┘  └────────────┬──────────────────────┘
               │                              │
┌──────────────▼──────────────┐  ┌────────────▼──────────────────────┐
│         MotionUart          │  │    MotionComposite (uart attr)     │
│      (pyserial VCP)         │  │                                    │
│  UartPacket framing + CRC   │  │  CommInterface   StreamInterface   │
└──────────────┬──────────────┘  │  (cmd/resp)      (histo / imu)    │
               │                 │  USB bulk IF 0   USB bulk IF 1+2   │
          pyserial               └────────────┬──────────────────────┘
               │                              │
          USB VCP                       libusb / pyusb
```

A separate **data pipeline** layer sits between the workflow orchestration and the sinks that consume processed data:

```
┌─────────────────────────────────────────────────────────────────────┐
│                       ScanWorkflow (per scan)                       │
│   ScanRequest → start_scan(...) → spawn ScanRunner                  │
└──────────────┬──────────────────────────────────────────────────────┘
               │
┌──────────────▼──────────────────────────────────────────────────────┐
│                       ScanRunner   (omotion/pipeline/runner.py)     │
│   pulls FrameBatches from Source, runs through Pipeline,            │
│   dispatches events to Sinks on named channels                      │
└──┬──────────────────────────────┬──────────────────────────────┬───┘
   │  Sources                     │  Pipeline (stages)           │ Sinks
   │ (omotion/pipeline/sources.py)│ (omotion/pipeline/*.py)      │ (omotion/pipeline/sinks.py)
   │                              │                              │
   │  LiveUsbSource               │ default_pipeline():          │  CsvSink  (raw, final)
   │  CsvReplaySource             │  Classify → TelemetryIngest  │  ScanDBSink (raw, final)
   │  DbReplaySource              │  → Tee(raw) → NoiseFloor     │  TelemetrySink (telemetry)
   │                              │  → Moments → PedestalSub     │  app live/final plot sinks
   │  ConsoleTelemetrySource      │  → DarkCorrection → ShotNoise│  CalibrationWorkflow sinks
   │  (separate thread)           │  → BfiBvi → SideAvg          │  ContactQuality sink
   │                              │  → Tee(live)                 │  + your own
   └──────────────────────────────┴──────────────────────────────┘
                  │                                          ▲
                  │  FrameBatch (mutated in place)            │
                  │  batch.events: LiveEmit / IntervalClosed  │
                  └─── channels: raw, live, final,           ──┘
                       telemetry, diagnostics
```

This package is the focus of [`SciencePipeline.md`](SciencePipeline.md), which documents every stage, every field, and the channel contract.

---

## Module reference

### Foundational

| Module | Purpose |
|---|---|
| `__init__.py` | Package entry point; exposes `_log_root`, `set_log_root()`, SDK version |
| `config.py` | All protocol constants: packet types, command bytes, PID/VID, hardware geometry |
| `connection_state.py` | `ConnectionState` enum: `DISCONNECTED → DISCOVERED → CONNECTING → CONNECTED / ERROR` |
| `utils.py` | CRC-16 lookup table, `util_crc16()`, VCP port listing, hex formatting |
| `CommandError.py` | `CommandError(RuntimeError)` — raised when hardware returns NAK / BAD_CRC / OW_ERROR |
| `usb_backend.py` | Platform-specific libusb-1.0 backend loader (vendored DLL on Windows) |

### Signal system

| Module | Purpose |
|---|---|
| `MotionSignal.py` | `MotionSignal` — lightweight signal with `.connect()` / `.disconnect()` / `.emit()` |
| `signal_wrapper.py` | `SignalWrapper` base class — uses real `pyqtSignal` if PyQt6 is present, falls back to `MotionSignal` |

Every class that exposes device events inherits `SignalWrapper` and uses the three standard signals: `signal_connect(str, str)`, `signal_disconnect(str, str)`, `signal_data_received(str, str)`.

### Packet structures

The SDK speaks two wire formats with the hardware: **UART control packets**
to the console module, and **USB-bulk histogram packets** from the sensor
modules. Both validate CRC on receive and raise `ValueError` on mismatch.
Authoritative spec: [`openmotion-console-fw/CommandHandling.md`](../../openmotion-console-fw/CommandHandling.md);
packet-type and opcode enums in [`omotion/config.py`](../omotion/config.py).

**UART packet** (`MotionUart` / `UartPacket.py`, also used for USB-bulk
command/response on the sensor):

```
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────────┬──────┬──────┐
│ 0xAA │ id:2 │ type │ cmd  │ addr │ rsvd │ len:2│ payload:N│ crc:2│ 0xDD │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────────┴──────┴──────┘
  SOF                                                          CRC-16  EOF
```

- `id` (LE u16) — sequence number, echoed in the response so async callers
  can match request → reply
- `type` — one of `OW_ACK`, `OW_NAK`, `OW_CMD`, `OW_RESP`, `OW_DATA`,
  `OW_JSON`, `OW_FPGA`, `OW_CAMERA`, `OW_IMU`, `OW_I2C_PASSTHRU`,
  `OW_CONTROLLER`, `OW_ERROR`
- `cmd` / `addr` — opcode + sub-address (semantics per packet type)
- `len` (LE u16) — payload byte count
- `crc` — CRC-16 lookup-table variant (`UartPacket._crc16`)
- Max payload: **2048 B (console)**, **8192 B (sensor)**

**Histogram packet** (sensor → host over USB bulk IF 1; parsed by
`omotion/MotionProcessing.py:parse_histogram_packet_structured`):

```
┌──────┬──────┬───────┬─[ optional ]─┬─[ per-camera block × N ]──────────────────────┬──────┬──────┐
│ 0xAA │ type │ len:4 │ timestamp:4  │ 0xFF │ cam │ histogram:4096 │ temp:4 │ 0xEE  │ crc:2│ 0xDD │
└──────┴──────┴───────┴──────────────┴──────┴──────┴────────────────┴────────┴───────┴──────┴──────┘
  SOF                                  SOH                                       EOH   CRC    EOF
```

- `type` — `TYPE_HISTO` (uncompressed) or `TYPE_HISTO_CMP` (RLE-compressed)
- `len` (LE u32) — total packet bytes
- `timestamp` (LE u32, ms; optional) — firmware TIM5 counter / 100, present
  when packet length matches `header + N × block + 4 + footer`. Wraps every
  ~42 949 s (~12 h); `parse_histogram_stream` unwraps it monotonically.
- Per-camera block (4103 B):
  - `cam` (1 B) — camera index 0–7
  - `histogram` (4096 B) — 1024 bins × LE u32; **last word's high byte is
    the 8-bit frame_id**, masked out before use
  - `temp` (LE u32) — camera temperature in °C × 100
- `crc` — CRC-CCITT (poly 0x1021, init 0xFFFF, `binascii.crc_hqx`)
- Compressed variant inserts a payload-CRC ahead of the transport CRC so the
  decompressor can verify it produced the right bytes:
  `[header][compressed:M][uncmp_crc:2][crc:2][0xDD]`
- Max packet size: **32 837 B** (~8 cameras + headers); typical scan: one
  camera per packet at 40 Hz.
- Validation: each parsed sample's `Σ bins` is compared against
  `EXPECTED_HISTOGRAM_SUM` (`2_457_606`) and dropped on mismatch.

### Transport layer

The console and sensor modules use entirely separate transport stacks. They share no base classes at this layer.

**Console transport — `MotionUart`** — communicates with the console over a USB virtual COM port using pyserial. Frames messages as `UartPacket` (start byte, ID, type, command, data, CRC-16, end byte). Supports sync mode (blocking read) and async mode (background read thread with per-ID response queues). Connection lifecycle is driven by `ConnectionMonitor`.

**Sensor transport — `USBInterfaceBase`** — base class that claims a USB bulk interface and locates its endpoints. `CommInterface` and `StreamInterface` both subclass it. Used exclusively by the sensor path.

**`CommInterface`** — bidirectional command/response over a sensor USB bulk interface (interface 0). Maintains a read thread and a contiguous `_read_buffer`. Supports two modes:

- **Sync mode** — `send_packet()` writes and then blocks until a complete response packet is found in the buffer.
- **Async mode** — a second thread parses packets from the buffer and routes them to per-packet-ID `queue.Queue` objects; `send_packet()` waits on the appropriate queue.

**`StreamInterface`** — input-only bulk streaming for sensor histogram and IMU data (interfaces 1 and 2). Reads fixed-size chunks (one histogram block = 4105 bytes) into a `queue.Queue`. No framing or CRC at this layer — the caller handles packet parsing.

### Device abstraction

**`MotionComposite`** — represents one physical sensor module. Owns three interface instances:

| Interface | Index | Class | Direction | Purpose |
|---|---|---|---|---|
| COMM | 0 | `CommInterface` | Full-duplex | Command / response |
| HISTO | 1 | `StreamInterface` | IN only | Histogram bulk stream |
| IMU | 2 | `StreamInterface` | IN only | IMU data stream |

Always creates `CommInterface` in async mode. Claims all three on `connect()`, releases all three on `disconnect()`.

**`MotionConsole`** — wraps `MotionUart` with the full console command set (ping, version, TEC, PDU monitor, I2C pass-through, LSYNC counter, FPGA programming commands, etc.). Creates a `ConsoleTelemetryPoller` at init time; the poller is started and stopped externally by `MotionInterface` in response to connection signals.

**`MotionSensor`** — one per side (`motion.left`, `motion.right`). Owns the connection state machine for that physical sensor module (`DISCONNECTED → CONNECTING → CONNECTED`). When CONNECTED, holds a `MotionComposite` on `self.uart` and exposes the sensor command set (FPGA control, camera enable / disable / config, histogram capture, IMU, firmware DFU). The handle itself is stable for the lifetime of the SDK — `_state` changes but `motion.left` is the same Python object across reconnects.

**`ConnectionMonitor`** — single daemon thread, owned by `MotionInterface`. Watches OS hotplug events (`WM_DEVICECHANGE` on Windows, libusb hotplug on Linux/macOS) for sub-50 ms detection, plus a 200 ms poll sweep as a fallback. Drives all three handles' state machines off a single event queue so they can't fight each other over a shared resource (the USB bus). USB-port topology is what assigns the two sensors to `left` vs `right` (`port_numbers[-1] == 2` → left, `== 3` → right; see `connection_monitor.py`).

**`MotionInterface`** — top-level entry point. Constructs the three handles (`console`, `left`, `right`) and the `ConnectionMonitor`, and composes the `ScanWorkflow`. The handles are **stable across reconnects** — apps subscribe to each handle's `signal_state_changed` once and cache the reference for the SDK's lifetime. Also starts / stops the console telemetry poller when the console handle enters / leaves `CONNECTED`.

### Data acquisition

**`MotionProcessing`** — stateless parsing primitives shared across the SDK. The histogram parsing helpers here feed the pipeline's `LiveUsbSource`; they no longer carry the per-scan science state.

| Class / Function | Purpose |
|---|---|
| `parse_histogram_packet()` / `parse_histogram_stream()` | Extract `HistogramSample`s from raw USB bulk bytes; handle multi-camera packets and the `DEBUG_FLAG_HISTO_CMP` decompression path |
| `bytes_to_integers()` | Converts 4096 histogram bytes to 1024 int bins + hidden figures |
| `EXPECTED_HISTOGRAM_SUM`, `HISTOGRAM_BYTES` | Validation and framing constants used by `LiveUsbSource` and the firmware-side packet writer |

**`omotion/pipeline/`** — the stage-based science pipeline package. Pure transformation over a typed `FrameBatch`; sinks subscribe to named channels for output. The default chain is built by `default_pipeline()`. Full reference: [`SciencePipeline.md`](SciencePipeline.md).

| Module | Purpose |
|---|---|
| `pipeline/batch.py` | `FrameBatch` dataclass — the typed data carrier; `BatchEvent` family (`LiveEmit`, `IntervalClosed`, `DarkIntegrityWarning`, `StencilFallback`, `TelemetryEvent`) |
| `pipeline/pipeline.py` | `Stage` protocol; `Pipeline` (ordered stage list + `reset()` + `on_scan_stop()` lifecycle) |
| `pipeline/runner.py` | `ScanRunner` — iterates a `Source`, runs the `Pipeline`, dispatches events to subscribed sinks, manages the parallel telemetry thread |
| `pipeline/sources.py` | `Source` protocol; `LiveUsbSource`, `CsvReplaySource`, `DbReplaySource`, `ConsoleTelemetrySource`; `_BaseSource` timestamp normalisation |
| `pipeline/sinks.py` | `Sink` protocol; `ScanMetadata`; built-in `CsvSink`, `ScanDBSink`, `TelemetrySink`. The live-plot UI sinks live in the bloodflow-app (`_LivePlotSink` + `_FinalBatchSink` in `motion_connector.py`), not here. |
| `pipeline/tee.py` | `Tee(channel)` — positional marker that emits `LiveEmit` for sinks subscribed to the named channel; supports `filter` and `max_duration_s` |
| `pipeline/factory.py` | `default_pipeline()` — composes the canonical 8-stage + 2-tee chain |
| `pipeline/pedestal.py` | `SensorPedestals` per-side, firmware-version-keyed pedestal lookup (replaces the legacy global `PEDESTAL_HEIGHT`) |
| `pipeline/telemetry.py` | `TelemetryAggregator` (thread-safe ring buffer) + `TelemetryIngestStage` (per-frame pdc/tcm/tcl attachment) |
| `pipeline/stages/classify.py` | `FrameClassificationStage` — frame-ID unwrap + `warmup`/`dark`/`light`/`stale` labelling |
| `pipeline/stages/noise_floor.py` | `NoiseFloorStage` — zeroes bins below threshold |
| `pipeline/stages/moments.py` | `MomentsStage` — vectorised μ₁, σ over raw histograms |
| `pipeline/stages/pedestal_sub.py` | `PedestalSubtractionStage` — `display_mean = max(0, mean_raw − pedestal)` |
| `pipeline/stages/dark.py` | `DarkCorrectionStage` + `HybridRealtimePredictor` + `LinearInterpolation` + `DarkFrameQuadraticStencil`; dual-output (realtime per-frame and batched per-interval) |
| `pipeline/stages/shot_noise.py` | `ShotNoiseCorrectionStage` — Poisson-variance subtraction on the realtime path |
| `pipeline/stages/bfi_bvi.py` | `BfiBviStage` — affine calibration map (contrast, mean) → (BFI, BVI) |
| `pipeline/stages/dark_frame_hold.py` | `DarkFrameHoldStage` — hold last light BFI/BVI across dark frames |
| `pipeline/stages/side_avg.py` | `LiveSideAverageStage` — realtime per-side spatial average (reduced mode) → `live_side` |
| `pipeline/stages/corrected_side_avg.py` | `CorrectedSideAverageStage` — dark-corrected per-side average (reduced mode) → `final_side` |

**`ScanWorkflow`** — orchestrates a complete acquisition:
1. Build a `ScanMetadata` and `SensorPedestals` from the connected sensors.
2. Construct `default_pipeline(metadata, calibration, pedestals, …)`.
3. Construct sinks (`CsvSink` by default; `ScanDBSink` if `db_path` is set; app-injected sinks for live UI, calibration, contact quality).
4. Construct a `LiveUsbSource` and (if any sink subscribes to `"telemetry"`) a `ConsoleTelemetrySource`.
5. Wrap them in a `ScanRunner` and spawn a worker thread that calls `runner.run()`.
6. The runner streams `FrameBatch`es through the pipeline, dispatches events to sinks on the appropriate channels, and runs `Pipeline.on_scan_stop()` at the end for the terminal-dark flush.
7. On cancellation or completion, the runner calls `source.close()` and `sink.on_complete()` in a `finally` block.

**`ScanDBSink` / `ScanDatabase`** — optional SQLite endpoint for both raw histogram blobs and final corrected output. Off by default; enabled by constructing `MotionInterface(db_path=...)`. When enabled, every scan opens a row in a `sessions` table; the sink subscribes to `"raw"` and `"final"` channels and writes `session_raw` blobs / `session_data` corrected rows accordingly. See [`ScanDatabase.md`](ScanDatabase.md) for schema, lifecycle, and how to query.

---

## Science pipeline

The full algorithm reference — every stage, every formula, every fallback — lives in [`SciencePipeline.md`](SciencePipeline.md). The summary below is just enough orientation for an architectural reader.

The pipeline is a list of `Stage`s driven by a `ScanRunner` that pulls `FrameBatch`es from a `Source` and dispatches stage-produced events to subscribed `Sink`s on named channels. Every stage mutates the `FrameBatch` in place; no stage does I/O. The default chain is built by `omotion.pipeline.factory.default_pipeline()`:

```
FrameClassificationStage    — abs frame ID unwrap; warmup/dark/light/stale label
TelemetryIngestStage        — attach per-frame pdc / tcm / tcl from console
Tee("raw")                  — full FrameBatch (incl. warmup) to "raw" sinks
NoiseFloorStage             — zero histogram bins below threshold (default 10)
MomentsStage                — vectorised μ₁, σ over raw histograms
PedestalSubtractionStage    — display_mean = max(0, mean_raw − per-side pedestal)
DarkCorrectionStage         — dual-output: realtime (predicted) + batched (interpolated);
                              emits IntervalClosed(EnrichedCorrectedInterval) when an
                              interval closes
ShotNoiseCorrectionStage    — Poisson variance subtraction on the realtime path
BfiBviStage                 — affine calibration (contrast, mean) → (BFI, BVI)
DarkFrameHoldStage          — hold last light BFI/BVI across dark frames
LiveSideAverageStage        — realtime per-side spatial average (reduced) → "live_side"
CorrectedSideAverageStage   — dark-corrected per-side average (reduced) → "final_side"
Tee("live")                 — corrected per-frame FrameBatch to "live" sinks
```

### Channels

Sinks declare which channels they consume (`channels: set[str]`):

| Channel | Payload | Cadence | Typical consumers |
|---|---|---|---|
| `raw` | `FrameBatch` (incl. warmup) | per batch | `CsvSink`, `ScanDBSink` |
| `live` | `FrameBatch` (excl. warmup/stale) | per batch | app live-plot sink (realtime points, later overwritten via `"final"`), `ContactQualityWorkflow` sink, `CalibrationWorkflow` sink (dark frames) |
| `final` | `EnrichedCorrectedInterval` | per closed dark interval (~15 s at defaults) | `CsvSink` (corrected CSV), `ScanDBSink` (`session_data`), app final-batch sink (overwrites realtime points with interval-corrected values), `CalibrationWorkflow` sink |
| `telemetry` | `TelemetryEvent` | ~10 Hz (separate thread) | `TelemetrySink` |
| `diagnostics` | `DarkIntegrityWarning`, `StencilFallback`, etc. | as they occur | any opt-in sink |

### Dark correction — two paths in one stage

`DarkCorrectionStage` runs **two corrections in parallel** from one shared `DarkHistory`:

- **Realtime (per light frame, predicted).** `HybridRealtimePredictor` produces a baseline `(û₁, σ̂)` from the last few darks — average of last 3 μ₁ values, linear extrapolation of σ across the two most recent darks, ZOH fallback during warmup. Available immediately; populates `dark_baseline_rt`, `mean_dc_rt`, `std_dc_rt` for `ShotNoiseCorrectionStage` and `BfiBviStage`. This feeds the live UI.
- **Batched (per closed dark interval, interpolated).** All non-dark frames between two bounding darks are buffered in a `PendingInterval`. When the closing dark arrives, `LinearInterpolation` linearly interpolates the dark baseline across the interval and emits an `IntervalClosed(EnrichedCorrectedInterval)` event with fully dark-subtracted, shot-noise-corrected, BFI/BVI-calibrated frames. The leading dark `D_prev` is included with values computed via a 4-point quadratic stencil over its neighbours. This feeds the corrected CSV / scan DB.

`on_scan_stop` performs a terminal-dark flush so any scan that reached at least the first scheduled dark + one light frame produces corrected output, even if it stopped before the second scheduled dark.

### Calibration

BFI and BVI are computed via per-camera min/max constants `(C_min, C_max, I_min, I_max)`, each a `(2, 8)` array indexed by `[side_idx][cam_id]`:

```
BFI = (1 − (K − C_min) / (C_max − C_min)) × 10
BVI = (1 − (μ₁ − I_min) / (I_max − I_min)) × 10
```

Fallback (zero span): identity scaling, `BFI = K × 10`, `BVI = μ₁ × 10`. Calibration is loaded from the console EEPROM at scan start or computed by `CalibrationWorkflow`.

### Configuration

**`MotionConfig`** — persistent device configuration stored as a 16-byte binary header (`magic`, `version`, `seq`, `crc`, `json_len`) followed by a JSON payload. Used to store and retrieve per-device parameters.

### Firmware programming

| Module | Mechanism |
|---|---|
| `DFUProgrammer` | Spawns `dfu-util` subprocess; parses progress output; reports phase and percent via callback |
| `FPGAProgrammer` | Page-by-page FPGA flash over the console UART; erases, writes CFG pages in 32-page batches, optionally verifies, writes feature row, refreshes |
| `jedecParser` | Parses JEDEC ASCII fuse files into `JedecImage` (rows × 16 bytes) + extra feature row data |
| `GitHubReleases` | GitHub Releases API client; lists, fetches, and downloads release assets |

---

## Threading model

| Thread | Owner | Daemon | Lifecycle | Purpose |
|---|---|---|---|---|
| `CommInterface.read_thread` | `CommInterface` | Yes | `claim()` → `release()` | USB bulk read into `_read_buffer` |
| `CommInterface.response_thread` | `CommInterface` | Yes | async mode only | Parse packets from buffer, route to response queues |
| `MotionUart.read_thread` | `MotionUart` | Yes | `connect()` → `disconnect()` | Serial read, parse packets or queue by ID |
| `StreamInterface.thread` | `StreamInterface` | Yes | `start_streaming()` → `stop_streaming()` | Fixed-size USB reads into data queue |
| `ConsoleTelemetryPoller._thread` | `ConsoleTelemetryPoller` | Yes | `start()` → `stop()` | ~1 Hz console health polls |
| `ScanWorkflow._thread` | `ScanWorkflow` | No | `start_scan()` → completion | Runs `ScanRunner.run()` — iterates the `Source`, drives the `Pipeline`, dispatches to sinks |
| `ScanWorkflow._config_thread` | `ScanWorkflow` | No | `start_configure_camera_sensors()` → completion | Camera configuration |
| `LiveUsbSource-{left,right}` | `LiveUsbSource` | Yes | per scan | Per-side packet parsing → `FrameBatch` → shared batch queue |
| `ScanRunner-telemetry` | `ScanRunner` | Yes | per scan, if telemetry source present | Poll `ConsoleTelemetrySource`, update `TelemetryAggregator`, dispatch to `"telemetry"` sinks |

**Synchronisation primitives in use:**

| Primitive | Location | Protects |
|---|---|---|
| `threading.RLock` | `CommInterface._io_lock` | USB write/read operations |
| `threading.RLock` | `MotionUart._io_lock` | Serial write/read + alignment padding |
| `threading.Lock` | `CommInterface._buffer_lock` | `_read_buffer` |
| `threading.Condition` | `CommInterface._buffer_condition` | Wait for data in async mode |
| `threading.Lock` | `MotionUart.response_lock` | `response_queues` dict |
| `threading.Lock` | `ConsoleTelemetryPoller._lock` | `_snapshot`, `_listeners` |
| `threading.Event` | `ConsoleTelemetryPoller._wake` | Smart sleep interrupt |
| `threading.Event` | `ScanWorkflow._stop_evt` | Scan cancellation |
| `threading.Lock` | `ScanWorkflow._lock` | `_running` guard |

Listener callbacks in `ConsoleTelemetryPoller` are copied under the lock but invoked outside it, preventing deadlocks at the cost of snapshot staleness. They run on the poller thread and must be non-blocking.

---

## Signal and event flow

```
USB insert / serial port appears
         │
    OS hotplug (WM_DEVICECHANGE / libusb)  ──or──  200ms ConnectionMonitor poll
         │
         ▼
    ConnectionMonitor event queue
         │  HotplugWake / PollArrived
         ▼
    Handle state machine transitions  (MotionConsole / MotionSensor)
         │  DISCONNECTED → CONNECTING → CONNECTED
         │  ├─ open transport (MotionUart / MotionComposite)
         │  ├─ refresh cached IDs (HWID, camera UIDs, version)
         │  └─ start ConsoleTelemetryPoller (console only)
         │
         ▼  signal_state_changed(ConnectionState.CONNECTED)
    Application (QML connector / headless script)
```

The same flow operates in reverse on disconnect — `ConnectionMonitor` sees the
device leave, the handle transitions back to `DISCONNECTED`, and the
`signal_state_changed` callback fires once more. Apps subscribe to each
handle's `signal_state_changed` once and rely on the handle reference being
stable across reconnects; they never touch `MotionUart` or `MotionComposite`
directly.

---

## Error handling

The SDK uses a layered catch-log-reraise pattern: each layer catches hardware exceptions, logs them with the module-scoped logger, and re-raises so that the caller can decide what to do.

| Exception | Raised by | Meaning |
|---|---|---|
| `CommandError(RuntimeError)` | `MotionUart`, `MotionConsole`, `MotionSensor` | Device returned NAK, BAD_CRC, or OW_ERROR |
| `ValueError` | All packet parsers, device methods | CRC mismatch, invalid framing, device not connected, bad argument |
| `TypeError` | `MotionConsole.echo()` | Wrong argument type |
| `TimeoutError` | `CommInterface`, `MotionUart` | No response within timeout |
| `serial.SerialException` | `MotionUart` | Serial port failure |
| `usb.core.USBError` | `CommInterface`, `StreamInterface` | USB communication error |
| `JedecError` | `jedecParser` | JEDEC file format violation |
| `FpgaUpdateError` | `FPGAProgrammer` | FPGA programming sequence failure |
| `FileNotFoundError` | `DFUProgrammer`, `usb_backend` | Missing dfu-util binary or libusb DLL |

USB timeout errnos (110 on Linux, 10060 on Windows) are suppressed inside read loops and treated as normal idle conditions. Errno 32 (broken pipe) and 19/5 (IO/no-device) trigger disconnect callbacks.

---

## Logging

Every module creates its logger as:

```python
logger = logging.getLogger(f"{_log_root}.ModuleName" if _log_root else "ModuleName")
```

`_log_root` defaults to `"openmotion.sdk"` and is set once by the application via `set_log_root()`. If no handlers are configured on the root logger at import time, a console handler is added automatically. This allows applications to control the entire SDK log hierarchy through a single prefix.

---

## Demo mode

`MotionInterface(demo_mode=True)` (or `OPENMOTION_DEMO=1` in the environment)
short-circuits device discovery and connection: `ConnectionMonitor` is not
started, and the three handles report `CONNECTED` immediately. `MotionUart`
skips serial I/O; `MotionConsole` returns hardcoded mock values from
`tec_status()`, `get_version()`, etc.; `MotionSensor` command methods
short-circuit on `self.demo_mode` and return canned responses. This lets the
application UI be developed and tested without physical hardware.

---

## Key data types

| Type | Module | Description |
|---|---|---|
| `UartPacket` | `UartPacket` | Parsed or constructed UART frame |
| `MotionConfigHeader` / `MotionConfig` | `MotionConfig` | Binary header + JSON device configuration |
| `HistogramSample` | `MotionProcessing` | One camera's histogram for one frame (parser output, before batching) |
| `FrameBatch` | `pipeline/batch.py` | N frames worth of typed arrays (cam_ids, frame_ids, raw_histograms, temperature_c, timestamp_s, pdc/tcm/tcl, plus per-stage outputs); the data carrier through every stage |
| `BatchEvent` family | `pipeline/batch.py` | `LiveEmit`, `IntervalClosed`, `DarkIntegrityWarning`, `StencilFallback`, `TelemetryEvent` — appended to `batch.events`, dispatched by the runner |
| `ScanMetadata` | `pipeline/sinks.py` | Immutable per-scan handle handed to every sink at `on_scan_start` |
| `CorrectedFrame` / `CorrectedInterval` | `pipeline/stages/dark.py` | One light frame's dark-subtracted moments (no shot-noise yet) / a closed interval's worth of them |
| `EnrichedCorrectedFrame` / `EnrichedCorrectedInterval` | `pipeline/stages/dark.py` | Post-shot-noise, post-calibration corrected frame / interval — payload of the `"final"` channel |
| `TelemetryEvent` | `pipeline/batch.py` | One snapshot from `ConsoleTelemetrySource` — payload of the `"telemetry"` channel |
| `ConsoleTelemetry` | `ConsoleTelemetry` | One snapshot of all console health data |
| `PDUMon` | `Console` | 16-channel ADC raw counts and scaled voltages |
| `TelemetrySample` | `Console` | Timestamped temperature + TEC ADC snapshot |
| `ScanRequest` / `ScanResult` | `ScanWorkflow` | Scan parameters and outcome |
| `JedecImage` | `jedecParser` | Parsed FPGA bitstream rows |
| `DFUProgress` / `DFUResult` | `DFUProgrammer` | Firmware flash progress and result |
