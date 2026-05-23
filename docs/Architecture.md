# OpenMotion SDK — Software Architecture

## Overview

The OpenMotion SDK is a Python library for controlling optical speckle imaging hardware. It manages two classes of physical device: a **console module** (connected via USB virtual COM port / UART) and up to two **sensor modules** (connected via composite USB bulk-transfer interfaces). The SDK handles device discovery, connection lifecycle, command/response communication, high-speed histogram streaming, science computation, and firmware programming. A thin signal abstraction makes the same SDK usable in both PyQt6 desktop applications and headless Python scripts.

---

## Layer diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Application / QML UI                         │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ signals / callbacks
┌──────────────────────────────▼──────────────────────────────────────┐
│                         MOTIONInterface                              │
│          console_module  ·  sensors  ·  scan_workflow               │
└──────────────┬──────────────────────────────┬───────────────────────┘
               │                              │
               │  Console path                │  Sensor path
               │  (USB VCP / pyserial)        │  (USB bulk transfer)
               │                              │
┌──────────────▼──────────────┐  ┌────────────▼──────────────────────┐
│       MOTIONConsole         │  │       DualMotionComposite          │
│  + ConsoleTelemetryPoller   │  │   (left + right MotionComposite)   │
└──────────────┬──────────────┘  └────────────┬──────────────────────┘
               │                              │
┌──────────────▼──────────────┐  ┌────────────▼──────────────────────┐
│         MOTIONUart          │  │    MotionComposite  (per side)     │
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
   │  DbReplaySource              │  → Tee(raw) → NoiseFloor     │  QtUiSink   (live)
   │                              │  → Moments → PedestalSub     │  TelemetrySink (telemetry)
   │  ConsoleTelemetrySource      │  → DarkCorrection → ShotNoise│  CalibrationWorkflow sinks
   │  (separate thread)           │  → BfiBvi → SideAvg          │  ContactQuality sink
   │                              │  → Tee(live) → RollingAvg    │  + your own
   │                              │  → Tee(rolling)              │
   └──────────────────────────────┴──────────────────────────────┘
                  │                                          ▲
                  │  FrameBatch (mutated in place)            │
                  │  batch.events: LiveEmit / IntervalClosed  │
                  └─── channels: raw, live, rolling, final, ──┘
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
| `MotionSignal.py` | `MOTIONSignal` — lightweight signal with `.connect()` / `.disconnect()` / `.emit()` |
| `signal_wrapper.py` | `SignalWrapper` base class — uses real `pyqtSignal` if PyQt6 is present, falls back to `MOTIONSignal` |

Every class that exposes device events inherits `SignalWrapper` and uses the three standard signals: `signal_connect(str, str)`, `signal_disconnect(str, str)`, `signal_data_received(str, str)`.

### Packet structures

| Module | Wire format | CRC |
|---|---|---|
| `UartPacket.py` | `[0xAA][id:2][type][cmd][addr][rsv][len:2][data:N][crc:2][0xDD]` | CRC-16 lookup table |
| `i2c_packet.py` | `<HBHBH` (little-endian) | CRC-16-CCITT-FALSE (crcmod) |
| `i2c_data_packet.py` | `<BHBBB` + payload | CRC-16-CCITT-FALSE |
| `i2c_status_packet.py` | `<HBBBBH` | CRC-16-CCITT-FALSE |

All packet types validate CRC on receive and raise `ValueError` on mismatch.

### Transport layer

The console and sensor modules use entirely separate transport stacks. They share no base classes at this layer.

**Console transport — `MOTIONUart`** — communicates with the console over a USB virtual COM port using pyserial. Frames messages as `UartPacket` (start byte, ID, type, command, data, CRC-16, end byte). Supports sync mode (blocking read) and async mode (background read thread with per-ID response queues). Emits `signal_connect` / `signal_disconnect` on port insertion/removal.

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

**`DualMotionComposite`** — scans USB for devices matching the sensor PID and assigns them to left/right slots based on USB port topology (`port_numbers[-1] == 2` → left, `== 3` → right). Manages a `monitor_usb_status()` async coroutine that auto-connects arriving devices and auto-disconnects departing ones.

**`MOTIONConsole`** — wraps `MOTIONUart` with the full console command set (ping, version, TEC, PDU monitor, I2C pass-through, LSYNC counter, FPGA programming commands, etc.). Creates a `ConsoleTelemetryPoller` at init time; the poller is started and stopped externally by `MOTIONInterface` in response to connection signals.

**`MOTIONSensor`** — wraps `MotionComposite` with the full sensor command set (FPGA control, camera enable/disable/config, histogram capture, IMU, firmware DFU). Provides `stream_histograms_to_queue()` and `stream_histograms_to_csv()` for data acquisition.

**`MOTIONInterface`** — top-level entry point. Composes console, dual-composite, and scan workflow. Intercepts raw USB signals from the transport layer and re-emits them as named events (`"CONSOLE"`, `"SENSOR_LEFT"`, `"SENSOR_RIGHT"`). Starts and stops the console telemetry poller in response to console connect/disconnect events.

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
| `pipeline/sinks.py` | `Sink` protocol; `ScanMetadata`; built-in `CsvSink`, `ScanDBSink`, `TelemetrySink`, `QtUiSink` |
| `pipeline/tee.py` | `Tee(channel)` — positional marker that emits `LiveEmit` for sinks subscribed to the named channel; supports `filter` and `max_duration_s` |
| `pipeline/factory.py` | `default_pipeline()` — composes the canonical 10-stage + 3-tee chain |
| `pipeline/pedestal.py` | `SensorPedestals` per-side, firmware-version-keyed pedestal lookup (replaces the legacy global `PEDESTAL_HEIGHT`) |
| `pipeline/telemetry.py` | `TelemetryAggregator` (thread-safe ring buffer) + `TelemetryIngestStage` (per-frame pdc/tcm/tcl attachment) |
| `pipeline/stages/classify.py` | `FrameClassificationStage` — frame-ID unwrap + `warmup`/`dark`/`light`/`stale` labelling |
| `pipeline/stages/noise_floor.py` | `NoiseFloorStage` — zeroes bins below threshold |
| `pipeline/stages/moments.py` | `MomentsStage` — vectorised μ₁, σ over raw histograms |
| `pipeline/stages/pedestal_sub.py` | `PedestalSubtractionStage` — `display_mean = max(0, mean_raw − pedestal)` |
| `pipeline/stages/dark.py` | `DarkCorrectionStage` + `HybridRealtimePredictor` + `LinearInterpolation` + `DarkFrameQuadraticStencil`; dual-output (realtime per-frame and batched per-interval) |
| `pipeline/stages/shot_noise.py` | `ShotNoiseCorrectionStage` — Poisson-variance subtraction on the realtime path |
| `pipeline/stages/bfi_bvi.py` | `BfiBviStage` — affine calibration map (contrast, mean) → (BFI, BVI) |
| `pipeline/stages/side_avg.py` | `SideAveragingStage` — per-side averaging for reduced-mode display |
| `pipeline/stages/rolling_avg.py` | `RollingAverageStage` — sliding-window mean of BFI/BVI |

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
SideAveragingStage          — per-side averaging (reduced mode only)
Tee("live")                 — corrected per-frame FrameBatch to "live" sinks
RollingAverageStage         — sliding-window mean (default 10 frames)
Tee("rolling")              — smoothed per-frame FrameBatch to "rolling" sinks
```

### Channels

Sinks declare which channels they consume (`channels: set[str]`):

| Channel | Payload | Cadence | Typical consumers |
|---|---|---|---|
| `raw` | `FrameBatch` (incl. warmup) | per batch | `CsvSink`, `ScanDBSink` |
| `live` | `FrameBatch` (excl. warmup/stale) | per batch | `QtUiSink`, `ContactQualityWorkflow` sink, `CalibrationWorkflow` sink (dark frames) |
| `rolling` | `FrameBatch` with smoothed BFI/BVI | per batch | smoothed-trace UI, test harnesses |
| `final` | `EnrichedCorrectedInterval` | per closed dark interval (~15 s at defaults) | `CsvSink` (corrected CSV), `ScanDBSink` (`session_data`), `CalibrationWorkflow` sink |
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
| `MOTIONUart.read_thread` | `MOTIONUart` | Yes | `connect()` → `disconnect()` | Serial read, parse packets or queue by ID |
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
| `threading.RLock` | `MOTIONUart._io_lock` | Serial write/read + alignment padding |
| `threading.Lock` | `CommInterface._buffer_lock` | `_read_buffer` |
| `threading.Condition` | `CommInterface._buffer_condition` | Wait for data in async mode |
| `threading.Lock` | `MOTIONUart.response_lock` | `response_queues` dict |
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
    MOTIONUart / DualMotionComposite
         │  signal_connect("CONSOLE" | "SENSOR_LEFT" | "SENSOR_RIGHT", ...)
         ▼
    MOTIONInterface._on_console_connect / _on_sensor_connect
         │  ├─ start ConsoleTelemetryPoller (console only)
         │  └─ instantiate MOTIONSensor (sensors only)
         │  signal_connect(forwarded)
         ▼
    Application (MOTIONConnector / QML)
```

The same flow operates in reverse on disconnect. Applications register with `MOTIONInterface.signal_connect` / `signal_disconnect`; they never reference `MOTIONUart` or `DualMotionComposite` directly.

---

## Error handling

The SDK uses a layered catch-log-reraise pattern: each layer catches hardware exceptions, logs them with the module-scoped logger, and re-raises so that the caller can decide what to do.

| Exception | Raised by | Meaning |
|---|---|---|
| `CommandError(RuntimeError)` | `MOTIONUart`, `MOTIONConsole`, `MOTIONSensor` | Device returned NAK, BAD_CRC, or OW_ERROR |
| `ValueError` | All packet parsers, device methods | CRC mismatch, invalid framing, device not connected, bad argument |
| `TypeError` | `MOTIONConsole.echo()` | Wrong argument type |
| `TimeoutError` | `CommInterface`, `MOTIONUart` | No response within timeout |
| `serial.SerialException` | `MOTIONUart` | Serial port failure |
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

`MOTIONUart`, `MotionComposite`, and `DualMotionComposite` all accept a `demo_mode` flag. When set, `MOTIONUart` skips serial I/O and emits a synthetic connect signal immediately. `MOTIONConsole` returns hardcoded mock values from `tec_status()`, `get_version()`, etc. This allows the application UI to be developed and tested without physical hardware.

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
