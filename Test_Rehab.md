# SDK Test Rehab

Notes for rehabilitating the openmotion-sdk test suite.

## SDK Subsystems

### 1. Facade / Public API
- **`MotionInterface.py`** — The single front door. Device discovery, connect/disconnect, start/stop scans, configuration.
- **`MotionSignal.py`** / **`signal_wrapper.py`** — pyqtSignal abstraction (falls back to `MotionSignal` when PyQt is absent).

### 2. Console Device (UART)
- **`MotionConsole.py`** — UART-side device driver (~2900 lines). Trigger control, TEC, fan, FPGA programming, telemetry, odometer, debug flags. The single biggest file in the repo.
- **`MotionUart.py`** — UART framing layer. CRC-16 packet encode/decode.
- **`UartPacket.py`** — Packet dataclass for the UART wire protocol.

### 3. Sensor Device (USB)
- **`MotionSensor.py`** — USB-side device driver. Camera configuration, histogram streaming, IMU, DFU trigger.
- **`MotionComposite.py`** — Wraps a sensor's three USB interfaces into one object (comm + histo stream + IMU stream).
- **`CommInterface.py`** — USB bulk command/response on sensor interface 0.
- **`StreamInterface.py`** — USB bulk streaming on sensor interfaces 1 (histo) and 2 (IMU). Daemon reader thread per endpoint.
- **`USBInterfaceBase.py`** — Shared USB interface base class.
- **`usb_backend.py`** — libusb backend selection.

### 4. Connection / Hotplug
- **`connection_monitor.py`** — Daemon thread watching for device connect/disconnect events.
- **`connection_state.py`** — Connection state machine.
- **`hotplug/`** — Platform-specific hotplug backends:
  - `win32.py` — Windows WMI-based hotplug.
  - `libusb_hotplug.py` — libusb hotplug (Linux/macOS).
  - `poll_only.py` — Polling fallback.

### 5. Science Pipeline (`omotion/pipeline/`)
The stage-based BFI/BVI computation engine.

- **Core framework:**
  - `pipeline.py` — `Pipeline` class and `Stage` protocol.
  - `runner.py` — `ScanRunner` — channel-based dispatch with sink isolation.
  - `factory.py` — `default_pipeline()` factory assembles the canonical stage chain.
  - `batch.py` — `FrameBatch` typed data carrier.
  - `tee.py` — `Tee` stage for channel-based emission (raw, rolling, live).

- **Stages** (`pipeline/stages/`):
  - `noise_floor.py` — Bin-count threshold zeroing.
  - `pedestal_sub.py` — Per-side FW-version-keyed pedestal subtraction.
  - `moments.py` — Einsum-vectorized histogram moments (mean, variance).
  - `shot_noise.py` — Poisson variance subtraction.
  - `classify.py` — Frame classification (warmup / stale / dark / light).
  - `dark.py` — Dark correction orchestrator (DarkHistory, DarkIntegrityGuard, estimators, LinearInterpolation).
  - `dark_frame_hold.py` — Dark frame hold/release logic.
  - `bfi_bvi.py` — Affine calibration map for BFI/BVI.
  - `side_avg.py` — Rolling-window smoothing for the rolling tee.
  - `corrected_side_avg.py` — Per-side averaging for reduced-mode display.

- **Sources** (`pipeline/sources.py`):
  - `LiveUsbSource` — Real-time USB histogram stream.
  - `CsvReplaySource` — Replay from raw CSV files.
  - `DbReplaySource` — Replay from scan database.

- **Sinks** (`pipeline/sinks.py`):
  - `CsvSink` — Writes raw + corrected CSV files.
  - `ScanDBSink` — Writes to SQLite scan database.
  - `QtUiSink` — Stub for live UI channel.

- **Pedestal data** (`pipeline/pedestal.py`):
  - Per-side, per-FW-version pedestal height lookup tables.

### 6. Scan Workflow
- **`ScanWorkflow.py`** — Full acquisition orchestration. Hardware bring-up, trigger config, frame streaming into the pipeline, teardown. The bridge between device layer and pipeline.

### 7. Calibration
- **`CalibrationWorkflow.py`** — Per-camera gain/I_max calibration procedure (~1600 lines).
- **`Calibration.py`** — Calibration data structures / utilities.

### 8. Contact Quality
- **`ContactQualityWorkflow.py`** — SDK-owned contact quality check procedure. Short scan, evaluates mean/contrast thresholds per camera.

### 9. Telemetry
- **`ConsoleTelemetry.py`** — PDC + TEC poller daemon thread. Snapshots for per-frame dark correction.
- **`console_telemetry_conversions.py`** — Raw ADC to physical units (temperature, current, voltage).

### 10. Laser
- **`laser.py`** — Laser power configuration. SDK-owned setup so scans work without the app.

### 11. Programming / Firmware Update
- **`FPGAProgrammer.py`** — Page-by-page Lattice XO2 flash over I2C.
- **`DFUProgrammer.py`** — STM32 DFU over USB (uses vendored `dfu-util`).
- **`jedecParser.py`** — JEDEC file parser for FPGA bitstreams.

### 12. Storage / Playback
- **`ScanDatabase.py`** — SQLite scan database schema and access.
- **`SessionPlayback.py`** — Replay recorded sessions from the DB.

### 13. Histogram Parsing (Legacy Shim)
- **`MotionProcessing.py`** — Wire-level histogram packet parsing. Thin shim feeding the pipeline. Slated to dissolve eventually.

### 14. I2C Passthrough
- **`i2c_packet.py`**, **`i2c_data_packet.py`**, **`i2c_status_packet.py`** — I2C passthrough packet structures (used by FPGA programming path).

### 15. Configuration
- **`config.py`** — VID/PID, baud rate, packet types, command opcodes, debug flag bits. Single source of truth for protocol constants.
- **`MotionConfig.py`** — Higher-level configuration helpers.

### 16. Utilities
- **`utils.py`** — Shared utility functions.
- **`CommandError.py`** — Exception types for command failures.
- **`GitHubReleases.py`** — GitHub release API client (used for FPGA bitstream downloads).

---

## Existing Test Coverage

| Test file | Subsystem | HW required? |
|---|---|---|
| `test_motion_interface.py` | Facade | Yes |
| `test_console.py` | Console | Yes |
| `test_console_telemetry_unit.py` | Telemetry | No |
| `test_sensor.py` | Sensor | Yes |
| `test_comm_paths.py` | Transport | Yes |
| `test_comm_transport_down.py` | Transport | Yes |
| `test_errors.py` | Error handling | ? |
| `test_laser.py` | Laser | Yes |
| `test_get_pdc_buffer.py` | Telemetry/Console | Yes |
| `test_pedestal_height.py` | Pipeline/Pedestal | No |
| `test_motion_processing_shim.py` | Parsing shim | No |
| `test_scan_workflow.py` | Scan workflow | Yes |
| `test_scan_database.py` | Storage | No |
| `test_calibration.py` | Calibration | Yes |
| `test_calibration_console.py` | Calibration | Yes |
| `test_calibration_procedure.py` | Calibration | Yes |
| `test_calibration_workflow.py` | Calibration | Yes |
| `test_calibration_workflow_compute.py` | Calibration | No |
| `test_contact_quality_workflow.py` | Contact quality | No |
| `test_reduced_mode.py` | Reduced mode | Yes |
| `test_run_collection_scan.py` | Scan workflow | Yes |
| `test_sequences.py` | Sequences | Yes |
| `test_wheel_dfu.py` | DFU/packaging | No |
| `test_zz_dfu.py` | DFU | Yes |
| `test_pipeline/` (28 files) | Pipeline (all stages, sinks, sources, runner, factory) | No |

---

## Science Pipeline — Detailed Walkthrough

### Overview

The science pipeline transforms raw 1024-bin histogram frames from the camera sensors into Blood Flow Index (BFI) and Blood Volume Index (BVI) values. It operates on `FrameBatch` objects — numpy-backed bundles of N frames (typically 10–100 from USB) with shape `(N, 2_sides, 8_cameras, 1024_bins)`.

The pipeline has **two parallel output paths**:
- **Realtime path**: runs inline as each batch arrives. Populates batch fields (`mean_dc_rt`, `std_dc_rt`, `bfi_live`, `bvi_live`). Emits to `"live"` channel sinks for UI display.
- **Batch/final path**: accumulates frames between dark boundaries, then corrects the entire interval at once with linear interpolation. Emits `IntervalClosed` events to `"final"` channel sinks for storage (CSV, DB).

### Full Stage Chain (as assembled by `default_pipeline()`)

```
                     FrameBatch from Source
                            │
                  ┌─────────▼──────────┐
                  │ 1. FrameClassify   │  Unwrap 8-bit frame IDs → monotonic abs_id
                  │                    │  Label: warmup / stale / dark / light
                  └─────────┬──────────┘
                            │
                  ┌─────────▼──────────┐
                  │ 2. Tee("raw")      │──────────► "raw" channel sinks (CsvSink raw CSV)
                  │    filter: !stale  │            ⚠ DATA DISCARDED: stale frames never
                  │    max_duration_s  │              reach raw sinks
                  └─────────┬──────────┘            ⚠ DATA DISCARDED: if max_duration_s
                            │                         exceeded, no more raw emission
                  ┌─────────▼──────────┐
                  │ 3. NoiseFloor      │  Zero bins with count < threshold
                  │                    │  ⚠ DESTRUCTIVE: mutates raw_histograms in place
                  └─────────┬──────────┘
                            │
                  ┌─────────▼──────────┐
                  │ 4. Moments         │  Compute mean (u1), variance, std from histogram
                  │                    │  Sets: mean_raw, std_raw
                  │                    │  contrast_raw intentionally left None
                  └─────────┬──────────┘
                            │
                  ┌─────────▼──────────┐
                  │ 5. PedestalSub     │  display_mean = max(0, mean_raw - pedestal)
                  │                    │  ⚠ DATA CLAMPED: negative values → 0.0
                  └─────────┬──────────┘
                            │
                  ┌─────────▼──────────┐
                  │ 6. DarkCorrection  │  DUAL OUTPUT:
                  │                    │
                  │  Realtime:         │  Predict dark baseline from history → subtract
                  │    → mean_dc_rt    │  Populate: dark_baseline_rt, mean_dc_rt, std_dc_rt
                  │    → std_dc_rt     │  NaN where no dark observed yet (warmup)
                  │                    │
                  │  Batch:            │  Buffer lights between dark boundaries
                  │    → IntervalClosed│  On interval close: linear-interpolate dark baseline
                  │      events        │  + shot-noise + BFI/BVI enrichment per frame
                  │                    │  + quadratic stencil for dark frame value
                  └─────────┬──────────┘
                            │
                  ┌─────────▼──────────┐
                  │ 7. ShotNoise       │  Subtract Poisson shot-noise variance from std²
                  │   (realtime only)  │  Sets: std_sn_rt, contrast_sn_rt
                  └─────────┬──────────┘
                            │
                  ┌─────────▼──────────┐
                  │ 8. BfiBvi          │  Affine calibration: contrast → BFI, mean → BVI
                  │   (realtime only)  │  Sets: bfi_live, bvi_live
                  └─────────┬──────────┘
                            │
                  ┌─────────▼──────────┐
                  │ 9. DarkFrameHold   │  During dark frames, hold last light's BFI/BVI
                  │                    │  Prevents display spikes during laser-off intervals
                  │                    │  ⚠ DATA REPLACED: dark frame bfi_live/bvi_live
                  │                    │    overwritten with last light frame values
                  └─────────┬──────────┘
                            │
                  ┌─────────▼──────────┐
                  │10. LiveSideAvg     │  (reduced mode only)
                  │                    │  Spatial average across selected cameras per side
                  │                    │  Emits: LiveEmit("live_side", SideAverageSample)
                  └─────────┬──────────┘
                            │
                  ┌─────────▼──────────┐
                  │11. CorrectedSideAvg│  (reduced mode only)
                  │                    │  Reads IntervalClosed events from DarkCorrection
                  │                    │  Groups per-camera corrected values by frame_id
                  │                    │  Emits: LiveEmit("final_side", SideAverageSample)
                  └─────────┬──────────┘
                            │
                  ┌─────────▼──────────┐
                  │12. Tee("live")     │──────────► "live" channel sinks (UI, ScanDBSink)
                  │    filter: !warmup │            ⚠ DATA DISCARDED: warmup + stale frames
                  │            !stale  │              never reach live sinks
                  └────────────────────┘
```

### Channel Routing (ScanRunner)

```
  batch.events after pipeline.process()
       │
       ├── LiveEmit(channel="raw")     ──► sinks with "raw" in channels     (CsvSink raw path)
       ├── LiveEmit(channel="live")    ──► sinks with "live" in channels    (ScanDBSink, QtUiSink)
       ├── LiveEmit(channel="live_side") ► sinks with "live_side"           (reduced mode UI)
       ├── LiveEmit(channel="final_side")► sinks with "final_side"          (ScanDBSink reduced)
       ├── IntervalClosed              ──► sinks with "final" in channels   (CsvSink corrected, ScanDBSink)
       └── DarkIntegrityWarning, etc.  ──► sinks with "diagnostics"        (logging, notes)
```

### Stage-by-Stage Detail

#### Stage 1: FrameClassificationStage (`classify.py`)

**Purpose:** Unwrap the camera's 8-bit rolling frame counter into a monotonic absolute ID, then classify each frame.

**Frame ID Unwrapping:**
- Cameras emit frame IDs as `uint8` (0–255), rolling over at 256.
- The `_FrameUnwrapper` tracks per-(side, cam_id) state: `epoch` counter, `last_raw` ID.
- On rollover detection (delta ≤ 128 but raw < last): increments epoch.
- `abs_id = epoch * 256 + raw_frame_id`

**Classification rules (in priority order):**
1. **"stale"** — The very first frame from a camera arrived with `raw_frame_id != 1`. This is a leftover from a prior scan still in the USB buffer. Stale for that one frame only.
2. **"warmup"** — `abs_id <= discard_count`. Camera is still stabilizing.
3. **"dark"** — Frame where laser is off (baseline measurement). Dark if `abs_id == discard_count + 1` (first post-warmup frame is always dark) OR `(abs_id - 1) % dark_interval == 0`.
4. **"light"** — Everything else. Laser is on, real measurement.

**Magic numbers:**
- `_FRAME_ID_MODULUS = 256` — 8-bit counter wraps at 256
- `_FRAME_ROLLOVER_THRESHOLD = 128` — delta ≤ 128 is a forward step; > 128 is backwards (rollover)
- `discard_count = 9` (default) — first 9 frames discarded as warmup
- `dark_interval = 600` (default) — one dark frame every 600 frames (= every 15 sec at 40 fps)

**Data discarded:** Stale and warmup frames propagate through the pipeline but are filtered out at Tee stages before reaching sinks. They are never written to CSV or DB.

---

#### Stage 2: Tee("raw") (`tee.py`)

**Purpose:** Snapshot the batch for raw data sinks (CsvSink's raw CSV path).

- **Filter:** `ft != "stale"` — stale frames are excluded from raw output.
- **max_duration_s:** When set, once `batch.timestamp_s[0]` exceeds this value, no more `LiveEmit("raw")` events are appended. Raw CSV writing stops.
- **Payload:** The entire `FrameBatch` reference (not a copy).

**Data discarded:**
- Stale frames: never emitted to raw sinks.
- All frames after `max_duration_s`: silently dropped from raw output.

---

#### Stage 3: NoiseFloorStage (`noise_floor.py`)

**Purpose:** Zero out histogram bins with counts below a threshold, removing detector noise.

**Operation:** `np.putmask(raw_histograms, raw_histograms < threshold, 0)`

**Magic numbers:**
- `threshold = 10` (default) — bins with fewer than 10 photon counts are zeroed

**Data discarded:** All sub-threshold bin counts are irreversibly set to 0. This is a destructive in-place mutation of `raw_histograms`. Since the Tee("raw") already captured the batch reference before this stage, the raw tee payload also sees the zeroed histograms (they share the same numpy array).

---

#### Stage 4: MomentsStage (`moments.py`)

**Purpose:** Compute the first moment (mean), second moment, variance, and standard deviation of each histogram.

**Operation:**
```
h = raw_histograms                          # (N, 2, 8, 1024)
counts = h.sum(axis=-1)                     # total photon counts
u1 = Σ(bin_value × bin_count) / counts      # first moment (mean)
u2 = Σ(bin_value² × bin_count) / counts     # second moment
var = max(0, u2 - u1²)                      # variance (clamped ≥ 0)
std = √var
```

Uses precomputed `_BIN_VALUES = [0, 1, 2, ..., 1023]` and `_BIN_VALUES_SQ = [0, 1, 4, ..., 1023²]` as `float64`.

**Outputs:** `mean_raw`, `std_raw` as `float32`. `contrast_raw` is explicitly set to `None` — contrast requires pedestal subtraction first.

**Edge case:** If `counts == 0` (empty histogram), uses `safe_counts = 1` to avoid division by zero, then sets `mean = NaN`.

**Magic numbers:**
- `1024` — histogram bin count (10-bit ADC range)

---

#### Stage 5: PedestalSubtractionStage (`pedestal_sub.py`)

**Purpose:** Subtract the sensor's dark-current pedestal from the raw mean to get the optical signal.

**Operation:** `display_mean = max(0.0, mean_raw - pedestal)` per side.

**Pedestal values** (from `pedestal.py`):
- FW version ≤ 1.5.2: pedestal = **64.0** DN
- FW version > 1.5.2: pedestal = **128.0** DN

**Data clamped:** Negative values after subtraction are clamped to 0.0. A camera seeing less light than the pedestal reads as zero signal, not negative.

**Magic numbers:**
- `64.0` — legacy sensor pedestal height (DN)
- `128.0` — current sensor pedestal height (DN)

---

#### Stage 6: DarkCorrectionStage (`dark.py`)

The most complex stage. Maintains extensive per-(side, cam) state across batches.

##### Realtime Path (inline, per batch)

For each **light** frame:
1. `HybridRealtimePredictor` predicts the current dark baseline:
   - **u1 prediction:** average of last 3 dark observations (or fewer if < 3 available). Zero-order hold.
   - **std prediction:** linear extrapolation through last 2 darks. Falls back to ZOH if < 2 darks or timestamps equal.
2. Subtract predicted baseline:
   - `mean_dc_rt = mean_raw - u1_predicted`
   - `std_dc_rt = √max(0, std_raw² - std_predicted²)`
3. If no darks observed yet for this camera → fields stay `NaN` (warmup window).

For each **dark** frame:
1. `DarkIntegrityGuard` checks if `u1 > pedestal + max_above_pedestal`. If so, logs a WARNING and emits `DarkIntegrityWarning`. **The frame is still used** — it is not dropped.
2. Dark observation appended to `DarkHistory` (ring buffer, default max 4 entries per camera).
3. Dark frame's realtime output uses the **last light frame's** realtime values (via `_last_realtime` cache), not its own meaningless laser-off measurement.

##### Batch Path (deferred, interval-based)

```
  Dark₁ ──── Light Light Light Light ──── Dark₂
  ◄──────── PendingInterval ────────────►
                                          │
                                    interval closes
                                          │
                              LinearInterpolation
                                          │
                                  CorrectedInterval
                                          │
                              _enrich (shot-noise + BFI/BVI)
                                          │
                              DarkFrameQuadraticStencil for Dark₁
                                          │
                              IntervalClosed event → "final" sinks
```

- `PendingInterval` buffers light frames between two bounding dark frames.
- When the right dark arrives, the interval closes:
  1. `LinearInterpolation.correct_interval()`:
     - For each light frame, interpolate the dark baseline linearly between left and right dark boundaries using **abs_frame_id fraction** (not timestamp).
     - `t_frac = (light_abs_id - left_dark_abs_id) / (right_dark_abs_id - left_dark_abs_id)`
     - `baseline_u1 = dark_left.u1 + t_frac × (dark_right.u1 - dark_left.u1)`
     - `baseline_var = dark_left.std² + t_frac × (dark_right.std² - dark_left.std²)`
     - `corrected_mean = light.u1 - baseline_u1`
     - `corrected_var = max(0, (light.u2 - light.u1²) - baseline_var)`
     - `corrected_std = √corrected_var`
  2. Enrichment (if calibration available):
     - Shot-noise subtraction per frame: `shot_var = adc_gain × max(0, corrected_mean) × camera_gain`
     - `final_std = √max(0, corrected_std² - shot_var)`
     - Contrast: `final_std / corrected_mean` (0 if mean ≤ 0)
     - BFI/BVI: same affine calibration as BfiBviStage
  3. `DarkFrameQuadraticStencil` for the left dark frame (D_prev):
     - 4-point stencil: `v(D) = (-1/6)v(D-2) + (2/3)v(D-1) + (2/3)v(D+1) + (-1/6)v(D+2)`
     - Neighbours: v(D-1), v(D-2) from previous interval's last 2 frames; v(D+1), v(D+2) from current interval's first 2 frames.
     - Fallback chain: full 4-point → right_only → simple_avg → repeat_right

##### Terminal Dark Flush (`on_scan_stop`)

When the scan ends, firmware guarantees a final dark frame. It may arrive as a "light" in the pipeline's classification. The stage:
1. Finds trailing dark-like frames at the end of each `PendingInterval._light` list (u1 ≤ pedestal + threshold).
2. Promotes the last one to a right dark boundary.
3. Removes the entire trailing dark-like tail from the light list.
4. Emits the final interval.

**Data discarded:** Terminal dark-like frames removed from the light list are not emitted as corrected light frames (they would have meaningless values).

**Magic numbers:**
- `realtime_history_size = 4` — dark history ring buffer keeps last 4 darks per camera
- `integrity_max_above_pedestal = 5.0` DN — dark frame u1 threshold for contamination warning
- `11_000` — electrons at full scale, used to compute ADC gain: `(1024 - pedestal) / 11_000`
- Stencil coefficients: `-1/6`, `2/3`, `2/3`, `-1/6`

---

#### Stage 7: ShotNoiseCorrectionStage (`shot_noise.py`)

**Purpose:** Remove Poisson (photon) shot noise from the realtime variance estimate.

**Operation:**
```
shot_var = adc_gain × max(0, mean_dc_rt) × camera_gain_map
corrected_var = max(0, std_dc_rt² - shot_var)
std_sn_rt = √corrected_var
contrast_sn_rt = std_sn_rt / mean_dc_rt     (0 if mean ≤ 0, NaN if mean is NaN)
```

**ADC gain:** `(1024 - pedestal) / 11_000` per side.

**Camera gain map** (from `config.py`):
```
CAMERA_GAIN_MAP = [16, 4, 2, 1, 1, 2, 4, 16]
```
Outer cameras (positions 0, 7) have 16× gain; inner cameras (3, 4) have 1×. Compensates for reduced illumination at array periphery.

**Data clamped:** Negative corrected variance is clamped to 0.

**Magic numbers:**
- `CAMERA_GAIN_MAP = [16, 4, 2, 1, 1, 2, 4, 16]` — per-camera analog gain compensation
- `11_000` — full-scale electron count
- `1024` — full-scale DN range (10-bit ADC)

---

#### Stage 8: BfiBviStage (`bfi_bvi.py`)

**Purpose:** Map (contrast, mean) into clinically meaningful (BFI, BVI) via per-camera affine calibration.

**Operation:**
```
BFI = (1 - (contrast - c_min) / (c_max - c_min)) × 10
BVI = (1 - (mean    - i_min) / (i_max - i_min)) × 10
```

- `c_min`, `c_max`: per-camera contrast calibration bounds (from CalibrationWorkflow).
- `i_min`, `i_max`: per-camera intensity calibration bounds.
- All are `(2, 8)` arrays (2 sides × 8 cameras).

**Fallback:** When `c_span == 0` (uncalibrated): `BFI = contrast × 10`. When `i_span == 0`: `BVI = mean × 10`.

**Magic numbers:**
- `10.0` — BFI and BVI scale factor. Output range is nominally 0–10.

---

#### Stage 9: DarkFrameHoldStage (`dark_frame_hold.py`)

**Purpose:** Prevent display spikes during dark intervals by holding the last light frame's BFI/BVI.

**Operation:** For each dark frame, overwrite `bfi_live[i, side, cam]` and `bvi_live[i, side, cam]` with the last seen finite light-frame values for that camera.

**Only holds:** `bfi_live`, `bvi_live` (display metrics). Does NOT hold `mean_dc_rt` or `contrast_sn_rt` — those feed the correction path and must reflect actual measurements.

**Data replaced:** Dark frame display values are silently replaced. The frame stays labeled "dark" — only its display values change.

---

#### Stage 10: LiveSideAverageStage (`side_avg.py`)

**Purpose:** (Reduced mode only) Compute a single BFI and BVI per side per capture for simplified clinical display.

**Operation:** Groups per-row USB frames by `(side, frame_id)`. When a new frame_id begins for a side, emits the previous capture's spatial average:
- `bfi_avg = nanmean(bfi_live[selected_cameras])`
- `bvi_avg = nanmean(bvi_live[selected_cameras])`

Camera selection via bitmask (e.g. `0xC3` = cameras 0,1,6,7 = "Far" pattern).

**Data discarded:** Individual per-camera values are collapsed into a single average. The per-camera detail is only available from the "live" channel (Tee at end).

---

#### Stage 11: CorrectedSideAverageStage (`corrected_side_avg.py`)

**Purpose:** (Reduced mode only) Same spatial averaging as LiveSideAvg, but over the dark-corrected (batch path) values for DB persistence.

**Operation:** Reads `IntervalClosed` events from `batch.events`. Groups the enclosed `EnrichedCorrectedFrame`s by `(side, frame_id)`. Spatially averages BFI, BVI, mean, contrast across selected cameras. Emits `LiveEmit("final_side", SideAverageSample)`.

---

#### Stage 12: Tee("live") (`tee.py`)

**Purpose:** Snapshot the fully-processed batch for live display sinks.

- **Filter:** `ft != "warmup" and ft != "stale"` — only dark + light frames reach live sinks.
- **Payload:** The `FrameBatch` reference with all computed fields populated.

**Data discarded:** Warmup and stale frames never reach live sinks.

---

### Data Flow Summary: What Gets Discarded

| What | Where | Why |
|---|---|---|
| Stale frames | Tee("raw") filter, Tee("live") filter | Leftover from prior scan in USB buffer |
| Warmup frames (first 9) | Tee("live") filter | Camera stabilization period |
| Sub-threshold histogram bins (< 10 counts) | NoiseFloorStage | Detector noise removal |
| Negative pedestal-subtracted mean | PedestalSubtractionStage clamp | No physical meaning |
| Negative variance after dark/shot correction | DarkCorrection, ShotNoise clamp | Mathematical artifact |
| Raw frames after max_duration_s | Tee("raw") time gate | User-configured raw CSV cap |
| Dark frame BFI/BVI display values | DarkFrameHoldStage | Replaced with last light values |
| Terminal dark-like light frames | DarkCorrectionStage.on_scan_stop | Promoted to dark boundary, removed from light list |
| Realtime values during warmup (no dark yet) | DarkCorrectionStage | NaN propagated — no prediction possible |

### All Magic Numbers

| Value | Location | Meaning |
|---|---|---|
| `1024` | MomentsStage, pedestal.py, config.py | Histogram bins / 10-bit ADC full-scale DN |
| `256` | classify.py `_FRAME_ID_MODULUS` | 8-bit frame counter modulus |
| `128` | classify.py `_FRAME_ROLLOVER_THRESHOLD` | Forward-step detection threshold for rollover |
| `9` | factory.py `discard_count` default | Warmup frames to discard at scan start |
| `600` | factory.py `dark_interval` default | Frames between dark baselines (15 sec at 40 fps) |
| `10` | noise_floor.py `threshold` default | Minimum bin count to survive noise floor |
| `64.0` | pedestal.py | Legacy sensor pedestal (FW ≤ 1.5.2) |
| `128.0` | pedestal.py | Current sensor pedestal (FW > 1.5.2) |
| `11_000` | pedestal.py `adc_gain_for_pedestal()` | Full-scale electron count for ADC gain calc |
| `5.0` | dark.py `DarkIntegrityGuard` | Max DN above pedestal for valid dark frame |
| `4` | factory.py `realtime_dark_history_size` | Dark history ring buffer depth |
| `3` | dark.py `HybridRealtimePredictor` | Number of darks averaged for u1 prediction |
| `2` | dark.py `HybridRealtimePredictor` | Number of darks for std linear extrapolation |
| `-1/6, 2/3` | dark.py `DarkFrameQuadraticStencil` | 4-point quadratic interpolation coefficients |
| `[16,4,2,1,1,2,4,16]` | config.py `CAMERA_GAIN_MAP` | Per-camera analog gain (outer cameras higher) |
| `10.0` | bfi_bvi.py, dark.py enrichment | BFI/BVI output scale factor |
| `(1, 5, 2)` | pedestal.py `pedestal_for_fw()` | FW version threshold for pedestal change |

---

## Notes

_(space for ongoing rehab notes)_
