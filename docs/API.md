# omotion — Public API / Interface Guide

How a host application or script drives the Open-Motion system through the
`omotion` package. This is the **consumer-facing interface**: connect to
hardware, run scans, read the data back, and (optionally) plug into the
processing pipeline. For internals (transport framing, FPGA programming, the
science of the pipeline) see [Architecture.md](./Architecture.md),
[SciencePipeline.md](./SciencePipeline.md), and [ScanDatabase.md](./ScanDatabase.md).

> **One entry point:** `omotion.MotionInterface`. Everything else hangs off it.
> Apps construct it once, `start()` it, hold the handles, and connect signals.

---

## Contents

1. [Quick start](#1-quick-start)
2. [`MotionInterface` — the facade](#2-motioninterface--the-facade)
3. [Running a scan](#3-running-a-scan)
4. [Configure / contact quality / calibration](#4-configure--contact-quality--calibration)
5. [Reading scans back — `ScanDatabase`](#5-reading-scans-back--scandatabase)
6. [Custom processing & replay — `omotion.pipeline`](#6-custom-processing--replay--omotionpipeline)
7. [Signals & threading](#7-signals--threading)
8. [Constants — `omotion.config`](#8-constants--omotionconfig)
9. [Logging & versioning](#9-logging--versioning)

---

## 1. Quick start

```python
from omotion import MotionInterface
from omotion.ScanWorkflow import ScanRequest

iface = MotionInterface(
    data_dir="C:/scans",                 # output root (optional)
    operator_id="alice",
)
iface.start(wait=True, wait_timeout=3.0)  # discover + connect (spawns daemons)

console_ok, left_ok, right_ok = iface.is_device_connected()

iface.start_scan(ScanRequest(
    subject_id="subj-001",
    duration_sec=60,
    left_camera_mask=0xFF, right_camera_mask=0xFF,
))
iface.scan_workflow.await_complete()      # scans run on a worker thread
if iface.scan_workflow.last_scan_error:
    print("scan failed:", iface.scan_workflow.last_scan_error)

iface.stop()
```

**Without hardware** — pass `demo_mode=True` (or set env `OPENMOTION_DEMO=1`) to
skip USB/serial discovery and generate synthetic data. The same API works
headless; signals fall back from `pyqtSignal` to `MotionSignal` automatically.

```python
iface = MotionInterface(demo_mode=True)
iface.start()
```

### Runnable examples

[`scripts/sdk_examples.py`](../scripts/sdk_examples.py) drives each operation in
this guide against **connected hardware** and prints the result. Run one, or all
in sequence:

```
python scripts/sdk_examples.py connect          # §2  connect + version
python scripts/sdk_examples.py configure        # §4  configure cameras
python scripts/sdk_examples.py contact-quality  # §4  per-camera CQ verdicts
python scripts/sdk_examples.py scan             # §3  run a short scan
python scripts/sdk_examples.py scan --duration 60   # §3  run a 60s scan
python scripts/sdk_examples.py read-scan        # §5  summarize the scan DB
python scripts/sdk_examples.py test-scan        # §4  per-camera test rows
python scripts/sdk_examples.py                  # all of the above, one connection
```

(`scan` and `test-scan` turn the laser on; `read-scan` is read-only and needs no
hardware. `--duration` (seconds) applies to `scan` / `test-scan`. Output lands
under a temp directory printed at connect time.)

To plot a finished scan, [`scripts/visualize_scan.py`](../scripts/visualize_scan.py)
renders BFI / BVI / mean / contrast over time (16 cameras, per-side average bold)
from the corrected CSV the `scan` example writes:

```
python scripts/visualize_scan.py --csv <scan_id>_<subject>.csv   # -> <stem>_viz.png
```

---

## 2. `MotionInterface` — the facade

`from omotion import MotionInterface`

### Construction

```python
MotionInterface(
    vid: int = 0x0483,
    sensor_pid: int = SENSOR_MODULE_PID,     # 0x5A5A
    console_pid: int = CONSOLE_MODULE_PID,   # 0xA53E
    baudrate: int = 921600,
    timeout: int = 30,
    demo_mode: bool = False,
    default_trigger_config: dict | None = None,
    data_dir: str | None = None,
    scan_db_path: str | None = None,
    operator_id: str | None = None,
)
```

| Param | Purpose |
|---|---|
| `demo_mode` | No hardware; synthetic data. |
| `data_dir` | Output root. When set, scans auto-write CSVs here (corrected/raw/telemetry, gated by the per-scan flags). |
| `scan_db_path` | SQLite file. When set, scans auto-write `session_data` rows (the queryable record used for replay). Recommended over CSV. |
| `operator_id` | Stamped into scan metadata / DB session meta. |
| `default_trigger_config` | Overrides merged over the SDK trigger defaults for every scan. |
| `vid` / `*_pid` / `baudrate` | USB/serial identifiers — defaults match production hardware. |

`data_dir` and `scan_db_path` are independent: set either, both, or neither.
With **neither**, a scan runs but persists nothing unless you supply your own
sinks (see §6). If a `scan_db_path` is set but the DB can't be opened, the scan
is **refused** (no silent data loss) — see [`start_scan`](#start_scan).

### Lifecycle

| Method | Description |
|---|---|
| `start(wait=True, wait_timeout=2.0)` | Begin discovery, open the console + both sensors, start the hotplug monitor (daemon thread). With `wait`, blocks up to `wait_timeout` for the first connection. |
| `stop()` | Tear down monitor + transports. Idempotent. |

### Device handles (stable for the process lifetime)

| Attribute | Type | What it is |
|---|---|---|
| `console` | `MotionConsole` | UART device — trigger, TEC, fan, FPGA, telemetry. |
| `left`, `right` | `MotionSensor` | USB devices — cameras, histograms, IMU, DFU. |

These are created once and never replaced, so apps cache them and connect their
signals once. Their full command sets live in `MotionConsole` / `MotionSensor`
(see Architecture.md) — most consumers only touch them through the workflows
below.

### Connection state

| Method | Returns |
|---|---|
| `is_device_connected()` | `(console_ok, left_ok, right_ok)` bools. |
| `connected_sensors()` | List of connected `MotionSensor`s. |
| `wait_for_ready(...)` | Block until the requested handles report ready. |

### Workflow entry points

| Method / property | Purpose |
|---|---|
| `start_scan(request) -> bool` | Launch a scan (see §3). |
| `cancel_scan(**kw)` | Stop the running scan. |
| `start_configure_camera_sensors(request) -> bool` | Program/enable cameras (see §4). |
| `start_calibration(request) -> bool` / `start_test_scan(request)` | Calibration / validation (see §4). |
| `apply_laser_power(*, force_fault=False) -> bool` | Write laser-driver config over I2C — a **cold-start prerequisite** for any laser scan (see §3). |
| `get_single_histogram(side, camera_id, test_pattern_id=4, auto_upload=True)` | One-shot histogram grab. |
| `scan_workflow` | The `ScanWorkflow` instance (lazy). |
| `calibration_workflow` / `contact_quality_workflow` | The other workflows (lazy; CQ shares the scan workflow). |

`start_scan` / `start_calibration` / `start_configure_camera_sensors` are thin
forwarders onto `scan_workflow`; use the property directly when you need to poll
state or call `await_complete()`.

---

## 3. Running a scan

A scan runs **asynchronously on a worker thread**. `start_scan` returns `True`
once the worker is spawned (or `False` if it refused — see below); use the
`scan_workflow` properties to observe progress and completion.

> **Laser-power cold start.** After a power-cycle the laser-driver registers are
> cleared, so the trigger fires but **no light is emitted** — scans complete but
> produce no signal. Call `iface.apply_laser_power()` once after connecting and
> before the first laser scan (scan / contact-quality / calibration / test). It
> writes the SDK's bundled laser config over I2C and persists in the firmware
> until the next power-cycle. (The bloodflow app does this automatically; raw
> SDK consumers must call it themselves.)

### `ScanRequest`

`from omotion.ScanWorkflow import ScanRequest`

```python
@dataclass
class ScanRequest:
    subject_id: str
    duration_sec: int
    left_camera_mask: int            # bit i set = camera i enabled
    right_camera_mask: int
    disable_laser: bool = False
    expected_size: int = 32837
    write_corrected_csv: bool = True # opt out when the DB is the system of record
    write_telemetry_csv: bool = True
    reduced_mode: bool = False       # per-side averaged BFI/BVI instead of per-camera
    sinks: list = field(default_factory=list)        # custom pipeline sinks (§6)
    skip_default_storage: bool = False               # don't auto-inject CSV/DB sinks
    raw_save_max_duration_s: float | None = None     # cap raw output; 0 = no raw
    batch_size_frames: int = 10
    trigger_config: dict | None = None               # override; None = interface default
```

`start_scan` (re)sends the resolved trigger config before starting the trigger,
which resets the firmware fsync/dark-frame schedule so dark correction stays
aligned — so you normally never need to set `trigger_config`.

**reduced mode** is the clinical path: the pipeline averages the active cameras
into one left + one right BFI/BVI value per capture. In reduced mode the scan DB
stores only the per-side average (`cam_id = -1` rows), not per-camera rows.

### `start_scan`

```python
ok = iface.start_scan(request)   # -> bool
```

Returns `False` (scan refused, never started) when a previous scan is still
running **or** the configured scan DB fails its pre-flight open. When the DB is
the only record (corrected CSV opt-in/off), this refusal is deliberate — it
aborts before the laser fires rather than run a scan whose data is lost. The
reason is on `scan_workflow.last_scan_error`.

### Observing & finishing

`iface.scan_workflow` exposes:

| Member | Meaning |
|---|---|
| `running` (bool) | A scan worker is active. |
| `await_complete(timeout_sec=None)` | Block until the worker exits. |
| `cancel_scan(join_timeout=5.0)` | User-stop with orderly teardown. |
| `last_scan_error` (str \| None) | Error from the most recent scan, else `None`. |
| `last_scan_canceled` (bool) | True if the most recent scan was user-canceled. |
| `current_scan_label` (str \| None) | `"{scan_id}_{subject_id}"` of the most recent scan — the DB `session_label`, valid as soon as `start_scan` returns. |

`ScanResult` (the per-scan outcome object, surfaced via the legacy callback path
and the calibration workflow) carries `ok`, `error`, `canceled`,
`scan_timestamp`, the CSV paths, and `dark_integrity_warnings`.

### Where the data goes

- **CSV** (if `data_dir` set): corrected, raw (duration-capped), and telemetry
  CSVs, gated by the per-scan flags. Corrected CSV is opt-in when a DB is set.
- **Scan DB** (if `scan_db_path` set): `session_data` rows per frame — per-camera
  BFI/BVI/mean/contrast in normal mode, or the per-side average (`cam_id=-1`) in
  reduced mode. This is what §5 reads back for replay.

---

## 4. Configure / contact quality / calibration

### Configure cameras

```python
from omotion.ScanWorkflow import ConfigureRequest
iface.start_configure_camera_sensors(
    ConfigureRequest(left_camera_mask=0xFF, right_camera_mask=0xFF,
                     power_off_unused_cameras=False))
```

### Contact quality

`iface.contact_quality_workflow.check(duration_sec=..., rolling_window=...,
dark_threshold_per_camera=..., light_threshold_per_camera=...)` runs a short
acquisition and reports per-camera ambient-light / poor-contact status. It is a
pipeline consumer that maintains its own rolling-window average.

### Calibration & test

```python
from omotion import CalibrationRequest, CalibrationThresholds
iface.start_calibration(CalibrationRequest(...))   # computes per-camera C_max / I_max
iface.start_test_scan(request)                     # validates against thresholds
```

`Calibration` (`c_min`, `c_max`, `i_min`, `i_max`, `source`) is the affine map
the pipeline uses for BFI/BVI. The connected console's calibration is loaded at
connect; `iface.scan_workflow.set_realtime_calibration(...)` overrides it.
`CalibrationResult` / `CalibrationResultRow` / `CalibrationThresholds` describe
the outcome and the pass/fail gates.

---

## 5. Reading scans back — `ScanDatabase`

`from omotion import ScanDatabase`

Open the same SQLite file the scan wrote, read-only is fine for replay:

```python
db = ScanDatabase(db_path="C:/scans/scans.db")
for s in db.iter_sessions():                 # {id, session_label, session_start, ...}
    print(s["id"], s["session_label"])

session = db.get_session_by_label("20260528_211930_subj-001")
for row in db.iter_session_data(session["id"], t_lo=0.0, t_hi=30.0):
    # row: cam_id, side(0/1), frame_id, timestamp_s, bfi, bvi, mean, contrast
    ...
db.close()
```

Key read methods:

| Method | Purpose |
|---|---|
| `iter_sessions()` / `stream_sessions(batch_size=100)` | All sessions, oldest first. |
| `get_session(id)` / `get_session_by_label(label)` | One session. |
| `iter_session_data(session_id, side=None, cam_id=None, t_lo=None, t_hi=None)` | Per-frame BFI/BVI/mean/contrast; optional side/camera/time-range filters. |
| `iter_raw_frames(...)` / `get_raw_frame(id)` | Raw histograms (only if `write_raw_to_db`). |

**`session_data` layout:** `cam_id` 0..7 are per-camera rows (normal mode);
`cam_id = -1` is the reduced-mode dark-corrected **per-side average** (one per
capture per side). `side` is `0` = left, `1` = right.

`omotion.materialize_corrected_csv(...)` rebuilds a corrected CSV from a DB
session (legacy-format export / playback).

---

## 6. Custom processing & replay — `omotion.pipeline`

Most consumers never touch this — the workflows assemble it for you. Reach here
to **replay recorded data through the pipeline**, or to **tap the live stream**
with a custom sink.

```python
from omotion.pipeline import (
    default_pipeline, ScanRunner, ScanMetadata, SensorPedestals,
    CsvReplaySource, CsvSink, ScanDBSink, CriticalSinkError,
)
```

A run is **Source → Pipeline → Sinks**, driven by `ScanRunner`:

```python
meta = ScanMetadata(scan_id="x", subject_id="y", operator="z",
                    started_at_iso="...", duration_sec=60,
                    left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False)
pipeline = default_pipeline(metadata=meta, calibration=cal,
                            pedestals=SensorPedestals(left=64.0, right=64.0))
source = CsvReplaySource(raw_csv_left="left.raw.csv", raw_csv_right=None,
                         batch_size_frames=20, metadata=meta)
ScanRunner(source=source, pipeline=pipeline, sinks=[CsvSink(output_dir="./out")]).run()
```

### Sinks & channels

A **sink** subscribes to channels and receives the pipeline's output:

```python
class MySink:
    channels = {"live"}          # which channels to receive
    critical = False             # True → a failed on_scan_start aborts the scan
    def on_scan_start(self, meta): ...
    def consume(self, channel, payload): ...   # payload depends on the channel
    def on_complete(self): ...
```

| Channel | Payload | Cadence |
|---|---|---|
| `"raw"` | `FrameBatch` (raw histograms) | per batch |
| `"live"` | `FrameBatch` after BFI/BVI (realtime) | per batch |
| `"live_side"` | `SideAverageSample` (realtime per-side avg) | per capture, reduced mode |
| `"final"` | `EnrichedCorrectedInterval` (dark-corrected frames; in reduced mode also the cam_id=-1 side-average intervals) | per closed dark interval (~15 s) |
| `"diagnostics"` | `BatchEvent` (e.g. `DarkIntegrityWarning`, trigger transitions) | as they occur |

`SideAverageSample` carries `t, frame_id, side, bfi, bvi`. A
sink whose `on_scan_start` raises is disabled for the scan unless it sets
`critical = True`, in which case `ScanRunner` raises `CriticalSinkError` and the
scan aborts (this is how `ScanDBSink` guarantees no silent data loss). Pass
custom sinks via `ScanRequest.sinks` (+ `skip_default_storage=True` to suppress
the auto-injected CSV/DB sinks).

Sources: `LiveUsbSource` (hardware), `CsvReplaySource` (raw CSV — the only
raw record; the scan DB stores corrected data only). See
[SciencePipeline.md](./SciencePipeline.md) for the stage chain.

---

## 7. Signals & threading

Device handles emit events via signals. When **PyQt6 is importable** they are
real `pyqtSignal`s; otherwise they are `MotionSignal` objects with the same
`.connect()` / `.emit()` API — so headless scripts behave like the GUI apps.

Up to three daemon threads run concurrently after `start()`: the hotplug
**ConnectionMonitor**, the per-endpoint **stream readers**, and the **telemetry
poller**. Scans add a worker thread. **Signals fire from these threads** — UI
code must marshal to the main thread (the apps use `Qt.QueuedConnection`).
Connection transitions surface as `ConnectionState` updates on the handles.

---

## 8. Constants — `omotion.config`

`from omotion import config` (or `from omotion.config import *`). Single source
of truth for wire-level values:

| Name | Value | Meaning |
|---|---|---|
| `CONSOLE_MODULE_PID` | `0xA53E` | Console USB PID (VID `0x0483`). |
| `SENSOR_MODULE_PID` | `0x5A5A` | Sensor USB PID. |
| `BAUD_RATE` | `921600` | Console UART baud. |
| `HISTO_SIZE_WORDS` | `1024` | Histogram bins. |
| `OW_ACK`/`OW_CMD`/`OW_DATA`/… | — | Packet type bytes (shared protocol). |
| `DEBUG_FLAG_*` | bit flags | Firmware debug toggles (`FAKE_DATA`, `HISTO_THROTTLE`, `HISTO_CMP`, `USB_PRINTF`, …). |

Debug flags are pushed to firmware with `MotionSensor.set_debug_flags(...)`.

---

## 9. Logging & versioning

- `omotion.set_log_root("MyApp")` prefixes every SDK logger name (e.g.
  `MyApp.Console`). Call it before `start()`.
- `omotion.__version__` is derived from git tags (`setuptools_scm`) — never hand-edited.
