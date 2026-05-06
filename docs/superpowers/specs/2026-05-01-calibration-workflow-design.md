# CalibrationWorkflow — Acquire-Compute-Write-Validate Procedure

**Date:** 2026-05-01
**Status:** Draft
**Builds on:** `2026-05-01-console-eeprom-calibration-design.md`

## Problem

The SDK now stores per-device calibration on the console EEPROM (the
prior spec). Today the only way to *populate* that calibration is to
hand-write the JSON via `MotionConsole.write_calibration(...)`. We need
an in-app procedure: a single button on the bloodflow app that performs
a guided calibration run and reports a pass/fail result, similar in
spirit to the existing EOL test (`motion_connector.py:1621-1791`) but
with the added responsibility of producing the calibration arrays in
the first place.

The previous EOL test:

- Ran one scan, used the existing hardcoded calibration.
- Wrote a CSV with per-camera mean / avg_contrast and pass/fail vs.
  thresholds in `app_config.json`.
- Did **not** compute or write `C_min/C_max/I_min/I_max`.

The new feature adds calibration computation, two-scan orchestration,
and pushes the procedure logic from the bloodflow app into the SDK so
the test app or any future host can reuse it.

## Goal

A new SDK workflow that, given a `CalibrationRequest`, runs:

1. A short calibration scan.
2. Compute `C_min`, `C_max`, `I_min`, `I_max` from scan #1.
3. Write to the console (which auto-refreshes the SDK cache).
4. A second short validation scan, using the freshly-written calibration.
5. Per-camera mean / contrast / BFI / BVI evaluated against thresholds;
   write a CSV; return a pass/fail `CalibrationResult`.

## Non-goals

- Replacing or refactoring the existing `_log_scan_image_stats` EOL flow
  in `motion_connector.py`. The two can coexist; retirement is a later
  cleanup.
- Changing how the science pipeline computes BFI/BVI.
- Changing the calibration JSON schema on the EEPROM.
- Hot-reloading thresholds at runtime — the app reads them once when
  building the `CalibrationRequest`.

## Architecture

A new module `omotion/CalibrationWorkflow.py` parallels `ScanWorkflow.py`.
It owns its own worker thread, stop event, and lock. It does **not**
talk to USB/UART directly; it submits two `ScanRequest`s to the existing
`ScanWorkflow` and processes the resulting CSVs.

```
App (BloodFlow.qml "Run Calibration" button)
  ↓
motion_connector.runCalibration() (Qt slot)
  ↓ builds CalibrationRequest from app_config.json + active masks
interface.start_calibration(req, on_complete_fn=…)
  ↓ async, returns immediately
CalibrationWorkflow worker thread
  │
  ├─ phase 1: scan_workflow.start_scan(scan_req_1)  ── waits ──┐
  ├─ phase 2: read CSVs, compute (2,8) arrays                  │
  ├─ phase 3: interface.write_calibration(...)                 │
  ├─ phase 4: scan_workflow.start_scan(scan_req_2)  ── waits ──┤
  ├─ phase 5: read CSVs, build rows, write CSV, evaluate       │
  └─ phase 6: emit CalibrationResult via on_complete_fn ───────┘
```

## Public API

### `omotion.CalibrationWorkflow` module

```python
@dataclass
class CalibrationThresholds:
    """Per-camera lower bounds. Length 8, indexed by cam_id 0..7.
    Applied symmetrically: cam_id 3 on left and on right share threshold[3].
    """
    min_mean_per_camera:     list[float]
    min_contrast_per_camera: list[float]
    min_bfi_per_camera:      list[float]
    min_bvi_per_camera:      list[float]


@dataclass
class CalibrationRequest:
    operator_id: str               # tagged into output filenames; analog of
                                   # ScanRequest.subject_id
    output_dir: str                # SDK writes raw scan CSVs and the result
                                   # CSV under this directory
    left_camera_mask: int
    right_camera_mask: int
    thresholds: CalibrationThresholds
    duration_sec: int              # Averaging window for both sub-scans.
                                   # Required — caller supplies from config.
    scan_delay_sec: int = CALIBRATION_DEFAULT_SCAN_DELAY_SEC
                                   # Lead-in skip. Each sub-scan actually runs
                                   # for (duration_sec + scan_delay_sec); only
                                   # the last duration_sec worth of frames are
                                   # used for averaging. Lets the laser /
                                   # sensors stabilize before the operating
                                   # samples are taken. Applied to BOTH the
                                   # calibration scan AND the validation scan.
    max_duration_sec: int = CALIBRATION_DEFAULT_MAX_DURATION_SEC
                                   # Watchdog timeout for the whole procedure.
                                   # If the worker hasn't finished within this
                                   # many seconds, stop_evt is set, sub-scans
                                   # are canceled, and the result is returned
                                   # with ok=False, error="watchdog timeout".
    notes: str = ""


@dataclass
class CalibrationResultRow:
    camera_index: int     # ordinal in the row list
    side: str             # "left" / "right"
    cam_id: int           # 0..7
    mean: float
    avg_contrast: float
    bfi: float
    bvi: float
    mean_test: str        # "PASS" / "FAIL"
    contrast_test: str
    bfi_test: str
    bvi_test: str
    security_id: str
    hwid: str


@dataclass
class CalibrationResult:
    ok: bool                              # procedure reached completion
    passed: bool                          # all rows PASS on all 4 sub-tests
    canceled: bool                        # cancel_calibration() was called
    error: str                            # populated when ok=False
    csv_path: str                         # "" if no CSV produced
    calibration: Optional[Calibration]    # what was written; None if write
                                          # never happened (failure pre-write)
    rows: list[CalibrationResultRow]
    calibration_scan_left_path: str
    calibration_scan_right_path: str
    validation_scan_left_path: str
    validation_scan_right_path: str
    started_timestamp: str                # YYYYMMDD_HHMMSS


class CalibrationWorkflow:
    def __init__(self, interface: "MotionInterface"): ...

    @property
    def running(self) -> bool: ...

    def start_calibration(
        self,
        request: CalibrationRequest,
        *,
        on_log_fn: Callable[[str], None] | None = None,
        on_progress_fn: Callable[[str], None] | None = None,
        on_complete_fn: Callable[[CalibrationResult], None] | None = None,
    ) -> bool: ...

    def cancel_calibration(self, *, join_timeout: float = 10.0) -> None: ...
```

### `omotion.MotionInterface` facade additions

```python
@property
def calibration_workflow(self) -> CalibrationWorkflow:
    """Lazy construction, mirrors scan_workflow."""

def start_calibration(self, request, **kwargs) -> bool:
    return self.calibration_workflow.start_calibration(request, **kwargs)

def cancel_calibration(self, **kwargs) -> None:
    self.calibration_workflow.cancel_calibration(**kwargs)
```

## Calibration math

For scan #1, the workflow re-uses the same CSV reader the bloodflow app
already uses (`VisualizeBloodflow._readdata` or an SDK-side equivalent
helper) to load per-frame, per-camera contrast and mean arrays. The
science pipeline already classifies frames as light vs. dark via the
configured `dark_interval`; the workflow restricts its aggregation to
light frames.

**Lead-in skip.** Each sub-scan actually runs for `duration_sec +
scan_delay_sec` seconds. Before aggregation, the workflow drops the
first `int(round(scan_delay_sec * CAPTURE_HZ))` frames (where
`CAPTURE_HZ` lives in `omotion.config`, currently 40 Hz) and only
averages the remaining tail (`duration_sec` worth). The skip counts
*all* frames, not just light frames; light/dark classification then
runs over the surviving tail.

For each `(module, cam_pos)` indexed by `m ∈ {0..MODULES-1}` (`0=left`,
`1=right`) and `c ∈ {0..CAMS_PER_MODULE-1}`:

```python
C_min[m, c] = 0.0
I_min[m, c] = 0.0
C_max[m, c] = mean(contrast[m, c, light_frames])
I_max[m, c] = CALIBRATION_I_MAX_MULTIPLIER * mean(mean[m, c, light_frames])
```

where `MODULES`, `CAMS_PER_MODULE`, and `CALIBRATION_I_MAX_MULTIPLIER`
come from `omotion.config`.

Inactive cameras (the corresponding bit in `left_camera_mask` /
`right_camera_mask` is clear, or no light-frame data was captured) get
`Calibration.default()` values for that position so the resulting
`(2, 8)` array always satisfies the SDK's monotonicity check
(`C_max > C_min`, `I_max > I_min`).

**Degenerate active camera** (the bit *was* set, but `C_max ≤ 0` or
`I_max ≤ 0` — e.g. camera dead, no light reaching the sensor): the
procedure aborts before the write phase. The result has
`ok=False, error="active camera (left, cam_id=3) produced zero contrast — calibration aborted"`,
`calibration=None`, no CSV written. The previous calibration on the
console is left intact.

## CSV output

Path: `{request.output_dir}/calibration-{YYYYMMDD_HHMMSS}.csv`

Columns (in this order):

```
camera_index, side, cam_id, mean, avg_contrast, bfi, bvi,
mean_test, contrast_test, bfi_test, bvi_test,
security_id, hwid
```

One row per active camera, written in the order returned by the science
pipeline. The CSV is produced **only after the validation scan** (phase
5). If the procedure aborts before phase 5, no CSV is produced and
`csv_path = ""`.

## Threshold evaluation

For each row, against `request.thresholds`:

```
mean_test     = "PASS" if mean         >= min_mean_per_camera[cam_id]     else "FAIL"
contrast_test = "PASS" if avg_contrast >= min_contrast_per_camera[cam_id] else "FAIL"
bfi_test      = "PASS" if bfi          >= min_bfi_per_camera[cam_id]      else "FAIL"
bvi_test      = "PASS" if bvi          >= min_bvi_per_camera[cam_id]      else "FAIL"
```

Overall `passed = bool(rows) and all rows pass on all four metrics`.

A row with no threshold available for one of the metrics (caller passed
`None` or empty) is treated as PASS for that metric — this matches the
existing `_log_scan_image_stats` semantics and lets the app omit
thresholds it hasn't configured yet.

## Procedure flow (worker thread)

```
ts = strftime("%Y%m%d_%H%M%S")
out = request.output_dir

phase 1 — calibration scan
  scan_req_1 = ScanRequest(
      subject_id     = f"calib1_{request.operator_id}",
      duration_sec   = request.duration_sec + request.scan_delay_sec,
      left_camera_mask  = request.left_camera_mask,
      right_camera_mask = request.right_camera_mask,
      data_dir       = out,
      disable_laser  = False,
      write_raw_csv  = True,
      write_corrected_csv = False,
      write_telemetry_csv = False,
      reduced_mode   = False,
  )
  evt = threading.Event()
  scan_result_box = [None]
  scan_workflow.start_scan(scan_req_1,
      on_complete_fn=lambda r: (scan_result_box.__setitem__(0, r), evt.set()))
  # Wait for completion or cancel
  while not evt.wait(timeout=0.1):
      if stop_evt.is_set():
          scan_workflow.cancel_scan()
          continue   # let it finish reporting then exit
  if stop_evt.is_set(): return CalibrationResult(canceled=True, ok=False, ...)
  if not scan_result_box[0].ok:
      return CalibrationResult(ok=False, error=scan_result_box[0].error, ...)

phase 2 — compute calibration
  # Drop the first scan_delay_sec worth of frames before aggregation.
  c_min, c_max, i_min, i_max = compute_from_csvs(
      scan_result_box[0].left_path,
      scan_result_box[0].right_path,
      request.left_camera_mask,
      request.right_camera_mask,
      skip_leading_frames=int(round(request.scan_delay_sec * CAPTURE_HZ)),
  )
  if degenerate(c_max, i_max, request.left_camera_mask, request.right_camera_mask):
      return CalibrationResult(ok=False, error="active cam zero ...", ...)

phase 3 — write to console
  cal = self._interface.write_calibration(c_min, c_max, i_min, i_max)
  # cal is the refreshed Calibration in the SDK cache (source="console").

phase 4 — validation scan
  scan_req_2 = ScanRequest(
      subject_id = f"calib2_{request.operator_id}",
      ...same as scan_req_1 except subject_id (duration_sec is still
      duration_sec + scan_delay_sec — same lead-in skip applies)
  )
  ...same wait pattern as phase 1...

phase 5 — build result rows + CSV
  # Drop the same lead-in window before averaging.
  rows = build_rows(
      scan_result_box[0].left_path, scan_result_box[0].right_path,
      thresholds  = request.thresholds,
      sensor_left  = self._interface.left,
      sensor_right = self._interface.right,
      skip_leading_frames = int(round(request.scan_delay_sec * CAPTURE_HZ)),
  )
  csv_path = os.path.join(out, f"calibration-{ts}.csv")
  write_csv(csv_path, rows)
  passed = bool(rows) and all_rows_pass(rows)

phase 6 — return
  return CalibrationResult(
      ok=True, passed=passed, canceled=False, error="",
      csv_path=csv_path, calibration=cal, rows=rows, ...
  )
```

## Cancel semantics

`cancel_calibration()` sets the workflow's `stop_evt` and forwards to
`scan_workflow.cancel_scan()` if a sub-scan is currently active.
Worker checks `stop_evt` between phases and during its wait loops.

| Cancel timing | Effect |
|---|---|
| During phase 1 (calibration scan) | Sub-scan canceled; nothing written; no CSV; `canceled=True, calibration=None` |
| Between phases 1 and 3 | Nothing written; `canceled=True, calibration=None` |
| Phase 3 mid-write | Inherits underlying behavior (write is one round-trip, hard to interrupt). Treated as not-canceled — let it finish. |
| Between phases 3 and 4 | Calibration stays on console; `canceled=True, calibration=<cal>`, no CSV |
| During phase 4 | Sub-scan canceled; calibration stays; `canceled=True, calibration=<cal>` |
| Between phases 4 and 5 | Same as above; no CSV |
| During CSV write | Treated as not-canceled — finish writing. |

The intent: once the new calibration is in flash, there's no value in
"un-writing" it; the operator is informed (`calibration` is populated)
and can re-run the procedure.

## Error handling

| Failure | Result |
|---|---|
| Sub-scan reports `ok=False` | `CalibrationResult(ok=False, error=<scan error>, calibration=None or <prior write>)` |
| CSV file missing / unreadable in phase 2 | `ok=False, error="calibration scan CSVs missing or unreadable"` |
| Degenerate active camera in phase 2 | `ok=False, error="active camera (...) produced zero ..."` |
| `interface.write_calibration` raises | `ok=False, error=<exception>, calibration=None` |
| CSV file missing in phase 5 | `ok=False, error="validation scan CSVs missing"`, calibration written, no CSV. The console is in a known state (just-written cal) but the validation didn't complete. |
| Threshold list shorter than 8 | Treat missing entries as "no threshold" → PASS. Same as `_log_scan_image_stats`. |
| `start_calibration` called while running | Returns `False` and logs (mirrors `start_scan`). |
| Procedure exceeds `max_duration_sec` | Watchdog sets stop_evt; sub-scan canceled if active; `ok=False, error="calibration exceeded max_duration_sec=<N>"`. If write already happened, `calibration` is populated and `csv_path=""`. |

## App-side changes (out of SDK scope but enumerated for the plan)

### `openmotion-bloodflow-app/config/app_config.json`

Add (alongside existing `eol_min_mean_per_camera` and
`eol_min_contrast_per_camera`):

```json
"eol_min_bfi_per_camera":  [<8 values>],
"eol_min_bvi_per_camera":  [<8 values>],
"max_calibration_time_sec": 600,
"calibration_scan_duration_sec": 5,
"calibration_scan_delay_sec": 1
```

| Key | Maps to | Notes |
|---|---|---|
| `max_calibration_time_sec` | `CalibrationRequest.max_duration_sec` | UI label: "Max Calibration Time". Also the right-hand side of the running counter. |
| `calibration_scan_duration_sec` | `CalibrationRequest.duration_sec` | Averaging window per sub-scan. |
| `calibration_scan_delay_sec` | `CalibrationRequest.scan_delay_sec` | Lead-in skip. Default 1 s. Applied to both sub-scans. |

### `openmotion-bloodflow-app/motion_connector.py`

- Load `eol_min_bfi_per_camera`, `eol_min_bvi_per_camera`,
  `max_calibration_time_sec`, `calibration_scan_duration_sec`, and
  `calibration_scan_delay_sec` from `app_config.json` (extend the
  existing reads near lines 391-406).
- Expose three Qt properties for QML:
  - `calibrationRunning: bool` — True between start and on_complete.
  - `calibrationStatus: str` — `""` (never run), `"running"`,
    `"passed"`, `"failed"`, `"aborted"`.
  - `maxCalibrationTimeSec: int` — read-only, sourced from the config.
- Expose a Qt signal `calibrationStateChanged()` so QML re-evaluates
  the bound properties when state transitions.
- Add a `@pyqtSlot() runCalibration(self)`:
  - Validate console + at-least-one-sensor connected. If not, emit
    `captureLog` and return without changing state.
  - Read active camera masks from existing connector state.
  - Build `CalibrationThresholds(...)` and `CalibrationRequest(...)`,
    setting `duration_sec = self._calibration_scan_duration_sec`,
    `scan_delay_sec = self._calibration_scan_delay_sec`, and
    `max_duration_sec = self._max_calibration_time_sec`.
  - Set `calibrationStatus = "running"`, emit
    `calibrationStateChanged`.
  - Call `self._interface.start_calibration(req,
    on_log_fn=self._on_calib_log,
    on_complete_fn=self._on_calib_complete)`.
- `_on_calib_complete(self, result)` runs on the worker thread; it
  marshals to the Qt event loop (this file's pattern: a queued
  `pyqtSignal` from worker → main, see existing examples around line
  830 for the connection-state slot) and:
  - Sets `calibrationStatus` to `"passed"` if `result.passed`, else
    `"failed"` if `result.ok`, else `"aborted"`.
  - Emits `captureLog` with one of "✅ Calibration: PASS" /
    "❌ Calibration: FAIL" / "⚠️ Calibration aborted: <error>".
  - Emits `calibrationStateChanged`.

(Cancel-from-UI is intentionally not added in this iteration — the
watchdog is the safety net.)

### `openmotion-bloodflow-app/pages/Settings.qml`

Add a Calibration row to the existing settings layout (near the other
device-action affordances such as fan / TEC / firmware-update). The row
has three elements left-to-right:

```
[Run Calibration]   ●   <status text>
```

- **Button** — disabled when console disconnected or
  `motion.calibrationRunning` is True.
- **Indicator light** — a small filled circle (~12-14 px). Color rules:
  - `calibrationStatus == ""` → gray (no run yet)
  - `calibrationStatus == "running"` → blue (or material accent)
  - `calibrationStatus == "passed"` → green
  - `calibrationStatus == "failed"` → red
  - `calibrationStatus == "aborted"` → orange/yellow
- **Status text** — bound to a QML expression:
  - `running`: `"Calibrating... (" + elapsedSec + "s / " + motion.maxCalibrationTimeSec + "s)"`
    — `elapsedSec` ticks via a local `Timer` element with
    `interval: 1000`, `running: motion.calibrationRunning`,
    `repeat: true`, that increments a `property int elapsedSec` on each
    tick. The timer resets to 0 in `onCalibrationStateChanged` when
    transitioning into `"running"`.
  - `passed`: `"Calibration Passed"`
  - `failed`: `"Calibration Failed"`
  - `aborted`: `"Calibration Aborted"` (and the error string in a
    smaller line beneath, if convenient)
  - `""`: empty.

## Testing

### SDK unit tests — pure, no hardware

`openmotion-sdk/tests/test_calibration_workflow_compute.py`:

- Compute defaults match published math against fixed synthetic CSVs:
  - All-active masks, uniform light-frame contrast 0.3 → `C_max[0,c]=0.3`.
  - All-active, mean=100 → `I_max[0,c]=200`.
  - One inactive camera → that position falls back to
    `Calibration.default()` value, monotonicity preserved.
  - One active camera with zero contrast → degenerate, raises an
    internal `_DegenerateCalibrationError` (or returns sentinel) so the
    workflow can map to `ok=False`.
- Threshold evaluation:
  - Row with mean=200 vs threshold 150 → mean_test=PASS.
  - Row with bfi=4 vs threshold 5 → bfi_test=FAIL.
  - Pass/fail aggregation: any FAIL → overall FAIL.
  - Empty rows → overall passed=False.
  - Threshold list length 5 → entries 5..7 default to PASS.

### SDK integration tests — mocked ScanWorkflow

`openmotion-sdk/tests/test_calibration_workflow.py`:

- Patch `ScanWorkflow.start_scan` to immediately call `on_complete_fn`
  with a fake `ScanResult` pointing at fixture CSV paths.
- Patch `MotionInterface.write_calibration` to record arguments without
  hitting hardware.
- Happy path: both scans succeed → CSV written, `ok=True, passed=True`,
  `calibration.source == "console"`.
- Phase-1 cancel: stop_evt set during wait → `canceled=True, ok=False,
  csv_path=""`, no write recorded.
- Phase-1 error: fake `ScanResult.ok=False` → `ok=False`, no write.
- Phase-2 degenerate: synthetic CSV with one zero-contrast active cam →
  `ok=False`, no write.
- Between-phases cancel (after write, before phase 4): write recorded,
  `canceled=True`, `calibration` populated, no CSV.
- Mocked threshold evaluation: rows reflect input thresholds.

### Manual hardware smoke test

Documented in the plan as Task N: run the app, click the button against
a connected console + at-least-one-sensor, confirm two scans run, CSV
appears at the expected path, pass/fail status renders, and
`interface.get_calibration().source == "console"` after.

## Open questions

- **Default scan duration.** Spec uses 5 seconds. If too short to
  produce stable per-camera averages in practice, increase here before
  implementing.
- **Light-frame extraction.** The spec assumes the workflow can identify
  light vs. dark frames from the raw histogram CSV. The science
  pipeline already does this internally; if the simplest path is "feed
  CSV through `VisualizeBloodflow.compute()` and read its filtered
  arrays," we go that way. Verify during implementation.

## Scale trade-off: corrected calibration vs. uncorrected live display

The bloodflow app's live BFI/BVI plot is driven by
`compute_realtime_metrics`, which fires once per non-dark frame and
computes
```
mean    = raw_mean − PEDESTAL_HEIGHT          (PEDESTAL_HEIGHT is dynamic per
                                               sensor firmware version: 64 for
                                               FW ≤ 1.5.2, 128 for FW ≥ 1.5.3,
                                               set at sensor connect)
contrast = std / mean
BFI     = (1 − (contrast − C_min) / (C_max − C_min)) × 10
BVI     = (1 − (mean     − I_min) / (I_max − I_min)) × 10
```

The `CalibrationWorkflow` derives `C_max` / `I_max` from the science
pipeline's *corrected* stream (`_emit_corrected_for_camera`), which uses
```
mean    = u1 − dark_u1                         (actual measured dark baseline,
                                               linearly interpolated between
                                               the two bounding dark frames
                                               of the scan — typically a few
                                               DN above PEDESTAL_HEIGHT)
std     = sqrt(var − shot_noise_var)           (shot noise removed)
contrast = std / mean
```

**The two streams use different denominators**, so the calibration
ranges `C_max` / `I_max` we write are on the corrected scale but the
live UI reads them through the uncorrected formula. The live BFI/BVI
will land in `[0, 10]` but typically clustered near one end of the
range rather than spanning it. As blood flow varies the values still
move, but the dynamic range an operator sees live is compressed
relative to what a corrected-stream consumer would see.

**Why we accept this:**

- Calibrating against the corrected stream produces physically
  meaningful values (proper dark subtraction, shot-noise removal, no
  pedestal-height assumption) that survive sensor-to-sensor variation
  in dark current.
- The bloodflow app's primary downstream consumer is the *corrected
  CSV*, not the live plot — calibration accuracy matters most where
  the corrected values are persisted and analyzed.
- Switching the live plot to the corrected stream is feasible
  (set `uncorrectedOnly: false` in `app_config.json`) but the
  corrected batch only emits once per `dark_interval` (15 s by
  default), so the plot would update once every 15 s instead of
  every frame. Not acceptable for a "live" view today.

**Future work — real-time partial correction:** see "Future work"
below.

## Future work

- **Frame-rate dark correction in the uncorrected path.** The most
  promising approach: maintain a per-camera "most recent dark `u1`
  / `var`" cache, updated each time the science pipeline processes a
  scheduled dark. `compute_realtime_metrics` then subtracts that
  cached dark instead of the static `PEDESTAL_HEIGHT`. Mean and
  variance both correct. Latency is at most one `dark_interval`. No
  firmware change required. This brings the live stream onto the
  same scale as the corrected batch (modulo shot-noise correction,
  which can be added trivially since `corrected_mean` is computed
  per frame in the new path).

  Trade-off: between a fresh dark and the next, the dark estimate is
  stale and can drift slightly; a Kalman/exponential filter over the
  dark history smooths this if needed.

- **Reduce `dark_interval`.** If firmware allows, dropping the dark
  schedule from every 600 frames to every 40 (once per second)
  collapses the corrected-batch latency to ~1 s. Then both the live
  plot and the calibration can run on the corrected stream with no
  scale mismatch and no need for a partial-correction approximation.
  Requires firmware support and accepts a small duty-cycle hit on
  light frames.

- **Two-tier display.** Plot the uncorrected stream at full frame
  rate as the *primary* trace and overlay the corrected batch values
  (one point per 15 s) as a *reference* trace. Operators get both
  the smooth live view and the absolute truth.
