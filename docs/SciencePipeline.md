# Open-Motion Science Pipeline — Technical Reference

**Implementation:** `omotion/pipeline/` — a stage-based, channel-dispatched pipeline package. The entry point is `omotion.pipeline.factory.default_pipeline()`, which composes the canonical chain of stages used by every live scan and CSV replay.

**Audience:**

1. **Scientists and engineers** who need a mathematically precise description of every transformation between the FPGA's raw 1024-bin histogram and the BFI/BVI values that appear in the corrected CSV, the scan DB, and the live UI.
2. **Contributors** extending a stage, adding a new sink, or wiring a new source. The "Architecture overview", "Stages", "Channels", and "Sinks" sections describe the contracts you must satisfy.
3. **Auditors** who need a static, end-to-end account of how raw acquisition data becomes the clinical output. Every transformation, every fallback, and every guard rail described below is implemented in the cited file at the cited line; the field-ownership table (§2) lists which stage is permitted to write each field of the in-flight `FrameBatch`.

---

## 1. Architecture overview

A scan is driven by a `ScanRunner` (`omotion/pipeline/runner.py`). It pulls `FrameBatch` objects from a `Source`, passes each batch through a `Pipeline` of `Stage`s, and dispatches the resulting events to subscribed `Sink`s on named channels.

```
┌────────────┐   FrameBatch  ┌─────────────────────────────────────────┐
│   Source   ├──────────────►│           Pipeline (Stage list)         │
│ (live USB, │  N frames per │ Classify → TelemetryIngest* → Tee(raw) →│
│ CSV)       │  batch        │ TimestampRepair → NoiseFloor → Moments →│
└────────────┘               │ PedestalSub → DarkCorrection →          │
                             │ ShotNoise → BfiBvi → DarkFrameHold →    │
                             │ SideAvg → Tee(live)                     │
                             └──────────────┬──────────────────────────┘
                                            │ batch.events (LiveEmit /
                                            │ IntervalClosed / diagnostics)
                                            ▼
                              ┌──────────────────────────┐
                              │       ScanRunner         │
                              │  channel-based dispatch  │
                              └─┬─────┬──────┬─────┬────┘
                                │     │      │     │
                              raw  live(_side) final diagnostics
                                │     │      │     │
                                ▼     ▼      ▼     ▼
                                  Sinks (CsvSink, ScanDBSink,
                                  DiagnosticsLogSink, app
                                  live/final plot sinks, …)
```

(*TelemetryIngest is present when a `TelemetryAggregator` is wired — every
live scan with a running console telemetry poller. The poller also feeds the
per-scan telemetry CSV via a separate listener, outside the pipeline. See §9.)

The pipeline is **pure transformation**: stages mutate a typed `FrameBatch` dataclass in place and append events to `batch.events`. They never perform I/O. All side effects — file writes, UI emission, DB inserts — happen in sinks downstream of the runner.

Three things characterise this design and make it auditable:

- **One owner per field.** Each field on `FrameBatch` is written by exactly one stage. The ownership table (§2) is part of the contract; tests assert it.
- **No global mutable state.** Per-scan state (pedestal values, calibration arrays, dark history) is constructor-injected into stages and reset by `Pipeline.reset()` at scan start.
- **Channel-based output.** Sinks declare which channels they care about (`channels: set[str]`). The runner dispatches each `LiveEmit` / `IntervalClosed` event to the matching sinks. Adding a new consumer is a class with a `channels` set and a `consume(channel, payload)` method; no pipeline code changes.

---

## 2. The FrameBatch

`omotion/pipeline/batch.py` defines `FrameBatch`, the typed data carrier that flows through every stage. All per-frame fields are NumPy arrays of shape `(N, …)`, where N is the batch size (typically 10–100 frames). Stages mutate the batch in place for performance — no per-batch allocation churn.

### 2.1 Field ownership

| Field | Shape | Owner stage | Meaning |
|---|---|---|---|
| `cam_ids` | `(N,)` int8 | Source (parse) | Camera index 0..7 |
| `frame_ids` | `(N,)` uint8 | Source (parse) | Firmware rolling 8-bit counter |
| `raw_histograms` | `(N, 2, 8, 1024)` uint32 | Source (parse) | Raw 1024-bin histogram per side × cam; mutated in place by NoiseFloorStage |
| `temperature_c` | `(N, 2, 8)` float32 | Source (parse) | Sensor-reported temperature |
| `timestamp_s` | `(N,)` float64 | Source (parse) | Sensor timestamp; normalised to scan start by `_BaseSource` |
| `pdc` | `(N,)` float32 \| None | TelemetryIngestStage (or replay source) | PDC reading (mA) at the frame's capture time; NaN before the first telemetry sample; None when no aggregator is wired |
| `tcm` | `(N,)` int64 \| None | TelemetryIngestStage (or replay source) | MCU trigger counter (lsync pulses) |
| `tcl` | `(N,)` int64 \| None | TelemetryIngestStage (or replay source) | Laser trigger counter |
| `abs_frame_ids` | `(N,)` int64 | FrameClassificationStage | Monotonic unwrapped frame counter |
| `frame_type` | `(N,)` `<U8` | FrameClassificationStage | One of `"warmup"`, `"dark"`, `"light"`, `"stale"` |
| `mean_raw` | `(N, 2, 8)` float32 | MomentsStage | First moment μ₁ of raw histogram (NaN where count == 0) |
| `std_raw` | `(N, 2, 8)` float32 | MomentsStage | √(μ₂ − μ₁²) of raw histogram |
| `contrast_raw` | always `None` | MomentsStage | Reserved; pedestal-subtracted contrast is computed downstream |
| `subtracted_mean` | `(N, 2, 8)` float32 | PedestalSubtractionStage | `max(0, mean_raw − pedestal)` |
| `dark_baseline_rt` | `(N, 2, 8)` float32 | DarkCorrectionStage | Realtime-predicted dark baseline û₁ (NaN before first dark) |
| `mean_dc_rt` | `(N, 2, 8)` float32 | DarkCorrectionStage | `mean_raw − û₁` (best-effort dark-subtracted mean) |
| `std_dc_rt` | `(N, 2, 8)` float32 | DarkCorrectionStage | √(std_raw² − σ̂²) (best-effort dark-subtracted std) |
| `std_sn_rt` | `(N, 2, 8)` float32 | ShotNoiseCorrectionStage | std after Poisson shot-noise removal |
| `contrast_sn_rt` | `(N, 2, 8)` float32 | ShotNoiseCorrectionStage | `std_sn_rt / mean_dc_rt`, 0 where mean ≤ 0 |
| `bfi_live` | `(N, 2, 8)` float32 | BfiBviStage | BFI from realtime corrected (K, μ₁) via calibration |
| `bvi_live` | `(N, 2, 8)` float32 | BfiBviStage | BVI from realtime corrected mean via calibration |
| `events` | `list[BatchEvent]` | Stages append | Out-of-band events: `LiveEmit`, `IntervalClosed`, diagnostics. Reduced-mode side averages ride here too: realtime as `LiveEmit(channel="live_side", SideAverageSample)`, corrected as synthetic `IntervalClosed` intervals whose frames carry `cam_id=-1` (see §5.10) |

### 2.2 BatchEvent types

Stages produce events when something doesn't fit cleanly into per-frame arrays. The runner inspects `batch.events` after the pipeline returns and dispatches each event to the right channel.

| Event | Producer | Routed to |
|---|---|---|
| `LiveEmit(channel, payload)` | `Tee` stages, `SideAverageStage` (realtime path, `"live_side"`) | The named channel's sinks |
| `IntervalClosed(corrected_batch)` | `DarkCorrectionStage` (per-camera; enriched + stencilled downstream), `SideAverageStage` (reduced-mode `cam_id=-1` side averages) | `"final"` channel sinks |
| `DarkIntegrityWarning(...)` | `DarkIntegrityGuard` (inside DarkCorrectionStage) | `"diagnostics"` |
| `StencilFallback(...)` | `DarkFrameQuadraticStencil` (inside DarkFrameHoldStage) | `"diagnostics"` |
| `PipelineError(...)` | `ScanRunner` (a stage raised; the batch was dropped, state preserved) | `"diagnostics"` |
| `TimestampMisalignmentWindow(...)` | `TimestampRepairStage` (per-side coalesced window; the terminal stop-frame artifact — the firmware's laser-off frame fires ~150 ms off-grid at every scan stop — is reclassified at INFO and NOT reported) | `"diagnostics"` |
| `TerminalDarkResult(...)` | `DarkCorrectionStage.on_scan_stop` | `"diagnostics"` |
| `TriggerStateEvent(...)` | `ScanWorkflow` (out of band, via `ScanRunner.dispatch_event`) | `"diagnostics"` |

---

## 3. The Stage protocol

`omotion/pipeline/pipeline.py` defines the contract:

```python
@runtime_checkable
class Stage(Protocol):
    name: str
    def process(self, batch: FrameBatch) -> FrameBatch: ...
    def reset(self) -> None: ...
```

Stages may also implement an optional `on_scan_stop(batch)` lifecycle hook (used by `DarkCorrectionStage` for the terminal-dark flush — see §6.6).

`Pipeline.process(batch)` calls each stage's `process()` in order. `Pipeline.reset()` calls every stage's `reset()` and is invoked at scan start and whenever any stage raises during a scan (`ScanRunner.run()` then continues with the next batch).

The full chain assembled by `default_pipeline()` is:

```
FrameClassificationStage
TelemetryIngestStage                                       # when telemetry wired
Tee("raw", emit_if_any=ft != "stale", max_duration_s=…)   # conditional
TimestampRepairStage
NoiseFloorStage
MomentsStage
PedestalSubtractionStage
DarkCorrectionStage
ShotNoiseCorrectionStage
BfiBviStage
DarkFrameHoldStage
SideAverageStage             (reduced mode; emits LiveEmit "live_side" +
                              cam_id=-1 IntervalClosed → "final")
Tee("live", filter=ft not in {"warmup","stale"})
```

---

## 4. Hardware context

- **Cameras:** up to 8 OV2312 cameras per sensor module, up to 2 sensor modules (left + right), 16 cameras max.
- **Frame rate:** 40 Hz, frame sync controlled by the console MCU.
- **Histogram:** 1024 bins per camera per frame, 32-bit counts. The expected total count per valid frame is **2,457,606** (= 1920 × 1280 pixels + 6 sentinel counts), validated as `EXPECTED_HISTOGRAM_SUM` in `omotion/MotionProcessing.py`. Frames whose sum does not match are dropped by the parser before they ever reach the pipeline.
- **Dark frame protocol:** the firmware deterministically cuts laser illumination on a fixed schedule (see §5.2). The pipeline never has to infer dark/light from the data.
- **Frame ID:** each histogram packet carries an 8-bit rolling counter (0–255). The pipeline unwraps this per `(side, cam_id)` pair (§5.1).
- **Pedestal:** the camera-reported value at zero illumination. Per-side, firmware-version-keyed: 64.0 DN for sensor firmware ≤ 1.5.2, 128.0 DN after. `omotion/pipeline/pedestal.py` resolves this from the connected sensors at scan start (`SensorPedestals.from_sensors(left, right)`).

---

## 5. Stage reference

The remaining sections describe each stage's math and side effects. The order matches the pipeline order built by `default_pipeline()`.

### 5.1 FrameClassificationStage

**File:** `omotion/pipeline/stages/classify.py`. **Writes:** `abs_frame_ids`, `frame_type`.

Per `(side, cam_id)` pair, the stage maintains a `_FrameUnwrapper` (the side is inferred from `argmax` of the per-side histogram sum, since each packet's payload lives in only one of the two side slots).

**Frame-ID unwrap.** The firmware's 8-bit counter wraps from 255 → 0. The unwrapper maintains an epoch counter:

```
delta = (raw_frame_id - last_raw) & 0xFF
if delta <= 128 and raw_frame_id < last_raw:
    epoch += 1
abs_frame_id = epoch * 256 + raw_frame_id
```

A `delta > 128` (apparent large backward jump) is treated as a packet anomaly and the epoch is left unchanged.

**Stale-first guard.** The very first frame received for any `(side, cam_id)` must have `raw_frame_id == 1`. Anything else is a leftover packet from a previous scan; the unwrapper flags it and any subsequent frames whose `abs_frame_id == raw_frame_id` (i.e. epoch still 0) are labelled `"stale"` so they are dropped by the `Tee("raw", filter=ft != "stale")` and live tees.

**Frame-type labelling.** With `d = discard_count` (default 9) and `Δ = dark_interval` (default 600):

| Condition | `frame_type` |
|---|---|
| `first_was_stale` and `abs_id == raw_id` | `"stale"` |
| `abs_id ≤ d` | `"warmup"` |
| `abs_id == d + 1` OR (`abs_id > d + 1` AND `(abs_id - 1) mod Δ == 0`) | `"dark"` |
| otherwise | `"light"` |

Under the defaults, dark frames occur at `n = 10, 601, 1201, 1801, …`. Every other frame with `n > d` is `"light"`.

### 5.2 TelemetryIngestStage

**File:** `omotion/pipeline/telemetry.py`. **Writes:** `pdc`, `tcm`, `tcl`.
Present only when `default_pipeline(telemetry=...)` receives a
`TelemetryAggregator` (every live scan with a running console telemetry
poller; omitted for replays and tests).

Stamps each frame with the most recent `TelemetrySample` at-or-before the
frame's capture time: `pdc[i]` (photodiode current, mA), `tcm[i]` (MCU
trigger count), `tcl[i]` (laser trigger count). Frames before the first
sample get `pdc = NaN`, `tcm = tcl = 0` — the CSV writers render those as
blank cells.

**Clock bridging.** Frame timestamps are sensor-firmware clock normalised
to scan start; telemetry samples carry host `time.time()`. The stage
captures `wall_offset = time.time() − first_frame_ts` on its first
non-empty batch — alignment error is bounded by the source's flush
latency (~0.25 s), comfortably inside the poller's ~10 Hz cadence.

`reset()` re-anchors the offset but deliberately does **not** clear the
aggregator — telemetry history is owned by the scan, not the pipeline pass.

### 5.3 Tee("raw")

**File:** `omotion/pipeline/tee.py`. **Writes:** appends `LiveEmit(channel="raw", payload=batch)`.

Positional marker. Routes the full FrameBatch — including warmup frames — to any sink subscribed to `"raw"` (e.g. `CsvSink`). Two gates can suppress emission:

- `emit_if_any=lambda ft: ft != "stale"` — never emit batches whose every frame is stale. **This is a batch-level gate, not a row filter**: if any frame passes, the whole batch (stale rows included) is emitted, and sinks do per-row filtering via `FrameBatch.iter_rows(exclude=...)`.
- `max_duration_s` — once the batch's first timestamp exceeds this cap, no further raw emission (used to bound raw-CSV file size on long clinical scans)

If `raw_save_max_duration_s=0` is passed to `default_pipeline()`, the raw tee is omitted entirely.

### 5.4 NoiseFloorStage

**File:** `omotion/pipeline/stages/noise_floor.py`. **Writes:** mutates `raw_histograms` in place (no new field).

Zeroes histogram bins whose count is strictly below `threshold` (default 10). Implemented as a single `np.putmask`:

```
np.putmask(raw_histograms, raw_histograms < threshold, 0)
```

**Rationale:** low-count bins are dominated by read noise. Zeroing them before moment computation tightens μ₁ and σ against detector noise tails without affecting bins carrying real photon counts.

### 5.5 MomentsStage

**File:** `omotion/pipeline/stages/moments.py`. **Writes:** `mean_raw`, `std_raw`. Leaves `contrast_raw = None`.

Fully vectorised across `(N, 2, 8)`:

```
k          = arange(1024)
counts     = h.sum(axis=-1)                    # (N, 2, 8)
safe       = where(counts > 0, counts, 1)
μ₁         = einsum('nsci,i->nsc', h, k)    / safe
μ₂         = einsum('nsci,i->nsc', h, k²)   / safe
var        = max(μ₂ − μ₁², 0)                  # numerical floor
σ          = √var
mean_raw   = where(counts > 0, μ₁, NaN)
std_raw    = σ
```

`contrast_raw` is intentionally left `None`. The valid speckle contrast definition is `K = σ / (μ₁ − pedestal)`, and MomentsStage doesn't know the pedestal; pedestal-subtracted contrast is computed downstream by `ShotNoiseCorrectionStage` (after `mean_dc_rt` is available).

### 5.6 PedestalSubtractionStage

**File:** `omotion/pipeline/stages/pedestal_sub.py`. **Writes:** `subtracted_mean`.

Per-side pedestal subtraction with a clamp at zero:

```
subtracted_mean = max(0, mean_raw − pedestal)        # per-side broadcast (1, 2, 1)
```

`subtracted_mean` is the right quantity for measuring **ambient light on a dark frame** — its baseline is the zero-light pedestal, so any non-zero value is stray light leaking onto the sensor. `ContactQualityWorkflow` reads it that way for its AMBIENT_LIGHT threshold on dark frames (§11.2). For light frames, the right "intensity" quantity is `mean_dc_rt` (mean above the just-measured dark baseline, not above pedestal) — the live UI emits it, and `_ContactQualitySink` reads it for the POOR_CONTACT threshold. The dark-correction path uses `mean_raw` (un-pedestal-subtracted) directly, because the pedestal cancels exactly when you subtract one dark mean from one light mean — both terms carry the same pedestal.

### 5.7 DarkCorrectionStage

**File:** `omotion/pipeline/stages/dark.py`. **Writes:** `dark_baseline_rt`, `mean_dc_rt`, `std_dc_rt`. Appends `IntervalClosed` and `DarkIntegrityWarning` events.

This is the largest stage. It runs **two parallel corrections**:

- **Realtime (per-frame, predicted)** — produces `dark_baseline_rt`, `mean_dc_rt`, `std_dc_rt` on every light frame using the rolling dark history. Available immediately; lower fidelity at interval boundaries. Used by the live UI.
- **Batched (per dark-interval, interpolated)** — buffers all light frames in the open interval, and when the closing dark arrives, emits `IntervalClosed` carrying a raw `CorrectedInterval` (dark-subtracted mean/std only). The event is then **mutated in place by the downstream stages in the same pass**: `ShotNoiseCorrectionStage` applies shot-noise correction (§5.7.5 math), `BfiBviStage` upgrades it to an `EnrichedCorrectedInterval` with calibrated BFI/BVI, and `DarkFrameHoldStage` prepends the quadratic-stencilled dark row (§5.7.6). By the time the runner dispatches it to the `"final"` channel it is the fully corrected interval. Used by the corrected CSV, the scan DB, and any consumer that needs reproducible offline-grade output.

The realtime predictor and the batched corrector share one `DarkHistory` (a per-`(side, cam_id)` ring buffer of `DarkObservation(t, u1, std)`, default capacity 4).

#### 5.7.1 DarkIntegrityGuard (dark frames)

A genuine dark frame's μ₁ should be within ~5 DN of the sensor pedestal. Any higher and the laser likely wasn't actually off (firmware off-by-one, unwrapper alignment quirk). The guard appends a `DarkIntegrityWarning(side, cam_id, abs_frame_id, u1, pedestal, threshold)` event — a diagnostic, not a drop signal. The frame is still appended to history and used downstream.

#### 5.7.2 HybridRealtimePredictor — realtime baseline

**Why this exists.** The batched corrector waits for both bounding darks of an interval before emitting, then linearly interpolates the dark baseline backward across the interval. That's accurate but lagged by up to one full dark interval (~15 s at default settings). The realtime stream emits **immediately** with a forward-estimate of where the next dark *would* fall, so the operator's live trace is continuously dark-corrected.

**Algorithm.** Given the rolling history `D` for one camera and a target light-frame time `t`:

```
case |D| == 0  (warmup, no darks yet):
    no realtime correction emitted for this frame
    (mean_dc_rt[i, side, cam] = NaN)

case |D| == 1  (warmup, one dark):
    û₁ = D[-1].u1                              # zero-order hold
    σ̂  = D[-1].std                             # zero-order hold

case |D| >= 2  (steady state):
    û₁ = mean( last min(3, |D|) entries of D.u1 )      # avg of last 3 (ZOH if fewer)
    let a = D[-2], b = D[-1]
    let Δt = b.t − a.t
    if Δt <= 0:
        σ̂ = b.std                              # degenerate timestamps → ZOH
    else:
        slope = (b.std − a.std) / Δt
        σ̂ = b.std + slope · (t − b.t)         # linear extrapolation in time
```

In plain terms: the **mean baseline** is an average of the last few darks (stable against single-frame noise); the **std baseline** is linearly extrapolated forward from the two most recent darks (tracks slow drift in dark noise across an interval). When only one dark exists, both fall back to a zero-order hold so a realtime sample can be emitted as soon as the very first scheduled dark has arrived (relaxed from a prior "wait for two darks" rule).

**Application.** Given the predicted `(û₁, σ̂)`:

```
mean_dc_rt[i, side, cam]      = mean_raw − û₁
σ²_raw                          = std_raw²
σ²_dc                           = max(0, σ²_raw − σ̂²)
std_dc_rt[i, side, cam]        = √σ²_dc
dark_baseline_rt[i, side, cam] = û₁
```

`mean_dc_rt` and `std_dc_rt` feed ShotNoiseCorrectionStage (§5.8) and ultimately BfiBviStage (§5.9), producing the per-frame realtime BFI/BVI seen by the live UI.

The pedestal cancels exactly in `mean_raw − û₁` because both terms carry the same pedestal; no explicit pedestal subtraction occurs in this path.

#### 5.7.3 PendingInterval — batched buffering

For each `(side, cam_id)` the stage holds a `PendingInterval` with:

- `_left: _DarkBoundary` — the most recent dark observation (sets the interval's left edge)
- `_light: list[_LightSample]` — all light frames received since `_left` was set
- `_right: _DarkBoundary` — the next dark observation (closes the interval)

On every dark frame: append to history and either set `_left` (first dark) or set `_right` and flush. On every light frame: append to `_light`.

When `_right` is set, `flush()` returns a closed `Interval` and rolls `_left ← _right` for the next pass.

#### 5.7.4 LinearInterpolation — batched correction (§8.1, §8.2)

For each light frame `lf` in the closed interval `[D_prev, D_next]`:

```
t_frac        = (lf.t − D_prev.t) / (D_next.t − D_prev.t)        ∈ [0, 1]
baseline_u1   = D_prev.u1  + t_frac · (D_next.u1  − D_prev.u1)
baseline_var  = D_prev.std² + t_frac · (D_next.std² − D_prev.std²)

mean          = lf.u1 − baseline_u1
raw_var       = max(0, lf.u2 − lf.u1²)
corrected_var = max(0, raw_var − baseline_var)
std           = √corrected_var
```

The two clamps to zero prevent imaginary standard deviations when dark subtraction over-corrects due to statistical fluctuations. The result is a `CorrectedFrame(abs_frame_id, t, side, cam_id, mean, std, raw_u1, raw_var, dark_var)`.

#### 5.7.5 Enrichment — shot-noise correction + BFI/BVI calibration

After linear interpolation, the in-flight `IntervalClosed` event is enriched by the **downstream stages in the same pipeline pass**: `ShotNoiseCorrectionStage` applies shot-noise correction to each `CorrectedFrame`, then `BfiBviStage` replaces the payload with an `EnrichedCorrectedInterval` of `EnrichedCorrectedFrame`s. Same math as the realtime arrays (§5.8, §5.9) but applied once per interval — the dark-frame stencil (§5.7.6) interpolates already-enriched values, not raw means.

For one corrected frame `f`:

```
g_cam         = CAMERA_GAIN_MAP[cam_id % 8]
shot_var      = ADC_GAIN · max(0, f.mean) · g_cam
corr_var      = max(0, f.std² − shot_var)
shot_std      = √corr_var

contrast      = shot_std / f.mean       if f.mean > 0 else 0
bfi           = (1 − (contrast − c_min) / (c_max − c_min)) · 10
bvi           = (1 − (f.mean   − i_min) / (i_max − i_min)) · 10
```

When `c_min == c_max` (degenerate calibration), the fallback is identity scaling: `bfi = contrast · 10`, `bvi = mean · 10`. Constants:

| Symbol | Value | Defined in |
|---|---|---|
| `ADC_GAIN` | `(1024 − pedestal_height) / 11_000`. ≈ 0.0873 DN/e⁻ at pedestal 64 (FW ≤ 1.5.2); ≈ 0.0815 at pedestal 128 (current). | `omotion/pipeline/pedestal.py` (`adc_gain_for_pedestal`) |
| `CAMERA_GAIN_MAP` | `[16, 4, 2, 1, 1, 2, 4, 16]` indexed by `cam_id % 8` | `omotion/config.py` |

`ADC_GAIN` is **per-side**, derived from each sensor's pedestal at stage construction. `ShotNoiseCorrectionStage` takes a `SensorPedestals` and computes `[adc_gain_for_pedestal(left), adc_gain_for_pedestal(right)]` once, indexing by `side_idx ∈ {0, 1}` per frame — for both the realtime arrays and the in-flight intervals. Mixed-firmware sensor modules (left and right with different pedestals) get the right gain on each side.

Outer cameras (positions 0 and 7) use higher analog gain to compensate for reduced illumination at the array periphery; central cameras (3 and 4) run at unity gain.

#### 5.7.6 DarkFrameQuadraticStencil — the dark frame's own corrected value

The leading dark frame `D_prev` of the interval is included in the emitted interval. Its corrected value is not computed by baseline subtraction (its histogram *is* the baseline). Instead, **`DarkFrameHoldStage`** (which runs after `BfiBviStage`, so the interval is already enriched) fills in each metric (`mean`, `std`, `contrast`, `bfi`, `bvi`) with a 4-point quadratic stencil and prepends the resulting row:

```
v(D_prev) = (−1/6)·v(D_prev − 2)
          + ( 2/3)·v(D_prev − 1)
          + ( 2/3)·v(D_prev + 1)
          + (−1/6)·v(D_prev + 2)
```

- `v(D_prev − 1)` and `v(D_prev − 2)` come from `self._prev_interval_tail[(side, cam_id)]` — the last two `EnrichedCorrectedFrame`s of the previous interval.
- `v(D_prev + 1)` and `v(D_prev + 2)` come from the first two frames of the current enriched interval.

**Fallback chain** (in order, when neighbours are unavailable):

| Available | Formula |
|---|---|
| All four (steady state) | Full 4-point quadratic above |
| Left missing, ≥ 2 right (first interval) | `(v(+1) + v(+2)) / 2` |
| Only `v(−1)` and `v(+1)` | `(v(−1) + v(+1)) / 2` |
| Only `v(+1)` | `v(+1)` (repeat right neighbour) |
| Nothing at all | Stencil raises; `D_prev` not included |

After producing the stencil value, the stage updates `_prev_interval_tail` with the last two frames of *this* interval (excluding the prepended dark row), ready for the next pass.

#### 5.7.7 Interval emission

By dispatch time the `IntervalClosed` carries an `EnrichedCorrectedInterval` with frames in chronological order: `[D_prev_stencilled, L_1, L_2, …, L_k]`. The closing dark `D_next` is **not** in this interval — it becomes `D_prev` of the next interval and gets its stencil value then. (The scan's terminal dark therefore never receives a corrected row — the one by-design gap besides warmup.)

#### 5.7.8 Terminal-dark flush — `on_scan_stop(batch)`

The firmware guarantees the **last frame of every scan is a dark frame**, regardless of when the scan was stopped. For scans shorter than one full dark interval, that terminal dark won't fall on a scheduled dark position, so the pipeline receives it as the last buffered light frame in `pi._light`.

`on_scan_stop()` (called by `ScanRunner` after the source's iterator drains) walks every `(side, cam_id)` with buffered lights and:

1. Pops the **last entry in `pi._light`** — the hardware-guaranteed terminal dark.
2. Synthesises a `DarkObservation` using the **last scheduled dark's** `u1` and `std` (no independent moment measurement available for the terminal frame), stamped at the terminal frame's actual timestamp.
3. Removes the terminal frame from `_light` so it is not double-counted as a light.
4. Closes the synthetic interval `[D_prev, terminal_dark]` and calls `_emit_interval()` — normal stencil and enrichment apply.

This guarantees the corrected CSV / DB always has output for any scan that reached at least frame 10 (first scheduled dark) and at least one light frame.

If a camera has no dark history at all (scan stopped before frame 10), the flush is silently skipped — there is no baseline to subtract.

### 5.8 ShotNoiseCorrectionStage

**File:** `omotion/pipeline/stages/shot_noise.py`. **Writes:** `std_sn_rt`, `contrast_sn_rt`.

Vectorised Poisson-variance subtraction on the realtime path. Operates on `mean_dc_rt` and `std_dc_rt` from DarkCorrectionStage:

```
var          = std_dc_rt²
shot_var     = ADC_GAIN · max(0, mean_dc_rt) · gain_map           # broadcast (1, 1, 8)
corr_var     = max(0, var − shot_var)
std_sn_rt    = √corr_var
contrast_sn_rt = where(mean_dc_rt > 0, std_sn_rt / mean_dc_rt, 0)
```

`ADC_GAIN` and `CAMERA_GAIN_MAP` are the same constants used in the batched enrichment (§5.7.5). After dark subtraction the remaining variance still contains photon shot noise; subtracting the expected shot-noise contribution isolates the speckle variance, yielding a contrast `K̃` that is independent of mean photon flux. Without this correction, higher-intensity frames would appear to have artificially lower contrast.

### 5.9 BfiBviStage

**File:** `omotion/pipeline/stages/bfi_bvi.py`. **Writes:** `bfi_live`, `bvi_live`.

Affine calibration map from `(contrast_sn_rt, mean_dc_rt)` to `(BFI, BVI)`:

```
K          = contrast_sn_rt                       # (N, 2, 8)
m          = mean_dc_rt                           # (N, 2, 8)
c_span     = c_max − c_min                        # (1, 2, 8)
i_span     = i_max − i_min                        # (1, 2, 8)

bfi_live   = (1 − (K − c_min) / c_span) · 10      # where c_span > 0
bvi_live   = (1 − (m − i_min) / i_span) · 10      # where i_span > 0
```

Fallback (degenerate calibration where the span is zero): identity scaling `bfi = K · 10`, `bvi = m · 10`.

The calibration object passed to `default_pipeline()` must expose `c_min`, `c_max`, `i_min`, `i_max` as `(2, 8)` ndarrays. `omotion.Calibration.Calibration` provides this; in practice it is loaded from the console EEPROM at scan start, or computed by `CalibrationWorkflow` (§11.1).

### 5.10 Reduced-mode side averages (`SideAverageStage`)

**File:** `omotion/pipeline/stages/side_avg.py`. Active only when `metadata.reduced_mode` is True; a pass-through otherwise.

The reduced-mode per-side average is a **purely spatial** mean across the enabled cameras at one capture instant (the shared `spatial_side_average` helper) — never a temporal/rolling average. The live USB path delivers one camera per frame row, so the stage gathers a capture's cameras (sharing a `frame_id`) and emits **one value per capture per side**. One stage implements both paths:

- **Realtime path** — averages the realtime `bfi_live`/`bvi_live` → `LiveEmit(channel="live_side", SideAverageSample(t, frame_id, side, bfi, bvi))`. Drives the live reduced-mode display (immediate, best-effort). After `DarkFrameHoldStage`, so dark intervals hold steady.
- **Corrected path** — gathers the per-`(side,cam)` `EnrichedCorrectedInterval` events (`IntervalClosed`), averages the dark-corrected BFI/BVI/mean/contrast per capture, and emits one synthetic `IntervalClosed` per side whose `EnrichedCorrectedFrame`s carry **`cam_id=-1`** — the side-average convention. These ride the ordinary `"final"` channel: `ScanDBSink` persists them as the reduced-mode record (skipping per-camera frames in reduced mode), and `CsvSink` reads them for the reduced corrected CSV.

Both paths finalize a capture/window when the next begins and flush the last at `on_scan_stop`. The live and corrected averages differ by design — realtime display vs corrected record. Enabled cameras come from `metadata.left_camera_mask` / `right_camera_mask`.

### 5.11 Tee("live")

Emits the FrameBatch on the `"live"` channel after filtering out warmup and stale frames. This is the per-frame live stream consumed by the bloodflow-app's `_LivePlotSink` (realtime BFI/BVI/mean/contrast traces — later overwritten in place by the `"final"`-channel refinement, see §8.3), `ContactQualityWorkflow` (DN-scale ambient-light + poor-contact thresholding — see §11.2), and `CalibrationWorkflow` (dark-frame collection).

---

## 6. Channels

Sinks subscribe to channels by declaring a `channels: set[str]` attribute. The runner inspects every event in `batch.events` and dispatches to all sinks whose set contains the event's channel.

| Channel | Payload | Cadence | Source | Typical consumers |
|---|---|---|---|---|
| `"raw"` | `FrameBatch` (full, including warmup) | Per batch (~10–100 frames) | `Tee("raw")` | `CsvSink` (raw per-cam CSV — **the only raw record**; the scan DB does not store raw histograms) |
| `"live"` | `FrameBatch` (excluding warmup/stale) | Per batch | `Tee("live")` | bloodflow-app `_LivePlotSink` (realtime per-frame plot — later overwritten by `"final"` corrections, see §8.3), `ContactQualityWorkflow._ContactQualitySink` (DN thresholding), `CalibrationWorkflow._CalibrationCollectorSink` (dark frames) |
| `"live_side"` | `SideAverageSample` | Per capture per side (reduced mode only) | `SideAverageStage` (realtime path) | bloodflow-app `_LivePlotSink` (reduced-mode live trace) |
| `"final"` | `EnrichedCorrectedInterval` | Per closed dark interval (~1 per `dark_interval/40` seconds; default ~15 s) | `IntervalClosed` from `DarkCorrectionStage` (per-camera; enriched + stencilled by downstream stages) and `SideAverageStage` (reduced-mode `cam_id=-1` side averages) | `CsvSink` (corrected CSV), `ScanDBSink` (`session_data` — the DB's only science record), bloodflow-app `_FinalBatchSink` (overwrites the realtime points plotted from `"live"` with interval-corrected BFI/BVI/mean/contrast), `CalibrationWorkflow` (corrected light samples) |
| `"diagnostics"` | `DarkIntegrityWarning`, `StencilFallback`, `TerminalDarkResult`, `PipelineError`, `TriggerStateEvent` | As they occur | Stages append to `batch.events`; the runner also routes out-of-band events here | `DiagnosticsLogSink` (always injected — WARNING logs + scan-end summary), `ScanDBSink` (integrity summary → `session_meta`), bloodflow-app `_TriggerStateSink` |

(There is no `"telemetry"` channel today — console telemetry is written by a
poller listener outside the pipeline; see §9.)

The runner is fail-soft: if a sink raises during `consume()`, the exception is logged and the next sink is invoked — one broken sink does not break the run.

---

## 7. Sources

`omotion/pipeline/sources.py` defines:

```python
@runtime_checkable
class Source(Protocol):
    metadata: ScanMetadata
    def __iter__(self) -> Iterator[FrameBatch]: ...
    def close(self) -> None: ...
```

The runner consumes `for batch in source`, so any object that yields `FrameBatch`es is a valid source. The package ships two concrete sources.

### 7.1 LiveUsbSource

**Used by:** every live scan, via `ScanWorkflow.start_scan()`.

Per-side packet queues feed per-side reader threads that run `omotion.MotionProcessing.parse_histogram_stream`. Parsed `HistogramSample`s accumulate into `FrameBatch`es (default `batch_size_frames=10`, with a `flush_interval_s=0.25` time-based flush) and are pushed to a shared batch queue that the runner iterates.

`close()` follows a strict shutdown sequence to avoid losing the firmware's terminal dark frame:

1. `stop_streaming()` + `drain_final()` on each side's `StreamInterface`. Drained chunks are pushed into the per-side packet queue while the parser thread is still running, so the parser consumes them on its next iteration.
2. Set `self._stop` so `parse_histogram_stream`'s drain-then-exit loop wakes and exits.
3. Join the per-side reader threads (they push any final FrameBatch to the shared queue before returning).
4. Push a `None` sentinel onto the batch queue so the runner's iteration exits cleanly after delivering the last real batch.

### 7.2 CsvReplaySource

**Used by:** offline analysis, regression testing, the `view_corrected_scan.py` script.

Replays a raw-histogram CSV produced by `CsvSink` (one CSV per side, optionally one or both). Schema: `cam_id, frame_id, timestamp_s, type, 0..1023, temperature, sum, tcm, tcl, pdc`. Yields `FrameBatch`es of `batch_size_frames` (default 100) per side, in order.

The metadata to attach is the caller's responsibility — replay sources don't know the original scan's `scan_id` / `subject_id` / camera masks.

### 7.3 ScanMetadata

`ScanMetadata` (in `omotion/pipeline/sinks.py`) is the immutable per-scan handle every sink receives at `on_scan_start`:

```python
@dataclass(frozen=True)
class ScanMetadata:
    scan_id:           str
    subject_id:        str
    operator:          str
    started_at_iso:    str
    duration_sec:      int
    left_camera_mask:  int
    right_camera_mask: int
    reduced_mode:      bool
```

---

## 8. Sinks

`omotion/pipeline/sinks.py` defines:

```python
@runtime_checkable
class Sink(Protocol):
    channels: set[str]
    def on_scan_start(self, meta: ScanMetadata) -> None: ...
    def consume(self, channel: str, payload: Any) -> None: ...
    def on_complete(self) -> None: ...
```

The runner calls `on_scan_start(metadata)` on every sink before the first batch, `consume(channel, payload)` for every dispatched event, and `on_complete()` once iteration finishes (in a `finally` block, so it runs even on cancellation). Any sink that raises is logged and skipped — the run continues.

### 8.1 CsvSink

**Channels:** `{"raw", "final"}`.

Writes the two legacy CSV families. File creation is lazy on first `consume()`.

- **Raw CSV** — one file per side, named `{scan_id}_{subject_id}_{side}_mask{XX}_raw.csv`. Schema: `cam_id, frame_id, timestamp_s, type, 0..1023, temperature, sum, tcm, tcl, pdc`. Each frame's `type` column is the `frame_type` written by FrameClassificationStage (`"warmup" | "dark" | "light" | "stale"`). Telemetry columns (`tcm`, `tcl`, `pdc`) carry TelemetryIngestStage's per-frame stamps (§5.2); cells are blank when no telemetry sample preceded the frame or no aggregator was wired (replays of pre-telemetry scans stay column-compatible).
- **Corrected CSV** — one file per scan, named `{scan_id}_corrected.csv`.
  - **Normal mode** (82 columns): `frame_id, timestamp_s, {bfi,bvi,mean,contrast,temp}_{l,r}{1..8}`. Per-frame rows are accumulated in `_corrected_acc` until every expected `(side, cam)` slot has contributed a `mean`; only then is the row written.
  - **Reduced mode** (6 columns): `frame_id, timestamp_s, bfi_left, bfi_right, bvi_left, bvi_right`. Each `EnrichedCorrectedFrame` writes into the row's `bfi_{side}`/`bvi_{side}` slot based on `frame.side`; a row is flushed once both expected sides have contributed (or only one, if the other's camera mask is zero).

On `on_complete`, any partial accumulator rows are flushed verbatim (with blanks for the missing slots) so no completed frames are lost.

### 8.2 ScanDBSink

**Channels:** `{"final"}`.

SQLite endpoint — **the corrected (final-branch) record only**. On `on_scan_start`, opens a `ScanDatabase` and creates a session row labelled `{scan_id}_{subject_id}`, stamping `session_meta` with `scan_id`, `subject_id`, `operator`, `started_at_iso`, `duration_sec`, `data_semantics: "final"`, and `sdk_flags` (`reduced_mode`, camera masks). Sessions without `data_semantics` were written by older SDKs and hold realtime (live-branch) values in `session_data`.

- **Normal mode** — one `session_data` row per per-camera `EnrichedCorrectedFrame` (`cam_id` 0..7, `side` 0/1, `frame_id` = absolute frame id) carrying `bfi`, `bvi`, `mean`, `contrast`, `quality`. The stencilled leading dark frame of each interval is included, so the record is gapless at 40 Hz (except warmup frames 1..`discard_count` and the scan's terminal dark frame).
- **Reduced mode** — only the `cam_id=-1` side-average frames emitted by `SideAverageStage` are persisted; per-camera frames are skipped.

The DB stores **no raw histograms and no realtime values**: raw lives only in the raw CSVs (`Tee("raw")` → `CsvSink`), and live values exist only on the `"live"`/`"live_side"` channels for the GUI. Consequence: corrected rows trail the scan by up to one dark interval (~15 s) and an unclean shutdown loses that tail — an accepted trade-off.

See [`ScanDatabase.md`](ScanDatabase.md) for the full schema. The CSV-vs-DB
output model (DB-if-present-else-CSV; CSV forced on when no DB) is described in
[`API.md`](API.md) §"Where the data goes".

### 8.3 Live-plot UI sinks (bloodflow-app)

**Channels:** `{"live", "final"}`, split across two sinks in the bloodflow-app's `motion_connector.py`:

- `_LivePlotSink` (`channels = {"live"}`) — forwards per-frame BFI/BVI/mean/contrast to the plot widget via PyQt6 signals.
- `_FinalBatchSink` (`channels = {"final"}`) — forwards per-`EnrichedCorrectedInterval` refined values to the same widget.

The plot widget uses a **two-pass refinement pattern** for BFI, BVI, mean, and contrast: the `"live"` channel emits realtime values per frame (using `bfi_live` / `bvi_live` / `mean_dc_rt` / `contrast_sn_rt`, which use the realtime predictor's dark baseline), and the `"final"` channel emits the more-accurate values per closed dark interval (from `EnrichedCorrectedFrame.bfi` / `.bvi` / `.mean` / `.contrast`, which use the linearly-interpolated baseline between bounding darks). The QML side matches by `frame_id` and overwrites the previously-plotted realtime point in place, so each plotted sample silently "settles" to the more accurate value as the trailing interval closes (~every 15 s at the default `dark_interval`).

The SDK itself ships no UI sink — only the bloodflow-app wires PyQt6 signals to QML.

### 8.4 DiagnosticsLogSink

**Channels:** `{"diagnostics"}`. **Always injected** by `ScanWorkflow` (independent of storage flags).

Logs every integrity event at WARNING — `DarkIntegrityWarning` (laser apparently on during a dark frame), `TerminalDarkResult(found=False)` (terminal interval lost), `StencilFallback`, `PipelineError` (a batch was dropped) — and emits a per-type count summary at scan end. Routine events (`TriggerStateEvent`, successful `TerminalDarkResult`) are ignored. `ScanDBSink` independently writes the same summary (count + first/last frame per type) into the session's `session_meta["diagnostics"]`, so the DB record itself shows whether a scan had integrity problems.

### 8.5 Writing your own sink

A minimal example: a sink that prints every interval's mean BFI to stdout.

```python
class StdoutBfiSink:
    channels = {"final"}

    def on_scan_start(self, meta):
        print(f"scan {meta.scan_id} started")

    def consume(self, channel, payload):
        if channel != "final":
            return
        means = [f.bfi for f in payload.frames]
        print(f"interval {payload.left_abs}-{payload.right_abs}: "
              f"mean BFI = {sum(means)/len(means):.3f}")

    def on_complete(self):
        print("scan complete")
```

Drop the instance into the `sinks=[...]` list passed to `ScanRunner` and it works. No pipeline change required.

Sink-authoring rules:

- `consume()` must not raise. Wrap your own work in `try/except`; the runner only catches as a last resort.
- `consume()` runs on the runner thread (or the telemetry thread, for `"telemetry"` events). UI work must be marshalled to the UI thread (e.g. emit a Qt signal).
- `on_complete()` must be idempotent — for `LiveUsbSource` close paths it can be called from cleanup paths after partial scans.

---

## 9. Telemetry

Two independent telemetry paths exist, both fed by the long-lived
`ConsoleTelemetryPoller` daemon thread (~10 Hz; see
[`ConsoleTelemetry.md`](ConsoleTelemetry.md)) via per-scan listeners that
`ScanWorkflow` registers at scan start and removes in the worker's
`finally`:

- **Per-frame stamping (in the pipeline)** — `omotion/pipeline/telemetry.py`.
  A `TelemetryFeeder` listener converts each snapshot to a
  `TelemetrySample(timestamp_s, pdc_ma, tcm, tcl)` and pushes it into a
  per-scan `TelemetryAggregator` (thread-safe ring buffer, default 256
  entries ≈ 25 s). `TelemetryIngestStage` (§5.2) queries
  `snapshot_at(t)` per frame and stamps `batch.pdc/tcm/tcl`, which flow
  into the raw CSV's `tcm,tcl,pdc` columns. `CsvReplaySource` reads those
  columns back, so replays carry the recorded telemetry without needing a
  live aggregator.
- **Telemetry CSV (outside the pipeline)** — the `_TelemetryCsvWriter`
  listener writes one row per snapshot to
  `{scan_id}_{subject_id}_telemetry.csv` (full snapshot: TEC, PDU,
  safety, counters). Not a pipeline sink; no channel.

The aggregator and the CSV writer are independent listeners; closing one
does not affect the other.

---

## 10. Threading model

| Thread | Owner | Lifecycle | Purpose |
|---|---|---|---|
| `LiveUsbSource-{left,right}` | `LiveUsbSource` | per scan, daemon | Per-side packet parsing → FrameBatch → shared batch queue |
| `ConsoleTelemetryPoller` | `MotionConsole.telemetry` | long-lived, daemon | ~10 Hz console telemetry snapshots; per-scan listeners feed the `TelemetryAggregator` (per-frame stamping, §5.2) and the telemetry CSV writer (§9) |
| Runner thread (`ScanWorkflow._worker`) | `ScanWorkflow` | per scan, non-daemon | Iterate the source, call `pipeline.process(batch)`, dispatch events to sinks |

The pipeline stages and all pipeline sinks run **on the runner thread** — there is no cross-thread interaction inside the pipeline itself.

If any stage raises mid-scan, the runner **drops that batch and preserves all stage state** (emitting a `PipelineError` on the `"diagnostics"` channel), then continues with the next batch. Stage state is deliberately NOT reset: clearing the frame unwrappers would re-trip the stale-first guard (§5.1) and permanently misalign the positional dark schedule. The gap left by a dropped batch is the same shape as USB packet loss, which every stage already tolerates. `Pipeline.reset()` is for scan start / replay reuse only.

---

## 11. Example consumers

Two SDK-internal consumers illustrate the pattern.

### 11.1 CalibrationWorkflow — light + dark collection

`omotion/CalibrationWorkflow.py` defines `_CalibrationCollectorSink` with `channels = {"final", "live"}`:

- On `"final"` (each `EnrichedCorrectedInterval` from DarkCorrectionStage), the sink slices each `EnrichedCorrectedFrame` into a legacy `Sample`-shaped object (`mean`, `std_dev`, `contrast`, `bfi`, `bvi`, `is_corrected=True`). These feed `_compute_calibration_from_samples` to produce the per-camera `(2, 8)` calibration arrays.
- On `"live"` (each FrameBatch), the sink picks out rows where `frame_type == "dark"` and emits a `Sample` whose `mean` is `subtracted_mean` (i.e. `max(0, mean_raw − pedestal)`) — the legacy "u1 − PEDESTAL_HEIGHT" semantics used by the ambient-light gate.

After the scan, the workflow drains the sink's `corrected_samples` and `dark_samples` and applies the existing frame-id windowing on the lights (skip-leading + `frame_window_count` cap), then either uploads the resulting calibration to the console EEPROM or returns it for inspection.

### 11.2 ContactQualityWorkflow — DN-scale thresholding

`omotion/ContactQualityWorkflow.py` defines `_ContactQualitySink` with `channels = {"live"}`. For each FrameBatch, it reads **two different DN-scale signals depending on frame type**:

- For `frame_type == "dark"` rows, tracks the per-camera maximum `subtracted_mean` (= `max(0, mean_raw − pedestal)`). Baseline is the zero-light pedestal; this measures ambient light leaking onto the sensor — the right quantity for the **AMBIENT_LIGHT** gate.
- For all other non-warmup/non-stale rows (light frames), maintains a per-camera rolling deque (default 10 samples) of `mean_dc_rt` (= `mean_raw − predicted_dark_baseline`). Baseline is the just-measured dark, not the pedestal; this measures actual laser-driven signal strength — the right quantity for the **POOR_CONTACT** gate. Early light frames before the first dark observation have `mean_dc_rt = NaN` (predictor returned `None`) and are skipped; the window fills up once the first dark lands.

After the scan, `result()` evaluates per camera: `no_signal` if no light samples were collected, `ambient_light` if the dark-frame max exceeds the per-cam dark threshold, `poor_contact` if the rolling light average falls below the per-cam light threshold, else `ok`. The verdict is rolled up into `ContactQualityResult.passed`.

Both consumers are pure sinks — they add no pipeline stages, do not modify FrameBatch fields, and can be turned off by simply not constructing them.

---

## 12. Key defaults

| Parameter | Default | Defined in | Meaning |
|---|---|---|---|
| `discard_count` | 9 | `FrameClassificationStage` | Warmup frames dropped at start |
| `dark_interval` | 600 | `FrameClassificationStage` | Frames between scheduled darks (15 s at 40 Hz) |
| `noise_floor_threshold` | 10 | `NoiseFloorStage` | Bins below this count are zeroed before moment computation |
| `pedestal` | 64.0 (FW ≤ 1.5.2) / 128.0 | `SensorPedestals.from_sensors` | Per-side ADC zero-light bias |
| `realtime_history_size` | 4 | `DarkHistory` (inside DarkCorrectionStage) | Max dark observations kept in the realtime predictor's ring buffer |
| `integrity_max_above_pedestal` | 5.0 | `DarkIntegrityGuard` | A dark frame whose u1 exceeds pedestal + 5 raises `DarkIntegrityWarning` |
| `ADC_GAIN` | `(1024 − pedestal_height) / 11_000` (≈ 0.0873 at pedestal 64, ≈ 0.0815 at pedestal 128) | `omotion/pipeline/pedestal.py` (`adc_gain_for_pedestal`) | Sensor ADC gain used for shot-noise correction; derived per-scan from the pedestal |
| `CAMERA_GAIN_MAP` | `[16, 4, 2, 1, 1, 2, 4, 16]` | `omotion/config.py` | Per-camera analog gain by `cam_id % 8` |
| `HISTO_BINS` / `HISTO_BINS_SQ` | `[0..1023]` / element-wise square | `omotion/config.py` | Bin-index arrays for moment computations and CSV column names |
| `EXPECTED_HISTOGRAM_SUM` | 2_457_606 | `omotion/MotionProcessing.py` | Required total count per valid frame (1920 × 1280 px + 6 sentinel) |
| `FRAME_ID_MODULUS` | 256 | `FrameClassificationStage` | Firmware 8-bit counter rollover period |
| `_FRAME_ROLLOVER_THRESHOLD` | 128 | `FrameClassificationStage` | Max forward delta before rollover is detected |
| `batch_size_frames` | 10 (live) / 100 (replay) | `LiveUsbSource` / `CsvReplaySource` | N frames per FrameBatch |
| `flush_interval_s` | 0.25 | `LiveUsbSource` | Time-based flush so partial batches don't stall the live UI |
| `TelemetryAggregator` ring size | 256 (≈25 s at ~10 Hz) | `omotion/pipeline/telemetry.py` | Telemetry samples retained for per-frame stamping |

---

## 13. Output data types reference

`omotion/pipeline/stages/dark.py` defines the corrected-output types.

```python
@dataclass
class CorrectedFrame:
    abs_frame_id: int
    t:             float
    side:          str          # "left" | "right"
    cam_id:        int          # 0..7
    mean:          float        # dark-subtracted u1 (no shot-noise correction)
    std:           float        # dark-subtracted std (no shot-noise correction)
    raw_u1:        float
    raw_var:       float
    dark_var:      float

@dataclass
class CorrectedInterval:
    left_abs:  int              # D_prev absolute frame id
    right_abs: int              # D_next absolute frame id
    frames:    list[CorrectedFrame]

@dataclass
class EnrichedCorrectedFrame:
    abs_frame_id: int
    t:        float
    side:     str
    cam_id:   int
    mean:     float
    std:      float             # after shot-noise correction
    contrast: float
    bfi:      float
    bvi:      float

@dataclass
class EnrichedCorrectedInterval:
    left_abs:  int
    right_abs: int
    frames:    list[EnrichedCorrectedFrame]   # includes D_prev (stencilled), then lights
```

The `"final"` channel always carries `EnrichedCorrectedInterval` (the enrichment path runs whenever calibration is available, which is the case in every production build of `default_pipeline()`). `CorrectedInterval` would be emitted only by a pipeline configured without `adc_gain`/`camera_gain_map`/`calibration`, which the factory never produces in normal use.

