# OpenMotion Science Pipeline — Technical Reference

**Audience:** Scientists and engineers who need a complete, unambiguous description of every data transformation that occurs between the raw FPGA histogram output and the BFI/BVI values that appear in the corrected CSV and live display.

**Implementation:** `omotion/MotionProcessing.py` — class `SciencePipeline` and supporting functions.

---

## 1. Summary

Each camera produces a 1024-bin histogram at 40 Hz.  The science pipeline:

1. **Discards** the first 9 frames from every camera (hardware warmup).
2. **Identifies dark frames** by a fixed schedule based on firmware timing.  Dark frames are acquired with the laser off and provide a measurement of the ambient + dark-current floor.
3. **Zeroes noise-floor bins** — histogram bins below the noise floor threshold (default 10 counts) are zeroed before moment computation to suppress low-level dark noise.
4. **Emits an uncorrected sample** for every non-dark frame immediately.  The emitted `mean` has the sensor pedestal (64 counts) subtracted so consumers see a zero-referenced intensity signal.  For dark frames, the previous non-dark frame's values are re-emitted so the live display shows no artefact.
5. **Emits a realtime dark-corrected sample** for every non-dark frame once at least one dark frame has been observed.  Uses a rolling-history predictor (avg-of-last-3 for the dark mean, linear-extrapolation for the dark std) to produce a best-effort dark-subtracted and shot-noise-corrected BFI/BVI immediately, without waiting for the next scheduled dark to close an interval (§7.4).
6. **Buffers** the raw (un-pedestal-subtracted) first and second moments of every non-dark frame for use in the batched corrected path.
7. **When a second consecutive dark frame arrives**, linearly interpolates the dark baseline across the buffered interval, subtracts it frame-by-frame, recomputes corrected contrast and intensity, applies the per-camera BFI/BVI calibration, and emits a `CorrectedBatch`.  The leading dark frame of the interval also receives a corrected value derived from its four nearest non-dark neighbors using a quadratic stencil.
8. **On scan stop**, performs a terminal dark flush so the corrected CSV is always populated even for scans shorter than one full dark interval (§8.6).

Consumers therefore receive:

| Stream | Type | Rate | `is_corrected` | Destination | Callback |
|---|---|---|---|---|---|
| Uncorrected | `Sample` per camera | ~40 Hz (every non-dark + every dark) | `False` | Live plot (raw) | `on_uncorrected_fn` |
| Realtime corrected | `Sample` per camera | ~40 Hz (every non-dark, after first dark) | `True` | Live plot (best-effort corrected) | `on_realtime_corrected_fn` |
| Corrected batch | `CorrectedBatch` per camera | ~1 per 15 s (configurable) | `True` | Corrected CSV (streaming) + plot snap | `on_corrected_batch_fn` |

The realtime-corrected and batch-corrected samples both carry `is_corrected = True`; consumers distinguish them by which callback delivered the sample.

---

## 2. Hardware context

- **Cameras:** Up to 8 OV2312 cameras per sensor module, up to 2 sensor modules (left + right), for a maximum of 16 cameras total.
- **Frame rate:** 40 Hz (camera frame sync controlled by the console MCU).
- **Histogram:** 1024 bins, each bin is a 32-bit count value.  A counts-per-electron-volt conversion factor is available but not applied within the pipeline.  The total count per frame is fixed at **2,457,606** (validated by `EXPECTED_HISTOGRAM_SUM`): the OV2312 sensor is 1920 × 1280 = 2,457,600 pixels, plus a sentinel value of 6 added by the firmware.
- **Dark frame protocol:** The firmware periodically cuts laser illumination for one frame cycle so the camera captures only ambient light and dark current.  The pipeline never has to infer whether a frame is dark from the data — it follows a deterministic schedule (§4).
- **Frame ID:** The firmware embeds an 8-bit rolling counter (0–255) in the last word of each histogram.  The pipeline unwraps this to a monotonic **absolute frame index** (§3).

---

## 3. Frame ID unwrapping

**Class:** `FrameIdUnwrapper` — one instance per `(side, cam_id)` pair.

The firmware's 8-bit counter wraps from 255 → 0.  The unwrapper maintains an epoch counter that increments on each detected rollover:

```python
delta = (raw_frame_id - last_raw) & 0xFF   # unsigned 8-bit forward distance

if delta <= 128 and raw_frame_id < last_raw:
    epoch += 1  # genuine rollover crossing

absolute_frame_id = epoch * 256 + raw_frame_id
```

A `delta > 128` (apparent large backward jump) is treated as a packet anomaly and the epoch is left unchanged.  The `absolute_frame_id` is therefore strictly monotonic for normal packet delivery and weakly monotonic under anomalies.

All downstream logic uses `absolute_frame_id`.  The raw `frame_id` is retained only for CSV column compatibility.

---

## 4. Frame classification

### 4.1 Warmup discard

Frames with `absolute_frame_id` ∈ {1, 2, …, `discard_count`} (default `discard_count = 9`) are silently dropped.  These frames correspond to the first 9 camera trigger cycles after the pipeline starts, during which the sensor's internal state (AGC, PLL, etc.) has not yet settled.

No callback is fired and no data is stored for these frames.

### 4.2 Dark frame schedule

A frame at absolute index *n* is classified as a **dark frame** if and only if:

```
n == discard_count + 1                         # first dark (frame 10 by default)
OR
n > discard_count + 1  AND  (n − 1) mod dark_interval == 0
```

where `dark_interval` defaults to 600 (= 15 seconds at 40 Hz).  Under the defaults the dark frames occur at:

> n = 10, 601, 1201, 1801, …

Every other frame with `n > discard_count` is a **bright (non-dark) frame**.

Note that frame n = 1 satisfies the formula `(n−1) mod 600 == 0` but is already discarded as a warmup frame, so it is never observed by the pipeline.

---

## 5. Noise floor decimation

Before moment computation, bins with a count strictly below `noise_floor` (default **10**) are zeroed:

```python
below = hist < noise_floor
if below.any():
    hist = hist.copy()
    hist[below] = 0
    row_sum = int(hist.sum())
```

This suppresses low-level dark noise that would otherwise bias the mean and variance estimates.  The recalculated `row_sum` is used for all subsequent moment math.  This step runs on both dark and bright frames.

---

## 6. Moment computation

For every frame that passes the discard and dark-schedule checks, the pipeline computes the first two moments of the (noise-floor-decimated) histogram.  Let **h** = (h₀, h₁, …, h₁₀₂₃) be the bin values (in counts) and *N* = Σ_k h_k the total count after decimation.

```
μ₁ = (1/N) Σ_{k=0}^{1023} k · h_k          # first moment (mean bin index)
μ₂ = (1/N) Σ_{k=0}^{1023} k² · h_k         # second moment
σ² = μ₂ − μ₁²                               # variance  (always ≥ 0 by construction)
```

In NumPy:

```python
HISTO_BINS    = np.arange(1024, dtype=np.float64)
HISTO_BINS_SQ = HISTO_BINS ** 2

μ₁ = np.dot(hist, HISTO_BINS)    / row_sum
μ₂ = np.dot(hist, HISTO_BINS_SQ) / row_sum
σ² = max(0.0, μ₂ - μ₁**2)
```

These are computed identically for both dark and bright frames.  Speckle contrast is derived from these moments in §7 (uncorrected stream) and §8 (corrected batch), where pedestal and shot-noise corrections are applied first.

---

## 7. Uncorrected stream

### 7.1 Pedestal subtraction

The camera sensor ADC settles at bin index **64** when no photons reach the sensor — a fixed DC bias called the **pedestal**.  The pedestal is present in every frame (both bright and dark) and must be removed before the mean is meaningful as an intensity signal.

Let `raw_μ₁` be the histogram mean as computed in §6.  The pedestal-adjusted mean used for uncorrected output is:

```
mean_val = max(0.0, raw_μ₁ − PEDESTAL_HEIGHT)     PEDESTAL_HEIGHT = 64.0
```

Variance is **invariant** to a constant shift of bin indices (subtracting a constant C from every bin centre shifts the mean by C but leaves E[X²]−E[X]² unchanged).  Therefore:

```
σ² = max(0.0, μ₂ − raw_μ₁²)    # computed from raw_μ₁, not mean_val
σ  = √σ²
K  = σ / mean_val               # contrast denominator uses pedestal-adjusted mean
```

This means contrast — and therefore BFI — reflects the relative fluctuation above the true signal baseline, not above the pedestal floor.

**Important:** the raw `u1` and `u2` values stored in `_pending_moments` and `_dark_history` for later dark-frame correction are **not** pedestal-adjusted.  Both the bright-frame `fm.u1` and the dark-frame `dark_u1` carry the same pedestal, so it cancels exactly in the dark-subtracted corrected stream:

```
corrected_mean = fm.u1 − dark_u1      # pedestal present in both, cancels
```

No explicit pedestal subtraction is performed anywhere in the corrected path.

### 7.2 Bright frames

For each bright frame *n*, the pipeline immediately constructs and emits a `Sample` with `is_corrected = False`:

| Field | Value |
|---|---|
| `mean` | `mean_val` = max(0, raw_μ₁(n) − 64) |
| `std_dev` | σ(n) — computed from raw_μ₁ (pedestal-invariant) |
| `contrast` | K(n) = σ(n) / mean_val |
| `bfi` | BFI(K(n), mean_val) — see §9 |
| `bvi` | BVI(K(n), mean_val) — see §9 |
| `is_corrected` | `False` |

This fires the `on_uncorrected_fn` callback immediately, before any dark-frame correction is available.

The **raw** moments `u1 = raw_μ₁(n)` and `u2 = μ₂(n)` (without pedestal subtraction) are stored in `_pending_moments[key]` as a `_StoredFrameMoments` object for later dark-frame correction.

### 7.3 Dark frames — live display masking

**Rule:** When a dark frame *D* arrives, the `on_uncorrected_fn` callback receives a `Sample` whose metric values (`mean`, `std_dev`, `contrast`, `bfi`, `bvi`) are copied from the immediately preceding bright frame's `Sample`.  Only `frame_id`, `absolute_frame_id`, and `timestamp_s` are updated to reflect the dark frame's true position.

```python
# Pseudo-code
dark_uncorrected = copy(last_uncorrected[key])
dark_uncorrected.frame_id          = D_raw
dark_uncorrected.absolute_frame_id = D
dark_uncorrected.timestamp_s       = ts
dark_uncorrected.is_corrected      = False
emit on_uncorrected_fn(dark_uncorrected)
```

**Rationale:** A dark frame has anomalously low intensity and contrast because the laser is off.  Emitting the raw dark values would cause the live trace to drop sharply and then recover, appearing as a ~25 ms artefact every 15 seconds.  Repeating the previous value makes the dark frame invisible to the live display.

If no preceding bright frame exists (i.e. the very first frame in the scan is a dark frame), no uncorrected sample is emitted for that dark frame.

### 7.4 Realtime dark-corrected stream

A third per-frame emission complements the uncorrected stream by applying a dark-baseline subtraction *predicted* from the rolling history of recent dark frames, rather than the linear interpolation used in the lagged batch path (§8). The realtime stream fires on the same ~40 Hz cadence as the uncorrected stream but emits samples with `is_corrected = True` whose dark subtraction reflects the **best estimate available now**, several seconds before the next scheduled dark frame closes a batch interval.

**Implementation:** `SciencePipeline._emit_realtime_corrected()`. Reference: [`data-processing/dark-drift-study/online_estimators.md`](../data-processing/dark-drift-study/online_estimators.md).

**When it fires.** Each non-dark frame, once at least one prior dark frame has been observed for that `(side, cam_id)`. Frames before the first dark do not produce a realtime-corrected sample (no baseline to subtract; warmup window is typically the first ~0.25 s of the scan until the first scheduled dark at frame 10).

#### 7.4.1 Predictor algorithm

Per `(side, cam_id)` pair, the pipeline maintains a rolling deque of recent dark observations `(timestamp_s, u1, std)` of size `realtime_dark_history_size` (default **4**).

For a target light frame at time *t*, let *D* be the deque of recent darks for that camera. The predictor produces a baseline `(û₁, σ̂)`:

```
case |D| == 1 (warmup — only one dark seen):
    û₁ = D[-1].u1                                    # zero-order hold
    σ̂  = D[-1].σ                                     # zero-order hold

case |D| >= 2 (steady state):
    û₁ = mean( last min(3, |D|) entries of D.u1 )    # average of last 3 darks (fewer if |D| < 3)

    let a = D[-2], b = D[-1]                          # two most recent darks
    let Δt = b.t − a.t
    if Δt <= 0:
        σ̂ = b.σ                                       # degenerate timestamps → ZOH
    else:
        slope = (b.σ − a.σ) / Δt
        σ̂ = b.σ + slope · (t − b.t)                  # linear extrapolation in time
```

In plain language: the **mean baseline** is an average of recent darks (stable against single-frame noise), while the **standard deviation baseline** is linearly extrapolated forward from the two most recent darks (tracks slow drift in dark noise across an interval). When fewer than two darks have been recorded — the warmup window immediately after scan start — both quantities fall back to a zero-order hold on whatever single dark is available, so a realtime-corrected sample can be emitted as soon as the very first dark has arrived (relaxed from a previous "wait for two darks" rule).

The linear extrapolation operates on `std`, not on `variance`. `std` is the smooth quantity over a dark interval; predicted variance is recovered as `σ̂²` at the application step below.

#### 7.4.2 Sample construction

Given the predicted baseline `(û₁, σ̂)` and the current light frame's raw moments `(raw_μ₁(n), μ₂(n))` (the same raw, un-pedestal-subtracted moments stored in `_pending_moments`), the realtime-corrected sample carries:

```
μ̃₁(n)     = raw_μ₁(n) − û₁                          # predictor-corrected mean
raw_σ²(n)  = μ₂(n) − raw_μ₁(n)²                       # raw variance
σ̃²(n)     = raw_σ²(n) − σ̂² − σ²_shot(n)              # dark- and shot-noise-subtracted variance
σ̃²(n)     = max(0, σ̃²(n))                            # clamp
σ̃(n)      = √σ̃²(n)
K̃(n)      = σ̃(n) / μ̃₁(n)     if μ̃₁(n) > 0
            = 0.0                otherwise
```

with shot-noise variance identical to the batched path (§8.3):

```
σ²_shot(n) = ADC_GAIN · g_cam · max(0, μ̃₁(n))
```

BFI and BVI are computed from `K̃(n)` and `μ̃₁(n)` via the calibration mapping (§9). The pedestal cancels exactly in `raw_μ₁(n) − û₁` because the same pedestal is present in both terms; no explicit pedestal subtraction occurs in this stream.

The emitted `Sample` has `is_corrected = True` — the same flag used for samples in a `CorrectedBatch`. Consumers distinguish realtime-corrected from batch-corrected samples by which callback delivered them (`on_realtime_corrected_fn` vs. `on_corrected_batch_fn`), not by a field on the sample.

#### 7.4.3 Why predictor (realtime) vs. interpolator (batch)?

The batch-corrected stream (§8) waits for both bounding dark frames of an interval before emitting, then linearly interpolates the dark baseline backward across the interval. This is accurate but lagged by up to one full dark interval (~15 s).

The realtime stream emits immediately using a forward-extrapolated estimate of where the next dark *would* fall. The result is less accurate at the end of an interval (when the next dark is about to land and refute the prediction) but available now — suitable for live displays that show a "best-effort" dark-corrected BFI/BVI trace to the operator.

A live plot reading the realtime-corrected stream sees a continuously updating dark-corrected trace; a saved corrected CSV reading the batch stream sees the accurate result that an offline analysis would reproduce.

---

## 8. Corrected batch computation

The corrected batch is computed and emitted by `_emit_corrected_for_camera(key)`, which is called each time a second (or later) consecutive dark frame arrives for a given `(side, cam_id)` pair.

Let the two bounding dark frames be at absolute positions **D_prev** and **D_next**, with:

- μ₁(D_prev), σ²(D_prev) — moments of the earlier dark frame
- μ₁(D_next), σ²(D_next) — moments of the later dark frame
- Δ = D_next − D_prev — interval width in frames

### 8.1 Baseline interpolation

For each bright frame *n* ∈ (D_prev, D_next) (i.e. strictly between the two dark frames), the dark baseline is linearly interpolated:

```
t(n) = (n − D_prev) / Δ            ∈ (0, 1)

μ̄₁(n) = μ₁(D_prev) + t(n) · [μ₁(D_next) − μ₁(D_prev)]
σ̄²(n) = σ²(D_prev) + t(n) · [σ²(D_next) − σ²(D_prev)]
```

`t(n)` is 0 at the first dark and 1 at the second dark, so the interpolation assigns more dark-frame weight to frames closer in time to that dark measurement.

### 8.2 Dark-subtracted moments

```
μ̃₁(n) = μ₁(n) − μ̄₁(n)                   # corrected mean

raw_σ²(n) = μ₂(n) − μ₁(n)²               # raw variance (from stored moments)
σ̃²(n)  = raw_σ²(n) − σ̄²(n) − σ²_shot(n) # dark- and shot-noise-corrected variance
σ̃²(n)  = max(0, σ̃²(n))                   # clamp to prevent imaginary std
σ̃(n)   = √σ̃²(n)

K̃(n) = σ̃(n) / μ̃₁(n)    if μ̃₁(n) > 0
      = 0.0               otherwise
```

The three subtractions remove: (1) the interpolated dark baseline variance, (2) the expected photon shot noise — see §8.3 below.

Clamping `σ̃²` to zero prevents imaginary standard deviations when dark subtraction or shot-noise removal over-corrects due to statistical fluctuations.

### 8.3 Shot-noise correction

Photon shot noise follows Poisson statistics: the variance (in electrons) equals the mean (in electrons).  After converting to digital-number (DN) units and accounting for each camera's analog gain:

```
σ²_shot(n) = ADC_GAIN · g_cam · max(0, μ̃₁(n))
```

where:

| Symbol | Value | Meaning |
|---|---|---|
| `ADC_GAIN` | (1024 − 64) / 11 000 ≈ 0.0873 DN/e⁻ | Sensor ADC gain: full-scale DN range above pedestal divided by electrons at full scale |
| `g_cam` | `CAMERA_GAIN_MAP[cam_pos]` | Per-camera analog gain (see table below) |
| `μ̃₁(n)` | corrected mean (DN) | Dark-subtracted mean; clamped to 0 so a slightly negative corrected mean does not inflate σ² |

**`CAMERA_GAIN_MAP`** — analog gain by camera position within the 8-camera array:

| Position | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| Gain | 16 | 4 | 2 | 1 | 1 | 2 | 4 | 16 |

Outer cameras (positions 0 and 7) use higher analog gain to compensate for reduced illumination at the array periphery; central cameras (positions 3 and 4) run at unity gain.

**Rationale:** After dark subtraction the remaining variance contains both the true speckle signal and Poisson shot noise.  Subtracting the expected shot-noise contribution isolates the speckle variance, yielding a contrast `K̃` that is independent of mean photon flux.  Without this correction, higher-intensity frames would appear to have artificially lower contrast.

### 8.3 Corrected BFI/BVI

BFI and BVI are computed from K̃(n) and μ̃₁(n) via the calibration mapping (§9), yielding `bfi` and `bvi` for each frame.  These samples are assembled into a `CorrectedBatch` with `is_corrected = True`.

### 8.4 Corrected value for the dark frame itself

The dark frame D_prev is included in the corrected batch.  Its corrected BFI/BVI are not computed by baseline subtraction (its histogram *is* the baseline); instead they are filled in using the same **4-point quadratic stencil** used by the legacy `VisualizeBloodflow` pipeline, which gives a smooth, accurate interpolation through the dark-frame gap:

```
v(D_prev) = (−1/6)·v(D_prev − 2) + (2/3)·v(D_prev − 1)
          + (2/3)·v(D_prev + 1)  + (−1/6)·v(D_prev + 2)
```

applied independently to each metric: `bfi`, `bvi`, `mean`, `std_dev`, `contrast`.

- **v(D_prev − 1)** and **v(D_prev − 2)** are the last two corrected samples of the *previous* batch.
- **v(D_prev + 1)** and **v(D_prev + 2)** are the first two corrected samples of the *current* batch.

**Fallback rules (applied in order when neighbours are unavailable):**

| Available neighbours | Formula used |
|---|---|
| All four (normal case) | Full 4-point quadratic stencil above |
| Left neighbours missing (first interval) but ≥2 right | Linear: `[v(+1) + v(+2)] / 2` — only immediate right used |
| Only v(−1) and v(+1) available | Simple average: `[v(−1) + v(+1)] / 2` |
| No left neighbours at all | Repeat `v(D_prev + 1)` |

The current dark frame D_next is **not** included in this batch.  It becomes D_prev for the next batch and will receive its interpolated corrected value at that time.

**Rationale:** The dark frame's laser-off histogram is a valid measurement of the background floor, not of blood flow.  Interpolating between neighbors removes the dark-frame artefact from the corrected time series.

### 8.5 Batch ordering and content

The emitted `CorrectedBatch` contains samples in ascending `absolute_frame_id` order:

```
[D_prev, D_prev+1, D_prev+2, ..., D_next-1]
```

All samples have `is_corrected = True`.  The batch is emitted via `on_corrected_batch_fn`.  Internally, `_pending_moments[key]` is then truncated to discard all stored moments up to and including D_next − 1; any moments at or beyond D_next are retained for the next interval (should not occur in normal operation).

### 8.6 Terminal dark flush (short scans)

The firmware guarantees that the **last frame of every scan is always a dark frame** (laser off), regardless of when the scan is stopped.  However, for scans that end before the second scheduled dark position (e.g. a scan stopped at frame 300, before the frame-601 dark), `_emit_corrected_for_camera` would never be called and the corrected CSV would remain empty.

When the science worker's ingress queue fully drains after the stop event is set, `_flush_terminal_dark()` is called once.  For each camera that has buffered moments but has not yet emitted a corrected batch for this interval, it:

1. Takes the **last entry in `_pending_moments[key]`** — this is the hardware-guaranteed terminal dark frame.
2. Computes its variance (`max(0, u2 − u1²)`) and appends it to `_dark_history[key]` as a synthetic dark entry.
3. Removes it from `_pending_moments[key]` so it is not double-counted as a bright frame.
4. Calls `_emit_corrected_for_camera(key)` to perform the normal baseline interpolation and emit the corrected batch for all buffered frames.

This ensures the corrected CSV always contains output for any scan that reached at least:
- Frame 10 (first scheduled dark — provides the opening dark baseline), and
- At least one bright frame after it.

If a camera has no dark history at all (scan stopped before frame 10), the flush logs a `WARNING` and skips that camera — there is no baseline to subtract.

---

## 9. BFI/BVI calibration mapping

Both the uncorrected and corrected paths share the same linear calibration mapping from (contrast, mean) to (BFI, BVI).  The calibration constants are stored in four NumPy arrays of shape **(2, 8)** — axis 0 is the module index (0 = left sensor, 1 = right sensor), axis 1 is the camera position (0–7).

```python
module_idx = 0 if side == "left" else 1
cam_pos    = cam_id % 8

BFI = (1.0 - (K  − C_min[module_idx, cam_pos]) /
              (C_max[module_idx, cam_pos] − C_min[module_idx, cam_pos])) × 10

BVI = (1.0 - (μ₁ − I_min[module_idx, cam_pos]) /
              (I_max[module_idx, cam_pos] − I_min[module_idx, cam_pos])) × 10
```

where:
- `C_min`, `C_max` — minimum and maximum speckle contrast values over the calibration population (`bfi_c_min`, `bfi_c_max` constructor arguments)
- `I_min`, `I_max` — minimum and maximum mean bin-index values (`bfi_i_min`, `bfi_i_max`)

For the uncorrected stream, K and μ₁ are the raw (not dark-subtracted) values.  For the corrected batch, K̃ and μ̃₁ are used.

**Fallback:** If the module or camera index is out of bounds for the calibration arrays, BFI = K × 10 and BVI = μ₁ × 10 (identity scaling, no clipping).

---

## 10. Corrected CSV output

The `ScanWorkflow` opens the corrected CSV file at the **start** of the scan (not at the end) and writes the header row immediately.  Each time `on_corrected_batch_fn` fires, the workflow accumulates samples from the batch into an in-memory `corrected_by_frame` dict keyed by `absolute_frame_id`.  As soon as all expected cameras have contributed to a given frame, that row is written to disk and removed from the dict.  Rows that are still incomplete at scan end (e.g. the last partial interval) are flushed on teardown.

This means the corrected CSV grows incrementally during the scan rather than being written all-at-once post-scan.  Data up to the last completed interval is on disk even if the scan is interrupted.

**Corrected CSV columns (normal mode):**

```
frame_id, timestamp_s,
bfi_l1..bfi_l8, bfi_r1..bfi_r8,
bvi_l1..bvi_l8, bvi_r1..bvi_r8,
mean_l1..mean_l8, mean_r1..mean_r8,
contrast_l1..contrast_l8, contrast_r1..contrast_r8,
temp_l1..temp_l8, temp_r1..temp_r8
```

**Corrected CSV columns (reduced mode — see §16):**

```
frame_id, timestamp_s,
bfi_left, bfi_right,
bvi_left, bvi_right
```

`timestamp_s` is relative to the first corrected frame written (i.e. normalised to zero at the start of the corrected output).

---

## 11. Input validation and guard rails

### Histogram sum check

Before any frame enters the pipeline, the total histogram bin sum is compared to `EXPECTED_HISTOGRAM_SUM = 2,457,606` (= 1920 × 1280 pixels + 6 sentinel counts).  Any frame whose sum does not match is silently dropped with a `WARNING`-level log message.  This rejects corrupt packets before they can contaminate science results.

This check occurs in two places:
1. In `parse_histogram_packet_structured()` during raw binary parsing.
2. Again in `_science_worker` for samples enqueued directly (e.g. in tests).

### First-frame staleness check

For each `(side, cam_id)` pair, the very first frame received after pipeline start must have `raw_frame_id == 1`.  Any other value indicates a stale packet from a previous scan that was delivered after the pipeline restarted.  Such frames are dropped and a `WARNING` is logged.

---

## 12. Threading model

The pipeline runs a single background daemon thread (`SciencePipeline` thread).  All histogram samples are ingested via a `queue.Queue` (`_ingress_queue`).  The worker thread consumes from this queue and is the sole writer of all internal state (`_unwrappers`, `_dark_history`, `_realtime_dark_history`, `_pending_moments`, `_last_uncorrected`, `_last_corrected`).  No locks are needed for pipeline-internal state.

Callbacks (`on_uncorrected_fn`, `on_realtime_corrected_fn`, `on_corrected_batch_fn`) are invoked on the science thread.  Implementations that touch UI state must marshal to the UI thread (e.g. via a Qt signal).

---

## 13. Complete data flow diagram

```
Sensor firmware
  │  8-bit rolling frame_id + 1024-bin histogram (4096 bytes)
  │  at 40 Hz per camera
  ▼
parse_histogram_packet_structured()
  │  → HistogramSample (cam_id, frame_id, histogram, row_sum, timestamp_s, temperature_c)
  │  Drop if row_sum ≠ 2,457,606  (1920×1280 px + 6 sentinel)
  ▼
SciencePipeline._ingress_queue
  │  (side, cam_id, frame_id, timestamp_s, hist, row_sum, temperature_c)
  ▼
_science_worker (single background thread)
  │
  ├─ [row_sum check] → drop if mismatch
  ├─ [first-frame check] → drop if first raw_frame_id ≠ 1
  ├─ FrameIdUnwrapper.unwrap() → absolute_frame_id
  ├─ [discard check] → drop if absolute_frame_id ≤ 9
  ├─ [noise floor] → zero bins < 10; recompute row_sum
  ├─ Compute raw_μ₁, μ₂ (used for storage AND dark history)
  │
  ├─ [dark frame?] ──── YES ──────────────────────────────────────────┐
  │                                                                    │
  │  Store (abs_frame, raw_fid, ts, raw_μ₁, σ²) → _dark_history[key] │
  │                                                                    │
  │  If ≥2 dark frames in history:                                    │
  │    _emit_corrected_for_camera(key)  ──► CorrectedBatch            │
  │       dark-subtracted BFI/BVI for interval (D_prev, D_next)       │
  │       + interpolated corrected value for D_prev itself            │
  │       → on_corrected_batch_fn()                                   │
  │       → corrected CSV (streaming, flushed per complete row)       │
  │                                                                    │
  │  Emit uncorrected sample with values from _last_uncorrected[key]  │
  │    (frame IDs + timestamp updated to D; BFI/BVI held constant)    │
  │    → on_uncorrected_fn()                                          │
  │                                                                    │
  │  continue ◄──────────────────────────────────────────────────────┘
  │
  └─ [bright frame] ──────────────────────────────────────────────────┐
                                                                       │
     Store _StoredFrameMoments(raw_μ₁, μ₂) → _pending_moments[key]   │
     (raw values preserved for dark subtraction; pedestal cancels)    │
                                                                       │
     compute_realtime_metrics() → Sample (uncorrected)                │
       mean_val = max(0, raw_μ₁ − 64)  ← pedestal subtracted         │
       σ² from raw_μ₁ (variance is shift-invariant)                  │
       K = σ / mean_val  (no shot-noise correction; uncorrected)      │
       BFI/BVI via calibration mapping                                │
                                                                       │
     Emit Sample(is_corrected=False)                                  │
       → on_uncorrected_fn()                                          │
       → _last_uncorrected[key]                                       │
                                                                       │
     If _realtime_dark_history[key] has ≥1 entry:                    │
       _emit_realtime_corrected() → Sample (realtime corrected)       │
         û₁ = avg of last 3 darks (ZOH with 1)                       │
         σ̂  = linear extrap through last 2 darks (ZOH with 1)        │
         μ̃₁ = raw_μ₁ − û₁                                            │
         σ̃² = (μ₂ − raw_μ₁²) − σ̂² − ADC_GAIN·g_cam·μ̃₁  (clamp ≥0)  │
         BFI/BVI via calibration; is_corrected=True                   │
       → on_realtime_corrected_fn()                                   │
                                                              ◄────────┘

[queue drained after stop_event]
  │
  └─ _flush_terminal_dark()
       For each key with buffered moments and ≥1 dark in history:
         promote last pending moment → synthetic terminal dark entry
         _emit_corrected_for_camera(key) → CorrectedBatch
         (ensures corrected CSV populated for scans < dark_interval)
```

---

## 14. Key constants and defaults

| Parameter | Default | Meaning |
|---|---|---|
| `discard_count` | 9 | Warmup frames dropped at start |
| `dark_interval` | 600 | Frames between dark acquisitions (15 s at 40 Hz) |
| `noise_floor` | 10 | Bins below this count are zeroed before moment computation |
| `realtime_dark_history_size` | 4 | Max dark observations kept in the realtime predictor's ring buffer (§7.4) |
| `PEDESTAL_HEIGHT` | 64.0 | ADC zero-light bias subtracted from mean in the uncorrected stream |
| `ADC_GAIN` | (1024−64)/11000 ≈ 0.0873 DN/e⁻ | Sensor ADC gain used for shot-noise correction |
| `CAMERA_GAIN_MAP` | [16,4,2,1,1,2,4,16] | Per-camera analog gain (index = cam position 0–7) |
| `EXPECTED_HISTOGRAM_SUM` | 2,457,606 | Required total count per valid frame (1920 × 1280 px + 6 sentinel) |
| `FRAME_ID_MODULUS` | 256 | Firmware 8-bit counter rollover period |
| `FRAME_ROLLOVER_THRESHOLD` | 128 | Max forward delta before rollover is detected |
| Histogram bins | 1024 | Bin indices k ∈ {0, 1, …, 1023} |
| Cameras per sensor | 8 | OV2312 cameras per sensor module |
| Max sensors | 2 | Left and right modules |

---

## 15. Output data types

```python
@dataclass
class Sample:
    side: str               # "left" or "right"
    cam_id: int             # 0–7
    frame_id: int           # raw 8-bit firmware counter value
    absolute_frame_id: int  # monotonic unwrapped counter
    timestamp_s: float      # sensor-reported timestamp
    row_sum: int            # total histogram counts after noise floor decimation
    temperature_c: float    # sensor temperature
    mean: float             # uncorrected: max(0, raw_μ₁ − 64) (pedestal removed)
                            # corrected:   μ̃₁ = fm.u1 − dark_u1 (pedestal cancels)
    std_dev: float          # σ  or σ̃  (variance is shift-invariant; pedestal has no effect)
    contrast: float         # K  or K̃  (σ / mean, using pedestal-adjusted mean)
    bfi: float              # BFI on [0, 10] scale
    bvi: float              # BVI on [0, 10] scale
    is_corrected: bool      # False = uncorrected stream; True = corrected batch

# Backward-compat aliases
RealtimeSample = Sample
CorrectedSample = Sample

@dataclass
class CorrectedBatch:
    dark_frame_start: int   # absolute_frame_id of D_prev
    dark_frame_end: int     # absolute_frame_id of D_next
    samples: list[Sample]   # chronological, is_corrected=True
                            # includes D_prev, excludes D_next
```

---

## 16. Reduced mode (per-side averaging)

**Implementation:** `omotion/ScanWorkflow.py` — activated by setting `ScanRequest.reduced_mode = True`.

**Purpose:** In the clinical "reduced mode", the application displays only two aggregated traces — left and right — rather than individual per-camera plots.  When reduced mode is enabled, the SDK averages all active cameras on each side into single left/right BFI and BVI values at the scan workflow level.  This ensures that both the live display and the corrected CSV contain only the averaged data, so the data files a clinician reviews match what was shown on screen during the scan.

### 16.1 What changes vs. normal mode

| Aspect | Normal mode | Reduced mode |
|---|---|---|
| Uncorrected samples emitted | One `Sample` per camera per frame (up to 16 at 40 Hz) | One `Sample` per **side** per frame (up to 2 at 40 Hz) |
| Corrected batches emitted | One `Sample` per camera per dark-frame interval | One `Sample` per **side** per dark-frame interval |
| Corrected CSV columns | `bfi_l1..l8, bfi_r1..r8, bvi_l1..l8, bvi_r1..r8, mean_*, contrast_*, temp_*` (80 data columns) | `bfi_left, bfi_right, bvi_left, bvi_right` (4 data columns) |
| Raw histogram CSV | Unchanged (per-camera histograms written normally) | Unchanged |
| Science pipeline internals | Unchanged (per-camera BFI/BVI computed normally) | Unchanged — averaging happens **after** the science pipeline, in `ScanWorkflow` |

### 16.2 Architecture — where averaging occurs

The science pipeline (§1–§9) is completely unmodified in reduced mode.  It continues to compute per-camera BFI/BVI from histograms as described in the preceding sections.  The averaging is performed in two interception points inside `ScanWorkflow.start_scan()._worker()`:

```
SciencePipeline (per-camera BFI/BVI)
         │
         ├── on_uncorrected_fn ──► Reduced uncorrected accumulator (§16.3)
         │                             └── emits 1 averaged Sample per side per frame
         │
         └── on_corrected_batch_fn ──► Reduced corrected handler (§16.4)
                                          ├── averages per side, writes reduced CSV row
                                          └── emits averaged CorrectedBatch to UI
```

This design keeps the science pipeline generic and testable.  The averaging logic is isolated in `ScanWorkflow` and only activates when `reduced_mode=True`.

### 16.3 Uncorrected sample averaging (live display path)

Each uncorrected `Sample` emitted by the science pipeline is intercepted by `_on_uncorrected_sample()`.  Instead of forwarding it immediately to the UI, the workflow buffers it in a per-frame, per-side accumulator:

**Accumulator key:** `(side, absolute_frame_id)` — e.g. `("left", 1542)`.

**Accumulator entry:**
```python
{
    "bfi_sum": float,       # running sum of BFI values
    "bvi_sum": float,       # running sum of BVI values
    "count": int,           # number of cameras that have contributed
    "timestamp_s": float,   # timestamp from the first contributing sample
    "frame_id": int,        # raw firmware frame counter
    "abs_frame_id": int,    # unwrapped monotonic frame counter
    "side": str,            # "left" or "right"
}
```

**Flush condition:** The entry is flushed (averaged and emitted) when `count` reaches the number of active cameras for that side.  The active camera count is derived from the camera bitmask at scan start — e.g. mask `0x66` (binary `01100110`) has 4 bits set, so `expected_count = 4`.

**Averaged sample:** When flushed, a synthetic `Sample` is constructed:

```python
Sample(
    side    = entry["side"],
    cam_id  = 0,                                    # sentinel — not a real camera
    bfi     = entry["bfi_sum"] / entry["count"],    # arithmetic mean of all cameras
    bvi     = entry["bvi_sum"] / entry["count"],    # arithmetic mean of all cameras
    mean    = 0.0,                                  # not meaningful in reduced mode
    std_dev = 0.0,
    contrast = 0.0,
    temperature_c = 0.0,
    is_corrected = False,
    ...
)
```

This sample is then forwarded to the UI via `on_uncorrected_fn`.

**Stale entry eviction:** After flushing, any accumulator entries for the same side whose `absolute_frame_id` is more than 5 frames behind the current frame are deleted.  This prevents unbounded memory growth if a camera drops frames and an entry never reaches its expected count.

**Effect on the UI:** The QML `ReducedPlotView` component previously performed its own JavaScript-level averaging of individual camera samples.  With reduced mode enabled at the SDK level, it now receives pre-averaged samples (one per side, `cam_id=0`) and can display them directly.

### 16.4 Corrected batch averaging (CSV + corrected display path)

Each `CorrectedBatch` emitted by the science pipeline is intercepted by `_on_corrected_batch()`.  Two things happen:

#### 16.4.1 Corrected CSV writing

Samples in the batch are accumulated into `corrected_by_frame` (the same dict used in normal mode, but with a different structure).  Each frame entry carries an `_accum` dict keyed by side:

```python
corrected_by_frame[frame_key] = {
    "timestamp_s": float,
    "_accum": {
        "left":  {"bfi_sum": float, "bvi_sum": float, "count": int},
        "right": {"bfi_sum": float, "bvi_sum": float, "count": int},
    },
}
```

A frame row is flushed to the CSV when **all expected sides** have their full camera count.  For example, if both left and right sensors are active with 4 cameras each, the row is written when `left.count >= 4` and `right.count >= 4`.

The CSV row contains:
```
frame_id, timestamp_s, bfi_left, bfi_right, bvi_left, bvi_right
```

where each value is the arithmetic mean of all contributing cameras for that side:
```
bfi_left = left.bfi_sum / left.count
```

**No mean, contrast, or temperature columns are written** — these per-camera metrics are not meaningful after side-level averaging.

#### 16.4.2 Averaged CorrectedBatch to UI

After CSV writing, the batch samples are grouped by `(side, absolute_frame_id)` and averaged the same way.  A new `CorrectedBatch` is constructed with one averaged `Sample` per side per frame (`cam_id=0`, `is_corrected=True`) and emitted to the UI via `on_corrected_batch_fn`.

### 16.5 What is NOT changed in reduced mode

- **Raw histogram CSVs** — per-camera histogram data continues to be written to `*_left_mask*.csv` and `*_right_mask*.csv` files at full resolution.  These files are the ground-truth record and can be reprocessed offline if needed.
- **Science pipeline** — all per-camera computations (frame classification, dark subtraction, shot-noise correction, BFI/BVI calibration) run identically.
- **Telemetry CSV** — console temperature, PDC, and safety data are unaffected.

### 16.6 Activation

Reduced mode is activated by the application through the `ScanRequest` dataclass:

```python
req = ScanRequest(
    subject_id="...",
    duration_sec=43200,
    left_camera_mask=0x66,
    right_camera_mask=0x66,
    data_dir="...",
    disable_laser=False,
    reduced_mode=True,          # ← enables per-side averaging
)
```

The bloodflow application sets `reduced_mode=True` when its `appConfig.reducedMode` flag is enabled (clinical mode).  The flag is read from `config/app_config.json` and passed through `motion_connector.py` to the SDK at scan start.
