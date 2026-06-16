# EFT Timestamp Repair — Design Spec

**Date:** 2026-06-05
**Branch:** `feature/eft-timestamp-repair` (off `next`)
**Scope:** R1 (drift-free timestamp correction), R2 (NaN-fill missing frames), R3 (coalesced logging), plus quality column and pipeline reordering.

---

## 1. Problem statement

EMI (electrical fast transients) during clinical scans corrupts the device `timestamp_s` field — the MCU's TIM5 timer is sampled inside an interrupt that EMI fires at the wrong moment. The `frame_id` (camera-embedded) is reliable. The pipeline currently trusts the device timestamp for dark-interval interpolation, skips missing frames silently, and flushes corrected output in completion order, causing row-ordering corruption (FM-R).

**Verified model:** `frame_id` is reliable, `timestamp_s` is the EMI-corrupted signal. Grouping by `frame_id` is correct (already done). The fix is repairing the time axis and representing missing data.

---

## 2. Design decisions (settled)

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Divergence detection tolerance | 8 ms | Tight enough to catch the observed 10ms EMI timestamp offset, loose enough to avoid rewriting normal ~2ms timestamp quantization |
| Quality representation | String column (`"ok"`, `"ts_corrected"`, `"nan_filled"`) | Human-readable, extensible, simple downstream consumption |
| Stage position | After FrameClassification, before Tee("raw") moves before it | Raw CSV stays untouched; all downstream stages see corrected timestamps |
| NaN-fill scope | Pipeline-wide (synthetic FrameBatch rows) | NaN propagates naturally through all stages; no special-casing downstream |
| Live path | Bounded look-ahead buffer (16 frames max) | Bounded memory + latency; only during divergence |
| Logging | Coalesced per-window WARNING + end-of-scan summary | Spec requirement; no per-frame spam |
| Raw CSV | Byte-identical to current | Tee("raw") fires before repair stage |

---

## 3. Pipeline order (new)

```
FrameClassificationStage
Tee("raw", filter=ft != "stale", max_duration_s=...)    ← moved before repair
TimestampRepairStage                                     ← NEW
NoiseFloorStage
MomentsStage
PedestalSubtractionStage
DarkCorrectionStage
ShotNoiseCorrectionStage
BfiBviStage
DarkFrameHoldStage
SideAverageStage
Tee("live", filter=not_warmup_or_stale)
```

The raw tee captures the unmodified `timestamp_s` and original frame count (no synthetic rows). Everything after TimestampRepairStage sees corrected timestamps and a complete frame grid.

---

## 4. TimestampRepairStage — detailed design

### 4.1 Inputs and outputs

**Reads:** `batch.abs_frame_ids`, `batch.frame_ids`, `batch.timestamp_s`, `batch.side_ids`, `batch.cam_ids`

**Writes:** `batch.timestamp_s` (in-place overwrite for corrected frames), `batch.quality` (new field, `(N,)` str array)

**May expand batch:** Inserts synthetic NaN-fill rows for missing `abs_frame_id` gaps. The returned batch may be larger than the input.

### 4.2 Per-scan state

| State | Type | Purpose |
|-------|------|---------|
| `_nominal_period` | float | Running estimate of true frame period (init 25.0 ms, converges from good frames) |
| `_last_good` | dict[(side, cam) → (abs_frame_id, timestamp_s)] | Last known good frame per camera — the left anchor for interpolation |
| `_buffer` | list[BufferedFrame] | Look-ahead buffer holding frames during a bad run (max 16 frames) |
| `_in_bad_run` | bool | Whether we're currently accumulating a bad run |
| `_window_onset` | (abs_frame_id, timestamp_s) or None | Start of current bad window (for logging) |
| `_scan_stats` | {windows, frames_corrected, frames_nan_filled} | End-of-scan summary counters |

### 4.3 Divergence detection

A frame is **bad** if EITHER condition fires:

**Condition 1 — Timestamp deviation:**
```
expected_Δt = (this_abs_frame_id - last_good_abs_frame_id) × nominal_period
actual_Δt = this_timestamp_s - last_good_timestamp_s
bad = |actual_Δt - expected_Δt| > 0.008   (8 ms)
```

The `last_good` reference is per `(side, cam)` since each camera has its own unwrapped `abs_frame_id` stream.

**Condition 2 — In-packet frame_id disagreement:**
Within each batch, group rows by `timestamp_s` (exact float equality — rows from the same firmware packet share the exact same MCU timer sample). For each timestamp group, compute `set(frame_ids)`. If `len(set) > 1`, all rows in that group are bad.

**Combined:** A frame is bad if condition 1 OR condition 2 fires. Both conditions feed into the same buffer/re-anchoring logic.

**Implementation order within `process()`:**
1. First pass: group rows in this batch by `timestamp_s`, identify disagreeing groups (condition 2). Build a set of row indices marked bad.
2. Second pass: for each row, check condition 1 against the per-camera `_last_good`. OR with condition 2 result.
3. Route bad frames into buffer, good frames through (flushing buffer if a bad run just ended).

### 4.4 Nominal period estimation

The true frame period is ~25.02 ms (not exactly 25.0). Using a fixed 25.0 ms would accumulate ~0.08%/scan drift on the interpolated timestamps. Instead:

- Initialize at 25.0 ms.
- On each good frame (condition 1 passes, condition 2 passes), update: `nominal_period = 0.99 × nominal_period + 0.01 × measured_Δt` (exponential moving average, slow convergence to avoid noise).
- Only update from single-step frames (frame_id gap == 1) to avoid polluting with gap-spanning intervals.

This converges within ~100 good frames (2.5s) and tracks any slow drift in the firmware's actual trigger rate.

### 4.5 Re-anchoring interpolation

When a good frame arrives after a bad run (the "re-anchor"):

```
t_start = last_good_timestamp_s       (left anchor)
t_end   = this_good_timestamp_s       (right anchor)
fid_start = last_good_abs_frame_id
fid_end   = this_good_abs_frame_id

For each buffered frame with abs_frame_id = fid:
    corrected_t = t_start + (fid - fid_start) / (fid_end - fid_start) × (t_end - t_start)
    frame.timestamp_s = corrected_t
    frame.quality = "ts_corrected"
```

This distributes the buffered frames evenly between the two real device timestamps, scaled by frame_id count. No drift accumulates because both anchors are real device timestamps.

### 4.6 NaN-fill for missing frames (R2)

After re-anchoring (or during good-frame processing), check for gaps in `abs_frame_id`:

```
If this_abs_frame_id - prev_abs_frame_id > 1:
    For each missing fid in (prev + 1 ... this - 1):
        Synthesize a FrameBatch row:
            cam_ids = [cam_id]
            frame_ids = [fid & 0xFF]
            abs_frame_ids = [fid]
            side_ids = [side_idx]
            raw_histograms = zeros((1, 2, 8, 1024))  — all bins zero
            temperature_c = zeros((1, 2, 8))
            timestamp_s = interpolated from surrounding good timestamps
            frame_type = "light"
            quality = "nan_filled"
            pdc, tcm, tcl = None (or NaN if arrays exist on batch)
```

Synthetic rows have zero-count histograms. MomentsStage divides by zero count and produces NaN mean/std. All downstream stages propagate NaN naturally. The corrected CSV/DB gets complete rows with NaN values for missing data.

**Gap detection scope:** Per `(side, cam)`. A gap in one camera doesn't mean other cameras are missing — they may have different `abs_frame_id` sequences due to the per-camera unwrapper. However, under the verified model, all cameras on a side share the same true frame sequence. So a gap detected on any camera on a side means that frame is missing for that camera specifically.

### 4.7 Bounded look-ahead buffer

- Maximum size: 16 frames (400 ms at 40 Hz).
- If the buffer fills without finding a re-anchor, force-flush it: interpolate using `nominal_period` from the last good frame (slightly less accurate than anchored interpolation, but prevents unbounded memory/latency). Log a WARNING noting the forced flush.
- On `on_scan_stop`: flush any remaining buffer using the same forced-flush logic. The scan is over — there's no future anchor coming.

### 4.8 Batch boundary handling

A bad run can span multiple batches. The stage is stateful — `_buffer` and `_in_bad_run` persist across `process()` calls. When a new batch arrives:

- If `_in_bad_run` is True, continue accumulating into the buffer.
- If a good frame arrives in the new batch, it re-anchors and flushes the buffer from the previous batch(es).
- Flushed frames are prepended to the current batch's output (maintaining frame_id order).

The returned batch from `process()` may be larger than the input (NaN fills + flushed buffer frames prepended).

---

## 5. FrameBatch changes

### 5.1 New field

```python
@dataclass
class FrameBatch:
    ...existing fields...

    # (N,) str — per-frame quality flag. Set by TimestampRepairStage.
    # "ok" = timestamp passed through unchanged
    # "ts_corrected" = timestamp replaced by re-anchoring interpolation
    # "nan_filled" = synthetic row for missing frame (zero histogram)
    quality: Optional[np.ndarray] = None
```

**Field ownership:** TimestampRepairStage (sole writer).

### 5.2 No other FrameBatch changes

The stage rewrites `timestamp_s` in place (already a Source field). It may expand the batch (insert rows), which means all array fields grow in lockstep. The stage must produce correctly-sized arrays for all existing fields when synthesizing NaN-fill rows.

---

## 6. DarkCorrectionStage — quality threading

### 6.1 _LightSample

Add a `quality: str` field (default `"ok"`). When DarkCorrectionStage buffers a light frame into `PendingInterval._light`, it reads `batch.quality[i]` and stores it.

### 6.2 CorrectedFrame / EnrichedCorrectedFrame

```python
@dataclass
class CorrectedFrame:
    ...existing fields...
    quality: str = "ok"

@dataclass
class EnrichedCorrectedFrame:
    ...existing fields...
    quality: str = "ok"
```

LinearInterpolation and the enrichment path pass `quality` through from `_LightSample` to `CorrectedFrame` to `EnrichedCorrectedFrame`. No logic change — just plumbing.

### 6.3 Dark frame stencil quality

The dark frame's stencilled row gets `quality = "ok"` (it's a real frame with a computed value, not a corrected or missing one). If its stencil inputs are NaN (from NaN-filled neighbours), the stencil produces NaN values — which is correct (missing data propagates).

---

## 7. CsvSink changes

### 7.1 Corrected CSV headers

**Normal mode (83 columns now):**
```
frame_id, timestamp_s, bfi_l1..bfi_r8, bvi_l1..bvi_r8, mean_l1..mean_r8,
contrast_l1..contrast_r8, temp_l1..temp_r8, quality
```

**Reduced mode (7 columns now):**
```
frame_id, timestamp_s, bfi_left, bfi_right, bvi_left, bvi_right, quality
```

### 7.2 Row accumulation

The `_corrected_acc[abs_id]` entry gets a `"quality"` field. When frames arrive for an `abs_id`:
- If any contributing frame has `quality != "ok"`, the row's quality is the "worst" value: `"nan_filled"` > `"ts_corrected"` > `"ok"`.
- In practice, all cameras for a given `abs_frame_id` will share the same quality (they were all corrected or all missing together), so this is a simple passthrough.

### 7.3 Row ordering

With the NaN grid, every `abs_frame_id` in the scan gets a row. Frames arrive at the sink in `abs_frame_id` order (the repair stage guarantees this by flushing in frame_id order). The accumulator completes frames in order → rows are written in order → no FM-R.

### 7.4 Raw CSV

Untouched. Tee("raw") fires before the repair stage.

---

## 8. ScanDBSink changes

### 8.1 Schema addition

The `session_data` table gets a `quality TEXT DEFAULT 'ok'` column. Applied via:
```sql
ALTER TABLE session_data ADD COLUMN quality TEXT DEFAULT 'ok'
```
On first write to an existing DB without the column, catch the OperationalError and run the ALTER. New DBs created by `ScanDatabase.create_session()` include the column in the CREATE TABLE.

### 8.2 Row writing

`_consume_live` and `_consume_side` write `quality` from the batch/sample. Existing rows in older DBs without the column get the DEFAULT and are unaffected.

---

## 9. factory.py changes

```python
def default_pipeline(...) -> Pipeline:
    stages = [
        FrameClassificationStage(discard_count=discard_count, dark_interval=dark_interval),
    ]

    # Raw tee BEFORE timestamp repair — raw CSV sees untouched device timestamps
    if raw_save_max_duration_s is None or raw_save_max_duration_s > 0:
        stages.append(
            Tee("raw", filter=lambda ft: ft != "stale", max_duration_s=raw_save_max_duration_s)
        )

    stages.append(
        TimestampRepairStage(
            tolerance_s=0.008,
            max_buffer_frames=16,
        )
    )

    stages.extend([
        NoiseFloorStage(threshold=noise_floor_threshold),
        MomentsStage(),
        PedestalSubtractionStage(pedestals=pedestals),
        DarkCorrectionStage(...),
        ...
    ])
```

---

## 10. R3 — Coalesced logging

**Logger:** `logging.getLogger("openmotion.sdk.pipeline.stages.timestamp_repair")`

### 10.1 Per-window WARNING

Emitted when a bad run closes (re-anchor found or buffer force-flushed):

```
Misalignment window: frames {onset_fid}–{end_fid} (t={onset_t:.2f}–{end_t:.2f}s),
{n_corrected} frames re-timestamped, {n_nan} frames NaN-filled
```

### 10.2 End-of-scan summary

Emitted at `on_scan_stop`:

```
Scan summary: {n_windows} misalignment window(s), {total_corrected} frames re-timestamped,
{total_nan} frames NaN-filled ({pct:.1f}% of scan affected)
```

### 10.3 Clean scan

Zero log output. No warnings, no summary (or optionally a DEBUG-level "no misalignments detected" for traceability — but the spec says zero output on clean scans, so omit entirely).

---

## 11. Testing strategy

### 11.1 Clean-scan regression (golden baseline)

Replay `owYWB8TN` (mask66) and `owYZ7T66 13:04` (maskC3) through the modified pipeline. Assert corrected CSV is numerically identical to current output for all pre-existing columns (the new `quality` column is an addition). All quality values must be `"ok"`. Zero NaN rows, zero log warnings.

### 11.2 Degraded-scan structural assertions

For all 8 degraded scans (including 3x `owEFTTEST1` from `final_tests/`):
- Gap-free `abs_frame_id` grid in corrected CSV
- NaN where data was absent
- Monotonic non-decreasing `timestamp_s`
- No drift: at re-anchor points, corrected ts matches device ts within 1 ms
- Quality column populated correctly
- Raw CSV byte-identical to input
- Coalesced logging (one WARNING per window + summary)

### 11.3 Unit tests

- `test_timestamp_repair_stage.py`: divergence detection (condition 1 + 2), buffer flush, re-anchoring math, NaN-fill insertion, nominal period estimation, batch boundary spanning
- Integration with DarkCorrectionStage: verify NaN-filled frames produce NaN corrected output
- Integration with CsvSink: verify quality column appears, NaN rows present

### 11.4 Test data location

- Clean: `C:\Users\ethan\Projects\eft-testing\scans\` (owYWB8TN, owYZ7T66 13:04)
- Degraded: `C:\Users\ethan\Projects\eft-testing\scans\` (owM8T7HS, owSPZMD1, owF0JJO2 ×2, owYZ7T66 13:05) + `C:\Users\ethan\Projects\eft-testing\final_tests-20260603T234709Z-3-001\final_tests\` (owEFTTEST1 ×3)

---

## 12. Out of scope

- Camera dropout (FM-5) — separate, non-EMI issue
- Teardown 151 ms glitch (FM-6) — benign, ignored
- Firmware debounce changes — separate sensor-fw task
- Per-frame logging — explicitly prohibited by spec
- Raw CSV modifications — spec requires byte-identical
