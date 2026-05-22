# Integration proposal — real-time dark correction in `SciencePipeline`

This proposes how to integrate the online dark estimators picked in
[`online_estimators.md`](online_estimators.md) into the SDK's
`SciencePipeline`, with minimum disruption to the existing batched
correction path that feeds the saved CSV / DB.

## Where it lives in the current pipeline

Current `SciencePipeline` (in `omotion/MotionProcessing.py`) emits two
streams via callbacks:

| callback | fires | content |
|---|---|---|
| `on_uncorrected_fn` | every non-dark frame, immediately | per-(side, cam, frame) `Sample` with `is_corrected=False`, no dark subtraction applied |
| `on_corrected_batch_fn` | once per dark-frame interval, after a second consecutive dark has been observed | a `CorrectedBatch` containing per-frame corrected samples for every frame in the just-closed interval |

The bloodflow app's live plots subscribe to `on_uncorrected_fn`.
The saved CSV and `session_data` rows are written from
`on_corrected_batch_fn` — that's what currently produces the "wait for
the next dark" lag.

The real-time estimator slots **between** these — it produces a
per-frame corrected `Sample` immediately, exactly as the existing
batched path would have eventually produced for the same frame, but
using *predicted* darks instead of *observed-and-interpolated* darks.

## Proposed API surface

### New callback

```python
on_realtime_corrected_fn: Callable[[Sample], None] | None = None
```

Fires per non-dark frame, immediately, **only when the predictor has
seen ≥ 2 darks for that camera** (the warmup gate). Receives a
`Sample` with `is_corrected=True` whose `bfi`, `bvi`, `mean`,
`contrast` are computed from `light_u1 − predicted_dark_u1` and
`light_std² − predicted_dark_std²`. Until warmup completes, this
callback simply doesn't fire — the existing `on_uncorrected_fn` is the
fallback for that ~15 s window.

This is **purely additive**. `on_uncorrected_fn` keeps emitting its
existing uncorrected stream. `on_corrected_batch_fn` keeps emitting
its existing batched-interpolation stream. The real-time path is a
third stream the bloodflow app can opt into for live plotting.

### Pipeline constructor change

```python
create_science_pipeline(
    ...
    on_realtime_corrected_fn=None,   # new
    realtime_dark_history_size=4,    # new (default = 4 darks per cam)
    ...
)
```

`realtime_dark_history_size` configures the ring buffer that holds
recent per-camera dark observations. Default 4 is the minimum that
supports both `avg-of-last-3-u1` (needs 3 most recent) plus one extra
slot to avoid race conditions during the moment a new dark is being
finalised. Exposed for tunability without re-shipping.

### Internal state per (side, cam)

```python
@dataclass
class _RealtimeDarkState:
    u1_history:  deque[float]   # last N u1 values
    std_history: deque[float]   # last N std values
    t_history:   deque[float]   # last N timestamps
    T_history:   deque[float]   # last N temperatures (unused by v1; reserved)
```

Total RAM: 16 cameras × 4 deques × 4 entries × 8 bytes ≈ 2 KB.

### Pipeline branches in `_on_sample`

Today's `_on_sample` already classifies each frame as dark / light
and runs the storage-and-batch path. The new code adds **two** small
edits around the existing logic:

```
on dark frame:
    [existing: store first/second moments for later batched correction]
    realtime_state[cam].push(t, u1, std, T)           # NEW

on light frame:
    [existing: compute uncorrected sample, fire on_uncorrected_fn]
    if on_realtime_corrected_fn is not None:           # NEW
        pred_u1  = predict_avg3(realtime_state[cam])
        pred_std = predict_linear(realtime_state[cam], t_l)
        if pred_u1 is not None and pred_std is not None:
            corrected = compute_corrected_sample(
                light_u1=u1, light_std=std,
                dark_u1=pred_u1, dark_std=pred_std,
            )
            on_realtime_corrected_fn(corrected)
```

`compute_corrected_sample` is the same arithmetic the batched path
already does — we just feed it a predicted dark instead of an
interpolated one. The function should be factored out of the existing
batched code path so both call sites share it.

### Bloodflow app glue

In `motion_singleton` / `motion_connector`:

```python
self._interface.start_scan(
    request,
    on_realtime_corrected_fn=self._on_realtime_corrected,  # new
    # existing callbacks unchanged
)
```

And a new connector method that mirrors the existing
`on_uncorrected_fn` handler but emits the *corrected* `Sample` to the
QML plot stream. The QML side doesn't need to change — it already
expects `Sample` shapes.

A config flag in `app_config.json`:

```json
"realtimeDarkCorrection": true
```

When `true`, the connector subscribes to the new callback and routes
its samples to the live plots. When `false`, behaviour matches today
(live plots show uncorrected). Default `true` once we've shipped one
release with it `false` for safety.

## Validation strategy

Three layers:

### 1. Unit tests

Add `tests/test_realtime_dark_estimator.py`:

* `test_u1_predict_avg_warmup_uses_what_is_available` — predictor
  returns mean of 1 dark with 1 in history, mean of 2 with 2, etc.
* `test_std_predict_linear_returns_none_before_two_darks` — explicit
  warmup behavior.
* `test_std_predict_linear_recovers_a_known_line` — feed darks at
  `std = a + b·t` for known a, b; assert prediction matches `a + b·t_query`
  exactly.
* `test_realtime_state_ring_buffer_caps_at_history_size` — push more
  than history-size; oldest is evicted.

### 2. Equivalence test against the long-scan fixture

A new test that drives the SciencePipeline with the existing fixture
and asserts the **real-time** stream matches the **batched** stream
to within the prediction RMSE we measured (0.020 bin u1, 0.022 bin std):

* For every light frame the pipeline emits, capture both streams.
* Per-(side, cam, frame), the difference between real-time-corrected
  and batched-corrected is bounded by `2 × RMSE` (≈ 0.04 bin u1,
  0.044 bin std) — the 2× margin covers worst-case prediction noise
  plus minor numerical differences between linear-interpolation and
  linear-extrapolation arithmetic.
* For BFI/BVI specifically, assert the two streams' per-cell deltas
  are within (say) 0.1% of typical magnitude.

This test is the load-bearing claim that the real-time stream is
trustworthy for clinical display.

### 3. Hardware verification

After landing, run the bloodflow app with `realtimeDarkCorrection:
true` and visually compare the live plots against the post-scan
batched-corrected CSV. They should look the same shape; any
disagreement is interesting.

## Rollout sequence

1. **SDK change in a single PR**: add the new callback, the state
   class, the per-frame branch, the helper extraction, and the unit +
   equivalence tests. No behavior change for callers that don't pass
   `on_realtime_corrected_fn`.
2. **App opt-in in a separate PR**: bloodflow-app subscribes to the
   new callback under the `realtimeDarkCorrection` flag (default
   `false`). Ship a dev-build for hardware testing.
3. **Default flip**: after one or two weeks of running with the flag
   on locally, change the app default to `true`. The batched path
   remains the saved CSV / DB ground truth for the foreseeable future.

## Out of scope (deferred)

* **Per-camera adaptive history depth.** A camera that's stable could
  use a deeper average; one that's still settling could use a shorter.
  Worth revisiting if precision becomes a problem.
* **Temperature-aware predictor.** The Phase 2 study showed `linear_T`
  doesn't work because of T quantisation noise. A future version could
  smooth T over multiple darks and use that as a predictor — gives
  better warmup behaviour at the cost of complexity. Not needed today.
* **Cross-camera correlation.** All 8 cameras share thermal coupling;
  a Kalman filter that pools information across them could reduce
  per-camera prediction noise. Probably not needed given current RMSEs
  are already at the per-frame measurement noise floor.
* **Predictor health monitoring.** Counter for "darks since last
  prediction", alarm if `predicted − observed` diverges past some
  threshold — useful for surfacing thermal anomalies or firmware
  issues. Easy to add later as a sink-style observer.

## Estimated scope

* SDK changes: ~1–2 days. New callback wire, ~30 lines of estimator
  code (the helpers are already prototyped in
  [`simulate_online_estimators.py`](simulate_online_estimators.py)),
  unit tests, equivalence test.
* App glue: ~half a day. New slot, config flag, QML route.
* Hardware verification: ~1 day of focused scanning + comparison.

Total: ~3–4 days end-to-end including verification.
