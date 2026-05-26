# Pipeline-Cutover Review & Test Plan

> **Goal:** Slowly, deliberately review what's about to land on `next` and
> ship it today. Three review buckets: **data pipeline**, **data sinks**,
> **ancillary**.
>
> **Branches → merge target:**
> - openmotion-sdk: `feature/pipeline-cutover` → `next` (**134 commits, +28217/−5263 LOC**)
> - openmotion-bloodflow-app: `feature/pipeline-sinks` → `next` (11 commits, +1792/−627 LOC)
> - openmotion-test-app: `feature/pipeline-sinks` → `next` (1 commit, +13/−1 LOC)
>
> **⚠️ Scope note.** The SDK branch contains more than just the pipeline
> cutover — it also includes issue #92 (Scan database sink), per-frame PDC
> telemetry research, dark-drift study, PDC investigation, and various
> design docs. All of this would land on `next` together.
>
> **⚠️ Coexisting sink modules.** Both old (`omotion/Sink.py` /
> `omotion/CsvSink.py` / `omotion/ScanDBSink.py`, issue #92) and new
> (`omotion/pipeline/sinks.py`, cutover) sink modules ship in this branch.
> ScanWorkflow imports the new ones in the active path but still has a
> top-level import of the old `CsvSink`. **Should be confirmed dead and
> deleted before merge** — see Bucket B.

---

## Bucket A — Data Pipeline Rework

**Scope:** The new `omotion/pipeline/` package replacing
`MotionProcessing.SciencePipeline`. Same math, new architecture:
stage-based pipeline with `FrameBatch` as the typed carrier, fed by a
`Source`, producing events that dispatch to `Sink`s via named channels.

### Files

**New pipeline package** (`omotion/pipeline/`, ~13 files):
- `batch.py` — `FrameBatch` + `BatchEvent` types (`LiveEmit`, `IntervalClosed`, `DarkIntegrityWarning`, `StencilFallback`, `TriggerStateEvent`)
- `pipeline.py` — `Pipeline` + `Stage` protocol
- `runner.py` — `ScanRunner` channel dispatch, `dispatch_event`, `_failed_sinks` skip
- `sources.py` — `Source` protocol, `LiveUsbSource` (with the orphan-stream / sentinel-wait / parallel-teardown fixes), `CsvReplaySource`, `DbReplaySource`
- `tee.py` — `Tee` with `filter` + `max_duration_s`
- `factory.py` — `default_pipeline()` composition
- `pedestal.py` — `SensorPedestals` + `pedestal_for_fw` heuristic
- `stages/classify.py` — frame-id unwrapping + warmup/dark/light/stale classification; uses `batch.side_ids[i]`
- `stages/noise_floor.py` — threshold subtraction
- `stages/moments.py` — vectorized u1/u2/std
- `stages/pedestal_sub.py` — `display_mean = max(0, mean_raw - pedestal)`
- `stages/dark.py` — **the big one.** `HybridRealtimePredictor` (avg-of-3 u1 + linear-extrap std), `PendingInterval`, `LinearInterpolation` (interpolates **variance** — critical), `DarkFrameQuadraticStencil` (4-point), terminal-flush, first-dark NaN propagation
- `stages/shot_noise.py` — Poisson-variance subtraction with NaN propagation (today's fix)
- `stages/bfi_bvi.py` — affine map `(1 - (K - C_min) / (C_max - C_min)) * 10`
- `stages/side_avg.py` — reduced-mode per-side averaging
- `stages/rolling_avg.py` — rolling window over light frames

**Workflow integration:**
- `omotion/ScanWorkflow.py` — `start_scan` builds Source → Pipeline → ScanRunner; `_duration_guard` teardown order (`stop_trigger → 0.5s → disable_camera → 0.35s → close`); `cancel_scan` mirrors same order; `_emit_trigger_event` dispatches `TriggerStateEvent`; `last_scan_error` / `last_scan_canceled` props
- `omotion/StreamInterface.py` — `start_streaming` recovers from stale alive threads; `stop_streaming` logs orphan; `_stream_loop` bounded `data_queue.put`
- `omotion/MotionProcessing.py` — collapsed 1941 → 752 LOC; retains only `parse_histogram_stream` + dataclasses + constants

**Docs:**
- `docs/SciencePipeline.md` — rewritten (1010 LOC) for the new architecture
- `docs/Architecture.md` — updated layer diagram + module table
- `docs/superpowers/specs/2026-05-22-data-pipeline-rearchitecture-design.md` — original design doc
- `docs/superpowers/plans/2026-05-22-data-pipeline-rearchitecture.md` — execution plan
- `docs/superpowers/plans/2026-05-22-pipeline-cutover.md` — PR-2/3 plan
- `docs/superpowers/specs/2026-05-22-pipeline-cutover-design.md` — cutover design

**Tests (pure software, fast):**
- All `tests/test_pipeline/test_*` (24 files)
- `tests/test_motion_processing_shim.py`
- `tests/test_scan_workflow.py` (some scan workflow integration)
- Deleted: `test_pipeline_csv.py`, `test_pipeline_perf.py`, `test_rolling_average.py`, `test_realtime_dark_*` (covered by new tests)

### Review checklist

1. Read `docs/SciencePipeline.md` end-to-end first. It's the canonical algorithm reference — every stage's behavior should be derivable from it.
2. Walk through `omotion/pipeline/stages/dark.py` carefully — this is where ~half the pipeline complexity lives. Verify against §7 + §8 of the spec.
3. Verify `omotion/pipeline/factory.py:default_pipeline()` stage order matches the spec exactly.
4. Sanity-check the threading model in `runner.py` + `sources.py` (LiveUsbSource has per-side reader threads + a batch queue with a sentinel).
5. Look at `ScanWorkflow.py` `_duration_guard` and `cancel_scan` to confirm both end-of-scan paths use the same teardown order.

### Automated tests for this bucket

```
cd openmotion-sdk
pytest tests/test_pipeline tests/test_scan_workflow.py tests/test_motion_processing_shim.py -q
```
Expected: all green. Includes 24 pipeline-specific files + scan workflow integration.

### Bench scenarios

| # | Scenario | Pass criteria |
|---|---|---|
| A1 | Long scan (~60 s), reduced mode | Corrected CSV produced. BFI/BVI sensible (≈0–1 on phantom). No NaN/Inf in CSV. **No spike on first sample.** |
| A2 | Short scan (<15 s, under one `dark_interval`) | Corrected CSV produced via terminal-dark flush. No "no terminal dark frame found" warnings. |
| A3 | Scan with one side disconnected | The connected side produces data correctly. No misroute of frames to the wrong side. |
| A4 | Replay a saved raw CSV via `scripts/test_pipeline_real_scan.py` | Per-cell BFI/BVI diff against the legacy output within tolerance. |
| A5 | Back-to-back scans (3+ without app restart) | Each scan produces both raw CSVs (left + right), each runs through to corrected CSV. No "Stream already running" or "not READY" errors. |
| A6 | Press Stop mid-scan | Stop returns in ~1 s. No 3-second hang. Notes file + corrected CSV present. |

---

## Bucket B — Data Sinks Rework

**Scope:** Two layers of sink work landed on this branch. Both should be
reviewed together because the new pipeline sinks supersede the older ones
and dead code likely needs deletion.

### B.1 — Legacy issue-#92 sinks (older, pre-cutover)

These were the foundation that the new pipeline sinks were built on.
**Need to confirm they're dead and delete them.**

| File | Status |
|---|---|
| `omotion/Sink.py` | Old Sink protocol. Imported only by `omotion/CsvSink.py` + `omotion/ScanDBSink.py` (the old ones). |
| `omotion/CsvSink.py` | Old CSV sink. Imported by `omotion/ScanWorkflow.py:19` — check if that import is vestigial (the active path uses the new one). |
| `omotion/ScanDBSink.py` | Old DB sink. Same situation. |
| `omotion/ScanDatabase.py` | The SQLite schema + ORM. **Still used by the new pipeline ScanDBSink** — keep. |
| `omotion/SessionPlayback.py` | Reads a past scan back from the DB. Independent of which sink wrote the data — keep. |
| `docs/ScanDatabase.md` + `docs/ScanDatabase-HardwareVerification.md` | Keep — schema docs are still relevant. |
| `docs/superpowers/specs/2026-04-14-scan-db-sink-design.md` + plan | Historical record. Keep or move out of `next` — your call. |

**Action items for this section:**
1. Confirm `omotion/Sink.py`, `omotion/CsvSink.py`, `omotion/ScanDBSink.py` are not used by any production code path or test.
2. If dead, **delete them** before merging. Otherwise reviewers will be confused by two sets of files with the same names.
3. Find and delete the dead `from omotion.CsvSink import CsvSink` in `ScanWorkflow.py:19`.

### B.2 — New pipeline-based sinks (active)

| File | What to verify |
|---|---|
| `omotion/MotionInterface.py` | New ctor kwargs: `data_dir`, `scan_db_path`, `operator_id`. `contact_quality_workflow` lazy property. Auto-injection of default sinks (`CsvSink` + `ScanDBSink`) at `start_scan` based on those kwargs. |
| `omotion/pipeline/sinks.py` | `Sink` Protocol. `ScanMetadata` fields. `CsvSink` (raw + corrected, post-#44 naming `{scan_id}_{subject_id}.csv` + `..._raw.csv`). `ScanDBSink` (SQLite writes via `omotion/ScanDatabase.py`). `QtUiSink`. Helpers `_scalar_or_blank` / `_scalar_or_default` for missing pdc/tcm/tcl. `_source_side_indices` fallback. |
| `omotion/ScanWorkflow.py` (sink-relevant parts) | `ScanRequest` reshape: `sinks: list`, `skip_default_storage`, `raw_save_max_duration_s`. Default sink injection. |
| `omotion/CalibrationWorkflow.py` | `_CalibrationCollectorSink` subscribes to "final" + "live" channels, exposes `corrected_samples` + `dark_samples`. `_run_subscan_capture` drives the sink and reads `last_scan_error` / `last_scan_canceled`. |
| `omotion/ContactQualityWorkflow.py` | NEW. `_ContactQualitySink` subscribes to "live" channel, evaluates **DN-scale** thresholds (`display_mean`), produces per-camera `CamCQResult`. Rolling window logic. AMBIENT_LIGHT vs POOR_CONTACT semantics. |

**bloodflow-app sinks** (in `motion_connector.py`):

| Sink | Channel | What it does |
|---|---|---|
| `_LivePlotSink` | `live` | Emits per-frame Qt signals (`scanBfiSampled`, etc.) to the plot. Skips NaN. Dropout watchdog gate. |
| `_FinalBatchSink` | `final` | Emits `scanCorrectedBatch` signal for the post-interval plot update. |
| `_TriggerStateSink` | `diagnostics` | Mirrors `_trigger_state` + `_trigger_on_mono` + `_trigger_cumulative_s`. |
| `_CompletionSink` | (none — lifecycle) | Calls `_on_pipeline_complete` from `on_complete()`. |

**bloodflow-app**:
- `main.py` — `MotionInterface(data_dir=…, scan_db_path=…, operator_id=…)` init
- `pages/BloodFlow.qml` — scan-timer `triggerState` binding, CQ-modal warning ordering fix
- `pages/scan/ScanRunner.qml` — stage rename `post` → `finish`

**test-app**:
- `motion_singleton.py` — MotionInterface init kwargs only

### Automated tests

```
cd openmotion-sdk
pytest tests/test_pipeline/test_sinks_protocol.py \
       tests/test_pipeline/test_csv_sink.py \
       tests/test_pipeline/test_scan_db_sink.py \
       tests/test_pipeline/test_qt_ui_sink.py \
       tests/test_motion_interface.py \
       tests/test_calibration_workflow.py \
       tests/test_contact_quality_workflow.py \
       tests/test_scan_workflow.py \
       tests/test_scan_database.py \
       tests/test_scan_db_sink.py -q
```

### Bench scenarios

| # | Scenario | Pass criteria |
|---|---|---|
| B1 | Default scan with `MotionInterface(data_dir=…, scan_db_path=…)` | Both CSV + DB write. CSV files named `{ts}_{subject}.csv`, `..._raw.csv`. DB has rows in `session_raw` + `session_corrected`. |
| B2 | `data_dir=None` | No CSV written; DB still writes (if path set). |
| B3 | `scan_db_path=None` | No DB writes; CSVs still written. |
| B4 | `ScanRequest(raw_save_max_duration_s=10)` for 30 s scan | Raw CSV stops at ~10 s; corrected continues. |
| B5 | `ScanRequest(raw_save_max_duration_s=0)` | No raw CSV written; corrected still written. |
| B6 | `ScanRequest(raw_save_max_duration_s=None)` | Raw CSV covers the entire scan. |
| B7 | Calibration cycle | Produces calibration JSON. Collector sink got both light + dark samples. No "no corrected samples" error. |
| B8 | Contact-quality check | Returns per-camera DN values. Thresholds 3.0/15.0 DN behave correctly (cover a cam → poor_contact; ambient light into a covered cam → ambient_light). |
| B9 | History modal | Dropdown shows `YYYYMMDD_HHMMSS_<subject>` entries. Visualize buttons enabled when corrected CSV exists. Both legacy + new visualizers work. |
| B10 | Scan notes + duration | `..._notes.txt` written. Duration line shows "(trigger)" not "(wall-clock)". |
| B11 | SessionPlayback (DB read-back) | Open a past scan from DB via `MotionInterface` → frames come back, BFI/BVI match the original. |

---

## Bucket C — Ancillary

**Scope:** Things that aren't strictly pipeline or sinks but landed on the
branch alongside them.

### C.1 — TEC ADC conversion math moved to SDK (today's work)

The user flagged: "the telemetry pipeline has some bits in the motionconfig
now." Concretely, TEC handling is now split across three files:

| File | Role |
|---|---|
| `omotion/console_telemetry_conversions.py` (SDK, NEW) | ADC→°C/V/A math. V_REF=2.459, R_1/R_2/R_3/R_s, lazy-loaded RT lookup. |
| `omotion/models/10K3CG_R-T.csv` (SDK, NEW) | Thermistor R-T lookup table. |
| `pyproject.toml` (SDK) | `models/*.csv` added to package data. |
| `motion_connector.py:tec_status` (bloodflow-app) | Uses SDK helpers instead of inline math. Dropped local `V_REF`/`R_1`/`R_2`/`R_3`/`R_s` constants and dead `_data_RT` loader. |
| `motion_config.py:load_tec_params` (bloodflow-app, **unchanged**) | Reads `tec_params.json` for `TEC_VOLTAGE_DEFAULT` — separate from ADC conversion. **Worth confirming nothing should have been touched here.** |

### C.2 — Per-frame PDC telemetry (research design only, no SDK code yet)

| File | What |
|---|---|
| `docs/superpowers/specs/2026-05-20-per-frame-pdc-telemetry-design.md` | Design doc |
| `docs/superpowers/plans/2026-05-20-per-frame-pdc-telemetry.md` | Execution plan |
| `omotion/ConsoleTelemetry.py` (+140 LOC) | Some telemetry plumbing — verify this is read-side only and doesn't break the deleted-telemetry refactor |
| `omotion/MotionConsole.py` (+49 LOC) | Companion changes |
| `tests/test_get_pdc_buffer.py` (NEW) | Test for PDC buffer accessor |
| `tests/test_console_telemetry_unit.py` (NEW) | Unit test |

**Action item:** Verify these don't reintroduce the pipeline telemetry path
we deleted. Should be console-side only.

### C.3 — Dark-drift study (research, no production code)

| File | What |
|---|---|
| `data-processing/dark-drift-study/*` | Plots, simulation scripts, findings doc, online estimators study, integration proposal |
| `scripts/plot_dark_drift.py` | Plot script |

Self-contained research. **Decide:** ship with the cutover or move out.
No impact on the runtime pipeline.

### C.4 — PDC investigation (research)

| File | What |
|---|---|
| `pdc-investigation/*` | Plots, scripts, FINDINGS.md |

Same as C.3 — self-contained, no impact. Decide whether to ship.

### C.5 — App-level changes (bloodflow-app)

| File | What |
|---|---|
| `pages/BloodFlow.qml` | Scan timer `triggerState` binding (today's fix), CQ modal warning order (today's fix) |
| `components/SettingsModal.qml` | "Mean / C" → "Mean / Contrast" label |
| `pages/scan/ScanRunner.qml` | Stage rename `post` → `finish` |
| `docs/superpowers/specs/2026-05-25-dark-histogram-correction-research-design.md` | Spec for separate research project. 234 LOC, docs only. |
| `docs/superpowers/plans/2026-05-25-dark-histogram-correction-research.md` | Plan for the same. 1072 LOC, docs only. |

### C.6 — Top-level repo housekeeping

| File | What |
|---|---|
| `AGENTS.md` (SDK, NEW) | Agent-collaboration docs |
| `CLAUDE.md` (SDK, NEW) | The repo-specific Claude guide we set up |
| `omotion/__init__.py`, `omotion/config.py` | Small additions |

### Bench scenarios for this bucket

| # | Scenario | Pass criteria |
|---|---|---|
| C1 | Open Settings → TEC Status panel | Shows TEC temp / setpoint / current / voltage with correct values. Identical readings to legacy build (within rounding). |
| C2 | Open Settings → label check | "Mean / Contrast" not "Mean / C". |
| C3 | Build the wheel (`python -m build`) | `omotion/models/10K3CG_R-T.csv` is included in the wheel. Install in a fresh venv → `tec_thermistor_voltage_to_celsius(0.6)` returns a sensible °C value. |

---

## Pre-merge gates

Before opening the SDK → `next` PR (and the two app PRs → `next`):

1. **Full SDK fast test suite green:**
   ```
   cd openmotion-sdk
   pytest tests/ -m "not destructive and not slow and not sensor and not fpga and not imu and not hardware" -q
   ```
   Expected: 306 passed, ~37 skipped, ~136 deselected.

2. **All bench scenarios above (A1–A6, B1–B11, C1–C3) pass on hardware.**

3. **Bucket B housekeeping decision made:**
   - Delete dead `omotion/Sink.py`, `omotion/CsvSink.py`, `omotion/ScanDBSink.py` (if confirmed unused), OR document explicitly that they coexist.
   - Decide on dead import in `omotion/ScanWorkflow.py:19`.

4. **Bucket C housekeeping decision made:**
   - Dark-drift study + PDC investigation + per-frame PDC telemetry research docs: ship with cutover or split out.

5. **No uncommitted local prefs:**
   - `git -C openmotion-bloodflow-app diff config/app_config.json` should show only intentional default changes (your local `dataDirectory`, `writeRawCsv`, `developerMode` prefs should NOT go in).

6. **Branch state:**
   - SDK `feature/pipeline-cutover` clean working tree.
   - bloodflow-app `feature/pipeline-sinks` clean working tree.
   - test-app `feature/pipeline-sinks` clean working tree.

## Merge plan

**Option 1: Direct to `next`** (simplest):
1. PR SDK `feature/pipeline-cutover` → `next`. Review the 134-commit diff in the PR.
2. PR bloodflow-app `feature/pipeline-sinks` → `next`.
3. PR test-app `feature/pipeline-sinks` → `next`.

**Option 2: Staged via `next-next`** (was the original plan when the cutover started):
1. PR SDK `feature/pipeline-cutover` → `next-next` (exists on origin). Review there.
2. After SDK merge to `next-next`, PR `next-next` → `next`.
3. PRs apps → `next`.

For an SDK release tag, you still need `next → main` per `docs/Releasing.md`.
