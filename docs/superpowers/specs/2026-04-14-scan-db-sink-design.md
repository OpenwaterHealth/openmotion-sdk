# Scan Database Sink for the Corrected Data Pipeline

**Status:** Design approved; plan not yet written.
**Date:** 2026-04-14
**Repo:** openmotion-sdk

## Summary

Make the SQLite database implementation currently living in `stream-db/` the default data endpoint of the corrected data pipeline in `ScanWorkflow`. The `_corrected.csv` file becomes optional and runs alongside the database when enabled. Raw per-frame histograms can also be written to the database, behind a separate flag that defaults off.

## Decisions

- DB writes are **opt-in via a flag but default ON**. CSV output remains opt-in and stays as-is for back-compatibility.
- `session_data` is always populated when the DB sink is enabled. `session_raw` is gated behind its own flag (`write_raw_to_db`, default `False`).
- `scan_db.py` is re-homed into the `omotion` package as `omotion/ScanDatabase.py`. The standalone tools (`db_browser.py`, `db_validator.py`, `importer.py`, `sensor_module_simulator.py`) stay in `stream-db/` but import from `omotion`. The old `stream-db/scan_db.py` is deleted.
- One shared database file per `data_dir` (default: `<data_dir>/scans.db`), with one new session row per scan.
- `session_label` mirrors CSV naming: `f"{ts}_{subject_id}"`.
- `session_notes` is populated from a new `ScanRequest.notes` field (caller supplies whatever they also write to the sidecar notes.txt).
- `session_meta` is a JSON blob that captures everything the SDK knows about the scan — request fields, active cameras, firmware/FPGA versions, SDK flags, and host info — best-effort, with missing values stored as `null`.
- Implementation shape: a new `ScanDBSink` class exposed from `omotion`, wired in by the `MOTIONInterface.start_scan` façade via the existing corrected-batch callback plus a new raw-frame callback. `ScanWorkflow` itself stays unaware of the database.

## Architecture

### New / moved files

- **`omotion/ScanDatabase.py`** — `ScanDatabase` class moved verbatim from `stream-db/scan_db.py`. Same schema (`sessions`, `session_raw`, `session_data`), same public API (`create_session`, `insert_raw_frame`, `insert_session_data`, `close_session`), same `compress_raw_hist` behavior with transparent decompression on read.
- **`omotion/ScanDBSink.py`** — new. Thin adapter owning one DB connection and one open session row for the duration of a scan. Public surface:

  ```python
  class ScanDBSink:
      def __init__(
          self,
          db_path: str,
          *,
          write_raw: bool = False,
          compress_raw_hist: bool = True,
          raw_batch_size: int = 200,
      ): ...

      def open(
          self,
          *,
          label: str,
          start_ts: float,
          notes: str,
          meta: dict,
      ) -> int: ...  # returns session_id

      def on_raw_frame(
          self,
          side: str,
          cam_id: int,
          frame_id: int,
          timestamp_s: float,
          hist: bytes,
          temp: float,
          sum_counts: int,
          tcm: float,
          tcl: float,
          pdc: float,
      ) -> None: ...  # no-op when write_raw=False

      def on_corrected_batch(self, batch: "CorrectedBatch") -> None: ...

      def close(self, end_ts: float) -> None: ...  # idempotent

      @property
      def insert_errors(self) -> int: ...
  ```

### Changed files

- **`omotion/ScanWorkflow.py`** — `start_scan` gains two new optional callbacks:
  - `on_raw_frame_fn: Callable[..., None] | None = None` — invoked from the per-side raw writer thread after a row has been fed to the science pipeline.
  - `on_scan_start_fn: Callable[[str, float], None] | None = None` — invoked once at the top of `_worker`, passing the `ts` string (`YYYYMMDD_HHMMSS`) and the wall-clock `session_start` captured immediately after. This is the single authoritative start point used for both CSV naming and DB `session_start`.

  No DB code lives in this file.
- **`omotion/Interface.py`** — `MOTIONInterface.start_scan` is the wiring point. When `request.write_to_db` is `True`, it:
  1. Resolves `db_path = request.db_path or os.path.join(request.data_dir, "scans.db")`.
  2. Constructs `ScanDBSink(db_path, write_raw=request.write_raw_to_db)` (does *not* open the session yet — the scan thread hasn't produced `ts` yet).
  3. Provides an `on_scan_start_fn` that calls `sink.open(label=f"{ts}_{subject_id}", start_ts=session_start, notes=request.notes, meta=meta_dict)`. `meta_dict` is built at this point so firmware/FPGA version getters run once the scan is actually underway.
  4. Chains `sink.on_raw_frame`, `sink.on_corrected_batch`, and a wrapped `on_complete_fn` (which calls `sink.close(time.time())` first, then the caller's original callback) into the kwargs it forwards to `ScanWorkflow.start_scan`.
  5. If the user passed their own `on_scan_start_fn` / `on_raw_frame_fn` / `on_corrected_batch_fn` / `on_complete_fn`, the sink's callback is invoked first, then the caller's.
- **`omotion/__init__.py`** — re-exports `ScanDatabase` and `ScanDBSink`.

### New `ScanRequest` fields

```python
write_to_db: bool = True
write_raw_to_db: bool = False
db_path: str | None = None   # defaults to <data_dir>/scans.db
notes: str = ""              # becomes session_notes
```

### Deleted / migrated

- `stream-db/scan_db.py` is deleted.
- `stream-db/db_browser.py`, `db_validator.py`, `importer.py`, `sensor_module_simulator.py` change their imports from `from scan_db import ScanDatabase` to `from omotion import ScanDatabase` (or the fully qualified module path).

## Data flow

1. `MOTIONInterface.start_scan(request, ...)` resolves the DB path, constructs the sink, and registers an `on_scan_start_fn` that will open the session once the scan thread produces its canonical `ts` and `session_start`.
2. When `_worker` fires `on_scan_start_fn(ts, session_start)`, the sink is opened with session metadata:
   - `session_label` = `f"{ts}_{subject_id}"`
   - `session_start` = the wall-clock value passed by `_worker`, captured at the same instant as `ts`
   - `session_notes` = `request.notes`
   - `session_meta` (JSON) includes: `subject_id`, `duration_sec`, `expected_size`, `fps` (40), `left_camera_mask`, `right_camera_mask`, decoded `active_left_cams` / `active_right_cams`, `disable_laser`, `sdk_version`, `console_fw_version`, `sensor_fw_versions` per side, `fpga_versions` per side, `host`, and `sdk_flags` (`write_raw_csv`, `write_corrected_csv`, `write_telemetry_csv`, `write_to_db`, `write_raw_to_db`). Every value is best-effort; anything the interface can't supply is stored as JSON `null`, never raised.
3. `start_scan` injects three callbacks ahead of any caller-supplied ones:
   - `on_raw_frame_fn` → `sink.on_raw_frame`
   - `on_corrected_batch_fn` → `sink.on_corrected_batch`
   - `on_complete_fn` → calls `sink.close(time.time())` first, then the caller's callback.
4. `ScanWorkflow._worker` runs unchanged except: inside `_row_handler`, after feeding the science pipeline, it invokes `on_raw_frame_fn` (if any) with the raw-frame fields already in hand.
5. `ScanWorkflow`'s existing `_on_corrected_batch` already forwards to `on_corrected_batch_fn`. `ScanDBSink.on_corrected_batch` opens a single transaction, iterates each `Sample` in the batch, and writes one `session_data` row per (cam, side, timestamp) using the sample's `bfi`, `bvi`, `contrast`, `mean`. `session_raw_id` is left `NULL` — ScanWorkflow does not track which raw-row a corrected sample derived from.
6. On scan completion (success, error, or cancel), the wrapped `on_complete_fn` calls `sink.close(end_ts=time.time())`. `close()` is idempotent and swallows secondary errors so it never masks the real scan error.

## Concurrency model

- `ScanDBSink` creates its SQLite connection lazily inside `open()` with `check_same_thread=False`. A single `threading.Lock` serializes all writes.
- Corrected batches fire on the pipeline thread at roughly one batch per camera per interval (≤ 1 Hz per camera); `session_data` inserts happen inside a single transaction per batch.
- Raw frames fire on the per-side raw writer thread at 40 Hz per active camera. `on_raw_frame` appends to an in-memory buffer (separate per instance, guarded by the same lock) and flushes via `executemany` once the buffer reaches `raw_batch_size` (default 200). A buffer flush also happens at the start of each `on_corrected_batch` call (cheap synchronization point) and again during `close()`.

## Failure handling

- **Sink open failure** (bad path, permission denied, schema migration failure) raises from `MOTIONInterface.start_scan` before any cameras are enabled, matching today's behavior when a CSV file can't be opened.
- **Per-row insert failure** inside a callback is logged at WARNING, increments `sink.insert_errors`, and does not re-raise. This matches the current "log and continue" CSV-writer behavior; a broken DB row must not tear down the writer or pipeline thread.
- **Close failure** is logged and swallowed. A partial session row with `session_end = NULL` is acceptable and is how a crashed scan is recognized.

## Testing

**Unit tests (no hardware):**

- `tests/test_scan_database.py` — round-trip inserts and reads for all three tables; `compress_raw_hist=True` writes compressed blobs that read back identical; `session_meta` JSON survives nested dicts, `None` values, and unicode.
- `tests/test_scan_db_sink.py`:
  - `open()` creates exactly one session row with the expected label, start, notes, and meta JSON.
  - `on_corrected_batch` with a synthetic `CorrectedBatch` (mocked Samples, 4 cameras × 2 sides) writes one `session_data` row per Sample with correct `bfi`/`bvi`/`contrast`/`mean`/`timestamp_s`/`cam_id` and side encoding (`0` = left, `1` = right).
  - `on_raw_frame` is a no-op when `write_raw=False`; writes one row per call when `True`; batched inserts flush on the next corrected batch and on `close()`.
  - `close()` is idempotent — calling twice does not raise and does not overwrite `session_end` with a later value.
  - Per-row insert failure (forced by a connection-level error hook) increments `insert_errors` and does not raise.
  - Thread-safety: four concurrent threads firing `on_raw_frame` produce the expected total row count.

**Integration test (no hardware):**

- `tests/test_scan_workflow_db_integration.py` — drives `ScanWorkflow` with a stub `MOTIONInterface` emitting a canned histogram stream (reuse the pattern from existing workflow tests):
  - Default flags: `<data_dir>/scans.db` is created, exactly one session row exists with `session_end` set, `session_data` row count matches the expected cams × corrected-intervals product.
  - With `write_raw_to_db=True`: `session_raw` row count equals frames-emitted across both sides.
  - With `write_to_db=False`: no DB file is created.

**Regression / equivalence:**

- `tests/test_db_matches_corrected_csv.py` — run the same canned scan with both CSV and DB sinks enabled, then assert the DB's `session_data` rows reproduce the `_corrected.csv` rows cell-for-cell (after side encoding and column-name mapping). This is the load-bearing test that proves the DB really is the same endpoint as the CSV.
- Existing `tests/test_corrected_csv_output.py` stays green unchanged.

**Manual / smoke:**

- Run `stream-db/db_validator.py` (now importing from `omotion`) against a sim-generated DB.
- Open the resulting `scans.db` in `db_browser.py` and visually confirm a session's per-camera traces render.

## Out of scope

- Schema migrations: the database version is whatever the current `stream-db/scan_db.py` creates. No versioning/migration framework is introduced.
- Multi-process writers on the same `scans.db`: one scan at a time per DB file. SQLite's default locking handles the degenerate case by raising; no WAL tuning is included here.
- Retroactive import of existing CSV-only scans. The existing `importer.py` continues to handle that path.
- Changes to `MotionProcessing.py` or the science pipeline itself.
