# Scan Database

OpenMOTION supports persisting scan output to a per-installation SQLite database in
addition to (or eventually instead of) the per-scan CSVs the SDK has always written.
This document covers the rationale, configuration, schema, lifecycle, and how to
inspect the data.

## TL;DR

* **One file per installation** (typically `<scan_data>/scans.db`) that accumulates
  every scan as its own row in a `sessions` table.
* **Three data tables** — `sessions` (one row per scan), `session_data` (corrected
  per-camera per-frame BFI/BVI/contrast/mean), and `session_raw` (optional raw
  histogram blobs).
* **Opt-in at SDK construction.** Default off. Apps pass `db_path=...` to
  `MotionInterface(...)`; when set, every scan writes to that file. When unset,
  the SDK behaves byte-for-byte the same as it always has — no DB code runs.
* **CSVs are unaffected.** The DB is additive. CSVs are still written; the two
  endpoints agree cell-for-cell.

## Configuration

The DB is enabled at SDK construction:

```python
from omotion import MotionInterface

iface = MotionInterface(db_path="/path/to/scans.db")
iface.start()
```

`db_path=None` (the default) means no sink is constructed; the SDK behaves
identically to today and no DB file is ever created.

The OpenMOTION Bloodflow app exposes this via two `app_config.json` keys:

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `scanDbEnabled` | bool | `false` | Toggles the SDK sink on. **Startup-only** — changing it at runtime requires an app restart because the path is fixed at `MotionInterface` construction. When `true`, the path is `<output_base>/scan_data/scans.db`. |
| `writeRawData` | bool | `false` | Master raw-histogram persistence switch. Drives both raw-CSV writes and raw-DB writes when their respective targets are active. Live-toggleable from Settings → Developer → "Save raw data". |
| `writeRawDataDurationSec` | float \| null | `null` | Cap for how long raw data is recorded per scan. Applies to **both** the raw CSV writer and the DB `session_raw` sink — once any writer-thread deadline fires, both targets stop accepting new raw frames so they capture the same window. `null` means full scan duration. **Important:** without this cap, a multi-hour scan with `writeRawData=true` and `scanDbEnabled=true` will write one `session_raw` row per camera per frame for the entire duration (roughly 320 rows/s × scan length × ~250 B of compressed histogram payload + ~80 B row meta), so a 12-hour scan produces ~7–8 GB of raw rows in the DB on top of the corrected data. |

## Schema

`omotion.ScanDatabase` creates four tables and three indexes on first open. The
schema lives in [`omotion/ScanDatabase.py`](../omotion/ScanDatabase.py); this is
a reference rendering.

### `sessions`

One row per scan, written at the moment `ScanWorkflow._worker` produces the
canonical `YYYYMMDD_HHMMSS` timestamp (which is also the prefix of the CSV
filenames). The row gets `session_end` populated when the scan completes
(success or cancel).

```sql
CREATE TABLE sessions (
    id             INTEGER PRIMARY KEY,
    session_label  TEXT    NOT NULL,    -- "{ts}_{subject_id}" — matches CSV naming
    session_start  REAL    NOT NULL,    -- wall-clock seconds (time.time()) at scan start
    session_end    REAL,                -- wall-clock seconds at scan end
    session_notes  TEXT,                -- free text from ScanRequest.notes
    session_meta   TEXT                 -- JSON blob, see "Session metadata" below
);
```

#### Session metadata

`session_meta` is a JSON blob populated at scan start by
`MotionInterface._wrap_kwargs_with_db_sink._build_meta()`. Stored as text and
re-parsed on read.

```json
{
  "subject_id": "owUCEKHF",
  "duration_sec": 3600,
  "expected_size": 32837,
  "fps": 40,
  "left_camera_mask": 102,
  "right_camera_mask": 102,
  "active_left_cams": [2, 3, 6, 7],
  "active_right_cams": [2, 3, 6, 7],
  "disable_laser": false,
  "sdk_version": "1.6.0",
  "console_fw_version": "1.5.8-rc.2",
  "console_hw_id": "23004c00065133333735383300000000",
  "left_fw_version": "1.5.4-dev.0",
  "right_fw_version": "1.5.4-dev.0",
  "left_hw_id": "2d004e00065133333735383300000000",
  "right_hw_id": "28004f00065133333735383300000000",
  "sdk_flags": {
    "write_raw_csv": true,
    "write_corrected_csv": true,
    "write_telemetry_csv": true,
    "write_raw_to_db": true
  }
}
```

Firmware versions / hardware IDs are filled in best-effort — `MotionInterface`
wraps every accessor in a `_safe_call`, so disconnected handles produce `null`
placeholders rather than failing the scan.

### `session_data`

The corrected per-camera per-frame data — one row per `Sample` emitted by the
science pipeline. This is the load-bearing analytical surface.

```sql
CREATE TABLE session_data (
    id               INTEGER PRIMARY KEY,
    session_id       INTEGER NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    session_raw_id   INTEGER REFERENCES session_raw(id) ON DELETE SET NULL,
    cam_id           INTEGER NOT NULL,                      -- 0-indexed (0..7)
    side             INTEGER NOT NULL CHECK(side IN (0, 1)),-- 0 = left, 1 = right
    timestamp_s      REAL    NOT NULL,                      -- seconds since scan start
    bfi              REAL,
    bvi              REAL,
    contrast         REAL,
    mean             REAL
);

CREATE INDEX idx_session_data_session_time ON session_data(session_id, timestamp_s);
CREATE INDEX idx_session_data_session_cam  ON session_data(session_id, side, cam_id, timestamp_s);
```

Notes:

* All `REAL` fields are stored rounded to 6 decimals (project-wide precision
  policy, matches the corrected CSV writer). Anything beyond 6 decimals is
  measurement noise.
* `timestamp_s` is **seconds since scan start**, not firmware-clock seconds. The
  per-scan baseline is captured by `parse_histogram_stream`'s `t0_normalizer`
  at the first sample observed across both sides, so every output of the same
  scan agrees on the time origin.
* `cam_id` is **0-indexed**. The corrected CSV column names use 1-indexed
  cameras (e.g. `bfi_l3` is camera 3 (1-indexed) = `cam_id=2` (0-indexed)).
* Left and right sensors have **independent firmware clocks**. The shared
  `t0_normalizer` captures whichever side fires first; the other side's per-row
  timestamp can therefore be offset by a few ms from its sibling's. Cell values
  agree exactly with the merged CSV row format, but joining a CSV row to the
  DB on `timestamp_s` directly will only match the side that defined `t0`.

### `session_raw`

Optional per-frame raw histogram blobs (1024 × `uint32` bins per frame). Only
populated when `ScanRequest.write_raw_to_db=True` (gated upstream by the app's
`writeRawData` master flag).

```sql
CREATE TABLE session_raw (
    id           INTEGER PRIMARY KEY,
    session_id   INTEGER NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    side         TEXT    NOT NULL CHECK(side IN ('left', 'right')),
    cam_id       INTEGER NOT NULL,                  -- 0-indexed
    frame_id     INTEGER NOT NULL,                  -- per-side firmware frame id
    timestamp_s  REAL    NOT NULL,                  -- seconds since scan start
    hist         BLOB    NOT NULL,                  -- 1024 × uint32, optionally zlib-compressed
    temp         REAL,                              -- camera temperature, °C
    sum          INTEGER,                           -- sum of histogram bins
    tcm          REAL    NOT NULL DEFAULT 0,        -- console TCM telemetry
    tcl          REAL    NOT NULL DEFAULT 0,        -- console TCL telemetry
    pdc          REAL    NOT NULL DEFAULT 0         -- console PDC telemetry
);

CREATE INDEX idx_session_raw_session_time     ON session_raw(session_id, timestamp_s);
CREATE INDEX idx_session_raw_session_cam_time ON session_raw(session_id, side, cam_id, timestamp_s);
```

`side` is stored as `TEXT` ('left' / 'right') in `session_raw` but as
`INTEGER` (0 / 1) in `session_data`. This is historical — the raw path
came first and preserved the SDK's string convention; `session_data` uses
an integer for compactness. `ScanDatabase.get_raw_frame` /
`ScanDatabase.stream_raw_frames` return the side as text either way.

`hist` is `bytes` on the way in. If the database's `compress_raw_hist`
setting is on (default for new DBs), the blob is zlib-compressed at write
time and transparently decompressed by `get_raw_frame` /
`stream_raw_frames`. Compression ratios of ~10–20× are typical for histograms
that have only a few dozen non-zero bins.

### `database_settings`

A tiny key/value table for per-DB settings that have to outlive a single
process (currently just `compress_raw_hist`). Set on first open; checked on
every subsequent open so a mismatch raises rather than silently corrupting
the read-back of older blobs.

```sql
CREATE TABLE database_settings (
    key    TEXT PRIMARY KEY,
    value  TEXT NOT NULL
);
```

## Lifecycle

```
MotionInterface.start_scan(request)
        │
        │  if self._db_path is not None:
        │     kwargs = self._wrap_kwargs_with_db_sink(request, kwargs)
        │
        ▼
ScanWorkflow.start_scan(request, on_scan_start_fn=..., on_raw_frame_fn=...,
                                  on_corrected_batch_fn=..., on_complete_fn=...)
        │
        ▼  _worker thread spawned
ScanWorkflow._worker
        │
        │  ts = datetime.now().strftime("YYYYMMDD_HHMMSS")
        │  on_scan_start_fn(ts, time.time())     ──► ScanDBSink.open(label, start_ts, notes, meta)
        │                                              └── INSERT INTO sessions ...
        │
        │  ┌───────────────────────────────────────────────────────────────────┐
        │  │ per-side writer threads run parse_histogram_stream                │
        │  │   ├── normalize sample.timestamp_s to scan-start via t0_normalizer│
        │  │   ├── csv_writer.writerow(sample.to_csv_row(...))                 │
        │  │   └── on_row_fn(...)  ─► p.enqueue(...) (science pipeline)        │
        │  │                       ─► on_raw_frame_fn(...) ─► ScanDBSink       │
        │  │                                                  .on_raw_frame() │
        │  │                                                  └── batched     │
        │  │                                                       executemany│
        │  │                                                       on flush   │
        │  └───────────────────────────────────────────────────────────────────┘
        │
        │  science_pipeline emits CorrectedBatch
        │     └── on_corrected_batch_fn(batch)
        │            └── ScanDBSink.on_corrected_batch(batch)
        │                  ├── flushes pending raw buffer
        │                  └── insert_session_data_rows(rows)
        │
        ▼  scan ends (cancel, error, or duration reached)
on_complete_fn(result)
        │
        ▼
ScanDBSink.close(end_ts)
        ├── flush any remaining raw frames
        ├── UPDATE sessions SET session_end = end_ts WHERE id = self._session_id
        └── close the connection
```

Two write-amplification properties worth knowing:

* **Raw frames buffer at the sink, not at the DB.** `on_raw_frame_fn` fires
  ~40 × (active cameras) times per second per side. `ScanDBSink` appends each
  call to an in-memory list and flushes via `insert_raw_frames` (which uses
  `executemany` inside one transaction) when the buffer hits `raw_batch_size`
  (default 200 frames, ~5 s of data per side) — or when `on_corrected_batch`
  fires (the corrected pipeline is a natural sync point) — or on `close()`.
* **Corrected rows write per-batch.** A dark-frame interval is 600 frames
  / 15 s at 40 Hz, so `on_corrected_batch` fires every ~15 s with up to
  600 × `cams_per_side` × 2 rows in a single `executemany`.

## Sink internals (`omotion.ScanDBSink`)

The sink is a thin adapter between `ScanWorkflow`'s callback surface and
`ScanDatabase`'s SQL inserts. It owns:

* one `ScanDatabase` connection (one file per scan);
* one open session row (one `INSERT INTO sessions` per scan);
* a `threading.Lock` to serialize the raw-frame buffer between writer threads.

```python
class ScanDBSink:
    def __init__(self, db_path, *, write_raw=False, compress_raw_hist=True,
                 raw_batch_size=200): ...
    def open(self, *, label, start_ts, notes, meta) -> int: ...
    def close(self, end_ts: float) -> None: ...
    def on_corrected_batch(self, batch: CorrectedBatch) -> None: ...
    def on_raw_frame(self, side, cam_id, frame_id, timestamp_s, hist,
                     temp, sum_counts, tcm, tcl, pdc) -> None: ...
```

* `close()` is idempotent (second call is a no-op; doesn't bump `session_end`).
* Callbacks raise `RuntimeError` if invoked before `open()`.
* Both callbacks swallow exceptions internally and count failures via
  `sink.insert_errors`, so a transient DB error doesn't propagate into the
  worker thread and abort the scan.

## Querying

The DB is a plain SQLite file. Any tooling that speaks SQLite works.

### `sqlite3` CLI

```sh
sqlite3 /path/to/scans.db <<'SQL'
.headers on
.mode column

SELECT id, session_label,
       session_end - session_start AS duration_s,
       json_extract(session_meta, '$.subject_id')         AS subject,
       json_extract(session_meta, '$.console_fw_version') AS console_fw
FROM sessions
ORDER BY session_start DESC
LIMIT 10;

-- BFI/BVI per camera for the most recent scan
SELECT side, cam_id, COUNT(*) n, MIN(bfi) bfi_lo, MAX(bfi) bfi_hi
FROM session_data
WHERE session_id = (SELECT MAX(id) FROM sessions)
GROUP BY side, cam_id
ORDER BY side, cam_id;
SQL
```

### `omotion.ScanDatabase` (Python)

```python
from omotion import ScanDatabase

db = ScanDatabase(db_path="/path/to/scans.db")
try:
    for s in db.iter_sessions():
        print(s["session_label"], s["session_meta"])

    sid = ...  # session id of interest
    for batch in db.stream_session_data(sid):
        for row in batch:
            print(row["side"], row["cam_id"], row["timestamp_s"], row["bfi"])

    for batch in db.stream_raw_frames(sid):
        for row in batch:
            print(row["frame_id"], len(row["hist"]), row["sum"])
finally:
    db.close()
```

`session_meta` is automatically JSON-decoded into a Python dict by
`get_session` / `iter_sessions`. `hist` blobs are transparently decompressed
by `get_raw_frame` / `stream_raw_frames` based on the DB's
`compress_raw_hist` setting.

### GUI browser

The repo ships a PyQt-based browser under `stream-db/db_browser.py`:

```sh
python stream-db/db_browser.py /path/to/scans.db
```

It enumerates sessions, lets you scrub through `session_data` columns, and
plots stored histograms from `session_raw`. Same dependencies as the rest of
the SDK plus `pyqtgraph`.

## Relationship to the CSV outputs

For every scan the sink runs, the SDK *also* writes the existing files:

| File | Source | Has frame_id | Time origin |
| --- | --- | --- | --- |
| `{ts}_{subject}_left_mask{XX}_raw.csv` | per-side raw stream | yes | scan start (`Sample.timestamp_s`) |
| `{ts}_{subject}_right_mask{XX}_raw.csv` | per-side raw stream | yes | scan start |
| `{ts}_{subject}.csv` (corrected) | science pipeline | yes (per row) | scan start, merged across sides |
| `{ts}_{subject}_telemetry.csv` (developerMode only) | telemetry poller | n/a | scan start |
| `scans.db / session_data` | science pipeline | **no** | scan start, per-side |
| `scans.db / session_raw` | per-side raw stream | yes | scan start, per-side |

The most surprising difference is the **timestamp origin within a single
frame**: the corrected CSV uses the minimum of left and right timestamps for
the merged row, while `session_data` stores per-side rows with each side's
own timestamp. The two sensors have independent firmware clocks, so the
per-side rows can differ by a few milliseconds even within the same
`frame_id`. Both representations are correct; the DB just preserves the
per-side resolution that the CSV flattens away.

## Notes on schema evolution

There is no schema version column yet. The DB-only mode shipped recently
(issue #92); when a forward-incompatible schema change becomes necessary,
the convention will be to write a `schema_version` key into
`database_settings` and have `ScanDatabase._init_schema()` perform a guarded
migration. Until then, existing DBs are safely opened by current code — the
`CREATE TABLE IF NOT EXISTS` and `CREATE INDEX IF NOT EXISTS` clauses make
every open idempotent.
