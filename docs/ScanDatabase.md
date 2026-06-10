# Scan Database

Open-Motion persists each scan's **corrected science record** to a
per-installation SQLite database. This document covers the rationale,
configuration, schema, lifecycle, and how to inspect the data.

## TL;DR

* **One file per installation** (typically `<scan_data>/scans.db`) that accumulates
  every scan as its own row in a `sessions` table.
* **Two data tables** — `sessions` (one row per scan) and `session_data`
  (corrected per-camera per-frame BFI/BVI/contrast/mean — the final-branch
  output of the science pipeline).
* **The DB stores corrected data only.** Raw histograms are written exclusively
  to the per-side raw CSVs (the pipeline's `Tee("raw")` → `CsvSink`); realtime
  ("live") values exist only on the in-process channels feeding the GUI and are
  never persisted. Databases created by older SDKs may contain a `session_raw`
  table and live-valued `session_data` rows — see "Legacy databases" below.
* **Opt-in at SDK construction.** Default off. Apps pass `scan_db_path=...` to
  `MotionInterface(...)`; when set, every scan writes to that file. When unset,
  the SDK falls back to writing the corrected CSV so there is always at least
  one persisted record.

## Configuration

The DB is enabled at SDK construction:

```python
from omotion import MotionInterface

iface = MotionInterface(scan_db_path="/path/to/scans.db")
iface.start()
```

`scan_db_path=None` (the default) means no DB sink is constructed; the SDK
forces the corrected CSV on instead.

The Open-Motion Bloodflow app exposes this via `app_config.json`:

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `scanDbEnabled` | bool | `false` | Toggles the SDK sink on. **Startup-only** — changing it at runtime requires an app restart because the path is fixed at `MotionInterface` construction. When `true`, the path is `<output_base>/scan_data/scans.db`. |
| `writeRawData` | bool | `false` | Raw-histogram persistence switch — gates the pipeline's `Tee("raw")`, which feeds the raw CSVs (the only raw target). |
| `writeRawDataDurationSec` | float \| null | `null` | Cap for how long raw data is recorded per scan (the raw tee's `max_duration_s`). `null` means full scan duration. |

## Configuration semantics — final-branch only

`session_data` holds the pipeline's **final-branch** output: the
interval-corrected values emitted on the `"final"` channel after each dark
interval closes (linear dark interpolation between bounding darks, shot-noise
correction, BFI/BVI calibration, and the quadratic-stencilled dark row — see
[`SciencePipeline.md`](SciencePipeline.md) §5.7). The realtime values shown
live in the GUI use a forward-predicted dark baseline and are deliberately
not stored.

Consequences:

* DB rows trail the scan by up to one dark interval (~15 s at defaults). An
  unclean shutdown (crash, power loss) loses that tail — accepted trade-off.
* The record is gapless at 40 Hz except warmup frames (1..`discard_count`)
  and the scan's terminal dark frame.

## Schema

`omotion.ScanDatabase` creates three tables and three indexes on first open. The
schema lives in [`omotion/ScanDatabase.py`](../omotion/ScanDatabase.py); this is
a reference rendering.

### `sessions`

One row per scan, written by `ScanDBSink.on_scan_start()`. The row gets
`session_end` populated when the scan completes (success or cancel).

```sql
CREATE TABLE sessions (
    id             INTEGER PRIMARY KEY,
    session_label  TEXT    NOT NULL,    -- "{scan_id}_{subject_id}" — matches CSV naming
    session_start  REAL    NOT NULL,    -- wall-clock seconds (time.time()) at scan start
    session_end    REAL,                -- wall-clock seconds at scan end
    session_notes  TEXT,                -- free text
    session_meta   TEXT                 -- JSON blob, see "Session metadata" below
);
```

#### Session metadata

`session_meta` is a JSON blob stamped at scan start by the pipeline's
`ScanDBSink.on_scan_start()` (see `omotion/pipeline/sinks.py`). Stored as
text and re-parsed on read.

```json
{
  "scan_id": "20260610_104500",
  "subject_id": "owUCEKHF",
  "operator": "tech1",
  "started_at_iso": "2026-06-10T10:45:00Z",
  "duration_sec": 3600,
  "data_semantics": "final",
  "sdk_flags": {
    "reduced_mode": false,
    "left_camera_mask": 102,
    "right_camera_mask": 102
  }
}
```

* `data_semantics: "final"` marks the session as holding final-branch
  (interval-corrected) values. **Sessions missing this key were written by
  older SDKs whose `session_data` held realtime (live-branch) values** —
  readers should treat the missing key as "legacy live-valued data".
* `sdk_flags.reduced_mode` drives the column layout chosen by
  `materialize_corrected_csv` (see "Playback" below).

### `session_data`

The corrected per-camera per-frame data — the load-bearing analytical surface.

```sql
CREATE TABLE session_data (
    id               INTEGER PRIMARY KEY,
    session_id       INTEGER NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    cam_id           INTEGER NOT NULL,                      -- 0-indexed (0..7); -1 = side average
    side             INTEGER NOT NULL CHECK(side IN (0, 1)),-- 0 = left, 1 = right
    frame_id         INTEGER NOT NULL DEFAULT -1,           -- absolute (unwrapped) frame id
    timestamp_s      REAL    NOT NULL,                      -- seconds since scan start
    bfi              REAL,
    bvi              REAL,
    contrast         REAL,
    mean             REAL,
    quality          TEXT DEFAULT 'ok'
);

CREATE INDEX idx_session_data_session_time  ON session_data(session_id, timestamp_s);
CREATE INDEX idx_session_data_session_cam   ON session_data(session_id, side, cam_id, timestamp_s);
CREATE INDEX idx_session_data_session_frame ON session_data(session_id, frame_id);
```

Notes:

* **`cam_id = -1` means "side average"** — the reduced-mode spatial average
  across the side's enabled cameras, emitted by `SideAverageStage` on the
  `"final"` channel. In reduced mode these are the *only* rows written; in
  normal mode only per-camera rows (`cam_id` 0..7) exist.
* Metric values are stored rounded (matching the corrected CSV writer's
  precision policy). Non-finite values (NaN) are stored as NULL; a frame with
  no finite metric at all is skipped.
* `timestamp_s` is **seconds since scan start**, not firmware-clock seconds.
* `cam_id` is **0-indexed**. The corrected CSV column names use 1-indexed
  cameras (e.g. `bfi_l3` is camera 3 (1-indexed) = `cam_id=2` (0-indexed)).
* Left and right sensors have **independent firmware clocks**. The shared
  t0 captures whichever side fires first; the other side's per-row
  timestamp can therefore be offset by a few ms from its sibling's.

### `database_settings`

A tiny key/value table for per-DB settings that have to outlive a single
process. Currently unused by the writer; reserved for a future
`schema_version` key.

```sql
CREATE TABLE database_settings (
    key    TEXT PRIMARY KEY,
    value  TEXT NOT NULL
);
```

## Lifecycle

`ScanDBSink` (in `omotion/pipeline/sinks.py`) is a channel-subscribed pipeline
sink — see [`SciencePipeline.md`](SciencePipeline.md) §8.2.

```
MotionInterface.start_scan(request)
        │
        │  ScanWorkflow auto-injects ScanDBSink when
        │  MotionInterface(scan_db_path=...) is set
        ▼
ScanRunner
        │  on_scan_start(meta)  ──► INSERT INTO sessions (label, meta...)
        │
        │  per closed dark interval (~15 s):
        │    IntervalClosed → "final" channel → ScanDBSink._consume_final()
        │       └── buffered rows → insert_session_data_rows (executemany)
        │
        ▼  scan ends (cancel, error, or duration reached)
on_complete()
        ├── flush remaining buffered rows
        ├── UPDATE sessions SET session_end = ...
        └── close the connection
```

* The sink is **critical**: if the DB can't be opened at scan start, the scan
  is aborted rather than run with no durable record
  (`ScanRunner.CriticalSinkError`).
* Rows buffer in memory (default 200) and flush via a single `executemany`
  transaction.
* `on_complete()` is idempotent.

## Querying

The DB is a plain SQLite file. Any tooling that speaks SQLite works.

### `sqlite3` CLI

```sh
sqlite3 /path/to/scans.db <<'SQL'
.headers on
.mode column

SELECT id, session_label,
       session_end - session_start AS duration_s,
       json_extract(session_meta, '$.subject_id')      AS subject,
       json_extract(session_meta, '$.data_semantics')  AS semantics
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
finally:
    db.close()
```

`session_meta` is automatically JSON-decoded into a Python dict by
`get_session` / `iter_sessions`.

## Relationship to the CSV outputs

| File | Source | Contents |
| --- | --- | --- |
| `{ts}_{subject}_{side}_mask{XX}_raw.csv` | `Tee("raw")` → `CsvSink` | **The only raw-histogram record.** Gated by `writeRawData` / capped by `writeRawDataDurationSec`. |
| `{ts}_{subject}.csv` (corrected) | `"final"` channel → `CsvSink` | Same final-branch values as `session_data`. Opt-in when the DB is active (`writeCorrectedCsv`); forced on when no DB is configured. |
| `{ts}_{subject}_telemetry.csv` | `"telemetry"` channel → `TelemetrySink` | Console telemetry snapshots. |
| `scans.db / session_data` | `"final"` channel → `ScanDBSink` | The corrected record (this document). |

The corrected CSV and `session_data` are fed by the same `"final"` channel, so
their values agree. One subtlety: the corrected CSV uses the minimum of left
and right timestamps for a merged row, while `session_data` stores per-side
rows with each side's own timestamp.

## Playback — rebuild a corrected CSV from the DB

`omotion.materialize_corrected_csv(db_path, session_id, output_path)`
reads `session_data` for a session and writes a corrected-format CSV
to disk that is **value-equivalent** to what `CsvSink` would have
written live. This exists so a session recorded with `csvEnabled=false`
(DB-only mode — no on-disk CSV) can still be fed to existing CSV-based
tooling like `plot_corrected_scan.py`.

```python
from omotion import materialize_corrected_csv
materialize_corrected_csv(
    "scan_data/scans.db",
    session_id=6,
    output_path="scan_data/.playback_sid6.csv",
)
```

What's preserved exactly:

- Column ordering — full 82-col layout, or the reduced 6-col layout
  (read from `session_meta.sdk_flags.reduced_mode`).
- Per-frame row layout — one row per distinct `frame_id`, merged
  across all (side, cam) cells.
- Per-frame `timestamp_s` — minimum of all contributing samples,
  same rule as `CsvSink`.
- `bfi`, `bvi`, `contrast`, `mean` cell values.

What's *not* preserved:

- `temp_*` cells are always empty (the DB doesn't carry temperature
  in `session_data`). `plot_corrected_scan.py` ignores temperature,
  so visualization still works.

Pre-#92 Step F sessions raise `RuntimeError` — their `frame_id` rows
are all the sentinel `-1`, so the row layout can't be reconstructed.
Callers should fall back to the on-disk corrected CSV in that case.

## Legacy databases

Databases written by older SDK versions can differ in three ways. Current
code opens them safely (`CREATE TABLE IF NOT EXISTS` is idempotent and
schema migrations only ever ADD columns), but readers should know:

* **`session_raw` table** — older SDKs wrote raw histogram blobs to the DB.
  Current code neither reads nor writes that table; the data is left
  untouched. (The blobs may be zlib-compressed, indicated by the
  `compress_raw_hist` key in `database_settings`.)
* **Live-valued `session_data`** — sessions whose `session_meta` lacks
  `data_semantics` hold realtime (live-branch) values written per-frame,
  with dark frames skipped. They are *not* the corrected record.
* **`session_raw_id` column** — old `session_data` rows may carry a
  (now-meaningless) foreign key into `session_raw`. New DBs are created
  without the column; inserts name their columns explicitly so both layouts
  accept writes.

## Notes on schema evolution

There is no schema version column yet. When a forward-incompatible schema
change becomes necessary, the convention will be to write a `schema_version`
key into `database_settings` and have `ScanDatabase._init_schema()` perform a
guarded migration. Until then, existing DBs are safely opened by current
code — the `CREATE TABLE IF NOT EXISTS` and `CREATE INDEX IF NOT EXISTS`
clauses make every open idempotent.

#### #92 Step F: `session_data.frame_id`

`session_data` originally did not carry `frame_id`, only `timestamp_s`.
Step F added the column so `materialize_corrected_csv` can merge by
the same key the live CSV writer uses (per-side clock skew makes
timestamp-based merging unreliable). `_init_schema()` runs an
idempotent `ALTER TABLE … ADD COLUMN frame_id INTEGER NOT NULL
DEFAULT -1` on legacy DBs; rows from pre-Step-F sessions get the
sentinel value `-1` and `materialize_corrected_csv` refuses to
reconstruct from them. Fresh DBs created on the post-Step-F SDK get
the column natively and a `(session_id, frame_id)` index for fast
lookups.
