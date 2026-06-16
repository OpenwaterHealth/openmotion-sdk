# Hardware verification — issue #92 storage-endpoint refactor

To run **after** the Sink protocol + CsvSink + ScanDBSink + MotionInterface
composition land on `feature/92-scan-db-sink` and the bloodflow-app glue
on `feature/92-scan-db-glue`. The unit + equivalence test suite covers
the science-pipeline → writer path comprehensively; this plan exercises
the bits that only run with real sensors attached.

If any check fails, see the bottom of this document for bisection guidance.

## Setup

```sh
cd C:/Users/ethan/Projects/openmotion-sdk
git fetch origin && git checkout feature/92-scan-db-sink && git pull

cd C:/Users/ethan/Projects/openmotion-bloodflow-app
git fetch origin && git checkout feature/92-scan-db-glue && git pull
```

Note your `scan_data/` baseline:
```sh
ls C:/Users/ethan/Projects/scan_data
```

## Pass 1 — CSV-only mode (regression check)

The default mode prior to issue #92 — should behave **byte-identically**
to the pre-refactor SDK.

`config/app_config.json`:
```json
"writeRawData": true,
"writeRawDataDurationSec": 30,
"scanDbEnabled": false
```

Launch the bloodflow app, run a 60-second scan with your normal camera selection.

**Pass criteria** (for the new scan timestamp `<TS>`):
- [ ] `<TS>_<subject>.csv` (corrected) exists.
- [ ] `<TS>_<subject>_left_mask<XX>_raw.csv` and `..._right_mask<XX>_raw.csv` exist.
- [ ] `<TS>_<subject>_telemetry.csv` exists (assuming `developerMode: true`).
- [ ] Corrected CSV header is exactly: `frame_id, timestamp_s, bfi_l1..bfi_r8, bvi_l1..bvi_r8, mean_l1..mean_r8, contrast_l1..contrast_r8, temp_l1..temp_r8` (82 cols).
- [ ] First `timestamp_s` is between 0.0 and 1.0 (per-scan t=0 normalization).
- [ ] Row count ≈ `duration_sec × 40 fps`, ±5 to allow for dark-frame discards.
- [ ] Raw CSVs both stop at ~30 seconds (the `writeRawDataDurationSec` cap).
- [ ] **No `scans.db`** appears (or its mtime is unchanged from before).
- [ ] App log contains **no** `Scan DB sink enabled` line.

## Pass 2 — Both modes (equivalence check)

The expected "shipping" config — both endpoints active, each cross-checks
the other.

`config/app_config.json`:
```json
"writeRawData": true,
"writeRawDataDurationSec": 30,
"scanDbEnabled": true
```

Run a 60-second scan.

**Pass criteria**:
- [ ] Same four CSV files as Pass 1.
- [ ] `scans.db` exists or grew.
- [ ] App log shows `Scan DB sink enabled, writing to ...\scan_data\scans.db`.
- [ ] `python C:/Users/ethan/Projects/openmotion-sdk/stream-db/db_browser.py "C:\Users\ethan\Projects\scan_data\scans.db"` opens and shows the new session.
- [ ] **DB ↔ CSV cross-check** — run this snippet, expect `OK`:

```python
import sqlite3, csv
db = sqlite3.connect("C:/Users/ethan/Projects/scan_data/scans.db")
db.row_factory = sqlite3.Row
sid = db.execute("SELECT MAX(id) FROM sessions").fetchone()[0]
label = db.execute(
    "SELECT session_label FROM sessions WHERE id=?", (sid,)
).fetchone()[0]
csv_path = f"C:/Users/ethan/Projects/scan_data/{label}.csv"
with open(csv_path, newline="") as f:
    csv_rows = list(csv.DictReader(f))
db_rows = db.execute(
    "SELECT cam_id, side, timestamp_s, bfi, bvi, contrast, mean "
    "FROM session_data WHERE session_id=? ORDER BY id",
    (sid,),
).fetchall()
csv0 = csv_rows[0]
ts = round(float(csv0["timestamp_s"]), 6)
match = [
    r for r in db_rows
    if r["side"] == 0 and r["cam_id"] == 2 and abs(r["timestamp_s"] - ts) < 1e-6
]
assert match, "no DB row for CSV row 0, side=L, cam=3"
assert float(csv0["bfi_l3"]) == match[0]["bfi"], (
    f"bfi mismatch: csv={csv0['bfi_l3']} db={match[0]['bfi']}"
)
print("OK")
```

## Pass 3 — DB-only mode (new feature)

`config/app_config.json`:
```json
"writeRawData": true,
"writeRawDataDurationSec": 30,
"scanDbEnabled": true,
"csvEnabled": false
```

Run a 60-second scan.

**Pass criteria**:
- [ ] **No new corrected CSV** appeared in `scan_data/` — only the raw CSVs (gated by `writeRawData`, capped at 30 s) and `scans.db` changed.
- [ ] App log shows `Scan DB sink enabled, ...` AND `CSV output disabled`.
- [ ] `scans.db` has a new `sessions` row; `session_data` row count ≈ `60 × 40 × active_cams × 2 sides` (final-branch rows — warmup frames and the terminal dark are absent, and rows trail the live scan by up to one dark interval).
- [ ] Raw CSV rows cover ≈ the first 30 s (cap honored). No `session_raw` table appears in a fresh DB — raw is CSV-only.

## Smoke checks for regressions

- [ ] **History modal**: open the bloodflow app's History pane; the most recent session appears in the dropdown and the DB-sourced provenance block (FW versions, hw IDs, etc.) renders.
- [ ] **Visualize** (after Step D playback lands): "Visualize BFI/BVI" on a DB-only session opens the same plot view it would for a CSV session.
- [ ] **Contact-quality check** still works — open the side panel, click the check button, confirm no errors.
- [ ] **Reduced mode**: flip `reducedMode: true` in app_config.json, run a scan; corrected CSV has only `bfi_left, bfi_right, bvi_left, bvi_right` columns.

## Bisection guidance

| Failure | Likely culprit | First place to look |
|---|---|---|
| Pass 1 fails (CSV regression) | CsvSink replaced inline writer incorrectly | Diff `omotion/ScanWorkflow.py` between the cut-over commit and its parent; verify the deleted inline-CSV state is matched by CsvSink's `on_*` hooks |
| Pass 2 cross-check fails (DB ↔ CSV divergence) | One sink rounded differently or got a different sample order | Extend `tests/test_csv_sink.py::test_csv_sink_matches_inline_corrected_csv` to cover the failing scenario |
| Pass 3 fails (CSV file appeared with `csvEnabled=false`) | The csv_enabled gate in `MotionInterface._build_sinks` is wrong | `omotion/MotionInterface.py` `_build_sinks` / `start_scan` |
| Reduced-mode CSV columns wrong | CsvSink reduced-mode path drifted from the inline reduced-mode aggregation | `omotion/CsvSink.py` `_on_corrected_batch_reduced` |
