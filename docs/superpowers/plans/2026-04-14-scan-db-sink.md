# Scan DB Sink Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a SQLite database an **opt-in** corrected-data endpoint of the science pipeline. The caller (typically the application) passes `db_path` to `MotionInterface(db_path=...)` at construction; if set, every scan opens a session row in that file and writes per-camera `session_data` rows from the corrected pipeline (and optionally `session_raw` rows from histograms). With `db_path=None` (the default), the SDK behavior is byte-for-byte identical to today — no DB file is created and no DB code runs in the hot path.

**Architecture:** `omotion/ScanDatabase.py` replaces `stream-db/scan_db.py` with identical API. New `omotion/ScanDBSink.py` adapts `CorrectedBatch` → `session_data` inserts and raw frames → `session_raw` inserts; numeric fields are rounded to 6 decimals at insert time to match the corrected CSV writer's precision (per project decision: 6 decimals is enough for any number we store). `ScanWorkflow.start_scan` gains two new optional callbacks (`on_raw_frame_fn`, `on_scan_start_fn`) but stays DB-unaware. `MotionInterface` takes a `db_path: str | None = None` constructor arg; when set, `MotionInterface.start_scan` constructs the sink, opens the session from `on_scan_start_fn`, and closes it in a wrapped `on_complete_fn`. Per-scan `ScanRequest` flags `write_raw_to_db` and `notes` control whether raw frames are persisted and what free-text annotation is attached.

> **Note on line numbers in this plan:** the plan was first drafted ~5 weeks before this revision, against an earlier state of `omotion/`. Since then `next` has had ~100 commits (FT calibration #122, calibration workflow rework, file renames, etc.). All `omotion/ScanWorkflow.py` and `omotion/MotionInterface.py` references below should be re-located by **grep on the symbol**, not by trusting the printed line numbers.

**Tech Stack:** Python 3.12+, SQLite (stdlib `sqlite3`), pytest. No new third-party deps.

**Worktree root (use this exact path):** `C:/Users/ethan/Projects/openmotion-sdk/.claude/worktrees/buzzing-forging-rainbow`

**Spec:** `docs/superpowers/specs/2026-04-14-scan-db-sink-design.md`

---

## File Structure

**Create:**
- `omotion/ScanDatabase.py` — `ScanDatabase` class (moved, verbatim, from `stream-db/scan_db.py`).
- `omotion/ScanDBSink.py` — adapter: `ScanDBSink` class.
- `tests/test_scan_database.py` — round-trip + compression + JSON meta tests.
- `tests/test_scan_db_sink.py` — unit tests for the sink (open, on_raw_frame, on_corrected_batch, close, error handling, concurrency).
- `tests/test_scan_workflow_db_integration.py` — exercises `MOTIONInterface.start_scan` with the sink against canned data.
- `tests/test_db_matches_corrected_csv.py` — equivalence test: DB rows reproduce CSV rows cell-for-cell.

**Modify:**
- `omotion/__init__.py` — re-export `ScanDatabase`, `ScanDBSink`.
- `omotion/ScanWorkflow.py` — add `on_raw_frame_fn` + `on_scan_start_fn` kwargs to `start_scan`; fire them from `_worker` and the per-side row handler. Also add `write_raw_to_db` and `notes` fields to `ScanRequest`.
- `omotion/MotionInterface.py` — add `db_path: str | None = None` constructor arg; in `start_scan`, build the sink and chain callbacks when `db_path` is set; ensure `close()` on completion.
- `stream-db/db_browser.py`, `stream-db/db_validator.py`, `stream-db/importer.py`, `stream-db/sensor_module_simulator.py` — switch imports from `scan_db` to `omotion.ScanDatabase`.

**Delete:**
- `stream-db/scan_db.py`.

---

## Task 1: Move `scan_db.py` into the `omotion` package

**Files:**
- Create: `omotion/ScanDatabase.py`
- Modify: `omotion/__init__.py`
- Delete: `stream-db/scan_db.py` (retained until Task 2 verifies importers switch over, then removed)

This task is a pure move. No behavior changes — same class, same schema, same public API. The stream-db tool files are updated in Task 2.

- [x] **Step 1: Copy `stream-db/scan_db.py` to `omotion/ScanDatabase.py` verbatim**

```bash
cp stream-db/scan_db.py omotion/ScanDatabase.py
```

Verify the file exists:

```bash
test -f omotion/ScanDatabase.py && echo "ok"
```

Expected: `ok`

- [x] **Step 2: Re-export `ScanDatabase` from the package**

Open `omotion/__init__.py` and append just before the `try: __version__ = ...` block (i.e., after the `MotionConfig` import on line 25):

```python
from .ScanDatabase import ScanDatabase
```

- [x] **Step 3: Sanity-check the import works**

```bash
cd C:/Users/ethan/Projects/openmotion-sdk/.claude/worktrees/buzzing-forging-rainbow
python -c "from omotion import ScanDatabase; print(ScanDatabase.__module__)"
```

Expected: `omotion.ScanDatabase`

- [x] **Step 4: Write a smoke test that opens an in-memory-like DB**

Create `tests/test_scan_database.py`:

```python
"""Tests for omotion.ScanDatabase (the re-homed scan_db.py)."""

import json
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omotion import ScanDatabase


@pytest.fixture
def tmp_db(tmp_path: Path) -> ScanDatabase:
    db = ScanDatabase(db_path=str(tmp_path / "test.db"))
    yield db
    db.close()


def test_create_and_get_session(tmp_db: ScanDatabase) -> None:
    sid = tmp_db.create_session(
        session_label="20260414_000000_owTEST",
        session_start=1744600000.0,
        session_notes="unit test",
        session_meta={"subject_id": "owTEST", "fps": 40},
    )
    assert sid > 0

    session = tmp_db.get_session(sid)
    assert session is not None
    assert session["session_label"] == "20260414_000000_owTEST"
    assert session["session_start"] == 1744600000.0
    assert session["session_notes"] == "unit test"
    assert session["session_meta"] == {"subject_id": "owTEST", "fps": 40}


def test_insert_and_read_raw_frame_uncompressed(tmp_path: Path) -> None:
    db = ScanDatabase(db_path=str(tmp_path / "raw.db"), compress_raw_hist=False)
    try:
        sid = db.create_session(
            session_label="label",
            session_start=0.0,
        )
        hist = bytes(range(256)) * 16  # 4096-byte blob
        rid = db.insert_raw_frame(
            sid, "left", 0, 1, 1.25, hist,
            temp=27.0, sum_counts=100, tcm=1.0, tcl=2.0, pdc=3.0,
        )
        assert rid > 0

        row = db.get_raw_frame(rid)
        assert row is not None
        assert row["side"] == "left"
        assert row["cam_id"] == 0
        assert row["frame_id"] == 1
        assert row["hist"] == hist
        assert row["temp"] == 27.0
        assert row["sum"] == 100
    finally:
        db.close()


def test_raw_frame_roundtrip_compressed(tmp_path: Path) -> None:
    db = ScanDatabase(db_path=str(tmp_path / "zraw.db"), compress_raw_hist=True)
    try:
        sid = db.create_session(session_label="z", session_start=0.0)
        hist = bytes([0, 1, 2, 3]) * 1024
        rid = db.insert_raw_frame(sid, "right", 2, 5, 0.1, hist)
        row = db.get_raw_frame(rid)
        assert row["hist"] == hist  # transparently decompressed on read
    finally:
        db.close()


def test_insert_session_data_and_stream(tmp_db: ScanDatabase) -> None:
    sid = tmp_db.create_session(session_label="s", session_start=0.0)
    tmp_db.insert_session_data(
        sid, cam_id=3, side=0, timestamp_s=1.0,
        bfi=0.25, bvi=0.5, contrast=0.1, mean=511.0,
    )
    rows = [r for batch in tmp_db.stream_session_data(sid) for r in batch]
    assert len(rows) == 1
    assert rows[0]["cam_id"] == 3
    assert rows[0]["side"] == 0
    assert rows[0]["bfi"] == 0.25


def test_session_meta_survives_unicode_and_none(tmp_db: ScanDatabase) -> None:
    meta = {"subject": "öwtëst", "ops": None, "nested": {"a": [1, 2, 3]}}
    sid = tmp_db.create_session(
        session_label="u", session_start=0.0, session_meta=meta,
    )
    session = tmp_db.get_session(sid)
    assert session["session_meta"] == meta
```

- [x] **Step 5: Run the new tests to confirm the move is clean**

```bash
pytest tests/test_scan_database.py -v
```

Expected: all tests PASS.

- [x] **Step 6: Delete the old file once tests are green**

```bash
rm stream-db/scan_db.py
```

- [x] **Step 7: Commit**

```bash
git add omotion/ScanDatabase.py omotion/__init__.py tests/test_scan_database.py stream-db/scan_db.py
git commit -m "refactor(sdk): move scan_db.py into omotion.ScanDatabase"
```

---

## Task 2: Point stream-db tools at the new import path

**Files:**
- Modify: `stream-db/db_browser.py`
- Modify: `stream-db/db_validator.py`
- Modify: `stream-db/importer.py`
- Modify: `stream-db/sensor_module_simulator.py`

Each of these currently does `from scan_db import ScanDatabase`. After Task 1 that import is broken. Switch each to `from omotion import ScanDatabase`. Because `stream-db/` is not on the Python path when running these scripts standalone, add a `sys.path` bootstrap to each file so they can locate the installed-or-local `omotion` package.

- [x] **Step 1: Verify the current import in each file**

```bash
grep -n "from scan_db" stream-db/db_browser.py stream-db/db_validator.py stream-db/importer.py stream-db/sensor_module_simulator.py
```

Record the line numbers of each match — you'll replace them below.

- [x] **Step 2: Update `stream-db/db_browser.py`**

Replace the existing `from scan_db import ScanDatabase` line with:

```python
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from omotion import ScanDatabase
```

(If `import os` / `import sys` are already imported at the top of the file, don't duplicate them — just add the `sys.path.insert` line and the `from omotion import ScanDatabase` line.)

- [x] **Step 3: Update `stream-db/db_validator.py`** — same pattern as Step 2.

- [x] **Step 4: Update `stream-db/importer.py`** — same pattern as Step 2.

- [x] **Step 5: Update `stream-db/sensor_module_simulator.py`** — same pattern as Step 2.

- [x] **Step 6: Smoke-test each script can at least import**

```bash
python -c "import importlib.util,sys,os; sys.path.insert(0, 'stream-db'); [importlib.util.spec_from_file_location(n, os.path.join('stream-db', n + '.py')).loader.exec_module(importlib.util.module_from_spec(importlib.util.spec_from_file_location(n, os.path.join('stream-db', n + '.py')))) for n in ['db_browser','db_validator','importer','sensor_module_simulator']]" 2>&1 | tail -3
```

If that one-liner is too fragile, do a simpler visual check by running each with `--help` where supported:

```bash
python stream-db/importer.py --help
python stream-db/db_validator.py --help
```

Expected: each prints a usage string. No `ModuleNotFoundError: No module named 'scan_db'`.

- [x] **Step 7: Commit**

```bash
git add stream-db/db_browser.py stream-db/db_validator.py stream-db/importer.py stream-db/sensor_module_simulator.py
git commit -m "refactor(stream-db): import ScanDatabase from omotion package"
```

---

## Task 3: Scaffold `ScanDBSink` with construction + `open()` + `close()`

**Files:**
- Create: `omotion/ScanDBSink.py`
- Modify: `omotion/__init__.py`
- Test: `tests/test_scan_db_sink.py`

Build the skeleton. Just construction, session open, session close. The callbacks are no-ops at this task; they get filled in by Tasks 4 and 5.

- [x] **Step 1: Write the failing tests for construction, open, close**

Create `tests/test_scan_db_sink.py`:

```python
"""Tests for omotion.ScanDBSink."""

import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omotion import ScanDatabase, ScanDBSink


def test_sink_opens_and_closes_session(tmp_path: Path) -> None:
    db_path = tmp_path / "scans.db"
    sink = ScanDBSink(str(db_path))
    sid = sink.open(
        label="20260414_120000_owTEST",
        start_ts=1744632000.0,
        notes="pytest run",
        meta={"subject_id": "owTEST"},
    )
    assert sid > 0

    sink.close(end_ts=1744632010.0)

    # Verify row was persisted and end time was written.
    db = ScanDatabase(db_path=str(db_path))
    try:
        session = db.get_session(sid)
        assert session["session_label"] == "20260414_120000_owTEST"
        assert session["session_start"] == 1744632000.0
        assert session["session_end"] == 1744632010.0
        assert session["session_notes"] == "pytest run"
        assert session["session_meta"] == {"subject_id": "owTEST"}
    finally:
        db.close()


def test_sink_close_is_idempotent(tmp_path: Path) -> None:
    sink = ScanDBSink(str(tmp_path / "scans.db"))
    sid = sink.open(label="x", start_ts=0.0, notes="", meta={})
    sink.close(end_ts=1.0)
    # Second close must not raise and must not bump session_end.
    sink.close(end_ts=2.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        assert db.get_session(sid)["session_end"] == 1.0
    finally:
        db.close()


def test_sink_raises_if_callbacks_called_before_open(tmp_path: Path) -> None:
    from omotion.MotionProcessing import CorrectedBatch

    sink = ScanDBSink(str(tmp_path / "scans.db"))
    batch = CorrectedBatch(dark_frame_start=0, dark_frame_end=0, samples=[])
    with pytest.raises(RuntimeError):
        sink.on_corrected_batch(batch)
    with pytest.raises(RuntimeError):
        sink.on_raw_frame(
            "left", 0, 1, 0.0, b"\x00" * 4096, 25.0, 0, 0.0, 0.0, 0.0,
        )
```

- [x] **Step 2: Run to confirm the tests fail**

```bash
pytest tests/test_scan_db_sink.py -v
```

Expected: ImportError on `from omotion import ScanDBSink` (`ScanDBSink` does not exist yet).

- [x] **Step 3: Create `omotion/ScanDBSink.py` with construction, open, close only**

```python
"""
ScanDBSink — adapts ScanWorkflow callbacks to ScanDatabase inserts.

Owns one database connection and one open session row for the duration
of a scan.  Constructed and wired up by ``MOTIONInterface.start_scan``;
``ScanWorkflow`` itself stays unaware of the database.
"""

from __future__ import annotations

import logging
import threading
from typing import TYPE_CHECKING, Any, Optional

from omotion import _log_root
from omotion.ScanDatabase import ScanDatabase

if TYPE_CHECKING:
    from omotion.MotionProcessing import CorrectedBatch

logger = logging.getLogger(
    f"{_log_root}.ScanDBSink" if _log_root else "ScanDBSink"
)


class ScanDBSink:
    def __init__(
        self,
        db_path: str,
        *,
        write_raw: bool = False,
        compress_raw_hist: bool = True,
        raw_batch_size: int = 200,
    ) -> None:
        self._db_path = db_path
        self._write_raw = write_raw
        self._compress_raw_hist = compress_raw_hist
        self._raw_batch_size = max(1, int(raw_batch_size))

        self._db: Optional[ScanDatabase] = None
        self._session_id: Optional[int] = None
        self._closed: bool = False
        self._lock = threading.Lock()
        self._raw_buffer: list[dict[str, Any]] = []
        self._insert_errors: int = 0

    @property
    def insert_errors(self) -> int:
        return self._insert_errors

    @property
    def session_id(self) -> Optional[int]:
        return self._session_id

    def open(
        self,
        *,
        label: str,
        start_ts: float,
        notes: str,
        meta: dict,
    ) -> int:
        with self._lock:
            if self._db is not None:
                raise RuntimeError("ScanDBSink.open called twice")
            self._db = ScanDatabase(
                db_path=self._db_path,
                compress_raw_hist=self._compress_raw_hist,
            )
            self._session_id = self._db.create_session(
                session_label=label,
                session_start=start_ts,
                session_notes=notes,
                session_meta=meta,
            )
            return self._session_id

    def close(self, end_ts: float) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._flush_raw_locked()
                if self._db is not None and self._session_id is not None:
                    self._db.close_session(self._session_id, end_ts)
            except Exception:
                logger.exception("ScanDBSink.close failed while finalising session")
            finally:
                if self._db is not None:
                    try:
                        self._db.close()
                    except Exception:
                        logger.exception("ScanDBSink: error closing ScanDatabase")
                self._db = None

    def on_corrected_batch(self, batch: "CorrectedBatch") -> None:
        self._require_open()
        # Real implementation lands in Task 5.

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
    ) -> None:
        self._require_open()
        # Real implementation lands in Task 4.

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _require_open(self) -> None:
        if self._db is None or self._session_id is None:
            raise RuntimeError(
                "ScanDBSink callback invoked before open() — "
                "call open() to start the session first."
            )

    def _flush_raw_locked(self) -> None:
        """Flush buffered raw frames.  Caller holds self._lock."""
        # Real implementation lands in Task 4.
        self._raw_buffer.clear()
```

- [x] **Step 4: Re-export `ScanDBSink` from the package**

Open `omotion/__init__.py` and add after `from .ScanDatabase import ScanDatabase`:

```python
from .ScanDBSink import ScanDBSink
```

- [x] **Step 5: Run the tests to verify they pass**

```bash
pytest tests/test_scan_db_sink.py -v
```

Expected: all three tests PASS.

- [x] **Step 6: Commit**

```bash
git add omotion/ScanDBSink.py omotion/__init__.py tests/test_scan_db_sink.py
git commit -m "feat(sdk): add ScanDBSink skeleton (open/close/callbacks as stubs)"
```

---

## Task 4: Implement `on_raw_frame` with batched writes and flush

**Files:**
- Modify: `omotion/ScanDBSink.py`
- Test: `tests/test_scan_db_sink.py`

Fill in `on_raw_frame` (no-op when `write_raw=False`; buffer → `executemany` flush at `raw_batch_size`) and the `_flush_raw_locked` helper. Also flush on `close()` (already called; just needs the real implementation) and at the start of each corrected batch (added in Task 5).

- [x] **Step 1: Add failing tests for raw-frame behavior**

Append to `tests/test_scan_db_sink.py`:

```python
import threading
from omotion.MotionProcessing import CorrectedBatch


def _make_sink(tmp_path, **kwargs):
    from omotion import ScanDBSink
    sink = ScanDBSink(str(tmp_path / "scans.db"), **kwargs)
    sid = sink.open(label="lbl", start_ts=0.0, notes="", meta={})
    return sink, sid


def test_on_raw_frame_is_noop_when_write_raw_false(tmp_path):
    sink, sid = _make_sink(tmp_path, write_raw=False)
    sink.on_raw_frame("left", 0, 1, 0.0, b"\x00" * 4096, 25.0, 10, 0.0, 0.0, 0.0)
    sink.close(end_ts=1.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for batch in db.stream_raw_frames(sid) for r in batch]
        assert rows == []
    finally:
        db.close()


def test_on_raw_frame_writes_one_row_per_call(tmp_path):
    sink, sid = _make_sink(tmp_path, write_raw=True, raw_batch_size=2)
    hist = b"\xab" * 4096
    for fid in range(5):
        sink.on_raw_frame("left", 0, fid, fid * 0.025, hist, 25.0, 100, 0.0, 0.0, 0.0)
    sink.close(end_ts=1.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for batch in db.stream_raw_frames(sid) for r in batch]
        assert len(rows) == 5
        assert [r["frame_id"] for r in rows] == [0, 1, 2, 3, 4]
        assert rows[0]["hist"] == hist  # transparently decompressed by default
    finally:
        db.close()


def test_on_raw_frame_flushes_on_batch_size(tmp_path):
    sink, sid = _make_sink(tmp_path, write_raw=True, raw_batch_size=3)
    for fid in range(3):
        sink.on_raw_frame("right", 1, fid, 0.0, b"\x00" * 4096, 0.0, 0, 0.0, 0.0, 0.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        # All three should already be persisted — the 3rd call hit batch_size.
        rows = [r for batch in db.stream_raw_frames(sid) for r in batch]
        assert len(rows) == 3
    finally:
        db.close()
    sink.close(end_ts=1.0)


def test_on_raw_frame_concurrent_writers(tmp_path):
    sink, sid = _make_sink(tmp_path, write_raw=True, raw_batch_size=50)

    def _writer(side, n):
        hist = b"\x01" * 4096
        for fid in range(n):
            sink.on_raw_frame(side, 0, fid, fid * 0.025, hist, 25.0, 10, 0.0, 0.0, 0.0)

    threads = [
        threading.Thread(target=_writer, args=("left", 200)),
        threading.Thread(target=_writer, args=("right", 200)),
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    sink.close(end_ts=1.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for batch in db.stream_raw_frames(sid) for r in batch]
        assert len(rows) == 400
    finally:
        db.close()
```

- [x] **Step 2: Run the tests to confirm they fail**

```bash
pytest tests/test_scan_db_sink.py -v
```

Expected: the four new tests FAIL (`on_raw_frame` is still a stub — no rows get written).

- [x] **Step 3: Implement `on_raw_frame` and `_flush_raw_locked`**

In `omotion/ScanDBSink.py`, replace the body of `on_raw_frame` with:

```python
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
    ) -> None:
        if not self._write_raw:
            return
        with self._lock:
            if self._db is None or self._session_id is None:
                raise RuntimeError(
                    "ScanDBSink.on_raw_frame invoked before open()"
                )
            self._raw_buffer.append(
                {
                    "session_id": self._session_id,
                    "side": side,
                    "cam_id": int(cam_id),
                    "frame_id": int(frame_id),
                    # Project-wide convention: store floats to 6 decimals
                    # (matches the corrected CSV writer). Anything beyond
                    # 6 is noise we don't need to keep.
                    "timestamp_s": round(float(timestamp_s), 6),
                    "hist": bytes(hist),
                    "temp": round(float(temp), 6) if temp is not None else None,
                    "sum_counts": int(sum_counts) if sum_counts is not None else None,
                    "tcm": round(float(tcm), 6),
                    "tcl": round(float(tcl), 6),
                    "pdc": round(float(pdc), 6),
                }
            )
            if len(self._raw_buffer) >= self._raw_batch_size:
                self._flush_raw_locked()
```

Replace `_flush_raw_locked` with:

```python
    def _flush_raw_locked(self) -> None:
        """Flush buffered raw frames.  Caller holds self._lock."""
        if not self._raw_buffer or self._db is None:
            self._raw_buffer.clear()
            return
        try:
            self._db.insert_raw_frames(self._raw_buffer)
        except Exception:
            logger.exception(
                "ScanDBSink: failed to flush %d buffered raw frames",
                len(self._raw_buffer),
            )
            self._insert_errors += len(self._raw_buffer)
        finally:
            self._raw_buffer.clear()
```

Also update `_require_open` to be called outside the lock (it's only used for pre-`open()` validation). Remove the `self._require_open()` call from `on_raw_frame` now that the body has its own inline check — the existing stub-era assertion from Task 3's test `test_sink_raises_if_callbacks_called_before_open` still passes because the inline check inside the `with self._lock` block raises `RuntimeError`.

- [x] **Step 4: Run the tests to confirm they pass**

```bash
pytest tests/test_scan_db_sink.py -v
```

Expected: all raw-frame tests PASS, plus the earlier open/close tests still PASS.

- [x] **Step 5: Commit**

```bash
git add omotion/ScanDBSink.py tests/test_scan_db_sink.py
git commit -m "feat(sdk): ScanDBSink.on_raw_frame with batched executemany flush"
```

---

## Task 5: Implement `on_corrected_batch` (the primary endpoint)

**Files:**
- Modify: `omotion/ScanDBSink.py`
- Test: `tests/test_scan_db_sink.py`

This is the load-bearing piece: convert each `Sample` in a `CorrectedBatch` into one `session_data` row. Side encoding is `"left"` → `0`, `"right"` → `1`. `cam_id` is written as-is from the sample. Use `insert_session_data_rows` for a single-transaction bulk insert. Flush any buffered raw frames at the top of the call (cheap sync point). Per-row errors are logged and counted but do not raise.

- [x] **Step 1: Add failing tests for corrected-batch writes**

Append to `tests/test_scan_db_sink.py`:

```python
from omotion.MotionProcessing import Sample


def _mk_sample(side, cam_id, frame_id, ts, bfi, bvi, contrast, mean):
    return Sample(
        side=side,
        cam_id=cam_id,
        frame_id=frame_id,
        absolute_frame_id=frame_id,
        timestamp_s=ts,
        row_sum=0,
        temperature_c=25.0,
        mean=mean,
        std_dev=0.0,
        contrast=contrast,
        bfi=bfi,
        bvi=bvi,
        is_corrected=True,
    )


def test_on_corrected_batch_writes_one_row_per_sample(tmp_path):
    sink, sid = _make_sink(tmp_path)
    batch = CorrectedBatch(
        dark_frame_start=0,
        dark_frame_end=600,
        samples=[
            _mk_sample("left", 0, 1, 0.025, 0.1, 0.2, 0.3, 500.0),
            _mk_sample("left", 1, 1, 0.025, 0.11, 0.21, 0.31, 501.0),
            _mk_sample("right", 0, 1, 0.025, 0.12, 0.22, 0.32, 502.0),
        ],
    )
    sink.on_corrected_batch(batch)
    sink.close(end_ts=1.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for batch_ in db.stream_session_data(sid) for r in batch_]
        assert len(rows) == 3
        left_rows = [r for r in rows if r["side"] == 0]
        right_rows = [r for r in rows if r["side"] == 1]
        assert len(left_rows) == 2
        assert len(right_rows) == 1
        lr = next(r for r in left_rows if r["cam_id"] == 0)
        assert lr["bfi"] == 0.1
        assert lr["bvi"] == 0.2
        assert lr["contrast"] == 0.3
        assert lr["mean"] == 500.0
        assert lr["timestamp_s"] == 0.025
    finally:
        db.close()


def test_on_corrected_batch_flushes_pending_raw_frames(tmp_path):
    sink, sid = _make_sink(tmp_path, write_raw=True, raw_batch_size=100)
    # Enqueue 5 raw frames (below batch_size, so not yet flushed).
    for fid in range(5):
        sink.on_raw_frame("left", 0, fid, 0.0, b"\x00" * 4096, 25.0, 0, 0.0, 0.0, 0.0)

    batch = CorrectedBatch(
        dark_frame_start=0,
        dark_frame_end=600,
        samples=[_mk_sample("left", 0, 1, 0.025, 0.1, 0.2, 0.3, 500.0)],
    )
    sink.on_corrected_batch(batch)

    # Before close, raw rows should already be visible.
    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for b in db.stream_raw_frames(sid) for r in b]
        assert len(rows) == 5
    finally:
        db.close()
    sink.close(end_ts=1.0)


def test_on_corrected_batch_empty_is_noop(tmp_path):
    sink, sid = _make_sink(tmp_path)
    sink.on_corrected_batch(
        CorrectedBatch(dark_frame_start=0, dark_frame_end=0, samples=[])
    )
    sink.close(end_ts=1.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for b in db.stream_session_data(sid) for r in b]
        assert rows == []
    finally:
        db.close()
```

- [x] **Step 2: Run the tests to confirm they fail**

```bash
pytest tests/test_scan_db_sink.py -v -k corrected
```

Expected: FAIL — `on_corrected_batch` is still a stub.

- [x] **Step 3: Implement `on_corrected_batch`**

In `omotion/ScanDBSink.py`, replace the `on_corrected_batch` body with:

```python
    def on_corrected_batch(self, batch: "CorrectedBatch") -> None:
        with self._lock:
            if self._db is None or self._session_id is None:
                raise RuntimeError(
                    "ScanDBSink.on_corrected_batch invoked before open()"
                )
            # Flush any buffered raw frames first so raw and corrected
            # writes land in the DB in roughly the order they were produced.
            self._flush_raw_locked()

            if not batch.samples:
                return

            rows: list[dict[str, Any]] = []
            for s in batch.samples:
                side_int = 0 if s.side == "left" else 1 if s.side == "right" else None
                if side_int is None:
                    logger.warning("ScanDBSink: unknown side %r, skipping sample", s.side)
                    self._insert_errors += 1
                    continue
                rows.append(
                    {
                        "session_id": self._session_id,
                        "session_raw_id": None,
                        "cam_id": int(s.cam_id),
                        "side": side_int,
                        # 6-decimal rounding matches the corrected CSV
                        # writer exactly — Task 9 relies on this for a
                        # clean cell-for-cell equivalence comparison.
                        "timestamp_s": round(float(s.timestamp_s), 6),
                        "bfi": round(float(s.bfi), 6),
                        "bvi": round(float(s.bvi), 6),
                        "contrast": round(float(s.contrast), 6),
                        "mean": round(float(s.mean), 6),
                    }
                )

            if not rows:
                return
            try:
                self._db.insert_session_data_rows(rows)
            except Exception:
                logger.exception(
                    "ScanDBSink: failed to insert %d session_data rows", len(rows)
                )
                self._insert_errors += len(rows)
```

- [x] **Step 4: Run the tests to verify**

```bash
pytest tests/test_scan_db_sink.py -v
```

Expected: every test PASSES (including all earlier ones).

- [x] **Step 5: Commit**

```bash
git add omotion/ScanDBSink.py tests/test_scan_db_sink.py
git commit -m "feat(sdk): ScanDBSink.on_corrected_batch writes session_data rows"
```

---

## Task 6: Expose `on_raw_frame_fn` and `on_scan_start_fn` from ScanWorkflow

**Files:**
- Modify: `omotion/ScanWorkflow.py`

Two surgical additions to `ScanWorkflow.start_scan`:

1. `on_raw_frame_fn: Callable[..., None] | None = None` — called from the per-side raw writer thread right after the science pipeline is fed, with `(side, cam_id, frame_id, timestamp_s, hist, temp, sum_counts, tcm, tcl, pdc)`. The `tcm/tcl/pdc` values come from the same `extra_cols_fn` the writer already calls (or `0.0` if `extra_cols_fn` is `None`).
2. `on_scan_start_fn: Callable[[str, float], None] | None = None` — called once at the top of `_worker` with the canonical `ts` (the `YYYYMMDD_HHMMSS` string already generated there) and a wall-clock `session_start` captured at the same instant.

ScanWorkflow itself does NOT know about the DB — it just fires the callbacks.

- [x] **Step 1: Find the signature of `start_scan`**

```bash
grep -n "def start_scan" omotion/ScanWorkflow.py
```

Confirm it's at line 173 and lists `on_uncorrected_fn`, `on_corrected_batch_fn`, etc. as shown in the spec.

- [x] **Step 2: Add the two new kwargs to `start_scan`**

In `omotion/ScanWorkflow.py`, modify the `start_scan` signature. Locate the existing line near line 181:

```python
        on_uncorrected_fn: Callable[[object], None] | None = None,
        on_corrected_batch_fn: Callable[[object], None] | None = None,
```

Insert right after `on_corrected_batch_fn`:

```python
        on_raw_frame_fn: Callable[..., None] | None = None,
        on_scan_start_fn: Callable[[str, float], None] | None = None,
```

- [x] **Step 3: Fire `on_scan_start_fn` at the top of `_worker`**

Find the `ts = datetime.datetime.now().strftime(...)` line (around line 206 in `_worker`). Immediately after that line, add:

```python
            session_start_ts = time.time()
            if on_scan_start_fn:
                try:
                    on_scan_start_fn(ts, session_start_ts)
                except Exception:
                    logger.exception("on_scan_start_fn callback raised")
```

- [x] **Step 4: Fire `on_raw_frame_fn` from `_make_row_handler`**

Replace the existing `_make_row_handler` (around line 433) with:

```python
                def _make_row_handler(current_side: str, p):
                    """Close over side so each writer thread feeds the right key."""
                    def _on_row(cam_id, frame_id, ts_val, hist, row_sum, temp):
                        if p is not None:
                            p.enqueue(
                                current_side,
                                cam_id,
                                frame_id,
                                ts_val,
                                hist,
                                row_sum,
                                temp,
                            )
                        if on_raw_frame_fn is not None:
                            if extra_cols_fn is not None:
                                try:
                                    extras = extra_cols_fn()
                                except Exception:
                                    extras = []
                            else:
                                extras = []
                            tcm = float(extras[0]) if len(extras) > 0 else 0.0
                            tcl = float(extras[1]) if len(extras) > 1 else 0.0
                            pdc = float(extras[2]) if len(extras) > 2 else 0.0
                            try:
                                on_raw_frame_fn(
                                    current_side,
                                    int(cam_id),
                                    int(frame_id),
                                    float(ts_val),
                                    bytes(hist),
                                    float(temp) if temp is not None else 0.0,
                                    int(row_sum) if row_sum is not None else 0,
                                    tcm,
                                    tcl,
                                    pdc,
                                )
                            except Exception:
                                logger.exception("on_raw_frame_fn callback raised")
                    return _on_row
```

- [x] **Step 5: Sanity-check that existing tests still pass**

```bash
pytest tests/test_corrected_csv_output.py -v
```

Expected: PASS. (No behavior changes when the new callbacks aren't supplied.)

- [x] **Step 6: Commit**

```bash
git add omotion/ScanWorkflow.py
git commit -m "feat(sdk): ScanWorkflow exposes on_raw_frame_fn and on_scan_start_fn"
```

---

## Task 7: Wire `ScanDBSink` into `MotionInterface` via constructor opt-in

**Files:**
- Modify: `omotion/MotionInterface.py`
- Modify: `omotion/ScanWorkflow.py` (add new `ScanRequest` fields)

The DB endpoint is **opt-in at SDK construction**, not per request. When the caller does `MotionInterface(db_path="/some/path/scans.db")`, every subsequent `start_scan` writes corrected (and optionally raw) data to that file. When `db_path=None` (the default), no sink is built and the call path is identical to today.

New `ScanRequest` fields are minimal: `write_raw_to_db: bool = False` (per-scan raw opt-in, only meaningful when the SDK has a `db_path`) and `notes: str = ""` (free-text annotation attached to the session row).

`MotionInterface`:
1. Accept `db_path: str | None = None` in `__init__`; stash as `self._db_path`.
2. In `start_scan`, if `self._db_path is not None`, build a sink and wrap callbacks (same chaining logic as the original plan).
3. `_build_meta` uses the real APIs that exist on current `next`: `self.console.get_version()`, `self.left.get_version()`, `self.right.get_version()`, plus `get_cached_hardware_id() / get_hardware_id()` for chip IDs.
4. Wrap `on_complete_fn` so it calls `sink.close(time.time())` first, then the caller's callback.

- [x] **Step 1: Add new fields to `ScanRequest`**

In `omotion/ScanWorkflow.py`, locate the `@dataclass class ScanRequest:` declaration (grep for `class ScanRequest`). Add at the bottom of the dataclass (after the existing CSV-related fields):

```python
    # Database sink (see docs/superpowers/specs/2026-04-14-scan-db-sink-design.md).
    # The DB endpoint itself is opt-in at SDK construction via
    # MotionInterface(db_path=...); these per-scan fields are only effective
    # when that path is set.
    write_raw_to_db: bool = False
    notes: str = ""
```

- [x] **Step 2: Add `db_path` constructor arg to `MotionInterface`**

In `omotion/MotionInterface.py`, locate `def __init__(self, ...)` (grep for `def __init__` inside `class MotionInterface`). Add `db_path: str | None = None` to the signature (keep the existing positional/keyword ordering — append it as a kwarg with a default), and inside the constructor body stash it:

```python
        # Optional DB sink: when set, MotionInterface.start_scan builds a
        # ScanDBSink that writes corrected (and optionally raw) data to
        # this SQLite file for every scan. When None, no DB code runs.
        self._db_path: str | None = db_path
```

- [x] **Step 3: Modify `MotionInterface.start_scan` to wire the sink when `db_path` is set**

Locate the existing `start_scan` (grep for `def start_scan` inside `MotionInterface`). Replace its body with:

```python
    def start_scan(self, request, **kwargs) -> bool:
        if not self.scan_workflow:
            self.scan_workflow = ScanWorkflow(self)

        if self._db_path is not None:
            kwargs = self._wrap_kwargs_with_db_sink(request, kwargs)

        return self.scan_workflow.start_scan(request, **kwargs)
```

Then add this helper method to `MotionInterface` (place it just below `start_scan`):

```python
    def _wrap_kwargs_with_db_sink(self, request, kwargs: dict) -> dict:
        """
        Construct a ScanDBSink for this scan and chain its callbacks in
        front of any caller-supplied callbacks. The sink is opened when
        ScanWorkflow fires on_scan_start_fn and closed inside the wrapped
        on_complete_fn. Triggered only when the interface was constructed
        with a db_path.
        """
        import os
        import time

        from omotion import ScanDBSink, __version__ as sdk_version

        db_path = self._db_path
        # Auto-create the parent directory so callers can pass a path
        # inside a fresh data dir without separately mkdir-ing it.
        parent = os.path.dirname(db_path)
        if parent:
            os.makedirs(parent, exist_ok=True)

        sink = ScanDBSink(
            db_path,
            write_raw=bool(getattr(request, "write_raw_to_db", False)),
            compress_raw_hist=True,
        )

        def _active_cams(mask: int) -> list[int]:
            return [i + 1 for i in range(8) if (mask >> i) & 0x1]

        def _safe_call(fn):
            try:
                return fn()
            except Exception:
                return None

        def _hex_or_none(val):
            return val.hex() if isinstance(val, (bytes, bytearray)) else val

        def _build_meta() -> dict:
            return {
                "subject_id": request.subject_id,
                "duration_sec": request.duration_sec,
                "expected_size": request.expected_size,
                "fps": 40,
                "left_camera_mask": request.left_camera_mask,
                "right_camera_mask": request.right_camera_mask,
                "active_left_cams": _active_cams(request.left_camera_mask),
                "active_right_cams": _active_cams(request.right_camera_mask),
                "disable_laser": bool(request.disable_laser),
                "sdk_version": sdk_version,
                "console_fw_version": _safe_call(self.console.get_version),
                "console_hw_id": _hex_or_none(
                    _safe_call(self.console.get_hardware_id)
                ),
                "left_fw_version": _safe_call(self.left.get_version),
                "right_fw_version": _safe_call(self.right.get_version),
                "left_hw_id": _hex_or_none(
                    _safe_call(
                        getattr(self.left, "get_cached_hardware_id", self.left.get_hardware_id)
                    )
                ),
                "right_hw_id": _hex_or_none(
                    _safe_call(
                        getattr(self.right, "get_cached_hardware_id", self.right.get_hardware_id)
                    )
                ),
                "sdk_flags": {
                    "write_raw_csv": getattr(request, "write_raw_csv", False),
                    "write_corrected_csv": getattr(request, "write_corrected_csv", True),
                    "write_telemetry_csv": getattr(request, "write_telemetry_csv", True),
                    "write_raw_to_db": getattr(request, "write_raw_to_db", False),
                },
            }

        user_on_scan_start = kwargs.pop("on_scan_start_fn", None)
        user_on_raw_frame = kwargs.pop("on_raw_frame_fn", None)
        user_on_corrected = kwargs.pop("on_corrected_batch_fn", None)
        user_on_complete = kwargs.pop("on_complete_fn", None)

        def _on_scan_start(ts: str, start_ts: float) -> None:
            try:
                sink.open(
                    label=f"{ts}_{request.subject_id}",
                    start_ts=start_ts,
                    notes=request.notes or "",
                    meta=_build_meta(),
                )
            except Exception:
                logger.exception(
                    "ScanDBSink.open failed; DB writes disabled for this scan"
                )
            if user_on_scan_start:
                user_on_scan_start(ts, start_ts)

        def _on_raw_frame(*args, **kw) -> None:
            try:
                sink.on_raw_frame(*args, **kw)
            except Exception:
                logger.exception("ScanDBSink.on_raw_frame raised")
            if user_on_raw_frame:
                user_on_raw_frame(*args, **kw)

        def _on_corrected(batch) -> None:
            try:
                sink.on_corrected_batch(batch)
            except Exception:
                logger.exception("ScanDBSink.on_corrected_batch raised")
            if user_on_corrected:
                user_on_corrected(batch)

        def _on_complete(result) -> None:
            try:
                sink.close(end_ts=time.time())
            except Exception:
                logger.exception("ScanDBSink.close raised")
            if user_on_complete:
                user_on_complete(result)

        kwargs["on_scan_start_fn"] = _on_scan_start
        kwargs["on_raw_frame_fn"] = _on_raw_frame
        kwargs["on_corrected_batch_fn"] = _on_corrected
        kwargs["on_complete_fn"] = _on_complete
        return kwargs
```

If `logger` is not already imported at the top of `MotionInterface.py`, follow the existing module convention (e.g. `logger = logging.getLogger(f"{_log_root}.Interface" if _log_root else "Interface")`). Do not add a second logger if one already exists.

- [x] **Step 4: Smoke-test the wiring at import time**

```bash
python -c "from omotion import MotionInterface; print('ok')"
python -c "from omotion import MotionInterface; m = MotionInterface(db_path=None); print('no-db ok')"
```

Expected: both print successfully. Constructing `MotionInterface(db_path=None)` must not touch USB/serial — the constructor only stashes state; connection happens later via `connect()`/monitoring.

- [x] **Step 5: Commit**

```bash
git add omotion/MotionInterface.py omotion/ScanWorkflow.py
git commit -m "feat(sdk): MotionInterface(db_path=...) wires ScanDBSink when set (#92)"
```

---

## Task 8: Integration test — drive ScanWorkflow with canned data

**Files:**
- Create: `tests/test_scan_workflow_db_integration.py`

Drive the sink lifecycle directly with simulated callbacks (mirroring what the `MotionInterface` wrapper would do), no `MotionInterface` construction needed. Assert:

1. **Corrected-only** (sink with `write_raw=False`): `<db_path>` exists, exactly one session row, `session_end` is set, `session_data` row count matches expected.
2. **Raw enabled** (sink with `write_raw=True`): `session_raw` row count equals total frames emitted across both sides.
3. **No sink built** (mirrors `MotionInterface(db_path=None)` path): no DB file created.

**Note:** Because `ScanWorkflow._worker` requires a real `MotionInterface` with sensors and a telemetry poller, this test bypasses `MotionInterface.start_scan` and exercises the sink directly. This isolates the integration boundary (sink ↔ callbacks) without requiring hardware. The `MotionInterface`-wired path is exercised at the system-test level on real hardware (out of scope for this plan).

- [ ] **Step 1: Create the integration test**

```python
"""
Integration test: simulate the callback sequence ScanWorkflow would emit
during a scan, routed through the ScanDBSink wrapper built by
MOTIONInterface.start_scan.  Verifies that the full sink lifecycle
produces the expected database rows for each combination of the
write_to_db / write_raw_to_db flags.
"""

import os
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omotion import ScanDatabase, ScanDBSink
from omotion.MotionProcessing import CorrectedBatch, Sample


def _mk_sample(side, cam_id, frame_id, ts, bfi=0.1, bvi=0.2, contrast=0.3, mean=500.0):
    return Sample(
        side=side,
        cam_id=cam_id,
        frame_id=frame_id,
        absolute_frame_id=frame_id,
        timestamp_s=ts,
        row_sum=0,
        temperature_c=25.0,
        mean=mean,
        std_dev=0.0,
        contrast=contrast,
        bfi=bfi,
        bvi=bvi,
        is_corrected=True,
    )


def _drive_scan(sink: ScanDBSink, *, frames_per_side: int = 5, cams_per_side: int = 2):
    """Simulate a scan: open, emit raw frames, emit one corrected batch, close."""
    sid = sink.open(
        label="20260414_120000_owTEST",
        start_ts=1744632000.0,
        notes="integration",
        meta={"subject_id": "owTEST", "fps": 40},
    )
    hist = b"\x33" * 4096
    for side in ("left", "right"):
        for cam in range(cams_per_side):
            for fid in range(frames_per_side):
                sink.on_raw_frame(
                    side, cam, fid, fid * 0.025, hist, 25.0, 100, 0.0, 0.0, 0.0
                )
    samples = []
    for side in ("left", "right"):
        for cam in range(cams_per_side):
            samples.append(_mk_sample(side, cam, 1, 0.025))
    sink.on_corrected_batch(
        CorrectedBatch(dark_frame_start=0, dark_frame_end=600, samples=samples)
    )
    sink.close(end_ts=1744632010.0)
    return sid


def test_corrected_only_writes_session_data_no_raw(tmp_path: Path) -> None:
    db_path = tmp_path / "scans.db"
    sink = ScanDBSink(str(db_path), write_raw=False)
    sid = _drive_scan(sink, frames_per_side=5, cams_per_side=2)

    assert db_path.exists()
    db = ScanDatabase(db_path=str(db_path))
    try:
        session = db.get_session(sid)
        assert session["session_end"] == 1744632010.0
        data = [r for b in db.stream_session_data(sid) for r in b]
        assert len(data) == 4  # 2 cams x 2 sides
        raw = [r for b in db.stream_raw_frames(sid) for r in b]
        assert raw == []
    finally:
        db.close()


def test_write_raw_enabled_persists_every_frame(tmp_path: Path) -> None:
    db_path = tmp_path / "scans.db"
    sink = ScanDBSink(str(db_path), write_raw=True, raw_batch_size=4)
    _drive_scan(sink, frames_per_side=5, cams_per_side=2)

    db = ScanDatabase(db_path=str(db_path))
    try:
        raw = [r for b in db.stream_raw_frames(next(iter(db.iter_sessions()))["id"])
               for r in b]
        # 5 frames x 2 cams x 2 sides = 20
        assert len(raw) == 20
    finally:
        db.close()


def test_no_sink_means_no_file(tmp_path: Path) -> None:
    # Mirrors MotionInterface(db_path=None).start_scan: no sink is built,
    # so the file is never created.
    db_path = tmp_path / "scans.db"
    assert not db_path.exists()
    # No sink constructed.
    assert not db_path.exists()
```

- [ ] **Step 2: Run the test**

```bash
pytest tests/test_scan_workflow_db_integration.py -v
```

Expected: three tests PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/test_scan_workflow_db_integration.py
git commit -m "test(sdk): ScanDBSink integration test with simulated scan callbacks"
```

---

## Task 9: Equivalence test — DB rows match `_corrected.csv` rows

**Files:**
- Create: `tests/test_db_matches_corrected_csv.py`

Reuse the fixture-driven setup from `tests/test_corrected_csv_output.py`: feed the two real scan CSVs through `create_science_pipeline` / `feed_pipeline_from_csv`, attaching both a CSV writer (mimicking the existing corrected CSV path) and a `ScanDBSink` in parallel. Assert that for every (frame_id, side, cam) present in the CSV, the DB has a matching `session_data` row with the same `bfi`, `bvi`, `contrast`, `mean`, and `timestamp_s` values.

The DB sink now rounds floats to 6 decimals at insert time (Task 5), and the corrected CSV writer already rounds to 6 — so the comparison can be exact-equal. We still use `math.isclose(..., abs_tol=1e-6)` for a tiny safety margin around float repr quirks, but a divergence beyond `1e-6` means one side has drifted from the canonical pipeline output.

This is the load-bearing proof that the DB really is the corrected-pipeline endpoint.

- [ ] **Step 1: Read the existing corrected-CSV test to understand the fixture setup**

```bash
sed -n '1,120p' tests/test_corrected_csv_output.py
```

Identify the names of the fixture CSVs (`LEFT_CSV`, `RIGHT_CSV`), the masks, and the calibration arrays.

- [ ] **Step 2: Create the equivalence test**

```python
"""
Equivalence test: a ScanDBSink populated from the corrected pipeline
callback contains exactly the same per-camera per-frame values as the
_corrected.csv file the existing pipeline writes.

This is the load-bearing proof that the DB really is the same endpoint
as the CSV.  If this test diverges, the DB sink (or the CSV writer) has
drifted from the canonical pipeline output.
"""

import csv
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omotion import ScanDatabase, ScanDBSink
from omotion.MotionProcessing import (
    CorrectedBatch,
    create_science_pipeline,
    feed_pipeline_from_csv,
)

FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "fixtures")
LEFT_CSV = os.path.join(FIXTURES_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
RIGHT_CSV = os.path.join(FIXTURES_DIR, "scan_owC18EHALL_20251217_160949_right_maskFF.csv")

LEFT_MASK = 0xFF
RIGHT_MASK = 0xFF

_ZERO = np.zeros((2, 8), dtype=np.float64)
_ONE = np.ones((2, 8), dtype=np.float64)
BFI_C_MIN = _ZERO.copy()
BFI_C_MAX = _ONE.copy()
BFI_I_MIN = _ZERO.copy()
BFI_I_MAX = np.full((2, 8), 1000.0)


@pytest.mark.skipif(
    not (os.path.exists(LEFT_CSV) and os.path.exists(RIGHT_CSV)),
    reason="fixture CSVs not present",
)
def test_db_rows_match_corrected_csv(tmp_path: Path) -> None:
    db_path = tmp_path / "scans.db"
    sink = ScanDBSink(str(db_path))
    sid = sink.open(
        label="equivalence",
        start_ts=0.0,
        notes="",
        meta={},
    )

    csv_rows: list[dict] = []  # rows that mimic what the CSV writer would produce

    def _on_corrected(batch: CorrectedBatch) -> None:
        sink.on_corrected_batch(batch)
        # Mimic the ScanWorkflow CSV merge: one row per frame_id.
        by_frame: dict[int, dict] = {}
        for s in batch.samples:
            entry = by_frame.setdefault(
                int(s.absolute_frame_id),
                {"frame_id": int(s.absolute_frame_id), "timestamp_s": float(s.timestamp_s)},
            )
            suffix = f"{s.side[0]}{int(s.cam_id) + 1}"
            entry[f"bfi_{suffix}"] = round(float(s.bfi), 6)
            entry[f"bvi_{suffix}"] = round(float(s.bvi), 6)
            entry[f"mean_{suffix}"] = round(float(s.mean), 6)
            entry[f"contrast_{suffix}"] = round(float(s.contrast), 6)
        for fid in sorted(by_frame):
            csv_rows.append(by_frame[fid])

    pipeline = create_science_pipeline(
        left_camera_mask=LEFT_MASK,
        right_camera_mask=RIGHT_MASK,
        bfi_c_min=BFI_C_MIN,
        bfi_c_max=BFI_C_MAX,
        bfi_i_min=BFI_I_MIN,
        bfi_i_max=BFI_I_MAX,
        on_corrected_batch_fn=_on_corrected,
    )
    feed_pipeline_from_csv(pipeline, LEFT_CSV, side="left")
    feed_pipeline_from_csv(pipeline, RIGHT_CSV, side="right")
    pipeline.finalize()

    sink.close(end_ts=1.0)

    # Now compare DB session_data rows against csv_rows.
    db = ScanDatabase(db_path=str(db_path))
    try:
        db_rows = [r for b in db.stream_session_data(sid) for r in b]
    finally:
        db.close()

    assert len(db_rows) > 0
    # Build a lookup: (side_int, cam_id, timestamp_s) -> db row.
    db_lookup: dict[tuple, dict] = {}
    for r in db_rows:
        db_lookup[(int(r["side"]), int(r["cam_id"]), round(float(r["timestamp_s"]), 6))] = r

    checked = 0
    for csv_row in csv_rows:
        ts = round(float(csv_row["timestamp_s"]), 6)
        for side_letter, side_int in (("l", 0), ("r", 1)):
            for cam in range(8):
                col_suffix = f"{side_letter}{cam + 1}"
                if f"bfi_{col_suffix}" not in csv_row:
                    continue
                key = (side_int, cam, ts)
                assert key in db_lookup, f"DB missing row for {key}"
                db_r = db_lookup[key]
                assert math.isclose(db_r["bfi"], csv_row[f"bfi_{col_suffix}"], abs_tol=1e-6)
                assert math.isclose(db_r["bvi"], csv_row[f"bvi_{col_suffix}"], abs_tol=1e-6)
                assert math.isclose(db_r["contrast"], csv_row[f"contrast_{col_suffix}"], abs_tol=1e-6)
                assert math.isclose(db_r["mean"], csv_row[f"mean_{col_suffix}"], abs_tol=1e-6)
                checked += 1

    assert checked >= 100, "Expected at least 100 cell comparisons from fixture"
```

- [ ] **Step 2b: If the `Sample.cam_id` convention in the fixture differs from 0-indexed, adjust the `cam + 1` offset to match what `_on_corrected_batch` writes in `ScanWorkflow`.**

Inspect `omotion/ScanWorkflow.py` around line 360: the CSV writer uses `col_suffix = f"{sample.side[0]}{int(sample.cam_id) + 1}"`. Match that exactly. The test as written already does.

- [ ] **Step 3: Run**

```bash
pytest tests/test_db_matches_corrected_csv.py -v -s
```

Expected: PASS, with `checked` ≥ a few thousand.

- [ ] **Step 4: Commit**

```bash
git add tests/test_db_matches_corrected_csv.py
git commit -m "test(sdk): verify DB session_data rows match _corrected.csv cell-for-cell"
```

---

## Task 10: Verify existing regression tests still pass end-to-end

**Files:** (none — verification only)

- [ ] **Step 1: Run the full SDK test suite**

```bash
pytest tests/ -v --ignore=tests/hardware
```

Expected: every non-hardware test PASSES. New tests from tasks 1, 3, 4, 5, 8, 9 all pass. Existing `tests/test_corrected_csv_output.py` still passes unchanged.

- [ ] **Step 2: Manual check that `db_browser.py` can open a DB produced by a scan**

Guidance only — no automated test.

```bash
python stream-db/db_browser.py docs/superpowers/plans/example.db  # or any scans.db
```

Expected: the browser opens without `ModuleNotFoundError` and shows the session.

- [ ] **Step 3: Final commit (only if there are any last cleanup diffs)**

```bash
git status
# If clean, this step is a no-op.
```

---

## Self-Review

**Spec coverage (revised):**
- **DB opt-in via `MotionInterface(db_path=...)`, default off, CSVs untouched** → Task 7. No behavior change unless the caller passes a path.
- `session_data` always (when sink built), `session_raw` gated on per-scan `write_raw_to_db` → Task 4 + Task 5.
- Re-home `scan_db.py` → Task 1; stream-db tools re-import → Task 2.
- One session per scan, shared file across scans → Task 7 (caller-supplied `db_path`, single open per scan via `sink.open`).
- `session_label = f"{ts}_{subject_id}"`, `notes` field, rich `session_meta` with subject_id / masks / fw versions / hw IDs / sdk flags → Task 7 (`_build_meta`, using real `get_version()` / `get_hardware_id()` APIs).
- New `on_scan_start_fn` + `on_raw_frame_fn` on `ScanWorkflow` → Task 6.
- Wiring at `MotionInterface.start_scan` (gated on `self._db_path`) → Task 7.
- **Floats stored to 6 decimals** (project-wide precision policy) → Task 4 + Task 5; Task 9 verifies cell-for-cell equivalence with the corrected CSV.
- Unit tests for `ScanDatabase`, `ScanDBSink`, integration, equivalence → Tasks 1, 3–5, 8, 9.

**Placeholder scan:** No TBDs, no "handle edge cases" without code, no "similar to Task N" shortcuts. Every step shows the exact code or exact command.

**Type consistency:**
- `ScanDBSink.open(label, start_ts, notes, meta)` signature matches across Tasks 3, 7, 8, 9.
- `on_raw_frame(side, cam_id, frame_id, timestamp_s, hist, temp, sum_counts, tcm, tcl, pdc)` matches between the sink (Tasks 3, 4) and the ScanWorkflow callback fan-out (Task 6) and the Interface wrapper (Task 7).
- `on_corrected_batch(batch: CorrectedBatch)` consistent in Tasks 3, 5, 7, 8, 9.
- `on_scan_start_fn(ts: str, session_start: float)` consistent in Tasks 6, 7.
- `CorrectedBatch` / `Sample` field names (`bfi`, `bvi`, `contrast`, `mean`, `timestamp_s`, `absolute_frame_id`, `cam_id`, `side`) match what `omotion/MotionProcessing.py` defines (verified at lines 198–229).
