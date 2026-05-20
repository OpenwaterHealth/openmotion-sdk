"""
SessionPlayback round-trip test (#92 / Step D).

Builds a fresh DB by feeding the fixture scan CSVs through the science
pipeline + ScanDBSink (same path as test_db_matches_corrected_csv),
then calls ``materialize_corrected_csv`` and asserts the resulting CSV
matches an in-memory corrected merge of the exact same Sample stream.

This is the load-bearing claim for DB-only playback: a DB session is
visualizable as a corrected CSV that's value-equivalent to what
ScanWorkflow + CsvSink would have produced live.
"""

from __future__ import annotations

import csv
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omotion import ScanDBSink, materialize_corrected_csv
from omotion.MotionProcessing import (
    CorrectedBatch,
    Sample,
    create_science_pipeline,
    feed_pipeline_from_csv,
)
from omotion.ScanWorkflow import ScanRequest


FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "fixtures")
LEFT_CSV  = os.path.join(FIXTURES_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
RIGHT_CSV = os.path.join(FIXTURES_DIR, "scan_owC18EHALL_20251217_160949_right_maskFF.csv")
LEFT_MASK  = 0xFF
RIGHT_MASK = 0xFF


def _fake_request(subject_id: str = "playback") -> ScanRequest:
    return ScanRequest(
        subject_id=subject_id,
        duration_sec=60,
        left_camera_mask=LEFT_MASK,
        right_camera_mask=RIGHT_MASK,
        data_dir=".",
        disable_laser=False,
    )


@pytest.mark.skipif(
    not (os.path.exists(LEFT_CSV) and os.path.exists(RIGHT_CSV)),
    reason="fixture CSVs not present",
)
def test_materialize_corrected_csv_matches_live_merge(tmp_path: Path) -> None:
    db_path = tmp_path / "scans.db"
    out_csv = tmp_path / "playback.csv"

    sink = ScanDBSink(str(db_path))
    sid = sink.on_scan_start(
        ts="playback", session_start_ts=0.0,
        request=_fake_request(), meta={},
    )

    # In-memory merge that exactly mirrors what CsvSink writes: per-frame
    # buffering keyed by absolute_frame_id, 6-decimal rounding, per-frame
    # timestamp = min over contributors. We compare the materialized
    # CSV against this builder cell-by-cell.
    inline_by_frame: dict[int, dict] = {}

    def _on_batch(batch: CorrectedBatch) -> None:
        sink.on_corrected_batch(batch)
        for s in batch.samples:
            fid = int(s.absolute_frame_id)
            entry = inline_by_frame.get(fid)
            if entry is None:
                entry = {"ts": float(s.timestamp_s), "values": {}}
                inline_by_frame[fid] = entry
            else:
                entry["ts"] = min(entry["ts"], float(s.timestamp_s))
            suffix = f"{s.side[0]}{int(s.cam_id) + 1}"
            entry["values"][f"bfi_{suffix}"]      = round(float(s.bfi), 6)
            entry["values"][f"bvi_{suffix}"]      = round(float(s.bvi), 6)
            entry["values"][f"mean_{suffix}"]     = round(float(s.mean), 6)
            entry["values"][f"contrast_{suffix}"] = round(float(s.contrast), 6)

    _ZERO = np.zeros((2, 8), dtype=np.float64)
    _ONE  = np.ones((2, 8),  dtype=np.float64)
    pipeline = create_science_pipeline(
        left_camera_mask=LEFT_MASK,
        right_camera_mask=RIGHT_MASK,
        bfi_c_min=_ZERO.copy(),
        bfi_c_max=_ONE.copy(),
        bfi_i_min=_ZERO.copy(),
        bfi_i_max=np.full((2, 8), 1000.0),
        on_corrected_batch_fn=_on_batch,
    )
    feed_pipeline_from_csv(LEFT_CSV,  "left",  pipeline)
    feed_pipeline_from_csv(RIGHT_CSV, "right", pipeline)
    pipeline.stop(timeout=120.0)
    sink.on_complete()
    assert sink.insert_errors == 0

    # Round-trip through the DB.
    materialize_corrected_csv(
        str(db_path), session_id=sid, output_path=str(out_csv),
    )
    assert out_csv.exists() and out_csv.stat().st_size > 0

    with open(out_csv, newline="") as f:
        out_rows = list(csv.DictReader(f))

    # Frames the pipeline produced are also the frames the CSV
    # writer/db would have emitted.
    assert len(out_rows) == len(inline_by_frame), (
        f"materialized rows {len(out_rows)} != inline merge frames {len(inline_by_frame)}"
    )

    # Cell-for-cell: every BFI/BVI/contrast/mean cell in the materialized
    # CSV must match the inline merge. The materialized CSV will have
    # empty cells for temp_* (DB doesn't carry temperature) — that's
    # expected; we just don't check those.
    METRICS = ("bfi", "bvi", "contrast", "mean")
    checked = 0
    for out_row in out_rows:
        fid = int(out_row["frame_id"])
        entry = inline_by_frame[fid]
        # Per-frame timestamp: min over contributors. The materialized
        # CSV pulls timestamp from DB, which was rounded at insert. The
        # inline merge keeps unrounded. Compare with isclose.
        assert math.isclose(
            float(out_row["timestamp_s"]),
            round(entry["ts"], 6),
            abs_tol=1e-6,
        )
        for metric in METRICS:
            for side_letter in ("l", "r"):
                for cam in range(1, 9):
                    col = f"{metric}_{side_letter}{cam}"
                    out_cell = out_row[col]
                    inline_cell = entry["values"].get(col, "")
                    if inline_cell == "":
                        # Inactive cam — both should be empty.
                        assert out_cell == ""
                    else:
                        assert out_cell != "", (
                            f"frame {fid} {col}: materialized blank but "
                            f"inline has {inline_cell}"
                        )
                        assert math.isclose(
                            float(out_cell), float(inline_cell), abs_tol=1e-6
                        ), f"frame {fid} {col}: {out_cell} vs {inline_cell}"
                        checked += 1
    assert checked > 1000, f"expected >1000 cells checked, got {checked}"


def test_materialize_raises_for_unknown_session(tmp_path: Path) -> None:
    db_path = tmp_path / "scans.db"
    # Open + close to create an empty DB with the schema.
    sink = ScanDBSink(str(db_path))
    sink.on_complete()
    with pytest.raises(ValueError, match="not found"):
        materialize_corrected_csv(
            str(db_path), session_id=999, output_path=str(tmp_path / "out.csv"),
        )


def test_materialize_raises_for_pre_step_f_session(tmp_path: Path) -> None:
    """A session whose data rows all have frame_id=-1 (sentinel from
    the migration default) is unreconstructable; should raise so the
    caller can fall back to the on-disk CSV."""
    import sqlite3
    db_path = tmp_path / "scans.db"
    sink = ScanDBSink(str(db_path))
    sid = sink.on_scan_start(
        ts="legacy", session_start_ts=0.0,
        request=_fake_request(), meta={},
    )
    sink.on_complete()

    # Insert a few rows directly with frame_id=-1 (simulating a legacy
    # session that pre-dates the schema migration).
    conn = sqlite3.connect(str(db_path))
    conn.executemany(
        "INSERT INTO session_data (session_id, cam_id, side, frame_id, "
        "timestamp_s, bfi, bvi, contrast, mean) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
        [(sid, 0, 0, -1, 0.1, 1.0, 5.0, 0.5, 100.0)],
    )
    conn.commit()
    conn.close()

    with pytest.raises(RuntimeError, match="Step F"):
        materialize_corrected_csv(
            str(db_path), session_id=sid, output_path=str(tmp_path / "out.csv"),
        )
