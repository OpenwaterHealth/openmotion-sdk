"""materialize_corrected_csv — DB → corrected-format CSV export.

This is the backend of the bloodflow-app's History → Export CSV. Before
issue #221 it had no direct coverage, and the temp_* columns it emitted
were always empty (the DB stored no temperature).
"""

import csv

import pytest

from omotion.ScanDatabase import ScanDatabase
from omotion.SessionPlayback import materialize_corrected_csv
from omotion.pipeline.sinks import _NORMAL_HEADERS


def _make_db(tmp_path, rows):
    """A one-session DB holding ``rows`` (session_id filled in here)."""
    db_path = str(tmp_path / "scans.db")
    db = ScanDatabase(db_path=db_path)
    sid = db.create_session(
        session_label="s1", session_start=1.0, session_notes=None,
        session_meta={
            "data_semantics": "final",
            "sdk_flags": {"reduced_mode": False,
                          "left_camera_mask": 0x03, "right_camera_mask": 0},
        },
    )
    db.insert_session_data_rows([{**r, "session_id": sid} for r in rows])
    db.close()
    return db_path, sid


def _row(frame_id, cam_id, *, t=None, bfi=4.0, bvi=6.0, mean=100.0,
         contrast=0.02, temp=None, quality="ok"):
    return {
        "cam_id": cam_id, "side": 0, "frame_id": frame_id,
        "timestamp_s": (t if t is not None else frame_id * 0.025),
        "bfi": bfi, "bvi": bvi, "mean": mean, "contrast": contrast,
        "temp": temp, "quality": quality,
    }


def _read(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def test_export_populates_temp_columns(tmp_path):
    """Rows carry their stored camera temperature; rows without one (NULL —
    e.g. recorded pre-#221) leave the cell empty."""
    db_path, sid = _make_db(tmp_path, [
        _row(10, 0),                     # no stored stamp (NULL temp)
        _row(10, 1),
        _row(11, 0, temp=45.625),
        _row(11, 1, temp=44.5),
        _row(12, 0, temp=45.75),
        _row(12, 1, temp=44.625),
    ])
    out = str(tmp_path / "out.csv")
    materialize_corrected_csv(db_path, sid, out, include_quality=True)

    rows = _read(out)
    assert [r["frame_id"] for r in rows] == ["10", "11", "12"]
    assert rows[0]["temp_l1"] == "" and rows[0]["temp_l2"] == ""
    assert float(rows[1]["temp_l1"]) == pytest.approx(45.625)
    assert float(rows[1]["temp_l2"]) == pytest.approx(44.5)
    assert float(rows[2]["temp_l1"]) == pytest.approx(45.75)
    # Untouched columns still come through alongside.
    assert float(rows[1]["bfi_l1"]) == pytest.approx(4.0)
    assert rows[1]["quality_l1"] == "ok"


def test_export_header_matches_live_writer(tmp_path):
    """The export's column layout is CsvSink's own header list (plus the
    export-only quality columns) — the two writers cannot drift."""
    db_path, sid = _make_db(tmp_path, [_row(10, 0, temp=40.0)])
    out = str(tmp_path / "out.csv")
    materialize_corrected_csv(db_path, sid, out, include_quality=True)

    with open(out, newline="", encoding="utf-8") as fh:
        header = next(csv.reader(fh))
    expected = list(_NORMAL_HEADERS)
    expected += [f"quality_l{i}" for i in range(1, 9)]
    expected += [f"quality_r{i}" for i in range(1, 9)]
    assert header == expected
