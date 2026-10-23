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


def _make_db(tmp_path, rows, *, reduced=False):
    """A one-session DB holding ``rows`` (session_id filled in here)."""
    db_path = str(tmp_path / "scans.db")
    db = ScanDatabase(db_path=db_path)
    sid = db.create_session(
        session_label="s1", session_start=1.0, session_notes=None,
        session_meta={
            "data_semantics": "final",
            "sdk_flags": {"reduced_mode": reduced,
                          "left_camera_mask": 0x03, "right_camera_mask": 0},
        },
    )
    db.insert_session_data_rows([{**r, "session_id": sid} for r in rows])
    db.close()
    return db_path, sid


def _row(frame_id, cam_id, *, side=0, t=None, bfi=4.0, bvi=6.0, mean=100.0,
         contrast=0.02, temp=None, quality="ok"):
    return {
        "cam_id": cam_id, "side": side, "frame_id": frame_id,
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


def test_reduced_export_leaves_null_side_average_empty(tmp_path):
    """Reduced (Clinical) layout: a side average stored as NULL is an empty
    cell, not 0.0, and a side's mean counts only its non-NULL values
    (issue #339). Covering every camera on a module is the usual cause:
    contrast is NaN, so BFI is NULL, while BVI is still finite."""
    db_path, sid = _make_db(tmp_path, [
        _row(10, -1, side=0, bfi=None, bvi=-3.0),
        _row(10, -1, side=0, bfi=None, bvi=-3.0),
        _row(10, -1, side=1, bfi=4.0, bvi=6.0),
        _row(11, -1, side=0, bfi=None, bvi=-3.0),
        _row(11, -1, side=0, bfi=2.0, bvi=-5.0),
        _row(11, -1, side=1, bfi=4.0, bvi=6.0),
        _row(12, -1, side=0, bfi=None, bvi=-2.0),   # final frame
    ], reduced=True)
    out = str(tmp_path / "out.csv")
    materialize_corrected_csv(db_path, sid, out, include_quality=True)

    rows = _read(out)
    assert list(rows[0]) == ["frame_id", "timestamp_s", "bfi_left",
                             "bfi_right", "bvi_left", "bvi_right"]
    assert [r["frame_id"] for r in rows] == ["10", "11", "12"]
    assert rows[0]["bfi_left"] == ""
    assert float(rows[0]["bvi_left"]) == pytest.approx(-3.0)
    assert float(rows[0]["bfi_right"]) == pytest.approx(4.0)
    assert float(rows[1]["bfi_left"]) == pytest.approx(2.0)
    assert float(rows[1]["bvi_left"]) == pytest.approx(-4.0)
    assert rows[2]["bfi_left"] == ""
    assert float(rows[2]["bvi_left"]) == pytest.approx(-2.0)
    assert rows[2]["bfi_right"] == "" and rows[2]["bvi_right"] == ""


def test_reduced_export_skips_frame_with_no_value(tmp_path):
    """A reduced frame whose side averages are all NULL gets no row,
    rather than a row of zeros or blanks."""
    db_path, sid = _make_db(tmp_path, [
        _row(10, -1, side=0),
        _row(11, -1, side=0, bfi=None, bvi=None),
        _row(12, -1, side=0),
    ], reduced=True)
    out = str(tmp_path / "out.csv")
    materialize_corrected_csv(db_path, sid, out, include_quality=True)

    assert [r["frame_id"] for r in _read(out)] == ["10", "12"]
