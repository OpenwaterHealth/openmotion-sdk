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


def _make_db(tmp_path, rows, *, reduced=False, left_mask=0x03, right_mask=0):
    """A one-session DB holding ``rows`` (session_id filled in here)."""
    db_path = str(tmp_path / "scans.db")
    db = ScanDatabase(db_path=db_path)
    sid = db.create_session(
        session_label="s1", session_start=1.0, session_notes=None,
        session_meta={
            "data_semantics": "final",
            "sdk_flags": {"reduced_mode": reduced,
                          "left_camera_mask": left_mask,
                          "right_camera_mask": right_mask},
        },
    )
    db.insert_session_data_rows([{**r, "session_id": sid} for r in rows])
    db.close()
    return db_path, sid


def _row(frame_id, cam_id, *, side=0, t=None, bfi=4.0, bvi=6.0, mean=100.0,
         contrast=0.02, temp=None, status="", cq=None):
    return {
        "cam_id": cam_id, "side": side, "frame_id": frame_id,
        "timestamp_s": (t if t is not None else frame_id * 0.025),
        "bfi": bfi, "bvi": bvi, "mean": mean, "contrast": contrast,
        "temp": temp, "correction_status": status, "contact_quality": cq,
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
    materialize_corrected_csv(db_path, sid, out, include_status=True)

    rows = _read(out)
    assert [r["frame_id"] for r in rows] == ["10", "11", "12"]
    assert rows[0]["temp_l1"] == "" and rows[0]["temp_l2"] == ""
    assert float(rows[1]["temp_l1"]) == pytest.approx(45.625)
    assert float(rows[1]["temp_l2"]) == pytest.approx(44.5)
    assert float(rows[2]["temp_l1"]) == pytest.approx(45.75)
    # Untouched columns still come through alongside.
    assert float(rows[1]["bfi_l1"]) == pytest.approx(4.0)
    assert rows[1]["correction_status"] == ""


def test_export_header_matches_live_writer(tmp_path):
    """The export's column layout is CsvSink's own header list (plus the
    export-only status columns) — the two writers cannot drift."""
    db_path, sid = _make_db(tmp_path, [_row(10, 0, temp=40.0)])
    out = str(tmp_path / "out.csv")
    materialize_corrected_csv(db_path, sid, out, include_status=True)

    with open(out, newline="", encoding="utf-8") as fh:
        header = next(csv.reader(fh))
    expected = list(_NORMAL_HEADERS)
    expected += ["correction_status"]
    expected += [f"cq_l{i}" for i in range(1, 9)]
    expected += [f"cq_r{i}" for i in range(1, 9)]
    assert header == expected


# ---------------------------------------------------------------------------
# correction_status + cq_* columns (bloodflow-app#589)
# ---------------------------------------------------------------------------

def test_export_joins_corrections_into_one_camera_tagged_column(tmp_path):
    db_path, sid = _make_db(tmp_path, [
        _row(10, 0),
        _row(10, 1),
        _row(11, 0, status="nan_filled"),
        _row(11, 1, status="ts_corrected"),
        _row(11, 4, side=1, status="ts_corrected"),
        _row(12, 0, status="ok"),                  # legacy pre-v3 clean marker
    ], right_mask=0x10)
    out = str(tmp_path / "out.csv")
    materialize_corrected_csv(db_path, sid, out, include_status=True)
    rows = _read(out)
    assert [r["correction_status"] for r in rows] == [
        "", "l1:nan_filled,l2:ts_corrected,r5:ts_corrected", ""]
    assert not any(k.startswith("quality") for k in rows[0])


def test_export_cq_columns_per_camera(tmp_path):
    db_path, sid = _make_db(tmp_path, [
        _row(10, 0, cq="ok"),
        _row(10, 1, cq="poor_contact"),
        _row(11, 0, cq="poor_contact,ambient_light"),
        _row(11, 1),                                # recorded unmonitored
    ])
    out = str(tmp_path / "out.csv")
    materialize_corrected_csv(db_path, sid, out, include_status=True)
    rows = _read(out)
    assert (rows[0]["cq_l1"], rows[0]["cq_l2"]) == ("ok", "poor_contact")
    assert (rows[1]["cq_l1"], rows[1]["cq_l2"]) == ("poor_contact,ambient_light", "")
    assert rows[0]["cq_l3"] == "" and rows[0]["cq_r1"] == ""  # not in the scan


def test_export_include_quality_is_an_alias(tmp_path):
    db_path, sid = _make_db(tmp_path, [_row(10, 0, cq="ok")])
    out = str(tmp_path / "out.csv")
    materialize_corrected_csv(db_path, sid, out, include_quality=True)
    assert _read(out)[0]["cq_l1"] == "ok"


def test_export_without_status_has_no_status_columns(tmp_path):
    db_path, sid = _make_db(tmp_path, [_row(10, 0, cq="ok")])
    out = str(tmp_path / "out.csv")
    materialize_corrected_csv(db_path, sid, out)
    assert "correction_status" not in _read(out)[0]


def test_export_reduced_mode_decodes_side_average_lists(tmp_path):
    """Clinical (reduced) sessions store one tagged row per side; the export
    still yields one correction_status column and per-camera cq_ cells."""
    db_path, sid = _make_db(tmp_path, [
        _row(10, -1, side=0, status="l2:nan_filled", cq="l2:poor_contact"),
        _row(10, -1, side=1, status="", cq=""),
        _row(11, -1, side=0, status="", cq=None),     # unmonitored
    ], reduced=True, left_mask=0x03, right_mask=0x01)
    out = str(tmp_path / "out.csv")
    materialize_corrected_csv(db_path, sid, out, include_status=True)
    rows = _read(out)
    assert rows[0]["correction_status"] == "l2:nan_filled"
    assert (rows[0]["cq_l1"], rows[0]["cq_l2"], rows[0]["cq_r1"]) == (
        "ok", "poor_contact", "ok")
    assert rows[0]["cq_l3"] == "" and rows[0]["cq_r2"] == ""   # masked off
    assert rows[1]["cq_l1"] == ""
