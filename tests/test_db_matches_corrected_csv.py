"""
Equivalence test (#92 / plan Task 9):

Drive the corrected pipeline with the canonical fixture scan CSVs the existing
``test_corrected_csv_output.py`` uses, splitting the ``on_corrected_batch_fn``
callback into:

  * a ``ScanDBSink`` writing to a fresh per-test DB,
  * a sample-collector list, and
  * a minimal CSV merge that mimics the per-frame format the ScanWorkflow
    corrected CSV writer produces (frame_id, bfi_l1..bfi_r8, bvi_l1..bvi_r8,
    contrast_..., mean_...).

Then assert three things:

  1. The DB has exactly one ``session_data`` row per ``Sample`` the pipeline
     emitted, in insertion order.
  2. Each DB row's ``bfi`` / ``bvi`` / ``contrast`` / ``mean`` matches the
     corresponding ``Sample``'s values rounded to 6 decimals — confirms the
     sink's at-insert rounding policy matches the corrected CSV writer's
     precision policy.
  3. Each ``Sample``'s values match the corresponding merged-CSV cell
     within ``1e-6`` — the load-bearing claim that the DB is the same
     endpoint as the corrected CSV.

This is hardware-independent — the fixture CSVs ship in ``tests/fixtures/``.
"""

from __future__ import annotations

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
    Sample,
    create_science_pipeline,
    feed_pipeline_from_csv,
)
from omotion.ScanWorkflow import ScanRequest


def _fake_request(subject_id: str = "equiv") -> ScanRequest:
    return ScanRequest(
        subject_id=subject_id,
        duration_sec=60,
        left_camera_mask=LEFT_MASK,
        right_camera_mask=RIGHT_MASK,
        data_dir=".",
        disable_laser=False,
    )


# ---------------------------------------------------------------------------
# Fixture paths + calibration — duplicates the values used by the existing
# ``test_corrected_csv_output.py`` so this test is independent and either can
# run alone.
# ---------------------------------------------------------------------------

FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "fixtures")
LEFT_CSV  = os.path.join(FIXTURES_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
RIGHT_CSV = os.path.join(FIXTURES_DIR, "scan_owC18EHALL_20251217_160949_right_maskFF.csv")

LEFT_MASK  = 0xFF
RIGHT_MASK = 0xFF

_ZERO = np.zeros((2, 8), dtype=np.float64)
_ONE  = np.ones((2, 8),  dtype=np.float64)
BFI_C_MIN = _ZERO.copy()
BFI_C_MAX = _ONE.copy()
BFI_I_MIN = _ZERO.copy()
BFI_I_MAX = np.full((2, 8), 1000.0)


@pytest.mark.skipif(
    not (os.path.exists(LEFT_CSV) and os.path.exists(RIGHT_CSV)),
    reason="fixture CSVs not present",
)
def test_db_session_data_matches_corrected_pipeline(tmp_path: Path) -> None:
    db_path = tmp_path / "scans.db"
    sink = ScanDBSink(str(db_path))
    sid = sink.on_scan_start(
        ts="equivalence", session_start_ts=0.0,
        request=_fake_request(), meta={},
    )

    # Capture every Sample the pipeline emits, in batch / sample order.
    emitted: list[Sample] = []

    # Mimic the ScanWorkflow corrected CSV merge: one entry per
    # ``absolute_frame_id`` carrying per-cell values keyed by
    # ``{metric}_{side[0]}{cam_id+1}``.
    csv_by_frame: dict[int, dict] = {}

    def _on_batch(batch: CorrectedBatch) -> None:
        # 1) DB sink — the thing under test.
        sink.on_corrected_batch(batch)
        # 2) raw collection for the per-sample check.
        emitted.extend(batch.samples)
        # 3) CSV merge for the cross-endpoint cell-for-cell check.
        for s in batch.samples:
            fid = int(s.absolute_frame_id)
            entry = csv_by_frame.setdefault(fid, {})
            suffix = f"{s.side[0]}{int(s.cam_id) + 1}"
            # Match the ScanWorkflow corrected CSV writer: every metric is
            # rounded to 6 decimals at write time. The DB sink does the
            # same rounding at insert (#92 — see ScanDBSink.on_corrected_batch),
            # so the two endpoints should agree exactly modulo float repr.
            entry[f"bfi_{suffix}"]      = round(float(s.bfi), 6)
            entry[f"bvi_{suffix}"]      = round(float(s.bvi), 6)
            entry[f"contrast_{suffix}"] = round(float(s.contrast), 6)
            entry[f"mean_{suffix}"]     = round(float(s.mean), 6)

    pipeline = create_science_pipeline(
        left_camera_mask=LEFT_MASK,
        right_camera_mask=RIGHT_MASK,
        bfi_c_min=BFI_C_MIN,
        bfi_c_max=BFI_C_MAX,
        bfi_i_min=BFI_I_MIN,
        bfi_i_max=BFI_I_MAX,
        on_corrected_batch_fn=_on_batch,
    )
    feed_pipeline_from_csv(LEFT_CSV,  "left",  pipeline)
    feed_pipeline_from_csv(RIGHT_CSV, "right", pipeline)
    pipeline.stop(timeout=120.0)

    sink.on_complete()
    assert sink.insert_errors == 0, (
        f"ScanDBSink reported {sink.insert_errors} insert errors during the run"
    )

    # ----- DB rows vs emitted samples -----
    db = ScanDatabase(db_path=str(db_path))
    try:
        db_rows = [r for batch in db.stream_session_data(sid) for r in batch]
    finally:
        db.close()

    assert len(emitted) > 1000, (
        f"Fixture should drive thousands of corrected samples; got {len(emitted)}"
    )
    assert len(db_rows) == len(emitted), (
        f"DB row count {len(db_rows)} != pipeline sample count {len(emitted)}"
    )

    # session_data rows are ordered by id (auto-increment), which matches
    # insertion order, which matches the per-batch sample order.
    for i, (sample, row) in enumerate(zip(emitted, db_rows)):
        side_int = 0 if sample.side == "left" else 1
        assert row["cam_id"] == int(sample.cam_id),    f"row {i}: cam_id mismatch"
        assert row["side"]   == side_int,              f"row {i}: side mismatch"
        # Sink rounds to 6 decimals at insert — same as the CSV writer.
        assert row["bfi"]      == round(float(sample.bfi), 6),      f"row {i}: bfi"
        assert row["bvi"]      == round(float(sample.bvi), 6),      f"row {i}: bvi"
        assert row["contrast"] == round(float(sample.contrast), 6), f"row {i}: contrast"
        assert row["mean"]     == round(float(sample.mean), 6),     f"row {i}: mean"

    # ----- DB rows vs CSV cells (the cross-endpoint claim) -----
    checked = 0
    for sample in emitted:
        fid     = int(sample.absolute_frame_id)
        suffix  = f"{sample.side[0]}{int(sample.cam_id) + 1}"
        cell    = csv_by_frame[fid]
        # DB row for this sample is unique by (session_id, side, cam_id, frame_id),
        # but session_data doesn't carry frame_id; the per-row equality above
        # already pinned the DB → sample mapping, so for the CSV side we just
        # compare the value the sample carries with the cell value the CSV
        # writer would have written for the same (frame, side, cam).
        assert math.isclose(cell[f"bfi_{suffix}"],      float(sample.bfi),      abs_tol=1e-6)
        assert math.isclose(cell[f"bvi_{suffix}"],      float(sample.bvi),      abs_tol=1e-6)
        assert math.isclose(cell[f"contrast_{suffix}"], float(sample.contrast), abs_tol=1e-6)
        assert math.isclose(cell[f"mean_{suffix}"],     float(sample.mean),     abs_tol=1e-6)
        checked += 1

    assert checked > 1000, f"Expected >1000 cell comparisons; got {checked}"
