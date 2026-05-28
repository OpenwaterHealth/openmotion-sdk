"""Reduced-mode round-trip — stored cam_id=-1 == CorrectedSideAverageStage output.

Runs a synthetic reduced-mode scan through default_pipeline + ScanDBSink, reads
the persisted cam_id=-1 side-average rows back, and asserts they equal what
Stage B emitted on the "final_side" channel. This is the single-source-of-truth
guarantee: what the DB stores (and replay reads) IS the corrected side average
the pipeline produced — no divergence, no re-derivation.

Note: this compares the DB against the CORRECTED-path output, NOT the realtime
live display — those differ by design.
"""

from __future__ import annotations

import math
import pathlib
import sqlite3
from dataclasses import dataclass

import numpy as np
import pytest

from omotion.pipeline.factory import default_pipeline
from omotion.pipeline.runner import ScanRunner
from omotion.pipeline.sources import CsvReplaySource
from omotion.pipeline.sinks import ScanDBSink, ScanMetadata
from omotion.pipeline.pedestal import SensorPedestals


HERE = pathlib.Path(__file__).parent / "data"
_PEDESTAL = 64.0
_DARK_INTERVAL = 20  # matches the golden fixture


@dataclass
class _TrivialCal:
    c_min: np.ndarray
    c_max: np.ndarray
    i_min: np.ndarray
    i_max: np.ndarray


def _trivial_calibration() -> _TrivialCal:
    return _TrivialCal(
        c_min=np.zeros((2, 8), dtype=np.float32),
        c_max=np.ones((2, 8), dtype=np.float32),
        i_min=np.zeros((2, 8), dtype=np.float32),
        i_max=np.full((2, 8), 500.0, dtype=np.float32),
    )


def _reduced_meta() -> ScanMetadata:
    return ScanMetadata(
        scan_id="roundtrip", subject_id="subj", operator="op",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=10,
        left_camera_mask=0x01, right_camera_mask=0, reduced_mode=True,
    )


class _FinalSideCapture:
    """Records every SideAverageSample dispatched on the 'final_side' channel —
    i.e. exactly what CorrectedSideAverageStage emitted."""
    channels = {"final_side"}

    def __init__(self):
        self.samples = []

    def on_scan_start(self, meta):
        pass

    def consume(self, channel, payload):
        if channel == "final_side":
            self.samples.append(payload)

    def on_complete(self):
        pass


def _r9(v):
    if v is None:
        return None
    v = float(v)
    return round(v, 9) if math.isfinite(v) else None


def test_reduced_round_trip_stored_equals_corrected_output(tmp_path):
    raw_csv = HERE / "normal_short_scan.raw.csv"
    if not raw_csv.exists():
        pytest.skip("Raw fixture not found — run regenerate_goldens.py")

    meta = _reduced_meta()
    pipeline = default_pipeline(
        metadata=meta,
        calibration=_trivial_calibration(),
        pedestals=SensorPedestals(left=_PEDESTAL, right=_PEDESTAL),
        dark_interval=_DARK_INTERVAL,
    )
    source = CsvReplaySource(
        raw_csv_left=raw_csv, raw_csv_right=None,
        batch_size_frames=20, metadata=meta,
    )
    db_path = tmp_path / "scan.db"
    db_sink = ScanDBSink(db_path=str(db_path))
    capture = _FinalSideCapture()
    ScanRunner(source=source, pipeline=pipeline, sinks=[db_sink, capture]).run()

    # Stage B must have produced a corrected side average (an interval closed).
    assert capture.samples, "no final_side emitted — no interval closed in reduced mode"

    # What the DB persisted at cam_id=-1.
    conn = sqlite3.connect(str(db_path))
    db_rows = conn.execute(
        "SELECT side, frame_id, bfi, bvi, mean, contrast "
        "FROM session_data WHERE cam_id = -1 ORDER BY frame_id, side"
    ).fetchall()
    # Reduced mode persists ONLY the side average — no per-camera rows.
    per_cam = conn.execute(
        "SELECT COUNT(*) FROM session_data WHERE cam_id >= 0"
    ).fetchone()[0]
    conn.close()
    assert per_cam == 0, "reduced mode must not persist per-camera rows"

    # Expected = exactly Stage B's output, rounded as the sink rounds, skipping
    # samples the sink drops (both bfi and bvi non-finite).
    expected = {}
    for s in capture.samples:
        bfi, bvi = _r9(s.bfi), _r9(s.bvi)
        if bfi is None and bvi is None:
            continue
        expected[(int(s.frame_id), int(s.side))] = (bfi, bvi, _r9(s.mean), _r9(s.contrast))

    assert len(db_rows) == len(expected)
    for side, frame_id, bfi, bvi, mean, contrast in db_rows:
        exp = expected[(int(frame_id), int(side))]
        assert bfi == pytest.approx(exp[0])
        assert bvi == pytest.approx(exp[1])
        assert mean == pytest.approx(exp[2])
        assert contrast == pytest.approx(exp[3])
