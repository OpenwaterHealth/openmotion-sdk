"""Contact quality in the corrected record — end to end (bloodflow-app#589).

Replays a recorded scan through default_pipeline with a live
ContactQualityMonitor and a ScanDBSink wired to it (what ScanWorkflow does
when the request attaches a monitor), then checks that every persisted row
carries the verdict the monitor had latched *at that row's frame* — not
whatever it held when the row was written, which is up to one dark interval
later — and that the export surfaces it as cq_* columns.
"""

from __future__ import annotations

import csv
import pathlib
import sqlite3
from dataclasses import dataclass

import numpy as np
import pytest

from omotion.contact_quality import CQThresholds, ContactQualityMonitor
from omotion.pipeline.factory import default_pipeline
from omotion.pipeline.pedestal import SensorPedestals
from omotion.pipeline.runner import ScanRunner
from omotion.pipeline.sinks import ScanDBSink, ScanMetadata
from omotion.pipeline.sources import CsvReplaySource
from omotion.SessionPlayback import materialize_corrected_csv

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


class _FlippingMonitor(ContactQualityMonitor):
    """A real monitor whose light threshold is raised mid-scan, so the
    recorded history has an ok -> poor_contact edge partway through."""

    def __init__(self, flip_after_batches, **kw):
        super().__init__(**kw)
        self._batches = 0
        self._flip_after = flip_after_batches

    def consume(self, channel, batch):
        self._batches += 1
        if self._batches == self._flip_after:
            self._thresholds = CQThresholds.from_sequences([1e9] * 8, [1e9] * 8)
        super().consume(channel, batch)


def test_db_rows_carry_the_verdict_latched_at_their_frame(tmp_path):
    raw_csv = HERE / "normal_short_scan.raw.csv"
    if not raw_csv.exists():
        pytest.skip("Raw fixture not found — run regenerate_goldens.py")

    meta = ScanMetadata(
        scan_id="cq", subject_id="subj", operator="op",
        started_at_iso="2026-09-23T00:00:00Z", duration_sec=10,
        left_camera_mask=0x01, right_camera_mask=0, reduced_mode=False,
    )
    pipeline = default_pipeline(
        metadata=meta,
        calibration=_trivial_calibration(),
        pedestals=SensorPedestals(left=_PEDESTAL, right=_PEDESTAL),
        dark_interval=_DARK_INTERVAL,
    )
    source = CsvReplaySource(
        raw_csv_left=raw_csv, raw_csv_right=None,
        batch_size_frames=7, metadata=meta,
    )
    monitor = _FlippingMonitor(
        flip_after_batches=4,
        thresholds=CQThresholds.from_sequences([1e9] * 8, [0.0] * 8),
        on_transition=lambda *a: None,
        rolling_window=1, light_activate_debounce=1, light_clear_debounce=1,
    )
    db_path = str(tmp_path / "scan.db")
    db_sink = ScanDBSink(db_path=db_path, cq_source=monitor)
    ScanRunner(source=source, pipeline=pipeline, sinks=[db_sink, monitor]).run()

    conn = sqlite3.connect(db_path)
    rows = conn.execute(
        "SELECT frame_id, cam_id, correction_status, contact_quality"
        " FROM session_data ORDER BY frame_id").fetchall()
    sid = conn.execute("SELECT id FROM sessions").fetchone()[0]
    conn.close()

    assert rows, "no corrected rows persisted"
    verdicts = [r[3] for r in rows]
    assert "ok" in verdicts and "poor_contact" in verdicts, verdicts
    for fid, cam_id, status, cq in rows:
        assert cq == monitor.verdict_at("left", cam_id, fid), fid
        assert "ok" not in status.split(",")      # clean is empty, never 'ok'
    # Monotone: one edge, so ok strictly precedes poor_contact.
    first_poor = verdicts.index("poor_contact")
    assert set(verdicts[:first_poor]) == {"ok"}
    assert set(verdicts[first_poor:]) == {"poor_contact"}

    out = str(tmp_path / "export.csv")
    materialize_corrected_csv(db_path, sid, out, include_status=True)
    with open(out, newline="", encoding="utf-8") as fh:
        exported = list(csv.DictReader(fh))
    assert [r["cq_l1"] for r in exported] == verdicts
    assert all(r["cq_l2"] == "" for r in exported)   # outside the mask
