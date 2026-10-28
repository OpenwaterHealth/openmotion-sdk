"""Contact quality in the stored record, end to end (bloodflow-app#589).

A synthetic two-sided scan runs through ``default_pipeline()`` with a live
``ContactQualityMonitor`` and a ``ScanDBSink`` wired to it, as ScanWorkflow
does when the request attaches a monitor. One camera per side loses contact
for a stretch of frames, with the warning raised and cleared in the same
batch that closes a dark interval.

Every stored row must carry the verdict the monitor had latched for that
camera *when it observed that frame*. The expected value is recorded right
after each observation, independently of the monitor's history, so the
test cannot agree with itself. The earlier version (sdk#291) kept one
watermark for both sensors; with single-side batches it stamped a side's
rows before the monitor had seen them, and this test fails on that.
"""

from __future__ import annotations

import csv
import sqlite3
from dataclasses import dataclass

import numpy as np
import pytest

from omotion.config import CAMERA_GAIN_MAP
from omotion.contact_quality import CQThresholds, ContactQualityMonitor
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.factory import default_pipeline
from omotion.pipeline.pedestal import SensorPedestals, adc_gain_for_pedestal
from omotion.pipeline.quality_flags import OK, POOR_CONTACT, join, parse
from omotion.pipeline.runner import ScanRunner
from omotion.pipeline.sinks import ScanDBSink, ScanMetadata
from omotion.SessionPlayback import materialize_corrected_csv
from tests.test_pipeline._speckle import FRAME_PERIOD_S, Camera, Scan, _ListSource

ADC_GAIN = adc_gain_for_pedestal(128.0)
N_FRAMES = 101
FRAMES_PER_BATCH = 3   # darks fall at 21, 41, 61, 81: mid-batch at 40-42, 79-81
CAMS = (0, 1)
LIGHT_THRESHOLD_DN = 20.0
# (side, cam_id) -> frames with poor contact (10 DN above dark: lit, but
# under the light threshold). Right camera 1 is raised at 40 and cleared at
# 79, both inside a batch that also closes a dark (41, 81).
DIM = {("left", 0): range(30, 50), ("right", 1): range(40, 79)}


@dataclass
class _Calibration:
    c_min: np.ndarray
    c_max: np.ndarray
    i_min: np.ndarray
    i_max: np.ndarray


CAL = _Calibration(c_min=np.zeros((2, 8)), c_max=np.full((2, 8), 0.4),
                   i_min=np.zeros((2, 8)), i_max=np.full((2, 8), 300.0))


def _camera(cam_id: int, mean_dn: float) -> Camera:
    gain = float(CAMERA_GAIN_MAP[cam_id])
    return Camera(cam_id=cam_id, dn_per_e=ADC_GAIN * gain,
                  read_sigma=17.0 if gain == 16 else 4.4, mean_dn=mean_dn)


class _RecordingMonitor(ContactQualityMonitor):
    """Records, right after each observation, the verdict then latched."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.seen: dict = {}

    def _observe(self, fid, side, cam_id, *args):
        super()._observe(fid, side, cam_id, *args)
        self.seen[(side, cam_id, fid)] = join(*(
            reason for (s, c, reason), latch in self._latches.items()
            if s == side and c == cam_id and latch.active
        ))


def _batches(scan: Scan, order: str) -> list[FrameBatch]:
    lit = {cam: _camera(cam, 60.0) for cam in CAMS}
    dim = {cam: _camera(cam, 10.0) for cam in CAMS}
    per_side = {0: [], 1: []}
    for side_idx, side in enumerate(("left", "right")):
        for start in range(1, N_FRAMES + 1, FRAMES_PER_BATCH):
            frames = range(start, min(start + FRAMES_PER_BATCH, N_FRAMES + 1))
            rows = len(frames) * len(CAMS)
            hist = np.zeros((rows, 2, 8, 1024), dtype=np.uint32)
            cam_ids, fids, ts = [], [], []
            for r, (n, cam) in enumerate((n, c) for n in frames for c in CAMS):
                model = dim[cam] if n in DIM.get((side, cam), ()) else lit[cam]
                hist[r, side_idx, cam] = scan.histogram(model, n)
                cam_ids.append(cam)
                fids.append(n)
                ts.append((n - 1) * FRAME_PERIOD_S)
            per_side[side_idx].append(FrameBatch(
                cam_ids=np.array(cam_ids, dtype=np.int8),
                frame_ids=np.array(fids, dtype=np.uint8),
                packet_ids=np.repeat(np.arange(start, start + len(frames)), len(CAMS)),
                side_ids=np.full(rows, side_idx, dtype=np.int8),
                raw_histograms=hist,
                temperature_c=np.full((rows, 2, 8), 40.0, dtype=np.float32),
                timestamp_s=np.array(ts),
                pdc=np.full(rows, np.nan, dtype=np.float32),
                tcm=np.zeros(rows, dtype=np.int64), tcl=np.zeros(rows, dtype=np.int64),
            ))
    if order == "interleaved":   # the live source: each sensor's own batches
        return [b for pair in zip(per_side[0], per_side[1]) for b in pair]
    return per_side[0] + per_side[1]   # a CSV replay: one side, then the other


@pytest.mark.parametrize("order", ["interleaved", "left_then_right"])
def test_rows_carry_the_verdict_latched_at_their_frame(tmp_path, order):
    scan = Scan([_camera(c, 60.0) for c in CAMS], n_frames=N_FRAMES)
    mask = sum(1 << c for c in CAMS)
    meta = ScanMetadata(scan_id="cq", subject_id="s", operator="o",
                        started_at_iso="", duration_sec=0,
                        left_camera_mask=mask, right_camera_mask=mask,
                        reduced_mode=False)
    pipeline = default_pipeline(
        metadata=meta, calibration=CAL,
        pedestals=SensorPedestals(left=scan.pedestal, right=scan.pedestal),
        noise_floor_threshold=scan.noise_floor_threshold,
        discard_count=scan.discard_count, dark_interval=scan.dark_interval,
        raw_save_max_duration_s=0,
    )
    monitor = _RecordingMonitor(
        thresholds=CQThresholds.from_sequences([1e9] * 8, [LIGHT_THRESHOLD_DN] * 8),
        on_transition=lambda *a: None,
        rolling_window=1, light_activate_debounce=1, light_clear_debounce=1,
    )
    db_path = str(tmp_path / "scan.db")
    db_sink = ScanDBSink(db_path=db_path, cq_source=monitor)
    ScanRunner(source=_ListSource(meta, _batches(scan, order)), pipeline=pipeline,
               sinks=[db_sink, monitor]).run()

    conn = sqlite3.connect(db_path)
    rows = conn.execute(
        "SELECT side, cam_id, frame_id, quality FROM session_data"
        " ORDER BY side, cam_id, frame_id").fetchall()
    sid = conn.execute("SELECT id FROM sessions").fetchone()[0]
    conn.close()

    assert rows, "no corrected rows persisted"
    for side_int, cam_id, fid, quality in rows:
        side = ("left", "right")[side_int]
        assert (side, cam_id, fid) in monitor.seen, (side, cam_id, fid)
        assert quality == monitor.seen[(side, cam_id, fid)], (side, cam_id, fid)

    by_cam = {}
    for side_int, cam_id, fid, quality in rows:
        by_cam.setdefault((("left", "right")[side_int], cam_id), {})[fid] = quality
    # Each dim camera is flagged on exactly its dim frames; the others never.
    for key, frames in by_cam.items():
        poor = {fid for fid, q in frames.items() if POOR_CONTACT in parse(q)}
        assert poor == set(DIM.get(key, ())) & set(frames), key
    # The edges that share a batch with a dark close are stored correctly.
    right1 = by_cam[("right", 1)]
    assert right1[40] == POOR_CONTACT and right1[39] == OK
    assert right1[78] == POOR_CONTACT and right1[79] == OK and right1[80] == OK

    out = str(tmp_path / "export.csv")
    materialize_corrected_csv(db_path, sid, out, include_quality=True)
    with open(out, newline="", encoding="utf-8") as fh:
        exported = {int(r["frame_id"]): r for r in csv.DictReader(fh)}
    for fid, quality in right1.items():
        assert exported[fid]["quality_r2"] == quality, fid
    assert all(r["quality_l3"] == "" for r in exported.values())   # outside the mask
