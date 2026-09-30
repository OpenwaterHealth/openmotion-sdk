"""Camera-dropout timeout (#298): a mask camera that never delivers, or stops
delivering mid-scan while its side keeps streaming, raises one
CameraDropoutTimeout once it has been silent for ``camera_dropout_abort_s``
of device time. CameraDropoutWatchdogSink turns that event into a scan abort.

Silence is measured on packet timestamps, not by counting frames, so the
limit means the same thing at any trigger rate.
"""

import numpy as np

from omotion.pipeline.batch import CameraDropoutTimeout, CameraStreamGap, FrameBatch
from omotion.pipeline.sinks import CameraDropoutWatchdogSink
from omotion.pipeline.stages.classify import FrameClassificationStage


def _packet_batch(rows):
    """Build rows of (packet_id, timestamp_s, side, cam_id, frame_id)."""
    n = len(rows)
    return FrameBatch(
        cam_ids=np.array([r[3] for r in rows], dtype=np.int8),
        frame_ids=np.array([r[4] % 256 for r in rows], dtype=np.uint8),
        side_ids=np.array([r[2] for r in rows], dtype=np.int8),
        packet_ids=np.array([r[0] for r in rows], dtype=np.int64),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.array([r[1] for r in rows], dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


def _captures(first, last, cams, *, side=0, hz=40.0):
    """One packet per capture ``fid`` in [first, last], carrying ``cams``."""
    rows = []
    for fid in range(first, last + 1):
        t = (fid - 1) / hz
        for cam in cams:
            rows.append((fid, t, side, cam, fid))
    return _packet_batch(rows)


def _timeouts(batch):
    return [e for e in batch.events if isinstance(e, CameraDropoutTimeout)]


def _run(stage, batches):
    events = []
    for b in batches:
        stage.process(b)
        events.extend(_timeouts(b))
    return events


def test_never_delivering_camera_times_out_once():
    stage = FrameClassificationStage(expected_camera_masks=(0x03, 0),
                                     camera_dropout_abort_s=5.0)
    # Cam 1 never shows up. 5 s at 40 Hz = capture 201 (t = 5.0 s).
    before = _run(stage, [_captures(1, 200, [0])])
    assert before == [], "timed out before 5 s of silence"

    after = _run(stage, [_captures(201, 400, [0])])
    assert len(after) == 1, "must time out exactly once"
    ev = after[0]
    assert (ev.side, ev.cam_id) == (0, 1)
    assert ev.never_delivered is True
    assert ev.silent_s >= 5.0
    assert ev.threshold_s == 5.0
    assert ev.first_missing_packet_id == 1


def test_camera_dying_mid_scan_times_out():
    stage = FrameClassificationStage(expected_camera_masks=(0x03, 0),
                                     camera_dropout_abort_s=5.0)
    # Both cams for 10 s, then cam 1 dies.
    events = _run(stage, [_captures(1, 400, [0, 1]),
                          _captures(401, 600, [0]),     # 5 s missing, t=15.0 at 601
                          _captures(601, 700, [0])])
    assert len(events) == 1
    ev = events[0]
    assert (ev.side, ev.cam_id) == (0, 1)
    assert ev.never_delivered is False
    assert ev.first_missing_packet_id == 401


def test_brief_gap_below_limit_does_not_time_out():
    stage = FrameClassificationStage(expected_camera_masks=(0x03, 0),
                                     camera_dropout_abort_s=5.0)
    # Cam 1 misses 4 s (well past the 9-packet CameraStreamGap alert) then
    # resumes; then misses another 4 s. Each gap is timed from its own start.
    batches = [_captures(1, 100, [0, 1]),
               _captures(101, 260, [0]),
               _captures(261, 300, [0, 1]),
               _captures(301, 460, [0]),
               _captures(461, 500, [0, 1])]
    gap_events = []
    for b in batches:
        stage.process(b)
        gap_events.extend(e for e in b.events if isinstance(e, CameraStreamGap))
    assert not any(_timeouts(b) for b in batches)
    assert [e.state for e in gap_events] == ["missing", "resumed",
                                             "missing", "resumed"]


def test_timeout_is_measured_in_time_not_frames():
    """At 10 Hz, 5 s of silence is 50 captures, not 200."""
    stage = FrameClassificationStage(expected_camera_masks=(0x03, 0),
                                     camera_dropout_abort_s=5.0)
    assert _run(stage, [_captures(1, 50, [0], hz=10.0)]) == []   # t = 4.9 s
    assert len(_run(stage, [_captures(51, 52, [0], hz=10.0)])) == 1


def test_other_side_and_unmasked_cameras_are_ignored():
    stage = FrameClassificationStage(expected_camera_masks=(0x01, 0x01),
                                     camera_dropout_abort_s=5.0)
    # Left cam 0 and right cam 0 both stream; unmasked cams never appear.
    left = _captures(1, 400, [0], side=0)
    right = _captures(1, 400, [0], side=1)
    assert _run(stage, [left, right]) == []


def test_disabled_by_default():
    stage = FrameClassificationStage(expected_camera_masks=(0x03, 0))
    assert _run(stage, [_captures(1, 2000, [0])]) == []


def test_reset_clears_timeout_latch():
    stage = FrameClassificationStage(expected_camera_masks=(0x03, 0),
                                     camera_dropout_abort_s=5.0)
    assert len(_run(stage, [_captures(1, 400, [0])])) == 1
    stage.reset()
    assert len(_run(stage, [_captures(1, 400, [0])])) == 1


# ── Watchdog sink ────────────────────────────────────────────────────────

def _timeout_event(side=0, cam=1):
    return CameraDropoutTimeout(
        side=side, cam_id=cam, never_delivered=True, silent_s=5.0,
        threshold_s=5.0, packet_id=201, timestamp_s=5.0,
        first_missing_packet_id=1, first_missing_timestamp_s=0.0,
    )


def test_watchdog_sink_calls_back_once_on_first_timeout():
    calls = []
    sink = CameraDropoutWatchdogSink(calls.append)
    assert sink.channels == {"diagnostics"}
    sink.on_scan_start(None)
    first = _timeout_event(cam=1)
    sink.consume("diagnostics", first)
    sink.consume("diagnostics", _timeout_event(cam=2))
    assert calls == [first]


def test_watchdog_sink_ignores_other_events_and_rearms_per_scan():
    calls = []
    sink = CameraDropoutWatchdogSink(calls.append)
    sink.on_scan_start(None)
    sink.consume("diagnostics", CameraStreamGap(
        side=0, cam_id=1, state="missing", missing_frames=9, packet_id=10,
        timestamp_s=0.225, first_missing_packet_id=2,
        first_missing_timestamp_s=0.025,
    ))
    assert calls == []
    sink.consume("diagnostics", _timeout_event())
    sink.on_scan_start(None)
    sink.consume("diagnostics", _timeout_event())
    assert len(calls) == 2


def test_watchdog_sink_swallows_callback_errors():
    def _boom(_event):
        raise RuntimeError("callback failed")
    sink = CameraDropoutWatchdogSink(_boom)
    sink.on_scan_start(None)
    sink.consume("diagnostics", _timeout_event())   # must not raise


def test_default_pipeline_passes_threshold_to_classifier():
    from omotion.pipeline.factory import default_pipeline
    from omotion.pipeline.pedestal import SensorPedestals
    from omotion.pipeline.sinks import ScanMetadata
    from omotion.Calibration import Calibration

    meta = ScanMetadata(
        scan_id="s", subject_id="x", operator="t", started_at_iso="",
        duration_sec=1, left_camera_mask=0x03, right_camera_mask=0,
        reduced_mode=True,
    )
    kw = dict(metadata=meta, calibration=Calibration.default(),
              pedestals=SensorPedestals(left=64.0, right=64.0))
    classify = default_pipeline(camera_dropout_abort_s=3.5, **kw).stages[0]
    assert classify.camera_dropout_abort_s == 3.5
    assert default_pipeline(**kw).stages[0].camera_dropout_abort_s is None
