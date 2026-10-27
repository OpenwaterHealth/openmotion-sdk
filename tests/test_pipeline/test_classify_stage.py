"""FrameClassificationStage — abs_frame_id unwrap + frame_type labeling."""

import logging

import numpy as np
import pytest
from omotion.pipeline.batch import (
    CameraLateStart, CameraStreamGap, FrameBatch, FrameQuarantined,
)
from omotion.pipeline.stages.classify import FrameClassificationStage


def _batch_with_raw_ids(raw_ids_per_side_cam):
    """raw_ids_per_side_cam: dict {(side_idx, cam_id): list_of_raw_frame_ids}."""
    rows = []
    for (side_idx, cam_id), raw_ids in raw_ids_per_side_cam.items():
        for raw_id in raw_ids:
            rows.append((side_idx, cam_id, raw_id))

    n = len(rows)
    cam_ids = np.array([r[1] for r in rows], dtype=np.int8)
    frame_ids = np.array([r[2] for r in rows], dtype=np.uint8)
    side_ids = np.array([r[0] for r in rows], dtype=np.int8)
    raw_hists = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    for i, (s, c, _) in enumerate(rows):
        raw_hists[i, s, c, 0] = 1
    return FrameBatch(
        cam_ids=cam_ids,
        frame_ids=frame_ids,
        side_ids=side_ids,
        raw_histograms=raw_hists,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64) * 0.025,
        pdc=None, tcm=None, tcl=None,
    )


def _packet_batch(rows):
    """Build rows of (packet_id, timestamp_s, side, cam_id, frame_id)."""
    n = len(rows)
    return FrameBatch(
        cam_ids=np.array([r[3] for r in rows], dtype=np.int8),
        frame_ids=np.array([r[4] for r in rows], dtype=np.uint8),
        side_ids=np.array([r[2] for r in rows], dtype=np.int8),
        packet_ids=np.array([r[0] for r in rows], dtype=np.int64),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.array([r[1] for r in rows], dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


def test_first_frame_with_raw_id_1_is_warmup_not_stale():
    batch = _batch_with_raw_ids({(0, 0): [1, 2, 3]})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    np.testing.assert_array_equal(batch.abs_frame_ids, [1, 2, 3])
    np.testing.assert_array_equal(batch.frame_type, ["warmup", "warmup", "warmup"])


def test_first_frame_with_raw_id_other_than_1_is_stale():
    batch = _batch_with_raw_ids({(0, 0): [42, 43, 44]})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    assert list(batch.frame_type) == ["stale", "stale", "stale"]


def test_leading_garbage_does_not_seed_unwrapper_state():
    batch = _batch_with_raw_ids({(0, 0): [173, 1, 2, 3]})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)

    assert list(batch.frame_type) == ["stale", "warmup", "warmup", "warmup"]
    np.testing.assert_array_equal(batch.abs_frame_ids, [173, 1, 2, 3])


def test_warmup_range_marks_first_9_as_warmup():
    raw_ids = list(range(1, 15))
    batch = _batch_with_raw_ids({(0, 0): raw_ids})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    expected = ["warmup"] * 9 + ["dark"] + ["light"] * 4
    np.testing.assert_array_equal(batch.frame_type, expected)


def test_dark_schedule_fires_at_intervals():
    raw_ids = list(range(1, 25))
    batch = _batch_with_raw_ids({(0, 0): raw_ids})
    FrameClassificationStage(discard_count=9, dark_interval=10).process(batch)

    expected_darks_at = {10, 11, 21}
    for i, raw_id in enumerate(raw_ids):
        is_dark = (i + 1) in expected_darks_at
        assert (batch.frame_type[i] == "dark") == is_dark, \
            f"abs={i+1}: got {batch.frame_type[i]}, expected dark={is_dark}"


def test_unwrap_handles_8bit_rollover():
    raw_ids = list(range(1, 256)) + list(range(0, 5))
    batch = _batch_with_raw_ids({(0, 0): raw_ids})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    expected_abs = list(range(1, 261))
    np.testing.assert_array_equal(batch.abs_frame_ids, expected_abs)


def test_stale_leftover_frames_at_start_are_rejected_not_offsetting_epoch():
    """Reproduces the Varun-unit left-sensor capture (issue #172): an unflushed
    histogram buffer emits stale frames (raw 255, 173) after the real first
    frame, then the real sequence resumes (4, 5, 6 …). The old unwrapper read
    255 as a forward jump and 4 as a rollover, injecting a permanent +256
    epoch offset that shifted the entire dark/warmup schedule. The stale frames
    must be rejected without advancing the counter."""
    raw_ids = [1, 255, 173, 4, 5, 6, 7, 8, 9, 10, 11]
    batch = _batch_with_raw_ids({(0, 0): raw_ids})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    # 255 and 173 rejected as stale; real frames keep their true abs_id (no +256).
    np.testing.assert_array_equal(
        batch.frame_type,
        ["warmup", "stale", "stale", "warmup", "warmup", "warmup",
         "warmup", "warmup", "warmup", "dark", "light"],
    )
    np.testing.assert_array_equal(
        batch.abs_frame_ids, [1, 255, 173, 4, 5, 6, 7, 8, 9, 10, 11]
    )


def test_midscan_counter_blip_is_rejected_and_sequence_resumes():
    """A mid-scan counter glitch (…3, 4, [2, 3], 5 …) must not reset the epoch:
    the spurious backward 2,3 are rejected and 5 continues the run."""
    raw_ids = [1, 2, 3, 4, 2, 3, 5, 6]
    batch = _batch_with_raw_ids({(0, 0): raw_ids})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    # backward 2,3 rejected; the real run 1..6 keeps monotonic abs ids.
    assert list(batch.frame_type) == [
        "warmup", "warmup", "warmup", "warmup",
        "stale", "stale", "warmup", "warmup"]
    np.testing.assert_array_equal(
        batch.abs_frame_ids, [1, 2, 3, 4, 2, 3, 5, 6]
    )


def test_zero_filled_row_keeps_source_assigned_side():
    """A row whose raw_histogram is all zeros (e.g. firmware-dropped frame)
    must still be routed to its source-assigned side.

    Before the side_ids fix, classify.py inferred side via
    ``np.argmax(raw_histograms[i].sum(axis=(-2, -1)))`` which silently
    defaults to 0 whenever the histogram is all zeros, so right-side dropped
    frames were misclassified as left.
    """
    # Two right-side rows, both with all-zero histograms — would have
    # defaulted to side=0 under the old logic.
    raw_ids = [1, 2]
    n = len(raw_ids)
    batch = FrameBatch(
        cam_ids=np.array([0, 0], dtype=np.int8),
        frame_ids=np.array(raw_ids, dtype=np.uint8),
        side_ids=np.array([1, 1], dtype=np.int8),  # right
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )

    stage = FrameClassificationStage(discard_count=9, dark_interval=600)
    stage.process(batch)

    # The unwrapper key is (side_idx, cam_id) — verify state went to side=1.
    assert (1, 0) in stage._unwrappers
    assert (0, 0) not in stage._unwrappers


def test_dropped_stale_frames_are_logged_and_summarised(caplog):
    """Stale/non-monotonic frames the unwrapper drops must be visible in the
    log — a hardware-health signal. First occurrence logs live per camera;
    on_scan_stop emits a per-scan total."""
    batch = _batch_with_raw_ids({(0, 0): [1, 255, 173, 4, 5, 6]})
    stage = FrameClassificationStage(discard_count=9, dark_interval=600)
    with caplog.at_level(logging.WARNING):
        stage.process(batch)
        assert any("quarantining frame" in r.message for r in caplog.records), \
            "expected a live WARNING when a frame is quarantined"
        caplog.clear()
        stage.on_scan_stop(batch)
    summary = [r.message for r in caplog.records if "Scan summary" in r.message]
    assert summary, "expected a quarantine scan summary at on_scan_stop"
    assert "2" in summary[0]  # 255 and 173 were dropped


def test_clean_scan_logs_no_stale_warnings(caplog):
    batch = _batch_with_raw_ids({(0, 0): list(range(1, 15))})
    stage = FrameClassificationStage(discard_count=9, dark_interval=600)
    with caplog.at_level(logging.WARNING):
        stage.process(batch)
        stage.on_scan_stop(batch)
    assert not any("stale" in r.message.lower() for r in caplog.records)


def test_reset_clears_unwrapper_state():
    stage = FrameClassificationStage(discard_count=9, dark_interval=600)
    batch1 = _batch_with_raw_ids({(0, 0): [1, 2, 3]})
    stage.process(batch1)
    assert batch1.abs_frame_ids[0] == 1

    stage.reset()
    batch2 = _batch_with_raw_ids({(0, 0): [1, 2, 3]})
    stage.process(batch2)
    assert batch2.abs_frame_ids[0] == 1
    assert batch2.frame_type[0] == "warmup"


def test_camera_gap_alerts_after_eight_then_reports_recovery(caplog):
    stage = FrameClassificationStage(expected_camera_masks=(0x03, 0))
    first = _packet_batch([
        (1, 0.000, 0, 0, 1),
        (1, 0.000, 0, 1, 1),
    ])
    stage.process(first)

    # Camera 1 is absent from exactly eight captures: no alert yet.
    eight_missing = _packet_batch([
        (fid, (fid - 1) * 0.025, 0, 0, fid)
        for fid in range(2, 10)
    ])
    with caplog.at_level(logging.WARNING):
        stage.process(eight_missing)
    assert not any(isinstance(e, CameraStreamGap)
                   for e in eight_missing.events)
    assert "CAMERA STREAM ALERT" not in caplog.text

    # The ninth missing capture crosses the threshold, exactly once.
    ninth_missing = _packet_batch([(10, 0.225, 0, 0, 10)])
    with caplog.at_level(logging.WARNING):
        stage.process(ninth_missing)
    alerts = [e for e in ninth_missing.events
              if isinstance(e, CameraStreamGap)]
    assert len(alerts) == 1
    assert alerts[0].state == "missing"
    assert alerts[0].missing_frames == 9
    assert alerts[0].first_missing_packet_id == 2
    assert alerts[0].packet_id == 10
    assert "side=left(0) cam_id=1" in caplog.text

    still_missing = _packet_batch([(11, 0.250, 0, 0, 11)])
    stage.process(still_missing)
    assert not any(isinstance(e, CameraStreamGap)
                   for e in still_missing.events)

    resumed = _packet_batch([
        (12, 0.275, 0, 0, 12),
        (12, 0.275, 0, 1, 12),
    ])
    with caplog.at_level(logging.WARNING):
        stage.process(resumed)
    recoveries = [e for e in resumed.events
                  if isinstance(e, CameraStreamGap)]
    assert len(recoveries) == 1
    assert recoveries[0].state == "resumed"
    assert recoveries[0].missing_frames == 10
    assert recoveries[0].packet_id == 12
    assert "camera stream resumed" in caplog.text

    # The first return is isolated; the next coherent packet safely rejoins.
    assert resumed.frame_type[1] == "stale"
    confirmed = _packet_batch([
        (13, 0.300, 0, 0, 13),
        (13, 0.300, 0, 1, 13),
    ])
    stage.process(confirmed)
    assert confirmed.frame_type[1] == "light"
    assert confirmed.abs_frame_ids[1] == 13


def test_large_gap_re_anchors_once_the_next_frame_confirms_it():
    # A +10 jump the device clock agrees with, then a coherent +1: a real gap
    # (#286). Only the first resumed frame is held back; nothing needs
    # separate evidence that the camera went missing.
    stage = FrameClassificationStage()
    batch = _batch_with_raw_ids({(0, 0): [1, 11, 12]})
    batch.timestamp_s[:] = [0.000, 0.250, 0.275]
    stage.process(batch)

    assert list(batch.frame_type) == ["warmup", "stale", "light"]
    np.testing.assert_array_equal(batch.abs_frame_ids, [1, 11, 12])
    quarantined = [e for e in batch.events
                   if isinstance(e, FrameQuarantined)]
    assert len(quarantined) == 1
    assert quarantined[0].reason == "gap_too_large"


def test_large_jump_the_clock_does_not_support_stays_quarantined():
    # frame_id +40 in 25 ms is physically impossible: no candidate is held,
    # and the next genuine frame continues the old sequence.
    stage = FrameClassificationStage()
    batch = _batch_with_raw_ids({(0, 0): [1, 41, 42, 2]})
    batch.timestamp_s[:] = [0.000, 0.025, 0.050, 0.025]
    stage.process(batch)

    assert list(batch.frame_type) == ["warmup", "stale", "stale", "warmup"]
    assert batch.abs_frame_ids[3] == 2


def _capture_batch(frames, *, side=0, cams=range(8), first_packet=0):
    """Packet-shaped rows: every camera of `side` delivers each frame in
    `frames`, timestamped on the 25 ms cadence of its true frame number."""
    rows = [(first_packet + p, (f - 1) * 0.025, side, cam, f & 0xFF)
            for p, f in enumerate(frames) for cam in cams]
    return _packet_batch(rows)


def _run_module_gap(missing):
    """1..1836, then a whole-module gap of `missing` frames, then 700 more
    (far enough to reach the next scheduled dark)."""
    stage = FrameClassificationStage(expected_camera_masks=(0xFF, 0))
    truth = list(range(1, 1837)) + list(range(1837 + missing, 1837 + missing + 700))
    rows = []
    for i in range(0, len(truth), 16):
        chunk = truth[i:i + 16]
        batch = _capture_batch(chunk, first_packet=i)
        stage.process(batch)
        rows += list(zip(np.repeat(chunk, 8), batch.abs_frame_ids, batch.frame_type))
    return rows


def test_module_wide_gap_loses_only_the_first_resumed_capture():
    # sdk#286: all 8 cameras skip 17 frames together. The old classifier
    # quarantined ~240 captures and left every later id 256 low.
    rows = _run_module_gap(17)
    stale = [(t, a) for t, a, ft in rows if ft == "stale"]
    assert [t for t, _ in stale] == [1854] * 8
    assert all(a == t for t, a, ft in rows if ft != "stale")
    darks = sorted({t for t, a, ft in rows if ft == "dark" and t > 1837})
    assert darks == [2401]


@pytest.mark.parametrize("missing", [200, 256 + 17, 256])
def test_long_module_gap_takes_its_epoch_from_the_clock(missing):
    # Gaps past 127 frames are ambiguous on the 8-bit wire id: 256+17 reads
    # as +17, and a gap of 256 missing frames reads as a single +1 step. The
    # device clock fixes the epoch, so ids stay right after the gap.
    rows = _run_module_gap(missing)
    resumed = 1837 + missing
    assert [t for t, _, ft in rows if ft == "stale"] == [resumed] * 8
    assert all(a == t for t, a, ft in rows if ft != "stale")
    # The next scheduled dark after the gap lands on the true dark frame.
    next_dark = next(k for k in range(resumed + 1, resumed + 700) if (k - 1) % 600 == 0)
    assert min(t for t, a, ft in rows if ft == "dark" and t > resumed) == next_dark


def test_large_corrupt_pair_does_not_poison_clean_counter_state():
    stage = FrameClassificationStage()
    batch = _batch_with_raw_ids({(0, 0): [1, 65, 2]})
    batch.timestamp_s[:] = [0.000, 1.600, 0.025]
    stage.process(batch)

    assert list(batch.frame_type) == ["warmup", "stale", "warmup"]
    np.testing.assert_array_equal(batch.abs_frame_ids, [1, 65, 2])


# ---------------------------------------------------------------------------
# Rule 5: a camera whose frame 1 never arrived anchors from the module clock
# (#319). On 2026-10-12 the parser dropped frame 1 of left cam 8, right cam 8
# and right cam 2 (bad histogram sums); each camera's frame 2 was then
# quarantined as a leading frame and the camera was lost for the whole check.
# ---------------------------------------------------------------------------

def _scan_rows(frames, *, drop=(), sides=(0, 1), cams=range(8), t0=0.0):
    """Packet rows for every (side, cam) over `frames`, minus the
    (side, cam, frame) captures in `drop` (never delivered)."""
    rows = []
    for p, f in enumerate(frames):
        for side in sides:
            for cam in cams:
                if (side, cam, f) in drop:
                    continue
                rows.append((p * 2 + side, t0 + (f - 1) * 0.025, side, cam, f & 0xFF))
    return rows


def test_camera_whose_frame_1_was_dropped_anchors_from_the_module_clock(caplog):
    drop = {(0, 7, 1), (1, 7, 1), (1, 1, 1)}
    batch = _packet_batch(_scan_rows(range(1, 45), drop=drop))
    stage = FrameClassificationStage(discard_count=9, dark_interval=600)
    with caplog.at_level(logging.WARNING):
        stage.process(batch)

    stale = sorted(
        (int(batch.side_ids[i]), int(batch.cam_ids[i]), int(batch.frame_ids[i]))
        for i in range(len(batch.frame_ids)) if batch.frame_type[i] == "stale"
    )
    # Each affected camera loses only its held frame 2 (a warmup frame).
    assert stale == [(0, 7, 2), (1, 1, 2), (1, 7, 2)]
    accepted = batch.frame_type != "stale"
    np.testing.assert_array_equal(batch.abs_frame_ids[accepted],
                                  batch.frame_ids[accepted])
    for side, cam in [(0, 7), (1, 1), (1, 7)]:
        rows = [i for i in range(len(batch.frame_ids))
                if batch.side_ids[i] == side and batch.cam_ids[i] == cam]
        types = {int(batch.frame_ids[i]): batch.frame_type[i] for i in rows}
        assert types[3] == "warmup" and types[10] == "dark" and types[11] == "light"

    late = [e for e in batch.events if isinstance(e, CameraLateStart)]
    assert sorted((e.side, e.cam_id, e.abs_frame_id, e.missing_frames)
                  for e in late) == [(0, 7, 3, 2), (1, 1, 3, 2), (1, 7, 3, 2)]
    assert "camera anchored late: side=right(1) cam_id=1" in caplog.text


def test_late_start_needs_an_anchored_camera_on_the_same_module():
    # The right module's clock is independent of the left one, so a left
    # camera with no anchored left module-mate stays quarantined (today's
    # fail-closed behaviour), even though the right module anchored.
    rows = _scan_rows(range(1, 13), drop={(0, 0, 1)}, cams=[0])
    batch = _packet_batch(rows)
    FrameClassificationStage().process(batch)

    left = [batch.frame_type[i] for i in range(len(rows)) if batch.side_ids[i] == 0]
    right = [batch.frame_type[i] for i in range(len(rows)) if batch.side_ids[i] == 1]
    assert left == ["stale"] * 11
    assert "stale" not in right
    assert not any(isinstance(e, CameraLateStart) for e in batch.events)


def test_stale_leftovers_are_not_anchored_by_the_module_clock():
    # Leftovers from the previous scan can carry small frame ids, but their
    # device timestamps predate this scan's frame 1, so the module clock never
    # places them. The camera's real frame 1 still anchors normally.
    rows = [
        (0, 100.000, 0, 0, 1),
        (1, 98.500, 0, 1, 2),     # leftover, small id, old timestamp
        (1, 98.525, 0, 1, 3),     # leftover
        (2, 100.000, 0, 1, 1),
        (3, 100.025, 0, 0, 2),
        (3, 100.025, 0, 1, 2),
    ]
    batch = _packet_batch(rows)
    FrameClassificationStage().process(batch)

    assert list(batch.frame_type) == [
        "warmup", "stale", "stale", "warmup", "warmup", "warmup"]
    np.testing.assert_array_equal(batch.abs_frame_ids[[0, 3, 4, 5]], [1, 1, 2, 2])
    assert not any(isinstance(e, CameraLateStart) for e in batch.events)


def test_late_candidate_is_adopted_only_when_the_next_frame_confirms_it():
    # Frame 2 is held; a corrupt id then breaks the chain, so nothing is
    # adopted. Frame 4 is held afresh and frame 5 confirms it.
    rows = [
        (0, 0.000, 0, 0, 1),
        (1, 0.025, 0, 0, 2), (1, 0.025, 0, 1, 2),
        (2, 0.050, 0, 0, 3), (2, 0.050, 0, 1, 77),
        (3, 0.075, 0, 0, 4), (3, 0.075, 0, 1, 4),
        (4, 0.100, 0, 0, 5), (4, 0.100, 0, 1, 5),
    ]
    batch = _packet_batch(rows)
    FrameClassificationStage().process(batch)

    cam1 = [i for i in range(len(rows)) if batch.cam_ids[i] == 1]
    assert [batch.frame_type[i] for i in cam1] == [
        "stale", "stale", "stale", "warmup"]
    assert batch.abs_frame_ids[cam1[-1]] == 5
    late = [e for e in batch.events if isinstance(e, CameraLateStart)]
    assert [(e.cam_id, e.abs_frame_id, e.missing_frames) for e in late] == [(1, 5, 4)]


def test_reset_forgets_the_module_clock():
    stage = FrameClassificationStage()
    stage.process(_packet_batch(_scan_rows(range(1, 5), sides=[0], cams=[0])))
    stage.reset()
    # Next scan: only cam 1 delivers, starting at frame 2. Last scan's clock
    # must not anchor it.
    batch = _packet_batch(_scan_rows(range(2, 6), sides=[0], cams=[1]))
    stage.process(batch)
    assert list(batch.frame_type) == ["stale"] * 4


def test_late_start_event_carries_detail_to_the_diagnostics_log():
    from omotion.pipeline.sinks import _diagnostic_detail
    event = CameraLateStart(side=1, cam_id=7, abs_frame_id=3, missing_frames=2,
                            packet_id=4, timestamp_s=0.05)
    assert _diagnostic_detail(event)["missing_frames"] == 2
