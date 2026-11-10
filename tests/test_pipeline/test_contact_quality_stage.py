"""ContactQualityStage — the live verdict inside the pipeline (bloodflow-app#589).

The stage judges every batch's realtime values, publishes each edge on the
"contact_quality" channel, and adds the verdict latched at each frame to the
quality flags of the per-camera corrected frames that close in that batch.
"""

import numpy as np
import pytest

from omotion.contact_quality import (
    CONTACT_QUALITY_CHANNEL, CQSettings, CQThresholds, ContactQualityMonitor,
    ContactQualityTransition, ContactQualityTransitionSink,
)
from omotion.pipeline.batch import FrameBatch, IntervalClosed, LiveEmit
from omotion.pipeline.factory import default_pipeline
from omotion.pipeline.pedestal import SensorPedestals
from omotion.pipeline.sinks import ScanMetadata, Sink
from omotion.pipeline.stages.contact_quality import ContactQualityStage
from omotion.pipeline.stages.dark import EnrichedCorrectedFrame, EnrichedCorrectedInterval

SETTINGS = CQSettings(
    thresholds=CQThresholds.from_sequences(dark=[10.0] * 8, light=[20.0] * 8),
    rolling_window=1, light_activate_debounce=1, light_clear_debounce=1,
)


def _meta(left=0x01, right=0x00, reduced=False):
    return ScanMetadata(
        scan_id="s", subject_id="x", operator="o", started_at_iso="",
        duration_sec=0, left_camera_mask=left, right_camera_mask=right,
        reduced_mode=reduced,
    )


def _batch(side_idx, fids, values, *, cam_id=0, frame_types=None):
    """One camera on one side, a row per frame: realtime DN ``values``."""
    n = len(fids)
    arr = np.full((n, 2, 8), np.nan, dtype=np.float32)
    arr[:, side_idx, cam_id] = values
    return FrameBatch(
        cam_ids=np.full(n, cam_id, dtype=np.int8),
        frame_ids=np.array(fids, dtype=np.int64) & 0xFF,
        abs_frame_ids=np.array(fids, dtype=np.int64),
        side_ids=np.full(n, side_idx, dtype=np.int8),
        raw_histograms=None, temperature_c=None,
        timestamp_s=np.zeros(n), pdc=None, tcm=None, tcl=None,
        frame_type=np.array(frame_types or ["light"] * n, dtype="<U8"),
        subtracted_mean=arr, mean_dc_rt=arr.copy(),
    )


def _closed(frames):
    return IntervalClosed(corrected_batch=EnrichedCorrectedInterval(
        left_abs=frames[0].abs_frame_id - 1, right_abs=frames[-1].abs_frame_id + 1,
        frames=frames,
    ))


def _ef(fid, *, side="left", cam_id=0, quality="ok"):
    return EnrichedCorrectedFrame(
        abs_frame_id=fid, t=fid * 0.025, side=side, cam_id=cam_id,
        mean=60.0, std=5.0, contrast=0.3, bfi=5.0, bvi=5.0, quality=quality,
    )


def test_stage_publishes_each_edge_on_the_contact_quality_channel():
    stage = ContactQualityStage(SETTINGS, metadata=_meta())
    batch = stage.process(_batch(0, [10, 11, 12, 13], [60, 5, 5, 60]))
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert [e.channel for e in emits] == [CONTACT_QUALITY_CHANNEL] * 2
    assert [e.payload for e in emits] == [
        ContactQualityTransition("left", 0, "poor_contact", pytest.approx(5.0), True, 11),
        ContactQualityTransition("left", 0, "poor_contact", pytest.approx(60.0), False, 13),
    ]


def test_stage_stamps_each_frame_with_the_verdict_at_that_frame():
    """The interval closes in the same batch that holds its last frames: the
    stage judges the rows first, so even the frame right before the edge
    and the edge frame itself get the right verdict."""
    stage = ContactQualityStage(SETTINGS, metadata=_meta())
    batch = _batch(0, [10, 11, 12, 13], [60, 5, 5, 60])
    frames = [_ef(10), _ef(11, quality="ts_corrected"), _ef(12), _ef(13)]
    batch.events.append(_closed(frames))
    stage.process(batch)
    assert [f.quality for f in frames] == [
        "ok", "ts_corrected;poor_contact", "poor_contact", "ok",
    ]


def test_stage_leaves_side_average_frames_alone():
    """Reduced mode (Clinical): side-average rows carry no verdict."""
    stage = ContactQualityStage(SETTINGS, metadata=_meta(reduced=True))
    batch = _batch(0, [10, 11], [5, 5])
    side = _ef(11, cam_id=-1, quality="low_light")
    cam = _ef(11)
    batch.events.append(_closed([cam]))
    batch.events.append(_closed([side]))
    stage.process(batch)
    assert cam.quality == "poor_contact"
    assert side.quality == "low_light"


def test_stage_skips_cameras_outside_the_scan_mask():
    stage = ContactQualityStage(SETTINGS, metadata=_meta(left=0x01))
    batch = _batch(0, [10], [5], cam_id=1)
    frame = _ef(10, cam_id=1)
    batch.events.append(_closed([frame]))
    stage.process(batch)
    assert frame.quality == "ok"


def test_stage_stamps_the_terminal_flush():
    stage = ContactQualityStage(SETTINGS, metadata=_meta())
    stage.process(_batch(0, [10, 11], [60, 5]))
    flush = _batch(0, [], [])
    frames = [_ef(10), _ef(11)]
    flush.events.append(_closed(frames))
    stage.on_scan_stop(flush)
    assert [f.quality for f in frames] == ["ok", "poor_contact"]


def test_reset_starts_a_fresh_verdict_history():
    stage = ContactQualityStage(SETTINGS, metadata=_meta())
    stage.process(_batch(0, [10], [5]))
    stage.reset()
    batch = _batch(0, [10], [60])
    frame = _ef(10)
    batch.events.append(_closed([frame]))
    stage.process(batch)
    assert frame.quality == "ok"


def test_the_stage_and_the_transition_sink_share_one_channel():
    """A typo in either channel name would leave the live warning silently
    dead, the failure #364 was about. The monitor itself is no sink: a
    full scan runs it inside the stage, so it must not be attached too."""
    sink = ContactQualityTransitionSink(on_transition=lambda *a: None)
    assert isinstance(sink, Sink)
    assert sink.channels == {CONTACT_QUALITY_CHANNEL}
    assert not hasattr(ContactQualityMonitor, "channels")


def test_transition_sink_calls_back_with_the_edge():
    seen = []
    sink = ContactQualityTransitionSink(on_transition=lambda *a: seen.append(a))
    sink.consume(CONTACT_QUALITY_CHANNEL,
                 ContactQualityTransition("right", 3, "ambient_light", 42.0, True, 600))
    sink.consume("live", object())   # other channels ignored
    assert seen == [("right", 3, "ambient_light", 42.0, True)]


def test_transition_sink_swallows_a_raising_callback(caplog):
    def boom(*a):
        raise RuntimeError("consumer bug")
    sink = ContactQualityTransitionSink(on_transition=boom)
    sink.consume(CONTACT_QUALITY_CHANNEL,
                 ContactQualityTransition("left", 0, "poor_contact", 1.0, True, 5))
    assert "transition callback raised" in caplog.text


@pytest.mark.parametrize("settings", [None, SETTINGS])
def test_default_pipeline_runs_the_stage_just_before_the_live_tee(settings):
    class _Cal:
        c_min = np.zeros((2, 8)); c_max = np.ones((2, 8))
        i_min = np.zeros((2, 8)); i_max = np.full((2, 8), 500.0)
    pipeline = default_pipeline(
        metadata=_meta(), calibration=_Cal(),
        pedestals=SensorPedestals(left=128.0, right=128.0),
        contact_quality=settings,
    )
    names = [s.name for s in pipeline.stages]
    if settings is None:
        assert "contact_quality" not in names
    else:
        assert names[-3:] == ["side_average", "contact_quality", "tee:live"]
