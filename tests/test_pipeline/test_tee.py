"""Tests for Tee — emits a LiveEmit event for the named channel."""

import numpy as np
from omotion.pipeline.batch import FrameBatch, LiveEmit
from omotion.pipeline.tee import Tee


def _batch_with_frame_types(types):
    n = len(types)
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        frame_type=np.array(types, dtype="<U8"),
    )


def test_tee_with_no_filter_emits_one_event_per_call():
    tee = Tee("raw", filter=None)
    batch = _batch_with_frame_types(["light"])
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert len(emits) == 1
    assert emits[0].channel == "raw"
    assert emits[0].payload is batch


def test_tee_with_filter_skips_emit_when_filter_excludes_all_frames():
    tee = Tee("live", filter=lambda ft: ft != "warmup" and ft != "stale")
    batch = _batch_with_frame_types(["warmup", "stale", "warmup"])
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert emits == []


def test_tee_with_filter_emits_when_any_frame_passes():
    tee = Tee("live", filter=lambda ft: ft != "warmup" and ft != "stale")
    batch = _batch_with_frame_types(["warmup", "light", "warmup"])
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert len(emits) == 1
    assert emits[0].channel == "live"


def test_tee_reset_is_a_noop():
    tee = Tee("raw", filter=None)
    tee.reset()


def test_tee_with_max_duration_s_emits_within_budget():
    tee = Tee("raw", filter=None, max_duration_s=60.0)
    batch = _batch_with_frame_types(["light"])
    batch.timestamp_s = np.array([0.0], dtype=np.float64)
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert len(emits) == 1


def test_tee_with_max_duration_s_skips_after_budget():
    tee = Tee("raw", filter=None, max_duration_s=60.0)
    batch = _batch_with_frame_types(["light"])
    batch.timestamp_s = np.array([65.0], dtype=np.float64)
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert emits == []


def test_tee_with_max_duration_s_none_means_unbounded():
    tee = Tee("raw", filter=None, max_duration_s=None)
    batch = _batch_with_frame_types(["light"])
    batch.timestamp_s = np.array([9999.0], dtype=np.float64)
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert len(emits) == 1


# ── snapshot semantics (raw-CSV faithfulness, SDK_BUGREPORT.md Defect 1) ──


def test_tee_default_emits_payload_by_reference():
    """Default Tee (snapshot=False) emits the live batch by reference.

    Back-compat: the "live" tee runs last in the pipeline, so reference
    semantics are correct and cheaper there.
    """
    tee = Tee("live", filter=None)
    batch = _batch_with_frame_types(["light"])
    tee.process(batch)
    emit = [e for e in batch.events if isinstance(e, LiveEmit)][0]
    assert emit.payload is batch


def test_tee_snapshot_decouples_payload_from_later_mutation():
    """Tee(snapshot=True) freezes the batch's arrays at tee time.

    Downstream stages mutate FrameBatch arrays in place (NoiseFloorStage
    zeroes raw_histograms; TimestampRepairStage rewrites timestamp_s). The
    raw tee runs *before* those stages but its event is dispatched *after*
    the whole pipeline finishes — so a by-reference payload would serialize
    post-mutation data. A snapshot payload must be immune.
    """
    tee = Tee("raw", filter=None, snapshot=True)
    batch = _batch_with_frame_types(["light"])
    batch.raw_histograms[:] = 100
    batch.timestamp_s = np.array([1.234], dtype=np.float64)

    tee.process(batch)
    emit = [e for e in batch.events if isinstance(e, LiveEmit)][0]

    # Simulate later in-place mutation by NoiseFloor + TimestampRepair.
    batch.raw_histograms[:] = 0
    batch.timestamp_s[0] = 9.999

    assert emit.payload is not batch
    assert np.all(emit.payload.raw_histograms == 100)
    assert emit.payload.timestamp_s[0] == 1.234


def test_tee_snapshot_survives_noise_floor_in_pipeline():
    """Integration: Tee('raw', snapshot=True) → NoiseFloorStage.

    Directly reproduces SDK_BUGREPORT.md Defect 1b — the raw payload's
    histogram sum must NOT drop when NoiseFloorStage zeroes low bins.
    """
    from omotion.pipeline.pipeline import Pipeline
    from omotion.pipeline.stages.noise_floor import NoiseFloorStage

    tee = Tee("raw", filter=None, snapshot=True)
    pipe = Pipeline([tee, NoiseFloorStage(threshold=10)])

    batch = _batch_with_frame_types(["light"])
    batch.raw_histograms[0, 0, 0, 0] = 5     # below threshold — NoiseFloor zeroes
    batch.raw_histograms[0, 0, 0, 1] = 100   # above threshold — survives
    orig_sum = int(batch.raw_histograms.sum())

    result = pipe.process(batch)
    emit = [e for e in result.events
            if isinstance(e, LiveEmit) and e.channel == "raw"][0]

    # NoiseFloor zeroed the sub-threshold bin in the live batch...
    assert result.raw_histograms[0, 0, 0, 0] == 0
    # ...but the raw snapshot kept the faithful capture.
    assert emit.payload.raw_histograms[0, 0, 0, 0] == 5
    assert int(emit.payload.raw_histograms.sum()) == orig_sum
