"""DarkIntegrityGuard — flag dark frames whose u1 looks too bright."""

from omotion.pipeline.batch import DarkIntegrityWarning
from omotion.pipeline.stages.dark import DarkIntegrityGuard


def test_passes_silently_when_u1_within_range():
    guard = DarkIntegrityGuard(max_above_pedestal=30.0)
    events = []
    ok = guard.check(side="left", cam_id=0, abs_frame_id=10, u1=80.0,
                     pedestal=64.0, events=events)
    assert ok is True
    assert events == []


def test_appends_warning_when_u1_exceeds_pedestal_plus_threshold():
    guard = DarkIntegrityGuard(max_above_pedestal=30.0)
    events = []
    ok = guard.check(side="left", cam_id=3, abs_frame_id=10, u1=200.0,
                     pedestal=64.0, events=events)
    assert ok is False
    assert len(events) == 1
    w = events[0]
    assert isinstance(w, DarkIntegrityWarning)
    assert w.side == "left"
    assert w.cam_id == 3
    assert w.u1 == 200.0
    # threshold must be pedestal + max_above_pedestal = 64 + 30 = 94, not 30
    assert w.threshold == 94.0


def test_threshold_is_configurable():
    guard = DarkIntegrityGuard(max_above_pedestal=50.0)
    events = []
    ok = guard.check(side="left", cam_id=0, abs_frame_id=10, u1=100.0,
                     pedestal=64.0, events=events)
    assert ok is True
    assert events == []
