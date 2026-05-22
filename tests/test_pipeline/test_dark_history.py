"""DarkHistory — per-camera ring buffer of (timestamp_s, u1, std) observations."""

import pytest
from omotion.pipeline.stages.dark import DarkHistory


def test_history_starts_empty():
    h = DarkHistory(max_darks=4)
    assert h.size("left", 0) == 0
    assert h.is_empty("left", 0)


def test_append_then_size_increments():
    h = DarkHistory(max_darks=4)
    h.append("left", 0, t=0.0, u1=100.0, std=10.0)
    assert h.size("left", 0) == 1
    assert not h.is_empty("left", 0)


def test_ring_buffer_bounded_by_max_darks():
    h = DarkHistory(max_darks=3)
    for i in range(5):
        h.append("left", 0, t=float(i), u1=100.0 + i, std=10.0)
    assert h.size("left", 0) == 3
    entries = h.recent("left", 0, n=10)
    assert len(entries) == 3
    assert [e.t for e in entries] == [2.0, 3.0, 4.0]


def test_recent_n_returns_last_n_in_chronological_order():
    h = DarkHistory(max_darks=10)
    for i in range(5):
        h.append("right", 7, t=float(i), u1=100.0 + i, std=10.0)
    last3 = h.recent("right", 7, n=3)
    assert [e.u1 for e in last3] == [102.0, 103.0, 104.0]


def test_separate_cameras_have_independent_histories():
    h = DarkHistory(max_darks=4)
    h.append("left", 0, t=0.0, u1=100.0, std=10.0)
    h.append("right", 0, t=0.0, u1=200.0, std=20.0)
    assert h.recent("left", 0, n=1)[0].u1 == 100.0
    assert h.recent("right", 0, n=1)[0].u1 == 200.0


def test_clear_empties_all_cameras():
    h = DarkHistory(max_darks=4)
    h.append("left", 0, t=0.0, u1=100.0, std=10.0)
    h.append("right", 5, t=1.0, u1=200.0, std=20.0)
    h.clear()
    assert h.size("left", 0) == 0
    assert h.size("right", 5) == 0
