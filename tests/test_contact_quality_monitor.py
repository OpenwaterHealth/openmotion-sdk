"""Tests for omotion/contact_quality.py — shared CQ semantics + live monitor.

Pure-software; no hardware, no Qt. Thresholds and values are in
background-subtracted DN, matching ContactQualityWorkflow.
"""

import logging
import math

import numpy as np
import pytest

from omotion.contact_quality import (
    REASON_AMBIENT_LIGHT,
    REASON_NO_SIGNAL,
    REASON_OK,
    REASON_POOR_CONTACT,
    TRANSITION_ACTIVATED,
    TRANSITION_CLEARED,
    TRANSITION_NONE,
    CameraLatch,
    ContactQualityMonitor,
    CQThresholds,
    _cams_from_masks,
    evaluate_reason,
    is_ambient_light,
    is_poor_contact,
)
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.pedestal import SensorPedestals
from omotion.pipeline.sinks import Sink
from omotion.pipeline.stages.dark import (
    DarkCorrectionStage,
    HybridRealtimePredictor,
    LinearInterpolation,
)
from omotion.pipeline.stages.moments import MomentsStage
from omotion.pipeline.stages.pedestal_sub import PedestalSubtractionStage

from _cq_helpers import _dn_batch


THRESHOLDS = CQThresholds.from_sequences(
    [3.0, 3.0, 3.0, 3.0, 3.0, 9.0, 3.0, 3.0],
    [15.0, 15.0, 15.0, 15.0, 15.0, 40.0, 15.0, 15.0],
)


def test_thresholds_index_per_camera():
    t = CQThresholds.from_sequences([1.0, 2.0], [10.0, 20.0])
    assert t.dark_for(0) == 1.0
    assert t.dark_for(1) == 2.0
    assert t.light_for(1) == 20.0


def test_thresholds_fail_open_out_of_range():
    """Out-of-range indices — including negatives, which the legacy
    _ContactQualitySink lookup silently wrapped instead — must return
    values that can never trip."""
    t = CQThresholds.from_sequences([1.0], [10.0])
    assert t.dark_for(7) == math.inf      # nothing exceeds inf
    assert t.light_for(7) == 0.0          # nothing falls below 0
    # Legacy _ContactQualitySink indexed with a bare `self._dark[cam_id]`,
    # so a negative cam_id would wrap to the last element instead of
    # failing open. This is an intentional divergence, not a bug.
    assert t.dark_for(-1) == math.inf
    assert t.light_for(-1) == 0.0


def test_from_sequences_warns_on_wrong_length_but_fails_open(caplog):
    """A 6-element array must not fail silently — it should warn that
    cameras 6-7 are permanently unflaggable, while still failing open
    (never raising)."""
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.contact_quality"):
        t = CQThresholds.from_sequences([1.0] * 6, [10.0] * 6)
    assert "6 entries" in caplog.text
    assert "expected 8" in caplog.text
    assert "fail open" in caplog.text
    assert "cq_dark_threshold_per_camera" in caplog.text
    assert "cq_light_threshold_per_camera" in caplog.text
    assert t.dark_for(7) == math.inf
    assert t.light_for(7) == 0.0


def test_from_sequences_warns_on_long_length_but_ignores_extras(caplog):
    """A 16-element array has nothing that fails open — cameras 0-7 all get
    a real threshold and the extra entries are simply unused — so the
    warning must say the extras are ignored, and must NOT claim anything
    fails open (that claim would be false for this branch)."""
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.contact_quality"):
        t = CQThresholds.from_sequences([1.0] * 16, [10.0] * 16)
    assert "16 entries" in caplog.text
    assert "expected 8" in caplog.text
    assert "ignored" in caplog.text
    assert "fail open" not in caplog.text
    assert "cq_dark_threshold_per_camera" in caplog.text
    assert "cq_light_threshold_per_camera" in caplog.text
    assert t.dark_for(7) == 1.0
    assert t.light_for(7) == 10.0


def test_thresholds_empty_sequences_fail_open_for_every_camera(caplog):
    """Degenerate case: empty arrays are the documented fail-open contract
    taken to its limit — every camera reads back as ok.

    This isn't testing the wrong-length warning, so silence it. Note the
    level is CRITICAL, not WARNING: this logger's ambient level is already
    WARNING, so at_level(WARNING) is a no-op that still lets the warning
    reach pytest's log_cli reporter — only raising above WARNING actually
    keeps it out of CI output.
    """
    with caplog.at_level(logging.CRITICAL, logger="openmotion.sdk.contact_quality"):
        t = CQThresholds.from_sequences([], [])
    assert evaluate_reason(
        light_avg=1.0, dark_max=99.0, thresholds=t, cam_id=0
    ) == REASON_OK


def test_thresholds_from_sequences_coerces_ints_to_float(caplog):
    """Config arrives from JSON as ints; from_sequences must coerce to
    float so comparisons behave consistently downstream.

    2 entries also triggers the wrong-length warning; silence it (see the
    comment on test_thresholds_empty_sequences_fail_open_for_every_camera
    for why CRITICAL, not WARNING, is the level that actually suppresses
    it in log_cli output).
    """
    with caplog.at_level(logging.CRITICAL, logger="openmotion.sdk.contact_quality"):
        t = CQThresholds.from_sequences([1, 2], [10, 20])
    assert isinstance(t.dark, tuple)
    assert t.dark_for(0) == 1.0
    assert isinstance(t.dark_for(0), float)
    assert t.light_for(1) == 20.0
    assert isinstance(t.light_for(1), float)


def test_is_ambient_light_only_above_threshold():
    assert is_ambient_light(3.5, THRESHOLDS, 0) is True
    assert is_ambient_light(3.0, THRESHOLDS, 0) is False   # strict >
    assert is_ambient_light(-1.0, THRESHOLDS, 0) is False


def test_is_ambient_light_false_for_non_finite():
    assert is_ambient_light(float("nan"), THRESHOLDS, 0) is False


def test_is_poor_contact_only_below_threshold():
    assert is_poor_contact(5.0, THRESHOLDS, 0) is True
    assert is_poor_contact(15.0, THRESHOLDS, 0) is False    # strict <
    assert is_poor_contact(60.0, THRESHOLDS, 0) is False


def test_is_poor_contact_false_for_non_finite():
    assert is_poor_contact(float("nan"), THRESHOLDS, 0) is False


def test_predicates_use_the_cameras_own_threshold():
    """A mutation that ignores cam_id and always reads camera 0's threshold
    must fail here — per-camera routing is the whole point of the arrays."""
    assert is_ambient_light(5.0, THRESHOLDS, 0) is True    # cam 0 bar = 3.0
    assert is_ambient_light(5.0, THRESHOLDS, 5) is False   # cam 5 bar = 9.0
    assert is_poor_contact(20.0, THRESHOLDS, 0) is False
    assert is_poor_contact(20.0, THRESHOLDS, 5) is True


def test_infinite_reading_is_unusable_not_ambient_light():
    """An infinite reading is treated as unusable data, not as evidence of
    ambient light — matches legacy's math.isfinite guard exactly."""
    assert is_ambient_light(float("inf"), THRESHOLDS, 0) is False
    assert evaluate_reason(
        light_avg=float("inf"), dark_max=0.0, thresholds=THRESHOLDS, cam_id=0
    ) == REASON_NO_SIGNAL


def test_evaluate_reason_precedence_matches_legacy_order():
    """no_signal > ambient_light > poor_contact > ok."""
    nan = float("nan")
    assert evaluate_reason(
        light_avg=nan, dark_max=99.0, thresholds=THRESHOLDS, cam_id=0
    ) == REASON_NO_SIGNAL
    # ambient wins over poor contact when both conditions hold
    assert evaluate_reason(
        light_avg=1.0, dark_max=99.0, thresholds=THRESHOLDS, cam_id=0
    ) == REASON_AMBIENT_LIGHT
    assert evaluate_reason(
        light_avg=1.0, dark_max=0.0, thresholds=THRESHOLDS, cam_id=0
    ) == REASON_POOR_CONTACT
    assert evaluate_reason(
        light_avg=60.0, dark_max=0.0, thresholds=THRESHOLDS, cam_id=0
    ) == REASON_OK


def test_latch_debounce_one_is_immediate():
    latch = CameraLatch(activate_debounce=1, clear_debounce=1)
    assert latch.observe(True) == TRANSITION_ACTIVATED
    assert latch.observe(True) == TRANSITION_NONE      # steady state, no repeat
    assert latch.observe(False) == TRANSITION_CLEARED


def test_latch_default_debounce_is_one():
    """Both debounces default to 1 — not just values tests happen to pass."""
    latch = CameraLatch()
    assert latch.activate_debounce == 1
    assert latch.clear_debounce == 1
    assert latch.observe(True) == TRANSITION_ACTIVATED


def test_latch_requires_consecutive_agreeing_observations():
    latch = CameraLatch(activate_debounce=3, clear_debounce=3)
    assert latch.observe(True) == TRANSITION_NONE
    assert latch.active is False   # pending evidence, not yet latched
    assert latch.observe(True) == TRANSITION_NONE
    assert latch.observe(True) == TRANSITION_ACTIVATED


def test_latch_streak_resets_on_disagreement():
    """A dip shorter than the debounce must produce no transition at all."""
    latch = CameraLatch(activate_debounce=3, clear_debounce=3)
    assert latch.observe(True) == TRANSITION_NONE
    assert latch.observe(True) == TRANSITION_NONE
    assert latch.observe(False) == TRANSITION_NONE     # streak broken
    assert latch.observe(True) == TRANSITION_NONE
    assert latch.observe(True) == TRANSITION_NONE
    assert latch.observe(True) == TRANSITION_ACTIVATED


def test_latch_full_activate_clear_reactivate_round_trip():
    """Activate -> clear -> re-activate at debounce > 1.

    A single clear edge is the easy case; this exact cycle repeats thousands
    of times over a 12 h scan — a state leak from one edge into the next
    would surface here, not in a single-edge test."""
    latch = CameraLatch(activate_debounce=2, clear_debounce=2)
    latch.observe(True)
    assert latch.observe(True) == TRANSITION_ACTIVATED
    assert latch.observe(False) == TRANSITION_NONE
    assert latch.observe(False) == TRANSITION_CLEARED
    assert latch.observe(True) == TRANSITION_NONE
    assert latch.observe(True) == TRANSITION_ACTIVATED


@pytest.mark.parametrize("debounce", [0, -1, -5])
def test_latch_debounce_floor_is_one(debounce):
    """Zero or negative debounce behaves as debounce=1 on both edges."""
    latch = CameraLatch(activate_debounce=debounce, clear_debounce=debounce)
    assert latch.activate_debounce == 1
    assert latch.clear_debounce == 1
    assert latch.observe(True) == TRANSITION_ACTIVATED


def test_latch_activate_and_clear_debounce_independently():
    """The two edges are governed by separate bounds. With a fast activate
    (2) and a slow clear (5): RAISE fires on the 2nd consecutive bad — not
    the 5th — and once active, CLEAR fires only on the 5th consecutive good
    — not the 2nd. This is the safety asymmetry: warn quickly, dismiss
    conservatively."""
    latch = CameraLatch(activate_debounce=2, clear_debounce=5)
    # RAISE on the 2nd bad, well before the clear bound of 5.
    assert latch.observe(True) == TRANSITION_NONE
    assert latch.observe(True) == TRANSITION_ACTIVATED
    # Now active. The 2nd good must NOT clear — the clear edge needs 5, and
    # the activate bound of 2 does not leak into this direction.
    assert latch.observe(False) == TRANSITION_NONE
    assert latch.observe(False) == TRANSITION_NONE
    assert latch.observe(False) == TRANSITION_NONE
    assert latch.observe(False) == TRANSITION_NONE
    assert latch.observe(False) == TRANSITION_CLEARED   # only the 5th good


# ---------------------------------------------------------------------------
# ContactQualityMonitor — live sink
# ---------------------------------------------------------------------------

class _FakeMeta:
    """Stands in for ScanMetadata — the monitor reads only the two masks."""

    def __init__(self, left_camera_mask=0xFF, right_camera_mask=0xFF):
        self.left_camera_mask = left_camera_mask
        self.right_camera_mask = right_camera_mask


def _monitor(events, **kwargs):
    kwargs.setdefault("rolling_window", 1)
    kwargs.setdefault("light_activate_debounce", 1)
    kwargs.setdefault("light_clear_debounce", 1)
    kwargs.setdefault("dark_debounce", 1)
    return ContactQualityMonitor(
        thresholds=THRESHOLDS,
        on_transition=lambda *a: events.append(a),
        **kwargs,
    )


def test_monitor_satisfies_sink_protocol_on_the_live_channel():
    """channels is otherwise completely untested: a class-attribute typo —
    e.g. frozenset({"final"}) instead of frozenset({"live"}) — leaves every
    other test in this file green, because every other test constructs the
    monitor and calls consume("live", ...) directly, bypassing the runner's
    channel-based dispatch entirely. In a real scan the runner would simply
    never route a single batch to the monitor. This is exactly the
    never-wired-up failure shape the whole feature exists to correct."""
    mon = _monitor([])
    assert isinstance(mon, Sink)
    assert "live" in mon.channels


def test_monitor_reports_poor_contact_when_light_drops():
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(1, 60.0))   # healthy
    assert events == []
    mon.consume("live", _dn_batch(1, 2.0))    # fiber pulled
    assert events == [("left", 0, REASON_POOR_CONTACT, pytest.approx(2.0), True)]


def test_monitor_clears_poor_contact_on_recovery():
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(1, 2.0))
    events.clear()
    mon.consume("live", _dn_batch(1, 60.0))
    assert events == [("left", 0, REASON_POOR_CONTACT, pytest.approx(60.0), False)]


def test_monitor_debounce_suppresses_short_dip():
    """A dip shorter than light_activate_debounce must not raise at all."""
    events = []
    mon = _monitor(events, light_activate_debounce=3, light_clear_debounce=3)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(2, 2.0))    # 2 bad observations, need 3
    assert events == []
    mon.consume("live", _dn_batch(1, 60.0))   # recovered, streak resets
    assert events == []


def test_monitor_light_activate_and_clear_debounce_independently():
    """Monitor-level asymmetry: a camera driven below threshold RAISES after
    light_activate_debounce bad frames (fast — a late warning is a safety
    miss), but once raised CLEARS only after light_clear_debounce good frames
    (slow — a premature dismiss strands the operator). window=1 (the _monitor
    default) so each frame is its own observation."""
    events = []
    mon = _monitor(events, light_activate_debounce=2, light_clear_debounce=5)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    # RAISE only on the 2nd consecutive bad frame.
    mon.consume("live", _dn_batch(1, 2.0))
    assert events == []
    mon.consume("live", _dn_batch(1, 2.0))
    assert events == [("left", 0, REASON_POOR_CONTACT, pytest.approx(2.0), True)]
    events.clear()
    # Once raised, 4 good frames must NOT clear — the fast activate bound of 2
    # does not apply to the clear edge.
    mon.consume("live", _dn_batch(4, 60.0))
    assert events == []
    # The 5th consecutive good frame clears.
    mon.consume("live", _dn_batch(1, 60.0))
    assert events == [("left", 0, REASON_POOR_CONTACT, pytest.approx(60.0), False)]


def test_monitor_dark_debounce_is_independent_of_light_debounce():
    """Every other test in this file sets all debounces to 1, so a
    dark_debounce <-> light-debounce mix-up in the dark branch survives
    unnoticed. Production leaves dark_debounce=1 (the scheduled ~15 s
    cadence is treated as an immediate latch) and drives the light debounces
    much higher (activate 10 / clear 80); swapping the dark bound for a light
    one would silently push ambient-light latency from ~15 s to tens of dark
    observations (~minutes at the default dark_interval)."""
    events = []
    mon = _monitor(events, light_activate_debounce=5, light_clear_debounce=5,
                   dark_debounce=1)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(1, 9.0, frame_types=["dark"]))
    assert events == [("left", 0, REASON_AMBIENT_LIGHT, pytest.approx(9.0), True)]


def test_monitor_dark_uses_latest_not_running_max():
    """A running max would latch an ambient warning for the rest of a 12 h
    scan. The live monitor must track the current dark instead."""
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(1, 9.0, frame_types=["dark"]))
    events.clear()
    mon.consume("live", _dn_batch(1, 0.5, frame_types=["dark"]))
    assert events == [("left", 0, REASON_AMBIENT_LIGHT, pytest.approx(0.5), False)]


def test_monitor_ignores_cameras_outside_scan_mask():
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(1, 2.0))
    cams = {(side, cam) for side, cam, *_ in events}
    assert cams == {("left", 0)}


def test_monitor_skips_warmup_and_stale_rows():
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(2, 2.0, frame_types=["warmup", "stale"]))
    assert events == []


@pytest.mark.parametrize(
    "frame_types",
    [None, ["dark"]],
    ids=["light", "dark"],
)
def test_monitor_skips_non_finite_values(frame_types):
    """A NaN reading must be skipped, not turned into a transition — on the
    light path and the dark path alike. Frame loss belongs to the consumer's
    camera-dropout watchdog; reporting it as a contact-quality fault would
    misdirect the operator."""
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(1, float("nan"), frame_types=frame_types))
    assert events == []


def test_monitor_never_emits_no_signal_reason():
    """REASON_NO_SIGNAL is preflight-only (ContactQualityWorkflow's precedence
    order) — total frame loss belongs to the consumer's camera-dropout
    watchdog, never to the live contact-quality stream.

    The parametrized non-finite-skip test above already pins "stays silent"
    for an isolated bad reading (both its light and dark cases), which is the
    strongest possible assertion in that shape (an empty list rules out an
    event of any reason, no_signal included). What it can't cover is a stream
    that legitimately DOES emit
    real events (poor_contact / ambient_light) while garbage readings are
    interleaved — this test pins that the noise never leaks REASON_NO_SIGNAL
    into that otherwise-real event stream, e.g. if a future refactor routed
    the dark or light path through evaluate_reason() (which does have a
    no_signal branch) without first filtering non-finite values."""
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(1, float("nan"), frame_types=["dark"]))
    mon.consume("live", _dn_batch(1, 2.0))                        # real poor-contact
    mon.consume("live", _dn_batch(1, float("nan"), frame_types=["dark"]))
    mon.consume("live", _dn_batch(1, 9.0, frame_types=["dark"]))  # real ambient-light
    assert len(events) == 2   # the two real detections, and nothing else
    reasons = {reason for _side, _cam, reason, _value, _active in events}
    assert REASON_NO_SIGNAL not in reasons
    assert reasons == {REASON_POOR_CONTACT, REASON_AMBIENT_LIGHT}


def test_monitor_ignores_other_channels():
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("final", _dn_batch(1, 2.0))
    mon.consume("diagnostics", _dn_batch(1, 2.0))
    assert events == []


def test_monitor_on_scan_start_resets_state():
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(1, 2.0))
    events.clear()
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(1, 2.0))
    # Fresh scan: the camera re-activates rather than staying silently latched.
    assert events == [("left", 0, REASON_POOR_CONTACT, pytest.approx(2.0), True)]


def test_monitor_on_scan_start_clears_light_window():
    """test_monitor_on_scan_start_resets_state above only pins the latch
    resetting — it never fills the rolling window, so a missing
    _light_window.clear() would pass it unnoticed. Fill the window with
    healthy samples in scan 1 (leaving it short of maxlen, so a fresh
    per-key deque and a merely-unfull old one are distinguishable); a
    correctly-reset scan 2 must judge its own single bad sample entirely on
    its own, not blended with scan 1's leftovers."""
    events = []
    mon = _monitor(events, rolling_window=4, light_activate_debounce=1,
                   light_clear_debounce=1)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(3, 60.0))    # 3/4 of the window, healthy
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    events.clear()
    mon.consume("live", _dn_batch(1, 2.0))     # single bad sample, fresh scan
    assert events == [("left", 0, REASON_POOR_CONTACT, pytest.approx(2.0), True)]


def test_monitor_emits_edges_only_not_every_frame():
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(10, 2.0))
    assert len(events) == 1


def test_monitor_rolling_window_averages_light_frames():
    """window=4 over [60,60,60,0] averages 45 — above threshold, no warning."""
    events = []
    mon = _monitor(events, rolling_window=4)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(3, 60.0))
    mon.consume("live", _dn_batch(1, 0.0))
    assert events == []


def test_monitor_rolling_window_zero_clamps_to_one():
    """rolling_window=0 must clamp to 1 rather than constructing
    deque(maxlen=0) — appending to a maxlen=0 deque silently discards the
    item, so sum(window)/len(window) becomes 0/0 and raises
    ZeroDivisionError on every light row. The bloodflow-app's `_cq_int_or`
    helper is reported to rely on exactly this clamp when a config value
    coerces to 0 or is missing (unverified from this SDK worktree — the
    app repo isn't checked out here)."""
    events = []
    mon = _monitor(events, rolling_window=0)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(1, 2.0))     # must not raise
    assert events == [("left", 0, REASON_POOR_CONTACT, pytest.approx(2.0), True)]


def test_monitor_survives_a_raising_callback():
    """_safe_consume (runner.py) logs and continues on a raising sink — it
    does not disable one; only an on_scan_start failure does that. But an
    uncaught callback exception would still unwind this whole consume()
    call, abandoning every remaining row in the batch (silently skipping
    the other cameras) and logging a fresh traceback per batch at ~40 Hz.
    A bad UI callback must not do that to contact-quality monitoring."""
    def boom(*_args):
        raise RuntimeError("UI exploded")

    mon = ContactQualityMonitor(
        thresholds=THRESHOLDS,
        on_transition=boom,
        rolling_window=1,
        light_activate_debounce=1,
        light_clear_debounce=1,
    )
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(1, 2.0))    # must not raise
    mon.on_complete()


def test_monitor_handles_batch_without_dn_fields():
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta())
    batch = _dn_batch(1, 2.0)
    batch.subtracted_mean = None
    mon.consume("live", batch)                # must not raise
    assert events == []

    # The mean_dc_rt check is the second operand of the guard's `or` and
    # was previously untested on its own: with subtracted_mean=None, the
    # `or` short-circuits before mean_dc_rt is ever evaluated. Null only
    # mean_dc_rt so the second operand is what's actually exercised.
    batch2 = _dn_batch(1, 2.0)
    batch2.mean_dc_rt = None
    mon.consume("live", batch2)                # must not raise
    assert events == []


# ---------------------------------------------------------------------------
# Unlit frame (disconnected fiber) via low_light_rt — issue #364
#
# mean_dc_rt is NaN for two different reasons and _dn_batch's uniform-value
# construction can only represent one of them (warmup). DarkCorrectionStage
# also leaves mean_dc_rt NaN for a light-typed frame that arrived but was
# unlit (low_light_rt=True) — a covered sensor, a lifted sensor, or a
# decoupled fiber. That must surface as poor_contact, not be silently
# skipped the way a warmup NaN is.
# ---------------------------------------------------------------------------

def test_monitor_reports_poor_contact_for_unlit_frame_via_low_light_rt():
    """The disconnected-fiber case issue #364 was filed about: the frame
    DID arrive (low_light_rt=True), it was just unlit. Must report
    poor_contact, not be treated the same as a warmup/missing-frame NaN."""
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    batch = _dn_batch(1, 2.0)
    batch.mean_dc_rt[:] = float("nan")     # DarkCorrectionStage suppressed it
    low = np.zeros((16, 2, 8), dtype=bool)
    low[0, 0, 0] = True                    # row 0 == (left, cam 0)
    batch.low_light_rt = low
    mon.consume("live", batch)
    assert events == [("left", 0, REASON_POOR_CONTACT, pytest.approx(2.0), True)]


def test_monitor_reports_unlit_frame_as_poor_contact_even_above_light_threshold():
    """The low_light_rt=True branch reports poor_contact unconditionally —
    NOT via is_poor_contact(subtracted_mean, ...) — because subtracted_mean
    is pedestal-referenced while the light threshold was calibrated against
    mean_dc_rt's dark-baseline-referenced scale; comparing the two against
    the same bound would be apples-to-oranges (see the comment in
    consume()). Every other unlit fixture in this file happens to use a
    subtracted_mean already below the light threshold, so a mutant that ran
    it through is_poor_contact would still report poor_contact by
    coincidence and pass unnoticed. Use subtracted_mean=40.0 — above cam
    0's 15.0 light bar: the real code still reports poor_contact (this is
    judged structurally, not by threshold), while that mutant would clear
    it as healthy."""
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    batch = _dn_batch(1, 40.0)               # above the 15.0 light threshold
    batch.mean_dc_rt[:] = float("nan")       # DarkCorrectionStage suppressed it
    low = np.zeros((16, 2, 8), dtype=bool)
    low[0, 0, 0] = True
    batch.low_light_rt = low
    mon.consume("live", batch)
    assert events == [("left", 0, REASON_POOR_CONTACT, pytest.approx(40.0), True)]


def test_monitor_skips_non_finite_light_when_low_light_rt_present_but_false():
    """low_light_rt existing on the batch but False for this row means 'no
    verdict yet' (e.g. still in the warmup window before the first dark),
    not 'unlit' — must still be skipped, not treated as poor contact.

    subtracted_mean must be finite here, unlike mean_dc_rt — that is the
    real warmup shape: a frame that arrived carrying a real signal but has
    no dark baseline yet to subtract. Building both fields as NaN (as a
    naive fixture would) is inert: it can't distinguish the correct code
    from a mutant that drops the "not bool(low_light_rt[...])" half of the
    guard, because that mutant falls through to the subtracted_mean check,
    which was ALSO NaN by construction and skips anyway, by coincidence.
    With subtracted_mean finite, that same mutant instead reports a false
    poor-contact warning at the start of every scan."""
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    batch = _dn_batch(1, 60.0)              # finite subtracted_mean — real signal
    batch.mean_dc_rt[:] = float("nan")      # no dark baseline yet
    batch.low_light_rt = np.zeros((16, 2, 8), dtype=bool)   # all False
    mon.consume("live", batch)
    assert events == []


def test_monitor_skips_unlit_frame_when_subtracted_mean_also_non_finite():
    """Defensive: even when low_light_rt says the frame was unlit, a
    non-finite subtracted_mean (e.g. corrupted upstream) must still be
    skipped rather than reported with a garbage value."""
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    batch = _dn_batch(1, float("nan"))   # both fields NaN by construction
    low = np.zeros((16, 2, 8), dtype=bool)
    low[0, 0, 0] = True
    batch.low_light_rt = low
    mon.consume("live", batch)
    assert events == []


def test_monitor_catches_disconnected_fiber_through_real_pipeline_stages():
    """The regression test that would have caught the bug: mean_dc_rt is
    NaN for the reason DarkCorrectionStage actually produces it for an
    unlit frame (low_light_rt=True), a combination the synthetic _dn_batch
    helper cannot represent (it sets subtracted_mean and mean_dc_rt to the
    same finite array — DarkCorrectionStage never produces that pairing
    for an unlit frame). Route real DN data through the real stages: a
    dark frame, a healthy light frame, then an unlit light frame — the
    disconnected-fiber shape issue #364 was filed about — on one camera."""
    n = 3
    pedestals = SensorPedestals(left=64.0, right=64.0)
    # dark @ u1=64 (== pedestal, avoids an incidental ambient-light trip);
    # healthy light @ u1=500 (mean_dc_rt = 500-64 = 436, far above the 15
    # DN light threshold); unlit light @ u1=65 (<= pedestal 64 + guard 5
    # => dark_like => low_light_rt=True, mean_dc_rt stays NaN).
    raw = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    raw[0, 0, 0, 64]  = 1000
    raw[1, 0, 0, 500] = 1000
    raw[2, 0, 0, 65]  = 1000

    batch = FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        side_ids=np.zeros(n, dtype=np.int8),
        raw_histograms=raw,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64) * 0.025,
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.array([10, 11, 12], dtype=np.int64),
        frame_type=np.array(["dark", "light", "light"], dtype="<U8"),
    )

    MomentsStage().process(batch)
    PedestalSubtractionStage(pedestals).process(batch)
    DarkCorrectionStage(
        realtime_estimator=HybridRealtimePredictor(),
        batch_estimator=LinearInterpolation(),
        pedestals=pedestals,
    ).process(batch)

    # Confirm the setup actually produced the shape of bug this guards
    # against, rather than trusting it blindly.
    assert math.isnan(batch.mean_dc_rt[2, 0, 0])
    assert bool(batch.low_light_rt[2, 0, 0]) is True

    events = []
    mon = ContactQualityMonitor(
        thresholds=THRESHOLDS,
        on_transition=lambda *a: events.append(a),
        rolling_window=1,
        light_activate_debounce=1,
        light_clear_debounce=1,
        dark_debounce=1,
    )
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", batch)

    assert events == [("left", 0, REASON_POOR_CONTACT, pytest.approx(1.0), True)]


# ---------------------------------------------------------------------------
# Per-camera keying — every test above this line uses exactly one camera
# (left, index 0); _dn_batch fills all 16 (side, cam) slots identically, so
# per-camera divergence was entirely unrepresentable and a shared latch, a
# shared window, or a hardcoded cam_id=0 predicate lookup could all pass
# the full suite unnoticed.
# ---------------------------------------------------------------------------

def _dn_batch_percam(n_frames, values, *, default=60.0, frame_types=None):
    """Like _dn_batch, but each (side, cam_id) can carry its own DN value.

    ``values`` maps (side, cam_id) -> dn_value for the slots that matter;
    every other (side, cam_id) gets ``default`` (healthy against every
    camera's own threshold, so an untouched camera never accidentally
    trips). subtracted_mean and mean_dc_rt both carry the same per-camera
    value, same convention as _dn_batch.
    """
    if frame_types is None:
        frame_types = ["light"] * n_frames
    rows = n_frames * 16
    cam_ids = np.tile(np.arange(8, dtype=np.int8), n_frames * 2)
    side_ids = np.tile(np.repeat(np.array([0, 1], dtype=np.int8), 8), n_frames)
    subtracted = np.full((rows, 2, 8), default, dtype=np.float32)
    dc_rt = np.full((rows, 2, 8), default, dtype=np.float32)
    for (side, cam_id), v in values.items():
        side_idx = 0 if side == "left" else 1
        subtracted[:, side_idx, cam_id] = v
        dc_rt[:, side_idx, cam_id] = v
    return FrameBatch(
        cam_ids=cam_ids,
        frame_ids=np.tile(np.arange(n_frames, dtype=np.uint8).repeat(16), 1),
        side_ids=side_ids,
        raw_histograms=None,
        temperature_c=None,
        timestamp_s=np.zeros(rows, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        frame_type=np.repeat(np.array(frame_types, dtype="<U8"), 16),
        subtracted_mean=subtracted,
        mean_dc_rt=dc_rt,
        std_raw=np.full((rows, 2, 8), 2.5, dtype=np.float32),
    )


def test_monitor_keys_latch_and_window_per_camera_not_globally():
    """L3 and R2 stay healthy; only L5 goes bad, and only against its own
    40.0 light threshold (25 DN reads as healthy against the generic 15.0
    bar, so this also pins per-camera threshold lookup). A shared latch, a
    shared window, or a forced cam_id=0 predicate lookup would each corrupt
    this differently — verified against all three below."""
    events = []
    mon = _monitor(events, rolling_window=2)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x28, right_camera_mask=0x04))
    mon.consume("live", _dn_batch_percam(1, {
        ("left", 3): 60.0,    # healthy against its own 15.0 bar
        ("left", 5): 25.0,    # bad against its OWN 40.0 bar; "healthy" vs 15.0
        ("right", 2): 60.0,   # healthy against its own 15.0 bar
    }))
    assert events == [("left", 5, REASON_POOR_CONTACT, pytest.approx(25.0), True)]


def test_monitor_dark_row_never_pollutes_light_window():
    """Real batches are 10-100 frames and routinely span a dark boundary —
    a bug where the dark branch appended to _light_window would corrupt
    every rolling average downstream of a dark frame. An extreme (and
    physically valid — subtracted_mean can go negative) dark reading would
    immediately trip poor_contact if it ever leaked into the light window;
    it must not."""
    events = []
    mon = _monitor(events, rolling_window=3)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(1, -500.0, frame_types=["dark"]))
    mon.consume("live", _dn_batch(1, 60.0))   # healthy light
    mon.consume("live", _dn_batch(1, 60.0))   # healthy light
    assert events == []


def test_cams_from_masks_none_means_evaluate_everything():
    assert _cams_from_masks(None) == set()


def test_monitor_on_scan_start_none_evaluates_every_camera():
    """meta=None means 'no mask restriction' — every camera must still be
    evaluated, not silently skipped (e.g. a bare pipeline test with no real
    ScanMetadata available)."""
    events = []
    mon = _monitor(events)
    mon.on_scan_start(None)
    mon.consume("live", _dn_batch(1, 2.0))   # uniformly bad, all 16 (side,cam)
    cams = {(side, cam) for side, cam, *_ in events}
    assert len(cams) == 16   # both sides, all 8 cameras — nothing masked out


# ---------------------------------------------------------------------------
# on_complete summary — the feature was dead for two months in 2026 because
# silence looked identical to success; on_complete must say which happened.
# ---------------------------------------------------------------------------

def test_monitor_on_complete_logs_a_summary(caplog):
    events = []
    mon = _monitor(events)
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _dn_batch(3, 60.0))    # healthy, no transitions
    mon.consume("live", _dn_batch(1, 2.0))     # one activation
    with caplog.at_level(logging.INFO, logger="openmotion.sdk.contact_quality"):
        mon.on_complete()
    assert "1 transition(s) emitted" in caplog.text
    assert "1 camera(s) observed" in caplog.text
    assert "4 observation(s) processed" in caplog.text


# ---------------------------------------------------------------------------
# Parity with the one-shot sink — the whole reason the core is shared rather
# than duplicated.
# ---------------------------------------------------------------------------

def test_shared_evaluator_matches_one_shot_sink_verdicts():
    """The one-shot sink and the shared evaluator must agree — this is the
    whole reason the core is shared rather than duplicated.

    Each case is a sequence of consume() batches to feed the sink, paired
    with the reason its own result() should produce. The pure cases (60.0,
    2.0) alone never call is_ambient_light and is_poor_contact in the same
    evaluation, so a precedence bug in evaluate_reason could pass unnoticed
    even though "the sink and the evaluator agree" holds trivially in both
    directions. The co-occurrence case below is the one that actually
    exercises the precedence branch: a bright dark frame (9.0 > the 3.0 dark
    threshold) together with dim light frames (2.0 < the 15.0 light
    threshold) makes both is_ambient_light and is_poor_contact true at once,
    so only the precedence order — ambient_light before poor_contact —
    decides the verdict. The no-light-rows case does the same for the
    no_signal branch: it pairs a tripped dark threshold with zero light
    data, so only "no_signal outranks ambient_light" decides the verdict,
    not merely "no data at all reports no_signal".
    """
    from omotion.ContactQualityWorkflow import _ContactQualitySink

    cases = (
        ([_dn_batch(4, 60.0)], REASON_OK),
        ([_dn_batch(4, 2.0)], REASON_POOR_CONTACT),
        (
            [
                _dn_batch(1, 9.0, frame_types=["dark"]),   # dark_max 9.0 > 3.0
                _dn_batch(4, 2.0),                         # light_avg 2.0 < 15.0
            ],
            REASON_AMBIENT_LIGHT,
        ),
        (
            [_dn_batch(1, 9.0, frame_types=["dark"])],     # no light rows at all
            REASON_NO_SIGNAL,
        ),
    )

    for batches, expected in cases:
        sink = _ContactQualitySink(
            dark_thresholds=[3.0] * 8, light_thresholds=[15.0] * 8
        )
        sink.on_scan_start(None)
        for batch in batches:
            sink.consume("live", batch)
        cam = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0).per_camera[
            ("left", 0)
        ]
        assert cam.reason == expected
        assert cam.reason == evaluate_reason(
            light_avg=cam.light_avg_dn,
            dark_max=cam.dark_max_dn,
            thresholds=CQThresholds.from_sequences([3.0] * 8, [15.0] * 8),
            cam_id=0,
        )


# ---------------------------------------------------------------------------
# Verdict history — verdict_at / observed_through (bloodflow-app#589)
# ---------------------------------------------------------------------------

def _fid_batch(first_fid, n_frames, dn_value, frame_types=None):
    b = _dn_batch(n_frames, dn_value, frame_types)
    b.abs_frame_ids = np.repeat(
        np.arange(first_fid, first_fid + n_frames, dtype=np.int64), 16)
    return b


def test_verdict_string_is_ok_or_fixed_order_reasons():
    from omotion.contact_quality import verdict_string
    assert verdict_string([]) == "ok"
    assert verdict_string([REASON_AMBIENT_LIGHT, REASON_POOR_CONTACT]) == \
        "poor_contact,ambient_light"


def test_verdict_at_follows_the_latched_state_by_frame():
    mon = _monitor([])
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    assert mon.observed_through() == -1
    mon.consume("live", _fid_batch(100, 1, 60.0))   # healthy
    mon.consume("live", _fid_batch(101, 1, 2.0))    # latches poor contact
    mon.consume("live", _fid_batch(102, 1, 60.0))   # clears
    assert mon.observed_through() == 102
    assert mon.verdict_at("left", 0, 99) == "ok"
    assert mon.verdict_at("left", 0, 100) == "ok"
    assert mon.verdict_at("left", 0, 101) == "poor_contact"
    assert mon.verdict_at("left", 0, 102) == "ok"
    assert mon.verdict_at("left", 0, 500) == "ok"


def test_verdict_at_combines_simultaneous_conditions():
    mon = _monitor([])
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _fid_batch(10, 1, 2.0))                 # poor contact
    mon.consume("live", _fid_batch(11, 1, 50.0, ["dark"]))      # + ambient
    assert mon.verdict_at("left", 0, 10) == "poor_contact"
    assert mon.verdict_at("left", 0, 11) == "poor_contact,ambient_light"


def test_verdict_at_is_none_outside_the_scan_mask():
    mon = _monitor([])
    mon.on_scan_start(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    mon.consume("live", _fid_batch(10, 1, 60.0))
    assert mon.verdict_at("left", 1, 10) is None
    assert mon.verdict_at("right", 0, 10) is None


def test_history_resets_per_scan():
    mon = _monitor([])
    meta = _FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00)
    mon.on_scan_start(meta)
    mon.consume("live", _fid_batch(10, 1, 2.0))
    mon.on_scan_start(meta)
    assert mon.observed_through() == -1
    assert mon.verdict_at("left", 0, 10) == "ok"
