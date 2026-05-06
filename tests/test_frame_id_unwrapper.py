"""Unit tests for FrameIdUnwrapper.

Covers normal cycling, the camera firmware's spurious-first-frame
quirk (raw=1 then cycle 0..255), rollover detection, and the reset()
path.
"""

from omotion.MotionProcessing import FrameIdUnwrapper


def test_clean_sequence_starts_at_zero():
    """raw=0, 1, 2, ... → abs=0, 1, 2, ..."""
    u = FrameIdUnwrapper()
    seen = [u.unwrap(r) for r in range(20)]
    assert seen == list(range(20))


def test_normal_rollover():
    """raw=254, 255, 0, 1 → abs=254, 255, 256, 257."""
    u = FrameIdUnwrapper()
    abs_ids = []
    for r in range(254, 256):
        abs_ids.append(u.unwrap(r))
    abs_ids.append(u.unwrap(0))
    abs_ids.append(u.unwrap(1))
    assert abs_ids == [254, 255, 256, 257]


def test_spurious_first_frame_is_realigned():
    """The camera firmware quirk: very first raw is 1, then the cycle
    starts properly at 0 on the second frame. The unwrapper must
    produce a monotonic sequence with no duplicates."""
    u = FrameIdUnwrapper()
    sequence = [1] + list(range(256)) + list(range(20))  # 1, 0..255, 0..19
    abs_ids = [u.unwrap(r) for r in sequence]
    # Strictly monotonically increasing.
    assert all(abs_ids[i] < abs_ids[i + 1] for i in range(len(abs_ids) - 1))
    # First spurious frame goes into a "preamble" slot.
    assert abs_ids[0] == 0
    # Second frame (raw=0) is the first proper cycle frame.
    assert abs_ids[1] == 1
    # Third frame (raw=1) increments by one.
    assert abs_ids[2] == 2
    # The rollover from 255 → 0 happens at sequence index 257
    # (1 spurious + 256 proper). After that we keep counting up.
    assert abs_ids[257] == abs_ids[256] + 1


def test_anomalous_backward_jump_does_not_increment_epoch():
    """raw delta > FRAME_ROLLOVER_THRESHOLD with raw < last is treated
    as transmission noise, not a rollover."""
    u = FrameIdUnwrapper()
    # Walk up to 100, then a glitch back to 50, then continue from 50
    # (treated as an anomaly that doesn't change the epoch).
    for r in range(101):
        u.unwrap(r)
    # delta = (50 - 100) & 0xFF = 206, > 128 → anomaly
    abs_after_glitch = u.unwrap(50)
    # The unwrapper accepts the anomaly into _last_raw without
    # incrementing the epoch. Returned abs is epoch*256 + 50 = 50.
    assert abs_after_glitch == 50


def test_reset_clears_state():
    u = FrameIdUnwrapper()
    for r in range(10):
        u.unwrap(r)
    u.reset()
    # After reset the unwrapper behaves like a fresh instance.
    assert u.unwrap(0) == 0
    assert u.unwrap(1) == 1


def test_reset_preserves_quirk_handling():
    """A reset followed by the spurious-first-frame pattern still
    realigns correctly."""
    u = FrameIdUnwrapper()
    u.unwrap(0)
    u.unwrap(1)
    u.reset()
    # Quirk again: raw=1 then 0.
    assert u.unwrap(1) == 0
    assert u.unwrap(0) == 1
    assert u.unwrap(1) == 2
