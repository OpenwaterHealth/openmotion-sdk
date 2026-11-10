"""The ``quality`` flag list (bloodflow-app#589).

A frame can carry several flags at once, e.g. a re-anchored timestamp on an
unlit camera. Before the list, the later stage replaced the earlier flag and
the side average kept only the worst one, so the record lost the rest.
"""

from __future__ import annotations

from omotion.pipeline.quality_flags import (
    LOW_LIGHT, NAN_FILLED, OK, TS_CORRECTED, has, join, parse,
)
from omotion.pipeline.stages.dark import (
    DarkObservation, Interval, LinearInterpolation, _DarkBoundary, _LightSample,
)


def test_parse_drops_ok_and_blanks():
    assert parse(None) == []
    assert parse("") == []
    assert parse(OK) == []
    assert parse("ts_corrected;low_light") == [TS_CORRECTED, LOW_LIGHT]
    assert parse(" ts_corrected ; ok ;") == [TS_CORRECTED]


def test_join_is_ok_without_flags():
    assert join() == OK
    assert join(None, "", OK) == OK


def test_join_lists_each_flag_once_in_a_fixed_order():
    assert join(LOW_LIGHT, TS_CORRECTED) == "ts_corrected;low_light"
    assert join("low_light;ts_corrected", TS_CORRECTED) == "ts_corrected;low_light"
    assert join(NAN_FILLED, LOW_LIGHT, TS_CORRECTED) == "ts_corrected;nan_filled;low_light"


def test_join_keeps_unknown_flags_after_known_ones():
    assert join("future_b", LOW_LIGHT, "future_a") == "low_light;future_b;future_a"


def test_has():
    assert has("ts_corrected;low_light", LOW_LIGHT)
    assert not has("ts_corrected", LOW_LIGHT)
    assert not has(None, LOW_LIGHT)
    assert not has(OK, OK)


def test_unlit_frame_keeps_its_earlier_flag():
    """An unlit frame whose timestamp was also re-anchored carries both."""
    dark = DarkObservation(t=0.0, u1=128.0, std=2.0)
    interval = Interval(
        left=_DarkBoundary(obs=dark, abs_frame_id=0),
        right=_DarkBoundary(obs=DarkObservation(t=1.0, u1=128.0, std=2.0),
                            abs_frame_id=40),
        light_frames=[
            _LightSample(abs_frame_id=10, t=0.25, u1=128.2, u2=128.2 ** 2 + 4.0,
                         quality=TS_CORRECTED, low_light=True),
            _LightSample(abs_frame_id=11, t=0.275, u1=128.1, u2=128.1 ** 2 + 4.0,
                         low_light=True),
            _LightSample(abs_frame_id=12, t=0.3, u1=188.0, u2=188.0 ** 2 + 400.0,
                         quality=TS_CORRECTED),
        ],
    )
    frames = LinearInterpolation().correct_interval(interval, side="left", cam_id=0).frames
    assert [f.quality for f in frames] == [
        "ts_corrected;low_light", "low_light", "ts_corrected",
    ]
