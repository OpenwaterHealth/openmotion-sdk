"""MotionConfig writes only what the console stores: a strict JSON object.

Console firmware with the strict-config check (console-fw fix/jsmn-strict)
refuses a config that is not strict JSON or does not fit its page. The SDK
raises before sending instead (no NaN / Infinity from json.dumps), and
strict_json_problem() explains a stored config that would not pass. No
hardware needed.
"""

import json

import pytest

from omotion.MotionConfig import MOTION_CFG_JSON_MAX_LEN, MotionConfig, strict_json_problem

pytestmark = pytest.mark.unit


def _cfg(data):
    m = MotionConfig()
    m.json_data = data
    return m


def test_valid_config_round_trips_compact():
    data = {"TEC_TRIP": 40, "EE_GAIN": 1.86, "cal": {"c_min": [0.1, 0.2]}, "flag": True, "x": None}
    wire = _cfg(data).to_wire_bytes()
    text = wire[16:].decode()
    assert json.loads(text) == data
    assert strict_json_problem(text) is None
    assert MotionConfig.from_wire_bytes(wire).json_data == data


@pytest.mark.parametrize(
    ("data", "path"),
    [
        ({"TEC_TRIP": float("nan")}, "TEC_TRIP"),
        ({"a": float("inf")}, "a"),
        ({"a": {"b": [1.0, float("-inf")]}}, "a.b[1]"),
    ],
)
def test_non_finite_values_raise_naming_the_key(data, path):
    with pytest.raises(ValueError, match=path.replace("[", r"\[").replace("]", r"\]")):
        _cfg(data).to_wire_bytes()


def test_oversized_config_raises():
    with pytest.raises(ValueError, match=str(MOTION_CFG_JSON_MAX_LEN)):
        _cfg({"pad": "x" * MOTION_CFG_JSON_MAX_LEN}).to_wire_bytes()


def test_largest_config_that_fits():
    filler = MOTION_CFG_JSON_MAX_LEN - len('{"pad":""}')
    wire = _cfg({"pad": "x" * filler}).to_wire_bytes()
    assert len(wire) - 16 == MOTION_CFG_JSON_MAX_LEN


@pytest.mark.parametrize(
    ("text", "fragment"),
    [
        ('{"a":NaN}', "NaN"),
        ('{"a":Infinity}', "Infinity"),
        ('{"a":-Infinity}', "-Infinity"),
        ('{"a":1e999}', "out of range"),
        ('{"a":bare}', "Expecting value"),
        ('{"a":1', "Expecting"),
        ("[1,2]", "not a JSON object"),
        ("{" + '"p":"' + "x" * MOTION_CFG_JSON_MAX_LEN + '"}', "more than"),
    ],
)
def test_strict_json_problem_explains_rejections(text, fragment):
    why = strict_json_problem(text)
    assert why is not None and fragment in why


def test_strict_json_problem_accepts_strict_objects():
    assert strict_json_problem('{"TEC_TRIP":40,"OPT_THRESH":7143,"EE_GAIN":1.86}') is None
    assert strict_json_problem("{}") is None
