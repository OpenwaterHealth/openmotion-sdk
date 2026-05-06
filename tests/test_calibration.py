"""Unit tests for omotion.Calibration (no hardware required)."""

import numpy as np
import pytest

from omotion.Calibration import (
    Calibration,
    CALIBRATION_JSON_KEY,
    parse_calibration,
    serialize_calibration,
)


# Reference defaults — copied verbatim from
# openmotion-bloodflow-app/processing/visualize_bloodflow.py as of 2026-05-01.
# This is the golden test that pins the SDK defaults to the values the
# bloodflow app has been using.
_REF_C_MIN = np.zeros((2, 8), dtype=float)
_REF_C_MAX = np.array(
    [[0.4, 0.4, 0.45, 0.55, 0.55, 0.45, 0.4, 0.4],
     [0.4, 0.4, 0.45, 0.55, 0.55, 0.45, 0.4, 0.4]],
    dtype=float,
)
_REF_I_MIN = np.zeros((2, 8), dtype=float)
_REF_I_MAX = np.array(
    [[150, 300, 300, 300, 300, 300, 300, 150],
     [150, 300, 300, 300, 300, 300, 300, 150]],
    dtype=float,
)


def test_default_values_match_visualize_bloodflow_defaults():
    cal = Calibration.default()
    np.testing.assert_array_equal(cal.c_min, _REF_C_MIN)
    np.testing.assert_array_equal(cal.c_max, _REF_C_MAX)
    np.testing.assert_array_equal(cal.i_min, _REF_I_MIN)
    np.testing.assert_array_equal(cal.i_max, _REF_I_MAX)


def test_default_source_label():
    cal = Calibration.default()
    assert cal.source == "default"


def test_default_returns_independent_copies():
    a = Calibration.default()
    b = Calibration.default()
    a.c_max[0, 0] = 999.0
    # Mutating one default must not bleed into the next call.
    assert b.c_max[0, 0] == _REF_C_MAX[0, 0]


def test_default_arrays_have_correct_shape_and_dtype():
    cal = Calibration.default()
    for arr in (cal.c_min, cal.c_max, cal.i_min, cal.i_max):
        assert arr.shape == (2, 8)
        assert arr.dtype == np.float64


# ----- parse_calibration -----

def _valid_block():
    return {
        CALIBRATION_JSON_KEY: {
            "C_min": [[0.0]*8, [0.0]*8],
            "C_max": [[0.4]*8, [0.4]*8],
            "I_min": [[0.0]*8, [0.0]*8],
            "I_max": [[200.0]*8, [200.0]*8],
        }
    }


def test_parse_valid_returns_console_source():
    cal = parse_calibration(_valid_block())
    assert cal is not None
    assert cal.source == "console"
    np.testing.assert_array_equal(cal.c_max, np.full((2, 8), 0.4))
    np.testing.assert_array_equal(cal.i_max, np.full((2, 8), 200.0))


def test_parse_missing_block_returns_none():
    assert parse_calibration({}) is None
    assert parse_calibration({"some_other_key": 1}) is None


def test_parse_block_not_a_dict_returns_none():
    assert parse_calibration({CALIBRATION_JSON_KEY: "not-a-dict"}) is None
    assert parse_calibration({CALIBRATION_JSON_KEY: [1, 2, 3]}) is None


def test_parse_missing_one_subkey_returns_none(caplog):
    blk = _valid_block()
    del blk[CALIBRATION_JSON_KEY]["I_max"]
    with caplog.at_level("WARNING"):
        assert parse_calibration(blk) is None
    assert any("I_max" in rec.message for rec in caplog.records)


@pytest.mark.parametrize("bad_shape", [
    [[0.0]*8],                                # (1, 8)
    [[0.0]*7, [0.0]*7],                       # (2, 7)
    [[0.0]*8, [0.0]*8, [0.0]*8],              # (3, 8)
    0.0,                                       # scalar
    [0.0, 0.0],                                # 1-D
])
def test_parse_wrong_shape_returns_none(bad_shape):
    blk = _valid_block()
    blk[CALIBRATION_JSON_KEY]["C_min"] = bad_shape
    assert parse_calibration(blk) is None


def test_parse_non_numeric_returns_none():
    blk = _valid_block()
    blk[CALIBRATION_JSON_KEY]["C_min"] = [["abc"]*8, ["abc"]*8]
    assert parse_calibration(blk) is None


def test_parse_nan_returns_none():
    blk = _valid_block()
    blk[CALIBRATION_JSON_KEY]["C_min"][0][0] = float("nan")
    assert parse_calibration(blk) is None


def test_parse_inf_returns_none():
    blk = _valid_block()
    blk[CALIBRATION_JSON_KEY]["C_max"][0][0] = float("inf")
    assert parse_calibration(blk) is None


def test_parse_non_monotonic_c_returns_none():
    # C_max must be > C_min element-wise
    blk = _valid_block()
    blk[CALIBRATION_JSON_KEY]["C_max"][1][3] = 0.0  # equal to C_min there
    assert parse_calibration(blk) is None


def test_parse_non_monotonic_i_returns_none():
    blk = _valid_block()
    blk[CALIBRATION_JSON_KEY]["I_max"][0][0] = -1.0  # less than I_min
    assert parse_calibration(blk) is None


# ----- serialize_calibration -----

def test_serialize_round_trip():
    c_min = np.zeros((2, 8))
    c_max = np.full((2, 8), 0.5)
    i_min = np.zeros((2, 8))
    i_max = np.full((2, 8), 250.0)
    blob = serialize_calibration(c_min, c_max, i_min, i_max)
    assert CALIBRATION_JSON_KEY in blob
    cal = parse_calibration(blob)
    assert cal is not None
    np.testing.assert_array_equal(cal.c_min, c_min)
    np.testing.assert_array_equal(cal.c_max, c_max)
    np.testing.assert_array_equal(cal.i_min, i_min)
    np.testing.assert_array_equal(cal.i_max, i_max)


def test_serialize_emits_lists_not_ndarrays():
    blob = serialize_calibration(
        np.zeros((2, 8)), np.full((2, 8), 0.5),
        np.zeros((2, 8)), np.full((2, 8), 250.0),
    )
    inner = blob[CALIBRATION_JSON_KEY]
    for key in ("C_min", "C_max", "I_min", "I_max"):
        assert isinstance(inner[key], list)
        assert isinstance(inner[key][0], list)
        assert isinstance(inner[key][0][0], float)


def test_serialize_rejects_wrong_shape():
    with pytest.raises(ValueError, match="shape"):
        serialize_calibration(
            np.zeros((2, 7)), np.full((2, 7), 0.5),
            np.zeros((2, 7)), np.full((2, 7), 250.0),
        )


def test_serialize_rejects_nan():
    bad = np.zeros((2, 8))
    bad[0, 0] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        serialize_calibration(bad, np.full((2, 8), 0.5),
                              np.zeros((2, 8)), np.full((2, 8), 250.0))


def test_serialize_rejects_non_monotonic():
    with pytest.raises(ValueError, match="monotonic|greater"):
        serialize_calibration(
            np.zeros((2, 8)), np.zeros((2, 8)),  # C_max == C_min
            np.zeros((2, 8)), np.full((2, 8), 250.0),
        )
