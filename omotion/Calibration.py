"""Calibration arrays for the BFI/BVI science pipeline.

The four arrays — ``C_min``, ``C_max``, ``I_min``, ``I_max`` — are stored
on the console's EEPROM JSON config under the ``"calibration"`` key. When
the SDK connects to a console it tries to load them; if they are missing
or fail validation it falls back to the defaults defined here.

Defaults are a verbatim copy of the values in
``openmotion-bloodflow-app/processing/visualize_bloodflow.py`` as of
2026-05-01 (the values the app has been shipping with).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Literal, Optional

import numpy as np

from omotion import _log_root
from omotion.config import CAMS_PER_MODULE, MODULES

logger = logging.getLogger(
    f"{_log_root}.Calibration" if _log_root else "Calibration"
)

# JSON keys
CALIBRATION_JSON_KEY = "calibration"
_C_MIN_KEY = "C_min"
_C_MAX_KEY = "C_max"
_I_MIN_KEY = "I_min"
_I_MAX_KEY = "I_max"
_ALL_ARRAY_KEYS = (_C_MIN_KEY, _C_MAX_KEY, _I_MIN_KEY, _I_MAX_KEY)

# Required shape — (modules, cams_per_module).
_EXPECTED_SHAPE = (MODULES, CAMS_PER_MODULE)

# Defaults — copied verbatim from
# openmotion-bloodflow-app/processing/visualize_bloodflow.py.
_DEFAULT_C_MIN = np.zeros(_EXPECTED_SHAPE, dtype=float)
_DEFAULT_C_MAX = np.array(
    [[0.4, 0.4, 0.45, 0.55, 0.55, 0.45, 0.4, 0.4],
     [0.4, 0.4, 0.45, 0.55, 0.55, 0.45, 0.4, 0.4]],
    dtype=float,
)
_DEFAULT_I_MIN = np.zeros(_EXPECTED_SHAPE, dtype=float)
_DEFAULT_I_MAX = np.array(
    [[150, 300, 300, 300, 300, 300, 300, 150],
     [150, 300, 300, 300, 300, 300, 300, 150]],
    dtype=float,
)

CalibrationSource = Literal["console", "default", "override"]


@dataclass(frozen=True)
class Calibration:
    """Resolved BFI/BVI calibration with provenance.

    All four arrays are shape ``(2, 8)`` float64. ``source`` tells callers
    where the values came from:

    - ``"console"``: parsed from the device EEPROM JSON.
    - ``"default"``: SDK-owned defaults (no console JSON, or invalid).
    - ``"override"``: supplied directly via
      :meth:`omotion.ScanWorkflow.set_realtime_calibration`.
    """

    c_min: np.ndarray
    c_max: np.ndarray
    i_min: np.ndarray
    i_max: np.ndarray
    source: CalibrationSource

    @classmethod
    def default(cls) -> "Calibration":
        """Return a fresh ``Calibration`` populated with SDK defaults.

        Each call returns independent array copies so mutating one
        instance never bleeds into another.
        """
        return cls(
            c_min=_DEFAULT_C_MIN.copy(),
            c_max=_DEFAULT_C_MAX.copy(),
            i_min=_DEFAULT_I_MIN.copy(),
            i_max=_DEFAULT_I_MAX.copy(),
            source="default",
        )


def parse_calibration(json_data: dict) -> Optional[Calibration]:
    """Parse a console JSON config dict into a Calibration or None.

    Returns ``None`` (not raises) when the calibration block is absent or
    invalid. Callers fall back to ``Calibration.default()``.
    """
    if not isinstance(json_data, dict):
        return None

    block = json_data.get(CALIBRATION_JSON_KEY)
    if block is None:
        return None
    if not isinstance(block, dict):
        logger.warning(
            "Console calibration invalid (calibration block is %s, not a dict); "
            "falling back to SDK defaults.",
            type(block).__name__,
        )
        return None

    arrays: dict[str, np.ndarray] = {}
    for key in _ALL_ARRAY_KEYS:
        if key not in block:
            logger.warning(
                "Console calibration invalid (missing key %s); "
                "falling back to SDK defaults.",
                key,
            )
            return None
        try:
            arr = np.asarray(block[key], dtype=float)
        except (TypeError, ValueError):
            logger.warning(
                "Console calibration invalid (%s is non-numeric); "
                "falling back to SDK defaults.",
                key,
            )
            return None
        if arr.shape != _EXPECTED_SHAPE:
            logger.warning(
                "Console calibration invalid (%s has shape %s, expected %s); "
                "falling back to SDK defaults.",
                key, arr.shape, _EXPECTED_SHAPE,
            )
            return None
        if not np.all(np.isfinite(arr)):
            logger.warning(
                "Console calibration invalid (%s contains NaN or inf); "
                "falling back to SDK defaults.",
                key,
            )
            return None
        arrays[key] = arr

    if not np.all(arrays[_C_MAX_KEY] > arrays[_C_MIN_KEY]):
        logger.warning(
            "Console calibration invalid (C_max not strictly greater than "
            "C_min element-wise); falling back to SDK defaults."
        )
        return None
    if not np.all(arrays[_I_MAX_KEY] > arrays[_I_MIN_KEY]):
        logger.warning(
            "Console calibration invalid (I_max not strictly greater than "
            "I_min element-wise); falling back to SDK defaults."
        )
        return None

    return Calibration(
        c_min=arrays[_C_MIN_KEY],
        c_max=arrays[_C_MAX_KEY],
        i_min=arrays[_I_MIN_KEY],
        i_max=arrays[_I_MAX_KEY],
        source="console",
    )


def serialize_calibration(c_min, c_max, i_min, i_max) -> dict:
    """Return a ``{"calibration": {...}}`` dict ready to merge into the
    console JSON. Validates inputs and raises ``ValueError`` on bad data.
    """
    arrays = {
        _C_MIN_KEY: c_min,
        _C_MAX_KEY: c_max,
        _I_MIN_KEY: i_min,
        _I_MAX_KEY: i_max,
    }
    typed: dict[str, np.ndarray] = {}
    for key, val in arrays.items():
        try:
            arr = np.asarray(val, dtype=float)
        except (TypeError, ValueError) as e:
            raise ValueError(f"{key} is not numeric: {e}") from e
        if arr.shape != _EXPECTED_SHAPE:
            raise ValueError(
                f"{key} has shape {arr.shape}; expected {_EXPECTED_SHAPE}"
            )
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"{key} contains non-finite values (NaN or inf)")
        typed[key] = arr

    if not np.all(typed[_C_MAX_KEY] > typed[_C_MIN_KEY]):
        raise ValueError(
            "C_max must be strictly greater than C_min element-wise (monotonic)"
        )
    if not np.all(typed[_I_MAX_KEY] > typed[_I_MIN_KEY]):
        raise ValueError(
            "I_max must be strictly greater than I_min element-wise (monotonic)"
        )

    return {
        CALIBRATION_JSON_KEY: {
            key: arr.tolist() for key, arr in typed.items()
        }
    }
