"""Conversion math for console telemetry raw-ADC values.

The console firmware reports TEC thermistor / current / voltage as raw ADC
LSBs from a resistor-divider network. Bloodflow-app historically owned the
conversion (V_REF, R_1..R_3, R_s, and a Steinhart-Hart-ish lookup table for
the 10K3CG_R-T thermistor). It now lives in the SDK so every consumer
    (bloodflow-app live display, SDK telemetry displays, post-hoc analysis)
applies the same formula.

The RT lookup table is shipped as a CSV next to this module
(``models/10K3CG_R-T.csv``) and loaded lazily on first conversion.

These constants were measured empirically against the V1 console board:
V_REF is the ADC reference voltage (2.459 V, not the nominal 2.5 V), and
R_1/R_2/R_3 are the surrounding divider network resistors (designators
R221/R224/R225). R_s is the TEC current sense resistor (R217).
"""
from __future__ import annotations

import logging
import os
import threading
from typing import Optional

import numpy as np

logger = logging.getLogger("openmotion.sdk.console_telemetry_conversions")


# Resistor-network constants (V1 console board).
V_REF: float = 2.459
R_1:   float = 18_000.0   # R221
R_2:   float = 8_160.0    # R224
R_3:   float = 49_900.0   # R225
R_s:   float = 0.020      # R217 (TEC current sense)

_RT_TABLE_PATH = os.path.join(os.path.dirname(__file__), "models", "10K3CG_R-T.csv")

_rt_table: Optional[np.ndarray] = None
_rt_lock = threading.Lock()


def _load_rt_table() -> Optional[np.ndarray]:
    """Lazy-load the thermistor R-T lookup table. Returns None if missing."""
    global _rt_table
    with _rt_lock:
        if _rt_table is not None:
            return _rt_table
        if not os.path.exists(_RT_TABLE_PATH):
            logger.warning(
                "RT lookup table missing at %s — temperature conversions "
                "will return NaN", _RT_TABLE_PATH,
            )
            return None
        try:
            _rt_table = np.loadtxt(_RT_TABLE_PATH, delimiter=",", skiprows=1)
        except Exception as exc:
            logger.error("Failed to parse RT table %s: %s", _RT_TABLE_PATH, exc)
            return None
        return _rt_table


def tec_thermistor_voltage_to_celsius(v_raw: float) -> float:
    """Convert a TEC thermistor ADC reading (raw ADC LSBs as float) to °C.

    Returns NaN if the RT lookup table is unavailable.
    """
    table = _load_rt_table()
    if table is None:
        return float("nan")
    try:
        r_th = 1.0 / ((float(v_raw) / (V_REF / 2.0 * R_3)) - 1.0 / R_3 + 1.0 / R_1) - R_2
        # Table columns: [0]=°C, [1]=resistance. Reverse so interp sees ascending x.
        return float(np.interp(r_th, table[:, 1][::-1], table[:, 0][::-1]))
    except (ValueError, ZeroDivisionError):
        return float("nan")


def tec_current_to_amps(curr_raw: float) -> float:
    """Convert TEC current monitor ADC LSBs to amps."""
    try:
        return (float(curr_raw) - 0.5 * V_REF) / (25.0 * R_s)
    except (ValueError, TypeError):
        return float("nan")


def tec_voltage_to_volts(volt_raw: float) -> float:
    """Convert TEC voltage monitor ADC LSBs to volts."""
    try:
        return (float(volt_raw) - 0.5 * V_REF) * 4.0
    except (ValueError, TypeError):
        return float("nan")
