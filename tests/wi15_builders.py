"""Shared evidence/dataclass builders for the WI-00015 calibration tests.

Imported with absolute imports (``from wi15_builders import valid_measurement``)
by the ``test_wi15_*`` files. Fakes live in ``wi15_fakes``; the console-script
harness lives in ``wi15_script_harness``.
"""

from dataclasses import replace
from datetime import timedelta

from omotion.calibration.laser import EnergyMeasurement, FpgaFirmwareRevision
from omotion.calibration.safety import PowerCycleEvidence
from omotion.calibration.single_sensor_laser import (
    OphirEvidenceApplicability,
    OphirSettingEvidence,
)


FPGA_REVISIONS = tuple(
    FpgaFirmwareRevision(controller, "1.2.3")
    for controller in ("TA", "SEED", "SAFETY_EE", "SAFETY_OPT")
)


def valid_measurement(**changes):
    """A passing 26-pulse 350 uJ Ophir observation, overridable per field."""
    values = {
        "n": 26,
        "discarded": 0,
        "mean_uj": 350.0,
        "stdev_uj": 10.0,
        "rate_hz": 40.0,
        "min_uj": 330.0,
        "max_uj": 370.0,
        "duration_s": 0.65,
    }
    values.update(changes)
    return EnergyMeasurement(**values)


def valid_measurement_for_mean(mean_uj: float, **changes) -> EnergyMeasurement:
    """A passing observation centered on ``mean_uj`` with min/max +- 10 uJ."""
    return replace(
        EnergyMeasurement(
            n=26,
            discarded=0,
            mean_uj=mean_uj,
            stdev_uj=10.0,
            rate_hz=40.0,
            min_uj=mean_uj - 10.0,
            max_uj=mean_uj + 10.0,
            duration_s=0.65,
        ),
        **changes,
    )


def valid_ophir_setting_evidence():
    """The exact seven-entry approved Ophir setting evidence tuple."""
    applicable = OphirEvidenceApplicability.APPLICABLE
    not_applicable = OphirEvidenceApplicability.NOT_APPLICABLE
    return (
        OphirSettingEvidence("measurement_mode", "Energy", "Energy", applicable, True),
        OphirSettingEvidence("range_mj", 2.0, 2.0, applicable, True),
        OphirSettingEvidence("wavelength_nm", 795, 795, applicable, True),
        OphirSettingEvidence("pulse_length_ms", 1.0, 1.0, applicable, True),
        OphirSettingEvidence(
            "threshold", "minimum_available", "minimum_available", applicable, True
        ),
        OphirSettingEvidence("display_averaging_s", 3, None, not_applicable, True),
        OphirSettingEvidence("graph_mode", "Statistics", None, not_applicable, True),
    )


def valid_safety_config(**changes):
    """The ten-key WI-00015 user configuration, overridable per key."""
    config = {
        "TA_PULSE_WIDTH": 500,
        "TA_CURRENT_DRV": 5000,
        "SEED_CW_GAIN": 140,
        "EE_PULSE_WIDTH_UL": 550,
        "EE_RATE_LL": 23125,
        "EE_DRIVE_CL": 9999,
        "OPT_PULSE_WIDTH_UL": 550,
        "OPT_RATE_LL": 23125,
        "OPT_DRIVE_CL": 9999,
        "TEC_TRIP": 40,
    }
    config.update(changes)
    return config


def valid_power_cycle(
    now,
    *,
    serial="C-1",
    restart_proof="same Motion handle disconnected and reconnected",
):
    """A fully observed 15-second power cycle anchored at ``now``."""
    return PowerCycleEvidence(
        off_requested_at=now,
        disconnect_observed_at=now + timedelta(seconds=1),
        on_allowed_at=now + timedelta(seconds=16),
        on_requested_at=now + timedelta(seconds=16),
        reconnect_observed_at=now + timedelta(seconds=20),
        off_duration_s=15.0,
        disconnect_observed=True,
        reconnect_observed=True,
        restart_proven=True,
        restart_proof=restart_proof,
        console_serial_before=serial,
        console_serial_after=serial,
    )
