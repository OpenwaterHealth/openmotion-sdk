"""Per-side, firmware-version-keyed pedestal lookup.

Replaces the legacy global mutated PEDESTAL_HEIGHT with a per-pipeline-instance
object. Supports dual-sensor systems with mixed firmware versions.
"""

from __future__ import annotations

from dataclasses import dataclass

from omotion.config import ELECTRON_WELL_CAPACITY, HISTO_SIZE_WORDS


def pedestal_for_fw(version: tuple[int, int, int]) -> float:
    """Return the pedestal height (in DN) for a given sensor firmware version."""
    if version <= (1, 5, 2):
        return 64.0
    return 128.0


def adc_gain_for_pedestal(pedestal_height: float) -> float:
    """Sensor ADC gain in DN per electron, derived from the pedestal height.

    ADC_GAIN = (full-scale DN range above pedestal) / (electrons at full scale)
             = (HISTO_SIZE_WORDS − pedestal_height) / ELECTRON_WELL_CAPACITY

    The pedestal occupies the bottom of the 10-bit ADC range, so the usable
    range above it determines how many DN one electron at full scale produces.
    Legacy sensors (FW ≤ 1.5.2, pedestal = 64) give ≈ 0.0873 DN/e⁻; current
    sensors (pedestal = 128) give ≈ 0.0815 DN/e⁻.
    """
    return (float(HISTO_SIZE_WORDS) - float(pedestal_height)) / float(ELECTRON_WELL_CAPACITY)


@dataclass(frozen=True)
class SensorPedestals:
    """Per-side pedestal values to feed into PedestalSubtractionStage."""
    left:  float
    right: float

    @classmethod
    def from_sensors(cls, *, left, right) -> "SensorPedestals":
        return cls(
            left=pedestal_for_fw(left.version),
            right=pedestal_for_fw(right.version),
        )
