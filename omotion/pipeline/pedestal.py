"""Per-side, firmware-version-keyed pedestal lookup.

Replaces the legacy global mutated PEDESTAL_HEIGHT with a per-pipeline-instance
object. Supports dual-sensor systems with mixed firmware versions.
"""

from __future__ import annotations

from dataclasses import dataclass


def pedestal_for_fw(version: tuple[int, int, int]) -> float:
    """Return the pedestal height (in DN) for a given sensor firmware version."""
    if version <= (1, 5, 2):
        return 64.0
    return 128.0


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
