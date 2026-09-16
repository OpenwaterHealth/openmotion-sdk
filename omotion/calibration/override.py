"""Operator override of the WI-00015 acceptance criteria.

Override mode is the deliberate, password-gated way for an engineer to write
a calibration that does not meet the factory pass/fail criteria (epic
OpenwaterHealth/openmotion-bloodflow-app#482). It exists for the test app's
Procedures pane and for headless bench runs; the clinical bloodflow-app never
enables it, so the engine default stays "one camera out of bounds means the
whole run fails and nothing is written" (#256).

Three invariants hold wherever override mode is honoured:

* It is off unless the script was started with ``--allow-override`` and the
  operator typed the override password, before any hardware is touched.
* Anything written outside the factory acceptance band asks the operator at
  the write point, against the measured numbers, and records the decision
  (operator, justification, timestamp) in the run evidence.
* An accepted override ends the run in a distinct terminal state
  (``ProcedureStatus.OVERRIDDEN`` / ``CalibrationOutcome.OVERRIDDEN``),
  prints ``Final result: OVERRIDE`` and exits with ``EXIT_OVERRIDE`` - never
  a PASS.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import hmac
import math
from types import MappingProxyType
from typing import Callable, Mapping

from .laser import (
    MAX_ACCEPTABLE_ENERGY_UJ,
    MIN_ACCEPTABLE_ENERGY_UJ,
    TARGET_ENERGY_UJ,
)

__all__ = [
    "EXIT_OVERRIDE",
    "OVERRIDE_MAX_ENERGY_UJ",
    "OVERRIDE_MIN_ENERGY_UJ",
    "OVERRIDE_PASSWORD_ATTEMPTS",
    "OVERRIDE_PASSWORD_PROMPT",
    "OverrideConsentFn",
    "OverrideDecision",
    "OverrideRequest",
    "OverrideSettings",
    "factory_band_description",
    "hash_override_password",
    "verify_override_password",
    "within_factory_band",
]

# Process exit code of a script whose run ended OVERRIDDEN. Distinct from 0
# (PASS), 1 (FAIL / CANCELED) and 2 (argparse usage error), so a host that
# does not understand overrides still treats the run as not passed.
EXIT_OVERRIDE = 3

# The only energies an operator may choose for an override band or target,
# in uJ (inclusive). Anything outside is refused, typed or flagged.
OVERRIDE_MIN_ENERGY_UJ = 10
OVERRIDE_MAX_ENERGY_UJ = 1000

# The override password is the engineering password shared with the
# bloodflow-app's engineering-mode unlock, stored here only as a salted
# SHA-256 digest. To rotate it, run
#   python -c "from omotion.calibration.override import hash_override_password as h; print(h('new password'))"
# and paste the printed digest below.
OVERRIDE_PASSWORD_SALT = "openmotion-wi15-override-v1"
OVERRIDE_PASSWORD_SHA256 = (
    "b4fbde6a83d8cdb5e4fb91bb24fddd8b7ec3bce1409b6bbb54e14a8d1e199e04"
)
OVERRIDE_PASSWORD_ATTEMPTS = 3
OVERRIDE_PASSWORD_PROMPT = "Override password: "


def hash_override_password(password: str) -> str:
    """Salted SHA-256 digest of ``password`` in the stored format."""
    return hashlib.sha256(
        f"{OVERRIDE_PASSWORD_SALT}:{password}".encode("utf-8")
    ).hexdigest()


def verify_override_password(candidate: object) -> bool:
    """True only for the exact override password (constant-time compare)."""
    if not isinstance(candidate, str):
        return False
    return hmac.compare_digest(
        hash_override_password(candidate), OVERRIDE_PASSWORD_SHA256
    )


def factory_band_description() -> str:
    return f"{MIN_ACCEPTABLE_ENERGY_UJ:g} to {MAX_ACCEPTABLE_ENERGY_UJ:g} uJ"


def within_factory_band(mean_uj: object) -> bool:
    """Whether a finite mean energy satisfies the inclusive factory window."""
    try:
        value = float(mean_uj)
    except (TypeError, ValueError):
        return False
    return (
        math.isfinite(value)
        and MIN_ACCEPTABLE_ENERGY_UJ <= value <= MAX_ACCEPTABLE_ENERGY_UJ
    )


@dataclass(frozen=True)
class OverrideSettings:
    """Acceptance criteria an authorised operator chose for one run.

    The defaults are the factory values, so ``OverrideSettings()`` is
    "override mode with the factory band": tuning behaves exactly as usual
    and the operator is only asked when a result would be written outside
    300-400 uJ. A custom band steers the tuning loops' candidate acceptance;
    it never turns an out-of-spec result into a PASS. Every energy must lie
    within ``OVERRIDE_MIN_ENERGY_UJ``-``OVERRIDE_MAX_ENERGY_UJ``.
    """

    minimum_energy_uj: float = MIN_ACCEPTABLE_ENERGY_UJ
    maximum_energy_uj: float = MAX_ACCEPTABLE_ENERGY_UJ
    target_energy_uj: float = TARGET_ENERGY_UJ
    authorized_by: str = ""
    authorized_at: datetime = field(
        default_factory=lambda: datetime.now(timezone.utc)
    )

    def __post_init__(self) -> None:
        values: dict[str, float] = {}
        for name in ("minimum_energy_uj", "maximum_energy_uj", "target_energy_uj"):
            raw = getattr(self, name)
            if isinstance(raw, bool) or not isinstance(raw, int | float):
                raise ValueError(f"{name} must be a number of uJ")
            value = float(raw)
            if not (
                math.isfinite(value)
                and OVERRIDE_MIN_ENERGY_UJ <= value <= OVERRIDE_MAX_ENERGY_UJ
            ):
                raise ValueError(
                    f"{name} must be between {OVERRIDE_MIN_ENERGY_UJ:g} and "
                    f"{OVERRIDE_MAX_ENERGY_UJ:g} uJ"
                )
            values[name] = value
            object.__setattr__(self, name, value)
        if not (
            values["minimum_energy_uj"]
            <= values["target_energy_uj"]
            <= values["maximum_energy_uj"]
        ):
            raise ValueError(
                "override energies must satisfy minimum <= target <= maximum"
            )
        if not isinstance(self.authorized_by, str):
            raise ValueError("authorized_by must be text")

    @property
    def uses_factory_band(self) -> bool:
        return (
            self.minimum_energy_uj == MIN_ACCEPTABLE_ENERGY_UJ
            and self.maximum_energy_uj == MAX_ACCEPTABLE_ENERGY_UJ
            and self.target_energy_uj == TARGET_ENERGY_UJ
        )

    def band_description(self) -> str:
        return f"{self.minimum_energy_uj:g} to {self.maximum_energy_uj:g} uJ"

    def describe(self) -> str:
        return f"{self.band_description()}, target {self.target_energy_uj:g} uJ"


@dataclass(frozen=True)
class OverrideRequest:
    """What the operator is asked to accept, with the numbers behind it."""

    criterion: str
    reason: str
    measured: Mapping[str, object] = field(default_factory=dict)
    accepted_band: str = ""
    factory_band: str = ""
    proposed_configuration: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "measured", MappingProxyType(dict(self.measured)))
        object.__setattr__(
            self,
            "proposed_configuration",
            MappingProxyType(dict(self.proposed_configuration)),
        )


@dataclass(frozen=True)
class OverrideDecision:
    """The operator's answer to an ``OverrideRequest`` - part of the evidence."""

    request: OverrideRequest
    accepted: bool
    operator: str
    justification: str | None = None
    decided_at: datetime = field(default_factory=lambda: datetime.now(timezone.utc))


OverrideConsentFn = Callable[[OverrideRequest], OverrideDecision]
