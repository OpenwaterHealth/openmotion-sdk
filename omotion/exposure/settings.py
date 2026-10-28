"""Optimize Laser settings, hardware limits and the camera gain ladder.

:class:`OptimizeLaserSettings` is the Settings-menu form (bloodflow-app
docs/auto_exposure/HANDOFF.md section 1); :meth:`OptimizeLaserSettings.errors`
validates it against :class:`HardwareLimits` before anything reaches the
hardware. A nonsensical or non-monotonic bound is an error rather than a
clamp, because a silently-adjusted laser limit is exactly what this check
exists to stop.

:class:`HardwareLimits` carries what the connected hardware accepts. It is
an input rather than a constant because the exposure and the console decide
it. Console firmware with the 1.8.2-rc.3 safety envelope refuses the
extended limits outright (:mod:`.firmware`); older firmware accepts up to the
trigger's 2200 us. Two caps hold whatever a caller passes:
:data:`ABSOLUTE_MAX_PULSE_WIDTH_US` and :data:`ABSOLUTE_MAX_CURRENT_MA`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

# Never exceeded, whatever the settings or the limits say.
ABSOLUTE_MAX_PULSE_WIDTH_US = 2200.0
ABSOLUTE_MAX_CURRENT_MA = 5000.0

CAMERAS_PER_SENSOR = 8
GAIN_ROWS = 4               # mirrored pairs: 1&8, 2&7, 3&6, 4&5
GAIN_CODE_SCALE = 16        # analog gain register = gain x 16 (0x3508[4:0].0x3509[7:4])


class Knob(str, Enum):
    GAIN = "gain"
    PULSE_WIDTH = "pulse_width"
    CURRENT = "current"


class Method(str, Enum):
    FIT = "fit"             # Option 1
    BASIC = "basic"         # Option 2


class StartPoint(str, Enum):
    MAX = "max"             # every knob at its upper bound
    CURRENT = "current"     # whatever the device is set to now


class GainMode(str, Enum):
    INDIVIDUAL = "individual"
    ROW = "row"             # each mirrored pair shares one gain, across both modules


class GainPriority(str, Enum):
    NEAR = "near"           # cameras 1 & 8 drive the shared laser knobs
    FAR = "far"             # cameras 4 & 5
    NONE = "none"           # all cameras


DEFAULT_ORDER = (Knob.GAIN, Knob.PULSE_WIDTH, Knob.CURRENT)


def gain_row(position: int) -> int:
    """Row index 0..3 of a camera position 0..7 (row 0 = cameras 1 & 8)."""
    if not 0 <= position < CAMERAS_PER_SENSOR:
        raise ValueError(f"camera position {position} outside 0..{CAMERAS_PER_SENSOR - 1}")
    return min(position, CAMERAS_PER_SENSOR - 1 - position)


@dataclass(frozen=True)
class Bounds:
    """Inclusive lower/upper bound. Equal bounds fix the knob (it's skipped)."""

    low: float
    high: float

    @property
    def fixed(self) -> bool:
        return self.low == self.high

    def clamp(self, value: float) -> float:
        return min(max(value, self.low), self.high)


@dataclass(frozen=True)
class HardwareLimits:
    """What the connected hardware accepts, plus the brightness model's facts.

    Defaults describe console firmware without the 1.8.2 safety envelope (the
    only firmware :func:`.firmware.require_extended_limits` lets through) and
    the app's longest exposure (244 rows x 9 us).
    """

    gain_min: float = 1.0
    gain_max: float = 16.0
    pulse_width_min_us: float = 100.0       # HANDOFF floor; the TA hardware floor is 20 us
    pulse_width_max_us: float = ABSOLUTE_MAX_PULSE_WIDTH_US
    exposure_us: float = 2196.0
    laser_delay_us: float = 100.0           # trigger LaserPulseDelayUsec
    # The TA driver drops the drive 55 ticks early (driver_control.v), so the
    # emitted pulse is this much shorter than the commanded width.
    pulse_truncation_us: float = 17.3
    current_min_ma: float = 2500.0
    current_max_ma: float = ABSOLUTE_MAX_CURRENT_MA
    current_step_ma: float = 0.16           # TA_CURRENT_DRV LSB

    @property
    def effective_pulse_width_max_us(self) -> float:
        """Longest pulse that still lands inside the exposure window."""
        return min(ABSOLUTE_MAX_PULSE_WIDTH_US, self.pulse_width_max_us,
                   self.exposure_us - self.laser_delay_us)

    @property
    def effective_current_max_ma(self) -> float:
        return min(ABSOLUTE_MAX_CURRENT_MA, self.current_max_ma)


@dataclass(frozen=True)
class GainLadder:
    """The gain steps a camera accepts, and each one's true multiplier.

    ``codes`` are register values (gain x 16); ``multipliers`` the measured
    gain of each code, ascending. :meth:`nominal` assumes multiplier = code
    / 16 until a measured table exists (plan section 6).
    """

    codes: tuple[int, ...]
    multipliers: tuple[float, ...]

    def __post_init__(self):
        if not self.codes or len(self.codes) != len(self.multipliers):
            raise ValueError("GainLadder needs one multiplier per code")
        if any(b <= a for a, b in zip(self.multipliers, self.multipliers[1:])):
            raise ValueError("GainLadder multipliers must be strictly ascending")

    @classmethod
    def nominal(cls, gain_min: float = 1.0, gain_max: float = 16.0) -> GainLadder:
        lo = int(round(gain_min * GAIN_CODE_SCALE))
        hi = int(round(gain_max * GAIN_CODE_SCALE))
        codes = tuple(range(lo, hi + 1))
        return cls(codes, tuple(c / GAIN_CODE_SCALE for c in codes))

    def nearest(self, multiplier: float, bounds: Bounds) -> tuple[int, float]:
        """The code inside ``bounds`` (nominal gain) whose multiplier is
        closest to ``multiplier`` in ratio terms."""
        best = None
        for code, mult in zip(self.codes, self.multipliers):
            nominal = code / GAIN_CODE_SCALE
            if not bounds.low <= nominal <= bounds.high:
                continue
            # A non-positive request can't be matched in ratio terms: take the lowest gain.
            err = abs(math.log(mult / multiplier)) if multiplier > 0 else mult
            if best is None or err < best[0]:
                best = (err, code, mult)
        if best is None:
            raise ValueError(f"no gain code inside bounds {bounds}")
        return best[1], best[2]


def _default_gain_bounds() -> tuple[Bounds, ...]:
    return tuple(Bounds(1.0, 16.0) for _ in range(GAIN_ROWS))


@dataclass(frozen=True)
class OptimizeLaserSettings:
    """The Optimize Laser Settings menu (HANDOFF section 1).

    ``target_mean`` and ``passing_range`` are dark-subtracted DN; a mean
    within ``target_mean +/- passing_range`` counts as on target.
    ``gain_bounds`` has one entry per row: row 0 = cameras 1 & 8 ... row 3
    = cameras 4 & 5. ``frames_per_iteration`` frames are summed into one
    evaluation point, and ``in_range_points`` consecutive points must be in
    range before the run exits.
    """

    method: Method = Method.FIT
    target_mean: float = 200.0
    passing_range: float = 20.0
    start_point: StartPoint = StartPoint.MAX
    order: tuple[Knob, ...] = DEFAULT_ORDER
    gain_bounds: tuple[Bounds, ...] = field(default_factory=_default_gain_bounds)
    pulse_width_bounds: Bounds = Bounds(100.0, 2096.0)
    current_bounds: Bounds = Bounds(2500.0, 5000.0)
    gain_mode: GainMode = GainMode.INDIVIDUAL
    gain_priority: GainPriority = GainPriority.NEAR
    frames_per_iteration: int = 1
    in_range_points: int = 1

    def errors(self, limits: HardwareLimits | None = None) -> list[str]:
        """Every problem with these settings; empty when they are valid."""
        limits = limits or HardwareLimits()
        problems: list[str] = []

        for name, value, enum in (
            ("method", self.method, Method),
            ("start point", self.start_point, StartPoint),
            ("gain mode", self.gain_mode, GainMode),
            ("gain priority", self.gain_priority, GainPriority),
        ):
            if not _is_member(value, enum):
                problems.append(f"{name} {value!r} is not one of {[m.value for m in enum]}")

        order = tuple(self.order)
        if (len(order) != len(Knob)
                or not all(_is_member(k, Knob) for k in order)
                or {Knob(k) for k in order if _is_member(k, Knob)} != set(Knob)):
            problems.append(
                f"optimization order {order!r} must list gain, pulse_width and current once each")

        if not (_finite(self.target_mean) and _finite(self.passing_range)):
            problems.append("target mean and passing range must be finite numbers")
        else:
            if self.passing_range <= 0:
                problems.append(f"passing range {self.passing_range:g} DN must be positive")
            if self.target_mean - self.passing_range <= 0:
                problems.append(f"target {self.target_mean:g} - range "
                                f"{self.passing_range:g} DN must be above 0")
            if self.target_mean + self.passing_range >= 1023:
                problems.append(
                    f"target {self.target_mean:g} + range {self.passing_range:g} DN must be below "
                    "full scale (1023)")

        for name, value in (("frames per iteration", self.frames_per_iteration),
                            ("in-range points to exit", self.in_range_points)):
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                problems.append(f"{name} {value!r} must be a whole number of at least 1")

        gains = tuple(self.gain_bounds)
        if len(gains) != GAIN_ROWS:
            problems.append(f"gain bounds need {GAIN_ROWS} rows, got {len(gains)}")
        for row, b in enumerate(gains):
            label = f"row {row + 1} gain"
            if _bounds_problem(problems, label, b, limits.gain_min, limits.gain_max, "x"):
                continue
            for v in (b.low, b.high):
                if abs(v * GAIN_CODE_SCALE - round(v * GAIN_CODE_SCALE)) > 1e-9:
                    problems.append(f"{label} {v:g} is not a multiple of 1/{GAIN_CODE_SCALE}")

        _bounds_problem(problems, "pulse width", self.pulse_width_bounds,
                        limits.pulse_width_min_us, limits.effective_pulse_width_max_us, " us")
        _bounds_problem(problems, "laser drive current", self.current_bounds,
                        limits.current_min_ma, limits.effective_current_max_ma, " mA")
        return problems

    def validate(self, limits: HardwareLimits | None = None) -> None:
        """Raise ``ValueError`` naming every problem, if there are any."""
        problems = self.errors(limits)
        if problems:
            raise ValueError("Invalid Optimize Laser settings: " + "; ".join(problems))


def _is_member(value, enum) -> bool:
    try:
        enum(value)
    except ValueError:
        return False
    return True


def _finite(value) -> bool:
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


def _bounds_problem(problems: list[str], label: str, bounds, minimum: float,
                    maximum: float, unit: str) -> bool:
    """Append any problem with ``bounds``; True if it was unusable."""
    if not isinstance(bounds, Bounds) or not (_finite(bounds.low) and _finite(bounds.high)):
        problems.append(f"{label} bounds {bounds!r} must be two finite numbers")
        return True
    bad = False
    if bounds.low > bounds.high:
        problems.append(
            f"{label} lower bound {bounds.low:g}{unit} is above the upper bound "
            f"{bounds.high:g}{unit}")
        bad = True
    if bounds.low < minimum:
        problems.append(f"{label} lower bound {bounds.low:g}{unit} is below the hardware "
                        f"minimum {minimum:g}{unit}")
        bad = True
    if bounds.high > maximum:
        problems.append(f"{label} upper bound {bounds.high:g}{unit} is above the hardware "
                        f"maximum {maximum:g}{unit}")
        bad = True
    return bad
