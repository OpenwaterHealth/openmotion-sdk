"""Multi-camera auto-exposure controller for Optimize Laser.

Port of the single-camera reference (bloodflow-app
docs/auto_exposure/exposure_control.py) to the device: per-camera gain,
one shared laser (pulse width and drive current), a configurable knob
order and starting point, and a status instead of a boolean. Pure Python:
it never touches hardware. A caller (the Optimize Laser workflow) applies
:attr:`ExposureController.settings`, captures frames, and passes them to
:meth:`ExposureController.update` until :attr:`ExposureController.done`.

Brightness model, per camera c::

    signal_c = k_c * gain_c * (pulse_width - truncation) * P(current)

Gain and pulse width scale the image linearly, so each moves with one
proportional jump. Laser output P(current) is zero below threshold, roughly
linear above it and flat above the knee, all unit-specific, so current is
set by a bracketed secant search (regula falsi, Illinois variant) on the
brightness normalized by gain x pulse width.

Each iteration:

1. Estimate every camera's dark-subtracted signal (fit or basic) and the
   brightness factor it needs: ``target / signal``, aiming slightly low out
   of a saturated frame, or a 16x cut for a frame too clipped to fit.
2. Walk the knob order (reversed when the laser has to brighten, so knobs
   come off their minimum in the opposite order they went to it). Each
   knob absorbs what its bounds allow. Gain moves per camera (or per
   mirrored row). A shared laser knob picks a factor every camera can
   still reach the target from with its own gain; only when no such factor
   exists does gain priority decide (the priority cameras stay reachable,
   and the rest get as close as they can). See ``_shared_factor``.
3. Re-solve every gain against the new shared settings: gain is the only
   per-camera lever, so every camera reaches the target with its gain, or
   alerts when it runs out.

The run exits when the priority cameras have the required number of
consecutive in-range evaluation points and every other camera is on target
or out of gain; or when nothing more can change.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Mapping, Sequence

import numpy as np

from .estimate import SignalEstimate, estimate_signal
from .settings import (
    Bounds,
    GainLadder,
    GainMode,
    GainPriority,
    HardwareLimits,
    Knob,
    Method,
    OptimizeLaserSettings,
    StartPoint,
    CAMERAS_PER_SENSOR,
    GAIN_CODE_SCALE,
    gain_row,
)

logger = logging.getLogger("openmotion.sdk.exposure")

Camera = tuple[int, int]    # (side: 0 left / 1 right, position 0..7 = camera 1..8)

NEAR_POSITIONS = frozenset({3, 4})      # cameras 4 & 5, nearest the laser
FAR_POSITIONS = frozenset({0, 7})       # cameras 1 & 8

FALLBACK_DROP = 16.0            # brightness cut for a frame too clipped to fit
THRESHOLD_GUESS_MA = 2500.0     # default lasing threshold (varies unit to unit)
KNEE_GUESS_MA = 4400.0          # low end of the knee range; output assumed flat above it
CURRENT_RESOLUTION_MA = 5.0     # stop the current search once the bracket is this narrow
MAX_CURRENT_STEPS = 6           # safety limit on current-search steps
DEFAULT_MAX_ITERATIONS = 20


class OptimizeStatus(str, Enum):
    """How a finished run ended. Every status but NOT_CONVERGED passes
    (the scan may go ahead); all but ON_TARGET should alert the user."""

    ON_TARGET = "on_target"                 # every camera in range
    PARTIAL = "partial"                     # priority cameras in range; others out of gain
    TOO_DIM_AT_MAX = "too_dim_at_max"       # every knob at its upper bound, still too dim
    TOO_BRIGHT_AT_MIN = "too_bright_at_min"  # every knob at its lower bound, still too bright
    SPREAD_TOO_WIDE = "spread_too_wide"     # priority cameras need opposite changes
    NOT_CONVERGED = "not_converged"         # iteration or current-search limit reached

    @property
    def passed(self) -> bool:
        return self is not OptimizeStatus.NOT_CONVERGED


class CameraStatus(str, Enum):
    ON_TARGET = "on_target"
    TOO_DIM = "too_dim"             # needs more light than its knobs can give
    TOO_BRIGHT = "too_bright"       # still too bright with its knobs at minimum
    ADJUSTING = "adjusting"
    NO_DATA = "no_data"


@dataclass(frozen=True)
class CameraObservation:
    """One camera's light frames from one iteration, and its dark level (DN)."""

    histograms: Sequence[np.ndarray]
    dark_dn: float = 0.0


@dataclass(frozen=True)
class ExposureSettings:
    """Settings for the next capture. ``gains`` are true multipliers,
    ``gain_codes`` the register values (gain x 16)."""

    gain_codes: Mapping[Camera, int]
    gains: Mapping[Camera, float]
    pulse_width_us: float
    current_ma: float


@dataclass(frozen=True)
class CameraRecord:
    gain: float
    gain_code: int
    dark_dn: float
    mean_raw: float | None
    signal: float | None
    kind: str | None
    sat_frac: float | None
    points: int
    points_in_range: int
    on_target: bool
    desired_factor: float | None
    status: CameraStatus


@dataclass(frozen=True)
class IterationRecord:
    """Everything one iteration measured and decided (the CSV row source)."""

    iteration: int
    settings: ExposureSettings          # what the frames were captured with
    cameras: Mapping[Camera, CameraRecord]
    next_settings: ExposureSettings     # what the next capture uses (== settings when done)
    done: bool
    status: OptimizeStatus | None


@dataclass(frozen=True)
class _Measurement:
    estimate: SignalEstimate
    dark: float
    points: int
    points_in_range: int
    on_target: bool
    factor: float                       # brightness factor this camera wants


class ExposureController:
    """Steps camera gains, laser pulse width and drive current toward the
    target mean. See the module docstring for the algorithm.

    ``cameras`` are the (side, position) pairs in the scan masks.
    ``initial`` is required for ``StartPoint.CURRENT``: the device's
    settings now. ``threshold_guess`` / ``knee_guess`` are this laser
    unit's threshold and knee if known; after a run that searched current,
    :meth:`learned_threshold` gives an updated threshold to store per unit.
    """

    def __init__(
        self,
        config: OptimizeLaserSettings,
        cameras: Iterable[Camera],
        *,
        limits: HardwareLimits | None = None,
        ladder: GainLadder | None = None,
        initial: ExposureSettings | None = None,
        threshold_guess: float = THRESHOLD_GUESS_MA,
        knee_guess: float = KNEE_GUESS_MA,
        max_iterations: int = DEFAULT_MAX_ITERATIONS,
    ):
        self.limits = limits or HardwareLimits()
        config.validate(self.limits)
        self.config = config
        self._method = Method(config.method).value
        self._order = tuple(Knob(k) for k in config.order)
        self._gain_bounds = tuple(config.gain_bounds)
        self._pw_bounds = config.pulse_width_bounds
        self._current_bounds = config.current_bounds
        self._target = float(config.target_mean)
        self._band = float(config.passing_range)
        # Residual factors inside this are "nothing left to do": half the
        # passing range, which also covers the 1/16 gain quantization.
        self._tol = max(0.01, 0.5 * self._band / self._target)
        self._ladder = ladder or GainLadder.nominal(self.limits.gain_min, self.limits.gain_max)
        self.threshold_guess = float(threshold_guess)
        self.knee_guess = float(knee_guess)
        self.max_iterations = int(max_iterations)

        cams = sorted(set(cameras))
        if not cams:
            raise ValueError("no cameras to optimize")
        for side, pos in cams:
            if side not in (0, 1) or not 0 <= pos < CAMERAS_PER_SENSOR:
                raise ValueError(f"camera {(side, pos)} is not a (side 0/1, position 0..7) pair")
        self._cams: tuple[Camera, ...] = tuple(cams)
        self.priority_cameras = self._select_priority(GainPriority(config.gain_priority))

        if GainMode(config.gain_mode) is GainMode.ROW:
            rows: dict[int, list[Camera]] = {}
            for c in self._cams:
                rows.setdefault(gain_row(c[1]), []).append(c)
            self._gain_groups = tuple(tuple(g) for _, g in sorted(rows.items()))
        else:
            self._gain_groups = tuple((c,) for c in self._cams)

        self._gains: dict[Camera, tuple[int, float]] = {}
        if StartPoint(config.start_point) is StartPoint.MAX:
            for c in self._cams:
                b = self._row_bounds(c)
                self._gains[c] = self._ladder.nearest(b.high, b)
            self._pw = self._pw_bounds.high
            self._current = self._current_bounds.high
        else:
            if initial is None:
                raise ValueError("start point 'current' needs the device's settings (initial=)")
            for group in self._gain_groups:
                want = _gmean(float(initial.gains[c]) for c in group)
                choice = self._ladder.nearest(want, self._row_bounds(group[0]))
                for c in group:
                    self._gains[c] = choice
            self._pw = self._quantize_pw(self._pw_bounds.clamp(float(initial.pulse_width_us)))
            self._current = self._quantize_current(float(initial.current_ma))

        # Current search state: bracket ends are (current mA, normalized
        # brightness or None if unmeasurable). lo is too dim, hi too bright.
        self._lo: tuple[float, float | None] | None = None
        self._hi: tuple[float, float | None] | None = None
        self._last_side: str | None = None
        self._current_steps = 0
        self._current_stepped = False
        self._current_exhausted = False
        self._history: list[tuple[float, float | None]] = []

        self.iteration = 0
        self.records: list[IterationRecord] = []
        self.done = False
        self.status: OptimizeStatus | None = None
        # The next settings a NOT_CONVERGED iteration had chosen, for resume().
        self._pending: tuple | None = None

    # ---- public state -----------------------------------------------------

    @property
    def cameras(self) -> tuple[Camera, ...]:
        return self._cams

    @property
    def frames_needed(self) -> int:
        """Light frames each iteration should deliver per camera."""
        return self.config.frames_per_iteration * self.config.in_range_points

    @property
    def settings(self) -> ExposureSettings:
        """Settings for the next capture."""
        return ExposureSettings(
            gain_codes={c: g[0] for c, g in self._gains.items()},
            gains={c: g[1] for c, g in self._gains.items()},
            pulse_width_us=self._pw,
            current_ma=self._current,
        )

    @property
    def camera_status(self) -> dict[Camera, CameraStatus]:
        if not self.records:
            return {c: CameraStatus.NO_DATA for c in self._cams}
        return {c: r.status for c, r in self.records[-1].cameras.items()}

    # ---- the step ---------------------------------------------------------

    def update(self, observations: Mapping[Camera, CameraObservation]) -> IterationRecord:
        """Take one iteration's frames (captured at :attr:`settings`) and
        choose the next settings. Returns the iteration's record."""
        if self.done:
            raise RuntimeError("the controller has finished; read .status")
        self.iteration += 1
        used = self.settings
        meas = {c: self._measure(observations.get(c)) for c in self._cams}
        known = [c for c in self._cams if meas[c] is not None]
        pri = [c for c in self.priority_cameras if meas[c] is not None] or known

        gains = dict(self._gains)
        pw, current = self._pw, self._current
        if pri:
            resid = {c: (meas[c].factor if meas[c] is not None else 1.0) for c in self._cams}
            # Walk the order forward when the shared laser has to dim, reversed
            # when it has to brighten, so knobs leave their minimum in the
            # opposite order they reached it.
            laser = self._shared_factor(pri, known, resid, gains, True)
            if laser == 1.0:
                laser = _gmean(meas[c].factor for c in pri)
            order = self._order if laser < 1.0 else tuple(reversed(self._order))
            norm = self._normalized_brightness(pri, meas, used)
            gains_moved = False
            for knob in order:
                if knob is Knob.GAIN:
                    self._gain_pass(gains, resid)
                    gains_moved = True
                    continue
                factor = self._shared_factor(pri, known, resid, gains, gains_moved)
                if knob is Knob.PULSE_WIDTH:
                    pw, applied = self._move_pulse_width(pw, factor)
                else:
                    goal = self._normalized_goal(pri, gains, pw)
                    current, applied = self._move_current(current, factor, norm, goal)
                if applied != 1.0:
                    for c in resid:
                        resid[c] /= applied
            self._gain_pass(gains, resid)

        changed = (pw != self._pw or current != self._current
                   or any(gains[c][0] != self._gains[c][0] for c in self._cams))
        status = self._decide(meas, gains, pw, current, changed)
        done = status is not None
        if not done:
            self._gains, self._pw, self._current = gains, pw, current
        elif status is OptimizeStatus.NOT_CONVERGED:
            # Kept for resume(); settings stay at what this frame used, which
            # is what the device holds (and what "accept" keeps).
            self._pending = (gains, pw, current)

        statuses = self._camera_statuses(meas, used, done)
        record = IterationRecord(
            iteration=self.iteration,
            settings=used,
            cameras={c: self._camera_record(c, meas[c], used, statuses[c]) for c in self._cams},
            next_settings=self.settings,
            done=done,
            status=status,
        )
        self.records.append(record)
        if done:
            self.done, self.status = True, status
            logger.info("Optimize Laser finished after %d iteration(s): %s",
                        self.iteration, status.value)
        else:
            nxt = record.next_settings
            logger.debug("Optimize Laser iteration %d: next pw %.0f us, current %.1f mA, "
                         "gains %s", self.iteration, nxt.pulse_width_us, nxt.current_ma,
                         sorted(set(nxt.gains.values())))
        return record

    def resume(self, extra_iterations: int) -> None:
        """Keep iterating after NOT_CONVERGED (the caller's choice, e.g. a
        passing range too narrow to settle in the iteration limit): allow
        ``extra_iterations`` more, continue from the settings the last
        iteration had chosen, and reopen the current search."""
        if self.status is not OptimizeStatus.NOT_CONVERGED:
            raise RuntimeError("only a run that did not converge can continue")
        extra = int(extra_iterations)
        if extra < 1:
            raise ValueError(f"extra iterations must be at least 1, got {extra_iterations!r}")
        self.max_iterations = self.iteration + extra
        if self._pending is not None:
            self._gains, self._pw, self._current = self._pending
        self._pending = None
        self._current_steps = 0
        self._current_exhausted = False
        self.done, self.status = False, None
        logger.info("Optimize Laser continuing: up to %d more iteration(s)", extra)

    def learned_threshold(self) -> float | None:
        """Estimate this unit's lasing threshold from the current search.

        Extends a line through the two most widely spaced usable points
        (lit, below the knee) down to zero brightness. The threshold doesn't
        depend on how bright the sample is, so it is the value worth storing
        per unit and passing back as ``threshold_guess``. None without two
        usable points.
        """
        lit = [(i, n) for i, n in self._history if n is not None and n > 0]
        if not lit:
            return None
        floor = 0.05 * max(n for _, n in lit)
        pts = sorted((i, n) for i, n in lit if n > floor and i < self.knee_guess)
        if len(pts) < 2:
            return None
        (i1, n1), (i2, n2) = pts[0], pts[-1]
        if n2 <= n1 or i2 <= i1:
            return None
        return i1 - n1 * (i2 - i1) / (n2 - n1)

    # ---- measurement ------------------------------------------------------

    def _measure(self, obs: CameraObservation | None) -> _Measurement | None:
        if obs is None or len(obs.histograms) == 0:
            return None
        hists = [np.asarray(h, dtype=np.float64) for h in obs.histograms]
        dark = float(obs.dark_dn)
        if not math.isfinite(dark):
            dark = 0.0
        est = estimate_signal(np.sum(hists, axis=0), dark, self._method)

        per_point = self.config.frames_per_iteration
        flags = [
            self._in_range(estimate_signal(np.sum(hists[i:i + per_point], axis=0), dark,
                                           self._method))
            for i in range(0, len(hists) - per_point + 1, per_point)
        ]
        needed = self.config.in_range_points
        on_target = len(flags) >= needed and all(flags[-needed:])

        if est.signal is None:
            factor = 1.0 / FALLBACK_DROP
        elif on_target:
            factor = 1.0
        elif not est.clean:
            # Out of saturation: aim slightly low, inside the band, so the
            # next frame is unsaturated.
            factor = (self._target - 0.5 * self._band) / est.signal
        else:
            factor = self._target / max(est.signal, 0.5)
        return _Measurement(est, dark, len(flags), sum(flags), on_target, factor)

    def _in_range(self, est: SignalEstimate) -> bool:
        return est.clean and abs(est.signal - self._target) <= self._band

    def _normalized_brightness(self, cams, meas, used: ExposureSettings) -> float | None:
        """Mean priority brightness per unit gain x emitted pulse width: the
        laser output the current search brackets. None if any priority
        camera was too clipped to estimate."""
        emitted = used.pulse_width_us - self.limits.pulse_truncation_us
        values = []
        for c in cams:
            sig = meas[c].estimate.signal
            if sig is None:
                return None
            values.append(max(sig, 0.0) / (used.gains[c] * emitted))
        return float(np.mean(values))

    def _normalized_goal(self, cams, gains: dict, pw: float) -> float:
        """The normalized brightness that puts the priority cameras on
        target at these gains and this pulse width (the current search's
        target; independent of the frame, so a dark frame still gives one)."""
        emitted = pw - self.limits.pulse_truncation_us
        return float(np.mean([self._target / (gains[c][1] * emitted) for c in cams]))

    # ---- knobs ------------------------------------------------------------

    def _row_bounds(self, cam: Camera) -> Bounds:
        return self._gain_bounds[gain_row(cam[1])]

    def _gain_pass(self, gains: dict, resid: dict) -> None:
        """Move each gain (or row of gains) by what its cameras still want."""
        for group in self._gain_groups:
            want = _gmean(gains[c][1] * resid[c] for c in group)
            code, mult = self._ladder.nearest(want, self._row_bounds(group[0]))
            for c in group:
                resid[c] *= gains[c][1] / mult
                gains[c] = (code, mult)

    def _shared_factor(self, pri: Sequence[Camera], known: Sequence[Camera], resid: dict,
                       gains: dict, gains_moved: bool) -> float:
        """The brightness factor a shared laser knob should apply.

        A camera can still reach the target with its own gain for shared
        factors F in [gain * resid / gain_max, gain * resid / gain_min].
        When those intervals overlap for every camera, everyone can be on
        target: pick the point of the overlap nearest the preference (no
        shared change if gains already had their turn in the knob order,
        else what the priority cameras ask for). When they don't, gain
        priority decides: keep the priority cameras feasible and, within
        that, get as close as possible to balancing all cameras. If even the
        priority cameras disagree, balance them.
        """
        prefer = 1.0 if gains_moved else _gmean(resid[c] for c in pri)
        all_lo, all_hi = self._feasible_factors(known, resid, gains)
        if all_lo <= all_hi:
            factor = min(max(prefer, all_lo), all_hi)
        else:
            pri_lo, pri_hi = self._feasible_factors(pri, resid, gains)
            balance = math.sqrt(all_lo * all_hi)
            if pri_lo <= pri_hi:
                factor = min(max(balance, pri_lo), pri_hi)
            else:
                factor = math.sqrt(pri_lo * pri_hi)
        return 1.0 if abs(math.log(factor)) <= math.log1p(self._tol) else factor

    def _feasible_factors(self, cams: Sequence[Camera], resid: dict,
                          gains: dict) -> tuple[float, float]:
        """Shared factors for which every camera in ``cams`` can reach the
        target with its gain (the intersection; low > high when empty)."""
        low, high = 0.0, math.inf
        for c in cams:
            need = gains[c][1] * resid[c]
            b = self._row_bounds(c)
            low = max(low, need / b.high)
            high = min(high, need / b.low)
        return low, high

    def _move_pulse_width(self, pw: float, factor: float) -> tuple[float, float]:
        if factor == 1.0 or self._pw_bounds.fixed:
            return pw, 1.0
        trunc = self.limits.pulse_truncation_us
        new = self._quantize_pw(self._pw_bounds.clamp((pw - trunc) * factor + trunc))
        if new == pw:
            return pw, 1.0
        return new, (new - trunc) / (pw - trunc)

    def _quantize_pw(self, pw: float) -> float:
        return self._pw_bounds.clamp(float(round(pw)))

    def _quantize_current(self, current: float) -> float:
        step = self.limits.current_step_ma
        return self._current_bounds.clamp(round(current / step) * step)

    def _move_current(self, current: float, factor: float, norm: float | None,
                      goal: float) -> tuple[float, float]:
        """One bracketed-secant step on the laser drive current.

        ``factor`` is what the priority cameras still need; ``norm`` the
        normalized brightness measured at ``current``; ``goal`` the
        normalized brightness that would put them on target. The measured
        point replaces the bracket end on its side whenever the previous
        iteration stepped the current. A dark frame is a valid too-dim
        point and a saturated one a valid too-bright point.
        """
        # n x factor is the same goal when the frame is lit; it is exact for
        # the aggregate, so prefer it. A dark frame (n = 0) carries no scale.
        target = norm * factor if norm else goal
        stepped, self._current_stepped = self._current_stepped, False
        if self._lo is not None and stepped:
            self._ingest_current_point(current, norm, factor, target)
        if (abs(math.log(factor)) <= math.log1p(self._tol) or self._current_exhausted
                or self._current_bounds.fixed):
            return current, 1.0

        if self._lo is None:
            self._history.append((current, norm))
            if factor < 1.0 or norm is None:
                # Too bright: the measured point is the bright end. Output is
                # flat above the knee, so at full current that brightness is
                # taken to hold at the knee, which skips the flat region.
                self._lo = (self.threshold_guess, 0.0)
                self._hi = (min(current, self.knee_guess), norm)
            else:
                self._lo = (current, norm)
                self._hi = None

        raw = self._next_current(target)
        new = self._quantize_current(raw)
        if new == current:
            return current, 1.0
        self._current_steps += 1
        self._current_stepped = True
        clipped = abs(new - raw) > self.limits.current_step_ma
        return new, (self._current_model_ratio(new, current) if clipped else factor)

    def _ingest_current_point(self, current: float, norm: float | None, factor: float,
                              target: float) -> None:
        self._history.append((current, norm))
        if norm is None or factor < 1.0:
            side, self._hi = "hi", (current, norm)
        else:
            side, self._lo = "lo", (current, max(norm, 0.0))
        # Illinois: the same end moving twice in a row means the other end
        # is stale (often a default guess); halving its distance from the
        # target pulls the next step toward it so the search can't stall.
        if side == self._last_side and target is not None:
            other = self._lo if side == "hi" else self._hi
            if other is not None and other[1] is not None:
                adjusted = (other[0], target + (other[1] - target) / 2.0)
                if side == "hi":
                    self._lo = adjusted
                else:
                    self._hi = adjusted
        self._last_side = side
        width = (self._hi[0] - self._lo[0]) if self._hi is not None else math.inf
        if abs(width) < CURRENT_RESOLUTION_MA or self._current_steps >= MAX_CURRENT_STEPS:
            self._current_exhausted = True

    def _next_current(self, target: float | None) -> float:
        """Where the line between the bracket ends crosses the target."""
        i_lo, n_lo = self._lo
        if self._hi is None:
            # No too-bright point yet: extend the line from the threshold
            # through the dim point, or go to full current if that fails.
            th = self.threshold_guess
            if target is None or not n_lo or i_lo <= th:
                return self._current_bounds.high
            return th + (i_lo - th) * target / n_lo
        i_hi, n_hi = self._hi
        if n_hi is None or target is None or n_hi == n_lo:
            return 0.5 * (i_lo + i_hi)          # bright end unmeasurable: bisect
        return i_lo + (target - n_lo) * (i_hi - i_lo) / (n_hi - n_lo)

    def _current_model_ratio(self, new: float, old: float) -> float:
        """Predicted output ratio between two currents (threshold/knee model),
        used only when a step was clipped by the current bounds."""
        def out(i):
            return max(min(i, self.knee_guess) - self.threshold_guess, 0.0)
        o_old, o_new = out(old), out(new)
        if o_old <= 0:
            return 1.0 if o_new <= 0 else FALLBACK_DROP
        return max(o_new / o_old, 1.0 / FALLBACK_DROP)

    # ---- outcome ----------------------------------------------------------

    def _gain_stuck(self, cam: Camera, wants_more: bool, gains=None) -> bool:
        nominal = (gains or self._gains)[cam][0] / GAIN_CODE_SCALE
        b = self._row_bounds(cam)
        return nominal >= b.high if wants_more else nominal <= b.low

    def _shared_at(self, high: bool, pw: float, current: float) -> bool:
        if high:
            return pw >= self._pw_bounds.high and current >= self._current_bounds.high
        return pw <= self._pw_bounds.low and current <= self._current_bounds.low

    def _decide(self, meas, gains, pw, current, changed) -> OptimizeStatus | None:
        pri = set(self.priority_cameras)
        pri_off = [c for c in self.priority_cameras
                   if meas[c] is None or not meas[c].on_target]
        others_unsettled = [
            c for c in self._cams
            if c not in pri and meas[c] is not None and not meas[c].on_target
            and not self._gain_stuck(c, meas[c].factor > 1.0, gains)
        ]
        if not pri_off and not others_unsettled:
            everyone = all(meas[c] is not None and meas[c].on_target for c in self._cams)
            return OptimizeStatus.ON_TARGET if everyone else OptimizeStatus.PARTIAL

        if not changed and not others_unsettled:
            if any(meas[c] is None for c in pri_off):
                return OptimizeStatus.NOT_CONVERGED
            dim = [c for c in pri_off if meas[c].factor > 1.0]
            bright = [c for c in pri_off if meas[c].factor < 1.0]
            if (dim and not bright and self._shared_at(True, pw, current)
                    and all(self._gain_stuck(c, True, gains) for c in dim)):
                return OptimizeStatus.TOO_DIM_AT_MAX
            if (bright and not dim and self._shared_at(False, pw, current)
                    and all(self._gain_stuck(c, False, gains) for c in bright)):
                return OptimizeStatus.TOO_BRIGHT_AT_MIN
            if (dim and bright and all(self._gain_stuck(c, True, gains) for c in dim)
                    and all(self._gain_stuck(c, False, gains) for c in bright)):
                return OptimizeStatus.SPREAD_TOO_WIDE
            if self._current_exhausted:
                return OptimizeStatus.NOT_CONVERGED
        if self.iteration >= self.max_iterations:
            return OptimizeStatus.NOT_CONVERGED
        return None

    def _camera_statuses(self, meas, used: ExposureSettings,
                         done: bool) -> dict[Camera, CameraStatus]:
        """A camera out of range is TOO_DIM / TOO_BRIGHT once its gain is at
        the bound it pushes against and nothing else will move for it: the
        shared laser is at that bound too, or (non-priority cameras, or a
        finished run) the shared laser belongs to someone else."""
        out = {}
        pri = set(self.priority_cameras)
        gains = {c: (used.gain_codes[c], used.gains[c]) for c in self._cams}
        for c in self._cams:
            m = meas[c]
            if m is None:
                out[c] = CameraStatus.NO_DATA
            elif m.on_target:
                out[c] = CameraStatus.ON_TARGET
            else:
                more = m.factor > 1.0
                stuck = self._gain_stuck(c, more, gains) and (
                    done or c not in pri
                    or self._shared_at(more, used.pulse_width_us, used.current_ma))
                if stuck:
                    out[c] = CameraStatus.TOO_DIM if more else CameraStatus.TOO_BRIGHT
                else:
                    out[c] = CameraStatus.ADJUSTING
        return out

    def _camera_record(self, cam, m: _Measurement | None, used: ExposureSettings,
                       status: CameraStatus) -> CameraRecord:
        if m is None:
            return CameraRecord(used.gains[cam], used.gain_codes[cam], 0.0, None, None, None,
                                None, 0, 0, False, None, status)
        e = m.estimate
        return CameraRecord(used.gains[cam], used.gain_codes[cam], m.dark, e.mean_raw, e.signal,
                            e.kind, e.sat_frac, m.points, m.points_in_range, m.on_target,
                            m.factor, status)

    def _select_priority(self, priority: GainPriority) -> tuple[Camera, ...]:
        positions = {GainPriority.NEAR: NEAR_POSITIONS,
                     GainPriority.FAR: FAR_POSITIONS}.get(priority)
        if positions is None:
            return self._cams
        chosen = tuple(c for c in self._cams if c[1] in positions)
        if not chosen:
            logger.warning("Optimize Laser: no %s-priority camera is in the scan masks; "
                           "driving the laser from all cameras", priority.value)
            return self._cams
        return chosen


def _gmean(values: Iterable[float]) -> float:
    vals = [float(v) for v in values]
    if not vals:
        return 1.0
    return math.exp(sum(math.log(max(v, 1e-12)) for v in vals) / len(vals))
