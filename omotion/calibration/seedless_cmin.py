"""Seedless C_min measurement: the speckle contrast of an unseeded TA.

Requested by Brad (bahartl) through Ethan, 2026-11-24. Engineering
measurement on the static phantom; nothing is written to the device.

Physics: with the seed laser off, the TA emits broadband, low-coherence
light, so the speckle contrast it produces on a static phantom is the floor
the system can see (C_min). Contrast estimates depend on intensity (shot
noise, quantisation), so the floor is measured at the *same* image mean the
seeded system produces: the TA pulse is lengthened (not driven harder) until
the seedless image mean matches the nominal one.

Sequence (:class:`SeedlessCminProcedure`):

1. **Nominal** - a normal laser scan (seed on, standard pulse and exposure).
   The per-camera dark-corrected image mean is the target. The readout must
   look like a phantom (the factory mean and contrast floors) or the run
   stops before the seed is touched.
2. **Seedless search** - seed off, exposure 2295 us, then short bursts of
   ``burst_light_frames`` light frames. The first burst runs at the normal
   500 us pulse: it must come out clearly dimmer than nominal and with lower
   contrast, else the seed is not really off or the phantom is not in place,
   and the run stops. Each later burst lengthens the pulse (proportional
   first, then secant), never more than ``max_step_factor`` per step and
   never past ``MAX_TA_PULSE_WIDTH_US``, until the mean ratio is within
   ``tolerance`` of 1.
3. **C_min capture** - one second of seedless light frames at the matched
   pulse width. C_min is the average contrast (average of per-frame ratios,
   the project-wide convention, sdk#148): per camera, and over all cameras.

When the longest pulse cannot reach the nominal brightness (bench
2026-11-24: 0.75x at 2150 us, seeded and seedless both at 5 A), the TA drive current
is stepped up (Brad / Ethan 2026-11-24): at each current, bursts at
``round_fractions`` of the longest pulse; if a least-squares line through
them reaches 1.0x within ``max_ta_us``, the width is refined there (fit, then
secant); otherwise the current goes up by ``current_step_ma`` and the round
repeats, up to ``max_current_ma`` (5 A). Each current step must raise the
mean. Out of reach at 5 A fails, unless the opt-in **plateau check**
(Brad's review, 2026-11-24): seedless contrast is measured per camera at
``plateau_points`` increasing pulse widths, from the width that gives
``plateau_min_ratio`` of nominal brightness up to the longest pulse (the
last one is the 1 s C_min capture). If every camera's contrast stays within
``plateau_tolerance`` (max - min over mean) across those widths, contrast
does not depend on brightness there and C_min is reported from the capture
at the longest pulse. Otherwise the run fails and still reports what it
measured.

Every frame from every phase, the search history and the pulse width go into
the report (:mod:`omotion.calibration.seedless_cmin_report`).

All frames analysed are *light* frames from the science pipeline's corrected
stream (dark-subtracted mean, shot-noise-corrected std, contrast =
std/mean). The seedless register lifecycle is :class:`omotion.seedless.
SeedlessController`, owned by the bench adapter; this module is pure logic
over a bench interface, so it is testable without hardware.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, replace
from typing import Callable, Optional, Protocol, Sequence

from omotion.seedless import (
    MAX_TA_CURRENT_MA,
    MAX_TA_PULSE_WIDTH_US,
    MIN_TA_CURRENT_MA,
    MIN_TA_PULSE_WIDTH_US,
    TA_CURRENT_TICK_MA,
    ta_ticks_from_us,
    ta_us_from_ticks,
)

PHASE_NOMINAL = "nominal"
PHASE_SEARCH = "search"
PHASE_CMIN = "cmin"
PHASE_PLATEAU = "plateau"
PHASE_NOMINAL_AFTER = "nominal_after"
PHASE_WARMUP = "warmup"
MATCH_BY_CURRENT = "current"
MATCH_BY_WIDTH = "width"

#: How C_min was obtained: at matched brightness, or from the plateau check.
BASIS_MATCHED = "matched"
BASIS_PLATEAU = "plateau"

VERDICT_PASS = "PASS"
VERDICT_FAIL = "FAIL"


@dataclass(frozen=True)
class FrameRecord:
    """One camera's corrected light frame."""
    side: str
    cam_id: int            # 0-7
    abs_frame_id: int
    mean: float            # dark-subtracted
    std: float             # shot-noise corrected
    contrast: float        # std / mean


@dataclass(frozen=True)
class TaggedFrame:
    """A frame as it goes in the report: which phase/burst and pulse width."""
    phase: str
    burst: int             # 0 for nominal / cmin; 1.. for search bursts
    ta_pulse_width_us: float
    seed: str              # "on" / "off"
    side: str
    cam_id: int
    abs_frame_id: int
    mean: float
    std: float
    contrast: float
    ta_current_ma: Optional[float] = None


@dataclass(frozen=True)
class CameraStats:
    side: str
    cam_id: int
    n: int
    mean: float            # average of frame means
    mean_sd: float         # frame-to-frame SD of the mean
    contrast: float        # average of frame contrasts (avg-of-ratios)

    @property
    def label(self) -> str:
        return f"{'L' if self.side == 'left' else 'R'}{self.cam_id + 1}"


@dataclass(frozen=True)
class BurstResult:
    burst: int
    ta_pulse_width_us: float
    ratio: float           # mean over cameras of seedless/nominal mean
    contrast: float        # mean over cameras of average contrast
    cameras: tuple[CameraStats, ...]
    ta_current_ma: Optional[float] = None


@dataclass
class CminSettings:
    nominal_light_frames: int = 80          # 2 s at 40 Hz
    burst_light_frames: int = 10            # "quick bursts of 10 pulses"
    cmin_light_frames: int = 40             # 1 s at 40 Hz
    start_ta_us: float = 500.0              # the normal pulse: seed-off check
    max_ta_us: float = MAX_TA_PULSE_WIDTH_US
    tolerance: float = 0.03                 # |ratio - 1| to call it matched
    cmin_tolerance: float = 0.06            # the 1 s capture may drift this far
    max_bursts: int = 10
    max_step_factor: float = 2.0            # pulse may at most double per burst
    min_frame_fraction: float = 0.8         # frames a camera must deliver
    # Phantom check on the nominal readout: the factory measurement-
    # calibration floors (corner cameras 1/8 relaxed).
    nominal_min_mean: tuple[float, ...] = (40.0, 80.0, 80.0, 80.0,
                                           80.0, 80.0, 80.0, 40.0)
    nominal_min_contrast: float = 0.25
    # Phantom / seed-off check on the first seedless burst (at the normal
    # pulse width, so a seed that is still on would read ratio ~1). Bench
    # 2026-07-17: 2 ms unseeded ~0.975x nominal, 500 us -> 2 ms gain 3.70x,
    # so ~0.26x expected; contrast fell from K~0.40 to K~0.09.
    first_burst_min_ratio: float = 0.05
    first_burst_max_ratio: float = 0.75
    first_burst_max_contrast_fraction: float = 0.75
    # Each lengthening must raise the mean by at least this (relative).
    min_ratio_gain: float = 0.02
    # Plateau check when the nominal brightness is out of reach (see the
    # module docstring). Brad's review 2026-11-24: per camera, 5 %.
    plateau_check: bool = False             # opt-in (--plateau-check)
    plateau_min_ratio: float = 0.5          # lowest brightness it looks at
    plateau_points: int = 3                 # pulse widths, the C_min capture included
    plateau_tolerance: float = 0.05         # per-camera (max - min) / mean
    # Drive-current stepping (Brad / Ethan 2026-11-24): at each current, 3
    # bursts at progressive pulse widths; if a line through them says the
    # nominal brightness is out of reach within max_ta_us, raise the TA
    # drive current by current_step_ma and go again, up to max_current_ma.
    current_stepping: bool = True
    current_step_ma: float = 100.0
    max_current_ma: float = MAX_TA_CURRENT_MA
    round_fractions: tuple[float, ...] = (0.5, 0.75, 1.0)   # of max_ta_us
    min_current_gain: float = 0.005         # a current step must raise the mean
    max_total_bursts: int = 80
    # Confidence in the per-camera C_min (Ethan 2026-11-24):
    cmin_repeats: int = 3                   # 1 s captures at the matched setting
    camera_tolerance: float = 0.05          # each camera's brightness vs its nominal
    repeat_spread_max: float = 0.02         # per camera, (max - min) / mean
    correction_min_ratio: float = 0.6       # bursts used for the 1.0x correction
    correction_flag: float = 0.01           # flag a correction bigger than this
    nominal_recheck: bool = True            # seeded nominal again at the end
    nominal_recheck_frames: int = 40
    nominal_drift_max: float = 0.02         # per camera, after vs before
    # Bench 2026-11-24: C_min moved ~8 % between a match at 4300 mA/2150 us
    # and one at 4400 mA/2059 us. Seedless contrast depends on the pulse
    # width, so the match holds the width at the ceiling and tunes the
    # current ("current"); "width" is the earlier width refinement.
    match_by: str = MATCH_BY_CURRENT
    current_match_tolerance: float = 0.01   # 1.0x +- this, at the full pulse
    max_current_step_ma: float = 300.0      # per refining step
    # Seeded warm-up before the nominal: the cold run's seeded image rose
    # 0.6-2.0 % during the run. Chunks until two in a row agree.
    warmup: bool = True
    warmup_chunk_frames: int = 800          # 20 s per chunk
    warmup_stable: float = 0.002            # chunk-to-chunk, per camera
    warmup_min_chunks: int = 2
    warmup_max_chunks: int = 15             # 5 min, then FAIL

    def validate(self) -> None:
        if not (MIN_TA_PULSE_WIDTH_US <= self.start_ta_us <= self.max_ta_us
                <= MAX_TA_PULSE_WIDTH_US):
            raise ValueError(
                "need MIN <= start_ta_us <= max_ta_us <= "
                f"{MAX_TA_PULSE_WIDTH_US:g} us")
        for name in ("nominal_light_frames", "burst_light_frames",
                     "cmin_light_frames", "max_bursts"):
            if int(getattr(self, name)) < 1:
                raise ValueError(f"{name} must be >= 1")
        if not (0 < self.tolerance < self.cmin_tolerance < 1):
            raise ValueError("need 0 < tolerance < cmin_tolerance < 1")
        if self.max_step_factor <= 1:
            raise ValueError("max_step_factor must be > 1")
        if not (0 < self.current_step_ma <= 1000):
            raise ValueError("current_step_ma must be in (0, 1000]")
        if not (0 < self.max_current_ma <= MAX_TA_CURRENT_MA + 1e-6):
            raise ValueError(f"max_current_ma must be <= {MAX_TA_CURRENT_MA:g}")
        if (len(self.round_fractions) < 2
                or list(self.round_fractions) != sorted(set(self.round_fractions))
                or not all(0 < f <= 1 for f in self.round_fractions)):
            raise ValueError("round_fractions: >= 2 increasing values in (0, 1]")
        if int(self.plateau_points) < 2:
            raise ValueError("plateau_points must be >= 2")
        if not (0 < self.plateau_min_ratio < 1):
            raise ValueError("plateau_min_ratio must be between 0 and 1")
        if not (0 < self.plateau_tolerance < 1):
            raise ValueError("plateau_tolerance must be between 0 and 1")
        if int(self.cmin_repeats) < 2:
            raise ValueError("cmin_repeats must be >= 2 (one capture has no spread)")
        if int(self.nominal_recheck_frames) < 1:
            raise ValueError("nominal_recheck_frames must be >= 1")
        for name in ("camera_tolerance", "repeat_spread_max", "correction_flag",
                     "nominal_drift_max"):
            if not (0 < getattr(self, name) < 1):
                raise ValueError(f"{name} must be between 0 and 1")
        if not (0 < self.correction_min_ratio < 1):
            raise ValueError("correction_min_ratio must be between 0 and 1")
        if self.match_by not in (MATCH_BY_CURRENT, MATCH_BY_WIDTH):
            raise ValueError("match_by must be 'current' or 'width'")
        if not (0 < self.current_match_tolerance <= self.tolerance):
            raise ValueError("need 0 < current_match_tolerance <= tolerance")
        if not (0 < self.max_current_step_ma <= 1000):
            raise ValueError("max_current_step_ma must be in (0, 1000]")
        if not (2 <= int(self.warmup_min_chunks) <= int(self.warmup_max_chunks)):
            raise ValueError("need 2 <= warmup_min_chunks <= warmup_max_chunks")
        if int(self.warmup_chunk_frames) < 1 or not (0 < self.warmup_stable < 1):
            raise ValueError("bad warm-up settings")


@dataclass(frozen=True)
class CameraCmin:
    """One camera's C_min and the evidence for it."""
    side: str
    cam_id: int
    nominal_mean: float
    nominal_contrast: float
    ratio: float                 # capture mean / nominal mean (all repeats)
    cmin_raw: float              # average contrast over the repeats
    repeats: tuple[float, ...]   # each repeat's average contrast
    repeat_spread: float         # (max - min) / mean of the repeats
    slope: Optional[float]       # d contrast / d ratio near the match
    cmin: float                  # corrected to ratio 1.0 (raw if no slope)
    correction: float            # cmin / cmin_raw - 1
    nominal_drift: Optional[float] = None   # nominal after / before - 1

    @property
    def label(self) -> str:
        return f"{'L' if self.side == 'left' else 'R'}{self.cam_id + 1}"

    @property
    def cmin_over_nominal(self) -> float:
        return self.cmin / self.nominal_contrast if self.nominal_contrast else math.nan


def contrast_slope(points: Sequence[tuple[float, float]]) -> Optional[float]:
    """Least-squares d contrast / d ratio, or None without a usable spread
    of brightness (at least 2 points spanning 5 %)."""
    if len(points) < 2:
        return None
    xs = [p[0] for p in points]
    if max(xs) - min(xs) < 0.05:
        return None
    mx = sum(xs) / len(xs)
    my = sum(p[1] for p in points) / len(points)
    den = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in points) / den


class CminBench(Protocol):
    """What the procedure needs from the hardware."""

    def capture(self, light_frames: int, *, seedless: bool) -> list[FrameRecord]:
        """Run one scan and return its first ``light_frames`` corrected light
        frames per active camera. ``seedless`` selects the seedless trigger
        override; the registers must already be in that state."""

    def enter_seedless(self, ta_pulse_width_us: float) -> None:
        """Seed off, safety limits widened, exposure 2295 us, TA width set.
        Raises on failure (after rolling back what it changed)."""

    def set_ta_pulse_width_us(self, ta_pulse_width_us: float) -> None:
        """Between bursts only. Raises on failure."""

    def ta_current_ma(self) -> float:
        """The TA drive current now set (after enter_seedless)."""

    def set_ta_current_ma(self, ta_current_ma: float) -> None:
        """Between bursts only. Raises on failure."""

    def exit_seedless(self) -> Optional[str]:
        """Restore everything; ``None`` when verified, else the problem."""


class ProcedureStop(Exception):
    """The run cannot continue; the message is for the operator."""


class OutOfReach(ProcedureStop):
    """The longest pulse leaves the seedless image dimmer than nominal."""


@dataclass
class CminResult:
    verdict: str = VERDICT_FAIL
    problem: str = ""
    nominal: tuple[CameraStats, ...] = ()
    bursts: list[BurstResult] = field(default_factory=list)
    matched_ta_us: Optional[float] = None
    cmin_capture: Optional[BurstResult] = None
    cmin_per_camera: dict[str, float] = field(default_factory=dict)
    cmin: Optional[float] = None
    frames: list[TaggedFrame] = field(default_factory=list)
    restore_problem: Optional[str] = None
    seedless_entered: bool = False
    basis: Optional[str] = None             # BASIS_MATCHED / BASIS_PLATEAU
    start_current_ma: Optional[float] = None    # the unit's own drive current
    matched_current_ma: Optional[float] = None
    currents_tried_ma: list[float] = field(default_factory=list)
    plateau: list[BurstResult] = field(default_factory=list)  # below the capture
    plateau_spread: dict[str, float] = field(default_factory=dict)
    cmin_captures: list[BurstResult] = field(default_factory=list)
    cameras: list[CameraCmin] = field(default_factory=list)
    nominal_after: tuple[CameraStats, ...] = ()
    seedless_exited: bool = False
    warmup: list[float] = field(default_factory=list)   # chunk change, worst cam
    match_by: str = MATCH_BY_CURRENT
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        def stats(cams):
            return [dict(asdict(c), label=c.label) for c in cams]

        def burst(b: Optional[BurstResult]):
            if b is None:
                return None
            return {"burst": b.burst, "ta_pulse_width_us": b.ta_pulse_width_us,
                    "ta_current_ma": b.ta_current_ma,
                    "ratio": b.ratio, "contrast": b.contrast,
                    "cameras": stats(b.cameras)}

        return {
            "verdict": self.verdict,
            "problem": self.problem,
            "c_min": self.cmin,
            "c_min_per_camera": dict(self.cmin_per_camera),
            "c_min_basis": self.basis,
            "matched_ta_pulse_width_us": self.matched_ta_us,
            "matched_ta_current_ma": self.matched_current_ma,
            "start_ta_current_ma": self.start_current_ma,
            "ta_currents_tried_ma": list(self.currents_tried_ma),
            "plateau": [burst(b) for b in self.plateau],
            "cmin_repeats": [burst(b) for b in self.cmin_captures],
            "cameras": [dict(asdict(c), label=c.label,
                             cmin_over_nominal=c.cmin_over_nominal)
                        for c in self.cameras],
            "nominal_after": stats(self.nominal_after),
            "notes": list(self.notes),
            "warmup_chunk_change": list(self.warmup),
            "match_by": self.match_by,
            "plateau_contrast_spread": dict(self.plateau_spread),
            "nominal": stats(self.nominal),
            "search": [burst(b) for b in self.bursts],
            "cmin_capture": burst(self.cmin_capture),
            "seedless_entered": self.seedless_entered,
            "restore_problem": self.restore_problem,
            "frame_count": len(self.frames),
        }


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def quantize_ta_us(us: float) -> float:
    """Round to what TA_PULSE_WIDTH can hold (0.32 us ticks)."""
    return ta_us_from_ticks(ta_ticks_from_us(us))


def camera_stats(frames: Sequence[FrameRecord]) -> tuple[CameraStats, ...]:
    """Per-camera averages, ordered left 1..8 then right 1..8. Frames with a
    non-finite mean or contrast are kept out of the averages (a real frame
    with no light has a NaN contrast: the per-camera count then shows it)."""
    groups: dict[tuple[str, int], list[FrameRecord]] = {}
    for f in frames:
        groups.setdefault((f.side, f.cam_id), []).append(f)
    out = []
    for side in ("left", "right"):
        for cam in range(8):
            group = [f for f in groups.get((side, cam), [])
                     if math.isfinite(f.mean) and math.isfinite(f.contrast)]
            if not group:
                if (side, cam) in groups:
                    out.append(CameraStats(side, cam, 0, math.nan, math.nan,
                                           math.nan))
                continue
            means = [f.mean for f in group]
            avg = sum(means) / len(means)
            sd = (math.sqrt(sum((m - avg) ** 2 for m in means) / (len(means) - 1))
                  if len(means) > 1 else 0.0)
            out.append(CameraStats(
                side, cam, len(group), avg, sd,
                sum(f.contrast for f in group) / len(group)))
    return tuple(out)


def _check_frame_counts(cams: Sequence[CameraStats], expected_cams: set,
                        requested: int, fraction: float, what: str) -> None:
    need = max(1, math.ceil(requested * fraction))
    seen = {(c.side, c.cam_id): c for c in cams}
    short = []
    for key in sorted(expected_cams):
        c = seen.get(key)
        n = c.n if c is not None else 0
        if n < need:
            label = f"{'L' if key[0] == 'left' else 'R'}{key[1] + 1}"
            short.append(f"{label} ({n} of {requested})")
    if short:
        raise ProcedureStop(
            f"the {what} delivered too few usable light frames: "
            + ", ".join(short))


def compare_to_nominal(burst: int, ta_us: float,
                       cams: Sequence[CameraStats],
                       nominal: Sequence[CameraStats]) -> BurstResult:
    """Mean ratio and contrast over the cameras both captures share."""
    nom = {(c.side, c.cam_id): c for c in nominal}
    ratios, contrasts = [], []
    for c in cams:
        n = nom.get((c.side, c.cam_id))
        if n is None or not (n.mean > 0) or not math.isfinite(c.mean):
            continue
        ratios.append(c.mean / n.mean)
        contrasts.append(c.contrast)
    if not ratios:
        raise ProcedureStop("no camera has both a nominal and a seedless reading")
    return BurstResult(burst, ta_us, sum(ratios) / len(ratios),
                       sum(contrasts) / len(contrasts), tuple(cams))


def check_nominal(cams: Sequence[CameraStats], settings: CminSettings) -> None:
    """Phantom check on the nominal (seeded) readout."""
    problems = []
    for c in cams:
        floor = settings.nominal_min_mean[c.cam_id]
        if not (c.mean >= floor):
            problems.append(f"{c.label} mean {c.mean:.1f} < {floor:g}")
        if not (c.contrast >= settings.nominal_min_contrast):
            problems.append(f"{c.label} contrast {c.contrast:.3f} < "
                            f"{settings.nominal_min_contrast:g}")
    if problems:
        raise ProcedureStop(
            "the normal (seeded) readout does not look like the phantom: "
            + "; ".join(problems))


def check_first_burst(first: BurstResult, nominal: Sequence[CameraStats],
                      settings: CminSettings) -> None:
    """Seed-off / phantom check on the first seedless burst."""
    nominal_contrast = sum(c.contrast for c in nominal) / len(nominal)
    if first.ratio > settings.first_burst_max_ratio:
        raise ProcedureStop(
            f"the first seedless burst at {first.ta_pulse_width_us:.0f} us is "
            f"{first.ratio:.2f}x the normal brightness (expected well below "
            f"{settings.first_burst_max_ratio:g}x): the seed may not be off")
    if first.ratio < settings.first_burst_min_ratio:
        raise ProcedureStop(
            f"the first seedless burst is only {first.ratio:.3f}x the normal "
            f"brightness (expected above {settings.first_burst_min_ratio:g}x): "
            "the TA may be shut down or the phantom moved")
    limit = settings.first_burst_max_contrast_fraction * nominal_contrast
    if not (first.contrast <= limit):
        raise ProcedureStop(
            f"the first seedless burst has contrast {first.contrast:.3f}, not "
            f"clearly below the normal {nominal_contrast:.3f} (limit "
            f"{limit:.3f}): this does not look like unseeded light")


def next_ta_us(history: Sequence[BurstResult], settings: CminSettings) -> float:
    """Next pulse width aiming at ratio 1. Proportional from one point,
    secant from the last two; step-limited and clamped; tick-quantized.
    Raises ProcedureStop when the mean does not respond or the target is out
    of reach."""
    cur = history[-1]
    w, r = cur.ta_pulse_width_us, cur.ratio
    if len(history) >= 2:
        prev = history[-2]
        dw = w - prev.ta_pulse_width_us
        if dw > 0 and (r - prev.ratio) < settings.min_ratio_gain * prev.ratio:
            raise ProcedureStop(
                f"lengthening the pulse from {prev.ta_pulse_width_us:.0f} to "
                f"{w:.0f} us did not raise the image mean ({prev.ratio:.3f}x "
                f"-> {r:.3f}x): the pulse is not reaching the cameras as set")
    if len(history) >= 2 and history[-1].ta_pulse_width_us != history[-2].ta_pulse_width_us:
        prev = history[-2]
        slope = (r - prev.ratio) / (w - prev.ta_pulse_width_us)
        target = w + (1.0 - r) / slope if slope > 0 else w * (1.0 / r)
    else:
        target = w / r if r > 0 else w * settings.max_step_factor
    lo = max(MIN_TA_PULSE_WIDTH_US, w / settings.max_step_factor)
    hi = min(settings.max_ta_us, w * settings.max_step_factor)
    nxt = quantize_ta_us(min(max(target, lo), hi))
    if nxt > hi:   # rounding to a tick must not cross the ceiling
        nxt = ta_us_from_ticks(math.floor(hi / 0.32))
    if abs(nxt - w) < 0.32 / 2:
        if r < 1.0:
            raise OutOfReach(
                f"the normal brightness is out of reach: at the "
                f"{w:.0f} us limit the seedless mean is only {r:.3f}x nominal")
        raise ProcedureStop(
            f"the pulse cannot get short enough: at {w:.0f} us the seedless "
            f"mean is already {r:.3f}x nominal")
    return nxt


def width_for_ratio(history: Sequence[BurstResult], ratio: float,
                    lo: float, hi: float) -> float:
    """Pulse width that gives ``ratio`` of nominal brightness, from a
    least-squares line through the bursts, clamped to [lo, hi]."""
    xs = [b.ta_pulse_width_us for b in history]
    ys = [b.ratio for b in history]
    if len(set(xs)) < 2:
        return lo
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    slope = (sum((x - mx) * (y - my) for x, y in zip(xs, ys))
             / sum((x - mx) ** 2 for x in xs))
    if slope <= 0:
        return lo
    return min(max(mx + (ratio - my) / slope, lo), hi)


def extrapolate_width(history: Sequence[BurstResult], ratio: float = 1.0) -> float:
    """Pulse width at which a least-squares line through ``history`` reaches
    ``ratio`` (unclamped; ``inf`` when the line does not rise)."""
    xs = [b.ta_pulse_width_us for b in history]
    ys = [b.ratio for b in history]
    if len(set(xs)) < 2:
        return math.inf
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    slope = (sum((x - mx) * (y - my) for x, y in zip(xs, ys))
             / sum((x - mx) ** 2 for x in xs))
    if slope <= 0:
        return math.inf
    return mx + (ratio - my) / slope


def plateau_widths(lo: float, hi: float, points: int) -> list[float]:
    """``points`` evenly spaced, tick-quantized widths from lo to hi; the
    last is exactly ``hi`` (the C_min capture)."""
    step = (hi - lo) / (points - 1)
    out = [quantize_ta_us(lo + i * step) for i in range(points - 1)]
    return [min(w, hi) for w in out] + [hi]


def contrast_spread(values: Sequence[float]) -> float:
    """(max - min) / mean of one camera's contrasts across pulse widths."""
    m = sum(values) / len(values)
    return (max(values) - min(values)) / m if m > 0 else math.inf


# ---------------------------------------------------------------------------
# The procedure
# ---------------------------------------------------------------------------

class SeedlessCminProcedure:
    def __init__(self, bench: CminBench, settings: CminSettings,
                 expected_cameras: set, *,
                 output_func: Callable[[str], None] = print,
                 detail_func: Callable[[str], None] | None = None):
        settings.validate()
        if not expected_cameras:
            raise ValueError("no cameras selected")
        self._bench = bench
        self._current: Optional[float] = None
        self._s = settings
        self._cams = set(expected_cameras)
        self._out = output_func
        self._detail = detail_func or (lambda _m: None)

    def _tag(self, result: CminResult, frames, phase, burst, ta_us, seed,
             current=None):
        result.frames.extend(
            TaggedFrame(phase, burst, ta_us, seed, f.side, f.cam_id,
                        f.abs_frame_id, f.mean, f.std, f.contrast, current)
            for f in frames)

    def _burst(self, result: CminResult, index: int, ta_us: float,
               light_frames: int, phase: str, what: str) -> BurstResult:
        current = self._current
        frames = self._bench.capture(light_frames, seedless=True)
        self._tag(result, frames, phase, index, ta_us, "off", current)
        cams = camera_stats(frames)
        _check_frame_counts(cams, self._cams, light_frames,
                            self._s.min_frame_fraction, what)
        b = replace(compare_to_nominal(index, ta_us, cams, result.nominal),
                    ta_current_ma=current)
        cur = f", {current:.0f} mA" if current is not None else ""
        self._detail(f"{what}: TA {ta_us:.1f} us{cur} -> mean {b.ratio:.3f}x "
                     f"nominal, contrast {b.contrast:.4f}")
        return b

    def run(self) -> CminResult:
        result = CminResult(match_by=self._s.match_by)
        s = self._s
        try:
            if s.warmup:
                self._warm_up(result)
            self._out(f"Measuring the normal image ({s.nominal_light_frames} "
                      "frames, seed on) ...")
            nominal_frames = self._bench.capture(s.nominal_light_frames,
                                                 seedless=False)
            self._tag(result, nominal_frames, PHASE_NOMINAL, 0,
                      quantize_ta_us(500.0), "on")
            cams = camera_stats(nominal_frames)
            _check_frame_counts(cams, self._cams, s.nominal_light_frames,
                                s.min_frame_fraction, "normal capture")
            result.nominal = tuple(c for c in cams
                                   if (c.side, c.cam_id) in self._cams)
            for c in result.nominal:
                self._detail(f"nominal {c.label}: mean {c.mean:.2f} "
                             f"(sd {c.mean_sd:.2f}), contrast {c.contrast:.4f}, "
                             f"n={c.n}")
            check_nominal(result.nominal, s)
            nominal_mean = sum(c.mean for c in result.nominal) / len(result.nominal)
            self._out(f"Normal image mean {nominal_mean:.1f}: target for the "
                      "seedless frames.")

            self._out("Turning the seed OFF. Searching for the TA pulse width "
                      "that matches the normal brightness ...")
            ta = quantize_ta_us(s.start_ta_us)
            result.seedless_entered = True
            self._bench.enter_seedless(ta)
            self._current = self._bench.ta_current_ma()
            result.start_current_ma = self._current
            self._out(f"TA drive current {self._current:.0f} mA (the normal "
                      "image was taken at this current).")
            if s.current_stepping and self._current >= s.max_current_ma - 0.01:
                self._out("  The current is already at the ceiling: the search "
                          "can only lengthen the pulse.")
            # The nominal scan ran at the unit's own current.
            result.frames = [replace(f, ta_current_ma=self._current)
                             if f.phase == PHASE_NOMINAL else f
                             for f in result.frames]
            if s.current_stepping:
                self._search_with_current(result)
                if result.matched_ta_us is None:      # plateau path finished
                    return result                     # (finally restores)
            else:
                for index in range(1, s.max_bursts + 1):
                    if index > 1:
                        self._bench.set_ta_pulse_width_us(ta)
                    b = self._burst(result, index, ta, s.burst_light_frames,
                                    PHASE_SEARCH, f"burst {index}")
                    result.bursts.append(b)
                    self._out(f"  burst {index}: pulse {ta:.0f} us -> "
                              f"{b.ratio:.2f}x normal brightness")
                    if index == 1:
                        check_first_burst(b, result.nominal, s)
                    if abs(b.ratio - 1.0) <= s.tolerance:
                        result.matched_ta_us = ta
                        result.matched_current_ma = self._current
                        break
                    try:
                        ta = next_ta_us(result.bursts, s)
                    except OutOfReach as exc:
                        if not s.plateau_check:
                            raise
                        self._plateau(result, str(exc), result.bursts)
                        return result      # the finally below still restores
                else:
                    raise ProcedureStop(
                        f"no match within {s.max_bursts} bursts (last "
                        f"{result.bursts[-1].ratio:.3f}x at "
                        f"{result.bursts[-1].ta_pulse_width_us:.0f} us)")

            self._capture_and_evaluate(result)
        except ProcedureStop as exc:
            result.problem = str(exc)
        except Exception as exc:
            result.problem = f"{type(exc).__name__}: {exc}"
        finally:
            if result.seedless_entered and not result.seedless_exited:
                try:
                    result.restore_problem = self._bench.exit_seedless()
                except Exception as exc:
                    result.restore_problem = f"{type(exc).__name__}: {exc}"
                if not result.restore_problem:
                    self._out("Laser settings restored, seed back ON.")
                if result.restore_problem:
                    result.verdict = VERDICT_FAIL
                    if not result.problem:
                        result.problem = ("the normal laser settings were not "
                                          "restored: " + result.restore_problem)
        return result

    def _warm_up(self, result: CminResult) -> None:
        """Seeded chunks until each camera's mean changes by at most
        warmup_stable from one chunk to the next."""
        s = self._s
        secs = s.warmup_chunk_frames / 40.0
        self._out(f"Warming up the laser (seed on, {secs:.0f} s at a time, "
                  f"until the image holds within {s.warmup_stable:.1%}) ...")
        prev = None
        for k in range(1, int(s.warmup_max_chunks) + 1):
            frames = self._bench.capture(int(s.warmup_chunk_frames),
                                         seedless=False)
            self._tag(result, frames, PHASE_WARMUP, k, quantize_ta_us(500.0), "on")
            cams = camera_stats(frames)
            _check_frame_counts(cams, self._cams, int(s.warmup_chunk_frames),
                                s.min_frame_fraction, f"warm-up {k}")
            stats = {(c.side, c.cam_id): c.mean for c in cams
                     if (c.side, c.cam_id) in self._cams}
            if prev is not None:
                change = max(abs(stats[key] / prev[key] - 1.0) for key in stats)
                result.warmup.append(change)
                self._detail(f"warm-up {k}: largest change {change:.2%}")
                if k >= s.warmup_min_chunks and change <= s.warmup_stable:
                    self._out(f"  Laser warm after {k * secs:.0f} s.")
                    return
            prev = stats
        raise ProcedureStop(
            f"the seeded image did not settle within "
            f"{s.warmup_max_chunks * secs:.0f} s of warm-up (last change "
            f"{result.warmup[-1]:.2%}, limit {s.warmup_stable:.1%})")

    def _capture_and_evaluate(self, result: CminResult) -> None:
        """Repeat captures at the matched setting, per-camera C_min with a
        correction to exactly 1.0x, then (after the restore) the seeded
        nominal again. Every gate failure raises ProcedureStop; the per-camera
        results are filled in first, so the report always has them."""
        s = self._s
        n = int(s.cmin_repeats)
        self._out(f"Matched at {result.matched_ta_us:.1f} us, "
                  f"{self._current:.0f} mA. Capturing {n} x "
                  f"{s.cmin_light_frames} seedless frames ...")
        frames_before = len(result.frames)
        for i in range(1, n + 1):
            cap = self._burst(result, i, result.matched_ta_us,
                              s.cmin_light_frames, PHASE_CMIN,
                              f"C_min capture {i} of {n}")
            result.cmin_captures.append(cap)
            if abs(cap.ratio - 1.0) > s.cmin_tolerance:
                raise ProcedureStop(
                    f"C_min capture {i} drifted to {cap.ratio:.3f}x the normal "
                    f"brightness (allowed {1 - s.cmin_tolerance:.2f}.."
                    f"{1 + s.cmin_tolerance:.2f}x)")
        all_cap = [f for f in result.frames[frames_before:]]
        combined = replace(compare_to_nominal(
            0, result.matched_ta_us,
            camera_stats([FrameRecord(f.side, f.cam_id, f.abs_frame_id, f.mean,
                                      f.std, f.contrast) for f in all_cap]),
            result.nominal), ta_current_ma=self._current)
        result.cmin_capture = combined

        nominal = {(c.side, c.cam_id): c for c in result.nominal}
        if s.match_by == MATCH_BY_CURRENT:
            # Width held at the ceiling: brightness there varied by current.
            local = [b for b in result.bursts
                     if abs(b.ta_pulse_width_us - result.matched_ta_us) < 0.5]
        else:
            local = [b for b in result.bursts if b.ta_current_ma == self._current]
        local = local + result.cmin_captures
        cams: list[CameraCmin] = []
        for c in combined.cameras:
            key = (c.side, c.cam_id)
            if key not in self._cams:
                continue
            nom = nominal[key]
            reps = tuple(cc.contrast for b in result.cmin_captures
                         for cc in b.cameras if (cc.side, cc.cam_id) == key)
            if not all(math.isfinite(v) for v in reps) or not reps:
                raise ProcedureStop(f"no contrast for {c.label}")
            raw = sum(reps) / len(reps)
            ratio = c.mean / nom.mean
            pts = [(cc.mean / nom.mean, cc.contrast) for b in local
                   for cc in b.cameras if (cc.side, cc.cam_id) == key
                   and cc.mean / nom.mean >= s.correction_min_ratio]
            slope = contrast_slope(pts)
            corrected = raw + slope * (1.0 - ratio) if slope is not None else raw
            cams.append(CameraCmin(
                c.side, c.cam_id, nom.mean, nom.contrast, ratio, raw, reps,
                contrast_spread(reps), slope, corrected,
                corrected / raw - 1.0 if raw else math.nan))
        result.cameras = cams
        result.cmin_per_camera = {c.label: c.cmin for c in cams}
        result.cmin = sum(c.cmin for c in cams) / len(cams)
        result.basis = BASIS_MATCHED
        for c in cams:
            self._detail(
                f"{c.label}: brightness {c.ratio:.3f}x, C_min raw {c.cmin_raw:.4f}"
                f" -> {c.cmin:.4f} at 1.0x (slope "
                + ("n/a" if c.slope is None else f"{c.slope:+.4f}")
                + f"), repeats spread {c.repeat_spread:.1%}")
            if c.slope is None:
                result.notes.append(f"{c.label}: no brightness spread near the "
                                    "match, C_min not corrected to 1.0x")
            elif abs(c.correction) > s.correction_flag:
                result.notes.append(f"{c.label}: correction to 1.0x is "
                                    f"{c.correction:+.1%}")

        # Gate failures are collected, not raised, so the seeded re-check
        # below still runs and the report has the drift either way.
        problems = []
        off = [c for c in cams if abs(c.ratio - 1.0) > s.camera_tolerance]
        if off:
            problems.append(
                "brightness not matched per camera (limit "
                f"{s.camera_tolerance:.0%}): "
                + ", ".join(f"{c.label} {c.ratio:.3f}x" for c in off))
        noisy = [c for c in cams if not (c.repeat_spread <= s.repeat_spread_max)]
        if noisy:
            problems.append(
                f"C_min does not repeat within {s.repeat_spread_max:.0%}: "
                + ", ".join(f"{c.label} {c.repeat_spread:.1%}" for c in noisy))

        if s.nominal_recheck:
            result.seedless_exited = True
            result.restore_problem = self._bench.exit_seedless()
            if result.restore_problem:
                raise ProcedureStop("; ".join(problems + [
                    "the normal laser settings were not restored: "
                    + result.restore_problem]))
            self._out(f"Laser settings restored, seed back ON. Re-measuring the normal image "
                      f"({s.nominal_recheck_frames} frames) ...")
            frames = self._bench.capture(s.nominal_recheck_frames,
                                         seedless=False)
            self._tag(result, frames, PHASE_NOMINAL_AFTER, 0,
                      quantize_ta_us(500.0), "on", result.start_current_ma)
            after = camera_stats(frames)
            _check_frame_counts(after, self._cams, s.nominal_recheck_frames,
                                s.min_frame_fraction, "normal re-check")
            result.nominal_after = tuple(c for c in after
                                         if (c.side, c.cam_id) in self._cams)
            by_key = {(c.side, c.cam_id): c for c in result.nominal_after}
            result.cameras = cams = [
                replace(c, nominal_drift=by_key[(c.side, c.cam_id)].mean
                        / c.nominal_mean - 1.0) for c in cams]
            drift = [c for c in cams
                     if not (abs(c.nominal_drift) <= s.nominal_drift_max)]
            for c in cams:
                self._detail(f"{c.label}: normal brightness moved "
                             f"{c.nominal_drift:+.1%} during the run")
            if drift:
                problems.append(
                    "the normal (seeded) brightness moved during the run "
                    f"(limit {s.nominal_drift_max:.0%}): "
                    + ", ".join(f"{c.label} {c.nominal_drift:+.1%}" for c in drift))
        if problems:
            raise ProcedureStop("; ".join(problems))
        result.verdict = VERDICT_PASS

    def _search_with_current(self, result: CminResult) -> None:
        """Rounds of bursts at progressive pulse widths, one round per drive
        current. Sets result.matched_ta_us / matched_current_ma, or runs the
        (opt-in) plateau check, or raises ProcedureStop."""
        s = self._s
        index = 0

        def burst(ta: float, what: str) -> BurstResult:
            nonlocal index
            index += 1
            if index > s.max_total_bursts:
                raise ProcedureStop(f"no match within {s.max_total_bursts} bursts")
            if index > 1:
                self._bench.set_ta_pulse_width_us(ta)
            b = self._burst(result, index, ta, s.burst_light_frames,
                            PHASE_SEARCH, f"burst {index}")
            result.bursts.append(b)
            self._out(f"  burst {index}: {self._current:.0f} mA, pulse "
                      f"{ta:.0f} us -> {b.ratio:.2f}x normal brightness"
                      + (f" ({what})" if what else ""))
            return b

        by_current = s.match_by == MATCH_BY_CURRENT

        def matched(b: BurstResult) -> bool:
            if by_current:
                ok = (abs(b.ta_pulse_width_us - s.max_ta_us) < 0.5
                      and abs(b.ratio - 1.0) <= s.current_match_tolerance)
            else:
                ok = abs(b.ratio - 1.0) <= s.tolerance
            if ok:
                result.matched_ta_us = b.ta_pulse_width_us
                result.matched_current_ma = self._current
                return True
            return False

        first = burst(quantize_ta_us(s.start_ta_us), "seed-off check")
        check_first_burst(first, result.nominal, s)
        if matched(first):
            return
        round_bursts: list[BurstResult] = [first]
        prev_top: Optional[BurstResult] = None
        prev_current: Optional[float] = None
        widths = [quantize_ta_us(f * s.max_ta_us) for f in s.round_fractions]
        widths[-1] = min(widths[-1], s.max_ta_us)
        while True:
            result.currents_tried_ma.append(self._current)
            for w in widths:
                if any(abs(w - b.ta_pulse_width_us) < 0.5 for b in round_bursts):
                    continue
                b = burst(w, "")
                lower = [x for x in round_bursts if x.ta_pulse_width_us < w]
                if lower:
                    p = max(lower, key=lambda x: x.ta_pulse_width_us)
                    if b.ratio - p.ratio < s.min_ratio_gain * p.ratio:
                        raise ProcedureStop(
                            f"lengthening the pulse from {p.ta_pulse_width_us:.0f} "
                            f"to {w:.0f} us did not raise the image mean "
                            f"({p.ratio:.3f}x -> {b.ratio:.3f}x): the pulse is "
                            "not reaching the cameras as set")
                round_bursts.append(b)
                if matched(b):
                    return
            top = max(round_bursts, key=lambda x: x.ta_pulse_width_us)
            if prev_top is not None and not (
                    top.ratio >= prev_top.ratio * (1 + s.min_current_gain)):
                raise ProcedureStop(
                    f"raising the drive current to {self._current:.0f} mA did "
                    f"not raise the image mean ({prev_top.ratio:.3f}x -> "
                    f"{top.ratio:.3f}x at {top.ta_pulse_width_us:.0f} us)")
            need = extrapolate_width(round_bursts, 1.0)
            self._detail(f"at {self._current:.0f} mA a line through "
                         f"{len(round_bursts)} bursts reaches 1.0x at "
                         f"{need:.0f} us (limit {s.max_ta_us:.0f} us)")
            if need <= s.max_ta_us and by_current:
                full = [(c, b.ratio) for c, b in
                        ((prev_current, prev_top), (self._current, top))
                        if b is not None
                        and abs(b.ta_pulse_width_us - s.max_ta_us) < 0.5]
                self._refine_current(result, full, burst, matched)
                return
            if need <= s.max_ta_us:
                # Reachable at this current: refine (fit first, then secant).
                hist = sorted(round_bursts, key=lambda x: x.ta_pulse_width_us)
                ta = quantize_ta_us(min(max(need, MIN_TA_PULSE_WIDTH_US),
                                        s.max_ta_us))
                for _ in range(s.max_bursts):
                    b = burst(ta, "refine")
                    hist.append(b)
                    if matched(b):
                        return
                    try:
                        ta = next_ta_us(hist, s)
                    except OutOfReach:
                        break      # the line was optimistic: more current
                else:
                    raise ProcedureStop(
                        f"no match within {s.max_bursts} refining bursts at "
                        f"{self._current:.0f} mA")
                round_bursts = hist
                top = max(round_bursts, key=lambda x: x.ta_pulse_width_us)
            if self._current >= s.max_current_ma - 0.01:
                reason = (f"the normal brightness is out of reach: at "
                          f"{self._current:.0f} mA and {top.ta_pulse_width_us:.0f} us "
                          f"the seedless mean is only {top.ratio:.3f}x nominal "
                          f"(a line through this current's bursts needs "
                          f"{need:.0f} us)")
                if s.plateau_check:
                    self._plateau(result, reason, round_bursts)
                    return
                raise OutOfReach(reason)
            new = min(self._current + s.current_step_ma, s.max_current_ma)
            self._out(f"Out of reach at {self._current:.0f} mA (needs about "
                      f"{need:.0f} us). Raising the TA drive current to "
                      f"{new:.0f} mA ...")
            prev_current = self._current
            self._bench.set_ta_current_ma(new)
            self._current = self._bench.ta_current_ma()
            prev_top = top
            round_bursts = []

    def _refine_current(self, result, points, burst, matched) -> None:
        """Hold the pulse at the ceiling and tune the drive current until the
        seedless mean is 1.0x (secant on (current, ratio))."""
        s = self._s
        w = s.max_ta_us
        pts = sorted(points)
        self._out(f"Holding the pulse at {w:.0f} us and tuning the TA drive "
                  "current to match ...")
        for _ in range(s.max_bursts):
            if len(pts) >= 2:
                (c0, r0), (c1, r1) = pts[-2], pts[-1]
                if abs(r1 - r0) < 1e-6:
                    raise ProcedureStop("the drive current does not change the "
                                        "image mean")
                target = c1 + (1.0 - r1) * (c1 - c0) / (r1 - r0)
            else:
                c1, r1 = pts[-1]
                target = c1 + (s.current_step_ma if r1 < 1 else -s.current_step_ma)
            step = max(-s.max_current_step_ma,
                       min(s.max_current_step_ma, target - self._current))
            new = min(max(self._current + step, MIN_TA_CURRENT_MA),
                      s.max_current_ma)
            if abs(new - self._current) < TA_CURRENT_TICK_MA:
                break
            self._bench.set_ta_current_ma(new)
            self._current = self._bench.ta_current_ma()
            b = burst(w, "current refine")
            if matched(b):
                return
            pts.append((self._current, b.ratio))
            pts = pts[-2:] if len(pts) > 2 else pts
        raise ProcedureStop(
            f"could not match the brightness by current at {w:.0f} us "
            f"(last {pts[-1][1]:.3f}x at {pts[-1][0]:.0f} mA)")

    def _plateau(self, result: CminResult, reason: str,
                 history: Sequence[BurstResult]) -> None:
        """Brightness cannot be matched: check that each camera's contrast
        is flat across the upper part of the reachable range instead.
        Called inside run()'s try, so a ProcedureStop fails the run and the
        restore still happens."""
        s = self._s
        top = max(history, key=lambda b: b.ta_pulse_width_us)
        w_max = top.ta_pulse_width_us
        if top.ratio < s.plateau_min_ratio:
            raise ProcedureStop(
                f"{reason}, below the {s.plateau_min_ratio:g}x the plateau "
                "check needs")
        w_lo = width_for_ratio(history, s.plateau_min_ratio,
                               s.start_ta_us, w_max)
        if w_max - w_lo < 0.1 * w_max:
            raise ProcedureStop(
                f"{reason}; the pulse range above {s.plateau_min_ratio:g}x "
                f"({w_lo:.0f}..{w_max:.0f} us) is too narrow for the plateau "
                "check")
        widths = plateau_widths(w_lo, w_max, int(s.plateau_points))
        self._out(f"Normal brightness is out of reach ({top.ratio:.2f}x at "
                  f"{w_max:.0f} us). Checking that contrast stays flat per "
                  f"camera from {widths[0]:.0f} to {w_max:.0f} us ...")
        index = len(result.bursts)
        for w in widths[:-1]:
            index += 1
            self._bench.set_ta_pulse_width_us(w)
            b = self._burst(result, index, w, s.burst_light_frames,
                            PHASE_PLATEAU, f"plateau burst {index}")
            result.plateau.append(b)
            self._out(f"  plateau: pulse {w:.0f} us -> {b.ratio:.2f}x normal "
                      f"brightness, contrast {b.contrast:.4f}")
        self._bench.set_ta_pulse_width_us(w_max)
        self._out(f"Capturing {s.cmin_light_frames} seedless frames at "
                  f"{w_max:.0f} us ...")
        cap = self._burst(result, 0, w_max, s.cmin_light_frames, PHASE_CMIN,
                          "C_min capture")
        result.cmin_capture = cap
        per_cam = {c.label: c.contrast for c in cap.cameras
                   if (c.side, c.cam_id) in self._cams}
        bad = [k for k, v in per_cam.items() if not math.isfinite(v)]
        if bad:
            raise ProcedureStop(f"no contrast for {', '.join(bad)}")
        result.cmin_per_camera = per_cam
        result.cmin = sum(per_cam.values()) / len(per_cam)
        result.basis = BASIS_PLATEAU
        for label in per_cam:
            values = [c.contrast for b in (*result.plateau, cap)
                      for c in b.cameras if c.label == label]
            result.plateau_spread[label] = contrast_spread(values)
        for label, spread in result.plateau_spread.items():
            self._detail(f"plateau {label}: contrast spread {spread:.1%}")
        over = {k: v for k, v in result.plateau_spread.items()
                if not (v <= s.plateau_tolerance)}
        if over:
            raise ProcedureStop(
                f"{reason}, and contrast is not flat from {widths[0]:.0f} to "
                f"{w_max:.0f} us (limit {s.plateau_tolerance:.0%} per camera): "
                + ", ".join(f"{k} {v:.1%}" for k, v in over.items()))
        result.verdict = VERDICT_PASS
