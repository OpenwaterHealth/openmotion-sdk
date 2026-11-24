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
    MIN_TA_PULSE_WIDTH_US,
    ta_ticks_from_us,
    ta_us_from_ticks,
)

PHASE_NOMINAL = "nominal"
PHASE_SEARCH = "search"
PHASE_CMIN = "cmin"
PHASE_PLATEAU = "plateau"

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
        result = CminResult()
        s = self._s
        try:
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

            self._out(f"Matched at {result.matched_ta_us:.1f} us, "
                      f"{self._current:.0f} mA. Capturing "
                      f"{s.cmin_light_frames} seedless frames ...")
            cap = self._burst(result, 0, result.matched_ta_us,
                              s.cmin_light_frames, PHASE_CMIN, "C_min capture")
            result.cmin_capture = cap
            if abs(cap.ratio - 1.0) > s.cmin_tolerance:
                raise ProcedureStop(
                    f"the C_min capture drifted to {cap.ratio:.3f}x the normal "
                    f"brightness (allowed {1 - s.cmin_tolerance:.2f}.."
                    f"{1 + s.cmin_tolerance:.2f}x)")
            per_cam = {c.label: c.contrast for c in cap.cameras
                       if (c.side, c.cam_id) in self._cams}
            bad = [k for k, v in per_cam.items() if not math.isfinite(v)]
            if bad:
                raise ProcedureStop(f"no contrast for {', '.join(bad)}")
            result.cmin_per_camera = per_cam
            result.cmin = sum(per_cam.values()) / len(per_cam)
            result.basis = BASIS_MATCHED
            result.verdict = VERDICT_PASS
        except ProcedureStop as exc:
            result.problem = str(exc)
        except Exception as exc:
            result.problem = f"{type(exc).__name__}: {exc}"
        finally:
            if result.seedless_entered:
                try:
                    result.restore_problem = self._bench.exit_seedless()
                except Exception as exc:
                    result.restore_problem = f"{type(exc).__name__}: {exc}"
                if result.restore_problem:
                    result.verdict = VERDICT_FAIL
                    if not result.problem:
                        result.problem = ("the normal laser settings were not "
                                          "restored: " + result.restore_problem)
        return result

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

        def matched(b: BurstResult) -> bool:
            if abs(b.ratio - 1.0) <= s.tolerance:
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
            self._bench.set_ta_current_ma(new)
            self._current = self._bench.ta_current_ma()
            prev_top = top
            round_bursts = []

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
