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
from dataclasses import asdict, dataclass, field
from typing import Callable, Optional, Protocol, Sequence

from omotion.seedless import (
    MAX_TA_PULSE_WIDTH_US,
    MIN_TA_PULSE_WIDTH_US,
    ta_ticks_from_us,
    ta_us_from_ticks,
)

PHASE_NOMINAL = "nominal"
PHASE_SEARCH = "search"
PHASE_CMIN = "cmin"

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

    def exit_seedless(self) -> Optional[str]:
        """Restore everything; ``None`` when verified, else the problem."""


class ProcedureStop(Exception):
    """The run cannot continue; the message is for the operator."""


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

    def to_dict(self) -> dict:
        def stats(cams):
            return [dict(asdict(c), label=c.label) for c in cams]

        def burst(b: Optional[BurstResult]):
            if b is None:
                return None
            return {"burst": b.burst, "ta_pulse_width_us": b.ta_pulse_width_us,
                    "ratio": b.ratio, "contrast": b.contrast,
                    "cameras": stats(b.cameras)}

        return {
            "verdict": self.verdict,
            "problem": self.problem,
            "c_min": self.cmin,
            "c_min_per_camera": dict(self.cmin_per_camera),
            "matched_ta_pulse_width_us": self.matched_ta_us,
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
            raise ProcedureStop(
                f"the normal brightness is out of reach: at the "
                f"{w:.0f} us limit the seedless mean is only {r:.3f}x nominal")
        raise ProcedureStop(
            f"the pulse cannot get short enough: at {w:.0f} us the seedless "
            f"mean is already {r:.3f}x nominal")
    return nxt


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
        self._s = settings
        self._cams = set(expected_cameras)
        self._out = output_func
        self._detail = detail_func or (lambda _m: None)

    def _tag(self, result: CminResult, frames, phase, burst, ta_us, seed):
        result.frames.extend(
            TaggedFrame(phase, burst, ta_us, seed, f.side, f.cam_id,
                        f.abs_frame_id, f.mean, f.std, f.contrast)
            for f in frames)

    def _burst(self, result: CminResult, index: int, ta_us: float,
               light_frames: int, phase: str, what: str) -> BurstResult:
        frames = self._bench.capture(light_frames, seedless=True)
        self._tag(result, frames, phase, index, ta_us, "off")
        cams = camera_stats(frames)
        _check_frame_counts(cams, self._cams, light_frames,
                            self._s.min_frame_fraction, what)
        b = compare_to_nominal(index, ta_us, cams, result.nominal)
        self._detail(f"{what}: TA {ta_us:.1f} us -> mean {b.ratio:.3f}x "
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
                    break
                ta = next_ta_us(result.bursts, s)
            else:
                raise ProcedureStop(
                    f"no match within {s.max_bursts} bursts (last "
                    f"{result.bursts[-1].ratio:.3f}x at "
                    f"{result.bursts[-1].ta_pulse_width_us:.0f} us)")

            self._out(f"Matched at {result.matched_ta_us:.1f} us. Capturing "
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
