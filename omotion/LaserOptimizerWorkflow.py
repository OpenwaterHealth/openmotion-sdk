"""LaserOptimizerWorkflow — find the laser pulse width and camera gain that put
the near cameras at a target image mean, then scan at those settings.

For experiments where the stock settings are far too dim, such as blood flow
on horse hooves (sdk#335, bloodflow-app#745). It starts at the brightest
setting the hardware allows (16x analog gain, 2000 µs laser pulse), steps
down until the dark-corrected mean of the near cameras is inside the target
band, then runs the caller's scan at that setting.

How it is put together, so each part can be checked on its own:

* :func:`plan_next` is the whole algorithm: a pure function from the probes
  so far to the next action. No hardware, no threads.
* :func:`setting_for_signal` turns a wanted signal level into a gain and a
  pulse width (the balanced curve described there).
* A *probe* is one 1-second laser scan through the shared short-scan engine
  (:func:`~omotion.ScanWorkflow.run_collection_scan`, the one behind the
  contact-quality check). Every probe builds a fresh pipeline and takes its
  own boundary dark frame after the new gain is written, so the dark mean is
  re-measured at each probe's own gain and exposure. :class:`_ProbeSink`
  reduces it to one :class:`CameraReading` per camera.
* :class:`LaserOptimizerWorkflow` runs setup, probes, the final scan and the
  restore on a worker thread. The restore runs in a ``finally`` on every exit.

Engineering experiments only: setup widens the safety FPGAs' pulse-width and
rate limits past what a console's stored configuration may hold, for as long
as the workflow runs. Pulse width is the only laser setting it changes; drive
current and the current limits are never touched.
"""

from __future__ import annotations

import dataclasses
import enum
import logging
import math
import statistics
import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Optional, Sequence

from omotion import _log_root
from omotion.register_overrides import (
    CAMERA_GAIN_CHOICES,
    FW_DEFAULT_CAMERA_GAINS,
    FW_DEFAULT_EXPOSURE_ROWS,
    TA_PULSE_TICK_US,
    TA_PULSE_TRUNCATION_TICKS,
    TA_PULSE_WIDTH_MAX_US,
    TA_PULSE_WIDTH_MIN_US,
    emitted_pulse_us,
    exposure_rows_for_us,
    read_safety_faults,
    write_camera_settings,
    write_safety_limits,
    write_ta_pulse_width,
)
from omotion.ScanWorkflow import ScanRequest, run_collection_scan

logger = logging.getLogger(
    f"{_log_root}.LaserOptimizer" if _log_root else "LaserOptimizer"
)

SIDES = ("left", "right")

# Gain of each camera position as a multiple of the near cameras' gain:
# row 1 (positions 4 and 5, at the laser) 1x, row 2 (3 and 6) 2x, row 3
# (2 and 7) 4x, row 4 (1 and 8) 8x, each capped at CAMERA_GAIN_CEILING.
ROW_GAIN_RATIO = (8, 4, 2, 1, 1, 2, 4, 8)
CAMERA_GAIN_CEILING = max(CAMERA_GAIN_CHOICES)   # 16x, the sensor's top analog gain


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class OptimizerConfig:
    """Every number the optimizer uses. Defaults are the agreed design."""

    # Dark-corrected mean (DN) the near cameras should land on, and the
    # accepted band around it (+/- tolerance as a fraction).
    target_mean_dn: float = 100.0
    tolerance: float = 0.15
    # Near cameras: positions 4 and 5 of each module (0-based ids 3 and 4),
    # the pair at the laser end. The brightest of them on any active side
    # decides.
    near_camera_ids: tuple = (3, 4)
    # Brightest setting, where every run starts (near-camera gain, pulse).
    max_gain: int = 16
    max_pulse_width_us: int = 2000
    min_pulse_width_us: int = TA_PULSE_WIDTH_MIN_US
    # The other rows' gains as multiples of the near gain (ROW_GAIN_RATIO).
    row_gain_ratio: tuple = ROW_GAIN_RATIO
    # Pulse width the balance curve pairs with 1x gain: W = this * sqrt(gain),
    # which also passes through (16x, 2000 µs). 500 µs is the stock setting.
    balance_width_us: float = 500.0
    # Fixed for the whole run. 2196 µs (244 rows) is the camera maximum and
    # covers the 100 µs laser delay plus the 2000 µs pulse.
    exposure_us: float = 2196.0
    # Dark frames displace the pulse to start at delay + this; it must start
    # after the exposure ends or the dark reference catches it.
    dark_skip_delay_us: int = 2400
    # Safety FPGA limits for the duration of the run (both EE and OPT).
    # PULSE_WIDTH_UL stays above the longest pulse so a stuck-on TA still
    # trips; RATE_LL sits under the 22,600 µs post-dark gap at 40 Hz.
    safety_pulse_width_ul_us: float = 2200.0
    safety_rate_ll_us: float = 22000.0
    # One probe: seconds of laser-on capture.
    probe_duration_sec: float = 1.0
    # A light frame's pixels in histogram bins >= clip_bin count as clipped;
    # a camera whose mean clipped fraction exceeds clip_fraction is clipped.
    clip_bin: int = 1020
    clip_fraction: float = 0.01
    # Signal multiplier after a clipped probe (its mean is only a lower
    # bound), and the bigger step once most of the image is clipped: a bright
    # target (the bench phantom, 2026-10-21) sat >75% clipped for the first
    # four halvings, which cost four probes. An overshoot only costs one
    # probe, because an unclipped reading jumps straight back via the model.
    clipped_step: float = 0.5
    heavy_clip_fraction: float = 0.5
    heavy_clipped_step: float = 0.25
    max_probes: int = 10

    def __post_init__(self) -> None:
        if not 0 < self.target_mean_dn < 1000:
            raise ValueError(f"target_mean_dn {self.target_mean_dn} outside 0-1000 DN")
        if not 0 < self.tolerance < 1:
            raise ValueError(f"tolerance {self.tolerance} outside 0-1")
        if self.max_gain not in CAMERA_GAIN_CHOICES:
            raise ValueError(f"max_gain {self.max_gain} not one of {CAMERA_GAIN_CHOICES}")
        if not (TA_PULSE_WIDTH_MIN_US <= self.min_pulse_width_us
                <= self.max_pulse_width_us <= TA_PULSE_WIDTH_MAX_US):
            raise ValueError(
                f"pulse widths {self.min_pulse_width_us}-{self.max_pulse_width_us} µs "
                f"outside {TA_PULSE_WIDTH_MIN_US}-{TA_PULSE_WIDTH_MAX_US} µs")
        if (len(self.row_gain_ratio) != 8
                or any(r not in CAMERA_GAIN_CHOICES for r in self.row_gain_ratio)):
            raise ValueError(f"row_gain_ratio {self.row_gain_ratio} is not 8 gain steps")

    @property
    def band(self) -> tuple[float, float]:
        return (self.target_mean_dn * (1.0 - self.tolerance),
                self.target_mean_dn * (1.0 + self.tolerance))

    @property
    def max_setting(self) -> "OptimizerSetting":
        return OptimizerSetting(self.max_gain, self.max_pulse_width_us)

    @property
    def near_mask(self) -> int:
        return sum(1 << c for c in self.near_camera_ids)


# ---------------------------------------------------------------------------
# Settings and the signal model
# ---------------------------------------------------------------------------

def width_for_emitted_us(emitted_us: float) -> float:
    """Commanded TA width that emits ``emitted_us`` (inverse of
    :func:`~omotion.register_overrides.emitted_pulse_us`)."""
    return emitted_us + TA_PULSE_TRUNCATION_TICKS * TA_PULSE_TICK_US


@dataclass(frozen=True)
class OptimizerSetting:
    """One point the optimizer can probe: the near cameras' analog gain and
    the TA pulse width (whole µs, as commanded)."""

    near_gain: int
    pulse_width_us: int

    @property
    def signal(self) -> float:
        """Relative brightness, gain x emitted pulse (µs). The model: the
        dark-corrected mean is proportional to this until pixels clip."""
        return self.near_gain * emitted_pulse_us(self.pulse_width_us)

    def camera_gains(self, row_ratio: Sequence[int] = ROW_GAIN_RATIO) -> tuple:
        """8 gains by camera position: ``near_gain`` times the row's ratio
        (1x/2x/4x/8x from the laser end out), capped at 16x. 16x everywhere
        at 16x near gain; 8/4/2/1/1/2/4/8 at 1x."""
        return tuple(min(CAMERA_GAIN_CEILING, self.near_gain * r) for r in row_ratio)

    def describe(self) -> str:
        return f"{self.near_gain}x / {self.pulse_width_us} µs"


def setting_for_signal(signal: float, cfg: OptimizerConfig) -> OptimizerSetting:
    """The gain and pulse width for a wanted ``signal`` (gain x emitted µs).

    The cut from the brightest setting is split evenly between gain and pulse
    width, in log terms, along W = balance_width x sqrt(gain): the curve
    through both (16x, 2000 µs) and the stock (1x, 500 µs). Gain is discrete
    (1/2/4/8/16), so pick the gain nearest that curve (ties go to the lower
    gain), then solve the pulse width exactly. A width above the maximum
    moves up a gain; below the minimum moves down one. Halving from the top:
    16x/2000, 8x/2000, 8x/1000, 4x/1000, 2x/1000, 2x/500, 1x/500, 1x/250, ...
    """
    gains = [g for g in CAMERA_GAIN_CHOICES if g <= cfg.max_gain]
    signal = max(float(signal), 1e-9)
    # On the curve, signal = G * balance * sqrt(G), so G = (signal/balance)^(2/3).
    g_curve = (signal / cfg.balance_width_us) ** (2.0 / 3.0)
    idx = math.ceil(math.log2(g_curve) - 0.5)   # nearest power of 2, ties down
    idx = min(max(idx, 0), len(gains) - 1)

    def width(i: int) -> float:
        return width_for_emitted_us(signal / gains[i])

    while width(idx) > cfg.max_pulse_width_us and idx < len(gains) - 1:
        idx += 1
    while width(idx) < cfg.min_pulse_width_us and idx > 0:
        idx -= 1
    w = int(round(width(idx)))
    w = min(max(w, cfg.min_pulse_width_us), cfg.max_pulse_width_us)
    return OptimizerSetting(gains[idx], w)


def _same_setting(a: OptimizerSetting, b: OptimizerSetting) -> bool:
    """Equal for the planner's purposes: same gain, width within 1% (a model
    jump that lands 1 µs from a probed width would just repeat it)."""
    return (a.near_gain == b.near_gain
            and abs(a.pulse_width_us - b.pulse_width_us)
            <= max(2, 0.01 * b.pulse_width_us))


# ---------------------------------------------------------------------------
# Probe results
# ---------------------------------------------------------------------------

@dataclass
class CameraReading:
    """One camera's summary of one probe."""

    side: str
    cam_id: int                 # 0-based position in the module
    mean_dn: float              # median dark-corrected light-frame mean; NaN = no frames
    clipped_fraction: float     # mean fraction of pixels at full scale; NaN = no frames
    dark_dn: float              # raw mean of this probe's dark frame(s): the re-measured dark
    light_frames: int


@dataclass
class ProbeResult:
    index: int                  # 1-based
    setting: OptimizerSetting
    near: list                  # CameraReading, near cameras on active sides
    others: list                # CameraReading, every other active camera
    clip_fraction_limit: float

    @property
    def control(self) -> Optional[CameraReading]:
        """The near camera that decides: the brightest one with data."""
        with_data = [r for r in self.near if math.isfinite(r.mean_dn)]
        return max(with_data, key=lambda r: r.mean_dn) if with_data else None

    @property
    def mean_dn(self) -> float:
        c = self.control
        return c.mean_dn if c is not None else float("nan")

    @property
    def clipped_fraction(self) -> float:
        vals = [r.clipped_fraction for r in self.near
                if math.isfinite(r.clipped_fraction)]
        return max(vals) if vals else float("nan")

    @property
    def clipped(self) -> bool:
        f = self.clipped_fraction
        return math.isfinite(f) and f > self.clip_fraction_limit

    def to_dict(self) -> dict:
        def cam(r: CameraReading) -> dict:
            return {
                "side": r.side, "camera": r.cam_id + 1,
                "mean_dn": _round(r.mean_dn), "clipped_fraction": _round(r.clipped_fraction, 4),
                "dark_dn": _round(r.dark_dn), "light_frames": r.light_frames,
            }
        c = self.control
        return {
            "probe": self.index,
            "near_gain": self.setting.near_gain,
            "pulse_width_us": self.setting.pulse_width_us,
            "mean_dn": _round(self.mean_dn),
            "control": f"{c.side} camera {c.cam_id + 1}" if c else None,
            "clipped_fraction": _round(self.clipped_fraction, 4),
            "clipped": self.clipped,
            "cameras": [cam(r) for r in self.near + self.others],
        }

    def describe(self) -> str:
        c = self.control
        where = f" ({c.side} camera {c.cam_id + 1})" if c else ""
        darks = ", ".join(
            f"{r.side[0].upper()}{r.cam_id + 1} {r.dark_dn:.1f}"
            for r in self.near if math.isfinite(r.dark_dn)
        )
        return (f"probe {self.index}: {self.setting.describe()}: near mean "
                f"{self.mean_dn:.1f} DN{where}, clipped "
                f"{self.clipped_fraction:.2%}, dark DN [{darks}]")


def _round(v: float, nd: int = 1):
    return round(v, nd) if isinstance(v, (int, float)) and math.isfinite(v) else None


# ---------------------------------------------------------------------------
# The planner
# ---------------------------------------------------------------------------

class Action(str, enum.Enum):
    PROBE = "probe"                         # probe Decision.setting next
    ACCEPT = "accept"                       # scan at Decision.setting
    INSUFFICIENT_SIGNAL = "insufficient_signal"  # too dim at the brightest setting
    TOO_BRIGHT = "too_bright"               # still above the band at the dimmest
    NO_CONVERGENCE = "no_convergence"       # out of probes, or would repeat one


@dataclass(frozen=True)
class Decision:
    action: Action
    setting: Optional[OptimizerSetting]
    reason: str


def plan_next(probes: Sequence[ProbeResult], cfg: OptimizerConfig) -> Decision:
    """The next step, given every probe so far (oldest first).

    * No probes yet: probe the brightest setting.
    * Last probe clipped: its mean is only a lower bound, so ask for
      ``clipped_step`` (half) of its signal, or ``heavy_clipped_step`` (a
      quarter) when more than ``heavy_clip_fraction`` of the pixels clip.
    * Last probe in the band: accept it.
    * Last probe below the band at the brightest setting: not enough light.
    * Otherwise: the model wants ``signal x target / mean``. That is capped
      below the dimmest setting seen clipping (staying under the clip onset),
      then mapped to a setting by :func:`setting_for_signal`.
    * A next setting equal to one already probed ends the run (TOO_BRIGHT
      at the dimmest end, otherwise NO_CONVERGENCE), as does max_probes.
    """
    top = cfg.max_setting
    if not probes:
        return Decision(Action.PROBE, top, "start at the brightest setting")

    last = probes[-1]
    lo, hi = cfg.band
    mean = last.mean_dn
    if not math.isfinite(mean):
        mean = 0.0   # no light frames with a baseline: treat as no signal

    if not last.clipped and lo <= mean <= hi:
        return Decision(Action.ACCEPT, last.setting,
                        f"near mean {mean:.1f} DN is inside {lo:.0f}-{hi:.0f} DN")

    if last.clipped:
        step = (cfg.heavy_clipped_step
                if last.clipped_fraction > cfg.heavy_clip_fraction
                else cfg.clipped_step)
        want = last.setting.signal * step
        reason = (f"{last.clipped_fraction:.1%} of pixels clipped: "
                  f"{step:g}x the signal")
    else:
        if mean < lo and _same_setting(last.setting, top):
            return Decision(
                Action.INSUFFICIENT_SIGNAL, None,
                f"near mean {mean:.1f} DN at the brightest setting "
                f"({top.describe()}) is below {lo:.0f} DN",
            )
        scale = cfg.target_mean_dn / max(mean, 1e-6)
        want = last.setting.signal * scale
        reason = f"near mean {mean:.1f} DN: {scale:.2f}x the signal"

    clipped_signals = [p.setting.signal for p in probes if p.clipped]
    if clipped_signals:
        want = min(want, 0.9 * min(clipped_signals))
    want = min(want, top.signal)

    if len(probes) >= cfg.max_probes:
        return Decision(Action.NO_CONVERGENCE, None,
                        f"no setting inside {lo:.0f}-{hi:.0f} DN after "
                        f"{len(probes)} probes")

    nxt = setting_for_signal(want, cfg)
    if any(_same_setting(nxt, p.setting) for p in probes):
        if _same_setting(nxt, last.setting) and (last.clipped or mean > hi):
            return Decision(Action.TOO_BRIGHT, None,
                            f"still above the band at the dimmest setting "
                            f"({nxt.describe()})")
        return Decision(Action.NO_CONVERGENCE, None,
                        f"next setting {nxt.describe()} was already probed "
                        f"({reason})")
    return Decision(Action.PROBE, nxt, reason)


# ---------------------------------------------------------------------------
# Probe sink
# ---------------------------------------------------------------------------

class _ProbeSink:
    """Collects per-camera readings from one probe's "live" channel.

    Light frames: ``mean_dc_rt`` (raw mean minus this probe's own dark
    baseline) and the fraction of the histogram at full scale. A light frame
    the pipeline flags ``low_light_rt`` (camera saw nothing) counts as 0 DN.
    Light frames before the probe's first dark have no baseline yet and are
    skipped. Dark frames: ``mean_raw``, reported so the probe table shows
    the dark level re-measured at each gain.
    """

    channels = frozenset({"live"})

    def __init__(self, clip_bin: int) -> None:
        self._clip_bin = int(clip_bin)
        self._means: dict = {}
        self._clipped: dict = {}
        self._darks: dict = {}

    def on_scan_start(self, meta) -> None:
        self._means.clear()
        self._clipped.clear()
        self._darks.clear()

    def consume(self, channel: str, batch) -> None:
        if channel != "live" or batch.mean_dc_rt is None:
            return
        low_light = getattr(batch, "low_light_rt", None)
        hist = getattr(batch, "raw_histograms", None)
        for i, s, c, ft in batch.iter_rows(exclude={"warmup", "stale"}):
            if s < 0 or not 0 <= c < 8:
                continue
            key = (s, c)
            if ft == "dark":
                if batch.mean_raw is not None:
                    v = float(batch.mean_raw[i, s, c])
                    if math.isfinite(v):
                        self._darks.setdefault(key, []).append(v)
                continue
            v = float(batch.mean_dc_rt[i, s, c])
            if not math.isfinite(v):
                if low_light is not None and bool(low_light[i, s, c]):
                    v = 0.0
                else:
                    continue
            self._means.setdefault(key, []).append(v)
            if hist is not None:
                h = hist[i, s, c]
                total = float(h.sum())
                if total > 0:
                    self._clipped.setdefault(key, []).append(
                        float(h[self._clip_bin:].sum()) / total)

    def on_complete(self) -> None:
        pass

    def reading(self, side_idx: int, cam_id: int) -> CameraReading:
        key = (side_idx, cam_id)
        means = self._means.get(key, [])
        clipped = self._clipped.get(key, [])
        darks = self._darks.get(key, [])
        return CameraReading(
            side=SIDES[side_idx],
            cam_id=cam_id,
            mean_dn=float(statistics.median(means)) if means else float("nan"),
            clipped_fraction=(sum(clipped) / len(clipped)) if clipped else float("nan"),
            dark_dn=(sum(darks) / len(darks)) if darks else float("nan"),
            light_frames=len(means),
        )


# ---------------------------------------------------------------------------
# Request / result
# ---------------------------------------------------------------------------

class OptimizerOutcome(str, enum.Enum):
    COMPLETED = "completed"                 # final scan ran to its end
    SCAN_STOPPED = "scan_stopped"           # final scan ran and was stopped early
    INSUFFICIENT_SIGNAL = "insufficient_signal"
    TOO_BRIGHT = "too_bright"
    NO_CONVERGENCE = "no_convergence"
    CANCELED = "canceled"                   # stopped before the final scan
    ERROR = "error"


@dataclass
class LaserOptimizerRequest:
    """``final_scan`` is the caller's normal scan request (subject, duration,
    masks, sinks). Its masks also pick the probed cameras. The workflow adds
    the dark skip delay and pulse width to its trigger config, the cameras'
    real gains (``camera_gain_map``) and a ``laser_optimizer`` block of
    ``extra_session_meta``; everything else is used as given."""

    final_scan: ScanRequest
    config: OptimizerConfig = field(default_factory=OptimizerConfig)
    # Held around the restore's apply_laser_power, like the app's console
    # mutex (anything with lock()/unlock()). None = no external lock.
    console_lock: Any = None


@dataclass
class LaserOptimizerResult:
    outcome: OptimizerOutcome
    message: str
    probes: list = field(default_factory=list)       # ProbeResult
    setting: Optional[OptimizerSetting] = None       # the accepted setting
    scan_label: Optional[str] = None                 # final scan's DB session label
    scan_error: Optional[str] = None
    restored: bool = False                           # hardware back to stock

    @property
    def scan_ran(self) -> bool:
        return self.scan_label is not None


def summary_text(result: LaserOptimizerResult, cfg: OptimizerConfig) -> str:
    """Plain-text block for scan notes: the chosen setting and the probe log."""
    lines = ["Laser optimizer (engineering): " + (
        f"scanned at {result.setting.describe()}, camera gains "
        f"{'/'.join(str(g) for g in result.setting.camera_gains(cfg.row_gain_ratio))}, "
        f"exposure {cfg.exposure_us:g} µs"
        if result.setting else result.message)]
    lo, hi = cfg.band
    lines.append(f"Target {cfg.target_mean_dn:g} DN ({lo:.0f}-{hi:.0f}) on the near cameras.")
    lines.extend(p.describe() for p in result.probes)
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# The workflow
# ---------------------------------------------------------------------------

class _Canceled(Exception):
    pass


class LaserOptimizerWorkflow:
    """Runs one optimizer pass on a worker thread. One instance per
    :class:`~omotion.MotionInterface.MotionInterface`."""

    def __init__(self, interface) -> None:
        self._interface = interface
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()
        self._running = False
        self.last_result: Optional[LaserOptimizerResult] = None
        # What setup changed, so restore undoes exactly that.
        self._touched_console = False
        self._touched_sides: list = []      # (sensor, mask)

    @property
    def running(self) -> bool:
        with self._lock:
            return self._running

    def start(
        self,
        request: LaserOptimizerRequest,
        *,
        on_probe: Optional[Callable[[ProbeResult, Decision], None]] = None,
        on_scan_started: Optional[Callable[[str, OptimizerSetting], None]] = None,
        on_complete: Optional[Callable[[LaserOptimizerResult], None]] = None,
    ) -> bool:
        """Start a run. Returns False (and logs why) when it cannot start.

        Callbacks fire on the worker thread: ``on_probe(probe, decision)``
        after each probe with what the planner decided next;
        ``on_scan_started(session_label, setting)`` once the final scan is
        running; ``on_complete(result)`` last, after the restore.
        """
        problem = self._preflight(request)
        if problem:
            logger.error("Laser optimizer refused: %s", problem)
            self.last_result = LaserOptimizerResult(
                outcome=OptimizerOutcome.ERROR, message=problem, restored=True)
            return False
        with self._lock:
            if self._running:
                logger.error("Laser optimizer refused: a run is already active")
                return False
            self._running = True
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(
            target=self._worker,
            args=(request, on_probe, on_scan_started, on_complete),
            daemon=True, name="LaserOptimizer",
        )
        self._thread.start()
        return True

    def cancel(self) -> None:
        """Stop the run: cancels a probe or the final scan in progress. The
        worker restores the hardware and calls on_complete. Non-blocking."""
        self._stop_evt.set()

    def await_complete(self, timeout_sec: Optional[float] = None) -> None:
        t = self._thread
        if t is not None:
            t.join(timeout=timeout_sec)

    # -- preflight ----------------------------------------------------------

    def _active_sides(self, request: LaserOptimizerRequest) -> list:
        out = []
        for side, mask in (("left", request.final_scan.left_camera_mask),
                           ("right", request.final_scan.right_camera_mask)):
            sensor = getattr(self._interface, side, None)
            if int(mask) and sensor is not None and sensor.is_connected():
                out.append((side, sensor, int(mask)))
        return out

    def _preflight(self, request: LaserOptimizerRequest) -> Optional[str]:
        """Why this request cannot run, or None. Checks the trigger timing the
        run depends on against the interface's resolved trigger config."""
        cfg = request.config
        if request.final_scan.disable_laser:
            return "the final scan has the laser disabled"
        sw = self._interface.scan_workflow
        if sw.running:
            return "a scan is already running"
        console = self._interface.console
        if console is None or not console.is_connected():
            return "console not connected"
        if getattr(console, "laser_limits_error", None):
            return console.laser_limits_error
        sides = self._active_sides(request)
        if not any(mask & cfg.near_mask for _, _, mask in sides):
            return ("no near camera (positions "
                    + "/".join(str(c + 1) for c in cfg.near_camera_ids)
                    + ") is active on a connected sensor")
        try:
            exposure_rows_for_us(cfg.exposure_us)
        except ValueError as e:
            return str(e)

        trig = self._interface.resolve_trigger_config(self._trigger_override(
            cfg, cfg.max_setting, request.final_scan.trigger_config))
        period = 1e6 / float(trig["TriggerFrequencyHz"])
        delay = float(trig["LaserPulseDelayUsec"])
        longest = emitted_pulse_us(cfg.max_pulse_width_us)
        if delay + cfg.max_pulse_width_us > cfg.exposure_us:
            return (f"exposure {cfg.exposure_us:g} µs does not cover the laser "
                    f"delay {delay:g} µs + pulse {cfg.max_pulse_width_us} µs")
        if delay + cfg.dark_skip_delay_us <= cfg.exposure_us:
            return (f"dark-frame pulse starts at {delay + cfg.dark_skip_delay_us:g} µs, "
                    f"inside the {cfg.exposure_us:g} µs exposure")
        if period - cfg.dark_skip_delay_us < cfg.safety_rate_ll_us:
            return (f"post-dark pulse gap {period - cfg.dark_skip_delay_us:g} µs is "
                    f"under the RATE_LL limit {cfg.safety_rate_ll_us:g} µs")
        if cfg.safety_pulse_width_ul_us <= longest:
            return (f"PULSE_WIDTH_UL {cfg.safety_pulse_width_ul_us:g} µs does not "
                    f"clear the {longest:.0f} µs pulse")
        from omotion.config import validate_trigger_config
        try:
            validate_trigger_config(trig)
        except ValueError as e:
            return f"trigger config: {e}"
        return None

    @staticmethod
    def _trigger_override(cfg: OptimizerConfig, setting: OptimizerSetting,
                          base: Optional[dict] = None) -> dict:
        # The TA times the optical pulse itself (pulse_width register); the
        # trigger's LaserPulseWidthUsec is mirrored so scan records show it.
        return {**(base or {}),
                "LaserPulseSkipDelayUsec": int(cfg.dark_skip_delay_us),
                "LaserPulseWidthUsec": int(setting.pulse_width_us)}

    # -- hardware steps -----------------------------------------------------

    def _apply(self, request: LaserOptimizerRequest, setting: OptimizerSetting) -> None:
        """Write the cameras' exposure/gains and the TA pulse width."""
        cfg = request.config
        rows = exposure_rows_for_us(cfg.exposure_us)
        gains = setting.camera_gains(cfg.row_gain_ratio)
        for side, sensor, mask in self._active_sides(request):
            if (sensor, mask) not in self._touched_sides:
                self._touched_sides.append((sensor, mask))
            failures = write_camera_settings(sensor, mask, exposure_rows=rows, gains=gains)
            if failures:
                raise RuntimeError(f"{side} camera write failed: {', '.join(failures)}")
        write_ta_pulse_width(self._interface.console, setting.pulse_width_us)
        logger.info("Laser optimizer applied %s: exposure %d rows, gains %s",
                    setting.describe(), rows, list(gains))

    def _probe(self, request: LaserOptimizerRequest, setting: OptimizerSetting,
               index: int) -> ProbeResult:
        cfg = request.config
        self._apply(request, setting)
        sink = _ProbeSink(cfg.clip_bin)
        finished = run_collection_scan(
            self._interface.scan_workflow, sink,
            subject_id="_laser_optimizer_probe",
            duration_sec=cfg.probe_duration_sec,
            left_camera_mask=request.final_scan.left_camera_mask,
            right_camera_mask=request.final_scan.right_camera_mask,
            trigger_config=self._trigger_override(
                cfg, setting, request.final_scan.trigger_config),
            stop_evt=self._stop_evt,
            raise_on_error=True,
        )
        if not finished:
            raise _Canceled()
        self._check_interlock("after probe %d" % index)
        near, others = [], []
        for side, _, mask in self._active_sides(request):
            s = SIDES.index(side)
            for cam in range(8):
                if mask & (1 << cam):
                    (near if cam in cfg.near_camera_ids else others).append(
                        sink.reading(s, cam))
        return ProbeResult(index=index, setting=setting, near=near, others=others,
                           clip_fraction_limit=cfg.clip_fraction)

    def _check_interlock(self, when: str) -> None:
        faults = read_safety_faults(self._interface.console)
        if faults is None:
            raise RuntimeError(f"laser safety status unreadable {when}")
        if faults:
            raise RuntimeError(
                f"laser safety interlock latched {when}: {', '.join(faults)} "
                "(console power-cycle clears it)")

    def _run_final_scan(self, request: LaserOptimizerRequest,
                        setting: OptimizerSetting, probes: list,
                        on_scan_started) -> tuple:
        """Start the caller's scan at ``setting`` and wait for it. Returns
        (session label, error, stopped)."""
        cfg = request.config
        gains = list(setting.camera_gains(cfg.row_gain_ratio))
        meta = {
            "laser_optimizer": {
                "near_gain": setting.near_gain,
                "pulse_width_us": setting.pulse_width_us,
                "emitted_pulse_us": round(emitted_pulse_us(setting.pulse_width_us), 1),
                "camera_gains": gains,
                "exposure_us": cfg.exposure_us,
                "dark_skip_delay_us": cfg.dark_skip_delay_us,
                "target_mean_dn": cfg.target_mean_dn,
                "tolerance": cfg.tolerance,
                "safety_pulse_width_ul_us": cfg.safety_pulse_width_ul_us,
                "safety_rate_ll_us": cfg.safety_rate_ll_us,
                "probes": [p.to_dict() for p in probes],
            }
        }
        base = request.final_scan
        final = dataclasses.replace(
            base,
            trigger_config=self._trigger_override(cfg, setting, base.trigger_config),
            camera_gain_map=[gains, gains],
            extra_session_meta={**(base.extra_session_meta or {}), **meta},
        )
        # Re-assert the accepted setting (the last probe may not have been it).
        self._apply(request, setting)
        sw = self._interface.scan_workflow
        if not sw.start_scan(final):
            return None, sw.last_scan_error or "scan refused", False
        label = sw.current_scan_label
        logger.warning("Laser optimizer final scan started at %s (%s s, %s)",
                       setting.describe(), final.duration_sec, label)
        if on_scan_started is not None:
            try:
                on_scan_started(label, setting)
            except Exception:
                logger.exception("on_scan_started callback raised")
        while sw.running:
            sw.await_complete(timeout_sec=0.2)
            if self._stop_evt.is_set() and sw.running:
                sw.cancel_scan()
        return label, sw.last_scan_error, bool(sw.last_scan_canceled)

    def _restore(self, request: LaserOptimizerRequest) -> bool:
        """Undo setup: firmware camera defaults, then the console's laser load
        (TA pulse width + safety limits from the stock set and the console's
        stored config, verified by read-back). Never raises."""
        ok = True
        for sensor, mask in self._touched_sides:
            try:
                failures = write_camera_settings(
                    sensor, mask, exposure_rows=FW_DEFAULT_EXPOSURE_ROWS,
                    gains=FW_DEFAULT_CAMERA_GAINS)
            except Exception as e:
                failures = [str(e)]
            if failures:
                ok = False
                logger.error("Laser optimizer restore: camera defaults not written: %s",
                             ", ".join(failures))
        self._touched_sides = []
        if self._touched_console:
            try:
                laser_ok = bool(self._interface.apply_laser_power(lock=request.console_lock))
            except Exception:
                logger.exception("Laser optimizer restore: apply_laser_power raised")
                laser_ok = False
            if not laser_ok:
                ok = False
                logger.error(
                    "Laser optimizer restore: laser load did not verify; laser scans "
                    "stay refused until a load succeeds (reconnect the console)")
            self._touched_console = False
        logger.warning("Laser optimizer restore %s", "complete" if ok else "INCOMPLETE")
        return ok

    # -- worker -------------------------------------------------------------

    def _worker(self, request, on_probe, on_scan_started, on_complete) -> None:
        cfg = request.config
        probes: list = []
        result = LaserOptimizerResult(outcome=OptimizerOutcome.ERROR,
                                      message="did not run", probes=probes)
        try:
            self._check_interlock("before setup")
            self._touched_console = True
            write_safety_limits(
                self._interface.console,
                pulse_width_ul_us=cfg.safety_pulse_width_ul_us,
                rate_ll_us=cfg.safety_rate_ll_us,
            )
            lo, hi = cfg.band
            logger.warning(
                "Laser optimizer started: target %g DN (%.0f-%.0f), exposure %g µs, "
                "dark skip %d µs", cfg.target_mean_dn, lo, hi, cfg.exposure_us,
                cfg.dark_skip_delay_us)

            decision = plan_next(probes, cfg)
            while decision.action == Action.PROBE:
                if self._stop_evt.is_set():
                    raise _Canceled()
                probe = self._probe(request, decision.setting, len(probes) + 1)
                probes.append(probe)
                decision = plan_next(probes, cfg)
                logger.warning("Laser optimizer %s; next: %s%s (%s)", probe.describe(),
                               decision.action.value,
                               f" {decision.setting.describe()}" if decision.setting else "",
                               decision.reason)
                if on_probe is not None:
                    try:
                        on_probe(probe, decision)
                    except Exception:
                        logger.exception("on_probe callback raised")

            if decision.action != Action.ACCEPT:
                result = LaserOptimizerResult(
                    outcome=OptimizerOutcome(decision.action.value),
                    message=decision.reason, probes=probes)
            else:
                if self._stop_evt.is_set():
                    raise _Canceled()
                result.setting = decision.setting
                label, error, stopped = self._run_final_scan(
                    request, decision.setting, probes, on_scan_started)
                result.scan_label = label
                result.scan_error = error
                if error:
                    result.outcome, result.message = OptimizerOutcome.ERROR, error
                elif stopped:
                    result.outcome = OptimizerOutcome.SCAN_STOPPED
                    result.message = f"scan at {decision.setting.describe()} stopped early"
                else:
                    result.outcome = OptimizerOutcome.COMPLETED
                    result.message = f"scanned at {decision.setting.describe()}"
        except _Canceled:
            result = LaserOptimizerResult(outcome=OptimizerOutcome.CANCELED,
                                          message="stopped before the scan",
                                          probes=probes)
        except Exception as e:
            logger.exception("Laser optimizer failed")
            result = LaserOptimizerResult(outcome=OptimizerOutcome.ERROR,
                                          message=str(e) or type(e).__name__,
                                          probes=probes,
                                          setting=result.setting,
                                          scan_label=result.scan_label)
        finally:
            result.restored = self._restore(request)
            self.last_result = result
            with self._lock:
                self._running = False
            logger.warning("Laser optimizer finished: %s: %s",
                           result.outcome.value, result.message)
            if on_complete is not None:
                try:
                    on_complete(result)
                except Exception:
                    logger.exception("on_complete callback raised")
