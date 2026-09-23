"""Contact-quality semantics — thresholds, verdict, and the live monitor.

Two consumers share this module:

* :class:`~omotion.ContactQualityWorkflow.ContactQualityWorkflow` — the
  one-shot pre-scan check. For the dark signal, accumulates the *worst*
  (maximum) value seen per camera across a short scan; the light signal
  uses the same rolling-window mean as :class:`ContactQualityMonitor`.
  Rolls both conditions into a single precedence-ordered verdict via
  :func:`evaluate_reason`.
* :class:`ContactQualityMonitor` — the live sink attached to a full scan.
  For the dark signal, tracks only the *current* reading rather than a
  running max — the light-signal rolling window is unchanged from the
  one-shot check. Debounces each condition independently and reports edges
  through a callback.

Both read the same two DN-scale signals off the ``"live"`` channel and apply
the same two predicates, so the *ambient-light* and *poor-contact* verdicts
cannot drift apart between the preflight check and the live monitor.
``REASON_NO_SIGNAL`` is deliberately preflight-only: the live monitor skips
readings from frames that never arrived at all (non-finite with no
``low_light_rt`` corroboration), because total loss of frames belongs to
the consumer's camera-dropout watchdog rather than to contact quality. A
frame that DID arrive but was unlit — DarkCorrectionStage's own
``low_light_rt`` determination — is reported as poor contact instead of
silently dropped; that is the disconnected-fiber case the live monitor
exists to catch (see :class:`ContactQualityMonitor`).

Thresholds are background-subtracted DN. See docs/SciencePipeline.md §11.2
and §11.3.
"""

from __future__ import annotations

import bisect
import collections
import logging
import math
from dataclasses import dataclass
from typing import Callable, Sequence

logger = logging.getLogger("openmotion.sdk.contact_quality")

# Verdict vocabulary. These strings cross the SDK/app boundary — the
# bloodflow-app maps them to operator-facing text — so they are API.
REASON_OK            = "ok"
REASON_POOR_CONTACT  = "poor_contact"
REASON_AMBIENT_LIGHT = "ambient_light"
REASON_NO_SIGNAL     = "no_signal"

# Order of the reasons in a verdict_string() list — fixed so the stored
# value for a given state is always the same string.
_VERDICT_ORDER = (REASON_POOR_CONTACT, REASON_AMBIENT_LIGHT)


def verdict_string(active_reasons) -> str:
    """The recorded contact-quality verdict for one camera: ``"ok"`` when no
    condition is latched, else the latched reasons comma-joined in a fixed
    order (``"poor_contact,ambient_light"``)."""
    active = set(active_reasons)
    reasons = [r for r in _VERDICT_ORDER if r in active]
    return ",".join(reasons) if reasons else REASON_OK


# CameraLatch.observe() return values.
TRANSITION_NONE      = "none"
TRANSITION_ACTIVATED = "activated"
TRANSITION_CLEARED   = "cleared"

_SIDE_NAMES = ("left", "right")

# Camera count per sensor module.
_CAMERAS_PER_SENSOR = 8


@dataclass(frozen=True)
class CQThresholds:
    """Per-camera DN thresholds with fail-open out-of-range behavior.

    ``dark`` is an upper bound — exceeding it means ambient light is leaking
    onto the sensor. ``light`` is a lower bound — falling below it means the
    laser is not coupling into tissue.

    Out-of-range indices — including negatives, which the legacy
    ``_ContactQualitySink`` lookup silently wrapped to the end of the list —
    return a value that can never trip.
    """

    dark:  tuple[float, ...]
    light: tuple[float, ...]

    @classmethod
    def from_sequences(cls, dark: Sequence[float], light: Sequence[float]) -> CQThresholds:
        dark_t = tuple(float(v) for v in dark)
        light_t = tuple(float(v) for v in light)
        for name, config_key, seq in (
            ("dark", "cq_dark_threshold_per_camera", dark_t),
            ("light", "cq_light_threshold_per_camera", light_t),
        ):
            n = len(seq)
            if n < _CAMERAS_PER_SENSOR:
                logger.warning(
                    "CQ %s thresholds (%s) have %d entries, expected %d — "
                    "cameras with index >= %d will fail open (never flagged)",
                    name, config_key, n, _CAMERAS_PER_SENSOR, n,
                )
            elif n > _CAMERAS_PER_SENSOR:
                logger.warning(
                    "CQ %s thresholds (%s) have %d entries, expected %d — "
                    "entries past index %d are ignored",
                    name, config_key, n, _CAMERAS_PER_SENSOR, _CAMERAS_PER_SENSOR - 1,
                )
        return cls(dark=dark_t, light=light_t)

    def dark_for(self, cam_id: int) -> float:
        return self.dark[cam_id] if 0 <= cam_id < len(self.dark) else math.inf

    def light_for(self, cam_id: int) -> float:
        return self.light[cam_id] if 0 <= cam_id < len(self.light) else 0.0


def is_ambient_light(dark_dn: float, thresholds: CQThresholds, cam_id: int) -> bool:
    """True when a dark-frame DN reading exceeds the camera's dark threshold."""
    return math.isfinite(dark_dn) and dark_dn > thresholds.dark_for(cam_id)


def is_poor_contact(light_dn: float, thresholds: CQThresholds, cam_id: int) -> bool:
    """True when a light-frame DN reading falls below the camera's light threshold."""
    return math.isfinite(light_dn) and light_dn < thresholds.light_for(cam_id)


def evaluate_reason(
    *,
    light_avg: float,
    dark_max: float,
    thresholds: CQThresholds,
    cam_id: int,
) -> str:
    """Single precedence-ordered verdict for one camera.

    no_signal > ambient_light > poor_contact > ok. Used by the one-shot
    check; the live monitor uses the two predicates directly because a
    camera can be ambient-lit and poorly coupled at the same time and the
    UI renders those as separate rows.
    """
    if not math.isfinite(light_avg):
        return REASON_NO_SIGNAL
    if is_ambient_light(dark_max, thresholds, cam_id):
        return REASON_AMBIENT_LIGHT
    if is_poor_contact(light_avg, thresholds, cam_id):
        return REASON_POOR_CONTACT
    return REASON_OK


class CameraLatch:
    """Debounced edge detector for one camera / one condition.

    The two edges are debounced independently. RAISE flips only after
    ``activate_debounce`` consecutive *bad* observations; CLEAR flips only
    after ``clear_debounce`` consecutive *good* ones. Any disagreeing
    observation resets the running streak. ``activate_debounce=1`` /
    ``clear_debounce=1`` reproduces the legacy immediate latch/clear.

    The asymmetry is deliberate: a contact-quality warning should RAISE
    quickly (a late warning is a safety miss) but CLEAR conservatively (a
    premature dismiss strands the operator on a still-bad camera), so
    callers give a short activate debounce and a longer clear debounce.

    Consequence: time-to-transition is unbounded — a spurious disagreeing
    observation arriving more often than once per (respective) debounce
    postpones that edge indefinitely. With the asymmetric defaults this
    matters most on the clear edge, which is the conservative direction.

    Returns a transition only on an edge — steady state returns
    ``TRANSITION_NONE`` so callers emit one event per genuine change
    rather than once per frame.
    """

    __slots__ = ("_activate_debounce", "_clear_debounce", "_active", "_streak")

    def __init__(self, activate_debounce: int = 1, clear_debounce: int = 1) -> None:
        self._activate_debounce = max(1, int(activate_debounce))
        self._clear_debounce = max(1, int(clear_debounce))
        self._active = False
        self._streak = 0

    @property
    def active(self) -> bool:
        return self._active

    @property
    def activate_debounce(self) -> int:
        """Consecutive bad observations required to RAISE — always >= 1."""
        return self._activate_debounce

    @property
    def clear_debounce(self) -> int:
        """Consecutive good observations required to CLEAR — always >= 1."""
        return self._clear_debounce

    def observe(self, bad: bool) -> str:
        """Feed one observation; return activated / cleared / none."""
        bad = bool(bad)
        if bad == self._active:
            self._streak = 0
            return TRANSITION_NONE
        self._streak += 1
        # Approaching RAISE when bad, CLEAR when good — each edge its own bound.
        needed = self._activate_debounce if bad else self._clear_debounce
        if self._streak < needed:
            return TRANSITION_NONE
        self._active = bad
        self._streak = 0
        return TRANSITION_ACTIVATED if self._active else TRANSITION_CLEARED


def _cams_from_masks(meta) -> set:
    """Set of (side, cam_id) the scan actually uses.

    Empty when ``meta`` is None (or carries no masks), which the monitor
    treats as "evaluate everything".
    """
    if meta is None:
        return set()
    cams = set()
    for side, attr in (("left", "left_camera_mask"), ("right", "right_camera_mask")):
        mask = int(getattr(meta, attr, 0) or 0)
        for cam_id in range(_CAMERAS_PER_SENSOR):
            if mask & (1 << cam_id):
                cams.add((side, cam_id))
    return cams


class ContactQualityMonitor:
    """Live contact-quality sink — reports per-camera transitions mid-scan.

    Attach by appending an instance to ``ScanRequest.sinks``. Subscribes to
    the ``"live"`` channel and reads the same two DN signals as the one-shot
    check:

      * ``frame_type == "dark"`` rows -> ``subtracted_mean``
        (``mean_raw - pedestal``), measuring ambient light against the
        zero-light pedestal.
      * every other non-warmup/non-stale row -> ``mean_dc_rt``
        (``mean_raw - predicted_dark_baseline``), measuring laser-driven
        signal above the just-measured dark.

    A non-finite ``mean_dc_rt`` is handled two ways. A warmup/never-arrived
    frame carries no signal and is skipped, left to the consumer's
    camera-dropout watchdog. A frame that arrived but was unlit
    (``low_light_rt=True`` — a covered/lifted sensor or a decoupled fiber)
    is reported as ``poor_contact`` — the disconnected-fiber case issue
    #364 was filed about. That case is reported via ``subtracted_mean``,
    NOT run through :func:`is_poor_contact`, because ``subtracted_mean`` is
    pedestal-referenced while the light threshold was calibrated against
    dark-baseline-referenced ``mean_dc_rt``, so comparing the two would be
    apples-to-oranges. See :meth:`consume` for the full rationale.

    **Dark-signal accumulation differs deliberately from the one-shot
    check; the light-signal rolling window does not.** The check keeps the
    worst (maximum) dark reading seen across its short window; this keeps
    only the *current* dark reading, because a running dark max would latch
    an ambient warning for the remaining hours of a free-run scan. Both
    consumers average light readings over an identical
    ``deque(maxlen=rolling_window)`` and threshold the mean the same way.

    ``on_transition(side, cam_id, reason, value, active)`` fires on edges
    only. ``reason`` is ``REASON_AMBIENT_LIGHT`` or ``REASON_POOR_CONTACT``;
    ``REASON_NO_SIGNAL`` is deliberately never reported here — total loss of
    frames is the camera-dropout watchdog's job, and reporting it as poor
    contact would send the operator to fix the wrong thing.

    ``light_debounce`` and ``dark_debounce`` are frame counts on two
    DIFFERENT clocks — not seconds, and not the same clock as each other.
    ``light_debounce`` counts consecutive light-frame observations, which
    arrive at the ~40 Hz capture rate (so the default of 80 is roughly 2 s).
    ``dark_debounce`` counts consecutive dark-frame observations, which are
    scheduled roughly every ``dark_interval`` frames — about 15 s apart at
    the default. Do not tune one against the other assuming a shared clock.

    The callback is invoked on the pipeline runner thread. A GUI consumer
    must marshal to its own thread. Exceptions from the callback are logged
    and swallowed so a broken consumer cannot disable the sink.

    **Verdict history (bloodflow-app#589).** Every edge is also recorded
    against the ``abs_frame_id`` of the frame that caused it, so
    :meth:`verdict_at` can answer "what was this camera's latched verdict at
    frame N" after the fact. :class:`~omotion.pipeline.sinks.ScanDBSink`
    uses it to stamp each corrected row, which it writes up to one dark
    interval (~15 s) after the live frame went by — so it needs the verdict
    as of that frame, not the current one. :meth:`observed_through` tells
    it how far the history is complete. Both are read on the runner
    thread, like everything else here.
    """

    channels = frozenset({"live"})

    def __init__(
        self,
        *,
        thresholds: CQThresholds,
        on_transition: Callable[[str, int, str, float, bool], None],
        rolling_window: int = 10,
        light_activate_debounce: int = 10,
        light_clear_debounce: int = 80,
        dark_debounce: int = 1,
    ) -> None:
        self._thresholds = thresholds
        self._on_transition = on_transition
        self._window_size = max(1, int(rolling_window))
        # Asymmetric: RAISE fast (a late warning is a safety miss), CLEAR
        # slow (a premature dismiss strands the operator). Dark/ambient stays
        # symmetric — scheduled darks are ~15 s apart, their own debounce.
        self._light_activate_debounce = max(1, int(light_activate_debounce))
        self._light_clear_debounce = max(1, int(light_clear_debounce))
        self._dark_debounce = max(1, int(dark_debounce))
        # (side, cam_id) -> deque[float] of recent light-frame mean_dc_rt
        self._light_window: dict = {}
        # (side, cam_id, reason) -> CameraLatch
        self._latches: dict = {}
        # (side, cam_id) pairs in the scan mask; empty means "all"
        self._active_cams: set = set()
        # (side, cam_id) -> ([abs_frame_id, ...], [verdict_string, ...]),
        # one entry per edge, ascending frame id.
        self._history: dict = {}
        # Highest abs_frame_id seen on "live"; -1 before the first batch.
        self._observed_through = -1
        # on_complete() summary counters — reset per scan in on_scan_start;
        # see on_complete's docstring for why they exist.
        self._transitions_emitted = 0
        self._observations_processed = 0
        self._cameras_seen: set = set()

    def on_scan_start(self, meta) -> None:
        self._light_window.clear()
        self._latches.clear()
        self._active_cams = _cams_from_masks(meta)
        self._history = {}
        self._observed_through = -1
        self._transitions_emitted = 0
        self._observations_processed = 0
        self._cameras_seen = set()
        if meta is None:
            mask_desc = "no metadata (evaluating all cameras)"
        else:
            left_mask = int(getattr(meta, "left_camera_mask", 0) or 0)
            right_mask = int(getattr(meta, "right_camera_mask", 0) or 0)
            mask_desc = f"left=0x{left_mask:02X} right=0x{right_mask:02X}"
        # Deliberate tripwire: this line is the evidence that live
        # contact-quality monitoring is actually attached. Its absence is
        # how the feature stayed silently dead for two months in 2026.
        logger.info(
            "live contact-quality monitor attached: %s, "
            "dark<=%s DN, light>=%s DN, window=%d, "
            "light debounce activate=%d clear=%d, dark debounce=%d",
            mask_desc,
            list(self._thresholds.dark) or "n/a",
            list(self._thresholds.light) or "n/a",
            self._window_size,
            self._light_activate_debounce,
            self._light_clear_debounce,
            self._dark_debounce,
        )

    def consume(self, channel: str, batch) -> None:
        if channel != "live":
            return
        fids = batch.abs_frame_ids if batch.abs_frame_ids is not None else batch.frame_ids
        try:
            self._consume_rows(batch, fids)
        finally:
            # Advance even when the batch carried nothing to evaluate: the
            # history is just as complete through these frames.
            if fids is not None and len(fids):
                # Stale rows carry epoch-shifted advisory ids; don't let one
                # push the watermark past frames not yet observed.
                ft = batch.frame_type
                best = self._observed_through
                for i in range(len(fids)):
                    if ft is not None and str(ft[i]) == "stale":
                        continue
                    best = max(best, int(fids[i]))
                self._observed_through = best

    def _consume_rows(self, batch, fids) -> None:
        if batch.subtracted_mean is None or batch.mean_dc_rt is None:
            return
        low_light_rt = batch.low_light_rt
        for i, side_idx, cam_id, ft in batch.iter_rows(exclude={"warmup", "stale"}):
            if (not (0 <= side_idx < len(_SIDE_NAMES))
                    or not (0 <= cam_id < _CAMERAS_PER_SENSOR)):
                continue
            side = _SIDE_NAMES[side_idx]
            key = (side, cam_id)
            if self._active_cams and key not in self._active_cams:
                continue
            if ft == "dark":
                value = float(batch.subtracted_mean[i, side_idx, cam_id])
                if not math.isfinite(value):
                    continue
                self._observe(
                    int(fids[i]), side, cam_id, REASON_AMBIENT_LIGHT,
                    is_ambient_light(value, self._thresholds, cam_id),
                    value, self._dark_debounce, self._dark_debounce,
                )
            else:
                value = float(batch.mean_dc_rt[i, side_idx, cam_id])
                if not math.isfinite(value):
                    # NaN means two different things here. A warmup row
                    # carries no signal at all and belongs to the
                    # consumer's dropout watchdog. But DarkCorrectionStage
                    # also suppresses realtime emission for a frame that
                    # ARRIVED and was unlit (low_light_rt) — a covered
                    # sensor, a lifted sensor, a decoupled fiber. That is
                    # total contact loss: the strongest poor-contact
                    # evidence the pipeline produces, and the case issue
                    # #364 (fiber disconnect not detected) was filed about.
                    if low_light_rt is None or not bool(low_light_rt[i, side_idx, cam_id]):
                        continue
                    value = float(batch.subtracted_mean[i, side_idx, cam_id])
                    if not math.isfinite(value):
                        continue
                    # low_light_rt is DarkCorrectionStage's own
                    # determination — the same dark_like test used to
                    # classify real dark frames — that this frame received
                    # no light at all. That is stronger, more direct
                    # evidence than any threshold comparison, so report it
                    # unconditionally rather than running subtracted_mean
                    # through is_poor_contact: subtracted_mean is
                    # pedestal-referenced while the light threshold was
                    # calibrated against dark-baseline-referenced
                    # mean_dc_rt, so comparing the two would be
                    # apples-to-oranges. Kept out of the rolling window for
                    # the same reason — averaging it in would mix
                    # reference frames.
                    self._observe(
                        int(fids[i]), side, cam_id, REASON_POOR_CONTACT,
                        True,
                        value,
                        self._light_activate_debounce,
                        self._light_clear_debounce,
                    )
                    continue
                window = self._light_window.get(key)
                if window is None:
                    window = collections.deque(maxlen=self._window_size)
                    self._light_window[key] = window
                window.append(value)
                avg = sum(window) / len(window)
                self._observe(
                    int(fids[i]), side, cam_id, REASON_POOR_CONTACT,
                    is_poor_contact(avg, self._thresholds, cam_id),
                    avg,
                    self._light_activate_debounce,
                    self._light_clear_debounce,
                )

    def _observe(self, fid, side, cam_id, reason, bad, value,
                 activate_debounce, clear_debounce) -> None:
        self._observations_processed += 1
        self._cameras_seen.add((side, cam_id))
        latch_key = (side, cam_id, reason)
        latch = self._latches.get(latch_key)
        if latch is None:
            latch = CameraLatch(activate_debounce, clear_debounce)
            self._latches[latch_key] = latch
        transition = latch.observe(bad)
        if transition == TRANSITION_NONE:
            return
        self._transitions_emitted += 1
        active = transition == TRANSITION_ACTIVATED
        self._record(fid, side, cam_id)
        logger.info(
            "live CQ %s%d: %s %s (%.2f DN)",
            "L" if side == "left" else "R", cam_id + 1,
            reason, "RAISED" if active else "CLEARED", value,
        )
        try:
            self._on_transition(side, cam_id, reason, value, active)
        except Exception:
            # NOT "the runner disables a sink that raises" — _safe_consume
            # (runner.py) logs and continues; it does not disable anything.
            # Only an on_scan_start failure does that. The real risk here is
            # narrower but still real: an uncaught exception would unwind
            # this whole consume() call, abandoning every remaining row in
            # the batch after the one that raised (silently skipping the
            # other cameras), and would log a fresh traceback per batch —
            # at ~40 Hz — for as long as a broken callback keeps raising.
            logger.exception("contact-quality transition callback raised")

    def _record(self, fid: int, side: str, cam_id: int) -> None:
        verdict = verdict_string(
            r for r in _VERDICT_ORDER
            if (latch := self._latches.get((side, cam_id, r))) is not None
            and latch.active
        )
        fids, verdicts = self._history.setdefault((side, cam_id), ([], []))
        if fids and fids[-1] >= fid:
            # Same frame (or an out-of-order id): the later edge wins.
            fids[-1], verdicts[-1] = max(fids[-1], fid), verdict
        else:
            fids.append(fid)
            verdicts.append(verdict)

    def observed_through(self) -> int:
        """Highest ``abs_frame_id`` consumed from the live channel (-1 before
        the first batch). :meth:`verdict_at` is final for any frame at or
        below it."""
        return self._observed_through

    def verdict_at(self, side: str, cam_id: int, frame_id: int):
        """Latched verdict for one camera as of ``frame_id`` — see
        :func:`verdict_string` — or None for a camera outside the scan mask.
        A monitored camera with no edge yet reads ``"ok"``."""
        if self._active_cams and (side, cam_id) not in self._active_cams:
            return None
        entry = self._history.get((side, cam_id))
        if not entry:
            return REASON_OK
        fids, verdicts = entry
        i = bisect.bisect_right(fids, int(frame_id)) - 1
        return verdicts[i] if i >= 0 else REASON_OK

    def on_complete(self) -> None:
        # A silent scan is ambiguous without this: "contact was good
        # throughout" and "the sink was attached but nothing ever reached
        # the predicates" look identical from the absence of transitions
        # alone — this feature was dead for two months in 2026 precisely
        # because that silence looked like success.
        logger.info(
            "live contact-quality monitor: %d transition(s) emitted, "
            "%d camera(s) observed, %d observation(s) processed",
            self._transitions_emitted, len(self._cameras_seen),
            self._observations_processed,
        )
