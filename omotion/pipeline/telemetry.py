"""Per-frame console telemetry stamping.

Restores the per-frame ``pdc`` / ``tcm`` / ``tcl`` stamping that was removed
in 9a2d8e0 ("drop telemetry stage/source/sink for now"). Three pieces:

- ``TelemetrySample`` — one (wall-clock timestamp, pdc, tcm, tcl) tuple.
- ``TelemetryAggregator`` — thread-safe ring buffer. ``update()`` is called
  from the ConsoleTelemetryPoller thread (via the feeder listener that
  ScanWorkflow registers for the duration of a scan); ``snapshot_at()`` is
  called from the pipeline/runner thread by TelemetryIngestStage.
- ``TelemetryIngestStage`` — stamps each frame with the most recent
  telemetry sample at-or-before the frame's capture time.

Clock domains: frame timestamps are sensor-firmware clock normalised to
scan start (t=0 at the first frame); telemetry samples carry host
``time.time()``. The stage bridges the two by capturing
``wall_offset = time.time() - first_frame_ts`` when it sees its first
non-empty batch — the batch is processed within one source flush interval
(~0.25 s) of capture, so the alignment error is bounded by that latency.
Telemetry is ~10 Hz context data (laser power, trigger counters), so
sub-poll-interval alignment is not required.

When no aggregator is supplied (replay sources, tests), the stage is a
no-op and ``FrameBatch.pdc/tcm/tcl`` keep whatever the source set.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from .batch import FrameBatch

logger = logging.getLogger("openmotion.sdk.pipeline.telemetry")

_DEFAULT_RING_SIZE = 256   # ~25 s of history at the poller's ~10 Hz


@dataclass(frozen=True)
class TelemetrySample:
    """One console-telemetry observation, host wall-clock stamped."""
    timestamp_s: float   # host time.time() when the poll completed
    pdc_ma: float        # photodiode current, mA
    tcm: int             # MCU trigger (lsync) count
    tcl: int             # laser trigger count


class TelemetryAggregator:
    """Thread-safe ring buffer of TelemetrySamples.

    ``update()`` runs on the poller thread; ``snapshot_at()`` on the
    runner thread. Owned by ScanWorkflow (one per scan) — the ingest
    stage deliberately does NOT clear it on reset(), so telemetry history
    survives a dropped batch.
    """

    def __init__(self, maxlen: int = _DEFAULT_RING_SIZE) -> None:
        self._lock = threading.Lock()
        self._samples: deque[TelemetrySample] = deque(maxlen=max(2, int(maxlen)))

    def update(self, sample: TelemetrySample) -> None:
        with self._lock:
            self._samples.append(sample)

    def snapshot_at(self, t_wall: float) -> Optional[TelemetrySample]:
        """Most recent sample with ``timestamp_s <= t_wall``, else None.

        Scans from the newest end — frames query near-now, so the hit is
        almost always within the first couple of entries.
        """
        with self._lock:
            for sample in reversed(self._samples):
                if sample.timestamp_s <= t_wall:
                    return sample
        return None

    def __len__(self) -> int:
        with self._lock:
            return len(self._samples)


class TelemetryFeeder:
    """ConsoleTelemetryPoller listener → TelemetryAggregator adapter.

    Registered by ScanWorkflow for the duration of one scan (same pattern
    as the telemetry CSV writer). Runs on the poller thread — must stay
    non-blocking. ``close()`` unregisters; safe to call twice.
    """

    def __init__(self, aggregator: TelemetryAggregator, poller) -> None:
        self._aggregator = aggregator
        self._poller = poller
        poller.add_listener(self)

    def __call__(self, snap) -> None:
        try:
            self._aggregator.update(TelemetrySample(
                timestamp_s=float(snap.timestamp),
                pdc_ma=float(snap.pdc),
                tcm=int(snap.tcm),
                tcl=int(snap.tcl),
            ))
        except Exception:
            logger.exception("telemetry feeder failed to ingest snapshot")

    def close(self) -> None:
        try:
            self._poller.remove_listener(self)
        except Exception:
            pass


class TelemetryIngestStage:
    """Stamp each frame with the telemetry context at its capture time.

    Writes ``batch.pdc`` (float32, NaN when no sample), ``batch.tcm`` and
    ``batch.tcl`` (int64, 0 when no sample). Placed right after
    FrameClassificationStage and BEFORE Tee("raw"), so the raw CSV's
    telemetry columns carry the stamped values.

    ``reset()`` clears the wall-clock offset (a new scan re-anchors) but
    deliberately does NOT clear the aggregator — telemetry history is
    owned by the scan, not the pipeline pass.
    """

    name = "telemetry_ingest"

    def __init__(self, aggregator: Optional[TelemetryAggregator] = None, *,
                 now: Callable[[], float] = time.time) -> None:
        self._aggregator = aggregator
        self._now = now            # injectable for tests
        self._wall_offset: Optional[float] = None

    def process(self, batch: FrameBatch) -> FrameBatch:
        agg = self._aggregator
        if agg is None:
            return batch
        n = batch.timestamp_s.shape[0] if batch.timestamp_s is not None else 0
        if n == 0:
            return batch

        if self._wall_offset is None:
            # First non-empty batch: anchor scan-relative frame time to the
            # host clock. Error is bounded by the source's flush latency.
            self._wall_offset = self._now() - float(batch.timestamp_s[0])

        pdc = np.full(n, np.nan, dtype=np.float32)
        tcm = np.zeros(n, dtype=np.int64)
        tcl = np.zeros(n, dtype=np.int64)
        for i in range(n):
            sample = agg.snapshot_at(float(batch.timestamp_s[i]) + self._wall_offset)
            if sample is not None:
                pdc[i] = sample.pdc_ma
                tcm[i] = sample.tcm
                tcl[i] = sample.tcl
        batch.pdc = pdc
        batch.tcm = tcm
        batch.tcl = tcl
        return batch

    def reset(self) -> None:
        self._wall_offset = None
