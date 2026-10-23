"""Synthetic camera histograms from a known physical model, and a runner that
pushes them through the production pipeline.

The model is the one the pipeline's corrections assume:

* fully developed speckle: per-pixel intensity is gamma-distributed with
  contrast K (std / mean of the intensity);
* Poisson shot noise on the photo-electrons. Gamma intensity mixed with
  Poisson counting is exactly a negative binomial with r = 1/K²;
* a conversion of ``dn_per_e`` DN per electron (ADC gain × analog gain);
* Gaussian read noise of ``read_sigma`` DN around a dark ``offset`` (DN).

``expected_histogram`` returns the *expected* bin counts for a frame of
``n_pixels`` pixels rather than a random draw, so a test sees no sampling
noise: any difference between the pipeline's output and the model's true
contrast and mean is the pipeline's own error. Each electron count's read
noise is a Gaussian sampled on the integer DN grid; for read_sigma >= 1 its
variance is read_sigma² to high precision whatever the (continuous) centre,
so the read-noise and quantization terms cancel exactly between a light frame
and a dark frame, as the dark subtraction assumes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.factory import default_pipeline
from omotion.pipeline.pedestal import SensorPedestals
from omotion.pipeline.runner import ScanRunner
from omotion.pipeline.sinks import ScanMetadata

N_PIXELS = 1920 * 1280
FRAME_PERIOD_S = 0.025
_REF_OFFSET = 127
_BINS = np.arange(1024, dtype=np.float64)
_lgamma = np.vectorize(math.lgamma, otypes=[np.float64])


def expected_histogram(*, mean_e: float, contrast: float, dn_per_e: float,
                       read_sigma: float, offset: float,
                       n_pixels: int = N_PIXELS) -> np.ndarray:
    """Expected 1024-bin histogram (uint32 counts) of one frame.

    ``mean_e = 0`` gives a dark frame: read noise around ``offset`` only.
    """
    if mean_e > 0:
        r = 1.0 / contrast ** 2
        p = r / (r + mean_e)
        e_max = int(mean_e * (1 + 12 * contrast) + 12 * math.sqrt(mean_e) + 50)
        e = np.arange(e_max + 1, dtype=np.float64)
        log_pmf = (_lgamma(e + r) - math.lgamma(r) - _lgamma(e + 1)
                   + r * math.log(p) + e * math.log1p(-p))
        weights = np.exp(log_pmf)
        assert abs(weights.sum() - 1.0) < 1e-9, "electron range truncated"
        # Drop the far tail that holds under 0.1 expected pixels in total.
        tail = np.cumsum(weights[::-1])[::-1]
        keep = int(np.nonzero(tail * n_pixels >= 0.1)[0][-1]) + 1
        e, weights = e[:keep], weights[:keep]
    else:
        e = np.zeros(1)
        weights = np.ones(1)

    centres = offset + dn_per_e * e
    # Beyond 6 sigma the Gaussian mass per frame rounds to zero counts.
    assert centres[-1] + 6 * read_sigma < 1023, "histogram would clip at 1023"
    assert offset - 6 * read_sigma > 0, "histogram would clip at 0"

    prob = np.zeros(1024)
    for lo in range(0, len(e), 512):
        c = centres[lo:lo + 512, None]
        kern = np.exp(-0.5 * ((_BINS[None, :] - c) / read_sigma) ** 2)
        kern /= kern.sum(axis=1, keepdims=True)
        prob += weights[lo:lo + 512] @ kern
    return np.rint(prob * n_pixels).astype(np.uint32)


@dataclass
class Camera:
    """One camera's model parameters. ``mean_dn`` is the signal above the
    dark offset in DN; 0 makes every frame dark (a covered camera)."""
    cam_id: int
    dn_per_e: float
    read_sigma: float
    mean_dn: float
    contrast: float = 0.3
    light_offset_excess: float = 0.0   # extra DN on light frames (covered-camera jitter)
    light_sigma_excess: float = 0.0    # extra read sigma on light frames

    @property
    def mean_e(self) -> float:
        return self.mean_dn / self.dn_per_e


@dataclass
class Scan:
    cameras: list[Camera]
    n_frames: int = 61
    discard_count: int = 2
    dark_interval: int = 20
    dark_offset: Callable[[int], float] = lambda n: 127.0
    reduced_mode: bool = False
    noise_floor_threshold: int = 10
    pedestal: float = 128.0
    _cache: dict = field(default_factory=dict, repr=False)

    def is_dark(self, n: int) -> bool:
        d = self.discard_count
        return n == d + 1 or (n > d + 1 and (n - 1) % self.dark_interval == 0)

    def histogram(self, cam: Camera, n: int) -> np.ndarray:
        offset = self.dark_offset(n)
        mean_e, sigma = cam.mean_e, cam.read_sigma
        if self.is_dark(n):
            mean_e = 0.0
        elif cam.mean_dn == 0:  # covered camera on a light frame
            offset += cam.light_offset_excess
            sigma += cam.light_sigma_excess
        # A whole-DN change of offset only shifts the histogram, so compute
        # once per fractional offset and roll; this keeps drift scans cheap.
        whole = math.floor(offset)
        frac = offset - whole
        key = (cam.cam_id, mean_e, sigma, round(frac, 9))
        if key not in self._cache:
            self._cache[key] = expected_histogram(
                mean_e=mean_e, contrast=cam.contrast, dn_per_e=cam.dn_per_e,
                read_sigma=sigma, offset=_REF_OFFSET + frac)
        base = self._cache[key]
        shift = whole - _REF_OFFSET
        wrapped = base[-shift:] if shift > 0 else base[:-shift] if shift < 0 else base[:0]
        assert not wrapped.any(), "offset shift would wrap the histogram"
        return np.roll(base, shift)


class _ListSource:
    def __init__(self, metadata: ScanMetadata, batches: list[FrameBatch]):
        self.metadata = metadata
        self._batches = batches

    def __iter__(self):
        return iter(self._batches)

    def close(self) -> None:
        pass


class _Collector:
    channels = {"final", "live"}

    def __init__(self):
        self.final = []   # EnrichedCorrectedFrames, per camera and cam_id=-1
        self.live = []    # FrameBatches from the realtime path

    def on_scan_start(self, meta) -> None:
        pass

    def on_complete(self) -> None:
        pass

    def consume(self, channel, payload) -> None:
        if channel == "final":
            self.final.extend(payload.frames)
        else:
            self.live.append(payload)


def run(scan: Scan, calibration) -> _Collector:
    """Replay ``scan`` (left side only, one packet per capture) through
    ``default_pipeline()``; return the collected "final" frames and "live"
    batches."""
    mask = sum(1 << c.cam_id for c in scan.cameras)
    meta = ScanMetadata(scan_id="synthetic", subject_id="s", operator="o",
                        started_at_iso="", duration_sec=0,
                        left_camera_mask=mask, right_camera_mask=0,
                        reduced_mode=scan.reduced_mode)
    batches = []
    for n in range(1, scan.n_frames + 1):
        rows = len(scan.cameras)
        hist = np.zeros((rows, 2, 8, 1024), dtype=np.uint32)
        for i, cam in enumerate(scan.cameras):
            hist[i, 0, cam.cam_id] = scan.histogram(cam, n)
        batches.append(FrameBatch(
            cam_ids=np.array([c.cam_id for c in scan.cameras], dtype=np.int8),
            frame_ids=np.full(rows, n & 0xFF, dtype=np.uint8),
            packet_ids=np.full(rows, n, dtype=np.int64),
            side_ids=np.zeros(rows, dtype=np.int8),
            raw_histograms=hist,
            temperature_c=np.full((rows, 2, 8), 40.0, dtype=np.float32),
            timestamp_s=np.full(rows, (n - 1) * FRAME_PERIOD_S),
            pdc=np.full(rows, np.nan, dtype=np.float32),
            tcm=np.zeros(rows, dtype=np.int64), tcl=np.zeros(rows, dtype=np.int64),
        ))
    sink = _Collector()
    pipeline = default_pipeline(
        metadata=meta, calibration=calibration,
        pedestals=SensorPedestals(left=scan.pedestal, right=scan.pedestal),
        noise_floor_threshold=scan.noise_floor_threshold,
        discard_count=scan.discard_count, dark_interval=scan.dark_interval,
        raw_save_max_duration_s=0,
    )
    ScanRunner(source=_ListSource(meta, batches), pipeline=pipeline,
               sinks=[sink]).run()
    return sink
