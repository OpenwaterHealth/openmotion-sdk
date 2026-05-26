"""Per-stage perf budgets.

Run with:
    pytest tests/test_pipeline/benchmarks/ --benchmark-only -v

Budgets are calibrated to a representative dev laptop (Windows 11,
mid-range CPU). Tighten budgets here when the target machine is known.
Flag any drift > 20% in CI by re-running and comparing against baseline.

Stages covered:
    MomentsStage            — most expensive; vectorized einsum
    NoiseFloorStage         — in-place putmask; very cheap
    ShotNoiseCorrectionStage — vectorised; cheap
"""

from __future__ import annotations

import numpy as np
import pytest

from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.moments import MomentsStage
from omotion.pipeline.stages.noise_floor import NoiseFloorStage
from omotion.pipeline.stages.shot_noise import ShotNoiseCorrectionStage


# Representative batch size: 2.5 s at 40 Hz
N = 100


def _full_batch(n: int = N) -> FrameBatch:
    """Synthesize a fully-populated N-frame batch for benchmarking."""
    rng = np.random.default_rng(0)
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=rng.integers(0, 1000, (n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.linspace(0.0, 2.5, n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


# ---------------------------------------------------------------------------
# MomentsStage
# ---------------------------------------------------------------------------

def test_moments_stage_under_50ms(benchmark) -> None:
    """Budget: 50 ms per batch-of-100 (was ~150 ms in the Python-loop legacy path).

    The vectorised einsum implementation should stay comfortably under this
    even on a slow laptop. Tighten to 20 ms once we have a stable baseline.
    """
    stage = MomentsStage()
    batch = _full_batch(N)

    benchmark(stage.process, batch)

    assert benchmark.stats["mean"] < 0.050, (
        f"MomentsStage too slow: mean={benchmark.stats['mean']*1000:.1f} ms "
        f"(budget 50 ms)"
    )


# ---------------------------------------------------------------------------
# NoiseFloorStage
# ---------------------------------------------------------------------------

def test_noise_floor_stage_under_5ms(benchmark) -> None:
    """Budget: 5 ms per batch-of-100 (in-place putmask should be ~1 ms)."""
    stage = NoiseFloorStage(threshold=10)
    batch = _full_batch(N)

    benchmark(stage.process, batch)

    assert benchmark.stats["mean"] < 0.005, (
        f"NoiseFloorStage too slow: mean={benchmark.stats['mean']*1000:.1f} ms "
        f"(budget 5 ms)"
    )


# ---------------------------------------------------------------------------
# ShotNoiseCorrectionStage
# ---------------------------------------------------------------------------

def test_shot_noise_stage_under_5ms(benchmark) -> None:
    """Budget: 5 ms per batch-of-100 (vectorised variance arithmetic)."""
    from omotion.pipeline.pedestal import SensorPedestals
    stage = ShotNoiseCorrectionStage(
        pedestals=SensorPedestals(left=64.0, right=64.0),
        camera_gain_map=np.array([16, 4, 2, 1, 1, 2, 4, 16], dtype=np.float32),
    )
    batch = _full_batch(N)
    # ShotNoiseCorrectionStage reads mean_dc_rt and std_dc_rt
    batch.mean_dc_rt = np.full((N, 2, 8), 500.0, dtype=np.float32)
    batch.std_dc_rt  = np.full((N, 2, 8), 20.0,  dtype=np.float32)

    benchmark(stage.process, batch)

    assert benchmark.stats["mean"] < 0.005, (
        f"ShotNoiseCorrectionStage too slow: mean={benchmark.stats['mean']*1000:.1f} ms "
        f"(budget 5 ms)"
    )
