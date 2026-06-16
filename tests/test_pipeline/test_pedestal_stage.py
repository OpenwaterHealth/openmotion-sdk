"""PedestalSubtractionStage + SensorPedestals — per-side pedestal applied to mean_raw."""

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.pedestal import SensorPedestals, pedestal_for_fw
from omotion.pipeline.stages.pedestal_sub import PedestalSubtractionStage


def test_pedestal_for_fw_below_1_5_3_returns_64():
    assert pedestal_for_fw((1, 5, 2)) == 64.0
    assert pedestal_for_fw((1, 0, 0)) == 64.0


def test_pedestal_for_fw_1_5_3_or_above_returns_128():
    assert pedestal_for_fw((1, 5, 3)) == 128.0
    assert pedestal_for_fw((1, 6, 0)) == 128.0
    assert pedestal_for_fw((2, 0, 0)) == 128.0


def test_sensor_pedestals_per_side_independent():
    class _FakeSensor:
        def __init__(self, version):
            self.version = version

    peds = SensorPedestals.from_sensors(
        left=_FakeSensor(version=(1, 5, 2)),
        right=_FakeSensor(version=(1, 5, 3)),
    )
    assert peds.left == 64.0
    assert peds.right == 128.0


def test_pedestal_stage_subtracts_per_side_pedestal():
    n = 2
    mean_raw = np.array([
        [[100] * 8, [200] * 8],
        [[50]  * 8, [10]  * 8],
    ], dtype=np.float32)

    batch = FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        mean_raw=mean_raw,
    )

    peds = SensorPedestals(left=64.0, right=128.0)
    PedestalSubtractionStage(pedestals=peds).process(batch)

    # Negative values are valid — frame mean below pedestal is normal noise.
    expected = np.array([
        [[36]   * 8, [72]   * 8],    # 100-64=36,  200-128=72
        [[-14]  * 8, [-118] * 8],    # 50-64=-14,  10-128=-118
    ], dtype=np.float32)
    np.testing.assert_array_equal(batch.subtracted_mean, expected)
    np.testing.assert_array_equal(batch.mean_raw, mean_raw)
