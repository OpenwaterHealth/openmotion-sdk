"""Per-frame telemetry stamping — TelemetryAggregator / Feeder / IngestStage."""

from types import SimpleNamespace

import numpy as np
import pytest

from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.telemetry import (
    TelemetryAggregator,
    TelemetryFeeder,
    TelemetryIngestStage,
    TelemetrySample,
)


def _batch(timestamps):
    n = len(timestamps)
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        side_ids=np.zeros(n, dtype=np.int8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.array(timestamps, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


# ── TelemetryAggregator ─────────────────────────────────────────────────────

def test_aggregator_empty_returns_none():
    agg = TelemetryAggregator()
    assert agg.snapshot_at(1000.0) is None


def test_aggregator_returns_most_recent_at_or_before():
    agg = TelemetryAggregator()
    agg.update(TelemetrySample(timestamp_s=10.0, pdc_ma=1.0, tcm=100, tcl=50))
    agg.update(TelemetrySample(timestamp_s=10.1, pdc_ma=2.0, tcm=104, tcl=52))
    agg.update(TelemetrySample(timestamp_s=10.2, pdc_ma=3.0, tcm=108, tcl=54))

    assert agg.snapshot_at(10.15).pdc_ma == pytest.approx(2.0)
    assert agg.snapshot_at(10.2).pdc_ma == pytest.approx(3.0)   # inclusive
    assert agg.snapshot_at(99.0).pdc_ma == pytest.approx(3.0)
    assert agg.snapshot_at(9.9) is None                          # all in future


def test_aggregator_ring_bound():
    agg = TelemetryAggregator(maxlen=3)
    for i in range(10):
        agg.update(TelemetrySample(timestamp_s=float(i), pdc_ma=float(i), tcm=i, tcl=i))
    assert len(agg) == 3
    assert agg.snapshot_at(100.0).pdc_ma == pytest.approx(9.0)
    # Oldest entries were evicted.
    assert agg.snapshot_at(5.0) is None


# ── TelemetryIngestStage ────────────────────────────────────────────────────

def test_stage_stamps_pdc_tcm_tcl_via_wall_offset():
    """Frame times are scan-relative; aggregator samples are wall-clock.
    The stage anchors the offset on its first batch using the injected
    clock, then stamps each frame with the sample at-or-before its
    capture time."""
    agg = TelemetryAggregator()
    wall0 = 1000.0
    agg.update(TelemetrySample(timestamp_s=wall0 - 0.05, pdc_ma=1.5, tcm=10, tcl=5))
    agg.update(TelemetrySample(timestamp_s=wall0 + 0.10, pdc_ma=2.5, tcm=14, tcl=7))

    # Injected clock: "now" is wall0, and the first frame is at t=0.0 →
    # wall_offset = wall0.
    stage = TelemetryIngestStage(agg, now=lambda: wall0)
    batch = _batch([0.0, 0.05, 0.15])
    stage.process(batch)

    assert batch.pdc[0] == pytest.approx(1.5)   # wall0 + 0.00 → first sample
    assert batch.pdc[1] == pytest.approx(1.5)   # wall0 + 0.05 → still first
    assert batch.pdc[2] == pytest.approx(2.5)   # wall0 + 0.15 → second
    assert list(batch.tcm) == [10, 10, 14]
    assert list(batch.tcl) == [5, 5, 7]


def test_stage_nan_before_first_sample():
    agg = TelemetryAggregator()
    agg.update(TelemetrySample(timestamp_s=2000.0, pdc_ma=9.0, tcm=1, tcl=1))
    stage = TelemetryIngestStage(agg, now=lambda: 1000.0)   # samples all in future
    batch = _batch([0.0, 0.025])
    stage.process(batch)
    assert np.isnan(batch.pdc).all()
    assert list(batch.tcm) == [0, 0]
    assert list(batch.tcl) == [0, 0]


def test_stage_without_aggregator_is_noop():
    stage = TelemetryIngestStage(None)
    batch = _batch([0.0, 0.025])
    stage.process(batch)
    assert batch.pdc is None and batch.tcm is None and batch.tcl is None


def test_stage_reset_reanchors_offset_but_keeps_aggregator():
    agg = TelemetryAggregator()
    agg.update(TelemetrySample(timestamp_s=1000.0, pdc_ma=4.0, tcm=2, tcl=1))
    clock = {"now": 1000.0}
    stage = TelemetryIngestStage(agg, now=lambda: clock["now"])
    stage.process(_batch([0.0]))
    stage.reset()
    # Aggregator history survives reset; the offset re-anchors on the
    # next batch using the (advanced) clock.
    clock["now"] = 1010.0
    batch = _batch([10.0])   # scan-relative 10 s → wall 1010 + (10-10) ...
    stage.process(batch)
    assert batch.pdc[0] == pytest.approx(4.0)
    assert len(agg) == 1


# ── TelemetryFeeder ─────────────────────────────────────────────────────────

class _FakePoller:
    def __init__(self):
        self.listeners = []

    def add_listener(self, fn):
        self.listeners.append(fn)

    def remove_listener(self, fn):
        self.listeners.remove(fn)


def test_feeder_registers_converts_and_unregisters():
    agg = TelemetryAggregator()
    poller = _FakePoller()
    feeder = TelemetryFeeder(agg, poller)
    assert poller.listeners == [feeder]

    snap = SimpleNamespace(timestamp=1234.5, pdc=3.25, tcm=400, tcl=200)
    for listener in poller.listeners:
        listener(snap)
    sample = agg.snapshot_at(1234.5)
    assert sample == TelemetrySample(timestamp_s=1234.5, pdc_ma=3.25, tcm=400, tcl=200)

    feeder.close()
    assert poller.listeners == []
    feeder.close()   # idempotent


def test_feeder_swallows_bad_snapshots():
    agg = TelemetryAggregator()
    poller = _FakePoller()
    feeder = TelemetryFeeder(agg, poller)
    feeder(SimpleNamespace())   # missing fields — must not raise
    assert len(agg) == 0
    feeder.close()


# ── Factory wiring ──────────────────────────────────────────────────────────

def _factory_pipeline(telemetry=None):
    from omotion.pipeline.factory import default_pipeline
    from omotion.pipeline.pedestal import SensorPedestals
    from omotion.pipeline.sinks import ScanMetadata

    class _Cal:
        c_min = np.zeros((2, 8)); c_max = np.ones((2, 8))
        i_min = np.zeros((2, 8)); i_max = np.ones((2, 8))

    meta = ScanMetadata(
        scan_id="t", subject_id="s", operator="o",
        started_at_iso="2026-06-10T00:00:00Z", duration_sec=10,
        left_camera_mask=0x01, right_camera_mask=0, reduced_mode=False,
    )
    return default_pipeline(metadata=meta, calibration=_Cal(),
                            pedestals=SensorPedestals(left=64.0, right=64.0),
                            telemetry=telemetry)


def test_factory_omits_telemetry_stage_by_default():
    names = [s.name for s in _factory_pipeline().stages]
    assert "telemetry_ingest" not in names


def test_factory_inserts_telemetry_stage_before_raw_tee():
    names = [s.name for s in _factory_pipeline(telemetry=TelemetryAggregator()).stages]
    assert "telemetry_ingest" in names
    # Must run before the raw tee so the raw CSV carries the stamps.
    assert names.index("telemetry_ingest") < names.index("tee:raw")
    assert names.index("frame_classification") < names.index("telemetry_ingest")


# ── CsvReplaySource roundtrip ───────────────────────────────────────────────

def test_csv_replay_carries_recorded_telemetry(tmp_path):
    from omotion.pipeline.sources import CsvReplaySource
    from omotion.pipeline.sinks import ScanMetadata

    bins = ",".join("0" for _ in range(1024))
    header = "cam_id,frame_id,timestamp_s,type," + \
        ",".join(str(b) for b in range(1024)) + ",temperature,sum,tcm,tcl,pdc"
    rows = [
        f"0,1,0.025,light,{bins},27.0,0,400,200,3.25",
        f"0,2,0.050,light,{bins},27.0,0,,,",   # blank cells → no telemetry
    ]
    path = tmp_path / "replay_raw.csv"
    path.write_text(header + "\n" + "\n".join(rows) + "\n", encoding="utf-8")

    meta = ScanMetadata(
        scan_id="r", subject_id="s", operator="o",
        started_at_iso="2026-06-10T00:00:00Z", duration_sec=10,
        left_camera_mask=0x01, right_camera_mask=0, reduced_mode=False,
    )
    src = CsvReplaySource(raw_csv_left=path, raw_csv_right=None,
                          batch_size_frames=10, metadata=meta)
    batch = next(iter(src))
    assert batch.pdc[0] == pytest.approx(3.25)
    assert np.isnan(batch.pdc[1])
    assert list(batch.tcm) == [400, 0]
    assert list(batch.tcl) == [200, 0]
