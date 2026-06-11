"""Source protocol, CsvReplaySource, LiveUsbSource tests."""

from __future__ import annotations

import csv
import time

import numpy as np
import pytest

from omotion.pipeline.sources import Source, _BaseSource, CsvReplaySource, LiveUsbSource
from omotion.pipeline.sinks import ScanMetadata


def _meta():
    return ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
    )


def _write_raw_csv(tmp_path, rows):
    """Write a raw CSV in the new schema:
    cam_id, frame_id, timestamp_s, type, 0..1023, temperature, sum, tcm, tcl, pdc
    """
    path = tmp_path / "raw.csv"
    header = ["cam_id", "frame_id", "timestamp_s", "type"] + \
             [str(i) for i in range(1024)] + \
             ["temperature", "sum", "tcm", "tcl", "pdc"]
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        for row in rows:
            w.writerow(row)
    return path


# ---------------------------------------------------------------------------
# Task 19: Source protocol
# ---------------------------------------------------------------------------

def test_source_protocol_is_runtime_checkable():
    class _Mock:
        metadata = _meta()
        def __iter__(self): yield from []
        def close(self): pass
    assert isinstance(_Mock(), Source)


def test_base_source_close_is_noop_by_default():
    class _Sub(_BaseSource):
        def __iter__(self): yield from []

    src = _Sub(metadata=_meta())
    src.close()


# ---------------------------------------------------------------------------
# Task 20: CsvReplaySource
# ---------------------------------------------------------------------------

def test_csv_replay_yields_one_batch_per_chunk(tmp_path):
    bins = [0] * 1024
    bins[100] = 2_457_606
    rows = [
        [0, 1, 0.000, "warmup"] + bins + [27.0, 2_457_606, 0.0, 0.0, 0.0],
        [0, 2, 0.025, "warmup"] + bins + [27.0, 2_457_606, 0.0, 0.0, 0.0],
    ]
    path = _write_raw_csv(tmp_path, rows)

    src = CsvReplaySource(
        raw_csv_left=path, raw_csv_right=None,
        batch_size_frames=10, metadata=_meta(),
    )
    batches = list(src)
    assert len(batches) >= 1
    first = batches[0]
    assert first.raw_histograms.shape[-1] == 1024
    assert first.frame_ids.tolist()[:2] == [1, 2]
    assert first.raw_histograms[0, 0, 0, 100] == 2_457_606


def test_csv_replay_none_path_yields_nothing(tmp_path):
    src = CsvReplaySource(
        raw_csv_left=None, raw_csv_right=None,
        batch_size_frames=10, metadata=_meta(),
    )
    assert list(src) == []


def test_csv_replay_splits_into_multiple_batches(tmp_path):
    bins = [0] * 1024
    rows = [
        [0, i, i * 0.025, "warmup"] + bins + [27.0, 0, 0.0, 0.0, 0.0]
        for i in range(1, 6)
    ]
    path = _write_raw_csv(tmp_path, rows)

    src = CsvReplaySource(
        raw_csv_left=path, raw_csv_right=None,
        batch_size_frames=3, metadata=_meta(),
    )
    batches = list(src)
    # 5 rows, batch_size=3 → 2 batches (3 + 2)
    assert len(batches) == 2
    assert len(batches[0].frame_ids) == 3
    assert len(batches[1].frame_ids) == 2


# ---------------------------------------------------------------------------
# Task 22: LiveUsbSource skeleton
# ---------------------------------------------------------------------------

def test_live_usb_source_is_a_source():
    """LiveUsbSource satisfies the Source protocol."""
    src = LiveUsbSource(
        console=None, left=None, right=None,
        metadata=_meta(),
    )
    assert isinstance(src, Source)


def test_live_usb_source_close_stops_event():
    """close() sets the stop event."""
    src = LiveUsbSource(
        console=None, left=None, right=None,
        metadata=_meta(),
    )
    assert not src._stop.is_set()
    src.close()
    assert src._stop.is_set()


def test_live_usb_source_reader_loop_builds_batches_from_packet_queue(monkeypatch):
    """Mock parse_histogram_stream to feed fake samples; verify _reader_loop
    accumulates them into FrameBatches and pushes to the batch queue."""
    import queue as _queue
    import numpy as np
    from omotion.pipeline.batch import FrameBatch

    # Fake parse_histogram_stream: ignores the real queue and fires on_row_fn
    # with 15 synthetic samples positionally (matching the real call site), then returns.
    def _fake_parse_histogram_stream(q, stop_evt, buf, *, on_row_fn=None,
                                     expected_row_sum=None, t0_normalizer=None):
        for i in range(15):
            if on_row_fn is not None:
                on_row_fn(
                    0,                              # cam_id
                    i + 1,                          # frame_id
                    0.025 * (i + 1),                # ts
                    np.ones(1024, dtype=np.uint32), # histogram
                    1024,                           # row_sum
                    27.0,                           # temperature_c (temp)
                )
            if stop_evt.is_set():
                return 15
        return 15

    monkeypatch.setattr(
        "omotion.MotionProcessing.parse_histogram_stream",
        _fake_parse_histogram_stream,
    )

    class _FakeSensor:
        class _FakeStream:
            def start_streaming(self, q, expected_size):
                pass
            def stop_streaming(self):
                pass
            def drain_final(self, expected_size):
                return []
        class _FakeUart:
            def __init__(self, stream):
                self.histo = stream
        def __init__(self):
            self.uart = self._FakeUart(self._FakeStream())

    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=2,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
    )

    src = LiveUsbSource(
        console=None, left=_FakeSensor(), right=None,
        batch_size_frames=10, metadata=meta,
    )
    batches = []
    for batch in src:
        batches.append(batch)
        if len(batches) >= 2:
            src.close()
            break

    # 15 samples with batch_size=10 → at least one full batch (10) and a partial (5)
    assert len(batches) >= 1
    total_frames = sum(b.raw_histograms.shape[0] for b in batches)
    assert total_frames >= 10
    # Each batch has the correct histogram shape
    for b in batches:
        assert b.raw_histograms.shape[-1] == 1024


@pytest.mark.sensor
def test_live_usb_source_smoke_yields_framebatches(console, sensor_left):
    """Hardware-marked smoke test — requires a connected sensor.

    Uses the session ``console``/``sensor_left`` fixtures so it skips
    gracefully on a rig with no sensor attached. Passing an unconnected
    ``motion.left`` handle (whose ``uart`` is still None) straight into
    LiveUsbSource would crash in __iter__ at ``sensor.uart.histo`` — the
    source legitimately assumes it only ever sees connected handles, so the
    presence gate belongs here in the test, matching every other HIL test."""
    from omotion.pipeline.sources import LiveUsbSource
    from omotion.pipeline.sinks import ScanMetadata

    meta = ScanMetadata(
        scan_id="smoke", subject_id="x", operator="test",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=2,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
    )
    src = LiveUsbSource(
        console=console, left=sensor_left, right=None,
        batch_size_frames=10, metadata=meta,
    )
    batches_seen = 0
    for batch in src:
        batches_seen += 1
        assert batch.raw_histograms.shape[-1] == 1024
        if batches_seen >= 3:
            src.close()
            break
    assert batches_seen >= 1


# ---------------------------------------------------------------------------
# Timestamp normalization
# ---------------------------------------------------------------------------

def test_csv_replay_normalizes_timestamps_to_zero(tmp_path):
    """CsvReplaySource subtracts first frame's timestamp so scans start at t=0."""
    bins = [0] * 1024
    # Timestamps starting at 630.915 (firmware-absolute style)
    rows = [
        [0, 1, 630.915, "light"] + bins + [27.0, 0, 0.0, 0.0, 0.0],
        [0, 2, 630.940, "light"] + bins + [27.0, 0, 0.0, 0.0, 0.0],
        [0, 3, 630.965, "light"] + bins + [27.0, 0, 0.0, 0.0, 0.0],
    ]
    path = _write_raw_csv(tmp_path, rows)

    src = CsvReplaySource(
        raw_csv_left=path, raw_csv_right=None,
        batch_size_frames=10, metadata=_meta(),
    )
    batches = list(src)
    assert len(batches) == 1
    ts = batches[0].timestamp_s
    assert ts[0] == pytest.approx(0.0)
    assert ts[1] == pytest.approx(0.025)
    assert ts[2] == pytest.approx(0.050)


def test_csv_replay_normalization_spans_batch_boundaries(tmp_path):
    """t0 is preserved across batch boundaries."""
    bins = [0] * 1024
    rows = [
        [0, i, 100.0 + i * 0.025, "light"] + bins + [27.0, 0, 0.0, 0.0, 0.0]
        for i in range(1, 6)
    ]
    path = _write_raw_csv(tmp_path, rows)

    src = CsvReplaySource(
        raw_csv_left=path, raw_csv_right=None,
        batch_size_frames=3, metadata=_meta(),
    )
    batches = list(src)
    all_ts = np.concatenate([b.timestamp_s for b in batches])
    assert all_ts[0] == pytest.approx(0.0)
    assert all_ts[-1] == pytest.approx(4 * 0.025)


def test_csv_replay_no_normalization_when_disabled(tmp_path):
    """With normalize_timestamps=False, raw timestamps are preserved."""
    bins = [0] * 1024
    rows = [[0, 1, 630.915, "light"] + bins + [27.0, 0, 0.0, 0.0, 0.0]]
    path = _write_raw_csv(tmp_path, rows)

    src = CsvReplaySource(
        raw_csv_left=path, raw_csv_right=None,
        batch_size_frames=10, metadata=_meta(),
        normalize_timestamps=False,
    )
    batches = list(src)
    assert batches[0].timestamp_s[0] == pytest.approx(630.915)
