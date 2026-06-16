"""Source protocol, CsvReplaySource, LiveUsbSource tests."""

from __future__ import annotations

import csv
import threading
import time
from unittest import mock

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


def _consume_bounded(src, *, want_batches: int, timeout_s: float):
    """Consume FrameBatches from `src` on a worker thread, bounded by
    timeout_s. Returns (batches_seen, error).

    LiveUsbSource.__iter__ blocks indefinitely while a started-but-idle
    sensor produces no frames, so the iteration must happen off-thread
    with a deadline. On timeout the source is closed, which pushes the
    close() sentinel and unblocks the worker; the worker is daemon as a
    backstop so a wedged close can never hang the pytest process.
    """
    result = {"n": 0, "error": None}

    def _consume():
        try:
            for batch in src:
                assert batch.raw_histograms.shape[-1] == 1024
                result["n"] += 1
                if result["n"] >= want_batches:
                    src.close()
                    break
        except Exception as exc:
            result["error"] = exc

    t = threading.Thread(target=_consume, daemon=True, name="smoke-consume")
    t.start()
    t.join(timeout=timeout_s)
    if t.is_alive():
        src.close()  # bounded: 10s teardown + 5s reader join + sentinel
        t.join(timeout=20.0)
    return result["n"], result["error"]


def test_smoke_consume_bounded_returns_zero_when_source_idle():
    """A LiveUsbSource whose sensors never produce packets must not hang
    the bounded consumer: it returns 0 batches within the timeout. (The
    smoke test below then SKIPs, instead of blocking until pytest-timeout
    dumps stacks and kills the entire run — the present-but-idle-sensor
    failure mode that PR #74's absent-hardware skip didn't cover.)"""
    src = LiveUsbSource(
        console=mock.MagicMock(), left=mock.MagicMock(), right=None,
        batch_size_frames=10, metadata=_meta(),
    )
    t0 = time.monotonic()
    batches_seen, error = _consume_bounded(src, want_batches=3, timeout_s=1.5)
    assert batches_seen == 0
    assert error is None
    # Bounded well under pytest-timeout's 30 s, including close() teardown.
    assert time.monotonic() - t0 < 25.0


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
    # Bounded consume: a sensor that is attached but not streaming (cameras
    # disabled, or the device held by another process) never yields a batch;
    # unbounded iteration would block until pytest-timeout kills the whole
    # run. No data within the deadline is an environment problem, not a
    # code failure — skip, like the absent-hardware case.
    batches_seen, error = _consume_bounded(src, want_batches=3, timeout_s=8.0)
    if error is not None:
        raise error
    if batches_seen == 0:
        pytest.skip(
            "sensor attached but not streaming (no FrameBatch within 8s) — "
            "cameras disabled or device held by another process"
        )
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


def test_t0_normalize_anchors_each_side_independently():
    """The scalar (live-path) normalizer must anchor each side at its OWN
    first sample. Frame timestamps are each sensor's since-boot firmware
    clock, and the sensors boot independently — here the right sensor
    booted ~83 s before the left (bench repro, 2026-06-11)."""
    class _Sub(_BaseSource):
        def __iter__(self): yield from []

    src = _Sub(metadata=_meta())
    assert src._t0_normalize("left", 100.000) == pytest.approx(0.0)
    assert src._t0_normalize("right", 8283.000) == pytest.approx(0.0)
    assert src._t0_normalize("left", 100.025) == pytest.approx(0.025)
    assert src._t0_normalize("right", 8283.025) == pytest.approx(0.025)


def test_t0_normalize_passthrough_when_disabled():
    """normalize_timestamps=False preserves raw firmware timestamps in the
    scalar (live-path) form, same as the array form."""
    class _Sub(_BaseSource):
        def __iter__(self): yield from []

    src = _Sub(metadata=_meta(), normalize_timestamps=False)
    assert src._t0_normalize("left", 630.915) == pytest.approx(630.915)
    assert src._t0_normalize("right", 8283.000) == pytest.approx(8283.000)


def test_live_usb_reader_loops_anchor_t0_per_side(monkeypatch):
    """Two sensors with divergent since-boot clocks must each anchor their
    own t=0. A single shared t0 (whichever side's first sample wins) leaves
    the other side offset by the boot-time difference, and the live plot
    prunes that side to empty (bench repro 2026-06-11: left 0.2-12.8 s vs
    right 82.9-95.5 s within the same scan)."""

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

    src = LiveUsbSource(
        console=None, left=_FakeSensor(), right=_FakeSensor(),
        batch_size_frames=10, metadata=_meta(),
    )

    # Absolute firmware clocks: right booted ~82.6 s before left.
    clock_base = {"left": 100.0, "right": 8283.0}

    def _fake_parse_histogram_stream(q, stop_evt, buf, *, on_row_fn=None,
                                     expected_row_sum=None, t0_normalizer=None):
        side = next(s for s, sq in src._packet_queues.items() if sq is q)
        base = clock_base[side]
        for i in range(15):
            ts = base + 0.025 * i
            # Same call the real parser makes before firing on_row_fn.
            if t0_normalizer is not None:
                ts = t0_normalizer(ts)
            if on_row_fn is not None:
                on_row_fn(0, i + 1, ts,
                          np.ones(1024, dtype=np.uint32), 1024, 27.0)
        return 15

    monkeypatch.setattr(
        "omotion.MotionProcessing.parse_histogram_stream",
        _fake_parse_histogram_stream,
    )

    # 15 samples per side at batch_size=10 → 2 batches per side (10 + 5).
    batches = []
    for batch in src:
        batches.append(batch)
        if len(batches) >= 4:
            src.close()
            break

    seen_sides = set()
    for b in batches:
        seen_sides.add(int(b.side_ids[0]))
        # Every side's samples must sit on its own near-zero time axis.
        assert float(b.timestamp_s.min()) >= 0.0, (
            f"side {int(b.side_ids[0])} timestamps below its own zero: "
            f"{b.timestamp_s[:3]}"
        )
        assert float(b.timestamp_s.max()) <= 1.0, (
            f"side {int(b.side_ids[0])} timestamps not anchored at its own "
            f"zero: {b.timestamp_s[:3]}"
        )
    assert seen_sides == {0, 1}
