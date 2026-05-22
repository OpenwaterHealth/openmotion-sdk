"""Source protocol, CsvReplaySource, DbReplaySource, LiveUsbSource tests."""

from __future__ import annotations

import csv
import time

import numpy as np
import pytest

from omotion.pipeline.sources import Source, _BaseSource, CsvReplaySource, DbReplaySource, LiveUsbSource
from omotion.pipeline.sinks import ScanMetadata


def _meta():
    return ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
        write_raw_csv=False, raw_csv_duration_sec=None,
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
# Task 21: DbReplaySource
# ---------------------------------------------------------------------------

def test_db_replay_yields_batches_from_session_raw(tmp_path):
    """Use the actual ScanDatabase API (create_session + insert_raw_frame)."""
    from omotion.ScanDatabase import ScanDatabase

    db_path = tmp_path / "scan.db"
    db = ScanDatabase(str(db_path))

    session_id = db.create_session(
        session_label="test-session",
        session_start=time.time(),
    )

    # insert_raw_frame uses: hist (bytes), temp, sum_counts
    bins = bytes(1024 * 4)  # 4096 bytes of zeros
    for fid in range(1, 4):
        db.insert_raw_frame(
            session_id=session_id,
            side="left",
            cam_id=0,
            frame_id=fid,
            timestamp_s=fid * 0.025,
            hist=bins,
            temp=27.0,
            sum_counts=2_457_606,
        )
    db.close()

    src = DbReplaySource(
        db_path=str(db_path),
        session_id=session_id,
        batch_size_frames=10,
        metadata=_meta(),
    )
    batches = list(src)
    assert len(batches) >= 1
    assert batches[0].frame_ids.tolist() == [1, 2, 3]


def test_db_replay_empty_session_yields_nothing(tmp_path):
    from omotion.ScanDatabase import ScanDatabase

    db_path = tmp_path / "empty.db"
    db = ScanDatabase(str(db_path))
    session_id = db.create_session(
        session_label="empty-session",
        session_start=time.time(),
    )
    db.close()

    src = DbReplaySource(
        db_path=str(db_path),
        session_id=session_id,
        batch_size_frames=10,
        metadata=_meta(),
    )
    assert list(src) == []


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


def test_live_usb_source_reader_loop_raises_not_implemented():
    """_reader_loop raises NotImplementedError (body deferred to PR 2)."""
    src = LiveUsbSource(
        console=None, left=None, right=None,
        metadata=_meta(),
    )
    with pytest.raises(NotImplementedError):
        src._reader_loop("left", object())


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


def test_db_replay_normalizes_timestamps_to_zero(tmp_path):
    """DbReplaySource normalizes timestamps on replay."""
    import time as _time
    from omotion.ScanDatabase import ScanDatabase

    db_path = tmp_path / "scan.db"
    db = ScanDatabase(str(db_path))
    session_id = db.create_session(
        session_label="ts-test",
        session_start=_time.time(),
    )
    bins = bytes(1024 * 4)
    for i, t in enumerate([500.0, 500.025, 500.050], start=1):
        db.insert_raw_frame(
            session_id=session_id, side="left", cam_id=0,
            frame_id=i, timestamp_s=t,
            hist=bins, temp=27.0, sum_counts=0,
        )
    db.close()

    src = DbReplaySource(
        db_path=str(db_path), session_id=session_id,
        batch_size_frames=10, metadata=_meta(),
    )
    batches = list(src)
    ts = batches[0].timestamp_s
    assert ts[0] == pytest.approx(0.0)
    assert ts[1] == pytest.approx(0.025)
    assert ts[2] == pytest.approx(0.050)
