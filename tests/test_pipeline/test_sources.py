"""Source protocol, CsvReplaySource, DbReplaySource, LiveUsbSource tests."""

from __future__ import annotations

import csv
import time

import numpy as np
import pytest

from omotion.pipeline.sources import Source, _BaseSource, CsvReplaySource
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
