"""Tests for omotion.CsvSink — the unified CSV writing sink.

Step B1 covers ``on_corrected_batch`` only. Raw and telemetry CSV paths
land in later commits and will get their own test sections here.

The load-bearing test is ``test_csv_sink_matches_inline_corrected_csv``
which feeds the canonical fixture scan CSVs through ``create_science_pipeline``
into both the existing ``_CorrectedCsvBuilder`` (mimics today's inline
ScanWorkflow path) and a ``CsvSink``, then diffs the two output CSV files
line-by-line. They must be byte-identical for the extraction to be safe.
"""

from __future__ import annotations

import csv
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omotion import CsvSink
from omotion.CsvSink import _corrected_columns, _expected_col_suffixes
from omotion.MotionProcessing import (
    CorrectedBatch,
    Sample,
    create_science_pipeline,
    feed_pipeline_from_csv,
)
from omotion.ScanWorkflow import ScanRequest


# ---------------------------------------------------------------------------
# Fixture paths + calibration (duplicated from test_corrected_csv_output.py)
# ---------------------------------------------------------------------------

FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "fixtures")
LEFT_CSV  = os.path.join(FIXTURES_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
RIGHT_CSV = os.path.join(FIXTURES_DIR, "scan_owC18EHALL_20251217_160949_right_maskFF.csv")
LEFT_MASK  = 0xFF
RIGHT_MASK = 0xFF

_ZERO = np.zeros((2, 8), dtype=np.float64)
_ONE  = np.ones((2, 8),  dtype=np.float64)
BFI_C_MIN = _ZERO.copy()
BFI_C_MAX = _ONE.copy()
BFI_I_MIN = _ZERO.copy()
BFI_I_MAX = np.full((2, 8), 1000.0)


def _make_request(
    tmp_path: Path,
    *,
    left_mask: int = LEFT_MASK,
    right_mask: int = RIGHT_MASK,
    reduced_mode: bool = False,
    subject_id: str = "csvSinkTest",
    write_corrected_csv: bool = True,
) -> ScanRequest:
    return ScanRequest(
        subject_id=subject_id,
        duration_sec=60,
        left_camera_mask=left_mask,
        right_camera_mask=right_mask,
        data_dir=str(tmp_path),
        disable_laser=False,
        write_corrected_csv=write_corrected_csv,
        reduced_mode=reduced_mode,
    )


def _mk_sample(side, cam_id, frame_id, ts, bfi, bvi, contrast, mean):
    return Sample(
        side=side,
        cam_id=cam_id,
        frame_id=frame_id,
        absolute_frame_id=frame_id,
        timestamp_s=ts,
        row_sum=0,
        temperature_c=25.0,
        mean=mean,
        std_dev=0.0,
        contrast=contrast,
        bfi=bfi,
        bvi=bvi,
        is_corrected=True,
    )


# ---------------------------------------------------------------------------
# Standalone shape + lifecycle
# ---------------------------------------------------------------------------

def test_csv_sink_opens_corrected_csv_with_header(tmp_path: Path) -> None:
    sink = CsvSink()
    sink.on_scan_start(
        ts="20260520_120000",
        session_start_ts=0.0,
        request=_make_request(tmp_path),
        meta={},
    )
    sink.on_complete()

    out = tmp_path / "20260520_120000_csvSinkTest.csv"
    assert out.exists()
    assert sink.corrected_path == str(out)
    with open(out, newline="") as fh:
        header = next(csv.reader(fh))
    assert header[:2] == ["frame_id", "timestamp_s"]
    # 5 metrics × 16 (sides × cams) = 80 data columns
    assert len(header) == 2 + 80


def test_csv_sink_no_file_when_corrected_disabled(tmp_path: Path) -> None:
    sink = CsvSink()
    sink.on_scan_start(
        ts="t",
        session_start_ts=0.0,
        request=_make_request(tmp_path, write_corrected_csv=False),
        meta={},
    )
    sink.on_complete()
    assert sink.corrected_path is None
    assert not any(tmp_path.iterdir())


def test_csv_sink_writes_complete_frames(tmp_path: Path) -> None:
    """A frame with all expected (side, cam) cells reporting flushes
    immediately on the batch that completes it."""
    sink = CsvSink()
    sink.on_scan_start(
        ts="t",
        session_start_ts=0.0,
        request=_make_request(tmp_path, left_mask=0x03, right_mask=0x03),
        meta={},
    )
    # Mask 0x03 → cams 0 + 1 active per side, so the row "completes" when
    # all of l1, l2, r1, r2 have reported.
    batch = CorrectedBatch(
        dark_frame_start=0, dark_frame_end=600,
        samples=[
            _mk_sample("left",  0, 1, 0.025, 0.1, 0.2, 0.3, 500.0),
            _mk_sample("left",  1, 1, 0.025, 0.11, 0.21, 0.31, 501.0),
            _mk_sample("right", 0, 1, 0.025, 0.12, 0.22, 0.32, 502.0),
            _mk_sample("right", 1, 1, 0.025, 0.13, 0.23, 0.33, 503.0),
        ],
    )
    sink.on_corrected_batch(batch)
    # Row should be flushed immediately — assert via rows_written before close.
    assert sink.rows_written == 1
    sink.on_complete()

    with open(sink.corrected_path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 1
    r = rows[0]
    assert r["frame_id"] == "1"
    assert float(r["timestamp_s"]) == 0.025
    assert float(r["bfi_l1"]) == 0.1
    assert float(r["bfi_l2"]) == 0.11
    assert float(r["bfi_r1"]) == 0.12
    assert float(r["bfi_r2"]) == 0.13


def test_csv_sink_holds_incomplete_frames_until_all_cams_report(tmp_path: Path) -> None:
    """A frame missing one cam shouldn't write until that cam catches up
    or the scan ends (then it's flushed as partial)."""
    sink = CsvSink()
    sink.on_scan_start(
        ts="t",
        session_start_ts=0.0,
        request=_make_request(tmp_path, left_mask=0x03, right_mask=0x03),
        meta={},
    )
    # Only 3/4 expected cells for frame 1.
    sink.on_corrected_batch(CorrectedBatch(
        dark_frame_start=0, dark_frame_end=600,
        samples=[
            _mk_sample("left",  0, 1, 0.025, 0.1, 0.2, 0.3, 500.0),
            _mk_sample("left",  1, 1, 0.025, 0.11, 0.21, 0.31, 501.0),
            _mk_sample("right", 0, 1, 0.025, 0.12, 0.22, 0.32, 502.0),
        ],
    ))
    assert sink.rows_written == 0
    # Late arrival completes the row.
    sink.on_corrected_batch(CorrectedBatch(
        dark_frame_start=0, dark_frame_end=600,
        samples=[_mk_sample("right", 1, 1, 0.025, 0.13, 0.23, 0.33, 503.0)],
    ))
    assert sink.rows_written == 1
    sink.on_complete()


def test_csv_sink_flushes_partial_frames_at_complete(tmp_path: Path) -> None:
    """Frames still missing cams at scan end are flushed as partial rows
    (empty cells for the missing cams) so the operator doesn't silently
    lose them."""
    sink = CsvSink()
    sink.on_scan_start(
        ts="t",
        session_start_ts=0.0,
        request=_make_request(tmp_path, left_mask=0x03, right_mask=0x03),
        meta={},
    )
    sink.on_corrected_batch(CorrectedBatch(
        dark_frame_start=0, dark_frame_end=600,
        samples=[
            _mk_sample("left", 0, 1, 0.025, 0.1, 0.2, 0.3, 500.0),
            _mk_sample("left", 1, 1, 0.025, 0.11, 0.21, 0.31, 501.0),
        ],
    ))
    assert sink.rows_written == 0
    sink.on_complete()
    assert sink.rows_written == 1
    with open(sink.corrected_path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    r = rows[0]
    assert float(r["bfi_l1"]) == 0.1
    assert float(r["bfi_l2"]) == 0.11
    # right side never reported → empty cells
    assert r["bfi_r1"] == ""
    assert r["bfi_r2"] == ""


def test_csv_sink_reduced_mode_writes_averaged_columns(tmp_path: Path) -> None:
    sink = CsvSink()
    sink.on_scan_start(
        ts="t",
        session_start_ts=0.0,
        request=_make_request(
            tmp_path, reduced_mode=True,
            left_mask=0x03, right_mask=0x03,
        ),
        meta={},
    )
    # Two cams per side; reduced mode averages them.
    sink.on_corrected_batch(CorrectedBatch(
        dark_frame_start=0, dark_frame_end=600,
        samples=[
            _mk_sample("left",  0, 1, 0.025, 0.10, 0.20, 0.30, 500.0),
            _mk_sample("left",  1, 1, 0.025, 0.20, 0.30, 0.40, 501.0),
            _mk_sample("right", 0, 1, 0.025, 0.30, 0.40, 0.50, 502.0),
            _mk_sample("right", 1, 1, 0.025, 0.40, 0.50, 0.60, 503.0),
        ],
    ))
    sink.on_complete()
    with open(sink.corrected_path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 1
    assert list(rows[0].keys()) == [
        "frame_id", "timestamp_s",
        "bfi_left", "bfi_right", "bvi_left", "bvi_right",
    ]
    assert float(rows[0]["bfi_left"])  == pytest.approx(0.15, abs=1e-6)
    assert float(rows[0]["bfi_right"]) == pytest.approx(0.35, abs=1e-6)
    assert float(rows[0]["bvi_left"])  == pytest.approx(0.25, abs=1e-6)
    assert float(rows[0]["bvi_right"]) == pytest.approx(0.45, abs=1e-6)


# ---------------------------------------------------------------------------
# Bit-identical equivalence with the inline ScanWorkflow path
# ---------------------------------------------------------------------------

class _InlineCorrectedMirror:
    """Replica of the SDK's pre-extraction inline corrected-CSV writer
    in ``ScanWorkflow._on_corrected_batch`` — same merge logic, same
    6-decimal rounding, same set of columns and same complete-row
    flush gate. Used as the cross-check ``CsvSink`` is supposed to be
    byte-identical with after extraction."""

    def __init__(self, left_mask: int, right_mask: int) -> None:
        self.columns = _corrected_columns(reduced_mode=False)
        self.expected = _expected_col_suffixes(left_mask, right_mask)
        self.by_frame: dict[int, dict] = {}
        self.complete_rows: list[list] = []

    def on_corrected_batch(self, batch: CorrectedBatch) -> None:
        for sample in batch.samples:
            fid = int(sample.absolute_frame_id)
            col_suffix = f"{sample.side[0]}{int(sample.cam_id) + 1}"
            entry = self.by_frame.get(fid)
            if entry is None:
                entry = {
                    "timestamp_s": float(sample.timestamp_s),
                    "values": {},
                }
                self.by_frame[fid] = entry
            else:
                entry["timestamp_s"] = min(
                    float(entry["timestamp_s"]),
                    float(sample.timestamp_s),
                )
            entry["values"][f"bfi_{col_suffix}"]      = round(float(sample.bfi), 6)
            entry["values"][f"bvi_{col_suffix}"]      = round(float(sample.bvi), 6)
            entry["values"][f"mean_{col_suffix}"]     = round(float(sample.mean), 6)
            entry["values"][f"contrast_{col_suffix}"] = round(float(sample.contrast), 6)
            entry["values"][f"temp_{col_suffix}"]     = float(sample.temperature_c)

        if self.expected:
            complete = [
                fid for fid, e in self.by_frame.items()
                if all(f"bfi_{s}" in e["values"] for s in self.expected)
            ]
            for fid in sorted(complete):
                e = self.by_frame.pop(fid)
                row = [fid, float(e["timestamp_s"])]
                row.extend(e["values"].get(col, "") for col in self.columns)
                self.complete_rows.append(row)

    def flush_remaining(self) -> None:
        for fid in sorted(self.by_frame.keys()):
            e = self.by_frame[fid]
            row = [fid, float(e["timestamp_s"])]
            row.extend(e["values"].get(col, "") for col in self.columns)
            self.complete_rows.append(row)
        self.by_frame.clear()

    def write_csv(self, path: str) -> None:
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["frame_id", "timestamp_s", *self.columns])
            for row in self.complete_rows:
                w.writerow(row)


@pytest.mark.skipif(
    not (os.path.exists(LEFT_CSV) and os.path.exists(RIGHT_CSV)),
    reason="fixture CSVs not present",
)
def test_csv_sink_matches_inline_corrected_csv(tmp_path: Path) -> None:
    """Drive the corrected pipeline with the canonical fixture CSVs.
    Multiplex every ``CorrectedBatch`` to BOTH a ``CsvSink`` and an
    ``_InlineCorrectedMirror`` that replays the exact pre-extraction
    inline writer logic. The two output files must be byte-identical
    — otherwise the extraction has regressed the corrected CSV format.
    """
    sink = CsvSink()
    sink.on_scan_start(
        ts="20260520_120000",
        session_start_ts=0.0,
        request=_make_request(tmp_path, subject_id="parityCheck"),
        meta={},
    )

    mirror = _InlineCorrectedMirror(LEFT_MASK, RIGHT_MASK)

    def _multiplex(batch: CorrectedBatch) -> None:
        mirror.on_corrected_batch(batch)
        sink.on_corrected_batch(batch)

    pipeline = create_science_pipeline(
        left_camera_mask=LEFT_MASK,
        right_camera_mask=RIGHT_MASK,
        bfi_c_min=BFI_C_MIN,
        bfi_c_max=BFI_C_MAX,
        bfi_i_min=BFI_I_MIN,
        bfi_i_max=BFI_I_MAX,
        on_corrected_batch_fn=_multiplex,
    )
    feed_pipeline_from_csv(LEFT_CSV,  "left",  pipeline)
    feed_pipeline_from_csv(RIGHT_CSV, "right", pipeline)
    pipeline.stop(timeout=120.0)
    mirror.flush_remaining()
    mirror_path = tmp_path / "mirror_corrected.csv"
    mirror.write_csv(str(mirror_path))

    sink.on_complete()

    # File-level byte-identical check.
    with open(sink.corrected_path, "rb") as fa, open(mirror_path, "rb") as fb:
        a = fa.read()
        b = fb.read()
    assert len(a) > 0
    assert len(a) == len(b), f"file size mismatch: sink={len(a)} mirror={len(b)}"
    if a != b:
        # On mismatch, surface the first differing line for a usable error.
        for i, (la, lb) in enumerate(zip(a.splitlines(), b.splitlines())):
            if la != lb:
                pytest.fail(
                    f"first differing line at index {i}:\n"
                    f"  sink:   {la[:200]!r}\n"
                    f"  mirror: {lb[:200]!r}"
                )
        pytest.fail("file lengths match but bytes differ (trailing whitespace?)")
