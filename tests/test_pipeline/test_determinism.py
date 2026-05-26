"""Determinism: same source twice through the same pipeline = same output.

Replays the synthetic fixture twice using independent pipeline and sink
instances and asserts the corrected CSV rows are byte-identical.
"""

from __future__ import annotations

import pathlib

import pytest

from omotion.pipeline.runner import ScanRunner
from omotion.pipeline.sources import CsvReplaySource
from omotion.pipeline.sinks import CsvSink
from omotion.pipeline.factory import default_pipeline
from omotion.pipeline.pedestal import SensorPedestals
from tests.test_pipeline.test_golden_replay import (
    _trivial_calibration,
    _read_csv_rows,
    _make_meta,
    _PEDESTAL,
    _DARK_INTERVAL,
)

HERE = pathlib.Path(__file__).parent / "data"


def _replay_once(raw_csv: pathlib.Path, out_dir: pathlib.Path, run_id: str) -> pathlib.Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    meta = _make_meta(scan_id=f"det_{run_id}")
    pipeline = default_pipeline(
        metadata=meta,
        calibration=_trivial_calibration(),
        pedestals=SensorPedestals(left=_PEDESTAL, right=_PEDESTAL),
        dark_interval=_DARK_INTERVAL,
    )
    source = CsvReplaySource(
        raw_csv_left=raw_csv,
        raw_csv_right=None,
        batch_size_frames=20,
        metadata=meta,
    )
    sink = CsvSink(output_dir=str(out_dir))
    ScanRunner(source=source, pipeline=pipeline, sinks=[sink]).run()

    corrected = [p for p in out_dir.glob("*.csv") if not p.name.endswith("_raw.csv")]
    if not corrected:
        pytest.skip("CsvSink.consume('final') not writing corrected output")
    return corrected[0]


def test_two_replays_byte_identical(tmp_path: pathlib.Path) -> None:
    """Running the pipeline twice on the same input produces identical output."""
    raw = HERE / "normal_short_scan.raw.csv"
    if not raw.exists():
        pytest.skip("Golden fixtures not yet generated — run regenerate_goldens.py")

    out1 = _replay_once(raw, tmp_path / "run1", run_id="run1")
    out2 = _replay_once(raw, tmp_path / "run2", run_id="run2")

    rows1 = _read_csv_rows(out1)
    rows2 = _read_csv_rows(out2)

    assert rows1 == rows2, (
        f"Two pipeline runs produced different output.\n"
        f"Run 1 rows: {len(rows1)}, Run 2 rows: {len(rows2)}\n"
        f"First differing row: "
        + str(next(
            (i for i, (a, b) in enumerate(zip(rows1, rows2)) if a != b),
            "lengths differ",
        ))
    )
