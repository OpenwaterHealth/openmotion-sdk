"""Helper script — run once manually to regenerate golden fixtures.

Usage (from repo root):
    python tests/test_pipeline/data/regenerate_goldens.py

This script:
1. Generates a synthetic raw CSV (50 frames, 1 camera, 1 side) with
   realistic histogram moments. The sequence includes warmup + first dark +
   light frames + second dark + a terminal dark-like final frame, so
   DarkCorrectionStage emits closed and terminal-flushed intervals.
2. Runs the pipeline once against that raw CSV (dark_interval=20 so the
   second dark falls at frame 30 within 50 frames).
3. Saves the raw CSV as ``normal_short_scan.raw.csv`` and the pipeline's
   corrected output as ``normal_short_scan.corrected.golden.csv``.

These files are committed and used by test_golden_replay.py to detect
regressions. They are NOT generated during CI — regenerate manually when
the pipeline algorithm changes intentionally.
"""

from __future__ import annotations

import csv
import pathlib
import shutil
import sys
import tempfile

import numpy as np

# Make sure the repo root is on the path when run directly.
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from omotion.pipeline.factory import default_pipeline
from omotion.pipeline.runner import ScanRunner
from omotion.pipeline.sources import CsvReplaySource
from omotion.pipeline.sinks import CsvSink, ScanMetadata
from omotion.config import HISTO_SIZE_WORDS
from omotion.pipeline.pedestal import SensorPedestals

HERE = pathlib.Path(__file__).parent
RAW_OUT  = HERE / "normal_short_scan.raw.csv"
GOLD_OUT = HERE / "normal_short_scan.corrected.golden.csv"

# Fixture-shape constants
_N_FRAMES   = 50
_CAM_ID     = 0
_PEDESTAL   = 64.0
_DARK_INTERVAL = 20   # pipeline param; keeps second dark at frame 30


def _make_histogram(u1_target: float, std_target: float, rng: np.random.Generator) -> np.ndarray:
    """Synthesize a 1024-bin histogram with given mean ± std (Gaussian-shaped)."""
    bins = np.arange(HISTO_SIZE_WORDS, dtype=np.float64)
    weights = np.exp(-0.5 * ((bins - u1_target) / max(std_target, 1.0)) ** 2)
    weights = np.maximum(weights, 0.0)
    total = weights.sum()
    if total <= 0:
        weights[int(u1_target)] = 1.0
        total = 1.0
    probs = weights / total
    counts = rng.multinomial(10_000, probs).astype(np.uint32)
    return counts


def _write_raw_csv(path: pathlib.Path) -> None:
    """Write the synthetic raw CSV fixture."""
    rng = np.random.default_rng(42)

    # Raw CSV headers (matching CsvSink._RAW_PIPELINE_HEADERS):
    # cam_id, frame_id, timestamp_s, type, 0..1023, temperature, sum, tcm, tcl, pdc
    headers = [
        "cam_id", "frame_id", "timestamp_s", "type",
        *[str(b) for b in range(HISTO_SIZE_WORDS)],
        "temperature", "sum", "tcm", "tcl", "pdc",
    ]

    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(headers)

        for frame_num in range(1, _N_FRAMES + 1):
            ts = (frame_num - 1) / 40.0  # 40 Hz

            # u1 target: dark frames are near pedestal; light frames are brighter
            # (matching what FrameClassificationStage expects at dark_interval=20)
            discard_count = 9
            is_warmup = frame_num <= discard_count
            is_dark_10 = (frame_num == discard_count + 1)
            is_dark_30 = (frame_num > discard_count + 1) and ((frame_num - 1) % _DARK_INTERVAL == 0)
            is_terminal_dark = frame_num == _N_FRAMES

            if is_dark_10 or is_dark_30 or is_terminal_dark:
                u1 = _PEDESTAL + rng.uniform(2.0, 5.0)
                std = 3.0 + rng.uniform(0.0, 1.0)
            elif is_warmup:
                u1 = _PEDESTAL + rng.uniform(50.0, 80.0)
                std = 10.0 + rng.uniform(0.0, 5.0)
            else:
                u1 = _PEDESTAL + 150.0 + rng.uniform(-10.0, 10.0)
                std = 15.0 + rng.uniform(0.0, 3.0)

            histo = _make_histogram(u1, std, rng)
            histo_sum = int(histo.sum())
            temp = 36.5 + rng.uniform(-0.2, 0.2)

            w.writerow([
                _CAM_ID,        # cam_id
                frame_num,      # frame_id (raw 8-bit; stays < 256 for 50 frames)
                round(ts, 6),
                "",             # type column (not used by CsvReplaySource)
                *histo.tolist(),
                round(temp, 4),
                histo_sum,
                "",             # tcm
                "",             # tcl
                "",             # pdc
            ])

    print(f"Wrote raw fixture: {path}")


def _run_pipeline(raw_csv: pathlib.Path, out_dir: pathlib.Path) -> pathlib.Path:
    meta = ScanMetadata(
        scan_id="golden_normal",
        subject_id="x",
        operator="x",
        started_at_iso="2026-05-22T00:00:00Z",
        duration_sec=10,
        left_camera_mask=0x01,   # camera 0 only
        right_camera_mask=0,
        reduced_mode=False,
    )

    from dataclasses import dataclass

    @dataclass
    class _TrivialCal:
        c_min: np.ndarray
        c_max: np.ndarray
        i_min: np.ndarray
        i_max: np.ndarray

    cal = _TrivialCal(
        c_min=np.zeros((2, 8), dtype=np.float32),
        c_max=np.ones((2, 8), dtype=np.float32),
        i_min=np.zeros((2, 8), dtype=np.float32),
        i_max=np.full((2, 8), 500.0, dtype=np.float32),
    )

    pipeline = default_pipeline(
        metadata=meta,
        calibration=cal,
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

    corrected_files = [p for p in out_dir.glob("*.csv") if not p.name.endswith("_raw.csv")]
    if not corrected_files:
        raise RuntimeError(
            "Pipeline produced no corrected CSV — "
            "CsvSink.consume('final') may still be a no-op or no interval closed."
        )
    return corrected_files[0]


def main() -> None:
    print("Regenerating golden fixtures...")

    # 1. Generate raw fixture
    _write_raw_csv(RAW_OUT)

    # 2. Run pipeline in a temp dir and collect corrected output
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = pathlib.Path(tmp)
        corrected = _run_pipeline(RAW_OUT, tmp_path)

        # 3. Copy corrected CSV to the fixtures directory
        shutil.copy2(corrected, GOLD_OUT)
        print(f"Wrote golden fixture: {GOLD_OUT}")

    print("Done. Commit both files.")


if __name__ == "__main__":
    main()
