#!/usr/bin/env python3
"""Standalone dark-frame schedule integrity check for raw histogram CSVs.

Given a ``*_raw.csv`` produced by ScanWorkflow, verify that every frame
the firmware schedule expects to be a dark frame actually looks dark in
the data. Catches firmware off-by-one symptoms (where a light frame
lands in a slot the SDK treats as dark, polluting the dark-frame
interpolation) and gross ambient-light contamination of the dark slots.

Standalone by design — only depends on ``csv`` and ``numpy``. No SDK
import, so it can be vendored into post-processing pipelines or run on
machines without the openmotion package installed.

Usage
-----
    python scripts/check_dark_schedule.py path/to/scan_left_maskFF_raw.csv

    # custom thresholds / schedule:
    python scripts/check_dark_schedule.py raw.csv \\
        --pedestal 128 --max-u1-above-pedestal 30 --max-std 20 \\
        --discard-count 9 --dark-interval 600

Exit code
---------
    0 — every scheduled dark frame on every camera passes both bounds
    1 — at least one scheduled dark frame failed (or CSV unreadable)

Notes
-----
- Schedule formula matches ``SciencePipeline._is_dark_frame``: first
  dark at ``discard_count + 1`` (= 10), subsequent at
  ``(absolute_frame - 1) % dark_interval == 0`` for
  absolute_frame > discard+1.
- ``frame_id`` in the raw CSV is an 8-bit firmware counter that runs
  ``1,2,…,255,0,1,…`` and wraps every 256 frames. The SDK normally
  unwraps it via ``FrameIdUnwrapper``; this script does its own
  per-camera unwrap (backward-jump = wrap) so it can apply the schedule
  formula on the un-wrapped ``absolute_frame``. Rows are assumed to be
  in arrival order, which is what the raw writer produces.
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from typing import Iterator

import numpy as np

HISTOGRAM_BINS = 1024
HISTO_BIN_INDICES = np.arange(HISTOGRAM_BINS, dtype=np.float64)
HISTO_BIN_SQUARES = HISTO_BIN_INDICES**2
# Firmware ships frame_id as a single byte (counts 1..255 then 0,1,…).
FRAME_ID_MODULUS = 256


@dataclass
class Thresholds:
    pedestal: float = 128.0
    max_u1_above_pedestal: float = 2.0
    max_std: float = 20.0
    discard_count: int = 9
    dark_interval: int = 600

    @property
    def max_u1(self) -> float:
        return self.pedestal + self.max_u1_above_pedestal


@dataclass
class FrameStats:
    cam_id: int
    raw_frame_id: int
    absolute_frame: int
    u1: float
    std: float
    csv_type: str
    fail_reasons: list[str]

    @property
    def ok(self) -> bool:
        return not self.fail_reasons


@dataclass
class CameraStats:
    """Accumulated statistics for one camera."""

    cam_id: int
    total_frames: int = 0
    dark_checked: int = 0
    dark_passed: int = 0
    dark_failed: int = 0
    mislabeled: int = 0
    last_absolute_frame: int = 0
    last_frame_is_dark: bool = False
    last_frame_looks_dark: bool = False
    u1_values: list[float] | None = None
    std_values: list[float] | None = None

    def __post_init__(self) -> None:
        if self.u1_values is None:
            self.u1_values = []
        if self.std_values is None:
            self.std_values = []


class FrameIdUnwrapper:
    """Per-camera ``raw_frame_id`` (0..255, wraps) -> monotonic
    ``absolute_frame``. A backward step from the previous raw value
    counts as one wrap; first frame seen is taken as the absolute
    starting point.
    """

    def __init__(self) -> None:
        self._state: dict[int, dict[str, int]] = {}

    def unwrap(self, cam_id: int, raw: int) -> int:
        s = self._state.get(cam_id)
        if s is None:
            self._state[cam_id] = {"prev_raw": raw, "wraps": 0}
            return raw
        if raw < s["prev_raw"]:
            s["wraps"] += 1
        s["prev_raw"] = raw
        return s["wraps"] * FRAME_ID_MODULUS + raw


def is_scheduled_dark_frame(
    frame_id: int,
    discard_count: int,
    dark_interval: int,
    last_frame: int | None = None,
) -> bool:
    """Mirror of ``SciencePipeline._is_dark_frame``.

    Three rules:
    1. First post-warmup frame (``discard_count + 1``).
    2. Every ``dark_interval`` frames thereafter: ``(frame_id - 1) % dark_interval == 0``.
    3. The final frame of the scan (``last_frame``), which the SDK always
       captures as a closing dark to bracket the interpolation.
    """
    if frame_id == discard_count + 1:
        return True
    if last_frame is not None and frame_id == last_frame:
        return True
    return frame_id > discard_count + 1 and (frame_id - 1) % dark_interval == 0


def compute_u1_std(histogram: np.ndarray) -> tuple[float, float]:
    """Compute the population mean (``u1``) and std of a 1024-bin histogram.

    Returns ``(nan, nan)`` on an empty histogram so the caller can decide
    whether to surface or silently skip.
    """
    total = float(histogram.sum())
    if total <= 0:
        return float("nan"), float("nan")
    u1 = float((HISTO_BIN_INDICES * histogram).sum() / total)
    u2 = float((HISTO_BIN_SQUARES * histogram).sum() / total)
    variance = max(0.0, u2 - u1 * u1)
    return u1, float(np.sqrt(variance))


def parse_row(row: list[str]) -> tuple[int, int, str, np.ndarray] | None:
    """Pull ``(cam_id, frame_id, csv_type, histogram)`` out of one raw CSV row.

    The raw CSV layout is::

        cam_id, frame_id, timestamp_s, type, 0..1023, temperature, sum, tcm, tcl, pdc

    Returns None when the row is too short to contain a full histogram —
    happens occasionally on truncated CSVs (e.g. scan canceled mid-row).
    """
    HISTO_START = 4
    if len(row) < HISTO_START + HISTOGRAM_BINS:
        return None
    try:
        cam_id = int(row[0])
        frame_id = int(row[1])
    except ValueError:
        return None
    csv_type = row[3].strip()
    try:
        histogram = np.array(
            row[HISTO_START : HISTO_START + HISTOGRAM_BINS], dtype=np.int64
        )
    except ValueError:
        return None
    return cam_id, frame_id, csv_type, histogram


@dataclass
class CheckResult:
    """Full results from a single CSV check."""

    cam_stats: dict[int, CameraStats]
    failures: list[FrameStats]
    mislabels: list[FrameStats]
    final_frame_warnings: list[str]

    @property
    def total_checked(self) -> int:
        return sum(cs.dark_checked for cs in self.cam_stats.values())

    @property
    def ok(self) -> bool:
        return (
            not self.failures and not self.mislabels and not self.final_frame_warnings
        )


def _looks_dark(u1: float, std: float, thresholds: Thresholds) -> bool:
    """Independent brightness test — does the histogram data look dark?"""
    if np.isnan(u1):
        return False
    return u1 <= thresholds.max_u1 and std <= thresholds.max_std


def check_file(csv_path: str, thresholds: Thresholds) -> CheckResult:
    """Stream the raw CSV and perform a 3-way cross-check on every
    non-warmup frame:

    1. **Schedule** — is this frame on the dark schedule?
    2. **Label** — does the CSV ``type`` column say ``"dark"``?
    3. **Data** — does the histogram actually look dark (u1 / std)?

    Any disagreement among the three is surfaced.
    """
    unwrapper = FrameIdUnwrapper()
    cam_stats: dict[int, CameraStats] = {}
    failures: list[FrameStats] = []
    mislabels: list[FrameStats] = []

    with open(csv_path, "r", newline="", encoding="utf-8") as fh:
        reader = csv.reader(fh)
        try:
            header = next(reader)
        except StopIteration:
            return CheckResult({}, [], [], [])
        if not header or header[0] != "cam_id":
            raise ValueError(
                f"{csv_path}: header does not start with 'cam_id' — "
                "is this really a raw histogram CSV?"
            )

        for row in reader:
            parsed = parse_row(row)
            if parsed is None:
                continue
            cam_id, raw_frame_id, csv_type, histogram = parsed
            absolute_frame = unwrapper.unwrap(cam_id, raw_frame_id)

            # Per-camera bookkeeping.
            cs = cam_stats.get(cam_id)
            if cs is None:
                cs = CameraStats(cam_id=cam_id)
                cam_stats[cam_id] = cs
            cs.total_frames += 1

            # The three independent axes.
            scheduled_dark = is_scheduled_dark_frame(
                absolute_frame,
                thresholds.discard_count,
                thresholds.dark_interval,
            )
            labeled_dark = csv_type == "dark"
            u1, std = compute_u1_std(histogram)
            looks_dark = _looks_dark(u1, std, thresholds)

            # Track final frame per camera (rows arrive in order).
            cs.last_absolute_frame = absolute_frame
            cs.last_frame_is_dark = scheduled_dark
            cs.last_frame_looks_dark = looks_dark

            # Skip warmup frames entirely — they precede the schedule.
            if csv_type == "warmup":
                continue

            # --- 3-way cross-check ---
            fail_reasons: list[str] = []
            is_mislabel = False

            if scheduled_dark:
                # Scheduled dark: check label and data agree.
                if not labeled_dark:
                    fail_reasons.append(f"type='{csv_type}' but schedule says dark")
                    is_mislabel = True
                if np.isnan(u1):
                    fail_reasons.append("empty histogram")
                elif not looks_dark:
                    parts = []
                    if u1 > thresholds.max_u1:
                        parts.append(
                            f"u1={u1:.2f} > pedestal+"
                            f"{thresholds.max_u1_above_pedestal:.0f}"
                            f"={thresholds.max_u1:.0f}"
                        )
                    if std > thresholds.max_std:
                        parts.append(f"std={std:.2f} > {thresholds.max_std:.0f}")
                    fail_reasons.append("data looks bright: " + "; ".join(parts))
            else:
                # Not a scheduled dark: label should NOT be "dark".
                if labeled_dark:
                    fail_reasons.append(
                        f"type='dark' but frame {absolute_frame} is not "
                        f"on the dark schedule"
                    )
                    is_mislabel = True
                    if not looks_dark:
                        fail_reasons.append(
                            "data also looks bright — likely a labeling bug"
                        )

            # Build the FrameStats for this row (always, for scheduled darks;
            # only on problems for non-scheduled frames).
            if scheduled_dark or fail_reasons:
                fs = FrameStats(
                    cam_id=cam_id,
                    raw_frame_id=raw_frame_id,
                    absolute_frame=absolute_frame,
                    u1=u1,
                    std=std,
                    csv_type=csv_type,
                    fail_reasons=fail_reasons,
                )

                if scheduled_dark:
                    cs.dark_checked += 1
                    if not np.isnan(u1):
                        cs.u1_values.append(u1)
                        cs.std_values.append(std)
                    if fs.ok:
                        cs.dark_passed += 1
                    else:
                        cs.dark_failed += 1
                        failures.append(fs)

                if is_mislabel:
                    mislabels.append(fs)
                    cs.mislabeled += 1

    # --- Reconcile final frames ---
    # During streaming we didn't know which frame was last, so final
    # frames that are legitimate closing darks may have been flagged as
    # mislabels.  Walk each camera's last frame and fix up.
    final_frame_warnings: list[str] = []
    for cid in sorted(cam_stats):
        cs = cam_stats[cid]
        if cs.total_frames == 0:
            continue
        last = cs.last_absolute_frame

        if cs.last_frame_is_dark:
            # Already on the periodic schedule — nothing to reconcile.
            continue

        # The final frame is a scheduled dark (rule 3).  Find any
        # mislabel entry we created for it and reclassify.
        reconciled = False
        for i, fs in enumerate(mislabels):
            if fs.cam_id == cid and fs.absolute_frame == last:
                mislabels.pop(i)
                cs.mislabeled -= 1
                # Re-evaluate as a scheduled dark.
                new_reasons: list[str] = []
                if np.isnan(fs.u1):
                    new_reasons.append("empty histogram")
                elif not _looks_dark(fs.u1, fs.std, thresholds):
                    parts = []
                    if fs.u1 > thresholds.max_u1:
                        parts.append(
                            f"u1={fs.u1:.2f} > pedestal+"
                            f"{thresholds.max_u1_above_pedestal:.0f}"
                            f"={thresholds.max_u1:.0f}"
                        )
                    if fs.std > thresholds.max_std:
                        parts.append(f"std={fs.std:.2f} > {thresholds.max_std:.0f}")
                    new_reasons.append("data looks bright: " + "; ".join(parts))
                fs.fail_reasons = new_reasons
                cs.dark_checked += 1
                if not np.isnan(fs.u1):
                    cs.u1_values.append(fs.u1)
                    cs.std_values.append(fs.std)
                if fs.ok:
                    cs.dark_passed += 1
                else:
                    cs.dark_failed += 1
                    failures.append(fs)
                cs.last_frame_is_dark = True
                reconciled = True
                break

        if not reconciled:
            # Final frame wasn't labeled "dark" and wasn't on the
            # periodic schedule — it was never flagged as a mislabel,
            # but it's still missing its closing dark.
            cs.last_frame_is_dark = False
            detail = "looks dark" if cs.last_frame_looks_dark else "looks bright"
            final_frame_warnings.append(
                f"cam {cid}: final frame {last} is not dark ({detail})"
            )

    return CheckResult(cam_stats, failures, mislabels, final_frame_warnings)


def _format_failure(fs: FrameStats) -> str:
    return (
        f"  cam={fs.cam_id} frame={fs.absolute_frame} (raw={fs.raw_frame_id})  "
        f"type={fs.csv_type:<5s}  u1={fs.u1:>7.2f}  std={fs.std:>6.2f}  "
        f"-> {'; '.join(fs.fail_reasons)}"
    )


def _print_stats_table(result: CheckResult, thresholds: Thresholds) -> None:
    """Print a per-camera summary table to stdout."""
    if not result.cam_stats:
        return

    # Table header.
    hdr = (
        f"{'cam':>3s}  {'frames':>6s}  {'darks':>5s}  "
        f"{'pass':>4s}  {'fail':>4s}  "
        f"{'u1 mean':>7s}  {'u1 max':>6s}  "
        f"{'std mean':>8s}  {'std max':>7s}  "
        f"{'mislbl':>6s}  {'last dark':>9s}"
    )
    sep = "-" * len(hdr)
    print()
    print(sep)
    print(hdr)
    print(sep)

    tot_frames = tot_darks = tot_pass = tot_fail = tot_mis = 0
    all_u1: list[float] = []
    all_std: list[float] = []
    last_dark_ok = True

    for cid in sorted(result.cam_stats):
        cs = result.cam_stats[cid]
        tot_frames += cs.total_frames
        tot_darks += cs.dark_checked
        tot_pass += cs.dark_passed
        tot_fail += cs.dark_failed
        tot_mis += cs.mislabeled

        u1_mean = f"{np.mean(cs.u1_values):.1f}" if cs.u1_values else "--"
        u1_max = f"{np.max(cs.u1_values):.1f}" if cs.u1_values else "--"
        std_mean = f"{np.mean(cs.std_values):.1f}" if cs.std_values else "--"
        std_max = f"{np.max(cs.std_values):.1f}" if cs.std_values else "--"
        last_ok = "yes" if cs.last_frame_is_dark else "NO"
        if not cs.last_frame_is_dark:
            last_dark_ok = False
        mis_str = str(cs.mislabeled) if cs.mislabeled == 0 else f"!{cs.mislabeled}"

        all_u1.extend(cs.u1_values)
        all_std.extend(cs.std_values)

        print(
            f"{cid:>3d}  {cs.total_frames:>6d}  {cs.dark_checked:>5d}  "
            f"{cs.dark_passed:>4d}  {cs.dark_failed:>4d}  "
            f"{u1_mean:>7s}  {u1_max:>6s}  "
            f"{std_mean:>8s}  {std_max:>7s}  "
            f"{mis_str:>6s}  {last_ok:>9s}"
        )

    # Totals row.
    print(sep)
    u1_mean_t = f"{np.mean(all_u1):.1f}" if all_u1 else "--"
    u1_max_t = f"{np.max(all_u1):.1f}" if all_u1 else "--"
    std_mean_t = f"{np.mean(all_std):.1f}" if all_std else "--"
    std_max_t = f"{np.max(all_std):.1f}" if all_std else "--"
    last_t = "yes" if last_dark_ok else "NO"
    mis_t = str(tot_mis) if tot_mis == 0 else f"!{tot_mis}"
    print(
        f"{'all':>3s}  {tot_frames:>6d}  {tot_darks:>5d}  "
        f"{tot_pass:>4d}  {tot_fail:>4d}  "
        f"{u1_mean_t:>7s}  {u1_max_t:>6s}  "
        f"{std_mean_t:>8s}  {std_max_t:>7s}  "
        f"{mis_t:>6s}  {last_t:>9s}"
    )
    print(sep)
    print(
        f"  thresholds: pedestal={thresholds.pedestal:.0f}  "
        f"max_u1={thresholds.max_u1:.0f}  max_std={thresholds.max_std:.0f}  "
        f"discard={thresholds.discard_count}  interval={thresholds.dark_interval}"
    )


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Verify scheduled dark frames in a raw histogram CSV "
        "actually look dark.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("csv_path", help="Path to the *_raw.csv file to check.")
    p.add_argument(
        "--pedestal",
        type=float,
        default=128.0,
        help="Sensor pedestal height in DN (128 for sensor-fw >= 1.5.0, "
        "64 for older firmware).",
    )
    p.add_argument(
        "--max-u1-above-pedestal",
        type=float,
        default=30.0,
        help="Max DN above pedestal for a frame's u1 to still count as dark.",
    )
    p.add_argument(
        "--max-std",
        type=float,
        default=20.0,
        help="Max histogram std for a frame to still count as dark.",
    )
    p.add_argument(
        "--discard-count",
        type=int,
        default=9,
        help="Number of warmup frames the science pipeline discards.",
    )
    p.add_argument(
        "--dark-interval",
        type=int,
        default=600,
        help="Frames between scheduled darks (default 600 = 15 s @ 40 fps).",
    )
    p.add_argument(
        "-q",
        "--quiet",
        action="store_true",
        help="Only print failures and final summary.",
    )
    p.add_argument(
        "--max-print",
        type=int,
        default=20,
        help="Cap the number of failure lines printed (set to 0 for all).",
    )
    return p


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    thresholds = Thresholds(
        pedestal=args.pedestal,
        max_u1_above_pedestal=args.max_u1_above_pedestal,
        max_std=args.max_std,
        discard_count=args.discard_count,
        dark_interval=args.dark_interval,
    )

    try:
        result = check_file(args.csv_path, thresholds)
    except FileNotFoundError:
        print(f"ERROR: file not found: {args.csv_path}", file=sys.stderr)
        return 1
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1

    # --- Statistics table ---
    if not args.quiet:
        print(f"{args.csv_path}")
        _print_stats_table(result, thresholds)

    # --- Mislabel alerts ---
    if result.mislabels:
        print(f"\nMISLABEL: {len(result.mislabels)} dark-frame type mismatches:")
        shown = result.mislabels
        if args.max_print > 0:
            shown = result.mislabels[: args.max_print]
        for fs in shown:
            print(_format_failure(fs))
        if args.max_print > 0 and len(result.mislabels) > args.max_print:
            print(f"  ... (+{len(result.mislabels) - args.max_print} more)")

    # --- Final-frame warnings ---
    if result.final_frame_warnings:
        print(f"\nWARNING: final frame is not a scheduled dark:")
        for w in result.final_frame_warnings:
            print(f"  {w}")

    # --- Brightness failures ---
    if result.failures:
        print(
            f"\nFAIL: {len(result.failures)}/{result.total_checked} "
            f"scheduled dark rows look bright:"
        )
        shown = result.failures
        if args.max_print > 0:
            shown = result.failures[: args.max_print]
        for fs in shown:
            print(_format_failure(fs))
        if args.max_print > 0 and len(result.failures) > args.max_print:
            print(f"  ... (+{len(result.failures) - args.max_print} more)")

    # --- Verdict ---
    if result.ok:
        if not args.quiet:
            print("\nOK: all scheduled dark frames pass.")
        return 0

    return 1


if __name__ == "__main__":
    sys.exit(main())
