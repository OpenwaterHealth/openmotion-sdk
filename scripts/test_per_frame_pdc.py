#!/usr/bin/env python3
"""Smoke-test the per-frame PDC pipeline against real hardware.

Uses the proper SDK API: subscribes to ConsoleTelemetryPoller's pdc_listener
channel, which drains the firmware ring buffer at 10 Hz and fires once per
sample. The poller is auto-started when the console connects.

Run from the openmotion-sdk directory:

    python scripts/test_per_frame_pdc.py
"""
from __future__ import annotations

import argparse
import csv
import logging
import threading
import time
from pathlib import Path

from omotion.ConsoleTelemetry import PdcSample
from omotion.MotionInterface import MotionInterface
from omotion.ScanWorkflow import _TELEMETRY_HEADERS, _pdc_row


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--csv", type=Path, default=None,
                        help="Write samples to this telemetry CSV using ScanWorkflow's _pdc_row.")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    log = logging.getLogger("test_per_frame_pdc")

    iface = MotionInterface()
    iface.start(wait=False)

    log.info("Waiting for console (up to 10 s)…")
    iface.wait_for_ready(console=True, sensors=0, timeout=10.0)
    if not iface.console.is_connected():
        log.error("Console not connected after 10 s")
        iface.stop()
        return 2
    log.info("Console connected.")

    # Subscribe BEFORE starting trigger so we don't miss early samples.
    samples_lock = threading.Lock()
    samples: list[PdcSample] = []

    def on_pdc(s: PdcSample) -> None:
        with samples_lock:
            samples.append(s)

    iface.console.telemetry.add_pdc_listener(on_pdc)
    log.info("Subscribed to pdc listener.")

    log.info("Setting trigger config and starting trigger (laser ON)…")
    cfg = iface.default_trigger_config
    iface.console.set_trigger_json({
        "frequencyHz": cfg["TriggerFrequencyHz"],
        "triggerPulseWidthUsec": cfg["TriggerPulseWidthUsec"],
        "laserPulseDelayUsec": cfg["LaserPulseDelayUsec"],
        "laserPulseWidthUsec": cfg["LaserPulseWidthUsec"],
        "LaserPulseSkipInterval": cfg["LaserPulseSkipInterval"],
        "LaserPulseSkipDelayUsec": cfg["LaserPulseSkipDelayUsec"],
        "EnableSyncOut": cfg["EnableSyncOut"],
        "EnableTaTrigger": cfg["EnableTaTrigger"],
    })
    iface.console.start_trigger()

    duration_s = args.duration
    log.info("Collecting for %.1f s…", duration_s)
    time.sleep(duration_s)

    log.info("Stopping trigger…")
    iface.console.stop_trigger()
    iface.console.telemetry.remove_pdc_listener(on_pdc)

    # Optionally write a telemetry CSV using the same _pdc_row builder
    # that ScanWorkflow uses. Slow columns are carry-forwarded from the
    # poller's snapshot at the moment each PdcSample was received.
    if args.csv is not None:
        with samples_lock:
            snap_for_csv = list(samples)
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with open(args.csv, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(_TELEMETRY_HEADERS)
            for s in snap_for_csv:
                w.writerow(_pdc_row(s, iface.console.telemetry.get_snapshot()))
        log.info("Wrote %d rows to %s", len(snap_for_csv), args.csv)

    # Stats
    with samples_lock:
        snap = list(samples)

    print()
    print("=" * 60)
    print("PER-FRAME PDC SMOKE TEST RESULTS")
    print("=" * 60)
    print(f"Samples collected:    {len(snap)}")
    total_dropped = sum(s.dropped_delta for s in snap)
    print(f"Total firmware drops: {total_dropped}")
    if snap:
        first = snap[0]
        last = snap[-1]
        elapsed = last.host_recv_timestamp - first.host_recv_timestamp
        rate = (len(snap) - 1) / elapsed if elapsed > 0 else 0
        dark_count = sum(1 for s in snap if s.dark_slot)
        print(f"First frame_idx:      {first.frame_idx}")
        print(f"Last frame_idx:       {last.frame_idx}")
        print(f"frame_idx span:       {last.frame_idx - first.frame_idx + 1}")
        print(f"Observed rate:        {rate:.1f} Hz over {elapsed:.2f} s")
        print(f"dark_slot count:      {dark_count} / {len(snap)}")
        idxs = [s.frame_idx for s in snap]
        gap_list = [(a, b, b - a) for a, b in zip(idxs, idxs[1:]) if b - a > 1]
        backsteps = sum(1 for a, b in zip(idxs, idxs[1:]) if b <= a)
        print(f"Monotonic gaps (>1):  {len(gap_list)}")
        print(f"Non-monotonic steps:  {backsteps}")
        expected = idxs[-1] - idxs[0] + 1
        print(f"Expected samples:     {expected}, got {len(snap)}, missing {expected - len(snap)}")
        if gap_list:
            print(f"\nFirst 10 gap locations:")
            for a, b, skip in gap_list[:10]:
                print(f"  {a:4d} -> {b:4d}  (skipped {skip-1})")
        print()
        print("First 5 samples:")
        for s in snap[:5]:
            print(f"  frame={s.frame_idx:6d}  pdc_mA={s.pdc_mA:7.2f}  dark={int(s.dark_slot)}  dropped_delta={s.dropped_delta}")
        if len(snap) > 5:
            print("...")
            print("Last 3 samples:")
            for s in snap[-3:]:
                print(f"  frame={s.frame_idx:6d}  pdc_mA={s.pdc_mA:7.2f}  dark={int(s.dark_slot)}")
    else:
        print("!!! No samples received")
    print("=" * 60)
    iface.stop()
    return 0 if snap else 1


if __name__ == "__main__":
    raise SystemExit(main())
