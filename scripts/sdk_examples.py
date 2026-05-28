#!/usr/bin/env python3
"""Runnable examples for the omotion SDK public API (see docs/API.md).

Each example drives ONE operation against **connected hardware** (console +
both sensor modules) and prints the result to the terminal. Run one:

    python scripts/sdk_examples.py connect
    python scripts/sdk_examples.py contact-quality
    python scripts/sdk_examples.py configure
    python scripts/sdk_examples.py scan
    python scripts/sdk_examples.py read-scan
    python scripts/sdk_examples.py test-scan

or run them all in sequence (shares one connection):

    python scripts/sdk_examples.py            # same as 'all'

These assume the hardware is attached. They are examples / smoke checks, not
unit tests — the `scan` and `test-scan` examples turn the laser on. Output
(scan DB + CSVs) goes under a temp directory printed at connect time.
"""

from __future__ import annotations

import argparse
import tempfile
import threading
from collections import defaultdict
from pathlib import Path

from omotion import MotionInterface, ScanDatabase, __version__
from omotion.ScanWorkflow import ScanRequest, ConfigureRequest
from omotion.CalibrationWorkflow import CalibrationRequest, CalibrationThresholds

# ── shared config ────────────────────────────────────────────────────────────
DATA_DIR = str(Path(tempfile.gettempdir()) / "omotion_examples")
DB_PATH = str(Path(DATA_DIR) / "scans.db")
LEFT_MASK = 0xFF      # all 8 left cameras
RIGHT_MASK = 0xFF     # all 8 right cameras


def _banner(title: str) -> None:
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def connect(*, timeout: float = 10.0) -> MotionInterface:
    """Construct + start a MotionInterface and block until the hardware is up.

    Devices enumerate asynchronously after ``start()`` (the console and both
    sensors land over a few seconds via the hotplug monitor), so we wait on
    ``wait_for_ready`` rather than reading connection state immediately."""
    Path(DATA_DIR).mkdir(parents=True, exist_ok=True)
    iface = MotionInterface(
        data_dir=DATA_DIR,
        scan_db_path=DB_PATH,
        operator_id="sdk-examples",
    )
    iface.start(wait=True, wait_timeout=2.0)
    iface.wait_for_ready(console=True, sensors=2, timeout=timeout)
    console, left, right = iface.is_device_connected()
    print(f"connected — console={console}  left={left}  right={right}")
    print(f"output dir: {DATA_DIR}")
    if not (console and left and right):
        print("WARNING: not all devices connected — examples may produce no data.")
    return iface


# ── examples (one per API operation) ─────────────────────────────────────────

def example_connect(iface: MotionInterface) -> None:
    _banner("connect — MotionInterface")
    print("SDK version:", __version__)
    print("default trigger config keys:", sorted(iface.default_trigger_config))


def example_contact_quality(iface: MotionInterface) -> None:
    _banner("contact quality — contact_quality_workflow.check()")
    result = iface.contact_quality_workflow.check(
        duration_sec=2.0,
        rolling_window=10,
        dark_threshold_per_camera=[3.0] * 8,
        light_threshold_per_camera=[15.0] * 8,
        left_camera_mask=LEFT_MASK,
        right_camera_mask=RIGHT_MASK,
    )
    print(f"overall passed: {result.passed}   ({result.duration_sec:.1f}s)")
    print(f"{'cam':>6} {'pass':>6} {'light_dn':>9} {'dark_dn':>8}   reason")
    for (side, cam), r in sorted(result.per_camera.items()):
        print(f"{side[0].upper()}{cam + 1:<5} {str(r.passed):>6} "
              f"{r.light_avg_dn:>9.2f} {r.dark_max_dn:>8.2f}   {r.reason}")


def example_configure(iface: MotionInterface) -> None:
    _banner("configure cameras — start_configure_camera_sensors()")
    done = threading.Event()
    holder: dict = {}
    started = iface.start_configure_camera_sensors(
        ConfigureRequest(left_camera_mask=LEFT_MASK, right_camera_mask=RIGHT_MASK),
        on_complete_fn=lambda res: (holder.__setitem__("res", res), done.set()),
    )
    print("started:", started)
    done.wait(timeout=60)
    res = holder.get("res")
    print("result:", f"ok={res.ok}  error={res.error!r}" if res else "(no result within timeout)")


def example_scan(iface: MotionInterface, *, duration_sec: int = 5) -> None:
    _banner("scan — start_scan()")
    started = iface.start_scan(ScanRequest(
        subject_id="example",
        duration_sec=duration_sec,
        left_camera_mask=LEFT_MASK,
        right_camera_mask=RIGHT_MASK,
    ))
    print("started:", started)
    if not started:
        print("refused:", iface.scan_workflow.last_scan_error)
        return
    sw = iface.scan_workflow
    while sw.running:
        sw.await_complete(timeout_sec=1.0)
    print("session label:", sw.current_scan_label)
    print("error:        ", sw.last_scan_error)
    print("canceled:     ", sw.last_scan_canceled)
    print("scan DB:      ", DB_PATH)


def example_read_scan(iface: MotionInterface | None = None) -> None:
    _banner("read scan — ScanDatabase")
    if not Path(DB_PATH).exists():
        print(f"no scan DB at {DB_PATH} — run the 'scan' example first.")
        return
    db = ScanDatabase(db_path=DB_PATH)
    try:
        sessions = list(db.iter_sessions())
        print(f"{len(sessions)} session(s); most recent:")
        for s in sessions[-5:]:
            print(f"  id={s['id']:>4}  {s['session_label']}")
        if not sessions:
            return
        sid = sessions[-1]["id"]
        counts: dict = defaultdict(int)
        t_lo: dict = defaultdict(lambda: float("inf"))
        t_hi: dict = defaultdict(lambda: float("-inf"))
        for row in db.iter_session_data(sid):
            key = (row["side"], row["cam_id"])
            counts[key] += 1
            t = row["timestamp_s"]
            t_lo[key] = min(t_lo[key], t)
            t_hi[key] = max(t_hi[key], t)
        print(f"latest session {sid} — {sum(counts.values())} session_data rows:")
        for key in sorted(counts):
            dur = t_hi[key] - t_lo[key]
            rate = counts[key] / dur if dur > 0 else 0.0
            print(f"  side={key[0]} cam={key[1]:>3}  n={counts[key]:>6}  ~{rate:.0f} Hz")
    finally:
        db.close()


def example_test_scan(iface: MotionInterface, *, duration_sec: int = 5) -> None:
    _banner("test scan — start_test_scan()")
    # Loose thresholds so the example reports values rather than gating on them.
    thresholds = CalibrationThresholds(
        min_mean_per_camera=[0.0] * 8,
        min_contrast_per_camera=[0.0] * 8,
        min_bfi_per_camera=[-1e9] * 8,
        min_bvi_per_camera=[-1e9] * 8,
    )
    done = threading.Event()
    holder: dict = {}
    started = iface.start_test_scan(
        CalibrationRequest(
            operator_id="sdk-examples",
            output_dir=DATA_DIR,
            left_camera_mask=LEFT_MASK,
            right_camera_mask=RIGHT_MASK,
            thresholds=thresholds,
            duration_sec=duration_sec,
        ),
        on_complete_fn=lambda res: (holder.__setitem__("res", res), done.set()),
    )
    print("started:", started)
    done.wait(timeout=duration_sec + 60)
    res = holder.get("res")
    if res is None:
        print("(no result within timeout)")
        return
    print(f"ok={res.ok}  passed={res.passed}  canceled={res.canceled}  error={res.error!r}")
    print(f"{'cam':>6} {'mean':>9} {'contrast':>9} {'bfi':>8} {'bvi':>8}   bfi_test")
    for r in res.rows:
        print(f"{r.side[0].upper()}{r.cam_id + 1:<5} {r.mean:>9.2f} {r.avg_contrast:>9.4f} "
              f"{r.bfi:>8.3f} {r.bvi:>8.3f}   {r.bfi_test}")


# ── dispatch ─────────────────────────────────────────────────────────────────

_EXAMPLES = {
    "connect": example_connect,
    "contact-quality": example_contact_quality,
    "configure": example_configure,
    "scan": example_scan,
    "read-scan": example_read_scan,
    "test-scan": example_test_scan,
}


def _run_all(iface: MotionInterface) -> None:
    for name in ("connect", "configure", "contact-quality", "scan", "read-scan", "test-scan"):
        try:
            _EXAMPLES[name](iface)
        except Exception as exc:  # keep the tour going if one step fails
            print(f"  !! {name} example raised: {exc!r}")


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="omotion SDK API examples — run against connected hardware.")
    parser.add_argument(
        "example", nargs="?", default="all",
        choices=list(_EXAMPLES) + ["all"],
        help="which example to run (default: all)")
    args = parser.parse_args(argv)

    if args.example == "read-scan":
        example_read_scan()  # read-only — no hardware needed
        return

    iface = connect()
    try:
        if args.example == "all":
            _run_all(iface)
        else:
            _EXAMPLES[args.example](iface)
    finally:
        iface.stop()


if __name__ == "__main__":
    main()
