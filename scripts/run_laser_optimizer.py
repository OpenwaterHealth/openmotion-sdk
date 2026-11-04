#!/usr/bin/env python3
"""Run the laser optimizer (sdk#335) headless.

Connects, configures the cameras, loads laser power, then lets
LaserOptimizerWorkflow tune the TA pulse width and camera gains until the
near cameras read the target dark-corrected mean, scans at that setting and
restores the stock settings. Prints every probe, the result, the session
metadata written to the scan DB, and a read-back of the console registers
the run changed.

Engineering experiments only: the run widens the safety FPGAs' pulse-width
and rate limits while it is active (see omotion/LaserOptimizerWorkflow.py).

Usage
-----
    python scripts/run_laser_optimizer.py --right-mask 0xFF --duration 10
    python scripts/run_laser_optimizer.py --left-mask 0x18 --right-mask 0x00 --target 120
"""

import argparse
import json
import logging
import os
import sys
import threading
import time

from omotion import MotionInterface
from omotion.LaserOptimizerWorkflow import (
    LaserOptimizerRequest,
    OptimizerConfig,
    summary_text,
)
from omotion.ScanWorkflow import ConfigureRequest, ScanRequest
from omotion.laser import FpgaMap


def parse_args():
    def mask(x):
        return int(x, 0)

    p = argparse.ArgumentParser(description="Laser optimizer scan (sdk#335)")
    p.add_argument("--left-mask", type=mask, default=0xFF)
    p.add_argument("--right-mask", type=mask, default=0xFF)
    p.add_argument("--duration", type=int, default=10, help="final scan seconds")
    p.add_argument("--target", type=float, default=100.0, help="target mean, DN")
    p.add_argument("--probe-frames", type=int, default=30,
                   help="light frames each probe averages")
    p.add_argument("--subject-id", default="laser_optimizer")
    p.add_argument("--data-dir", default="scan_data")
    p.add_argument("--verbose", action="store_true")
    return p.parse_args()


def read_console_registers(console) -> dict:
    fpga_map = FpgaMap()
    out = {}
    for name in ("TA_PULSE_WIDTH", "EE_PULSE_WIDTH_UL", "OPT_PULSE_WIDTH_UL",
                 "EE_RATE_LL", "OPT_RATE_LL", "EE_STATUS", "OPT_STATUS"):
        e = fpga_map.get_entry_by_friendly_name(name)
        n = int(str(e["data_size"]).rstrip("B")) // 8
        data, _ = console.read_i2c_packet(
            mux_index=e["mux_idx"], channel=e["channel"], device_addr=e["i2c_addr"],
            reg_addr=e["start_address"], read_len=n)
        raw = int.from_bytes(bytes(data), "little") if data else None
        scale = e.get("scale")
        out[name] = (raw, round(raw * scale, 1) if raw is not None and scale else None)
    return out


def read_camera_settings(iface, left: int, right: int) -> dict:
    """Commanded exposure (rows) and analog gain per active camera, from the
    sensor firmware's cached camera telemetry (refreshed ~1 s). expo_cmd is
    the register the writers set; expo_applied only moves while streaming."""
    out = {}
    for side, sensor, mask in (("left", iface.left, left), ("right", iface.right, right)):
        if not mask:
            continue
        telem = sensor.get_camera_telemetry() or {}
        cams = telem.get("cameras") or []
        out[side] = [
            (cam + 1, c.get("expo_cmd"), c.get("again_x"))
            for cam, c in enumerate(cams) if mask & (1 << cam)
        ]
    return out


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )
    os.makedirs(args.data_dir, exist_ok=True)
    db_path = os.path.join(args.data_dir, "scans.db")
    iface = MotionInterface(data_dir=args.data_dir, scan_db_path=db_path,
                            operator_id="run_laser_optimizer")
    iface.start(wait=True, wait_timeout=10)
    try:
        if not iface.wait_for_ready(console=True, sensors=1, timeout=30):
            print("Console and at least one sensor must be connected.")
            return 1
        # Give each requested side time to finish connecting.
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and not all(
                s.is_connected() for s, m in ((iface.left, args.left_mask),
                                              (iface.right, args.right_mask)) if m):
            time.sleep(0.2)
        left = args.left_mask if iface.left.is_connected() else 0
        right = args.right_mask if iface.right.is_connected() else 0
        print(f"Masks: left=0x{left:02X} right=0x{right:02X}")

        done = threading.Event()
        configured = {}
        iface.start_configure_camera_sensors(
            ConfigureRequest(left_camera_mask=left, right_camera_mask=right),
            on_complete_fn=lambda r: (configured.update(ok=r.ok, error=r.error), done.set()),
        )
        if not done.wait(180) or not configured.get("ok"):
            print(f"Camera configure failed: {configured.get('error')}")
            return 1
        if not iface.apply_laser_power():
            print(f"Laser power load failed: {iface.console.laser_limits_error}")
            return 1
        print("Before:", read_console_registers(iface.console))
        print("Cameras before (cam, exposure rows, gain):",
              read_camera_settings(iface, left, right))

        cfg = OptimizerConfig(target_mean_dn=args.target,
                              probe_light_frames=args.probe_frames)
        request = LaserOptimizerRequest(
            final_scan=ScanRequest(
                subject_id=args.subject_id, duration_sec=args.duration,
                left_camera_mask=left, right_camera_mask=right,
                write_corrected_csv=False, write_telemetry_csv=False,
            ),
            config=cfg,
        )

        def on_probe(probe, decision):
            print(probe.describe())
            for r in probe.others:
                print(f"    {r.side} camera {r.cam_id + 1}: {r.mean_dn:.1f} DN, "
                      f"clipped {r.clipped_fraction:.2%}, dark {r.dark_dn:.1f}")
            nxt = f" {decision.setting.describe()}" if decision.setting else ""
            print(f"  -> {decision.action.value}{nxt}: {decision.reason}")

        finished = threading.Event()
        holder = {}
        if not iface.start_laser_optimizer(
            request, on_probe=on_probe,
            on_scan_started=lambda label, s: print(f"Final scan {label} at {s.describe()}"),
            on_complete=lambda r: (holder.update(result=r), finished.set()),
        ):
            print(f"Refused: {iface.laser_optimizer_workflow.last_result.message}")
            return 1
        finished.wait()
        result = holder["result"]
        print()
        print(summary_text(result, cfg))
        print(f"Outcome: {result.outcome.value}: {result.message}; restored={result.restored}")
        print("After:", read_console_registers(iface.console))
        time.sleep(2.0)   # camera telemetry cache refreshes ~1 s
        print("Cameras after (cam, exposure rows, gain):",
              read_camera_settings(iface, left, right))

        if result.scan_label:
            from omotion.ScanDatabase import ScanDatabase
            db = ScanDatabase(db_path)
            try:
                sess = db.get_session_by_label(result.scan_label)
                meta = sess.get("session_meta") if sess else None
                meta = json.loads(meta) if isinstance(meta, str) else meta
                block = (meta or {}).get("laser_optimizer", {})
                print("Session meta laser_optimizer:",
                      {k: v for k, v in block.items() if k != "probes"})
                rows = sum(1 for _ in db.iter_session_data(int(sess["id"])))
                print(f"Session rows: {rows}")
            finally:
                db.close()
        return 0 if result.outcome.value == "completed" else 2
    finally:
        iface.stop()


if __name__ == "__main__":
    sys.exit(main())
