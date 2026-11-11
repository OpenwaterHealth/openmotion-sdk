#!/usr/bin/env python3
"""Bench pass for Optimize Laser (sdk#335), headless.

Runs parts A to C of bloodflow-app docs/optimize-laser-bench-checklist.md on
a connected console and sensor, printing PASS / FAIL per step:

  A  stock scans still work (start_scan gained the scan preparer)
  B  a run, then scans at its settings (re-applied by start_scan itself)
  C  every way back to stock (gate drop, cancel, restore)

The run widens the safety FPGAs' pulse-width and rate limits and starts at
maximum laser output. Put the sensor on a phantom first, and close the app:
USB access is exclusive. Whatever happens, the defaults are restored before
the script exits.

Usage
-----
    python scripts/bench_optimize_laser.py --right-mask 0xFF
    python scripts/bench_optimize_laser.py --left-mask 0xFF --right-mask 0xFF --out bench_out
"""

import argparse
import dataclasses
import json
import logging
import os
import sys
import threading
import time

import numpy as np

from omotion import MotionInterface
from omotion.exposure.estimate import estimate_signal
from omotion.exposure.settings import OptimizeLaserSettings
from omotion.fpga_registers import FpgaRegisterIO
from omotion.OptimizeLaserWorkflow import _OptimizeLaserSink, cameras_from_masks
from omotion.ScanWorkflow import ConfigureRequest, ScanRequest, run_collection_scan

REGISTERS = ("TA_PULSE_WIDTH", "TA_CURRENT_DRV",
             "EE_PULSE_WIDTH_UL", "OPT_PULSE_WIDTH_UL",
             "EE_RATE_LL", "OPT_RATE_LL", "EE_DRIVE_CL", "OPT_DRIVE_CL",
             "EE_STATUS", "OPT_STATUS")
STATUS = ("EE_STATUS", "OPT_STATUS")
RESULTS: list[tuple[str, bool, str]] = []


def step(name: str, ok: bool, detail: str = "") -> bool:
    RESULTS.append((name, bool(ok), detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f": {detail}" if detail else ""), flush=True)
    return bool(ok)


def label(cam) -> str:
    return f"{'LR'[cam[0]]}{cam[1] + 1}"


class Bench:
    def __init__(self, iface, left: int, right: int):
        self.iface = iface
        self.left, self.right = left, right
        self.io = FpgaRegisterIO(iface.console)
        self.cams = cameras_from_masks(left, right)

    def registers(self) -> dict:
        return {name: round(self.io.read(name), 2) for name in REGISTERS}

    def cameras(self) -> dict:
        """Commanded exposure rows and analog gain per camera, from the
        sensor firmware's camera telemetry."""
        out = {}
        for side, sensor, mask in ((0, self.iface.left, self.left),
                                   (1, self.iface.right, self.right)):
            if not mask:
                continue
            cams = (sensor.get_camera_telemetry() or {}).get("cameras") or []
            for pos, c in enumerate(cams):
                if mask & (1 << pos):
                    out[label((side, pos))] = (c.get("expo_cmd"), c.get("again_x"))
        return out

    def cameras_settled(self, expected: dict, timeout: float = 5.0) -> dict:
        """The camera settings once they match ``expected``, or the last
        reading after ``timeout``. The sensor refreshes its camera telemetry
        about once a second, so a read right after a write can be stale."""
        deadline = time.monotonic() + timeout
        while True:
            cams = self.cameras()
            if cams == expected or time.monotonic() >= deadline:
                return cams
            time.sleep(0.5)

    def idle(self, timeout: float = 15.0) -> None:
        """Wait for the previous scan's worker to finish its teardown."""
        deadline = time.monotonic() + timeout
        while self.iface.scan_workflow.running and time.monotonic() < deadline:
            time.sleep(0.1)

    def latched(self) -> list[str]:
        return [f"{n}=0x{int(self.io.read(n)):02X}" for n in STATUS if int(self.io.read(n))]

    def same_as(self, stock: dict) -> tuple[bool, str]:
        now = self.registers()
        diff = {n: (stock[n], v) for n, v in now.items() if abs(v - stock[n]) > 1.0}
        return not diff, ", ".join(f"{n} {a} -> {b}" for n, (a, b) in diff.items())

    def measure(self, seconds: float, left=None, right=None) -> tuple[dict, str | None]:
        """A short scan with no storage; per-camera dark-subtracted signal."""
        left = self.left if left is None else left
        right = self.right if right is None else right
        sink = _OptimizeLaserSink(cameras_from_masks(left, right), 40)
        self.idle()
        try:
            run_collection_scan(self.iface.scan_workflow, sink, subject_id="_bench",
                                duration_sec=seconds, left_camera_mask=left,
                                right_camera_mask=right, raise_on_error=True)
        except RuntimeError as e:
            return {}, f"{e} ({self.iface.scan_workflow.last_scan_error})"
        signals = {}
        for cam, obs in sink.observations().items():
            est = estimate_signal(np.sum(obs.histograms, axis=0), obs.dark_dn, "fit")
            signals[label(cam)] = None if est.signal is None else round(est.signal, 1)
        return signals, None

    def stored_scan(self, seconds: int, subject: str) -> tuple[bool, str | None, str | None]:
        """A scan with the default storage (scan DB); returns (ok, error, label)."""
        wf = self.iface.scan_workflow
        self.idle()
        req = ScanRequest(subject_id=subject, duration_sec=seconds,
                          left_camera_mask=self.left, right_camera_mask=self.right,
                          write_corrected_csv=False, write_telemetry_csv=False,
                          raw_save_max_duration_s=0)
        if not wf.start_scan(req):
            return False, wf.last_scan_error or "start_scan refused", None
        scan_label = wf.current_scan_label
        wf.await_complete(timeout_sec=seconds + 20)
        return wf.last_scan_error is None, wf.last_scan_error, scan_label


def optimize(bench: Bench, settings, *, cancel_after: int | None = None):
    stop = threading.Event()

    def on_iteration(record):
        s = record.settings
        sig = {label(c): (None if r.signal is None else round(r.signal))
               for c, r in sorted(record.cameras.items())}
        print(f"    iter {record.iteration:2d}: {s.pulse_width_us:6.0f} us "
              f"{s.current_ma:6.0f} mA  signal {sig}", flush=True)
        if cancel_after is not None and record.iteration >= cancel_after:
            stop.set()

    bench.idle()
    log_path = os.path.join(bench.out, f"optimize-{time.strftime('%Y%m%d_%H%M%S')}.csv")
    t0 = time.monotonic()
    result = bench.iface.optimize_laser_workflow.run(
        settings, left_camera_mask=bench.left, right_camera_mask=bench.right,
        log_path=log_path, on_iteration=on_iteration, stop_evt=stop)
    return result, time.monotonic() - t0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--left-mask", type=lambda x: int(x, 0), default=0)
    p.add_argument("--right-mask", type=lambda x: int(x, 0), default=0xFF)
    p.add_argument("--out", default="bench_optimize_laser")
    p.add_argument("--target", type=float, default=100.0)
    p.add_argument("--range", type=float, default=10.0, dest="band")
    p.add_argument("--in-range-points", type=int, default=20)
    p.add_argument("--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    os.makedirs(args.out, exist_ok=True)
    db_path = os.path.join(args.out, "scans.db")
    iface = MotionInterface(data_dir=args.out, scan_db_path=db_path,
                            operator_id="bench_optimize_laser")
    iface.start(wait=True, wait_timeout=10)
    workflow = None
    try:
        if not iface.wait_for_ready(console=True, sensors=1, timeout=30):
            print("Console and at least one sensor must be connected (is the app open?).")
            return 2
        deadline = time.monotonic() + 20
        want = [(iface.left, args.left_mask), (iface.right, args.right_mask)]
        while time.monotonic() < deadline and not all(s.is_connected() for s, m in want if m):
            time.sleep(0.2)
        left = args.left_mask if iface.left.is_connected() else 0
        right = args.right_mask if iface.right.is_connected() else 0
        if not (left or right):
            print("No requested sensor is connected.")
            return 2
        print(f"Console firmware {iface.console.get_version()}, "
              f"masks left=0x{left:02X} right=0x{right:02X}", flush=True)

        for sensor, mask in ((iface.left, left), (iface.right, right)):
            if mask:
                sensor.enable_camera_power(0xFF)
        done, configured = threading.Event(), {}
        iface.start_configure_camera_sensors(
            ConfigureRequest(left_camera_mask=left, right_camera_mask=right,
                             power_off_unused_cameras=False),
            on_complete_fn=lambda r: (configured.update(ok=r.ok, error=r.error), done.set()))
        if not done.wait(180) or not configured.get("ok"):
            print(f"Camera configure failed: {configured.get('error')}")
            return 2
        if not iface.apply_laser_power():
            print(f"Laser power load failed: {iface.console.laser_limits_error}")
            return 2

        bench = Bench(iface, left, right)
        bench.out = args.out
        stock = bench.registers()
        stock_cams = bench.cameras()
        print("Stock registers:", stock)
        print("Stock cameras (rows, gain):", stock_cams, flush=True)
        if bench.latched():
            print("Safety fault latched before the pass:", bench.latched())
            return 2

        settings = dataclasses.replace(
            OptimizeLaserSettings(), target_mean=args.target, passing_range=args.band,
            in_range_points=args.in_range_points)

        # ---- A: stock scans ------------------------------------------------
        signals, err = bench.measure(5)
        step("A1 stock scan before any run",
             err is None and len(signals) == len(bench.cams) and not bench.latched(),
             err or f"signal {signals}")
        ok, diff = bench.same_as(stock)
        step("A2 registers unchanged after a stock scan", ok, diff)
        stock_signals = signals

        # ---- B: a run and scans at its settings ----------------------------
        workflow = iface.optimize_laser_workflow
        result, seconds = optimize(bench, settings)
        if not step("B1 Optimize Laser run passes", result.passed,
                    f"{result.status.value if result.status else result.error or 'canceled'}"
                    f", {result.iterations} iteration(s), {seconds:.0f} s"):
            print("    restore problems:", list(result.restore_problems))
            return 1
        s = result.settings
        print(f"    result: {s.pulse_width_us:.0f} us, {s.current_ma:.0f} mA, gains "
              f"{ {label(c): g for c, g in sorted(s.gains.items())} }", flush=True)
        regs = bench.registers()
        step("B2 limits widened, laser at the result, no fault",
             abs(regs["EE_PULSE_WIDTH_UL"] - 2200) < 1 and abs(regs["OPT_PULSE_WIDTH_UL"] - 2200) < 1
             and regs["EE_RATE_LL"] <= 22401 and regs["OPT_RATE_LL"] <= 22401
             and abs(regs["TA_PULSE_WIDTH"] - s.pulse_width_us) < 1
             and workflow.active is result and not bench.latched(),
             str(regs))

        ok, err, scan_label = bench.stored_scan(10, "bench_optimized")
        record = None
        if scan_label:
            from omotion.ScanDatabase import ScanDatabase
            session = ScanDatabase(db_path=db_path).get_session_by_label(scan_label) or {}
            meta = session.get("session_meta")
            meta = json.loads(meta) if isinstance(meta, str) else (meta or {})
            record = meta.get("optimize_laser")
        step("B3a stored scan at the settings starts and completes", ok and not bench.latched(),
             err or scan_label or "")
        step("B3b session_meta.optimize_laser is in the scan DB",
             bool(record) and record.get("pulse_width_us") == s.pulse_width_us,
             json.dumps(record)[:300] if record else "missing")
        signals, err = bench.measure(3)
        priority = [label(c) for c in iface.optimize_laser_workflow.active.cameras
                    if c[1] in (3, 4)] or list(signals)
        on_target = {c: signals.get(c) for c in priority}
        step("B3c priority cameras read on target",
             err is None and all(v is not None and abs(v - args.target) <= args.band
                                 for v in on_target.values()),
             err or f"priority {on_target}, all {signals}")

        # The laser driver back at its stock values, limits left widened.
        # Not a full laser reload (iface.apply_laser_power, what a console
        # reconnect does): that tightens DRIVE_CL under the held peak of the
        # last pulse and latches the interlock on a console calibrated below
        # the run's current (bench run 2). A reconnect with settings active
        # is a known power-cycle; this checks the re-apply itself.
        bench.io.write("TA_PULSE_WIDTH", stock["TA_PULSE_WIDTH"])
        bench.io.write("TA_CURRENT_DRV", stock["TA_CURRENT_DRV"])
        reset = bench.registers()
        signals, err = bench.measure(3)
        regs = bench.registers()
        step("B4 scan re-applies the settings after the laser driver was reset",
             err is None and abs(reset["TA_PULSE_WIDTH"] - stock["TA_PULSE_WIDTH"]) < 1
             and abs(regs["TA_PULSE_WIDTH"] - s.pulse_width_us) < 1
             and abs(regs["EE_PULSE_WIDTH_UL"] - 2200) < 1 and not bench.latched(),
             err or f"TA pulse {reset['TA_PULSE_WIDTH']} -> {regs['TA_PULSE_WIDTH']} us")

        half_l, half_r = left & 0x0F, right & 0x0F
        signals, err = bench.measure(3, half_l, half_r)
        step("B5 scan on fewer cameras than the run",
             err is None and len(signals) == len(cameras_from_masks(half_l, half_r))
             and not bench.latched(), err or f"signal {signals}")
        # The half-mask scan powered the other cameras off; bring them back.
        for sensor, mask in ((iface.left, left), (iface.right, right)):
            if mask:
                sensor.enable_camera_power(0xFF)
        done.clear()
        iface.start_configure_camera_sensors(
            ConfigureRequest(left_camera_mask=left, right_camera_mask=right,
                             power_off_unused_cameras=False),
            on_complete_fn=lambda r: (configured.update(ok=r.ok, error=r.error), done.set()))
        done.wait(180)

        bench.idle()
        try:
            cq = iface.contact_quality_workflow.check(
                duration_sec=2.0, rolling_window=5,
                dark_threshold_per_camera=[3.0] * 8, light_threshold_per_camera=[15.0] * 8,
                left_camera_mask=left, right_camera_mask=right)
            cq_err = iface.scan_workflow.last_scan_error
            detail = cq_err or f"verdict ok={getattr(cq, 'ok', None)} (informational)"
        except Exception as e:
            cq_err, detail = str(e), str(e)
        step("B6 contact-quality check runs with settings active",
             cq_err is None and not bench.latched(), detail)

        # ---- C: back to stock ----------------------------------------------
        dropped = []
        workflow.on_active_changed = dropped.append
        workflow.gate = lambda: "bench: settings may not be used"
        signals, err = bench.measure(3)
        workflow.gate = None
        ok, diff = bench.same_as(stock)
        cams = bench.cameras_settled(stock_cams)
        step("C1 gate drop: scan runs at stock",
             err is None and workflow.active is None and ok and cams == stock_cams
             and dropped == ["bench: settings may not be used"] and not bench.latched(),
             err or diff or f"signal {signals}, cameras {cams if cams != stock_cams else 'stock'}")

        result, seconds = optimize(bench, settings, cancel_after=2)
        ok, diff = bench.same_as(stock)
        cams = bench.cameras_settled(stock_cams)
        step("C2 canceled run restores the defaults",
             result.canceled and result.restored and not result.restore_problems
             and workflow.active is None and ok and cams == stock_cams and not bench.latched(),
             diff or f"{result.iterations} iteration(s), problems {list(result.restore_problems)}"
             + ("" if cams == stock_cams else f", cameras {cams}"))

        result, seconds = optimize(bench, settings)
        passed = result.passed
        problems = workflow.restore_defaults()
        ok, diff = bench.same_as(stock)
        cams = bench.cameras_settled(stock_cams)
        step("C3 run, then Restore default laser settings",
             passed and not problems and workflow.active is None and ok
             and cams == stock_cams and not bench.latched(),
             diff or f"run {'passed' if passed else 'did not pass'} in {result.iterations} "
             f"iteration(s), {seconds:.0f} s, problems {problems}"
             + ("" if cams == stock_cams else f", cameras {cams}"))

        signals, err = bench.measure(5)
        close = all(signals.get(c) is not None and stock_signals.get(c) is not None
                    and abs(signals[c] - stock_signals[c]) <= max(10.0, 0.25 * stock_signals[c])
                    for c in stock_signals)
        step("C4 stock scan after everything matches the first",
             err is None and close and not bench.latched(),
             err or f"signal {signals} (first {stock_signals})")
    finally:
        try:
            if workflow is not None and (workflow.active is not None):
                print("Restoring defaults before exit:", workflow.restore_defaults())
            elif workflow is not None:
                # Belt and braces: a failed step may have left settings behind.
                # Only the cameras this pass configured: a connected sensor
                # that was left out has its cameras powered off.
                print("Final restore:", workflow.restore_defaults(left, right))
        except Exception as e:
            print("FINAL RESTORE FAILED:", e)
        iface.stop()

    failed = [name for name, ok, _ in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} steps passed"
          + (f"; FAILED: {failed}" if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
