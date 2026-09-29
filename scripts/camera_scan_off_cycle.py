#!/usr/bin/env python3
"""Camera on/off duty-cycle scan experiment with a swept off period.

Runs N scans (default 7) with the cameras powered off between scans for a
stepped-up wait: by default the off time sweeps 5 -> 55 minutes in 10-minute
increments (5, 15, 25, 35, 45, 55), and every off period is followed by one
more scan. Each cycle:

  1. Configure cameras on BOTH sensors (default mask 0x0F). The configure
     workflow powers the masked cameras on itself, programs the FPGAs and
     writes the camera registers — required every cycle because step 4
     power-cycles the cameras.
  2. Re-apply the laser driver configuration (registers are volatile).
  3. Run a scan (default 20 min) with the FULL raw histogram CSV recorded
     (raw_save_max_duration_s=None = unbounded raw tee), plus corrected CSV,
     telemetry CSV and the scan DB.
  4. Power the cameras OFF.
  5. Idle for the next value in the off-time schedule, then go again.

--light replaces the full raw CSV with a per-frame statistics CSV
({scan_id}_{subject}_C{NN}_{side}_mask0F_light.csv) holding cam_id,
frame_id, timestamp_s, type, mean, std, temperature — mean/std computed
over each frame's 1024-bin histogram (bin index = 10-bit pixel value).
Corrected CSV, telemetry CSV and the scan DB are written as usual.

The LAST scan is not followed by an idle (no scan would measure its effect;
the cameras are still powered off at exit) — pass --final-off to add one.
If --cycles asks for more waits than the schedule holds, the last schedule
value repeats.
A failed cycle is logged and the experiment continues with the next cycle
after its off period; the next bring-up starts from scratch anyway.
Ctrl-C aborts cleanly: cancels any active scan/configure, powers cameras off.

Output lands in --data-dir:
    {scan_id}_{subject}_C{NN}_{side}_mask0F_raw.csv     full raw histograms
    {scan_id}_{subject}_C{NN}_{side}_mask0F_light.csv   per-frame stats (--light)
    {scan_id}_{subject}_C{NN}.csv                       corrected per-frame BFI/BVI
    {scan_id}_{subject}_C{NN}_telemetry.csv             console telemetry
    scans.db                                            scan database
    camera_scan_off_cycle_YYYYMMDD_HHMMSS.log           this script's log

Disk budget: a 20-minute scan at mask 0x0F on both sides (8 cameras x 40 fps)
is ~380k raw rows ~= 2 GB per scan -> ~14 GB for the default 7-scan run.
With --light the same run is ~25 MB per scan (~170 MB total).

Usage
-----
    # the real experiment: 7 x 20-min scans, off sweep 5,15,25,35,45,55 min (~5.6 h)
    python scripts/camera_scan_off_cycle.py

    # same but saving per-frame stats instead of full raw bins
    python scripts/camera_scan_off_cycle.py --light

    # quick smoke test without the laser
    python scripts/camera_scan_off_cycle.py --scan-minutes 1 --off-schedule 0.5,1 --disable-laser
"""

import argparse
import csv
import logging
import math
import os
import shutil
import sys
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np

from omotion import MotionInterface
from omotion.pipeline.sinks import CsvSink as PipelineCsvSink
from omotion.pipeline.sinks import ScanDBSink, _source_side_indices
# _TelemetryCsvWriter is workflow-private, but light mode must replicate the
# telemetry wiring that skip_default_storage turns off (see run_scan).
from omotion.ScanWorkflow import ConfigureRequest, ScanRequest, _TelemetryCsvWriter

CONFIGURE_TIMEOUT_S = 240.0    # power-on + FPGA programming + camera config (sides run in parallel)
SCAN_FINISH_GRACE_S = 300.0    # pipeline flush / batch correction / DB writes after the duration gate
RECONNECT_GRACE_S = 60.0       # per-cycle wait for console + both sensors to be connected
RAW_CSV_BYTES_PER_ROW = 5500   # ~1024 comma-separated bins + metadata per histogram row
LIGHT_CSV_BYTES_PER_ROW = 60   # cam,frame,ts,type,mean,std,temperature

log = logging.getLogger("camera_scan_off_cycle")


class LightCsvSink:
    """Pipeline sink for --light mode: per-frame statistics instead of bins.

    Subscribes to the same "raw" channel the stock CsvSink uses, but each
    frame becomes one small row: cam_id, frame_id, timestamp_s, type, mean,
    std, temperature. Mean/std are computed over the frame's 1024-bin
    histogram (bin index = 10-bit pixel value), so they describe the raw,
    pre-correction pixel distribution. One file per side:
    ``{scan_id}_{subject_id}_{side}_mask{XX}_light.csv``.
    """

    channels = {"raw"}

    _HEADERS = ["cam_id", "frame_id", "timestamp_s", "type", "mean", "std", "temperature"]

    def __init__(self, output_dir) -> None:
        self._output_dir = str(output_dir)
        self._meta = None
        self._fhs: dict = {}      # side -> file handle
        self._writers: dict = {}  # side -> csv.writer

    def on_scan_start(self, meta) -> None:
        self._meta = meta
        self._fhs = {}
        self._writers = {}

    def consume(self, channel: str, batch) -> None:
        if channel != "raw" or self._meta is None:
            return
        for i, _side_idx, cam_id, frame_type in batch.iter_rows(exclude={"stale"}):
            frame_id = int(batch.frame_ids[i])
            ts = float(batch.timestamp_s[i])
            for side_idx, side_name, mask in _source_side_indices(
                    batch, i, cam_id, self._meta):
                writer = self._writer_for(side_name, mask)
                if writer is None:
                    continue
                histo = np.asarray(
                    batch.raw_histograms[i, side_idx, cam_id, :], dtype=np.float64
                )
                total = float(histo.sum())
                if total > 0:
                    bins = np.arange(histo.size, dtype=np.float64)
                    mean = float((bins * histo).sum() / total)
                    var = float((bins * bins * histo).sum() / total) - mean * mean
                    mean_val = round(mean, 6)
                    std_val = round(math.sqrt(max(var, 0.0)), 6)
                else:
                    mean_val = ""
                    std_val = ""
                temp = (
                    float(batch.temperature_c[i, side_idx, cam_id])
                    if batch.temperature_c is not None else ""
                )
                writer.writerow([cam_id, frame_id, ts, frame_type, mean_val, std_val, temp])

    def on_complete(self) -> None:
        for side, fh in list(self._fhs.items()):
            try:
                fh.flush()
                fh.close()
            except Exception:
                log.exception("LightCsvSink: failed to close %s light CSV", side)
        self._fhs.clear()
        self._writers.clear()

    def _writer_for(self, side: str, mask: int):
        if side in self._writers:
            return self._writers[side]
        meta = self._meta
        try:
            os.makedirs(self._output_dir, exist_ok=True)
            filename = f"{meta.scan_id}_{meta.subject_id}_{side}_mask{mask:02X}_light.csv"
            path = os.path.join(self._output_dir, filename)
            fh = open(path, "w", newline="", encoding="utf-8")
            writer = csv.writer(fh)
            writer.writerow(self._HEADERS)
            self._fhs[side] = fh
            self._writers[side] = writer
            return writer
        except Exception:
            log.exception("LightCsvSink: failed to open light CSV for side=%s", side)
            return None


class _CorrectedOnlyCsvSink(PipelineCsvSink):
    """The stock CsvSink minus its "raw" channel — light mode writes the raw
    record via LightCsvSink instead of the full 1024-bin CSV."""
    channels = {"final"}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Repeated scan / cameras-off duty-cycle experiment with full raw recording"
    )
    parser.add_argument("--cycles", type=int, default=None,
                        help="number of scans (default: one more than the off-schedule "
                             "length, so every off period is followed by a scan)")
    parser.add_argument("--scan-minutes", type=float, default=20.0,
                        help="scan duration per cycle in minutes (default 20)")
    parser.add_argument("--off-schedule", default="5,15,25,35,45,55",
                        help="comma-separated cameras-off minutes between scans, used in "
                             "order; the last value repeats if more waits are needed "
                             "(default: 5..55 sweep in 10-minute steps)")
    parser.add_argument("--mask", type=lambda x: int(x, 0), default=0x0F,
                        help="camera bitmask applied to BOTH sensors (default 0x0F)")
    parser.add_argument("--sensors", type=int, default=2,
                        help="number of sensor modules that must be connected before "
                             "scanning (default 2; use 1 for a single-module bench). "
                             "The scan runs on whichever sensors are present.")
    parser.add_argument("--subject", default="OFFSWEEP",
                        help="subject-id prefix; cycle number is appended (default OFFSWEEP)")
    parser.add_argument("--data-dir", default="scan_off_cycle_data",
                        help="output directory for CSVs, scans.db and the log (default scan_off_cycle_data)")
    parser.add_argument("--light", action="store_true",
                        help="save a per-frame stats CSV (cam, frame, mean, std, temperature) "
                             "instead of the full 1024-bin raw CSV")
    parser.add_argument("--disable-laser", action="store_true",
                        help="use internal frame sync and skip laser power (dry runs)")
    parser.add_argument("--final-off", action="store_true",
                        help="also idle after the last scan (uses the next schedule value)")
    args = parser.parse_args()
    if not (0x01 <= args.mask <= 0xFF):
        parser.error(f"--mask must be 0x01..0xFF, got {args.mask:#x}")
    try:
        args.off_minutes = [float(tok) for tok in args.off_schedule.split(",") if tok.strip()]
    except ValueError:
        parser.error(f"--off-schedule must be comma-separated minutes, got {args.off_schedule!r}")
    if not args.off_minutes:
        parser.error("--off-schedule must contain at least one value")
    if any(m < 0 for m in args.off_minutes):
        parser.error("--off-schedule values must be >= 0")
    if args.cycles is None:
        args.cycles = len(args.off_minutes) + 1
    if args.cycles < 1:
        parser.error("--cycles must be >= 1")
    if not (1 <= args.sensors <= 2):
        parser.error("--sensors must be 1 or 2")
    return args


def warn_if_low_disk(data_dir: Path, cycles: int, scan_s: int, mask: int,
                     light: bool) -> None:
    cameras = 2 * bin(mask & 0xFF).count("1")
    bytes_per_row = LIGHT_CSV_BYTES_PER_ROW if light else RAW_CSV_BYTES_PER_ROW
    estimate = cycles * cameras * 40 * scan_s * bytes_per_row
    try:
        free = shutil.disk_usage(str(data_dir)).free
    except OSError:
        return
    log.info("Estimated raw-CSV footprint for the run: ~%.1f GB (disk free: %.1f GB)",
             estimate / 1e9, free / 1e9)
    if free < estimate * 1.3:
        log.warning("Disk space looks tight for the full run — free some space or reduce --cycles.")


def configure_cameras(iface: MotionInterface, mask: int) -> None:
    """Bring up the masked cameras on both sides: the configure workflow
    powers them on, programs the FPGAs and writes the camera registers.
    power_off_unused_cameras=True keeps anything outside the mask dark."""
    done = threading.Event()
    box: dict = {}

    def _on_complete(result) -> None:
        box["result"] = result
        done.set()

    accepted = iface.start_configure_camera_sensors(
        ConfigureRequest(
            left_camera_mask=mask,
            right_camera_mask=mask,
            power_off_unused_cameras=True,
        ),
        on_complete_fn=_on_complete,
    )
    if not accepted:
        raise RuntimeError("configure refused — a previous configure is still running")
    if not done.wait(CONFIGURE_TIMEOUT_S):
        try:
            iface.cancel_configure_camera_sensors()
        except Exception:
            log.exception("cancel_configure_camera_sensors raised after timeout")
        raise RuntimeError(f"camera configure timed out after {CONFIGURE_TIMEOUT_S:.0f}s")
    result = box.get("result")
    if result is None or not result.ok:
        raise RuntimeError(
            f"camera configure failed: {result.error if result else 'no result delivered'}"
        )
    log.info("Cameras configured (mask 0x%02X both sides)", mask)


def run_scan(iface: MotionInterface, subject_id: str, mask: int,
             duration_s: int, disable_laser: bool, light: bool) -> None:
    # Light mode: the auto-injected CsvSink writes full 1024-bin raw CSVs
    # whenever the raw tee emits, so skip default storage and rebuild the
    # sink set with LightCsvSink standing in for the raw record. Corrected
    # CSV and scan DB stay as in full mode.
    sinks: list = []
    if light:
        sinks.append(LightCsvSink(output_dir=iface.data_dir))
        sinks.append(_CorrectedOnlyCsvSink(output_dir=iface.data_dir, write_corrected=True))
        if iface.scan_db_path:
            sinks.append(ScanDBSink(db_path=iface.scan_db_path))
    request = ScanRequest(
        subject_id=subject_id,
        duration_sec=duration_s,
        left_camera_mask=mask,
        right_camera_mask=mask,
        disable_laser=disable_laser,
        write_telemetry_csv=True,      # full mode; light mode wires its own writer below
        raw_save_max_duration_s=None,  # unbounded raw tee: feeds full bins or light stats
        skip_default_storage=light,
        sinks=sinks,
    )
    if not iface.start_scan(request):
        raise RuntimeError("start_scan refused (previous scan still active?)")
    workflow = iface.scan_workflow
    log.info("Scan %s started: %.1f min, mask 0x%02X both sides, laser %s, %s output",
             workflow.current_scan_label, duration_s / 60, mask,
             "DISABLED (internal fsin)" if disable_laser else "enabled (external fsin)",
             "light-stats" if light else "full-raw")

    # skip_default_storage also skips the workflow's telemetry CSV wiring —
    # recreate it here so light mode keeps the telemetry record.
    telemetry_writer = None
    if light:
        poller = getattr(iface.console, "telemetry", None)
        if poller is not None:
            telemetry_path = Path(iface.data_dir) / f"{workflow.current_scan_label}_telemetry.csv"
            try:
                telemetry_writer = _TelemetryCsvWriter(str(telemetry_path), poller)
            except Exception:
                log.exception("failed to open telemetry CSV %s", telemetry_path)

    try:
        started = time.monotonic()
        deadline = started + duration_s + SCAN_FINISH_GRACE_S
        next_report = started + 300.0
        while workflow.running and time.monotonic() < deadline:
            workflow.await_complete(timeout_sec=10.0)
            now = time.monotonic()
            if workflow.running and now >= next_report:
                log.info("Scan in progress — %.1f/%.1f min elapsed",
                         (now - started) / 60, duration_s / 60)
                next_report += 300.0

        if workflow.running:
            log.error("Scan still running %.0f s past its duration — canceling",
                      SCAN_FINISH_GRACE_S)
            workflow.cancel_scan()
            raise RuntimeError("scan overran its completion grace and was canceled")
        if workflow.last_scan_error:
            raise RuntimeError(f"scan finished with error: {workflow.last_scan_error}")
        if workflow.last_scan_canceled:
            raise RuntimeError("scan was canceled")
        log.info("Scan %s complete in %.1f min",
                 workflow.current_scan_label, (time.monotonic() - started) / 60)
    finally:
        if telemetry_writer is not None:
            telemetry_writer.close()


def power_off_cameras(iface: MotionInterface) -> None:
    """Best-effort power-off of ALL cameras on both sensors."""
    try:
        results = iface.run_on_sensors("disable_camera_power", 0xFF)
    except Exception:
        log.exception("disable_camera_power raised")
        return
    for side, ok in results.items():
        if ok is True:
            log.info("%s: camera power OFF", side)
        else:
            log.warning("%s: camera power-off FAILED (result=%s)", side, ok)


def idle_wait(seconds: float, label: str) -> None:
    if seconds <= 0:
        return
    end = time.monotonic() + seconds
    next_report = 0.0
    while True:
        remaining = end - time.monotonic()
        if remaining <= 0:
            return
        if time.monotonic() >= next_report:
            log.info("%s — %.1f min remaining", label, remaining / 60)
            next_report = time.monotonic() + 300.0
        time.sleep(min(30.0, remaining))


def main() -> int:
    args = parse_args()
    data_dir = Path(args.data_dir).resolve()
    data_dir.mkdir(parents=True, exist_ok=True)
    log_file = data_dir / f"camera_scan_off_cycle_{datetime.now():%Y%m%d_%H%M%S}.log"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(log_file, encoding="utf-8"),
        ],
    )

    scan_s = max(1, int(round(args.scan_minutes * 60)))
    n_waits = args.cycles - 1 + (1 if args.final_off else 0)
    # Wait after scan i (1-based) is waits_s[i-1]; the schedule's last value
    # repeats when --cycles needs more waits than the schedule holds.
    waits_s = [args.off_minutes[min(i, len(args.off_minutes) - 1)] * 60.0
               for i in range(n_waits)]
    estimate_s = args.cycles * (scan_s + 120) + sum(waits_s)

    log.info("=== Camera on/off duty-cycle experiment (off-time sweep) ===")
    log.info("cycles=%d  scan=%.1f min  mask=0x%02X (both sensors)  laser=%s  output=%s",
             args.cycles, scan_s / 60, args.mask,
             "disabled" if args.disable_laser else "enabled",
             "light-stats" if args.light else "full-raw")
    log.info("off periods between scans (min): %s",
             ", ".join(f"{w / 60:g}" for w in waits_s) if waits_s else "none")
    if n_waits > len(args.off_minutes):
        log.info("off-schedule provides %d value(s) but %d waits are needed — "
                 "the last value (%g min) repeats",
                 len(args.off_minutes), n_waits, args.off_minutes[-1])
    log.info("data dir: %s", data_dir)
    log.info("estimated finish: %s (~%.1f h from now)",
             (datetime.now() + timedelta(seconds=estimate_s)).strftime("%Y-%m-%d %H:%M"),
             estimate_s / 3600)
    warn_if_low_disk(data_dir, args.cycles, scan_s, args.mask, args.light)

    iface = MotionInterface(data_dir=str(data_dir), scan_db_path=str(data_dir / "scans.db"))
    iface.start()
    outcomes: dict[int, str] = {}
    try:
        if not iface.wait_for_ready(console=True, sensors=args.sensors, timeout=20.0):
            console_ok, left_ok, right_ok = iface.is_device_connected()
            log.error("System not ready (console=%s left=%s right=%s, need %d sensor(s)) "
                      "— aborting. Close the app first: USB access is exclusive.",
                      console_ok, left_ok, right_ok, args.sensors)
            return 1
        log.info("Console + both sensors connected.")

        for cycle in range(1, args.cycles + 1):
            log.info("=== Cycle %d/%d ===", cycle, args.cycles)
            try:
                if not iface.wait_for_ready(console=True, sensors=args.sensors,
                                            timeout=RECONNECT_GRACE_S):
                    raise RuntimeError(
                        "devices not connected (console/left/right = "
                        f"{iface.is_device_connected()})"
                    )
                configure_cameras(iface, args.mask)
                if not args.disable_laser and not iface.apply_laser_power():
                    raise RuntimeError("apply_laser_power failed")
                time.sleep(1.0)
                run_scan(iface, f"{args.subject}_C{cycle:02d}", args.mask,
                         scan_s, args.disable_laser, args.light)
                outcomes[cycle] = "ok"
            except Exception as exc:
                log.exception("Cycle %d FAILED: %s", cycle, exc)
                outcomes[cycle] = f"FAILED: {exc}"
                try:
                    if iface.scan_workflow.running:
                        iface.scan_workflow.cancel_scan()
                except Exception:
                    log.exception("cancel_scan raised during cycle-failure cleanup")
            finally:
                power_off_cameras(iface)

            if cycle < args.cycles or args.final_off:
                wait_s = waits_s[cycle - 1]
                idle_wait(wait_s,
                          f"Cameras OFF after cycle {cycle}/{args.cycles} "
                          f"({wait_s / 60:g} min off period)")
    except KeyboardInterrupt:
        log.warning("Interrupted by user — cleaning up (cameras will be powered off).")
        try:
            if iface.scan_workflow.running:
                iface.scan_workflow.cancel_scan()
        except Exception:
            log.exception("cancel_scan raised during interrupt cleanup")
        try:
            iface.cancel_configure_camera_sensors()
        except Exception:
            pass
    finally:
        power_off_cameras(iface)
        iface.stop()

    succeeded = sum(1 for verdict in outcomes.values() if verdict == "ok")
    log.info("=== Experiment finished: %d/%d cycles succeeded ===", succeeded, args.cycles)
    for cycle, verdict in sorted(outcomes.items()):
        log.info("  cycle %2d: %s", cycle, verdict)
    log.info("Data in %s", data_dir)
    return 0 if succeeded == args.cycles else 1


if __name__ == "__main__":
    sys.exit(main())
