"""Thermal soak: hours of full-frame composite images under near-normal operation.

Captures stride-composite full frames (1920x1280 RAW10) from the cameras of
one or both sensor modules for hours, while the sensors run as close to a
normal scan as the image path allows, and logs camera die temperatures and
console temperatures alongside. See docs/ThermalSoak.md (openmotion-sdk#296).

What stays normal:
  - production sensor registers (``--timing production``, the default): the
    exposure, the laser overlap and the time rows wait for readout are those
    of a scan;
  - production trigger: 40 Hz, laser on every frame except the scheduled
    dark frame (every LaserPulseSkipInterval-th); the rows of those dark
    exposures are flagged per composite and summarized as a dark reference;
  - the cameras a normal scan powers (``--scan-mask``, default the app's
    clinical mask 0xC3 = cameras 0, 1, 6, 7; the app powers the others off)
    are powered, configured and triggered at 40 Hz. Cameras of the mask not
    being imaged keep computing histograms (``--idle-fpga histogram``), which
    the firmware simply does not receive.
What differs: imaged cameras run the map-v3 image bitstream in image mode and
USB carries image rows. One sensor's USB sustains ~1,850 rows/s, so only a few
cameras stream at a time (``--concurrent``); the script rotates through groups
of cameras every ``--dwell-s``, stopping the trigger for about a second to
switch (sensor commands must not run against a live row stream).

Outputs in --out:
  run.json                 arguments, versions, trigger config, start time
  composites.csv           one row per composite (stats, provenance)
  camera_telemetry.csv     per camera die temps etc., every --telemetry-s
  console_telemetry.csv    console temperatures + TEC, every --telemetry-s
  events.csv               segments, rotations, per-segment firmware losses,
                           errors, recoveries
  images/<side>/cam<N>/*.npz   lossless composites: image (uint16 raw 10-bit),
                           row_fc, row_t_s, dark_rows, meta (JSON)
  soak.log                 everything, including SDK logging

Run with the SDK checkout (feature/296) on PYTHONPATH; close the app first.
Stop early with Ctrl-C (the run tears down cleanly and keeps its data).
"""
import argparse
import csv
import datetime as dt
import json
import logging
import platform
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from omotion import MotionInterface
from omotion.ImageCapture import CompositeSession, composite_stats, composite_stride, classify_dark_rows
from omotion.config import DEFAULT_TRIGGER_CONFIG

GAIN = [16, 4, 2, 1, 1, 2, 4, 16]
log = logging.getLogger("thermal_soak")


class Stall(RuntimeError):
    pass


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sides", nargs="+", choices=["left", "right"], default=["left"])
    ap.add_argument("--scan-mask", type=lambda x: int(x, 0), default=0xC3,
                    help="cameras powered and triggered, as a normal scan's mask "
                         "(default 0xC3 = app clinical; research default is 0x66)")
    ap.add_argument("--cams", type=int, nargs="+", default=None,
                    help="cameras to image, rotated in groups of --concurrent (default: the scan mask)")
    ap.add_argument("--concurrent", type=int, default=1,
                    help="cameras streaming at once per sensor (production timing: 1-3)")
    ap.add_argument("--timing", choices=["production", "composite"], default="production",
                    help="production = normal sensor registers, 2 s/image for 1 camera; "
                         "composite = 18 us rows, 1 s/image, less normal")
    ap.add_argument("--stride", type=int, default=None, help="override the computed FPGA STRIDE")
    ap.add_argument("--duration-h", type=float, default=3.0)
    ap.add_argument("--dwell-s", type=float, default=120.0,
                    help="seconds per camera group before rotating (ignored with one group)")
    ap.add_argument("--telemetry-s", type=float, default=10.0)
    ap.add_argument("--save-every", type=int, default=1,
                    help="save every Nth composite per camera as npz (stats are logged for all)")
    ap.add_argument("--laser-schedule", choices=["production", "all-lit"], default="production",
                    help="production keeps the scheduled dark frame; all-lit = every pulse lit")
    ap.add_argument("--no-laser", action="store_true", help="TA trigger off (dark soak)")
    ap.add_argument("--idle-fpga", choices=["histogram", "quiet"], default="histogram")
    ap.add_argument("--pedestal", type=float, default=128.0, help="black level for K / dark classification")
    ap.add_argument("--min-free-gb", type=float, default=10.0, help="stop when free disk falls below this")
    ap.add_argument("--max-die-c", type=float, default=105.0,
                    help="stop the run when any camera die temperature exceeds this (0 = no limit)")
    ap.add_argument("--stall-s", type=float, default=30.0,
                    help="recover when an imaged camera sends no row for this long")
    ap.add_argument("--max-recoveries", type=int, default=5)
    ap.add_argument("--power-cycle-cmd", default=None,
                    help="shell command that power-cycles the rig, run before a recovery reconnect")
    ap.add_argument("--no-fpga-load", action="store_true",
                    help="skip the map-v3 SRAM load (only if already loaded since camera power-on)")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    if not 0 < a.scan_mask <= 0xFF:
        ap.error("--scan-mask must be 0x01-0xFF")
    in_mask = [c for c in range(8) if a.scan_mask & (1 << c)]
    a.cams = list(dict.fromkeys(a.cams if a.cams is not None else in_mask))
    if any(not 0 <= c <= 7 for c in a.cams):
        ap.error("cams are 0-7")
    if not set(a.cams) <= set(in_mask):
        ap.error(f"--cams {a.cams} must be inside --scan-mask 0x{a.scan_mask:02X} (cameras {in_mask})")
    if not 1 <= a.concurrent <= len(a.cams):
        ap.error("--concurrent must be between 1 and the number of cameras")
    return a


def groups_of(cams, n):
    return [cams[i:i + n] for i in range(0, len(cams), n)]


def sdk_version():
    import omotion
    root = Path(omotion.__file__).resolve().parent.parent
    try:
        return subprocess.run(["git", "-C", str(root), "describe", "--always", "--dirty", "--tags"],
                              capture_output=True, text=True, timeout=10).stdout.strip() or "unknown"
    except Exception:
        return "unknown"


class CsvLog:
    def __init__(self, path, fields):
        self.f = open(path, "a", newline="", encoding="utf-8")
        self.w = csv.DictWriter(self.f, fieldnames=fields, extrasaction="ignore")
        if self.f.tell() == 0:
            self.w.writeheader()

    def row(self, **kw):
        self.w.writerow(kw)
        self.f.flush()

    def close(self):
        self.f.close()


COMP_FIELDS = ["wall_time", "t_s", "side", "cam", "gain", "seq", "segment", "stride", "file",
               "n_exposures", "fc_first", "fc_last", "lines", "overrun", "dark_exposures", "dark_rows",
               "lit_mean", "lit_std", "K", "band0_mean", "band1_mean", "band2_mean", "band3_mean",
               "sat_frac", "zero_frac", "dark_mean", "dark_std", "t_first_s", "t_last_s"]
CAM_FIELDS = ["wall_time", "t_s", "side", "cam", "imaged", "age_ms", "tpm_avg_c", "tpm0_c", "tpm1_c", "yavg",
              "again_x", "expo_applied", "trig_error", "i2c_err_count", "updated_ms", "sweep_count"]
CON_FIELDS = ["wall_time", "t_s", "t1_c", "t2_c", "t3_c", "tec_v", "tec_set", "tec_curr", "tec_volt", "tec_good"]
EVT_FIELDS = ["wall_time", "t_s", "event", "side", "detail"]


def main(argv=None):
    a = parse_args(argv)
    start_wall = dt.datetime.now().astimezone()
    out = Path(a.out or f"thermal_soak_{start_wall:%Y%m%d_%H%M%S}")
    out.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(level=logging.INFO, filename=str(out / "soak.log"),
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    console_h = logging.StreamHandler(sys.stdout)
    console_h.setLevel(logging.INFO)
    console_h.setFormatter(logging.Formatter("%(asctime)s %(message)s", "%H:%M:%S"))
    log.addHandler(console_h)
    mono0 = time.monotonic()

    def now_s():
        return time.monotonic() - mono0

    def wall(t_s=None):
        return (start_wall + dt.timedelta(seconds=now_s() if t_s is None else t_s)).isoformat(timespec="milliseconds")

    comp_csv = CsvLog(out / "composites.csv", COMP_FIELDS)
    cam_csv = CsvLog(out / "camera_telemetry.csv", CAM_FIELDS)
    con_csv = CsvLog(out / "console_telemetry.csv", CON_FIELDS)
    evt_csv = CsvLog(out / "events.csv", EVT_FIELDS)

    def event(name, side="", detail=""):
        evt_csv.row(wall_time=wall(), t_s=round(now_s(), 3), event=name, side=side,
                    detail=detail if isinstance(detail, str) else json.dumps(detail))
        log.info("%s %s %s", name, side, detail if isinstance(detail, str) else json.dumps(detail))

    stop = {"flag": False, "why": ""}

    def on_sigint(*_):
        stop["flag"], stop["why"] = True, "interrupted (Ctrl-C)"
    signal.signal(signal.SIGINT, on_sigint)

    cfg = dict(DEFAULT_TRIGGER_CONFIG)          # production: 40 Hz, dark frame every 600
    if a.laser_schedule == "all-lit":
        cfg.update(LaserPulseSkipInterval=0, LaserPulseSkipDelayUsec=0)   # both, or the laser stays dark
    cfg.update(EnableSyncOut=True, EnableTaTrigger=not a.no_laser, TriggerFrequencyHz=40.0)

    row_s = (432 if a.timing == "production" else 866) * 9.032e-6 / 432
    stride = a.stride or composite_stride(a.concurrent, row_s)
    groups = groups_of(a.cams, a.concurrent)
    comp_period = stride / 40.0
    rate = len(a.sides) * a.concurrent / comp_period
    est_bytes_h = rate * 3600 * 3.3e6 / max(a.save_every, 1)
    free = shutil.disk_usage(out).free
    log.info("stride %d -> one image per %.2f s per imaged camera; groups %s; ~%.1f GB/h of images "
             "(estimate); %.0f GB free", stride, comp_period, groups, est_bytes_h / 1e9, free / 1e9)
    need = est_bytes_h * a.duration_h + a.min_free_gb * 1e9
    if need > free:
        log.warning("projected %.0f GB (+%.0f GB margin) exceeds %.0f GB free: the run will stop early "
                    "at the margin; raise --save-every to fit", est_bytes_h * a.duration_h / 1e9,
                    a.min_free_gb, free / 1e9)

    iface = MotionInterface(data_dir=str(out / "sdkdata"))
    iface.start(wait=True, wait_timeout=3.0)
    sessions: dict = {}
    trig_saved = None
    run_info = {"args": vars(a), "start": start_wall.isoformat(), "host": platform.node(),
                "python": sys.version.split()[0], "sdk": sdk_version(), "stride": stride,
                "row_us": row_s * 1e6, "image_period_s": comp_period, "groups": groups,
                "trigger": cfg, "gains": GAIN}
    seq = {}
    segment = 0
    recoveries = 0
    t_end = a.duration_h * 3600.0

    def connect():
        iface.wait_for_ready(console=True, sensors=len(a.sides), timeout=60)
        t_w = time.monotonic()
        while any(getattr(iface, sd) is None or not getattr(iface, sd).is_connected() for sd in a.sides):
            if time.monotonic() - t_w > 60:
                raise RuntimeError(f"sensors {a.sides} not all connected")
            time.sleep(0.5)
        iface.console.stop_trigger()
        if not a.no_laser:
            event("laser_power", detail=str(iface.apply_laser_power()))
        for sd in a.sides:
            s = getattr(iface, sd)
            run_info.setdefault("sensor_fw", {})[sd] = s.get_version()
            sess = CompositeSession(s, timing=a.timing, power_mask=a.scan_mask, idle_fpga=a.idle_fpga,
                                    load_fpga=not a.no_fpga_load or recoveries > 0)
            sess.open(log=lambda m, sd=sd: event("open", sd, m))
            sessions[sd] = sess
        try:
            run_info["console_fw"] = iface.console.get_version()
        except Exception:
            pass
        (out / "run.json").write_text(json.dumps(run_info, indent=1, default=str))

    def teardown():
        try:
            iface.console.stop_trigger()
        except Exception:
            log.exception("stop_trigger failed")
        for sd, sess in list(sessions.items()):
            try:
                sess.drain(max_s=2.0)
            except Exception:
                pass
            sess.close()
        sessions.clear()

    def handle(sd, f):
        k = (sd, f.cam_id)
        seq[k] = seq.get(k, 0) + 1
        st = composite_stats(f, a.pedestal)
        t_first, t_last = f.t_first - mono0, f.t_last - mono0
        name = ""
        if (seq[k] - 1) % max(a.save_every, 1) == 0:
            d = out / "images" / sd / f"cam{f.cam_id}"
            d.mkdir(parents=True, exist_ok=True)
            stamp = (start_wall + dt.timedelta(seconds=t_last)).strftime("%Y%m%dT%H%M%S")
            name = f"{sd}_cam{f.cam_id}_{seq[k]:06d}_{stamp}.npz"
            dark, _, dark_fcs = classify_dark_rows(f, a.pedestal)
            meta = {"side": sd, "cam": f.cam_id, "gain": GAIN[f.cam_id], "seq": seq[k], "segment": segment,
                    "stride": stride, "timing": a.timing, "frame_cnts": f.frame_cnts,
                    "dark_fcs": dark_fcs, "wall_first": wall(t_first), "wall_last": wall(t_last),
                    "overrun": f.overrun, "lines": f.lines, "pedestal": a.pedestal}
            np.savez_compressed(d / name, image=f.image, row_fc=f.row_fc,
                                row_t_s=(f.row_t - mono0).astype(np.float64), dark_rows=dark,
                                meta=np.array(json.dumps(meta)))
            name = f"images/{sd}/cam{f.cam_id}/{name}"
        comp_csv.row(wall_time=wall(t_last), t_s=round(t_last, 3), side=sd, cam=f.cam_id,
                     gain=GAIN[f.cam_id], seq=seq[k], segment=segment, stride=stride, file=name,
                     t_first_s=round(t_first, 3), t_last_s=round(t_last, 3),
                     **{kk: (round(v, 5) if isinstance(v, float) else v) for kk, v in st.items()})

    last_temp = {}

    def telemetry(armed):
        t = round(now_s(), 3)
        for sd, sess in sessions.items():
            try:
                tm = sess.sensor.get_camera_telemetry()
            except Exception as e:
                event("telemetry_error", sd, repr(e))
                continue
            if not tm:
                continue
            for c in range(8):
                if not a.scan_mask & (1 << c):
                    continue            # powered off: the firmware cache keeps its LAST reading
                cam = tm["cameras"][c]
                age = tm.get("uptime_ms", 0) - cam.get("updated_ms", 0)
                if not cam.get("valid") or age > 5000:
                    if last_temp.get((sd, c)) is not None:
                        event("camera_dropout", sd, f"cam{c}: telemetry {'invalid' if not cam.get('valid') else f'stale ({age} ms)'}")
                        last_temp[(sd, c)] = None
                    continue
                cam_csv.row(wall_time=wall(t), t_s=t, side=sd, cam=c, imaged=int(c in armed), age_ms=age,
                            **{kk: cam.get(kk) for kk in CAM_FIELDS[6:]})
                temp = cam.get("tpm_avg_c")
                if temp is None:
                    continue
                prev = last_temp.get((sd, c))
                if prev is not None and prev - temp > 15.0:
                    # a camera that stops (e.g. its regulator's thermal shutdown) cools fast
                    event("camera_dropout", sd, f"cam{c}: die temp fell {prev:.1f} -> {temp:.1f} C")
                last_temp[(sd, c)] = temp
                if a.max_die_c and temp > a.max_die_c and not stop["flag"]:
                    stop["flag"], stop["why"] = True, f"{sd} cam{c} die {temp:.1f} C > --max-die-c {a.max_die_c}"
                    event("thermal_limit", sd, stop["why"])
        con = iface.console
        row = {"wall_time": wall(t), "t_s": t}
        try:
            row.update(zip(("t1_c", "t2_c", "t3_c"), (round(x, 3) for x in con.get_temperatures())))
        except Exception as e:
            event("telemetry_error", "console", repr(e))
        try:
            row.update(zip(("tec_v", "tec_set", "tec_curr", "tec_volt", "tec_good"), con.tec_status()))
        except Exception:
            pass
        con_csv.row(**row)

    log.info("output: %s", out.resolve())
    try:
        trig_saved = None
        while not stop["flag"] and now_s() < t_end:
            try:
                if not sessions:
                    connect()
                    if trig_saved is None:
                        trig_saved = iface.console.get_trigger_json()
                    event("sessions_open", detail={"sides": a.sides, "stride": stride, "groups": groups})
                for group in groups:
                    if stop["flag"] or now_s() >= t_end:
                        break
                    segment += 1
                    for sess in sessions.values():
                        sess.arm(group, stride)
                    if not iface.console.set_trigger_json(data=cfg):      # fresh dark schedule, as a scan
                        raise RuntimeError("set_trigger_json failed")
                    if not iface.console.start_trigger():
                        raise RuntimeError("start_trigger failed")
                    event("segment_start", detail={"segment": segment, "cams": group})
                    seg_end = now_s() + (a.dwell_s if len(groups) > 1 else t_end)
                    next_tel = 0.0
                    next_disk = now_s() + 60.0
                    while not stop["flag"] and now_s() < min(seg_end, t_end):
                        for sd, sess in sessions.items():
                            for f in sess.poll(timeout=0.05):
                                handle(sd, f)
                        if now_s() >= next_tel:
                            telemetry(group)
                            next_tel = now_s() + a.telemetry_s
                        for sd, sess in sessions.items():
                            for c in group:
                                if sess.line_age(c) > a.stall_s:
                                    raise Stall(f"{sd} cam{c}: no rows for {sess.line_age(c):.0f} s")
                        if now_s() >= next_disk:
                            next_disk = now_s() + 60.0
                            free_gb = shutil.disk_usage(out).free / 1e9
                            if free_gb < a.min_free_gb:
                                stop["flag"], stop["why"] = True, f"disk free {free_gb:.1f} GB < {a.min_free_gb}"
                    iface.console.stop_trigger()
                    for sd, sess in sessions.items():
                        for f in sess.drain():
                            handle(sd, f)
                        st = sess.disarm()
                        if st:
                            event("segment_end", sd, {"segment": segment, "cams": group, **{
                                kk: [st[kk][c] for c in group] for kk in
                                ("lines_ok", "stage_full", "link_err", "bad_magic", "resync") if kk in st}})
                        else:
                            event("segment_end", sd, {"segment": segment, "cams": group, "fw_status": None})
            except Exception as e:                      # includes Stall
                recoveries += 1
                event("error", detail=f"{type(e).__name__}: {e}")
                log.exception("capture error")
                teardown()
                if recoveries > a.max_recoveries:
                    stop["flag"], stop["why"] = True, f"gave up after {a.max_recoveries} recoveries"
                    break
                if a.power_cycle_cmd:
                    event("power_cycle", detail=a.power_cycle_cmd)
                    subprocess.run(a.power_cycle_cmd, shell=True, timeout=120)
                    time.sleep(10.0)
                event("recovery", detail=f"attempt {recoveries}")
    finally:
        teardown()
        if trig_saved is not None:
            try:
                if isinstance(trig_saved, str):
                    trig_saved = json.loads(trig_saved)
                iface.console.set_trigger_json(data=trig_saved)
            except Exception:
                log.exception("restoring trigger config failed")
        event("end", detail=stop["why"] or "duration reached")
        iface.stop()
        for c in (comp_csv, cam_csv, con_csv, evt_csv):
            c.close()
        total = sum(seq.values())
        log.info("done: %d composites over %.2f h, %d recoveries -> %s", total, now_s() / 3600, recoveries, out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
