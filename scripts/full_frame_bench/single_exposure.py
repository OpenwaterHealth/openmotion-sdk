"""Single-exposure full frames with matched dark subtraction (epic app#480).

Each camera streams every row of every frame at >=0.7 ms rows (one exposure
per image), in phases at identical sensor settings (default laser off -> on
-> off) so the analysis can subtract a dark taken beside the lit frames.

--mode trigger (default): production trigger mode at the console's 1.0 Hz
floor. HTS 34300 x VTS 1378 = 717 us rows / 0.988 s frame. The exposure opens
~10 rows after FSIN (~7 ms at this row time), so the laser pulse needs
--laser-delay ~8300 (fully lit plateau 7.5-9.0 ms, bench 2026-09-29).
--mode freerun: sensor free-runs, laser at 40 Hz. Bench 2026-09-29: the
exposure register does not behave as rows here (25 ms and 250 ms: no laser
light; 1 s: saturated), so don't trust freerun lit frames.

Several cameras (--cam 0 1 ... 7): USB carries ~1,850 rows/s per sensor, so
only one camera streams at a time. All cameras are retimed together and warm
up together (--warmup-s, no rows streamed), then each camera in turn is armed
and run through the phases. Camera switches happen with the trigger stopped
and the frame in flight drained: sensor COMM against a live row stream can
wedge COMM (reproduced 2026-09-29).

Laser toggles happen right after a frame's last row arrives, and the first
--skip-frames frames of every phase are dropped (the first pulse after a
trigger start is black). Output: <out>/cam<N>/ with every kept frame (npy +
lossless raw16 png) and phases.json, which single_exposure_analyze.py reads.
Run with PYTHONPATH pointing at the SDK feature/167 worktree. Close the app.
"""
import argparse
import json
import logging
import queue
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

from omotion import MotionInterface
from omotion.ImageCapture import (
    FpgaRegs, ImageLineError, parse_image_packet, _STREAM_READ_SIZE,
    force_load_fpga,
)
from omotion.i2c_packet import I2C_Packet

SENSOR = 0x36
ROW_S_PER_HTS = 9.032e-6 / 432
PROD = {0x380C: 0x01, 0x380D: 0xB0, 0x380E: 0x0A, 0x380F: 0xD0,
        0x3881: 0x00, 0x3882: 0x0A, 0x3883: 0xD0,
        0x3501: 0x00, 0x3502: 0x48,
        0x3823: 0x20, 0x382E: 0x03, 0x3880: 0x05}
BLC_REGS = (0x4001, 0x4004, 0x4005)
SETTLE_S = 0.1


def wsen(s, cam, reg, val):
    s.switch_camera(cam)
    ok = s.camera_i2c_write(I2C_Packet(device_address=SENSOR, register_address=reg, data=val))
    time.sleep(0.02)
    return ok


def rsen(s, cam, reg):
    r = s.i2c_read_register(SENSOR, reg, read_len=1, reg_addr_size=2, mux_channel=cam)
    return None if (r is False or r is None) else r[0]


def save_png16(path, img):
    try:
        from PIL import Image
        Image.fromarray(img.astype(np.uint16)).save(path)
    except Exception as e:  # PNG is a convenience; the npy is the record
        print("png save failed", e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", default="left")
    ap.add_argument("--cam", type=int, nargs="+", required=True)
    ap.add_argument("--mode", choices=["trigger", "freerun"], default="trigger")
    ap.add_argument("--trigger-hz", type=float, default=1.0)
    ap.add_argument("--hts", type=int, default=34300, help="34300 -> 717 us rows")
    ap.add_argument("--vts", type=int, default=1378, help="sensor minimum")
    ap.add_argument("--expo", type=int, default=8, help="rows")
    ap.add_argument("--skip-frames", type=int, default=2)
    ap.add_argument("--blc", choices=["default", "off", "target"], default="off")
    ap.add_argument("--blc-target", type=int, default=512, help="with --blc target")
    ap.add_argument("--phase-s", type=float, default=7.0)
    ap.add_argument("--phases", default="DLD", help="laser state per phase: D = off, L = on")
    ap.add_argument("--laser-delay", type=int, default=8300, help="LaserPulseDelayUsec for lit phases")
    ap.add_argument("--delays", default=None,
                    help="comma list of LaserPulseDelayUsec: phases become D, L@each, D")
    ap.add_argument("--warmup-s", type=float, default=0.0,
                    help="run all cameras at the slow timing this long before the first phase")
    ap.add_argument("--no-program", action="store_true")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    if a.delays:
        plan = [("D", None)] + [("L", int(d)) for d in a.delays.split(",")] + [("D", None)]
    else:
        plan = [(ph, a.laser_delay if ph == "L" else None) for ph in a.phases]
    cams = list(dict.fromkeys(a.cam))
    segments = [(c, ph, d) for c in cams for ph, d in plan]

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(level=logging.WARNING, filename=str(out / "sdk.log"),
                        format="%(relativeCreated)8d %(name)s %(message)s")
    log = open(out / "run.txt", "w")

    def P(*x):
        msg = " ".join(str(v) for v in x)
        print(msg, flush=True)
        log.write(msg + "\n")
        log.flush()

    row_s = a.hts * ROW_S_PER_HTS
    frame_s = a.vts * row_s
    P("args", vars(a))
    P(f"row {row_s * 1e6:.2f} us, frame {frame_s:.3f} s, {len(segments)} segments")
    if a.mode == "trigger" and frame_s >= 0.995 / a.trigger_hz:
        raise SystemExit("frame must end before the next FSIN")
    iface = MotionInterface(data_dir=str(out / "sdkdata"))
    iface.start(wait=True, wait_timeout=3.0)
    iface.wait_for_ready(console=True, sensors=1, timeout=20)
    t_w = time.monotonic()
    while getattr(iface, a.side) is None or not getattr(iface, a.side).is_connected():
        if time.monotonic() - t_w > 20:
            raise SystemExit(f"{a.side} sensor not connected")
        time.sleep(0.2)
    s = getattr(iface, a.side)
    con = iface.console
    P("fw", s.get_version())
    mask = sum(1 << c for c in cams)
    histo = s.uart.histo
    image_q, discard_q = queue.Queue(), queue.Queue()
    trig_saved = con.get_trigger_json()
    if isinstance(trig_saved, str):
        trig_saved = json.loads(trig_saved)

    streaming = trig_on = img_on = False
    regs = {c: FpgaRegs(s, c) for c in cams}
    blc_saved = {}
    raw = []            # (t, packet)
    toggles = []        # (t, segment index)
    t0 = time.monotonic()

    def collect_for(secs):
        t_end = time.monotonic() + secs
        while time.monotonic() < t_end:
            try:
                raw.append((time.monotonic() - t0, image_q.get(timeout=0.05)))
            except queue.Empty:
                pass

    def drain(min_s, quiet_s=0.3, max_s=5.0):
        """Collect until at least min_s passed and no packet for quiet_s."""
        t_start = time.monotonic()
        last = t_start
        while time.monotonic() - t_start < max_s:
            try:
                raw.append((time.monotonic() - t0, image_q.get(timeout=0.05)))
                last = time.monotonic()
            except queue.Empty:
                if time.monotonic() - t_start >= min_s and time.monotonic() - last >= quiet_s:
                    return True
        return False

    def wait_frame_end(limit_s):
        """Block until a frame's last row arrives (+100 ms of blanking)."""
        t_lim = time.monotonic() + limit_s
        while time.monotonic() < t_lim:
            try:
                pkt = image_q.get(timeout=0.05)
            except queue.Empty:
                continue
            raw.append((time.monotonic() - t0, pkt))
            try:
                if parse_image_packet(pkt).line == 1279:
                    time.sleep(0.1)
                    return True
            except ImageLineError:
                pass
        return False

    try:
        assert s.enable_camera_power(mask), "power"
        time.sleep(0.5)
        if not a.no_program:
            for c in cams:          # one camera per command (multi-camera loads lost COMM)
                t = time.time()
                assert force_load_fpga(s, 1 << c), f"cam{c} force program"
                P(f"cam{c} force-program ok {time.time() - t:.1f}s")
                time.sleep(0.3)
        assert s.camera_configure_registers(mask), "configure"
        from omotion.config import DEFAULT_TRIGGER_CONFIG
        cfg = dict(DEFAULT_TRIGGER_CONFIG)
        cfg.update(LaserPulseSkipInterval=0, LaserPulseSkipDelayUsec=0,   # every pulse lit
                   EnableSyncOut=True, TriggerFrequencyHz=float(a.trigger_hz))
        P("apply_laser_power", iface.apply_laser_power())

        def phase_cfg(ph, delay):
            c = dict(cfg, EnableTaTrigger=(ph == "L"))
            if delay is not None:
                c["LaserPulseDelayUsec"] = delay
            return c

        assert con.set_trigger_json(data=phase_cfg("D", None)), "set trigger"
        assert s.enable_camera_fsin_ext(), "fsin ext"
        histo.flush_stale_data(expected_size=_STREAM_READ_SIZE)
        histo.start_streaming(discard_q, _STREAM_READ_SIZE, image_queue=image_q)
        streaming = True
        assert s.enable_camera(mask), "enable_camera"
        time.sleep(0.5)
        assert s.set_camera_image_mode(True, mask), "image mode"
        img_on = True
        time.sleep(0.3)
        for c in cams:
            assert regs[c].check_id() and regs[c].check_version(), f"cam{c} FPGA"
            if regs[c].stride_capable():
                regs[c].set_stride(0)                   # every row, every frame
            regs[c].quiet()

        # --- sensors: slow timing + BLC variant, all cameras, FSIN stopped ---
        writes = [(0x3823, 0x00), (0x382E, 0x01), (0x3880, 0x00)] if a.mode == "freerun" else []
        # VTS before HTS: an intermediate frame longer than the FSIN period wedges trigger mode
        writes += [(0x380E, a.vts >> 8), (0x380F, a.vts & 0xFF),
                   (0x380C, a.hts >> 8), (0x380D, a.hts & 0xFF),
                   (0x3501, (a.expo >> 8) & 0xFF), (0x3502, a.expo & 0xFF),
                   (0x3881, 0x00), (0x3882, a.vts >> 8), (0x3883, a.vts & 0xFF)]
        if a.blc == "off":
            writes += [(0x4001, 0x00)]
        elif a.blc == "target":
            writes += [(0x4004, (a.blc_target >> 8) & 0xFF), (0x4005, a.blc_target & 0xFF)]
        for c in cams:
            blc_saved[c] = {r: rsen(s, c, r) for r in BLC_REGS}
            ok = all(wsen(s, c, r, v) for r, v in writes)
            bad = {hex(r): got for r, v in writes if (got := rsen(s, c, r)) != v}
            P(f"cam{c}: gain 0x3508={rsen(s, c, 0x3508)} BLC before "
              f"{ {hex(r): v for r, v in blc_saved[c].items()} } writes ok {ok} mismatches {bad or 'none'}")
        time.sleep(max(1.0, 2.5 * frame_s))

        assert con.start_trigger(), "start trigger"
        trig_on = True
        if a.warmup_s > 0:
            P(f"warm-up {a.warmup_s:.0f} s (all cameras framing, no rows streamed)")
            collect_for(a.warmup_s)
            raw.clear()

        for i, (cam, ph, delay) in enumerate(segments):
            prev = segments[i - 1][0] if i else None
            if prev == cam:
                if not wait_frame_end(3.0 / a.trigger_hz):
                    P("no frame end seen before toggle")
                con.stop_trigger()
            else:
                if prev is not None and not wait_frame_end(3.0 / a.trigger_hz):
                    P("no frame end seen before camera switch")
                con.stop_trigger()
                if a.mode == "freerun" and prev is not None:
                    regs[prev].quiet()          # unavoidable: free-run keeps framing
                if not drain(frame_s + 0.2):
                    P("stream did not go quiet before camera switch")
                if prev is not None:
                    regs[prev].quiet()
                regs[cam].arm_sweep(0)
                P(f"[{time.monotonic() - t0:6.1f}s] cam{cam} armed")
            con.set_trigger_json(data=phase_cfg(ph, delay))
            con.start_trigger()
            toggles.append((time.monotonic() - t0, i))
            collect_for(a.phase_s)
    finally:
        # Teardown order: stop FSIN, let the frame in flight drain, THEN talk
        # to the sensor (sensor COMM against a live row stream wedges COMM).
        if trig_on:
            try:
                con.stop_trigger()
            except Exception as e:
                P("stop trigger", e)
        if a.mode == "freerun":
            for r in regs.values():
                try:
                    r.quiet()
                except Exception as e:
                    P("quiet", e)
        drain(frame_s + 0.5)
        for c in cams:
            try:
                regs[c].quiet()
                for r, v in PROD.items():
                    wsen(s, c, r, v)
                for r, v in blc_saved.get(c, {}).items():
                    if v is not None:
                        wsen(s, c, r, v)
            except Exception as e:
                P(f"cam{c} restore regs failed", e)
        time.sleep(0.2)
        for c in cams:
            try:
                regs[c].exit_image_mode()
            except Exception as e:
                P(f"cam{c} fpga exit image failed", e)
        try:
            con.set_trigger_json(data=trig_saved)
        except Exception as e:
            P("restore trigger", e)
        if img_on:
            try:
                P("image-mode exit", s.image_mode_exit_status())
            except Exception as e:
                P("image mode off", e)
        for fn in (lambda: s.disable_camera_fsin_ext(), lambda: s.disable_camera(mask)):
            try:
                fn()
            except Exception as e:
                P("teardown", e)
        if streaming:
            histo.stop_streaming()
            try:
                histo.drain_final(expected_size=_STREAM_READ_SIZE)
            except Exception:
                pass
        iface.stop()

    # --- assemble frames: consecutive rows of one (camera, frame counter) ---
    frames, cur, bad = [], None, 0
    for t, pkt in raw:
        try:
            ln = parse_image_packet(pkt)
        except ImageLineError:
            bad += 1
            continue
        if cur is None or (ln.cam_id, ln.frame_cnt) != (cur["cam"], cur["fc"]):
            cur = {"cam": ln.cam_id, "fc": ln.frame_cnt, "t_first": t, "rows": {}, "overrun": False}
            frames.append(cur)
        cur["rows"][ln.line] = ln.pixels
        cur["overrun"] |= ln.overrun
    P(f"{len(raw)} packets, {bad} bad, {len(frames)} frame runs")

    first_seg = {c: next(i for i, x in enumerate(segments) if x[0] == c) for c in cams}
    seen = defaultdict(int)
    per_cam = defaultdict(list)
    for f in frames:
        seg = max((i for t, i in toggles if t <= f["t_first"]), default=None)
        entry = {"fc": f["fc"], "t_first": round(f["t_first"], 3), "rows": len(f["rows"]),
                 "overrun": f["overrun"], "phase": None}
        if seg is not None and segments[seg][0] == f["cam"]:
            seen[seg] += 1
            t_seg = next(t for t, i in toggles if i == seg)
            if f["t_first"] - t_seg >= SETTLE_S and seen[seg] > a.skip_frames:
                cam, ph, delay = segments[seg]
                k = seg - first_seg[cam]                    # phase index within this camera
                entry["phase"] = f"{ph}{k + 1}" + ("" if delay is None else f"d{delay}")
        if entry["phase"] and entry["rows"] == 1280:
            cdir = out / f"cam{f['cam']}"
            cdir.mkdir(exist_ok=True)
            img = np.stack([f["rows"][r] for r in range(1280)])
            stem = f"{entry['phase']}_fc{f['fc']:03d}"
            np.save(cdir / f"{stem}.npy", img)
            save_png16(cdir / f"{stem}_raw16.png", img)
            entry.update(file=stem, mean=round(float(img.mean()), 2), std=round(float(img.std()), 2),
                         clip0=round(float(np.mean(img == 0)), 4), sat=round(float(np.mean(img >= 1023)), 4))
        per_cam[f["cam"]].append(entry)
    for c in cams:
        kept = [e for e in per_cam[c] if "file" in e]
        by_ph = defaultdict(list)
        for e in kept:
            by_ph[e["phase"]].append(e["mean"])
        P(f"cam{c}: {len(kept)} frames kept; " + ", ".join(
            f"{ph} x{len(v)} mean {np.mean(v):.1f}" for ph, v in by_ph.items()))
        cdir = out / f"cam{c}"
        cdir.mkdir(exist_ok=True)
        (cdir / "phases.json").write_text(json.dumps({
            "args": vars(a), "cam": c,
            "toggles": [(t, segments[i]) for t, i in toggles if segments[i][0] == c],
            "blc_before": {hex(k): v for k, v in blc_saved.get(c, {}).items()},
            "frames": per_cam[c]}, indent=1))
    log.close()


if __name__ == "__main__":
    main()
