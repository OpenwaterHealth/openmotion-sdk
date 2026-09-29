"""Single-exposure full frames with matched dark subtraction (epic app#480).

One continuous stream per camera at >=0.7 ms rows (every row of every frame
drains over the FPGA->MCU link), in phases at identical sensor settings
(default laser off -> on -> off).

--mode trigger (default): production trigger mode at the console's 1.0 Hz
floor. HTS 34300 x VTS 1378 = 717 us rows / 0.988 s frame, 1-row exposure
starting at FSIN, so the 500 us production laser pulse (100 us delay) lands
inside it exactly as in production.
--mode freerun: sensor free-runs, laser at 40 Hz. Bench 2026-09-29: the
exposure register does not behave as rows here (25 ms and 250 ms: no laser
light; 1 s: saturated), so don't trust freerun lit frames.

Laser toggles happen right after a frame's last row arrives (no FSIN can land
mid-readout), and the first --skip-frames frames of every phase are dropped
(the first pulse after a trigger start is black).

Saves every complete frame (npy + lossless raw16 png) plus phases.json.
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
    ap.add_argument("--cam", type=int, required=True)
    ap.add_argument("--mode", choices=["trigger", "freerun"], default="trigger")
    ap.add_argument("--trigger-hz", type=float, default=1.0)
    ap.add_argument("--hts", type=int, default=34300, help="34300 -> 717 us rows")
    ap.add_argument("--vts", type=int, default=1378, help="sensor minimum")
    ap.add_argument("--expo", type=int, default=1, help="rows")
    ap.add_argument("--skip-frames", type=int, default=2)
    ap.add_argument("--blc", choices=["default", "off", "target"], default="default")
    ap.add_argument("--blc-target", type=int, default=512, help="with --blc target")
    ap.add_argument("--phase-s", type=float, default=7.0)
    ap.add_argument("--phases", default="DLD", help="laser state per phase: D = off, L = on")
    ap.add_argument("--laser-delay", type=int, default=None, help="LaserPulseDelayUsec for lit phases")
    ap.add_argument("--delays", default=None,
                    help="comma list of LaserPulseDelayUsec: phases become D, L@each, D")
    ap.add_argument("--no-program", action="store_true")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    if a.delays:
        plan = [("D", None)] + [("L", int(d)) for d in a.delays.split(",")] + [("D", None)]
    else:
        plan = [(ph, a.laser_delay) for ph in a.phases]

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
    P("args", vars(a))
    P(f"row {row_s * 1e6:.2f} us, exposure {a.expo * row_s * 1e3:.3f} ms, frame {a.vts * row_s:.3f} s")
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
    cam, mask = a.cam, 1 << a.cam
    histo = s.uart.histo
    image_q, discard_q = queue.Queue(), queue.Queue()
    trig_saved = con.get_trigger_json()
    if isinstance(trig_saved, str):
        trig_saved = json.loads(trig_saved)

    streaming = trig_on = img_on = False
    regs = FpgaRegs(s, cam)
    blc_saved = {}
    raw = []            # (t, packet)
    toggles = []        # (t, (phase, laser delay))
    t0 = time.monotonic()
    try:
        assert s.enable_camera_power(mask), "power"
        time.sleep(0.5)
        if not a.no_program:
            t = time.time()
            assert force_load_fpga(s, mask), "force program"
            P(f"force-program ok {time.time() - t:.1f}s")
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
        P("trigger (first phase)", phase_cfg(*plan[0]))
        assert con.set_trigger_json(data=phase_cfg(*plan[0])), "set trigger"
        assert s.enable_camera_fsin_ext(), "fsin ext"
        histo.flush_stale_data(expected_size=_STREAM_READ_SIZE)
        histo.start_streaming(discard_q, _STREAM_READ_SIZE, image_queue=image_q)
        streaming = True
        assert s.enable_camera(mask), "enable_camera"
        time.sleep(0.5)
        assert s.set_camera_image_mode(True, mask), "image mode"
        img_on = True
        time.sleep(0.3)
        P("FPGA id", regs.check_id(), "ver", regs.check_version())
        regs.quiet()

        # --- sensor: free-run, long rows, one-period exposure, BLC variant ---
        blc_saved = {r: rsen(s, cam, r) for r in BLC_REGS}
        P("BLC before", {hex(r): v for r, v in blc_saved.items()},
          "again 0x3508/09", rsen(s, cam, 0x3508), rsen(s, cam, 0x3509))
        frame_s = a.vts * row_s
        if a.mode == "trigger":
            assert frame_s < 0.995 / a.trigger_hz, "frame must end before the next FSIN"
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
        ok = all(wsen(s, cam, r, v) for r, v in writes)
        rb = {hex(r): rsen(s, cam, r) for r, _ in writes}
        P("sensor writes ok", ok, "readback", rb)
        time.sleep(max(1.0, 2.5 * a.vts * row_s))     # let the new timing settle

        # --- stream: arm sweep once, then toggle the laser on the console only ---
        # (no sensor COMM traffic while lines stream: it can wedge COMM)
        assert con.start_trigger(), "start trigger"
        trig_on = True
        while not image_q.empty():
            image_q.get_nowait()
        regs.arm_sweep(0)
        t0 = time.monotonic()
        toggles.append((0.0, plan[0]))

        def collect(until):
            while time.monotonic() - t0 < until:
                try:
                    raw.append((time.monotonic() - t0, image_q.get(timeout=0.05)))
                except queue.Empty:
                    pass

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

        def set_phase(ph, delay):
            if not wait_frame_end(3.0 / a.trigger_hz):
                P("no frame end seen before toggle")
            con.stop_trigger()
            con.set_trigger_json(data=phase_cfg(ph, delay))
            con.start_trigger()
            toggles.append((time.monotonic() - t0, (ph, delay)))

        collect(a.phase_s)
        for i, (ph, delay) in enumerate(plan[1:], start=2):
            set_phase(ph, delay)
            collect(i * a.phase_s)
    finally:
        # Teardown order matters: no sensor COMM while lines stream (it wedges
        # COMM). Trigger mode: stop FSIN, let the frame in flight drain, then
        # talk to the sensor. Free-run keeps framing without FSIN, so the
        # sweep stop is the one unavoidable command against a live stream.
        if trig_on:
            try:
                con.stop_trigger()
            except Exception as e:
                P("stop trigger", e)
        if a.mode == "freerun":
            try:
                regs.stop_sweep()
            except Exception as e:
                P("stop sweep", e)
        t_d = time.monotonic() + a.vts * row_s + 0.5
        while time.monotonic() < t_d:
            try:
                raw.append((time.monotonic() - t0, image_q.get(timeout=0.05)))
            except queue.Empty:
                pass
        try:
            regs.stop_sweep()
            for r, v in PROD.items():
                wsen(s, cam, r, v)
            for r, v in blc_saved.items():
                if v is not None:
                    wsen(s, cam, r, v)
        except Exception as e:
            P("restore regs failed", e)
        try:
            regs.exit_image_mode()
        except Exception as e:
            P("fpga exit image failed", e)
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

    # --- assemble frames by FPGA frame counter (consecutive runs of one fc) ---
    frames, cur, bad = [], None, 0
    for t, pkt in raw:
        try:
            ln = parse_image_packet(pkt)
        except ImageLineError:
            bad += 1
            continue
        if cur is None or ln.frame_cnt != cur["fc"]:
            cur = {"fc": ln.frame_cnt, "t_first": t, "rows": {}, "overrun": False}
            frames.append(cur)
        cur["rows"][ln.line] = ln.pixels
        cur["overrun"] |= ln.overrun
    P(f"{len(raw)} packets, {bad} bad, {len(frames)} frame runs, toggles {toggles}")

    seen = defaultdict(int)

    def phase_of(t):
        idx = max(i for i, (tt, _) in enumerate(toggles) if tt <= t)
        seen[idx] += 1
        if t - toggles[idx][0] < SETTLE_S or seen[idx] <= a.skip_frames:
            return None
        ph, delay = plan[idx]
        return f"{ph}{idx + 1}" + ("" if delay is None else f"d{delay}")

    meta = []
    for f in frames:
        n = len(f["rows"])
        ph = phase_of(f["t_first"])
        entry = {"fc": f["fc"], "t_first": round(f["t_first"], 3), "rows": n,
                 "overrun": f["overrun"], "phase": ph}
        if n == 1280 and ph is not None:
            img = np.stack([f["rows"][r] for r in range(1280)])
            stem = f"{ph}_fc{f['fc']:03d}"
            np.save(out / f"{stem}.npy", img)
            save_png16(out / f"{stem}_raw16.png", img)
            entry.update(file=stem, mean=round(float(img.mean()), 2), std=round(float(img.std()), 2),
                         clip0=round(float(np.mean(img == 0)), 4), sat=round(float(np.mean(img >= 1023)), 4))
        meta.append(entry)
        P(" ", entry)
    (out / "phases.json").write_text(json.dumps({"args": vars(a), "toggles": toggles,
                                                 "blc_before": {hex(k): v for k, v in blc_saved.items()},
                                                 "frames": meta}, indent=1))
    log.close()


if __name__ == "__main__":
    main()
