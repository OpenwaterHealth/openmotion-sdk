"""Drip-scan bench experiment: slow sensor frames + FPGA sweep, collect lines.

Modes:
  trigger  - production FSIN trigger_mod (only HTS/VTS/expo changed)
  freerun  - FSIN sync + trigger_mod + fix_cnt disabled; sensor free-runs at HTS*VTS

Run with PYTHONPATH pointing at the SDK feature/167 worktree.
"""
import argparse
import json
import logging
import queue
import time
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from omotion.ImageCapture import force_load_fpga as force_program_fpga  # noqa: E402

from omotion import MotionInterface  # noqa: E402
from omotion.ImageCapture import (  # noqa: E402
    FpgaRegs, ImageLineError, parse_image_packet, _STREAM_READ_SIZE,
    REG_STATUS, REG_LINE_CUR_L, REG_LINE_CUR_H,
)
from omotion.i2c_packet import I2C_Packet  # noqa: E402

SENSOR = 0x36
PROD = {0x380C: 0x01, 0x380D: 0xB0, 0x380E: 0x0A, 0x380F: 0xD0,
        0x3881: 0x00, 0x3882: 0x0A, 0x3883: 0xD0,
        0x3501: 0x00, 0x3502: 0x48,
        0x3823: 0x20, 0x382E: 0x03, 0x3880: 0x05}


def wsen(s, cam, reg, val):
    s.switch_camera(cam)
    ok = s.camera_i2c_write(I2C_Packet(device_address=SENSOR, register_address=reg, data=val))
    time.sleep(0.02)
    return ok


def rsen(s, cam, reg):
    r = s.i2c_read_register(SENSOR, reg, read_len=1, reg_addr_size=2, mux_channel=cam)
    return None if (r is False or r is None) else r[0]


def fc_rate(regs, secs):
    """Count FRAME_CNT increments over `secs` (8-bit wrap aware, polled)."""
    t0 = time.monotonic()
    last = regs.frame_count()
    total = 0
    while time.monotonic() - t0 < secs:
        time.sleep(0.05)
        v = regs.frame_count()
        total += (v - last) & 0xFF
        last = v
    return total / secs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", default="left")
    ap.add_argument("--cam", type=int, default=1)
    ap.add_argument("--mode", choices=["trigger", "freerun"], default="freerun")
    ap.add_argument("--hts", type=int, default=36000)
    ap.add_argument("--vts", type=int, default=1312)
    ap.add_argument("--expo", type=int, default=1, help="exposure in rows")
    ap.add_argument("--scene", choices=["dark", "laser"], default="dark")
    ap.add_argument("--trigger-hz", type=float, default=40.0)
    ap.add_argument("--collect-s", type=float, default=4.0)
    ap.add_argument("--no-program", action="store_true")
    ap.add_argument("--stream-toggle", action="store_true",
                    help="write timing/mode regs with sensor in standby (0x0100=0)")
    ap.add_argument("--testpattern", type=lambda x: int(x, 0), default=None,
                    help="write 0x5080 test-pattern reg")
    ap.add_argument("--start-line", type=int, default=0)
    ap.add_argument("--rate-s", type=float, default=3.0)
    ap.add_argument("--no-sweep", action="store_true")
    ap.add_argument("--stride", type=int, default=0, help="FPGA STRIDE register (map v3)")
    ap.add_argument("--raw-collect", action="store_true", help="store packets raw during capture, parse afterwards")
    ap.add_argument("--maxexpo", type=int, default=None, help="write max_expo_a 0x3881-83")
    ap.add_argument("--laser-delay", type=int, default=None)
    ap.add_argument("--composite-steps", type=int, default=0, help="host-step sweep start 0..N-1")
    ap.add_argument("--step-frames", type=float, default=2.0)
    ap.add_argument("--fsin-off", action="store_true", help="stop console trigger before sweep (free-run only)")
    ap.add_argument("--out", default="run")
    a = ap.parse_args()

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(level=logging.WARNING, filename=str(out / "sdk.log"), format="%(relativeCreated)8d %(message)s")
    log = open(out / "run.txt", "w")

    def P(*x):
        msg = " ".join(str(v) for v in x)
        print(msg, flush=True)
        log.write(msg + "\n")
        log.flush()

    P("args", vars(a))
    iface = MotionInterface(data_dir=str(out / "sdkdata"))
    iface.start(wait=True, wait_timeout=3.0)
    iface.wait_for_ready(console=True, sensors=1, timeout=20)
    s = getattr(iface, a.side)
    con = iface.console
    P("fw", s.get_version())
    # (USB printf deliberately left off: it wedges COMM during image streaming)
    cam, mask = a.cam, 1 << a.cam

    histo = s.uart.histo
    image_q = queue.Queue()
    discard_q = queue.Queue()
    trig_saved = con.get_trigger_json()
    if isinstance(trig_saved, str):
        trig_saved = json.loads(trig_saved)
    P("trigger saved", trig_saved)

    streaming = False
    trig_on = False
    img_on = False
    regs = FpgaRegs(s, cam)
    lines = []
    try:
        assert s.enable_camera_power(mask), "power"
        time.sleep(0.5)
        if not a.no_program:
            t = time.time()
            assert force_program_fpga(s, mask), "force program"
            P(f"force-program ok {time.time()-t:.1f}s")
            time.sleep(0.3)
        assert s.camera_configure_registers(mask), "configure"
        from omotion.config import DEFAULT_TRIGGER_CONFIG
        cfg = dict(DEFAULT_TRIGGER_CONFIG)
        cfg["LaserPulseSkipInterval"] = 0      # every frame lit (no scheduled darks) --
        cfg["LaserPulseSkipDelayUsec"] = 0     # REQUIRED with interval 0, else every pulse is a mistimed dark-slot pulse
        if a.laser_delay is not None:
            cfg["LaserPulseDelayUsec"] = a.laser_delay
        if a.scene == "laser":
            P("apply_laser_power", iface.apply_laser_power())
        cfg["EnableSyncOut"] = True
        cfg["EnableTaTrigger"] = (a.scene == "laser")
        cfg["TriggerFrequencyHz"] = float(a.trigger_hz)
        P("set trigger", con.set_trigger_json(data=cfg))
        assert s.enable_camera_fsin_ext(), "fsin ext"
        histo.flush_stale_data(expected_size=_STREAM_READ_SIZE)
        histo.start_streaming(discard_q, _STREAM_READ_SIZE, image_queue=image_q)
        streaming = True
        assert s.enable_camera(mask), "enable_camera"
        time.sleep(0.5)
        # Image mode BEFORE FSIN: suspends the firmware's histogram
        # stall detector (3 missed frames -> 10 s rail-off), which trips
        # intermittently on SPI overruns at scan start.
        assert s.set_camera_image_mode(True, mask), "image mode"
        img_on = True
        time.sleep(0.3)
        P("FPGA id ok", regs.check_id(), "ver ok", regs.check_version(),
          "status", hex(regs.read(REG_STATUS)))
        regs.set_start_line(4095)  # single-line target never reached -> link quiet
        regs.stop_sweep()          # CTRL=image, no sweep: no pushes at all
        assert con.start_trigger(), "start_trigger"
        trig_on = True
        time.sleep(0.5)
        P(f"prod FRAME_CNT rate {fc_rate(regs, 1.5):.1f}/s")
        # drop any histogram-era packets
        while not image_q.empty():
            image_q.get_nowait()

        writes = []
        if a.mode == "freerun":
            writes += [(0x3823, 0x00), (0x382E, 0x01), (0x3880, 0x00)]
        # Order: shrink VTS before stretching HTS -- in trigger mode an
        # intermediate HTS*VTS longer than the FSIN period wedges framing.
        writes += [(0x380E, a.vts >> 8), (0x380F, a.vts & 0xFF),
                   (0x380C, a.hts >> 8), (0x380D, a.hts & 0xFF),
                   (0x3501, (a.expo >> 8) & 0xFF), (0x3502, a.expo & 0xFF)]
        if a.maxexpo is not None:
            writes += [(0x3881, (a.maxexpo >> 16) & 0xFF), (0x3882, (a.maxexpo >> 8) & 0xFF), (0x3883, a.maxexpo & 0xFF)]
        if a.testpattern is not None:
            writes += [(0x5000, 0x3F), (0x5100, a.testpattern), (0x5102, 0x20), (0x5103, 0x04)]
        if a.stream_toggle:
            writes = [(0x0100, 0x00)] + writes + [(0x0100, 0x01)]
        ok = all(wsen(s, cam, r, v) for r, v in writes)
        P("sensor writes ok", ok)
        rb = {hex(r): (None if (v := rsen(s, cam, r)) is None else hex(v))
              for r, _ in writes if r != 0x0100}
        P("readback", rb)
        frame_s = a.hts * a.vts * (9.032e-6 / 432)
        P(f"expected frame period {frame_s:.3f}s")
        time.sleep(max(1.0, 1.5 * frame_s))
        def sfc():
            r = s.i2c_read_register(SENSOR, 0x4610, read_len=4, reg_addr_size=2, mux_channel=cam)
            return None if r in (False, None) else int.from_bytes(bytes(r[:4]), "big")
        sf0 = sfc(); t_sf = time.monotonic()
        rate = fc_rate(regs, max(a.rate_s, 3 * frame_s))
        sf1 = sfc(); dt_sf = time.monotonic() - t_sf
        P(f"sensor vfifo_fcnt {sf0} -> {sf1} over {dt_sf:.1f}s "
          f"({(sf1 - sf0) / dt_sf if None not in (sf0, sf1) else float('nan'):.3f}/s)")
        P(f"slow FRAME_CNT rate {rate:.3f}/s (expect {1/frame_s:.3f})")

        if a.fsin_off and trig_on:
            con.stop_trigger(); trig_on = False
            time.sleep(0.2)
            P("console trigger stopped for sweep")
        if a.no_sweep:
            a.collect_s = 0
        elif a.composite_steps:
            frame_s0 = a.hts * a.vts * (9.032e-6 / 432)
            a.collect_s = 0.3
            t_c0 = time.monotonic()
            for st_line in range(a.composite_steps):
                regs.arm_sweep(st_line)
                t_end = time.monotonic() + a.step_frames * frame_s0
                while time.monotonic() < t_end:
                    try:
                        pkt = image_q.get(timeout=0.01)
                    except queue.Empty:
                        continue
                    try:
                        lines.append((time.monotonic() - t_c0, parse_image_packet(pkt)))
                    except ImageLineError as e:
                        lines.append((time.monotonic() - t_c0, str(e)))
            P(f"composite stepping took {time.monotonic() - t_c0:.2f}s")
        else:
            if a.stride:
                P("FPGA VERSION", regs.read(0x01))
                regs.write(0x0A, a.stride)
                P("STRIDE readback", regs.read(0x0A))
            regs.arm_sweep(a.start_line)
        t0 = time.monotonic()
        raw_pkts = []
        while a.raw_collect and time.monotonic() - t0 < a.collect_s:
            try:
                raw_pkts.append((time.monotonic() - t0, image_q.get(timeout=0.25)))
            except queue.Empty:
                pass
        for tt, pkt in raw_pkts:
            try:
                lines.append((tt, parse_image_packet(pkt)))
            except ImageLineError as e:
                lines.append((tt, str(e)))
        while (not a.raw_collect) and time.monotonic() - t0 < a.collect_s:
            try:
                pkt = image_q.get(timeout=0.25)
            except queue.Empty:
                pass
            else:
                try:
                    ln = parse_image_packet(pkt)
                    lines.append((time.monotonic() - t0, ln))
                except ImageLineError as e:
                    lines.append((time.monotonic() - t0, str(e)))
        regs.stop_sweep()
        try:
            st = regs.read(REG_STATUS)
            cur = regs.read(REG_LINE_CUR_L) | (regs.read(REG_LINE_CUR_H) << 8)
            P(f"after sweep STATUS={st:#04x} LINE_CUR={cur} FRAME_CNT={regs.frame_count()}")
        except IOError as e:
            P("status read failed", e)
        # drain stragglers
        t1 = time.monotonic()
        while time.monotonic() - t1 < 1.0:
            try:
                pkt = image_q.get(timeout=0.2)
                try:
                    lines.append((time.monotonic() - t0, parse_image_packet(pkt)))
                except ImageLineError as e:
                    lines.append((time.monotonic() - t0, str(e)))
            except queue.Empty:
                break
    finally:
        try:
            for r, v in PROD.items():
                wsen(s, cam, r, v)
            if a.testpattern is not None:
                wsen(s, cam, 0x5100, 0x00); wsen(s, cam, 0x5000, 0x3E)
        except Exception as e:
            P("restore regs failed", e)
        if trig_on:
            try:
                con.stop_trigger()
            except Exception as e:
                P("stop trigger", e)
        time.sleep(0.2)
        try:
            if a.stride:
                regs.write(0x0A, 0)
            regs.exit_image_mode()
        except Exception as e:
            P("fpga exit image failed", e)
        try:
            con.set_trigger_json(data=trig_saved)
        except Exception as e:
            P("restore trigger", e)
        if img_on:
            try:
                from omotion.config import OW_CAMERA, OW_CAMERA_IMAGE_MODE
                r = s._send(packetType=OW_CAMERA, command=OW_CAMERA_IMAGE_MODE,
                            reserved=0, data=bytes([mask]), timeout=1.5)
                d = bytes(r.data[: r.data_len]) if r.data else b""
                if len(d) >= 36:
                    gaps = np.frombuffer(d[4:36], dtype="<u4")
                    P(f"image-mode exit: active={d[0]} mask={d[1]:#x} gap_count={gaps.tolist()}")
                else:
                    P("image-mode exit reply", d.hex())
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

    good = [(t, l) for t, l in lines if not isinstance(l, str)]
    bad = [(t, l) for t, l in lines if isinstance(l, str)]
    P(f"lines: {len(good)} good, {len(bad)} bad")
    for t, e in bad[:5]:
        P("  bad", f"{t:.3f}", e)
    if good:
        fcs = Counter(l.frame_cnt for _, l in good)
        P("frame_cnt histogram", dict(sorted(fcs.items())))
        per = defaultdict(list)
        for t, l in good:
            per[l.frame_cnt].append(l)
        items = sorted(per.items())
        if len(items) > 12:
            items = items[:6] + items[-6:]
        for fc, ls in items:
            nums = sorted(x.line for x in ls)
            uniq = sorted(set(nums))
            flags = Counter(x.flags for x in ls)
            gaps = [b - a2 for a2, b in zip(uniq, uniq[1:])]
            P(f" fc={fc}: {len(ls)} lines, uniq {len(uniq)}, first {uniq[:6]} last {uniq[-4:]},"
              f" gap hist {dict(Counter(gaps).most_common(4))}, flags {dict(flags)}")
        # save
        arr_lines = np.array([l.line for _, l in good], np.int32)
        arr_fc = np.array([l.frame_cnt for _, l in good], np.int32)
        arr_t = np.array([t for t, _ in good])
        arr_flags = np.array([l.flags for _, l in good], np.int32)
        px = np.stack([l.pixels for _, l in good])
        np.savez_compressed(out / "lines.npz", line=arr_lines, fc=arr_fc, t=arr_t,
                            flags=arr_flags, px=px)
        means = px.reshape(len(good), -1).mean(axis=1)
        P(f"pixel mean over lines: min {means.min():.1f} med {np.median(means):.1f} max {means.max():.1f}")
        P("line arrival t (first 10):", [f"{t:.3f}:{l.line}" for t, l in good[:10]])
    log.close()


if __name__ == "__main__":
    main()
