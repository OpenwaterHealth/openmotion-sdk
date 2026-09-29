"""Sensor-only frame-rate scan: image mode on (stall detector off), FPGA quiet,
write timing/mode, measure sensor vfifo_fcnt rate."""
import os
import tempfile
import time, json, logging, argparse
logging.basicConfig(level=logging.WARNING, filename="ratescan_sdk.log", format="%(relativeCreated)8d %(message)s")
from omotion.ImageCapture import force_load_fpga as force_program_fpga
from omotion import MotionInterface
from omotion.ImageCapture import FpgaRegs
from omotion.i2c_packet import I2C_Packet

ap = argparse.ArgumentParser()
ap.add_argument("--cam", type=int, default=1)
ap.add_argument("--program", action="store_true")
ap.add_argument("--mode", default="freerun")
ap.add_argument("--fsin-off-test", action="store_true")
ap.add_argument("pairs", nargs="*", help="hts:vts[:expo]")
a = ap.parse_args()
iface = MotionInterface(data_dir=os.path.join(tempfile.gettempdir(), "ffprobe"))
iface.start(wait=True, wait_timeout=3.0); iface.wait_for_ready(console=True, sensors=1, timeout=20)
s = iface.left; con = iface.console; cam = a.cam; mask = 1 << cam
T = 9.032e-6 / 432
def w(reg, val):
    s.switch_camera(cam)
    ok = s.camera_i2c_write(I2C_Packet(device_address=0x36, register_address=reg, data=val)); time.sleep(0.02); return ok
def r(reg, n=1):
    v = s.i2c_read_register(0x36, reg, read_len=n, reg_addr_size=2, mux_channel=cam)
    return None if v in (False, None) else int.from_bytes(bytes(v[:n]), "big")
def rate(secs):
    f0 = r(0x4610, 4); t0 = time.monotonic(); time.sleep(secs); f1 = r(0x4610, 4)
    return (f1 - f0) / (time.monotonic() - t0) if None not in (f0, f1) else float("nan")
saved = con.get_trigger_json(); saved = json.loads(saved) if isinstance(saved, str) else saved
try:
    s.enable_camera_power(mask); time.sleep(0.3)
    if a.program:
        print("program", force_program_fpga(s, mask)); time.sleep(0.3)
    s.camera_configure_registers(mask)
    cfg = dict(saved); cfg.update(EnableSyncOut=True, EnableTaTrigger=False, TriggerFrequencyHz=40.0)
    con.set_trigger_json(data=cfg); s.enable_camera_fsin_ext()
    s.enable_camera(mask); time.sleep(0.5)
    s.set_camera_image_mode(True, mask); time.sleep(0.2)
    regs = FpgaRegs(s, cam)
    try:
        regs.set_start_line(4095); regs.stop_sweep()
    except IOError as e:
        print("fpga quiet failed", e)
    con.start_trigger(); time.sleep(0.5)
    print(f"prod (trigger) rate {rate(2.0):.2f}/s", flush=True)
    if a.mode == "freerun":
        for reg, v in ((0x3823, 0x00), (0x382E, 0x01), (0x3880, 0x00)):
            w(reg, v)
    time.sleep(0.3)
    print(f"prod timing, mode={a.mode}: {rate(2.0):.2f}/s", flush=True)
    if a.fsin_off_test:
        con.stop_trigger(); time.sleep(0.3)
        print(f"  ...with FSIN stopped: {rate(2.0):.2f}/s", flush=True)
        con.start_trigger(); time.sleep(0.3)
    for p in a.pairs:
        parts = [int(x) for x in p.split(":")]
        hts, vts = parts[0], parts[1]; expo = parts[2] if len(parts) > 2 else 1
        mx = parts[3] if len(parts) > 3 else 2768
        for reg, v in ((0x380C, hts >> 8), (0x380D, hts & 0xFF), (0x380E, vts >> 8), (0x380F, vts & 0xFF),
                       (0x3501, expo >> 8), (0x3502, expo & 0xFF),
                       (0x3881, mx >> 16), (0x3882, (mx >> 8) & 0xFF), (0x3883, mx & 0xFF)):
            w(reg, v)
        fp = hts * vts * T
        time.sleep(min(3.0, 1.2 * fp) + 0.2)
        win = max(3.0, 4 * fp)
        rb = (r(0x380C, 2), r(0x380E, 2), r(0x3501, 2))
        print(f"HTS {hts:6d} VTS {vts:5d} expo {expo:4d} maxexpo {mx:5d}: expect {1/fp:8.3f}/s  got {rate(win):8.3f}/s  (win {win:.1f}s) rb={rb}", flush=True)
finally:
    try:
        for reg, v in ((0x380C, 0x01), (0x380D, 0xB0), (0x380E, 0x0A), (0x380F, 0xD0), (0x3501, 0), (0x3502, 0x48),
                       (0x3823, 0x20), (0x382E, 0x03), (0x3880, 0x05), (0x3881, 0), (0x3882, 0x0A), (0x3883, 0xD0)):
            w(reg, v)
    except Exception as e: print("restore", e)
    try: con.stop_trigger()
    except Exception as e: print("stop", e)
    con.set_trigger_json(data=saved)
    s.set_camera_image_mode(False, mask); s.disable_camera_fsin_ext(); s.disable_camera(mask)
    iface.stop()
