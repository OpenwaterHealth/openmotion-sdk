"""Stream production histograms from one sensor, laser on vs off, report per-camera mean level."""
import os
import tempfile
import sys, time, queue, logging, struct
import numpy as np
logging.basicConfig(level=logging.WARNING, format="%(message)s")
from omotion import MotionInterface
from omotion.config import DEFAULT_TRIGGER_CONFIG
side = sys.argv[1]; mask = int(sys.argv[2], 0)
i = MotionInterface(data_dir=os.path.join(tempfile.gettempdir(), "ffprobe")); i.start(wait=True, wait_timeout=3.0); i.wait_for_ready(console=True, sensors=2, timeout=20)
s = getattr(i, side); c = i.console
print(side, "fw", s.get_version())
s.enable_camera_power(mask); time.sleep(0.5)
ok = s.program_fpga(camera_position=mask, manual_process=False); print("program", ok)
print("configure", s.camera_configure_registers(mask))
print("laser power", i.apply_laser_power())
def run(laser):
    cfg = dict(DEFAULT_TRIGGER_CONFIG); cfg["LaserPulseSkipInterval"] = 0; cfg["LaserPulseSkipDelayUsec"] = 0; cfg["EnableTaTrigger"] = laser
    c.set_trigger_json(data=cfg); s.enable_camera_fsin_ext()
    q = queue.Queue(); h = s.uart.histo
    h.flush_stale_data(expected_size=32837); h.start_streaming(q, 32837)
    s.enable_camera(mask); time.sleep(0.3); c.start_trigger(); time.sleep(1.5); c.stop_trigger(); time.sleep(0.2)
    s.disable_camera(mask); h.stop_streaming()
    buf = b"".join(q.queue)
    means = {}; tots = {}
    k = 0
    while True:
        k = buf.find(b"\xaa\x00", k)
        if k < 0 or k + 10 > len(buf): break
        ln = struct.unpack_from("<I", buf, k + 2)[0]
        if ln < 20 or k + ln > len(buf): k += 1; continue
        p = buf[k:k+ln]; off = 10
        while off + 2 + 4096 + 5 <= ln - 3 and p[off] == 0xFF:
            cam = p[off+1]; hist = (np.frombuffer(p[off+2:off+2+4096], "<u4") & 0xFFFFFF).astype(float)
            tot = hist.sum()
            if tot > 0: means.setdefault(cam, []).append((hist * np.arange(1024)).sum() / tot); tots.setdefault(cam, []).append(tot)
            off += 2 + 4096 + 4 + 1
        k += ln
    return {cam: (round(float(np.median(v)), 1), int(np.median(tots[cam]))) for cam, v in sorted(means.items())}
print("laser OFF per-cam mean:", run(False))
print("laser ON  per-cam mean:", run(True))
s.disable_camera_fsin_ext(); i.stop()
