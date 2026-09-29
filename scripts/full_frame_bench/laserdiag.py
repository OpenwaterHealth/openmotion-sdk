import os
import tempfile
import time, logging
logging.basicConfig(level=logging.WARNING, format="%(message)s")
from omotion import MotionInterface
from omotion import ConsoleTelemetry as CT
from omotion.config import DEFAULT_TRIGGER_CONFIG
i = MotionInterface(data_dir=os.path.join(tempfile.gettempdir(), "ffprobe")); i.start(wait=True, wait_timeout=3.0); i.wait_for_ready(console=True, sensors=1, timeout=15)
c = i.console
def safety():
    out = {}
    for name, ch in (("SE", CT._SAFETY_SE_CHANNEL), ("SO", CT._SAFETY_SO_CHANNEL)):
        raw, _ = c.read_i2c_packet(mux_index=CT._MUX_IDX, channel=ch, device_addr=CT._I2C_ADDR, reg_addr=CT._SAFETY_REG, read_len=CT._SAFETY_LEN)
        out[name] = (raw.hex() if raw else None, CT._decode_safety_faults(raw[0] & CT._SAFETY_FAULT_MASK) if raw else None)
    return out
print("safety before:", safety())
try: print("tec_status:", c.tec_status())
except Exception as e: print("tec err", e)
print("apply_laser_power:", i.apply_laser_power())
cfg = dict(DEFAULT_TRIGGER_CONFIG); cfg["LaserPulseSkipInterval"] = 0; cfg["LaserPulseSkipDelayUsec"] = 0
print("set:", c.set_trigger_json(data=cfg)); print("start:", c.start_trigger())
time.sleep(1.0)
try:
    n, samples = c.get_pdc_buffer(64); print("pdc buffer n=", n, "last samples:", samples[-5:])
except Exception as e: print("pdc err", e)
print("safety during:", safety())
print("trigger now:", c.get_trigger_json())
time.sleep(0.5); c.stop_trigger()
print("safety after:", safety())
i.stop()
