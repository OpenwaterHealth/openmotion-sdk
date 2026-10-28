"""Fake console, sensors and short scans for the Optimize Laser tests.

The console stores raw I2C bytes at the real register locations
(``omotion.data.fpga_model``), so ``FpgaRegisterIO`` and the actuator run
unchanged on top of it. ``Rig`` ties a console and two sensors to a
brightness model and a safety interlock: a scan whose pulse, rate or current
the safety limits don't allow latches E-202 instead of delivering frames,
which is what a wrong write order would do on hardware.
"""

from __future__ import annotations

import numpy as np

from omotion.data.fpga_model import FPGA_MODEL
from omotion.exposure.actuator import FW_DEFAULT_EXPOSURE_ROWS, FW_DEFAULT_GAIN_CODES
from omotion.pipeline.batch import FrameBatch

PEDESTAL = 64.0
TRUNCATION_US = 17.3

# The bench unit (1.8.1) as probed on 2026-10-27.
BENCH_REGISTERS = {
    "TA_PULSE_WIDTH": 530.0, "TA_CURRENT_DRV": 5000.0, "TA_CURRENT_LIMIT": 5500.0,
    "TA_STATUS": 0,
    "EE_PULSE_WIDTH_LL": 0.0, "OPT_PULSE_WIDTH_LL": 0.0,
    "EE_PULSE_WIDTH_UL": 660.0, "OPT_PULSE_WIDTH_UL": 660.0,
    "EE_RATE_LL": 23125.0, "OPT_RATE_LL": 23125.0,
    "EE_DRIVE_CL": 9999.0, "OPT_DRIVE_CL": 9999.0,
    "EE_STATUS": 0, "OPT_STATUS": 0,
}
# A laser-calibrated unit at 4400 mA, DRIVE_CL from its measured peak.
CALIBRATED_REGISTERS = {**BENCH_REGISTERS, "TA_CURRENT_DRV": 4400.0,
                        "EE_DRIVE_CL": 4840.0, "OPT_DRIVE_CL": 5720.0}


def _register_table():
    table = {}
    for fpga in FPGA_MODEL:
        for fn in fpga.get("functions", []):
            name = fn.get("friendlyName")
            if not name:
                continue
            bits = int(str(fn["data_size"]).rstrip("B"))
            table[name] = ((fpga["mux_idx"], fpga["channel"], fpga["i2c_addr"],
                            fn["start_address"]), bits // 8, float(fn.get("scale") or 1.0))
    return table


_REGISTERS = _register_table()
_BY_LOCATION = {loc: name for name, (loc, _, _) in _REGISTERS.items()}


class FakeConsole:
    def __init__(self, *, version="1.8.1", owns=False, values=None, refuse=(),
                 clamp=None, restore_ok=True):
        self.version = version
        self.owns = owns
        self.laser_limits_error = None
        self.refuse = set(refuse)           # names whose writes NAK
        self.clamp = dict(clamp or {})      # name -> highest value the register keeps
        self.restore_ok = restore_ok
        self.flash = {**BENCH_REGISTERS, **(values or {})}
        self.store: dict = {}
        self.writes: list = []              # (name, value) in order
        self.restores = 0
        for name, value in self.flash.items():
            self.set(name, value)

    # register helpers
    def set(self, name, value):
        loc, width, scale = _REGISTERS[name]
        self.store[loc] = int(round(value / scale)).to_bytes(width, "little")

    def get(self, name) -> float:
        loc, _, scale = _REGISTERS[name]
        return int.from_bytes(self.store[loc], "little") * scale

    def wrote(self, name) -> list:
        return [v for n, v in self.writes if n == name]

    # MotionConsole surface
    def is_connected(self):
        return True

    def get_version(self):
        return self.version

    def get_hardware_id(self):
        return "fake-console"

    def firmware_owns_safety_state(self):
        return self.owns

    def read_i2c_packet(self, *, mux_index, channel, device_addr, reg_addr, read_len):
        data = self.store.get((mux_index, channel, device_addr, reg_addr))
        if data is None or len(data) != read_len:
            return None, 0
        return bytes(data), len(data)

    def write_i2c_packet(self, *, mux_index, channel, device_addr, reg_addr, data):
        loc = (mux_index, channel, device_addr, reg_addr)
        name = _BY_LOCATION.get(loc)
        _, width, scale = _REGISTERS[name]
        value = int.from_bytes(bytes(data), "little") * scale
        self.writes.append((name, value))
        if name in self.refuse:
            return False
        if name in self.clamp and value > self.clamp[name]:
            value = self.clamp[name]
        self.set(name, value)
        return True


def fake_apply_laser_power(console, lock=None):
    """Stands in for omotion.laser.apply_laser_power: baseline plus flash.
    A latched safety status survives it, as on hardware."""
    console.restores += 1
    for name, value in console.flash.items():
        if not name.endswith("_STATUS"):
            console.set(name, value)
    if not console.restore_ok:
        console.laser_limits_error = "Laser safety limit mismatch: EE_RATE_LL"
        return False
    return True


class FakeSensor:
    def __init__(self, *, connected=True, fail=()):
        self.connected = connected
        self.fail = set(fail)               # (position, register) writes that fail
        self.selected = None
        self.regs = {pos: {} for pos in range(8)}
        self.writes: list = []
        for pos in range(8):
            code = FW_DEFAULT_GAIN_CODES[pos]
            self.regs[pos] = {0x3501: FW_DEFAULT_EXPOSURE_ROWS >> 8,
                              0x3502: FW_DEFAULT_EXPOSURE_ROWS & 0xFF,
                              0x3508: code >> 4, 0x3509: (code & 0xF) << 4}

    def is_connected(self):
        return self.connected

    def switch_camera(self, pos):
        self.selected = pos

    def camera_i2c_write(self, packet):
        reg = packet.register_address
        self.writes.append((self.selected, reg, packet.data))
        if (self.selected, reg) in self.fail:
            return False
        self.regs[self.selected][reg] = packet.data
        return True

    def gain_code(self, pos) -> int:
        r = self.regs[pos]
        return ((r[0x3508] & 0x1F) << 4) | (r[0x3509] >> 4)

    def exposure_rows(self, pos) -> int:
        r = self.regs[pos]
        return (r[0x3501] << 8) | r[0x3502]


class Rig:
    """A console, two sensors, a laser and the cameras' light coupling.

    ``coupling[(side, pos)]`` is the signal (DN) per unit gain per emitted
    microsecond at full laser output.
    """

    def __init__(self, coupling, *, console=None, threshold_ma=2600.0, knee_ma=4600.0,
                 seed=0, pixels=4000, shape=6.0):
        self.console = console or FakeConsole()
        self.left, self.right = FakeSensor(), FakeSensor()
        self.coupling = dict(coupling)
        self.threshold_ma, self.knee_ma = threshold_ma, knee_ma
        self.rng = np.random.default_rng(seed)
        self.pixels, self.shape = pixels, shape
        self.scans: list = []               # (writes so far, trigger_config) per scan

    def sensor(self, side):
        return (self.left, self.right)[side]

    def signal(self, cam) -> float:
        side, pos = cam
        c = self.console
        gain = self.sensor(side).gain_code(pos) / 16.0
        emitted = max(c.get("TA_PULSE_WIDTH") - TRUNCATION_US, 0.0)
        current = c.get("TA_CURRENT_DRV")
        output = max(min(current, self.knee_ma) - self.threshold_ma, 0.0) / (
            self.knee_ma - self.threshold_ma)
        return self.coupling[cam] * gain * emitted * output

    def interlock(self, skip_us) -> str | None:
        """What the safety FPGAs would latch on this scan's first pulses."""
        c = self.console
        emitted = c.get("TA_PULSE_WIDTH") - TRUNCATION_US
        current = c.get("TA_CURRENT_DRV")
        for side in ("EE", "OPT"):
            if emitted > c.get(f"{side}_PULSE_WIDTH_UL"):
                c.set(f"{side}_STATUS", 0x02)
            elif 25000 - skip_us < c.get(f"{side}_RATE_LL"):
                c.set(f"{side}_STATUS", 0x04)
            elif current > c.get(f"{side}_DRIVE_CL"):
                c.set(f"{side}_STATUS", 0x01)
        if c.get("EE_STATUS") or c.get("OPT_STATUS"):
            return "laser-safety fault latched (E-202)"
        return None

    def histogram(self, mean_above_pedestal) -> np.ndarray:
        if mean_above_pedestal <= 0:
            values = PEDESTAL + self.rng.normal(0.0, 1.5, self.pixels)
        else:
            values = PEDESTAL + self.rng.gamma(self.shape, mean_above_pedestal / self.shape,
                                               self.pixels)
        values = np.clip(np.round(values), 0, 1023).astype(np.int64)
        return np.bincount(values, minlength=1024).astype(np.uint32)

    def batches(self, left_mask, right_mask, light_frames):
        """One FrameBatch per frame: 9 warmup, 1 dark, the light frames, and
        a laser-off frame labeled light (the scan's terminal frame)."""
        cams = [(s, p) for s, m in enumerate((left_mask, right_mask))
                for p in range(8) if m & (1 << p)]
        kinds = ["warmup"] * 9 + ["dark"] + ["light"] * light_frames + ["terminal"]
        for frame, kind in enumerate(kinds):
            n = len(cams)
            hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
            for row, cam in enumerate(cams):
                lit = kind in ("light", "warmup")
                hist[row, cam[0], cam[1]] = self.histogram(self.signal(cam) if lit else 0.0)
            yield FrameBatch(
                cam_ids=np.array([p for _, p in cams], dtype=np.int8),
                frame_ids=np.full(n, frame % 256, dtype=np.uint8),
                raw_histograms=hist,
                temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
                timestamp_s=np.full(n, frame / 40.0),
                pdc=None, tcm=None, tcl=None,
                side_ids=np.array([s for s, _ in cams], dtype=np.int8),
                frame_type=np.array(["light" if kind == "terminal" else kind] * n),
            )


class FakeScanWorkflow:
    """Runs a short scan synchronously against a Rig."""

    def __init__(self, rig, *, refuse_at=None, fail_at=None):
        self.rig = rig
        self.refuse_at, self.fail_at = refuse_at, fail_at
        self.requests: list = []
        self.running = False
        self.last_scan_error = None
        self.last_scan_canceled = False

    def start_scan(self, request):
        self.requests.append(request)
        n = len(self.requests)
        self.last_scan_error = None
        if n == self.refuse_at:
            self.last_scan_error = "previous scan is still active"
            return False
        skip = (request.trigger_config or {}).get("LaserPulseSkipDelayUsec", 1800)
        self.rig.scans.append((len(self.rig.console.writes), dict(request.trigger_config or {})))
        latched = self.rig.interlock(skip)
        if latched or n == self.fail_at:
            self.last_scan_error = latched or "sub-scan worker died"
            return True
        sink = request.sinks[0]
        sink.on_scan_start(None)
        frames = max(1, int(request.duration_sec * 40) - 11)
        for batch in self.rig.batches(request.left_camera_mask, request.right_camera_mask,
                                      min(frames, 14)):
            sink.consume("raw", batch)
        sink.on_complete()
        return True

    def await_complete(self, timeout_sec=None):
        pass

    def cancel_scan(self, **kwargs):
        self.last_scan_canceled = True


class FakeInterface:
    def __init__(self, rig, **scan_kwargs):
        self.console = rig.console
        self.left, self.right = rig.left, rig.right
        self.scan_workflow = FakeScanWorkflow(rig, **scan_kwargs)
