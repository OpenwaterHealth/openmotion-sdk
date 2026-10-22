"""Laser optimizer (sdk#335): the planner, the probe sink, the register
writers, and a full run against fake hardware.

The fake hardware is a simulated camera: each camera's dark-corrected mean is
k x gain x emitted pulse width, read from the registers the workflow actually
wrote (camera 0x3508 gain, TA pulse_width), with pixels clipping above a
per-camera onset. So the end-to-end tests exercise the real write path, the
real planner and the real sink, and only the photons are fake.
"""

import math

import numpy as np
import pytest

from omotion.config import DEFAULT_TRIGGER_CONFIG
from omotion.LaserOptimizerWorkflow import (
    Action,
    CameraReading,
    LaserOptimizerRequest,
    LaserOptimizerWorkflow,
    OptimizerConfig,
    OptimizerOutcome,
    OptimizerSetting,
    ProbeResult,
    _ProbeSink,
    plan_next,
    setting_for_signal,
    summary_text,
)
from omotion.pipeline.batch import FrameBatch
from omotion.register_overrides import (
    FW_DEFAULT_CAMERA_GAINS,
    FW_DEFAULT_EXPOSURE_ROWS,
    RegisterWriteError,
    emitted_pulse_us,
    read_safety_faults,
    ta_pulse_ticks,
    write_safety_limits,
    write_ta_pulse_width,
)
from omotion.ScanWorkflow import ScanRequest

pytestmark = pytest.mark.unit

CFG = OptimizerConfig()


# ---------------------------------------------------------------------------
# Settings and the balanced curve
# ---------------------------------------------------------------------------

def test_halving_walk_follows_the_balanced_curve():
    s = CFG.max_setting.signal
    walk = []
    for _ in range(8):
        st = setting_for_signal(s, CFG)
        walk.append((st.near_gain, st.pulse_width_us))
        s /= 2
    assert walk == [
        (16, 2000), (8, 2000), (8, 1009), (4, 1009),
        (2, 1009), (2, 513), (1, 513), (1, 265),
    ]


def test_three_eighths_of_max_is_8x_at_1500us():
    st = setting_for_signal(CFG.max_setting.signal * 0.375, CFG)
    assert (st.near_gain, st.pulse_width_us) == (8, 1504)


def test_width_never_exceeds_limits():
    for frac in np.geomspace(1e-4, 2.0, 60):
        st = setting_for_signal(CFG.max_setting.signal * frac, CFG)
        assert CFG.min_pulse_width_us <= st.pulse_width_us <= CFG.max_pulse_width_us
        assert st.near_gain in (1, 2, 4, 8, 16)


def test_camera_gains_scale_the_stock_ladder_capped_at_16():
    assert OptimizerSetting(16, 2000).camera_gains() == (16,) * 8
    assert OptimizerSetting(2, 500).camera_gains() == (16, 8, 4, 2, 2, 4, 8, 16)
    assert OptimizerSetting(1, 500).camera_gains() == FW_DEFAULT_CAMERA_GAINS


def test_signal_uses_the_emitted_pulse():
    # The TA drops ~54 ticks (~17 µs) of every pulse.
    assert emitted_pulse_us(2000) == pytest.approx(2000 - 17.28, abs=0.5)
    assert OptimizerSetting(8, 2000).signal == pytest.approx(8 * emitted_pulse_us(2000))


# ---------------------------------------------------------------------------
# Planner against a simulated camera
# ---------------------------------------------------------------------------

def _probe(setting, mean, clipped=0.0, index=1):
    r = CameraReading("right", 3, mean, clipped, 70.0, 30)
    return ProbeResult(index, setting, [r], [], CFG.clip_fraction)


def _simulate(k, clip_onset_dn=220.0, cfg=CFG):
    """Run the planner against mean = k x signal, clipping above the onset."""
    probes = []
    d = plan_next(probes, cfg)
    while d.action == Action.PROBE:
        true_mean = k * d.setting.signal
        frac = 0.0 if true_mean < clip_onset_dn else min(1.0, (true_mean - clip_onset_dn) / clip_onset_dn)
        probes.append(_probe(d.setting, min(true_mean, 900.0), frac, len(probes) + 1))
        d = plan_next(probes, cfg)
    return d, probes


def test_first_probe_is_the_brightest_setting():
    d = plan_next([], CFG)
    assert d.action == Action.PROBE and d.setting == OptimizerSetting(16, 2000)


def test_dim_target_converges_in_three_probes():
    k = 150.0 / OptimizerSetting(8, 2000).signal     # 300 DN (clipped) at the top
    d, probes = _simulate(k)
    assert d.action == Action.ACCEPT
    assert [p.setting.describe() for p in probes] == [
        "16x / 2000 µs", "8x / 2000 µs", "8x / 1339 µs"]
    assert 85 <= probes[-1].mean_dn <= 115


def test_bright_target_walks_down_then_jumps():
    k = 230.0 / OptimizerSetting(2, 1000).signal
    d, probes = _simulate(k)
    assert d.action == Action.ACCEPT
    assert len(probes) <= CFG.max_probes
    # Each clipped probe halves the signal, or quarters it when more than
    # half the pixels clip.
    for a, b in zip(probes, probes[1:]):
        if not a.clipped:
            continue
        step = 0.25 if a.clipped_fraction > 0.5 else 0.5
        assert b.setting.signal == pytest.approx(a.setting.signal * step, rel=0.02)
    assert d.setting == probes[-1].setting


def test_bench_phantom_readings_converge_within_the_budget():
    """Bench 2026-10-21, right module on a phantom: the readings each setting
    produced (clipped fraction, near mean). Halving alone needed all 8 of the
    old 8-probe budget; quarter steps under heavy clipping need 6."""
    measured = {   # (gain, width) -> (near mean DN, clipped fraction)
        (16, 2000): (895.4, 0.971), (8, 2000): (895.3, 0.971),
        (8, 1009): (894.2, 0.961), (4, 1009): (855.5, 0.786),
        (2, 1009): (681.4, 0.343), (2, 513): (410.4, 0.037),
        (1, 513): (210.1, 0.0002), (1, 253): (101.9, 0.0),
    }
    probes = []
    d = plan_next(probes, CFG)
    while d.action == Action.PROBE:
        key = (d.setting.near_gain, d.setting.pulse_width_us)
        mean, clip = measured[key]
        probes.append(_probe(d.setting, mean, clip, len(probes) + 1))
        d = plan_next(probes, CFG)
    assert d.action == Action.ACCEPT and d.setting == OptimizerSetting(1, 253)
    assert [(p.setting.near_gain, p.setting.pulse_width_us) for p in probes] == [
        (16, 2000), (8, 1009), (2, 1009), (2, 513), (1, 513), (1, 253)]


def test_not_enough_light_at_the_brightest_setting():
    d, probes = _simulate(50.0 / CFG.max_setting.signal)
    assert d.action == Action.INSUFFICIENT_SIGNAL
    assert len(probes) == 1 and d.setting is None


def test_no_frames_at_all_reads_as_no_signal():
    d = plan_next([_probe(CFG.max_setting, float("nan"))], CFG)
    assert d.action == Action.INSUFFICIENT_SIGNAL


def test_dimmer_than_target_after_a_clip_steps_back_up():
    probes = [_probe(CFG.max_setting, 900.0, 0.5),
              _probe(OptimizerSetting(8, 2000), 60.0, 0.0, 2)]
    d = plan_next(probes, CFG)
    assert d.action == Action.PROBE
    assert OptimizerSetting(8, 2000).signal < d.setting.signal < CFG.max_setting.signal


def test_never_asks_for_a_signal_at_or_above_a_clipped_one():
    # Clipping starts at 60 DN: the target is unreachable without clipping.
    d, probes = _simulate(400.0 / CFG.max_setting.signal, clip_onset_dn=60.0)
    assert d.action in (Action.NO_CONVERGENCE, Action.ACCEPT)
    for i, p in enumerate(probes):
        earlier_clips = [q.setting.signal for q in probes[:i] if q.clipped]
        if earlier_clips:
            assert p.setting.signal < min(earlier_clips)


def test_too_bright_at_the_dimmest_setting():
    floor = setting_for_signal(1e-9, CFG)
    d = plan_next([_probe(floor, 500.0, 0.3)], CFG)
    assert d.action == Action.TOO_BRIGHT


def test_gives_up_after_max_probes():
    cfg = OptimizerConfig(max_probes=2)
    probes = [_probe(cfg.max_setting, 900.0, 0.5),
              _probe(OptimizerSetting(8, 2000), 900.0, 0.5, 2)]
    assert plan_next(probes, cfg).action == Action.NO_CONVERGENCE


def test_brightest_near_camera_decides():
    dim = CameraReading("left", 3, 40.0, 0.0, 70.0, 30)
    bright = CameraReading("right", 4, 100.0, 0.0, 71.0, 30)
    p = ProbeResult(1, CFG.max_setting, [dim, bright], [], CFG.clip_fraction)
    assert p.control is bright
    assert plan_next([p], CFG).action == Action.ACCEPT


def test_summary_text_lists_every_probe():
    from omotion.LaserOptimizerWorkflow import LaserOptimizerResult
    probes = [_probe(CFG.max_setting, 900.0, 0.5), _probe(OptimizerSetting(8, 2000), 100.0, 0.0, 2)]
    text = summary_text(LaserOptimizerResult(OptimizerOutcome.COMPLETED, "ok", probes,
                                             OptimizerSetting(8, 2000)), CFG)
    assert "8x / 2000 µs" in text and "probe 1" in text and "probe 2" in text


# ---------------------------------------------------------------------------
# Probe sink
# ---------------------------------------------------------------------------

def _batch(rows, *, clip_bin=1020):
    """rows: list of (side, cam, frame_type, mean_dc_rt, mean_raw, low_light, clip_frac)."""
    n = len(rows)
    hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    mean_dc = np.full((n, 2, 8), np.nan, dtype=np.float32)
    mean_raw = np.full((n, 2, 8), np.nan, dtype=np.float32)
    low = np.zeros((n, 2, 8), dtype=bool)
    for i, (s, c, ft, dc, raw, ll, clip) in enumerate(rows):
        mean_dc[i, s, c] = dc
        mean_raw[i, s, c] = raw
        low[i, s, c] = ll
        total = 10000
        hist[i, s, c, 1023] = int(total * clip)
        hist[i, s, c, 200] = total - int(total * clip)
    return FrameBatch(
        cam_ids=np.array([r[1] for r in rows], dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        side_ids=np.array([r[0] for r in rows], dtype=np.int8),
        raw_histograms=hist,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n),
        pdc=None, tcm=None, tcl=None,
        frame_type=np.array([r[2] for r in rows], dtype="<U8"),
        mean_dc_rt=mean_dc, mean_raw=mean_raw, low_light_rt=low,
    )


def test_probe_sink_reads_median_mean_clip_and_dark():
    sink = _ProbeSink(clip_bin=1020)
    sink.on_scan_start(None)
    sink.consume("live", _batch([
        (1, 3, "warmup", 999.0, 999.0, False, 0.9),     # ignored
        (1, 3, "dark", np.nan, 72.0, False, 0.0),       # re-measured dark
        (1, 3, "light", np.nan, 150.0, False, 0.0),     # no baseline: skipped
        (1, 3, "light", 100.0, 172.0, False, 0.02),
        (1, 3, "light", 104.0, 176.0, False, 0.04),
        (1, 3, "light", 98.0, 170.0, False, 0.00),
    ]))
    r = sink.reading(1, 3)
    assert r.mean_dn == pytest.approx(100.0)
    assert r.clipped_fraction == pytest.approx(0.02)
    assert r.dark_dn == pytest.approx(72.0)
    assert r.light_frames == 3


def test_probe_sink_counts_unlit_frames_as_zero():
    sink = _ProbeSink(clip_bin=1020)
    sink.consume("live", _batch([(0, 4, "light", np.nan, 64.0, True, 0.0)] * 5))
    assert sink.reading(0, 4).mean_dn == 0.0


def test_probe_sink_with_no_data_is_nan():
    r = _ProbeSink(1020).reading(0, 3)
    assert math.isnan(r.mean_dn) and r.light_frames == 0


# ---------------------------------------------------------------------------
# Fake hardware
# ---------------------------------------------------------------------------

TA = (4, 0x00)
EE_PW_UL, OPT_PW_UL = (6, 0x04), (7, 0x04)
EE_RATE_LL, OPT_RATE_LL = (6, 0x08), (7, 0x08)
EE_STATUS, OPT_STATUS = (6, 0x24), (7, 0x24)


class FakeConsole:
    def __init__(self, *, fault_byte=0, corrupt=None):
        self.regs = {}
        self.writes = []            # (channel, offset, int value), in order
        self.fault_byte = fault_byte
        self.corrupt = corrupt      # (channel, offset) that reads back wrong
        self.laser_limits_error = None
        self.ta_writes_while_unprotected = 0

    def is_connected(self):
        return True

    def write_i2c_packet(self, mux_index, channel, device_addr, reg_addr, data):
        self.regs[(channel, reg_addr)] = bytes(data)
        self.writes.append((channel, reg_addr, int.from_bytes(bytes(data), "little")))
        if (channel, reg_addr) == TA:
            ul = self.value(EE_PW_UL)
            if ul is None or ul * 0.32 <= emitted_pulse_us(
                    int.from_bytes(bytes(data), "little") * 0.32):
                self.ta_writes_while_unprotected += 1
        return True

    def read_i2c_packet(self, mux_index, channel, device_addr, reg_addr, read_len):
        if (channel, reg_addr) in (EE_STATUS, OPT_STATUS):
            return bytes([self.fault_byte]), 0
        data = self.regs.get((channel, reg_addr), bytes(read_len))
        if (channel, reg_addr) == self.corrupt:
            data = bytes(read_len)
        return data, 0

    def value(self, key):
        d = self.regs.get(key)
        return None if d is None else int.from_bytes(d, "little")


class FakeSensor:
    def __init__(self, connected=True):
        self.connected = connected
        self.cam = None
        self.regs = {c: {} for c in range(8)}
        self.history = []           # (cam, reg, value), in order

    def is_connected(self):
        return self.connected

    def switch_camera(self, cam_id):
        self.cam = cam_id

    def camera_i2c_write(self, packet):
        self.regs[self.cam][packet.register_address] = packet.data
        self.history.append((self.cam, packet.register_address, packet.data))
        return True

    def gain(self, cam):
        return self.regs[cam].get(0x3508, FW_DEFAULT_CAMERA_GAINS[cam])


class FakeScanWorkflow:
    """Synchronous scans. A probe (a sink with skip_default_storage) gets a
    simulated batch built from the registers currently written."""

    def __init__(self, interface, k, clip_onset_dn, cancel_after_probes=None):
        self._iface = interface
        self.k = k                    # {(side_idx, cam): DN per signal unit}
        self.clip_onset_dn = clip_onset_dn
        self.running = False
        self.last_scan_error = None
        self.last_scan_canceled = False
        self.current_scan_label = None
        self.requests = []
        self.probe_count = 0
        self.cancel_after_probes = cancel_after_probes

    def start_scan(self, request):
        self.requests.append(request)
        if request.skip_default_storage:
            self.probe_count += 1
            sink = request.sinks[0]
            sink.on_scan_start(None)
            sink.consume("live", self._simulated_batch(request))
            sink.on_complete()
            if self.cancel_after_probes == self.probe_count:
                self._iface.optimizer.cancel()
        else:
            self.current_scan_label = f"20261021_000000_{request.subject_id}"
        return True

    def _simulated_batch(self, request):
        ticks = self._iface.console.value(TA)
        emitted = (ticks - 54) * 0.32
        rows = []
        for side_idx, (sensor, mask) in enumerate(
                ((self._iface.left, request.left_camera_mask),
                 (self._iface.right, request.right_camera_mask))):
            if not mask or not sensor.is_connected():
                continue
            for cam in range(8):
                if not mask & (1 << cam):
                    continue
                g = sensor.gain(cam)
                dark = 64.0 + 0.5 * g          # dark level grows with gain
                rows.append((side_idx, cam, "warmup", np.nan, dark, False, 0.0))
                rows.append((side_idx, cam, "dark", np.nan, dark, False, 0.0))
                m = self.k.get((side_idx, cam), 0.0) * g * emitted
                clip = 0.0 if m < self.clip_onset_dn else min(1.0, (m - self.clip_onset_dn) / self.clip_onset_dn)
                for _ in range(10):
                    rows.append((side_idx, cam, "light", min(m, 900.0), dark + m,
                                 m <= 5.0, clip))
        return _batch(rows)

    def await_complete(self, timeout_sec=None):
        pass

    def cancel_scan(self, **kw):
        self.last_scan_canceled = True
        self.running = False


class FakeInterface:
    def __init__(self, k, *, clip_onset_dn=220.0, console=None, left=None, right=None,
                 cancel_after_probes=None):
        self.console = console or FakeConsole()
        self.left = left or FakeSensor(connected=False)
        self.right = right or FakeSensor()
        self.scan_workflow = FakeScanWorkflow(self, k, clip_onset_dn, cancel_after_probes)
        self.laser_loads = 0
        self.optimizer = LaserOptimizerWorkflow(self)

    def resolve_trigger_config(self, override=None):
        return {**DEFAULT_TRIGGER_CONFIG, "LaserPulseSkipDelayUsec": 1800, **(override or {})}

    def apply_laser_power(self, *, force_fault=False, lock=None):
        # The connect-time load: stock TA width + this console's stored limits.
        self.laser_loads += 1
        for key, value in ((TA, 1563), (EE_PW_UL, 1719), (OPT_PW_UL, 1719),
                           (EE_RATE_LL, 72266), (OPT_RATE_LL, 72266)):
            self.console.write_i2c_packet(1, key[0], 0x41, key[1],
                                          value.to_bytes(3 if key == TA else 4, "little"))
        return True


def _final_scan(right_mask=0xFF, left_mask=0x00):
    return ScanRequest(subject_id="hoof1", duration_sec=10,
                       left_camera_mask=left_mask, right_camera_mask=right_mask,
                       sinks=["app sink"], extra_session_meta={"app": 1})


def _run(iface, request=None):
    request = request or LaserOptimizerRequest(final_scan=_final_scan())
    probes, done = [], []
    assert iface.optimizer.start(
        request,
        on_probe=lambda p, d: probes.append((p, d)),
        on_complete=done.append,
    )
    iface.optimizer.await_complete(timeout_sec=10)
    assert len(done) == 1
    return done[0], probes


def _k_for(mean_at, setting, cams=(3, 4), side=1, far_ratio=0.2):
    """Simulated brightness: near cameras read mean_at DN at setting."""
    k = {}
    for cam in range(8):
        k[(side, cam)] = mean_at / setting.signal * (1.0 if cam in cams else far_ratio)
    return k


# ---------------------------------------------------------------------------
# Register writers
# ---------------------------------------------------------------------------

def test_ta_ticks_floor_and_range():
    assert ta_pulse_ticks(2000) == 6250
    assert ta_pulse_ticks(20) == 62
    with pytest.raises(ValueError):
        ta_pulse_ticks(10)
    with pytest.raises(ValueError):
        ta_pulse_ticks(2300)


def test_write_safety_limits_writes_and_verifies_all_four():
    c = FakeConsole()
    written = write_safety_limits(c, pulse_width_ul_us=2200, rate_ll_us=22000)
    assert written == {"EE_PULSE_WIDTH_UL": 6875, "EE_RATE_LL": 68750,
                       "OPT_PULSE_WIDTH_UL": 6875, "OPT_RATE_LL": 68750}
    assert c.value(EE_PW_UL) == 6875 and c.value(OPT_RATE_LL) == 68750


def test_readback_mismatch_raises():
    c = FakeConsole(corrupt=OPT_RATE_LL)
    with pytest.raises(RegisterWriteError, match="OPT_RATE_LL"):
        write_safety_limits(c, pulse_width_ul_us=2200, rate_ll_us=22000)
    with pytest.raises(RegisterWriteError, match="TA_PULSE_WIDTH"):
        write_ta_pulse_width(FakeConsole(corrupt=TA), 1000)


def test_read_safety_faults_decodes_status():
    assert read_safety_faults(FakeConsole()) == []
    faults = read_safety_faults(FakeConsole(fault_byte=0x04))
    assert faults == ["EE RATE_LOWER_LIMIT_FAIL", "OPT RATE_LOWER_LIMIT_FAIL"]


# ---------------------------------------------------------------------------
# Full runs
# ---------------------------------------------------------------------------

def test_full_run_converges_scans_and_restores():
    iface = FakeInterface(_k_for(150.0, OptimizerSetting(8, 2000)))
    result, probes = _run(iface)

    assert result.outcome == OptimizerOutcome.COMPLETED, result.message
    assert result.setting == OptimizerSetting(8, 1339)
    assert [p.setting.describe() for p, _ in probes] == [
        "16x / 2000 µs", "8x / 2000 µs", "8x / 1339 µs"]
    assert result.restored and result.scan_label

    c = iface.console
    # Safety limits raised before the first TA write, and the TA was never
    # programmed past the PULSE_WIDTH_UL in force.
    first_ta = next(i for i, w in enumerate(c.writes) if (w[0], w[1]) == TA)
    first_ul = next(i for i, w in enumerate(c.writes) if (w[0], w[1]) == EE_PW_UL)
    assert first_ul < first_ta
    assert c.ta_writes_while_unprotected == 0

    # Each probe ran with its own trigger override (dark skip + pulse width).
    probe_reqs = [r for r in iface.scan_workflow.requests if r.skip_default_storage]
    assert [r.trigger_config["LaserPulseWidthUsec"] for r in probe_reqs] == [2000, 2000, 1339]
    assert all(r.trigger_config["LaserPulseSkipDelayUsec"] == 2400 for r in probe_reqs)

    # The final scan is the caller's request plus the optimizer's additions.
    final = iface.scan_workflow.requests[-1]
    assert not final.skip_default_storage and final.sinks == ["app sink"]
    assert final.duration_sec == 10 and final.subject_id == "hoof1"
    gains = [16, 16, 16, 8, 8, 16, 16, 16]
    assert [list(g) for g in final.camera_gain_map] == [gains, gains]
    assert final.trigger_config == {"LaserPulseSkipDelayUsec": 2400, "LaserPulseWidthUsec": 1339}
    meta = final.extra_session_meta
    assert meta["app"] == 1
    assert meta["laser_optimizer"]["pulse_width_us"] == 1339
    assert len(meta["laser_optimizer"]["probes"]) == 3

    # Restore: firmware camera defaults and one connect-time laser load.
    assert iface.laser_loads == 1
    assert c.value(TA) == 1563 and c.value(EE_RATE_LL) == 72266
    for cam in range(8):
        assert iface.right.regs[cam][0x3508] == FW_DEFAULT_CAMERA_GAINS[cam]
        assert iface.right.regs[cam][0x3502] == FW_DEFAULT_EXPOSURE_ROWS


def test_each_probe_measures_its_own_dark_at_its_gain():
    iface = FakeInterface(_k_for(150.0, OptimizerSetting(8, 2000)))
    _, probes = _run(iface)
    darks = [p.near[0].dark_dn for p, _ in probes]
    assert darks == [64.0 + 0.5 * 16, 64.0 + 0.5 * 8, 64.0 + 0.5 * 8]


def test_exposure_is_written_for_every_probe_and_restored():
    iface = FakeInterface(_k_for(150.0, OptimizerSetting(8, 2000)))
    _, probes = _run(iface)
    lsb = [(cam, v) for cam, reg, v in iface.right.history if reg == 0x3502]
    # 244 rows (2196 µs) on every camera for each probe plus the final scan,
    # then the firmware's 72 rows once at restore.
    assert lsb[-8:] == [(cam, FW_DEFAULT_EXPOSURE_ROWS) for cam in range(8)]
    assert lsb[:-8] == [(cam, 244) for cam in range(8)] * (len(probes) + 1)


def test_insufficient_signal_runs_no_scan_and_restores():
    iface = FakeInterface(_k_for(40.0, OptimizerSetting(16, 2000)))
    result, probes = _run(iface)
    assert result.outcome == OptimizerOutcome.INSUFFICIENT_SIGNAL
    assert len(probes) == 1 and not result.scan_ran
    assert all(r.skip_default_storage for r in iface.scan_workflow.requests)
    assert result.restored and iface.laser_loads == 1


def test_latched_interlock_refuses_before_touching_the_console():
    iface = FakeInterface(_k_for(150.0, OptimizerSetting(8, 2000)),
                          console=FakeConsole(fault_byte=0x02))
    result, probes = _run(iface)
    assert result.outcome == OptimizerOutcome.ERROR
    assert "latched" in result.message
    assert iface.console.writes == [] and iface.laser_loads == 0
    assert probes == []


def test_cancel_between_probes_restores():
    iface = FakeInterface(_k_for(150.0, OptimizerSetting(8, 2000)), cancel_after_probes=1)
    result, probes = _run(iface)
    assert result.outcome == OptimizerOutcome.CANCELED
    assert not result.scan_ran
    assert result.restored and iface.laser_loads == 1


def test_far_cameras_are_reported_but_do_not_decide():
    iface = FakeInterface(_k_for(150.0, OptimizerSetting(8, 2000), far_ratio=3.0))
    result, probes = _run(iface)
    # Far cameras clip (3x brighter); the near pair still lands in the band.
    assert result.outcome == OptimizerOutcome.COMPLETED
    first = probes[0][0]
    assert {r.cam_id for r in first.others} == {0, 1, 2, 5, 6, 7}


def test_refuses_without_an_active_near_camera():
    iface = FakeInterface({})
    ok = iface.optimizer.start(LaserOptimizerRequest(final_scan=_final_scan(right_mask=0xC3)))
    assert ok is False
    assert "near camera" in iface.optimizer.last_result.message


def test_refuses_an_exposure_shorter_than_delay_plus_pulse():
    iface = FakeInterface({})
    req = LaserOptimizerRequest(final_scan=_final_scan(),
                                config=OptimizerConfig(exposure_us=1998.0))
    assert iface.optimizer.start(req) is False
    assert "does not cover" in iface.optimizer.last_result.message


def test_refuses_a_rate_limit_above_the_post_dark_gap():
    iface = FakeInterface({})
    req = LaserOptimizerRequest(final_scan=_final_scan(),
                                config=OptimizerConfig(safety_rate_ll_us=23125.0))
    assert iface.optimizer.start(req) is False
    assert "RATE_LL" in iface.optimizer.last_result.message


def test_refuses_a_laser_off_final_scan():
    iface = FakeInterface({})
    req = LaserOptimizerRequest(final_scan=ScanRequest(
        subject_id="s", duration_sec=10, left_camera_mask=0, right_camera_mask=0xFF,
        disable_laser=True))
    assert iface.optimizer.start(req) is False
