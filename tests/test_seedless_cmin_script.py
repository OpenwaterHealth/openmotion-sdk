"""Seedless C_min: operator script and Motion hardware adapter, with fakes.

No hardware. The script tests inject the simulated phantom bench from
test_seedless_cmin; the adapter tests drive MotionCminBench against a fake
scan workflow and the register fake from test_seedless_controller.
"""

import json
from types import SimpleNamespace

import pytest

from test_seedless_cmin import FakeBench
from test_seedless_controller import LIVE, FakeConsole as RegisterConsole, FakeSensor as RegisterSensor
from wi15_script_harness import load_wi15_script, wi15_script_path

from omotion.calibration.seedless_cmin import FrameRecord
from omotion.calibration.seedless_cmin_hardware import (
    FIRST_LIGHT_ABS_ID,
    MotionCminBench,
    _LightFrameCollector,
    first_light_frames,
)
from omotion.seedless import SEEDLESS_TRIGGER_OVERRIDES

SCRIPT_PATH = wi15_script_path("seedless_cmin.py")


def load_script():
    return load_wi15_script(SCRIPT_PATH, "seedless_cmin_under_test")


class ScriptConsole:
    def __init__(self, boot_info=None, config_ma=3900.0, live_ma=3900.0):
        self.boot_info = boot_info
        self.device_writes = []
        self.config_ma = config_ma      # TA_CURRENT_DRV in the user config
        self.live_ma = live_ma          # what the TA FPGA holds

    def read_config(self):
        data = {} if self.config_ma is None else {"TA_CURRENT_DRV": self.config_ma}
        return SimpleNamespace(json_data=data)

    def read_i2c_packet(self, *, mux_index, channel, device_addr, reg_addr,
                        read_len):
        assert (channel, reg_addr, read_len) == (4, 0x06, 2)
        return round(self.live_ma / 0.16).to_bytes(2, "little"), 2

    def get_boot_info(self):
        return self.boot_info

    def read_serial_number(self):
        return "CONSN01"

    # Anything that would persist a value on the device. The procedure must
    # never call these: C_min goes to the report only.
    def write_config(self, *a, **k):
        self.device_writes.append(("write_config", a, k))

    def write_calibration(self, *a, **k):
        self.device_writes.append(("write_calibration", a, k))


class ScriptSensor:
    def __init__(self):
        self.powered = []

    def enable_camera_power(self, mask):
        self.powered.append(mask)
        return True

    def read_serial_number(self):
        return "SNR02"


class ScriptInterface:
    def __init__(self, boot_info=None):
        self.console = ScriptConsole(boot_info)
        self.left = ScriptSensor()
        self.right = ScriptSensor()
        self.configure = []
        self.laser_applied = 0
        self.stopped = 0

    def start(self):
        pass

    def stop(self):
        self.stopped += 1

    def wait_for_ready(self, **_k):
        return True

    def is_device_connected(self):
        return (True, True, True)

    def start_configure_camera_sensors(self, request, *, on_complete_fn):
        self.configure.append(request)
        on_complete_fn(SimpleNamespace(ok=True, error=""))
        return True

    def apply_laser_power(self):
        self.laser_applied += 1
        return True


def run_main(tmp_path, monkeypatch, *, boot_info=None, answers=("yes",),
             bench=None, argv_extra=("--side", "right"), console_kw=None):
    script = load_script()
    iface = ScriptInterface(boot_info)
    for k, v in (console_kw or {}).items():
        setattr(iface.console, k, v)
    bench = bench or FakeBench()
    created = []
    monkeypatch.setattr(script, "interface_factory", lambda **_k: iface)

    def make_bench(i, masks, trigger):
        created.append((masks, trigger))
        return bench

    monkeypatch.setattr(script, "bench_factory", make_bench)
    replies = iter(answers)
    lines = []
    code = script.main(
        ["--output-dir", str(tmp_path), "--operator", "op", "--fixture-id",
         "fx", *argv_extra],
        input_func=lambda _p: next(replies), output_func=lines.append)
    return code, lines, iface, created


def test_script_pass_prints_cmin_and_writes_report_only(tmp_path, monkeypatch):
    code, lines, iface, created = run_main(tmp_path, monkeypatch)
    assert code == 0, lines
    assert lines[-1] == "Final result: PASS"
    assert any(line.startswith("C_min = 0.0900") for line in lines)
    assert "Per camera (C_min corrected to exactly the normal brightness):" in lines
    assert sum(1 for line in lines if line.startswith("  R")) == 8
    assert "Nothing was written to the device." in lines
    assert iface.console.device_writes == []
    assert created[0][0] == {"left": 0x00, "right": 0xFF}
    assert iface.right.powered == [0xFF] and iface.left.powered == []
    assert iface.laser_applied == 1 and iface.stopped == 1
    (run_dir,) = tmp_path.iterdir()
    doc = json.loads(next(run_dir.glob("CONSN01-seedless-cmin-*.json")).read_text())
    assert doc["written_to_device"] is False
    assert doc["run"]["right sensor serial"] == "SNR02"
    assert doc["result"]["c_min"] == pytest.approx(0.09)
    assert list(run_dir.glob("*-frames.csv"))


def test_script_refuses_envelope_firmware_before_the_laser(tmp_path, monkeypatch):
    info = SimpleNamespace(firmware_owns_safety_state=True, factory_image=False)
    code, lines, iface, created = run_main(tmp_path, monkeypatch, boot_info=info)
    assert code == 1
    assert lines[-1] == "Final result: FAIL"
    assert any("660 us" in line and "Factory image" in line for line in lines)
    assert iface.laser_applied == 0 and created == []
    assert iface.right.powered == []


@pytest.mark.parametrize("console_kw, words", [
    ({"config_ma": None}, "has no TA_CURRENT_DRV"),
    ({"live_ma": 4999.68}, "is 5000 mA but the console config says 3900 mA"),
])
def test_script_refuses_a_nominal_not_at_the_calibrated_current(
        tmp_path, monkeypatch, console_kw, words):
    bench = FakeBench()
    code, lines, _, created = run_main(tmp_path, monkeypatch, bench=bench,
                                       console_kw=console_kw)
    assert code == 1 and lines[-1] == "Final result: FAIL"
    assert any(words in line for line in lines), lines
    assert created == [] and bench.log == []


def test_script_runs_on_firmware_that_leaves_limits_to_the_host(tmp_path,
                                                                monkeypatch):
    info = SimpleNamespace(firmware_owns_safety_state=False, factory_image=None)
    code, _, _, _ = run_main(tmp_path, monkeypatch, boot_info=info)
    assert code == 0


def test_script_steps_current_and_prints_it(tmp_path, monkeypatch):
    code, lines, iface, _ = run_main(
        tmp_path, monkeypatch, bench=FakeBench(seedless_per_us=0.75 / 2150))
    assert code == 0, lines
    assert any("TA drive current for matched brightness: 4700 mA" in line
               for line in lines)
    assert iface.console.device_writes == []


def test_script_plateau_pass_prints_basis(tmp_path, monkeypatch):
    code, lines, iface, _ = run_main(
        tmp_path, monkeypatch, bench=FakeBench(seedless_per_us=0.75 / 2150),
        argv_extra=("--side", "right", "--no-current-stepping",
                    "--plateau-check"))
    assert code == 0, lines
    assert any("plateau check" in line for line in lines)
    assert not any("matched brightness:" in line for line in lines)
    assert any(line.startswith("C_min = 0.0900") for line in lines)
    assert iface.console.device_writes == []


def test_script_phantom_declined_changes_nothing(tmp_path, monkeypatch):
    code, lines, iface, created = run_main(tmp_path, monkeypatch, answers=("no",))
    assert code == 1
    assert "canceled" in lines[-1]
    assert created == [] and iface.laser_applied == 0


def test_script_fail_still_writes_report(tmp_path, monkeypatch):
    code, lines, _, _ = run_main(tmp_path, monkeypatch,
                                 bench=FakeBench(seed_stays_on=True))
    assert code == 1
    assert any("seed may not be off" in line for line in lines)
    (run_dir,) = tmp_path.iterdir()
    assert list(run_dir.glob("*.json"))


def test_script_rejects_max_pulse_above_hard_limit(tmp_path, monkeypatch):
    code, lines, _, created = run_main(
        tmp_path, monkeypatch, argv_extra=("--side", "right",
                                           "--max-pulse-us", "2500"))
    assert code == 1 and created == []


# ---------------------------------------------------------------------------
# MotionCminBench
# ---------------------------------------------------------------------------

def frame(cam, abs_id, side="right"):
    return SimpleNamespace(side=side, cam_id=cam, abs_frame_id=abs_id,
                           mean=100.0, std=10.0, contrast=0.1, quality="ok")


class FakeWorkflow:
    def __init__(self, frames_by_scan, error=None):
        self.frames_by_scan = list(frames_by_scan)
        self.requests = []
        self.running = False
        self.last_scan_error = error

    def start_scan(self, request):
        self.requests.append(request)
        sink = request.sinks[0]
        sink.on_scan_start(None)
        sink.consume("final", SimpleNamespace(frames=self.frames_by_scan.pop(0)))
        sink.on_complete()
        return True

    def await_complete(self, timeout_sec=None):
        return True

    def cancel_scan(self):
        pass


class AdapterInterface:
    def __init__(self, workflow, console=None, *, trigger_ok=True):
        self.scan_workflow = workflow
        self.console = console or RegisterConsole()
        self.console.stop_trigger = lambda: True
        self.console.trigger = None
        console = self.console

        def set_trigger_json(data=None):
            if not trigger_ok:
                return None
            console.writes.append(("trigger", dict(data)))
            console.trigger = dict(data)
            return dict(data)

        console.set_trigger_json = set_trigger_json
        console.get_trigger_json = lambda: console.trigger
        self.left = RegisterSensor()
        self.right = RegisterSensor()


TRIGGER = {"TriggerFrequencyHz": 40, "LaserPulseSkipDelayUsec": 1800}


def test_first_light_frames_drops_preamble_and_trims():
    frames = [FrameRecord("right", 0, i, 1, 1, 1) for i in range(9, 30)]
    out = first_light_frames(frames, 10)
    assert [f.abs_frame_id for f in out] == list(range(FIRST_LIGHT_ABS_ID, 21))


def test_collector_skips_gap_fill():
    c = _LightFrameCollector()
    filled = frame(0, 12)
    filled.quality = "nan_filled"
    c.consume("final", SimpleNamespace(frames=[frame(0, 11), filled]))
    assert [f.abs_frame_id for f in c.frames] == [11]


def test_capture_nominal_then_seedless_trigger_and_duration():
    scan = [frame(c, i) for c in range(8) for i in range(11, 40)]
    wf = FakeWorkflow([scan, scan])
    iface = AdapterInterface(wf)
    bench = MotionCminBench(iface, {"right": 0xFF}, TRIGGER, settle_s=0.0)
    out = bench.capture(10, seedless=False)
    assert len(out) == 80
    req = wf.requests[0]
    assert req.trigger_config["LaserPulseSkipDelayUsec"] == 1800
    assert req.skip_default_storage and req.raw_save_max_duration_s == 0
    assert (req.left_camera_mask, req.right_camera_mask) == (0, 0xFF)
    assert 0.5 <= req.duration_sec < 1.0
    with pytest.raises(RuntimeError):
        bench.capture(10, seedless=True)        # registers not seedless yet
    bench.enter_seedless(500.0)
    bench.capture(10, seedless=True)
    assert wf.requests[1].trigger_config["LaserPulseSkipDelayUsec"] == \
        SEEDLESS_TRIGGER_OVERRIDES["LaserPulseSkipDelayUsec"]
    assert bench.exit_seedless() is None
    for key, value in LIVE.items():
        assert iface.console.regs[key] == value


def test_capture_raises_on_scan_error():
    wf = FakeWorkflow([[]], error="console refused to start the trigger")
    bench = MotionCminBench(AdapterInterface(wf), {"right": 0xFF}, TRIGGER)
    with pytest.raises(RuntimeError, match="refused"):
        bench.capture(10, seedless=False)


def test_enter_seedless_refused_on_envelope_firmware_writes_nothing():
    console = RegisterConsole(boot_info=SimpleNamespace(
        firmware_owns_safety_state=True, factory_image=False))
    bench = MotionCminBench(AdapterInterface(FakeWorkflow([]), console),
                            {"right": 0xFF}, TRIGGER, settle_s=0.0)
    with pytest.raises(RuntimeError, match="660 us"):
        bench.enter_seedless(500.0)
    assert console.writes == []
    assert bench.exit_seedless() is None


def test_exit_seedless_sends_the_standard_trigger_before_rearming_rate_ll():
    iface = AdapterInterface(FakeWorkflow([]))
    bench = MotionCminBench(iface, {"right": 0xFF}, TRIGGER, settle_s=0.0)
    bench.enter_seedless(1000.0)
    n = len(iface.console.writes)
    assert bench.exit_seedless() is None
    after = iface.console.writes[n:]
    keys = [w[0] for w in after]
    assert after[0] == ("trigger", TRIGGER)
    assert keys.index("trigger") < keys.index((6, 0x08)) and \
        keys.index("trigger") < keys.index((7, 0x08))
    assert iface.console.trigger["LaserPulseSkipDelayUsec"] == 1800


def test_exit_seedless_trigger_refused_still_restores_and_reports():
    iface = AdapterInterface(FakeWorkflow([]), trigger_ok=False)
    bench = MotionCminBench(iface, {"right": 0xFF}, TRIGGER, settle_s=0.0)
    bench.enter_seedless(1000.0)
    problem = bench.exit_seedless()
    assert problem and "trigger config" in problem
    for key, value in LIVE.items():
        assert iface.console.regs[key] == value


def test_seedless_capture_raises_when_a_safety_fault_latched():
    scan = [frame(c, i) for c in range(8) for i in range(11, 40)]
    iface = AdapterInterface(FakeWorkflow([scan]))
    bench = MotionCminBench(iface, {"right": 0xFF}, TRIGGER, settle_s=0.0)
    bench.enter_seedless(1000.0)
    iface.console.regs[(6, 0x24)] = bytes([0x01])      # EE power fault
    with pytest.raises(RuntimeError, match="EE STATUS=0x01"):
        bench.capture(10, seedless=True)


def test_adapter_current_round_trip():
    iface = AdapterInterface(FakeWorkflow([]))
    bench = MotionCminBench(iface, {"right": 0xFF}, TRIGGER, settle_s=0.0)
    bench.enter_seedless(1000.0)
    assert bench.ta_current_ma() == pytest.approx(3900.0)
    bench.set_ta_current_ma(4500.0)
    assert bench.ta_current_ma() == pytest.approx(4500.0)
    assert bench.exit_seedless() is None
    assert iface.console.regs[(4, 0x06)] == LIVE[(4, 0x06)]


def test_exit_seedless_reports_refused_write():
    console = RegisterConsole()
    bench = MotionCminBench(AdapterInterface(FakeWorkflow([]), console),
                            {"right": 0xFF}, TRIGGER, settle_s=0.0)
    bench.enter_seedless(1000.0)
    console.refuse = {(7, 0x08)}       # OPT_RATE_LL cannot be re-armed
    problem = bench.exit_seedless()
    assert problem and "power-cycle" in problem


def test_script_runs_on_a_factory_image(tmp_path, monkeypatch):
    info = SimpleNamespace(firmware_owns_safety_state=True, factory_image=True)
    code, lines, _, _ = run_main(tmp_path, monkeypatch, boot_info=info)
    assert code == 0, lines
