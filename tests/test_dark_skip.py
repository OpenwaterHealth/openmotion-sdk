"""Long-exposure dark skip and its RATE_LL floor (omotion.dark_skip,
bloodflow-app#747). Pure software: a fake register IO stands in for the
console's safety FPGAs."""

from types import SimpleNamespace
import pytest

from omotion.dark_skip import (
    CLEAN_EXPOSURE_MAX_US,
    LONG_DARK_SKIP_US,
    SAFETY_RATE_LL_US,
    LongExposureDarkSkip,
    needs_long_skip,
)

pytestmark = pytest.mark.unit


class FakeRegisters:
    def __init__(self, rate_ll=23125.0, refuse=(), status=0):
        self.values = {"EE_RATE_LL": rate_ll, "OPT_RATE_LL": rate_ll,
                       "EE_STATUS": status, "OPT_STATUS": status}
        self.refuse = set(refuse)
        self.writes = []

    def read(self, name):
        return self.values[name]

    def write(self, name, value):
        self.writes.append((name, value))
        if name in self.refuse:
            return None
        self.values[name] = float(value)
        return SimpleNamespace(actual=float(value))


# ---- the decision -----------------------------------------------------------

def test_defaults_never_need_the_long_skip():
    assert not needs_long_skip()
    assert not needs_long_skip(648, 500)


def test_the_boundary_is_1700_us_of_exposure():
    assert not needs_long_skip(CLEAN_EXPOSURE_MAX_US)
    assert needs_long_skip(CLEAN_EXPOSURE_MAX_US + 9)
    assert needs_long_skip(2196)


def test_delay_plus_pulse_width_counts_too():
    assert not needs_long_skip(None, 1600, pulse_delay_us=100)
    assert needs_long_skip(None, 1700, pulse_delay_us=100)
    assert not needs_long_skip(None, 1700, pulse_delay_us=0)


# ---- prepare ----------------------------------------------------------------

def test_no_timing_no_override_no_io():
    regs = FakeRegisters()
    ds = LongExposureDarkSkip(regs)
    assert ds.prepare(None) is None
    assert regs.writes == []


def test_long_exposure_lowers_both_floors_then_returns_the_skip():
    regs = FakeRegisters(rate_ll=23125.0)
    ds = LongExposureDarkSkip(regs)
    ds.set_device_timing(camera_exposure_us=2196)
    assert ds.prepare(None) == {"LaserPulseSkipDelayUsec": LONG_DARK_SKIP_US}
    assert regs.writes == [("EE_RATE_LL", SAFETY_RATE_LL_US), ("OPT_RATE_LL", SAFETY_RATE_LL_US)]
    assert ds.lowered


def test_a_second_long_scan_writes_nothing_more():
    regs = FakeRegisters()
    ds = LongExposureDarkSkip(regs)
    ds.set_device_timing(camera_exposure_us=2196)
    ds.prepare(None)
    regs.writes.clear()
    assert ds.prepare(None) == {"LaserPulseSkipDelayUsec": LONG_DARK_SKIP_US}
    assert regs.writes == []


def test_the_first_short_scan_puts_back_what_the_console_held_once():
    regs = FakeRegisters(rate_ll=23125.0)
    ds = LongExposureDarkSkip(regs)
    ds.set_device_timing(camera_exposure_us=2196)
    ds.prepare(None)
    ds.set_device_timing()
    regs.writes.clear()
    assert ds.prepare(None) is None
    assert regs.writes == [("EE_RATE_LL", 23125.0), ("OPT_RATE_LL", 23125.0)]
    assert not ds.lowered
    regs.writes.clear()
    assert ds.prepare(None) is None
    assert regs.writes == []


def test_a_floor_already_low_enough_is_not_touched_or_put_back():
    regs = FakeRegisters(rate_ll=22000.0)
    ds = LongExposureDarkSkip(regs)
    ds.set_device_timing(camera_exposure_us=2196)
    assert ds.prepare(None) == {"LaserPulseSkipDelayUsec": LONG_DARK_SKIP_US}
    ds.set_device_timing()
    ds.prepare(None)
    assert regs.writes == []


def test_a_refused_write_keeps_the_default_skip_and_puts_back_the_other():
    regs = FakeRegisters(rate_ll=23125.0, refuse={"OPT_RATE_LL"})
    ds = LongExposureDarkSkip(regs)
    ds.set_device_timing(camera_exposure_us=2196)
    assert ds.prepare(None) is None
    assert regs.values["EE_RATE_LL"] == 23125.0
    assert not ds.lowered


def test_a_failed_put_back_is_retried_at_the_next_scan():
    regs = FakeRegisters(rate_ll=23125.0)
    ds = LongExposureDarkSkip(regs)
    ds.set_device_timing(camera_exposure_us=2196)
    ds.prepare(None)
    ds.set_device_timing()
    regs.refuse = {"OPT_RATE_LL"}
    ds.prepare(None)
    assert regs.values["EE_RATE_LL"] == 23125.0
    assert ds.lowered
    regs.refuse = set()
    ds.prepare(None)
    assert regs.values["OPT_RATE_LL"] == 23125.0
    assert not ds.lowered


def test_a_console_that_reloaded_its_limits_is_not_written_again():
    regs = FakeRegisters(rate_ll=23125.0)
    ds = LongExposureDarkSkip(regs)
    ds.set_device_timing(camera_exposure_us=2196)
    ds.prepare(None)
    # Reconnect / apply_laser_power reloads the flash value.
    regs.values["EE_RATE_LL"] = regs.values["OPT_RATE_LL"] = 22500.0
    ds.set_device_timing()
    regs.writes.clear()
    ds.prepare(None)
    assert regs.writes == []
    assert not ds.lowered


def test_a_reloaded_console_is_lowered_again_and_put_back_to_its_new_value():
    regs = FakeRegisters(rate_ll=23125.0)
    ds = LongExposureDarkSkip(regs)
    ds.set_device_timing(camera_exposure_us=2196)
    ds.prepare(None)
    regs.values["EE_RATE_LL"] = regs.values["OPT_RATE_LL"] = 22500.0
    ds.prepare(None)
    ds.set_device_timing()
    ds.prepare(None)
    assert regs.values["EE_RATE_LL"] == regs.values["OPT_RATE_LL"] == 22500.0


def test_a_request_that_sets_its_own_skip_is_left_alone():
    """Optimize Laser's preparer (or a caller) owns the skip and the limits."""
    regs = FakeRegisters()
    ds = LongExposureDarkSkip(regs)
    ds.set_device_timing(camera_exposure_us=2196)
    assert ds.prepare({"LaserPulseSkipDelayUsec": 2400}) is None
    assert regs.writes == []


def test_a_laser_off_scan_needs_no_skip():
    regs = FakeRegisters()
    ds = LongExposureDarkSkip(regs)
    ds.set_device_timing(camera_exposure_us=2196)
    assert ds.prepare(None, disable_laser=True) is None
    assert regs.writes == []


def test_registers_given_as_a_factory_are_built_on_use():
    regs = FakeRegisters()
    built = []

    def factory():
        built.append(1)
        return regs

    ds = LongExposureDarkSkip(factory)
    assert ds.prepare(None) is None
    assert built == []
    ds.set_device_timing(laser_pulse_width_us=2000)
    assert ds.prepare(None) == {"LaserPulseSkipDelayUsec": LONG_DARK_SKIP_US}
    assert built


# ---- ScanWorkflow -------------------------------------------------------------

def _motion():
    from omotion.MotionInterface import MotionInterface
    return MotionInterface(demo_mode=True, data_dir=None, operator_id="test")


def _sent_trigger(motion, request):
    from tests.test_scan_workflow import _run_scan_capturing_trigger
    calls = _run_scan_capturing_trigger(motion, request)
    return next(payload for kind, payload in calls if kind == "set")


def _request(**kw):
    from omotion.ScanWorkflow import ScanRequest
    return ScanRequest(subject_id="x", duration_sec=30, left_camera_mask=0xFF,
                       right_camera_mask=0, skip_default_storage=True, **kw)


def test_start_scan_sends_the_long_skip_after_lowering_the_floor():
    motion = _motion()
    regs = FakeRegisters()
    motion.scan_workflow._dark_skip._registers_src = regs
    motion.scan_workflow.set_device_timing(camera_exposure_us=2196)
    sent = _sent_trigger(motion, _request())
    assert sent["LaserPulseSkipDelayUsec"] == LONG_DARK_SKIP_US
    assert ("EE_RATE_LL", SAFETY_RATE_LL_US) in regs.writes


def test_start_scan_keeps_the_default_skip_without_device_timing():
    motion = _motion()
    regs = FakeRegisters()
    motion.scan_workflow._dark_skip._registers_src = regs
    sent = _sent_trigger(motion, _request())
    assert sent["LaserPulseSkipDelayUsec"] == motion.resolve_trigger_config(None)[
        "LaserPulseSkipDelayUsec"]
    assert regs.writes == []


def test_a_preparer_skip_wins_and_nothing_is_written():
    motion = _motion()
    regs = FakeRegisters()
    motion.scan_workflow._dark_skip._registers_src = regs
    motion.scan_workflow.set_device_timing(camera_exposure_us=2196)
    motion.scan_workflow.set_scan_preparer(
        lambda left, right: {"trigger_config": {"LaserPulseSkipDelayUsec": 2400}})
    sent = _sent_trigger(motion, _request())
    assert sent["LaserPulseSkipDelayUsec"] == 2400
    assert regs.writes == []
