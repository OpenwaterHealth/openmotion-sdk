"""SeedlessController: register apply / TA width / restore. Pure software.

The fake console records every I2C write so the tests can assert exact
registers, values and ORDER. The restore ordering is safety-critical:
re-tightening the safety ULs while the TA still fires a long pulse latches
TA_shutdown in the safety FPGA.
"""

import pytest

from omotion.seedless import (
    EXPOSURE_RESTORE_BYTE,
    EXPOSURE_SEEDLESS_BYTE,
    MAX_TA_PULSE_WIDTH_US,
    PULSE_WIDTH_UL_BASELINE,
    PULSE_WIDTH_UL_SEEDLESS,
    RATE_LL_BASELINE,
    RATE_LL_SEEDLESS,
    SEED_CW_GAIN_BASELINE,
    TA_PULSE_WIDTH_BASELINE,
    SeedlessController,
    seedless_unsupported_reason,
    ta_ticks_from_us,
)

TA = (4, 0x00)
SEED_DDS, SEED_CW = (5, 0x02), (5, 0x04)
EE_UL, OPT_UL = (6, 0x04), (7, 0x04)
EE_RATE, OPT_RATE = (6, 0x08), (7, 0x08)
EE_DYN, OPT_DYN = (6, 0x22), (7, 0x22)
EE_ST, OPT_ST = (6, 0x24), (7, 0x24)

# A unit's own values, deliberately not the bundled baseline, so the tests
# prove restore writes back the snapshot (bench probe 2026-11-24: UL 1718,
# rate LL 72266, seed CW 2035).
LIVE = {
    TA: bytes(TA_PULSE_WIDTH_BASELINE),
    SEED_DDS: bytes([0, 0]),
    SEED_CW: (2035).to_bytes(2, "little"),
    EE_UL: (1718).to_bytes(4, "little"),
    OPT_UL: (1718).to_bytes(4, "little"),
    EE_RATE: (72266).to_bytes(4, "little"),
    OPT_RATE: (72266).to_bytes(4, "little"),
    EE_ST: bytes([0]),
    OPT_ST: bytes([0]),
}


def ta(us):
    return ta_ticks_from_us(us).to_bytes(3, "little")


class FakeConsole:
    def __init__(self, *, refuse=(), clamp=None, fail_reads=False,
                 latched=False, boot_info=None):
        self.regs = dict(LIVE)
        self.writes = []
        self.refuse = set(refuse)
        self.clamp = clamp or {}
        self.fail_reads = fail_reads
        self.boot_info = boot_info
        if latched:
            self.regs[EE_ST] = bytes([0x04])

    def get_boot_info(self):
        return self.boot_info

    def read_i2c_packet(self, mux_index, channel, device_addr, reg_addr, read_len):
        assert (mux_index, device_addr) == (1, 0x41)
        if self.fail_reads:
            return None, None
        data = self.regs[(channel, reg_addr)]
        return data, len(data)

    def write_i2c_packet(self, mux_index, channel, device_addr, reg_addr, data):
        assert (mux_index, device_addr) == (1, 0x41)
        key = (channel, reg_addr)
        if key in self.refuse:
            return False
        self.writes.append((key, bytes(data)))
        self.regs[key] = self.clamp.get(key, bytes(data))
        if key in (EE_DYN, OPT_DYN) and data[0] == 1:
            # The clear pulse clears the latched fault.
            self.regs[EE_ST if key == EE_DYN else OPT_ST] = bytes([0])
        return True

    def index(self, key, value):
        return self.writes.index((key, bytes(value)))


class FakeSensor:
    def __init__(self):
        self.calls = []

    def switch_camera(self, cam):
        self.calls.append(("switch", cam))

    def camera_i2c_write(self, packet):
        self.calls.append(("write", packet.register_address, packet.data))
        return True


def controller(console, sensor=None, us=500.0):
    return SeedlessController(
        console=console, sensors=[("right", sensor or FakeSensor(), 0x03)],
        ta_pulse_width_us=us, settle_s=0.0)


def test_apply_writes_seedless_values_in_safe_order():
    console = FakeConsole()
    ctrl = controller(console, us=1000.0)
    assert ctrl.apply() is True
    for key, value in ((EE_UL, PULSE_WIDTH_UL_SEEDLESS),
                       (OPT_UL, PULSE_WIDTH_UL_SEEDLESS),
                       (EE_RATE, RATE_LL_SEEDLESS), (OPT_RATE, RATE_LL_SEEDLESS),
                       (SEED_DDS, b"\0\0"), (SEED_CW, b"\0\0"),
                       (TA, ta(1000.0))):
        assert console.regs[key] == bytes(value)
    # Fault clear first; limits widen before the seed goes off; TA last.
    assert console.index(EE_DYN, [1, 0]) < console.index(EE_UL, PULSE_WIDTH_UL_SEEDLESS)
    assert console.index(EE_RATE, RATE_LL_SEEDLESS) < console.index(SEED_CW, b"\0\0")
    assert console.index(OPT_UL, PULSE_WIDTH_UL_SEEDLESS) < console.index(TA, ta(1000.0))


def test_apply_sets_exposure_on_masked_cameras_only():
    sensor = FakeSensor()
    controller(FakeConsole(), sensor).apply()
    assert [c for c in sensor.calls if c[0] == "switch"] == [("switch", 0), ("switch", 1)]
    assert ("write", 0x3502, EXPOSURE_SEEDLESS_BYTE) in sensor.calls


def test_refused_safety_write_fails_apply_and_restore_rolls_back():
    # Console firmware that owns the safety state refuses the UL write.
    console = FakeConsole(refuse={EE_UL})
    ctrl = controller(console)
    assert ctrl.apply() is False
    assert console.regs[TA] == LIVE[TA]           # never got as far as TA
    assert ctrl.restore() is True
    for key in (EE_UL, OPT_UL, EE_RATE, OPT_RATE, SEED_DDS, SEED_CW, TA):
        assert console.regs[key] == LIVE[key]


def test_clamped_ta_write_is_caught_by_readback():
    # A console that bounds the operating point silently (stores 500 us).
    console = FakeConsole(clamp={TA: bytes(TA_PULSE_WIDTH_BASELINE)})
    assert controller(console, us=2000.0).apply() is False


def test_unclearable_fault_aborts_before_any_config_write():
    console = FakeConsole(latched=True, refuse={EE_DYN})
    ctrl = controller(console)
    assert ctrl.apply() is False
    assert not any(k in (EE_UL, TA) for k, _ in console.writes)


def test_latched_fault_is_cleared():
    console = FakeConsole(latched=True)
    assert controller(console).apply() is True
    assert console.regs[EE_ST] == b"\0"


def test_set_ta_pulse_width_between_scans():
    console = FakeConsole()
    ctrl = controller(console)
    ctrl.apply()
    assert ctrl.set_ta_pulse_width_us(1500.0) is True
    assert console.regs[TA] == ta(1500.0)
    assert abs(ctrl.ta_pulse_width_us - 1500.0) < 0.32


def test_ta_pulse_width_is_bounded():
    ctrl = controller(FakeConsole())
    ctrl.apply()
    with pytest.raises(ValueError):
        ctrl.set_ta_pulse_width_us(MAX_TA_PULSE_WIDTH_US + 1)
    with pytest.raises(ValueError):
        controller(FakeConsole(), us=MAX_TA_PULSE_WIDTH_US + 1)


def test_set_ta_requires_apply():
    with pytest.raises(RuntimeError):
        controller(FakeConsole()).set_ta_pulse_width_us(1000.0)


def test_restore_order_and_values():
    console = FakeConsole()
    sensor = FakeSensor()
    ctrl = controller(console, sensor, us=2000.0)
    ctrl.apply()
    n = len(console.writes)
    assert ctrl.restore() is True
    restore = console.writes[n:]
    keys = [k for k, _ in restore]
    # TA narrowed before the ULs re-tighten; seed back after.
    assert keys.index(TA) < keys.index(EE_UL) < keys.index(SEED_CW)
    for key in (EE_UL, OPT_UL, EE_RATE, OPT_RATE, SEED_DDS, SEED_CW, TA):
        assert console.regs[key] == LIVE[key]     # the snapshot, not the baseline
    assert sensor.calls[-1] == ("write", 0x3502, EXPOSURE_RESTORE_BYTE)
    assert ctrl.verify_restored() == {}


def test_restore_is_idempotent_and_noop_before_apply():
    console = FakeConsole()
    ctrl = controller(console)
    assert ctrl.restore() is True and console.writes == []
    ctrl.apply()
    ctrl.restore()
    n = len(console.writes)
    assert ctrl.restore() is True and len(console.writes) == n


def test_restore_falls_back_to_baseline_when_snapshot_read_failed():
    console = FakeConsole()
    ctrl = controller(console)
    console.fail_reads = True
    ctrl.apply()          # fails at the first verified write; snapshot = baseline
    console.fail_reads = False
    ctrl.restore()
    assert console.regs[EE_UL] == bytes(PULSE_WIDTH_UL_BASELINE)
    assert console.regs[EE_RATE] == bytes(RATE_LL_BASELINE)
    assert console.regs[SEED_CW] == bytes(SEED_CW_GAIN_BASELINE)


def test_restore_still_tightens_limits_when_ta_write_fails():
    console = FakeConsole()
    ctrl = controller(console, us=2000.0)
    ctrl.apply()
    console.refuse = {TA}
    assert ctrl.restore() is False
    assert console.regs[EE_UL] == LIVE[EE_UL]
    assert console.regs[OPT_UL] == LIVE[OPT_UL]


def test_verify_restored_reports_mismatch():
    console = FakeConsole()
    ctrl = controller(console)
    ctrl.apply()
    ctrl.restore()
    console.regs[OPT_RATE] = bytes(4)
    assert set(ctrl.verify_restored()) == {"OPT_RATE_LL"}


class _Info:
    def __init__(self, owns, factory):
        self.firmware_owns_safety_state = owns
        self.factory_image = factory


@pytest.mark.parametrize("info, refused", [
    (None, False),                      # older firmware
    (_Info(False, None), False),
    # Firmware that owns the safety state enforces its envelope (pulse-width
    # UL <= 660 us) on every image, Factory included (bench 2026-11-24).
    (_Info(True, True), True),
    (_Info(True, False), True),
])
def test_unsupported_reason(info, refused):
    reason = seedless_unsupported_reason(FakeConsole(boot_info=info))
    assert bool(reason) is refused
