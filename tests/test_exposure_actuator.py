"""Tests for omotion/exposure/actuator.py against a fake console and sensors.

Pure-software: the fake console keeps raw I2C bytes at the real register
locations, so FpgaRegisterIO scaling and readback run for real.
"""

import pytest

from omotion.exposure.actuator import (
    FW_DEFAULT_EXPOSURE_ROWS,
    FW_DEFAULT_GAIN_CODES,
    SAFETY_DRIVE_CL,
    SAFETY_PULSE_WIDTH_UL_US,
    SAFETY_RATE_LL_US,
    LaserCameraActuator,
    camera_register_writes,
)
from omotion.exposure.firmware import ExtendedLimitsRefused
from omotion.exposure.settings import Bounds, HardwareLimits, OptimizeLaserSettings

from optimize_laser_fakes import (
    CALIBRATED_REGISTERS,
    FakeConsole,
    FakeSensor,
    fake_apply_laser_power,
)

pytestmark = pytest.mark.unit

LIMITS = HardwareLimits()
SETTINGS = OptimizeLaserSettings()
SAFETY_LIMITS = ("PULSE_WIDTH_UL", "RATE_LL", "DRIVE_CL")


def _actuator(console=None, **sensor_kwargs):
    console = console or FakeConsole()
    sensors = {0: FakeSensor(**sensor_kwargs), 1: FakeSensor()}
    return LaserCameraActuator(console, sensors, restore_laser=fake_apply_laser_power), \
        console, sensors


def _safety_writes(console):
    return [(n, v) for n, v in console.writes if n.split("_", 1)[1] in SAFETY_LIMITS]


# ---- camera registers -------------------------------------------------------


def test_camera_register_bytes():
    # 244 rows = 2196 us; code 37 = 2 + 5/16 x.
    assert camera_register_writes(244, 37) == (
        (0x3501, 0x00), (0x3502, 0xF4), (0x3508, 0x02), (0x3509, 0x50))
    assert camera_register_writes(72, 256)[2:] == ((0x3508, 0x10), (0x3509, 0x00))   # 16x
    assert camera_register_writes(72, 16)[2:] == ((0x3508, 0x01), (0x3509, 0x00))    # 1x


@pytest.mark.parametrize("rows, code", [(10, 16), (245, 16), (72, 15), (72, 257)])
def test_camera_register_writes_refuse_out_of_range(rows, code):
    with pytest.raises(ValueError):
        camera_register_writes(rows, code)


# ---- preflight --------------------------------------------------------------


def test_preflight_reads_the_baseline_and_writes_nothing():
    actuator, console, _ = _actuator()
    baseline = actuator.preflight(SETTINGS, LIMITS)
    assert baseline.firmware == "1.8.1"
    assert baseline.ta_pulse_width_us == pytest.approx(530.0, abs=0.32)
    assert baseline.ta_current_ma == pytest.approx(5000.0, abs=0.16)
    assert baseline.registers["EE_RATE_LL"] == pytest.approx(23125.0, abs=0.32)
    assert console.writes == []


@pytest.mark.parametrize("console, match", [
    (FakeConsole(version="1.8.2-rc.4"), "safety envelope"),
    (FakeConsole(version="1.8.1-3-gabc1234"), "between release tags"),
    (FakeConsole(owns=True), "owns the laser-safety limits"),
])
def test_preflight_refuses_envelope_firmware(console, match):
    actuator, console, _ = _actuator(console)
    with pytest.raises(ExtendedLimitsRefused, match=match):
        actuator.preflight(SETTINGS, LIMITS)
    assert console.writes == []


@pytest.mark.parametrize("values, settings, match", [
    ({"EE_STATUS": 0x04}, SETTINGS, "safety fault latched"),
    ({"TA_CURRENT_LIMIT": 4800.0}, SETTINGS, "TA_CURRENT_LIMIT"),
    ({"OPT_PULSE_WIDTH_LL": 90.0}, SETTINGS, "OPT_PULSE_WIDTH_LL"),
])
def test_preflight_refuses_a_console_the_run_would_trip(values, settings, match):
    actuator, console, _ = _actuator(FakeConsole(values=values))
    with pytest.raises(RuntimeError, match=match):
        actuator.preflight(settings, LIMITS)
    assert console.writes == []


def test_preflight_allows_a_lower_limit_below_the_shortest_pulse():
    actuator, _, _ = _actuator(FakeConsole(values={"EE_PULSE_WIDTH_LL": 50.0}))
    actuator.preflight(SETTINGS, LIMITS)        # 100 - 17.3 us emitted clears 50 us


def test_preflight_refuses_unverified_laser_limits():
    console = FakeConsole()
    console.laser_limits_error = "Laser safety limit mismatch: EE_RATE_LL"
    actuator, _, _ = _actuator(console)
    with pytest.raises(RuntimeError, match="not verified"):
        actuator.preflight(SETTINGS, LIMITS)


# ---- loosening the limits ---------------------------------------------------


def test_loosen_raises_ceiling_and_lowers_rate_on_both_fpgas():
    actuator, console, _ = _actuator()
    written = actuator.loosen_limits(actuator.preflight(SETTINGS, LIMITS), 5000.0)
    assert set(written) == {"EE_PULSE_WIDTH_UL", "OPT_PULSE_WIDTH_UL", "EE_RATE_LL", "OPT_RATE_LL"}
    for side in ("EE", "OPT"):
        assert console.get(f"{side}_PULSE_WIDTH_UL") == pytest.approx(SAFETY_PULSE_WIDTH_UL_US)
        assert console.get(f"{side}_RATE_LL") == pytest.approx(SAFETY_RATE_LL_US)
    # The bench unit already runs 5000 mA under DRIVE_CL 9999: nothing to raise.
    assert console.wrote("EE_DRIVE_CL") == [] and console.wrote("OPT_DRIVE_CL") == []


def test_loosen_raises_drive_cl_above_the_units_calibrated_current():
    actuator, console, _ = _actuator(FakeConsole(values=CALIBRATED_REGISTERS))
    written = actuator.loosen_limits(actuator.preflight(SETTINGS, LIMITS), 5000.0)
    assert {"EE_DRIVE_CL", "OPT_DRIVE_CL"} <= set(written)
    assert console.get("EE_DRIVE_CL") == pytest.approx(SAFETY_DRIVE_CL, abs=1.86)
    assert console.get("OPT_DRIVE_CL") == pytest.approx(SAFETY_DRIVE_CL, abs=1.86)


def test_raising_drive_cl_never_passes_through_a_lower_limit():
    """The safety FPGA takes the limit low byte first, so between the two
    bytes it holds old-high : new-low. Bench, 2026-11-10: 0x07F4 -> 0x1500
    went through 0x0700, under the held peak, and EE latched."""
    actuator, console, _ = _actuator(FakeConsole(values=CALIBRATED_REGISTERS))
    before = {s: int(round(console.get(f"{s}_DRIVE_CL") / 1.86)) for s in ("EE", "OPT")}
    actuator.loosen_limits(actuator.preflight(SETTINGS, LIMITS), 5000.0)
    for side, old in before.items():
        new = int(round(console.get(f"{side}_DRIVE_CL") / 1.86))
        between = (old & 0xFF00) | (new & 0x00FF)
        assert new & 0xFF == 0xFF and between >= old, (side, hex(old), hex(new))


def test_loosen_leaves_drive_cl_when_the_current_never_rises():
    actuator, console, _ = _actuator(FakeConsole(values=CALIBRATED_REGISTERS))
    actuator.loosen_limits(actuator.preflight(SETTINGS, LIMITS), 4400.0)
    assert console.wrote("EE_DRIVE_CL") == []


def test_loosen_never_tightens_a_limit():
    already_loose = {"EE_PULSE_WIDTH_UL": 3000.0, "OPT_PULSE_WIDTH_UL": 2200.0,
                     "EE_RATE_LL": 0.0, "OPT_RATE_LL": 22000.0,
                     "TA_CURRENT_DRV": 4400.0, "EE_DRIVE_CL": 12000.0, "OPT_DRIVE_CL": 13286.0}
    actuator, console, _ = _actuator(FakeConsole(values=already_loose))
    assert actuator.loosen_limits(actuator.preflight(SETTINGS, LIMITS), 5000.0) == []
    assert console.writes == []


@pytest.mark.parametrize("console", [
    FakeConsole(refuse={"OPT_RATE_LL"}),                        # NAK
    FakeConsole(clamp={"EE_PULSE_WIDTH_UL": 660.0}),            # accepted, not applied
])
def test_loosen_raises_when_a_write_does_not_stick(console):
    actuator, console, _ = _actuator(console)
    with pytest.raises(RuntimeError, match="PULSE_WIDTH_UL|RATE_LL"):
        actuator.loosen_limits(actuator.preflight(SETTINGS, LIMITS), 5000.0)


# ---- each iteration ---------------------------------------------------------


def test_apply_writes_the_laser_then_every_camera():
    actuator, console, sensors = _actuator()
    codes = {(0, 0): 256, (0, 3): 37, (1, 7): 100}
    actuator.apply(codes, 1500.0, 4321.0, 2196.0)
    assert console.get("TA_PULSE_WIDTH") == pytest.approx(1500.0, abs=0.32)
    assert console.get("TA_CURRENT_DRV") == pytest.approx(4321.0, abs=0.16)
    for (side, pos), code in codes.items():
        assert sensors[side].gain_code(pos) == code
        assert sensors[side].exposure_rows(pos) == 244
    assert sensors[0].gain_code(1) == FW_DEFAULT_GAIN_CODES[1]       # untouched


@pytest.mark.parametrize("pw, current", [(19.0, 4000.0), (2201.0, 4000.0), (500.0, 5001.0),
                                         (500.0, 0.0), (float("nan"), 4000.0)])
def test_apply_refuses_values_outside_the_hard_caps(pw, current):
    actuator, console, _ = _actuator()
    with pytest.raises(ValueError):
        actuator.apply({(0, 0): 64}, pw, current, 2196.0)
    assert console.writes == []


def test_apply_reports_every_failed_camera_write():
    actuator, _, _ = _actuator(fail={(2, 0x3508), (5, 0x3502)})
    with pytest.raises(RuntimeError, match="camera 3 reg 0x3508.*camera 6 reg 0x3502"):
        actuator.apply({(0, p): 64 for p in range(8)}, 500.0, 4000.0, 2196.0)


# ---- restore ----------------------------------------------------------------


def test_restore_runs_apply_laser_power_and_resets_the_cameras():
    actuator, console, sensors = _actuator()
    actuator.loosen_limits(actuator.preflight(SETTINGS, LIMITS), 5000.0)
    actuator.apply({(0, 0): 40, (1, 4): 200}, 2000.0, 4800.0, 2196.0)

    assert actuator.restore([(0, 0), (1, 4)]) == []
    assert console.restores == 1
    assert console.get("EE_PULSE_WIDTH_UL") == pytest.approx(660.0, abs=0.32)
    assert console.get("TA_PULSE_WIDTH") == pytest.approx(530.0, abs=0.32)
    assert sensors[0].gain_code(0) == FW_DEFAULT_GAIN_CODES[0]
    assert sensors[1].gain_code(4) == FW_DEFAULT_GAIN_CODES[4]
    assert sensors[1].exposure_rows(4) == FW_DEFAULT_EXPOSURE_ROWS


def test_restore_reports_a_laser_power_failure_but_still_resets_cameras():
    actuator, _, sensors = _actuator(FakeConsole(restore_ok=False))
    actuator.apply({(0, 2): 200}, 500.0, 4000.0, 2196.0)
    problems = actuator.restore([(0, 2)])
    assert problems and "laser power restore failed" in problems[0]
    assert sensors[0].gain_code(2) == FW_DEFAULT_GAIN_CODES[2]


def test_writes_hold_the_callers_lock():
    events = []

    class Lock:
        def lock(self):
            events.append("lock")

        def unlock(self):
            events.append("unlock")

    console = FakeConsole()
    actuator = LaserCameraActuator(console, {0: FakeSensor()}, lock=Lock(),
                                   restore_laser=fake_apply_laser_power)
    actuator.apply({(0, 0): 64}, 500.0, 4000.0, 2196.0)
    assert events == ["lock", "unlock"]


def test_settings_bounds_reach_the_preflight():
    actuator, _, _ = _actuator(FakeConsole(values={"EE_PULSE_WIDTH_LL": 150.0}))
    settings = OptimizeLaserSettings(pulse_width_bounds=Bounds(200.0, 2096.0))
    actuator.preflight(settings, LIMITS)        # 200 - 17.3 us emitted clears 150 us


# ---- a fault latched by the limit writes (bench, 2026-11-10) -----------------


class _LatchingConsole(FakeConsole):
    """Latches EE peak-current when a given register is written, as the bench
    console did during loosen_limits with the trigger off."""

    def __init__(self, trip_on, **kwargs):
        super().__init__(**kwargs)
        self._trip_on = trip_on

    def write_i2c_packet(self, **kwargs):
        ok = super().write_i2c_packet(**kwargs)
        if self.writes and self.writes[-1][0] == self._trip_on:
            self.set("EE_STATUS", 0x01)
        return ok


def test_loosen_stops_when_the_writes_latch_the_interlock():
    console = _LatchingConsole("EE_DRIVE_CL", values=CALIBRATED_REGISTERS)
    actuator, console, _ = _actuator(console)
    baseline = actuator.preflight(SETTINGS, LIMITS)
    with pytest.raises(RuntimeError, match="latched while the limits were being loosened"
                                           ".*EE_STATUS = 0x01.*power-cycle"):
        actuator.loosen_limits(baseline, 5000.0)


def test_restore_reports_an_interlock_that_is_still_latched():
    actuator, console, _ = _actuator()
    console.set("EE_STATUS", 0x01)
    problems = actuator.restore([(0, 0)])
    assert len(problems) == 1 and problems[0].startswith("laser power restore")
    assert "EE_STATUS = 0x01" in problems[0] and "power-cycle" in problems[0]
