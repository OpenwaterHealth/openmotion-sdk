"""Unit tests for the NVCM burn pacing guard and cold-start preflight.

Regression tests for the 2026-08-13 bricking incident (issue #245): with the
CommInterface fixed sleeps removed (PR #234), the burn replay ran at
~0.5 ms/transaction and corrupted CrossLink NVCM mid-array on 12 cameras —
permanently, since NVCM is one-time programmable. The guard makes the burn
own its pace (_SensorI2CDriver.MIN_DISPATCH_INTERVAL_S) instead of
inheriting whatever the transport delivers, and starts every burn from a
hard power-cycle of the target camera.

Software-only: no hardware fixtures, everything runs against fakes.
"""

import time

import omotion.NvcmProgrammer as nvcm_mod
from omotion.NvcmProgrammer import NvcmProgrammer, _SensorI2CDriver


class FakeSensor:
    """Instant-response sensor that records every call in order."""

    def __init__(self, power_off_ok=True, power_on_ok=True, switch_ok=True):
        self.calls = []
        self._power_off_ok = power_off_ok
        self._power_on_ok = power_on_ok
        self._switch_ok = switch_ok

    def disable_camera_power(self, mask):
        self.calls.append(("power_off", mask))
        return self._power_off_ok

    def enable_camera_power(self, mask):
        self.calls.append(("power_on", mask))
        return self._power_on_ok

    def switch_camera(self, idx):
        self.calls.append(("switch", idx))
        if not self._switch_ok:
            return None
        # packetType=None is never in MotionSensor._ERROR_TYPES.
        return type("Resp", (), {"packetType": None})()

    def i2c_write(self, addr, data):
        self.calls.append(("i2c_write", len(data)))
        return None

    def i2c_read(self, addr, num_bytes):
        self.calls.append(("i2c_read", num_bytes))
        return bytes(num_bytes)

    def i2c_write_read(self, addr, data, num_bytes):
        self.calls.append(("i2c_write_read", num_bytes))
        return bytes(num_bytes)

    def creset(self, value):
        self.calls.append(("creset", value))
        return 1 if value else 0


# ---------------------------------------------------------------------------
# Pacing floor
# ---------------------------------------------------------------------------

def test_pacing_floor_is_validated_envelope():
    # 2.0 ms/dispatch is inside the only proven-good zone (June 2026
    # validation: 2.26 ms/tx; 1.4.2 exe passes: 1.43-1.60 ms/tx). Every burn
    # observed at <=0.67 ms/tx bricked its OTP part. If you are editing this
    # constant you must revalidate on expendable hardware first (issue #245).
    assert _SensorI2CDriver.MIN_DISPATCH_INTERVAL_S == 0.002


def test_dispatches_are_paced():
    driver = _SensorI2CDriver(FakeSensor())
    driver.MIN_DISPATCH_INTERVAL_S = 0.02  # big floor so the test is robust

    t0 = time.monotonic()
    # 6 paced dispatches: 2 write STOPs, 2 reads, 2 cresets -> 5 gaps.
    for _ in range(2):
        driver.start()
        driver.write(bytes([0x80, 0xE0, 0x00]))
        driver.stop()
    for _ in range(2):
        driver.start()
        driver.write(bytes([0x81]))
        driver.read(4)
    driver.creset(0)
    driver.creset(1)
    elapsed = time.monotonic() - t0

    assert elapsed >= 5 * 0.02 * 0.95  # 5 start-to-start gaps at the floor


def test_non_dispatching_ops_are_not_paced():
    driver = _SensorI2CDriver(FakeSensor())
    driver.MIN_DISPATCH_INTERVAL_S = 0.05

    t0 = time.monotonic()
    for _ in range(20):  # empty STOPs dispatch nothing -> no pacing
        driver.start()
        driver.stop()
    elapsed = time.monotonic() - t0

    assert elapsed < 0.05


def test_sim_driver_is_not_paced():
    from omotion.NvcmProgrammer import _CountingSimDriver

    sim = _CountingSimDriver()
    t0 = time.monotonic()
    for _ in range(5000):
        sim.stop()
        sim.read(16)
    assert time.monotonic() - t0 < 0.5
    assert sim.count == 10000


# ---------------------------------------------------------------------------
# Cold-start preflight
# ---------------------------------------------------------------------------

def _stub_isp(return_code=0):
    """isp_entry_point stand-in: one countable transaction, then done."""

    def fake_isp(algo, data, driver=None):
        if driver.is_simulation():
            driver.read(1)  # give the progress pre-pass a nonzero total
        else:
            driver.start()
            driver.write(bytes([0x80, 0xE0]))
            driver.stop()
        return return_code

    return fake_isp


def test_burn_power_cycles_before_touching_the_fpga(monkeypatch):
    fake = FakeSensor()
    monkeypatch.setattr(nvcm_mod, "isp_entry_point", _stub_isp())
    monkeypatch.setattr(nvcm_mod, "_POWER_OFF_SETTLE_S", 0)
    monkeypatch.setattr(nvcm_mod, "_POWER_ON_SETTLE_S", 0)

    result = NvcmProgrammer(fake).burn(3)

    assert result.success
    assert fake.calls[:3] == [
        ("power_off", 0x04),
        ("power_on", 0x04),
        ("switch", 2),
    ]


def test_burn_aborts_when_power_off_is_refused(monkeypatch):
    fake = FakeSensor(power_off_ok=False)
    monkeypatch.setattr(nvcm_mod, "isp_entry_point", _stub_isp())
    monkeypatch.setattr(nvcm_mod, "_POWER_OFF_SETTLE_S", 0)
    monkeypatch.setattr(nvcm_mod, "_POWER_ON_SETTLE_S", 0)

    result = NvcmProgrammer(fake).burn(1)

    assert not result.success
    assert "power-cycle" in result.error
    # The refusal must stop everything: no power-on, no mux, no I2C.
    assert [c for c in fake.calls if c[0] != "power_off"] == []


def test_burn_aborts_when_power_on_is_refused(monkeypatch):
    fake = FakeSensor(power_on_ok=False)
    monkeypatch.setattr(nvcm_mod, "isp_entry_point", _stub_isp())
    monkeypatch.setattr(nvcm_mod, "_POWER_OFF_SETTLE_S", 0)
    monkeypatch.setattr(nvcm_mod, "_POWER_ON_SETTLE_S", 0)

    result = NvcmProgrammer(fake).burn(8)

    assert not result.success
    assert result.error == "failed to power camera"
    assert [c[0] for c in fake.calls] == ["power_off", "power_on"]


def test_burn_never_raises_on_transport_failure(monkeypatch):
    # Existing never-raises contract still holds with the preflight in place.
    fake = FakeSensor(switch_ok=False)
    monkeypatch.setattr(nvcm_mod, "isp_entry_point", _stub_isp())
    monkeypatch.setattr(nvcm_mod, "_POWER_OFF_SETTLE_S", 0)
    monkeypatch.setattr(nvcm_mod, "_POWER_ON_SETTLE_S", 0)

    result = NvcmProgrammer(fake).burn(2)

    assert not result.success
    assert "switch_camera" in result.error
