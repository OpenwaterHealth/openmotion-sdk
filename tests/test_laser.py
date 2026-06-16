"""omotion.laser — bundled laser-power config + I2C application."""

from omotion.laser import FpgaMap, apply_laser_power, load_laser_params


class _FakeConsole:
    """Records write_i2c_packet calls; read_config returns no user overrides."""

    def __init__(self, write_ok=True):
        self.writes = []
        self._write_ok = write_ok

    def read_config(self):
        return None

    def write_i2c_packet(self, *, mux_index, channel, device_addr, reg_addr, data):
        self.writes.append((mux_index, channel, device_addr, reg_addr, bytes(data)))
        return self._write_ok


def test_load_laser_params_returns_bundled_list():
    params = load_laser_params()
    assert params, "bundled laser_params.json should be non-empty"
    assert all("friendlyName" in p and "dataToSend" in p for p in params)


def test_load_laser_params_fault_set_available():
    assert load_laser_params(force_fault=True), "fault param set should load"


def test_fpga_map_lookup_known_entry():
    entry = FpgaMap().get_entry_by_friendly_name("TA_PULSE_WIDTH")
    assert entry is not None
    assert entry["mux_idx"] == 1
    assert entry["channel"] == 4
    assert entry["i2c_addr"] == 65
    assert entry["start_address"] == 0
    assert entry["data_size"] == "24B"


def test_fpga_map_unknown_entry_returns_none():
    assert FpgaMap().get_entry_by_friendly_name("NOT_A_REAL_NAME") is None


def test_apply_laser_power_writes_bundled_params():
    console = _FakeConsole()
    assert apply_laser_power(console) is True
    assert console.writes, "expected at least one I2C write"
    # First bundled param is TA_PULSE_WIDTH (dataToSend [27,6,0]) → TA block:
    # mux 1, channel 4, device 0x41 (65), register 0.
    assert console.writes[0] == (1, 4, 65, 0, bytes([27, 6, 0]))


def test_apply_laser_power_returns_false_on_write_failure():
    assert apply_laser_power(_FakeConsole(write_ok=False)) is False


def test_apply_laser_power_false_when_no_params():
    # Empty params + a map that finds nothing → nothing to apply.
    assert apply_laser_power(_FakeConsole(), laser_params=[]) is False


class _Lock:
    def __init__(self):
        self.locked = 0
        self.unlocked = 0

    def lock(self):
        self.locked += 1

    def unlock(self):
        self.unlocked += 1


def test_apply_laser_power_holds_lock_around_writes():
    lk = _Lock()
    assert apply_laser_power(_FakeConsole(), lock=lk) is True
    assert lk.locked == 1 and lk.unlocked == 1


def test_apply_laser_power_releases_lock_on_write_failure():
    lk = _Lock()
    assert apply_laser_power(_FakeConsole(write_ok=False), lock=lk) is False
    assert lk.locked == 1 and lk.unlocked == 1
