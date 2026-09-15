"""omotion.laser — bundled laser-power config + I2C application."""

from types import SimpleNamespace

from omotion.data.fpga_model import FPGA_MODEL
from omotion.data.laser_params import LASER_PARAMS
from omotion.data.laser_params_fault import LASER_PARAMS_FAULT
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


class _ConfigConsole(_FakeConsole):
    """Fake console whose read_config carries user-config overrides."""

    def __init__(self, cfg, write_ok=True):
        super().__init__(write_ok=write_ok)
        self._cfg = cfg

    def read_config(self):
        return SimpleNamespace(json_data=self._cfg)


def test_load_laser_params_returns_bundled_list():
    params = load_laser_params()
    assert params, "bundled LASER_PARAMS should be non-empty"
    assert all("friendlyName" in p and "dataToSend" in p for p in params)
    assert params == LASER_PARAMS


def test_load_laser_params_fault_set_available():
    fault = load_laser_params(force_fault=True)
    assert fault, "fault param set should load"
    assert fault == LASER_PARAMS_FAULT


# ── compiled-in data modules (sdk#278) ────────────────────────────────────
#
# The register baseline used to be JSON files parsed at every call; each call
# now returns a deep copy of a module constant. The copy matters: the app's
# alt-laser path edits the list it gets back, and the baseline behind it must
# survive that for the next connect.

def test_load_laser_params_returns_a_fresh_copy_each_call():
    first = load_laser_params()
    first[0]["dataToSend"][0] = 0xFF
    first.append({"friendlyName": "BOGUS", "dataToSend": [1]})
    second = load_laser_params()
    assert second == LASER_PARAMS
    assert second[0]["dataToSend"][0] == 27
    assert LASER_PARAMS[0]["dataToSend"][0] == 27


def test_fpga_map_does_not_alias_the_module_constant():
    fmap = FpgaMap()
    fmap._model[0]["channel"] = 99
    assert FPGA_MODEL[0]["channel"] == 4
    assert FpgaMap().get_entry_by_friendly_name("TA_PULSE_WIDTH")["channel"] == 4


def test_fault_set_mirrors_baseline_register_order():
    # Same registers in the same order; only the faulted value(s) differ, so
    # apply_laser_power walks both sets identically.
    assert [p["friendlyName"] for p in LASER_PARAMS_FAULT] == [
        p["friendlyName"] for p in LASER_PARAMS
    ]


def test_every_baseline_register_resolves_and_has_the_right_width():
    fmap = FpgaMap()
    for params in (LASER_PARAMS, LASER_PARAMS_FAULT):
        for p in params:
            entry = fmap.get_entry_by_friendly_name(p["friendlyName"])
            assert entry is not None, p["friendlyName"]
            width = int(entry["data_size"].rstrip("B")) // 8
            assert len(p["dataToSend"]) == width, p["friendlyName"]
            assert all(isinstance(b, int) and 0 <= b <= 0xFF for b in p["dataToSend"]), (
                p["friendlyName"]
            )


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


# ── force_fault vs user-config overrides (sdk#252) ────────────────────────
#
# The fault file's whole point is a register value engineered to trip the
# interlock; a console whose user config carries the same key (e.g.
# EE_PULSE_WIDTH_UL written during safety-param calibration) used to
# silently restore the safe value right after the fault was staged, so
# forceLaserFail never tripped on calibrated units.

def _faulted_entries():
    """{friendlyName: fault dataToSend} for keys the fault set changes."""
    normal = {p["friendlyName"]: p["dataToSend"] for p in load_laser_params()}
    fault = {
        p["friendlyName"]: p["dataToSend"]
        for p in load_laser_params(force_fault=True)
    }
    return {k: v for k, v in fault.items() if normal.get(k) != v}


def test_fault_set_differs_from_baseline():
    assert _faulted_entries(), "fault set should change at least one register"


def test_force_fault_user_override_cannot_neutralize_fault_registers():
    fmap = FpgaMap()
    faulted = _faulted_entries()
    # The bench scenario: user config carries an override for every faulted
    # key (value irrelevant — it must not win).
    console = _ConfigConsole({k: 550 for k in faulted})

    assert apply_laser_power(console, force_fault=True) is True

    for name, fault_data in faulted.items():
        entry = fmap.get_entry_by_friendly_name(name)
        writes = [
            w for w in console.writes
            if w[1] == entry["channel"] and w[3] == entry["start_address"]
        ]
        # Exactly one write to the faulted register, carrying the fault
        # bytes verbatim — no override rewrite, no trailing config write.
        assert writes == [(
            entry["mux_idx"], entry["channel"], entry["i2c_addr"],
            entry["start_address"], bytes(fault_data),
        )], name


def test_force_fault_still_applies_overrides_to_non_faulted_registers():
    fmap = FpgaMap()
    faulted = _faulted_entries()
    assert "TA_CURRENT_DRV" not in faulted  # else pick another register
    console = _ConfigConsole({"TA_CURRENT_DRV": 0})

    assert apply_laser_power(console, force_fault=True) is True

    entry = fmap.get_entry_by_friendly_name("TA_CURRENT_DRV")
    num_bytes = int(entry["data_size"].rstrip("B")) // 8
    writes = [
        w for w in console.writes
        if w[1] == entry["channel"] and w[3] == entry["start_address"]
    ]
    # Calibrated drive values still apply during the interlock test.
    assert writes == [(
        entry["mux_idx"], entry["channel"], entry["i2c_addr"],
        entry["start_address"], bytes(num_bytes),
    )]


def test_normal_apply_still_honors_override_on_fault_registers():
    # Without force_fault the per-device override must keep winning — the
    # exemption is scoped to the interlock test only.
    fmap = FpgaMap()
    name = next(iter(_faulted_entries()))
    entry = fmap.get_entry_by_friendly_name(name)
    scale = entry["scale"] or 1
    num_bytes = int(entry["data_size"].rstrip("B")) // 8
    byteorder = "big" if entry["isMsbFirst"] else "little"
    console = _ConfigConsole({name: 550})

    assert apply_laser_power(console) is True

    raw = int(round(550 / scale))
    writes = [
        w for w in console.writes
        if w[1] == entry["channel"] and w[3] == entry["start_address"]
    ]
    assert writes == [(
        entry["mux_idx"], entry["channel"], entry["i2c_addr"],
        entry["start_address"], raw.to_bytes(num_bytes, byteorder=byteorder),
    )]


def test_force_fault_keeps_trailing_drive_cl_write_when_not_faulted():
    # EE_THRESH/EE_GAIN drive the trailing Safety EE DRIVE CL write (ch 6,
    # reg 0x10). The current fault set doesn't fault DRIVE CL, so the
    # config-derived write must survive force_fault untouched.
    assert (6, 0x10) not in {
        (FpgaMap().get_entry_by_friendly_name(n)["channel"],
         FpgaMap().get_entry_by_friendly_name(n)["start_address"])
        for n in _faulted_entries()
    }
    console = _ConfigConsole({"EE_THRESH": 1000, "EE_GAIN": 2})

    assert apply_laser_power(console, force_fault=True) is True

    writes = [w for w in console.writes if w[1] == 6 and w[3] == 0x10]
    assert writes == [(1, 6, 0x41, 0x10, bytes([0xF4, 0x01]))]  # 500 LSB-first
