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


def _rate_ll_writes(console):
    # Both RATE_LL registers live at 0x41 offset 0x08 (EE ch 6, OPT ch 7).
    return {
        ch: data
        for (mux, ch, dev, reg, data) in console.writes
        if dev == 0x41 and reg == 0x08 and ch in (6, 7)
    }


def test_apply_laser_power_default_rate_keeps_baseline_rate_ll():
    console = _FakeConsole()
    assert apply_laser_power(console, trigger_freq_hz=40.0) is True
    writes = _rate_ll_writes(console)
    # Baseline: 70313 ticks x 0.32 us = 22,500 us min period.
    assert writes[6] == (70313).to_bytes(4, "little")
    assert writes[7] == (70313).to_bytes(4, "little")


def test_apply_laser_power_60hz_scales_rate_ll():
    console = _FakeConsole()
    assert apply_laser_power(console, trigger_freq_hz=60.0) is True
    writes = _rate_ll_writes(console)
    # 70313 * 40/60 = 46875 ticks x 0.32 us = 15,000 us min period —
    # same 0.9x proportional margin at the 16,667 us period of 60 Hz.
    assert writes[6] == (46875).to_bytes(4, "little")
    assert writes[7] == (46875).to_bytes(4, "little")


def test_apply_laser_power_none_rate_keeps_baseline_rate_ll():
    console = _FakeConsole()
    assert apply_laser_power(console) is True
    writes = _rate_ll_writes(console)
    assert writes[6] == (70313).to_bytes(4, "little")
    assert writes[7] == (70313).to_bytes(4, "little")


def test_apply_laser_power_60hz_scales_user_config_rate_ll_override():
    """A stored per-key RATE_LL user-config override (us, calibrated for
    40 Hz) must be rescaled like the bundled baseline — written verbatim
    at 60 Hz it would exceed the pulse period and trip the interlock on
    every pulse (sdk#129 review finding)."""
    class _FakeConsoleWithCfg(_FakeConsole):
        def read_config(self):
            class _Cfg:
                json_data = {"EE_RATE_LL": 22500.0}
            return _Cfg()

    console = _FakeConsoleWithCfg()
    assert apply_laser_power(console, trigger_freq_hz=60.0) is True
    writes = _rate_ll_writes(console)
    # override 22,500 us / 0.32 = 70312.5 raw ticks, x 40/60 = 46875.
    assert writes[6] == (46875).to_bytes(4, "little")
    # OPT side has no override — bundled baseline scaling still applies.
    assert writes[7] == (46875).to_bytes(4, "little")


def test_trigger_overrides_for_rate_rejects_unsupported_rate():
    from omotion.config import trigger_overrides_for_rate
    import pytest

    with pytest.raises(ValueError):
        trigger_overrides_for_rate(0)
    with pytest.raises(ValueError):
        trigger_overrides_for_rate(100)


def test_trigger_overrides_for_rate_scales_skip_delay():
    from omotion.config import trigger_overrides_for_rate

    o60 = trigger_overrides_for_rate(60)
    assert o60["TriggerFrequencyHz"] == 60
    # 1800 us x 40/60 = 1200 us: post-dark interval 16667-1200 = 15467 us
    # stays above the scaled 15000 us RATE_LL floor.
    assert o60["LaserPulseSkipDelayUsec"] == 1200
    o40 = trigger_overrides_for_rate(40)
    assert o40["LaserPulseSkipDelayUsec"] == 1800


def test_trigger_overrides_for_rate_moves_pulse_delay_with_vts():
    from omotion.config import trigger_overrides_for_rate

    # Band at 60 Hz sits (2768-1845) rows x 9.0318 us later -> pulse at
    # 8436 us (bench-swept). Known #68 limitation: this is inside the SPI
    # push window; see trigger_overrides_for_rate docstring.
    assert trigger_overrides_for_rate(60)["LaserPulseDelayUsec"] == 8436
    assert trigger_overrides_for_rate(40)["LaserPulseDelayUsec"] == 100


def test_trigger_overrides_for_rate_scales_pulse_width_for_60825():
    from omotion.config import trigger_overrides_for_rate

    # IEC 60825 (AEL calculator sheet): hold duty at the 40 Hz-validated
    # 2.0% -> gate width scales with the period. 500 us x 40/60 = 333 us;
    # 60 Hz x 333.33 us = 2.0% duty, so every average-power AEL row is
    # unchanged and per-pulse/pulse-train margins improve.
    assert trigger_overrides_for_rate(60)["LaserPulseWidthUsec"] == 333
    assert trigger_overrides_for_rate(40)["LaserPulseWidthUsec"] == 500


def test_apply_laser_power_60hz_scales_pulse_width_ul():
    console = _FakeConsole()
    assert apply_laser_power(console, trigger_freq_hz=60.0) is True
    # PULSE_WIDTH_UL lives at 0x41 offset 0x0C? No: RATE regs at 0x08/0x0C
    # are RATE_LL/UL; PULSE_WIDTH_LL/UL are the entries preceding them in
    # laser_params.json. Baseline UL raw 3125 x 0.32 us = 1000 us; at
    # 60 Hz the ceiling scales to 2083 raw = 666.7 us so the interlock
    # ENFORCES the shortened 60825-compliant gate.
    ul_writes = [
        data for (mux, ch, dev, reg, data) in console.writes
        if dev == 0x41 and reg == 0x04 and ch in (6, 7)
    ]
    assert ul_writes, "expected PULSE_WIDTH_UL writes"
    for data in ul_writes:
        assert data == (2083).to_bytes(4, "little")


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
