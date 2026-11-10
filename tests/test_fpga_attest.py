"""Console FPGA attestation and the firmware-owned safety state
(console-fw PTR-2026-1-5 change): boot-info v2, the OW_CMD_FPGA_ATTEST
report, config-write refusal codes, and apply_laser_power on firmware that
owns the safety limits."""

import struct
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

import omotion.laser as laser
from omotion.boot_mode import BootInfo, BootMode, parse_boot_info_details
from omotion.config import (
    OW_CMD_FPGA_ATTEST,
    OW_CODE_BUSY,
    OW_CODE_CFG_SAFETY_KEY,
    OW_ERROR,
    OW_RESP,
    TRIG_INTERLOCK_FPGA_ATTEST,
    TRIG_INTERLOCK_LIMITS,
    describe_trigger_refusal,
)
from omotion.fpga_attest import (
    ATTEST_FAIL,
    ATTEST_PASS,
    ATTEST_RUNNING,
    CONFIG_SAFETY_KEYS,
    FIRMWARE_OWNED_REGISTERS,
    HOST_LASER_REGISTERS,
    parse_attest_report,
)
from omotion.laser import FpgaMap, apply_laser_power, load_laser_params
from omotion.MotionConfig import MotionConfig
from omotion.MotionConsole import MotionConsole

SLOT_VTOR = 0x08020400


def _boot_info_v1(vtor=SLOT_VTOR):
    return struct.pack("<B3sII", 1, b"\x00" * 3, vtor, vtor)


def _boot_info_v2(interlocks=0, attest=ATTEST_PASS, build=0, vtor=SLOT_VTOR):
    return struct.pack("<B3sIIBBBB", 2, b"\x00" * 3, vtor, vtor, interlocks, attest, build, 0)


def _device(device, status, ufm_status=1, digest=b"\x11" * 32, expected=b"\x11" * 32):
    return struct.pack(
        "<BBBBHHI32s32s", device, status, ufm_status, 0xFF, 3198, 3100, 0x012BB043,
        digest, expected,
    )


def _report(state=ATTEST_PASS, devices=None, interlocked=0):
    devices = devices or [
        _device(0, 0x40, 0xFF), _device(1, 0x40, 0xFF), _device(2, 0x10), _device(3, 0x10),
    ]
    return struct.pack("<BBBBHH", 2, state, 100, interlocked, 1, 3198) + b"".join(devices)


# --------------------------------------------------------------- boot info v2


def test_boot_info_v2_decodes_the_safety_fields():
    info = parse_boot_info_details(_boot_info_v2(interlocks=0x40, attest=ATTEST_RUNNING, build=1))
    assert info == BootInfo(2, SLOT_VTOR, SLOT_VTOR, 0x40, ATTEST_RUNNING, 1)
    assert info.boot_mode is BootMode.BOOTLOADER
    assert info.firmware_owns_safety_state
    assert info.factory_image is True


def test_boot_info_v1_has_no_safety_fields_and_owns_nothing():
    info = parse_boot_info_details(_boot_info_v1(0x08000000))
    assert info.boot_mode is BootMode.BARE_METAL
    assert info.interlocks is None and info.fpga_attest is None
    assert not info.firmware_owns_safety_state
    assert info.factory_image is None


def test_boot_info_v2_header_without_the_tail_reads_as_v1_fields():
    payload = _boot_info_v2()[:12]
    info = parse_boot_info_details(payload)
    assert info.struct_version == 2 and info.build is None
    assert not info.firmware_owns_safety_state


@pytest.mark.parametrize("payload", [b"", None, b"\x02\x00"])
def test_boot_info_short_reply_is_none(payload):
    assert parse_boot_info_details(payload) is None


# ------------------------------------------------------------ attest report


def test_attest_report_parses_and_names_the_faulted_device():
    rep = parse_attest_report(
        _report(
            state=ATTEST_FAIL,
            interlocked=1,
            devices=[
                _device(0, 0x40, 0xFF), _device(1, 0x40, 0xFF),
                _device(2, 0x11, digest=b"\x22" * 32), _device(3, 0x10),
            ],
        )
    )
    assert rep.state_name == "fail" and not rep.passed and rep.done
    assert rep.interlocked and rep.cfg_pages == 3198
    assert [d.name for d in rep.devices] == ["seed", "TA", "EE", "OPT"]
    assert not rep.devices[0].checked
    assert rep.faults() == ["EE: configuration digest mismatch, safety record valid"]


def test_attest_report_names_an_invalid_record_reason():
    rep = parse_attest_report(
        _report(state=ATTEST_FAIL, devices=[
            _device(0, 0x40, 0xFF), _device(1, 0x40, 0xFF), _device(2, 0x10), _device(3, 0x04, 7),
        ])
    )
    assert rep.faults() == ["OPT: safety record invalid, outside the safety envelope"]


def test_attest_report_too_short_is_none():
    assert parse_attest_report(_report()[:-1]) is None
    assert parse_attest_report(b"") is None


def test_owned_registers_and_host_registers_do_not_overlap():
    assert not HOST_LASER_REGISTERS & FIRMWARE_OWNED_REGISTERS
    assert FIRMWARE_OWNED_REGISTERS <= CONFIG_SAFETY_KEYS
    baseline = {p["friendlyName"] for p in load_laser_params()}
    # every baseline register is classified one way or the other
    assert baseline <= HOST_LASER_REGISTERS | FIRMWARE_OWNED_REGISTERS


def test_trigger_refusal_names_the_attestation_interlock():
    assert "attest" in describe_trigger_refusal(TRIG_INTERLOCK_FPGA_ATTEST)
    assert "unknown" not in describe_trigger_refusal(TRIG_INTERLOCK_FPGA_ATTEST)


# ---------------------------------------------------------- MotionConsole


def _make_console(payload=b"", packet_type=OW_RESP, *, connected=True, demo=False):
    console = MotionConsole.__new__(MotionConsole)
    console.uart = MagicMock()
    console.uart.demo_mode = demo
    console.is_connected = MagicMock(return_value=connected)
    resp = MagicMock()
    resp.packetType = packet_type
    resp.data = bytes(payload)
    resp.data_len = len(payload)
    console.uart.send_packet.return_value = resp
    console.uart.clear_buffer = MagicMock()
    return console


def test_console_get_boot_info_and_ownership():
    console = _make_console(_boot_info_v2(build=0))
    info = console.get_boot_info()
    assert info.factory_image is False
    assert console.firmware_owns_safety_state()


def test_console_get_boot_info_old_firmware():
    assert _make_console(b"", OW_ERROR).get_boot_info() is None
    assert not _make_console(_boot_info_v1()).firmware_owns_safety_state()
    assert _make_console(_boot_info_v2(), connected=False).get_boot_info() is None


def test_console_get_fpga_attestation_report_and_start():
    console = _make_console(_report())
    assert console.get_fpga_attestation().passed
    console.get_fpga_attestation(start=True)
    kwargs = console.uart.send_packet.call_args.kwargs
    assert kwargs["command"] == OW_CMD_FPGA_ATTEST and kwargs["reserved"] == 1


def test_console_get_fpga_attestation_busy_is_none():
    assert _make_console(bytes([OW_CODE_BUSY]), OW_ERROR).get_fpga_attestation(start=True) is None


def test_console_write_config_records_the_refusal_reason():
    console = _make_console(bytes([OW_CODE_CFG_SAFETY_KEY]), OW_ERROR)
    assert console.write_config(MotionConfig(json_data={"EE_DRIVE_CL": 4000})) is None
    assert console.last_config_refusal == OW_CODE_CFG_SAFETY_KEY
    assert "firmware-owned safety parameter" in console.last_config_refusal_reason


# ------------------------------------------ apply_laser_power, owned firmware


class _OwnedConsole:
    """Fake console with boot-info v2: records writes, answers boot info from
    a queue (the last entry repeats) and an attestation report."""

    def __init__(self, boot_infos, report=None, cfg=None, write_ok=True):
        self.writes = []
        self.reads = []
        self._infos = list(boot_infos)
        self._report = report
        self._cfg = cfg
        self._write_ok = write_ok
        self.laser_limits_error = None

    def get_boot_info(self):
        info = self._infos[0]
        if len(self._infos) > 1:
            self._infos.pop(0)
        return info

    def get_fpga_attestation(self, start=False):
        return self._report

    def read_config(self):
        return SimpleNamespace(json_data=self._cfg) if self._cfg is not None else None

    def write_i2c_packet(self, *, mux_index, channel, device_addr, reg_addr, data):
        self.writes.append((channel, reg_addr, bytes(data)))
        return self._write_ok

    def read_i2c_packet(self, **kwargs):
        self.reads.append(kwargs)
        return None, None


def _info(interlocks=0, attest=ATTEST_PASS):
    return parse_boot_info_details(_boot_info_v2(interlocks=interlocks, attest=attest))


def _host_locations():
    m = FpgaMap()
    return {
        (m.get_entry_by_friendly_name(n)["channel"], m.get_entry_by_friendly_name(n)["start_address"])
        for n in HOST_LASER_REGISTERS
    }


def test_owned_load_writes_only_the_operating_point_and_passes():
    console = _OwnedConsole([_info()], cfg={"EE_THRESH": 3000, "TA_PULSE_WIDTH": 400})
    assert apply_laser_power(console) is True
    assert console.laser_limits_error is None
    assert {(ch, off) for ch, off, _ in console.writes} == _host_locations()
    assert console.reads == []  # verified through the console, not by read-back
    ta_pw = FpgaMap().get_entry_by_friendly_name("TA_PULSE_WIDTH")
    written = {(ch, off): data for ch, off, data in console.writes}
    assert written[(ta_pw["channel"], ta_pw["start_address"])] == round(400 / ta_pw["scale"]).to_bytes(3, "little")


def test_owned_load_waits_for_the_boot_attestation(monkeypatch):
    monkeypatch.setattr(laser.time, "sleep", lambda s: None)
    console = _OwnedConsole([_info(0x40, ATTEST_RUNNING)] * 3 + [_info()])
    assert apply_laser_power(console) is True


def test_owned_load_fails_on_attestation_failure_with_the_fault():
    report = parse_attest_report(_report(state=ATTEST_FAIL, devices=[
        _device(0, 0x40, 0xFF), _device(1, 0x40, 0xFF), _device(2, 0x11), _device(3, 0x10),
    ]))
    console = _OwnedConsole([_info(0x40, ATTEST_FAIL)], report=report)
    assert apply_laser_power(console) is False
    assert "attestation failed" in console.laser_limits_error
    assert "EE: configuration digest mismatch" in console.laser_limits_error


def test_owned_load_times_out_while_attestation_runs(monkeypatch):
    clock = iter(range(0, 1000, 5))
    monkeypatch.setattr(laser.time, "monotonic", lambda: next(clock))
    monkeypatch.setattr(laser.time, "sleep", lambda s: None)
    console = _OwnedConsole([_info(0x40, ATTEST_RUNNING)])
    assert apply_laser_power(console, attest_timeout_s=15) is False
    assert "did not finish" in console.laser_limits_error


@pytest.mark.parametrize("bit", [TRIG_INTERLOCK_LIMITS, TRIG_INTERLOCK_FPGA_ATTEST])
def test_owned_load_fails_while_a_limits_or_attest_interlock_is_set(bit):
    console = _OwnedConsole([_info(bit, ATTEST_PASS)])
    assert apply_laser_power(console) is False
    assert console.laser_limits_error


def test_owned_load_names_a_refused_operating_point_write():
    console = _OwnedConsole([_info()], write_ok=False)
    assert apply_laser_power(console) is False
    assert "write was not accepted" in console.laser_limits_error


def test_owned_load_refuses_force_fault_without_writing():
    console = _OwnedConsole([_info()])
    assert apply_laser_power(console, force_fault=True) is False
    assert "force_fault" in console.laser_limits_error
    assert console.writes == []


def test_v1_console_keeps_the_legacy_load():
    console = _OwnedConsole([parse_boot_info_details(_boot_info_v1())])
    assert apply_laser_power(console) is False  # legacy read-back: the fake reads nothing
    assert len(console.writes) == len(load_laser_params())
    assert console.reads  # legacy path reads the safety limits back


# ------------------------------------------------ calibration bench guard


@pytest.mark.parametrize(
    "payload, expected",
    [
        (_boot_info_v2(build=0), False),
        (_boot_info_v2(build=1), True),
        (_boot_info_v1(), None),
    ],
)
def test_bench_reports_whether_the_console_accepts_safety_keys(payload, expected):
    from omotion.calibration.motion_bench import MotionConsoleBenchBase

    bench = MotionConsoleBenchBase.__new__(MotionConsoleBenchBase)
    bench._console = SimpleNamespace(get_boot_info=lambda: parse_boot_info_details(payload))
    assert bench._console_safety_keys_writable() is expected
