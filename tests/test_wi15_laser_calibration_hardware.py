import math
import statistics

import pytest

from omotion.MotionConfig import MotionConfig
from omotion.calibration.laser import (
    EnergyMeasurement,
    OphirIdentity,
    SettingReadback,
    validate_energy_measurement,
)
from omotion.calibration.laser_hardware import (
    FpgaRegisterIO,
    MotionLaserCalibrationBench,
    OphirEnergyMeter,
)
from omotion.calibration.single_sensor_laser import (
    OphirEvidenceApplicability,
    OphirSettingEvidence,
)
from wi15_builders import valid_ophir_setting_evidence
from wi15_fakes import FakeClock, FakeDevice


def _ophir_preflight():
    identity = OphirIdentity(
        "Centauri",
        "meter-1",
        "PE10BF-C",
        "sensor-1",
        "meter: 2027-10-16; sensor: 2027-08-22",
    )
    return identity, valid_ophir_setting_evidence()


class FakeConsole(FakeDevice):
    def __init__(self, connected=True, calls=None):
        super().__init__(connected, "console-serial", "console-fw", "console-hwid")
        self.calls = calls if calls is not None else []
        self.config_reads = [MotionConfig(json_data={"before": 1})]
        self.write_result = MotionConfig(json_data={"written": 1})
        self.trigger_reads = [{"TriggerFrequencyHz": 20.0}]
        self.trigger_set_result = {"ok": True}
        self.i2c_write_result = True
        self.i2c_read_result = (b"\x12\x7a", 2)
        self.fpga_version_raw = {
            address: value
            for address, value in zip(
                range(101, 113),
                (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12),
                strict=True,
            )
        }
        self.start_trigger_result = True
        self.stop_trigger_result = True

    def echo(self, data):
        return data, len(data)

    def read_config(self):
        self.calls.append("read_config")
        return self.config_reads.pop(0)

    def write_config(self, config):
        self.calls.append(("write_config", dict(config.json_data)))
        return self.write_result

    def get_trigger_json(self):
        self.calls.append("get_trigger_json")
        if len(self.trigger_reads) > 1:
            return self.trigger_reads.pop(0)
        return self.trigger_reads[0]

    def set_trigger_json(self, config):
        self.calls.append(("set_trigger_json", dict(config)))
        return self.trigger_set_result

    def read_i2c_packet(self, **kwargs):
        self.calls.append(("read_i2c_packet", kwargs))
        if kwargs["reg_addr"] in self.fpga_version_raw:
            width = kwargs["read_len"]
            raw = self.fpga_version_raw[kwargs["reg_addr"]]
            return raw.to_bytes(width, "little"), width
        return self.i2c_read_result

    def write_i2c_packet(self, **kwargs):
        self.calls.append(("write_i2c_packet", kwargs))
        return self.i2c_write_result

    def start_trigger(self):
        self.calls.append("start_trigger")
        return self.start_trigger_result

    def stop_trigger(self):
        self.calls.append("stop_trigger")
        return self.stop_trigger_result


class FakeInterface:
    def __init__(self, topology=(True, True, False), calls=None):
        self.calls = calls if calls is not None else []
        self.console = FakeConsole(topology[0], self.calls)
        self.left = FakeDevice(topology[1], "left-serial", "left-fw", "left-hwid")
        self.right = FakeDevice(topology[2], "right-serial", "right-fw", "right-hwid")

    def start(self, **kwargs):
        self.calls.append(("interface.start", kwargs))

    def wait_for_ready(self, **kwargs):
        self.calls.append(("interface.wait_for_ready", kwargs))
        return True

    def stop(self):
        self.calls.append("interface.stop")

    def apply_laser_power(self):
        self.calls.append("apply_laser_power")
        return True


class FakeMeter:
    def __init__(self, calls=None):
        self.calls = calls if calls is not None else []

    def preflight(self):
        self.calls.append("meter.preflight")
        return _ophir_preflight()

    def close(self):
        self.calls.append("meter.close")

    def measure(self):
        self.calls.append("meter.measure")
        return EnergyMeasurement(26, 0, 350.0, 10.0, 40.0, 330.0, 370.0, 0.65)


class FakeMap:
    _VERSION_ADDRESSES = {
        name: address
        for address, name in zip(
            range(101, 113),
            (
                "TA_MAJOR",
                "TA_MINOR",
                "TA_REVISION",
                "SEED_MAJOR",
                "SEED_MINOR",
                "SEED_REVISION",
                "EE_MAJOR",
                "EE_MINOR",
                "EE_REVISION",
                "OPT_MAJOR",
                "OPT_MINOR",
                "OPT_REVISION",
            ),
            strict=True,
        )
    }

    def get_entry_by_friendly_name(self, name):
        if name in self._VERSION_ADDRESSES:
            return {
                "mux_idx": 1,
                "channel": 4,
                "i2c_addr": 0x41,
                "isMsbFirst": False,
                "start_address": self._VERSION_ADDRESSES[name],
                "data_size": "8B",
            }
        if name == "TA_CURRENT_DRV":
            return {
                "mux_idx": 1,
                "channel": 4,
                "i2c_addr": 0x41,
                "isMsbFirst": False,
                "start_address": 6,
                "data_size": "16B",
                "scale": 0.16,
            }
        return None


def _bench(topology=(True, True, False), *, meter=None, fpga_map=None, **bench_options):
    interface = FakeInterface(topology)
    meter = meter or FakeMeter(interface.calls)
    bench = MotionLaserCalibrationBench(
        meter,
        interface_factory=lambda: interface,
        fpga_map=fpga_map or FakeMap(),
        wait_timeout=3.5,
        topology_quiet_period_s=0.0,
        **bench_options,
    )
    return bench, interface, meter


@pytest.mark.parametrize(
    ("topology", "expected"),
    [
        ((True, True, False), (True, True, False)),
        ((True, True, True), (True, True, True)),
        ((True, False, False), (True, False, False)),
        ((True, False, True), (True, False, True)),
    ],
)
def test_preflight_waits_for_console_and_one_sensor_then_reports_exact_topology(
    topology, expected
):
    bench, interface, _ = _bench(topology)

    snapshot = bench.preflight("left")

    assert (
        snapshot.topology.console_connected,
        snapshot.topology.left_connected,
        snapshot.topology.right_connected,
    ) == expected
    assert interface.calls[:2] == [
        ("interface.start", {"wait": False}),
        (
            "interface.wait_for_ready",
            {"console": True, "sensors": 1, "timeout": 3.5},
        ),
    ]


def test_dual_preflight_waits_for_two_sensors_and_reports_both_identities():
    """Waiting for only one module would authorize an incomplete shipping topology."""
    bench, interface, _ = _bench((True, True, True))

    snapshot = bench.preflight_dual()

    assert interface.calls[:2] == [
        ("interface.start", {"wait": False}),
        (
            "interface.wait_for_ready",
            {"console": True, "sensors": 2, "timeout": 3.5},
        ),
    ]
    assert snapshot.topology.left_connected is True
    assert snapshot.topology.right_connected is True
    assert snapshot.left_sensor_identity.serial == "left-serial"
    assert snapshot.right_sensor_identity.serial == "right-serial"
    assert [
        (revision.controller, revision.version)
        for revision in snapshot.console_identity.fpga_firmware_revisions
    ] == [
        ("TA", "1.2.3"),
        ("SEED", "4.5.6"),
        ("SAFETY_EE", "7.8.9"),
        ("SAFETY_OPT", "10.11.12"),
    ]
    assert snapshot.ophir_ready is True


def test_preflight_rejects_incomplete_console_fpga_identity_before_mutation_or_firing():
    bench, interface, _ = _bench((True, True, True))
    del interface.console.fpga_version_raw[112]

    with pytest.raises(RuntimeError, match="OPT_REVISION"):
        bench.preflight_dual()

    assert "apply_laser_power" not in interface.calls
    assert "start_trigger" not in interface.calls
    assert not any(
        isinstance(call, tuple) and call[0] in ("write_config", "write_i2c_packet")
        for call in interface.calls
    )


def test_dual_topology_revalidation_returns_the_current_quiet_snapshot():
    bench, interface, _ = _bench((True, True, True))
    bench.preflight_dual()
    interface.right.connected = False

    snapshot = bench.revalidate_dual_topology()

    assert snapshot.left_connected is True
    assert snapshot.right_connected is False


def test_preflight_captures_quiet_topology_after_late_sensor_arrives_during_ophir_setup():
    """Sampling before Ophir setup would miss a deterministic late opposite sensor."""
    interface = FakeInterface((True, True, False))
    clock = FakeClock()

    class LateOppositeMeter(FakeMeter):
        def preflight(self):
            self.calls.append("meter.preflight")
            interface.right.connected = True
            return _ophir_preflight()

    meter = LateOppositeMeter(interface.calls)
    bench = MotionLaserCalibrationBench(
        meter,
        interface_factory=lambda: interface,
        fpga_map=FakeMap(),
        wait_timeout=3.5,
        topology_quiet_period_s=0.2,
        topology_poll_interval_s=0.05,
        clock=clock,
        sleep=clock.sleep,
    )

    snapshot = bench.preflight("left")

    assert snapshot.topology.right_connected is True
    assert clock.now == pytest.approx(0.2)
    assert interface.calls.index("meter.preflight") < len(interface.calls)


def test_preflight_keeps_valid_serials_when_an_independent_identity_read_fails():
    bench, interface, _ = _bench()
    interface.console.firmware_error = RuntimeError("firmware read exploded")

    snapshot = bench.preflight("left")

    assert snapshot.console_identity.serial == "console-serial"
    assert snapshot.console_identity.firmware is None
    assert snapshot.console_identity.hardware_id == "console-hwid"
    assert snapshot.selected_sensor_identity.serial == "left-serial"
    assert "firmware read exploded" not in snapshot.console_identity.serial


def test_close_stops_trigger_before_meter_and_motion_interface_even_when_trigger_stop_raises():
    bench, interface, _ = _bench()
    interface.console.stop_trigger = lambda: (_ for _ in ()).throw(
        RuntimeError("stop failed")
    )

    with pytest.raises(RuntimeError, match="stop failed"):
        bench.close()

    assert interface.calls[-2:] == ["meter.close", "interface.stop"]


def test_configuration_write_requires_motion_config_result_and_fresh_complete_readback():
    bench, interface, _ = _bench()
    interface.console.config_reads = [MotionConfig(json_data={"fresh": 2, "all": 3})]

    result = bench.write_user_configuration({"fresh": 2, "all": 3})

    assert result == {"fresh": 2, "all": 3}
    assert interface.calls == [
        ("write_config", {"fresh": 2, "all": 3}),
        "read_config",
    ]


@pytest.mark.parametrize("write_result", [None, {"not": "a MotionConfig"}])
def test_configuration_write_rejects_non_motion_config_result_without_claiming_readback(
    write_result,
):
    bench, interface, _ = _bench()
    interface.console.write_result = write_result

    assert bench.write_user_configuration({"fresh": 2}) is None
    assert interface.calls == [("write_config", {"fresh": 2})]


def test_trigger_write_requires_successful_response_and_verified_fresh_readback():
    bench, interface, _ = _bench()
    interface.console.trigger_reads = [
        {"TriggerFrequencyHz": 20.0, "TriggerStatus": 2},
        {"TriggerFrequencyHz": 40.0, "TriggerStatus": 2},
    ]

    assert bench.write_trigger_rate_hz(40.0) == SettingReadback(
        "trigger_rate_hz_write", 40.0, 40.0
    )
    assert interface.calls == [
        "get_trigger_json",
        ("set_trigger_json", {"TriggerFrequencyHz": 40.0, "TriggerStatus": 2}),
        "get_trigger_json",
    ]


def test_trigger_write_fails_closed_on_bad_response():
    bench, interface, _ = _bench()
    interface.console.trigger_set_result = None
    interface.console.trigger_reads = [
        {"TriggerFrequencyHz": 20.0},
        {"TriggerFrequencyHz": 40.0},
    ]

    assert bench.write_trigger_rate_hz(40.0) is None


def test_trigger_write_preserves_immediate_mismatching_readback_evidence():
    bench, interface, _ = _bench()
    interface.console.trigger_reads = [
        {"TriggerFrequencyHz": 20.0},
        {"TriggerFrequencyHz": 39.0},
    ]

    assert bench.write_trigger_rate_hz(40.0) == SettingReadback(
        "trigger_rate_hz_write", 40.0, 39.0
    )


@pytest.mark.parametrize(
    "nonfinite", [float("nan"), float("inf"), float("-inf")], ids=["nan", "inf", "-inf"]
)
def test_trigger_write_preserves_numeric_nonfinite_immediate_readback(nonfinite):
    bench, interface, _ = _bench()
    interface.console.trigger_reads = [
        {"TriggerFrequencyHz": 20.0},
        {"TriggerFrequencyHz": nonfinite},
    ]

    result = bench.write_trigger_rate_hz(40.0)

    assert result is not None
    assert result.name == "trigger_rate_hz_write"
    assert result.requested == 40.0
    assert math.isfinite(result.actual) is False


@pytest.mark.parametrize(
    "readback", [{}, {"TriggerFrequencyHz": "not numeric"}], ids=["missing", "unparseable"]
)
def test_trigger_write_returns_none_for_unavailable_or_unparseable_readback(readback):
    bench, interface, _ = _bench()
    interface.console.trigger_reads = [
        {"TriggerFrequencyHz": 20.0},
        readback,
    ]

    assert bench.write_trigger_rate_hz(40.0) is None


def test_fpga_write_requires_truthy_i2c_result_and_returns_immediate_scaled_readback():
    console = FakeConsole()
    register_io = FpgaRegisterIO(console, fpga_map=FakeMap())

    result = register_io.write("TA_CURRENT_DRV", 5000.0)

    assert result.name == "TA_CURRENT_DRV"
    assert result.requested == 5000.0
    assert result.actual == 5000.0
    assert console.calls == [
        (
            "write_i2c_packet",
            {
                "mux_index": 1,
                "channel": 4,
                "device_addr": 0x41,
                "reg_addr": 6,
                "data": b"\x12\x7a",
            },
        ),
        (
            "read_i2c_packet",
            {
                "mux_index": 1,
                "channel": 4,
                "device_addr": 0x41,
                "reg_addr": 6,
                "read_len": 2,
            },
        ),
    ]


def test_fpga_write_does_not_read_or_return_evidence_after_failed_i2c_write():
    console = FakeConsole()
    console.i2c_write_result = False
    register_io = FpgaRegisterIO(console, fpga_map=FakeMap())

    assert register_io.write("TA_CURRENT_DRV", 5000.0) is None
    assert [call[0] for call in console.calls] == ["write_i2c_packet"]


class FakeOphirCOM:
    def __init__(self):
        self.calls = []
        self.serials = ["3199176"]
        self.handle = 17
        self.sensor_exists = True
        self.device_info = ("Centauri", "CE5.02", "3199176")
        self.sensor_info = ("3200878", "Pyroelectric", "PE10BF-C")
        self.device_due = "2027-10-16"
        self.sensor_due = "2027-08-22"
        self.settings = {
            "MeasurementMode": [0, ["Power", "Energy", "Exposure"]],
            "Ranges": [0, ["10.00mJ", "2.00mJ", "200uJ", "20.0uJ"]],
            "Wavelengths": [0, ["193", "248", "355", "795", "1064", "2940"]],
            "PulseLengths": [1, ["1.0ms", "5.0ms"]],
            "Threshold": [2, ["Min", "2%", "5%", "10%"]],
        }
        self.ignore_set = set()
        self.data_batches = [([], [], [])]
        self.data_error = None

    def ScanUSB(self):
        self.calls.append("ScanUSB")
        return list(self.serials)

    def OpenUSBDevice(self, serial):
        self.calls.append(("OpenUSBDevice", serial))
        return self.handle

    def IsSensorExists(self, handle, channel):
        self.calls.append(("IsSensorExists", handle, channel))
        return self.sensor_exists

    def GetDeviceInfo(self, handle):
        self.calls.append(("GetDeviceInfo", handle))
        return self.device_info

    def GetSensorInfo(self, handle, channel):
        self.calls.append(("GetSensorInfo", handle, channel))
        return self.sensor_info

    def GetDeviceCalibrationDueDate(self, handle):
        self.calls.append(("GetDeviceCalibrationDueDate", handle))
        return self.device_due

    def GetSensorCalibrationDueDate(self, handle, channel):
        self.calls.append(("GetSensorCalibrationDueDate", handle, channel))
        return self.sensor_due

    def _get(self, name, handle, channel):
        self.calls.append((f"Get{name}", handle, channel))
        index, options = self.settings[name]
        return index, list(options)

    def _set(self, name, handle, channel, index):
        self.calls.append((f"Set{name.removesuffix('s')}", handle, channel, index))
        if name not in self.ignore_set:
            self.settings[name][0] = index

    def GetMeasurementMode(self, handle, channel):
        return self._get("MeasurementMode", handle, channel)

    def SetMeasurementMode(self, handle, channel, index):
        self.calls.append(("SetMeasurementMode", handle, channel, index))
        if "MeasurementMode" not in self.ignore_set:
            self.settings["MeasurementMode"][0] = index

    def GetRanges(self, handle, channel):
        return self._get("Ranges", handle, channel)

    def SetRange(self, handle, channel, index):
        return self._set("Ranges", handle, channel, index)

    def GetWavelengths(self, handle, channel):
        return self._get("Wavelengths", handle, channel)

    def SetWavelength(self, handle, channel, index):
        return self._set("Wavelengths", handle, channel, index)

    def GetPulseLengths(self, handle, channel):
        return self._get("PulseLengths", handle, channel)

    def SetPulseLength(self, handle, channel, index):
        return self._set("PulseLengths", handle, channel, index)

    def GetThreshold(self, handle, channel):
        return self._get("Threshold", handle, channel)

    def SetThreshold(self, handle, channel, index):
        self.calls.append(("SetThreshold", handle, channel, index))
        if "Threshold" not in self.ignore_set:
            self.settings["Threshold"][0] = index

    def StartStream(self, handle, channel):
        self.calls.append(("StartStream", handle, channel))

    def GetData(self, handle, channel):
        self.calls.append(("GetData", handle, channel))
        if self.data_error:
            raise self.data_error
        if len(self.data_batches) > 1:
            return self.data_batches.pop(0)
        return self.data_batches[0]

    def StopStream(self, handle, channel):
        self.calls.append(("StopStream", handle, channel))

    def StopAllStreams(self):
        self.calls.append("StopAllStreams")

    def Close(self, handle):
        self.calls.append(("Close", handle))

    def CloseAll(self):
        self.calls.append("CloseAll")


def _ophir_meter(com=None, *, duration_s=0.1):
    com = com or FakeOphirCOM()
    clock = FakeClock()
    meter = OphirEnergyMeter(
        com_factory=lambda: com,
        duration_s=duration_s,
        poll_interval_s=0.05,
        clock=clock,
        sleep=clock.sleep,
    )
    return meter, com, clock


def test_ophir_preflight_reports_com_construction_failure():
    def fail_factory():
        raise OSError("COM class unavailable")

    meter = OphirEnergyMeter(com_factory=fail_factory)

    with pytest.raises(
        RuntimeError, match="COM construction failed.*COM class unavailable"
    ):
        meter.preflight()


def test_ophir_preflight_rejects_empty_usb_scan():
    meter, com, _ = _ophir_meter()
    com.serials = []

    with pytest.raises(RuntimeError, match="No Ophir USB meters"):
        meter.preflight()


@pytest.mark.parametrize("handle", [None, 0, -1])
def test_ophir_preflight_rejects_invalid_open_handle(handle):
    meter, com, _ = _ophir_meter()
    com.handle = handle

    with pytest.raises(RuntimeError, match="valid handle"):
        meter.preflight()


def test_ophir_preflight_reports_open_exception():
    meter, com, _ = _ophir_meter()
    com.OpenUSBDevice = lambda serial: (_ for _ in ()).throw(OSError("open failed"))

    with pytest.raises(RuntimeError, match="open failed"):
        meter.preflight()


def test_ophir_preflight_requires_channel_zero_energy_sensor():
    meter, com, _ = _ophir_meter()
    com.sensor_exists = False

    with pytest.raises(RuntimeError, match="channel 0 energy sensor"):
        meter.preflight()


@pytest.mark.parametrize(
    ("attribute", "bad_value"),
    [
        ("device_info", ("Centauri", "CE5.02", "")),
        ("sensor_info", ("3200878", "Pyroelectric", None)),
        ("device_due", None),
        ("sensor_due", "  "),
    ],
)
def test_ophir_preflight_rejects_incomplete_identity_or_calibration_due(
    attribute, bad_value
):
    meter, com, _ = _ophir_meter()
    setattr(com, attribute, bad_value)

    with pytest.raises(RuntimeError, match="identity and calibration due"):
        meter.preflight()


def test_ophir_preflight_writes_and_reads_back_all_five_com_settings():
    meter, com, _ = _ophir_meter()

    identity, evidence = meter.preflight()

    assert identity == OphirIdentity(
        "Centauri",
        "3199176",
        "PE10BF-C",
        "3200878",
        "meter: 2027-10-16; sensor: 2027-08-22",
    )
    assert evidence == _ophir_preflight()[1]
    assert [
        call
        for call in com.calls
        if isinstance(call, tuple) and call[0].startswith("Set")
    ] == [
        ("SetMeasurementMode", 17, 0, 1),
        ("SetRange", 17, 0, 1),
        ("SetWavelength", 17, 0, 3),
        ("SetPulseLength", 17, 0, 0),
        ("SetThreshold", 17, 0, 0),
    ]
    for getter in (
        "GetMeasurementMode",
        "GetRanges",
        "GetWavelengths",
        "GetPulseLengths",
        "GetThreshold",
    ):
        assert (
            sum(call[0] == getter for call in com.calls if isinstance(call, tuple)) == 2
        )


def test_ophir_preflight_marks_display_only_settings_explicitly_not_applicable():
    meter, _, _ = _ophir_meter()

    _, evidence = meter.preflight()

    assert evidence[-2:] == (
        OphirSettingEvidence(
            "display_averaging_s",
            3,
            None,
            OphirEvidenceApplicability.NOT_APPLICABLE,
            True,
        ),
        OphirSettingEvidence(
            "graph_mode",
            "Statistics",
            None,
            OphirEvidenceApplicability.NOT_APPLICABLE,
            True,
        ),
    )


@pytest.mark.parametrize(
    "setting",
    ["MeasurementMode", "Ranges", "Wavelengths", "PulseLengths", "Threshold"],
)
def test_ophir_preflight_fails_when_any_setting_does_not_read_back(setting):
    meter, com, _ = _ophir_meter()
    com.ignore_set.add(setting)

    with pytest.raises(RuntimeError, match="readback mismatch"):
        meter.preflight()


@pytest.mark.parametrize(
    "failure_kind",
    ["missing_sensor", "incomplete_identity", "setting_mismatch"],
)
def test_failed_post_open_preflight_closes_handle_and_cannot_measure(failure_kind):
    meter, com, _ = _ophir_meter()
    if failure_kind == "missing_sensor":
        com.sensor_exists = False
    elif failure_kind == "incomplete_identity":
        com.device_due = None
    else:
        com.ignore_set.add("Ranges")

    with pytest.raises(RuntimeError):
        meter.preflight()

    assert ("Close", 17) in com.calls
    assert meter._preflight_passed is False
    assert meter._handle is None
    com.calls.clear()
    with pytest.raises(RuntimeError, match="preflight must pass"):
        meter.measure()
    assert not any(
        isinstance(call, tuple) and call[0] == "StartStream" for call in com.calls
    )


def test_failed_repreflight_cannot_inherit_prior_success_or_measure():
    meter, com, _ = _ophir_meter()
    meter.preflight()
    com.sensor_exists = False
    com.calls.clear()

    with pytest.raises(RuntimeError, match="channel 0 energy sensor"):
        meter.preflight()

    assert ("Close", 17) in com.calls
    assert meter._preflight_passed is False
    assert meter._handle is None
    com.calls.clear()
    with pytest.raises(RuntimeError, match="preflight must pass"):
        meter.measure()
    assert not any(
        isinstance(call, tuple) and call[0] == "StartStream" for call in com.calls
    )


def test_failed_post_open_preflight_preserves_primary_error_when_close_raises():
    meter, com, _ = _ophir_meter()
    com.sensor_exists = False

    def fail_close(handle):
        com.calls.append(("Close", handle))
        raise OSError("close also failed")

    com.Close = fail_close

    with pytest.raises(RuntimeError, match="channel 0 energy sensor"):
        meter.preflight()

    assert meter._preflight_passed is False
    assert meter._handle is None
    with pytest.raises(RuntimeError, match="preflight must pass"):
        meter.measure()


@pytest.mark.parametrize(
    ("setting", "options"),
    [
        ("Ranges", ["10.0mJ", "2", "200uJ"]),
        ("Ranges", ["10.0mJ", "2ms", "200uJ"]),
        ("Ranges", ["10.0mJ", "2.0mJ?", "200uJ"]),
        ("PulseLengths", ["1", "5.0ms"]),
        ("PulseLengths", ["1.0mJ", "5.0ms"]),
        ("PulseLengths", ["1.0ms?", "5.0ms"]),
    ],
)
def test_ophir_preflight_rejects_unitless_wrong_unit_and_malformed_range_or_pulse(
    setting, options
):
    meter, com, _ = _ophir_meter()
    com.settings[setting] = [0, options]

    with pytest.raises(RuntimeError, match="option is unavailable"):
        meter.preflight()


def test_ophir_measure_discards_nonzero_status_and_returns_direct_stream_statistics():
    meter, com, _ = _ophir_meter()
    meter.preflight()
    com.calls.clear()
    com.data_batches = [
        ([0.0001], [500.0], [0]),
        (
            [0.0003, 9.9, 0.0004, 8.8],
            [1000.0, 1010.0, 1025.0, 1030.0],
            [0, 1, 0, 2],
        ),
    ]

    measurement = meter.measure()

    assert measurement.n == 2
    assert measurement.discarded == 2
    assert measurement.mean_uj == pytest.approx(350.0)
    assert measurement.stdev_uj == pytest.approx(math.sqrt(5000.0))
    assert measurement.rate_hz == pytest.approx(40.0)
    assert measurement.min_uj == pytest.approx(300.0)
    assert measurement.max_uj == pytest.approx(400.0)
    assert measurement.duration_s == pytest.approx(0.1)
    assert com.calls[-1] == ("StopStream", 17, 0)


def test_ophir_measure_drains_stale_first_nonempty_batch_before_fresh_observation():
    meter, com, _ = _ophir_meter(duration_s=2.0)
    meter.preflight()
    com.calls.clear()
    stale_timestamps = [217707950.0, 217707974.999]
    fresh_timestamps = [217711184.0 + 25.0 * index for index in range(26)]
    fresh_values_uj = list(range(340, 366))
    com.data_batches = [
        ([], [], []),
        ([0.0001, 9.9], stale_timestamps, [0, 1]),
        (
            [value * 1e-6 for value in fresh_values_uj],
            fresh_timestamps,
            [0] * 26,
        ),
        pytest.fail,
    ]

    measurement = meter.measure()

    assert measurement.n == 26
    assert measurement.discarded == 0
    assert measurement.mean_uj == pytest.approx(statistics.fmean(fresh_values_uj))
    assert measurement.stdev_uj == pytest.approx(statistics.stdev(fresh_values_uj))
    assert measurement.min_uj == pytest.approx(340.0)
    assert measurement.max_uj == pytest.approx(365.0)
    assert measurement.rate_hz == pytest.approx(40.0)
    assert com.calls.count(("GetData", 17, 0)) == 3
    assert com.calls[-1] == ("StopStream", 17, 0)


def test_ophir_measure_restarts_after_stale_timestamp_prefix_in_fresh_batch():
    """A delayed buffered prefix must not corrupt an otherwise fresh 40 Hz batch."""
    meter, com, _ = _ophir_meter(duration_s=2.0)
    meter.preflight()
    com.calls.clear()
    fresh_values_uj = list(range(340, 367))
    com.data_batches = [
        ([0.0001], [500.0], [0]),
        (
            [0.0002] + [value * 1e-6 for value in fresh_values_uj],
            [1000.0] + [2045.0 + 25.0 * index for index in range(27)],
            [0] * 28,
        ),
        pytest.fail,
    ]

    measurement = meter.measure()

    assert measurement.n == 27
    assert measurement.discarded == 1
    assert measurement.mean_uj == pytest.approx(statistics.fmean(fresh_values_uj))
    assert measurement.stdev_uj == pytest.approx(statistics.stdev(fresh_values_uj))
    assert measurement.rate_hz == pytest.approx(40.0)
    assert measurement.min_uj == pytest.approx(340.0)
    assert measurement.max_uj == pytest.approx(366.0)
    assert com.calls.count(("GetData", 17, 0)) == 2
    assert com.calls[-1] == ("StopStream", 17, 0)


def test_ophir_measure_does_not_accept_stale_only_priming_batch_at_timeout():
    meter, com, _ = _ophir_meter(duration_s=0.05)
    meter.preflight()
    com.calls.clear()
    com.data_batches = [
        (
            [0.00035] * 30 + [9.9],
            [217707950.0 + 25.0 * index for index in range(31)],
            [0] * 30 + [1],
        ),
        pytest.fail,
    ]

    measurement = meter.measure()

    assert measurement.n == 0
    assert measurement.discarded == 0
    assert math.isnan(measurement.mean_uj)
    assert math.isnan(measurement.rate_hz)
    assert measurement.duration_s == pytest.approx(0.05)
    assert not all(item.passed for item in validate_energy_measurement(measurement))
    assert com.calls.count(("GetData", 17, 0)) == 1
    assert com.calls[-1] == ("StopStream", 17, 0)


def test_ophir_measure_continues_after_23_valid_plus_discarded_until_target():
    meter, com, _ = _ophir_meter(duration_s=2.0)
    meter.preflight()
    com.calls.clear()
    first_values_uj = list(range(300, 323))
    second_values_uj = list(range(323, 327))
    com.data_batches = [
        ([0.0001], [500.0], [0]),
        (
            [value * 1e-6 for value in first_values_uj] + [9.9],
            [1000.0 + 25.0 * index for index in range(23)] + [1560.0],
            [0] * 23 + [1],
        ),
        (
            [value * 1e-6 for value in second_values_uj],
            [1575.0 + 25.0 * index for index in range(4)],
            [0] * 4,
        ),
        ([], [], []),
    ]

    measurement = meter.measure()

    accepted_values_uj = first_values_uj + second_values_uj
    assert measurement.n == 27
    assert measurement.discarded == 1
    assert measurement.mean_uj == pytest.approx(statistics.fmean(accepted_values_uj))
    assert measurement.stdev_uj == pytest.approx(statistics.stdev(accepted_values_uj))
    assert measurement.min_uj == 300.0
    assert measurement.max_uj == 326.0
    assert measurement.rate_hz == pytest.approx(40.0)
    assert measurement.duration_s == pytest.approx(0.15)
    assert com.calls.count(("GetData", 17, 0)) == 3
    assert com.calls[-1] == ("StopStream", 17, 0)


def test_ophir_measure_stops_early_immediately_after_valid_target_is_met():
    meter, com, _ = _ophir_meter(duration_s=2.0)
    meter.preflight()
    com.calls.clear()
    com.data_batches = [
        ([0.0001], [500.0], [0]),
        (
            [0.00035] * 26,
            [1000.0 + 25.0 * index for index in range(26)],
            [0] * 26,
        ),
    ]

    measurement = meter.measure()

    assert measurement.n == 26
    assert measurement.duration_s == pytest.approx(0.1)
    assert com.calls.count(("GetData", 17, 0)) == 2
    assert com.calls[-1] == ("StopStream", 17, 0)


def test_ophir_measure_times_out_with_below_target_evidence_for_fail_closed_gate():
    meter, com, _ = _ophir_meter(duration_s=0.1)
    meter.preflight()
    com.calls.clear()
    com.data_batches = [
        ([0.0001], [500.0], [0]),
        (
            [0.00035] * 23 + [9.9],
            [1000.0 + 25.0 * index for index in range(24)],
            [0] * 23 + [1],
        ),
    ]

    measurement = meter.measure()

    assert measurement.n == 23
    assert measurement.discarded == 1
    assert measurement.duration_s == pytest.approx(0.1)
    assert not all(item.passed for item in validate_energy_measurement(measurement))
    assert com.calls.count(("GetData", 17, 0)) == 2
    assert com.calls[-1] == ("StopStream", 17, 0)


def test_ophir_measure_default_timeout_is_bounded_at_two_seconds():
    com = FakeOphirCOM()
    clock = FakeClock()
    meter = OphirEnergyMeter(
        com_factory=lambda: com,
        poll_interval_s=0.05,
        clock=clock,
        sleep=clock.sleep,
    )
    meter.preflight()
    com.calls.clear()

    measurement = meter.measure()

    assert measurement.n == 0
    assert measurement.duration_s == pytest.approx(2.0)
    assert com.calls.count(("GetData", 17, 0)) == 40
    assert com.calls[-1] == ("StopStream", 17, 0)


def test_ophir_measure_with_fewer_than_two_valid_values_returns_finite_gate_failure_data():
    meter, com, _ = _ophir_meter()
    meter.preflight()
    com.data_batches = [
        ([0.0001], [500.0], [0]),
        ([0.00035], [1000.0], [0]),
    ]

    measurement = meter.measure()

    assert measurement.n == 1
    assert math.isnan(measurement.stdev_uj)
    assert math.isnan(measurement.rate_hz)
    assert not all(item.passed for item in validate_energy_measurement(measurement))


def test_ophir_measure_stops_stream_when_data_read_raises():
    meter, com, _ = _ophir_meter()
    meter.preflight()
    com.calls.clear()
    com.data_error = OSError("stream failed")

    with pytest.raises(OSError, match="stream failed"):
        meter.measure()

    assert com.calls[-1] == ("StopStream", 17, 0)


def test_ophir_measure_rejects_misaligned_priming_batch_and_stops_stream():
    meter, com, _ = _ophir_meter()
    meter.preflight()
    com.calls.clear()
    com.data_batches = [([0.00035], [], [0])]

    with pytest.raises(RuntimeError, match="misaligned"):
        meter.measure()

    assert com.calls[-1] == ("StopStream", 17, 0)


def test_ophir_close_retries_measure_time_stop_failure_and_continues_cleanup():
    meter, com, _ = _ophir_meter(duration_s=0.15)
    meter.preflight()
    com.data_batches = [
        ([0.0001], [100.0], [0]),
        ([0.00035] * 26, [1000.0 + 25.0 * index for index in range(26)], [0] * 26),
    ]
    original_stop = com.StopStream
    stop_attempts = 0

    def fail_first_stop(handle, channel):
        nonlocal stop_attempts
        stop_attempts += 1
        original_stop(handle, channel)
        if stop_attempts == 1:
            raise OSError("measure stop failed")

    com.StopStream = fail_first_stop

    with pytest.raises(OSError, match="measure stop failed"):
        meter.measure()
    com.calls.clear()

    meter.close()

    assert com.calls == [
        ("StopStream", 17, 0),
        "StopAllStreams",
        ("Close", 17),
        "CloseAll",
    ]
    assert stop_attempts == 2


@pytest.mark.parametrize("failing_cleanup", ["StopStream", "Close"])
def test_ophir_close_clears_measurement_authorization_and_handle_when_cleanup_raises(
    failing_cleanup,
):
    meter, com, _ = _ophir_meter()
    meter.preflight()
    com.calls.clear()
    original = getattr(com, failing_cleanup)

    def fail_cleanup(*args):
        original(*args)
        raise OSError(f"{failing_cleanup} failed")

    setattr(com, failing_cleanup, fail_cleanup)
    if failing_cleanup == "StopStream":
        meter._stream_active = True

    with pytest.raises(OSError, match=f"{failing_cleanup} failed"):
        meter.close()

    assert meter._preflight_passed is False
    assert meter._handle is None
    com.calls.clear()
    with pytest.raises(RuntimeError, match="preflight must pass"):
        meter.measure()
    assert com.calls == []
    setattr(com, failing_cleanup, original)
    meter.close()
    assert com.calls == ["StopAllStreams", "CloseAll"]


class SafetyMap:
    _OFFSETS = {
        "TA_PULSE_WIDTH": 0,
        "EE_PULSE_WIDTH_UL": 4,
        "OPT_PULSE_WIDTH_UL": 8,
        **FakeMap._VERSION_ADDRESSES,
    }

    def get_entry_by_friendly_name(self, name):
        if name not in self._OFFSETS:
            return None
        is_version = name in FakeMap._VERSION_ADDRESSES
        return {
            "mux_idx": 1,
            "channel": 4,
            "i2c_addr": 0x41,
            "isMsbFirst": False,
            "start_address": self._OFFSETS[name],
            "data_size": "8B" if is_version else "16B",
            "scale": 1.0,
        }


class SafetyConsole(FakeConsole):
    def __init__(self, calls):
        super().__init__(True, calls)
        self.register_values = {
            0: 500,
            4: 550,
            8: 550,
            **{
                address: value
                for address, value in zip(
                    range(101, 113), range(1, 13), strict=True
                )
            },
        }
        self.trigger_reads = [{"TriggerFrequencyHz": 40.0}]

    def read_i2c_packet(self, **kwargs):
        self.calls.append(("read_i2c_packet", kwargs))
        raw = self.register_values[kwargs["reg_addr"]]
        return raw.to_bytes(kwargs["read_len"], "little"), kwargs["read_len"]


class SafetyInterface(FakeInterface):
    def __init__(self, calls):
        self.calls = calls
        self.console = SafetyConsole(calls)
        self.left = FakeDevice(True, "left-serial", "left-fw", "left-hwid")
        self.right = FakeDevice(False, "right-serial", "right-fw", "right-hwid")


def _firing_bench():
    calls = []
    interface = SafetyInterface(calls)
    meter = FakeMeter(calls)
    bench = MotionLaserCalibrationBench(
        meter,
        interface_factory=lambda: interface,
        fpga_map=SafetyMap(),
        topology_quiet_period_s=0.0,
    )
    bench.preflight("left")
    calls.clear()
    return bench, interface, meter, calls


def _dual_firing_bench():
    calls = []
    interface = SafetyInterface(calls)
    interface.right.connected = True
    meter = FakeMeter(calls)
    bench = MotionLaserCalibrationBench(
        meter,
        interface_factory=lambda: interface,
        fpga_map=SafetyMap(),
        topology_quiet_period_s=0.0,
    )
    bench.preflight_dual()
    calls.clear()
    return bench, interface, meter, calls


def test_measure_energy_verifies_exact_rate_and_both_limits_before_guaranteed_trigger_stop():
    bench, _, _, calls = _firing_bench()

    measurement = bench.measure_energy()

    assert measurement.mean_uj == 350.0
    interesting = [call if isinstance(call, str) else call[0] for call in calls]
    assert interesting == [
        "get_trigger_json",
        "read_i2c_packet",
        "read_i2c_packet",
        "read_i2c_packet",
        "start_trigger",
        "meter.measure",
        "stop_trigger",
    ]


def test_measure_energy_revalidates_declared_topology_before_firing_authorization():
    """A sensor attached after configuration must prevent trigger start."""
    bench, interface, _, calls = _firing_bench()
    interface.right.connected = True

    with pytest.raises(RuntimeError, match="exact declared topology"):
        bench.measure_energy()

    assert "start_trigger" not in calls


def test_dual_measurement_allows_exact_pair_and_rejects_a_missing_side_before_firing():
    bench, interface, _, calls = _dual_firing_bench()

    assert bench.measure_energy().mean_uj == 350.0
    assert "start_trigger" in calls

    calls.clear()
    interface.left.connected = False
    with pytest.raises(RuntimeError, match="exact declared dual topology"):
        bench.measure_energy()
    assert "start_trigger" not in calls


@pytest.mark.parametrize("rate", [39.0, 40.1])
def test_measure_energy_rejects_non_40_hz_trigger_before_firing(rate):
    bench, interface, _, calls = _firing_bench()
    interface.console.trigger_reads = [{"TriggerFrequencyHz": rate}]

    with pytest.raises(RuntimeError, match="exactly 40 Hz"):
        bench.measure_energy()

    assert "start_trigger" not in calls


@pytest.mark.parametrize("limit_offset", [4, 8])
def test_measure_energy_rejects_ta_pulse_not_strictly_below_each_active_limit(
    limit_offset,
):
    bench, interface, _, calls = _firing_bench()
    interface.console.register_values[limit_offset] = 500

    with pytest.raises(RuntimeError, match="below both active safety limits"):
        bench.measure_energy()

    assert "start_trigger" not in calls


def test_measure_energy_start_failure_is_fail_closed_and_still_attempts_stop():
    bench, interface, _, calls = _firing_bench()
    interface.console.start_trigger_result = False

    with pytest.raises(RuntimeError, match="failed to start trigger"):
        bench.measure_energy()

    assert calls[-2:] == ["start_trigger", "stop_trigger"]
    assert "meter.measure" not in calls


def test_measure_energy_start_exception_is_fail_closed_and_still_attempts_stop():
    bench, interface, _, calls = _firing_bench()
    interface.console.start_trigger = lambda: (_ for _ in ()).throw(
        OSError("start transport lost")
    )

    with pytest.raises(OSError, match="start transport lost"):
        bench.measure_energy()

    assert calls[-1] == "stop_trigger"


def test_measure_energy_meter_exception_is_fail_closed_and_stops_trigger():
    bench, _, meter, calls = _firing_bench()
    meter.measure = lambda: (_ for _ in ()).throw(OSError("meter lost"))

    with pytest.raises(OSError, match="meter lost"):
        bench.measure_energy()

    assert calls[-1] == "stop_trigger"


def test_measure_energy_stop_failure_is_reported_instead_of_returning_measurement():
    bench, interface, _, calls = _firing_bench()
    interface.console.stop_trigger_result = False

    with pytest.raises(RuntimeError, match="failed to stop trigger"):
        bench.measure_energy()

    assert calls[-2:] == ["meter.measure", "stop_trigger"]


def test_measure_energy_stop_exception_is_reported_instead_of_returning_measurement():
    bench, interface, _, calls = _firing_bench()

    def fail_stop():
        calls.append("stop_trigger")
        raise OSError("stop transport lost")

    interface.console.stop_trigger = fail_stop

    with pytest.raises(OSError, match="stop transport lost"):
        bench.measure_energy()

    assert calls[-2:] == ["meter.measure", "stop_trigger"]


# --- #263: preflight must report what actually failed ------------------------


class NeverReadyInterface(FakeInterface):
    def wait_for_ready(self, **kwargs):
        self.calls.append(("interface.wait_for_ready", kwargs))
        return False

    def describe_connections(self):
        return (
            "console=DISCONNECTED (connect_retry_exhausted:could not open port "
            "'COM5'); left=CONNECTED (ping_ok); right=CONNECTED (ping_ok); "
            "console COM port: COM5"
        )


def _never_ready_bench(interface):
    return MotionLaserCalibrationBench(
        FakeMeter(interface.calls),
        interface_factory=lambda: interface,
        fpga_map=FakeMap(),
        wait_timeout=3.5,
        topology_quiet_period_s=0.0,
    )


def test_preflight_fails_closed_with_connection_evidence_when_devices_never_connect():
    """Field case (#263): the child never connected to the console and the run
    was reported as a TA register read failure ~10.7 s in instead."""
    interface = NeverReadyInterface((False, True, True))

    with pytest.raises(RuntimeError) as excinfo:
        _never_ready_bench(interface).preflight_dual()

    assert str(excinfo.value) == (
        "Motion devices not ready within 3.5 s: console=DISCONNECTED "
        "(connect_retry_exhausted:could not open port 'COM5'); "
        "left=CONNECTED (ping_ok); right=CONNECTED (ping_ok); console COM port: COM5"
    )
    assert "meter.preflight" not in interface.calls
    assert not any(
        isinstance(call, tuple) and call[0] == "read_i2c_packet"
        for call in interface.calls
    )


def test_not_ready_reason_falls_back_to_topology_without_interface_diagnostics():
    class BareNeverReadyInterface(FakeInterface):
        def wait_for_ready(self, **kwargs):
            return False

    interface = BareNeverReadyInterface((False, True, False))

    with pytest.raises(
        RuntimeError,
        match="console not connected, left sensor connected, right sensor not connected",
    ):
        _never_ready_bench(interface).preflight("left")


def test_unresponsive_console_is_reported_before_any_fpga_register_is_read():
    bench, interface, _ = _bench((True, True, True))
    interface.console.echo = lambda data: (b"", 0)

    snapshot = bench.preflight_dual()

    assert snapshot.console_responsive is False
    assert snapshot.console_identity.serial == "console-serial"
    assert snapshot.console_identity.fpga_firmware_revisions == ()
    assert not any(
        isinstance(call, tuple) and call[0] == "read_i2c_packet"
        for call in interface.calls
    )


def test_fpga_revision_read_failure_names_the_register_and_its_location():
    bench, interface, _ = _bench((True, True, True))
    interface.console.fpga_version_raw.clear()
    interface.console.i2c_read_result = (None, None)

    with pytest.raises(
        RuntimeError,
        match=r"register TA_MAJOR: No I2C readback for TA_MAJOR "
        r"\(mux 1 ch 4 addr 0x41 reg 0x65\)",
    ):
        bench.preflight_dual()
