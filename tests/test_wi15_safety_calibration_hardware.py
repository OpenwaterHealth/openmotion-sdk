from datetime import datetime, timezone
import inspect

import pytest

from omotion.ConsoleTelemetry import ConsoleTelemetry
from omotion.MotionConfig import MotionConfig
from omotion.calibration.laser import SettingReadback
from omotion.calibration.safety import ShippingTopology
from omotion.calibration.safety_hardware import MotionSafetyCalibrationBench
from wi15_builders import valid_power_cycle
from wi15_fakes import FakeClock, FakeDevice


NOW = datetime(2026, 8, 13, 15, 0, tzinfo=timezone.utc)


class FakeTelemetry:
    def __init__(self):
        self.listeners = []
        self.snapshot = None

    def add_listener(self, listener):
        self.listeners.append(listener)

    def remove_listener(self, listener):
        if listener in self.listeners:
            self.listeners.remove(listener)

    def get_snapshot(self):
        return self.snapshot

    def emit(self, snapshot):
        self.snapshot = snapshot
        for listener in list(self.listeners):
            listener(snapshot)


class FakeConsole(FakeDevice):
    def __init__(self, calls, connected=True):
        super().__init__(connected, "C-1", "console-fw", "console-hw")
        self.calls = calls
        self.telemetry = FakeTelemetry()
        self.config_reads = [MotionConfig(json_data={"all": 1})]
        self.write_result = MotionConfig(json_data={"written": 1})
        self.trigger_reads = [{"TriggerFrequencyHz": 40.0}]
        self.trigger_set_result = True
        self.start_trigger_result = True
        self.stop_trigger_result = True
        self.raw_registers = {
            1: 5000,
            2: 500,
            3: 550,
            4: 550,
            5: 10,
            6: 20,
            **{
                address: value
                for address, value in zip(
                    range(101, 113), range(1, 13), strict=True
                )
            },
        }

    def echo(self, data):
        self.calls.append(("echo", data))
        return data, len(data)

    def read_config(self):
        self.calls.append("read_config")
        value = self.config_reads.pop(0) if len(self.config_reads) > 1 else self.config_reads[0]
        if isinstance(value, BaseException):
            raise value
        return value

    def write_config(self, config):
        self.calls.append(("write_config", dict(config.json_data)))
        return self.write_result

    def get_trigger_json(self):
        self.calls.append("get_trigger_json")
        return self.trigger_reads.pop(0) if len(self.trigger_reads) > 1 else self.trigger_reads[0]

    def set_trigger_json(self, config):
        self.calls.append(("set_trigger_json", dict(config)))
        return self.trigger_set_result

    def start_trigger(self):
        self.calls.append("start_trigger")
        return self.start_trigger_result

    def stop_trigger(self):
        self.calls.append("stop_trigger")
        return self.stop_trigger_result

    def read_i2c_packet(self, **kwargs):
        self.calls.append(("read_i2c_packet", dict(kwargs)))
        raw = self.raw_registers[kwargs["reg_addr"]]
        width = kwargs["read_len"]
        return raw.to_bytes(width, "little"), width

    def write_i2c_packet(self, **kwargs):
        self.calls.append(("write_i2c_packet", dict(kwargs)))
        return True


class FakeScanWorkflow:
    def __init__(self, calls, clock, telemetry):
        self.calls = calls
        self.clock = clock
        self.telemetry = telemetry
        self.start_result = True
        self.running = False
        self.last_scan_error = None
        self.last_scan_canceled = False
        self.request = None
        self.emitted = [
            ConsoleTelemetry(
                timestamp=2_000.0,
                safety_known=True,
                safety_ok=True,
                safety_se=0,
                safety_so=0,
                safety_faults=[],
            )
        ]

    def start_scan(self, request):
        self.calls.append(("start_scan", request))
        self.request = request
        self.running = bool(self.start_result)
        return self.start_result

    def await_complete(self, *, timeout_sec):
        self.calls.append(("await_complete", timeout_sec))
        self.clock.sleep(30.1)
        for snapshot in self.emitted:
            self.telemetry.emit(snapshot)
        self.running = False

    def cancel_scan(self, *, join_timeout=5.0):
        self.calls.append(("cancel_scan", join_timeout))
        self.last_scan_canceled = True
        self.running = False


class FakeInterface:
    def __init__(self, topology=(True, False, False), *, clock=None):
        self.calls = []
        self.console = FakeConsole(self.calls, topology[0])
        self.left = FakeDevice(topology[1], "L-1", "left-fw", "left-hw")
        self.right = FakeDevice(topology[2], "R-1", "right-fw", "right-hw")
        self.clock = clock or FakeClock()
        self.scan_workflow = FakeScanWorkflow(
            self.calls, self.clock, self.console.telemetry
        )

    def start(self, **kwargs):
        self.calls.append(("interface.start", kwargs))

    def wait_for_ready(self, **kwargs):
        self.calls.append(("interface.wait_for_ready", kwargs))
        return True

    def apply_laser_power(self):
        self.calls.append("apply_laser_power")
        return True

    def stop(self):
        self.calls.append("interface.stop")


class FakeMap:
    _ENTRIES = {
        "TA_CURRENT_DRV": (1, 1.0),
        "TA_PULSE_WIDTH": (2, 1.0),
        "EE_PULSE_WIDTH_UL": (3, 1.0),
        "OPT_PULSE_WIDTH_UL": (4, 1.0),
        "OPT_ADC_DATA": (5, 1.86),
        "EE_ADC_DATA": (6, 1.86),
        "TA_MAJOR": (101, 1.0),
        "TA_MINOR": (102, 1.0),
        "TA_REVISION": (103, 1.0),
        "SEED_MAJOR": (104, 1.0),
        "SEED_MINOR": (105, 1.0),
        "SEED_REVISION": (106, 1.0),
        "EE_MAJOR": (107, 1.0),
        "EE_MINOR": (108, 1.0),
        "EE_REVISION": (109, 1.0),
        "OPT_MAJOR": (110, 1.0),
        "OPT_MINOR": (111, 1.0),
        "OPT_REVISION": (112, 1.0),
    }

    def get_entry_by_friendly_name(self, name):
        item = self._ENTRIES.get(name)
        if item is None:
            return None
        address, scale = item
        return {
            "mux_idx": 1,
            "channel": 4,
            "i2c_addr": 0x41,
            "isMsbFirst": False,
            "start_address": address,
            "data_size": "16B",
            "scale": scale,
        }


def _valid_cycle(serial="C-1"):
    return valid_power_cycle(NOW, serial=serial)


class FakePowerCoordinator:
    def __init__(self):
        self.calls = []
        self.result = _valid_cycle()

    def perform(
        self,
        *,
        minimum_off_s,
        expected_console_serial,
        is_console_connected,
        read_console_serial,
    ):
        self.calls.append(
            (
                minimum_off_s,
                expected_console_serial,
                is_console_connected,
                read_console_serial,
            )
        )
        return self.result


def _bench(topology=(True, False, False), *, interface=None, coordinator=None):
    clock = FakeClock()
    interface = interface or FakeInterface(topology, clock=clock)
    coordinator = coordinator or FakePowerCoordinator()
    bench = MotionSafetyCalibrationBench(
        interface_factory=lambda: interface,
        power_cycle_coordinator=coordinator,
        fpga_map=FakeMap(),
        wait_timeout=3.5,
        safety_wait_timeout=0.2,
        scan_timeout_pad_s=2.0,
        clock=clock,
        wall_clock=lambda: 2_000.0,
        sleep=clock.sleep,
    )
    return bench, interface, coordinator, clock


def test_console_preflight_waits_for_console_only_and_keeps_identity_fields_independent():
    bench, interface, _, _ = _bench()
    interface.console.firmware_error = RuntimeError("firmware query failed")

    snapshot = bench.preflight_console()

    assert interface.calls[:2] == [
        ("interface.start", {"wait": False}),
        (
            "interface.wait_for_ready",
            {"console": True, "sensors": 0, "timeout": 3.5},
        ),
    ]
    assert snapshot.topology.console_connected
    assert not snapshot.topology.left_connected
    assert not snapshot.topology.right_connected
    assert snapshot.console_identity.serial == "C-1"
    assert snapshot.console_identity.firmware is None
    assert [
        (revision.controller, revision.version)
        for revision in snapshot.console_identity.fpga_firmware_revisions
    ] == [
        ("TA", "1.2.3"),
        ("SEED", "4.5.6"),
        ("SAFETY_EE", "7.8.9"),
        ("SAFETY_OPT", "10.11.12"),
    ]
    assert snapshot.console_responsive


def test_safety_hardware_module_has_no_ophir_dependency():
    source = inspect.getsource(inspect.getmodule(MotionSafetyCalibrationBench)).lower()
    assert "ophir" not in source


def test_configuration_write_returns_a_fresh_complete_readback():
    bench, interface, _, _ = _bench()
    interface.console.config_reads = [MotionConfig(json_data={"all": 1, "fresh": 2})]

    result = bench.write_user_configuration({"all": 1, "fresh": 2})

    assert result == {"all": 1, "fresh": 2}
    assert interface.calls == [
        ("write_config", {"all": 1, "fresh": 2}),
        "read_config",
    ]


def test_console_firing_preflight_uses_active_limits_without_requiring_sensors():
    bench, interface, _, _ = _bench((True, False, False))

    bench.start_trigger()

    assert "start_trigger" in interface.calls
    assert not interface.left.connected and not interface.right.connected


@pytest.mark.parametrize(
    ("register", "raw", "message"),
    [
        (2, 550, "below both active safety limits"),
        (3, 500, "below both active safety limits"),
        (4, 500, "below both active safety limits"),
    ],
)
def test_console_firing_preflight_rejects_unsafe_pulse_relationship(
    register, raw, message
):
    bench, interface, _, _ = _bench()
    interface.console.raw_registers[register] = raw

    with pytest.raises(RuntimeError, match=message):
        bench.start_trigger()

    assert "start_trigger" not in interface.calls


def test_safety_snapshot_is_fresh_for_the_firing_window_and_decoded_verbatim():
    bench, interface, _, _ = _bench()
    bench.start_trigger()
    interface.console.telemetry.snapshot = ConsoleTelemetry(
        timestamp=2_001.0,
        safety_known=True,
        safety_ok=False,
        safety_se=1,
        safety_so=4,
        safety_faults=["POWER_PEAK_CURRENT_LIMIT_FAIL", "RATE_LOWER_LIMIT_FAIL"],
    )

    warning = bench.read_safety_warning()

    assert not warning.safety_ok
    assert warning.faults == (
        "POWER_PEAK_CURRENT_LIMIT_FAIL",
        "RATE_LOWER_LIMIT_FAIL",
    )
    assert warning.raw_state["safety_se"] == 1
    assert warning.raw_state["safety_so"] == 4


def test_stale_safety_snapshot_times_out_as_unknown_instead_of_claiming_clear():
    bench, interface, _, clock = _bench()
    bench.start_trigger()
    interface.console.telemetry.snapshot = ConsoleTelemetry(
        timestamp=1_999.0,
        safety_known=True,
        safety_ok=True,
    )

    warning = bench.read_safety_warning()

    assert not warning.safety_known
    assert not warning.safety_ok
    assert "fresh" in warning.raw_state["error"]
    assert clock.now == pytest.approx(0.2)


def test_fresh_but_unknown_snapshot_waits_for_known_firing_telemetry():
    clock = FakeClock()
    interface = FakeInterface(clock=clock)
    coordinator = FakePowerCoordinator()
    interface.console.telemetry.snapshot = ConsoleTelemetry(
        timestamp=2_000.0,
        safety_known=False,
        safety_ok=True,
    )

    def advance(seconds):
        clock.sleep(seconds)
        if clock.now >= 0.05:
            interface.console.telemetry.snapshot = ConsoleTelemetry(
                timestamp=2_001.0,
                safety_known=True,
                safety_ok=True,
            )

    bench = MotionSafetyCalibrationBench(
        interface_factory=lambda: interface,
        power_cycle_coordinator=coordinator,
        fpga_map=FakeMap(),
        safety_wait_timeout=0.2,
        safety_poll_interval_s=0.05,
        clock=clock,
        wall_clock=lambda: 2_000.0,
        sleep=advance,
    )
    bench.start_trigger()

    warning = bench.read_safety_warning()

    assert warning.safety_known
    assert warning.safety_ok
    assert clock.now == pytest.approx(0.05)


def test_power_cycle_stops_activity_then_delegates_observed_state_callables():
    bench, interface, coordinator, _ = _bench()

    evidence = bench.power_cycle(minimum_off_s=15.0, expected_console_serial="C-1")

    assert evidence == _valid_cycle()
    assert interface.calls[0] == "stop_trigger"
    minimum, serial, connected, read_serial = coordinator.calls[0]
    assert (minimum, serial) == (15.0, "C-1")
    assert connected() is True
    assert read_serial() == "C-1"


@pytest.mark.parametrize(
    ("topology", "masks"),
    [
        (ShippingTopology.SINGLE_LEFT, (0xFF, 0)),
        (ShippingTopology.SINGLE_RIGHT, (0, 0xFF)),
        (ShippingTopology.DUAL, (0xFF, 0xFF)),
        (ShippingTopology.CONSOLE_ONLY, (0, 0)),
    ],
)
def test_normal_scan_uses_exact_shipping_masks_and_no_overrides(topology, masks):
    raw_topology = {
        ShippingTopology.SINGLE_LEFT: (True, True, False),
        ShippingTopology.SINGLE_RIGHT: (True, False, True),
        ShippingTopology.DUAL: (True, True, True),
        ShippingTopology.CONSOLE_ONLY: (True, False, False),
    }[topology]
    bench, interface, _, _ = _bench(raw_topology)

    evidence = bench.run_normal_scan(topology, duration_s=30.0)

    request = interface.scan_workflow.request
    assert (request.left_camera_mask, request.right_camera_mask) == masks
    assert request.duration_sec == 30
    assert request.disable_laser is False
    assert request.trigger_config is None
    assert evidence.overrides == {}
    assert evidence.started and evidence.completed
    assert evidence.actual_duration_s == pytest.approx(30.1)
    assert evidence.safety_observations[0].safety_known
    assert interface.console.telemetry.listeners == []


def test_normal_scan_refused_start_returns_complete_failure_evidence_and_cleans_up():
    bench, interface, _, _ = _bench((True, True, False))
    interface.scan_workflow.start_result = False

    evidence = bench.run_normal_scan(ShippingTopology.SINGLE_LEFT, duration_s=30.0)

    assert not evidence.started
    assert not evidence.completed
    assert "refused" in evidence.error
    assert "stop_trigger" in interface.calls
    assert interface.console.telemetry.listeners == []


def test_normal_scan_preserves_async_error_cancellation_and_decoded_safety_faults():
    bench, interface, _, _ = _bench((True, True, False))
    interface.scan_workflow.last_scan_error = "camera disconnected"
    interface.scan_workflow.last_scan_canceled = True
    interface.scan_workflow.emitted = [
        ConsoleTelemetry(
            timestamp=2_001.0,
            safety_known=True,
            safety_ok=False,
            safety_se=1,
            safety_so=0,
            safety_faults=["POWER_PEAK_CURRENT_LIMIT_FAIL"],
        )
    ]

    evidence = bench.run_normal_scan(ShippingTopology.SINGLE_LEFT, duration_s=30.0)

    assert evidence.error == "camera disconnected"
    assert evidence.canceled
    assert not evidence.completed
    assert evidence.warnings == ("POWER_PEAK_CURRENT_LIMIT_FAIL",)


def test_close_attempts_scan_trigger_and_interface_cleanup_after_first_error():
    bench, interface, _, _ = _bench((True, True, False))
    interface.scan_workflow.running = True
    interface.console.stop_trigger_result = False

    with pytest.raises(RuntimeError, match="stop trigger"):
        bench.close()

    assert any(
        isinstance(call, tuple) and call[0] == "cancel_scan"
        for call in interface.calls
    )
    assert interface.calls[-1] == "interface.stop"


def test_read_adc_ma_returns_scaled_engineering_units_not_raw_counts():
    """WI steps 30-31 operate on scaled mA (raw x 1.86), never raw counts."""
    bench, _, _, _ = _bench()
    assert bench.read_adc_ma("SAFETY_OPT") == pytest.approx(10 * 1.86)
    assert bench.read_adc_ma("SAFETY_EE") == pytest.approx(20 * 1.86)


def test_normal_scan_reapplies_the_laser_drive_point_before_scanning():
    """The preceding mains cycle cleared the laser-driver registers."""
    bench, interface, _, _ = _bench((True, True, False))
    bench.run_normal_scan(ShippingTopology.SINGLE_LEFT, duration_s=1.0)
    assert "apply_laser_power" in interface.calls


def test_console_preflight_fails_closed_when_devices_never_connect():
    """#263: a readiness timeout must not fall through to the first console
    command (the TA_MAJOR revision read) and be reported as that."""
    clock = FakeClock()
    interface = FakeInterface((False, False, False), clock=clock)
    interface.wait_for_ready = lambda **kwargs: False
    bench, interface, _, _ = _bench(interface=interface)

    with pytest.raises(
        RuntimeError,
        match="Motion devices not ready within 3.5 s: console not connected",
    ):
        bench.preflight_console()

    assert not any(
        isinstance(call, tuple) and call[0] == "read_i2c_packet"
        for call in interface.calls
    )
