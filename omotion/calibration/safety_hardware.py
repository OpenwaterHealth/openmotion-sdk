"""Production Motion hardware boundaries for WI-00015 Safety Calibration."""

from __future__ import annotations

from datetime import datetime, timezone
import math
import time
from typing import Callable, Mapping, Protocol

from omotion.MotionInterface import MotionInterface
from omotion.ScanWorkflow import ScanRequest
from .laser import DeviceIdentity
from .motion_bench import (
    FpgaRegisterIO,
    MotionConsoleBenchBase,
    default_interface_factory as _default_interface_factory,
)
from .safety import (
    NormalScanEvidence,
    PowerCycleEvidence,
    SafetyController,
    SafetyWarningEvidence,
    ShippingTopology,
    validate_shipping_topology,
)
from .safety_workflow import ConsolePreflightSnapshot


class PowerCycleCoordinator(Protocol):
    """UI or fixture boundary that performs and observes a real power cycle."""

    def perform(
        self,
        *,
        minimum_off_s: float,
        expected_console_serial: str,
        is_console_connected: Callable[[], bool],
        read_console_serial: Callable[[], str | None],
    ) -> PowerCycleEvidence: ...


class MotionSafetyCalibrationBench(MotionConsoleBenchBase):
    """One long-lived Motion session for console calibration and final scan."""

    _ADC_REGISTER_NAMES: Mapping[SafetyController, str] = {
        "SAFETY_OPT": "OPT_ADC_DATA",
        "SAFETY_EE": "EE_ADC_DATA",
    }

    def __init__(
        self,
        *,
        interface_factory: Callable[[], MotionInterface] = _default_interface_factory,
        power_cycle_coordinator: PowerCycleCoordinator | None = None,
        fpga_map=None,
        wait_timeout: float = 10.0,
        safety_wait_timeout: float = 2.0,
        safety_poll_interval_s: float = 0.05,
        # ScanWorkflow may spend up to 15 seconds in its post-stop source
        # drain safety hatch. This allowance waits for that ordinary cleanup;
        # it does not extend the requested laser-acquisition duration.
        scan_timeout_pad_s: float = 20.0,
        # The console power cycle also cold-boots the sensor modules, and
        # their USB re-enumeration can take well over the ordinary ready
        # window (live NCR: run WI-00015-20260814T172200Z timed out at 10 s).
        # The final scan therefore waits on its own, longer budget.
        scan_ready_timeout_s: float = 60.0,
        clock: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], float] = time.time,
        sleep: Callable[[float], None] = time.sleep,
    ):
        timing = (
            wait_timeout,
            safety_wait_timeout,
            safety_poll_interval_s,
            scan_timeout_pad_s,
            scan_ready_timeout_s,
        )
        if any(
            isinstance(value, bool)
            or not isinstance(value, int | float)
            or not math.isfinite(float(value))
            for value in timing
        ):
            raise ValueError("hardware timing values must be finite numbers")
        if (
            wait_timeout <= 0
            or safety_wait_timeout <= 0
            or safety_poll_interval_s <= 0
            or scan_ready_timeout_s <= 0
        ):
            raise ValueError("wait and safety timing values must be positive")
        if scan_timeout_pad_s < 0:
            raise ValueError("scan timeout pad must be nonnegative")

        self._interface = interface_factory()
        self._console = self._interface.console
        self._registers = FpgaRegisterIO(self._console, fpga_map=fpga_map)
        self._power_cycle_coordinator = power_cycle_coordinator
        self._wait_timeout = float(wait_timeout)
        self._safety_wait_timeout = float(safety_wait_timeout)
        self._safety_poll_interval_s = float(safety_poll_interval_s)
        self._scan_timeout_pad_s = float(scan_timeout_pad_s)
        self._scan_ready_timeout_s = float(scan_ready_timeout_s)
        self._clock = clock
        self._wall_clock = wall_clock
        self._sleep = sleep
        self._started = False
        self._console_ready = False
        self._ready_sensor_count = 0
        self._trigger_started_wall: float | None = None

    def _ensure_started(
        self, required_sensor_count: int = 0, timeout: float | None = None
    ) -> None:
        if not self._started:
            self._interface.start(wait=False)
            self._started = True
        if self._console_ready and self._ready_sensor_count >= required_sensor_count:
            return
        ready = self._interface.wait_for_ready(
            console=True,
            sensors=required_sensor_count,
            timeout=self._wait_timeout if timeout is None else timeout,
        )
        if not ready:
            raise RuntimeError(
                self._not_ready_reason(
                    self._wait_timeout if timeout is None else timeout
                )
            )
        self._console_ready = True
        self._ready_sensor_count = max(self._ready_sensor_count, required_sensor_count)

    def preflight_console(self) -> ConsolePreflightSnapshot:
        self._ensure_started(required_sensor_count=0)
        console_responsive, console_identity = self._console_preflight()
        return ConsolePreflightSnapshot(
            topology=self._topology_snapshot(),
            console_identity=console_identity,
            console_responsive=console_responsive,
        )

    def start_trigger(self) -> None:
        rate_hz = self.read_trigger_rate_hz()
        if not 39.0 <= rate_hz <= 41.0:
            raise RuntimeError("Trigger frequency must be within 39 to 41 Hz")
        ta_pulse = self.read_register("TA_PULSE_WIDTH")
        ee_limit = self.read_register("EE_PULSE_WIDTH_UL")
        opt_limit = self.read_register("OPT_PULSE_WIDTH_UL")
        if not (
            all(math.isfinite(value) for value in (ta_pulse, ee_limit, opt_limit))
            and ta_pulse < ee_limit
            and ta_pulse < opt_limit
        ):
            raise RuntimeError(
                "TA pulse width must be finite and below both active safety limits"
            )
        self._trigger_started_wall = self._wall_clock()
        if not self._console.start_trigger():
            self._trigger_started_wall = None
            raise RuntimeError("Console failed to start trigger")

    @staticmethod
    def _warning_from_snapshot(snapshot) -> SafetyWarningEvidence:
        timestamp_value = getattr(snapshot, "timestamp", None)
        if (
            isinstance(timestamp_value, bool)
            or not isinstance(timestamp_value, int | float)
            or not math.isfinite(float(timestamp_value))
        ):
            timestamp = datetime.now(timezone.utc)
        else:
            timestamp = datetime.fromtimestamp(float(timestamp_value), timezone.utc)
        return SafetyWarningEvidence(
            timestamp=timestamp,
            safety_known=bool(getattr(snapshot, "safety_known", False)),
            safety_ok=bool(getattr(snapshot, "safety_ok", False)),
            faults=tuple(str(item) for item in getattr(snapshot, "safety_faults", ())),
            raw_state={
                "safety_se": getattr(snapshot, "safety_se", None),
                "safety_so": getattr(snapshot, "safety_so", None),
                "read_ok": getattr(snapshot, "read_ok", None),
                "error": getattr(snapshot, "error", None),
            },
        )

    def read_safety_warning(self) -> SafetyWarningEvidence:
        if self._trigger_started_wall is None:
            raise RuntimeError("Laser trigger must be active before safety telemetry")
        deadline = self._clock() + self._safety_wait_timeout
        while self._clock() < deadline:
            snapshot = self._console.telemetry.get_snapshot()
            timestamp = getattr(snapshot, "timestamp", None) if snapshot else None
            if (
                not isinstance(timestamp, bool)
                and isinstance(timestamp, int | float)
                and math.isfinite(float(timestamp))
                and float(timestamp) >= self._trigger_started_wall
                and bool(getattr(snapshot, "safety_known", False))
            ):
                return self._warning_from_snapshot(snapshot)
            remaining = deadline - self._clock()
            self._sleep(min(self._safety_poll_interval_s, remaining))
        return SafetyWarningEvidence(
            timestamp=datetime.fromtimestamp(self._wall_clock(), timezone.utc),
            safety_known=False,
            safety_ok=False,
            faults=(),
            raw_state={"error": "No fresh safety telemetry arrived while firing."},
        )

    def read_adc_ma(self, controller: SafetyController) -> float:
        try:
            register_name = self._ADC_REGISTER_NAMES[controller]
        except KeyError as exc:
            raise ValueError("controller must be SAFETY_OPT or SAFETY_EE") from exc
        return self._registers.read(register_name)

    def stop_trigger(self) -> None:
        self._trigger_started_wall = None
        if not self._console.stop_trigger():
            raise RuntimeError("Console failed to stop trigger")

    def power_cycle(
        self, *, minimum_off_s: float, expected_console_serial: str
    ) -> PowerCycleEvidence:
        if self._power_cycle_coordinator is None:
            raise RuntimeError("No power-cycle coordinator is configured")
        scan_workflow = self._interface.scan_workflow
        if scan_workflow.running:
            scan_workflow.cancel_scan(join_timeout=5.0)
        self.stop_trigger()
        self._console_ready = False
        self._ready_sensor_count = 0
        evidence = self._power_cycle_coordinator.perform(
            minimum_off_s=minimum_off_s,
            expected_console_serial=expected_console_serial,
            is_console_connected=self._console_alive,
            read_console_serial=lambda: self._safe_call(
                self._console, "read_serial_number"
            ),
        )
        if evidence.reconnect_observed:
            self._ensure_started(required_sensor_count=0)
        return evidence

    def _console_alive(self) -> bool:
        """Is the console really powered and talking, right now?

        The connection-monitor state alone is not sufficient during an
        observed power cycle: on Windows a surprise-removed COM port can
        linger in enumeration while a handle stays open (live NCR: runs
        WI-00015-20260814T175646Z / T180030Z never observed a real
        power-off). A powered-off console cannot answer an echo, so the
        round-trip is required in addition to the monitor state.
        """
        try:
            if not self._console.is_connected():
                return False
            return self._console_responsive()
        except Exception:
            return False

    @staticmethod
    def _topology_masks(topology: ShippingTopology) -> tuple[int, int, int]:
        if topology is ShippingTopology.SINGLE_LEFT:
            return 0xFF, 0, 1
        if topology is ShippingTopology.SINGLE_RIGHT:
            return 0, 0xFF, 1
        if topology is ShippingTopology.DUAL:
            return 0xFF, 0xFF, 2
        if topology is ShippingTopology.CONSOLE_ONLY:
            return 0x00, 0x00, 0
        raise ValueError("Unsupported shipping topology")

    def _scan_identities(self) -> tuple[DeviceIdentity, DeviceIdentity]:
        return (
            self._identity("left sensor", self._interface.left),
            self._identity("right sensor", self._interface.right),
        )

    def _scan_failure_evidence(
        self,
        topology: ShippingTopology,
        duration_s: float,
        error: str,
    ) -> NormalScanEvidence:
        return NormalScanEvidence(
            declared_topology=topology,
            requested_duration_s=duration_s,
            actual_duration_s=0.0,
            started=False,
            completed=False,
            canceled=False,
            error=error,
            topology=self._topology_snapshot(),
            identities=self._scan_identities(),
            overrides={},
            safety_observations=(),
            warnings=(),
        )

    def run_normal_scan(
        self, declared_topology: ShippingTopology, *, duration_s: float
    ) -> NormalScanEvidence:
        left_mask, right_mask, sensor_count = self._topology_masks(declared_topology)
        self._ensure_started(
            required_sensor_count=sensor_count,
            timeout=self._scan_ready_timeout_s,
        )
        topology = self._topology_snapshot()
        identities = self._scan_identities()
        topology_check = validate_shipping_topology(
            declared_topology,
            topology,
            left_identity=identities[0],
            right_identity=identities[1],
        )
        if not topology_check.passed:
            return self._scan_failure_evidence(
                declared_topology, duration_s, topology_check.detail
            )

        # The mains cycle that precedes this stage clears the laser-driver
        # registers (see MotionInterface.apply_laser_power); without re-applying
        # the persisted drive point the "normal scan" would not fire the laser
        # at the calibrated settings.
        self.bring_up_laser_configuration()

        request = ScanRequest(
            subject_id="WI15-SAFETY-CALIBRATION",
            duration_sec=int(duration_s),
            left_camera_mask=left_mask,
            right_camera_mask=right_mask,
            disable_laser=False,
            trigger_config=None,
        )
        observations: list[SafetyWarningEvidence] = []

        def telemetry_listener(snapshot) -> None:
            observations.append(self._warning_from_snapshot(snapshot))

        scan_workflow = self._interface.scan_workflow
        started = False
        canceled = False
        error: str | None = None
        completed = False
        cleanup_error: str | None = None
        started_at = self._clock()
        self._console.telemetry.add_listener(telemetry_listener)
        try:
            started = bool(scan_workflow.start_scan(request))
            if not started:
                error = "ScanWorkflow refused the normal scan start."
            else:
                scan_workflow.await_complete(
                    timeout_sec=float(duration_s) + self._scan_timeout_pad_s
                )
                canceled = bool(scan_workflow.last_scan_canceled)
                error = scan_workflow.last_scan_error
                if scan_workflow.running and error is None:
                    error = "Normal scan did not finish before its bounded timeout."
                completed = not scan_workflow.running and not canceled and error is None
        except Exception as exc:
            error = str(exc) or type(exc).__name__
        finally:
            if scan_workflow.running:
                try:
                    scan_workflow.cancel_scan(join_timeout=5.0)
                except Exception as exc:
                    cleanup_error = str(exc) or type(exc).__name__
            try:
                self.stop_trigger()
            except Exception as exc:
                message = str(exc) or type(exc).__name__
                cleanup_error = (
                    message if cleanup_error is None else f"{cleanup_error}; {message}"
                )
            self._console.telemetry.remove_listener(telemetry_listener)
        if cleanup_error is not None:
            error = cleanup_error if error is None else f"{error}; {cleanup_error}"
            completed = False

        actual_duration = self._clock() - started_at
        topology = self._topology_snapshot()
        identities = self._scan_identities()
        warnings = tuple(
            dict.fromkeys(
                fault
                for observation in observations
                if observation.safety_known
                for fault in observation.faults
            )
        )
        return NormalScanEvidence(
            declared_topology=declared_topology,
            requested_duration_s=float(duration_s),
            actual_duration_s=actual_duration,
            started=started,
            completed=completed,
            canceled=canceled,
            error=error,
            topology=topology,
            identities=identities,
            overrides={},
            safety_observations=tuple(observations),
            warnings=warnings,
        )

    def close(self) -> None:
        first_error: Exception | None = None
        scan_workflow = self._interface.scan_workflow
        if scan_workflow.running:
            try:
                scan_workflow.cancel_scan(join_timeout=5.0)
            except Exception as exc:
                first_error = exc
        for cleanup in (self.stop_trigger, self._interface.stop):
            try:
                cleanup()
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise first_error
