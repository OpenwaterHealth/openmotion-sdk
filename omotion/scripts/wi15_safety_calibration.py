"""Operator entry point for WI-00015 Safety Calibration."""

from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import math
from pathlib import Path
import time
from typing import Callable, Sequence

import omotion
from omotion.calibration.reporting import JsonRunRecorder
from omotion.calibration.safety import PowerCycleEvidence, ShippingTopology
from omotion.calibration.safety_hardware import MotionSafetyCalibrationBench
from omotion.calibration.safety_report import SafetyCalibrationHtmlRunReport
from omotion.calibration.safety_workflow import (
    SafetyCalibrationRequest,
    SafetyCalibrationWorkflow,
)
from omotion.calibration.script_support import (
    APPROVED_PROCEDURE_REVISION,
    PROCEDURE_ID,
    BenchNarrator,
    EventEchoRecorder,
    OperatorRunReportRequest,
    apply_cleanup_failure,
    close_bench_capturing,
    close_best_effort as _close_best_effort,
    finalize_run_artifacts,
    forward_library_logging,
    make_parser,
    required_value as _required_value,
    utc_run_id as _run_id,
)


recorder_factory = JsonRunRecorder
bench_factory = MotionSafetyCalibrationBench
workflow_factory = SafetyCalibrationWorkflow
report_factory = SafetyCalibrationHtmlRunReport

# Plain step announcements, keyed by the bench call that begins each phase
# (see BenchNarrator). The workflow drives the bench in a fixed order:
# console preflight, configuration read, ADC firing, limit write, power
# cycle. The SDK's connection-retry warnings during the observed power cycle
# are expected churn, not operator information; forward_library_logging
# formats them as detail lines the pane hides unless Verbose is on.
NARRATION_STEPS = {
    ("preflight_console", 1):
        "Step 1 of 5: Checking the console ...",
    ("read_user_configuration", 1):
        "Step 2 of 5: Reading the current laser settings ...",
    ("start_trigger", 1):
        "Step 3 of 5: Turning the laser on to measure the safety monitors ...",
    ("write_user_configuration", 1):
        "Step 4 of 5: Writing the new safety limits to the console ...",
    ("power_cycle", 1):
        "Step 5 of 5: Checking that the settings survive a console restart.",
}

# Polled in tight sampling loops during the ADC acquisition; narrating every
# call would flood the verbose view.
QUIET_BENCH_METHODS = ("read_adc_ma", "read_safety_warning")


class ManualPowerCycleCoordinator:
    """Guide and independently observe the required manual console restart."""

    def __init__(
        self,
        *,
        input_func: Callable[[str], str],
        output_func: Callable[[str], None],
        clock: Callable[[], float] = time.monotonic,
        wall_clock: Callable[[], float] = time.time,
        sleep: Callable[[float], None] = time.sleep,
        poll_interval_s: float = 0.1,
        disconnect_timeout_s: float = 60.0,
        reconnect_timeout_s: float = 120.0,
    ):
        timing = (poll_interval_s, disconnect_timeout_s, reconnect_timeout_s)
        if any(
            isinstance(value, bool)
            or not isinstance(value, int | float)
            or not math.isfinite(float(value))
            or value <= 0
            for value in timing
        ):
            raise ValueError("manual power-cycle timing values must be finite and positive")
        self._input = input_func
        self._output = output_func
        self._clock = clock
        self._wall_clock = wall_clock
        self._sleep = sleep
        self._poll_interval_s = float(poll_interval_s)
        self._disconnect_timeout_s = float(disconnect_timeout_s)
        self._reconnect_timeout_s = float(reconnect_timeout_s)

    def _utc_now(self) -> datetime:
        return datetime.fromtimestamp(self._wall_clock(), timezone.utc)

    def _wait_for_state(
        self,
        predicate: Callable[[], bool],
        expected: bool,
        timeout_s: float,
    ) -> bool:
        deadline = self._clock() + timeout_s
        while self._clock() < deadline:
            try:
                if bool(predicate()) is expected:
                    return True
            except Exception:
                pass
            remaining = deadline - self._clock()
            self._sleep(min(self._poll_interval_s, remaining))
        try:
            return bool(predicate()) is expected
        except Exception:
            return False

    def perform(
        self,
        *,
        minimum_off_s: float,
        expected_console_serial: str,
        is_console_connected: Callable[[], bool],
        read_console_serial: Callable[[], str | None],
    ) -> PowerCycleEvidence:
        minimum_off_s = float(minimum_off_s)
        if not math.isfinite(minimum_off_s) or minimum_off_s <= 0:
            raise ValueError("minimum_off_s must be finite and positive")

        off_requested_at = self._utc_now()
        self._output(
            "Turn the console power OFF, then back ON. Keep it OFF for "
            f"at least {minimum_off_s:g} second(s). Do not press anything - "
            "the program watches for the power cycle."
        )
        if not self._wait_for_state(
            is_console_connected, False, self._disconnect_timeout_s
        ):
            self._output(
                f"The console did not turn OFF within "
                f"{self._disconnect_timeout_s:g} seconds."
            )
            return PowerCycleEvidence(
                off_requested_at,
                None,
                None,
                None,
                None,
                None,
                False,
                False,
                False,
                None,
                expected_console_serial,
                None,
            )

        disconnected_at_clock = self._clock()
        disconnect_observed_at = self._utc_now()
        self._output(
            "The console is OFF. Waiting for it to come back ON."
        )
        if not self._wait_for_state(
            is_console_connected, True, self._reconnect_timeout_s
        ):
            self._output("The console did not come back ON in time.")
            return PowerCycleEvidence(
                off_requested_at,
                disconnect_observed_at,
                disconnect_observed_at + timedelta(seconds=minimum_off_s),
                None,
                None,
                self._clock() - disconnected_at_clock,
                True,
                False,
                False,
                None,
                expected_console_serial,
                None,
            )

        reconnect_observed_at = self._utc_now()
        off_duration_s = self._clock() - disconnected_at_clock
        # The earliest instant power-on was permitted; a reconnect before it
        # means the operator cycled too quickly, and the workflow's dwell
        # validation fails the run on this evidence.
        on_allowed_at = disconnect_observed_at + timedelta(seconds=minimum_off_s)
        try:
            console_serial_after = read_console_serial()
        except Exception:
            console_serial_after = None
        self._output("The console is ON again. Checking the saved settings.")
        return PowerCycleEvidence(
            off_requested_at,
            disconnect_observed_at,
            on_allowed_at,
            reconnect_observed_at,
            reconnect_observed_at,
            off_duration_s,
            True,
            True,
            True,
            "Observed disconnect and reconnect on the same long-lived Motion "
            "console handle without operator confirmation gates.",
            expected_console_serial,
            console_serial_after,
        )


def _parser() -> argparse.ArgumentParser:
    return make_parser(__doc__)


def main(
    argv: Sequence[str] | None = None,
    *,
    input_func: Callable[[str], str] | None = None,
    output_func: Callable[[str], None] | None = None,
) -> int:
    """Collect audit metadata, execute the workflow, and finalize artifacts."""
    input_func = input if input_func is None else input_func
    output_func = print if output_func is None else output_func
    forward_library_logging()
    args = _parser().parse_args(argv)

    try:
        operator = _required_value(args.operator, "Operator: ", input_func)
        build_revision = args.build_revision or "unspecified"
        fixture_id = _required_value(args.fixture_id, "Bench or fixture ID: ", input_func)
        procedure_revision = _required_value(
            args.procedure_revision, "Procedure revision: ", input_func
        )
    except (EOFError, KeyboardInterrupt):
        output_func("Safety Calibration canceled. Nothing was changed.")
        return 1

    # The laser safety test is console-side only: it requires a connected,
    # responsive console and nothing else. Sensor modules may be attached or
    # absent; they are not used.
    topology = ShippingTopology.CONSOLE_ONLY

    recorder = None
    bench = None
    try:
        run_id = _run_id()
        recorder = EventEchoRecorder(
            recorder_factory(args.output_dir, PROCEDURE_ID, run_id), output_func
        )
        coordinator = ManualPowerCycleCoordinator(
            input_func=input_func,
            output_func=output_func,
        )
        bench = BenchNarrator(
            bench_factory(power_cycle_coordinator=coordinator),
            output_func,
            steps=NARRATION_STEPS,
            quiet=QUIET_BENCH_METHODS,
        )
        workflow = workflow_factory(bench, recorder)
        request = SafetyCalibrationRequest(
            shipping_topology=topology,
            operator=operator,
            build_id=build_revision,
            fixture_id=fixture_id,
            procedure_id=PROCEDURE_ID,
            output_root=Path(args.output_dir),
            run_id=run_id,
            sdk_version=getattr(omotion, "__version__", "unavailable"),
            started_at=datetime.now(timezone.utc),
        )
        result = workflow.run(request)
        cleanup_failure = close_bench_capturing(bench)
        bench = None
        result = apply_cleanup_failure(result, cleanup_failure, recorder)
        return finalize_run_artifacts(
            request=request,
            result=result,
            recorder=recorder,
            report_factory=report_factory,
            procedure_revision=procedure_revision,
            procedure_slug="safety-cal",
            output_func=output_func,
        )
    except Exception as exc:
        output_func(f"Safety Calibration stopped with an error: {exc}")
        output_func("Final result: FAIL")
        return 1
    finally:
        if bench is not None:
            _close_best_effort(bench)


if __name__ == "__main__":
    raise SystemExit(main())
