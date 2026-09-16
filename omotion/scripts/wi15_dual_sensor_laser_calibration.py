"""Operator entry point for WI-00015 dual-sensor laser calibration."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Sequence

import omotion
from omotion.calibration.dual_sensor_laser import (
    DualSensorLaserCalibrationRequest,
    DualSensorLaserCalibrationWorkflow,
    PlacementChangeRequest,
)
from omotion.calibration.dual_sensor_laser_report import DualSensorHtmlRunReport
from omotion.calibration.laser_hardware import (
    MotionLaserCalibrationBench,
    OphirEnergyMeter,
)
from omotion.calibration.reporting import JsonRunRecorder
from omotion.calibration.script_support import (
    APPROVED_PROCEDURE_REVISION,
    PROCEDURE_ID,
    BenchNarrator,
    EventEchoRecorder,
    OperatorRunReportRequest,
    OverrideNotAuthorized,
    add_override_arguments,
    apply_cleanup_failure,
    close_bench_capturing,
    close_best_effort as _close_best_effort,
    confirmed as _confirmed,
    emit_detail as _emit_detail,
    finalize_run_artifacts,
    forward_library_logging,
    make_override_consent,
    make_parser,
    required_value as _required_value,
    resolve_override_mode,
    utc_run_id as _run_id,
)


recorder_factory = JsonRunRecorder
meter_factory = OphirEnergyMeter
bench_factory = MotionLaserCalibrationBench
workflow_factory = DualSensorLaserCalibrationWorkflow
report_factory = DualSensorHtmlRunReport

# Plain step announcements, keyed by the bench call that begins each phase
# (see BenchNarrator). The placement prompts and the measured-energy
# heartbeat carry the story inside step 3.
NARRATION_STEPS = {
    ("preflight_dual", 1):
        "Step 1 of 4: Checking the console, both sensors, and the energy meter ...",
    ("write_user_configuration", 1):
        "Step 2 of 4: Writing the standard laser settings ...",
    ("measure_energy", 1):
        "Step 3 of 4: Measuring both sensors and adjusting the laser. "
        "This can take several minutes.",
    ("write_user_configuration", 2):
        "Step 4 of 4: Saving the final settings to the console ...",
}


def _parser() -> argparse.ArgumentParser:
    def extra(parser: argparse.ArgumentParser) -> None:
        parser.add_argument("--fixture-calibration-status")
        add_override_arguments(parser, energy_band=True)

    return make_parser(__doc__, extra)


def main(
    argv: Sequence[str] | None = None,
    *,
    input_func: Callable[[str], str] | None = None,
    output_func: Callable[[str], None] | None = None,
) -> int:
    """Collect metadata, run the dual workflow, and finalize both artifacts."""
    input_func = input if input_func is None else input_func
    output_func = print if output_func is None else output_func
    forward_library_logging()
    args = _parser().parse_args(argv)
    try:
        operator = _required_value(args.operator, "Operator: ", input_func)
        # Override mode is decided (and password-checked) before any other
        # prompt and before any hardware is touched.
        override = resolve_override_mode(
            args, operator=operator, input_func=input_func, output_func=output_func
        )
        build_revision = args.build_revision or "unspecified"
        fixture_id = _required_value(args.fixture_id, "Fixture ID: ", input_func)
        fixture_calibration_status = args.fixture_calibration_status
        procedure_revision = _required_value(
            args.procedure_revision, "Procedure revision: ", input_func
        )
    except OverrideNotAuthorized as exc:
        output_func(f"Override mode not enabled: {exc}. Nothing was changed.")
        output_func("Final result: FAIL")
        return 1
    except (EOFError, KeyboardInterrupt):
        output_func("Calibration canceled. Nothing was changed.")
        return 1

    recorder = None
    meter = None
    bench = None
    try:
        run_id = _run_id()
        recorder = EventEchoRecorder(
            recorder_factory(args.output_dir, PROCEDURE_ID, run_id), output_func
        )
        meter = meter_factory()
        bench = BenchNarrator(
            bench_factory(meter), output_func, steps=NARRATION_STEPS
        )

        def acknowledge_placement(change: PlacementChangeRequest) -> bool:
            _emit_detail(output_func, change.label)
            prompt = (
                f"Put the {change.to_side} sensor "
                f"(serial {change.sensor_serial}) into the 0 cm fixture. "
                "Is it in place? (yes/no): "
            )
            return _confirmed(prompt, input_func)

        # The factory call stays byte-identical outside override mode.
        workflow_kwargs = {}
        if override is not None:
            workflow_kwargs = {
                "override": override,
                "override_consent": make_override_consent(
                    input_func, output_func, operator=operator
                ),
            }
        workflow = workflow_factory(
            bench, recorder, acknowledge_placement, **workflow_kwargs
        )
        request = DualSensorLaserCalibrationRequest(
            operator=operator,
            build_id=build_revision,
            fixture_id=fixture_id,
            fixture_calibration_status=fixture_calibration_status,
            procedure_id=PROCEDURE_ID,
            output_root=Path(args.output_dir),
            run_id=run_id,
            sdk_version=getattr(omotion, "__version__", "unavailable"),
            started_at=datetime.now(timezone.utc),
        )
        result = workflow.run(request)
        cleanup_failure = close_bench_capturing(bench)
        bench = None
        meter = None
        result = apply_cleanup_failure(result, cleanup_failure, recorder)
        return finalize_run_artifacts(
            request=request,
            result=result,
            recorder=recorder,
            report_factory=report_factory,
            procedure_revision=procedure_revision,
            procedure_slug="dual-laser-cal",
            output_func=output_func,
        )
    except Exception as exc:
        output_func(f"Calibration stopped with an error: {exc}")
        output_func("Final result: FAIL")
        return 1
    finally:
        if bench is not None:
            _close_best_effort(bench)
        else:
            _close_best_effort(meter)


if __name__ == "__main__":
    raise SystemExit(main())
