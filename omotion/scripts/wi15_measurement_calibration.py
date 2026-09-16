"""Operator entry point for WI-00015 Measurement Calibration (one sensor).

Thin bench runner around the SDK calibration engine (CalibrationWorkflow via
MotionInterface.start_calibration) - the same engine behind the app's
Calibrate button: laser-on collection scan, per-camera mean/contrast
computation, console EEPROM write, validation scan. One sensor module is
calibrated per run; run it once per shipping side. The full auditable
Measurement Calibration workflow (evidence contracts, HTML report) is
specified in docs/calibration/2026-08-12-wi15-measurement-calibration.md
and remains future work.

LASER SAFETY: the calibration scan fires the laser. The module must be on the
static phantom with the included weight (WI Figure H) - never run this with a
module in the 0 cm energy-meter fixture; with permissive thresholds the
engine would write a garbage calibration block over a good one. The phantom
attestation prompt (or --phantom-confirmed) is mandatory for every run.
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import json
import os
from pathlib import Path
import threading
import time
from typing import Callable, Sequence

from omotion import (
    CalibrationRequest,
    CalibrationThresholds,
    factory_calibration_thresholds,
    ungated_cameras,
)
from omotion.MotionInterface import MotionInterface
from omotion.ScanWorkflow import ConfigureRequest
from omotion.calibration.override import EXIT_OVERRIDE, OverrideRequest
from omotion.calibration.reporting import _safe_component
from omotion.calibration.script_support import (
    OperatorCanceled as _OperatorCanceled,
    OverrideNotAuthorized,
    add_override_arguments,
    confirmed as _confirmed,
    emit_detail as _emit_detail,
    forward_library_logging,
    make_override_consent,
    make_parser,
    required_value as _required_value,
    resolve_override_mode,
    utc_run_id as _run_id,
)


interface_factory = MotionInterface

# Scan parameters follow the approved process addendum: 15-second
# calibration scan (also the bloodflow-app's shipped
# calibration_scan_duration_sec, not motion_connector's code fallback of 5)
# and 2-second validation scan (CalibrationRequest.validation_duration_sec).
# The scan_delay_sec leading skip applies to both sub-scans.
CAL_SCAN_DURATION_SEC = 15
VAL_SCAN_DURATION_SEC = 2
CAL_SCAN_DELAY_SEC = 1
CAL_MAX_DURATION_SEC = 600
READY_TIMEOUT_S = 20.0
CONFIGURE_TIMEOUT_S = 90.0

# Re-sent to the console before each sub-scan so the firmware-side
# fsync_counter resets and the dark schedule starts aligned (the app's flows
# do the same; see the trigger_config note on CalibrationRequest). This is
# the camera/FSIN trigger payload - the laser drive point itself comes from
# the tuned EPROM values via apply_laser_power.
STANDARD_TRIGGER_CONFIG = {
    "TriggerStatus": 2,
    "TriggerFrequencyHz": 40,
    "TriggerPulseWidthUsec": 500,
    "LaserPulseDelayUsec": 100,
    "LaserPulseWidthUsec": 500,
    "LaserPulseSkipInterval": 600,
    "LaserPulseSkipDelayUsec": 1800,
    "EnableSyncOut": True,
    "EnableTaTrigger": True,
}

# Per-camera acceptance thresholds: the FACTORY values (mean 40/80 with
# corner cameras relaxed, contrast 0.25, SPEC-69 BFI/BVI, dark <= 3.0),
# sourced from the SDK's canonical factory_calibration_thresholds() —
# the same values the bloodflow-app ships in config/app_config.json.
#
# The SPEC-69 BFI/BVI gates alone are nearly self-fulfilling right after
# calibration (i_max/c_max are normalized to the just-measured values, so the
# validation scan reads BVI ~5 / BFI ~0 by construction). The mean/contrast
# minimums are the only ABSOLUTE-brightness gates - without them, "passed"
# says nothing about signal level. BFI bounds MUST straddle zero: on a static
# phantom BFI legitimately reads slightly negative.
#
# On a dim dev bench use --bench-thresholds (disables the mean/contrast
# gates, loudly) or --thresholds-json for custom values.
FACTORY_THRESHOLDS = dataclasses.asdict(factory_calibration_thresholds())
BENCH_THRESHOLDS = {
    **FACTORY_THRESHOLDS,
    "min_mean_per_camera": [0.0] * 8,
    "min_contrast_per_camera": [0.0] * 8,
}


def _parser() -> argparse.ArgumentParser:
    def extra(parser: argparse.ArgumentParser) -> None:
        parser.add_argument("--side", choices=["left", "right"])
        parser.add_argument(
            "--phantom-confirmed", action="store_true",
            help="attest the module is on the static phantom with weight "
                 "(WI Figure H) and will not be touched")
        parser.add_argument(
            "--bench-thresholds", action="store_true",
            help="disable the absolute mean/contrast gates (dim dev bench). "
                 "PASSED then does NOT certify signal level.")
        parser.add_argument(
            "--thresholds-json", default=None,
            help="JSON file of CalibrationThresholds overrides; "
                 "default: factory values")
        add_override_arguments(parser, energy_band=False)

    return make_parser(__doc__, extra)


def _selected_side(
    value: str | None,
    input_func: Callable[[str], str],
    output_func: Callable[[str], None],
) -> str:
    candidate = value
    while True:
        if candidate is None:
            candidate = input_func("Which sensor do you want to calibrate? (left/right): ")
        side = candidate.strip().lower()
        if side in ("left", "right"):
            return side
        output_func("Please answer left or right.")
        candidate = None


# Plain lines for the calibration engine's progress stages; unknown stages
# surface as detail lines only. The engine names stay engineer-speak, so the
# raw token is always echoed as detail alongside the plain line.
_STAGE_LINES = {
    "flash_sensors": "Checking the camera programs ...",
    "calibration_scan": (
        f"Measuring for {CAL_SCAN_DURATION_SEC} seconds. "
        "Do not touch the setup."
    ),
    "compute_calibration": "Computing the calibration values ...",
    "gate": "Checking the values against the limits ...",
    "override": "One or more cameras are outside the limits. "
                "Asking about the override ...",
    "write_calibration": "Saving the calibration to the console ...",
    "validation_scan": "Running a short check scan ...",
    "evaluate": "Checking the final result ...",
}


def _serial_first_output_root(
    output_root: Path,
    console_serial: str,
    output_func: Callable[[str], None],
) -> Path:
    """Rename the run folder to ``<console-serial>-<original name>``.

    Serial first, matching the other WI-15 procedures, so a folder listing
    sorts by unit. Called only after the interface has released its file
    handles. A collision gets a ``-N`` suffix; a rename failure keeps the
    original name - naming must never cost evidence.
    """
    if not console_serial:
        return output_root
    basename = f"{_safe_component(console_serial)}-{output_root.name}"
    attempt = 0
    while True:
        name = basename if attempt == 0 else f"{basename}-{attempt}"
        target = output_root.parent / name
        # Explicit existence check: POSIX rename would silently replace an
        # empty target directory.
        if target.exists():
            attempt += 1
            continue
        try:
            os.rename(output_root, target)
        except FileExistsError:
            attempt += 1
            continue
        except OSError as exc:
            _emit_detail(output_func, f"run-folder rename failed, keeping "
                                      f"{output_root.name}: {exc}")
            return output_root
        return target


def _remap_path(path: str, old_root: Path, new_root: Path) -> str:
    """Re-anchor an engine artifact path after the run-folder rename."""
    if new_root == old_root:
        return path
    try:
        return str(new_root / Path(path).relative_to(old_root))
    except ValueError:
        return path


def _build_thresholds(
    path: str | None, bench: bool
) -> tuple[CalibrationThresholds, str]:
    """Resolve thresholds. Returns (thresholds, source-label-for-the-record)."""
    if path:
        data = dict(FACTORY_THRESHOLDS)
        with open(path, "r", encoding="utf-8") as f:
            data.update(json.load(f))
        return CalibrationThresholds(**data), f"custom ({os.path.basename(path)})"
    if bench:
        return (CalibrationThresholds(**BENCH_THRESHOLDS),
                "BENCH: mean/contrast gates DISABLED - passed does not "
                "certify signal level")
    return (CalibrationThresholds(**FACTORY_THRESHOLDS),
            "factory (mean 40/80, contrast 0.25, SPEC-69 BFI/BVI, dark 3.0)")


def _gate_override_prompt(consent, thresholds_label: str):
    """Adapt the engine's gate hook to the shared operator consent question."""

    def on_override(gate_rows):
        failing = [
            row for row in gate_rows
            if row.mean_test == "FAIL" or row.contrast_test == "FAIL"
        ]
        labels = [
            f"{'L' if row.side == 'left' else 'R'}{row.cam_id + 1}"
            for row in failing
        ]
        measured = {}
        for label, row in zip(labels, failing):
            measured[f"{label} mean"] = round(row.mean, 3)
            measured[f"{label} contrast"] = round(row.avg_contrast, 4)
        return consent(OverrideRequest(
            criterion="calibration_gate",
            reason=("Scan mean/contrast below the limits on "
                    f"{', '.join(labels) or 'no camera'}."),
            measured=measured,
            accepted_band=f"limits: {thresholds_label}",
            factory_band="factory (mean 40/80, contrast 0.25)",
            proposed_configuration={
                "console calibration block": "computed from this scan; "
                "written after the check scan unless the ambient-dark "
                "check fails",
            },
        ))

    return on_override


def main(
    argv: Sequence[str] | None = None,
    *,
    input_func: Callable[[str], str] | None = None,
    output_func: Callable[[str], None] | None = None,
) -> int:
    """Confirm phantom placement, calibrate one side, and report the outcome."""
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
        fixture_id = _required_value(
            args.fixture_id, "Fixture ID: ", input_func)
        side = _selected_side(args.side, input_func, output_func)
        if not args.phantom_confirmed and not _confirmed(
            f"Ensure the {side} sensor module has been moved from the "
            "0 cm fixture to the static phantom, with the weight "
            "(WI Figure H). Remove the covers. Do not touch the setup "
            "during the test. Ready? (yes/no): ",
            input_func,
        ):
            raise _OperatorCanceled
    except OverrideNotAuthorized as exc:
        output_func(f"Override mode not enabled: {exc}. Nothing was changed.")
        output_func("Final result: FAIL")
        return 1
    except (EOFError, KeyboardInterrupt, _OperatorCanceled):
        output_func("Measurement Calibration canceled. Nothing was changed.")
        return 1

    thresholds, thresholds_label = _build_thresholds(
        args.thresholds_json, args.bench_thresholds)
    _emit_detail(output_func, f"limits: {thresholds_label}")
    if args.bench_thresholds:
        output_func("*** WARNING: bench mode - a PASS here does NOT "
                    "prove image brightness ***")
    else:
        # Refuse limits that cannot fail (#256): without this, a limits
        # file that zeroes mean/contrast would write any calibration to
        # the console and call it PASS. Deliberate ungated runs say
        # --bench-thresholds instead.
        ungated = ungated_cameras(
            thresholds,
            0xFF if side == "left" else 0x00,
            0xFF if side == "right" else 0x00,
        )
        if ungated:
            output_func("Problem: the limits disable the brightness check "
                        f"for {', '.join(ungated)}. Use --bench-thresholds "
                        "if an ungated bench run is really intended.")
            output_func("Final result: FAIL")
            return 1

    run_id = _run_id()
    output_root = Path(args.output_dir) / f"measurement-cal-{run_id}"
    output_root.mkdir(parents=True, exist_ok=True)

    output_func(f"Step 1 of 3: Checking the console and the {side} sensor ...")
    iface = interface_factory(
        data_dir=str(output_root / "scans"), operator_id=operator)
    iface.start()
    # Filled during the run; consumed after iface.stop() has released the
    # file handles, when the run folder gains its serial-first name and the
    # final artifact paths and verdict are printed.
    run_info: dict = {"verdict": "FAIL", "result": None, "console_serial": ""}

    def calibrate() -> int:
        def fail(problem: str) -> int:
            # The single "Final result:" line is printed after the run.
            output_func(f"Problem: {problem}")
            return 1

        if not iface.wait_for_ready(console=True, sensors=0,
                                    timeout=READY_TIMEOUT_S):
            return fail("the console is not connected.")
        # Give the sensor side a moment, then require the chosen module.
        deadline = time.time() + READY_TIMEOUT_S
        while time.time() < deadline:
            _, l_ok, r_ok = iface.is_device_connected()
            if l_ok if side == "left" else r_ok:
                break
            time.sleep(1.0)
        _, l_ok, r_ok = iface.is_device_connected()
        if not (l_ok if side == "left" else r_ok):
            _emit_detail(output_func, f"connected: left={l_ok}, right={r_ok}")
            return fail(f"the {side} sensor is not connected.")

        # Cold-camera bring-up: scans only stream from powered AND configured
        # cameras, and a bare script must do that itself - the clinical app
        # does it on connect, the engineering app does not. Power the chosen
        # side, then configure exactly the cameras this run uses.
        sensor = iface.left if side == "left" else iface.right
        # Record which physical units this run belongs to. The same serials
        # land in the engine's CSV/JSON report files; echoing them here puts
        # the identity in the operator transcript as well. The console
        # serial also names the artifacts and (post-run) the run folder.
        console_serial = iface.console.read_serial_number() or ""
        sensor_serial = sensor.read_serial_number() or ""
        run_info["console_serial"] = console_serial
        _emit_detail(
            output_func,
            "serial numbers: console="
            f"{console_serial or 'unprogrammed'}, "
            f"{side} sensor={sensor_serial or 'unprogrammed'}",
        )
        output_func("Step 2 of 3: Preparing the cameras. "
                    "This can take one minute ...")
        if not sensor.enable_camera_power(0xFF):
            return fail("could not turn on the cameras.")
        configured = threading.Event()
        configure_holder: dict = {}

        def on_configured(result) -> None:
            configure_holder["result"] = result
            configured.set()

        if not iface.start_configure_camera_sensors(
            ConfigureRequest(
                left_camera_mask=0xFF if side == "left" else 0x00,
                right_camera_mask=0xFF if side == "right" else 0x00,
                power_off_unused_cameras=False,
            ),
            on_complete_fn=on_configured,
        ):
            return fail("camera setup could not start.")
        if not configured.wait(CONFIGURE_TIMEOUT_S):
            return fail("camera setup did not finish.")
        configure_result = configure_holder["result"]
        if not getattr(configure_result, "ok", False):
            return fail("camera setup failed: "
                        f"{getattr(configure_result, 'error', '')}")

        # Cold-start prerequisite: after any power cycle the laser-driver
        # registers are cleared. This also applies the tuned EPROM overrides
        # (TA_CURRENT_DRV etc.) written by the laser-calibration flow.
        if not iface.apply_laser_power():
            return fail("could not set the laser power.")

        cfg = iface.console.read_config()
        cfg_data = (cfg.json_data or {}) if cfg else {}
        laser_point = {key: cfg_data.get(key)
                       for key in ("TA_PULSE_WIDTH", "TA_CURRENT_DRV")}
        _emit_detail(
            output_func,
            "console laser drive: "
            + ", ".join(f"{key}={value}" for key, value in laser_point.items()),
        )

        request = CalibrationRequest(
            operator_id=operator,
            output_dir=str(output_root),
            left_camera_mask=0xFF if side == "left" else 0x00,
            right_camera_mask=0xFF if side == "right" else 0x00,
            thresholds=thresholds,
            duration_sec=CAL_SCAN_DURATION_SEC,
            validation_duration_sec=VAL_SCAN_DURATION_SEC,
            scan_delay_sec=CAL_SCAN_DELAY_SEC,
            max_duration_sec=CAL_MAX_DURATION_SEC,
            trigger_config=dict(STANDARD_TRIGGER_CONFIG),
            notes=f"WI-00015 Measurement Calibration, side={side}, "
                  f"run {run_id}, fixture={fixture_id}, "
                  f"thresholds: {thresholds_label}",
            allow_ungated=args.bench_thresholds,
            artifact_prefix=(f"{_safe_component(console_serial)}-"
                             if console_serial else ""),
        )

        done = threading.Event()
        holder: dict = {}

        def on_complete(result) -> None:
            holder["result"] = result
            done.set()

        def on_progress(stage: str) -> None:
            line = _STAGE_LINES.get(stage)
            if line is not None:
                output_func(line)
            _emit_detail(
                output_func, f"[{dt.datetime.now():%H:%M:%S}] stage: {stage}"
            )

        output_func(f"Step 3 of 3: Calibrating the {side} sensor. "
                    "The laser will turn ON.")
        output_func("*** Do not touch the setup while it runs. ***")
        # The engine enforces the never-write rule itself: any camera
        # outside any limit means FAILED and the console EEPROM is never
        # touched. In override mode (password-checked above) the engine
        # instead asks through on_override_fn at the pre-write gate; the
        # call stays byte-identical outside override mode.
        start_kwargs: dict = {}
        if override is not None:
            start_kwargs["on_override_fn"] = _gate_override_prompt(
                make_override_consent(input_func, output_func, operator=operator),
                thresholds_label,
            )
        if not iface.start_calibration(request, on_complete_fn=on_complete,
                                       on_progress_fn=on_progress,
                                       **start_kwargs):
            return fail("could not start (is another calibration running?)")
        if not done.wait(CAL_MAX_DURATION_SEC + 60):
            iface.cancel_calibration()
            return fail("calibration took too long and was stopped.")

        result = holder["result"]
        run_info["result"] = result
        outcome = getattr(result.outcome, "value", str(result.outcome))
        passed = outcome == "passed"
        overridden = outcome == "overridden"
        _emit_detail(output_func, f"outcome: {outcome}")
        if result.rows:
            _emit_detail(output_func,
                         f"{'side':<6} {'cam':>3} {'mean':>10} "
                         f"{'avg_contrast':>13} {'bfi':>8} {'bvi':>8}")
            for row in result.rows:
                # Cameras display 1-8, matching the engine's L#/R# labels.
                _emit_detail(
                    output_func,
                    f"{row.side:<6} {row.cam_id + 1:>3} {row.mean:>10.3f} "
                    f"{row.avg_contrast:>13.4f} {row.bfi:>8.3f} "
                    f"{row.bvi:>8.3f}")
        if passed:
            output_func("All cameras are within the limits.")
        elif overridden:
            justification = getattr(result, "override_justification", "")
            output_func(
                "One or more cameras are outside the limits. The calibration "
                "was saved to the console UNDER OPERATOR OVERRIDE"
                + (f": {justification}" if justification else "") + "."
            )
        else:
            if result.error:
                output_func(f"Problem: {result.error}")
            else:
                output_func("One or more cameras are outside the limits.")
            if not result.calibration_written:
                output_func("Nothing was saved to the console.")
        if passed:
            run_info["verdict"] = "PASS"
        elif overridden:
            run_info["verdict"] = "OVERRIDE"
        elif outcome == "canceled":
            run_info["verdict"] = "CANCELED"
        if passed:
            return 0
        return EXIT_OVERRIDE if overridden else 1

    try:
        code = calibrate()
    except Exception as exc:
        output_func(f"Measurement Calibration stopped with an error: {exc}")
        code = 1
    finally:
        # Must complete before the folder rename below - the interface
        # holds open files under output_root until it stops.
        try:
            iface.stop()
        except Exception:
            pass

    final_root = _serial_first_output_root(
        output_root, run_info["console_serial"], output_func)
    result = run_info["result"]
    if result is not None:
        for label, path in (("CSV", result.csv_path),
                            ("JSON", result.json_path)):
            if path:
                remapped = _remap_path(path, output_root, final_root)
                output_func(f"Saved data ({label}): {remapped}")
    output_func(f"Final result: {run_info['verdict']}")
    if run_info["verdict"] in ("PASS", "OVERRIDE"):
        output_func("Note: for a two-sensor unit, also run this for "
                    "the other side.")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
