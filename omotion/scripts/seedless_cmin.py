"""Seedless C_min measurement (engineering; static phantom only).

Measures C_min, the speckle contrast of the unseeded TA, at the same image
mean the seeded system produces:

1. a normal laser scan gives the nominal per-camera image mean;
2. the seed is turned OFF and the TA pulse is lengthened (never driven
   harder) in short 10-frame bursts until the seedless mean matches;
3. one second of seedless frames at that pulse width gives C_min, the
   average contrast, which is printed and saved in the report.

If a line through 3 bursts at progressive pulse widths says the nominal
brightness is out of reach within 2150 us, the TA drive current is raised by
100 mA and the bursts repeated, up to 5 A; past that the run fails (or, with
--plateau-check, accepts C_min if each camera's contrast is flat within 5 %).

C_min is NEVER written to the device. The report (frames CSV, JSON, text
summary) records every frame of every phase and the matched TA pulse width.
See omotion.calibration.seedless_cmin for the logic and its limits.

LASER SAFETY: the laser fires, and for the seedless part the TA runs
unseeded with pulses up to 2.15 ms, and its drive current may be stepped up
in 100 mA steps to at most 5 A (4999.68 mA, the EE drive rating), while the
EE/OPT pulse-width upper limits
are widened (2.5 ms) and the rate lower limits lowered to 19 ms. Those
changes, and the standard trigger config, are restored and read back at the
end of every run, pass or fail. The module
must be on the static phantom with the weight; if the first readouts do not
look like the phantom, the run stops by itself. Console firmware that owns
the laser-safety limits (1.8.2, Factory image included) enforces a WI-00015
envelope that refuses the widened limits; on it the run stops before the
laser fires.
"""

from __future__ import annotations

import argparse
import datetime as dt
from pathlib import Path
import threading
import time
from typing import Callable, Sequence

import omotion
from omotion.MotionInterface import MotionInterface
from omotion.ScanWorkflow import ConfigureRequest
from omotion.calibration.reporting import _safe_component
from omotion.calibration.script_support import (
    OperatorCanceled as _OperatorCanceled,
    confirmed as _confirmed,
    emit_detail as _emit_detail,
    forward_library_logging,
    make_parser,
    required_value as _required_value,
    utc_run_id as _run_id,
)
from omotion.calibration.seedless_cmin import (
    VERDICT_PASS,
    CminSettings,
    SeedlessCminProcedure,
)
from omotion.calibration.seedless_cmin_hardware import MotionCminBench
from omotion.calibration.seedless_cmin_report import write_cmin_report
from omotion.seedless import (
    MAX_TA_CURRENT_MA,
    MAX_TA_PULSE_WIDTH_US,
    read_ta_current_ma,
    seedless_unsupported_reason,
    ta_current_problem,
)

interface_factory = MotionInterface
bench_factory = MotionCminBench

READY_TIMEOUT_S = 20.0
CONFIGURE_TIMEOUT_S = 90.0

# The standard 40 Hz camera/laser trigger, re-sent before every capture so
# the firmware dark schedule restarts aligned (same payload as the
# measurement calibration).
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

SIDES = {"left": ("left",), "right": ("right",), "both": ("left", "right")}


def _parser() -> argparse.ArgumentParser:
    def extra(parser: argparse.ArgumentParser) -> None:
        parser.add_argument("--side", choices=sorted(SIDES))
        parser.add_argument(
            "--phantom-confirmed", action="store_true",
            help="attest the module(s) are on the static phantom with the "
                 "weight and will not be touched")
        parser.add_argument(
            "--max-pulse-us", type=float, default=MAX_TA_PULSE_WIDTH_US,
            help="longest TA pulse the search may use (at most "
                 f"{MAX_TA_PULSE_WIDTH_US:g} us)")
        parser.add_argument(
            "--max-current-ma", type=float, default=MAX_TA_CURRENT_MA,
            help="highest TA drive current the search may step up to (at "
                 f"most {MAX_TA_CURRENT_MA:g} mA)")
        parser.add_argument(
            "--no-current-stepping", action="store_true",
            help="keep the unit's own TA drive current (pulse width only)")
        parser.add_argument(
            "--plateau-check", action="store_true",
            help="when the normal brightness is out of reach at the highest "
                 "current, accept C_min if each camera's contrast is flat "
                 "within 5 %% (off by default)")
        parser.add_argument(
            "--plateau-min-ratio", type=float,
            default=CminSettings.plateau_min_ratio,
            help="when the normal brightness is out of reach, the lowest "
                 "brightness (fraction of normal) the per-camera contrast "
                 "flatness check starts at (default %(default)s)")
        parser.add_argument(
            "--tolerance", type=float, default=CminSettings.tolerance,
            help="how close the seedless mean must come to nominal "
                 "(fraction; default %(default)s)")

    return make_parser(__doc__.split("\n\n")[0], extra)


def _selected_side(value, input_func, output_func) -> str:
    candidate = value
    while True:
        if candidate is None:
            candidate = input_func(
                "Which sensor is on the phantom? (left/right/both): ")
        side = candidate.strip().lower()
        if side in SIDES:
            return side
        output_func("Please answer left, right or both.")
        candidate = None


def main(
    argv: Sequence[str] | None = None,
    *,
    input_func: Callable[[str], str] | None = None,
    output_func: Callable[[str], None] | None = None,
) -> int:
    input_func = input if input_func is None else input_func
    output_func = print if output_func is None else output_func
    forward_library_logging()
    args = _parser().parse_args(argv)

    settings = CminSettings(tolerance=args.tolerance,
                            cmin_tolerance=max(CminSettings.cmin_tolerance,
                                               2 * args.tolerance))
    settings.max_ta_us = args.max_pulse_us
    settings.plateau_min_ratio = args.plateau_min_ratio
    settings.plateau_check = args.plateau_check
    settings.current_stepping = not args.no_current_stepping
    settings.max_current_ma = args.max_current_ma
    try:
        settings.validate()
    except ValueError as exc:
        output_func(f"Problem: {exc}")
        output_func("Final result: FAIL")
        return 1

    try:
        operator = _required_value(args.operator, "Operator: ", input_func)
        fixture_id = _required_value(args.fixture_id, "Fixture ID: ", input_func)
        side = _selected_side(args.side, input_func, output_func)
        sides = SIDES[side]
        if not args.phantom_confirmed and not _confirmed(
            f"Ensure the {' and '.join(sides)} sensor module"
            f"{'s are' if len(sides) > 1 else ' is'} on the static phantom "
            "with the weight. Remove the covers. Do not touch the setup "
            "during the test. Ready? (yes/no): ",
            input_func,
        ):
            raise _OperatorCanceled
    except (EOFError, KeyboardInterrupt, _OperatorCanceled):
        output_func("Seedless C_min canceled. Nothing was changed.")
        return 1

    masks = {s: (0xFF if s in sides else 0x00) for s in ("left", "right")}
    run_id = _run_id()
    output_root = Path(args.output_dir) / f"seedless-cmin-{run_id}"
    output_root.mkdir(parents=True, exist_ok=True)

    output_func("Step 1 of 4: Checking the console and the sensors ...")
    iface = interface_factory(data_dir=str(output_root / "scans"),
                              operator_id=operator)
    iface.start()
    info: dict = {"result": None, "serials": {}, "paths": None}

    def run() -> int:
        def fail(problem: str) -> int:
            output_func(f"Problem: {problem}")
            return 1

        if not iface.wait_for_ready(console=True, sensors=0,
                                    timeout=READY_TIMEOUT_S):
            return fail("the console is not connected.")
        deadline = time.time() + READY_TIMEOUT_S
        while time.time() < deadline:
            _, l_ok, r_ok = iface.is_device_connected()
            if all({"left": l_ok, "right": r_ok}[s] for s in sides):
                break
            time.sleep(1.0)
        _, l_ok, r_ok = iface.is_device_connected()
        missing = [s for s in sides if not {"left": l_ok, "right": r_ok}[s]]
        if missing:
            return fail(f"the {' and '.join(missing)} sensor is not connected.")

        serials = {"console": iface.console.read_serial_number() or ""}
        for s in sides:
            serials[f"{s} sensor"] = getattr(iface, s).read_serial_number() or ""
        info["serials"] = serials
        _emit_detail(output_func, "serial numbers: " + ", ".join(
            f"{k}={v or 'unprogrammed'}" for k, v in serials.items()))

        # Before any laser: a console that will refuse the widened safety
        # limits would only waste a nominal scan on the phantom.
        reason = seedless_unsupported_reason(iface.console)
        if reason:
            return fail(reason + ".")

        output_func("Step 2 of 4: Preparing the cameras. This can take one "
                    "minute ...")
        for s in sides:
            if not getattr(iface, s).enable_camera_power(0xFF):
                return fail(f"could not turn on the {s} cameras.")
        configured = threading.Event()
        holder: dict = {}

        def on_configured(result) -> None:
            holder["result"] = result
            configured.set()

        if not iface.start_configure_camera_sensors(
            ConfigureRequest(left_camera_mask=masks["left"],
                             right_camera_mask=masks["right"],
                             power_off_unused_cameras=False),
            on_complete_fn=on_configured,
        ):
            return fail("camera setup could not start.")
        if not configured.wait(CONFIGURE_TIMEOUT_S):
            return fail("camera setup did not finish.")
        if not getattr(holder["result"], "ok", False):
            return fail("camera setup failed: "
                        f"{getattr(holder['result'], 'error', '')}")
        if not iface.apply_laser_power():
            return fail("could not set the laser power.")
        # The nominal target must be at the unit's calibrated drive current.
        problem = ta_current_problem(iface.console)
        if problem:
            return fail(problem + ".")
        _emit_detail(output_func, "TA drive current "
                     f"{read_ta_current_ma(iface.console):.0f} mA matches the "
                     "console config")

        output_func("Step 3 of 4: Measuring. The laser will turn ON; part of "
                    "the run is with the seed OFF.")
        output_func("*** Do not touch the setup while it runs. ***")
        bench = bench_factory(iface, masks, STANDARD_TRIGGER_CONFIG)
        expected = {(s, cam) for s in sides for cam in range(8)}
        procedure = SeedlessCminProcedure(
            bench, settings, expected, output_func=output_func,
            detail_func=lambda m: _emit_detail(
                output_func, f"[{dt.datetime.now():%H:%M:%S}] {m}"))
        result = procedure.run()
        info["result"] = result
        return 0 if result.verdict == VERDICT_PASS else 1

    try:
        code = run()
    except Exception as exc:
        output_func(f"Seedless C_min stopped with an error: {exc}")
        code = 1
    finally:
        try:
            iface.stop()
        except Exception:
            pass

    result = info["result"]
    if result is None:
        output_func("Nothing was written to the device.")
        output_func("Final result: FAIL")
        return 1

    output_func("Step 4 of 4: Writing the report ...")
    console_serial = info["serials"].get("console", "")
    stem = (f"{_safe_component(console_serial)}-" if console_serial else "") \
        + f"seedless-cmin-{run_id}"
    run_info = {
        "run_id": run_id,
        "operator": operator,
        "fixture_id": fixture_id,
        "sides": ", ".join(sides),
        **{f"{k} serial": v or "unprogrammed"
           for k, v in info["serials"].items()},
        "sdk_version": getattr(omotion, "__version__", "unavailable"),
        "finished_utc": dt.datetime.now(dt.timezone.utc).isoformat(
            timespec="seconds"),
    }
    try:
        paths = write_cmin_report(result, settings, folder=output_root,
                                  stem=stem, run_info=run_info)
        for label, path in paths.items():
            output_func(f"Saved report ({label}): {path}")
    except Exception as exc:
        output_func(f"Problem: the report could not be written: {exc}")
        code = 1

    if result.restore_problem:
        output_func(f"*** WARNING: {result.restore_problem} ***")
    if result.problem:
        output_func(f"Problem: {result.problem}")
    if result.cmin is not None:
        if result.cameras:
            output_func("Per camera (C_min corrected to exactly the normal "
                        "brightness):")
            output_func(f"  {'cam':<4}{'bright':>8}{'C_min':>9}{'raw':>9}"
                        f"{'repeat':>8}{'drift':>8}{'/seeded':>9}")
            for c in result.cameras:
                drift = ("" if c.nominal_drift is None
                         else f"{c.nominal_drift:+.1%}")
                output_func(f"  {c.label:<4}{c.ratio:>7.3f}x{c.cmin:>9.4f}"
                            f"{c.cmin_raw:>9.4f}{c.repeat_spread:>8.1%}"
                            f"{drift:>8}{c.cmin_over_nominal:>9.3f}")
            for note in result.notes:
                output_func(f"Note: {note}")
        else:
            for label, value in result.cmin_per_camera.items():
                _emit_detail(output_func, f"C_min {label}: {value:.4f}")
        if result.matched_ta_us is not None:
            output_func(f"TA pulse width for matched brightness: "
                        f"{result.matched_ta_us:.1f} us")
            if result.matched_current_ma is not None:
                output_func(f"TA drive current for matched brightness: "
                            f"{result.matched_current_ma:.0f} mA (the unit's "
                            f"own is {result.start_current_ma:.0f} mA)")
        elif result.cmin_capture is not None:
            for label, spread in result.plateau_spread.items():
                _emit_detail(output_func,
                             f"contrast spread {label}: {spread:.1%}")
            output_func(
                f"Brightness not matched: C_min from the plateau check at "
                f"{result.cmin_capture.ta_pulse_width_us:.1f} us "
                f"({result.cmin_capture.ratio:.2f}x normal brightness)")
        if result.verdict == VERDICT_PASS:
            output_func(f"C_min = {result.cmin:.4f}")
        else:
            output_func(f"C_min NOT confirmed (the run failed a check): "
                        f"{result.cmin:.4f}")
    output_func("Nothing was written to the device.")
    output_func(f"Final result: {'PASS' if code == 0 else 'FAIL'}")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
