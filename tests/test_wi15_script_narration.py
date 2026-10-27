"""Unit tests for the script-terminal narration helpers in script_support.

The convention under test: plain operator lines are unprefixed; engineer
detail lines carry DETAIL_PREFIX ("# ") and are hidden by the test app's
Procedures pane unless Verbose is on.
"""

import logging

import pytest

from omotion.calibration.laser import EnergyMeasurement, SettingReadback
from omotion.calibration.script_support import (
    DETAIL_PREFIX,
    BenchNarrator,
    EventEchoRecorder,
    _DetailLogHandler,
    forward_library_logging,
)


class FakeEvent:
    def __init__(self, message):
        self.message = message


class RecordingRecorder:
    def __init__(self):
        self.recorded = []
        self.checkpoints = []
        self.json_path = "run.json"

    def record(self, event):
        self.recorded.append(event)

    def checkpoint(self, result):
        self.checkpoints.append(result)


class FakeLaserBench:
    def __init__(self):
        self.calls = []
        self.closed = 0
        self.serial = "S-1"

    def preflight(self, side):
        self.calls.append(("preflight", side))
        return object()

    def write_user_configuration(self, configuration):
        self.calls.append(("write_user_configuration", dict(configuration)))
        return dict(configuration)

    def write_register(self, name, value):
        self.calls.append(("write_register", name, value))
        return SettingReadback(name, value, value - 2)

    def measure_energy(self):
        self.calls.append(("measure_energy",))
        return EnergyMeasurement(
            n=80,
            discarded=1,
            mean_uj=362.14,
            stdev_uj=8.35,
            rate_hz=40.02,
            min_uj=300.0,
            max_uj=390.0,
            duration_s=2.0,
        )

    def read_adc_ma(self, controller):
        self.calls.append(("read_adc_ma", controller))
        return 123.4

    def broken(self):
        raise RuntimeError("transport gone")

    def close(self):
        self.closed += 1


def details(lines):
    return [line for line in lines if line.startswith(DETAIL_PREFIX)]


def plain(lines):
    return [line for line in lines if not line.startswith(DETAIL_PREFIX)]


def test_event_echo_recorder_forwards_and_echoes_detail_lines():
    inner = RecordingRecorder()
    out = []
    recorder = EventEchoRecorder(inner, out.append)

    event = FakeEvent("Bench preflight completed.")
    recorder.record(event)
    recorder.checkpoint("checkpoint-1")

    assert inner.recorded == [event]
    assert inner.checkpoints == ["checkpoint-1"]
    assert recorder.json_path == "run.json"
    assert out == [f"{DETAIL_PREFIX}Bench preflight completed."]


def test_bench_narrator_announces_a_step_at_its_exact_occurrence():
    out = []
    narrator = BenchNarrator(
        FakeLaserBench(),
        out.append,
        steps={("write_user_configuration", 2): "Step 4 of 4: Saving ..."},
    )

    narrator.write_user_configuration({"TA_PULSE_WIDTH": 500})
    assert "Step 4 of 4: Saving ..." not in out
    narrator.write_user_configuration({"TA_PULSE_WIDTH": 500})

    assert plain(out) == ["Step 4 of 4: Saving ..."]
    # The step line precedes the second call's detail line.
    assert out.index("Step 4 of 4: Saving ...") < len(out) - 1


def test_bench_narrator_prints_visible_laser_drive_lines_with_units():
    out = []
    narrator = BenchNarrator(FakeLaserBench(), out.append)

    narrator.write_register("TA_CURRENT_DRV", 4950)
    narrator.write_register("TA_PULSE_WIDTH", 510)
    narrator.write_register("SEED_CW_GAIN", 140)

    assert plain(out) == [
        "Setting the laser current to 4950 mA ...",
        "Setting the laser pulse width to 510 us ...",
    ]
    assert (
        f"{DETAIL_PREFIX}write_register(SEED_CW_GAIN, 140) -> "
        "requested 140, read back 138"
    ) in out


def test_bench_narrator_prints_the_measured_energy_heartbeat():
    out = []
    narrator = BenchNarrator(
        FakeLaserBench(), out.append, target_energy_uj=350
    )

    narrator.measure_energy()

    assert "Measured 362 uJ. Target is 350 uJ." in out
    assert f"{DETAIL_PREFIX}measure_energy() ..." in out
    assert (
        f"{DETAIL_PREFIX}measure_energy() -> 80 pulses, mean 362.1 uJ, "
        "stdev 8.3 uJ, rate 40.0 Hz"
    ) in out


def test_bench_narrator_without_target_omits_the_target_clause():
    out = []
    narrator = BenchNarrator(FakeLaserBench(), out.append)

    narrator.measure_energy()

    assert "Measured 362 uJ." in out


def test_bench_narrator_quiet_methods_narrate_only_the_first_call():
    out = []
    narrator = BenchNarrator(
        FakeLaserBench(), out.append, quiet=("read_adc_ma",)
    )

    assert narrator.read_adc_ma("SAFETY_OPT") == 123.4
    assert narrator.read_adc_ma("SAFETY_EE") == 123.4
    assert narrator.read_adc_ma("SAFETY_OPT") == 123.4

    mentions = [line for line in out if "read_adc_ma" in line]
    assert mentions == [f"{DETAIL_PREFIX}read_adc_ma(SAFETY_OPT) -> 123.4"]


def test_bench_narrator_details_failures_and_reraises():
    out = []
    narrator = BenchNarrator(FakeLaserBench(), out.append)

    with pytest.raises(RuntimeError, match="transport gone"):
        narrator.broken()

    assert (
        f"{DETAIL_PREFIX}broken() failed: RuntimeError: transport gone" in out
    )


def test_bench_narrator_forwards_close_and_plain_attributes():
    out = []
    bench = FakeLaserBench()
    narrator = BenchNarrator(bench, out.append)

    assert narrator.serial == "S-1"
    narrator.close()

    assert bench.closed == 1
    assert f"{DETAIL_PREFIX}close() -> done" in out


@pytest.fixture
def clean_root_logging():
    yield
    root = logging.getLogger()
    for handler in list(root.handlers):
        if isinstance(handler, _DetailLogHandler):
            root.removeHandler(handler)


def test_forward_library_logging_is_idempotent_and_prefixes(
    capsys, clean_root_logging
):
    forward_library_logging()
    forward_library_logging()

    root = logging.getLogger()
    installed = [
        handler
        for handler in root.handlers
        if isinstance(handler, _DetailLogHandler)
    ]
    assert len(installed) == 1

    logging.getLogger("openmotion.sdk.narration_test").warning("retrying")
    stderr = capsys.readouterr().err
    assert (
        f"{DETAIL_PREFIX}WARNING openmotion.sdk.narration_test: retrying"
        in stderr
    )
