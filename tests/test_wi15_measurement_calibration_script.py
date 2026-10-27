import os
from types import SimpleNamespace

import pytest

from wi15_script_harness import load_wi15_script, wi15_script_path


SCRIPT_PATH = wi15_script_path("wi15_measurement_calibration.py")


def load_script():
    return load_wi15_script(SCRIPT_PATH, "wi15_measurement_cal_under_test")


class FakeConsole:
    def __init__(self, serial="CONSN01"):
        self.serial = serial

    def read_config(self):
        return SimpleNamespace(
            json_data={"TA_PULSE_WIDTH": 590.0, "TA_CURRENT_DRV": 5000.0}
        )

    def read_serial_number(self):
        # Mirrors MotionConsole: None when unprogrammed/unreadable.
        return self.serial


class FakeSensor:
    def __init__(self, serial=None):
        self.powered_masks = []
        self.serial = serial

    def enable_camera_power(self, mask):
        self.powered_masks.append(mask)
        return True

    def read_serial_number(self):
        # Mirrors MotionSensor: None when unprogrammed/unreadable.
        return self.serial


class FakeInterface:
    def __init__(self, *, outcome="passed", refuse_start=False,
                 gate_fail=False, console_serial="CONSN01", **_kwargs):
        self.console = FakeConsole(serial=console_serial)
        self.left = FakeSensor(serial="SNL01")
        self.right = FakeSensor(serial="SNR02")
        self.outcome = outcome
        self.refuse_start = refuse_start
        self.gate_fail = gate_fail
        self.requests = []
        self.configure_requests = []
        self.started = 0
        self.stopped = 0
        self.laser_applied = 0

    def start(self):
        self.started += 1

    def start_configure_camera_sensors(self, request, *, on_complete_fn):
        self.configure_requests.append(request)
        on_complete_fn(SimpleNamespace(ok=True, error=""))
        return True

    def stop(self):
        self.stopped += 1

    def wait_for_ready(self, **_kwargs):
        return True

    def is_device_connected(self):
        return (True, True, True)

    def apply_laser_power(self):
        self.laser_applied += 1
        return True

    def _artifact_paths(self, request):
        # Mirrors the engine's naming: <prefix>calibration-<ts>.csv/.json
        # inside request.output_dir.
        return (
            os.path.join(request.output_dir,
                         f"{request.artifact_prefix}calibration-x.csv"),
            os.path.join(request.output_dir,
                         f"{request.artifact_prefix}calibration-x.json"),
        )

    def start_calibration(self, request, *, on_complete_fn, on_progress_fn=None):
        self.requests.append(request)
        if self.refuse_start:
            return False
        csv_path, json_path = self._artifact_paths(request)
        if self.gate_fail:
            # Mirrors the engine's never-write gate failure: outcome
            # FAILED, the measured rows attached, nothing written.
            row = SimpleNamespace(side="right", cam_id=5, mean=62.1,
                                  avg_contrast=0.31, bfi=0.0, bvi=5.0)
            on_complete_fn(
                SimpleNamespace(
                    outcome=SimpleNamespace(value="failed"),
                    error="calibration scan below threshold on R6; "
                          "nothing written",
                    rows=[row],
                    csv_path=csv_path,
                    json_path=json_path,
                    calibration_written=False,
                )
            )
            return True
        on_complete_fn(
            SimpleNamespace(
                outcome=SimpleNamespace(value=self.outcome),
                error="",
                rows=[],
                csv_path=csv_path,
                json_path=json_path,
                calibration_written=self.outcome == "passed",
            )
        )
        return True

    def cancel_calibration(self):
        pass


def run_main(tmp_path, monkeypatch, *, argv_extra=(), answers=(),
             outcome="passed", refuse_start=False, gate_fail=False,
             console_serial="CONSN01"):
    script = load_script()
    fake = FakeInterface(outcome=outcome, refuse_start=refuse_start,
                         gate_fail=gate_fail, console_serial=console_serial)
    monkeypatch.setattr(script, "interface_factory", lambda **kwargs: fake)
    replies = iter(answers)
    lines = []
    code = script.main(
        ["--output-dir", str(tmp_path), "--operator", "op",
         "--fixture-id", "fx-1", *argv_extra],
        input_func=lambda prompt: next(replies),
        output_func=lines.append,
    )
    return code, fake, lines


def test_declined_phantom_attestation_cancels_before_hardware(
    monkeypatch, tmp_path
):
    code, fake, lines = run_main(
        tmp_path, monkeypatch, argv_extra=["--side", "left"], answers=("no",)
    )
    assert code == 1
    assert fake.started == 0
    assert any("canceled. Nothing was changed" in line for line in lines)


def test_side_prompt_reprompts_until_left_or_right(monkeypatch, tmp_path):
    code, fake, _ = run_main(
        tmp_path, monkeypatch,
        argv_extra=["--phantom-confirmed"],
        answers=("both", "LEFT"),
    )
    assert code == 0
    assert fake.requests[0].left_camera_mask == 0xFF
    assert fake.requests[0].right_camera_mask == 0x00


@pytest.mark.parametrize(
    ("side", "left_mask", "right_mask"),
    [("left", 0xFF, 0x00), ("right", 0x00, 0xFF)],
)
def test_only_the_selected_side_is_calibrated(
    monkeypatch, tmp_path, side, left_mask, right_mask
):
    code, fake, lines = run_main(
        tmp_path, monkeypatch,
        argv_extra=["--side", side, "--phantom-confirmed"],
    )
    assert code == 0
    request = fake.requests[0]
    assert request.left_camera_mask == left_mask
    assert request.right_camera_mask == right_mask
    assert request.trigger_config["TriggerFrequencyHz"] == 40
    configure = fake.configure_requests[0]
    assert configure.left_camera_mask == left_mask
    assert configure.right_camera_mask == right_mask
    powered = fake.left if side == "left" else fake.right
    unpowered = fake.right if side == "left" else fake.left
    assert powered.powered_masks == [0xFF]
    assert unpowered.powered_masks == []
    assert fake.laser_applied == 1
    assert fake.stopped == 1
    # The transcript records which physical units the run belongs to,
    # picking the serial of the side actually under calibration.
    expected_serial = "SNL01" if side == "left" else "SNR02"
    assert any(
        "serial numbers: console=CONSN01" in line
        and f"{side} sensor={expected_serial}" in line
        for line in lines
    ), lines


def test_unprogrammed_serials_are_reported_not_fatal(monkeypatch, tmp_path):
    """A missing serial (read returns None) must not fail the run."""
    script = load_script()
    fake = FakeInterface()
    fake.left.serial = None
    monkeypatch.setattr(script, "interface_factory", lambda **kwargs: fake)
    lines = []
    code = script.main(
        ["--output-dir", str(tmp_path), "--operator", "op",
         "--fixture-id", "fx-1", "--side", "left", "--phantom-confirmed"],
        input_func=lambda prompt: "",
        output_func=lines.append,
    )
    assert code == 0
    assert any("left sensor=unprogrammed" in line for line in lines), lines


def test_fixture_id_is_collected_like_the_other_procedures_and_recorded(
    monkeypatch, tmp_path
):
    """Fixture collection is mandatory: prompted when not supplied, blank
    answers reprompted, and the collected ID lands in the run evidence
    (engine request notes) - parity with the laser/safety procedures (#268)."""
    script = load_script()
    fake = FakeInterface()
    monkeypatch.setattr(script, "interface_factory", lambda **kwargs: fake)
    prompts = []
    replies = iter(("", "  ", "FIX-42"))

    def prompt(text):
        prompts.append(text)
        return next(replies)

    code = script.main(
        ["--output-dir", str(tmp_path), "--operator", "op",
         "--side", "left", "--phantom-confirmed"],
        input_func=prompt,
        output_func=lambda _line: None,
    )

    assert code == 0
    assert prompts == ["Fixture ID: "] * 3
    assert "fixture=FIX-42" in fake.requests[0].notes


def test_supplied_fixture_id_skips_the_prompt_and_is_recorded(
    monkeypatch, tmp_path
):
    code, fake, _lines = run_main(
        tmp_path, monkeypatch,
        argv_extra=["--side", "left", "--phantom-confirmed"],
    )
    assert code == 0
    assert "fixture=fx-1" in fake.requests[0].notes


def test_phantom_attestation_is_positive_and_names_the_static_phantom(
    monkeypatch, tmp_path
):
    """The prompt tells the operator what to do - move the module from the
    0 cm fixture to the static phantom - instead of warning what not to do,
    and calls the static phantom by its full name (#268)."""
    script = load_script()
    fake = FakeInterface()
    monkeypatch.setattr(script, "interface_factory", lambda **kwargs: fake)
    prompts = []

    def prompt(text):
        prompts.append(text)
        return "yes"

    code = script.main(
        ["--output-dir", str(tmp_path), "--operator", "op",
         "--fixture-id", "fx-1", "--side", "right"],
        input_func=prompt,
        output_func=lambda _line: None,
    )

    assert code == 0
    attestation = prompts[-1]
    assert "moved from the 0 cm fixture to the static phantom" in attestation
    assert "right sensor module" in attestation
    assert "NEVER" not in attestation


def test_scan_durations_match_the_approved_process(monkeypatch, tmp_path):
    """Process addendum: 15-second calibration scan, 2-second validation."""
    code, fake, _ = run_main(
        tmp_path, monkeypatch,
        argv_extra=["--side", "left", "--phantom-confirmed"],
    )
    assert code == 0
    request = fake.requests[0]
    assert request.duration_sec == 15
    assert request.validation_duration_sec == 2
    assert request.scan_delay_sec == 1


def test_factory_thresholds_encode_spec_69_and_straddle_zero_bfi(monkeypatch, tmp_path):
    code, fake, lines = run_main(
        tmp_path, monkeypatch,
        argv_extra=["--side", "left", "--phantom-confirmed"],
    )
    assert code == 0
    thresholds = fake.requests[0].thresholds
    assert thresholds.min_bfi_per_camera == [-0.5] * 8
    assert thresholds.max_bfi_per_camera == [0.5] * 8
    assert thresholds.min_bvi_per_camera == [4.5] * 8
    assert thresholds.max_bvi_per_camera == [5.5] * 8
    assert thresholds.min_mean_per_camera[0] == 40.0
    assert thresholds.min_mean_per_camera[1] == 80.0


def test_bench_thresholds_disable_brightness_gates_loudly(monkeypatch, tmp_path):
    code, fake, lines = run_main(
        tmp_path, monkeypatch,
        argv_extra=["--side", "left", "--phantom-confirmed",
                    "--bench-thresholds"],
    )
    assert code == 0
    thresholds = fake.requests[0].thresholds
    assert thresholds.min_mean_per_camera == [0.0] * 8
    assert thresholds.min_contrast_per_camera == [0.0] * 8
    assert any("does NOT prove" in line for line in lines)


@pytest.mark.parametrize(
    ("outcome", "expected_code"), [("passed", 0), ("failed", 1)]
)
def test_exit_code_follows_engine_outcome(
    monkeypatch, tmp_path, outcome, expected_code
):
    code, fake, lines = run_main(
        tmp_path, monkeypatch,
        argv_extra=["--side", "right", "--phantom-confirmed"],
        outcome=outcome,
    )
    assert code == expected_code
    assert fake.stopped == 1
    assert any(line.startswith("Final result:") for line in lines)


def test_refused_engine_start_fails_and_stops_interface(monkeypatch, tmp_path):
    code, fake, _ = run_main(
        tmp_path, monkeypatch,
        argv_extra=["--side", "left", "--phantom-confirmed"],
        refuse_start=True,
    )
    assert code == 1
    assert fake.stopped == 1


def test_run_folder_and_artifacts_are_named_serial_first(monkeypatch, tmp_path):
    """After the run the folder is <console-serial>-measurement-cal-<runid>
    and the engine artifacts carry the same serial-first prefix, so listings
    sort by unit (#268). The printed paths point at the renamed folder."""
    code, fake, lines = run_main(
        tmp_path, monkeypatch,
        argv_extra=["--side", "left", "--phantom-confirmed"],
    )

    assert code == 0
    assert fake.requests[0].artifact_prefix == "CONSN01-"
    renamed = list(tmp_path.glob("CONSN01-measurement-cal-*"))
    assert len(renamed) == 1
    assert not list(tmp_path.glob("measurement-cal-*"))
    csv_lines = [line for line in lines
                 if line.startswith("Saved data (CSV): ")]
    assert len(csv_lines) == 1
    assert str(renamed[0]) in csv_lines[0]
    assert "CONSN01-calibration-x.csv" in csv_lines[0]
    json_lines = [line for line in lines
                  if line.startswith("Saved data (JSON): ")]
    assert len(json_lines) == 1
    assert "CONSN01-calibration-x.json" in json_lines[0]


def test_failed_runs_also_get_the_serial_first_folder(monkeypatch, tmp_path):
    """Failure evidence must sort by unit too - the rename happens whenever
    the console serial was read, not only on a pass."""
    code, _fake, _lines = run_main(
        tmp_path, monkeypatch,
        argv_extra=["--side", "right", "--phantom-confirmed"],
        gate_fail=True,
    )

    assert code == 1
    assert len(list(tmp_path.glob("CONSN01-measurement-cal-*"))) == 1
    assert not list(tmp_path.glob("measurement-cal-*"))


def test_unprogrammed_console_keeps_the_plain_folder_and_no_prefix(
    monkeypatch, tmp_path
):
    code, fake, _lines = run_main(
        tmp_path, monkeypatch,
        argv_extra=["--side", "left", "--phantom-confirmed"],
        console_serial=None,
    )

    assert code == 0
    assert fake.requests[0].artifact_prefix == ""
    assert len(list(tmp_path.glob("measurement-cal-*"))) == 1


def test_below_threshold_gate_shows_rows_and_never_writes(monkeypatch, tmp_path):
    code, fake, lines = run_main(
        tmp_path, monkeypatch,
        argv_extra=["--side", "right", "--phantom-confirmed"],
        answers=(),  # any prompt would exhaust the empty iterator and raise
        gate_fail=True,
    )
    assert code == 1
    assert any("below threshold" in line for line in lines)
    # cam_id 5 displays as camera 6 - 1-based, matching the engine's labels
    assert any("right" in line and "  6 " in line and "62.100" in line
               for line in lines)
    assert any("Nothing was saved to the console." in line for line in lines)
    assert any("Final result: FAIL" in line for line in lines)
