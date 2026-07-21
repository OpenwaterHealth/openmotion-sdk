"""Unit tests for CameraTelemetryCsvLogger (#162) — no hardware.

Drives the logger with a fake sensor at a fast interval and checks file
naming, headers, row contents, failure rows, lifecycle idempotence, and
that ScanRequest defaults the feature off.
"""
import csv
import time

from omotion.camera_telemetry_csv import (
    CAMERA_TELEMETRY_HEADERS,
    CameraTelemetryCsvLogger,
)


def _fake_telem(uptime=5000):
    cam = {
        "valid": True, "updated_ms": uptime - 100, "sweep_count": 7,
        "i2c_err_count": 0,
        "avdd_v": 2.7793, "dovdd_v": 1.7504, "dvdd_v": 1.1621,
        "tpm_avg_c": 31.25, "tpm0_c": 31.0, "tpm1_c": 31.5,
        "tpm_status": 0, "vm_live": 0, "vm_cp": 0,
        "vm_latched": 0x40, "vm_cp_latched": 0,
        "wd_fault_a": 0x9E, "wd_fault_b": 0x1C, "wd_sticky": 0,
        "wd_state": 0, "sc_state": 6, "otp_crc": (0xAA, 0x55),
        "trig_error": 0, "yavg": 42, "tc_row": 1234,
        "expo_cmd": 0x48, "expo_applied": 0x48,
        "again_cmd": 0x1000, "again_x": 16.0, "dgain_x": 1.0,
        "aec_mode": 0xA8, "dcg_state": 0x40,
        "blc_ctrl": 0x23, "isp_ctrl": 0x34,
        "isp_real_gain": 0x100, "isp_dig_gain": 0x400,
        "isp_blc": 0x80, "isp_expo": 0x48,
        "blc_offsets": [256] * 8,
        # OB / black-level block (sensor-fw#103) — base config values.
        "z_avg": [128] * 4, "z_avg_mean": 128.0, "z_avg_spread": 0,
        "blc_offsets_z": [256] * 4,
        "blc_thres": 0, "blk_lvl_target": 128, "zero_ln_num": 2,
        "blc_trig_ctrl": 0xF9, "bl_start": 4, "bl_end": 0x1B,
        "blk_ln_num": 4, "blc_ln_mode": 0x50,
        "zl_start": 2, "zl_end": 0x0D,
        "zavg_ctrl": 0, "z_avg_sel": 0, "zl_start2": 8, "zl_end2": 0x0D,
        "blc_fault_latch": 0, "blc_fault_state": 0,
        "dig_test_fail": 0, "dtr_fault": 0,
    }
    return {"version": 2, "valid_mask": 0xFF, "fsin_pulse_count": 3,
            "uptime_ms": uptime, "cameras": [dict(cam) for _ in range(8)]}


class FakeSensor:
    def __init__(self):
        self.calls = 0

    def get_camera_telemetry(self):
        self.calls += 1
        return _fake_telem(uptime=5000 + self.calls * 1000)


class BrokenSensor:
    def get_camera_telemetry(self):
        raise RuntimeError("usb detached")


def _read_csv(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.reader(fh))


def test_writes_one_csv_per_camera_with_samples(tmp_path):
    sensor = FakeSensor()
    log = CameraTelemetryCsvLogger([("left", sensor)], str(tmp_path),
                                   "scan1_subj", interval_s=0.05)
    assert len(log.paths) == 8
    log.start()
    log.start()  # idempotent
    time.sleep(0.35)
    log.stop()
    log.stop()   # idempotent

    for cam_id in range(8):
        path = tmp_path / f"scan1_subj_left_cam{cam_id}_telemetry.csv"
        assert path.exists(), f"missing {path.name}"
        rows = _read_csv(path)
        assert rows[0] == CAMERA_TELEMETRY_HEADERS
        assert len(rows) >= 3, f"expected >=2 samples in {path.name}"
        # dict(zip(...)) below truncates silently on a width mismatch, so the
        # row width has to be asserted on its own.
        for n, row in enumerate(rows[1:], start=1):
            assert len(row) == len(CAMERA_TELEMETRY_HEADERS), (
                f"{path.name} row {n}: {len(row)} cols, "
                f"expected {len(CAMERA_TELEMETRY_HEADERS)}")
        r = dict(zip(CAMERA_TELEMETRY_HEADERS, rows[1]))
        assert r["side"] == "left" and int(r["cam"]) == cam_id
        assert int(r["read_ok"]) == 1 and r["error"] == ""
        assert float(r["avdd_v"]) == 2.7793
        assert float(r["tpm_avg_c"]) == 31.25
        assert int(r["wd_fault_a"]) == 0x9E
        assert float(r["again_x"]) == 16.0
        assert int(r["blc_offset_7"]) == 256
        assert int(r["fsin_pulse_count"]) == 3
        # OB block (sensor-fw#103): last column group must land intact.
        assert int(r["z_avg_00"]) == 128 and int(r["z_avg_11"]) == 128
        assert int(r["z_avg_spread"]) == 0
        assert int(r["zl_start"]) == 2 and int(r["zl_end"]) == 0x0D
        assert int(r["blk_lvl_target"]) == 128
        assert int(r["blc_offset_z_3"]) == 256
    assert sensor.calls >= 2


def test_failure_rows_and_survival(tmp_path):
    log = CameraTelemetryCsvLogger([("right", BrokenSensor())], str(tmp_path),
                                   "s2_x", interval_s=0.05)
    log.start()
    time.sleep(0.2)
    log.stop()
    rows = _read_csv(tmp_path / "s2_x_right_cam0_telemetry.csv")
    assert len(rows) >= 2
    assert len(rows[1]) == len(CAMERA_TELEMETRY_HEADERS)
    r = dict(zip(CAMERA_TELEMETRY_HEADERS, rows[1]))
    assert int(r["read_ok"]) == 0
    assert "usb detached" in r["error"]
    assert r["avdd_v"] == ""  # blank data columns on failed reads
    assert r["z_avg_00"] == "" and r["dtr_fault"] == ""


def test_two_sensors_get_sixteen_files(tmp_path):
    log = CameraTelemetryCsvLogger(
        [("left", FakeSensor()), ("right", FakeSensor())],
        str(tmp_path), "s3_y", interval_s=0.05)
    assert len(log.paths) == 16
    log.start()
    time.sleep(0.15)
    log.stop()
    assert (tmp_path / "s3_y_left_cam0_telemetry.csv").exists()
    assert (tmp_path / "s3_y_right_cam7_telemetry.csv").exists()


def test_scan_request_defaults_off():
    from omotion.ScanWorkflow import ScanRequest

    req = ScanRequest(subject_id="s", duration_sec=1,
                      left_camera_mask=0xFF, right_camera_mask=0)
    assert req.write_camera_telemetry_csv is False
    # Console telemetry CSV stays default-on — unchanged by this feature.
    assert req.write_telemetry_csv is True
