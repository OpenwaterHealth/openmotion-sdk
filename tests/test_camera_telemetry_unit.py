"""Unit tests for parse_camera_telemetry (sensor-fw#94 / #162, OB block #103).

Builds synthetic cam_telemetry_response_t blobs byte-for-byte against the wire
format in sensor-fw Core/Inc/camera_telemetry.h and checks the parser's
engineering-unit conversions and rejection paths. No hardware.
"""
import struct

from omotion.MotionSensor import (
    CAM_TELEMETRY_VERSION,
    _CAM_TELEM_CAM_FMT,
    _CAM_TELEM_CAM_SIZE,
    _CAM_TELEM_SIZE,
    parse_camera_telemetry,
)

NOMINAL = dict(
    updated_ms=123456, dgain_raw=0x010000,
    avdd=1911, dovdd=1229, dvdd=819,          # 2.799 / 1.800 / 1.200 V
    tpm_avg=0x2D80, tpm0=0x2D00, tpm1=0x2E00,  # 45.5 / 45.0 / 46.0 C
    tc_row=0x0123, expo_cmd=0x0048, expo_applied=0x0048,
    again_raw=0x1000,                          # 0x3508=0x10 -> 16x
    isp_real=0x0100, isp_dig=0x0400, isp_blc=0x0080, isp_expo=0x0048,
    blc=[0x0100] * 8,
    tpm_status=0, vm_live=0, vm_cp=0, vm_latched=0, vm_cp_latched=0,
    wd=[0, 0, 0, 0x2D, 0x80, 0], sc_state=0x07, otp=[0xAA, 0x55],
    trig=0, yavg=42, aec=0xA8, dcg=0x40, blc_ctrl=0x23, isp_ctrl=0x34,
    err=0, sweeps=5,
    # OB / black-level block (#103). Config values are the base config table's.
    z_avg=[0x0080] * 4, blc_z=[0x0100] * 4,
    blc_thres=0x0000, blk_lvl_target=0x0080, zero_ln_num=0x0002,
    blc_trig_ctrl=0xF9, bl_start=0x04, bl_end=0x1B, blk_ln_num=0x04,
    blc_ln_mode=0x50, zl_start=0x02, zl_end=0x0D, zavg_ctrl=0x00,
    zl_start2=0x08, zl_end2=0x0D, blc_fault_latch=0x00, blc_fault_state=0x00,
    dig_test_fail=0x00, dtr_fault=0x00,
)


def make_cam(**overrides):
    v = dict(NOMINAL)
    v.update(overrides)
    return struct.pack(
        _CAM_TELEM_CAM_FMT,
        v["updated_ms"], v["dgain_raw"],
        v["avdd"], v["dovdd"], v["dvdd"],
        v["tpm_avg"], v["tpm0"], v["tpm1"],
        v["tc_row"], v["expo_cmd"], v["expo_applied"], v["again_raw"],
        v["isp_real"], v["isp_dig"], v["isp_blc"], v["isp_expo"],
        *v["blc"],
        v["tpm_status"], v["vm_live"], v["vm_cp"], v["vm_latched"],
        v["vm_cp_latched"], *v["wd"], v["sc_state"], *v["otp"],
        v["trig"], v["yavg"], v["aec"], v["dcg"], v["blc_ctrl"],
        v["isp_ctrl"], v["err"], v["sweeps"],
        *v["z_avg"], *v["blc_z"],
        v["blc_thres"], v["blk_lvl_target"], v["zero_ln_num"],
        v["blc_trig_ctrl"], v["bl_start"], v["bl_end"], v["blk_ln_num"],
        v["blc_ln_mode"], v["zl_start"], v["zl_end"], v["zavg_ctrl"],
        v["zl_start2"], v["zl_end2"], v["blc_fault_latch"],
        v["blc_fault_state"], v["dig_test_fail"], v["dtr_fault"],
    )


def make_blob(cams=None, version=CAM_TELEMETRY_VERSION, valid=0xFF, size=None,
              pulse_count=42, uptime_ms=99000):
    cams = cams if cams is not None else [make_cam() for _ in range(8)]
    size = _CAM_TELEM_CAM_SIZE if size is None else size
    return struct.pack("<BBBBII", version, valid, size, 0,
                       pulse_count, uptime_ms) + b"".join(cams)


def test_wire_sizes_match_firmware():
    # Must track the _Static_asserts in sensor-fw Core/Inc/camera_telemetry.h.
    assert _CAM_TELEM_CAM_SIZE == 110
    assert _CAM_TELEM_SIZE == 892


def test_nominal_conversions():
    t = parse_camera_telemetry(make_blob())
    assert t is not None
    assert t["version"] == 2 and t["valid_mask"] == 0xFF
    assert t["fsin_pulse_count"] == 42 and t["uptime_ms"] == 99000
    c = t["cameras"][0]
    assert c["valid"] is True
    assert abs(c["avdd_v"] - 2.7993) < 0.001
    assert abs(c["dovdd_v"] - 1.8003) < 0.001
    assert abs(c["dvdd_v"] - 1.1997) < 0.001
    assert c["tpm_avg_c"] == 45.5
    assert c["tpm0_c"] == 45.0 and c["tpm1_c"] == 46.0
    assert c["again_x"] == 16.0          # code 0x100/16
    assert c["dgain_x"] == 1.0           # 0x350A=0x01 -> code 1024/1024
    assert c["expo_cmd"] == 0x48 and c["expo_applied"] == 0x48
    assert c["sc_state"] == 0x7
    assert c["otp_crc"] == (0xAA, 0x55)
    assert c["blc_ctrl"] == 0x23 and c["isp_ctrl"] == 0x34
    assert c["blc_offsets"] == [0x0100] * 8
    assert c["sweep_count"] == 5 and c["i2c_err_count"] == 0
    assert c["updated_ms"] == 123456


def test_negative_temperature_rule():
    # DS 10.5.23: 0xD000 -> -(0xD000-0xC000)/256 = -16.0 C
    t = parse_camera_telemetry(make_blob([make_cam(tpm_avg=0xD000)] + [make_cam()] * 7))
    assert t["cameras"][0]["tpm_avg_c"] == -16.0
    # Boundary: 0xC000 itself is positive (192.0 C) per the "> 0xC000" rule
    t = parse_camera_telemetry(make_blob([make_cam(tpm_avg=0xC000)] + [make_cam()] * 7))
    assert t["cameras"][0]["tpm_avg_c"] == 192.0


def test_analog_gain_code_assembly():
    # 0x3508=0x01, 0x3509=0x80 -> code[8:4]=1, code[3:0]=8 -> 0x18/16 = 1.5x
    t = parse_camera_telemetry(make_blob([make_cam(again_raw=0x0180)] + [make_cam()] * 7))
    assert t["cameras"][0]["again_x"] == 1.5


def test_digital_gain_bit_packing():
    # 0x350A=0x02, 0x350B=0x00, 0x350C=0x00 -> code 2<<10 = 2048 -> 2.0x
    t = parse_camera_telemetry(make_blob([make_cam(dgain_raw=0x020000)] + [make_cam()] * 7))
    assert t["cameras"][0]["dgain_x"] == 2.0
    # LSB bits: 0x350C=0xC0 contributes code[1:0]=3
    t = parse_camera_telemetry(make_blob([make_cam(dgain_raw=0x0100C0)] + [make_cam()] * 7))
    assert t["cameras"][0]["dgain_x"] == (1024 + 3) / 1024.0


def test_vm_raw_masking_and_blc_msb_mask():
    # Rail codes are 12-bit; reserved high nibble must be ignored.
    t = parse_camera_telemetry(make_blob([make_cam(avdd=0xF777)] + [make_cam()] * 7))
    assert abs(t["cameras"][0]["avdd_v"] - (0x777 * 6.0 / 4096.0)) < 1e-9
    # BLC applied offsets are 15-bit ({MSB[6:0],LSB}).
    t = parse_camera_telemetry(make_blob([make_cam(blc=[0x8123] * 8)] + [make_cam()] * 7))
    assert t["cameras"][0]["blc_offsets"][0] == 0x0123


def test_ob_block_nominal():
    """OB block decodes at the right offsets with the base config's values."""
    c = parse_camera_telemetry(make_blob())["cameras"][0]
    assert c["z_avg"] == [0x80] * 4
    assert c["z_avg_mean"] == 128.0
    assert c["z_avg_spread"] == 0
    assert c["blc_offsets_z"] == [0x0100] * 4
    assert c["blk_lvl_target"] == 0x080      # base config 0x4005 = 0x80
    assert c["bl_start"] == 4 and c["bl_end"] == 0x1B    # 0x4008/09
    assert c["zl_start"] == 2 and c["zl_end"] == 0x0D    # 0x4050/51
    assert c["blk_ln_num"] == 4 and c["zero_ln_num"] == 2
    assert c["blc_trig_ctrl"] == 0xF9        # base config 0x4000
    assert c["z_avg_sel"] == 0
    assert c["blc_fault_latch"] == 0 and c["blc_fault_state"] == 0


def test_z_avg_spread_flags_channel_mismatch():
    """Mono sensor: the four dark-row averages should agree; spread is the tell."""
    cam = make_cam(z_avg=[0x0080, 0x0082, 0x0080, 0x0091])
    c = parse_camera_telemetry(make_blob([cam] + [make_cam()] * 7))["cameras"][0]
    assert c["z_avg"] == [0x80, 0x82, 0x80, 0x91]
    assert c["z_avg_spread"] == 0x11
    assert c["z_avg_mean"] == (0x80 + 0x82 + 0x80 + 0x91) / 4.0


def test_ob_field_masking():
    """z_avg / offsets are 15-bit; the reserved MSB must not leak into values."""
    cam = make_cam(z_avg=[0x8123] * 4, blc_z=[0xFFFF] * 4,
                   blc_thres=0xF801, blk_lvl_target=0xF900,
                   zero_ln_num=0xFC05, bl_start=0xC4, bl_end=0xC1,
                   zavg_ctrl=0x3E, blc_fault_state=0xFE)
    c = parse_camera_telemetry(make_blob([cam] + [make_cam()] * 7))["cameras"][0]
    assert c["z_avg"] == [0x0123] * 4                 # bit 15 reserved
    assert c["blc_offsets_z"] == [0x7FFF] * 4
    assert c["blc_thres"] == 0x001                    # thres_l is 11-bit
    assert c["blk_lvl_target"] == 0x100               # 11-bit
    assert c["zero_ln_num"] == 0x005                  # 10-bit
    assert c["bl_start"] == 4 and c["bl_end"] == 1    # 0x4008/09 are [5:0]
    assert c["z_avg_sel"] == 2                        # 0x40E8[1:0]
    assert c["zavg_ctrl"] == 0x3E                     # raw byte preserved too
    assert c["blc_fault_state"] == 0                  # 0x40F2[0]


def test_valid_mask_bits():
    t = parse_camera_telemetry(make_blob(valid=0b00000101))
    valids = [c["valid"] for c in t["cameras"]]
    assert valids == [True, False, True] + [False] * 5


def test_rejects_malformed():
    assert parse_camera_telemetry(None) is None
    assert parse_camera_telemetry(b"") is None
    assert parse_camera_telemetry(make_blob()[:-1]) is None          # short
    assert parse_camera_telemetry(make_blob(version=3)) is None      # future version
    assert parse_camera_telemetry(make_blob(version=1)) is None      # pre-OB firmware
    assert parse_camera_telemetry(make_blob(size=109)) is None       # struct drift
