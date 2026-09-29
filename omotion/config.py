import re
from enum import IntEnum

import numpy as np

_SERIAL_RE = re.compile(r"^[A-Z0-9]{1,24}\Z")


def is_valid_serial(serial: str) -> bool:
    """True if serial is 1-24 uppercase-alphanumeric chars (console or sensor)."""
    return isinstance(serial, str) and bool(_SERIAL_RE.match(serial))


SERIAL_PORT = "COM24"  # Change this to your serial port
BAUD_RATE = 921600

CONSOLE_MODULE_PID = 0xA53E
SENSOR_MODULE_PID = 0x5A5A

# UART Packet structure constants
OW_START_BYTE = 0xAA
OW_END_BYTE = 0xDD
ID_COUNTER = 0  # Initializing the ID counter

# Histo Packet structure constants
HISTO_SIZE_WORDS = 1024
HISTO_BLOCK_SIZE = 1 + (HISTO_SIZE_WORDS * 4) + 1  # HID + HISTO + EOH

# Bin-index arrays used by moment computations and CSV column naming.
# HISTO_BINS[i] = i; HISTO_BINS_SQ[i] = i*i. Float64 so downstream Σ b·n(b)
# and Σ b²·n(b) keep precision for ~2.4M-count histograms.
HISTO_BINS: np.ndarray = np.arange(HISTO_SIZE_WORDS, dtype=np.float64)
HISTO_BINS_SQ: np.ndarray = HISTO_BINS * HISTO_BINS

# Full-well capacity of the OX02C1B sensor in electrons. Used to compute
# ADC gain (DN per electron) for shot-noise correction:
#   ADC_GAIN = (HISTO_SIZE_WORDS - pedestal) / ELECTRON_WELL_CAPACITY
ELECTRON_WELL_CAPACITY: int = 11_000

# Per-camera analog gain for the 8 cameras in a sensor module, indexed by
# cam_id % 8. Outer positions (0, 7) use higher gain to compensate for the
# reduced illumination at the array periphery. Used by ShotNoiseCorrectionStage
# and DarkCorrectionStage's enrichment path; see SciencePipeline.md §8.3.
CAMERA_GAIN_MAP: np.ndarray = np.array(
    [16, 4, 2, 1, 1, 2, 4, 16], dtype=np.float32
)


# Packet Types
OW_ACK = 0xE0
OW_NAK = 0xE1
OW_CMD = 0xE2
OW_RESP = 0xE3
OW_DATA = 0xE4
OW_JSON = 0xE5
OW_FPGA = 0xE6
OW_CAMERA = 0xE7
OW_IMU = 0xE8
OW_I2C_PASSTHRU = 0xE9
OW_CONTROLLER = 0xEA
OW_FPGA_PROG = 0xEB
OW_BAD_PARSE = 0xEC
OW_BAD_CRC = 0xED
OW_UNKNOWN = 0xEE
OW_ERROR = 0xEF

# FPGA Commands
OW_FPGA_SCAN = 0x10
OW_FPGA_ON = 0x11
OW_FPGA_OFF = 0x12
OW_FPGA_ACTIVATE = 0x13
OW_FPGA_ID = 0x14
OW_FPGA_ENTER_SRAM_PROG = 0x15
OW_FPGA_EXIT_SRAM_PROG = 0x16
OW_FPGA_ERASE_SRAM = 0x17
OW_FPGA_PROG_SRAM = 0x18
OW_FPGA_BITSTREAM = 0x19
OW_FPGA_USERCODE = 0x1D
OW_FPGA_STATUS = 0x1E
OW_FPGA_RESET = 0x1F
OW_FPGA_SOFT_RESET = 0x1A

# CAMERA Commands
OW_CAMERA_SCAN = 0x20
OW_CAMERA_ON = 0x21
OW_CAMERA_OFF = 0x22
OW_CAMERA_READ_TEMP = 0x24
OW_CAMERA_FSIN = 0x26
OW_CAMERA_SWITCH = 0x28
OW_CAMERA_SET_CONFIG = 0x29
OW_CAMERA_FSIN_EXTERNAL = 0x2A
OW_CAMERA_GET_HISTOGRAM = 0x2B
OW_CAMERA_SINGLE_HISTOGRAM = 0x2C
OW_CAMERA_SET_TESTPATTERN = 0x2D
OW_CAMERA_STATUS = 0x2E
OW_CAMERA_RESET = 0x2F
OW_CAMERA_POWER_ON = 0x50
OW_CAMERA_POWER_OFF = 0x51
OW_CAMERA_POWER_STATUS = 0x52
OW_CAMERA_READ_SECURITY_UID = 0x53
OW_CAMERA_GET_TELEMETRY = 0x54  # sensor-fw#94: cached cam_telemetry_response_t snapshot
OW_CAMERA_STREAM = 0x07

# Full-frame image (drip-scan) receive mode — camera-fpga#8. Firmware payload:
# reserved byte = enable (0/1), data[0] = camera bitmask. 0x30 is free in the
# OW_CAMERA command namespace (OW_IMU_INIT and FPGA_PROG_OPEN reuse the value
# in their own packet-type namespaces — no conflict).
OW_CAMERA_IMAGE_MODE = 0x30


# IMU Commands
OW_IMU_INIT = 0x30
OW_IMU_ON = 0x31
OW_IMU_OFF = 0x32
OW_IMU_SET_CONFIG = 0x33
OW_IMU_GET_TEMP = 0x34
OW_IMU_GET_ACCEL = 0x35
OW_IMU_GET_GYRO = 0x36
OW_IMU_GET_MAG = 0x37


OW_CODE_SUCCESS = 0x00
OW_CODE_IDENT_ERROR = 0xFD
OW_CODE_DATA_ERROR = 0xFE
OW_CODE_ERROR = 0xFF

OW_HISTO_PACKET = 0x01
OW_SCAN_PACKET = 0x02
OW_IMAGE_PACKET = 0x03

# Histogram streaming packet type bytes (byte[1] of histogram stream packets)
TYPE_HISTO = 0x00
TYPE_HISTO_CMP = 0x01  # RLE-compressed histogram packet
# TYPE_HISTO_CMP packets have an extra 2-byte CRC-16 of the uncompressed
# payload inserted before the normal footer.
CMP_UNCMP_CRC_SIZE = 2

# Image streaming packet type (byte[1] of stream packets on the HISTO
# endpoint) — sibling of TYPE_HISTO / TYPE_HISTO_CMP above. Same numeric value
# as OW_IMAGE_PACKET (0x03): that constant names the content class in the
# OW_*_PACKET family; TYPE_IMAGE is the stream-envelope type byte the reader
# thread dispatches on. Defined separately so each namespace stays coherent.
TYPE_IMAGE = 0x03

# Global Commands
OW_CMD_PING = 0x00
OW_CMD_DIAG_STATS = 0x01  # #70: cam_diag_stats_t snapshot, printf-independent
OW_CMD_VERSION = 0x02
OW_CMD_ECHO = 0x03
OW_CMD_TOGGLE_LED = 0x04
OW_CMD_HWID = 0x05
OW_CMD_SERIAL = 0x07
OW_CMD_I2C_REG_READ = 0x08
OW_CMD_MESSAGES = 0x09
# Sensor-module 0x09 (NOT console — there 0x09 is OW_CMD_MESSAGES above). Reports
# runtime SCB->VTOR so a host can tell bare-metal from bootloader-slot without a
# DFU cycle. The console equivalent will use a different ID (console-fw #45),
# since 0x09 is taken there. See openmotion-sensor-fw #110.
OW_CMD_BOOT_INFO = 0x09
# Console-module BOOT_INFO. 0x0B because 0x09 is OW_CMD_MESSAGES on the console.
# Same reply payload as the sensor's, so parse_boot_info covers both. See
# openmotion-console-fw #45.
OW_CMD_BOOT_INFO_CONSOLE = 0x0B
OW_CMD_USR_CFG = 0x0A
OW_CMD_DFU = 0x0D
OW_CMD_NOP = 0x0E
OW_CMD_RESET = 0x0F
OW_CMD_I2C_BROADCAST = 0x06
OW_CMD_DEBUG_FLAGS = 0x0C
OW_CMD_I2C_STATUS = 0x0B

# Debug flag bits.
DEBUG_FLAG_USB_PRINTF = 0x01  # Turn on or off USB printf logging
DEBUG_FLAG_HISTO_THROTTLE = (
    0x02  # Only send histogram packet every 5s; others pretend success
)
DEBUG_FLAG_FAKE_DATA = (
    0x04  # Turn on or off fake data mode, turns off cameras and sends fake data
)
DEBUG_FLAG_HISTO_CMP = 0x40  # Send compressed histogram packets (TYPE_HISTO_CMP)
DEBUG_FLAG_COMM_VERBOSE = 0x10  # Enable cmd id and "." response prints in uart_comms
DEBUG_FLAG_CMD_VERBOSE = 0x20  # Enable printf in command handlers (if_commands.c)
DEBUG_FLAG_SEND_DEFER = 0x80  # Defer per-frame histogram send out of the FSIN ISR into the main loop (sensor-fw#68)
DEBUG_FLAG_HISTO_STALL = 0x100  # Stop sending histogram frames after ~45 s while USB stays alive — deterministic camera-stall repro (sensor-fw#75)
DEBUG_FLAG_HISTO_SPARSE = 0x08  # Send histogram data in small chunks over ~15 s to reduce EMI
DEBUG_FLAG_CAMERA_CROP = 0x200  # Crop camera output to 1720x1280 (drop right 200 columns) at camera (re)configure — misaligned-optic test (sensor-fw#86)
DEBUG_FLAG_CAMERA_RAW = 0x400  # Raw "scientific sensor" mode: disable all on-sensor pixel corrections (BLC/DC-BLC/dither/OTP DPC) at camera (re)configure (sensor-fw#89)

# Controller Commands
OW_CTRL_I2C_SCAN = 0x10
OW_CTRL_SET_IND = 0x11
OW_CTRL_GET_IND = 0x12
OW_CTRL_SET_TRIG = 0x13
OW_CTRL_GET_TRIG = 0x14
OW_CTRL_START_TRIG = 0x15
OW_CTRL_STOP_TRIG = 0x16
OW_CTRL_SET_FAN = 0x17
OW_CTRL_GET_FAN = 0x18
OW_CTRL_I2C_RD = 0x19
OW_CTRL_I2C_WR = 0x1A
OW_CTRL_GET_FSYNC = 0x1B
OW_CTRL_GET_LSYNC = 0x1C
OW_CTRL_TEC_DAC = 0x1D
OW_CTRL_READ_ADC = 0x1E
OW_CTRL_READ_GPIO = 0x1F
OW_CTRL_GET_TEMPS = 0x20
OW_CTRL_TECADC = 0x21
OW_CTRL_TEC_STATUS = 0x22
OW_CTRL_BOARDID = 0x23
OW_CTRL_PDUMON = 0x24
OW_CTRL_GET_PDC_BUFFER = 0x25
# Lifetime usage counters persisted to console flash. System counter is minutes
# of uptime (uint32, ~8000 yr range); laser counter is cumulative LSYNC pulses
# across all scans (uint32, ~3.4 yr at 40 Hz continuous).
OW_CTRL_GET_SYSTEM_ODO = 0x26
OW_CTRL_GET_LASER_ODO = 0x27
# Payload: 1 byte target (0=system, 1=laser, 2=both). Missing payload defaults
# to both.
OW_CTRL_RESET_ODO = 0x28
OW_CTRL_I2C_STATUS = 0x29
OW_CTRL_FAN_CTL = 0x0A

# Page-by-page direct FPGA programming commands (0x30–0x3C)
FPGA_PROG_OPEN = 0x30
FPGA_PROG_ERASE = 0x31
FPGA_PROG_CFG_RESET = 0x32
FPGA_PROG_CFG_WRITE_PAGE = 0x33
FPGA_PROG_CFG_READ_PAGE = 0x34
FPGA_PROG_UFM_RESET = 0x35
FPGA_PROG_UFM_WRITE_PAGE = 0x36
FPGA_PROG_UFM_READ_PAGE = 0x37
FPGA_PROG_FEATROW_WRITE = 0x38
FPGA_PROG_FEATROW_READ = 0x39
FPGA_PROG_SET_DONE = 0x3A
FPGA_PROG_REFRESH = 0x3B
FPGA_PROG_CLOSE = 0x3C
FPGA_PROG_CFG_WRITE_PAGES = 0x3D  # Write N 16-byte CFG pages (N*16 bytes payload)
FPGA_PROG_UFM_WRITE_PAGES = 0x3E  # Write N 16-byte UFM pages (N*16 bytes payload)
FPGA_PROG_READ_STATUS = 0x3F  # Read 32-bit Status Register (no cfgEn required)

OW_FACTORY_I2C_SCAN = 0x60
OW_FACTORY_CRESET = 0x68
OW_FACTORY_I2C_RD = 0x69
OW_FACTORY_I2C_WR = 0x6A
OW_FACTORY_I2C_WRRD = 0x6B
OW_FACTORY_NVCM_CHECK = 0x6C

TEST_PATTERN_BARS = 0x00
TEST_PATTERN_SOLID = 0x01
TEST_PATTERN_CHECKERBOARD = 0x02
TEST_PATTERN_GRADIENT = 0x03
TEST_PATTERN_DISABLED = 0x04


# --------------------------------------------------------------------------- #
# MachXO2 device types (XO2Devices_t in XO2_dev.h)
# --------------------------------------------------------------------------- #
class XO2Devices(IntEnum):
    MachXO2_256 = 0
    MachXO2_640 = 1
    MachXO2_640U = 2
    MachXO2_1200 = 3
    MachXO2_1200U = 4
    MachXO2_2000 = 5
    MachXO2_2000U = 6
    MachXO2_4000 = 7
    MachXO2_7000 = 8


# --------------------------------------------------------------------------- #
# Transport constants
# --------------------------------------------------------------------------- #
COMMAND_MAX_SIZE: int = 4096
"""Maximum total frame size (matches firmware COMMAND_MAX_SIZE)."""

MAX_DATA_PER_FRAME: int = COMMAND_MAX_SIZE - 12
"""Max payload bytes per frame (total - framing overhead)."""

# ---------------------------------------------------------------------------
# Hardware geometry — shared by Calibration, ScanWorkflow, CalibrationWorkflow.
# ---------------------------------------------------------------------------
MODULES: int = 2
"""Number of sensor modules per device (left + right)."""

CAMS_PER_MODULE: int = 8
"""Cameras per sensor module (OX02C1B array)."""

CAPTURE_HZ: float = 40.0
"""Histogram capture rate per camera, in Hz."""

# ---------------------------------------------------------------------------
# CalibrationWorkflow defaults.
# ---------------------------------------------------------------------------
CALIBRATION_I_MAX_MULTIPLIER: float = 2.0
"""Multiplier applied to the average light-frame mean to derive I_max."""

CALIBRATION_DEFAULT_SCAN_DELAY_SEC: int = 1
"""Default lead-in skip per sub-scan, in seconds."""

CALIBRATION_DEFAULT_MAX_DURATION_SEC: int = 600
"""Default watchdog timeout for the whole calibration procedure, in seconds."""


XO2_FLASH_PAGE_SIZE: int = 16
"""Bytes per page in the MachXO2 Configuration and UFM flash sectors."""

FPGA_PROG_BATCH_PAGES: int = 32
"""Number of 16-byte pages bundled into a single FPGA_PROG_CFG/UFM_WRITE_PAGES command."""

# Erase mode bitmap (matches XO2ECA_CMD_ERASE_* macros in XO2_cmds.h)
ERASE_SRAM: int = 0x01
ERASE_FTROW: int = 0x02
ERASE_CFG: int = 0x04
ERASE_UFM: int = 0x08
ERASE_ALL: int = ERASE_UFM | ERASE_CFG | ERASE_FTROW  # 0x0E


class MuxChannel(IntEnum):
    FPGA_SEED = 0
    FPGA_TA = 1
    FPGA_SAFE_EE = 2
    FPGA_SAFE_OPT = 3


# ---------------------------------------------------------------------------
# Trigger config defaults
#
# Single source of truth for the JSON payload that
# ``MotionConsole.set_trigger_json`` expects. Workflows
# (CalibrationWorkflow, ScanWorkflow) consult this when their request
# doesn't carry a ``trigger_config`` override; an app can also pass
# ``MotionInterface(default_trigger_config=...)`` to layer its own
# overrides on top of these defaults at construction time.
#
# Values match what the bloodflow-app and the early CLI scripts have
# been hardcoding everywhere — extracted so changing the standard
# 40 Hz pulse pattern is a one-file edit.
# ---------------------------------------------------------------------------
DEFAULT_TRIGGER_CONFIG: dict = {
    "TriggerStatus":           2,     # 2 = laser ON, 1 = OFF
    "TriggerFrequencyHz":      40,
    "TriggerPulseWidthUsec":   500,
    "LaserPulseDelayUsec":     100,
    "LaserPulseWidthUsec":     500,
    "LaserPulseSkipInterval":  600,
    "LaserPulseSkipDelayUsec": 1800,
    "EnableSyncOut":           True,
    "EnableTaTrigger":         True,
}


def merge_trigger_config(*overrides) -> dict:
    """Shallow-merge a stack of trigger-config overrides on top of
    :data:`DEFAULT_TRIGGER_CONFIG`. Later args win over earlier ones;
    ``None`` entries are skipped. The result is a fresh dict — safe
    to mutate.

    Use this whenever a workflow needs to resolve 'the' trigger
    config from a request: caller passes
    ``merge_trigger_config(interface.default_trigger_config_override,
    request.trigger_config)`` and gets back a complete dict with all
    keys populated.
    """
    out: dict = dict(DEFAULT_TRIGGER_CONFIG)
    for override in overrides:
        if override:
            out.update(override)
    return out


# ---------------------------------------------------------------------------
# Drip-scan sweep retiming (camera-fpga#8, design spec 2026-07-19 §4.2).
#
# Ordered (register, value) writes for the OX02C1B, applied inside a sensor
# group-hold (0x3208) so they latch atomically at a frame boundary — see
# omotion/ImageCapture.py write_timing_profile(). Values are pinned by the
# cross-repo design; production values restore the shipped configuration in
# openmotion-sensor-fw Core/Inc/X02C1B_Sensor_Config.h (HTS/VTS lines 723-726,
# exposure 739-740, tc_r_initial 744-745).
# ---------------------------------------------------------------------------
OX02C1B_I2C_ADDR = 0x36
"""7-bit I2C address of the OX02C1B image sensor (see MotionSensor.camera_set_gain)."""

SWEEP_TIMING_PROFILE: tuple = (
    (0x380C, 0x8C), (0x380D, 0xA0),   # HTS = 36000 (~0.75 ms/row: drain margin ~9%; frame fits the console's 1.00 Hz trigger floor)
    (0x380E, 0x05), (0x380F, 0x20),   # VTS = 1312   (1280 active + minimal blanking; frame 0.987 s)
    (0x3501, 0x00), (0x3502, 0x01),   # exposure = 1 row (~0.75 ms shutter window)
)
"""Sweep (drip-scan) sensor timing (HTS/VTS/exposure — bench-verified values).

BENCH STATUS (2026-07-20, first HIL campaign): these timing values are correct
and the sensor accepts them, but the frame CADENCE mechanism at stretched
timing is still open: in the shipped trigger_mod streaming mode, changing VTS
stops FSIN-triggered frames (unresolved VTS-coupled register set — NB
0x3881-0x3883 is max_expo_a, NOT a sync point); the datasheet §3.7 snapshot
mode (0x3050=0x06) produces correct slow frames but gaps the MIPI clock,
which resets the camera FPGA (PLL-loss observed via STATUS bit0) and disarms
the sweep after one line. Resolution candidates: a clock-continuous trigger
register set, or a one-shot reset bridge in the FPGA so state survives clock
gaps. Do not expect capture_full_frames to complete until one lands."""

PRODUCTION_TIMING_PROFILE: tuple = (
    (0x380C, 0x01), (0x380D, 0xB0),   # HTS = 432
    (0x380E, 0x0A), (0x380F, 0xD0),   # VTS = 2768
    (0x3501, 0x00), (0x3502, 0x48),   # exposure = 72 rows
)
"""Shipped production timing (X02C1B_Sensor_Config.h) — restore after a capture.

tc_r_initial/0x388x registers are deliberately NOT touched in either profile:
production runs auto-tc_r (0x3823 bit4=0) and 0x3881-83 is max_expo_a
(bench-verified 2026-07-20; earlier spec/plan references to a "VTS-4 sync
point" were wrong)."""

SWEEP_FSIN_HZ: float = 1.0
"""FSIN trigger rate during a drip-scan capture.

Bench-pinned: the console firmware validates TriggerFrequencyHz to
1.00-100.00 Hz (console-fw trigger.c) and silently rejects lower, so 1.0 Hz
is the slowest possible external FSIN. The sweep profile (HTS=36000,
VTS=1312) gives a 0.987 s frame readout, inside the 1.000 s period."""

PRODUCTION_FSIN_HZ: float = 40.0
"""Normal histogram-mode FSIN rate (DEFAULT_TRIGGER_CONFIG TriggerFrequencyHz)."""

# ---------------------------------------------------------------------------
# 1 Hz full frames by stride composite (camera-fpga map v3, bench 2026-09-29,
# epic OpenwaterHealth/openmotion-bloodflow-app#480).
#
# Single-exposure drip-scan needs rows >= ~0.7 ms to drain each line over the
# FPGA->MCU link, and at that row time the OX02C1B's pixel noise is ~8x
# production (dark std ~140 DN vs 17, ~20% of pixels clipped to 0; measured
# std vs row time: 17 DN up to 27 us, 19 @ 42, 23 @ 84, 50 @ 167, 86 @ 335).
# The composite keeps production-quality 18 us rows and 40 Hz laser-synced
# frames; the FPGA's STRIDE register captures every COMPOSITE_STRIDE-th line
# with the phase advancing one line per frame, so COMPOSITE_STRIDE frames
# (1.000 s at 40 Hz) yield one complete 1920x1280 frame.
# ---------------------------------------------------------------------------
OX02C1B_MIN_VTS: int = 1378
"""Smallest VTS the sensor frames at (the datasheet default 0x562). Below it
the sensor stops producing frames in every mode (bench: 1360 stalls, 1378
runs) -- the real reason the original 1312-row sweep profile "stopped
framing" in trigger mode."""

COMPOSITE_STRIDE: int = 40
"""FPGA STRIDE for the 1 Hz composite: 40 x 18.1 us = 725 us between
captured lines, above the ~688 us line drain; 32 lines per frame."""

COMPOSITE_TIMING_PROFILE: tuple = (
    (0x380E, 0x05), (0x380F, 0x64),   # VTS = 1380 FIRST (see note)
    (0x380C, 0x03), (0x380D, 0x62),   # HTS = 866 (18.1 us rows; frame 24.98 ms < 25 ms FSIN)
    (0x3501, 0x00), (0x3502, 0x24),   # exposure = 36 rows (~650 us, as production)
    (0x3881, 0x00), (0x3882, 0x05), (0x3883, 0x64),   # max_expo_a = VTS (production keeps them equal)
)
"""Composite retiming, written register by register IN THIS ORDER (no group
hold). In trigger mode a frame longer than the FSIN period wedges framing, so
VTS must shrink before HTS grows: jumping straight from production passes
through HTS 866 x VTS 2768 = 50 ms and the sensor stops (bench-verified)."""

COMPOSITE_RESTORE_PROFILE: tuple = (
    (0x380C, 0x01), (0x380D, 0xB0),   # HTS = 432 FIRST (432 x 1380 fits the period)
    (0x380E, 0x0A), (0x380F, 0xD0),   # VTS = 2768
    (0x3501, 0x00), (0x3502, 0x48),   # exposure = 72 rows
    (0x3881, 0x00), (0x3882, 0x0A), (0x3883, 0xD0),   # max_expo_a = 2768
)
"""Back to the shipped timing (X02C1B_Sensor_Config.h), in the safe order."""
