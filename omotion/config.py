from enum import IntEnum

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
OW_CAMERA_STREAM = 0x07


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

# Global Commands
OW_CMD_PING = 0x00
OW_CMD_VERSION = 0x02
OW_CMD_ECHO = 0x03
OW_CMD_TOGGLE_LED = 0x04
OW_CMD_HWID = 0x05
OW_CMD_MESSAGES = 0x09
OW_CMD_USR_CFG = 0x0A
OW_CMD_DFU = 0x0D
OW_CMD_NOP = 0x0E
OW_CMD_RESET = 0x0F
OW_CMD_I2C_BROADCAST = 0x06
OW_CMD_DEBUG_FLAGS = 0x0C

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
"""Cameras per sensor module (OV2312 array)."""

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
