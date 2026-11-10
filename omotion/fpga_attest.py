"""Console FPGA configuration attestation (``OW_CMD_FPGA_ATTEST``).

Console firmware with the PTR-2026-1-5 safety-state change reads the
configuration flash of the two laser-safety MachXO2 devices (EE and OPT) at
boot, hashes it with SHA-256 and compares the digest with the one compiled
into the signed image. It also validates each device's per-unit safety record
in UFM pages 0-1 and loads the EE/OPT limit registers from it. Until a run
passes, the trigger is refused with ``TRIG_INTERLOCK_FPGA_ATTEST`` (0x40). The
seed and TA drivers are reported "not checked".

The firmware, not the host, owns the safety limits on that firmware: the
registers in :data:`FIRMWARE_OWNED_REGISTERS` and the config keys in
:data:`CONFIG_SAFETY_KEYS` are refused over USB on Release and Debug images.
Design and wire format: openmotion-console-fw ``docs/fpga-attestation.md`` and
``Core/Inc/fpga_attest.h``.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional

__all__ = [
    "ATTEST_IDLE",
    "ATTEST_RUNNING",
    "ATTEST_PASS",
    "ATTEST_FAIL",
    "ATTEST_STATE_NAMES",
    "CONFIG_SAFETY_KEYS",
    "FACTORY_IMAGE_REQUIRED",
    "FIRMWARE_OWNED_REGISTERS",
    "HOST_LASER_REGISTERS",
    "FpgaAttestDevice",
    "FpgaAttestReport",
    "parse_attest_report",
]

# fpga_attest_state_t
ATTEST_IDLE = 0
ATTEST_RUNNING = 1
ATTEST_PASS = 2
ATTEST_FAIL = 3
ATTEST_STATE_NAMES = {
    ATTEST_IDLE: "not run",
    ATTEST_RUNNING: "running",
    ATTEST_PASS: "pass",
    ATTEST_FAIL: "fail",
}

# Per-device status bits (fpga_attest_dev_report_t.status).
ST_DIGEST = 0x01
ST_READ_FAIL = 0x02
ST_UFM_INVALID = 0x04
ST_UFM_BLANK = 0x08
ST_UFM_VALID = 0x10
ST_NOT_CHECKED = 0x40
ST_PENDING = 0x80
ST_FAULT_MASK = ST_DIGEST | ST_READ_FAIL | ST_UFM_INVALID | ST_UFM_BLANK

_STATUS_TEXT = (
    (ST_DIGEST, "configuration digest mismatch"),
    (ST_READ_FAIL, "configuration read failed"),
    (ST_UFM_INVALID, "safety record invalid"),
    (ST_UFM_BLANK, "no safety record (UFM blank)"),
    (ST_UFM_VALID, "safety record valid"),
    (ST_NOT_CHECKED, "not checked"),
    (ST_PENDING, "pending"),
)

# safety_ufm_status_t (fpga_attest_dev_report_t.ufm_status); 0xFF = not a
# safety device.
UFM_STATUS_NAMES = {
    0: "blank",
    1: "valid",
    2: "bad magic",
    3: "bad layout",
    4: "FPGA loader on defaults",
    5: "bad checksum",
    6: "bad reserved bytes",
    7: "outside the safety envelope",
}

#: Mux 0 channel -> device name, as in the report.
DEVICE_NAMES = {0: "seed", 1: "TA", 2: "EE", 3: "OPT"}

#: Laser-driver registers a host may still write on firmware that owns the
#: safety state; the console bounds each one (``safety_envelope.h`` and the
#: unit's safety record).
HOST_LASER_REGISTERS = frozenset(
    {"TA_PULSE_WIDTH", "TA_CURRENT_DRV", "SEED_DDS_GAIN", "SEED_CW_GAIN"}
)

#: Registers the firmware loads itself and refuses over ``OW_CTRL_I2C_WR`` on
#: Release and Debug images (``motion_cfg_keys.c``, ``safety_envelope.c``).
FIRMWARE_OWNED_REGISTERS = frozenset(
    {
        "EE_PULSE_WIDTH_LL", "EE_PULSE_WIDTH_UL", "EE_RATE_LL", "EE_DRIVE_CL",
        "EE_PWM_CURRENT", "EE_CW_CURRENT",
        "OPT_PULSE_WIDTH_LL", "OPT_PULSE_WIDTH_UL", "OPT_RATE_LL", "OPT_DRIVE_CL",
        "OPT_PWM_CURRENT", "OPT_CW_CURRENT",
        "TA_PERIOD", "TA_CURRENT_LIMIT", "TA_PWM_MON_CL", "TA_CW_MON_CL",
        "SEED_DDS_CL", "SEED_CW_CL",
    }
)

#: Config keys the console refuses with ``OW_CODE_CFG_SAFETY_KEY`` on Release
#: and Debug images. A Factory image accepts them within the envelope.
CONFIG_SAFETY_KEYS = FIRMWARE_OWNED_REGISTERS | frozenset(
    {"TEC_TRIP", "EE_THRESH", "EE_GAIN", "OPT_THRESH", "OPT_GAIN"}
)

#: Why a WI-00015 calibration refuses to start on a Release or Debug image.
FACTORY_IMAGE_REQUIRED = (
    "WI-00015 calibration writes the safety limits, which this console firmware "
    "accepts only on a Factory image: install the Factory build first "
    "(openmotion-console-fw docs/fpga-attestation.md section 6.4)."
)

# fpga_attest_report_t, packed little-endian.
_HEADER = struct.Struct("<BBBBHH")
_DEVICE = struct.Struct("<BBBBHHI32s32s")
_DEVICE_COUNT = 4


@dataclass(frozen=True)
class FpgaAttestDevice:
    device: int
    status: int
    ufm_status: int
    loader_status: int
    pages_read: int
    elapsed_ms: int
    devid: int
    digest: bytes
    expected: bytes

    @property
    def name(self) -> str:
        return DEVICE_NAMES.get(self.device, f"device {self.device}")

    @property
    def faulted(self) -> bool:
        return bool(self.status & ST_FAULT_MASK)

    @property
    def checked(self) -> bool:
        return not self.status & ST_NOT_CHECKED

    def describe(self) -> str:
        parts = [text for bit, text in _STATUS_TEXT if self.status & bit]
        if self.ufm_status != 0xFF and self.status & ST_UFM_INVALID:
            parts.append(
                UFM_STATUS_NAMES.get(self.ufm_status, f"UFM status {self.ufm_status}")
            )
        return f"{self.name}: " + (", ".join(parts) or "ok")


@dataclass(frozen=True)
class FpgaAttestReport:
    struct_version: int
    state: int
    progress_pct: int
    interlocked: bool
    run_count: int
    cfg_pages: int
    devices: tuple[FpgaAttestDevice, ...]

    @property
    def state_name(self) -> str:
        return ATTEST_STATE_NAMES.get(self.state, f"state {self.state}")

    @property
    def passed(self) -> bool:
        return self.state == ATTEST_PASS

    @property
    def done(self) -> bool:
        return self.state in (ATTEST_PASS, ATTEST_FAIL)

    def faults(self) -> list[str]:
        """One line per faulted device, empty when none."""
        return [d.describe() for d in self.devices if d.faulted]


def parse_attest_report(payload) -> Optional[FpgaAttestReport]:
    """Decode an ``OW_CMD_FPGA_ATTEST`` reply. ``None`` when it is too short."""
    if not payload:
        return None
    payload = bytes(payload)
    if len(payload) < _HEADER.size + _DEVICE_COUNT * _DEVICE.size:
        return None
    ver, state, pct, interlocked, runs, pages = _HEADER.unpack_from(payload)
    devices = tuple(
        FpgaAttestDevice(*_DEVICE.unpack_from(payload, _HEADER.size + i * _DEVICE.size))
        for i in range(_DEVICE_COUNT)
    )
    return FpgaAttestReport(ver, state, pct, bool(interlocked), runs, pages, devices)
