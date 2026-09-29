"""Drip-scan full-frame single-exposure image capture (camera-fpga#8).

Host side of the drip-scan feature: the FPGA pushes each sensor row as a
2408-B packed-RAW10 line; sensor firmware forwards it blind on the HISTO USB
endpoint as a 2424-B TYPE_IMAGE (0x03) stream packet; this module parses,
CRC-verifies, and reassembles lines into 1280x1920 uint16 frames, retimes the
OX02C1B for the slow sweep, and orchestrates a capture end to end.

Wire contracts (must match the FPGA and sensor-fw companion implementations):

Line push (2408 B, FPGA -> MCU -> host, opaque to the MCU):
  [0]=magic 0xB6  [1]=version 0x01  [2]=line[7:0]  [3]={flags[3:0],line[11:8]}
  [4]=frame_cnt   [5]=0x00          [6..2405]=1920 px packed RAW10
  [2406..2407]=CRC-16 over bytes 0..2405, big-endian, CRC-CCITT-FALSE
  (poly 0x1021 / init 0xFFFF / MSB-first — byte-identical to sensor-fw
  utils.c util_crc16; computed here with binascii.crc_hqx, init 0xFFFF).
  flags bit0 = FPGA line-buffer overrun since sweep start.

RAW10 packing: 4 px -> 5 B; pixel k (k=0..3, readout order) occupies bits
[10k+9:10k] of a 40-bit little-endian group (low byte first on the wire).

USB stream envelope (2424 B, mirrors the histogram envelope conventions in
MotionProcessing.parse_histogram_packet_structured — including the 4-byte
FSIN timestamp):
  [0]=SOF 0xAA  [1]=TYPE_IMAGE 0x03  [2:6]=u32 LE total length (2424)
  [6:10]=FSIN timestamp (u32 LE, NOT parsed here)  [10]=SOH 0xFF  [11]=cam_id
  [12:2420]=line push  [2420]=EOH 0xEE
  [2421:2423]=transport CRC (NOT verified here — the MCU forwards blind, the
  line CRC above is the authoritative integrity check)  [2423]=EOF 0xDD
"""

import binascii
import json
import logging
import queue as _queue
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from omotion import _log_root
from omotion.config import (
    OX02C1B_I2C_ADDR,
    PRODUCTION_TIMING_PROFILE,
    SWEEP_FSIN_HZ,
    SWEEP_TIMING_PROFILE,
)
from omotion.i2c_packet import I2C_Packet

logger = logging.getLogger(
    f"{_log_root}.ImageCapture" if _log_root else "ImageCapture"
)

# --- Line-push geometry (pinned by the design spec) ------------------------
IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1280
IMAGE_LINE_MAGIC = 0xB6
IMAGE_LINE_VERSION = 0x01
IMAGE_LINE_PIXEL_BYTES = IMAGE_WIDTH * 5 // 4          # 2400
IMAGE_LINE_SIZE = 6 + IMAGE_LINE_PIXEL_BYTES + 2       # 2408
FLAG_OVERRUN = 0x1                                     # header flags bit0
FLAG_WEDGE = 0x2                                       # header flags bit1 — pusher-watchdog wedge

# --- USB envelope (contract B) ---------------------------------------------
_ENV_SOF, _ENV_SOH, _ENV_EOH, _ENV_EOF = 0xAA, 0xFF, 0xEE, 0xDD
IMAGE_PACKET_SIZE = 6 + 4 + 1 + 1 + IMAGE_LINE_SIZE + 1 + 3  # 2424


class ImageLineError(ValueError):
    """A line/packet failed framing, header, or CRC validation."""


# ---------------------------------------------------------------------------
# RAW10 pack / unpack
# ---------------------------------------------------------------------------

def unpack_raw10(packed) -> np.ndarray:
    """Unpack RAW10 bytes (4 px -> 5 B little-endian groups) to uint16 pixels.

    Vectorized: reshape to (n_groups, 5), rebuild each 40-bit group value,
    then extract the four 10-bit fields at shifts 0/10/20/30.
    """
    raw = np.frombuffer(bytes(packed), dtype=np.uint8)
    if raw.size == 0 or raw.size % 5:
        raise ValueError(
            f"packed RAW10 length {raw.size} is not a positive multiple of 5"
        )
    g = raw.reshape(-1, 5).astype(np.uint64)
    v = (g[:, 0]
         | (g[:, 1] << np.uint64(8))
         | (g[:, 2] << np.uint64(16))
         | (g[:, 3] << np.uint64(24))
         | (g[:, 4] << np.uint64(32)))
    shifts = (np.arange(4, dtype=np.uint64) * np.uint64(10))[None, :]
    px = ((v[:, None] >> shifts) & np.uint64(0x3FF)).astype(np.uint16)
    return px.reshape(-1)


def pack_raw10(pixels) -> bytes:
    """Reference packer — exact inverse of :func:`unpack_raw10`.

    Used by the test suite to synthesize wire-true lines and by bench tooling
    to build golden inputs. Not performance-critical.
    """
    px = np.asarray(pixels, dtype=np.uint64)
    if px.size == 0 or px.size % 4:
        raise ValueError(f"pixel count {px.size} is not a positive multiple of 4")
    px = px.reshape(-1, 4) & np.uint64(0x3FF)
    v = (px[:, 0] | (px[:, 1] << np.uint64(10))
         | (px[:, 2] << np.uint64(20)) | (px[:, 3] << np.uint64(30)))
    out = bytearray()
    for val in v:
        out += int(val).to_bytes(5, "little")
    return bytes(out)


# ---------------------------------------------------------------------------
# Line / packet parsing
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ImageLine:
    cam_id: int
    line: int
    flags: int
    overrun: bool
    wedge: bool
    frame_cnt: int
    pixels: np.ndarray   # uint16[IMAGE_WIDTH]


def parse_image_line(line_bytes, cam_id: int = -1) -> ImageLine:
    """Validate and decode one 2408-B line push. Raises ImageLineError."""
    b = bytes(line_bytes)
    if len(b) != IMAGE_LINE_SIZE:
        raise ImageLineError(
            f"line length {len(b)} != {IMAGE_LINE_SIZE}"
        )
    if b[0] != IMAGE_LINE_MAGIC:
        raise ImageLineError(f"bad magic 0x{b[0]:02X} (expected 0xB6)")
    if b[1] != IMAGE_LINE_VERSION:
        raise ImageLineError(f"bad format version 0x{b[1]:02X} (expected 0x01)")
    crc_expected = (b[IMAGE_LINE_SIZE - 2] << 8) | b[IMAGE_LINE_SIZE - 1]
    # binascii.crc_hqx with init 0xFFFF IS CRC-16/CCITT-FALSE (== util_crc16,
    # check value 0x29B1) at C speed: ~5 us/line vs ~400 us for the
    # pure-Python table loop, which at ~1.3 k lines/s starved the USB reader
    # thread of the GIL and overflowed the firmware's staging buffer.
    crc_actual = binascii.crc_hqx(b[: IMAGE_LINE_SIZE - 2], 0xFFFF)
    if crc_actual != crc_expected:
        raise ImageLineError(
            f"line CRC mismatch (got 0x{crc_actual:04X}, "
            f"expected 0x{crc_expected:04X})"
        )
    line = b[2] | ((b[3] & 0x0F) << 8)
    flags = (b[3] >> 4) & 0x0F
    return ImageLine(
        cam_id=cam_id,
        line=line,
        flags=flags,
        overrun=bool(flags & FLAG_OVERRUN),
        wedge=bool(flags & FLAG_WEDGE),
        frame_cnt=b[4],
        pixels=unpack_raw10(b[6 : 6 + IMAGE_LINE_PIXEL_BYTES]),
    )


def parse_image_packet(pkt) -> ImageLine:
    """Validate the 2424-B USB envelope and decode the line inside.

    The envelope transport-CRC field is intentionally NOT verified: the MCU
    forwards image lines blind (spec §4.1/§4.4) and the FPGA-computed line CRC
    inside the payload is the authoritative integrity check. The 4-byte FSIN
    timestamp at [6:10] (same convention as the histogram envelope this
    mirrors) is likewise not surfaced here.
    """
    b = bytes(pkt)
    if len(b) != IMAGE_PACKET_SIZE:
        raise ImageLineError(
            f"image packet length {len(b)} != {IMAGE_PACKET_SIZE}"
        )
    if b[0] != _ENV_SOF or b[1] != 0x03:
        raise ImageLineError(
            f"bad envelope header {b[0]:02X} {b[1]:02X} (expected AA 03)"
        )
    total = int.from_bytes(b[2:6], "little")
    if total != IMAGE_PACKET_SIZE:
        raise ImageLineError(f"envelope length field {total} != {IMAGE_PACKET_SIZE}")
    if b[10] != _ENV_SOH:
        raise ImageLineError("missing SOH")
    if b[12 + IMAGE_LINE_SIZE] != _ENV_EOH:
        raise ImageLineError("missing EOH")
    if b[-1] != _ENV_EOF:
        raise ImageLineError("missing EOF")
    return parse_image_line(b[12 : 12 + IMAGE_LINE_SIZE], cam_id=b[11])


# ---------------------------------------------------------------------------
# Frame assembly
# ---------------------------------------------------------------------------

class FrameAssembler:
    """Ordered reassembly of one 1280x1920 uint16 frame from ImageLines.

    Thread-safe (the collector thread adds lines while the orchestrator polls
    completeness — same cross-thread pattern as the rest of the SDK transport
    layer).

    Single-exposure enforcement: the first accepted line pins ``frame_cnt``;
    lines carrying a different value are rejected (counted in
    ``rejected_lines``) unless ``allow_mixed`` is set, in which case they fill
    their gap and the result is flagged ``mixed_exposure`` — the frame is then
    usable for focus inspection but is NOT a single-exposure speckle frame.
    """

    def __init__(self, height: int = IMAGE_HEIGHT, width: int = IMAGE_WIDTH,
                 allow_mixed: bool = False):
        self.height = height
        self.width = width
        self.allow_mixed = allow_mixed
        self._lock = threading.Lock()
        self._img = np.zeros((height, width), dtype=np.uint16)
        self._filled = np.zeros(height, dtype=bool)
        self._frame_cnts: set[int] = set()
        self.frame_cnt: int | None = None
        self.rejected_lines = 0
        self.overrun_seen = False
        self.wedge_seen = False

    def add(self, line: ImageLine) -> bool:
        """Accept one parsed line. Returns True if it was placed."""
        with self._lock:
            if not (0 <= line.line < self.height):
                self.rejected_lines += 1
                logger.warning("cam %d: line %d out of range — rejected",
                               line.cam_id, line.line)
                return False
            if self.frame_cnt is None:
                self.frame_cnt = line.frame_cnt
            elif line.frame_cnt != self.frame_cnt and not self.allow_mixed:
                self.rejected_lines += 1
                logger.warning(
                    "cam %d: line %d frame_cnt 0x%02X != pinned 0x%02X — "
                    "rejected (single-exposure enforcement)",
                    line.cam_id, line.line, line.frame_cnt, self.frame_cnt)
                return False
            if line.overrun:
                self.overrun_seen = True
            if line.wedge:
                self.wedge_seen = True
            self._frame_cnts.add(line.frame_cnt)
            self._img[line.line] = line.pixels
            self._filled[line.line] = True
            return True

    def missing(self) -> list[int]:
        with self._lock:
            return [int(i) for i in np.nonzero(~self._filled)[0]]

    @property
    def complete(self) -> bool:
        with self._lock:
            return bool(self._filled.all())

    @property
    def mixed_exposure(self) -> bool:
        with self._lock:
            return len(self._frame_cnts) > 1

    def image(self) -> np.ndarray:
        """Copy of the frame so far (unfilled rows are zero)."""
        with self._lock:
            return self._img.copy()

    def reset(self) -> None:
        """Discard everything and start a fresh exposure (strict retry)."""
        with self._lock:
            self._img.fill(0)
            self._filled.fill(False)
            self._frame_cnts.clear()
            self.frame_cnt = None
            self.rejected_lines = 0
            self.overrun_seen = False
            self.wedge_seen = False


# ---------------------------------------------------------------------------
# Camera-FPGA register access (I2C 0x5A, register map v2)
#
# Ported from openmotion-camera-fpga tools/full_frame_capture/fpga_link.py.
# Reads use MotionSensor.i2c_read_register directly; writes use the same
# passthrough with the 16-bit-register-address trick (reg_addr_size=2: the
# slave interprets the high byte as the register pointer and the low byte as
# a data write — see the feature/5 design spec).
#
# NOTE: the FPGA control plane is clocked from the MIPI-derived pixel clock —
# register access only works while the camera is streaming (enable first).
# ---------------------------------------------------------------------------

FPGA_I2C_ADDR = 0x5A
REG_ID, REG_VERSION, REG_SCRATCH, REG_CTRL = 0x00, 0x01, 0x02, 0x03
REG_LINE_L, REG_LINE_H, REG_LINE_CUR_L, REG_LINE_CUR_H = 0x04, 0x05, 0x06, 0x07
REG_FRAME_CNT, REG_STATUS = 0x08, 0x09
REG_STRIDE = 0x0A                # map v3: stride composite
FPGA_ID_VAL = 0x5A
FPGA_MAP_VERSION_MIN = 0x02      # map v2 = drip-scan capable
FPGA_MAP_VERSION_STRIDE = 0x03   # map v3 = STRIDE register (1 Hz composite)
CTRL_IMAGE_MODE = 0x01           # CTRL bit0
CTRL_SWEEP = 0x02                # CTRL bit1 — valid only with bit0, sampled at fv
STATUS_OVERRUN = 0x04            # STATUS bit2 — overrun latch, cleared on sweep arm
STATUS_WEDGE = 0x08             # STATUS bit3 — pusher-watchdog wedge latch


class FpgaRegs:
    """Register access to one camera FPGA through the sensor firmware's
    I2C passthrough (MotionSensor.i2c_read_register)."""

    def __init__(self, sensor, cam: int):
        self.sensor = sensor
        self.cam = cam

    def read(self, reg: int) -> int:
        r = self.sensor.i2c_read_register(
            FPGA_I2C_ADDR, reg, read_len=1, reg_addr_size=1,
            mux_channel=self.cam)
        if r is False or r is None:
            raise IOError(f"cam{self.cam}: I2C read reg 0x{reg:02X} failed")
        return r[0]

    def write(self, reg: int, value: int) -> None:
        r = self.sensor.i2c_read_register(
            FPGA_I2C_ADDR, ((reg & 0xFF) << 8) | (value & 0xFF),
            read_len=1, reg_addr_size=2, mux_channel=self.cam)
        if r is False or r is None:
            raise IOError(f"cam{self.cam}: I2C write reg 0x{reg:02X} failed")

    def check_id(self) -> bool:
        try:
            return self.read(REG_ID) == FPGA_ID_VAL
        except IOError:
            return False

    def check_version(self) -> bool:
        """True if the loaded bitstream speaks register map v2 (drip-scan)."""
        try:
            return self.read(REG_VERSION) >= FPGA_MAP_VERSION_MIN
        except IOError:
            return False

    def stride_capable(self) -> bool:
        """True if the loaded bitstream has the map-v3 STRIDE register."""
        try:
            return self.read(REG_VERSION) >= FPGA_MAP_VERSION_STRIDE
        except IOError:
            return False

    def set_stride(self, stride: int) -> None:
        """Map v3: sweep captures lines phase + j*stride, phase advancing one
        line per frame (0 on the arming frame). 0/1 = every line (v2).
        Quasi-static: write it before arming the sweep."""
        if not 0 <= stride <= 255:
            raise ValueError(f"stride {stride} out of range 0..255")
        self.write(REG_STRIDE, stride)

    def quiet(self) -> None:
        """Image mode, no sweep, single-line target 4095 (never reached): the
        FPGA pushes nothing. Keeps the link silent while the host retimes the
        sensor -- a 4100-B legacy or histogram push would misalign the
        firmware's 2-line receive ring (it resyncs, but costs a gap)."""
        self.set_start_line(4095)
        self.write(REG_CTRL, CTRL_IMAGE_MODE)

    def set_start_line(self, line: int) -> None:
        """Map v2: LINE_L/H hold the sweep start line (12-bit)."""
        self.write(REG_LINE_L, line & 0xFF)
        self.write(REG_LINE_H, (line >> 8) & 0x0F)

    def arm_sweep(self, start_line: int = 0) -> None:
        """Arm the sweep at ``start_line``. The FPGA samples CTRL at the
        frame-valid boundary and clears the overrun latch on arm; every
        subsequent frame pushes all lines >= start_line (every STRIDE-th in
        map v3 stride mode).

        Order matters (bench 2026-09-29): in SINGLE-line image mode the FPGA
        captures its target line each frame, pushes a 4100-B legacy packet
        and auto-increments the target. Writing the start line first and the
        sweep bit ~20-50 ms later let a frame slip in between: line 0 got
        pushed as a legacy packet (costing the firmware's line ring a
        resync), the target became 1, and the sweep then never captured row
        0 for the whole session. So: park the target out of reach (4095),
        set the sweep bit -- in sweep mode the target never auto-increments
        -- and only then lower the start line."""
        self.set_start_line(4095)
        self.write(REG_CTRL, CTRL_IMAGE_MODE | CTRL_SWEEP)
        self.set_start_line(start_line)

    def stop_sweep(self) -> None:
        """Stop sweeping but stay in image mode (no further line pushes)."""
        self.write(REG_CTRL, CTRL_IMAGE_MODE)

    def exit_image_mode(self) -> None:
        """Back to histogram mode. Host rule (feature/5, unchanged): the first
        histogram frame after leaving image mode is garbage — discard it."""
        self.write(REG_CTRL, 0x00)

    def frame_count(self) -> int:
        return self.read(REG_FRAME_CNT)

    def overrun(self) -> bool:
        """STATUS bit2: a line was dropped since the last sweep arm."""
        return bool(self.read(REG_STATUS) & STATUS_OVERRUN)

    def wedge(self) -> bool:
        """STATUS bit3 (map v2, spec §4.3): the pusher watchdog aborted a push
        with no serializer progress since the last sweep arm — an
        electrical/SEU wedge, distinct from a timing overrun (bit2)."""
        return bool(self.read(REG_STATUS) & STATUS_WEDGE)


# ---------------------------------------------------------------------------
# Sensor sweep retiming (group-hold, spec §4.2)
# ---------------------------------------------------------------------------

# OX02C1B group access register (OmniVision datasheet idiom; the shipped
# config table X02C1B_Sensor_Config.h never touches it — drip-scan is the
# first user in this system). Writes bracketed by HOLD_START/HOLD_END land in
# group 0's shadow bank; DELAYED_LAUNCH latches the whole group atomically at
# the next frame boundary. Atomicity matters: tc_r_initial (FSIN slave
# timing) is VTS-coupled and must never be visible with a mismatched VTS.
GROUP_ACCESS_REG = 0x3208
GROUP0_HOLD_START = 0x00
GROUP0_HOLD_END = 0x10
GROUP0_DELAYED_LAUNCH = 0xA0

# Settle delay between passthrough writes — same pacing MotionSensor uses for
# its own multi-write register sequences (camera_set_gain / camera_set_exposure).
_I2C_WRITE_SETTLE_S = 0.02


def write_timing_profile(sensor, cam: int, profile) -> bool:
    """Write one timing profile (config.SWEEP_TIMING_PROFILE or
    config.PRODUCTION_TIMING_PROFILE) to one camera as a single atomic
    group-hold, via the OW_I2C_PASSTHRU path (MotionSensor.camera_i2c_write).

    The new timing takes effect at the camera's NEXT frame boundary — after a
    restore at the 0.8 Hz sweep rate, wait up to one sweep period (1.25 s)
    before assuming production timing is live.

    Returns True only if every write acknowledged.
    """
    sensor.switch_camera(cam)
    sequence = (
        (GROUP_ACCESS_REG, GROUP0_HOLD_START),
        *profile,
        (GROUP_ACCESS_REG, GROUP0_HOLD_END),
        (GROUP_ACCESS_REG, GROUP0_DELAYED_LAUNCH),
    )
    ok = True
    for reg, val in sequence:
        ok = sensor.camera_i2c_write(
            I2C_Packet(device_address=OX02C1B_I2C_ADDR,
                       register_address=reg, data=val)
        ) and ok
        time.sleep(_I2C_WRITE_SETTLE_S)
    if not ok:
        logger.error("cam %d: timing-profile group write failed", cam)
    return ok


# ---------------------------------------------------------------------------
# Capture orchestration (graduates camera-fpga tools/full_frame_capture/)
# ---------------------------------------------------------------------------

# USB read size for the stream loop during an image session: the HISTO
# endpoint's max transfer (USB_HISTO_MAX_SIZE in sensor-fw usbd_histo.h).
# Image packets are 2424 B each; a single read may deliver one or several.
_STREAM_READ_SIZE = 32837

_SWEEP_PERIOD_S = 1.0 / SWEEP_FSIN_HZ   # 1.25 s per exposure at 0.8 Hz


def next_sweep_action(missing, attempt, max_sweeps, mixed_fill_sweeps):
    """Decide what the next sweep attempt should do for one camera.

    Returns (action, start_line):
      ("done", None)      — frame complete, stop.
      ("restart", 0)      — strict single-exposure retry: discard partial
                            assembly and re-sweep the whole frame.
      ("fill", first_gap) — mixed-exposure fallback for the last
                            ``mixed_fill_sweeps`` attempts: keep what we have,
                            re-sweep from the first missing line only.
      ("give_up", None)   — attempt budget exhausted.
    """
    if not missing:
        return ("done", None)
    if attempt > max_sweeps:
        return ("give_up", None)
    if attempt > max_sweeps - mixed_fill_sweeps:
        return ("fill", missing[0])
    return ("restart", 0)


@dataclass
class CameraCaptureResult:
    cam_id: int
    image: np.ndarray | None
    complete: bool
    mixed_exposure: bool
    missing_lines: list[int] = field(default_factory=list)
    frame_cnt: int | None = None
    overrun: bool = False
    rejected_lines: int = 0
    attempts: int = 0


def _collector_loop(image_queue, assemblers, stop_evt):
    """Drain the image queue into per-camera assemblers until stopped AND
    empty. Lines that fail CRC/framing are dropped and logged — the sweep
    retry policy re-requests whatever ends up missing."""
    while not stop_evt.is_set() or not image_queue.empty():
        try:
            pkt = image_queue.get(timeout=0.2)
        except _queue.Empty:
            continue
        try:
            line = parse_image_packet(pkt)
        except ImageLineError as exc:
            logger.warning("dropping bad image packet: %s", exc)
            continue
        asm = assemblers.get(line.cam_id)
        if asm is not None:
            asm.add(line)


def capture_full_frames(
    sensor,
    console,
    cams,
    out_dir=None,
    side: str = "left",
    max_sweeps: int = 6,
    mixed_fill_sweeps: int = 2,
    settle_timeout_s: float = 3.0,
) -> dict[int, CameraCaptureResult]:
    """Capture one full-frame single-exposure image from each camera in
    ``cams`` on one sensor module.

    Sequence (spec §4.5): enable histogram streaming (brings up the MIPI
    clock the FPGA control plane needs) -> enter image mode
    (OW_CAMERA_IMAGE_MODE; firmware suspends histograms and arms 2408-B line
    DMA) -> group-hold retime to the sweep profile -> slow FSIN to 0.8 Hz via
    the console trigger config (MotionConsole.set_trigger_json,
    TriggerFrequencyHz — the SDK's only FSIN-rate setter) -> collect with
    per-frame retry via the FPGA sweep start-line register -> restore timing
    and FSIN -> exit image mode -> discard the first (garbage) histogram
    frame accumulated across the session.

    ``console`` is required: SyncOut drives the sensors' FSIN on this
    hardware, and laser per-pulse parameters ride the existing trigger config
    untouched (only the repetition rate changes).

    Outputs (when ``out_dir`` is given): ``{side}_cam{c}.npy`` (uint16
    1280x1920) and, if PIL is importable (it ships transitively with the
    declared matplotlib dependency), a 16-bit ``{side}_cam{c}.png``; plus
    ``meta.json`` with per-camera capture status.
    """
    mask = 0
    for c in cams:
        mask |= 1 << c

    histo_if = sensor.uart.histo
    image_q: _queue.Queue = _queue.Queue()
    discard_q: _queue.Queue = _queue.Queue()   # stray histogram packets, dropped
    assemblers = {c: FrameAssembler() for c in cams}
    results: dict[int, CameraCaptureResult] = {}
    stop_evt = threading.Event()

    saved_trigger = console.get_trigger_json()
    if saved_trigger is None:
        raise RuntimeError(
            f"{side}: get_trigger_json() returned None — cannot capture "
            f"without the console trigger config")
    if isinstance(saved_trigger, str):
        saved_trigger = json.loads(saved_trigger)

    histo_if.flush_stale_data(expected_size=_STREAM_READ_SIZE)
    histo_if.start_streaming(discard_q, _STREAM_READ_SIZE, image_queue=image_q)
    collector = threading.Thread(
        target=_collector_loop, args=(image_q, assemblers, stop_evt),
        daemon=True)
    collector.start()

    trigger_started = False
    image_mode_on = False
    try:
        if not sensor.enable_camera(mask):
            raise RuntimeError(f"{side}: enable_camera(0x{mask:02X}) failed")
        time.sleep(1.0)   # MIPI clock + FPGA control plane come up with streaming

        # Drop cameras whose FPGA is absent or not drip-scan capable.
        regs = {}
        for c in list(cams):
            r = FpgaRegs(sensor, c)
            if not r.check_id():
                logger.warning("[%s] cam%d: FPGA control plane not answering "
                               "— skipped", side, c)
                del assemblers[c]
                continue
            if not r.check_version():
                logger.warning("[%s] cam%d: FPGA register map < v2 (no "
                               "drip-scan) — skipped", side, c)
                del assemblers[c]
                continue
            regs[c] = r
        active = sorted(regs)
        if not active:
            raise RuntimeError(f"{side}: no drip-scan-capable cameras")

        if not sensor.set_camera_image_mode(True, mask):
            raise RuntimeError(f"{side}: OW_CAMERA_IMAGE_MODE enable failed")
        image_mode_on = True

        for c in active:
            if not write_timing_profile(sensor, c, SWEEP_TIMING_PROFILE):
                raise RuntimeError(f"{side}: cam{c} sweep retime failed")

        slow = dict(saved_trigger)
        slow["TriggerFrequencyHz"] = SWEEP_FSIN_HZ
        # Bench-pinned: the console BOOTS with EnableSyncOut=false (console-fw
        # main.c) — without it no FSIN reaches the sensors and every sweep
        # returns zero lines. The restore path writes back saved_trigger
        # verbatim, so the inherited setting is preserved on exit.
        slow["EnableSyncOut"] = True
        if not console.set_trigger_json(data=slow):
            raise RuntimeError("set_trigger_json (sweep rate) failed")
        if not sensor.enable_camera_fsin_ext():
            raise RuntimeError(f"{side}: enable_camera_fsin_ext failed")
        if not console.start_trigger():
            raise RuntimeError("start_trigger failed")
        trigger_started = True
        # One frame at the old timing may still be in flight; the group-hold
        # launches at its boundary. From here every FSIN is a sweep exposure.

        attempts = {c: 0 for c in active}
        pending = set(active)
        while pending:
            for c in sorted(pending):
                attempts[c] += 1
                action, start_line = next_sweep_action(
                    assemblers[c].missing(), attempts[c],
                    max_sweeps, mixed_fill_sweeps)
                if action == "restart":
                    assemblers[c].reset()
                    regs[c].arm_sweep(0)
                elif action == "fill":
                    assemblers[c].allow_mixed = True
                    regs[c].arm_sweep(start_line)
                elif action == "give_up":
                    logger.error("[%s] cam%d: incomplete after %d sweeps "
                                 "(%d lines missing)", side, c, max_sweeps,
                                 len(assemblers[c].missing()))
                    regs[c].stop_sweep()
                    pending.discard(c)
            if not pending:
                break
            # One exposure + full drain per attempt, with settle margin.
            deadline = time.monotonic() + 2 * _SWEEP_PERIOD_S + settle_timeout_s
            while time.monotonic() < deadline:
                time.sleep(0.25)
                if all(assemblers[c].complete for c in pending):
                    break
            for c in [c for c in pending if assemblers[c].complete]:
                regs[c].stop_sweep()
                pending.discard(c)

        for c in active:
            asm = assemblers[c]
            results[c] = CameraCaptureResult(
                cam_id=c,
                image=asm.image(),
                complete=asm.complete,
                mixed_exposure=asm.mixed_exposure,
                missing_lines=asm.missing(),
                frame_cnt=asm.frame_cnt,
                overrun=asm.overrun_seen,
                rejected_lines=asm.rejected_lines,
                attempts=attempts[c],
            )
    finally:
        # --- Restore, tolerating partial bring-up ------------------------
        # Every device-restore step is attempted independently, and the
        # thread/stream teardown in the inner ``finally`` ALWAYS runs — even if
        # a mid-capture disconnect makes a device call raise (a lost transport
        # raises ValueError from _send, and stop_trigger re-raises). Without
        # this guarantee an escaping raise here would orphan the daemon
        # collector thread and leave histo streaming armed.
        try:
            try:
                for c in sorted(assemblers):
                    write_timing_profile(sensor, c, PRODUCTION_TIMING_PROFILE)
                # Group launch happens at the next frame boundary — at the
                # sweep rate that is up to one 1.25 s period away.
                time.sleep(_SWEEP_PERIOD_S + 0.25)
                for c in sorted(assemblers):
                    try:
                        FpgaRegs(sensor, c).exit_image_mode()
                    except IOError:
                        pass
            except Exception:
                logger.exception("%s: timing/FPGA restore failed", side)
            if trigger_started:
                try:
                    console.stop_trigger()
                except Exception:
                    logger.exception("%s: stop_trigger failed", side)
            try:
                console.set_trigger_json(data=saved_trigger)
            except Exception:
                logger.exception("restoring trigger config failed")
            if image_mode_on:
                try:
                    sensor.set_camera_image_mode(False, mask)
                except Exception:
                    logger.exception("%s: exit image mode failed", side)
            try:
                sensor.disable_camera_fsin_ext()
            except Exception:
                logger.exception("%s: disable_camera_fsin_ext failed", side)
            try:
                sensor.disable_camera(mask)
            except Exception:
                logger.exception("%s: disable_camera failed", side)
        finally:
            # Resource teardown MUST run regardless of any device-call failure
            # above: stop the collector and streaming so the daemon thread can
            # never be orphaned and the histo endpoint is never left armed.
            stop_evt.set()
            try:
                histo_if.stop_streaming()
            except Exception:
                logger.exception("%s: stop_streaming failed", side)
            # The first histogram frame after an image session carries counts
            # accumulated across the whole session (spec §4.4) — drain and
            # discard anything already in flight so the next scan starts clean.
            try:
                discarded = histo_if.drain_final(expected_size=_STREAM_READ_SIZE)
                if discarded:
                    logger.info("%s: discarded %d post-image-session chunk(s) "
                                "(first histogram frame after image mode is "
                                "garbage)", side, len(discarded))
            except Exception:
                pass
            collector.join(timeout=3.0)

    if out_dir is not None:
        _save_outputs(results, Path(out_dir), side)
    return results


def _save_outputs(results, out_dir: Path, side: str) -> None:
    """Write {side}_cam{c}.npy (+16-bit PNG when PIL is available) and merge
    per-camera status into meta.json — same shape as the retired
    tools/full_frame_capture/capture.py outputs."""
    out_dir.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image          # transitively present via matplotlib
    except ImportError:                # pragma: no cover - env-dependent
        Image = None
        logger.warning("PIL not importable — writing .npy only (PNG skipped)")

    meta_path = out_dir / "meta.json"
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    meta.update({
        "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "width": IMAGE_WIDTH, "height": IMAGE_HEIGHT, "bit_depth": 10,
        "scaling": "none — raw 10-bit sensor values 0..1023 in 16-bit files",
    })
    meta.setdefault("cameras", {})
    for c, res in sorted(results.items()):
        key = f"{side}_cam{c}"
        np.save(out_dir / f"{key}.npy", res.image)
        if Image is not None:
            Image.fromarray(res.image).save(out_dir / f"{key}.png")
        meta["cameras"][key] = {
            "complete": res.complete,
            "single_exposure": res.complete and not res.mixed_exposure,
            "mixed_exposure": res.mixed_exposure,
            "frame_cnt": res.frame_cnt,
            "missing_lines": res.missing_lines,
            "overrun": res.overrun,
            "rejected_lines": res.rejected_lines,
            "sweep_attempts": res.attempts,
        }
    meta_path.write_text(json.dumps(meta, indent=2))


# ---------------------------------------------------------------------------
# 1 Hz full frames: stride composite (camera-fpga map v3, bench 2026-09-29)
#
# Single-exposure sweeps need ~0.7 ms rows and the OX02C1B's pixel noise is
# ~8x production at that row time (see config.COMPOSITE_STRIDE notes). The
# composite keeps production 40 Hz laser-synced frames with 18 us rows; the
# FPGA sends lines phase, phase+STRIDE, ... with the phase advancing one line
# per frame, and the host assembles STRIDE consecutive frames into one image.
# ---------------------------------------------------------------------------

@dataclass
class CompositeFrame:
    """One assembled full frame. ``frame_cnts`` are the FPGA frame counters
    (8-bit) of the exposures it was built from; with an intact stream that is
    exactly ``stride`` consecutive frames."""
    cam_id: int
    image: np.ndarray               # uint16[1280, 1920], raw 10-bit values
    t_first: float                  # host monotonic time of the first line
    t_last: float                   # ... and of the line that completed it
    frame_cnts: list[int]
    overrun: bool
    lines: int                      # line pushes consumed (1280 when clean)


class CompositeAssembler:
    """Row-by-row assembly of stride composites.

    Every accepted line overwrites its row; when all rows have been written
    since the previous emission, a :class:`CompositeFrame` is returned and the
    fill map clears. Fed in arrival order from a sweep armed at phase 0, the
    emission lands on the last line of phase STRIDE-1, so the next frame
    (phase 0) starts a fresh composite. A dropped line delays its composite
    by one cycle (that row is then one cycle newer than the rest -- the
    returned ``frame_cnts`` span shows it).
    """

    def __init__(self, height: int = IMAGE_HEIGHT, width: int = IMAGE_WIDTH,
                 skip_exposures: int = 0):
        """``skip_exposures``: ignore lines from the first N exposures (FPGA
        frame counters) seen -- the laser needs a few pulses after the
        trigger starts (bench: first exposure black, next two ~5% dim)."""
        self.height = height
        self.width = width
        self._img = np.zeros((height, width), dtype=np.uint16)
        self._filled = np.zeros(height, dtype=bool)
        self._skip_left = int(skip_exposures)
        self._skip_fc: int | None = None
        self._reset_cycle()

    def _reset_cycle(self) -> None:
        self._filled.fill(False)
        self._frame_cnts: list[int] = []
        self._overrun = False
        self._lines = 0
        self._t_first: float | None = None

    def add(self, line: ImageLine, t: float) -> "CompositeFrame | None":
        """Place one line (arrival time ``t``). Returns a CompositeFrame when
        this line completes one, else None."""
        if not 0 <= line.line < self.height:
            logger.warning("cam %d: line %d out of range -- ignored",
                           line.cam_id, line.line)
            return None
        if self._skip_left > 0:
            if self._skip_fc is None:
                self._skip_fc = line.frame_cnt
            elif line.frame_cnt != self._skip_fc:
                self._skip_left -= 1
                self._skip_fc = line.frame_cnt
            if self._skip_left > 0:
                return None
        if self._t_first is None:
            self._t_first = t
        if not self._frame_cnts or self._frame_cnts[-1] != line.frame_cnt:
            self._frame_cnts.append(line.frame_cnt)
        self._overrun |= line.overrun
        self._lines += 1
        self._img[line.line] = line.pixels
        self._filled[line.line] = True
        if not self._filled.all():
            return None
        out = CompositeFrame(cam_id=line.cam_id, image=self._img.copy(), t_first=self._t_first,
                             t_last=t, frame_cnts=list(self._frame_cnts),
                             overrun=self._overrun, lines=self._lines)
        self._reset_cycle()
        return out


def write_register_sequence(sensor, cam: int, seq,
                            dev_addr: int = OX02C1B_I2C_ADDR) -> bool:
    """Plain ordered I2C writes (no group hold). Order is the point: see
    config.COMPOSITE_TIMING_PROFILE."""
    sensor.switch_camera(cam)
    ok = True
    for reg, val in seq:
        ok = sensor.camera_i2c_write(
            I2C_Packet(device_address=dev_addr, register_address=reg, data=val)
        ) and ok
        time.sleep(_I2C_WRITE_SETTLE_S)
    return ok


def force_load_fpga(sensor, camera_mask: int, timeout: int = 120) -> bool:
    """Force a camera-FPGA SRAM load from the sensor's flash-resident
    bitstream (OW_FPGA_PROG_SRAM, reserved=2). The fleet cameras boot their
    NVCM (production) image; the drip-scan/stride bitstream only runs after a
    forced SRAM load, and is lost at camera power-off. ~10 s per camera."""
    from omotion.config import OW_FPGA, OW_FPGA_PROG_SRAM
    from omotion.MotionSensor import _ERROR_TYPES
    r = sensor._send(packetType=OW_FPGA, command=OW_FPGA_PROG_SRAM,
                     addr=camera_mask, reserved=2, timeout=timeout)
    return r is not None and r.packetType not in _ERROR_TYPES


def capture_composite_frames(
    sensor,
    console,
    cams,
    n_frames: int = 5,
    laser: bool = True,
    stride: int | None = None,
    load_fpga: bool = True,
    on_frame=None,
    timeout_s: float | None = None,
    laser_delay_us: int | None = None,
    warmup_exposures: int = 3,
) -> list[CompositeFrame]:
    """Stream ~1 Hz full-frame composites from one or more cameras on one
    sensor module (``cams``: an index 0-7 or an iterable of them).

    Bring-up order (bench-proven 2026-09-29; each step matters):
      1. camera power, optional forced FPGA SRAM load, production sensor
         config;
      2. trigger config = DEFAULT_TRIGGER_CONFIG at 40 Hz with dark-frame
         skipping off (every frame lit) and TA per ``laser``; SyncOut on;
      3. histogram streaming reader with an image queue, camera streams on;
      4. firmware image mode BEFORE FSIN starts -- in histogram mode the
         firmware's stall detector rail-cycles a camera after 3 missed
         frames, which SPI overruns at scan start can trigger;
      5. FPGAs quiet (no pushes), trigger start, then the composite timing
         (VTS before HTS), then STRIDE, then arm the sweeps at line 0.

    Collects until every camera has produced ``n_frames`` composites (or the
    timeout). ``on_frame(CompositeFrame)`` is called for each composite as it
    completes (from this thread; ``frame.cam_id`` says which camera). Returns
    all composites in completion order. When ``laser`` is set the caller
    applies laser power first (MotionInterface.apply_laser_power -- the
    driver registers are cleared by a console power cycle). Keep the sensor's
    DEBUG_FLAG_USB_PRINTF off: firmware printf over USB during an image
    stream wedges the COMM endpoint (bench 2026-09-29).

    ``laser_delay_us`` overrides the console's LaserPulseDelayUsec (FSIN edge
    to laser pulse); None keeps DEFAULT_TRIGGER_CONFIG's production value,
    which lands the pulse inside the ~650 us exposure.
    """
    from omotion.config import (
        COMPOSITE_RESTORE_PROFILE,
        COMPOSITE_STRIDE,
        COMPOSITE_TIMING_PROFILE,
        DEFAULT_TRIGGER_CONFIG,
    )
    cams = sorted({int(cams)} if isinstance(cams, int) else {int(c) for c in cams})
    if not cams or any(not 0 <= c <= 7 for c in cams):
        raise ValueError(f"cams must be camera indices 0-7, got {cams}")
    # ``stride``: one value for every camera, or {cam: stride}. Throughput
    # budget: one sensor's USB moves ~1850 lines/s with the current firmware
    # (bench 2026-09-29), i.e. sum over cameras of 1280*40/stride must stay
    # below that or lines are dropped at the firmware's USB staging buffer
    # (staggering strides across cameras did not help).
    if stride is None:
        stride = COMPOSITE_STRIDE
    strides = ({int(c): int(v) for c, v in stride.items()}
               if isinstance(stride, dict) else {c: int(stride) for c in cams})
    if set(strides) != set(cams):
        raise ValueError(f"stride map {strides} does not cover cams {cams}")
    mask = 0
    for c in cams:
        mask |= 1 << c
    histo_if = sensor.uart.histo
    image_q: _queue.Queue = _queue.Queue()
    discard_q: _queue.Queue = _queue.Queue()
    regs = {c: FpgaRegs(sensor, c) for c in cams}
    frames: list[CompositeFrame] = []
    saved_trigger = console.get_trigger_json()
    if isinstance(saved_trigger, str):
        saved_trigger = json.loads(saved_trigger)
    if timeout_s is None:
        timeout_s = 10.0 + 2.0 * n_frames * max(strides.values()) / 40.0

    streaming = trigger_started = image_mode_on = retimed = False
    try:
        if not sensor.enable_camera_power(mask):
            raise RuntimeError(f"camera power-on failed (mask 0x{mask:02X})")
        time.sleep(0.3)
        # One camera per command: a multi-camera load blocks the firmware
        # for ~10 s per camera and a 2-camera (~20 s) load lost the COMM
        # pipe on the bench; single-camera loads are proven.
        for c in cams if load_fpga else ():
            if not force_load_fpga(sensor, 1 << c):
                raise RuntimeError(f"cam{c}: forced FPGA SRAM load failed")
        if not sensor.camera_configure_registers(mask):
            raise RuntimeError(f"sensor configuration failed (mask 0x{mask:02X})")

        cfg = dict(DEFAULT_TRIGGER_CONFIG)
        # Every frame lit: skip interval 0 AND skip delay 0. Console fw never
        # leaves its initial "dark" laser slot when LaserPulseSkipInterval is
        # 0, and that slot fires at delay + LaserPulseSkipDelayUsec (1800 us
        # by default: outside the exposure, and a 25.7 ms one-shot that
        # swallows every other FSIN). Zeroing the skip delay makes the dark
        # slot identical to the lit one (bench 2026-09-29).
        cfg.update(TriggerFrequencyHz=40.0, LaserPulseSkipInterval=0,
                   LaserPulseSkipDelayUsec=0,
                   EnableSyncOut=True, EnableTaTrigger=bool(laser))
        if laser_delay_us is not None:
            cfg["LaserPulseDelayUsec"] = int(laser_delay_us)
        if not console.set_trigger_json(data=cfg):
            raise RuntimeError("set_trigger_json failed")
        if not sensor.enable_camera_fsin_ext():
            raise RuntimeError("enable_camera_fsin_ext failed")

        histo_if.flush_stale_data(expected_size=_STREAM_READ_SIZE)
        histo_if.start_streaming(discard_q, _STREAM_READ_SIZE, image_queue=image_q)
        streaming = True
        if not sensor.enable_camera(mask):
            raise RuntimeError(f"enable_camera failed (mask 0x{mask:02X})")
        time.sleep(0.5)
        if not sensor.set_camera_image_mode(True, mask):
            raise RuntimeError("OW_CAMERA_IMAGE_MODE enable failed")
        image_mode_on = True
        time.sleep(0.2)
        for c, r in regs.items():
            if not r.check_id():
                raise RuntimeError(f"cam{c}: FPGA control plane not answering")
            if not r.stride_capable():
                raise RuntimeError(f"cam{c}: FPGA register map < v3 (no STRIDE)")
            r.quiet()
        # Everything below happens with FSIN stopped: no frames, so no line
        # pushes, so no COMM command ever competes with the image stream.
        # (Bench 2026-09-29: arming 8 cameras one by one while the first ones
        # already streamed wedged the COMM endpoint.) The sweeps arm on the
        # first frame after the trigger starts, all cameras on the same
        # frame, and the laser only fires while lines are being collected.
        retimed = True
        for c in cams:
            if not write_register_sequence(sensor, c, COMPOSITE_TIMING_PROFILE):
                raise RuntimeError(f"cam{c}: composite retiming failed")
        for c, r in regs.items():
            r.set_stride(strides[c])
            r.arm_sweep(0)
        while not image_q.empty():          # nothing from before the arm
            image_q.get_nowait()
        if not console.start_trigger():
            raise RuntimeError("start_trigger failed")
        trigger_started = True

        # The trigger (and laser) start with every sweep already armed, so
        # the first exposures include the laser's warm-up pulses.
        asms = {c: CompositeAssembler(skip_exposures=warmup_exposures) for c in cams}
        counts = {c: 0 for c in cams}
        lines_rx = {c: 0 for c in cams}
        t_end = time.monotonic() + timeout_s
        while min(counts.values()) < n_frames and time.monotonic() < t_end:
            try:
                pkt = image_q.get(timeout=0.25)
            except _queue.Empty:
                continue
            t = time.monotonic()
            try:
                line = parse_image_packet(pkt)
            except ImageLineError as exc:
                logger.warning("dropping bad image packet: %s", exc)
                continue
            asm = asms.get(line.cam_id)
            if asm is None or counts[line.cam_id] >= n_frames:
                continue
            lines_rx[line.cam_id] += 1
            done = asm.add(line, t)
            if done is not None:
                counts[line.cam_id] += 1
                frames.append(done)
                if on_frame is not None:
                    on_frame(done)
        short = {c: n for c, n in counts.items() if n < n_frames}
        if short:
            logger.error("composites short of %d within %.1f s: %s "
                         "(lines received per camera: %s)",
                         n_frames, timeout_s, short, lines_rx)
    finally:
        # Every restore step independently; a dead transport must not orphan
        # the reader thread (same policy as capture_full_frames). Stop FSIN
        # FIRST and let the last frame's pushes drain, so the sensor commands
        # below never run against a live line stream.
        if trigger_started:
            try:
                console.stop_trigger()
            except Exception:
                logger.exception("stop_trigger failed")
            time.sleep(0.1)
        for c, r in regs.items():
            try:
                r.set_stride(0)
                r.stop_sweep()
            except Exception:
                logger.exception("cam%d: FPGA sweep stop failed", c)
        if retimed:
            for c in cams:
                try:
                    write_register_sequence(sensor, c, COMPOSITE_RESTORE_PROFILE)
                except Exception:
                    logger.exception("cam%d: timing restore failed", c)
        for r in regs.values():
            try:
                r.exit_image_mode()
            except Exception:
                pass
        try:
            console.set_trigger_json(data=saved_trigger)
        except Exception:
            logger.exception("restoring trigger config failed")
        if image_mode_on:
            try:
                st = sensor.image_mode_exit_status()
                if st is None:
                    logger.error("image mode exit failed")
                else:
                    gaps = {c: st["gap_count"][c] for c in cams}
                    (logger.warning if any(gaps.values()) else logger.info)(
                        "image mode off; firmware lost-line events per camera: %s", gaps)
                    if "link_err" in st:
                        logger.info("firmware per-camera breakdown: %s", {
                            c: {k: st[k][c] for k in ("lines_ok", "link_err", "bad_magic",
                                                      "resync", "stage_full")}
                            for c in cams})
            except Exception:
                logger.exception("image mode exit failed")
        for fn in (sensor.disable_camera_fsin_ext,
                   lambda: sensor.disable_camera(mask)):
            try:
                fn()
            except Exception:
                logger.exception("teardown step failed (mask 0x%02X)", mask)
        if streaming:
            try:
                histo_if.stop_streaming()
                histo_if.drain_final(expected_size=_STREAM_READ_SIZE)
            except Exception:
                logger.exception("stop_streaming failed")
    return frames
