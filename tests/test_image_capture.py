"""Software-only unit tests for drip-scan image capture (camera-fpga#8, SDK issue #167).

Covers: pinned config constants, RAW10 pack/unpack bit layout, line parse with
CRC verification, USB envelope parse, FrameAssembler, sweep retiming sequence,
and the sweep retry policy. No hardware required.
"""

import numpy as np
import pytest

pytestmark = pytest.mark.unit


def test_config_constants_pinned_values():
    """The cross-plan pinned wire/protocol constants. If this test fails after
    an edit to config.py, firmware/FPGA interop is broken — these values are
    fixed by the drip-scan design spec and must not drift."""
    from omotion import config

    assert config.TYPE_IMAGE == 0x03
    assert config.OW_IMAGE_PACKET == 0x03          # pre-existing, same value, different namespace
    assert config.OW_CAMERA_IMAGE_MODE == 0x30
    assert config.SWEEP_FSIN_HZ == 1.0
    assert config.PRODUCTION_FSIN_HZ == 40.0
    # Sweep profile: HTS=38400, VTS=1312, tc_r_initial=1308, exposure=1 row.
    assert config.SWEEP_TIMING_PROFILE == (
        (0x380C, 0x8C), (0x380D, 0xA0),
        (0x380E, 0x05), (0x380F, 0x20),
        (0x3501, 0x00), (0x3502, 0x01),
    )
    # Restore profile: shipped production values from X02C1B_Sensor_Config.h.
    assert config.PRODUCTION_TIMING_PROFILE == (
        (0x380C, 0x01), (0x380D, 0xB0),
        (0x380E, 0x0A), (0x380F, 0xD0),
        (0x3501, 0x00), (0x3502, 0x48),
    )


# ---------------------------------------------------------------------------
# RAW10 packing / line parsing
# ---------------------------------------------------------------------------

# Hand vector computed independently during planning: pixel k of each 4-pixel
# group occupies bits [10k+9:10k] of a 40-bit little-endian group.
_HAND_PIXELS = [0x001, 0x3FF, 0x155, 0x2AA, 0x0F0, 0x10F, 0x333, 0x0CC]
_HAND_BYTES = bytes([0x01, 0xFC, 0x5F, 0x95, 0xAA, 0xF0, 0x3C, 0x34, 0x33, 0x33])

# Golden full line: p[k] = (7k+3) & 0x3FF, line=1234, flags=0, frame_cnt=0x5C.
# Header, first packed bytes, and CRC computed independently during planning.
_GOLDEN_HDR = bytes([0xB6, 0x01, 0xD2, 0x04, 0x5C, 0x00])
_GOLDEN_PACKED_HEAD = bytes([0x03, 0x28, 0x10, 0x01, 0x06, 0x1F, 0x98, 0xD0, 0x02, 0x0D])
_GOLDEN_CRC = 0xA25E


def _golden_pixels():
    return [(7 * k + 3) & 0x3FF for k in range(1920)]


def _golden_line_bytes(line=1234, flags=0, frame_cnt=0x5C, pixels=None):
    """Build a full 2408-B line push with a correct CRC (reference builder for
    tests; bit-layout independence is anchored by _HAND_BYTES/_GOLDEN_* which
    were computed outside this codebase)."""
    from omotion.ImageCapture import pack_raw10
    from omotion.utils import util_crc16

    packed = pack_raw10(pixels if pixels is not None else _golden_pixels())
    hdr = bytes([0xB6, 0x01, line & 0xFF,
                 ((flags & 0xF) << 4) | ((line >> 8) & 0xF), frame_cnt, 0x00])
    body = hdr + packed
    crc = util_crc16(body)
    return body + bytes([(crc >> 8) & 0xFF, crc & 0xFF])   # CRC big-endian


def test_crc16_is_ccitt_false():
    """Proves the SDK CRC used for line verification is the CRC-CCITT-FALSE
    variant implemented byte-identically in sensor-fw utils.c util_crc16
    (poly 0x1021, init 0xFFFF, MSB-first, no final XOR): check value 0x29B1."""
    import binascii
    from omotion.utils import util_crc16

    assert util_crc16(b"123456789") == 0x29B1
    assert binascii.crc_hqx(b"123456789", 0xFFFF) == 0x29B1


def test_unpack_raw10_hand_vector():
    """Proves the unpacker implements the exact pinned bit layout (spec §4.1:
    pixel k of a 4-px group at 40-bit-group bits [10k+9:10k], low byte first).
    Same math the FPGA packer TB anchors with its own hand vector; both were
    verified against the spec formulation independently."""
    from omotion.ImageCapture import unpack_raw10

    assert list(unpack_raw10(_HAND_BYTES)) == _HAND_PIXELS


def test_pack_raw10_hand_vector():
    """Reference packer is the exact inverse (same anchored bytes)."""
    from omotion.ImageCapture import pack_raw10

    assert pack_raw10(_HAND_PIXELS) == _HAND_BYTES


def test_unpack_raw10_rejects_bad_length():
    from omotion.ImageCapture import unpack_raw10

    with pytest.raises(ValueError):
        unpack_raw10(b"\x00" * 7)   # not a multiple of 5


def test_parse_image_line_golden_roundtrip():
    """Full-line proof: reference-packed golden line parses back to the exact
    header fields and all 1920 pixels, and the on-wire bytes match the
    independently computed header/packed-head/CRC anchors."""
    from omotion.ImageCapture import parse_image_line

    raw = _golden_line_bytes()
    assert len(raw) == 2408
    assert raw[:6] == _GOLDEN_HDR
    assert raw[6:16] == _GOLDEN_PACKED_HEAD
    assert raw[2406] == (_GOLDEN_CRC >> 8) and raw[2407] == (_GOLDEN_CRC & 0xFF)

    ln = parse_image_line(raw, cam_id=3)
    assert ln.cam_id == 3
    assert ln.line == 1234
    assert ln.flags == 0
    assert ln.overrun is False
    assert ln.frame_cnt == 0x5C
    assert ln.pixels.dtype == np.uint16
    assert list(ln.pixels) == _golden_pixels()


def test_parse_image_line_bad_crc_rejected():
    """A single flipped payload bit must fail CRC — the per-line integrity
    check that catches USART byte-slip (spec §6 risk table)."""
    from omotion.ImageCapture import ImageLineError, parse_image_line

    raw = bytearray(_golden_line_bytes())
    raw[100] ^= 0x01
    with pytest.raises(ImageLineError, match="CRC"):
        parse_image_line(bytes(raw), cam_id=0)


def test_parse_image_line_bad_magic_and_version():
    from omotion.ImageCapture import ImageLineError, parse_image_line

    good = _golden_line_bytes()
    bad_magic = b"\x00" + good[1:]
    with pytest.raises(ImageLineError, match="magic"):
        parse_image_line(bad_magic, cam_id=0)
    bad_ver = good[:1] + b"\x02" + good[2:]
    with pytest.raises(ImageLineError, match="version"):
        parse_image_line(bad_ver, cam_id=0)
    with pytest.raises(ImageLineError, match="length"):
        parse_image_line(good[:-1], cam_id=0)


def test_parse_image_line_overrun_flag():
    """flags live in the high nibble of byte 3: flags=1, line=1234 -> 0x14."""
    from omotion.ImageCapture import parse_image_line

    raw = _golden_line_bytes(flags=0x1)
    assert raw[3] == 0x14
    ln = parse_image_line(raw, cam_id=0)
    assert ln.overrun is True and ln.flags == 0x1 and ln.line == 1234


def test_parse_image_line_wedge_flag():
    """flags bit1 is the pusher-watchdog wedge (spec §4.3), distinct from the
    bit0 overrun. Wedge-only (0x2) sets .wedge without .overrun; the combined
    0x3 case matches the FPGA 'a wedge sets both bits' contract."""
    from omotion.ImageCapture import parse_image_line

    raw = _golden_line_bytes(flags=0x2)
    assert raw[3] == 0x24                       # high nibble 0x2, line[11:8]=0x4
    ln = parse_image_line(raw, cam_id=0)
    assert ln.wedge is True and ln.overrun is False
    assert ln.flags == 0x2 and ln.line == 1234

    raw = _golden_line_bytes(flags=0x3)         # wedge + overrun together
    assert raw[3] == 0x34
    ln = parse_image_line(raw, cam_id=0)
    assert ln.overrun is True and ln.wedge is True and ln.flags == 0x3


# ---------------------------------------------------------------------------
# USB envelope
# ---------------------------------------------------------------------------

def _envelope(line_bytes, cam_id=2):
    """Wrap a 2408-B line in the 2424-B TYPE_IMAGE stream envelope (contract B).
    Transport CRC field is 0x0000 — the SDK does not verify it for image
    packets (MCU forwards blind; the line CRC is authoritative). The 4-byte
    FSIN timestamp is zeroed — the parser ignores it (same convention as the
    histogram envelope)."""
    total = 6 + 4 + 1 + 1 + len(line_bytes) + 1 + 3
    return (bytes([0xAA, 0x03]) + total.to_bytes(4, "little")
            + bytes(4)                                    # FSIN timestamp (ignored)
            + bytes([0xFF, cam_id]) + line_bytes
            + bytes([0xEE, 0x00, 0x00, 0xDD]))


def test_parse_image_packet_envelope():
    from omotion.ImageCapture import IMAGE_PACKET_SIZE, parse_image_packet

    pkt = _envelope(_golden_line_bytes(), cam_id=5)
    assert len(pkt) == IMAGE_PACKET_SIZE == 2424
    ln = parse_image_packet(pkt)
    assert ln.cam_id == 5 and ln.line == 1234 and ln.frame_cnt == 0x5C


def test_parse_image_packet_bad_framing():
    from omotion.ImageCapture import ImageLineError, parse_image_packet

    pkt = bytearray(_envelope(_golden_line_bytes()))
    pkt[0] = 0x00
    with pytest.raises(ImageLineError):
        parse_image_packet(bytes(pkt))
    pkt = bytearray(_envelope(_golden_line_bytes()))
    pkt[-1] = 0x00   # EOF
    with pytest.raises(ImageLineError):
        parse_image_packet(bytes(pkt))


# ---------------------------------------------------------------------------
# FrameAssembler
# ---------------------------------------------------------------------------

def _mk_line(line_no, frame_cnt=0x10, value=None, overrun=False, wedge=False):
    """Cheap ImageLine for assembler tests (bypasses byte packing — the wire
    path is proven by the parser tests above)."""
    from omotion.ImageCapture import IMAGE_WIDTH, ImageLine

    px = np.full(IMAGE_WIDTH, value if value is not None else line_no,
                 dtype=np.uint16)
    flags = (0x1 if overrun else 0) | (0x2 if wedge else 0)
    return ImageLine(cam_id=0, line=line_no, flags=flags, overrun=overrun,
                     wedge=wedge, frame_cnt=frame_cnt, pixels=px)


def test_assembler_complete_frame():
    """All 1280 lines of one exposure -> complete, consistent, right shape,
    rows land at their line index."""
    from omotion.ImageCapture import IMAGE_HEIGHT, IMAGE_WIDTH, FrameAssembler

    asm = FrameAssembler()
    for i in range(IMAGE_HEIGHT):
        assert asm.add(_mk_line(i)) is True
    assert asm.complete is True
    assert asm.missing() == []
    assert asm.mixed_exposure is False
    assert asm.frame_cnt == 0x10
    img = asm.image()
    assert img.shape == (IMAGE_HEIGHT, IMAGE_WIDTH) and img.dtype == np.uint16
    assert img[7, 0] == 7 and img[1279, 100] == 1279


def test_assembler_gap_list_and_incomplete():
    from omotion.ImageCapture import IMAGE_HEIGHT, FrameAssembler

    asm = FrameAssembler()
    for i in range(IMAGE_HEIGHT):
        if i not in (5, 900):
            asm.add(_mk_line(i))
    assert asm.complete is False
    assert asm.missing() == [5, 900]


def test_assembler_rejects_frame_cnt_mismatch_by_default():
    """Single-exposure enforcement: a line from a different exposure is
    rejected and counted, and the image stays attributable to one frame_cnt."""
    from omotion.ImageCapture import FrameAssembler

    asm = FrameAssembler()
    assert asm.add(_mk_line(0, frame_cnt=0x10)) is True
    assert asm.add(_mk_line(1, frame_cnt=0x11)) is False
    assert asm.rejected_lines == 1
    assert asm.frame_cnt == 0x10
    assert asm.mixed_exposure is False
    assert 1 in asm.missing()


def test_assembler_allow_mixed_gap_fill():
    """Retry fallback: with allow_mixed=True a different-exposure line fills
    its gap but the result is flagged mixed_exposure."""
    from omotion.ImageCapture import FrameAssembler

    asm = FrameAssembler()
    asm.add(_mk_line(0, frame_cnt=0x10))
    asm.allow_mixed = True
    assert asm.add(_mk_line(1, frame_cnt=0x11)) is True
    assert asm.mixed_exposure is True
    assert asm.rejected_lines == 0


def test_assembler_rejects_out_of_range_line():
    from omotion.ImageCapture import IMAGE_HEIGHT, FrameAssembler

    asm = FrameAssembler()
    assert asm.add(_mk_line(IMAGE_HEIGHT)) is False   # line 1280 of 0..1279
    assert asm.rejected_lines == 1


def test_assembler_reset_and_overrun_tracking():
    """reset() starts a fresh exposure (used by the strict retry policy);
    overrun on any accepted line is latched for reporting."""
    from omotion.ImageCapture import FrameAssembler

    asm = FrameAssembler()
    asm.add(_mk_line(0, frame_cnt=0x10, overrun=True))
    assert asm.overrun_seen is True
    asm.reset()
    assert asm.overrun_seen is False
    assert asm.frame_cnt is None
    assert asm.add(_mk_line(0, frame_cnt=0x22)) is True
    assert asm.frame_cnt == 0x22


def test_assembler_wedge_tracking():
    """A pusher-watchdog wedge on any accepted line is latched for reporting
    (parallel to overrun), and cleared on reset() for a strict retry."""
    from omotion.ImageCapture import FrameAssembler

    asm = FrameAssembler()
    asm.add(_mk_line(0, frame_cnt=0x10, wedge=True))
    assert asm.wedge_seen is True
    asm.reset()
    assert asm.wedge_seen is False


# ---------------------------------------------------------------------------
# OW_CAMERA_IMAGE_MODE sender + FPGA sweep registers
# ---------------------------------------------------------------------------

class _FakeResp:
    def __init__(self, packetType):
        self.packetType = packetType


class _FakeComm:
    def __init__(self, resp_type):
        self.calls = []
        self._resp_type = resp_type

    def send_packet(self, **kwargs):
        self.calls.append(kwargs)
        return _FakeResp(self._resp_type)


def _bare_sensor(resp_type):
    """MotionSensor without running __init__ (it wires USB/hotplug state we
    don't need): _send only touches self.uart.comm.send_packet, demo_mode,
    and _check_camera_mask — set exactly those."""
    from types import SimpleNamespace
    from omotion.MotionSensor import MotionSensor

    ms = MotionSensor.__new__(MotionSensor)
    ms.demo_mode = False
    ms.uart = SimpleNamespace(comm=_FakeComm(resp_type))
    return ms


def test_set_camera_image_mode_wire_format():
    """Pinned opcode contract: OW_CAMERA packet, command 0x30, reserved byte
    carries enable, data[0] carries the camera bitmask."""
    from omotion.config import OW_CAMERA, OW_CAMERA_IMAGE_MODE, OW_RESP

    ms = _bare_sensor(OW_RESP)
    assert ms.set_camera_image_mode(True, 0x42) is True
    call = ms.uart.comm.calls[-1]
    assert call["packetType"] == OW_CAMERA
    assert call["command"] == OW_CAMERA_IMAGE_MODE == 0x30
    assert call["reserved"] == 1
    assert call["data"] == bytes([0x42])

    assert ms.set_camera_image_mode(False, 0x01) is True
    assert ms.uart.comm.calls[-1]["reserved"] == 0


def test_set_camera_image_mode_error_response():
    from omotion.config import OW_ERROR

    ms = _bare_sensor(OW_ERROR)
    assert ms.set_camera_image_mode(True, 0xFF) is False


def test_set_camera_image_mode_rejects_bad_mask():
    from omotion.config import OW_RESP

    ms = _bare_sensor(OW_RESP)
    with pytest.raises(ValueError):
        ms.set_camera_image_mode(True, 0x1FF)


class _FakeRegSensor:
    """Records FpgaRegs traffic through the i2c_read_register passthrough."""

    def __init__(self):
        self.ops = []
        self.regs = {0x00: 0x5A, 0x01: 0x02, 0x09: 0x00}   # ID, VERSION v2, STATUS

    def i2c_read_register(self, dev_addr, reg_addr, read_len=1,
                          reg_addr_size=1, mux_channel=None):
        assert dev_addr == 0x5A
        if reg_addr_size == 1:            # read
            self.ops.append(("rd", mux_channel, reg_addr))
            return bytes([self.regs.get(reg_addr, 0x00)])
        reg, val = (reg_addr >> 8) & 0xFF, reg_addr & 0xFF   # write trick
        self.ops.append(("wr", mux_channel, reg, val))
        self.regs[reg] = val
        return b"\x00"


def test_fpga_regs_v2_sweep_arm():
    """arm_sweep programs the start line then CTRL = image|sweep (0x03);
    stop_sweep drops back to image-only; exit clears CTRL. LINE_L/H are the
    sweep start line in map v2."""
    from omotion.ImageCapture import CTRL_IMAGE_MODE, CTRL_SWEEP, FpgaRegs

    s = _FakeRegSensor()
    r = FpgaRegs(s, cam=3)
    assert r.check_id() is True
    assert r.check_version() is True

    s.ops.clear()
    r.arm_sweep(start_line=0x2A5)
    # Park the target out of reach, set the sweep bit, THEN lower the start
    # line: in single-line mode a frame between the two would push the
    # target line as a legacy packet and auto-increment the target.
    assert s.ops == [
        ("wr", 3, 0x04, 0xFF), ("wr", 3, 0x05, 0x0F),       # target 4095
        ("wr", 3, 0x03, CTRL_IMAGE_MODE | CTRL_SWEEP),
        ("wr", 3, 0x04, 0xA5), ("wr", 3, 0x05, 0x02),       # start line 0x2A5
    ]

    r.stop_sweep()
    assert s.ops[-1] == ("wr", 3, 0x03, CTRL_IMAGE_MODE)
    r.exit_image_mode()
    assert s.ops[-1] == ("wr", 3, 0x03, 0x00)


def test_fpga_regs_overrun_latch():
    from omotion.ImageCapture import STATUS_OVERRUN, FpgaRegs

    s = _FakeRegSensor()
    r = FpgaRegs(s, cam=0)
    assert r.overrun() is False
    s.regs[0x09] = STATUS_OVERRUN
    assert r.overrun() is True


def test_fpga_regs_wedge_latch():
    """STATUS bit3 is the pusher-watchdog wedge latch (map v2, spec §4.3),
    distinct from the bit2 overrun latch."""
    from omotion.ImageCapture import STATUS_WEDGE, FpgaRegs

    s = _FakeRegSensor()
    r = FpgaRegs(s, cam=0)
    assert r.wedge() is False
    s.regs[0x09] = STATUS_WEDGE
    assert r.wedge() is True


# ---------------------------------------------------------------------------
# Group-hold sweep retiming
# ---------------------------------------------------------------------------

class _FakeI2CSensor:
    """Records switch_camera / camera_i2c_write traffic for one camera."""

    def __init__(self):
        self.ops = []

    def switch_camera(self, cam):
        self.ops.append(("switch", cam))

    def camera_i2c_write(self, packet):
        self.ops.append(("wr", packet.device_address,
                         packet.register_address, packet.data))
        return True


def test_write_timing_profile_group_hold_sequence():
    """The whole profile must land inside ONE group-hold: 0x3208=0x00 opens
    group 0, the timing registers follow in pinned order, 0x3208=0x10 closes
    the group, and 0x3208=0xA0 (delayed launch) latches everything atomically
    at the next frame boundary — the atomicity spec §4.2 requires because
    tc_r_initial is VTS-coupled."""
    from omotion.ImageCapture import write_timing_profile
    from omotion.config import SWEEP_TIMING_PROFILE

    s = _FakeI2CSensor()
    assert write_timing_profile(s, cam=6, profile=SWEEP_TIMING_PROFILE) is True

    assert s.ops[0] == ("switch", 6)
    writes = [(op[2], op[3]) for op in s.ops[1:]]
    assert all(op[1] == 0x36 for op in s.ops[1:])   # OX02C1B device address
    assert writes == [
        (0x3208, 0x00),
        (0x380C, 0x8C), (0x380D, 0xA0),
        (0x380E, 0x05), (0x380F, 0x20),
        (0x3501, 0x00), (0x3502, 0x01),
        (0x3208, 0x10),
        (0x3208, 0xA0),
    ]


def test_write_timing_profile_reports_failure():
    from omotion.ImageCapture import write_timing_profile
    from omotion.config import PRODUCTION_TIMING_PROFILE

    s = _FakeI2CSensor()
    s.camera_i2c_write = lambda packet: False
    assert write_timing_profile(s, cam=0, profile=PRODUCTION_TIMING_PROFILE) is False


# ---------------------------------------------------------------------------
# Sweep retry policy (pure decision function driving the orchestrator loop)
# ---------------------------------------------------------------------------

def test_next_sweep_action_policy():
    """Strict attempts restart from line 0 on a fresh exposure (preserving the
    single-exposure guarantee); the last mixed_fill_sweeps attempts gap-fill
    from the first missing line; done/give_up terminate."""
    from omotion.ImageCapture import next_sweep_action

    # complete -> done regardless of attempt
    assert next_sweep_action([], attempt=1, max_sweeps=6, mixed_fill_sweeps=2) == ("done", None)
    # attempts 1..4 of 6 (2 reserved for fill): strict restart
    for a in (1, 2, 3, 4):
        assert next_sweep_action([9, 40], a, 6, 2) == ("restart", 0)
    # attempts 5..6: mixed gap-fill from first missing line
    assert next_sweep_action([9, 40], 5, 6, 2) == ("fill", 9)
    assert next_sweep_action([40], 6, 6, 2) == ("fill", 40)
    # beyond budget
    assert next_sweep_action([40], 7, 6, 2) == ("give_up", None)


def test_next_sweep_action_no_fill_budget():
    from omotion.ImageCapture import next_sweep_action

    assert next_sweep_action([3], 2, 2, 0) == ("restart", 0)
    assert next_sweep_action([3], 3, 2, 0) == ("give_up", None)
