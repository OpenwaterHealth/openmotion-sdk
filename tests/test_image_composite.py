"""Software-only unit tests for the 1 Hz stride composite (camera-fpga map v3,
epic OpenwaterHealth/openmotion-bloodflow-app#480).

Covers: the composite timing profiles' write-order invariants, the STRIDE
register, CompositeAssembler cycle emission, and the C-speed line CRC.
No hardware required.
"""

import binascii
import os

import numpy as np
import pytest

pytestmark = pytest.mark.unit

ROW_S_PER_HTS = 9.032e-6 / 432          # OX02C1B row time per HTS unit
LINE_DRAIN_S = 2408 * 286e-9            # 2408-B push at ~286 ns/byte


def _profile_dict(profile):
    return dict(profile)


def _u16(d, hi):
    return (d[hi] << 8) | d[hi + 1]


def _u24(d, hi):
    return (d[hi] << 16) | (d[hi + 1] << 8) | d[hi + 2]


def test_composite_profile_values_and_order():
    """VTS is written before HTS (a frame longer than the FSIN period wedges
    trigger mode, and production->composite would pass through 866x2768),
    VTS respects the sensor minimum, max_expo_a tracks VTS, and the frame
    fits the 40 Hz FSIN period."""
    from omotion.config import COMPOSITE_TIMING_PROFILE, OX02C1B_MIN_VTS

    regs = [r for r, _ in COMPOSITE_TIMING_PROFILE]
    assert regs.index(0x380E) < regs.index(0x380C)     # VTS hi before HTS hi
    assert regs.index(0x380F) < regs.index(0x380D)
    d = _profile_dict(COMPOSITE_TIMING_PROFILE)
    hts, vts = _u16(d, 0x380C), _u16(d, 0x380E)
    assert (hts, vts) == (866, 1380)
    assert vts >= OX02C1B_MIN_VTS
    assert _u24(d, 0x3881) == vts
    assert hts * vts * ROW_S_PER_HTS < 1 / 40.0
    assert _u16(d, 0x3501) == 36


def test_composite_restore_profile_order():
    """Restore shrinks HTS before growing VTS back to production."""
    from omotion.config import COMPOSITE_RESTORE_PROFILE

    regs = [r for r, _ in COMPOSITE_RESTORE_PROFILE]
    assert regs.index(0x380C) < regs.index(0x380E)
    d = _profile_dict(COMPOSITE_RESTORE_PROFILE)
    assert (_u16(d, 0x380C), _u16(d, 0x380E)) == (432, 2768)
    assert _u16(d, 0x3501) == 72
    assert _u24(d, 0x3881) == 2768


def test_composite_stride_clears_line_drain():
    """STRIDE rows must outlast one line drain or the FPGA drops lines."""
    from omotion.config import COMPOSITE_STRIDE, COMPOSITE_TIMING_PROFILE

    hts = _u16(_profile_dict(COMPOSITE_TIMING_PROFILE), 0x380C)
    assert COMPOSITE_STRIDE * hts * ROW_S_PER_HTS > LINE_DRAIN_S * 1.03
    assert 1280 % COMPOSITE_STRIDE == 0                 # equal lines per frame
    assert COMPOSITE_STRIDE / 40.0 == pytest.approx(1.0)  # 1 Hz at 40 Hz FSIN


def test_line_crc_matches_util_crc16():
    """parse_image_line uses binascii.crc_hqx(.., 0xFFFF); it must equal the
    firmware-matched util_crc16 (CRC-16/CCITT-FALSE, check value 0x29B1)."""
    from omotion.utils import util_crc16

    assert binascii.crc_hqx(b"123456789", 0xFFFF) == 0x29B1
    for _ in range(5):
        d = os.urandom(2406)
        assert binascii.crc_hqx(d, 0xFFFF) == util_crc16(d)


class _FakeRegSensor:
    def __init__(self, version):
        self.ops = []
        self.regs = {0x00: 0x5A, 0x01: version}

    def i2c_read_register(self, dev_addr, reg_addr, read_len=1,
                          reg_addr_size=1, mux_channel=None):
        if reg_addr_size == 1:
            return bytes([self.regs.get(reg_addr, 0x00)])
        reg, val = (reg_addr >> 8) & 0xFF, reg_addr & 0xFF
        self.ops.append((mux_channel, reg, val))
        self.regs[reg] = val
        return b"\x00"


def test_fpga_regs_stride_and_quiet():
    from omotion.ImageCapture import CTRL_IMAGE_MODE, REG_STRIDE, FpgaRegs

    s = _FakeRegSensor(version=0x03)
    r = FpgaRegs(s, cam=2)
    assert r.stride_capable() is True
    r.set_stride(40)
    assert s.ops[-1] == (2, REG_STRIDE, 40)
    with pytest.raises(ValueError):
        r.set_stride(256)
    r.quiet()
    assert (2, 0x04, 0xFF) in s.ops and (2, 0x05, 0x0F) in s.ops   # line 4095
    assert s.ops[-1] == (2, 0x03, CTRL_IMAGE_MODE)                 # no sweep bit
    assert FpgaRegs(_FakeRegSensor(version=0x02), cam=0).stride_capable() is False


def _mk_line(line_no, frame_cnt, overrun=False):
    from omotion.ImageCapture import IMAGE_WIDTH, ImageLine

    px = np.full(IMAGE_WIDTH, line_no & 0x3FF, dtype=np.uint16)
    return ImageLine(cam_id=0, line=line_no, flags=int(overrun),
                     overrun=overrun, wedge=False, frame_cnt=frame_cnt, pixels=px)


def _stride_stream(stride, cycles, start_fc=7, drop=()):
    """Lines as the v3 FPGA emits them: frame k carries phase k % stride."""
    t = 0.0
    for k in range(stride * cycles):
        phase = k % stride
        for ln in range(phase, 1280, stride):
            if (k, ln) in drop:
                continue
            t += 1e-4
            yield _mk_line(ln, (start_fc + k) & 0xFF), t


def test_composite_assembler_emits_once_per_cycle():
    from omotion.ImageCapture import CompositeAssembler

    asm = CompositeAssembler()
    out = [f for line, t in _stride_stream(40, 3) if (f := asm.add(line, t))]
    assert len(out) == 3
    for i, f in enumerate(out):
        assert f.lines == 1280
        assert len(f.frame_cnts) == 40
        assert f.frame_cnts[0] == (7 + 40 * i) & 0xFF
        assert not f.overrun
        assert np.array_equal(f.image[:, 0], np.arange(1280) & 0x3FF)


def test_composite_assembler_dropped_line_delays_one_cycle():
    """A line lost in cycle 0 is only filled in cycle 1, so the first
    composite comes out one cycle late and spans 2*stride frames."""
    from omotion.ImageCapture import CompositeAssembler

    asm = CompositeAssembler()
    stream = _stride_stream(4, 3, drop={(1, 5)})       # frame 1 = phase 1
    out = [f for line, t in stream if (f := asm.add(line, t))]
    assert len(out) == 2
    assert out[0].lines > 1280
    assert len(out[0].frame_cnts) > 4


def test_composite_assembler_ignores_out_of_range():
    from omotion.ImageCapture import CompositeAssembler

    asm = CompositeAssembler()
    assert asm.add(_mk_line(1280, 1), 0.0) is None


def test_composite_assembler_skips_warmup_exposures():
    """The first N exposures are ignored; their rows fill from the next
    cycle, so the first composite has no warm-up (dark) rows."""
    from omotion.ImageCapture import CompositeAssembler

    asm = CompositeAssembler(skip_exposures=3)
    out = [f for line, t in _stride_stream(4, 3) if (f := asm.add(line, t))]
    assert len(out) == 2
    assert 7 not in out[0].frame_cnts and 9 not in out[0].frame_cnts   # fc 7..9 skipped
    assert out[0].frame_cnts[0] == 10
