"""Software-only tests for the thermal-soak capture (openmotion-sdk#296):
STRIDE budget, per-row provenance, scheduled-dark classification, composite
stats and the soak script's argument handling. No hardware required."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

pytestmark = pytest.mark.unit

ROW_PROD = 9.032e-6
ROW_COMPOSITE = 866 * 9.032e-6 / 432


def _line(line_no, fc, value):
    from omotion.ImageCapture import IMAGE_WIDTH, ImageLine

    px = np.full(IMAGE_WIDTH, value, dtype=np.uint16)
    return ImageLine(cam_id=2, line=line_no, flags=0, overrun=False, wedge=False,
                     frame_cnt=fc & 0xFF, pixels=px)


def _composites(stride, cycles, dark_k=(), lit=300, dark=128, start_fc=10):
    """Feed a CompositeAssembler lines as the v3 FPGA emits them; exposures
    whose index is in dark_k are unlit."""
    from omotion.ImageCapture import CompositeAssembler

    asm = CompositeAssembler()
    out, t = [], 0.0
    for k in range(stride * cycles):
        for ln in range(k % stride, 1280, stride):
            t += 1e-4
            f = asm.add(_line(ln, start_fc + k, dark if k in dark_k else lit), t)
            if f is not None:
                out.append(f)
    return out


def test_composite_stride_budget():
    from omotion.ImageCapture import composite_stride

    assert composite_stride(1, ROW_PROD) == 80          # 2.0 s per image at production timing
    assert composite_stride(2, ROW_PROD) == 159
    assert composite_stride(3, ROW_PROD) == 238
    assert composite_stride(1, ROW_COMPOSITE) == 40     # the 1 Hz composite
    with pytest.raises(ValueError):
        composite_stride(4, ROW_PROD)                   # would need STRIDE > 255


def test_composite_stride_respects_both_limits():
    from omotion.config import COMPOSITE_BURST_BUDGET_LINES_PER_S, COMPOSITE_LINE_DRAIN_S
    from omotion.ImageCapture import composite_stride

    for n in (1, 2, 3):
        s = composite_stride(n, ROW_PROD)
        assert s * ROW_PROD >= COMPOSITE_LINE_DRAIN_S * 1.03
        assert n / (s * ROW_PROD) <= COMPOSITE_BURST_BUDGET_LINES_PER_S
        assert n / ((s - 1) * ROW_PROD) > COMPOSITE_BURST_BUDGET_LINES_PER_S or \
            (s - 1) * ROW_PROD < COMPOSITE_LINE_DRAIN_S * 1.03     # minimal


def test_row_provenance():
    (f,) = _composites(4, 1)
    assert f.row_fc.shape == (1280,) and f.row_t.shape == (1280,)
    assert np.array_equal(f.row_fc[:8], [10, 11, 12, 13, 10, 11, 12, 13])
    assert np.all(np.diff(f.row_t[::4]) > 0)            # rows of one exposure arrive in order
    assert (f.row_fc >= 0).all()


def test_classify_dark_rows_finds_scheduled_dark():
    from omotion.ImageCapture import classify_dark_rows

    (f,) = _composites(8, 1, dark_k={3})
    dark, means, dark_fcs = classify_dark_rows(f)
    assert dark_fcs == [13]
    assert dark.sum() == 1280 // 8
    assert np.array_equal(np.where(dark)[0][:3], [3, 11, 19])
    assert means[13] == pytest.approx(128) and means[10] == pytest.approx(300)


def test_classify_dark_rows_needs_signal():
    from omotion.ImageCapture import classify_dark_rows

    (f,) = _composites(8, 1, dark_k={3}, lit=140)       # < 20 DN above the pedestal
    dark, _, dark_fcs = classify_dark_rows(f)
    assert dark_fcs == [] and not dark.any()


def test_composite_stats_separates_dark_rows():
    from omotion.ImageCapture import composite_stats

    (f,) = _composites(8, 1, dark_k={3})
    st = composite_stats(f)
    assert st["dark_exposures"] == 1 and st["dark_rows"] == 160
    assert st["lit_mean"] == pytest.approx(300)
    assert st["dark_mean"] == pytest.approx(128)
    assert st["n_exposures"] == 8 and st["lines"] == 1280
    assert st["K"] == pytest.approx(0.0)                # flat synthetic image
    assert all(st[f"band{i}_mean"] == pytest.approx(300) for i in range(4))


def test_composite_stats_speckle_K():
    from omotion.ImageCapture import COMPOSITE_ROI, CompositeFrame, composite_stats

    rng = np.random.default_rng(0)
    speckle = rng.exponential(200.0, size=(1280, 1920))     # fully developed speckle: K = 1
    img = np.clip(128 + speckle, 0, 1023).astype(np.uint16)
    f = CompositeFrame(cam_id=0, image=img, t_first=0, t_last=1, frame_cnts=[1], overrun=False,
                       lines=1280, row_fc=np.ones(1280, np.int16), row_t=np.zeros(1280))
    st = composite_stats(f)
    sub = img[COMPOSITE_ROI].astype(float) - 128
    assert st["K"] == pytest.approx(sub.std() / sub.mean(), rel=1e-9)
    assert 0.85 < st["K"] < 1.0                              # clipping at 1023 trims the tail
    assert st["sat_frac"] > 0


def _soak_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "thermal_soak.py"
    spec = importlib.util.spec_from_file_location("thermal_soak", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_soak_groups_and_args():
    mod = _soak_module()
    assert mod.groups_of([0, 7, 1, 6, 2], 2) == [[0, 7], [1, 6], [2]]
    a = mod.parse_args(["--scan-mask", "0x18", "--cams", "3", "3", "4", "--concurrent", "2"])
    assert a.cams == [3, 4] and a.timing == "production" and a.laser_schedule == "production"
    assert mod.parse_args([]).cams == [0, 1, 6, 7]               # app clinical mask 0xC3
    assert mod.parse_args(["--scan-mask", "0x66"]).cams == [1, 2, 5, 6]
    with pytest.raises(SystemExit):
        mod.parse_args(["--cams", "8"])
    with pytest.raises(SystemExit):
        mod.parse_args(["--cams", "3"])                           # not in the default mask
    with pytest.raises(SystemExit):
        mod.parse_args(["--cams", "1", "6", "--concurrent", "3"])
