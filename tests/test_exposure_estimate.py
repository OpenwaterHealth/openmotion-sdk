"""Tests for omotion/exposure/estimate.py: saturation-aware frame brightness.

Pure-software. The censored gamma fit is checked the way HANDOFF section 5
asks for on device data: take frames whose true mean is known, clip them at
full scale, and compare the fitted mean to the truth.
"""

import math

import numpy as np
import pytest

from omotion.exposure.estimate import (
    KIND_CLIPPED,
    KIND_FIT,
    KIND_PLAIN,
    METHOD_BASIC,
    METHOD_FIT,
    SAT_LEVEL,
    censored_gamma_mean,
    estimate_signal,
    histogram_mean,
    log_gammaincc,
)

pytestmark = pytest.mark.unit


def _speckle_hist(rng, mean, shape, pixels=100_000, dark=0.0):
    x = rng.gamma(shape, mean / shape, pixels) + dark
    return np.bincount(np.clip(x, 0, SAT_LEVEL).astype(int), minlength=SAT_LEVEL + 1), x


@pytest.mark.parametrize("a, x", [(1, 0.5), (3, 2.0), (5, 12.0), (2, 40.0), (10, 3.0)])
def test_log_gammaincc_matches_integer_closed_form(a, x):
    exact = math.exp(-x) * sum(x ** k / math.factorial(k) for k in range(a))
    assert math.exp(log_gammaincc(a, x)) == pytest.approx(exact, rel=1e-10)


@pytest.mark.parametrize("x", [1e-3, 0.1, 2.0, 30.0])
def test_log_gammaincc_matches_half_integer_closed_form(x):
    assert math.exp(log_gammaincc(0.5, x)) == pytest.approx(math.erfc(math.sqrt(x)), rel=1e-10)


def test_log_gammaincc_deep_tail_does_not_underflow():
    # Q(2, 800) ~ e^-800: far below float range, but its log is finite.
    assert log_gammaincc(2.0, 800.0) == pytest.approx(-800 + math.log(801.0), rel=1e-12)
    assert log_gammaincc(3.0, 0.0) == 0.0


@pytest.mark.parametrize("shape", [1.5, 3.0, 6.0, 12.0])
@pytest.mark.parametrize("scale_up", [1.0, 2.0, 4.0])
def test_censored_fit_recovers_true_mean_of_clipped_frame(shape, scale_up):
    rng = np.random.default_rng(int(shape * 10 + scale_up))
    true_mean = 300.0 * scale_up
    hist, x = _speckle_hist(rng, true_mean, shape)
    sat_frac = hist[SAT_LEVEL] / hist.sum()
    if 1 - sat_frac < 0.25:
        pytest.skip("too clipped for the fit by design")
    assert censored_gamma_mean(hist) == pytest.approx(x.mean(), rel=0.02)


def test_censored_fit_subtracts_dark():
    rng = np.random.default_rng(7)
    hist, x = _speckle_hist(rng, 700.0, 4.0, dark=40.0)
    assert censored_gamma_mean(hist, dark=40.0) == pytest.approx(x.mean() - 40.0, rel=0.02)


def test_plain_mean_when_barely_saturated():
    rng = np.random.default_rng(1)
    hist, x = _speckle_hist(rng, 200.0, 4.0)
    est = estimate_signal(hist, dark=10.0)
    assert est.kind == KIND_PLAIN and est.clean
    assert est.sat_frac <= 0.01
    assert est.mean_raw == pytest.approx(histogram_mean(hist))
    assert est.signal == pytest.approx(est.mean_raw - 10.0)


def test_fit_when_saturated_but_mostly_on_scale():
    rng = np.random.default_rng(2)
    hist, x = _speckle_hist(rng, 800.0, 6.0)
    est = estimate_signal(hist)
    assert est.kind == KIND_FIT and not est.clean
    assert est.mean_raw < x.mean()          # the plain mean reads low
    assert est.signal == pytest.approx(x.mean(), rel=0.02)


def test_clipped_when_mostly_saturated():
    hist = np.zeros(SAT_LEVEL + 1)
    hist[SAT_LEVEL] = 800
    hist[900] = 200
    est = estimate_signal(hist)
    assert est.kind == KIND_CLIPPED and est.signal is None and not est.clean


def test_basic_method_always_uses_plain_mean():
    hist = np.zeros(SAT_LEVEL + 1)
    hist[SAT_LEVEL] = 900
    hist[500] = 100
    est = estimate_signal(hist, method=METHOD_BASIC)
    assert est.kind == KIND_PLAIN
    assert est.signal == pytest.approx(0.9 * 1023 + 0.1 * 500)


def test_rejects_empty_histogram_and_unknown_method():
    with pytest.raises(ValueError):
        estimate_signal(np.zeros(SAT_LEVEL + 1))
    with pytest.raises(ValueError):
        estimate_signal(np.ones(SAT_LEVEL + 1), method="fancy")


def test_matches_scipy_reference_fit():
    """Same answer as the reference implementation's scipy-based fit."""
    stats = pytest.importorskip("scipy.stats")
    optimize = pytest.importorskip("scipy.optimize")
    rng = np.random.default_rng(11)
    hist, _ = _speckle_hist(rng, 650.0, 3.0, dark=20.0)
    dark, sat = 20.0, SAT_LEVEL

    v = np.arange(len(hist))
    x = v[:sat] - dark + 0.5
    h = hist[:sat]
    n_sat = hist[sat:].sum()
    keep = x > 0
    x, h = x[keep], h[keep]
    mu = np.average(x, weights=h)
    var = np.average((x - mu) ** 2, weights=h)

    def nll(p):
        k, s = np.exp(p)
        return -((h * stats.gamma.logpdf(x, k, scale=s)).sum()
                 + n_sat * stats.gamma.logsf(sat - dark, k, scale=s))

    k, s = np.exp(optimize.minimize(nll, np.log([mu ** 2 / var, var / mu]),
                                    method="Nelder-Mead").x)
    assert censored_gamma_mean(hist, dark) == pytest.approx(k * s, rel=2e-3)
    assert estimate_signal(hist, dark, METHOD_FIT).kind == KIND_FIT
