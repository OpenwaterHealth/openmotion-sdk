"""Frame brightness estimation for Optimize Laser, saturation-aware.

A speckle frame's intensity distribution is wide, so once a noticeable
fraction of pixels clips at full scale (bin 1023) the plain histogram mean
reads low. :func:`estimate_signal` tests the saturated fraction directly and,
for a frame that is saturated but still mostly on scale, estimates the true
mean with a censored gamma fit (:func:`censored_gamma_mean`): saturated
pixels count as "somewhere above full scale" rather than as exactly 1023.

Numpy only. The reference implementation (bloodflow-app
docs/auto_exposure/exposure_control.py) used scipy for the gamma functions
and the optimizer; neither the SDK nor the app depends on scipy, so the
regularized incomplete gamma function and a Nelder-Mead minimizer are
implemented here.

All values are DN on the 10-bit histogram scale. "Signal" means
dark-subtracted: ``mean - dark``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np

SAT_LEVEL = 1023            # 10-bit full scale: the last histogram bin
SAT_FRAC_MAX = 0.01         # above this saturated fraction the plain mean is biased low
MIN_UNSAT_FRAC = 0.25       # below this on-scale fraction the frame is too clipped to fit

METHOD_FIT = "fit"          # Option 1: censored gamma fit for saturated frames
METHOD_BASIC = "basic"      # Option 2: always the plain mean

KIND_PLAIN = "plain"        # plain mean (few saturated pixels, or METHOD_BASIC)
KIND_FIT = "fit"            # censored gamma estimate
KIND_CLIPPED = "clipped"    # too clipped to estimate

_TINY = 1e-300
_CF_EPS = 1e-15
_MAX_SERIES_TERMS = 1000


@dataclass(frozen=True)
class SignalEstimate:
    """Dark-subtracted brightness of one frame (or of summed frames).

    ``signal`` is None when the frame is too clipped to estimate
    (``kind == KIND_CLIPPED``). ``clean`` is True only for a plain-mean
    estimate, i.e. one that saturation did not bias.
    """

    signal: float | None
    kind: str
    mean_raw: float         # plain histogram mean, before dark subtraction
    sat_frac: float         # fraction of pixels in the saturated bin

    @property
    def clean(self) -> bool:
        return self.kind == KIND_PLAIN


def histogram_mean(hist: np.ndarray) -> float:
    """Plain mean DN of a histogram (bin index = DN)."""
    h = np.asarray(hist, dtype=np.float64)
    n = h.sum()
    if n <= 0:
        return float("nan")
    return float(np.dot(np.arange(h.size, dtype=np.float64), h) / n)


def estimate_signal(hist: np.ndarray, dark: float = 0.0, method: str = METHOD_FIT,
                    sat: int = SAT_LEVEL) -> SignalEstimate:
    """Dark-subtracted brightness of a frame, corrected for saturation.

    ``method == METHOD_BASIC`` always returns the plain mean (Option 2: no
    fitting; a saturated frame reads low, so steps under-correct but never
    overshoot). With ``METHOD_FIT``:

    * at most :data:`SAT_FRAC_MAX` saturated: plain mean;
    * saturated, but at least :data:`MIN_UNSAT_FRAC` on scale: censored
      gamma estimate;
    * otherwise: too clipped (``signal=None``).
    """
    if method not in (METHOD_FIT, METHOD_BASIC):
        raise ValueError(f"unknown method {method!r}")
    h = np.asarray(hist, dtype=np.float64)
    n = h.sum()
    if n <= 0:
        raise ValueError("histogram is empty")
    dark = float(dark)
    sat_frac = float(h[sat:].sum() / n)
    mean_raw = float(np.dot(np.arange(h.size, dtype=np.float64), h) / n)

    if method == METHOD_BASIC or sat_frac <= SAT_FRAC_MAX:
        return SignalEstimate(mean_raw - dark, KIND_PLAIN, mean_raw, sat_frac)
    if 1.0 - sat_frac < MIN_UNSAT_FRAC:
        return SignalEstimate(None, KIND_CLIPPED, mean_raw, sat_frac)
    return SignalEstimate(censored_gamma_mean(h, dark, sat), KIND_FIT, mean_raw, sat_frac)


def censored_gamma_mean(hist: np.ndarray, dark: float = 0.0, sat: int = SAT_LEVEL) -> float:
    """Estimate the unclipped, dark-subtracted mean of a saturated frame.

    Maximum-likelihood gamma fit to the on-scale bins, with the saturated
    pixels entering the likelihood through the survival function at full
    scale. The shape is fit per frame, so it adapts to speckle contrast and
    decorrelation (flow) without a stored calibration ratio.
    """
    h_all = np.asarray(hist, dtype=np.float64)
    x = np.arange(sat, dtype=np.float64) - dark + 0.5    # bin centers, dark-subtracted
    h = h_all[:sat]
    n_sat = float(h_all[sat:].sum())
    keep = (x > 0) & (h > 0)
    x, h = x[keep], h[keep]
    n_obs = float(h.sum())
    if n_obs <= 0:
        raise ValueError("no on-scale pixels above the dark level to fit")
    censor_at = float(sat) - dark

    # Starting guess from the on-scale pixels (method of moments).
    mu = float(np.dot(x, h) / n_obs)
    var = float(np.dot((x - mu) ** 2, h) / n_obs)
    if var <= 0:
        var = mu  # degenerate single-bin histogram: start at shape 1
    p0 = np.log([mu * mu / var, var / mu])

    log_x = np.log(x)
    sum_h_log_x = float(np.dot(h, log_x))
    sum_h_x = float(np.dot(h, x))

    def neg_log_likelihood(p: np.ndarray) -> float:
        if not np.all(np.isfinite(p)) or np.any(np.abs(p) > 50):
            return math.inf
        shape, scale = math.exp(p[0]), math.exp(p[1])
        ll = ((shape - 1.0) * sum_h_log_x - sum_h_x / scale
              - n_obs * (shape * math.log(scale) + math.lgamma(shape)))
        if n_sat:
            ll += n_sat * log_gammaincc(shape, censor_at / scale)
        return -ll

    p = _nelder_mead(neg_log_likelihood, p0)
    return float(math.exp(p[0] + p[1]))   # gamma mean = shape * scale


def log_gammaincc(a: float, x: float) -> float:
    """log Q(a, x): log of the regularized upper incomplete gamma function.

    Q(a, x) = Gamma(a, x) / Gamma(a), the gamma survival function at ``x``
    for unit scale. Series for P when x < a + 1, else the Lentz continued
    fraction for Q evaluated in log space, so deep tails don't underflow.
    """
    if a <= 0:
        raise ValueError("a must be positive")
    if x <= 0:
        return 0.0
    log_prefactor = -x + a * math.log(x) - math.lgamma(a)
    if x < a + 1.0:
        term = 1.0 / a
        total = term
        ap = a
        for _ in range(_MAX_SERIES_TERMS):
            ap += 1.0
            term *= x / ap
            total += term
            if abs(term) < abs(total) * _CF_EPS:
                break
        p = math.exp(log_prefactor + math.log(total))
        return math.log1p(-p) if p < 1.0 else math.log(_TINY)
    b = x + 1.0 - a
    c = 1.0 / _TINY
    d = 1.0 / b
    h = d
    for i in range(1, _MAX_SERIES_TERMS):
        an = -i * (i - a)
        b += 2.0
        d = an * d + b
        if abs(d) < _TINY:
            d = _TINY
        c = b + an / c
        if abs(c) < _TINY:
            c = _TINY
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1.0) < _CF_EPS:
            break
    return log_prefactor + math.log(h)


def _nelder_mead(f: Callable[[np.ndarray], float], x0: np.ndarray, *,
                 step: float = 0.1, xatol: float = 1e-4, fatol: float = 1e-4,
                 max_iter: int = 400) -> np.ndarray:
    """Minimize ``f`` from ``x0`` with the Nelder-Mead simplex method.

    Standard coefficients (reflect 1, expand 2, contract 0.5, shrink 0.5)
    and scipy's default absolute tolerances.
    """
    x0 = np.asarray(x0, dtype=np.float64)
    n = x0.size
    simplex = [x0]
    for i in range(n):
        x = x0.copy()
        x[i] += step
        simplex.append(x)
    fvals = [f(x) for x in simplex]

    for _ in range(max_iter):
        order = np.argsort(fvals)
        simplex = [simplex[i] for i in order]
        fvals = [fvals[i] for i in order]
        spread_x = max(float(np.max(np.abs(s - simplex[0]))) for s in simplex[1:])
        if spread_x <= xatol and abs(fvals[-1] - fvals[0]) <= fatol:
            break

        centroid = np.mean(simplex[:-1], axis=0)
        worst = simplex[-1]
        xr = centroid + (centroid - worst)
        fr = f(xr)
        if fr < fvals[0]:
            xe = centroid + 2.0 * (centroid - worst)
            fe = f(xe)
            simplex[-1], fvals[-1] = (xe, fe) if fe < fr else (xr, fr)
            continue
        if fr < fvals[-2]:
            simplex[-1], fvals[-1] = xr, fr
            continue
        if fr < fvals[-1]:
            xc = centroid + 0.5 * (xr - centroid)       # outside contraction
            fc = f(xc)
            accept = fc <= fr
        else:
            xc = centroid + 0.5 * (worst - centroid)    # inside contraction
            fc = f(xc)
            accept = fc < fvals[-1]
        if accept:
            simplex[-1], fvals[-1] = xc, fc
            continue
        for i in range(1, n + 1):                       # shrink toward the best
            simplex[i] = simplex[0] + 0.5 * (simplex[i] - simplex[0])
            fvals[i] = f(simplex[i])

    return simplex[int(np.argmin(fvals))]
