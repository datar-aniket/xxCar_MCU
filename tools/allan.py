"""Allan variance analysis for xxCar IMU logs.

Loads the .ulg parts a recording session produced, checks the dataset is
trustworthy, and extracts the three noise coefficients an EKF wants.

Kept free of any GUI so it can be tested against synthetic noise whose
coefficients are known exactly - which is the only way to believe the numbers
it reports. A wrong extraction here does not look wrong: it produces a
plausible small number that quietly mistunes the filter.

Coefficients, per IEEE Std 952 (sigma is the overlapping Allan deviation):

    white noise      sigma(tau) = N / sqrt(tau)          -> N at tau = 1 s
    bias instability sigma(tau) = B * sqrt(2 ln2 / pi)   -> B = min(sigma) / 0.664
    rate random walk sigma(tau) = K * sqrt(tau / 3)      -> K at tau = 3 s

N is angle random walk for a gyro and velocity random walk for an
accelerometer; it is also the noise density the EKF calls sigma_gyro /
sigma_accel. K is the bias random walk driving the EKF's bias process noise.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

G = 9.80665
RAD2DEG = 180.0 / math.pi

# Channels we analyse, and the ULog topic + multi-instance each comes from.
CHANNELS = (
    ("accel0", "sensor_accel", 0, "m/s^2"),
    ("gyro0", "sensor_gyro", 0, "rad/s"),
    ("accel1", "sensor_accel", 1, "m/s^2"),
    ("gyro1", "sensor_gyro", 1, "rad/s"),
)

PART_RE = re.compile(r"^log_(\d+)(?:_(\d+))?\.ulg$")


@dataclass
class Series:
    """One sensor's three axes, concatenated across every part of a session."""

    name: str
    unit: str
    t: np.ndarray                      # seconds, relative to the first sample
    xyz: np.ndarray                    # shape (3, n)
    fs: float = 0.0
    gaps: list = field(default_factory=list)   # (at_seconds, gap_seconds)

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0]) if self.t.size > 1 else 0.0


@dataclass
class Result:
    """Extracted coefficients for one axis."""

    N: float          # random walk, unit/sqrt(s) - the EKF's noise density
    B: float          # bias instability, unit
    K: float          # rate random walk, unit/s * sqrt(s)
    tau_B: float      # tau at the curve minimum, s
    tau: np.ndarray
    adev: np.ndarray


def find_sessions(folder: str | Path) -> dict[int, list[Path]]:
    """Group log_NNN_PP.ulg parts by session, each list in part order.

    Old single-file log_NNN.ulg names are treated as a session of one part, so
    a folder holding both still works.
    """
    folder = Path(folder)
    sessions: dict[int, list[tuple[int, Path]]] = {}
    for p in folder.iterdir():
        m = PART_RE.match(p.name)
        if not m:
            continue
        sess = int(m.group(1))
        part = int(m.group(2)) if m.group(2) is not None else 0
        sessions.setdefault(sess, []).append((part, p))
    return {s: [p for _, p in sorted(v)] for s, v in sorted(sessions.items())}


def load_session(paths, progress=None) -> dict[str, Series]:
    """Concatenate every part of a session into one Series per channel.

    Timestamps are the sensor's own, in microseconds since boot, so they run
    continuously across parts - which is what makes the gap check meaningful.
    """
    from pyulog import ULog

    raw: dict[str, list] = {name: [] for name, _, _, _ in CHANNELS}

    for i, path in enumerate(paths):
        if progress:
            progress(i, len(paths), path.name)
        ulog = ULog(str(path))
        for name, topic, mid, _unit in CHANNELS:
            for d in ulog.data_list:
                if d.name != topic or getattr(d, "multi_id", 0) != mid:
                    continue
                t = np.asarray(d.data["timestamp"], dtype=np.float64)
                xyz = np.vstack([np.asarray(d.data[a], dtype=np.float64)
                                 for a in ("x", "y", "z")])
                raw[name].append((t, xyz))

    out: dict[str, Series] = {}
    for name, _topic, _mid, unit in CHANNELS:
        chunks = raw[name]
        if not chunks:
            continue
        t = np.concatenate([c[0] for c in chunks])
        xyz = np.hstack([c[1] for c in chunks])

        order = np.argsort(t, kind="stable")
        t, xyz = t[order], xyz[:, order]

        t_s = (t - t[0]) / 1e6
        dt = np.diff(t_s)
        good = dt[dt > 0]
        fs = float(1.0 / np.median(good)) if good.size else 0.0

        # A gap is anything beyond five nominal sample intervals. Gaps matter
        # more than their size suggests: Allan variance assumes uniform
        # sampling, and a hole biases the long-tau end, which is the part
        # bias instability is read from.
        gaps = []
        if fs > 0:
            thresh = 5.0 / fs
            for idx in np.flatnonzero(dt > thresh):
                gaps.append((float(t_s[idx]), float(dt[idx])))

        out[name] = Series(name=name, unit=unit, t=t_s, xyz=xyz, fs=fs,
                           gaps=gaps)
    return out


def trim(series: Series, head_s: float, tail_s: float) -> Series:
    """Drop the first and last stretch of a run.

    Handling and settling at each end are not the noise process being
    measured, and they land in the long-tau region where they do the most
    damage.
    """
    if series.t.size == 0:
        return series
    t0 = series.t[0] + head_s
    t1 = series.t[-1] - tail_s
    keep = (series.t >= t0) & (series.t <= t1)
    if keep.sum() < 16:
        keep = np.ones_like(series.t, dtype=bool)
    return Series(name=series.name, unit=series.unit, t=series.t[keep],
                  xyz=series.xyz[:, keep], fs=series.fs,
                  gaps=[g for g in series.gaps if t0 <= g[0] <= t1])


def allan_deviation(x: np.ndarray, fs: float, points: int = 100):
    """Overlapping Allan deviation of a rate signal.

    Overlapping rather than plain: it reuses every sample at each averaging
    factor, so the long-tau end - where the data is thinnest and bias
    instability is read - has far more confidence for the same recording.
    """
    n = x.size
    dt = 1.0 / fs
    theta = np.cumsum(x) * dt                       # integrate to angle/velocity

    max_m = (n - 1) // 3                            # need N > 2m to average at all
    if max_m < 2:
        return np.array([]), np.array([])
    ms = np.unique(np.floor(
        np.logspace(0, math.log10(max_m), points)).astype(np.int64))
    ms = ms[ms >= 1]

    taus = np.empty(ms.size)
    adev = np.empty(ms.size)
    for i, m in enumerate(ms):
        tau = m * dt
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        avar = np.sum(d * d) / (2.0 * tau * tau * d.size)
        taus[i] = tau
        adev[i] = math.sqrt(avar)
    return taus, adev


def _fit_at(tau, adev, slope, tau_eval, lo, hi):
    """Fit a fixed-slope line in log-log over [lo,hi] and evaluate at tau_eval.

    Reading a single point off the curve is the textbook shortcut and is
    fragile: at tau = 1 s the white-noise asymptote is already contaminated by
    the bias-instability floor. Fitting the region whose local slope actually
    matches, then extrapolating, is what makes the number mean what it claims.
    """
    sel = (tau >= lo) & (tau <= hi) & (adev > 0)
    if sel.sum() < 3:
        return float("nan")
    lt = np.log10(tau[sel])
    la = np.log10(adev[sel])
    # fixed slope: only the intercept is free
    intercept = np.mean(la - slope * lt)
    return float(10.0 ** (intercept + slope * math.log10(tau_eval)))


def coefficients(tau: np.ndarray, adev: np.ndarray) -> Result:
    """Extract N, B and K from an Allan deviation curve."""
    if tau.size == 0:
        nan = float("nan")
        return Result(nan, nan, nan, nan, tau, adev)

    imin = int(np.argmin(adev))
    tau_B = float(tau[imin])
    B = float(adev[imin] / 0.664)

    # White noise lives left of the minimum; use the decade below it, bounded
    # away from the very shortest taus where filtering rolls the curve off.
    hi_n = max(tau[0] * 2.0, tau_B / 5.0)
    N = _fit_at(tau, adev, -0.5, 1.0, tau[0], hi_n)

    # Rate random walk lives right of the minimum.
    lo_k = tau_B * 2.0
    K = _fit_at(tau, adev, 0.5, 3.0, lo_k, tau[-1])

    return Result(N=N, B=B, K=K, tau_B=tau_B, tau=tau, adev=adev)


def analyse(series: Series, points: int = 100) -> list[Result]:
    """Per-axis coefficients for one sensor."""
    return [coefficients(*allan_deviation(series.xyz[i], series.fs, points))
            for i in range(3)]


def summarise(name: str, unit: str, res: list[Result]) -> dict:
    """Per-sensor summary in both SI and the units datasheets quote."""
    axes = "xyz"
    out = {"sensor": name, "unit": unit, "axes": {}}
    for i, r in enumerate(res):
        d = {"N": r.N, "B": r.B, "K": r.K, "tau_B": r.tau_B}
        if unit == "rad/s":
            # deg/sqrt(hour) for ARW, deg/hour for bias instability
            d["N_deg_sqrt_hr"] = r.N * RAD2DEG * 60.0
            d["B_deg_per_hr"] = r.B * RAD2DEG * 3600.0
        else:
            d["N_ug_sqrt_hz"] = r.N / G * 1e6      # micro-g/sqrt(Hz)
            d["B_ug"] = r.B / G * 1e6              # micro-g
        out["axes"][axes[i]] = d
    return out
