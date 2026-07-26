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
import struct
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
    resyncs: int = 0                   # stream breaks stepped over
    rejected: int = 0                  # messages refused as implausible

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


# ULog message types we expect to see; anything else means the stream is lost.
_KNOWN_TYPES = set(b"BFAILMPQSORDT")

# A record is 2 bytes of msg_id plus the 24-byte accel/gyro payload.
_REC_LEN = 26

# No real sample steps backwards, and none leaps an hour. Both are cheap to
# check and are what stops a resync locking onto plausible-looking garbage.
_MAX_STEP_US = 3_600_000_000


def read_ulg(path, progress=None):
    """Read a ULog file, stepping over damage rather than giving up.

    pyulog abandons a file at the first malformed message, which for a long
    recording throws away hours of good data because of a few bytes. A
    calibration run is worth recovering: the parts of the stream on either
    side of a break are intact, and the framing makes it possible to find
    where it resumes.

    Resynchronisation has to be strict or it is worse than useless - a naive
    search locks onto byte patterns that look like a header and invents
    records. So a candidate must yield forty consecutive well-formed messages,
    every 'D' among them must carry a msg_id the file's own subscription list
    declared, and its timestamp must move forwards by a sane amount.

    Returns (subs, data, resyncs, rejected), where data maps msg_id ->
    (timestamps_us, raw payload bytes).
    """
    b = Path(path).read_bytes()
    n = len(b)
    if n < 16 or bytes(b[:8]) != bytes([0x55, 0x4C, 0x6F, 0x67,
                                        0x01, 0x12, 0x35, 0x01]):
        raise ValueError(f"{path}: not a ULog file")

    subs: dict[int, tuple[str, int]] = {}
    off = 16
    while off + 3 <= n:                       # prologue
        ln, ty = struct.unpack_from("<HB", b, off)
        if ty not in _KNOWN_TYPES or not ln or off + 3 + ln > n:
            break
        if ty == ord("A"):
            mid = struct.unpack_from("<H", b, off + 4)[0]
            subs[mid] = (b[off + 6:off + 3 + ln].decode(errors="replace"),
                         b[off + 3])
        if ty == ord("D"):
            break
        off += 3 + ln

    valid = set(subs)
    ts_out = {m: [] for m in valid}
    pay_out = {m: [] for m in valid}
    last: dict[int, int] = {}
    resyncs = rejected = 0

    while off + 3 <= n:
        ln, ty = struct.unpack_from("<HB", b, off)
        ok = ty in _KNOWN_TYPES and ln and off + 3 + ln <= n
        if ok and ty == ord("D"):
            if ln != _REC_LEN:
                ok = False
            else:
                mid = struct.unpack_from("<H", b, off + 3)[0]
                if mid not in valid:
                    ok = False
                else:
                    t = struct.unpack_from("<Q", b, off + 5)[0]
                    prev = last.get(mid)
                    if prev is not None and not (0 < t - prev < _MAX_STEP_US):
                        ok = False
                    else:
                        last[mid] = t
                        ts_out[mid].append(t)
                        pay_out[mid].append(b[off + 13:off + 3 + ln])
        if ok:
            off += 3 + ln
            continue

        rejected += 1
        for d in range(1, 8192):
            p = off + d
            good = 0
            while good < 40 and p + 3 <= n:
                l2, t2 = struct.unpack_from("<HB", b, p)
                if t2 not in _KNOWN_TYPES or not l2 or p + 3 + l2 > n:
                    break
                if t2 == ord("D") and (
                        l2 != _REC_LEN or
                        struct.unpack_from("<H", b, p + 3)[0] not in valid):
                    break
                good += 1
                p += 3 + l2
            if good >= 40:
                off += d
                resyncs += 1
                break
        else:
            break                              # no resync: the tail is gone

    data = {}
    for m in valid:
        if ts_out[m]:
            xyz = np.frombuffer(b"".join(pay_out[m]),
                                dtype="<f4").reshape(-1, 4)[:, :3]
            data[m] = (np.asarray(ts_out[m], dtype=np.int64),
                       np.ascontiguousarray(xyz.T, dtype=np.float32))
    return subs, data, resyncs, rejected


def load_session(paths, progress=None, channels=None) -> dict[str, Series]:
    """Concatenate every part of a session into one Series per channel.

    Timestamps are the sensor's own, in microseconds since boot, so they run
    continuously across parts - which is what makes the gap check meaningful.

    `channels` restricts what is kept. A full-rate overnight run is tens of
    millions of samples per axis, and holding all four sensors at once costs
    over a gigabyte; the caller can work one sensor at a time instead.
    """
    wanted = {name: (topic, mid, unit)
              for name, topic, mid, unit in CHANNELS
              if channels is None or name in channels}
    raw: dict[str, list] = {name: [] for name in wanted}
    resyncs: dict[str, int] = {name: 0 for name in wanted}
    rejects: dict[str, int] = {name: 0 for name in wanted}

    for i, path in enumerate(paths):
        if progress:
            progress(i, len(paths), Path(path).name)
        subs, data, nres, nrej = read_ulg(path)
        by_key = {(nm, mid): m for m, (nm, mid) in subs.items()}
        for name, (topic, mid, _unit) in wanted.items():
            m = by_key.get((topic, mid))
            if m is None or m not in data:
                continue
            t, xyz = data[m]
            raw[name].append((t.astype(np.float64), xyz))
            resyncs[name] += nres
            rejects[name] += nrej

    out: dict[str, Series] = {}
    for name, (_topic, _mid, unit) in wanted.items():
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
                           gaps=gaps, resyncs=resyncs[name],
                           rejected=rejects[name])
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
                  gaps=[g for g in series.gaps if t0 <= g[0] <= t1],
                  resyncs=series.resyncs, rejected=series.rejected)


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
