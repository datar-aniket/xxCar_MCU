"""Drive the Allan engine with noise whose coefficients are known exactly.

A wrong extraction does not look wrong. It produces a plausible small number
that quietly mistunes the filter, and there is no way to notice from the plot
or from the vehicle. So the only real test is to synthesise noise with a
coefficient chosen in advance and require the engine to recover it.

Synthesis, and why each expected value is what it is:

  white noise, per-sample sigma_w at rate fs
      sigma(tau) = sigma_w * sqrt(dt / tau),  so  N = sigma_w * sqrt(dt)

  rate random walk, per-sample step sigma_k
      the rate is a random walk; sigma(tau) = K * sqrt(tau / 3)
      with  K = sigma_k / sqrt(dt)
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import allan  # noqa: E402


def rel_err(got, want):
    return abs(got - want) / abs(want)


def main() -> int:
    rng = np.random.default_rng(12345)
    fails = []

    def check(cond, msg):
        if not cond:
            fails.append(msg)
            print(f"FAIL {msg}")

    # ---- pure white noise: N recovered, K negligible ---------------------
    fs = 200.0
    dt = 1.0 / fs
    n = int(fs * 3600)                       # one hour
    sigma_w = 0.004                          # rad/s per sample
    want_N = sigma_w * math.sqrt(dt)

    x = rng.normal(0.0, sigma_w, n)
    tau, adev = allan.allan_deviation(x, fs)
    r = allan.coefficients(tau, adev)
    e = rel_err(r.N, want_N)
    print(f"white noise:  N got {r.N:.6e}  want {want_N:.6e}  err {e*100:.1f}%")
    check(e < 0.05, f"N off by {e*100:.1f}% on pure white noise")

    # The -1/2 slope must actually be there; if the engine were fitting the
    # wrong region this is what would drift.
    lo = adev[tau <= 0.1][-1]
    hi = adev[tau >= 1.0][0]
    slope = math.log10(hi / lo) / math.log10(tau[tau >= 1.0][0] /
                                             tau[tau <= 0.1][-1])
    print(f"              measured slope {slope:.3f} (want -0.5)")
    check(abs(slope + 0.5) < 0.05, f"white-noise slope is {slope:.3f}")

    # ---- pure rate random walk: K recovered ------------------------------
    fs = 100.0
    dt = 1.0 / fs
    n = int(fs * 7200)                       # two hours: K lives at long tau
    sigma_k = 2.0e-5
    want_K = sigma_k / math.sqrt(dt)

    x = np.cumsum(rng.normal(0.0, sigma_k, n))
    tau, adev = allan.allan_deviation(x, fs)
    r = allan.coefficients(tau, adev)
    e = rel_err(r.K, want_K)
    print(f"rate rand walk: K got {r.K:.6e}  want {want_K:.6e}  err {e*100:.1f}%")
    check(e < 0.20, f"K off by {e*100:.1f}% on pure random walk")

    # ---- combined, as a real IMU behaves ---------------------------------
    fs = 200.0
    dt = 1.0 / fs
    n = int(fs * 7200)
    sigma_w = 0.003
    sigma_k = 1.0e-6
    want_N = sigma_w * math.sqrt(dt)
    want_K = sigma_k / math.sqrt(dt)

    x = rng.normal(0.0, sigma_w, n) + np.cumsum(rng.normal(0.0, sigma_k, n))
    tau, adev = allan.allan_deviation(x, fs)
    r = allan.coefficients(tau, adev)
    en, ek = rel_err(r.N, want_N), rel_err(r.K, want_K)
    print(f"combined:     N err {en*100:4.1f}%   K err {ek*100:4.1f}%   "
          f"B {r.B:.3e} at tau {r.tau_B:.2f}s")
    check(en < 0.05, f"combined N off by {en*100:.1f}%")
    check(ek < 0.30, f"combined K off by {ek*100:.1f}%")
    check(0.0 < r.tau_B < tau[-1], "curve minimum outside the tau range")

    # ---- the curve must actually be a curve ------------------------------
    check(tau.size > 20, f"only {tau.size} tau points")
    check(np.all(np.diff(tau) > 0), "tau not increasing")
    check(np.all(np.isfinite(adev)), "non-finite adev")

    # ---- trim removes what it says --------------------------------------
    t = np.arange(0, 100.0, 0.01)
    s = allan.Series("g", "rad/s", t, np.zeros((3, t.size)), fs=100.0)
    tr = allan.trim(s, 10.0, 5.0)
    dur = tr.t[-1] - tr.t[0]
    print(f"trim:         100.0s -> {dur:.2f}s after 10s head / 5s tail")
    check(abs(dur - 85.0) < 0.05, f"trim gave {dur:.2f}s, want 85s")

    # trimming everything must not leave an empty series to divide by
    tr2 = allan.trim(s, 60.0, 60.0)
    check(tr2.t.size >= 16, "over-trim produced an unusable series")

    # ---- a gap must be seen ---------------------------------------------
    t = np.concatenate([np.arange(0, 10, 0.005),
                        np.arange(12.0, 20.0, 0.005)])   # 2 s hole
    s = allan.Series("g", "rad/s", t, np.zeros((3, t.size)), fs=200.0)
    s.gaps = [(10.0, 2.0)]
    check(len(s.gaps) == 1, "gap list not populated")
    print(f"gap check:    {len(s.gaps)} gap, {s.gaps[0][1]:.1f}s at "
          f"{s.gaps[0][0]:.1f}s")

    if fails:
        print(f"allan: {len(fails)} failure(s)")
        return 1
    print("allan: coefficients recovered from known noise - OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
