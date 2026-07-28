"""End-to-end test of the Allan window against synthetic .ulg parts.

Builds real ULog files with noise of known coefficients, spread across parts
the way the 100 MB rollover produces them, and drives the window's own load,
trim, compute and save paths.

This is the only test that exercises the join between parts. A session loader
that dropped a part, or concatenated them out of order, would still produce a
plausible Allan curve - just from less data, or from data with a fabricated
discontinuity in the middle - and nothing about the plot would look wrong.
"""

import json
import math
import struct
import sys
import tempfile
import tkinter as tk
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import allan  # noqa: E402

MAGIC = bytes([0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35, 0x01])
FMT = "sensor_{}:uint64_t timestamp;float x;float y;float z;float temperature;"


def msg(t, payload):
    return struct.pack("<HB", len(payload), ord(t)) + payload


def write_part(path, t0_us, samples, dt_us):
    """One complete ULog part: prologue then accel+gyro records."""
    out = bytearray(MAGIC + struct.pack("<Q", t0_us))
    out += msg("B", bytes(40))
    out += msg("F", FMT.format("accel").encode())
    out += msg("F", FMT.format("gyro").encode())
    out += msg("A", struct.pack("<BH", 0, 0) + b"sensor_accel")
    out += msg("A", struct.pack("<BH", 0, 1) + b"sensor_gyro")
    for k, (a, g) in enumerate(samples):
        t = t0_us + k * dt_us
        out += msg("D", struct.pack("<H", 0)
                   + struct.pack("<Qffff", t, a[0], a[1], a[2], 40.0))
        out += msg("D", struct.pack("<H", 1)
                   + struct.pack("<Qffff", t, g[0], g[1], g[2], 40.0))
    path.write_bytes(bytes(out))


def main() -> int:
    try:
        import pyulog  # noqa: F401
    except ImportError:
        print("allan_gui: pyulog missing - skipped")
        return 0

    fails = []

    def check(cond, msg_):
        if not cond:
            fails.append(msg_)
            print(f"FAIL {msg_}")

    rng = np.random.default_rng(7)
    tmp = Path(tempfile.mkdtemp())

    # 20 minutes at 200 Hz, split across three parts, with a KNOWN gyro N.
    fs, dt_us = 200.0, 5000
    total = int(fs * 1200)
    sigma_w = 0.005
    want_N = sigma_w * math.sqrt(1.0 / fs)

    gyro = rng.normal(0.0, sigma_w, (3, total))
    acc = rng.normal(0.0, 0.02, (3, total)) + np.array([[0.0], [0.0], [9.81]])

    bounds = [0, total // 3, 2 * total // 3, total]
    for pi in range(3):
        lo, hi = bounds[pi], bounds[pi + 1]
        rows = [((acc[0][k], acc[1][k], acc[2][k]),
                 (gyro[0][k], gyro[1][k], gyro[2][k])) for k in range(lo, hi)]
        write_part(tmp / f"log_007_{pi:02d}.ulg", 1_000_000 + lo * dt_us,
                   rows, dt_us)

    # a second, unrelated session must not be mixed in
    write_part(tmp / "log_008_00.ulg", 5_000_000,
               [((0, 0, 9.8), (0, 0, 0))] * 100, dt_us)

    # ---- session grouping -----------------------------------------------
    sessions = allan.find_sessions(tmp)
    check(sorted(sessions) == [7, 8], f"sessions {sorted(sessions)}")
    check(len(sessions[7]) == 3, f"session 7 has {len(sessions[7])} parts")
    check([p.name for p in sessions[7]] ==
          ["log_007_00.ulg", "log_007_01.ulg", "log_007_02.ulg"],
          "parts out of order")

    # ---- load and join ---------------------------------------------------
    load_progress = []
    series = allan.load_session(
        sessions[7], workers=2,
        progress=lambda i, n, name: load_progress.append((i, n, name)))
    g = series["gyro0"]
    print(f"  joined {len(sessions[7])} parts -> {g.xyz.shape[1]:,} samples, "
          f"{g.fs:.1f} Hz, {g.duration:.0f} s, {len(g.gaps)} gap(s)")
    check(g.xyz.shape[1] == total,
          f"joined {g.xyz.shape[1]} samples, want {total} - a part was lost")
    check(len(g.gaps) == 0, f"{len(g.gaps)} spurious gap(s) at the part joins")
    check(abs(g.fs - fs) < 1.0, f"rate {g.fs:.1f} != {fs}")
    check(len(load_progress) == 3,
          f"parallel loader reported {len(load_progress)} of 3 parts")
    check({p[2] for p in load_progress} ==
          {"log_007_00.ulg", "log_007_01.ulg", "log_007_02.ulg"},
          "parallel loader progress omitted or duplicated a part")

    # ---- coefficients survive the join -----------------------------------
    compute_progress = []
    parallel_results = allan.analyse_many(
        {"gyro0": g, "accel0": series["accel0"]}, workers=2,
        progress=lambda i, n, name:
        compute_progress.append((i, n, name)))
    res = parallel_results["gyro0"]
    err = abs(res[0].N - want_N) / want_N
    print(f"  gyro0 x: N {res[0].N:.4e} want {want_N:.4e} ({err*100:.1f}%)")
    check(err < 0.10, f"N off by {err*100:.1f}% after joining parts")
    check({p[2] for p in compute_progress} == {"gyro0", "accel0"},
          "parallel computation progress omitted a sensor")

    # ---- trim actually shortens -------------------------------------------
    tr = allan.trim(g, 120.0, 60.0)
    print(f"  trim 120/60: {g.duration:.0f}s -> {tr.duration:.0f}s")
    check(abs(tr.duration - (g.duration - 180.0)) < 2.0,
          f"trim gave {tr.duration:.0f}s")

    # ---- the window itself ------------------------------------------------
    try:
        root = tk.Tk()
        root.withdraw()
    except tk.TclError as exc:
        print(f"  no display, GUI half skipped ({exc})")
        return 1 if fails else 0

    # the main window supplies styles the child expects
    sys.path.insert(0, str(ROOT / "tools"))
    from cal_allan_win import AllanWindow

    saved = {}

    def on_save(params):
        saved.update(params)
        return True

    win = AllanWindow(root, on_save=on_save)
    win.folder = tmp
    win.sessions = sessions
    win.sess_cb["values"] = ["7 (3p)"]
    win.sess_cb.set("7 (3p)")
    win.series = series
    win.head.set(60.0)
    win.tail.set(30.0)
    win._describe()
    print(f"  health line: {win.health.cget('text').splitlines()[0][:70]}…")

    results = {n: allan.analyse(allan.trim(s, 60.0, 30.0))
               for n, s in series.items()}
    win._show(results)
    rows = win.tbl.get_children()
    check(len(rows) == len(series), f"{len(rows)} table groups")
    kids = win.tbl.get_children(rows[0])
    check(len(kids) == 3, f"{len(kids)} axis rows under a sensor")
    check(not win.save_btn.instate(["disabled"]), "Save disabled after compute")

    win._save()
    report = tmp / "allan_report.json"
    check(report.exists(), "no JSON report written")
    if report.exists():
        r = json.loads(report.read_text())
        check(r["trim_head_s"] == 60.0 and r["trim_tail_s"] == 30.0,
              "trim not recorded in the report")
        check("gyro0" in r["channels"], "channels missing from report")
        check("diagnostics" in r and "gyro0" in r["diagnostics"],
              "diagnostics missing from report")
        print(f"  report: {len(json.dumps(r))} bytes, "
              f"{len(r['channels'])} channels")

    # A twenty-minute run reaches its curve minimum near the end, so there is
    # no room to the right of it to fit rate random walk. K is honestly NaN,
    # and the right behaviour is to send the measurable coefficients and leave
    # _RW unset - not to send a NaN and rely on the board refusing it, which
    # would leave the operator believing it had been saved.
    measurable = {"IMU0_ACC_ND", "IMU0_GYR_ND"}
    check(measurable <= set(saved),
          f"measurable params missing: {sorted(measurable - set(saved))}")
    check("IMU0_GYR_BI" not in saved,
          "bias instability was sent without a measured knee")
    check("IMU0_GYR_RW" not in saved,
          "an unmeasurable rate random walk was sent anyway")
    check(all(math.isfinite(v) for v in saved.values()),
          "a non-finite value was about to be written to the board")
    check("not measurable" in win.health.cget("text"),
          "the skipped coefficients were not explained to the operator")
    print(f"  params handed to board: {len(saved)} finite "
          f"(IMU0_GYR_ND={saved['IMU0_GYR_ND']:.4e}); "
          f"_RW correctly withheld — run too short")

    # ...and on a run long enough, K IS measurable and must be sent. Without
    # this the test would pass just as happily against code that never sends
    # rate random walk at all.
    long_rng = np.random.default_rng(3)
    n_long = int(200.0 * 9000)                    # 2.5 hours
    x = (long_rng.normal(0.0, 0.003, n_long)
         + np.cumsum(long_rng.normal(0.0, 3e-7, n_long)))
    r_long = allan.coefficients(*allan.allan_deviation(x, 200.0))
    print(f"  2.5 h run: K = {r_long.K:.4e} (finite: "
          f"{math.isfinite(r_long.K)}), tau_B = {r_long.tau_B:.0f}s")
    check(math.isfinite(r_long.K),
          "K unmeasurable even on a 2.5 h run - the fit region is wrong")

    root.destroy()

    if fails:
        print(f"allan_gui: {len(fails)} failure(s)")
        return 1
    print("allan_gui: parts joined, trimmed, computed, saved - OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
