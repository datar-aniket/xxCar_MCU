#!/usr/bin/env python3
"""Run the guided sensor alignment against a board.

Six static positions read each IMU's rotation straight out of gravity, then one
rotation sweep solves the magnetometer and optical flow against them. Every
solve is align_solve's; this file is only the session - prompting, capturing,
and refusing to go on when a capture is not good enough.

Terminal driven rather than graphical, deliberately: it needs no display, so it
runs over ssh on the bench next to the vehicle, and every decision it makes is
visible in the scrollback afterwards when a result needs explaining.

    tools/align_run.py --port /dev/ttyACM0
    tools/align_run.py --port /dev/ttyACM0 --southern
    tools/align_run.py --replay session.json      # solve a saved capture
"""

import argparse
import json
import pathlib
import queue
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from align_solve import (AlignError, POSITIONS, _MIN_EXCITATION, _excitation,
                         _check_position, integrate_attitude, solve_alignment)
from rotation_table import ROTATIONS

# What the operator is actually asked to do. POSITIONS names the axis and sign;
# these name the physical act, which is the only thing that can be got wrong.
PROMPTS = {
    "level": "sitting normally, wheels down",
    "inverted": "upside down, roof down",
    "nose_up": "nose pointing at the ceiling",
    "nose_down": "nose pointing at the floor",
    "left_down": "rolled onto its LEFT side",
    "right_down": "rolled onto its RIGHT side",
}

# Order matters: alternating an axis with its opposite keeps the operator
# moving through pairs, so a skipped position is obvious at the time.
ORDER = ("level", "inverted", "nose_up", "nose_down",
         "left_down", "right_down")

IMUS = {"imu0": ("sensor_accel0", "sensor_gyro0"),
        "imu1": ("sensor_accel1", "sensor_gyro1")}

SWEEP_SECONDS = 25.0

# Far looser than cal6's 0.08. Alignment picks among 24 discrete rotations and
# tolerates a 45-degree error, so calibration-grade stillness would reject
# positions it is perfectly happy with.
STILL_LIMIT = 0.5


class Session:
    """One board connection, and the commands this procedure needs."""

    def __init__(self, port, baud=115200):
        from cal_link import Link

        self.q = queue.Queue()
        self.link = Link(port, baud, self.q)
        self.link.start()
        self.sensors = {}
        self.link.send("hello")
        self.link.send("list")
        time.sleep(0.5)
        self._drain_json()

    def close(self):
        self.link.send("align stop")
        self.link.send("stop")
        time.sleep(0.2)
        self.link.close()

    def _drain_json(self):
        out = []

        while True:
            try:
                kind, payload = self.q.get_nowait()
            except queue.Empty:
                return out

            if kind == "json":
                out.append(payload)

                if payload.get("evt") == "sensor":
                    self.sensors[payload.get("name")] = payload.get("id")
            elif kind == "error":
                print(f"  link: {payload}")

    def wait_json(self, evt, timeout=8.0):
        """Wait for one event, ignoring anything else."""
        deadline = time.time() + timeout

        while time.time() < deadline:
            try:
                kind, payload = self.q.get(timeout=0.2)
            except queue.Empty:
                continue

            if kind != "json":
                continue

            if payload.get("evt") == "error":
                raise AlignError(payload.get("msg", "board refused"))

            if payload.get("evt") == evt:
                return payload

        raise AlignError(f"board did not answer with {evt!r} in time")

    def still(self, sensor):
        self.link.send(f"still {sensor}")
        got = self.wait_json("still", timeout=10.0)
        return np.array(got["mean"], dtype=float), \
            np.array(got["sd"], dtype=float)

    def sweep(self, names, hz, seconds):
        """Stream several sensors and collect them, keyed by name."""
        by_id = {}

        for name in names:
            if name not in self.sensors:
                raise AlignError(f"board does not publish {name}")

            by_id[self.sensors[name]] = name

        self._drain_json()
        self.link.send("align %d %s" % (hz, " ".join(names)))
        self.wait_json("ok")

        rows = {name: [] for name in names}
        deadline = time.time() + seconds
        last_report = 0.0

        while time.time() < deadline:
            try:
                kind, payload = self.q.get(timeout=0.2)
            except queue.Empty:
                continue

            if kind != "batch":
                continue

            sid, _seq, _t0, _dt, batch = payload
            name = by_id.get(sid)

            if name is not None:
                rows[name].extend(batch)

            now = time.time()

            if now - last_report > 1.0:
                last_report = now
                self._report_progress(rows, deadline - now, hz)

        self.link.send("align stop")
        return {k: np.array(v, dtype=float) for k, v in rows.items()}

    def _report_progress(self, rows, remaining, hz):
        gyro = rows.get("sensor_gyro0")

        if not gyro or len(gyro) < 20:
            print(f"\r  collecting... {remaining:4.1f}s left", end="", flush=True)
            return

        att = integrate_attitude(np.array(gyro, dtype=float)[:, :3], 1.0 / hz)
        exc = _excitation(att)
        bars = "  ".join(
            "%s[%-8s]" % (axis, "#" * min(8, int(8 * e / _MIN_EXCITATION)))
            for axis, e in zip("xyz", exc))
        ok = "READY" if (exc >= _MIN_EXCITATION).all() else "keep turning"
        print(f"\r  {bars}  {ok}   {remaining:4.1f}s left  ",
              end="", flush=True)


def capture_positions(session):
    """Walk the six positions, refusing anything that is not good enough."""
    out = {name: {} for name in IMUS}

    for position in ORDER:
        while True:
            print(f"\nPut the vehicle {PROMPTS[position]}, hold it still,")
            input("then press Enter (Ctrl-C to abort): ")

            try:
                readings = {}

                for imu, (accel, _gyro) in IMUS.items():
                    mean, sd = session.still(accel)

                    if (sd > STILL_LIMIT).any():
                        raise AlignError(
                            f"{imu} was still moving (sd "
                            f"{sd[0]:.2f} {sd[1]:.2f} {sd[2]:.2f}, limit "
                            f"{STILL_LIMIT:.2f}) - hold it steadier")

                    # Check it HERE, while the operator is still holding the
                    # vehicle, rather than at the end when nothing can be done
                    # about it.
                    _check_position(position, mean)
                    readings[imu] = mean

            except AlignError as exc:
                print(f"  rejected: {exc}")
                continue

            for imu, mean in readings.items():
                out[imu][position] = mean
                print(f"  {imu}: {mean[0]:+7.3f} {mean[1]:+7.3f} "
                      f"{mean[2]:+7.3f} m/s^2")

            break

    return out


def capture_sweep(session, hz=100):
    """One rotation sweep, repeated until all three axes are exercised."""
    names = ["sensor_gyro0", "sensor_gyro1", "sensor_accel0", "sensor_mag0"]

    if "flow" in session.sensors:
        names.append("flow")

    while True:
        print(f"\nRotate the whole vehicle through ALL THREE axes for "
              f"{SWEEP_SECONDS:.0f}s - roll it, pitch it, yaw it. Keep it in "
              f"one place;\nturning it is what is measured, carrying it is "
              f"not.")
        input("Press Enter to start: ")

        rows = session.sweep(names, hz, SWEEP_SECONDS)
        print()

        gyro = rows.get("sensor_gyro0")

        if gyro is None or len(gyro) < 100:
            print("  rejected: almost no gyro data arrived")
            continue

        att = integrate_attitude(gyro[:, :3], 1.0 / hz)
        exc = _excitation(att)

        if (exc < _MIN_EXCITATION).any():
            weak = ", ".join("xyz"[i] for i in np.nonzero(
                exc < _MIN_EXCITATION)[0])
            print(f"  rejected: too little rotation about {weak} "
                  f"({np.round(exc, 3)}) - the solve would be "
                  "rank-deficient")
            continue

        return rows, hz


def build_session(positions, rows, hz, imu0_matrix):
    """Assemble what solve_alignment wants from what was captured."""
    n = min(len(v) for v in rows.values() if len(v))
    gyro0 = rows["sensor_gyro0"][:n, :3]

    # The magnetometer's dip test needs UP in the sweep's earth frame, which
    # means the accelerometer expressed in the VEHICLE frame - so it has to be
    # rotated by the IMU result the static positions just produced.
    accel = (imu0_matrix @ rows["sensor_accel0"][:n, :3].T).T

    sweep = {"dt": 1.0 / hz, "accel": accel,
             "gyro": {"imu0": gyro0}, "mag": rows["sensor_mag0"][:n, :3]}

    if len(rows.get("sensor_gyro1", [])) >= n:
        sweep["gyro"]["imu1"] = rows["sensor_gyro1"][:n, :3]

    if len(rows.get("flow", [])) >= n:
        # Flow reports integrated angle over its window; rate is that over the
        # integration time. Columns 4..6 are the gyro channels.
        flow = rows["flow"][:n]
        window = np.maximum(flow[:, 0:1], 1.0) * 1e-6
        sweep["gyro"]["flow"] = flow[:, 4:7] / window

    return {"positions": positions, "sweep": sweep}


def report(results):
    print("\n" + "=" * 68)
    print("ALIGNMENT RESULT")
    print("=" * 68)
    writable = {}

    for name in ("imu0", "imu1", "mag", "flow"):
        r = results.get(name)

        if r is None:
            continue

        if r.get("error"):
            print(f"  {name:5s} REFUSED  {r['error']}")
            continue

        if r.get("mirrored"):
            print(f"  {name:5s} MIRRORED - axes are reflected, not rotated.")
            print("           No rotation value can express this. It is "
                  "either a known")
            print("           part property that belongs in code, or a "
                  "wiring fault.")
            continue

        cross = r.get("cross_check")
        note = ""

        if cross is False:
            note = "  <-- GYRO DISAGREES with the accelerometer"
        elif cross is True:
            note = "  (gyro agrees)"

        print(f"  {name:5s} rotation {r['enum']:2d}   "
              f"{r['snap_deg']:5.2f} deg off{note}")
        writable[name] = r["enum"]

    print("=" * 68)
    return writable


def commit(session, writable):
    param = {"imu0": "SENS_IMU0_ROT", "imu1": "SENS_IMU1_ROT",
             "mag": "SENS_MAG0_ROT"}
    lines = [f"set {param[k]} {v}" for k, v in writable.items() if k in param]

    if not lines:
        print("Nothing to write.")
        return

    print("\nWould write:")

    for line in lines:
        print(f"  {line}")

    print("  set SENS_BOARD_ROT 0")
    print("\n(flow is solved but not written - no parameter reads it yet)")

    if input("\nCommit these? [y/N] ").strip().lower() != "y":
        print("Not written.")
        return

    for line in lines:
        session.link.send(line)

    session.link.send("set SENS_BOARD_ROT 0")
    session.link.send("commit")
    time.sleep(0.5)
    print("Written and committed.")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="serial port, e.g. /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--southern", action="store_true",
                    help="southern hemisphere: the field dips UP")
    ap.add_argument("--save", help="write the raw capture here")
    ap.add_argument("--replay", help="solve a saved capture instead")
    args = ap.parse_args()

    if args.replay:
        raw = json.loads(pathlib.Path(args.replay).read_text())
        positions = {k: {p: np.array(v) for p, v in d.items()}
                     for k, d in raw["positions"].items()}
        rows = {k: np.array(v) for k, v in raw["rows"].items()}
        imu0 = solve_alignment(
            {"positions": positions,
             "sweep": {"dt": 1.0 / raw["hz"], "accel": np.zeros((2, 3)),
                       "gyro": {"imu0": np.zeros((2, 3))}}})["imu0"]
        session = build_session(positions, rows, raw["hz"],
                                np.array(imu0["matrix"]))
        report(solve_alignment(session, dip_down=not args.southern))
        return 0

    if not args.port:
        ap.error("--port is required unless --replay is used")

    session = Session(args.port, args.baud)

    try:
        positions = capture_positions(session)

        # The IMU has to be solved before the sweep can be interpreted: the
        # magnetometer's dip test needs the accelerometer in the vehicle frame.
        imu0 = solve_alignment({"positions": {"imu0": positions["imu0"]},
                                "sweep": {"dt": 0.01,
                                          "accel": np.zeros((2, 3)),
                                          "gyro": {"imu0": np.zeros((2, 3))}}})

        if imu0["imu0"].get("error"):
            print(f"\nimu0 could not be solved: {imu0['imu0']['error']}")
            print("The sweep cannot be interpreted without it. Stopping.")
            return 1

        rows, hz = capture_sweep(session)

        if args.save:
            pathlib.Path(args.save).write_text(json.dumps(
                {"hz": hz,
                 "positions": {k: {p: list(map(float, v))
                                   for p, v in d.items()}
                               for k, d in positions.items()},
                 "rows": {k: v.tolist() for k, v in rows.items()}}))
            print(f"  raw capture saved to {args.save}")

        full = build_session(positions, rows, hz,
                             np.array(imu0["imu0"]["matrix"]))
        results = solve_alignment(full, dip_down=not args.southern)
        writable = report(results)

        if writable:
            commit(session, writable)

        return 0
    except KeyboardInterrupt:
        print("\nAborted. Nothing was written.")
        return 1
    finally:
        session.close()


if __name__ == "__main__":
    sys.exit(main())
