#!/usr/bin/env python3
"""Diagnose accelerometer axis signs from three gravity poses.

This is the short, read-only companion to align_run.py.  It captures only:

  1. nose down  -> vehicle -X points upward
  2. left down  -> vehicle -Y points upward
  3. Z down     -> vehicle -Z points upward (roof down, wheels up)

At rest an accelerometer measures specific force upward.  Those poses must
therefore become -X, -Y and -Z after the sensor-to-vehicle mapping.  The three
independent readings identify both axis order and every sign.  In particular,
they expose an all-axis negation as a reflection; no SENS_*_ROT value can
encode a reflection because all rotation enums are right-handed.

The utility never changes parameters.  SENS_IMU*_ROT is shared by accel and
gyro, so automatically applying an accelerometer-only diagnosis could break a
gyro mapping that is already correct.

Examples:

    tools/accel_sign_check.py --port /dev/pixhawk_6c --sensor accel0
    tools/accel_sign_check.py --replay accel-sign.json --current-rotation 2
"""

import argparse
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from align_run import STILL_LIMIT, Session
from align_solve import GRAVITY, AlignError, accel_sign_rotation
from rotation_table import ROTATIONS


POSES = (
    ("nose_down",
     "Point the NOSE straight at the floor (tail up).\n"
     "  This puts vehicle -X upward."),
    ("left_down",
     "Roll the vehicle onto its LEFT side (left side toward the floor).\n"
     "  This puts vehicle -Y upward."),
    ("z_down",
     "Turn the vehicle upside down: ROOF DOWN, WHEELS UP.\n"
     "  This puts vehicle -Z upward."),
)

EXPECTED = {
    "nose_down": np.array((-GRAVITY, 0.0, 0.0)),
    "left_down": np.array((0.0, -GRAVITY, 0.0)),
    "z_down": np.array((0.0, 0.0, -GRAVITY)),
}


def capture(session, sensor):
    """Capture the requested sensor, retrying a pose if it was moving."""
    if sensor not in session.sensors:
        raise AlignError(f"board has no sensor called {sensor!r}")

    if sensor not in session.present:
        raise AlignError(
            f"{sensor} is not publishing - run `sensor_status -t 1000` to "
            "check whether the driver is running")

    readings = {}
    deviations = {}

    for name, prompt in POSES:
        while True:
            print(f"\n{name.replace('_', ' ').upper()}\n{prompt}")
            input("Hold it still, then press Enter (Ctrl-C to abort): ")
            mean, sd = session.still(sensor)

            if (sd > STILL_LIMIT).any():
                print(f"  rejected: vehicle was moving (sd "
                      f"{sd[0]:.2f} {sd[1]:.2f} {sd[2]:.2f}, limit "
                      f"{STILL_LIMIT:.2f} m/s^2)")
                continue

            readings[name] = mean
            deviations[name] = sd
            print(f"  raw {sensor}: {mean[0]:+8.3f} {mean[1]:+8.3f} "
                  f"{mean[2]:+8.3f} m/s^2")
            break

    return readings, deviations


def _axis_map(matrix):
    lines = []

    for body_axis, body_name in enumerate("XYZ"):
        sensor_axis = int(np.flatnonzero(np.abs(matrix[body_axis]) > 0.5)[0])
        sign = "+" if matrix[body_axis, sensor_axis] > 0.0 else "-"
        lines.append(f"vehicle {body_name} = {sign} sensor "
                     f"{'XYZ'[sensor_axis]}")

    return lines


def _print_pose_table(poses, mapping, heading):
    print(f"\n{heading}")
    print("  pose          expected vehicle       resulting vehicle      error")

    for name, _prompt in POSES:
        expected = EXPECTED[name]
        got = mapping @ poses[name]
        error = np.linalg.norm(got - expected) / GRAVITY
        print(f"  {name:12s} "
              f"{expected[0]:+6.2f} {expected[1]:+6.2f} {expected[2]:+6.2f}"
              f"    {got[0]:+7.2f} {got[1]:+7.2f} {got[2]:+7.2f}"
              f"    {error:5.2f} g")


def report(poses, result, current_rotation=None):
    matrix = np.asarray(result["matrix"])
    determinant = float(np.linalg.det(matrix))

    print("\n" + "=" * 72)
    print("ACCELEROMETER SIGN RESULT (sensor -> vehicle FLU)")
    print("=" * 72)

    for line in _axis_map(matrix):
        print(f"  {line}")

    print("\n  signed-axis matrix:")
    for row in matrix.astype(int):
        print("    [ %2d %2d %2d ]" % tuple(row))

    hand = "REFLECTION" if result["mirrored"] else "right-handed rotation"
    print(f"\n  determinant: {determinant:+.0f} ({hand})")
    print(f"  worst three-pose residual: {result['residual_g']:.3f} g")
    _print_pose_table(poses, matrix, "Solved mapping applied to raw samples:")

    if result["mirrored"]:
        print("\nDIAGNOSIS: the accelerometer needs a reflected axis map. No "
              "SENS_IMU*_ROT or SENS_BOARD_ROT value can represent it.")
        print("If the gyro directions are already correct, do not change the "
              "shared IMU rotation to hide this; correct the accelerometer "
              "sign/frame at the driver boundary.")
    else:
        print(f"\nDIAGNOSIS: the gravity samples form rotation enum "
              f"{result['enum']}.")

    if current_rotation is not None:
        current = ROTATIONS[current_rotation]
        _print_pose_table(
            poses, current,
            f"Current rotation enum {current_rotation} applied to raw samples:")
        if np.array_equal(current, matrix):
            print(f"\nCurrent SENS_IMU*_ROT={current_rotation} matches the "
                  "three-pose result.")
        else:
            print(f"\nCurrent SENS_IMU*_ROT={current_rotation} does NOT match "
                  "the accelerometer sign result.")

    print("\nThis three-pose test does not estimate accelerometer bias. Use the "
          "six-pose alignment/calibration for a writable result.")
    print("=" * 72)


def _load(path):
    raw = json.loads(pathlib.Path(path).read_text())
    source = raw.get("poses", raw)
    return {name: np.asarray(source[name], dtype=float)
            for name, _prompt in POSES}


def _save(path, sensor, poses, deviations):
    pathlib.Path(path).write_text(json.dumps(
        {"sensor": sensor,
         "poses": {name: list(map(float, value))
                   for name, value in poses.items()},
         "sd": {name: list(map(float, value))
                for name, value in deviations.items()}},
        indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port, e.g. /dev/pixhawk_6c")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--sensor", choices=("accel0", "accel1"),
                        default="accel0")
    parser.add_argument("--current-rotation", type=int,
                        help="compare samples with this SENS_IMU*_ROT enum")
    parser.add_argument("--save", help="save raw pose means and deviations")
    parser.add_argument("--replay", help="re-run a saved capture without hardware")
    args = parser.parse_args()

    if args.current_rotation is not None and args.current_rotation not in ROTATIONS:
        parser.error(f"unsupported rotation enum {args.current_rotation}")

    if args.replay:
        poses = _load(args.replay)
    else:
        if not args.port:
            parser.error("--port is required unless --replay is used")

        session = Session(args.port, args.baud)

        try:
            poses, deviations = capture(session, args.sensor)
        except KeyboardInterrupt:
            print("\nAborted. Nothing was written.")
            return 130
        finally:
            session.close()

        if args.save:
            _save(args.save, args.sensor, poses, deviations)
            print(f"\nRaw capture saved to {args.save}")

    try:
        result = accel_sign_rotation(poses)
    except AlignError as exc:
        print(f"\nREFUSED: {exc}", file=sys.stderr)
        return 1

    report(poses, result, args.current_rotation)
    return 2 if result["mirrored"] else 0


if __name__ == "__main__":
    sys.exit(main())
