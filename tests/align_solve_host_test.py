#!/usr/bin/env python3
"""Deterministic checks for the host-side alignment solver."""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

REPO = pathlib.Path(__file__).resolve().parents[1]

from align_solve import (GRAVITY, POSITIONS, SNAP_LIMIT_DEG, AlignError,
                         accel_rotation, integrate_attitude, is_mirrored,
                         earth_up, mag_rotation, orthonormalise,
                         rate_rotation, snap, solve_alignment)
from rotation_table import ROTATIONS

# ROLL_180_YAW_90 (10) and PITCH_180_YAW_270 (27) are the same rotation by
# different Euler routes, as are ROLL_180_YAW_270 (14) and PITCH_180_YAW_90
# (26). snap() resolves each pair to the lowest value, so a test that feeds in
# the higher one must expect the lower one back.
CANONICAL = {27: 10, 26: 14}


def test_table_matches_c():
    """The committed table must still be what rotation.c produces.

    Regenerating and comparing is the whole point: a hand-edit to either side
    that the other did not follow is invisible at runtime, because every
    rotation value is a plausible orientation.
    """
    import gen_rotation_table

    fresh = gen_rotation_table.generate()
    stored = (REPO / "tools" / "rotation_table.py").read_text()
    assert fresh == stored, (
        "rotation_table.py is stale - run tools/gen_rotation_table.py")


def test_table_is_proper_signed_permutations():
    # 26 supported enum values, 24 distinct rotations - see CANONICAL above.
    assert len(ROTATIONS) == 26
    distinct = {tuple(np.asarray(m).astype(int).flatten())
                for m in ROTATIONS.values()}
    assert len(distinct) == 24

    for value, matrix in ROTATIONS.items():
        assert matrix.shape == (3, 3)
        # Every entry is a signed axis permutation: orthogonal, det +1.
        assert np.allclose(matrix @ matrix.T, np.eye(3)), value
        assert np.isclose(np.linalg.det(matrix), 1.0), value
        # And exactly one non-zero per row and column.
        assert np.all(np.count_nonzero(matrix, axis=0) == 1), value
        assert np.all(np.count_nonzero(matrix, axis=1) == 1), value


def test_the_known_duplicate_pairs_really_are_duplicates():
    """Pinned so a future rotation.c edit that separates them is noticed.

    If these stop being equal, CANONICAL above is wrong and every test that
    round-trips a rotation through snap() starts lying.
    """
    assert np.allclose(ROTATIONS[10], ROTATIONS[27])
    assert np.allclose(ROTATIONS[14], ROTATIONS[26])


def test_identity_is_rotation_none():
    assert np.allclose(ROTATIONS[0], np.eye(3))


def _rot_z(deg):
    a = np.radians(deg)
    return np.array(((np.cos(a), -np.sin(a), 0.0),
                     (np.sin(a), np.cos(a), 0.0),
                     (0.0, 0.0, 1.0)))


def test_snap_finds_the_exact_value():
    for value, matrix in ROTATIONS.items():
        got, angle = snap(matrix)
        assert got == CANONICAL.get(value, value), (value, got)
        assert angle < 1e-6


def test_snap_is_deterministic_across_duplicates():
    """The two duplicate pairs must always resolve the same way, or the same
    physical mounting would be reported under two different names depending on
    floating-point noise.
    """
    assert snap(ROTATIONS[27])[0] == 10
    assert snap(ROTATIONS[26])[0] == 14


def test_snap_reports_the_angle_off():
    got, angle = snap(_rot_z(8.0) @ ROTATIONS[0])
    assert got == 0
    assert 7.9 < angle < 8.1


def test_snap_refuses_a_reflection():
    """A mirrored triad is not a rotation and must not be snapped to one.

    This is not hypothetical - the IST8310 is genuinely mirrored against the
    vehicle frame. Snapping it would return the nearest 180-degree rotation,
    which looks converged and is wrong.
    """
    mirrored = np.diag((1.0, -1.0, 1.0))
    assert is_mirrored(mirrored)

    try:
        snap(mirrored)
    except AlignError as exc:
        assert "mirror" in str(exc).lower()
    else:
        assert False, "snap accepted a reflection"


def test_orthonormalise_preserves_handedness():
    """Polar decomposition must not quietly repair a reflection into a
    rotation - that would destroy the evidence the flip check depends on.
    """
    noisy = np.diag((1.0, -1.0, 1.0)) + np.full((3, 3), 0.01)
    fixed = orthonormalise(noisy)

    assert np.allclose(fixed @ fixed.T, np.eye(3), atol=1e-9)
    assert np.linalg.det(fixed) < 0


def _positions_for(rotation, bias=(0.0, 0.0, 0.0), noise=0.0, seed=3):
    """Synthesise the six readings a sensor at `rotation` would produce.

    rotation maps sensor -> vehicle, so a vehicle-frame reading v gives a
    sensor-frame reading R.T @ v.
    """
    rng = np.random.default_rng(seed)
    out = {}

    for name, (axis, sign) in POSITIONS.items():
        vehicle = np.zeros(3)
        vehicle[axis] = sign * GRAVITY
        sensor = np.asarray(rotation).T @ vehicle + np.asarray(bias,
                                                               dtype=float)

        if noise:
            sensor = sensor + rng.normal(0.0, noise, 3)

        out[name] = sensor

    return out


def test_accel_recovers_every_representable_rotation():
    for value, matrix in ROTATIONS.items():
        got = accel_rotation(_positions_for(matrix))
        assert got["enum"] == CANONICAL.get(value, value), (value,
                                                            got["enum"])
        assert got["snap_deg"] < 1e-3
        assert not got["mirrored"]


def test_accel_cancels_a_large_bias():
    """Opposite-position averaging removes bias, which is what lets alignment
    run on an UNCALIBRATED sensor - and is also why it cannot confirm a
    calibration.
    """
    got = accel_rotation(
        _positions_for(ROTATIONS[2], bias=(0.9, -0.7, 1.3)))
    assert got["enum"] == 2
    assert got["snap_deg"] < 1e-3


def test_accel_survives_realistic_noise():
    got = accel_rotation(_positions_for(ROTATIONS[6], noise=0.05))
    assert got["enum"] == 6


def test_accel_reports_a_mirrored_sensor():
    mirrored = np.diag((1.0, -1.0, 1.0))
    got = accel_rotation(_positions_for(mirrored))
    assert got["mirrored"]
    assert got["enum"] is None


def test_accel_refuses_an_off_axis_mounting():
    """A 30-degree mounting is not an axis permutation. Snapping it to the
    nearest would look like a result and be wrong by 30 degrees.

    It must be refused for the RIGHT reason. A 30-degree mounting produces a
    dominance ratio of 1.73, so an axis-identification threshold set too high
    catches it first and blames the operator for a badly held position - a
    refusal that sends you to re-tip the vehicle forever over a mounting that
    will never improve.
    """
    off = _rot_z(30.0)
    try:
        accel_rotation(_positions_for(off))
    except AlignError as exc:
        assert "degrees from the nearest representable" in str(exc), str(exc)
        assert "tipped between" not in str(exc), str(exc)
    else:
        assert False, "accepted a 30-degree mounting"


def test_accel_refuses_an_ambiguous_position():
    """Tipped halfway between two axes, the dominant component does not
    dominate. That is an operator error and must be named, not averaged in.
    """
    bad = _positions_for(ROTATIONS[0])
    bad["nose_up"] = np.array((GRAVITY * 0.7, GRAVITY * 0.7, 0.0))

    try:
        accel_rotation(bad)
    except AlignError as exc:
        assert "nose_up" in str(exc)
    else:
        assert False, "accepted an ambiguous position"


def test_accel_refuses_a_missing_position():
    bad = _positions_for(ROTATIONS[0])
    del bad["left_down"]

    try:
        accel_rotation(bad)
    except AlignError as exc:
        assert "left_down" in str(exc)
    else:
        assert False, "accepted a missing position"


def test_accel_refuses_two_positions_on_the_same_axis():
    """An operator repeating a position instead of moving to the next one.
    Without this the solve is rank-2 and still passes an orthogonality check
    on the rows it did get.
    """
    bad = _positions_for(ROTATIONS[0])
    bad["left_down"] = bad["nose_down"]
    bad["right_down"] = bad["nose_up"]

    try:
        accel_rotation(bad)
    except AlignError as exc:
        assert "same axis" in str(exc).lower()
    else:
        assert False, "accepted duplicate positions"


def test_accel_refuses_non_perpendicular_axes():
    """A position held badly skews one row without shortening it.
    Orthonormalising first would hide that, so it is checked before repair.
    """
    bad = _positions_for(ROTATIONS[0])
    bad["nose_up"] = np.array((GRAVITY * 0.93, GRAVITY * 0.37, 0.0))
    bad["nose_down"] = -bad["nose_up"]

    try:
        accel_rotation(bad)
    except AlignError as exc:
        assert "perpendicular" in str(exc)
    else:
        assert False, "accepted non-perpendicular axes"


def _sweep(count=600, dt=0.02, seed=11):
    """A rich three-axis rotation, and the attitudes it produces."""
    rng = np.random.default_rng(seed)
    t = np.arange(count) * dt
    gyro = np.stack((1.2 * np.sin(2 * np.pi * 0.30 * t),
                     1.0 * np.sin(2 * np.pi * 0.21 * t + 1.0),
                     0.9 * np.sin(2 * np.pi * 0.13 * t + 2.0)), axis=1)
    gyro += rng.normal(0.0, 0.002, gyro.shape)
    return gyro, dt


def _mag_for(rotation, gyro, dt, field=0.45, dip_deg=60.0, noise=0.0,
             seed=5):
    """What a magnetometer at `rotation` reads through that sweep.

    Also returns the vehicle-frame accelerometer, because the dip check needs
    to know which way is up in the sweep's earth frame - integrate_attitude
    starts at identity, so that frame is not gravity-aligned by itself.
    """
    rng = np.random.default_rng(seed)
    attitude = integrate_attitude(gyro, dt)
    dip = np.radians(dip_deg)
    # Northern hemisphere in a z-UP frame: the field dips DOWN, so -z.
    earth = np.array((field * np.cos(dip), 0.0, -field * np.sin(dip)))
    # Specific force points UP, and the synthetic sweep starts level, so up in
    # the earth frame is +z.
    up_earth = np.array((0.0, 0.0, GRAVITY))

    out = []
    accel = []

    for c in attitude:
        body = c.T @ earth
        sensor = np.asarray(rotation).T @ body

        if noise:
            sensor = sensor + rng.normal(0.0, noise, 3)

        out.append(sensor)
        accel.append(c.T @ up_earth)

    return np.array(out), np.array(accel), attitude


def test_mag_recovers_every_representable_rotation():
    gyro, dt = _sweep()

    for value, matrix in ROTATIONS.items():
        mag, accel, attitude = _mag_for(matrix, gyro, dt)
        got = mag_rotation(mag, attitude, earth_up(accel, attitude))
        assert got["enum"] == CANONICAL.get(value, value), (value,
                                                            got["enum"])
        assert got["snap_deg"] < 1.0


def test_mag_recovers_the_field_magnitude():
    gyro, dt = _sweep()
    mag, accel, attitude = _mag_for(ROTATIONS[0], gyro, dt, field=0.47)
    got = mag_rotation(mag, attitude, earth_up(accel, attitude))
    assert abs(got["field"] - 0.47) < 0.01


def test_mag_detects_a_mirrored_sensor():
    """The IST8310 case: y negated. The solver must say mirrored rather than
    converge on the nearest 180-degree rotation.
    """
    gyro, dt = _sweep()
    mag, accel, attitude = _mag_for(np.diag((1.0, -1.0, 1.0)), gyro, dt)
    got = mag_rotation(mag, attitude, earth_up(accel, attitude))
    assert got["mirrored"]
    assert got["enum"] is None


def test_mag_refuses_a_single_axis_sweep():
    """Rotation about one axis only leaves the solve rank-deficient. It must
    refuse and name the axis, not return a confident wrong answer.
    """
    count, dt = 600, 0.02
    t = np.arange(count) * dt
    gyro = np.stack((1.2 * np.sin(2 * np.pi * 0.3 * t),
                     np.zeros(count), np.zeros(count)), axis=1)
    mag, accel, attitude = _mag_for(ROTATIONS[0], gyro, dt)

    try:
        mag_rotation(mag, attitude, np.array((0.0, 0.0, 1.0)))
    except AlignError as exc:
        assert "axis" in str(exc).lower()
    else:
        assert False, "accepted a single-axis sweep"


def test_mag_refuses_excess_noise():
    gyro, dt = _sweep()
    mag, accel, attitude = _mag_for(ROTATIONS[0], gyro, dt, noise=0.25)

    try:
        mag_rotation(mag, attitude, earth_up(accel, attitude))
    except AlignError as exc:
        assert "residual" in str(exc).lower() or "still" in str(exc).lower()
    else:
        assert False, "accepted a field that does not hold still"


def test_mag_works_in_the_southern_hemisphere():
    """Dip up instead of down. Without the hemisphere setting every southern
    board would be reported as mirrored.
    """
    gyro, dt = _sweep()
    mag, accel, attitude = _mag_for(ROTATIONS[2], gyro, dt, dip_deg=-55.0)
    got = mag_rotation(mag, attitude, earth_up(accel, attitude),
                       dip_down=False)
    assert got["enum"] == 2
    assert not got["mirrored"]


def test_mag_refuses_a_dip_too_shallow_to_judge():
    """Near the magnetic equator the field is almost horizontal, so its sign
    against gravity carries no information and a mirrored sensor genuinely
    cannot be told from a rotated one. Refuse rather than guess.
    """
    gyro, dt = _sweep()
    mag, accel, attitude = _mag_for(ROTATIONS[0], gyro, dt, dip_deg=3.0)

    try:
        mag_rotation(mag, attitude, earth_up(accel, attitude))
    except AlignError as exc:
        assert "dip" in str(exc).lower()
    else:
        assert False, "accepted a dip too shallow to judge handedness"


def test_earth_up_refuses_a_translated_sweep():
    """If the vehicle was carried rather than turned in place, the dynamic
    accelerations do not average out and up cannot be recovered.
    """
    gyro, dt = _sweep()
    attitude = integrate_attitude(gyro, dt)
    tiny = np.full((len(attitude), 3), 0.01)

    try:
        earth_up(tiny, attitude)
    except AlignError as exc:
        assert "up" in str(exc).lower()
    else:
        assert False, "accepted an accelerometer that cannot show up"


def test_rate_recovers_every_representable_rotation():
    gyro, dt = _sweep()
    attitude = integrate_attitude(gyro, dt)

    for value, matrix in ROTATIONS.items():
        target = np.array([np.asarray(matrix).T @ w for w in gyro])
        got = rate_rotation(gyro, target, attitude)
        assert got["enum"] == CANONICAL.get(value, value), (value,
                                                            got["enum"])


def test_rate_refuses_a_single_axis_sweep():
    count, dt = 400, 0.02
    t = np.arange(count) * dt
    gyro = np.stack((np.sin(2 * np.pi * 0.3 * t),
                     np.zeros(count), np.zeros(count)), axis=1)
    attitude = integrate_attitude(gyro, dt)

    try:
        rate_rotation(gyro, gyro.copy(), attitude)
    except AlignError as exc:
        assert "axis" in str(exc).lower()
    else:
        assert False, "accepted a single-axis sweep"


def _session(imu0=None, imu1=None, mag=None):
    imu0 = ROTATIONS[0] if imu0 is None else imu0
    imu1 = ROTATIONS[2] if imu1 is None else imu1
    mag = ROTATIONS[0] if mag is None else mag
    gyro, dt = _sweep()
    mag_series, accel_series, _attitude = _mag_for(mag, gyro, dt)

    return {
        "positions": {
            "imu0": _positions_for(imu0),
            "imu1": _positions_for(imu1),
        },
        "sweep": {
            "dt": dt,
            "accel": accel_series,
            "gyro": {
                "imu0": gyro,
                "imu1": np.array([np.asarray(imu1).T @ w for w in gyro]),
                "flow": np.array([ROTATIONS[4].T @ w for w in gyro]),
            },
            "mag": mag_series,
        },
    }


def test_solve_reports_every_sensor():
    got = solve_alignment(_session())

    assert got["imu0"]["enum"] == 0
    assert got["imu1"]["enum"] == 2
    assert got["mag"]["enum"] == 0
    assert got["flow"]["enum"] == 4
    assert all(r.get("error") is None for r in got.values())


def test_solve_flags_a_gyro_cross_check_disagreement():
    """The gyro must land on the same value the accelerometer did. They share
    a package, so a disagreement means one of the two streams is wrong - and
    which one matters, so it is named rather than averaged away.
    """
    session = _session()
    session["sweep"]["gyro"]["imu1"] = np.array(
        [ROTATIONS[6].T @ w for w in session["sweep"]["gyro"]["imu0"]])

    got = solve_alignment(session)
    assert got["imu1"]["cross_check"] is False


def test_solve_isolates_one_bad_sensor():
    """A sensor that refuses must not take the others down with it."""
    session = _session()
    del session["positions"]["imu1"]["level"]

    got = solve_alignment(session)
    assert got["imu1"]["error"] is not None
    assert got["imu0"]["error"] is None
    assert got["imu0"]["enum"] == 0


def test_end_to_end_through_the_runner():
    """The whole pipeline, from board-shaped streams to four rotations.

    Covers align_run.build_session, which the solver tests never touch: it is
    where the sensor-frame streams are assembled, the accelerometer is lifted
    into the vehicle frame for the dip test, and optical flow's integrated
    angle is turned back into a rate. A shape or scale mistake there produces a
    plausible wrong rotation, which is exactly the failure this whole procedure
    exists to prevent.
    """
    import align_run

    imu0, imu1, mag, flow = (ROTATIONS[0], ROTATIONS[2], ROTATIONS[8],
                             ROTATIONS[4])
    hz = 100.0
    gyro, dt = _sweep(count=1200, dt=1.0 / hz)
    mag_s, accel_s, _att = _mag_for(mag, gyro, dt)

    positions = {"imu0": _positions_for(imu0), "imu1": _positions_for(imu1)}
    rows = {
        "sensor_gyro0": (imu0.T @ gyro.T).T,
        "sensor_gyro1": (imu1.T @ gyro.T).T,
        "sensor_accel0": (imu0.T @ accel_s.T).T,
        "sensor_mag0": mag_s,
    }

    n = len(gyro)
    flow_rows = np.zeros((n, 7))
    flow_rows[:, 0] = 1e6 / hz                     # integration window, us
    flow_rows[:, 4:7] = (flow.T @ gyro.T).T / hz   # angle over that window
    rows["flow"] = flow_rows

    got = solve_alignment(
        align_run.build_session(positions, rows, hz, imu0))

    assert got["imu0"]["enum"] == 0, got["imu0"]
    assert got["imu1"]["enum"] == 2, got["imu1"]
    assert got["mag"]["enum"] == 8, got["mag"]
    assert got["flow"]["enum"] == 4, got["flow"]
    assert got["imu0"]["cross_check"] is True
    assert got["imu1"]["cross_check"] is True


def main():
    test_table_matches_c()
    test_table_is_proper_signed_permutations()
    test_the_known_duplicate_pairs_really_are_duplicates()
    test_identity_is_rotation_none()
    test_snap_finds_the_exact_value()
    test_snap_is_deterministic_across_duplicates()
    test_snap_reports_the_angle_off()
    test_snap_refuses_a_reflection()
    test_orthonormalise_preserves_handedness()
    test_accel_recovers_every_representable_rotation()
    test_accel_cancels_a_large_bias()
    test_accel_survives_realistic_noise()
    test_accel_reports_a_mirrored_sensor()
    test_accel_refuses_an_off_axis_mounting()
    test_accel_refuses_an_ambiguous_position()
    test_accel_refuses_a_missing_position()
    test_accel_refuses_two_positions_on_the_same_axis()
    test_accel_refuses_non_perpendicular_axes()
    test_mag_recovers_every_representable_rotation()
    test_mag_recovers_the_field_magnitude()
    test_mag_detects_a_mirrored_sensor()
    test_mag_refuses_a_single_axis_sweep()
    test_mag_refuses_excess_noise()
    test_mag_works_in_the_southern_hemisphere()
    test_mag_refuses_a_dip_too_shallow_to_judge()
    test_earth_up_refuses_a_translated_sweep()
    test_rate_recovers_every_representable_rotation()
    test_rate_refuses_a_single_axis_sweep()
    test_solve_reports_every_sensor()
    test_solve_flags_a_gyro_cross_check_disagreement()
    test_solve_isolates_one_bad_sensor()
    test_end_to_end_through_the_runner()
    print("align_solve: rotations, gravity columns, magnetometer dip "
          "and every refusal path verified - OK")


if __name__ == "__main__":
    main()
