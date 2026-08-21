#!/usr/bin/env python3
"""Deterministic checks for the host-side alignment solver."""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

REPO = pathlib.Path(__file__).resolve().parents[1]

from align_solve import (GRAVITY, POSITIONS, SNAP_LIMIT_DEG, AlignError,
                         accel_rotation, is_mirrored, orthonormalise,
                         snap)
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
    print("align_solve: rotation table verified against rotation.c - OK")


if __name__ == "__main__":
    main()
