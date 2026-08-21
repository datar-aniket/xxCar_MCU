#!/usr/bin/env python3
"""Deterministic checks for the host-side alignment solver."""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

REPO = pathlib.Path(__file__).resolve().parents[1]

from align_solve import (AlignError, SNAP_LIMIT_DEG, is_mirrored,
                         orthonormalise, snap)
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
    print("align_solve: rotation table verified against rotation.c - OK")


if __name__ == "__main__":
    main()
