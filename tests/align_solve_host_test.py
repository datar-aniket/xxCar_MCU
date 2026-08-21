#!/usr/bin/env python3
"""Deterministic checks for the host-side alignment solver."""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

REPO = pathlib.Path(__file__).resolve().parents[1]

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


def main():
    test_table_matches_c()
    test_table_is_proper_signed_permutations()
    test_the_known_duplicate_pairs_really_are_duplicates()
    test_identity_is_rotation_none()
    print("align_solve: rotation table verified against rotation.c - OK")


if __name__ == "__main__":
    main()
