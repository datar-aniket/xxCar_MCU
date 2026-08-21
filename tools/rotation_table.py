#!/usr/bin/env python3
"""The representable rotations, GENERATED from apps/sensors/rotation.c.

Do not edit. Run tools/gen_rotation_table.py. The mapping is
verified against the C implementation by
tools/test-align-solve.sh, so the two cannot drift apart.

There are 26 supported enum values but only 24 DISTINCT
rotations: ROLL_180_YAW_90 (10) and PITCH_180_YAW_270 (27) are
the same rotation reached by different Euler routes, as are
ROLL_180_YAW_270 (14) and PITCH_180_YAW_90 (26). That is
inherent to the PX4 enum, which rotation.c reproduces exactly.
Callers matching a matrix back to a name must therefore choose
one deliberately - align_solve.snap() takes the lowest value.
"""

import numpy as np

ROTATIONS = {
    0: np.array(((1, 0, 0), (0, 1, 0), (0, 0, 1))).T,
    2: np.array(((0, 1, 0), (-1, 0, 0), (0, 0, 1))).T,
    4: np.array(((-1, 0, 0), (0, -1, 0), (0, 0, 1))).T,
    6: np.array(((0, -1, 0), (1, 0, 0), (0, 0, 1))).T,
    8: np.array(((1, 0, 0), (0, -1, 0), (0, 0, -1))).T,
    10: np.array(((0, 1, 0), (1, 0, 0), (0, 0, -1))).T,
    12: np.array(((-1, 0, 0), (0, 1, 0), (0, 0, -1))).T,
    14: np.array(((0, -1, 0), (-1, 0, 0), (0, 0, -1))).T,
    16: np.array(((1, 0, 0), (0, 0, 1), (0, -1, 0))).T,
    18: np.array(((0, 1, 0), (0, 0, 1), (1, 0, 0))).T,
    20: np.array(((1, 0, 0), (0, 0, -1), (0, 1, 0))).T,
    22: np.array(((0, 1, 0), (0, 0, -1), (-1, 0, 0))).T,
    24: np.array(((0, 0, -1), (0, 1, 0), (1, 0, 0))).T,
    25: np.array(((0, 0, 1), (0, 1, 0), (-1, 0, 0))).T,
    26: np.array(((0, -1, 0), (-1, 0, 0), (0, 0, -1))).T,
    27: np.array(((0, 1, 0), (1, 0, 0), (0, 0, -1))).T,
    28: np.array(((0, 0, -1), (1, 0, 0), (0, -1, 0))).T,
    29: np.array(((0, 0, -1), (0, -1, 0), (-1, 0, 0))).T,
    30: np.array(((0, 0, -1), (-1, 0, 0), (0, 1, 0))).T,
    31: np.array(((-1, 0, 0), (0, 0, -1), (0, -1, 0))).T,
    32: np.array(((-1, 0, 0), (0, 0, 1), (0, 1, 0))).T,
    33: np.array(((0, 0, 1), (-1, 0, 0), (0, -1, 0))).T,
    34: np.array(((0, 0, 1), (0, -1, 0), (1, 0, 0))).T,
    35: np.array(((0, 0, 1), (1, 0, 0), (0, 1, 0))).T,
    36: np.array(((0, -1, 0), (0, 0, -1), (1, 0, 0))).T,
    37: np.array(((0, -1, 0), (0, 0, 1), (-1, 0, 0))).T,
}
