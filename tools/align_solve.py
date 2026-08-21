#!/usr/bin/env python3
"""Solve each sensor's orientation relative to the vehicle.

Gravity is the reference: at rest the accelerometer measures specific force,
which points UP, so the vehicle axis pointing up reads +g. Tipping the vehicle
into six positions therefore reads the rotation matrix out directly, at
9.81 m/s^2, with no dynamics and no filtering.

The answer is one of only 24 distinct representable rotations, which is what
makes the procedure robust: a tip need only be within about 45 degrees to land
on the right one, so sloppy positioning produces a refusal rather than a wrong
answer.

Nothing here talks to a serial port or to Tk. That is what makes it testable.
"""

import numpy as np

from rotation_table import ROTATIONS

GRAVITY = 9.80665

# Beyond this the mounting is not an axis permutation, rotation.h cannot
# represent it, and approximating would silently read the vehicle sideways.
SNAP_LIMIT_DEG = 15.0


class AlignError(ValueError):
    """A refusal. Every rejection path raises this, never returns a value."""


def orthonormalise(m):
    """Nearest orthogonal matrix, HANDEDNESS PRESERVED.

    The usual polar decomposition is often written to force det=+1. That would
    repair a reflection into a rotation and destroy the evidence the flip check
    depends on, so it is deliberately not done here.
    """
    u, _s, vt = np.linalg.svd(np.asarray(m, dtype=float))
    return u @ vt


def is_mirrored(m):
    return np.linalg.det(np.asarray(m, dtype=float)) < 0.0


def _angle_between(a, b):
    """Rotation angle carrying a onto b, in degrees."""
    trace = np.trace(np.asarray(a).T @ np.asarray(b))
    return float(np.degrees(np.arccos(np.clip((trace - 1.0) / 2.0,
                                              -1.0, 1.0))))


def snap(m):
    """Nearest representable rotation and the angle to it, in degrees.

    Ties break to the LOWEST enum value. That is not cosmetic: the PX4 enum
    reaches two rotations by two Euler routes each - ROLL_180_YAW_90 (10) and
    PITCH_180_YAW_270 (27) are the same matrix, as are ROLL_180_YAW_270 (14)
    and PITCH_180_YAW_90 (26). Without a deterministic tie-break the same
    physical mounting would be reported under either name depending on
    floating-point noise.

    Raises AlignError for a reflection: a mirrored sensor is either a known
    part property, which belongs in code, or a wiring fault. Both need a human,
    and neither should be absorbed into a rotation value.
    """
    m = orthonormalise(m)

    if is_mirrored(m):
        raise AlignError(
            "sensor axes are mirrored, not rotated - no rotation value can "
            "express this; check the part's frame and the driver")

    value = min(ROTATIONS, key=lambda k: (round(_angle_between(ROTATIONS[k],
                                                               m), 6), k))
    return value, _angle_between(ROTATIONS[value], m)
