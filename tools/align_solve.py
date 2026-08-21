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


# Which vehicle axis points UP in each position, and with what sign. The axis
# pointing up reads +g, so "nose_up" means the vehicle-frame reading is
# (+g, 0, 0) and "nose_down" is (-g, 0, 0).
POSITIONS = {
    "nose_up": (0, +1),
    "nose_down": (0, -1),
    "left_down": (1, -1),
    "right_down": (1, +1),
    "level": (2, +1),
    "inverted": (2, -1),
}

_PAIRS = (
    (0, "nose_up", "nose_down"),
    (1, "right_down", "left_down"),
    (2, "level", "inverted"),
)

# The dominant component must beat the next largest by this factor.
#
# This is an AXIS IDENTIFICATION threshold, not a quality one. Its only job is
# to decide which sensor axis a position corresponds to, so it needs a clear
# winner and nothing more; judging whether the mounting is square is the snap
# limit's job, and the snap limit says so in a message that blames the
# mounting rather than the operator.
#
# 1.3 is a ratio of about 37 degrees. A truly ambiguous tip is 45 degrees,
# where the ratio is 1.0 and the axis assignment is a coin flip, so this keeps
# a margin over that. Set any higher and a genuinely off-axis mounting gets
# reported as a badly held position: a 30-degree mounting has a ratio of 1.73,
# which a threshold of 2.0 would reject with entirely the wrong explanation.
_DOMINANCE = 1.3

# How far the three measured rows may be from perpendicular. Generous, because
# the answer only has to pick among 24 discrete rotations - but not unbounded,
# because a badly held position skews a row without shortening it.
_MAX_GRAM_OFF = 0.25


def _check_position(name, reading):
    reading = np.asarray(reading, dtype=float)

    if reading.shape != (3,) or not np.all(np.isfinite(reading)):
        raise AlignError(f"{name}: reading is not three finite numbers")

    order = np.argsort(np.abs(reading))[::-1]
    largest = abs(reading[order[0]])
    second = abs(reading[order[1]])

    if largest < 0.5 * GRAVITY:
        raise AlignError(
            f"{name}: no axis reads close to 1 g "
            f"(largest {largest:.2f} m/s^2) - is the sensor producing?")

    if largest < _DOMINANCE * second:
        raise AlignError(
            f"{name}: tipped between two axes "
            f"({largest:.2f} vs {second:.2f} m/s^2) - hold it closer to "
            "square and repeat")

    return reading, int(order[0])


def accel_rotation(positions):
    """Rotation sensor -> vehicle from the six static positions.

    Each opposite pair gives one ROW of R: with R mapping sensor to vehicle, a
    vehicle reading of +g on axis k means the sensor reads g * (row k of R), so
    (up - down) / 2g is that row directly, with bias cancelled.
    """
    missing = [name for name in POSITIONS if name not in positions]

    if missing:
        raise AlignError("missing position(s): " + ", ".join(sorted(missing)))

    checked = {}
    dominant = {}

    for name in POSITIONS:
        checked[name], dominant[name] = _check_position(name,
                                                        positions[name])

    # Each pair must resolve to a different sensor axis. Two pairs landing on
    # the same axis means a position was repeated or skipped.
    seen = {}

    for _axis, up, _down in _PAIRS:
        sensor_axis = dominant[up]

        if sensor_axis in seen:
            raise AlignError(
                f"{up} and {seen[sensor_axis]} resolve to the same axis - a "
                "position was repeated or skipped")

        seen[sensor_axis] = up

    rows = []
    residual = 0.0

    for _axis, up, down in _PAIRS:
        row = (checked[up] - checked[down]) / (2.0 * GRAVITY)
        residual = max(residual, abs(float(np.linalg.norm(row)) - 1.0))
        rows.append(row)

    raw = np.array(rows)

    # The three rows came from three independent physical positions, so they
    # are only orthogonal if the readings are consistent. Orthonormalising
    # first would HIDE that - polar decomposition happily returns the nearest
    # orthogonal matrix to a badly skewed one. Check before repairing.
    gram = raw @ raw.T
    off = np.abs(gram - np.diag(np.diag(gram)))
    worst = float(off.max())

    if worst > _MAX_GRAM_OFF:
        i, j = np.unravel_index(off.argmax(), off.shape)
        pair = ("x", "y", "z")
        raise AlignError(
            f"axes {pair[i]} and {pair[j]} are not perpendicular "
            f"(dot {worst:.3f}, limit {_MAX_GRAM_OFF:.2f}) - a position was "
            "held badly, or the sensor is not rigid in the vehicle")

    matrix = orthonormalise(raw)

    if is_mirrored(matrix):
        return {"matrix": matrix, "enum": None, "snap_deg": None,
                "mirrored": True, "residual_g": residual}

    value, angle = snap(matrix)

    if angle > SNAP_LIMIT_DEG:
        raise AlignError(
            f"mounting is {angle:.1f} degrees from the nearest representable "
            f"rotation (limit {SNAP_LIMIT_DEG:.0f}) - this is not an axis "
            "permutation and cannot be stored")

    return {"matrix": matrix, "enum": value, "snap_deg": angle,
            "mirrored": False, "residual_g": residual}
