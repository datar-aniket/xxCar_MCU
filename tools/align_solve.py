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


def _skew(v):
    return np.array(((0.0, -v[2], v[1]),
                     (v[2], 0.0, -v[0]),
                     (-v[1], v[0], 0.0)))


def integrate_attitude(gyro, dt):
    """Relative attitude across a segment, starting at identity.

    Body-to-earth, where "earth" is the frame the vehicle occupied at the
    first sample. A sweep is seconds long, so gyro drift over the window is
    negligible and no bias handling is needed.
    """
    gyro = np.asarray(gyro, dtype=float)
    out = np.empty((len(gyro), 3, 3))
    c = np.eye(3)

    for i, w in enumerate(gyro):
        out[i] = c
        norm = float(np.linalg.norm(w))
        angle = norm * dt

        if angle > 1e-12:
            axis = w / norm
            k = _skew(axis)
            # Rodrigues, exact rather than first-order: a hand sweep reaches
            # several rad/s and the small-angle form would accumulate a
            # visible bias over hundreds of samples.
            step = (np.eye(3) + np.sin(angle) * k
                    + (1.0 - np.cos(angle)) * (k @ k))
            c = c @ step

    return out


def _excitation(attitude):
    """How much each body axis was rotated ABOUT across the segment.

    Measured from the spread of where each body axis points over the sweep. An
    axis that never moved contributes nothing and leaves the solve
    rank-deficient - which returns a confident wrong answer rather than an
    error, so it has to be caught here.
    """
    spread = []

    for axis in range(3):
        pointed = np.array([c[:, axis] for c in attitude])
        spread.append(float(np.linalg.norm(pointed.std(axis=0))))

    return np.array(spread)


_MIN_EXCITATION = 0.15
_MAX_MAG_RESIDUAL = 0.08   # fraction of field magnitude


def _require_excitation(attitude):
    weak = _excitation(attitude) < _MIN_EXCITATION

    if weak.any():
        names = ", ".join("xyz"[i] for i in np.nonzero(weak)[0])
        raise AlignError(
            f"the sweep did not rotate about the {names} axis - the solve "
            "would be rank-deficient; rotate about all three and repeat")


def _kabsch(wanted, measured):
    """Proper rotation minimising ||wanted - R @ measured||, row-stacked.

    Always forces det=+1. Letting this return a reflection would be pointless
    here: see mag_rotation, where -R fits the data exactly as well as R, so a
    reflection is never distinguishable by residual alone.
    """
    u, _s, vt = np.linalg.svd(wanted.T @ measured)
    d = np.diag((1.0, 1.0, float(np.sign(np.linalg.det(u @ vt)))))
    return u @ d @ vt


def _mag_fit(mag, attitude, iterations):
    """Alternating least squares for the rotation that stills the field.

    Given R the best earth field is the mean; given the field the best R is a
    Kabsch fit. Seeded at identity.
    """
    r = np.eye(3)

    for _ in range(iterations):
        earth = np.einsum("nij,jk,nk->ni", attitude, r, mag)
        target = earth.mean(axis=0)
        wanted = np.einsum("nji,j->ni", attitude, target)
        r = _kabsch(wanted, mag)

    earth = np.einsum("nij,jk,nk->ni", attitude, r, mag)
    target = earth.mean(axis=0)
    residual = float(np.linalg.norm(earth - target, axis=1).mean())
    return r, target, residual


def earth_up(accel, attitude):
    """The UP direction in the sweep's earth frame, from the accelerometer.

    integrate_attitude starts at identity, so its "earth" frame is whatever
    attitude the vehicle happened to be in at the first sample - arbitrary, and
    emphatically not gravity-aligned. Recovering which way is up is what lets
    the magnetometer's dip be checked, and dip is the only thing that resolves
    the sign ambiguity below.

    Specific force points up at rest. During a sweep there are dynamic
    accelerations too, but the vehicle is rotated roughly in place rather than
    carried anywhere, so those average out and gravity is what is left.

    accel must already be in the VEHICLE frame - rotate it by the Phase A
    result before calling.
    """
    accel = np.asarray(accel, dtype=float)
    attitude = np.asarray(attitude, dtype=float)

    if len(accel) != len(attitude):
        raise AlignError("accelerometer and attitude series do not match")

    mean = np.einsum("nij,nj->ni", attitude, accel).mean(axis=0)
    norm = float(np.linalg.norm(mean))

    if norm < 0.5 * GRAVITY:
        raise AlignError(
            f"cannot find up from the accelerometer (mean {norm:.2f} m/s^2) - "
            "the sweep translated the vehicle instead of rotating it in place")

    return mean / norm


# The dip must be at least this fraction of the field before its SIGN can be
# trusted. Near the magnetic equator the field is almost horizontal and the
# handedness test genuinely has nothing to work with; 0.15 is about 9 degrees
# of dip.
_MIN_DIP_FRACTION = 0.15


def mag_rotation(mag, attitude, up, dip_down=True, iterations=60):
    """Rotation sensor -> vehicle, by making the earth field stop moving.

    The field is constant in the earth frame, so the correct rotation is the
    one that minimises the variance of C(t) @ R @ m(t). This never needs to
    know what the field actually is, and it is heading-free - it does not care
    how the operator turned the vehicle, which is exactly why the static
    positions cannot be used for the magnetometer.

    THAT CRITERION CANNOT SEE A MIRRORED SENSOR. -I commutes with every
    rotation, so C(t)(-I)C(t)^T m_e = -m_e is just as constant as +m_e: R and
    -R fit identically, and in 3D exactly one of that pair is a reflection. No
    amount of comparing residuals distinguishes them.

    What does distinguish them is which way the recovered field POINTS. The
    earth's field dips downward in the northern hemisphere, so with `up` known
    from the accelerometer the sign of field . up decides it. If the proper fit
    disagrees with the expected dip, the sensor is mirrored and the true
    mapping is -R.
    """
    mag = np.asarray(mag, dtype=float)
    attitude = np.asarray(attitude, dtype=float)
    up = np.asarray(up, dtype=float)

    if len(mag) != len(attitude) or len(mag) < 50:
        raise AlignError("magnetometer and attitude series do not match, or "
                         "the sweep is too short")

    _require_excitation(attitude)

    r, target, residual = _mag_fit(mag, attitude, iterations)
    field = float(np.linalg.norm(target))

    if field < 1e-6:
        raise AlignError("recovered field is zero - is the magnetometer "
                         "producing?")

    if residual / field > _MAX_MAG_RESIDUAL:
        raise AlignError(
            f"the field does not hold still after alignment (residual "
            f"{residual / field:.1%} of {field:.3f} G) - interference, a bad "
            "calibration, or the sweep was too fast for the sample rate")

    dip = float(np.dot(target, up))

    if abs(dip) / field < _MIN_DIP_FRACTION:
        raise AlignError(
            f"magnetic dip is only {np.degrees(np.arcsin(dip / field)):.1f} "
            "degrees - too shallow to tell a mirrored sensor from a rotated "
            "one; this test does not work near the magnetic equator")

    mirrored = (dip < 0.0) != bool(dip_down)

    if mirrored:
        # The true mapping is -R, which is the reflection the criterion could
        # not rule out. Report it; do not store it.
        return {"matrix": -r, "enum": None, "snap_deg": None,
                "mirrored": True, "field": field, "residual": residual,
                "dip_deg": float(np.degrees(np.arcsin(dip / field)))}

    value, angle = snap(r)

    if angle > SNAP_LIMIT_DEG:
        raise AlignError(
            f"magnetometer is {angle:.1f} degrees from the nearest "
            f"representable rotation (limit {SNAP_LIMIT_DEG:.0f})")

    return {"matrix": r, "enum": value, "snap_deg": angle,
            "mirrored": False, "field": field, "residual": residual,
            "dip_deg": float(np.degrees(np.arcsin(dip / field)))}
