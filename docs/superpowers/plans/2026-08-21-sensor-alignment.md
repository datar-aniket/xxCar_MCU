# Guided Sensor Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Measure each sensor's orientation relative to the vehicle using gravity as a 9.81 m/s² reference and a guided procedure that refuses bad data, producing `IMU0→Body`, `IMU1→Body`, `MAG→Body` and `FLOW→Body`.

**Architecture:** The board streams and reports stillness; every solve happens on the host in Python, following the precedent set when the magnetometer ellipsoid fit moved off the board. Six static positions read each accelerometer rotation directly from gravity; one rotation sweep solves the magnetometer by making the earth field stop moving, and optical flow by rate correlation against the now-known IMU.

**Tech Stack:** Python 3 + NumPy (host solver and tests), Tkinter (GUI), C11/NuttX (board streaming only).

**Source spec:** `docs/superpowers/specs/2026-08-21-sensor-alignment-design.md`

## Global Constraints

- **Ordering: Phase 1 (Tasks 1–5) is host-only and needs no hardware.** The solver and its tests run on synthetic data with known answers. Only Tasks 6–8 require a flash.
- **The vehicle convention is +x FORWARD, +y LEFT, +z UP.** At rest, the vehicle axis pointing UP reads **+g**. Nose down gives a vehicle-frame reading of `(−g, 0, 0)`.
- **`R` maps sensor → vehicle**, i.e. `v_vehicle = R @ v_sensor`.
- **Only the 24 axis-permutation rotations are representable.** `rotation.h` refuses the 45° entries. A solved rotation more than **15°** from its nearest representable value is refused, not snapped.
- **No fine rotation is written.** `CAL_*_RV*` stays zero and unused by this procedure.
- **No lever arms.** Translation is out of scope entirely; `SENS_*_POS_*` stays hand-entered.
- **The magnetometer solver consumes raw `sensor_mag0`, never `vehicle_mag`.** `mag_frame.c` applies the handedness flip before publishing, so solving against `vehicle_mag` would be solving against the answer.
- **Python style:** match `tools/mag_cal.py` — module-level functions, NumPy, a `FitError`-style exception for refusals, no classes unless state demands one.
- **C style:** NuttX kernel style as used throughout `apps/` — two-space indent, braces on their own line, `FAR` on pointer parameters, `/* */` comments only.
- **Nothing is written to parameters without an explicit commit**, and a valid stored transform is never overwritten by an uncertain fit.
- **Commit after every task**, with the trailer `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`.
- **`tools/verify.sh` is the gate.** `test-cpu-runtime` is a known pre-existing failure, unrelated to this work, and must not be "fixed" here.

## File Structure

**Created:**

| File | Responsibility |
|---|---|
| `tools/align_solve.py` | Pure solver. Positions and sweep samples in; rotations plus diagnostics out. No serial, no Tk. |
| `tools/rotation_table.py` | The 24 representable rotations as matrices, **generated from `rotation.c`**, not transcribed. |
| `tools/gen_rotation_table.py` | Generator: compiles `rotation.c`, applies every enum to the basis vectors, writes the table. |
| `tests/align_solve_host_test.py` | Synthetic data with known answers; one test per refusal path. |
| `tools/test-align-solve.sh` | Runner, following `tools/test-mag-cal-host.sh`. |

**Modified:**

| File | Change |
|---|---|
| `apps/cal/cal.c` | Concurrent multi-sensor streaming; `still <sensor>` capture command. |
| `tools/cal_gui.py` | Alignment tab. |

Keeping `align_solve.py` free of Tk and pyserial is what makes it testable at all; the GUI is a shell around it.

---

### Task 1: The rotation table, generated rather than transcribed

`rotation.h` warns that every rotation value is a plausible orientation, so a transcription error is invisible — it does not fail, it reads the vehicle sideways. So the Python table is **generated from the C implementation** and a test proves it still matches.

**Files:**
- Create: `tools/gen_rotation_table.py`, `tools/rotation_table.py`
- Create: `tests/align_solve_host_test.py`, `tools/test-align-solve.sh`

**Interfaces:**
- Consumes: `apps/sensors/rotation.c`, `apps/sensors/rotation.h`
- Produces: `rotation_table.ROTATIONS` — `dict[int, numpy.ndarray]` mapping enum value to a 3×3 matrix `M` such that `M @ v` equals what `rotation_apply(value, v)` produces. Only supported (non-45°) values appear.

- [ ] **Step 1: Write the generator**

Create `tools/gen_rotation_table.py`:

```python
#!/usr/bin/env python3
"""Generate tools/rotation_table.py from apps/sensors/rotation.c.

rotation.h is explicit that a wrong rotation does not fail, it silently reads
the vehicle sideways. Transcribing 24 sign-and-swap matrices by hand into
Python is exactly the kind of thing that goes wrong once and is never noticed,
so the table is produced by ASKING the C code what it does.
"""

import pathlib
import subprocess
import sys
import tempfile

REPO = pathlib.Path(__file__).resolve().parents[1]

HARNESS = r"""
#include <stdio.h>
#include "rotation.h"

int main(void)
{
  int rot;

  for (rot = 0; rot < ROTATION_MAX_SUPPORTED; rot++)
    {
      float basis[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
      int axis;

      if (!rotation_supported((unsigned char)rot))
        {
          continue;
        }

      printf("%d", rot);

      for (axis = 0; axis < 3; axis++)
        {
          rotation_apply((unsigned char)rot, basis[axis]);
          printf(" %.0f %.0f %.0f",
                 (double)basis[axis][0], (double)basis[axis][1],
                 (double)basis[axis][2]);
        }

      printf("\n");
    }

  return 0;
}
"""


def generate() -> str:
    with tempfile.TemporaryDirectory() as tmp:
        src = pathlib.Path(tmp) / "harness.c"
        exe = pathlib.Path(tmp) / "harness"
        src.write_text(HARNESS)
        subprocess.run(
            ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-DFAR=",
             "-I", str(REPO / "apps" / "sensors"), str(src),
             str(REPO / "apps" / "sensors" / "rotation.c"),
             "-lm", "-o", str(exe)],
            check=True)
        out = subprocess.run([str(exe)], check=True,
                             capture_output=True, text=True).stdout

    lines = ["#!/usr/bin/env python3",
             '"""The representable rotations, GENERATED from '
             'apps/sensors/rotation.c.',
             "",
             "Do not edit. Run tools/gen_rotation_table.py. The mapping is",
             "verified against the C implementation by",
             "tools/test-align-solve.sh, so the two cannot drift apart.",
             '"""',
             "",
             "import numpy as np",
             "",
             "ROTATIONS = {"]

    for line in out.strip().splitlines():
        parts = line.split()
        rot = int(parts[0])
        v = [float(x) for x in parts[1:]]

        # rotation_apply transforms a vector, so applying it to each basis
        # vector yields the COLUMNS of the matrix transposed - basis i maps to
        # column i of M^T, i.e. row i of M. Emit rows directly.
        rows = [v[0:3], v[3:6], v[6:9]]
        body = ", ".join(
            "({:.0f}, {:.0f}, {:.0f})".format(*r) for r in rows)
        lines.append(f"    {rot}: np.array(({body})).T,")

    lines.append("}")
    lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    dest = REPO / "tools" / "rotation_table.py"
    dest.write_text(generate())
    print(f"wrote {dest}", file=sys.stderr)
```

- [ ] **Step 2: Run the generator**

```bash
python3 tools/gen_rotation_table.py && head -20 tools/rotation_table.py
```

Expected: `tools/rotation_table.py` written, containing 24 entries. If the count is not 24, stop — `rotation_supported()` and the 45° exclusions disagree with the spec and that must be understood before continuing.

Verify the count:

```bash
python3 -c "import sys; sys.path.insert(0,'tools'); from rotation_table import ROTATIONS; print(len(ROTATIONS))"
```

Expected: `24`

- [ ] **Step 3: Write the failing test**

Create `tests/align_solve_host_test.py`:

```python
#!/usr/bin/env python3
"""Deterministic checks for the host-side alignment solver."""

import pathlib
import subprocess
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

REPO = pathlib.Path(__file__).resolve().parents[1]

from rotation_table import ROTATIONS


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


def test_table_is_24_proper_rotations():
    assert len(ROTATIONS) == 24

    for value, matrix in ROTATIONS.items():
        assert matrix.shape == (3, 3)
        # Every entry is a signed axis permutation: orthogonal, det +1.
        assert np.allclose(matrix @ matrix.T, np.eye(3)), value
        assert np.isclose(np.linalg.det(matrix), 1.0), value
        # And exactly one non-zero per row and column.
        assert np.all(np.count_nonzero(matrix, axis=0) == 1), value
        assert np.all(np.count_nonzero(matrix, axis=1) == 1), value


def test_identity_is_rotation_none():
    assert np.allclose(ROTATIONS[0], np.eye(3))


def main():
    test_table_matches_c()
    test_table_is_24_proper_rotations()
    test_identity_is_rotation_none()
    print("align_solve: rotation table verified against rotation.c - OK")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Write the runner**

Create `tools/test-align-solve.sh`:

```bash
#!/usr/bin/env bash
# Host-side alignment solver: rotation table, gravity columns, magnetometer
# variance solve, flow correlation, and every refusal path.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$REPO/tests/align_solve_host_test.py"
```

Then `chmod +x tools/test-align-solve.sh`.

- [ ] **Step 5: Run it**

```bash
tools/test-align-solve.sh
```

Expected: `align_solve: rotation table verified against rotation.c - OK`

If `test_table_matches_c` fails on a freshly generated file, the generator is not deterministic — fix that before continuing, because the test's whole value is that it can be trusted to fire.

- [ ] **Step 6: Commit**

```bash
git add tools/gen_rotation_table.py tools/rotation_table.py \
        tests/align_solve_host_test.py tools/test-align-solve.sh
git commit -m "tools: generate the rotation table from rotation.c

rotation.h is explicit that a wrong rotation does not fail, it silently
reads the vehicle sideways. Transcribing 24 sign-and-swap matrices into
Python by hand is exactly the kind of thing that goes wrong once and is
never noticed, so the table is produced by asking the C code what it does,
and a test regenerates it to prove the two have not drifted apart.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Snapping and flip detection

**Files:**
- Create: `tools/align_solve.py`
- Modify: `tests/align_solve_host_test.py`

**Interfaces:**
- Consumes: `rotation_table.ROTATIONS`
- Produces:
  - `class AlignError(ValueError)` — every refusal raises this.
  - `orthonormalise(m) -> np.ndarray` — nearest orthogonal matrix, reflection preserved.
  - `snap(m) -> tuple[int, float]` — nearest representable enum value and the angle to it in degrees. Raises `AlignError` if `det(m) < 0`.
  - `is_mirrored(m) -> bool` — `det(m) < 0`.
  - `SNAP_LIMIT_DEG = 15.0`

- [ ] **Step 1: Write the failing test**

Add to `tests/align_solve_host_test.py`, above `main()`:

```python
from align_solve import (AlignError, SNAP_LIMIT_DEG, is_mirrored,
                         orthonormalise, snap)


def _rot_z(deg):
    a = np.radians(deg)
    return np.array(((np.cos(a), -np.sin(a), 0.0),
                     (np.sin(a), np.cos(a), 0.0),
                     (0.0, 0.0, 1.0)))


def test_snap_finds_the_exact_value():
    for value, matrix in ROTATIONS.items():
        got, angle = snap(matrix)
        assert got == value
        assert angle < 1e-6


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
```

Register them in `main()`:

```python
    test_snap_finds_the_exact_value()
    test_snap_reports_the_angle_off()
    test_snap_refuses_a_reflection()
    test_orthonormalise_preserves_handedness()
```

- [ ] **Step 2: Run to verify it fails**

```bash
tools/test-align-solve.sh
```

Expected: FAIL — `ModuleNotFoundError: No module named 'align_solve'`.

- [ ] **Step 3: Implement**

Create `tools/align_solve.py`:

```python
#!/usr/bin/env python3
"""Solve each sensor's orientation relative to the vehicle.

Gravity is the reference: at rest the accelerometer measures specific force,
which points UP, so the vehicle axis pointing up reads +g. Tipping the vehicle
into six positions therefore reads the rotation matrix out directly, at
9.81 m/s^2, with no dynamics and no filtering.

The answer is one of only 24 representable rotations, which is what makes the
procedure robust: a tip need only be within about 45 degrees to land on the
right one, so sloppy positioning produces a refusal rather than a wrong
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
    trace = np.trace(a.T @ b)
    return float(np.degrees(np.arccos(np.clip((trace - 1.0) / 2.0,
                                              -1.0, 1.0))))


def snap(m):
    """Nearest representable rotation and the angle to it, in degrees.

    Raises AlignError for a reflection: a mirrored sensor is either a known
    part property, which belongs in code, or a wiring fault. Both need a human,
    and neither should be absorbed into a rotation value.
    """
    m = orthonormalise(m)

    if is_mirrored(m):
        raise AlignError(
            "sensor axes are mirrored, not rotated - no rotation value can "
            "express this; check the part's frame and the driver")

    best = min(ROTATIONS.items(), key=lambda kv: _angle_between(kv[1], m))
    return best[0], _angle_between(best[1], m)
```

- [ ] **Step 4: Run to verify it passes**

```bash
tools/test-align-solve.sh
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/align_solve.py tests/align_solve_host_test.py
git commit -m "tools: snap a solved rotation, and refuse a mirrored one

orthonormalise() deliberately preserves handedness. The usual polar
decomposition is written to force det=+1, which would repair a reflection
into a rotation and destroy the evidence the flip check depends on - and
the IST8310 proves reflections are real here, not hypothetical.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Accelerometer rotation from six positions

**Files:**
- Modify: `tools/align_solve.py`, `tests/align_solve_host_test.py`

**Interfaces:**
- Consumes: `orthonormalise`, `snap`, `AlignError`, `GRAVITY` from Task 2.
- Produces:
  - `POSITIONS` — `dict[str, tuple[int, int]]` mapping position name to `(axis_index, sign)`, e.g. `"nose_up": (0, +1)`.
  - `accel_rotation(positions) -> dict` where `positions` is `dict[str, sequence[float]]` of averaged sensor-frame readings. Returns `{"matrix", "enum", "snap_deg", "mirrored", "residual_g"}`. Raises `AlignError` on any refusal.

- [ ] **Step 1: Write the failing test**

Add to `tests/align_solve_host_test.py`:

```python
from align_solve import GRAVITY, POSITIONS, accel_rotation


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
        sensor = rotation.T @ vehicle + np.asarray(bias, dtype=float)

        if noise:
            sensor = sensor + rng.normal(0.0, noise, 3)

        out[name] = sensor

    return out


def test_accel_recovers_every_representable_rotation():
    for value, matrix in ROTATIONS.items():
        got = accel_rotation(_positions_for(matrix))
        assert got["enum"] == value, (value, got["enum"])
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
    """
    off = _rot_z(30.0)
    try:
        accel_rotation(_positions_for(off))
    except AlignError as exc:
        assert "15" in str(exc) or "30" in str(exc)
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
    on the columns it did get.
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
```

Register all nine in `main()`.

- [ ] **Step 2: Run to verify it fails**

```bash
tools/test-align-solve.sh
```

Expected: FAIL — `ImportError: cannot import name 'POSITIONS'`.

- [ ] **Step 3: Implement**

Append to `tools/align_solve.py`:

```python
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

# The dominant component must beat the next largest by this factor, or the tip
# was ambiguous between two axes.
_DOMINANCE = 2.0

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

    for axis, up, down in _PAIRS:
        sensor_axis = dominant[up]

        if sensor_axis in seen:
            raise AlignError(
                f"{up} and {seen[sensor_axis]} resolve to the same axis - a "
                "position was repeated or skipped")

        seen[sensor_axis] = up

    rows = []
    residual = 0.0

    for axis, up, down in _PAIRS:
        row = (checked[up] - checked[down]) / (2.0 * GRAVITY)
        residual = max(residual, abs(np.linalg.norm(row) - 1.0))
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
```

- [ ] **Step 4: Run to verify it passes**

```bash
tools/test-align-solve.sh
```

Expected: PASS. If `test_accel_recovers_every_representable_rotation` fails for some values only, the row-versus-column convention is inverted — re-read the docstring: `(up − down)/2g` is a **row** of R, not a column.

- [ ] **Step 5: Commit**

```bash
git add tools/align_solve.py tests/align_solve_host_test.py
git commit -m "tools: read the accelerometer rotation straight out of gravity

Each opposite pair gives one row of R, with bias cancelled by the
averaging - which is what lets alignment run on an uncalibrated sensor,
and equally why it cannot confirm a calibration.

Refuses an ambiguous tip, a missing position, and two positions resolving
to the same axis. That last one is an operator repeating a position, and
without the check the solve is rank-2 while still passing orthogonality on
the rows it did get.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Magnetometer rotation from the sweep

**Files:**
- Modify: `tools/align_solve.py`, `tests/align_solve_host_test.py`

**Interfaces:**
- Consumes: everything from Tasks 2–3.
- Produces:
  - `integrate_attitude(gyro, dt) -> np.ndarray` — `(N, 3, 3)` relative attitudes, first is identity.
  - `mag_rotation(mag, attitude) -> dict` — same keys as `accel_rotation`, plus `"field"` (the recovered earth-field magnitude) and `"residual"`.

- [ ] **Step 1: Write the failing test**

Add to `tests/align_solve_host_test.py`:

```python
from align_solve import integrate_attitude, mag_rotation


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
    """What a magnetometer at `rotation` reads through that sweep."""
    rng = np.random.default_rng(seed)
    attitude = integrate_attitude(gyro, dt)
    dip = np.radians(dip_deg)
    # Northern hemisphere in a z-UP frame: the field dips DOWN, so -z.
    earth = np.array((field * np.cos(dip), 0.0, -field * np.sin(dip)))

    out = []

    for c in attitude:
        body = c.T @ earth
        sensor = rotation.T @ body

        if noise:
            sensor = sensor + rng.normal(0.0, noise, 3)

        out.append(sensor)

    return np.array(out), attitude


def test_mag_recovers_every_representable_rotation():
    gyro, dt = _sweep()

    for value, matrix in ROTATIONS.items():
        mag, attitude = _mag_for(matrix, gyro, dt)
        got = mag_rotation(mag, attitude)
        assert got["enum"] == value, (value, got["enum"])
        assert got["snap_deg"] < 1.0


def test_mag_recovers_the_field_magnitude():
    gyro, dt = _sweep()
    mag, attitude = _mag_for(ROTATIONS[0], gyro, dt, field=0.47)
    got = mag_rotation(mag, attitude)
    assert abs(got["field"] - 0.47) < 0.01


def test_mag_detects_a_mirrored_sensor():
    """The IST8310 case: y negated. The solver must say mirrored rather than
    converge on the nearest 180-degree rotation.
    """
    gyro, dt = _sweep()
    mag, attitude = _mag_for(np.diag((1.0, -1.0, 1.0)), gyro, dt)
    got = mag_rotation(mag, attitude)
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
    mag, attitude = _mag_for(ROTATIONS[0], gyro, dt)

    try:
        mag_rotation(mag, attitude)
    except AlignError as exc:
        assert "axis" in str(exc).lower()
    else:
        assert False, "accepted a single-axis sweep"


def test_mag_refuses_excess_noise():
    gyro, dt = _sweep()
    mag, attitude = _mag_for(ROTATIONS[0], gyro, dt, noise=0.25)

    try:
        mag_rotation(mag, attitude)
    except AlignError as exc:
        assert "residual" in str(exc).lower()
    else:
        assert False, "accepted a field that does not hold still"
```

Register all five in `main()`.

- [ ] **Step 2: Run to verify it fails**

```bash
tools/test-align-solve.sh
```

Expected: FAIL — `ImportError: cannot import name 'integrate_attitude'`.

- [ ] **Step 3: Implement**

Append to `tools/align_solve.py`:

```python
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
        angle = np.linalg.norm(w) * dt

        if angle > 1e-12:
            axis = w / np.linalg.norm(w)
            k = _skew(axis)
            # Rodrigues, exact rather than first-order: a hand sweep reaches
            # several rad/s and the small-angle form would accumulate a
            # visible bias over 600 samples.
            step = (np.eye(3) + np.sin(angle) * k
                    + (1.0 - np.cos(angle)) * (k @ k))
            c = c @ step

    return out


def _excitation(attitude):
    """How much each body axis was rotated ABOUT across the segment.

    Measured from the spread of where each body axis points over the sweep. An
    axis that never moved contributes nothing and leaves the solve
    rank-deficient.
    """
    spread = []

    for axis in range(3):
        pointed = np.array([c[:, axis] for c in attitude])
        spread.append(float(np.linalg.norm(pointed.std(axis=0))))

    return np.array(spread)


_MIN_EXCITATION = 0.15
_MAX_MAG_RESIDUAL = 0.08   # fraction of field magnitude


def mag_rotation(mag, attitude, iterations=40):
    """Rotation sensor -> vehicle, by making the earth field stop moving.

    The field is constant in the earth frame, so the correct rotation is the
    one that minimises the variance of C(t) @ R @ m(t). This never needs to
    know what the field actually is, and it is heading-free - it does not care
    how the operator turned the vehicle, which is exactly why the static
    positions cannot be used for the magnetometer.

    Solved by alternating: given R the best field is the mean, and given the
    field the best R is a Kabsch fit. Seeded at identity.
    """
    mag = np.asarray(mag, dtype=float)
    attitude = np.asarray(attitude, dtype=float)

    if len(mag) != len(attitude) or len(mag) < 50:
        raise AlignError("magnetometer and attitude series do not match, or "
                         "the sweep is too short")

    weak = _excitation(attitude) < _MIN_EXCITATION

    if weak.any():
        names = ", ".join("xyz"[i] for i in np.nonzero(weak)[0])
        raise AlignError(
            f"the sweep did not rotate about the {names} axis - the solve "
            "would be rank-deficient; rotate about all three and repeat")

    r = np.eye(3)

    for _ in range(iterations):
        earth = np.array([c @ r @ m for c, m in zip(attitude, mag)])
        target = earth.mean(axis=0)
        # Kabsch WITHOUT forcing det=+1: a genuine reflection must survive to
        # be reported rather than be repaired into the nearest rotation.
        wanted = np.array([c.T @ target for c in attitude])
        r = orthonormalise(wanted.T @ mag)

    earth = np.array([c @ r @ m for c, m in zip(attitude, mag)])
    target = earth.mean(axis=0)
    field = float(np.linalg.norm(target))
    residual = float(np.linalg.norm(earth - target, axis=1).mean())

    if field < 1e-6:
        raise AlignError("recovered field is zero - is the magnetometer "
                         "producing?")

    if residual / field > _MAX_MAG_RESIDUAL:
        raise AlignError(
            f"the field does not hold still after alignment (residual "
            f"{residual / field:.1%} of {field:.3f} G) - interference, a bad "
            "calibration, or the sweep was too fast for the sample rate")

    if is_mirrored(r):
        return {"matrix": r, "enum": None, "snap_deg": None,
                "mirrored": True, "field": field, "residual": residual}

    value, angle = snap(r)

    if angle > SNAP_LIMIT_DEG:
        raise AlignError(
            f"magnetometer is {angle:.1f} degrees from the nearest "
            f"representable rotation (limit {SNAP_LIMIT_DEG:.0f})")

    return {"matrix": r, "enum": value, "snap_deg": angle,
            "mirrored": False, "field": field, "residual": residual}
```

- [ ] **Step 4: Run to verify it passes**

```bash
tools/test-align-solve.sh
```

Expected: PASS.

If `test_mag_recovers_every_representable_rotation` fails for a handful of values, the alternating solve has converged to a local minimum — raise `iterations`, and if that does not fix it, seed `r` from the Kabsch fit of `mag` against `attitude[i].T @ mag[0]` instead of identity.

- [ ] **Step 5: Commit**

```bash
git add tools/align_solve.py tests/align_solve_host_test.py
git commit -m "tools: solve the magnetometer by making the earth field hold still

The field is constant in the earth frame, so the correct rotation is the
one that minimises the variance of C(t) R m(t). It never needs to know what
the field is, and it is heading-free - which is precisely why the static
positions cannot solve the magnetometer: they would need every position to
share one heading, and heading is degenerate with the nose vertical.

The Kabsch step deliberately does not force det=+1, so a genuine reflection
survives to be reported instead of being repaired into the nearest rotation.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Flow rotation, gyro cross-check, and the top-level solve

**Files:**
- Modify: `tools/align_solve.py`, `tests/align_solve_host_test.py`

**Interfaces:**
- Consumes: everything above.
- Produces:
  - `rate_rotation(reference, target, attitude) -> dict` — Kabsch on two rate streams; same keys as `mag_rotation` minus `"field"`.
  - `solve_alignment(session) -> dict` — orchestration. `session` is `{"positions": {sensor: {position: reading}}, "sweep": {"dt": float, "gyro": {...}, "mag": [...], "flow": [...]}}`. Returns `{sensor: result}` with `"error"` set on refusals rather than raising, so one bad sensor does not hide the others.

- [ ] **Step 1: Write the failing test**

Add to `tests/align_solve_host_test.py`:

```python
from align_solve import rate_rotation, solve_alignment


def test_rate_recovers_every_representable_rotation():
    gyro, dt = _sweep()
    attitude = integrate_attitude(gyro, dt)

    for value, matrix in ROTATIONS.items():
        target = np.array([matrix.T @ w for w in gyro])
        got = rate_rotation(gyro, target, attitude)
        assert got["enum"] == value, (value, got["enum"])


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
    mag_series, _attitude = _mag_for(mag, gyro, dt)

    return {
        "positions": {
            "imu0": _positions_for(imu0),
            "imu1": _positions_for(imu1),
        },
        "sweep": {
            "dt": dt,
            "gyro": {
                "imu0": gyro,
                "imu1": np.array([imu1.T @ w for w in gyro]),
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
```

Register all five in `main()`.

- [ ] **Step 2: Run to verify it fails**

```bash
tools/test-align-solve.sh
```

Expected: FAIL — `ImportError: cannot import name 'rate_rotation'`.

- [ ] **Step 3: Implement**

Append to `tools/align_solve.py`:

```python
def rate_rotation(reference, target, attitude):
    """Rotation target -> vehicle, from two rate streams.

    The reference is a gyro already known to be in the vehicle frame. Used for
    optical flow, whose integrated_*gyro channels work regardless of whether
    it can see a surface, and as a cross-check on each IMU's accelerometer
    result - the two share a package, so they must agree.
    """
    reference = np.asarray(reference, dtype=float)
    target = np.asarray(target, dtype=float)

    if reference.shape != target.shape or len(reference) < 50:
        raise AlignError("rate series do not match, or the sweep is too "
                         "short")

    weak = _excitation(attitude) < _MIN_EXCITATION

    if weak.any():
        names = ", ".join("xyz"[i] for i in np.nonzero(weak)[0])
        raise AlignError(
            f"the sweep did not rotate about the {names} axis - the solve "
            "would be rank-deficient; rotate about all three and repeat")

    r = orthonormalise(reference.T @ target)
    residual = float(
        np.linalg.norm(reference - (r @ target.T).T, axis=1).mean())
    scale = float(np.linalg.norm(reference, axis=1).mean())

    if scale < 1e-9:
        raise AlignError("reference gyro is silent")

    if is_mirrored(r):
        return {"matrix": r, "enum": None, "snap_deg": None,
                "mirrored": True, "residual": residual}

    value, angle = snap(r)

    if angle > SNAP_LIMIT_DEG:
        raise AlignError(
            f"{angle:.1f} degrees from the nearest representable rotation "
            f"(limit {SNAP_LIMIT_DEG:.0f})")

    return {"matrix": r, "enum": value, "snap_deg": angle,
            "mirrored": False, "residual": residual}


def _attempt(fn, *args, **kwargs):
    """Run a solver, turning a refusal into a reported result.

    One sensor refusing must not hide the others: the operator needs to see
    everything that WAS solved alongside what was not, or a session that is
    mostly fine looks like a total failure.
    """
    try:
        out = dict(fn(*args, **kwargs))
        out["error"] = None
        return out
    except AlignError as exc:
        return {"matrix": None, "enum": None, "snap_deg": None,
                "mirrored": False, "error": str(exc)}


def solve_alignment(session):
    """Solve every sensor in one captured session."""
    positions = session["positions"]
    sweep = session["sweep"]
    dt = sweep["dt"]
    results = {}

    for name in ("imu0", "imu1"):
        if name in positions:
            results[name] = _attempt(accel_rotation, positions[name])

    reference = np.asarray(sweep["gyro"]["imu0"], dtype=float)
    attitude = integrate_attitude(reference, dt)

    if "mag" in sweep:
        results["mag"] = _attempt(mag_rotation, sweep["mag"], attitude)

    if "flow" in sweep["gyro"]:
        results["flow"] = _attempt(rate_rotation, reference,
                                   sweep["gyro"]["flow"], attitude)

    # Cross-check each IMU's accelerometer answer against its own gyro. They
    # share a package, so the rotation is the same one and they must snap to
    # the same value.
    for name in ("imu0", "imu1"):
        if name not in results or name not in sweep["gyro"]:
            continue

        check = _attempt(rate_rotation, reference, sweep["gyro"][name],
                         attitude)

        if check["error"] is not None or results[name]["enum"] is None:
            results[name]["cross_check"] = None
        else:
            results[name]["cross_check"] = (
                check["enum"] == results[name]["enum"])

    return results
```

- [ ] **Step 4: Run to verify it passes**

```bash
tools/test-align-solve.sh
```

Expected: PASS — all suites.

- [ ] **Step 5: Commit**

```bash
git add tools/align_solve.py tests/align_solve_host_test.py
git commit -m "tools: solve flow by rate correlation, and cross-check the gyros

Accelerometer and gyroscope share a package in both the ICM-42688-P and the
BMI055, so the gyro rotation IS the accel rotation. That makes rate
correlation a cross-check rather than a second source of truth, which is
where it belongs - it is the noisier of the two.

A refusal is reported per sensor rather than raised, so one bad sensor does
not hide the others. A session that is mostly fine should not look like a
total failure.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Concurrent multi-sensor streaming

Phase B needs the magnetometer, gyro and flow **simultaneously** in order to correlate them. `cal.c` currently streams one sensor at a time by construction — see the comment at the streaming state declaration. The frame header already carries a sensor id and `tools/cal_gui.py`'s `_emit()` already unpacks it, so the wire format does not change.

**Files:**
- Modify: `apps/cal/cal.c`

**Interfaces:**
- Produces: command `sel <name> [<name> ...]` starts concurrent streaming of up to `CAL_STREAM_MAX` sensors; `stop` ends all. Frames are unchanged, each tagged with its own `sid`.

- [ ] **Step 1: Read the current streaming state**

```bash
sed -n '660,700p' apps/cal/cal.c
grep -n "st_idx\|st_sub\|st_dt_us\|st_seq\|nbatch\|batch_t0\|batch_since" apps/cal/cal.c
```

Every one of those single-valued variables becomes an array element. Note each site before changing any of them — they are referenced from the command parser and from the sampling loop, and a missed one compiles cleanly while streaming the wrong sensor.

- [ ] **Step 2: Introduce the per-stream struct**

Replace the streaming-state block (the `int st_idx` … `uint8_t st_seq` declarations and the batch accumulator that follows) with:

```c
/* Streaming state, one entry per concurrent sensor.
 *
 * This used to be a single sensor: "the GUI plots one at a time, and a single
 * subscription keeps the frame free of any which-sensor-is-this ambiguity
 * beyond the id byte". Alignment broke that assumption - correlating a
 * magnetometer against a gyro requires both at once, sharing a timebase.
 *
 * The frame format is unchanged. It always carried the id byte; nothing was
 * reading it as anything but a formality.
 */

#define CAL_STREAM_MAX 4

struct cal_stream_s
{
  int      idx;                    /* index into g_sensors, -1 = unused */
  int      sub;
  uint32_t dt_us;                  /* nominal spacing, for the frame header */
  uint8_t  seq;
  uint8_t  nbatch;
  uint32_t batch_t0;
  uint64_t batch_since;
  float    acc_f[CAL_BATCH_MAX * CAL_MAX_VALUES];
};

static struct cal_stream_s g_stream[CAL_STREAM_MAX];
```

`g_stream` is a file-scope static rather than a local: `CAL_BATCH_MAX * CAL_MAX_VALUES` floats times four entries is several kilobytes, and the `cal` task's stack cannot hold it.

- [ ] **Step 3: Initialise and tear down**

Where the old `st_idx = -1` initialisation was, add:

```c
  for (i = 0; i < CAL_STREAM_MAX; i++)
    {
      g_stream[i].idx = -1;
      g_stream[i].sub = -1;
    }
```

And wherever the old `stop` handling unsubscribed `st_sub`, loop over all entries, unsubscribing every non-negative `sub` and setting both fields back to `-1`.

- [ ] **Step 4: Parse the multi-sensor selection**

Extend the command parser so `sel` accepts several names. For each name, look it up as the existing single-sensor path does, subscribe, set the interval, and fill the next free `g_stream` slot. On more than `CAL_STREAM_MAX` names:

```c
              cal_emit(fd, "{\"evt\":\"error\",\"msg\":\"at most %d "
                           "sensors at once\"}\n", CAL_STREAM_MAX);
```

On an unknown name, emit the existing unknown-sensor error and start **none** of them — a partially started stream is worse than a refused one, because the host would silently solve against a missing sensor.

- [ ] **Step 5: Sample every active stream**

The sampling loop reads one subscription. Lift its whole body into a helper taking one `struct cal_stream_s *`, then call that helper for each active entry. Mechanical, but every `st_*` reference inside must become `s->*` — one missed reference compiles cleanly and streams the wrong sensor's data under another sensor's id.

```c
/* One stream's turn: drain what is available, batch it, emit when the batch
 * is full or the window has elapsed. Body lifted verbatim from the previous
 * single-sensor loop; every st_* became s->*.
 */

static void cal_stream_service(int fd, FAR struct cal_stream_s *s,
                               FAR uint8_t *frame)
{
  if (s->idx < 0 || s->sub < 0)
    {
      return;
    }

  /* ... previous loop body, with st_idx -> s->idx, st_sub -> s->sub,
   * st_dt_us -> s->dt_us, st_seq -> s->seq, nbatch -> s->nbatch,
   * batch_t0 -> s->batch_t0, batch_since -> s->batch_since,
   * acc_f -> s->acc_f ...
   */
}
```

And at the call site, replacing the old single-sensor block:

```c
      for (i = 0; i < CAL_STREAM_MAX; i++)
        {
          cal_stream_service(fd, &g_stream[i], frame);
        }
```

`frame` stays a single shared scratch buffer — it is filled and written within one call and never held across one, so the streams cannot interleave in it.

To prove no reference was missed, stream two sensors whose units differ visibly and confirm each plots in its own range:

```
sel sensor_accel0 sensor_gyro0
```

Accel should sit near 9.81 on one axis, gyro near zero on all three. If either shows the other's magnitudes, an `st_*` reference is still pointing at shared state.

- [ ] **Step 6: Build**

```bash
./tools/build.sh
```

Expected: exits 0, no warnings.

- [ ] **Step 7: Verify nothing else regressed**

```bash
./tools/verify.sh 2>&1 | grep -E "FAIL|exited|rebuilt"
```

Expected: only the known `test-cpu-runtime` failure.

- [ ] **Step 8: Commit**

```bash
git add apps/cal/cal.c
git commit -m "cal: stream several sensors at once

Alignment correlates a magnetometer against a gyro, which requires both at
once on a shared timebase. Streaming was single-sensor by construction -
'a single subscription keeps the frame free of any which-sensor-is-this
ambiguity beyond the id byte'.

The wire format does not change. The frame always carried that id byte and
the host decoder always unpacked it; nothing was reading it as more than a
formality. An unknown name in a selection starts none of them, because a
partially started stream would have the host solving against a sensor that
is silently absent.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: The `still` capture command

**Files:**
- Modify: `apps/cal/cal.c`

**Interfaces:**
- Produces: command `still <name>` — captures a stillness-checked average and emits `{"evt":"still","sensor":"<name>","mean":[x,y,z],"sd":[x,y,z],"n":N}`, or an error.

- [ ] **Step 1: Read the pattern to copy**

```bash
sed -n '1167,1220p' apps/cal/cal.c
```

`cal6 capture` already does exactly this — subscribe, `orb_set_interval`, `cal_capture_still()`, reject on standard deviation, unsubscribe. The new command is that sequence made general and reporting rather than accumulating.

- [ ] **Step 2: Implement**

Add to the command parser, beside the `cal6` handling:

```c
      else if (strncmp(line, "still ", 6) == 0)
        {
          float mean[3];
          float sd[3];
          int idx = cal_find(line + 6);
          int sub;
          int n;

          if (idx < 0)
            {
              cal_emit(fd, "{\"evt\":\"error\",\"msg\":\"unknown sensor\"}\n");
              continue;
            }

          sub = cal_subscribe(idx);

          if (sub < 0)
            {
              cal_emit(fd, "{\"evt\":\"error\","
                           "\"msg\":\"sensor not available\"}\n");
              continue;
            }

          orb_set_interval(sub, 2000);         /* 500 Hz is ample */
          n = cal_capture_still(idx, sub, 4000, mean, sd);
          orb_unsubscribe(sub);

          if (n < 0)
            {
              cal_emit(fd, "{\"evt\":\"error\","
                           "\"msg\":\"sensor produced nothing\"}\n");
              continue;
            }

          /* Report the deviation and let the HOST decide. Alignment tolerates
           * far more movement than a calibration does - it only has to pick
           * among 24 discrete rotations - so a threshold that is right for
           * cal6 would reject positions alignment is happy with.
           */

          cal_emit(fd, "{\"evt\":\"still\",\"sensor\":\"%s\","
                       "\"mean\":[%.5f,%.5f,%.5f],"
                       "\"sd\":[%.5f,%.5f,%.5f],\"n\":%d}\n",
                   g_sensors[idx].name,
                   (double)mean[0], (double)mean[1], (double)mean[2],
                   (double)sd[0], (double)sd[1], (double)sd[2], n);
        }
```

If `cal_find()` does not exist under that name, use whatever the existing name-to-index lookup is — find it with `grep -n "g_sensors\[i\].name" apps/cal/cal.c`.

- [ ] **Step 3: Build**

```bash
./tools/build.sh
```

Expected: exits 0.

- [ ] **Step 4: Commit**

```bash
git add apps/cal/cal.c
git commit -m "cal: add a general stillness capture command

cal6 capture already did this for one accelerometer. Alignment needs it for
any sensor, and needs the deviation REPORTED rather than judged on the
board: alignment tolerates far more movement than a calibration does, since
it only has to pick among 24 discrete rotations, so cal6's threshold would
reject positions alignment is perfectly happy with.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 8: The Alignment tab

**Files:**
- Modify: `tools/cal_gui.py`

**Interfaces:**
- Consumes: `align_solve.solve_alignment`, `align_solve.POSITIONS`, `align_solve.AlignError`; board commands `still <name>`, `sel <names...>`, `stop`.

- [ ] **Step 1: Read the existing tab structure**

```bash
grep -n "_sidebar\|_controls\|_build\|Notebook\|add(" tools/cal_gui.py | head -30
```

Follow whatever container the magnetometer panel uses. Reuse `Link` for transport and `_say()` for the log; do not add a second serial path.

- [ ] **Step 2: Phase A — the position walk**

Six prompts in `POSITIONS` order. For each: show the position name and a plain-language instruction ("nose up: point the front of the vehicle at the ceiling"), send `still sensor_accel0` then `still sensor_accel1`, and store both `mean` vectors under that position name.

Show the returned `sd` beside a threshold of **0.5 m/s²** and refuse to advance above it — that is far looser than `cal6`'s 0.08, deliberately, because a 45°-tolerant discrete answer does not need calibration-grade stillness.

Run `align_solve._check_position` on each reading as it arrives and surface the `AlignError` message verbatim rather than a generic failure. Naming which criterion failed is the difference between a procedure that teaches the operator and one that just says no.

- [ ] **Step 3: Phase B — the sweep**

Send `sel sensor_gyro0 sensor_gyro1 sensor_mag0 flow`, collect frames for 20 seconds routed by `sid`, then `stop`.

Show a live excitation bar per body axis, computed with `align_solve._excitation` on the running attitude, and hold the Finish button disabled until all three exceed `align_solve._MIN_EXCITATION`. That is the auto-capture gate: the operator cannot submit a rank-deficient sweep, which is most of what "foolproof" means here.

Resample each stream onto the gyro's timebase using its frame `t0`/`dt` before building the `session` dict.

- [ ] **Step 4: Preview and commit**

Call `solve_alignment`, then show a row per sensor: solved enum with its `rotation_name`, degrees off, cross-check pass/fail, and any error verbatim.

Commit is a separate explicit button, disabled unless every sensor either solved cleanly or is deliberately skipped. It sends:

```
set SENS_IMU0_ROT <n>
set SENS_IMU1_ROT <n>
set SENS_MAG0_ROT <n>
set SENS_BOARD_ROT 0
commit
```

`SENS_BOARD_ROT` goes to zero because with the board mounted, "sensor relative to board" and "sensor relative to vehicle" are the same thing, so each `SENS_*_ROT` carries the whole rotation.

**Flow is displayed but never written.** No parameter reads it, and adding one now would repeat exactly how `EK3_SRC*`, `SENS_MAG_RATE` and `SENS_MAG0_ROT` all came to be live parameters that nothing consumed.

A **mirrored** result blocks commit entirely and says so: it is either a known part property that belongs in code, as the IST8310 flip now is, or a wiring fault. Both need a human.

- [ ] **Step 5: Check the GUI still starts**

```bash
tools/test-cal-gui.sh
```

Expected: PASS — whatever that harness checks must keep passing.

- [ ] **Step 6: Full gate**

```bash
./tools/verify.sh 2>&1 | grep -E "FAIL|exited|rebuilt"
```

Expected: only `test-cpu-runtime`.

- [ ] **Step 7: Commit**

```bash
git add tools/cal_gui.py
git commit -m "cal_gui: guided sensor alignment

Six positions then one sweep. The Finish button stays disabled until all
three body axes have been rotated about, so a rank-deficient sweep cannot
be submitted - the solver would otherwise return a confident wrong answer
rather than an error.

Stillness is judged at 0.5 m/s^2 rather than cal6's 0.08. Alignment picks
among 24 discrete rotations and tolerates a 45-degree error, so
calibration-grade stillness would reject positions it is perfectly happy
with.

Flow is displayed and not written: no parameter reads it yet, and adding
one now is how EK3_SRC*, SENS_MAG_RATE and SENS_MAG0_ROT all became live
parameters that nothing consumed.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Hardware verification

Tasks 1–5 need no hardware. From Task 6 onward, flash and check in order.

**After Task 6–7 (firmware):**

```
cal
sel sensor_gyro0 sensor_mag0
```

Both should stream together, and `cal_gui` should plot both without one displacing the other. Then:

```
still sensor_accel0
```

Expect a `still` event with `mean` near `(0, 0, 9.81)` on a level bench and a small `sd`.

**After Task 8, run the full procedure.** The things to watch:

1. **`SENS_IMU1_ROT` is being re-measured.** It currently reads `2`, from motion correlation in the 2026-07-26 audit, cross-checked against PX4's fmu-v6c under the roll-180 conjugation that negates a yaw. If the procedure disagrees, chase it rather than override it — one of the two is wrong and it matters which.
2. **The magnetometer should NOT report mirrored**, because the solver consumes raw `sensor_mag0` while `mag_frame.c` applies the flip downstream. If it *does* report mirrored, that confirms the hardcoded flip is real and still needed. If it reports a clean rotation instead, the flip is wrong and the heading is currently backwards again.
3. **Cross-check must pass for both IMUs.** A failure there means the accelerometer and gyroscope of the same part disagree about their own mounting, which is a driver problem, not a mounting one.

## Known limitations

Carried from the spec, restated because they shape what a failure means:

- Alignment cannot confirm an accelerometer calibration. Opposite-position averaging cancels bias, which is what lets it run on an uncalibrated sensor and equally why a bias error cannot show up here.
- A platform too heavy to rotate through a rich sweep cannot have its magnetometer or flow sensor aligned. The IMUs would still be solved by Phase A.
- Optical flow's rotation is solved and shown but not stored until flow fusion exists to read it.
