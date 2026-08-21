# Guided Sensor Alignment — Design

Date: 2026-08-21
Branch: `step5-estimator-imu-pipeline`
Status: approved, ready for implementation planning

## Goal

Determine each sensor's orientation relative to the vehicle by measurement
rather than by assertion, using gravity as the reference and a guided
procedure that cannot accept a bad measurement.

The output is one rotation per sensor, direct to the body frame:
`IMU0 -> Body`, `IMU1 -> Body`, `MAG -> Body`, `FLOW -> Body`. The first
three are written to parameters; the flow result is reported only, because
no parameter reads it yet (see "Outputs").

## Why this exists

Every rotation parameter in the tree today is either a default nobody
measured or a value derived once from an offline correlation. `SENS_MAG0_ROT`
sat at its default and had never been applied by anything until Flash B, and
the magnetometer turned out to be mirrored rather than rotated - a case no
rotation parameter can express and which no amount of reading the code would
have revealed. `SENS_IMU1_ROT = 2` came from motion correlation in
`docs/imu-timestamp-audit-2026-07-26.md` and agrees with PX4's fmu-v6c
values, but has never been re-measured on this board.

`rotation.h` states the hazard directly: every rotation value is a plausible
orientation, so a wrong one does not fail, it silently reads the vehicle
sideways. A procedure that measures these is the only thing that turns them
from assumptions into facts.

## Non-goals

- **Translation / lever arms.** A sensor's position only shows up under
  rotation, through `a = omega_dot x r + omega x (omega x r)`. Two IMUs about
  2 cm apart at 3 rad/s produce roughly 0.18 m/s^2, which is marginal against
  accelerometer noise. For the magnetometer and barometer a lever arm is
  physically meaningless - field and pressure are uniform across a 5 cm
  board, so there is no effect to measure. `SENS_*_POS_*` stays hand-entered
  from the board layout, as the roadmap already recommended.
- **Fine rotation.** No residual is written. `CAL_*_RV*` stays zero and
  unused by this procedure; a mounting is either an axis permutation or it is
  refused.
- **External navigation.** `EK3_SRC*` can name `EXTERNAL_NAV = 6`, but no
  topic is declared and nothing publishes one. There is nothing to sample.
- **Re-deriving the IST8310 handedness fix.** See "Falsifying the hardcoded
  flip" below - the procedure validates it rather than rediscovering it,
  which is a deliberate choice with a stated cost.

## The physics, and why the procedure is shaped this way

**Gravity beats motion by an order of magnitude.** An earlier draft derived
the forward axis from a push: 1-2 m/s^2 of hand acceleration against 9.8
m/s^2 of gravity that must first be removed using the attitude being solved
for. Tipping the vehicle instead makes gravity itself the reference, at
9.81 m/s^2, static, with no filtering and no cross-stream timing.

**The answer is discrete, which is what makes it robust.** There are only 24
representable rotations, so a tip need only be within about 45 degrees to
land on the right one. Sloppy positioning cannot produce a wrong answer, only
a refused one. Two worries that applied to the motion-based approach -
ground slope, and whether the vehicle rolls straight - disappear entirely.

**The sign convention, stated once, because this is where errors hide.** At
rest an accelerometer measures specific force, which points UP. A level FLU
board reads `+g` on Z, which is what the estimator already assumes.
Therefore:

> The vehicle axis pointing UP reads `+g`.

Nose down puts `-X` up, so the vehicle-frame reading is `(-g, 0, 0)`.
Whatever the SENSOR reads in that position is that vehicle axis expressed in
sensor coordinates - one column of the rotation matrix, read directly, with
no solving.

**Gyros come free.** Accelerometer and gyroscope share a package in both the
ICM-42688-P and the BMI055, so the gyro rotation is the accel rotation. Rate
correlation is therefore a cross-check, not a second source of truth - which
is where it belongs, being the noisier of the two.

**The magnetometer cannot be solved from static positions.** The mathematics
works - `R_mag * m_k = T_k^T * m_e` across positions, six unknowns and
eighteen equations - but only if every position shares one common heading,
and heading is degenerate when the nose is vertical. Tipping nose-down and
then left-down does not naturally preserve a common yaw. It would be fragile
in exactly the way this procedure exists to avoid, so the magnetometer is
solved from the rotation sweep instead.

**Optical flow has no accelerometer.** The MTF-02 reports flow and
`integrated_xgyro/ygyro/zgyro` and nothing else. Its reference must be rate,
not acceleration. Those gyro channels work regardless of whether it can see a
surface, so no textured floor at a valid standoff is required.

## Procedure

One session, with the board mounted in the vehicle. That is what makes
"board" and "vehicle" the same frame, and what lets every sensor be solved
straight to body with no intermediate to get wrong.

### Phase A - six static positions

Level, inverted, nose up, nose down, left side down, right side down.

For each: the host prompts, the board reports stillness, and an average is
captured. `cal_capture_still()` in `apps/cal/cal.c` already does exactly this
- average a stretch and say whether the board was actually still - and is
reused unchanged.

Each opposite pair averages to cancel accelerometer bias, so the columns come
out clean even from an uncalibrated sensor. That is deliberate: alignment
must not depend on a calibration that itself depends on knowing the
orientation.

Produces `IMU0 -> Body` and `IMU1 -> Body`.

### Phase B - one rotation sweep

The operator rotates the whole vehicle richly for about 20 seconds. Capture
is automatic and gated on per-axis excitation; the procedure will not proceed
until all three axes have been exercised, because a rank-deficient solve
returns confident nonsense rather than an error.

Produces `MAG -> Body` and `FLOW -> Body`, both referenced to the IMU solved
in Phase A.

## Solvers

All solving is host-side. The board streams and reports stillness; it does
not fit. This follows the precedent set when the magnetometer ellipsoid fit
was moved to the host.

### Accelerometer columns

For each opposite pair, `(reading_up - reading_down) / 2` gives that vehicle
axis expressed in sensor coordinates, with bias removed. Three pairs give
three columns; assemble and orthonormalise.

### Magnetometer

Input is the RAW `sensor_mag0` topic, not `vehicle_mag`. `mag_frame.c`
applies the handedness flip and the mounting rotation before `vehicle_mag` is
published, so solving against it would be solving against the answer. See
"Falsifying the hardcoded flip".

Integrate the IMU gyro across the sweep to get relative attitude `C(t)`. The
earth's field is constant in the earth frame, so the correct rotation is the
one that makes it stop moving:

> choose `R_mag` to minimise the variance of `C(t) * R_mag * m(t)`

This never needs to know what the field actually is, and it is heading-free -
it does not care how the operator turned the vehicle. Segment length is
seconds, so gyro drift over the window is negligible.

### Optical flow

Rate correlation against the IMU gyro. The flow topic reports integrated
angle over a window, so rate is `integrated_*gyro / integration_time_us`.
Resample onto a common grid and solve Kabsch against the IMU rate.

### Flip detection

Standard Kabsch forces `det = +1` through `diag(1, 1, det(U V^T))`, which
would quietly turn a mirrored sensor into the nearest rotation. This is not
hypothetical: the IST8310 is genuinely mirrored against the vehicle frame.

So both fits are computed - the constrained proper rotation and the
unconstrained best orthogonal matrix - and their residuals compared. If the
reflection fits dramatically better, the sensor is mirrored, and that is
reported and nothing is written. A mirrored sensor is either a known part
property, which belongs in code as the IST8310 flip now does, or a wiring or
driver fault. Both need a human; neither should be absorbed into a
calibration.

For the accelerometer the same test is simply `det` of the assembled columns,
which is static and high-SNR and therefore the most reliable flip detection
in the system.

### Snapping

Every solved rotation is snapped to the nearest of the 24 representable
values, and the angle to that snap is reported. Beyond 15 degrees the
procedure refuses and shows how far off it was: the mounting is not an axis
permutation, `rotation.h` deliberately cannot represent it, and approximating
would silently read the vehicle sideways. Below the threshold the enum value
is written and the residual is discarded.

## Outputs

`SENS_IMU0_ROT`, `SENS_IMU1_ROT`, `SENS_MAG0_ROT`, each a full sensor-to-body
rotation, with `SENS_BOARD_ROT = NONE`.

With the board mounted, "sensor relative to board" and "sensor relative to
vehicle" coincide, so the existing `BOARD_ROT(SENS_ROT(raw))` composition
holds unchanged and nothing in the application path is touched. If the board
is later re-mounted differently, `SENS_BOARD_ROT` still works as the single
place to express that.

Optical flow has no rotation parameter today. One is added only when flow
fusion is built; until then the solved value is reported and logged, not
stored, because a parameter nothing reads is how `EK3_SRC*`, `SENS_MAG_RATE`
and `SENS_MAG0_ROT` all came to be dead in the first place.

Nothing is written without an explicit commit, and a valid stored transform
is never overwritten by an uncertain fit.

## What makes it refuse

| Condition | Response |
|---|---|
| Snap residual > 15 degrees | Refuse; report the angle |
| `det = -1` | Report "mirrored, not rotated", name the axis, write nothing |
| Rank-deficient sweep | Refuse; name the axis that was not exercised |
| Not still | Re-prompt; `cal_capture_still()` already reports this |
| Not within ~45 degrees of the target position | Re-prompt. Checked by taking the largest-magnitude accelerometer component: if it is not at least twice the next largest, the tip is ambiguous between two axes and the position is rejected |
| Two positions resolving to the same axis | Refuse; the operator repeated a position or skipped one |
| Columns not orthogonal within tolerance | Refuse; report the worst pair |
| Gyro cross-check disagrees with the accel result | Report which sensor is inconsistent. The check is Kabsch on the Phase B rate streams against the Phase A accel rotation for the same part; they must snap to the same enum value |

The operator cannot submit a bad segment. That is most of what "foolproof"
means here: prevention at capture, not diagnosis afterwards.

## Components

| File | Responsibility |
|---|---|
| `tools/align_solve.py` | Pure solver. Segments in, rotations plus diagnostics out. No serial, no GUI, no Tk. |
| `tools/cal_gui.py` | Alignment tab: prompts, live excitation and stillness meters, auto-capture, preview, commit. Reuses `Link` and `Strip`. |
| `apps/cal` | New `align` mode: streams the sensor set and reports stillness. All sequencing and solving stay on the host. |
| `tests/align_solve_test.py` | Synthetic data with known answers. |
| `tools/test-align-solve.sh` | Runner, following `tools/test-mag-cal-host.sh`. |

Keeping `align_solve.py` free of Tk and pyserial is what makes it testable at
all; the GUI is a shell around it.

## Testing

Synthetic streams generated through a KNOWN rotation must come back as that
rotation exactly. Beyond that, each refusal path gets a test, because a
refusal that does not fire is indistinguishable from a pass:

- a mirrored sensor is reported, not fitted
- single-axis-only sweep motion is refused as rank-deficient
- noise beyond tolerance is refused rather than reported
- a 30-degree off-axis mounting is refused, not snapped
- an opposite-position pair with a large bias still yields the correct column

## Known limitations

**It will re-measure `SENS_IMU1_ROT` and may disagree with the current value.**
The existing `2` came from motion correlation at 0.999 across three axes and
cross-checks correctly against PX4's fmu-v6c values under the roll-180
conjugation that negates a yaw. If this procedure disagrees, that is a real
finding worth chasing, not a result to override.

**Falsifying the hardcoded flip.** `mag_frame.c` applies the y-negation
before anything downstream sees it, so a solver consuming `vehicle_mag` would
see an already-corrected field and correctly report no flip - validating the
fix rather than independently rediscovering it. The solver therefore consumes
raw `sensor_mag0`, which keeps the hardcoded flip falsifiable. This is the
whole reason the raw topics are kept raw.

**Phase B requires the vehicle to be rotated by hand.** A platform too heavy
to rotate through a rich sweep cannot have its magnetometer or flow sensor
aligned by this procedure. The IMUs would still be solved by Phase A.

**Alignment does not validate calibration, and vice versa.** Deliberately
separate procedures: alignment uses opposite-position averaging so it does
not depend on the accelerometer calibration, which means it also cannot
confirm one.
