# Calibration, warm-up, and vehicle models

The estimator uses one explicit frame convention throughout:

- sensor and vehicle axes are body **FLU** (X forward, Y left, Z up);
- navigation position and velocity are local **ENU**;
- every position parameter is named `source -> destination` in its comment;
- angular lever-arm velocity is always `v_destination = v_source + omega x r`.

The calibration tools deliberately refuse to estimate degrees of freedom that
the recorded measurements cannot observe. A plausible-looking number from an
unobservable fit is more dangerous than leaving the parameter at its measured
default.

## Log-to-extrinsic calibration

Install the host dependencies once:

```sh
python3 -m pip install --user numpy pyulog
```

Record `LOG_EKF=1`, external pose, and wheel speed. Start with at least ten
seconds completely stationary, then drive more than five metres with smooth
left and right turns. Use both directions of travel if practical. Avoid wheel
spin and impacts during the calibration segment.

Solve and inspect a log:

```sh
python3 tools/extrinsic_cal.py flight.ulg --output extrinsics.json
python3 tools/extrinsic_cal.py flight.ulg --nsh
python3 tools/frame_gui.py
```

The GUI overlays BODY, IMU0, IMU1, MAG0, and MOCAP axes and shows the fitted
time offset, marker yaw, planar lever arm, residual, conditioning, thermal
settling, and motion-excitation checks. The generated `param set` lines are a
review artifact; the tool never changes a connected vehicle.
Any parameter whose fit-quality gate fails is emitted as a commented
`withheld` candidate rather than an executable command.

The **Sensor intrinsic calibration** button opens the existing live six-face
accelerometer, stationary gyro, and magnetometer wizard (`tools/cal_gui.py`).
Keeping intrinsic calibration as a deliberate static capture and the rigid
marker transform as a batch log solve prevents dynamic vehicle acceleration
from being misidentified as accelerometer scale error.

The current external-pose packet contains X, Y, and yaw only. It can identify
marker-to-IMU X/Y/yaw and relative timestamp delay. Marker Z, roll, and pitch
are unobservable and are therefore neither guessed nor overwritten. A full
six-degree-of-freedom solve requires the companion protocol and logger to
carry Z plus roll/pitch (preferably a quaternion). This is the same fundamental
observability requirement enforced by multi-sensor batch calibration tools
such as [Kalibr](https://github.com/ethz-asl/kalibr/wiki/Calibrating-the-VI-Sensor).

The solver uses local-polynomial derivatives on the irregular pose timestamps,
then:

1. correlates mocap yaw rate with gyro Z for relative time;
2. compares signed position course with marker yaw for relative yaw;
3. robustly solves rigid-body centripetal/tangential acceleration for X/Y
   lever arm and checks the design-matrix condition number.

Position-only course has an unavoidable 180-degree nose/tail ambiguity. A log
containing signed wheel velocity removes it. Always compare the reported lever
arm to a tape measurement and verify all displayed axes before applying it.

Sensor *intrinsics* (accelerometer scale/misalignment and gyro scale) are not a
rigid transform. A planar driving log cannot identify all of them. Keep using
the existing six-face accelerometer calibration and stationary gyro bias
alignment; do not ask this planar tool to manufacture a 3-D intrinsic result.

## Warm-up and readiness

Boot warm-up has two phases:

1. **Static alignment:** the existing ten-second stationary gate estimates
   gyro bias and accepts gravity only when accel/gyro variance passes.
2. **Operator-driven excitation:** after alignment, drive the configured
   distance and turn both left and right by the configured total angle.

`EK3_WARM_DIST` and `EK3_WARM_YAW` control the second phase. `ekf3 status` and
the `EST_HEALTH_MOTION_EXCITED` flag report progress. This is a readiness
diagnostic, not an actuator command and not a substitute for the safety state
machine. The vehicle must never move itself merely to warm an estimator.

For best bias repeatability, wait until the GUI reports a small temperature
slope before recording calibration data. The runtime filter remains usable
after static alignment; motion readiness says that online states have received
representative excitation, not that a cold vehicle is forbidden to operate.

## Vehicle model (`EK3_VEH_TYPE`)

| Value | Class | Runtime pseudo-measurement |
|---:|---|---|
| 0 | Generic | None |
| 1 | Car | Body-forward wheel speed, soft body-left zero speed, and gated soft body-Z ground constraint |
| 2 | Fixed-wing | None yet; reserved until calibrated airspeed/relative-wind data exists |
| 3 | Multirotor | None; all three translation axes remain independently mobile |

The car model is the standard non-holonomic constraint expressed in **body**
axes. It never clamps ENU Z. Consequently, a car climbing a hill has zero
velocity through its floor but a nonzero ENU vertical velocity. Wheel velocity
uses the complete attitude rotation and the complete body-rate lever-arm term,
so pitch/roll and an elevated IMU do not become false speed.

The body-Z observation is enabled only after accepted, non-slipping wheel
fusion and when acceleration magnitude indicates ground contact. Freefall,
large vertical impacts, clipped IMU samples, and the existing filtered slip
test block it. `EK3_CAR_VZ_NSE` controls its softness; `EK3_CAR_ZACC` controls
the residual-acceleration contact gate. Acceptance, rejection, blocking, and
NIS are printed by `ekf3 status` and recorded in `estimator_health`.

This approach follows published land-vehicle INS practice: non-holonomic
constraints are treated as noisy observations and disabled when the assumption
is invalid, rather than hard-clamping a state. See the peer-reviewed
[land-vehicle NHC and lever-arm calibration method](https://pmc.ncbi.nlm.nih.gov/articles/PMC6069284/)
and [robust NHC handling under abnormal motion](https://pmc.ncbi.nlm.nih.gov/articles/PMC7792609/).

Fixed-wing synthetic sideslip must use *relative air velocity* and a wind
estimate, not ground velocity. PX4's ECL derivation likewise formulates the
sideslip observation from body-relative air velocity
([derivation source](https://github.com/PX4/PX4-ECL/blob/master/EKF/python/ekf_derivation/main.py)).
It is intentionally not enabled here until an airspeed input and fault gates
exist. Multirotors receive no car-like constraint; future drag fusion would
require measured vehicle-specific drag coefficients.
