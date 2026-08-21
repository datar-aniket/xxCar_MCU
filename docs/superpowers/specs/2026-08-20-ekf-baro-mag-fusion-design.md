# EKF3 Barometer and Magnetometer Fusion — Design

Date: 2026-08-20
Branch: `step5-estimator-imu-pipeline`
Status: approved, ready for implementation planning

## Goal

Make EKF3 use the two aiding sensors the board already has and already
publishes: the MS5611 barometer and the IST8310 magnetometer. Today it uses
neither. The result should be an absolute heading (not the current arbitrary
relative yaw) and an observable vertical position, selected through the
`EK3_SRC*` parameters that already exist and are currently inert.

## Starting state

The audit that produced this design found the following.

`apps/ekf3/ekf3.c` subscribes to exactly one topic, `vehicle_imu`. There is no
second subscriber and no measurement queue. `ekf_core.c` is inertial propagation
plus one internal aiding update, `low_dynamics_updates()`, which observes the
gravity vector while the vehicle is near-stationary and explicitly projects yaw
out of the Kalman gain — correctly, since gravity carries no heading
information.

Yaw is initialised to a literal `0.0f` in `alignment_add()`. Roll and pitch come
from a one-second accelerometer mean; heading is arbitrary. That is why the
solution advertises `YAW_RELATIVE` and never an absolute heading.

`ekf_sources_load()` reads and validates all fifteen `EK3_SRC*` parameters at
startup. Its only consumer is a `printf` of the active set number in
`ekf3_main.c`. `EK3_SRC1_POSZ=1` (baro) and `EK3_SRC1_YAW=1` (compass) are
configured and fused by nothing.

Underneath, the drivers are healthy and publishing `sensor_mag0` (Gauss, raw
sensor axes) and `sensor_baro0` (hPa). Magnetometer calibration parameters are
complete: hard-iron offsets, a 3×3 soft-iron matrix, expected field strength, a
`CAL_MAG0_OK` validity flag, mounting rotation and position. The barometer has
no parameters at all beyond a rate.

Two further dead parameters surfaced: **`SENS_MAG_RATE` and `SENS_BARO_RATE` are
read by nothing.** Every driver implements `set_interval`, but the only caller of
`orb_set_interval` in the tree is `apps/cal/cal.c`, with hardcoded values. The
frontend introduced here will apply them.

This work is Steps 3 and 4 of the five-step roadmap agreed earlier (see
`tools/Conversation_hist.md`). Step 1 (parameters, MS5611 compensation) is
committed. Step 2 (host extrinsic calibration GUI) was never built and is not a
prerequisite: the fine rotation parameters it would populate default to identity,
which is the correct starting assumption for a board-mounted magnetometer.

## Non-goals

- **Expanding beyond 15 states.** ArduPilot's full three-axis compass fusion
  estimates earth-field and body-field states 16–21. Reproducing it faithfully
  requires those states. That is Step 5, after heading and height are proven.
- **GPS, optical flow, external navigation, wheel odometry.** The source
  parameters can name them; the frontend will report them as unavailable rather
  than pretend.
- **Automatic declination.** With no GPS there is no position to look one up
  from. Declination is a number the operator enters.
- **Horizontal position or velocity validity.** Nothing in this design makes
  either observable. They stay invalid.

## Phasing

Two flashes, because magnetometer and barometer fusion fail in unrelated ways and
should be diagnosable separately on hardware.

**Flash A** — delayed fusion horizon, measurement frontend, barometer height
fusion. Touches attitude only through the timing change, which is verifiable as
inert. Vertical position is currently advertised `[INVALID]` and consumed by
nothing, so a wrong barometer cannot break anything that works today.

**Flash B** — magnetometer heading initialisation and gated yaw fusion. This is
the change that can corrupt a presently-stable gyro-propagated heading, and it
gets its own flash for that reason.

## Flash A

### Corrected topics

Two new topics in `apps/uorb_msgs`, following the `vehicle_accel`/`vehicle_gyro`
pattern already established for the IMU.

```c
struct vehicle_mag_s
{
  uint64_t timestamp;         /* us, publication time              */
  uint64_t timestamp_sample;  /* us, driver sample time            */
  float    field[3];          /* Gauss, calibrated, body frame     */
  float    temperature;       /* degrees C                         */
  uint8_t  calibrated;        /* 0 = raw passthrough, 1 = corrected */
  uint8_t  instance;
  uint8_t  pad[2];
};

struct vehicle_baro_s
{
  uint64_t timestamp;
  uint64_t timestamp_sample;
  float    pressure;          /* hPa                               */
  float    temperature;       /* degrees C                         */
};
```

`vehicle_baro` deliberately carries pressure, not height. The pressure-to-height
conversion needs a reference pressure, and that reference is captured at EKF
alignment — it belongs to the estimator, not to the sensor. Publishing height
here would bake an estimator concern into a sensor topic and make the topic
meaningless before the EKF has aligned.

`NuttX`'s `struct sensor_mag` and `struct sensor_baro` each carry a single
`timestamp` field, which is the driver's sample time. That becomes
`timestamp_sample`; `timestamp` is set at publication.

### Frontend placement

Two new files under `apps/sensors/`:

- **`mag_correct.c` / `.h`** — pure calibration mathematics, no I/O. Host-testable
  behind a `MAG_CORRECT_HOST_TEST` guard, the same arrangement `cal_mag.c` and
  `ms5611_comp.c` already use.
- **`aux.c`** — one low-rate daemon thread polling `sensor_mag0` and
  `sensor_baro0`, publishing both corrected topics, and applying `SENS_MAG_RATE`
  and `SENS_BARO_RATE` via `orb_set_interval`.

`sensors.c` is untouched. It is already ~870 lines and its stated purpose is
turning one raw IMU into corrected body-frame topics; the magnetometer and
barometer are a separate concern on a separate timescale, and folding them in
would blur a boundary that currently reads cleanly.

Both sensors are slow — 50 Hz and 10 Hz — so a single thread polling both file
descriptors is correct. Separate threads would buy nothing but context switches.

### Magnetometer correction chain

```
    corrected = SOFT_IRON · (raw − OFFSET)      CAL_MAG0_{X,Y,Z}OFF,
                                                CAL_MAG0_{XX,YY,ZZ,XY,XZ,YZ}
    body      = BOARD_ROT( FINE_ROT( MAG_ROT( corrected ) ) )
                                                SENS_MAG0_ROT,
                                                CAL_MAG0_RV{X,Y,Z},
                                                SENS_BOARD_ROT
```

Ordering follows the same reasoning as `sensors.h` documents for the IMU:
calibration is measured in the sensor's own axes, so it must be applied before
any rotation. A soft-iron matrix means nothing once the axes have been mixed.

`CAL_MAG0_RV*` is the fine residual rotation Step 2's host calibration would
solve for. Defaulting to zero yields identity, which is the right assumption for
a magnetometer soldered to a known board.

When `CAL_MAG0_OK` is zero, `vehicle_mag` is published with `calibrated = 0` and
the raw field passed through. The topic stays observable for diagnostics; the EKF
declines to fuse it.

### Delayed fusion horizon

New files `ekf_delay.c` / `.h` in `apps/ekf3/`, holding:

- an IMU ring of `vehicle_imu` samples sized to cover the maximum horizon at
  400 Hz,
- timestamped measurement queues for magnetometer and barometer.

The filter advances only to `now − EK3_DELAY_MS`, consuming ring entries whose
`timestamp_sample` falls at or before the horizon. Measurements are popped and
fused when their sample time falls inside the step being taken, so each
observation corrects the state as it was when the observation was made.

`ekf_core_process()` keeps its current signature. The ring feeds it exactly the
samples it already expects, which means the 400 Hz nominal / 100 Hz covariance
cadence, every validity check in `sample_valid()`, and every fault counter
survive unchanged.

**`EK3_DELAY_MS` defaults to 0.** At zero the ring is drained every tick, the
horizon is the present, and the filter reproduces today's behaviour exactly. This
is deliberate: a parameter default should reproduce known-good behaviour, and it
lets the timing rewrite be confirmed inert on hardware before any measurement
starts correcting anything. The verification sequence raises it.

### Output predictor

On each publication, replay the IMU ring forward from the delayed filter state to
the present, and publish that.

This is chosen over ArduPilot's output observer — an incrementally maintained
output state corrected against the horizon through a complementary filter — for
two reasons. It is deterministic and cannot drift, so it introduces no new
failure mode to debug alongside first-ever aiding. And it has no tuning
constants, so there is nothing to get wrong. The cost is roughly forty quaternion
propagations per 400 Hz tick on an H7 FPU, which `top` will measure; if it proves
material, the output observer remains available as a later optimisation with the
re-propagation as its reference implementation.

### Barometer fusion

`ekf_core.c` gains `measurement_update_1d()`, mirroring the existing
`measurement_update_3d()`: same expanded Joseph covariance form, same NIS gate,
same attitude covariance reset and bias constraint afterwards. Sharing the
structure keeps one set of numerical-safety behaviours rather than two.

The measurement is height, converted from pressure against the reference captured
at alignment. In NED, `position[2]` is down-positive and barometric height is
up-positive, so the observation is `z_down = −height_up` and the observation
matrix has a single unit entry at state index 8.

Vertical velocity and Z accelerometer bias are corrected through the covariance
cross-terms. This is correct EKF behaviour and is precisely why `EK3_SRC1_VELZ=0`
remains the right setting: the barometer is not a velocity sensor, but height
observations do make vertical velocity observable.

New parameters for Flash A. Names follow ArduPilot where an equivalent exists;
units do not, and the difference is deliberate. ArduPilot encodes its innovation
gates as integer sigma×100 for historical reasons. This codebase uses floats
throughout, so gates are plain sigma.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `EK3_DELAY_MS` | int32 | 0 | Fusion horizon behind real time, ms |
| `EK3_ALT_M_NSE` | float | 2.0 | Barometer height measurement noise, m |
| `EK3_ALT_I_GATE` | float | 5.0 | Height innovation gate, sigma |

Barometer health uses fixed constants in the style of the existing `EKF_*`
gates in `ekf_core.c`, rather than more parameters:

- measurement age bound: 500 ms, after which the barometer is stale,
- rejection run before validity drops: 20 consecutive gated innovations,
- pressure sanity: 500–1200 hPa, outside which the sample is discarded before
  it reaches the queue.

On barometer timeout or sustained rejection, vertical position validity drops.
The filter continues on inertial propagation rather than holding a stale
correction.

### Output validity

The roadmap asks for granular validity, and `solution_status` is a `uint8_t` with
spare bits:

| Bit | Flag |
|---|---|
| 0 | `ATTITUDE` (roll/pitch) |
| 1 | `YAW_RELATIVE` |
| 2 | `YAW_ABSOLUTE` |
| 3 | `VELOCITY_HORIZ` |
| 4 | `VELOCITY_VERT` |
| 5 | `POSITION_HORIZ` |
| 6 | `POSITION_VERT` |

This **redefines** the existing `ESTIMATOR_VELOCITY_VALID` (bit 2) and
`ESTIMATOR_POSITION_VALID` (bit 3). That is a breaking change to the topic's
meaning, and it is safe only because velocity and position are advertised
`[INVALID]` today and no consumer exists. Bits 0 and 1 keep their current values
and meanings.

## Flash B

### Heading initialisation

The user requirement is that startup uses magnetometer and accelerometer together
to initialise the angles. `alignment_add()` already produces roll and pitch from
the accelerometer mean. It gains a heading step:

1. Average the calibrated field over the same alignment window, subject to the
   same stillness gates the accelerometer already passes.
2. Rotate the averaged field into the horizontal plane using the roll and pitch
   just derived.
3. Magnetic heading is `atan2(−h_y, h_x)`.
4. Add `EK3_MAG_DEC` to get true heading.
5. That value replaces the hardcoded `0.0f` in the `quaternion_from_euler()` call.

Yaw covariance initialises to `EK3_YAW_M_NSE²` instead of the current 180°
entry in `covariance_initialize()`. That is what tells the rest of the filter the
heading is now worth something — without it the state would be correct and the
covariance would still claim it was unknown.

If `CAL_MAG0_OK` is zero, the field magnitude disagrees with `CAL_MAG0_FIELD`, or
the magnetometer is stale, initialisation falls back to yaw = 0 and
`YAW_RELATIVE` — exactly today's behaviour. The filter does not refuse to start;
it declines to claim an absolute heading it cannot support. This matches how
`cal` already refuses to store an unchecked result rather than storing a bad one.

### Yaw fusion

Tilt-compensated heading, fused as a scalar innovation through
`measurement_update_1d()`, observing attitude error about the down axis.

This form was chosen over three-axis vector fusion because it observes exactly
the one quantity gravity cannot — heading — and nothing else. With no earth-field
or body-field states in a 15-state filter, a three-axis update would let any
hard-iron residual, soft-iron residual, or local magnetic disturbance leak
straight into roll and pitch, corrupting the one part of the solution that
currently works well.

Health gating, any failure of which drops `YAW_ABSOLUTE` while retaining
`YAW_RELATIVE` on gyro propagation:

- `CAL_MAG0_OK` set and `vehicle_mag.calibrated` non-zero,
- field magnitude within ±30% of `CAL_MAG0_FIELD`,
- measurement age under 500 ms,
- innovation within `EK3_YAW_I_GATE`,
- fewer than 20 consecutive gated innovations.

As with the barometer, the tolerances that are not tuning decisions are fixed
constants alongside the existing `EKF_*` gates rather than parameters. The ±30%
band is wide enough to tolerate ordinary vehicle-borne distortion and narrow
enough to catch a magnet, a motor, or a failed sensor.

Rejecting a bad heading and continuing on the gyro is always better than
injecting it. Yaw drifts slowly; a wrong absolute heading is wrong immediately.

New parameters for Flash B:

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `EK3_MAG_DEC` | float | 0.0 | Magnetic declination, degrees |
| `EK3_YAW_M_NSE` | float | 0.5 | Yaw measurement noise, rad |
| `EK3_YAW_I_GATE` | float | 5.0 | Yaw innovation gate, sigma |

`EK3_YAW_M_NSE` takes ArduPilot's default of 0.5 rad, which is far looser than a
compass's actual accuracy. That is intentional on their part and worth keeping
here: a loose measurement noise makes the filter lean on the gyro between
updates and limits how hard a single disturbed reading can pull the heading. It
is the first thing to tune once the path is proven on hardware.

All six new parameter names fit within `PARAM_NAME_MAX` (16).

## Source parameter semantics

`ekf_sources.c` already validates that a selection is physically meaningful. This
design connects the validated result to fusion:

- `EK3_SRCn_POSZ = 1` enables barometer height fusion. Any other value leaves
  vertical position unaided.
- `EK3_SRCn_YAW = 1` enables magnetometer yaw fusion. Any other value leaves
  heading relative.
- Values naming sensors that do not exist on this vehicle report as unavailable
  in `ekf3 status`. They do not silently fall back to something else.

A measurement never becomes valid merely because a parameter selected it. Source
selection makes a measurement *eligible*; health gating decides whether it is
*used*.

## Observability

`ekf3 status` gains per-source reporting, in the style of the existing gravity
update line:

- horizon depth and current ring occupancy,
- per source: last sample age, accept count, reject count, last innovation, last
  NIS, timeout count,
- magnetometer field magnitude against expected, and the declination in use,
- barometer reference pressure and derived height,
- the granular validity flags, decoded.

The existing fault counters, gravity update line, and yaw gauge suppression line
stay as they are.

## Error handling

Every new path follows the reject-and-report convention the estimator already
uses. A measurement that fails validity increments a named counter and is
dropped; it never partially updates the filter. Numerical failure inside an
update returns the same `-1` the 3-D update already returns, which triggers the
existing alignment restart. A frontend that cannot advertise its topics logs
which one failed by name — `sensors.c` records that ambiguity there costing a
flash cycle to diagnose.

## Testing

Host tests following the established `tests/*.c` plus `tools/test-*.sh` pattern:

| Test | Covers |
|---|---|
| `mag_correct_test` | Hard/soft-iron application, rotation composition, ordering |
| `ekf_delay_test` | Ring ordering, horizon arithmetic, measurement pop timing, re-propagation |
| `ekf_baro_test` | Height conversion, sign convention, gate, timeout, reference capture |
| `ekf_mag_yaw_test` | Tilt compensation, declination, gate, fallback to relative |
| `ekf_core_test` (extended) | `measurement_update_1d()` against the 3-D update; delay-zero equivalence |

The delay-zero equivalence test is the important one for Flash A: with
`EK3_DELAY_MS = 0` and no measurements, the reworked pipeline must produce
bit-identical output to the current core over a recorded IMU sequence.

## Hardware verification

**Flash A**, in order — each step confirmed before the next:

1. Boot with `EK3_DELAY_MS = 0`. Confirm `ekf3 status` attitude, rates and fault
   counters match the pre-flash baseline. The rewrite is inert.
2. Confirm `vehicle_mag` and `vehicle_baro` publish at `SENS_MAG_RATE` and
   `SENS_BARO_RATE`, and that changing those parameters changes the observed
   rates. Field magnitude should sit near `CAL_MAG0_FIELD`; pressure near
   800–1100 hPa.
3. Set `EK3_DELAY_MS = 30`. Confirm attitude still healthy and the output
   predictor tracks. Check CPU cost in `top`.
4. Confirm `EK3_SRC1_POSZ = 1`. Confirm height tracks a known vertical
   displacement and `POSITION_VERT` becomes valid.

**Flash B**:

1. Confirm heading at alignment agrees with a known reference bearing.
2. Rotate the vehicle through a full turn; confirm heading tracks and
   `YAW_ABSOLUTE` stays valid.
3. Introduce a magnetic disturbance; confirm the gate rejects, `YAW_ABSOLUTE`
   drops, `YAW_RELATIVE` persists, and it recovers cleanly on removal.

## Known limitations

**Declination has no source.** With no GPS there is no automatic lookup, so
`EK3_MAG_DEC` is entered per location. Left at zero the filter produces magnetic
heading, not true heading. That is adequate if downstream consumers need only a
consistent absolute reference, and wrong if they need true north.

**The barometer is weak on a ground vehicle.** MS5611 noise is on the order of
±0.1–0.3 m, against a car's real height changes. It is worth having as the
vertical anchor to bound drift; it will not be informative about terrain.

**Single magnetometer, no redundancy.** A failed IST8310 means heading falls back
to relative. There is no second compass to switch to, and source-set switching
between primary/secondary/tertiary is Step 5.
