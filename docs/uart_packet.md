# Companion UART link

The framed serial protocol between the flight controller and a companion
computer, and in particular `EXTERNAL_POSE`, the message a localisation stack
sends to feed the estimator.

Implemented in `apps/companion/comp_proto.h` and `.c`; the Python side of the
same format is `tools/comp_link.py`.

## 1. Port

The board claims whichever serial port has its `SER_*_FUNC` parameter set to
**6** (`SER_FUNC_COMPANION`). Baud comes from that port's `SER_*_BAUD`.

| Connector | Device | Func param | Baud param | Default baud |
|---|---|---|---|---|
| TELEM2 | `/dev/ttyS3` (UART5) | `SER_TEL2_FUNC` | `SER_TEL2_BAUD` | 921600 |

8N1, no flow control on the data lines. TELEM2's CTS pin is repurposed as the
PPS input — see section 8 — so hardware flow control must stay off.

The link starts at boot when `COMP_EN` is 1, which is the default, along with
the estimator (`EKF3_EN`), the IMU integrator (`IMU_DELTA_EN`) and the sensor
daemons (`SENS_EN`, `SENS_AUX_EN`). Nothing needs starting by hand.

## 2. Frame

```
0xFE | id | len | payload[len] | crc_lo | crc_hi
```

- `0xFE` sync byte.
- `id` message id, section 3.
- `len` payload length in bytes, 0..244.
- `crc` CRC16 over **`id`, `len` and `payload`** — not over the sync byte.

Total overhead is 5 bytes.

**The length field is authoritative.** A `0xFE` occurring inside a payload is
payload, not a new frame, and a receiver that restarts on every `0xFE` will
corrupt roughly one message in 256 per byte position. There is no escaping
and none is needed.

Anything that is not the sync byte while idle is discarded, so a lost byte
costs one message rather than wedging the stream.

## 3. Message ids

Inbound ids are low, outbound ids are high, so a message sent in the wrong
direction fails to route rather than half-working.

| Id | Name | Direction | Payload |
|---|---|---|---|
| 1 | `EXTERNAL_POSE` | companion → board | 48 |
| 2 | `CONTROL_TRAJ` | companion → board | `20 + 16*horizon`, max 244 |
| 3 | `TIMESYNC_REQ` | companion → board | 8 |
| 4 | `TIMESYNC_REP` | board → companion | 24 |
| 5 | `TIMESYNC_START` | companion → board | 8 |
| 6 | `TIMESYNC_END` | companion → board | 16 |
| 7 | `DIRECT_CONTROL` | companion → board | 24 |
| 16 | `VEHICLE_STATE` | board → companion | 96 |

An unknown id is counted and ignored — that is a companion newer than the
firmware, which is benign. A **known** id with the wrong length is counted
separately and is not benign: it means the two ends disagree about a format.

## 4. Byte order

**Little-endian, throughout.** Fixed-layout messages use naturally aligned C
structures. `CONTROL_TRAJ` is explicitly packed because its binary16 `dt` is
at an unaligned offset; use the documented offsets or the supplied codec
rather than `memcpy` of a compiler-defined structure.

> Note the contrast with [`can_packet.md`](can_packet.md), where every VESC
> field is **big**-endian. The two protocols in this tree disagree, and
> nothing warns you: a byte-swapped float is still a float, and a pose wrong
> by a byte swap still looks like a pose. `struct.pack('<')` for this link,
> `'>'` for CAN.

## 5. CRC16

CRC-16/CCITT-FALSE: polynomial `0x1021`, initial value `0xFFFF`, no input or
output reflection, no final XOR. Transmitted **low byte first**.

Check value: CRC of the ASCII string `123456789` is `0x29B1`.

The cal protocol uses the same polynomial and seed, so a host implementing
one has implemented both.

## 6. EXTERNAL_POSE (id 1, 48 bytes)

An absolute pose in the companion's map frame.

| Offset | Type | Field | Units |
|---|---|---|---|
| 0 | `uint64` | `timestamp_us` | µs, see below |
| 8 | `float32` | `x` | m, map frame |
| 12 | `float32` | `y` | m |
| 16 | `float32` | `yaw` | rad |
| 20 | `float32[6]` | `cov` | upper triangle, see below |
| 44 | `uint8` | `flags` | bit 0 = pose valid |
| 45 | `uint8` | `reset_counter` | source's frame-reset generation |
| 46 | `uint8[2]` | pad | send zero |

### Frame conventions

The map frame is **ENU** — x east, y north, z up — matching ROS REP-103.
`yaw` is measured **counter-clockwise from east** about the up axis, so a
vehicle pointing east reads 0 and one pointing north reads +π/2.

This is not PX4's NED/FRD. If your stack is ROS, the conventions already
agree; if it is PX4, roll and yaw run the opposite way.

**Only `x`, `y` and `yaw` are fused.** Height stays with the barometer and
there is no field for it.

### timestamp_us

Three distinct cases, and the difference matters:

| Value | Meaning |
|---|---|
| Non-zero, clocks synced | **UTC microseconds** since the Unix epoch. Converted to the board's monotonic clock on arrival. |
| Zero | "Not timestamped." The board stamps it on arrival. |
| Non-zero, clocks **not** synced | Cannot be converted, so it is treated as zero and counted as `rx_unsynced_stamp`. |

Sending zero is legitimate and costs the entire link latency as position
error — at 200 Hz over a 921600 link that is a few milliseconds, which at 1
m/s is a few millimetres, and at 10 m/s is a few centimetres. It is counted
separately from a working sync (`companion status` shows both) precisely so
that a timesync which has quietly stopped working is never mistaken for a
source that never timestamped at all.

Complete a timesync (section 8) before sending UTC timestamps. Sending them
beforehand is worse than sending zero: the board counts them and falls back
to arrival stamping anyway.

### Acceptance window

The estimator first applies `EK3_EXT_DLY_MS`, then checks the pose against its
UART receive time and the oldest state the delayed filter can still correct.
A physical pose cannot be newer than its receive time, and a pose older than
the already-processed state can no longer be fused at the right trajectory
point.

Errors within three `EK3_EXT_JIT_MS` standard deviations are clamped to the
corresponding boundary and counted by `extnav_time_clamped`. Larger errors are
refused and counted by `extnav_bad_time`. This absorbs ordinary 1-2 ms clock
jitter without allowing a broken timesync to move fusion arbitrarily through
the IMU history.

### Covariance

`cov` is the **upper triangle** of the 3×3 covariance for (x, y, yaw), in
row-major order:

| Index | Term |
|---|---|
| 0 | `xx` |
| 1 | `xy` |
| 2 | `x-yaw` |
| 3 | `yy` |
| 4 | `y-yaw` |
| 5 | `yaw-yaw` |

These are **variances**, not standard deviations. The estimator takes the
square root itself: report σ² = 0.04 for a 0.2 m one-sigma error.

**Indices 1, 2 and 4 are accepted and currently ignored.** Only the three
diagonal terms reach the filter. The off-diagonals are in the format so that
correlated errors can be used later without a wire-format change; today
sending them changes nothing.

A **zero** variance means "no estimate supplied" and the corresponding
parameter is used instead. Starting with zeros and tightening later needs no
format change.

The parameters are **floors, not defaults**. The value fused is
`max(reported σ, parameter)`, so an over-confident source cannot talk the
filter into trusting it more than the operator allowed:

| Parameter | Default | Applies to |
|---|---|---|
| `EK3_EXT_M_NSE` | 0.10 m | x and y |
| `EK3_EXT_YAW_NSE` | 0.05 rad | yaw |
| `EK3_EXT_I_GATE` | 5.0 | innovation gate, sigmas |
| `EK3_EXT_DLY_MS` | 0.0 ms | signed sample-time correction; positive means older |
| `EK3_EXT_JIT_MS` | 2.0 ms | timestamp uncertainty, one sigma |
| `EK3_EXT_POS_X/Y/Z` | 0.0 m | marker-centre to IMU/body-origin translation in marker axes |
| `EK3_EXT_ROLL/PITCH/YAW` | 0.0 deg | IMU/body orientation relative to marker axes |
| `EK3_EXT_TIMEOUT` | 1000 ms | silence before a dropout, and how long a bad ratio is tolerated before the source is condemned |

### flags

| Bit | Name | Meaning |
|---|---|---|
| 0 | `VALID` | The pose is usable. Clear it and the sample is ignored. |

Clearing bit 0 is the right way to signal "I am still here but currently
lost" — it keeps the stream alive without feeding the filter a guess.

### reset_counter

Increment this whenever the **source's** map frame jumps: a relocalisation, a
loop closure, a datum change. Anything downstream differentiating position
needs to know a discontinuity was intentional rather than seeing it as a
velocity spike.

### Gating, per pose

x and y are gated **jointly**, on the sum of both innovations against the
combined variance, the way ArduPilot's `posTestRatio` works — not
independently per axis. A pose 500 m wrong in x with y unchanged is rejected
as one bad pose. Gating the axes separately lets exactly that case through on
the good axis.

### Health, across poses

Separately from the per-pose gate, the estimator keeps a **low-passed
innovation ratio** for your source — ArduPilot keeps both for the same
reason: the gate handles one bad pose, this handles a source that is wrong as
a matter of course.

If that filtered ratio stays above 1 for longer than `EK3_EXT_TIMEOUT`, the
source is **condemned**:

- it stops being fused entirely, position *and* yaw
- `POSITION_HORIZ` is withdrawn from `solution_status`
- attitude, heading and height are unaffected

It re-earns trust by agreeing again — the filtered ratio has to fall back
below the threshold, which takes sustained agreement rather than one good
pose. `ekf3 status` shows the ratio, the health verdict and the fault count.

**This is the failure to design against.** Publishing a *stale* pose — a
position that stops updating while the vehicle keeps moving — is precisely
what condemns a source, and it is an easy thing to do accidentally when a
localisation pipeline stalls but the transport keeps running. Prefer one of:

- stop sending, and let `EK3_EXT_TIMEOUT` handle it as a dropout
- keep sending with **`VALID` cleared** in `flags`
- keep sending with an **honest, growing covariance**

All three are handled gracefully. A confident, frozen pose is not.

### Why a condemned source is not simply re-datumed

The estimator will re-datum to your source when it has been **silent** past
`EK3_EXT_TIMEOUT` and then returns — a localisation stack that restarted may
legitimately have a new origin.

It will **not** re-datum because your source disagrees. That distinction is
load-bearing: re-datuming on disagreement means snapping the filter's
position onto a source already known to be wrong, and the strapdown then has
nowhere to put the real motion except into accelerometer bias, which couples
into the gravity reference and takes attitude with it. That is a diverged
filter, reached from nothing worse than a frozen pose.

While your source and the IMU disagree, accelerometer bias learning is
**frozen** for exactly this reason. The IMU is treated as the reference: it
is redundant at board level and bounded by calibration, while an external
source can be arbitrarily wrong and still look plausible.

### Test vector

Encoder output for `timestamp_us = 1755000000000000`, `x = 1.5`,
`y = -2.25`, `yaw = 0.5`, `cov[0] = cov[3] = 0.04`, `cov[5] = 0.0025`,
`flags = 1`, `reset_counter = 3`:

```
fe 01 30 00 b0 94 c7 29 3c 06 00 00 00 c0 3f 00
00 10 c0 00 00 00 3f 0a d7 23 3d 00 00 00 00 00
00 00 00 0a d7 23 3d 00 00 00 00 0a d7 23 3b 01
03 00 00 92 47
```

53 bytes: `fe` sync, `01` id, `30` length (48), 48 payload bytes, then CRC
`0x4792` sent low byte first as `92 47`.

If your encoder reproduces this exactly, the framing, the little-endian
layout and the CRC are all correct.

## 7. VEHICLE_STATE (id 16, 96 bytes)

What the board sends back, at `EXT_TX_RATE` (default 200 Hz), driven by a
hardware timer.

| Offset | Type | Field | Frame | Units |
|---|---|---|---|---|
| 0 | `uint64` | `timestamp_us` | — | UTC µs once synced, else monotonic |
| 8 | `float32[3]` | `position` | local ENU | m |
| 20 | `float32[4]` | `quaternion` | body→ENU | w, x, y, z |
| 36 | `float32[3]` | `velocity` | **body FLU** | m/s |
| 48 | `float32[3]` | `angular_velocity` | body FLU | rad/s |
| 60 | `float32` | `side_slip_rad` | body FLU | rad, NaN below 0.3 m/s |
| 64 | `float32[3]` | `accel` | body FLU | m/s², gravity removed |
| 76 | `float32` | `wheel_torque_nm` | — | Nm |
| 80 | `float32` | `steering_angle` | — | selected steering feedback |
| 84 | `float32` | `motor_speed_ms` | — | m/s |
| 88 | `uint8` | `solution_status` | — | bits below |
| 89 | `uint8` | `reset_counter` | — | estimator reset generation |
| 90 | `uint8` | `source_valid` | — | which inputs were fresh |
| 91 | `uint8` | pad | — | zero |
| 92 | `uint32` | `rc_status` | — | packed RC/control state |

### rc_status

The final four bytes carry raw operator inputs and the state actually selected
by the safety router without increasing the 96-byte packet size:

| Bits | Meaning |
|---|---|
| 0..11 | steering PWM in µs, from `RC_MAP_STEERING` |
| 12..23 | throttle/control PWM in µs, from `RC_MAP_THROTTLE` |
| 24 | armed (`1`) / disarmed (`0`) |
| 25 | AUTO (`1`) / RC (`0`) source selected by the router |
| 26 | physical RC channel 6 trigger high (`1`) / low (`0`) |
| 27 | current/torque control (`1`) / duty-cycle control (`0`) |
| 28..31 | reserved, zero |

Each PWM occupies 12 bits (`value & 0x0fff`). When RC is unavailable both PWM
values are zero and `COMP_SRC_RC` is clear. The CH6 bit is its live input level
using `RC_SW_HIGH`; the control-mode bit is the router's latched selection.
They can therefore differ after a momentary CH6 trigger is released.

### The frames are not all the same

This follows ROS `nav_msgs/Odometry`: **pose in the world frame, twist in the
body frame.**

- `position` and `quaternion` are in local **ENU** — x east, y north, z up.
- `velocity`, `angular_velocity` and `accel` are in **body FLU** — x forward,
  y left, z up.

The estimator works in ENU velocity internally; the board converts before
sending. Angular velocity is body-frame by construction because it is the
gyro. Getting this backwards is a quiet failure: a velocity rotated by the
quaternion instead of its transpose is still a plausible velocity, and at
zero yaw the two are identical.

### side_slip_rad

The angle between where the vehicle is **going** and where it is **pointing**,
computed from the body velocity already in this packet as
`atan2(velocity[1], velocity[0])` — body x is the heading and body y is left,
so the comparison needs no separate yaw term.

Horizontal components only; a climb is not side slip. Left-positive, matching
body y and ISO 8855, so sliding to the left of the nose gives a positive
angle.

**NaN below 0.3 m/s.** At rest the direction of travel is noise, and
reporting whatever `atan2` makes of two near-zero numbers would be worse than
saying nothing. NaN is also what distinguishes "not computed" from a genuine
zero, which means "travelling straight ahead" — test with `isnan()`.

### accel

Calibrated specific force with both the EKF-tracked accelerometer bias and
attitude-projected gravity removed. It therefore reads **zero-mean at rest**
rather than retaining either 9.8 m/s² or a residual bias the EKF has already
learned. Sensor bandwidth still determines its instantaneous noise.

Gravity removal needs an attitude, so if the estimator is not running this
field stays zero and `COMP_SRC_ACCEL` is clear — rather than reporting the
raw 9.8 m/s² as vehicle acceleration.

### The VESC-derived channels

| Field | Source | Scalar | Default |
|---|---|---|---|
| `wheel_torque_nm` | `vesc_status.current_a` | `VESC_TORQUE_K` | 1.0 |
| `steering_angle`, `STEER_FB_SRC=0` | `vesc_status.adc_volts` | `VESC_STEER_K` | 1.0 |
| `steering_angle`, `STEER_FB_SRC=1` | last servo pulse sent to VESC | 1000–2000 us → -0.5–+0.5 | — |
| `motor_speed_ms` | tachometer rate | `VESC_SPEED_K` | 1.0 |
| (filter cutoff) | — | `VESC_SPD_LPF` | 100 Hz |
| (expected telemetry rate) | — | `VESC_TLM_HZ` | 400 Hz |

`VESC_STEER_OFS` accepts -300 to +300 us and is added to the mapped servo pulse after
`VESC_STEER_MIN/TRIM/MAX`. The transmitted result is bounded to 900–2100 us,
so command-derived feedback includes the applied offset. A valid, fresh RC
channel 7 adds a live trim on top in both RC and Auto modes: 1000–2000 us maps
linearly to -100–+100 us. If RC is stale, in failsafe, or CH7 is unavailable,
its contribution is zero.

All three scalars default to **1.0**, so until the vehicle is characterised
these carry raw amps, raw volts and raw counts per second. That is
deliberate: a guessed gear ratio is worse than an honest raw number, because
it looks calibrated.

#### Calibrating `VESC_SPEED_K`

`tools/wheel_cal.py <port>` estimates it from a drive: it collects
`VEHICLE_STATE` frames for 60 s and fits the estimator's forward speed
against the tachometer rate, through the origin, because zero counts must
mean zero velocity.

```
param set VESC_SPEED_K 1.0     # else the result is a correction, not the value
param save                     # then reboot
python3 tools/wheel_cal.py /dev/pixhawk_6c --seconds 60
```

Drive straight, over a range of speeds. Samples are discarded while turning
(`|yaw rate| > 0.35 rad/s` — a turn scrubs the wheels and a differential
drives them at rates the body does not travel at), below 0.3 m/s, and any
time the estimator lacks a horizontal position solution: it cannot be a speed
reference when it does not know its own speed. The fit is refused outright
below 200 samples or a 1 m/s speed span, since a dataset taken at one speed
yields a confident number describing only that speed.

The estimator's velocity is a legitimate reference here only because it is
not derived from the wheels. The zero-velocity update is deliberately
K-independent — it asserts zero, never a speed — so the fit is not circular.
Were wheel speed ever fused as a velocity measurement, this tool would have
to take its reference from the external fix directly.

`motor_speed_ms` is the time derivative of the tachometer, and it is computed
**in the VESC daemon, not here.** That placement is the point:

`STATUS_5` arrives at **400 Hz** and this downlink runs at 200, and
`vesc_status` is advertised without a queue — so a subscriber reading at
200 Hz sees only the newest message and **every other sample is already gone**
before it runs. Filtering on the consumer side cannot anti-alias a stream it
never received. The daemon sees every decoded frame, so the derivative and
the filter both live there and the topic carries the finished `speed_cps`.

The derivative comes first, the filter second: filtering the accumulated
count would smooth a *position*.

**The filter is the anti-alias filter for this downlink.** Two cascaded
one-pole sections, −40 dB/decade, cutoff `VESC_SPD_LPF` default **100 Hz** —
the 200 Hz downlink's Nyquist. Without it, everything between 100 and 200 Hz
in the 400 Hz stream folds down into the band you care about.

The interval used is an **exponential moving average**, seeded from
`VESC_TLM_HZ` (default 400) and following the real stream from there. This
matters because the count is an integer: at 400 Hz a single interval carries
only a few counts, and dividing a constant delta by a jittering timestamp
manufactures a speed ripple that is not there. An interval outside 0.25× to
4× the nominal is refused rather than averaged in, so one late frame cannot
drag the timebase.

Timestamps come from the FDCAN interrupt (TIM5, microseconds), not from
`CLOCK_MONOTONIC` — which advances in 1 ms steps here, and would read a
2.5 ms interval as 2 or 3 ms, a 20% error on every sample.

The requested cutoff is clamped to 40% of the measured rate. A cutoff at or
above Nyquist is not a filter: it does not go unstable, it simply stops
filtering.

The tachometer wraps, and that is fine — a finite difference across the wrap
is the correct small step, exactly as it would be for an angle, provided the
subtraction is unsigned. After a gap of more than 500 ms the filter restarts
rather than emitting one enormous value.

### angular_velocity and accel: filtered, and deliberately not the EKF's

The board keeps **two** IMU streams, the way ArduPilot separates
`INS_GYRO_FILTER` from what the estimator consumes:

| Stream | Filtering | Consumer |
|---|---|---|
| `sensor_gyro` / `sensor_accel` (raw driver topics) | calibration only — offset, scale, axis map | `imu_delta` → `vehicle_imu` → EKF3 |
| `vehicle_gyro` / `vehicle_accel` (corrected topics) | `SENS_GYR_LPF`, `SENS_ACC_LPF`, optional notch | this packet's twist and accel |

`angular_velocity` and `accel` in `VEHICLE_STATE` come from the **filtered**
side. The estimator never sees those filters — for the accelerometer as much
as the gyro. `imu_delta` subscribes to `sensor_accel` and `sensor_gyro`
directly, applies calibration only, and integrates delta angles and delta
velocities into `vehicle_imu`, whose own contract records that it "bypasses
every configurable software LPF/notch".

This matches ArduPilot, verified against its source rather than assumed.
`AP_NavEKF3_Measurements.cpp` reads only
`ins.get_delta_velocity()` / `ins.get_delta_angle()`, and in
`AP_InertialSensor_Backend.cpp` those accumulators are fed the **raw**
corrected sample — `_delta_velocity_acc[instance] += accel * dt` — while
`_accel_filtered` and `_gyro_filtered` are computed separately and never
reach them. So `INS_ACCEL_FILTER` and `INS_GYRO_FILTER` shape `get_accel()`
and `get_gyro()` for the control loops only, exactly as `SENS_ACC_LPF` and
`SENS_GYR_LPF` do here.

That is not an accident of wiring. A low-pass in the estimator's path adds
phase lag to the very signal the attitude solution integrates, and the
filter's delayed-fusion horizon already handles the problem a filter would be
there to solve. On the control and telemetry side the trade runs the other
way: lag costs less than noise.

**Defaults:** both `SENS_GYR_LPF` and `SENS_ACC_LPF` are **100 Hz**, chosen
for control bandwidth.

Note what that costs here. This downlink samples the 2 kHz corrected topics
at 200 Hz, and two poles at 100 Hz are only 3 dB down at the 100 Hz Nyquist —
so `angular_velocity` and `accel` carry some folded content. It is a
deliberate bandwidth-versus-aliasing trade, not an oversight. If those
channels look noisy and you do not need the bandwidth, drop both cutoffs to
around 30 Hz, which puts 100 Hz roughly 21 dB down.

`SENS_GYR_NF_FRQ` adds a notch for a known vibration line, off by default.

**Changing these does not change the estimator.** If you filter the twist
hard for a rate loop, EKF3 is unaffected.

### source_valid

| Bit | Name | Meaning |
|---|---|---|
| 0 | `COMP_SRC_ESTIMATOR` | pose, velocity and status are real |
| 1 | `COMP_SRC_GYRO` | `angular_velocity` is real |
| 2 | `COMP_SRC_ACCEL` | `accel` is real |
| 3 | `COMP_SRC_VESC` | VESC current and motor speed are real |
| 4 | `COMP_SRC_RC` | packed steering/throttle PWM values are fresh |
| 5 | `COMP_SRC_STEERING` | selected steering feedback is valid |

**Check this before trusting a zero.** A stopped VESC and a stationary
vehicle both report zero wheel torque, and only one of them means the vehicle
is under control. When `COMP_SRC_ESTIMATOR` is clear the quaternion is left
all-zero, which is not a rotation at all — an identity quaternion would have
been indistinguishable from a real level attitude.

### timestamp_us

The **IMU sample time** of the solution, not the time the message was sent.
So the age you measure on arrival legitimately includes the estimator's own
output latency; it is not all transport.

Guaranteed never to be in the future — clamped to the board's own UTC before
sending, and the PPS discipline in section 8 keeps that clock aligned to
yours.

### solution_status

| Bit | Name |
|---|---|
| 0 | `ATTITUDE_VALID` — roll and pitch |
| 1 | `YAW_RELATIVE` — heading, arbitrary datum |
| 2 | `YAW_ABSOLUTE` — heading against north |
| 3 | `VELOCITY_HORIZ` |
| 4 | `VELOCITY_VERT` |
| 5 | `POSITION_HORIZ` |
| 6 | `POSITION_VERT` |

**`POSITION_HORIZ` is the bit autonomy should gate on.** It is a claim that
external navigation is *actually correcting*, not that it is selected, and it
is withdrawn when the source is stale, silent, or condemned for disagreeing
with the IMU. Losing it means the horizontal position estimate is no longer
being held by anything and will drift on the IMU alone — a vehicle driving
autonomously should stop.

`ATTITUDE_VALID` surviving while `POSITION_HORIZ` drops is the expected
shape, not a contradiction: roll and pitch come from gravity and remain
trustworthy when a companion does not.

### reset_counter

Incremented when the **estimator's** datum moves, which happens when your
source returns after a dropout with a different origin. Position jumps
discontinuously when it happens.

Note it no longer moves on a rejection run — a disagreeing source is
condemned rather than adopted, so there is no jump to announce.

### Decoding

```python
import struct, math

VEHICLE_STATE = struct.Struct("<Q3f4f3f3ff3ffffBBB5x")
assert VEHICLE_STATE.size == 96

f = VEHICLE_STATE.unpack(payload)
state = {
    "timestamp_us": f[0],
    "position": f[1:4],            # local ENU
    "quaternion": f[4:8],          # w x y z
    "velocity": f[8:11],           # BODY frame
    "angular_velocity": f[11:14],  # body
    "side_slip_rad": f[14],        # NaN until estimated
    "accel": f[15:18],             # body, gravity removed
    "wheel_torque_nm": f[18],
    "steering_angle": f[19],
    "motor_speed_ms": f[20],
    "solution_status": f[21],
    "reset_counter": f[22],
    "source_valid": f[23],
    "rc_status": f[24],
}
```

`tools/comp_link.py` carries this as `decode_vehicle_state()`, and
`tests/comp_proto_cross_test.py` pins it against the C encoder byte for byte.

## 8. CONTROL_TRAJ (id 2, variable length)

A finite-horizon plan computed against a known pose. Receiving this message
publishes the `control_trajectory` uORB topic; it does **not** directly move
the vehicle. A trajectory follower can later select the time-appropriate
element and publish an immediate `control_cmd` through the safety router.

The payload is a 20-byte header, followed by exactly `horizon` pose pairs and
then exactly `horizon` control pairs:

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | `uint64` | `timestamp_us` | UTC µs, sender's current time |
| 8 | `uint64` | `solution_time_us` | UTC µs of the pose used to solve the plan |
| 16 | `uint8` | `horizon` | number of pose and control entries, 1..14 |
| 17 | `float16` | `dt` | seconds between trajectory entries |
| 19 | `uint8` | `control_method` | 0 = duty, 1 = current |
| 20 | `float32[horizon][2]` | `poses` | `(x, y)` in local ENU, metres |
| `20 + 8*horizon` | `float32[horizon][2]` | `controls` | `(steering, duty_or_amps)` |

`dt` is IEEE-754 binary16, little-endian. Every other floating-point field is
IEEE-754 binary32. Pose and control arrays have the same count: horizon `N`
means `N` poses and `N` controls, not `N+1` poses.

The maximum horizon is 14 because `20 + 16*14 = 244`, the protocol payload
ceiling. Both timestamps are required and converted back to TIM5 with the
affine clock relation. A plan is rejected when its current timestamp is stale
or far in the future, its solution time is in the future relative to current
time, `dt` is invalid, or any pose/control is non-finite or out of range.

`control_method` uses the board actuator enum:

- `0`: steering −1..+1 and duty −1..+1.
- `1`: steering −1..+1 and current −50..+50 A.

```python
frame = comp_link.encode_control_trajectory(
    timestamp_us=utc_now_us,
    solution_time_us=localization_pose_utc_us,
    dt=0.05,
    poses=[(1.0, 2.0), (1.1, 2.0)],
    controls=[(0.1, 0.20), (0.08, 0.18)],
    control_method=comp_link.THROTTLE_DUTY,
)
```

## 9. DIRECT_CONTROL (id 7, 24 bytes)

An immediate actuator command. This is the higher-priority half of the
autonomous input; `CONTROL_TRAJ` carries a complete non-actuating plan.

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | `uint64` | `timestamp_us` | UTC microseconds, when the companion sent it |
| 8 | `float32` | `steering` | −1.0 … +1.0, left positive |
| 12 | `float32` | `throttle` | duty −1.0 … +1.0, or amps −50.0 … +50.0 |
| 16 | `uint8` | `throttle_type` | 0 = duty, 1 = current |
| 17 | `uint8[7]` | `pad` | zero |

```c
struct comp_direct_control_s
{
  uint64_t timestamp_us;
  float    steering;
  float    throttle;
  uint8_t  throttle_type;
  uint8_t  pad[7];
};
```

### throttle_type is the board's enum

`0 = duty, 1 = current` — the same numbering as `ACTUATOR_MODE_*` in
`uorb_msgs.h`, so the byte reaches the control router unmapped.
`companion.c` carries a `static_assert` on that agreement. Send these the
wrong way round and "20 amps" arrives as "duty 20", which clamps to full
throttle; the numbering is shared precisely so there is no translation step
to get backwards.

### Range is rejected, not clamped

The limits in the table above are what the **format** can mean. A value
outside them is dropped and counted in `rx_direct_invalid`, because it says
the sender is wrong about the units or the mode, and clamping would turn that
into a command that looks deliberate. `NaN` and infinity are rejected the same
way.

The vehicle's real ceilings are `VESC_DUTY_MAX` (0.30 by default) and
`VESC_CUR_MAX` (20 A), applied by the control router afterwards. A legal 50 A
command on the wire still becomes 20 A at the motor.

### The timestamp is required

Unlike `EXTERNAL_POSE`, a zero timestamp is **not** accepted, and neither is
any timestamp before the clocks have been related (section 10). A pose with no
usable stamp costs accuracy; a command with no usable stamp cannot be aged,
and an actuator command of unknown age is the one thing this link must not act
on. Both cases are counted in `rx_direct_stale`.

### Two freshness checks, one budget

`AUTO_CMD_TO_MS` is read by both ends of the same journey, and each side
measures in its own clock:

| Check | Where | Clock | Catches |
|---|---|---|---|
| arrival age | `companion.c` | TIM5 | slow link, wrong UTC offset |
| staleness | `control_router` | `CLOCK_MONOTONIC` | companion stopped sending |

The published `control_cmd.timestamp` is deliberately **zero**, so the router
stamps it on arrival in its own clock — the same thing `vesc set` does. TIM5
and `CLOCK_MONOTONIC` are independent counters on this board, so a TIM5 value
placed in that field would read as permanently fresh or permanently stale
depending on which way the two had drifted.

Set the budget with `param set AUTO_CMD_TO_MS 100`, `param save`, and reboot.

### Being published is not being obeyed

A command that passes every check above is published to `control_cmd`, and
the control router acts on it only when the RC source switch selects AUTO and
its own arm sequence has completed. When commands stop, the router holds
neutral and reports `AUTO_STALE` — it does **not** hand control back to the
sticks, because the sticks may not be centred. Taking over is a deliberate
move of the source switch.

### Sending one

```python
import time, comp_link

link.send(comp_link.encode_direct_control(
    steering=-0.25,                      # left positive
    throttle=0.15,                       # duty ratio
    throttle_type=comp_link.THROTTLE_DUTY,
    timestamp_us=int(time.time() * 1e6)))
```

Repeat faster than `AUTO_CMD_TO_MS`; there is no repeat-last behaviour by
design.

`companion status` reports the outcome:

```
  direct     accepted 4213  stale 0  invalid 0
             last age 3.2 ms of 100 ms budget (AUTO_CMD_TO_MS)
```

## 10. Clock synchronisation

Timestamps are only meaningful once the clocks are related. Two mechanisms
work together.

### Timesync burst

A bracketed exchange, all initiated by the companion:

1. `TIMESYNC_START` — `{ uint32 count, uint32 pad }`, announcing how many
   exchanges follow.
2. `count` × `TIMESYNC_REQ` — `{ uint64 host_tx_us }`, the companion's UTC
   when it asked. The board answers each with `TIMESYNC_REP`:
   `{ uint64 host_tx_us, uint64 board_rx_us, uint64 board_tx_us }`.
3. `TIMESYNC_END` — `{ int64 utc_offset_us, uint32 trip_us, uint32 samples }`,
   telling the board the observed `UTC - TIM5` offset. The first completed
   sync establishes absolute UTC. Later observations estimate the scale in
   `corrected_UTC = a * TIM5 + b`; phase error is removed by a bounded rate
   slew, so corrected UTC never steps when a periodic sync completes.

With four timestamps per exchange:

```
offset     = ((board_rx - host_tx) + (board_tx - host_rx)) / 2
round_trip = (host_rx - host_tx) - (board_tx - board_rx)
```

Keep the exchange with the **smallest** round trip. The offset is only as
good as the path is symmetric, and the least-delayed exchange is the least
asymmetric one.

`board_tx_us` is what makes this work — without it the board's own processing
delay is indistinguishable from wire latency and lands entirely in the offset.

The bracket does not make the offset more accurate; the exchanges do that.
What it buys is that the *board* knows what the companion concluded, so
`companion status` can show the agreed offset instead of having no idea
whether its peer thinks the clocks are aligned.

**The board never adopts UTC as its own timebase.** Every internal timestamp
stays in TIM5, because monotonic is the only clock that cannot step. UTC is a
wire format: converted going out, converted back through the exact inverse,
and never seen by the estimator. `CLOCK_REALTIME`/RTC is set only by the first
authoritative sync and then free-runs; it is not used in either conversion.

### PPS

Drive **TELEM2 CTS** (PC9) with a 3.3 V rising edge on the UTC second. The
board captures it on TIM3 at 1 MHz and steers the affine UTC rate so the
established pulse phase remains fixed without stepping corrected UTC.

Because the pulse comes *from* the companion, its edge is that machine's own
second boundary — which makes this a direct microsecond-resolution
measurement against the very clock the companion will judge the board's
timestamps by. A timesync round trip only resolves milliseconds.

PPS corrects the **phase within a second**; it cannot say which second it is.
A timesync burst, or the RTC, must establish the absolute time first —
`companion status` reports `not disciplining` until both hold.

`pps status` shows lock state, period and edge counts.

## 11. Minimal sender

```python
import struct, serial

def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
                  else (crc << 1) & 0xFFFF
    return crc

def external_pose(x, y, yaw, timestamp_us=0, sigma_xy=0.2, sigma_yaw=0.05,
                  reset_counter=0):
    cov = [sigma_xy ** 2, 0.0, 0.0, sigma_xy ** 2, 0.0, sigma_yaw ** 2]
    payload = struct.pack('<Qfff6fBBxx', timestamp_us, x, y, yaw, *cov,
                          0x01, reset_counter)
    assert len(payload) == 48
    body = bytes([1, len(payload)]) + payload
    return bytes([0xFE]) + body + struct.pack('<H', crc16(body))

port = serial.Serial('/dev/ttyTHS1', 921600)
port.write(external_pose(1.5, -2.25, 0.5, 1755000000000000, reset_counter=3))
```

Check that against the test vector in section 6 before trusting it against
the board.

## 12. Diagnostics

`companion status` on the NSH console reports, among others:

| Field | Meaning |
|---|---|
| `rx_pose` | `EXTERNAL_POSE` frames routed and published |
| `crc_errors` | framing or wiring problem |
| `bad_length` | a known id at the wrong size — a format disagreement |
| `unknown_id` | benign; a companion newer than the firmware |
| `rx_unsynced_stamp` | UTC arrived before a sync could use it |
| `pps` | lock state, corrections applied, last residual |
| `tx_future_clamped` | a stamp that would have led the clock |

`ekf3 status` reports the consumption side:

| Field | Meaning |
|---|---|
| `extnav_in` | poses queued for fusion |
| `extnav_bad_time` | refused on the timestamp window |
| `extnav_untimed` | arrival-stamped; the source sent zero |
| `accept` / `reject` | per-pose gate outcomes, and the current reject run |
| `redatum` | times a datum was granted — only ever after silence |
| `health` | `OK`, or `UNHEALTHY - NOT FUSED` |
| `test_ratio` | the low-passed innovation ratio against the IMU |
| `faults` | times the source has been condemned |
| `accel-bias` | `learning` or `FROZEN` |

**Reading a bad run.** `test_ratio` climbing above 1 while `reject` grows
means your poses and the IMU disagree. If `accel-bias` shows `FROZEN` the
guard has engaged and the filter is protecting itself; if `health` then goes
`UNHEALTHY`, the source has been dropped and `POSITION_HORIZ` is gone. A
`redatum` count that is *not* increasing during all of this is the design
working — a disagreeing source is never adopted.

## Related

- [`can_packet.md`](can_packet.md) — the VESC CAN protocol. **Big-endian**,
  unlike this one.
- [`parameters-ekf.md`](parameters-ekf.md) — `EK3_SRC*` source selection, and
  what is actually fused today.
