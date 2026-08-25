# Companion Link and External Navigation — Design

Date: 2026-08-25
Branch: `step5-estimator-imu-pipeline`
Status: approved, ready for implementation planning

## Goal

Give the board a general packet link to the companion computer, and use its
first message to bound horizontal position drift.

Two things, deliberately separated:

- **A communication handler** that owns one serial port, parses framed
  packets, and routes them to uORB topics by message ID. It knows nothing
  about navigation.
- **External position fusion** in EKF3, consuming the `external_pose` topic
  the handler publishes, with the same robustness ArduPilot's EKF3 applies to
  external navigation data.

## Why the split

The link will carry more than pose. Control trajectory is already known to be
coming, and a daemon written around navigation would have to be reshaped for
each new message. Routing by ID against a table makes a new message a table
row plus a topic, with no new transport code and no change to any existing
consumer.

It also keeps EKF3's dependencies honest: the estimator subscribes to
`external_pose` exactly as it subscribes to `vehicle_mag`, and never learns
that a serial port exists.

## Starting state

`EK3_SRC1_POSXY` is `0` (none) and horizontal position integrates from
accelerometers with nothing to bound it. `EK3_SRC*` can name
`EXTERNAL_NAV = 6`, but no topic is declared and nothing publishes one.

The delayed fusion horizon, the measurement queues, `measurement_update_1d`,
and the yaw observation inside `ekf_core_fuse_mag` all already exist. This
work reuses every one of them; the only change to existing fusion code is
extracting the yaw update so external yaw and magnetic yaw go through one
path rather than two.

`SER_FUNC_CAL = 5` is the precedent for the port: it deliberately starts
nothing, so no shell sits in `read()` stealing the session's input. The
companion port needs the same treatment, which means extending
`SER_*_FUNC`'s range from 5 to 6.

## Non-goals

- **Control trajectory.** Deferred until the control side knows its own
  shape. ID 2 is reserved for it and the routing table is built to take it.
- **Optical flow fusion.** A separate subsystem, designed separately.
  `EK3_SRC1_VELXY = 5` names optical flow and stays inert after this work:
  external position bounds position, not velocity.
- **Timesync.** Implemented on the operator's side. IDs 3-4 are reserved and
  the board validates what arrives; it does not itself synchronise clocks.
- **Fusing measurement cross-covariance.** See "Covariance" below.
- **Velocity or height from the external source.** It reports x, y and yaw.
  Height stays with the barometer, which already owns `POSITION_VERT`.

## The companion link

### Structure

| File | Responsibility |
|---|---|
| `apps/companion/comp_proto.c/h` | Framing, CRC, ID dispatch. No I/O, no uORB, host-testable. |
| `apps/companion/companion.c/h` | Daemon: bytes in, route and publish; subscribe and transmit. |
| `apps/companion/companion_main.c` | `companion start \| stop \| status` |

The codec being free of I/O is what lets the packet format be tested while it
is still being iterated on, which is the stated plan for it.

### Framing

```
  0xFE          sync
  id            message id
  len           payload length
  payload[len]
  crc16         CCITT-FALSE over id, len and payload
```

Self-resynchronising: anything that is not the sync byte is discarded, so a
lost byte costs one message rather than wedging the stream. This is the same
shape `cal` already uses, and for the same reason.

### Message identifiers

```
   1  EXTERNAL_POSE       in    -> publishes external_pose
   2  CONTROL_TRAJECTORY  in       reserved
   3  TIMESYNC_REQUEST    in       reserved
   4  TIMESYNC_REPLY      out      reserved
  16  ESTIMATOR_POSE      out   <- subscribes estimator_state
```

Inbound identifiers are low and outbound are high. A message sent in the
wrong direction then fails to route at all, rather than half-working.

### ESTIMATOR_POSE, board to companion

```
  uint64  timestamp_us      board monotonic
  float32 position[3]       m, local NWU
  float32 quaternion[4]     w x y z, body to nav
  float32 velocity[3]       m/s, local NWU
  uint8   solution_status   ESTIMATOR_* validity bits
  uint8   reset_counter     estimator reset generation
  uint8   pad[2]
```

52 bytes. A quaternion rather than Euler angles: no singularity, and no
convention to argue about across a language boundary.

`reset_counter` matters more than it looks. The datum reset below moves
position discontinuously, and anything on the companion side differentiating
position needs to know that happened rather than seeing a spike.

Sent at `EXT_TX_RATE`. At 50 Hz this is 2.6 kB/s; at 400 Hz it is 208 kbps,
so rates above roughly 100 Hz need 460800 baud or better.

### EXTERNAL_POSE, companion to board

```
  uint64  timestamp_us      board timebase, per the operator's timesync
  float32 x                 m, map frame
  float32 y                 m, map frame
  float32 yaw               rad, map frame
  float32 cov[6]            xx, xy, xyaw, yy, yyaw, yawyaw
  uint8   flags             bit 0 pose valid
  uint8   reset_counter     source's own frame-reset generation
  uint8   pad[2]
```

48 bytes.

### Covariance

Three by three upper triangle for x, y and yaw - the states actually fused.
MAVLink's `VISION_POSITION_ESTIMATE` carries a 21-element 6x6 triangle, of
which fifteen entries would be decorative here.

**The diagonal is used; the off-diagonals are carried and not yet used.** That
is not a shortcut taken for convenience: ArduPilot's `FuseVelPosNED` fuses
North and East as sequential SCALAR updates and ignores measurement
cross-correlation in exactly the same way. A two-dimensional update with a
full R matrix is possible later, and is not what matching ArduPilot means.

A zero variance means "no estimate supplied, use the parameter", so the
companion side can start by sending zeros and tighten later without a format
change.

## Fusion

### Datum

The first accepted pose after alignment **sets** `position[0]`,
`position[1]` and yaw rather than being fused into them. The EKF's local
frame becomes the map frame from that moment, so every consumer reads map
coordinates with no transform to maintain anywhere.

Fusing instead of setting does not work: the map origin may be tens of metres
from where the filter aligned, so the first innovation would be enormous, the
gate would reject it, and it would go on rejecting every subsequent one
forever.

Position Z is untouched - the barometer owns it. Roll and pitch are
untouched.

The position covariance is reset alongside, or the filter keeps a confidence
that no longer matches where it now thinks it is. This needs a position
analogue of the existing `covariance_reset_attitude`.

### Updates

`x` and `y` as two sequential scalar updates through the existing
`measurement_update_1d`, observing state indices 6 and 7.

Yaw through a **shared `fuse_yaw()` extracted from `ekf_core_fuse_mag`**.
That extraction is the only modification to existing fusion code, and its
purpose is that external yaw and magnetic yaw cannot develop different ideas
about what a yaw update is.

Eligibility follows the source parameters: `EK3_SRCn_POSXY == 6` enables
position, `EK3_SRCn_YAW == 6` enables yaw. Selecting external yaw therefore
deselects the compass automatically, because a source set names one yaw
source.

### Robustness

Mirroring `AP_NavEKF3_PosVelFusion` rather than gesturing at it:

- **The parameter is a floor, not a default.** `noise = max(reported_sigma,
  EK3_EXT_M_NSE)`, which is what ArduPilot does with `posErr`. A source
  claiming millimetre accuracy cannot talk the filter into trusting it more
  than the operator configured.
- **Fusion at the sample's own timestamp**, through the existing horizon.
- **Innovation gate in sigma, per axis**, counted separately.
- **Timeout resets position to the measurement** rather than diverging -
  ArduPilot's `ResetPositionNE()`. Mechanically this is the datum reset run
  again, and it is what prevents a gate deadlock after a long dropout: the
  filter has drifted, every incoming pose now looks impossible, and only a
  reset recovers.
- **The source's reset counter is honoured.** If the companion says it jumped
  frames, re-datum rather than gating forever. Counted, so a source that
  keeps jumping is visible rather than silently re-anchoring.
- **Timestamp sanity.** Refused and counted when it is in the future by more
  than one transmit interval, or older than `EKF3_EXT_MAX_AGE_US`, fixed at
  500 ms to match the bounds the barometer and magnetometer queues already
  use. This is the canary for a timesync that is not working: without it a
  wrong clock corrupts position silently, which is the worst failure
  available here.

  Note the two bounds do different jobs. The 500 ms age bound is the
  measurement queue's - past it the filter has propagated somewhere else and
  the correction would land on the wrong part of the trajectory. The
  future-time bound has no such excuse: a timestamp ahead of the board's own
  clock means the timesync is wrong, full stop.
- **Health run.** A sustained rejection run drops `POSITION_HORIZ` and the
  filter continues on inertial rather than holding a stale correction.

## Parameters

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `SER_*_FUNC` | int32 | - | Range extended 0-5 to 0-6; 6 is the companion port |
| `EXT_TX_RATE` | int32 | 50 | Outgoing pose rate, Hz (1-400) |
| `EK3_EXT_M_NSE` | float | 0.10 | Position measurement noise FLOOR, m |
| `EK3_EXT_I_GATE` | float | 5.0 | Position innovation gate, sigma |
| `EK3_EXT_YAW_NSE` | float | 0.05 | Yaw measurement noise floor, rad |
| `EK3_EXT_TIMEOUT` | int32 | 1000 | Dropout before position validity is dropped, ms |

Baud reuses the existing `SER_*_BAUD`. Gates are plain sigma; ArduPilot
stores its as integer sigma times 100 for historical reasons this codebase
has no reason to copy.

All names fit `PARAM_NAME_MAX` (16).

## Failure handling

| Condition | Response |
|---|---|
| CRC mismatch | Count, discard one byte, resynchronise |
| Unknown message id | Count and skip; not an error, it is a newer companion |
| Payload length wrong for the id | Count and skip - a format mismatch, not corruption |
| `flags` not valid | Skip silently; the source is saying so deliberately |
| Timestamp ahead of board time, or older than 500 ms | Refuse and count |
| Innovation gated | Count; sustained run drops `POSITION_HORIZ` |
| Timeout | Drop `POSITION_HORIZ` and external yaw, continue on inertial |
| Source reset counter changed | Re-datum, counted |

Distinguishing "unknown id" from "wrong length for a known id" matters: the
first is a companion newer than the firmware and is benign, the second is the
two ends disagreeing about a format and is not.

## Observability

`companion status` reports bytes in and out, frames accepted, CRC failures,
unknown ids, per-id receive counts and the age of the last packet.

`ekf3 status` gains an external-navigation line: datum set or not, last
innovation and NIS per axis, accept and reject counts, the rejection run, the
age of the last accepted pose, and the noise actually used after the floor was
applied - so a source under-reporting its error is visible.

## Testing

`tests/comp_proto_test.c` with `tools/test-comp-proto.sh`, following the
`mavlink_codec` pattern:

- every field round-trips through encode and decode
- a corrupted byte fails CRC and is rejected
- a truncated frame is held, not misparsed
- garbage before a valid frame resynchronises to it
- **the parser is fed one byte at a time** and still produces the same
  result. A parser that only works on whole frames passes every other test
  and fails the first time a UART splits one, which it will.
- a known id with the wrong length is rejected distinctly from an unknown id

`tests/ekf_core_test.c` extensions:

- the datum reset sets position and yaw and leaves Z, roll and pitch alone
- x and y fusion reduce their own variance and are gated independently
- yaw through the shared path matches what the magnetometer path produced
- a reported variance below the floor is raised to it
- a timestamp older than 500 ms is refused, and so is one ahead of board time
- a dropout past `EK3_EXT_TIMEOUT` followed by a distant pose re-datums
  instead of gating forever

## Known limitations

**Horizontal velocity remains unaided.** External position bounds position,
not velocity, which still integrates from accelerometers between updates.
Fine at 50 Hz; visibly worse if the companion is slow or intermittent.
Optical flow is what fixes that, and is a separate subsystem.

**The datum reset is a discontinuity.** Position jumps from the origin to the
map fix the moment the first pose arrives. `reset_counter` in the outgoing
pose exists so the companion can see it happen; anything differentiating
position on that side must watch it.

**A wrong timesync degrades quietly within its sanity bounds.** The
timestamp check catches a clock that is grossly wrong. A clock off by 50 ms
passes every check and shows up only as position error proportional to speed.
