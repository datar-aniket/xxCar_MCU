# VESC CAN Transmit — Design

**Date:** 2026-08-25
**Status:** approved
**Predecessor:** [2026-08-25-vesc-can-link-design.md](2026-08-25-vesc-can-link-design.md)

## Goal

Command the drive motor and the steering servo over the CAN link that
already carries telemetry, from a uORB topic any future controller can
publish, with a failsafe that is a visible state rather than an absence of
traffic.

## Starting state

The receive side is built and running on hardware. `fdcan_init()` brings up
FDCAN1 at 1 Mbit/s, `fdcan_receive()` drains Rx FIFO 0, and the `vesc`
daemon decodes `CAN_PACKET_STATUS_5` into the `vesc_status` topic.

Two things were put in place for this phase and are used now rather than
changed:

- The Tx FIFO is already allocated in message RAM at word 512, 32 elements
  of four words. Adding it later would have shifted the Rx FIFO's start
  address, which corrupts frames silently.
- `VESC_CAN_ID`, `VESC_BITRATE` and `VESC_EN` exist, and the hardware filter
  narrows to one controller id once discovery has answered what it is.

`control_trajectory` is a reserved companion-link message id with no
definition. It stays reserved. This spec does not define it.

## Non-goals

- **A controller.** Nothing here decides what the setpoint should be. This
  is the actuator end of the pipe.
- **Steering angle in engineering units.** The command is normalised, and
  `vesc_status` still publishes the ADC feedback as raw volts. Closing that
  loop needs a calibration with its own zero and scale, which is its own
  piece of work.
- **RC passthrough.** RC reaches this through the topic like anything else,
  once something publishes it.
- **The plain command frames.** `SET_DUTY` (0x00), `SET_CURRENT` (0x01) and
  the `PROCESS_SHORT_BUFFER` servo packet (0x08) stay unimplemented. The
  combined frames cover every case.
- **FDCAN2, CAN FD, remote frames.** Unchanged from the receive design.

## Frames

Two host-to-VESC frames from `docs/can_packet.md`, both six bytes, both
big-endian, both carrying motor and steering together:

| Packet | Id | Bytes 0-3 | Bytes 4-5 |
|---|---|---|---|
| `SET_CURRENT_SERVO` | 0x45 | `int32` amps x 1000 | `int16` servo microseconds |
| `SET_DUTY_SERVO` | 0x46 | `int32` duty x 100000 | `int16` servo microseconds |

Sending motor and steering in one frame is the reason these custom packets
exist. Using the standard pair instead would double the frame rate and put a
variable skew between the two axes.

The CAN identifier is `(packet_id << 8) | controller_id`, extended, same
layout as the telemetry.

## The topic

`actuator_command`, 16 characters, well inside the 20 the uORB path budget
allows.

```c
struct actuator_command_s
{
  uint64_t timestamp;    /*  0: us */
  float    motor;        /*  8: amps if mode is CURRENT, duty ratio if DUTY */
  float    steering;     /* 12: normalised -1..+1, left positive */
  uint8_t  mode;         /* 16: ACTUATOR_MODE_DUTY or _CURRENT */
  uint8_t  pad[7];       /* 17 */
};                       /* 24 */
```

The pad is seven bytes, not three. The `uint64_t` gives the struct eight-byte
alignment, so a three-byte pad would leave four more bytes of padding the
compiler inserts and `o_format` does not know about. uORB prints a topic by
walking the format string and stepping the offset by each conversion's size,
with no alignment applied, so implicit padding makes `uorb_listener` print
garbage without failing. Making the tail explicit keeps the struct and the
format string in step, and `static_assert` on every offset plus `sizeof`
catches it if they drift.

### Why steering is normalised

Not microseconds: that hard-codes one servo's geometry into every publisher,
and changing a linkage would mean changing the controller.

Not radians: that claims a calibrated road-wheel angle. The receive design
deliberately published the steering ADC as raw volts rather than an angle,
because the conversion is a calibration with its own zero and scale. Emitting
a command in radians would make exactly the claim the other direction that
we refused to make on the way in.

So the topic carries an authority fraction, and the daemon owns the mapping
to microseconds through `VESC_STEER_MIN`, `VESC_STEER_TRIM` and
`VESC_STEER_MAX`. Positive is left, matching the body FLU and nav ENU
convention the estimator uses, so a positive yaw rate demand and a positive
steering command agree in sign.

### Why mode travels with the setpoint

Duty and current are different physical quantities and the frame id differs
between them. Carrying the mode as a parameter instead would let a publisher
that thinks it is asking for 0.2 duty ask for 0.2 amps, and neither end would
notice. In the message, the units are unambiguous at the point they are set.

## Components

| Unit | Responsibility |
|---|---|
| `apps/vesc/vesc_proto.c` | Encode a setpoint into six bytes. Pure, host-tested. |
| `boards/fmuv6c/src/fdcan.c` | `fdcan_transmit()`: one frame into the Tx FIFO. |
| `apps/uorb_msgs` | The `actuator_command` topic. |
| `apps/vesc/vesc.c` | Subscribe, arm state, failsafe, fixed-rate transmit. |
| `apps/vesc/vesc_main.c` | `arm`, `disarm`, `set`; status reporting. |

### Encoder interface

```c
/* Clamp, then encode. Returns false and writes neutral on a value that is
 * not finite, so a NaN setpoint cannot reach the wire as whatever bit
 * pattern the cast produced.
 */

bool vesc_encode_current_servo(float amps, uint16_t servo_us,
                               FAR uint8_t *out);
bool vesc_encode_duty_servo(float duty, uint16_t servo_us,
                            FAR uint8_t *out);
uint32_t vesc_can_id(uint8_t packet_id, uint8_t controller_id);
```

Both write six bytes and assume the caller has already clamped to the
parameter limits; the encoder's own range check is the last line of defence
against a value the protocol cannot represent, not a substitute for policy.

### Driver interface

```c
int fdcan_transmit(FAR const struct fdcan_frame_s *frame);
```

Returns `OK`, `-EAGAIN` when the Tx FIFO is full, or `-EINVAL` before init.

`fdcan_init()` gains an explicit `TXESC` write for the same reason it already
writes `RXESC`: the reset value happens to be the eight-byte element the RAM
layout assumes, and relying on a reset value for a number the layout depends
on is how the layout quietly stops being true.

`struct fdcan_stats_s` gains `tx` and `tx_full`.

## Cadence

Transmission runs at `VESC_TX_RATE`, default 50 Hz, on its own timer rather
than on topic arrival. A publisher that stops, stutters, or runs at 400 Hz
should not change what the bus sees. A steady heartbeat is also what keeps
the VESC's internal command timeout from firing during normal operation.

The existing 2 ms receive poll is unchanged; the transmit deadline is checked
in the same loop.

## Failsafe

Neutral is zero motor and `VESC_STEER_TRIM` microseconds, in whichever frame
`mode` last selected, or duty if no setpoint has ever arrived.

Three conditions produce it, each counted separately in `vesc status`:

1. **Disarmed.** The default at start.
2. **No setpoint yet.** Nothing has ever been published.
3. **Stale.** The newest setpoint is older than `VESC_CMD_TO_MS`, default
   200 ms — ten transmit periods at the default rate.

Neutral keeps transmitting at the full rate. Going silent instead would make
a failsafe indistinguishable from a crashed daemon or a severed cable, and
would leave the servo holding its last position with no indication why.

The VESC's own CAN timeout, roughly a second, remains the backstop for the
case this daemon cannot cover: the board itself being dead. Nothing in this
design replaces it.

## Limits

| Parameter | Default | Meaning |
|---|---|---|
| `VESC_TX_RATE` | 50 | Command frame rate, Hz |
| `VESC_CMD_TO_MS` | 200 | Setpoint age before neutral, ms |
| `VESC_CUR_MAX` | 20.0 | Motor current magnitude ceiling, A |
| `VESC_DUTY_MAX` | 0.30 | Duty magnitude ceiling, 0..1 |
| `VESC_STEER_MIN` | 1100 | Servo microseconds at steering -1 |
| `VESC_STEER_TRIM` | 1500 | Servo microseconds at steering 0 |
| `VESC_STEER_MAX` | 1900 | Servo microseconds at steering +1 |

All names are within `PARAM_NAME_MAX` of 16.

`VESC_DUTY_MAX` defaults to 0.30 and `VESC_CUR_MAX` to 20 A because the first
armed run happens on a bench with a vehicle that may be on its wheels. These
are deliberately low; raising them is a decision made with the hardware in
front of you.

The steering endpoints are separate parameters rather than a centre and a
span because servo linkages are rarely symmetric, and forcing symmetry would
mean trimming one end by giving up travel at the other. Each is constrained
to the 800..2200 microseconds `can_packet.md` gives, and `VESC_STEER_MIN` is
allowed to exceed `VESC_STEER_MAX`: that is how a reversed linkage is
handled, and rejecting it would push the reversal into the controller.

Clamping happens in the daemon, before encoding. A setpoint outside the
limits is counted, not silently accepted, so a controller with a scaling bug
shows up as a number rather than as a vehicle that feels sluggish.

Range checks are written `!(x >= lo && x <= hi)` rather than `x < lo ||
x > hi`. NaN compares false against everything, so the second form passes it
through and the cast to `int32_t` is undefined. The first form sends it to
neutral.

## Arming

`vesc arm` and `vesc disarm`. Disarmed is the state at start and after any
daemon restart; it does not persist to a parameter.

Arming is refused when a *fresh* setpoint is commanding a non-zero motor,
where fresh means younger than `VESC_CMD_TO_MS` and non-zero means `|motor|`
above 0.001. A vehicle that arms directly into a live throttle demand is the
accident this prevents.

A stale setpoint, or none at all, does not block arming. Both already produce
neutral output, so refusing them would buy no safety and would make the
daemon unusable on a bench: `vesc set` publishes one message, which is stale
200 ms later, so an arm gated on freshness could never be reached by hand.

Only the motor is checked. A steering command that is not centred is not
dangerous with the motor at zero, and refusing to arm on it would mean
straightening the wheels before every arm for no gain.

Disarming always succeeds and takes effect on the next transmit period, at
most 20 ms.

The daemon transmits neutral while disarmed rather than nothing. That makes
the whole path — driver, encoder, cadence, wiring, termination, and the
VESC's willingness to accept the frame — verifiable on a bench with the motor
guaranteed at zero. It is the difference between testing the link and testing
the link and the motor at once.

## Failure handling

| Condition | Response |
|---|---|
| Tx FIFO full | Count `tx_full`, drop the frame, carry on |
| Bus off | Reported in `vesc status`; recovery is not automatic |
| Setpoint not finite | Neutral, counted |
| Setpoint out of range | Clamped, counted |
| `mode` unrecognised | Neutral, counted |

A dropped command frame is not retried at the application level. At 50 Hz the
next one is 20 ms away and carries fresher intent; re-sending a stale
setpoint is worse than skipping it.

Automatic retransmission stays at the hardware default. This matters for
diagnosis: with no VESC on the bus, or with no terminator, a classic CAN
transmitter never gets an ACK and retries indefinitely, so the 32-deep FIFO
fills in about 0.6 seconds at 50 Hz and `tx_full` starts climbing. That is a
precise symptom rather than a silence, and it is the first thing to look at
when a bench run does nothing.

## Testing

Host tests for the encoders, run by a new `tools/test-vesc-tx.sh` alongside
the existing codec test:

- Neutral encodes to zero motor and the trim microseconds.
- Both extremes of each axis, and the sign of each.
- The x1000 and x100000 scalings at a value where a wrong factor is
  unmistakable.
- Byte order: a big-endian value that is not a palindrome, so a swap fails.
- Negative values, which is where a lost sign extension hides.
- Out-of-range and non-finite input reach neutral rather than the wire.

Every test is verified to fail when the encoder is deliberately broken —
little-endian, dropped sign, wrong scale factor — the same check applied to
the decoder, because a test that round-trips the implementation asserts
nothing.

The existing `tools/test-fdcan-ram.sh` already covers the Tx FIFO's placement
and element count.

## Hardware verification

In order, because each step makes the next one safe:

1. **Wheels off the ground, disarmed.** `vesc start`, then `vesc status`
   shows `tx` climbing at roughly 50 Hz and `tx_full` at zero. If `tx_full`
   climbs instead, nothing is acknowledging: check termination and wiring
   before anything else.
2. **Servo only, wheels still off the ground.** Disarmed, confirm the servo
   holds trim, and adjust `VESC_STEER_TRIM` until the wheels are straight.
   Checking the endpoints needs arming, because disarmed output is neutral on
   both axes: arm with the motor command at zero, sweep steering with `vesc
   set`, and watch for linkage binding at full travel before trusting
   `VESC_STEER_MIN` and `VESC_STEER_MAX`.
3. **Motor, wheels still off the ground.** Arm, command a small duty, confirm
   the tachometer moves in `vesc_status` in the direction expected.
4. **Failsafe.** With the motor turning, stop publishing and confirm it stops
   within `VESC_CMD_TO_MS` and the steering returns to trim. Then repeat with
   `vesc disarm` while a setpoint is live.
5. **Only then on the ground.**

Step 4 is the one worth doing twice. A failsafe that has never been observed
firing is a failsafe nobody knows the state of.

## Known limitations

- **Bus-off is reported, not recovered.** Clearing it needs an INIT cycle and
  a decision about when retrying is safe. Reporting it first means the
  recovery policy gets written against observed behaviour.
- **No steering feedback loop.** The ADC voltage arrives on `vesc_status` and
  nothing compares it to the command. Closing that needs the calibration this
  spec declines to invent.
- **One VESC.** `VESC_CAN_ID` addresses a single controller.
- **Transmit jitter is the poll interval.** Commands go out on a 2 ms polled
  loop, so a 50 Hz cadence carries up to 2 ms of jitter. Well inside what a
  servo or a motor controller cares about, and the cost of not owning an
  interrupt yet.
