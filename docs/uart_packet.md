# Companion UART link — EXTERNAL_POSE

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
PPS input — see section 7 — so hardware flow control must stay off.

## 2. Frame

```
0xFE | id | len | payload[len] | crc_lo | crc_hi
```

- `0xFE` sync byte.
- `id` message id, section 3.
- `len` payload length in bytes, 0..64.
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
| 2 | `CONTROL_TRAJ` | companion → board | *reserved, undefined* |
| 3 | `TIMESYNC_REQ` | companion → board | 8 |
| 4 | `TIMESYNC_REP` | board → companion | 24 |
| 5 | `TIMESYNC_START` | companion → board | 8 |
| 6 | `TIMESYNC_END` | companion → board | 16 |
| 16 | `ESTIMATOR_POSE` | board → companion | 56 |

An unknown id is counted and ignored — that is a companion newer than the
firmware, which is benign. A **known** id with the wrong length is counted
separately and is not benign: it means the two ends disagree about a format.

## 4. Byte order

**Little-endian, throughout.** Integers and IEEE-754 floats are laid out as
the STM32 stores them, so a payload is a direct `memcpy` of the struct at both
ends.

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

The estimator refuses a pose whose timestamp is more than **500 ms** away
from its own clock, in either direction, and counts it as `extnav_bad_time`.

The two directions fail for different reasons. Too old means the correction
would land on the wrong part of the buffered trajectory. Ahead of the board's
own clock has no innocent explanation at all — it means the timesync is
wrong.

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
| `EK3_EXT_TIMEOUT` | 1000 ms | dropout before position is dropped |

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

### Gating

x and y are gated **jointly**, on the sum of both innovations against the
combined variance, the way ArduPilot's `posTestRatio` works — not
independently per axis. A pose 500 m wrong in x with y unchanged is rejected
as one bad pose. Gating the axes separately lets exactly that case through on
the good axis.

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

## 7. ESTIMATOR_POSE (id 16, 56 bytes)

What the board sends back, at `EXT_TX_RATE` (default 200 Hz), driven by a
hardware timer.

| Offset | Type | Field | Units |
|---|---|---|---|
| 0 | `uint64` | `timestamp_us` | UTC µs once synced, else board monotonic |
| 8 | `float32[3]` | `position` | m, local ENU |
| 20 | `float32[4]` | `quaternion` | w, x, y, z — body to nav |
| 36 | `float32[3]` | `velocity` | m/s, local ENU |
| 48 | `uint8` | `solution_status` | bits below |
| 49 | `uint8` | `reset_counter` | estimator reset generation |
| 50 | `uint8[6]` | pad | |

`timestamp_us` is the **IMU sample time** of the solution, not the time the
message was sent. So the age you measure on arrival legitimately includes the
estimator's own output latency; it is not all transport.

This timestamp is guaranteed never to be in the future — it is clamped to the
board's own UTC before sending, and the PPS discipline in section 8 keeps
that clock aligned to yours.

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

### reset_counter

Incremented when the **estimator's** datum moves — which the external
position path does deliberately on a sustained rejection run. Position jumps
discontinuously when it happens.

## 8. Clock synchronisation

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
   telling the board what the companion concluded.

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
stays monotonic, because monotonic is the only clock that cannot step. UTC is
a wire format: converted going out, converted coming back, never seen by the
estimator.

### PPS

Drive **TELEM2 CTS** (PC9) with a 3.3 V rising edge on the UTC second. The
board captures it on TIM3 at 1 MHz and steers its UTC offset so that edge
lands on a whole second.

Because the pulse comes *from* the companion, its edge is that machine's own
second boundary — which makes this a direct microsecond-resolution
measurement against the very clock the companion will judge the board's
timestamps by. A timesync round trip only resolves milliseconds.

PPS corrects the **phase within a second**; it cannot say which second it is.
A timesync burst, or the RTC, must establish the absolute time first —
`companion status` reports `not disciplining` until both hold.

`pps status` shows lock state, period and edge counts.

## 9. Minimal sender

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

## 10. Diagnostics

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

`ekf3 status` reports the consumption side: `extnav_in`, `extnav_bad_time`,
`extnav_untimed`, and whether external position is actually being fused.

## Related

- [`can_packet.md`](can_packet.md) — the VESC CAN protocol. **Big-endian**,
  unlike this one.
- [`parameters-ekf.md`](parameters-ekf.md) — `EK3_SRC*` source selection, and
  what is actually fused today.
