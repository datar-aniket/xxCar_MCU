# VESC CAN Link — Design

Date: 2026-08-25
Branch: `step5-estimator-imu-pipeline`
Status: approved, ready for implementation planning

## Goal

Bring up FDCAN1 and receive telemetry from the VESC: tachometer, filtered
motor current, and the ADC1 voltage that is the steering feedback.

Receive only. Transmit - and therefore any ability to turn a motor - is a
later phase.

## Why receive-only first

A driver that can only listen cannot spin a wheel while you are still
finding out whether the bit timing is right. The first FDCAN driver on a
board is the wrong place to discover that a transmit path works but the
sample point is marginal, and a vehicle is the wrong place to discover it
twice.

It also makes the first deliverable answer three questions that have to be
answered anyway: is the bus wired and terminated, what node ID is the VESC
actually using, and which packet IDs does this firmware really emit. The
last one matters because `CAN_PACKET_STATUS_5` here is customised from
stock VESC - see `docs/can_packet.md`.

## Starting state

CAN is not enabled anywhere. `board.h` sets the FDCAN kernel clock to HSE
(16 MHz) and defines nothing else - no pins, no driver, no configuration.

NuttX ships only a SocketCAN driver for the H7 (`stm32_fdcan_sock.c`), which
needs `CONFIG_NET`. **PX4 does not use it.** Their fmu-v6c defconfig has no
`CONFIG_NET`, no `CONFIG_NET_CAN` and no `CONFIG_CAN`; they drive the
peripheral directly from
`src/drivers/uavcan/uavcan_drivers/stm32h7/driver/`. This design follows
PX4, both because adding a network stack to carry two message types is a
poor trade and because this tree already contains hand-written peripheral
drivers - `ist8310.c`, `ms5611.c`, `icm42688.c` - written for the same kind
of reason.

## Non-goals

- **Transmit.** No TX path, no command frames, no motor or servo output.
  The message RAM is laid out with room for it so adding TX later moves
  nothing, but none of it is written.
- **Control.** `CAN_PACKET_SET_CURRENT_SERVO` and `SET_DUTY_SERVO` are the
  eventual command path and belong with `control_trajectory` and a
  controller, not here.
- **Steering angle.** The ADC voltage is published raw. Converting it to an
  angle is a calibration with its own zero and scale, and baking a guess
  into the topic would repeat the mistake `SENS_MAG0_ROT` already made once
  in this tree.
- **FDCAN2.** One interface, one device.
- **CAN FD.** Classic 8-byte frames only. Every frame in `can_packet.md`
  fits, and FD would change the message RAM element size.

## Hardware

| | |
|---|---|
| Interface | FDCAN1 |
| Pins | PD0 RX, PD1 TX - the fmu-v6c assignment PX4 uses |
| Bitrate | 1 Mbit/s |
| Kernel clock | HSE, 16 MHz, already set in `board.h` |

### Bit timing

Derived, and cross-checked against PX4's generic solver so the numbers are
not a guess:

```
  tq per bit = 16 MHz / 1 Mbit/s = 16
  prescaler  = 1                       -> NBRP  = 0
  bs1_bs2_sum = 15
  bs1 = (7 * 15 - 1) / 8 = 13          -> NTSEG1 = 12
  bs2 = 15 - 13        =  2            -> NTSEG2 = 1
  sjw = 1                              -> NSJW   = 0
```

Registers hold each value minus one, which is why the constants look off by
one from the segment counts.

Sample point is `(1 + 13) / 16` = **87.5%**, the CiA recommendation. PX4's
solver produces exactly these values for this clock and bitrate.

### Message RAM

The H7's 2560 words are shared between FDCAN1 and FDCAN2, and every region's
start address is assigned by software. Two regions that overlap corrupt each
other's frames and present as bus errors, so the layout is pinned here
rather than derived at runtime.

Following PX4's allocation, which gives each interface the lower half:

| Region | Offset (words) | Elements | Words |
|---|---|---|---|
| Standard ID filters | 0 | 128 | 128 |
| Extended ID filters | 128 | 64 | 128 |
| RX FIFO0 | 256 | 64 | 256 |
| TX FIFO | 512 | 32 | 128 |

640 of the 1280 words available to FDCAN1. RX FIFO1 is not allocated -
nothing routes to it.

**The TX FIFO is reserved even though nothing transmits.** Laying it out now
costs nothing and means adding TX later does not move RX FIFO0, which would
otherwise be a silent corruption the day someone implements transmit.

Element size is four words: two header words plus eight data bytes. That is
the classic-CAN size and is why CAN FD is out of scope - FD elements are 18
words and the whole table would change.

### Initialisation

The sequence PX4 uses, and it is order-sensitive:

1. Enable the FDCAN1 peripheral clock and configure PD0/PD1.
2. `CCCR |= INIT`, then **wait** for `INIT` to read back set. It is not
   immediate, and configuring before it latches silently does nothing.
3. `CCCR |= CCE` to unlock the configuration registers.
4. Write `NBTP`, the message RAM region registers, and the filter
   configuration.
5. `CCCR &= ~INIT` to start.

## Filtering

Phase one accepts **every** extended frame, because the point is to find out
what is on the bus. `VESC_CAN_ID` defaults to 0 meaning accept-any; setting
it to the discovered node ID narrows the hardware filter so only that VESC
reaches the CPU.

Doing the filtering in hardware rather than in software matters more than it
looks: an unfiltered 1 Mbit/s bus can deliver over 8000 frames a second, and
discarding them in a task means waking up eight thousand times a second to
throw work away.

Standard (11-bit) frames are rejected outright. Every frame in this protocol
is extended.

## Identifier layout

From `docs/can_packet.md`:

```
  29-bit extended ID = (packet_id << 8) | controller_id

  packet_id     = (id >> 8) & 0xFF
  controller_id =  id       & 0xFF
```

## Telemetry: CAN_PACKET_STATUS_5 (0x1B)

Eight bytes, **big-endian**:

| Bytes | Type | Meaning | Scale |
|---|---|---|---|
| 0-3 | `int32` | Tachometer count | raw |
| 4-5 | `int16` | Filtered total current | `A = raw / 10.0` |
| 6-7 | `int16` | External ADC1 voltage | `V = raw / 1000.0` |

**Big-endian is the highest-risk detail in this subsystem.** The MCU is
little-endian, the payload is not, and a byte-order mistake produces numbers
that are wrong by a factor of millions - or, worse for a signed 16-bit
field, occasionally plausible. This is the single strongest argument for the
codec being a separate, host-tested file.

The tachometer is an accumulated position count, not a rate. Deriving a
speed from it is a consumer's job and is not done here.

## Components

| File | Responsibility |
|---|---|
| `boards/fmuv6c/src/fdcan.h` / `.c` | The peripheral. Clock, pins, bit timing, message RAM, filters, FIFO drain. Knows nothing about VESCs. |
| `apps/vesc/vesc_proto.h` / `.c` | Pure codec: ID split, `STATUS_5` decode. No I/O, no uORB, host-testable. |
| `apps/vesc/vesc.h` / `.c` | Daemon: drain frames, decode, publish, count what it sees. |
| `apps/vesc/vesc_main.c` | `vesc start \| stop \| status` |
| `apps/uorb_msgs` | `vesc_status` topic |
| `boards/fmuv6c/include/board.h` | `GPIO_CAN1_RX` PD0, `GPIO_CAN1_TX` PD1 |
| `boards/fmuv6c/configs/nsh/defconfig` | `CONFIG_STM32H7_FDCAN1` |

The split is the same one the companion link uses, and for the same reason:
the codec is where the bugs are and the codec is the part that can be tested
without hardware.

### Driver interface

```c
int  fdcan_init(uint32_t bitrate);
int  fdcan_receive(FAR struct fdcan_frame_s *frame);  /* 0, or -EAGAIN */
void fdcan_set_filter(uint8_t controller_id);         /* 0 = accept any */
bool fdcan_stats(FAR struct fdcan_stats_s *stats);
```

`fdcan_receive` is non-blocking and returns one frame. The daemon polls.

**Polled, not interrupt-driven, and this is a deliberate trade.** It is much
less code for a first driver, and adequate for telemetry - but it puts the
poll interval's jitter on the steering feedback. If the later control loop
needs tighter timing this is the thing to revisit, and it is recorded here
so that revisiting it looks like a planned step rather than a defect.

## The topic

```c
struct vesc_status_s
{
  uint64_t timestamp;         /* us, publication */
  uint64_t timestamp_sample;  /* us, when the frame was drained */
  int32_t  tachometer;        /* accumulated counts */
  float    current_a;         /* A, raw / 10 */
  float    adc_volts;         /* V, raw / 1000 - steering feedback */
  uint8_t  controller_id;
  uint8_t  pad[3];
};
```

Raw units throughout. No angle, no speed, no sign convention imposed.

## Parameters

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `VESC_EN` | int32 | 0 | Start the VESC link at boot |
| `VESC_CAN_ID` | int32 | 0 | Controller ID to accept; 0 accepts any |
| `VESC_BITRATE` | int32 | 1000000 | CAN bitrate |

`VESC_EN` defaults off. A new driver touching a new peripheral should not be
in the boot path until it has run.

All names fit `PARAM_NAME_MAX` (16).

## Discovery

`vesc status` reports every `(packet_id, controller_id)` pair seen, with a
count and a rate:

```
vesc: running on FDCAN1 at 1000000 bit/s, filter accept-any
  bus     rx 4821  lost 0  errors 0  state ERROR_ACTIVE
  seen    packet 0x1B  id 0x4A   1204 frames   50.1 Hz
          packet 0x09  id 0x4A   1204 frames   50.1 Hz
  status5 tach 182734  current 2.41 A  adc 1.652 V   age 18 ms
```

Frames whose packet ID is not decoded are still counted. A VESC emitting
something `can_packet.md` does not describe is a fact worth seeing, not an
error worth hiding.

## Failure handling

| Condition | Response |
|---|---|
| RX FIFO0 overrun | Count. It means the daemon is not draining fast enough. |
| Bus-off | Report state; do not auto-recover silently. Bus-off means sustained transmit errors and hiding it hides a wiring fault. |
| Error-passive | Report. Usually termination or a bitrate mismatch. |
| `STATUS_5` with wrong DLC | Count separately from an unknown packet ID. One is a firmware mismatch, the other is a newer VESC. |
| No frames at all | `vesc status` says so explicitly rather than showing zeros. |

The DLC distinction is the same one the companion link makes between an
unknown message ID and a known ID with the wrong length, and for the same
reason: one is benign and the other is the two ends disagreeing.

## Testing

`tests/vesc_proto_test.c` with `tools/test-vesc-proto.sh`:

- `STATUS_5` decodes to known values from a hand-built **big-endian** buffer
- a **negative** current and a negative tachometer round-trip correctly -
  sign extension across a byte-swapped `int16` is exactly where this breaks
- the ID split recovers packet and controller from a constructed 29-bit ID,
  including `controller_id = 0xFF` and `packet_id = 0xFF`
- a wrong DLC is rejected rather than read past
- the scale factors are asserted against the values in `can_packet.md`
  (÷10 and ÷1000), not against whatever the implementation does

Host-testable because the codec has no I/O. The driver itself is verified on
hardware - there is no way to test a peripheral's bit timing on a host, and
pretending otherwise with a mock would test the mock.

## Hardware verification

1. `vesc start` with nothing connected: `state ERROR_ACTIVE`, `rx 0`. Proves
   the peripheral initialises without a bus.
2. VESC connected, bus terminated: frames appear. `errors 0` is the real
   check - a bitrate mismatch shows as frames arriving with errors, not as
   silence.
3. `seen` lists the packet IDs. Confirm `0x1B` is present and note the
   controller ID.
4. Turn the wheel by hand: `adc` moves. Turn the motor by hand: `tach`
   moves. That is the end-to-end proof the decode is right, and it needs no
   transmit.
5. Set `VESC_CAN_ID` to the discovered ID and confirm `rx` keeps climbing -
   proves the hardware filter accepts rather than rejects everything.

## Known limitations

**Polled receive.** Jitter on telemetry is bounded by the poll interval, not
by the bus. Fine for feedback, revisit for control.

**No transmit.** The motor cannot be commanded from this firmware at all
after this phase. That is the point of it.

**One VESC.** The filter accepts a single controller ID. Multiple nodes
would need a filter per node and an instance per topic.

**Bus-off is reported, not recovered.** Automatic recovery would mask a
wiring or termination fault as intermittent behaviour. Recovery becomes
worth adding when transmit exists and a fault can be provoked deliberately.
