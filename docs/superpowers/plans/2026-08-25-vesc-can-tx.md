# VESC CAN Transmit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Command the VESC's drive motor and steering servo over the existing
CAN link, from a uORB topic, with an arm gate and a failsafe that transmits
neutral rather than going silent.

**Architecture:** Four layers, each testable without the one above it. Pure
encoders turn a setpoint into six big-endian bytes. A pure policy function
turns (arm state, setpoint, age, limits) into a resolved command. The FDCAN
driver puts one frame into the Tx FIFO. The daemon glues them to a topic and
a 50 Hz clock.

**Tech Stack:** C11, NuttX, uORB, STM32H7 FDCAN1, host tests built with `cc`.

**Spec:** [docs/superpowers/specs/2026-08-25-vesc-can-tx-design.md](../specs/2026-08-25-vesc-can-tx-design.md)

## Global Constraints

- **Style:** NuttX kernel style. Two-space indent, braces on their own line
  indented with the body, `FAR` on pointer parameters, 80-column limit
  (`./tools/verify.sh` fails the build on a violation).
- **No floating point in the FDCAN driver.** `boards/fmuv6c/src/` is board
  support; all scaling happens in `apps/vesc/`.
- **Every wire-format value is BIG-ENDIAN.** This MCU is little-endian.
- **uORB topic names:** at most 20 characters. `ORB_NAME_FITS` enforces it.
- **uORB structs must have no implicit padding.** `o_format` is walked with
  no alignment applied, so a padding byte the format string does not know
  about makes `uorb_listener` print garbage without failing.
- **Parameter names:** at most 16 characters (`PARAM_NAME_MAX`).
- **Rounding, not truncation,** on every float-to-integer scale. Whether a
  given value exposes this depends on where the float product lands: `0.53f`
  times `100000` truncates to `52999`, while `0.29f` rounds up to exactly
  `29000.0f` during the multiply and truncates correctly. Exposing values
  must be searched for, not guessed.
- **Range checks are written `!(x >= lo && x <= hi)`.** NaN compares false
  against everything, so `x < lo || x > hi` passes it through to an
  undefined cast.
- **Defaults are deliberately timid:** `VESC_DUTY_MAX` 0.30, `VESC_CUR_MAX`
  20.0. Raising them is a decision made with the hardware present.
- **Commit after each task.** End every commit message with
  `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`.

## File Structure

| File | Responsibility |
|---|---|
| `apps/vesc/vesc_proto.{h,c}` | *(modify)* Wire format. Gains the two encoders. No policy, no I/O. |
| `apps/vesc/vesc_cmd.{h,c}` | *(create)* Policy: arm gate, failsafe, clamping, steering map. Pure, no uORB, no hardware. |
| `boards/fmuv6c/src/fdcan.c`, `boards/fmuv6c/include/fdcan.h` | *(modify)* `fdcan_transmit()`, `TXESC`, tx counters. |
| `apps/uorb_msgs/uorb_msgs.{h,c}` | *(modify)* The `actuator_command` topic. |
| `apps/vesc/vesc.{h,c}` | *(modify)* Subscribe, arm state, 50 Hz transmit deadline. |
| `apps/vesc/vesc_main.c` | *(modify)* `arm`, `disarm`, `set`; extended status. |
| `apps/param/param.c` | *(modify)* Seven new parameters. |
| `tests/vesc_proto_test.c` | *(modify)* Encoder tests. |
| `tests/vesc_cmd_test.c` | *(create)* Policy tests. |
| `tools/test-vesc-cmd.sh` | *(create)* Runner for the policy tests. |

**Divergence from the spec:** it names a single `tools/test-vesc-tx.sh`. This
plan instead puts the encoder tests in the existing `tools/test-vesc-proto.sh`
— they test `vesc_proto.c`, which that script already compiles — and creates
`tools/test-vesc-cmd.sh` for the policy. `./tools/verify.sh` discovers
`tools/test-*.sh` automatically, so both run without registering them
anywhere.

`vesc_cmd` is a separate file from `vesc_proto` because they fail
differently. `vesc_proto` gets byte order and scaling wrong; `vesc_cmd` gets
*safety* wrong. Keeping the arm gate and the failsafe in a file with no
hardware and no uORB is what lets a host test drive them through every state.

---

### Task 1: The command encoders

**Files:**
- Modify: `apps/vesc/vesc_proto.h`, `apps/vesc/vesc_proto.c`
- Test: `tests/vesc_proto_test.c`
- Modify: `tools/test-vesc-proto.sh` (no change needed; it already compiles both files)

**Interfaces:**
- Consumes: nothing from other tasks.
- Produces:
  ```c
  #define VESC_CMD_SERVO_DLC     6
  #define VESC_SERVO_US_MIN      800u
  #define VESC_SERVO_US_MAX      2200u
  #define VESC_PROTO_CUR_LIMIT_A 200.0f
  #define VESC_PROTO_DUTY_LIMIT  1.0f

  bool vesc_encode_current_servo(float amps, uint16_t servo_us,
                                 FAR uint8_t *out);
  bool vesc_encode_duty_servo(float duty, uint16_t servo_us,
                              FAR uint8_t *out);
  uint32_t vesc_can_id(uint8_t packet_id, uint8_t controller_id);
  ```
  Both encoders write exactly `VESC_CMD_SERVO_DLC` bytes. They return `false`
  and write a zero motor value when the motor argument is not finite; they
  clamp silently to the protocol limits otherwise.

- [ ] **Step 1: Write the failing tests**

Append to `tests/vesc_proto_test.c`, before `main`:

```c
/* Command encoding. Every constant below is hand-computed from
 * docs/can_packet.md, big-endian, and chosen so a byte swap or a wrong
 * scale factor cannot accidentally produce the right answer.
 */

static void test_can_id_build(void)
{
  assert(vesc_can_id(VESC_PACKET_SET_CURRENT_SERVO, 0x4a) == 0x454a);
  assert(vesc_can_id(VESC_PACKET_SET_DUTY_SERVO, 0x00) == 0x4600);
  assert(vesc_can_id(0xff, 0xff) == 0xffff);

  /* Round trip against the split used on the receive path. */

  assert(vesc_packet_id(vesc_can_id(0x1b, 0x4a)) == 0x1b);
  assert(vesc_controller_id(vesc_can_id(0x1b, 0x4a)) == 0x4a);
}

/*   5.0 A  x 1000 = 5000   = 0x00001388
 *   1500 us               = 0x05DC
 */

static void test_encode_current_positive(void)
{
  uint8_t out[VESC_CMD_SERVO_DLC];

  assert(vesc_encode_current_servo(5.0f, 1500, out));
  assert(out[0] == 0x00 && out[1] == 0x00 &&
         out[2] == 0x13 && out[3] == 0x88);
  assert(out[4] == 0x05 && out[5] == 0xdc);
}

/*  -3.25 A x 1000 = -3250  = 0xFFFFF34E
 *   800 us                = 0x0320
 *
 * Negative is where a lost sign extension hides: it survives every positive
 * case and then commands the motor the wrong way.
 */

static void test_encode_current_negative(void)
{
  uint8_t out[VESC_CMD_SERVO_DLC];

  assert(vesc_encode_current_servo(-3.25f, 800, out));
  assert(out[0] == 0xff && out[1] == 0xff &&
         out[2] == 0xf3 && out[3] == 0x4e);
  assert(out[4] == 0x03 && out[5] == 0x20);
}

/*  0.53 duty x 100000 = 53000 = 0x0000CF08
 *  2200 us                    = 0x0898
 *
 * THIS TEST IS ABOUT ROUNDING, and the value is not arbitrary. The nearest
 * float to 0.53 is 0.52999997138977, and 0.53f * 100000.0f lands on the
 * float below 53000, so a truncating cast ships 52999 while rounding ships
 * 53000.
 *
 * Most values do NOT expose this. 0.29f * 100000.0f rounds up to exactly
 * 29000.0f during the multiply, so truncation gets the right answer there.
 * The value had to be searched for rather than guessed.
 */

static void test_encode_duty_rounds(void)
{
  uint8_t out[VESC_CMD_SERVO_DLC];

  assert(vesc_encode_duty_servo(0.53f, 2200, out));
  assert(out[0] == 0x00 && out[1] == 0x00 &&
         out[2] == 0xcf && out[3] == 0x08);
  assert(out[4] == 0x08 && out[5] == 0x98);
}

/*  8.03 A x 1000 = 8030 = 0x00001F5E - the same trap on the current scale,
 *  which has its own factor and so its own exposing values.
 */

static void test_encode_current_rounds(void)
{
  uint8_t out[VESC_CMD_SERVO_DLC];

  assert(vesc_encode_current_servo(8.03f, 1500, out));
  assert(out[0] == 0x00 && out[1] == 0x00 &&
         out[2] == 0x1f && out[3] == 0x5e);
}

/*  -0.30 duty x 100000 = -30000 = 0xFFFF8AD0 */

static void test_encode_duty_negative(void)
{
  uint8_t out[VESC_CMD_SERVO_DLC];

  assert(vesc_encode_duty_servo(-0.30f, 1500, out));
  assert(out[0] == 0xff && out[1] == 0xff &&
         out[2] == 0x8a && out[3] == 0xd0);
  assert(out[4] == 0x05 && out[5] == 0xdc);
}

/* The same number means different things in the two frames: current is
 * x1000 and duty is x100000. Feeding both the identical 0.05 makes a
 * copy-pasted scale factor fail immediately.
 *
 *   0.05 A    x 1000   = 50    = 0x00000032
 *   0.05 duty x 100000 = 5000  = 0x00001388
 */

static void test_encode_scales_differ(void)
{
  uint8_t cur[VESC_CMD_SERVO_DLC];
  uint8_t dut[VESC_CMD_SERVO_DLC];

  assert(vesc_encode_current_servo(0.05f, 1500, cur));
  assert(vesc_encode_duty_servo(0.05f, 1500, dut));

  assert(cur[2] == 0x00 && cur[3] == 0x32);
  assert(dut[2] == 0x13 && dut[3] == 0x88);
}

static void test_encode_neutral(void)
{
  uint8_t out[VESC_CMD_SERVO_DLC];

  assert(vesc_encode_current_servo(0.0f, 1500, out));
  assert(out[0] == 0 && out[1] == 0 && out[2] == 0 && out[3] == 0);
  assert(out[4] == 0x05 && out[5] == 0xdc);

  assert(vesc_encode_duty_servo(0.0f, 1500, out));
  assert(out[0] == 0 && out[1] == 0 && out[2] == 0 && out[3] == 0);
  assert(out[4] == 0x05 && out[5] == 0xdc);
}

/* A non-finite motor value must not reach the wire. The cast of a NaN to
 * int32_t is undefined, and in practice produces whatever the FPU had; on a
 * motor controller that is a random torque command.
 */

static void test_encode_rejects_non_finite(void)
{
  uint8_t out[VESC_CMD_SERVO_DLC];

  assert(!vesc_encode_current_servo(NAN, 1500, out));
  assert(out[0] == 0 && out[1] == 0 && out[2] == 0 && out[3] == 0);
  assert(out[4] == 0x05 && out[5] == 0xdc);

  assert(!vesc_encode_duty_servo(INFINITY, 1500, out));
  assert(out[0] == 0 && out[1] == 0 && out[2] == 0 && out[3] == 0);

  assert(!vesc_encode_duty_servo(-INFINITY, 1500, out));
  assert(out[0] == 0 && out[1] == 0 && out[2] == 0 && out[3] == 0);
}

/* Clamping is the last line of defence, not the policy. The daemon clamps to
 * the parameter limits first; these are the limits the PROTOCOL cannot
 * exceed.
 *
 *   200 A  x 1000   = 200000 = 0x00030D40
 *   1.0 duty x 1e5  = 100000 = 0x000186A0
 */

static void test_encode_clamps(void)
{
  uint8_t out[VESC_CMD_SERVO_DLC];

  assert(vesc_encode_current_servo(1.0e6f, 1500, out));
  assert(out[0] == 0x00 && out[1] == 0x03 &&
         out[2] == 0x0d && out[3] == 0x40);

  assert(vesc_encode_duty_servo(5.0f, 1500, out));
  assert(out[0] == 0x00 && out[1] == 0x01 &&
         out[2] == 0x86 && out[3] == 0xa0);

  /* Servo microseconds clamp to the range can_packet.md gives. */

  assert(vesc_encode_duty_servo(0.0f, 100, out));
  assert(out[4] == 0x03 && out[5] == 0x20);      /* 800 */

  assert(vesc_encode_duty_servo(0.0f, 5000, out));
  assert(out[4] == 0x08 && out[5] == 0x98);      /* 2200 */
}

static void test_encode_null(void)
{
  assert(!vesc_encode_current_servo(1.0f, 1500, NULL));
  assert(!vesc_encode_duty_servo(1.0f, 1500, NULL));
}
```

Add the calls inside `main`, after the existing ones:

```c
  test_can_id_build();
  test_encode_current_positive();
  test_encode_current_negative();
  test_encode_duty_rounds();
  test_encode_current_rounds();
  test_encode_duty_negative();
  test_encode_scales_differ();
  test_encode_neutral();
  test_encode_rejects_non_finite();
  test_encode_clamps();
  test_encode_null();
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `./tools/test-vesc-proto.sh`
Expected: compile error, `implicit declaration of function 'vesc_encode_current_servo'`.

- [ ] **Step 3: Extend the header**

In `apps/vesc/vesc_proto.h`, replace the comment above the command packet
ids — it currently says "nothing here transmits", which stops being true —
and add the new declarations before the `#endif`:

```c
/* Commands, host -> VESC.
 *
 * Only the two combined frames are encoded. They carry motor and steering in
 * one six-byte payload, which is the reason they exist: sending the standard
 * SET_DUTY plus a separate servo packet doubles the frame rate and puts a
 * variable skew between the two axes.
 *
 * The other three are defined so the discovery listing can name what it
 * sees.
 */
```

```c
#define VESC_CMD_SERVO_DLC            6

/* The microsecond range docs/can_packet.md gives for the servo field. */

#define VESC_SERVO_US_MIN             800u
#define VESC_SERVO_US_MAX             2200u

/* Protocol ceilings, NOT vehicle limits. The vehicle limits are parameters
 * and are applied before this. These exist so that a value the wire format
 * cannot represent is impossible rather than merely unlikely.
 */

#define VESC_PROTO_CUR_LIMIT_A        200.0f
#define VESC_PROTO_DUTY_LIMIT         1.0f

/* Build the 29-bit extended identifier. */

uint32_t vesc_can_id(uint8_t packet_id, uint8_t controller_id);

/* Encode a combined motor + steering command into VESC_CMD_SERVO_DLC bytes.
 *
 * Returns false, having written a ZERO motor value, when the motor argument
 * is not finite. A NaN cast to int32_t is undefined, and on a motor
 * controller "undefined" is a torque command nobody chose.
 *
 * Out-of-range motor and servo values are clamped silently: by the time a
 * value reaches here the daemon has already applied and counted the vehicle
 * limits, so this is a backstop, not a policy decision.
 */

bool vesc_encode_current_servo(float amps, uint16_t servo_us,
                               FAR uint8_t *out);
bool vesc_encode_duty_servo(float duty, uint16_t servo_us,
                            FAR uint8_t *out);
```

Add `#include <math.h>` to `vesc_proto.h`? No — put it in `vesc_proto.c`
only. The header must stay includable from files that do not want `math.h`.

- [ ] **Step 4: Implement the encoders**

In `apps/vesc/vesc_proto.c`, add `#include <math.h>` below the existing
include, then append:

```c
/* The mirror of vesc_be32 / vesc_be16: most significant byte first, written
 * one byte at a time for the same reasons.
 */

static void vesc_put_be32(FAR uint8_t *d, int32_t value)
{
  uint32_t v = (uint32_t)value;

  d[0] = (uint8_t)((v >> 24) & 0xff);
  d[1] = (uint8_t)((v >> 16) & 0xff);
  d[2] = (uint8_t)((v >> 8) & 0xff);
  d[3] = (uint8_t)(v & 0xff);
}

static void vesc_put_be16(FAR uint8_t *d, int16_t value)
{
  uint16_t v = (uint16_t)value;

  d[0] = (uint8_t)((v >> 8) & 0xff);
  d[1] = (uint8_t)(v & 0xff);
}

static float vesc_clampf(float v, float lo, float hi)
{
  if (v < lo)
    {
      return lo;
    }

  if (v > hi)
    {
      return hi;
    }

  return v;
}

static uint16_t vesc_clamp_servo(uint16_t us)
{
  if (us < VESC_SERVO_US_MIN)
    {
      return VESC_SERVO_US_MIN;
    }

  if (us > VESC_SERVO_US_MAX)
    {
      return VESC_SERVO_US_MAX;
    }

  return us;
}

/* Shared tail. `scaled` has already been rounded to an integer count.
 *
 * ROUNDED, not truncated. The nearest float to 0.29 is 0.28999999165, so
 * 0.29f * 100000.0f is 28999.999 and a plain cast ships 28999. That is a 3%
 * error that appears at some duty values and not others, which is the
 * hardest kind to notice on a bench.
 */

static bool vesc_encode_servo_frame(float motor, float scale, float limit,
                                    uint16_t servo_us, FAR uint8_t *out)
{
  if (out == NULL)
    {
      return false;
    }

  /* Written this way round on purpose: NaN compares false against every
   * bound, so `motor < -limit || motor > limit` would let it through.
   */

  if (!(motor >= -limit && motor <= limit))
    {
      if (isfinite(motor))
        {
          motor = vesc_clampf(motor, -limit, limit);
        }
      else
        {
          vesc_put_be32(&out[0], 0);
          vesc_put_be16(&out[4], (int16_t)vesc_clamp_servo(servo_us));
          return false;
        }
    }

  vesc_put_be32(&out[0], (int32_t)lroundf(motor * scale));
  vesc_put_be16(&out[4], (int16_t)vesc_clamp_servo(servo_us));
  return true;
}

uint32_t vesc_can_id(uint8_t packet_id, uint8_t controller_id)
{
  return ((uint32_t)packet_id << 8) | (uint32_t)controller_id;
}

bool vesc_encode_current_servo(float amps, uint16_t servo_us,
                               FAR uint8_t *out)
{
  return vesc_encode_servo_frame(amps, 1000.0f, VESC_PROTO_CUR_LIMIT_A,
                                 servo_us, out);
}

bool vesc_encode_duty_servo(float duty, uint16_t servo_us, FAR uint8_t *out)
{
  return vesc_encode_servo_frame(duty, 100000.0f, VESC_PROTO_DUTY_LIMIT,
                                 servo_us, out);
}
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `./tools/test-vesc-proto.sh`
Expected: PASS, both the plain and the sanitizer build.

- [ ] **Step 6: Verify the tests catch real bugs**

Do all three, one at a time, restoring `vesc_proto.c` after each. A test that
round-trips the implementation asserts nothing, and the only way to know is
to break the implementation.

1. Change `lroundf(motor * scale)` to `(motor * scale)`.
   Expected: `test_encode_duty_rounds` fails.
2. Swap `d[0]` and `d[3]` in `vesc_put_be32`.
   Expected: `test_encode_current_positive` fails.
3. Change `vesc_encode_duty_servo`'s scale from `100000.0f` to `1000.0f`.
   Expected: `test_encode_scales_differ` fails.

Record in the commit message that this was done.

- [ ] **Step 7: Commit**

```bash
git add apps/vesc/vesc_proto.h apps/vesc/vesc_proto.c tests/vesc_proto_test.c
git commit -m "vesc: encode the combined motor and steering frames

SET_CURRENT_SERVO and SET_DUTY_SERVO, six big-endian bytes each. Only the
combined frames are encoded: sending the standard SET_DUTY plus a separate
servo packet would double the frame rate and put a variable skew between the
two axes.

Scaling ROUNDS. The nearest float to 0.29 is 0.28999999165, so
0.29f * 100000.0f is 28999.999 and a truncating cast ships 28999 - a 3%
error that shows up at some duty values and not others.

A non-finite motor value writes zero and returns false rather than casting.
A NaN cast to int32_t is undefined, and on a motor controller undefined is a
torque command nobody chose. The range check is written
!(x >= lo && x <= hi) because NaN compares false against every bound, so the
obvious form would pass it straight through.

Confirmed the tests catch real bugs by breaking the encoder three ways:
truncating instead of rounding, swapping the byte order, and reusing the
current scale factor for duty. Each failed the test aimed at it.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: `fdcan_transmit()`

**Files:**
- Modify: `boards/fmuv6c/include/fdcan.h`
- Modify: `boards/fmuv6c/src/fdcan.c`

**Interfaces:**
- Consumes: `struct fdcan_frame_s` (already defined).
- Produces:
  ```c
  int fdcan_transmit(FAR const struct fdcan_frame_s *frame);
  ```
  Returns `OK`, `-EAGAIN` when the Tx FIFO is full, `-EINVAL` before init or
  on a bad argument. `struct fdcan_stats_s` gains `uint32_t tx;` and
  `uint32_t tx_full;`.

There is no host test for this task — it is register writes against hardware
that does not exist off the board. `tools/test-fdcan-ram.sh` already covers
the Tx FIFO's placement and element count, which is the part that can be
wrong arithmetically. The rest is verified in Task 6 on a bench.

- [ ] **Step 1: Update the header**

In `boards/fmuv6c/include/fdcan.h`, replace the "RECEIVE ONLY" paragraph in
the file comment:

```c
 * Receive plus a minimal transmit path. Transmit was deliberately left out
 * of the first version so that a driver still proving its bit timing could
 * not spin a motor; that timing is now confirmed on hardware.
```

Add the two counters to `struct fdcan_stats_s`, after `rejected`:

```c
  uint32_t tx;              /* frames queued for transmission */
  uint32_t tx_full;         /* dropped: Tx FIFO had no free element */
```

And declare the function after `fdcan_receive`:

```c
/* Queue one frame. Returns OK, -EAGAIN when the Tx FIFO is full, or -EINVAL
 * before init.
 *
 * "Queued", not "sent": this returns as soon as the element is in the FIFO
 * and the hardware has been told to send it.
 *
 * A FULL FIFO IS THE DIAGNOSTIC WORTH KNOWING. Classic CAN needs another
 * node to acknowledge, and the hardware retries a frame that is never
 * acknowledged for as long as it takes. So an absent VESC, or a missing bus
 * terminator, fills all 32 elements in about 0.6 s at 50 Hz and every call
 * after that returns -EAGAIN. That climbing tx_full is a precise symptom,
 * and it is the first thing to look at when a bench run does nothing.
 */

int fdcan_transmit(FAR const struct fdcan_frame_s *frame);
```

- [ ] **Step 2: Add the Tx element definitions**

In `boards/fmuv6c/src/fdcan.c`, next to the existing `FDCAN_RX_XTD` block,
add:

```c
/* Tx element word 0 flags. Same bit positions as the Rx element, which is
 * not a coincidence - the two share a layout.
 */

#define FDCAN_TX_XTD              (1u << 30)

/* Tx element word 1: DLC occupies bits [19:16]. FDF and BRS stay clear;
 * setting either would make this a CAN FD frame, which the four-word element
 * size cannot hold.
 */

#define FDCAN_TX_DLC_SHIFT        (16u)
```

- [ ] **Step 3: Configure TXESC in `fdcan_init`**

Immediately after the existing `putreg32(0, STM32_FDCAN1_RXESC);`, add:

```c
  /* TXESC 0: 8-byte data, matching RXESC. Explicit for the same reason - the
   * reset value happens to be what the four-word element in fdcan_ram.h
   * assumes, and leaning on a reset value for a number the RAM layout
   * depends on is how the layout quietly stops being true.
   */

  putreg32(0, STM32_FDCAN1_TXESC);
```

- [ ] **Step 4: Implement `fdcan_transmit`**

Add after `fdcan_receive`:

```c
int fdcan_transmit(FAR const struct fdcan_frame_s *frame)
{
  uint32_t status;
  uint32_t index;
  uint32_t element;
  uint32_t word;
  uint32_t i;

  if (!g_ready || frame == NULL || frame->dlc > 8)
    {
      return -EINVAL;
    }

  status = getreg32(STM32_FDCAN1_TXFQS);

  if ((status & FDCAN_TXFQS_TFQF) != 0)
    {
      /* Nothing on the bus is acknowledging, so the hardware is still
       * retrying frames queued up to 0.6 s ago. Dropping this one is right:
       * at 50 Hz the next carries fresher intent than anything stuck in the
       * queue.
       */

      g_stats.tx_full++;
      return -EAGAIN;
    }

  index = (status & FDCAN_TXFQS_TFQPI_MASK) >> FDCAN_TXFQS_TFQPI_SHIFT;
  element = FDCAN_RAM_TXF_OFF + (index * 4u);

  putreg32(FDCAN_TX_XTD | (frame->id & 0x1fffffffu),
           FDCAN_RAM_WORD(element));
  putreg32((uint32_t)frame->dlc << FDCAN_TX_DLC_SHIFT,
           FDCAN_RAM_WORD(element + 1));

  /* Data words are packed least significant byte first WITHIN each word,
   * while the CAN payload itself is big-endian. Those are two different
   * things: the byte order on the wire was already decided by the encoder,
   * and this only moves bytes into the message RAM in the order the
   * peripheral reads them out.
   */

  for (i = 0; i < 8u; i += 4u)
    {
      word = 0;

      if (i + 0 < frame->dlc) word |= (uint32_t)frame->data[i + 0] << 0;
      if (i + 1 < frame->dlc) word |= (uint32_t)frame->data[i + 1] << 8;
      if (i + 2 < frame->dlc) word |= (uint32_t)frame->data[i + 2] << 16;
      if (i + 3 < frame->dlc) word |= (uint32_t)frame->data[i + 3] << 24;

      putreg32(word, FDCAN_RAM_WORD(element + 2 + (i / 4u)));
    }

  /* Add request: one bit per element index. */

  putreg32(1u << index, STM32_FDCAN1_TXBAR);

  g_stats.tx++;
  return OK;
}
```

The one-line `if` bodies above violate the surrounding brace style. Rewrite
each as a braced block before committing:

```c
      if (i + 0 < frame->dlc)
        {
          word |= (uint32_t)frame->data[i + 0] << 0;
        }
```

- [ ] **Step 5: Build**

Run: `make -j"$(nproc)" 2>&1 | grep -iE "fdcan|error|warning: "`
Expected: `CC: fdcan.c` and `LD: nuttx`, no errors and no warnings.

- [ ] **Step 6: Re-run the RAM layout test**

Run: `./tools/test-fdcan-ram.sh`
Expected: `fdcan_ram: all checks passed`. Nothing should have changed; this
confirms the Tx FIFO reservation the new code writes into is the one the
test checks.

- [ ] **Step 7: Commit**

```bash
git add boards/fmuv6c/include/fdcan.h boards/fmuv6c/src/fdcan.c
git commit -m "board: add a transmit path to the FDCAN driver

One frame into the Tx FIFO at the hardware's put index, then TXBAR. The FIFO
was already allocated in message RAM at word 512 by the receive work, so
nothing moves - which was the point of reserving it then rather than now.

A full FIFO returns -EAGAIN and counts tx_full instead of blocking or
retrying. That counter is the diagnostic worth having: classic CAN needs
another node to acknowledge, and the hardware retries an unacknowledged
frame indefinitely, so an absent VESC or a missing terminator fills all 32
elements in about 0.6 s at 50 Hz. A climbing tx_full is a precise symptom
where a silent failure would be a guess.

Dropped frames are not retried at this level. At 50 Hz the next command is
20 ms away and carries fresher intent than anything stuck in the queue.

TXESC is now written explicitly alongside RXESC. Its reset value happens to
be the 8-byte element the RAM layout assumes, and relying on a reset value
for a number the layout depends on is how the layout quietly stops being
true.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: The `actuator_command` topic

**Files:**
- Modify: `apps/uorb_msgs/uorb_msgs.h`
- Modify: `apps/uorb_msgs/uorb_msgs.c`

**Interfaces:**
- Consumes: nothing.
- Produces:
  ```c
  #define ACTUATOR_MODE_DUTY     0
  #define ACTUATOR_MODE_CURRENT  1

  struct actuator_command_s
  {
    uint64_t timestamp;
    float    motor;
    float    steering;
    uint8_t  mode;
    uint8_t  pad[7];
  };

  ORB_DECLARE(actuator_command);
  int actuator_command_advertise(void);
  int actuator_command_publish(int fd,
                               FAR const struct actuator_command_s *msg);
  ```

- [ ] **Step 1: Add the struct to the header**

In `apps/uorb_msgs/uorb_msgs.h`, after the `vesc_status_s` definition:

```c
/* A command for the actuators: one drive motor and one steering servo.
 *
 * Steering is a NORMALISED authority fraction, not microseconds and not
 * radians.
 *
 * Not microseconds, because that hard-codes one servo's geometry into every
 * publisher and turns a linkage change into a controller change. The daemon
 * owns the mapping through VESC_STEER_MIN / _TRIM / _MAX.
 *
 * Not radians, because that claims a calibrated road-wheel angle. vesc_status
 * deliberately publishes the steering ADC as raw volts rather than an angle,
 * since the conversion is a calibration with its own zero and scale. A
 * command in radians would make exactly the claim the other direction that
 * was refused on the way in.
 *
 * Positive steering is LEFT, matching the body FLU and nav ENU convention the
 * estimator uses, so a positive yaw-rate demand and a positive steering
 * command agree in sign.
 *
 * `mode` travels in the message rather than in a parameter because duty and
 * current are different physical quantities carried in different CAN frames.
 * As a parameter, a publisher asking for 0.2 duty could silently be asking
 * for 0.2 amps, and neither end would notice.
 */

#define ACTUATOR_MODE_DUTY     0    /* motor is a duty ratio, -1..+1 */
#define ACTUATOR_MODE_CURRENT  1    /* motor is amps */

struct actuator_command_s
{
  uint64_t timestamp;             /*  0: us */
  float    motor;                 /*  8: duty ratio or amps, per mode */
  float    steering;              /* 12: normalised -1..+1, left positive */
  uint8_t  mode;                  /* 16: ACTUATOR_MODE_* */
  uint8_t  pad[7];                /* 17 */
};
```

The pad is SEVEN bytes so the struct's real 24-byte size is visible in the
source. `pad[3]` would compile to a byte-identical struct — the `uint64_t`
forces eight-byte alignment either way — so this is legibility, not
correctness. What the asserts actually guard is the absence of *internal*
padding, since uORB walks `o_format` stepping by each conversion's size with
no alignment applied.

Add the declaration next to the others:

```c
ORB_DECLARE(actuator_command);
```

And the prototypes next to `vesc_status`'s:

```c
int actuator_command_advertise(void);
int actuator_command_publish(int fd,
                             FAR const struct actuator_command_s *msg);
```

- [ ] **Step 2: Add the name check and layout asserts**

In `apps/uorb_msgs/uorb_msgs.c`, after `ORB_NAME_FITS("vesc_status");`:

```c
ORB_NAME_FITS("actuator_command");
```

After the `vesc_status_s` layout asserts:

```c
static_assert(offsetof(struct actuator_command_s, timestamp) ==  0, "layout");
static_assert(offsetof(struct actuator_command_s, motor)     ==  8, "layout");
static_assert(offsetof(struct actuator_command_s, steering)  == 12, "layout");
static_assert(offsetof(struct actuator_command_s, mode)      == 16, "layout");
static_assert(sizeof(struct actuator_command_s)              == 24, "layout");
```

- [ ] **Step 3: Add the format string and definition**

After `vesc_status_format`:

```c
static const char actuator_command_format[] =
  "timestamp:%" PRIu64
  ",motor:%hf,steering:%hf"
  ",mode:%hhu";
```

After `ORB_DEFINE(vesc_status, ...)`:

```c
ORB_DEFINE(actuator_command, struct actuator_command_s,
           actuator_command_format);
```

- [ ] **Step 4: Add the helpers**

After `vesc_status_publish`:

```c
int actuator_command_advertise(void)
{
  return orb_advertise(ORB_ID(actuator_command), NULL);
}

int actuator_command_publish(int fd,
                             FAR const struct actuator_command_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(actuator_command), fd, msg);
}
```

- [ ] **Step 5: Build**

Run: `make -j"$(nproc)" 2>&1 | grep -iE "uorb_msgs|error|warning: "`
Expected: `CC: uorb_msgs.c` and no errors. The `static_assert`s are the test
here — they fail the build if the layout drifts.

- [ ] **Step 6: Prove the layout asserts work**

Do NOT try this with `pad[3]`. The `uint64_t` forces eight-byte alignment, so
`pad[3]` and `pad[7]` compile to byte-identical 24-byte structs and every
assert passes. The seven is for the reader, not the compiler.

The asserts guard something real but different: the absence of INTERNAL
padding. Move `mode` above `steering`:

```c
  float    motor;
  uint8_t  mode;
  float    steering;
```

Rebuild. Expected: three `static assertion failed: "layout"` errors, because
`steering` is now at 16 rather than 12 and the struct grew. Restore the
original order.

That gap is the failure worth guarding: `o_format` lists the fields in order
and uORB steps the read offset by each conversion's size with no alignment
applied, so `uorb_listener` would print garbage without failing.

- [ ] **Step 7: Commit**

```bash
git add apps/uorb_msgs/uorb_msgs.h apps/uorb_msgs/uorb_msgs.c
git commit -m "uorb: add the actuator_command topic

Motor and steering for one drive motor and one steering servo, with the mode
carried in the message.

Steering is a normalised authority fraction. Microseconds would hard-code one
servo's geometry into every publisher; radians would claim a calibrated
road-wheel angle, which is exactly the claim vesc_status refused to make when
it published the steering ADC as raw volts. Positive is left, matching the
FLU/ENU convention the estimator already uses.

Mode is in the message rather than a parameter because duty and current are
different quantities in different CAN frames. As a parameter, a publisher
asking for 0.2 duty could silently be asking for 0.2 amps.

The pad is seven bytes, not three. The uint64_t forces eight-byte alignment,
so pad[3] leaves four more bytes the compiler inserts and o_format does not
know about - and uORB walks the format with no alignment applied, so
uorb_listener prints garbage without failing. Confirmed the sizeof assert
catches it: pad[3] compiles to the same 24-byte struct and fails the build
only because of that line.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: The command policy

**Files:**
- Create: `apps/vesc/vesc_cmd.h`, `apps/vesc/vesc_cmd.c`
- Create: `tests/vesc_cmd_test.c`, `tools/test-vesc-cmd.sh`

**Interfaces:**
- Consumes: `VESC_PACKET_SET_CURRENT_SERVO`, `VESC_PACKET_SET_DUTY_SERVO`
  from Task 1's `vesc_proto.h`.
- Produces:
  ```c
  #define VESC_MODE_DUTY        0
  #define VESC_MODE_CURRENT     1

  #define VESC_CMD_ARMED        0
  #define VESC_CMD_DISARMED     1
  #define VESC_CMD_NO_SETPOINT  2
  #define VESC_CMD_STALE        3
  #define VESC_CMD_BAD_MODE     4
  #define VESC_CMD_NREASON      5

  struct vesc_limits_s
  { float cur_max; float duty_max;
    uint16_t steer_min; uint16_t steer_trim; uint16_t steer_max; };

  struct vesc_cmd_out_s
  { uint8_t packet_id; float motor; uint16_t servo_us;
    uint8_t reason; bool clamped; };

  void vesc_cmd_resolve(bool armed, bool have_setpoint,
                        uint8_t mode, float motor, float steering,
                        uint64_t age_us, uint32_t timeout_ms,
                        FAR const struct vesc_limits_s *lim,
                        FAR struct vesc_cmd_out_s *out);

  bool vesc_cmd_may_arm(bool have_setpoint, float motor,
                        uint64_t age_us, uint32_t timeout_ms);

  FAR const char *vesc_cmd_reason_name(uint8_t reason);
  ```

This file takes plain scalars rather than a `struct actuator_command_s`
because that struct lives behind uORB headers that will not compile on a
host. The safety logic is exactly the part worth testing on a host, so the
dependency goes the other way: the daemon unpacks the topic.

- [ ] **Step 1: Write the failing tests**

Create `tests/vesc_cmd_test.c`:

```c
/****************************************************************************
 * tests/vesc_cmd_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The arm gate and the failsafe. This is the part of the VESC link that is
 * dangerous when it is wrong, and none of it needs hardware to exercise, so
 * every state is driven here rather than discovered on a bench.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "vesc_cmd.h"
#include "vesc_proto.h"

static const struct vesc_limits_s g_lim =
{
  .cur_max    = 20.0f,
  .duty_max   = 0.30f,
  .steer_min  = 1100,
  .steer_trim = 1500,
  .steer_max  = 1900,
};

#define CLOSE(a, b) (fabsf((a) - (b)) < 1.0e-4f)

/* Neutral is zero motor at the trim microseconds. Every failsafe path below
 * has to land here, so it gets its own check.
 */

static void expect_neutral(const struct vesc_cmd_out_s *o, uint8_t reason)
{
  assert(o->reason == reason);
  assert(CLOSE(o->motor, 0.0f));
  assert(o->servo_us == 1500);
}

static void test_disarmed_is_neutral(void)
{
  struct vesc_cmd_out_s o;

  /* A live, in-range, perfectly valid setpoint. Disarmed still wins. */

  vesc_cmd_resolve(false, true, VESC_MODE_DUTY, 0.25f, 1.0f,
                   0, 200, &g_lim, &o);
  expect_neutral(&o, VESC_CMD_DISARMED);
}

static void test_no_setpoint_is_neutral(void)
{
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, false, VESC_MODE_DUTY, 0.25f, 1.0f,
                   0, 200, &g_lim, &o);
  expect_neutral(&o, VESC_CMD_NO_SETPOINT);

  /* With nothing ever received there is no mode to honour, so it falls back
   * to duty rather than guessing current.
   */

  assert(o.packet_id == VESC_PACKET_SET_DUTY_SERVO);
}

static void test_stale_is_neutral(void)
{
  struct vesc_cmd_out_s o;

  /* Exactly at the timeout is still live; one microsecond past is not. */

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.25f, 1.0f,
                   200000, 200, &g_lim, &o);
  assert(o.reason == VESC_CMD_ARMED);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.25f, 1.0f,
                   200001, 200, &g_lim, &o);
  expect_neutral(&o, VESC_CMD_STALE);

  /* A stale setpoint still selects the frame it asked for: the VESC should
   * keep seeing the same packet id, carrying zeros.
   */

  vesc_cmd_resolve(true, true, VESC_MODE_CURRENT, 5.0f, 0.0f,
                   1000000, 200, &g_lim, &o);
  expect_neutral(&o, VESC_CMD_STALE);
  assert(o.packet_id == VESC_PACKET_SET_CURRENT_SERVO);
}

static void test_bad_mode_is_neutral(void)
{
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, 7, 0.25f, 1.0f, 0, 200, &g_lim, &o);
  expect_neutral(&o, VESC_CMD_BAD_MODE);
  assert(o.packet_id == VESC_PACKET_SET_DUTY_SERVO);
}

static void test_armed_passes_through(void)
{
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.25f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(o.reason == VESC_CMD_ARMED);
  assert(CLOSE(o.motor, 0.25f));
  assert(o.servo_us == 1500);
  assert(o.packet_id == VESC_PACKET_SET_DUTY_SERVO);
  assert(!o.clamped);

  vesc_cmd_resolve(true, true, VESC_MODE_CURRENT, -8.0f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(o.reason == VESC_CMD_ARMED);
  assert(CLOSE(o.motor, -8.0f));
  assert(o.packet_id == VESC_PACKET_SET_CURRENT_SERVO);
}

/* The two motor limits are different numbers on purpose: a mix-up would
 * clamp duty at 20 and current at 0.3, and both are catastrophic in
 * opposite directions.
 */

static void test_motor_clamps_per_mode(void)
{
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.95f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(CLOSE(o.motor, 0.30f));
  assert(o.clamped);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, -0.95f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(CLOSE(o.motor, -0.30f));
  assert(o.clamped);

  vesc_cmd_resolve(true, true, VESC_MODE_CURRENT, 100.0f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(CLOSE(o.motor, 20.0f));
  assert(o.clamped);

  vesc_cmd_resolve(true, true, VESC_MODE_CURRENT, -100.0f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(CLOSE(o.motor, -20.0f));
  assert(o.clamped);
}

static void test_steering_maps_to_endpoints(void)
{
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(o.servo_us == 1500);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 1.0f,
                   0, 200, &g_lim, &o);
  assert(o.servo_us == 1900);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, -1.0f,
                   0, 200, &g_lim, &o);
  assert(o.servo_us == 1100);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 0.5f,
                   0, 200, &g_lim, &o);
  assert(o.servo_us == 1700);

  /* Beyond full authority clamps to the endpoint, and is counted. */

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 3.0f,
                   0, 200, &g_lim, &o);
  assert(o.servo_us == 1900);
  assert(o.clamped);
}

/* Asymmetric travel is the normal case, not the exception: a linkage rarely
 * gives the same microseconds either side of straight. Each half is scaled
 * against its own endpoint, so trimming one side does not steal travel from
 * the other.
 */

static void test_steering_asymmetric(void)
{
  const struct vesc_limits_s lim =
  {
    .cur_max = 20.0f, .duty_max = 0.30f,
    .steer_min = 1200, .steer_trim = 1480, .steer_max = 1900,
  };
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 1.0f, 0, 200, &lim, &o);
  assert(o.servo_us == 1900);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, -1.0f, 0, 200, &lim, &o);
  assert(o.servo_us == 1200);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 0.0f, 0, 200, &lim, &o);
  assert(o.servo_us == 1480);

  /* Half left travels half of 280 us, not half of 350. */

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, -0.5f, 0, 200, &lim, &o);
  assert(o.servo_us == 1340);
}

/* A reversed linkage is expressed by MIN above MAX. Handling it here keeps
 * the reversal out of every controller that ever publishes a command.
 */

static void test_steering_reversed(void)
{
  const struct vesc_limits_s lim =
  {
    .cur_max = 20.0f, .duty_max = 0.30f,
    .steer_min = 1900, .steer_trim = 1500, .steer_max = 1100,
  };
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 1.0f, 0, 200, &lim, &o);
  assert(o.servo_us == 1100);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, -1.0f, 0, 200, &lim, &o);
  assert(o.servo_us == 1900);
}

/* NaN compares false against every bound. Written the wrong way round, the
 * range check passes it straight through to an undefined cast.
 */

static void test_non_finite_is_neutral(void)
{
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, NAN, 0.0f,
                   0, 200, &g_lim, &o);
  assert(CLOSE(o.motor, 0.0f));
  assert(o.clamped);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, INFINITY, 0.0f,
                   0, 200, &g_lim, &o);
  assert(CLOSE(o.motor, 0.0f));
  assert(o.clamped);

  /* A NaN steering command goes to trim, not to an endpoint. */

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, NAN,
                   0, 200, &g_lim, &o);
  assert(o.servo_us == 1500);
  assert(o.clamped);
}

/* The arm gate. It refuses exactly one thing: arming into a live throttle
 * demand. Everything else must be permitted, or the daemon cannot be armed
 * by hand at all - `vesc set` publishes once and is stale 200 ms later.
 */

static void test_may_arm(void)
{
  assert(vesc_cmd_may_arm(false, 0.0f, 0, 200));        /* nothing yet */
  assert(vesc_cmd_may_arm(false, 0.9f, 0, 200));        /* nothing yet */
  assert(vesc_cmd_may_arm(true, 0.0f, 0, 200));         /* fresh, zero */
  assert(vesc_cmd_may_arm(true, 0.9f, 200001, 200));    /* stale */
  assert(vesc_cmd_may_arm(true, 0.0005f, 0, 200));      /* below epsilon */

  assert(!vesc_cmd_may_arm(true, 0.9f, 0, 200));        /* fresh throttle */
  assert(!vesc_cmd_may_arm(true, -0.9f, 0, 200));       /* either sign */
  assert(!vesc_cmd_may_arm(true, 0.9f, 200000, 200));   /* exactly fresh */

  /* Steering is not part of the gate: a turned wheel is not dangerous with
   * the motor at zero, and refusing on it would mean straightening the
   * wheels before every arm.
   */

  assert(vesc_cmd_may_arm(true, 0.0f, 0, 200));
}

static void test_reason_names(void)
{
  int i;

  for (i = 0; i < VESC_CMD_NREASON; i++)
    {
      assert(vesc_cmd_reason_name((uint8_t)i) != NULL);
      assert(strlen(vesc_cmd_reason_name((uint8_t)i)) > 0);
    }

  assert(vesc_cmd_reason_name(200) != NULL);
}

int main(void)
{
  test_disarmed_is_neutral();
  test_no_setpoint_is_neutral();
  test_stale_is_neutral();
  test_bad_mode_is_neutral();
  test_armed_passes_through();
  test_motor_clamps_per_mode();
  test_steering_maps_to_endpoints();
  test_steering_asymmetric();
  test_steering_reversed();
  test_non_finite_is_neutral();
  test_may_arm();
  test_reason_names();

  printf("vesc_cmd: all checks passed\n");
  return 0;
}
```

- [ ] **Step 2: Write the test runner**

Create `tools/test-vesc-cmd.sh` and `chmod +x` it:

```bash
#!/usr/bin/env bash
# Host-side test for the VESC command policy: the arm gate and the failsafe.
#
# This is the part of the link that is dangerous when it is wrong, and none
# of it needs hardware, so every state is driven here rather than discovered
# on a bench with a vehicle on its wheels.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -I"$REPO/apps/vesc" \
   -o "$OUT/test" "$REPO/tests/vesc_cmd_test.c" \
   "$REPO/apps/vesc/vesc_cmd.c" "$REPO/apps/vesc/vesc_proto.c" -lm
"$OUT/test"

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -fsanitize=undefined,address -fno-sanitize-recover=all \
   -I"$REPO/apps/vesc" \
   -o "$OUT/test-san" "$REPO/tests/vesc_cmd_test.c" \
   "$REPO/apps/vesc/vesc_cmd.c" "$REPO/apps/vesc/vesc_proto.c" -lm
"$OUT/test-san"
```

- [ ] **Step 3: Run to verify it fails**

Run: `./tools/test-vesc-cmd.sh`
Expected: `fatal error: vesc_cmd.h: No such file or directory`.

- [ ] **Step 4: Write the header**

Create `apps/vesc/vesc_cmd.h`:

```c
/****************************************************************************
 * apps/vesc/vesc_cmd.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The command policy: arm gate, failsafe, limits, steering map.
 *
 * Separate from vesc_proto.c because the two fail differently. vesc_proto
 * gets byte order and scaling wrong; this file gets SAFETY wrong. Keeping it
 * free of uORB and hardware is what lets a host test drive every state,
 * including the ones a bench cannot produce on demand.
 *
 * It takes plain scalars rather than a struct actuator_command_s because
 * that struct lives behind uORB headers that will not compile on a host. The
 * daemon unpacks the topic.
 ****************************************************************************/

#ifndef __APPS_VESC_VESC_CMD_H
#define __APPS_VESC_VESC_CMD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

/* Mirrors ACTUATOR_MODE_* in uorb_msgs.h. Duplicated rather than included
 * because that header needs uORB. vesc.c carries a static_assert that the
 * two agree, so drift is a build failure rather than a silent unit change.
 */

#define VESC_MODE_DUTY        0
#define VESC_MODE_CURRENT     1

/* Why the output is what it is. Every one of these except ARMED means the
 * wire is carrying neutral.
 */

#define VESC_CMD_ARMED        0
#define VESC_CMD_DISARMED     1
#define VESC_CMD_NO_SETPOINT  2
#define VESC_CMD_STALE        3
#define VESC_CMD_BAD_MODE     4
#define VESC_CMD_NREASON      5

/* Below this the motor command counts as zero for the arm gate. Floats
 * arriving from a controller are rarely exactly 0.0f.
 */

#define VESC_CMD_ZERO_EPS     0.001f

struct vesc_limits_s
{
  float    cur_max;      /* A, magnitude ceiling in current mode */
  float    duty_max;     /* 0..1, magnitude ceiling in duty mode */
  uint16_t steer_min;    /* us at steering -1 */
  uint16_t steer_trim;   /* us at steering 0 */
  uint16_t steer_max;    /* us at steering +1 */
};

struct vesc_cmd_out_s
{
  uint8_t  packet_id;    /* VESC_PACKET_SET_DUTY_SERVO or _CURRENT_SERVO */
  float    motor;        /* clamped, in the units the packet id implies */
  uint16_t servo_us;
  uint8_t  reason;       /* VESC_CMD_* */
  bool     clamped;      /* input was out of range or not finite */
};

/* Resolve one transmit period's output.
 *
 * `age_us` is how old the setpoint is; the caller computes it and must guard
 * against a timestamp in the future, since this takes an unsigned value.
 *
 * Always writes `out`. There is no failure return: every input produces
 * something safe to put on the wire, which is the entire point.
 */

void vesc_cmd_resolve(bool armed, bool have_setpoint,
                      uint8_t mode, float motor, float steering,
                      uint64_t age_us, uint32_t timeout_ms,
                      FAR const struct vesc_limits_s *lim,
                      FAR struct vesc_cmd_out_s *out);

/* May the daemon be armed right now?
 *
 * Refuses exactly one thing: arming into a live non-zero motor demand. A
 * stale setpoint or none at all is permitted, because both already produce
 * neutral output - and refusing them would make the daemon impossible to arm
 * by hand, since `vesc set` publishes once and is stale 200 ms later.
 */

bool vesc_cmd_may_arm(bool have_setpoint, float motor,
                      uint64_t age_us, uint32_t timeout_ms);

FAR const char *vesc_cmd_reason_name(uint8_t reason);

#endif /* __APPS_VESC_VESC_CMD_H */
```

- [ ] **Step 5: Implement the policy**

Create `apps/vesc/vesc_cmd.c`:

```c
/****************************************************************************
 * apps/vesc/vesc_cmd.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <math.h>

#include "vesc_cmd.h"
#include "vesc_proto.h"

/* Every bound check in this file is written !(x >= lo && x <= hi) rather
 * than x < lo || x > hi. NaN compares false against everything, so the
 * second form passes it through untouched - and a NaN reaching the encoder
 * is an undefined cast on a motor command.
 */

static float cmd_clamp(float v, float lo, float hi, FAR bool *clamped)
{
  if (!(v >= lo && v <= hi))
    {
      *clamped = true;

      if (!isfinite(v))
        {
          /* Not finite: neither endpoint is the right answer, so fall to the
           * middle. For a motor limit that is zero, for steering it is trim.
           */

          return 0.0f;
        }

      return v < lo ? lo : hi;
    }

  return v;
}

/* Each half of the travel is scaled against its own endpoint. A linkage
 * rarely gives equal microseconds either side of straight, and forcing
 * symmetry would mean giving up travel on one side to trim the other.
 *
 * steer_min ABOVE steer_max is legal and means a reversed linkage. The
 * arithmetic handles it without a special case, which keeps the reversal out
 * of every controller that publishes a command.
 */

static uint16_t cmd_steer_us(float steering,
                             FAR const struct vesc_limits_s *lim)
{
  float span;
  float us;

  if (steering >= 0.0f)
    {
      span = (float)lim->steer_max - (float)lim->steer_trim;
    }
  else
    {
      span = (float)lim->steer_trim - (float)lim->steer_min;
    }

  us = (float)lim->steer_trim + steering * span;

  /* Round rather than truncate: 1699.9997 is 1700 microseconds, not 1699. */

  return (uint16_t)lroundf(us);
}

static void cmd_neutral(uint8_t packet_id, uint8_t reason,
                        FAR const struct vesc_limits_s *lim,
                        FAR struct vesc_cmd_out_s *out)
{
  out->packet_id = packet_id;
  out->motor = 0.0f;
  out->servo_us = lim->steer_trim;
  out->reason = reason;
  out->clamped = false;
}

void vesc_cmd_resolve(bool armed, bool have_setpoint,
                      uint8_t mode, float motor, float steering,
                      uint64_t age_us, uint32_t timeout_ms,
                      FAR const struct vesc_limits_s *lim,
                      FAR struct vesc_cmd_out_s *out)
{
  uint8_t packet_id;
  float limit;

  if (lim == NULL || out == NULL)
    {
      return;
    }

  /* Pick the frame first, so that a failsafe keeps sending the same packet
   * id the VESC was already seeing - carrying zeros. Switching frame ids at
   * the moment of a failsafe would look, from the far end, like a different
   * controller taking over.
   */

  if (have_setpoint && mode == VESC_MODE_CURRENT)
    {
      packet_id = VESC_PACKET_SET_CURRENT_SERVO;
      limit = lim->cur_max;
    }
  else
    {
      packet_id = VESC_PACKET_SET_DUTY_SERVO;
      limit = lim->duty_max;
    }

  if (!armed)
    {
      cmd_neutral(packet_id, VESC_CMD_DISARMED, lim, out);
      return;
    }

  if (!have_setpoint)
    {
      cmd_neutral(packet_id, VESC_CMD_NO_SETPOINT, lim, out);
      return;
    }

  if (mode != VESC_MODE_DUTY && mode != VESC_MODE_CURRENT)
    {
      cmd_neutral(packet_id, VESC_CMD_BAD_MODE, lim, out);
      return;
    }

  if (age_us > (uint64_t)timeout_ms * 1000ull)
    {
      cmd_neutral(packet_id, VESC_CMD_STALE, lim, out);
      return;
    }

  out->packet_id = packet_id;
  out->reason = VESC_CMD_ARMED;
  out->clamped = false;
  out->motor = cmd_clamp(motor, -limit, limit, &out->clamped);
  out->servo_us =
    cmd_steer_us(cmd_clamp(steering, -1.0f, 1.0f, &out->clamped), lim);
}

bool vesc_cmd_may_arm(bool have_setpoint, float motor,
                      uint64_t age_us, uint32_t timeout_ms)
{
  if (!have_setpoint)
    {
      return true;
    }

  if (age_us > (uint64_t)timeout_ms * 1000ull)
    {
      return true;
    }

  /* Fresh. Refuse only if it is actually commanding the motor. A non-finite
   * value counts as commanding: it is not zero, and it is not trustworthy.
   */

  if (!isfinite(motor))
    {
      return false;
    }

  return fabsf(motor) <= VESC_CMD_ZERO_EPS;
}

FAR const char *vesc_cmd_reason_name(uint8_t reason)
{
  switch (reason)
    {
      case VESC_CMD_ARMED:       return "armed";
      case VESC_CMD_DISARMED:    return "disarmed";
      case VESC_CMD_NO_SETPOINT: return "no-setpoint";
      case VESC_CMD_STALE:       return "stale";
      case VESC_CMD_BAD_MODE:    return "bad-mode";
      default:                   return "unknown";
    }
}
```

- [ ] **Step 6: Run to verify it passes**

Run: `./tools/test-vesc-cmd.sh`
Expected: `vesc_cmd: all checks passed`, twice.

- [ ] **Step 7: Verify the tests catch real bugs**

Restore after each:

1. In `vesc_cmd_resolve`, move the `!armed` check below the clamp so it no
   longer short-circuits — i.e. delete the `return` in that branch.
   Expected: `test_disarmed_is_neutral` fails.
2. In `cmd_clamp`, change `!(v >= lo && v <= hi)` to `v < lo || v > hi`.
   Expected: `test_non_finite_is_neutral` fails.
3. In `cmd_steer_us`, use `steer_max - steer_trim` for both halves.
   Expected: `test_steering_asymmetric` fails on the `-0.5` case.
4. In `vesc_cmd_may_arm`, drop the `age_us` check.
   Expected: `test_may_arm` fails on the stale case.

- [ ] **Step 8: Commit**

```bash
chmod +x tools/test-vesc-cmd.sh
git add apps/vesc/vesc_cmd.h apps/vesc/vesc_cmd.c \
        tests/vesc_cmd_test.c tools/test-vesc-cmd.sh
git commit -m "vesc: add the command policy

The arm gate, the failsafe, the limits and the steering map, in a file with
no uORB and no hardware so a host test can drive every state - including the
ones a bench cannot produce on demand.

Separate from vesc_proto because the two fail differently: vesc_proto gets
byte order and scaling wrong, this gets safety wrong.

The frame id is chosen before the failsafe branches, so a failsafe keeps
sending the packet id the VESC was already seeing, carrying zeros. Switching
ids at that moment would look from the far end like a different controller
taking over.

Each half of the steering travel scales against its own endpoint, so trimming
one side does not steal travel from the other, and steer_min above steer_max
falls out as a reversed linkage with no special case - which keeps the
reversal out of every controller that publishes a command.

Confirmed the tests catch real bugs by breaking four things: letting the
disarmed branch fall through, writing the range check as x < lo || x > hi so
NaN passes, using one span for both steering halves, and dropping the
staleness check from the arm gate.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Wire it into the daemon

**Files:**
- Modify: `apps/param/param.c`
- Modify: `apps/vesc/vesc.h`, `apps/vesc/vesc.c`, `apps/vesc/vesc_main.c`
- Modify: `apps/vesc/Makefile`

**Interfaces:**
- Consumes: everything from Tasks 1-4.
- Produces: `int vesc_arm(bool armed);` and the extended
  `struct vesc_daemon_status_s`.

- [ ] **Step 1: Add the parameters**

In `apps/param/param.c`, after the `VESC_BITRATE` entry:

```c
  /* Transmit. The two motor ceilings are deliberately timid: the first armed
   * run happens on a bench with a vehicle that may be on its wheels.
   * Raising them is a decision made with the hardware in front of you.
   *
   * VESC_STEER_MIN is allowed to exceed VESC_STEER_MAX. That is how a
   * reversed linkage is expressed, and rejecting it would push the reversal
   * into every controller that ever publishes a command.
   */

  { "VESC_TX_RATE", PARAM_TYPE_INT32, I32(50), I32(1), I32(200),
    "Command frame rate (Hz)" },
  { "VESC_CMD_TO_MS", PARAM_TYPE_INT32, I32(200), I32(20), I32(5000),
    "Setpoint age before failsafe neutral (ms)" },
  { "VESC_CUR_MAX", PARAM_TYPE_FLOAT, F32(20.0f), F32(0.0f), F32(200.0f),
    "Motor current magnitude ceiling (A)" },
  { "VESC_DUTY_MAX", PARAM_TYPE_FLOAT, F32(0.30f), F32(0.0f), F32(1.0f),
    "Motor duty magnitude ceiling (0-1)" },
  { "VESC_STEER_MIN", PARAM_TYPE_INT32, I32(1100), I32(800), I32(2200),
    "Servo pulse at steering -1, full right (us)" },
  { "VESC_STEER_TRIM", PARAM_TYPE_INT32, I32(1500), I32(800), I32(2200),
    "Servo pulse at steering 0, straight (us)" },
  { "VESC_STEER_MAX", PARAM_TYPE_INT32, I32(1900), I32(800), I32(2200),
    "Servo pulse at steering +1, full left (us)" },
```

- [ ] **Step 2: Extend the daemon status struct**

In `apps/vesc/vesc.h`, add the include and the new fields.

Add near the top with the other includes:

```c
#include "vesc_cmd.h"
```

Add to `struct vesc_daemon_status_s`, after `publish_errors`:

```c
  /* Transmit */

  bool     armed;
  uint32_t tx_rate;                     /* Hz, as read at start */
  uint32_t cmd_timeout_ms;
  struct vesc_limits_s limits;

  uint32_t setpoints;                   /* actuator_command messages taken */
  uint32_t tx_sent;                     /* frames handed to the driver */
  uint32_t tx_errors;                   /* driver refused the frame */
  uint32_t tx_clamped;                  /* setpoint was out of range */
  uint32_t reason_count[VESC_CMD_NREASON];
  uint8_t  last_reason;
  float    last_motor;                  /* what actually went out */
  uint16_t last_servo_us;
```

And declare the arm entry point next to `vesc_start`:

```c
/* Arm or disarm. Returns 0, -ESRCH when not running, or -EPERM when a live
 * non-zero motor command is already being published - arming into a throttle
 * demand somebody else set is the accident this refuses.
 */

int vesc_arm(bool armed);
```

- [ ] **Step 3: Add the transmit path to the daemon**

In `apps/vesc/vesc.c`:

Add to the includes:

```c
#include "vesc_cmd.h"
```

Add after the existing `#define`s:

```c
/* The two mode enumerations have to agree. They are declared separately
 * because vesc_cmd.h must compile on a host without uORB, so this is the
 * only place that can check them.
 */

static_assert(VESC_MODE_DUTY == ACTUATOR_MODE_DUTY, "mode mismatch");
static_assert(VESC_MODE_CURRENT == ACTUATOR_MODE_CURRENT, "mode mismatch");
```

Add `#include <assert.h>` for `static_assert`.

Add the file-scope arm flag next to `g_running`:

```c
/* Written by vesc_arm() from a caller's thread, read by the daemon. A bool
 * is a single-word store on this target, and the daemon only ever reads it;
 * the mutex protects the status copy, not this.
 */

static volatile bool g_armed;

/* Read by vesc_arm() from a caller's thread. It is set once at start and
 * never changes, but reading it out of g_status would mean reading a
 * structure the daemon rewrites every 2 ms under a mutex, for a value that
 * does not need one.
 */

static uint32_t g_cmd_timeout_ms;
```

Add the setpoint state and the transmit function before `vesc_daemon`:

```c
/* The newest setpoint, kept whole. Its own struct rather than fields on the
 * status so that resolving a command needs no lock: the daemon is the only
 * writer and the only reader.
 */

struct vesc_setpoint_s
{
  bool     valid;
  uint8_t  mode;
  float    motor;
  float    steering;
  uint64_t stamp_us;
};

static struct vesc_setpoint_s g_setpoint;

static void vesc_take_setpoints(int sub, FAR struct vesc_daemon_status_s *s)
{
  bool updated = false;
  int drained = 0;

  if (sub < 0 || orb_check(sub, &updated) < 0 || !updated)
    {
      return;
    }

  while (drained++ < VESC_DRAIN_MAX)
    {
      struct actuator_command_s message;
      uint64_t now;

      if (orb_copy(ORB_ID(actuator_command), sub, &message) < 0)
        {
          return;
        }

      now = vesc_now_us();

      /* A zero or future timestamp is stamped on arrival rather than
       * refused. Age is unsigned, and now - future underflows to something
       * enormous, which would read as permanently stale.
       */

      if (message.timestamp == 0 || message.timestamp > now)
        {
          message.timestamp = now;
        }

      g_setpoint.valid = true;
      g_setpoint.mode = message.mode;
      g_setpoint.motor = message.motor;
      g_setpoint.steering = message.steering;
      g_setpoint.stamp_us = message.timestamp;
      s->setpoints++;

      if (orb_check(sub, &updated) < 0 || !updated)
        {
          return;
        }
    }
}

static void vesc_transmit(FAR struct vesc_daemon_status_s *s)
{
  struct vesc_cmd_out_s cmd;
  struct fdcan_frame_s frame;
  uint64_t now = vesc_now_us();
  uint64_t age = 0;
  bool ok;

  if (g_setpoint.valid && now > g_setpoint.stamp_us)
    {
      age = now - g_setpoint.stamp_us;
    }

  vesc_cmd_resolve(g_armed, g_setpoint.valid, g_setpoint.mode,
                   g_setpoint.motor, g_setpoint.steering,
                   age, s->cmd_timeout_ms, &s->limits, &cmd);

  if (cmd.reason < VESC_CMD_NREASON)
    {
      s->reason_count[cmd.reason]++;
    }

  s->last_reason = cmd.reason;
  s->armed = g_armed;

  if (cmd.clamped)
    {
      s->tx_clamped++;
    }

  frame.id = vesc_can_id(cmd.packet_id, s->filter_id);
  frame.dlc = VESC_CMD_SERVO_DLC;

  if (cmd.packet_id == VESC_PACKET_SET_CURRENT_SERVO)
    {
      ok = vesc_encode_current_servo(cmd.motor, cmd.servo_us, frame.data);
    }
  else
    {
      ok = vesc_encode_duty_servo(cmd.motor, cmd.servo_us, frame.data);
    }

  /* The encoder refusing means it wrote a zero motor value, which is still
   * safe to send - and sending it is better than skipping, because the VESC
   * would otherwise see a gap it reads as a dropout.
   */

  if (!ok)
    {
      s->tx_clamped++;
    }

  s->last_motor = cmd.motor;
  s->last_servo_us = cmd.servo_us;

  if (fdcan_transmit(&frame) < 0)
    {
      s->tx_errors++;
      return;
    }

  s->tx_sent++;
}
```

**Note on `frame.id`:** it uses `s->filter_id`, the `VESC_CAN_ID` parameter.
With `VESC_CAN_ID` at 0 the daemon is in discovery mode and would address
controller 0, which is unlikely to be the right node. Guard the transmit at
the call site in Step 4 rather than sending to a guessed id.

- [ ] **Step 4: Call it from the loop**

In `vesc_daemon`, after `status.filter_id = ...`, add:

```c
  status.tx_rate = (uint32_t)param_i32("VESC_TX_RATE");
  status.cmd_timeout_ms = (uint32_t)param_i32("VESC_CMD_TO_MS");
  g_cmd_timeout_ms = status.cmd_timeout_ms;
  status.limits.cur_max = param_f32("VESC_CUR_MAX");
  status.limits.duty_max = param_f32("VESC_DUTY_MAX");
  status.limits.steer_min = (uint16_t)param_i32("VESC_STEER_MIN");
  status.limits.steer_trim = (uint16_t)param_i32("VESC_STEER_TRIM");
  status.limits.steer_max = (uint16_t)param_i32("VESC_STEER_MAX");
```

Declare the locals at the top of `vesc_daemon`:

```c
  int sub = -1;
  uint64_t next_tx_us;
  uint64_t tx_period_us;
```

After the `pub` check, subscribe:

```c
  sub = orb_subscribe(ORB_ID(actuator_command));

  if (sub < 0)
    {
      syslog(LOG_ERR, "[vesc] cannot subscribe actuator_command (%d)\n",
             errno);
      goto out;
    }

  tx_period_us = 1000000ull / status.tx_rate;
  next_tx_us = vesc_now_us();
```

Inside the `while (!g_should_stop)` loop, after the receive drain and before
`fdcan_stats`:

```c
      vesc_take_setpoints(sub, &status);

      /* Transmit only once the controller id is known. VESC_CAN_ID at 0 is
       * discovery mode, and there the id would be a guess - commanding a
       * node picked at random is worse than not commanding at all.
       */

      if (status.filter_id != 0 && vesc_now_us() >= next_tx_us)
        {
          next_tx_us += tx_period_us;

          /* If the loop fell behind, resynchronise rather than trying to
           * catch up: a burst of stale commands is worse than a late one.
           */

          if (next_tx_us <= vesc_now_us())
            {
              next_tx_us = vesc_now_us() + tx_period_us;
            }

          vesc_transmit(&status);
        }
```

In the `out:` block, before `orb_unadvertise(pub)`:

```c
  if (sub >= 0)
    {
      orb_unsubscribe(sub);
    }

  g_armed = false;
```

And in `vesc_start`, before `task_create`:

```c
  g_armed = false;
  memset(&g_setpoint, 0, sizeof(g_setpoint));
```

Add the arm entry point at the end of the file:

```c
int vesc_arm(bool armed)
{
  uint64_t now;
  uint64_t age = 0;

  if (!g_running)
    {
      return -ESRCH;
    }

  if (!armed)
    {
      /* Disarming always succeeds. A refusal path here would be a way to
       * fail to make the vehicle safe.
       */

      g_armed = false;
      return 0;
    }

  now = vesc_now_us();

  if (g_setpoint.valid && now > g_setpoint.stamp_us)
    {
      age = now - g_setpoint.stamp_us;
    }

  if (!vesc_cmd_may_arm(g_setpoint.valid, g_setpoint.motor, age,
                        g_cmd_timeout_ms))
    {
      return -EPERM;
    }

  g_armed = true;
  return 0;
}
```

- [ ] **Step 5: Add the commands**

In `apps/vesc/vesc_main.c`, extend `usage()`:

```c
  printf("Usage: vesc start | stop | status | arm | disarm\n"
         "       vesc set duty|current <motor> <steering> [seconds]\n"
         "\n"
         "  Receives VESC telemetry on FDCAN1 and publishes vesc_status.\n"
         "  Commands the motor and steering from actuator_command.\n"
         "\n"
         "  <motor>     duty ratio -1..+1, or amps, per mode\n"
         "  <steering>  normalised -1..+1, POSITIVE IS LEFT\n"
         "  [seconds]   how long to keep publishing; default 2, max 30\n"
         "\n"
         "  Transmit needs VESC_CAN_ID set to the real node id. At 0 the\n"
         "  link is in discovery mode and sends nothing.\n"
         "\n"
         "  VESC_CAN_ID     0 accepts any controller id (discovery)\n"
         "  VESC_BITRATE    bus bitrate; only 1000000 is implemented\n"
         "  VESC_EN         start at boot\n"
         "  VESC_TX_RATE    command frame rate, Hz\n"
         "  VESC_CMD_TO_MS  setpoint age before failsafe neutral\n"
         "  VESC_CUR_MAX    current ceiling, A\n"
         "  VESC_DUTY_MAX   duty ceiling, 0-1\n"
         "  VESC_STEER_*    MIN / TRIM / MAX servo pulse, us\n");
```

Add includes: `#include <stdlib.h>` is present; add `#include <unistd.h>`,
`#include <math.h>`, `#include <uORB/uORB.h>`, `#include "../param/param.h"`,
`#include "../uorb_msgs/uorb_msgs.h"`.

Add to `print_status`, after the `status5` block:

```c
  printf("  tx      %s  sent %" PRIu32 "  errors %" PRIu32
         "  clamped %" PRIu32 "\n",
         s.armed ? "ARMED" : "disarmed",
         s.tx_sent, s.tx_errors, s.tx_clamped);

  if (s.filter_id == 0)
    {
      printf("  tx      DISABLED - VESC_CAN_ID is 0 (discovery mode)\n");
    }

  printf("  tx      bus_full %" PRIu32 "  last %s  motor %.3f  servo %u us\n",
         s.bus.tx_full, vesc_cmd_reason_name(s.last_reason),
         (double)s.last_motor, (unsigned)s.last_servo_us);

  printf("  reasons armed %" PRIu32 "  disarmed %" PRIu32
         "  no-setpoint %" PRIu32 "  stale %" PRIu32 "  bad-mode %" PRIu32
         "\n",
         s.reason_count[VESC_CMD_ARMED],
         s.reason_count[VESC_CMD_DISARMED],
         s.reason_count[VESC_CMD_NO_SETPOINT],
         s.reason_count[VESC_CMD_STALE],
         s.reason_count[VESC_CMD_BAD_MODE]);

  printf("  setpts  %" PRIu32 "  limits cur %.1f A  duty %.2f  "
         "steer %u/%u/%u us\n",
         s.setpoints, (double)s.limits.cur_max, (double)s.limits.duty_max,
         (unsigned)s.limits.steer_min, (unsigned)s.limits.steer_trim,
         (unsigned)s.limits.steer_max);
```

Add the `set` implementation before `main`:

```c
/* Publish a setpoint at the daemon's own rate for a bounded time, then stop.
 *
 * Bounded on purpose. A one-shot publish goes stale in VESC_CMD_TO_MS and
 * could never sweep a servo; an unbounded one would leave a vehicle driving
 * after the command returned. Stopping also demonstrates the failsafe, which
 * is the behaviour most worth seeing on a bench.
 */

static int do_set(int argc, FAR char *argv[])
{
  struct actuator_command_s message;
  int32_t rate;
  double seconds = 2.0;
  int pub;
  int period_us;
  int ticks;
  int i;

  if (argc < 5)
    {
      usage();
      return EXIT_FAILURE;
    }

  memset(&message, 0, sizeof(message));

  if (strcmp(argv[2], "duty") == 0)
    {
      message.mode = ACTUATOR_MODE_DUTY;
    }
  else if (strcmp(argv[2], "current") == 0)
    {
      message.mode = ACTUATOR_MODE_CURRENT;
    }
  else
    {
      printf("vesc: mode must be duty or current\n");
      return EXIT_FAILURE;
    }

  message.motor = strtof(argv[3], NULL);
  message.steering = strtof(argv[4], NULL);

  if (argc >= 6)
    {
      seconds = strtod(argv[5], NULL);
    }

  if (!(seconds > 0.0 && seconds <= 30.0))
    {
      printf("vesc: seconds must be in 0..30\n");
      return EXIT_FAILURE;
    }

  if (!isfinite(message.motor) || !isfinite(message.steering))
    {
      printf("vesc: motor and steering must be finite\n");
      return EXIT_FAILURE;
    }

  rate = param_i32("VESC_TX_RATE");

  if (rate < 1)
    {
      rate = 50;
    }

  pub = actuator_command_advertise();

  if (pub < 0)
    {
      printf("vesc: cannot advertise actuator_command (%d)\n", errno);
      return EXIT_FAILURE;
    }

  period_us = 1000000 / (int)rate;
  ticks = (int)(seconds * (double)rate);

  printf("vesc: publishing %s %.3f steering %.3f for %.1f s\n",
         argv[2], (double)message.motor, (double)message.steering, seconds);

  for (i = 0; i < ticks; i++)
    {
      message.timestamp = 0;    /* the daemon stamps it on arrival */
      actuator_command_publish(pub, &message);
      usleep(period_us);
    }

  orb_unadvertise(pub);
  printf("vesc: stopped publishing; failsafe takes over\n");
  return EXIT_SUCCESS;
}
```

Add the dispatch branches in `main`, before the final `usage()`:

```c
  if (strcmp(argv[1], "arm") == 0 || strcmp(argv[1], "disarm") == 0)
    {
      bool want = strcmp(argv[1], "arm") == 0;

      ret = vesc_arm(want);

      if (ret == -ESRCH)
        {
          printf("vesc: not running\n");
          return EXIT_FAILURE;
        }

      if (ret == -EPERM)
        {
          printf("vesc: refused - a live setpoint is commanding the motor.\n"
                 "      Stop the publisher, or wait %" PRIi32 " ms for it to"
                 " go stale.\n", param_i32("VESC_CMD_TO_MS"));
          return EXIT_FAILURE;
        }

      printf("vesc: %s\n", want ? "ARMED" : "disarmed");
      return EXIT_SUCCESS;
    }

  if (strcmp(argv[1], "set") == 0)
    {
      return do_set(argc, argv);
    }
```

- [ ] **Step 6: Add `vesc_cmd.c` to the app build**

In `apps/vesc/Makefile`, change the `CSRCS` line and its comment:

```make
# vesc_proto.c is the wire format and vesc_cmd.c is the command policy;
# neither does I/O, so both are host-tested. vesc.c is the daemon;
# vesc_main.c is the command.
CSRCS   = vesc.c vesc_proto.c vesc_cmd.c
MAINSRC = vesc_main.c
```

- [ ] **Step 7: Build and verify**

```bash
RECONFIGURE=1 ./tools/build.sh 2>&1 | tail -3
./tools/verify.sh 2>&1 | grep -E "FAIL|exited|rebuilt|over 80"
```

Expected: build ends `>> done: .../build/xxcar.px4`; `verify.sh` shows
`test-vesc-proto PASS`, `test-vesc-cmd PASS`, `test-fdcan-ram PASS`,
`build.sh exited 0`, `lines over 80 columns: 0`, and only
`test-cpu-runtime FAIL`, which is the pre-existing hardware-only test.

- [ ] **Step 8: Commit**

```bash
git add apps/param/param.c apps/vesc/vesc.h apps/vesc/vesc.c \
        apps/vesc/vesc_main.c apps/vesc/Makefile
git commit -m "vesc: transmit commands from actuator_command

The daemon subscribes to actuator_command, resolves it through the policy
each transmit period, and sends one combined frame at VESC_TX_RATE.

The rate is its own deadline, not topic arrival. A publisher that stops,
stutters or runs at 400 Hz should not change what the bus sees, and a steady
heartbeat is what keeps the VESC's own command timeout from firing during
normal operation.

Transmit is suppressed entirely while VESC_CAN_ID is 0. That is discovery
mode, where the controller id would be a guess, and commanding a node picked
at random is worse than not commanding at all.

A setpoint with a zero or future timestamp is stamped on arrival. Age is
unsigned, so now minus a future stamp underflows to something enormous and
would read as permanently stale - the same trap the external-pose path hit.

`vesc set` publishes for a bounded time rather than once or forever. A single
publish goes stale in VESC_CMD_TO_MS and could never sweep a servo; an
unbounded one would leave the vehicle driving after the command returned.
Stopping at the end also demonstrates the failsafe, which is the behaviour
most worth seeing on a bench.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Hardware verification

**Files:** none. This task produces a record, not code.

Nothing here is optional and the order matters: each step is what makes the
next one safe.

- [ ] **Step 1: Set the node id**

Transmit does nothing while `VESC_CAN_ID` is 0. Use the id the discovery run
reported:

```
param set VESC_CAN_ID <id>
param save
reboot
```

- [ ] **Step 2: Wheels off the ground. Confirm the frames leave**

```
vesc start
vesc status
```

Wait two seconds, run `vesc status` again and compare.

Expected: `tx sent` climbing by roughly 100 over two seconds at the default
50 Hz, `bus_full` at zero, `last disarmed`, `motor 0.000`, `servo 1500 us`.

If `bus_full` climbs instead of `sent`, nothing on the bus is acknowledging.
Check termination and wiring before going further — no later step will work.

- [ ] **Step 3: Trim the steering, disarmed**

The servo should be holding trim. Adjust until the wheels are straight:

```
param set VESC_STEER_TRIM 1490
param save
vesc stop
vesc start
```

The daemon reads limits once at start, so a parameter change needs a
restart.

- [ ] **Step 4: Sweep the steering endpoints**

Disarmed output is neutral on both axes, so checking the endpoints needs
arming. Arm with the motor at zero:

```
vesc arm
vesc set duty 0 1.0 3
vesc set duty 0 -1.0 3
```

Watch for the linkage binding at full travel. If it binds, reduce
`VESC_STEER_MAX` or `VESC_STEER_MIN` before trusting either. If the vehicle
steers the wrong way, swap `VESC_STEER_MIN` and `VESC_STEER_MAX` — that is
what the reversed case is for.

- [ ] **Step 5: The motor, wheels still off the ground**

```
vesc arm
vesc set duty 0.05 0 3
```

Expected: the motor turns, and `tachometer` in `uorb_listener vesc_status`
moves in the direction a positive duty should produce. If it turns the wrong
way, that is a VESC-side configuration matter, not something to negate here.

- [ ] **Step 6: Observe the failsafe firing**

This is the step worth doing twice. A failsafe nobody has watched fire is a
failsafe nobody knows the state of.

First, by timeout — `vesc set` stops publishing on its own at the end:

```
vesc set duty 0.05 0.5 3
vesc status
```

Expected: the motor stops and the steering returns to trim within
`VESC_CMD_TO_MS` of the command ending, and `reasons stale` is now non-zero.

Then, by disarming while a setpoint is live. Run `vesc set duty 0.05 0 10`
and, in another session or before it finishes, `vesc disarm`.

Expected: output goes neutral within one transmit period, and
`reasons disarmed` climbs.

- [ ] **Step 7: Confirm the arm gate refuses**

With `vesc set duty 0.05 0 10` running, and the daemon disarmed:

```
vesc arm
```

Expected: refused, with the message naming the live setpoint. This is the
accident the gate exists for.

- [ ] **Step 8: Only now, on the ground**

Start at `VESC_DUTY_MAX` 0.30 and raise it only after the vehicle has driven
and stopped under command.

- [ ] **Step 9: Record what happened**

Append the observed frame rate, the trimmed steering values and anything
surprising to the "Known limitations" section of the spec, and commit. The
next person to touch this reads the spec, not the bench.

---

## Known gaps this plan does not close

- **Bus-off is reported, not recovered.** Clearing it needs an INIT cycle and
  a policy about when retrying is safe. Reporting first means that policy
  gets written against observed behaviour rather than guessed.
- **Limits are read once at start.** Changing a `VESC_*` parameter needs
  `vesc stop; vesc start`. Step 3 of Task 6 says so explicitly.
- **No steering feedback loop.** The ADC voltage arrives on `vesc_status` and
  nothing compares it to the command.
- **`VESC_EN` still defaults to 0.** Boot integration was deferred with the
  receive work and stays deferred; the transmit path makes that more
  obviously right, not less.
