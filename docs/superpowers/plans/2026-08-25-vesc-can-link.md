# VESC CAN Link Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring up FDCAN1 at 1 Mbit/s and receive VESC telemetry — tachometer, filtered motor current, and the ADC1 voltage that is the steering feedback.

**Architecture:** A direct FDCAN peripheral driver in `boards/fmuv6c/src`, following PX4 rather than NuttX — PX4 drives the peripheral itself and enables no network stack, and this tree already contains hand-written drivers for the same reason. A separate host-tested codec decodes the big-endian payloads, and a daemon publishes `vesc_status`. Receive only: no transmit path exists after this work.

**Tech Stack:** C11, NuttX, STM32H7 FDCAN, uORB.

**Spec:** `docs/superpowers/specs/2026-08-25-vesc-can-link-design.md`

## Global Constraints

- **Receive only.** No TX path, no command frames, no motor or servo output. The message RAM reserves a TX FIFO so adding it later moves nothing, but none of it is written.
- **Payloads are BIG-ENDIAN.** The MCU is little-endian. This is the highest-risk detail in the subsystem: a byte-order mistake on the `int32` tachometer is obvious, but on the signed `int16` current it is *occasionally plausible*, which is worse.
- **Classic CAN only**, 8-byte frames. CAN FD would change the message RAM element size from 4 words to 18 and invalidate the whole layout table.
- **Bit timing is fixed and derived**, not searched at runtime: 16 MHz FDCAN clock, 1 Mbit/s → `NBRP=0, NTSEG1=12, NTSEG2=1, NSJW=0`, sample point 87.5%. Cross-checked against PX4's solver.
- **`FDCAN_NBTP_NTSEG2_SHIFT` does not exist in NuttX's header.** `stm32_fdcan.h` defines NTSEG1, NBRP and NSJW but omits NTSEG2. The driver defines it: bits [6:0] of NBTP, shift 0.
- **Message RAM start-address fields are shifted by 2**, which converts a word offset to the byte address the hardware wants. Write `word_offset << FDCAN_*_SHIFT`, exactly as PX4 does.
- **uORB topic names are limited to 20 characters** — `/dev/uorb/<name><instance>` must fit `NAME_MAX` (32). Enforce with `ORB_NAME_FITS()`.
- **Every new uORB struct needs `static_assert` offset and size checks.** A `uint64_t` first member forces 8-byte alignment; declare padding explicitly.
- **`PARAM_NAME_MAX` is 16**, including the NUL.
- **C style:** NuttX kernel style as used throughout — two-space indent, braces on their own line, `FAR` on pointer parameters, `/* */` comments only, lines ≤ 80 columns (`verify.sh` checks).
- **Host tests compile with** `-std=c11 -Wall -Wextra -Werror -DFAR=`.
- **No dynamic allocation.**
- **Do not modify** `apps/ekf3/`, `apps/companion/`, or any existing driver in `boards/fmuv6c/src/`.
- **Commit after every task**, trailer `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`.
- **`tools/verify.sh` is the gate.** `test-cpu-runtime` is a known pre-existing failure and must not be "fixed" here.

## File Structure

**Created:**

| File | Responsibility |
|---|---|
| `apps/vesc/vesc_proto.h` / `.c` | Pure codec: 29-bit ID split, `STATUS_5` decode. No I/O, no uORB, no hardware. |
| `apps/vesc/vesc.h` / `.c` | Daemon: drain frames, decode, publish, count what it sees. |
| `apps/vesc/vesc_main.c` | `vesc start \| stop \| status` |
| `apps/vesc/Makefile`, `Make.defs`, `Kconfig` | Build wiring, following `apps/companion`. |
| `boards/fmuv6c/src/fdcan.h` / `.c` | The peripheral. Clock, pins, bit timing, message RAM, filters, FIFO drain. Knows nothing about VESCs. |
| `tests/vesc_proto_test.c`, `tools/test-vesc-proto.sh` | Codec tests. |

**Modified:**

| File | Change |
|---|---|
| `apps/uorb_msgs/uorb_msgs.h` / `.c` | `vesc_status` topic. |
| `apps/param/param.c` | `VESC_EN`, `VESC_CAN_ID`, `VESC_BITRATE`. |
| `boards/fmuv6c/include/board.h` | `GPIO_CAN1_RX` PD0, `GPIO_CAN1_TX` PD1. |
| `boards/fmuv6c/configs/nsh/defconfig` | `CONFIG_STM32H7_FDCAN1`. |
| `boards/fmuv6c/src/Makefile` | Build `fdcan.c`. |
| `boards/fmuv6c/src/stm32_bringup.c` | Start the daemon when `VESC_EN` is set. |

The codec being free of I/O is what makes the byte order testable without hardware, and the byte order is where the bugs are.

---

### Task 1: The codec

Written first and tested on the host, because it is the only part of this subsystem that *can* be tested without a VESC on a bench.

**Files:**
- Create: `apps/vesc/vesc_proto.h`, `apps/vesc/vesc_proto.c`
- Create: `tests/vesc_proto_test.c`, `tools/test-vesc-proto.sh`

**Interfaces:**
- Consumes: nothing.
- Produces: `VESC_PACKET_STATUS_5`, `struct vesc_status5_s`, `uint8_t vesc_packet_id(uint32_t can_id)`, `uint8_t vesc_controller_id(uint32_t can_id)`, `bool vesc_decode_status5(FAR const uint8_t *data, uint8_t dlc, FAR struct vesc_status5_s *out)`.

- [ ] **Step 1: Write the header**

Create `apps/vesc/vesc_proto.h`:

```c
/****************************************************************************
 * apps/vesc/vesc_proto.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * VESC CAN payloads. See docs/can_packet.md.
 *
 *   29-bit extended ID = (packet_id << 8) | controller_id
 *
 * Every payload is BIG-ENDIAN, and this MCU is not. That is the whole
 * reason this file exists separately from the driver and the daemon: a byte
 * order mistake on the int32 tachometer is obvious - wrong by millions - but
 * on the signed int16 current it is occasionally PLAUSIBLE, which is far
 * worse. Being testable on a host is what catches that.
 *
 * No I/O, no uORB, no hardware.
 ****************************************************************************/

#ifndef __APPS_VESC_VESC_PROTO_H
#define __APPS_VESC_VESC_PROTO_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

/* Telemetry, VESC -> host. */

#define VESC_PACKET_STATUS_5          0x1b

/* Commands, host -> VESC. Defined so the discovery listing can name what it
 * sees; nothing here transmits.
 */

#define VESC_PACKET_SET_DUTY          0x00
#define VESC_PACKET_SET_CURRENT       0x01
#define VESC_PACKET_PROCESS_SHORT_BUF 0x08
#define VESC_PACKET_SET_CURRENT_SERVO 0x45
#define VESC_PACKET_SET_DUTY_SERVO    0x46

#define VESC_STATUS_5_DLC             8

/* Decoded CAN_PACKET_STATUS_5.
 *
 * The tachometer is an accumulated POSITION count, not a rate. Turning it
 * into a speed is a consumer's job.
 */

struct vesc_status5_s
{
  int32_t tachometer;   /* accumulated counts */
  float   current_a;    /* A, raw / 10 */
  float   adc_volts;    /* V, raw / 1000 - the steering feedback */
};

/* The two halves of the 29-bit identifier. */

uint8_t vesc_packet_id(uint32_t can_id);
uint8_t vesc_controller_id(uint32_t can_id);

/* Decode a STATUS_5 payload. Returns false when the DLC is not what this
 * packet is defined to carry - which is a firmware mismatch, and a different
 * thing from an unknown packet id.
 */

bool vesc_decode_status5(FAR const uint8_t *data, uint8_t dlc,
                         FAR struct vesc_status5_s *out);

#endif /* __APPS_VESC_VESC_PROTO_H */
```

- [ ] **Step 2: Write the failing test**

Create `tests/vesc_proto_test.c`:

```c
/****************************************************************************
 * tests/vesc_proto_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "vesc_proto.h"

#define CLOSE(a, b) (fabsf((a) - (b)) < 1.0e-4f)

/* The identifier is (packet_id << 8) | controller_id. Both halves are a full
 * byte, so 0xFF has to work in each - a mask that is one bit short passes
 * every ordinary test and fails on exactly one node id.
 */

static void test_id_split(void)
{
  assert(vesc_packet_id(0x1b4a) == 0x1b);
  assert(vesc_controller_id(0x1b4a) == 0x4a);

  assert(vesc_packet_id(0x0000) == 0x00);
  assert(vesc_controller_id(0x0000) == 0x00);

  assert(vesc_packet_id(0xffff) == 0xff);
  assert(vesc_controller_id(0xffff) == 0xff);

  /* Bits above 16 belong to neither field and must not leak into either. */

  assert(vesc_packet_id(0x1fff1b4a) == 0x1b);
  assert(vesc_controller_id(0x1fff1b4a) == 0x4a);
}

/* Hand-built BIG-ENDIAN payload, values chosen so a byte swap cannot
 * accidentally produce the right answer.
 *
 *   tachometer 0x0002CA7B = 182395
 *   current    0x0019     = 25    -> 2.5 A
 *   adc        0x067E     = 1662  -> 1.662 V
 */

static void test_status5_decode(void)
{
  const uint8_t data[8] =
  {
    0x00, 0x02, 0xca, 0x7b,
    0x00, 0x19,
    0x06, 0x7e
  };
  struct vesc_status5_s out;

  assert(vesc_decode_status5(data, VESC_STATUS_5_DLC, &out));
  assert(out.tachometer == 182395);
  assert(CLOSE(out.current_a, 2.5f));
  assert(CLOSE(out.adc_volts, 1.662f));
}

/* Negative current. THE test this file exists for.
 *
 * Regenerative braking makes this an ordinary reading, not an edge case, and
 * sign extension across a byte-swapped int16 is exactly where a decoder
 * breaks while still producing a number that looks like a current.
 *
 *   0xFFE7 = -25 -> -2.5 A
 */

static void test_status5_negative_current(void)
{
  const uint8_t data[8] =
  {
    0x00, 0x00, 0x00, 0x00,
    0xff, 0xe7,
    0x00, 0x00
  };
  struct vesc_status5_s out;

  assert(vesc_decode_status5(data, VESC_STATUS_5_DLC, &out));
  assert(CLOSE(out.current_a, -2.5f));
  assert(out.current_a < 0.0f);
}

/* Negative tachometer - the motor turned backwards past the origin.
 *
 *   0xFFFD3585 = -182395
 */

static void test_status5_negative_tachometer(void)
{
  const uint8_t data[8] =
  {
    0xff, 0xfd, 0x35, 0x85,
    0x00, 0x00,
    0x00, 0x00
  };
  struct vesc_status5_s out;

  assert(vesc_decode_status5(data, VESC_STATUS_5_DLC, &out));
  assert(out.tachometer == -182395);
}

/* The extremes of both signed fields, so a decoder that is right in the
 * middle of the range and wrong at the ends is caught.
 */

static void test_status5_extremes(void)
{
  const uint8_t max_data[8] =
  {
    0x7f, 0xff, 0xff, 0xff,
    0x7f, 0xff,
    0xff, 0xff
  };
  const uint8_t min_data[8] =
  {
    0x80, 0x00, 0x00, 0x00,
    0x80, 0x00,
    0x00, 0x00
  };
  struct vesc_status5_s out;

  assert(vesc_decode_status5(max_data, VESC_STATUS_5_DLC, &out));
  assert(out.tachometer == 2147483647);
  assert(CLOSE(out.current_a, 3276.7f));
  assert(CLOSE(out.adc_volts, 65.535f));

  assert(vesc_decode_status5(min_data, VESC_STATUS_5_DLC, &out));
  assert(out.tachometer == -2147483647 - 1);
  assert(CLOSE(out.current_a, -3276.8f));
  assert(CLOSE(out.adc_volts, 0.0f));
}

/* The scale factors are asserted against docs/can_packet.md - current is
 * raw/10 and the ADC is raw/1000 - rather than against whatever the code
 * happens to do. A test that agrees with the implementation tests nothing.
 */

static void test_status5_scales(void)
{
  const uint8_t data[8] =
  {
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x0a,          /* 10 raw */
    0x03, 0xe8           /* 1000 raw */
  };
  struct vesc_status5_s out;

  assert(vesc_decode_status5(data, VESC_STATUS_5_DLC, &out));
  assert(CLOSE(out.current_a, 1.0f));    /* 10 / 10 */
  assert(CLOSE(out.adc_volts, 1.0f));    /* 1000 / 1000 */
}

/* A short frame must be REFUSED, not read past. The payload buffer is eight
 * bytes because the hardware element is; reading a 6-byte frame as STATUS_5
 * would decode two bytes of whatever the previous frame left behind.
 */

static void test_status5_wrong_dlc(void)
{
  const uint8_t data[8] = {0};
  struct vesc_status5_s out;

  assert(!vesc_decode_status5(data, 6, &out));
  assert(!vesc_decode_status5(data, 0, &out));
  assert(!vesc_decode_status5(data, 7, &out));
  assert(vesc_decode_status5(data, 8, &out));
}

static void test_status5_null_guards(void)
{
  const uint8_t data[8] = {0};
  struct vesc_status5_s out;

  assert(!vesc_decode_status5(NULL, VESC_STATUS_5_DLC, &out));
  assert(!vesc_decode_status5(data, VESC_STATUS_5_DLC, NULL));
}

int main(void)
{
  test_id_split();
  test_status5_decode();
  test_status5_negative_current();
  test_status5_negative_tachometer();
  test_status5_extremes();
  test_status5_scales();
  test_status5_wrong_dlc();
  test_status5_null_guards();

  puts("vesc_proto: big-endian decode, sign extension and DLC verified - OK");
  return 0;
}
```

- [ ] **Step 3: Write the runner**

Create `tools/test-vesc-proto.sh`:

```bash
#!/usr/bin/env bash
# Host-side test for the VESC CAN codec.
#
# The payloads are big-endian and this MCU is not. A sign-extension mistake
# across a byte-swapped int16 produces a current that is wrong and still
# looks like a current, which is why the negative and extreme cases are here
# rather than left to a bench.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -I"$REPO/apps/vesc" \
   -o "$OUT/test" "$REPO/tests/vesc_proto_test.c" \
   "$REPO/apps/vesc/vesc_proto.c" -lm
"$OUT/test"

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -fsanitize=undefined,address -fno-sanitize-recover=all \
   -I"$REPO/apps/vesc" \
   -o "$OUT/test-san" "$REPO/tests/vesc_proto_test.c" \
   "$REPO/apps/vesc/vesc_proto.c" -lm
"$OUT/test-san"
```

Then `chmod +x tools/test-vesc-proto.sh`.

The sanitiser pass matters here: a decoder indexing a payload from a
wire-supplied DLC is exactly where an overrun lives, and ASan is what proves
the bound is real rather than assumed.

- [ ] **Step 4: Run to verify it fails**

```bash
tools/test-vesc-proto.sh
```

Expected: FAIL at compile — `vesc_proto.c: No such file or directory`.

- [ ] **Step 5: Implement**

Create `apps/vesc/vesc_proto.c`:

```c
/****************************************************************************
 * apps/vesc/vesc_proto.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include "vesc_proto.h"

/* Assembled byte by byte rather than by casting the buffer and swapping.
 *
 * A cast to a wider type through a byte pointer is an alignment assumption
 * the CAN payload does not owe us, and a swap intrinsic hides which end is
 * which. Shifting each byte into place says what the wire format is.
 */

static int32_t vesc_be32(FAR const uint8_t *d)
{
  uint32_t v = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
               ((uint32_t)d[2] << 8)  | (uint32_t)d[3];

  return (int32_t)v;
}

static int16_t vesc_be16(FAR const uint8_t *d)
{
  uint16_t v = (uint16_t)(((uint16_t)d[0] << 8) | (uint16_t)d[1]);

  return (int16_t)v;
}

uint8_t vesc_packet_id(uint32_t can_id)
{
  return (uint8_t)((can_id >> 8) & 0xff);
}

uint8_t vesc_controller_id(uint32_t can_id)
{
  return (uint8_t)(can_id & 0xff);
}

bool vesc_decode_status5(FAR const uint8_t *data, uint8_t dlc,
                         FAR struct vesc_status5_s *out)
{
  if (data == NULL || out == NULL || dlc != VESC_STATUS_5_DLC)
    {
      return false;
    }

  out->tachometer = vesc_be32(&data[0]);
  out->current_a = (float)vesc_be16(&data[4]) / 10.0f;
  out->adc_volts = (float)vesc_be16(&data[6]) / 1000.0f;
  return true;
}
```

- [ ] **Step 6: Run to verify it passes**

```bash
tools/test-vesc-proto.sh
```

Expected: `vesc_proto: big-endian decode, sign extension and DLC verified - OK`, twice.

If only `test_status5_negative_current` fails, the `int16_t` cast is being
done on the wrong width somewhere — re-read `vesc_be16`: the value must be
assembled as `uint16_t` and cast once, not shifted into an `int`.

- [ ] **Step 7: Commit**

```bash
git add apps/vesc/vesc_proto.h apps/vesc/vesc_proto.c \
        tests/vesc_proto_test.c tools/test-vesc-proto.sh
git commit -m "vesc: add the CAN telemetry codec

Big-endian payloads on a little-endian MCU. A byte-order mistake on the
int32 tachometer is obvious - wrong by millions - but on the signed int16
current it is occasionally plausible, which is worse, so negative and
extreme values are tested rather than left to a bench.

Bytes are shifted into place rather than cast and swapped: a cast through a
byte pointer assumes an alignment the payload does not owe us, and a swap
intrinsic hides which end is which.

A wrong DLC is refused rather than read past - the buffer is eight bytes
because the hardware element is, so decoding a short frame would read
whatever the previous one left behind.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: The `vesc_status` topic

**Files:**
- Modify: `apps/uorb_msgs/uorb_msgs.h`, `apps/uorb_msgs/uorb_msgs.c`

**Interfaces:**
- Consumes: nothing (the topic is independent of the wire struct).
- Produces: `struct vesc_status_s`, `ORB_ID(vesc_status)`, `int vesc_status_advertise(void)`, `int vesc_status_publish(int fd, FAR const struct vesc_status_s *msg)`.

- [ ] **Step 1: Add the struct**

In `apps/uorb_msgs/uorb_msgs.h`, after `struct external_pose_s`:

```c
/* VESC telemetry, in RAW units.
 *
 * No steering angle: converting adc_volts to an angle is a calibration with
 * its own zero and scale, and baking a guess into the topic is the mistake
 * SENS_MAG0_ROT already made once in this tree.
 *
 * The tachometer is an accumulated POSITION count, not a rate.
 */

struct vesc_status_s
{
  uint64_t timestamp;             /*  0: us, publication time */
  uint64_t timestamp_sample;      /*  8: us, when the frame was drained */
  int32_t  tachometer;            /* 16: accumulated counts */
  float    current_a;             /* 20: A, filtered total */
  float    adc_volts;             /* 24: V, ADC1 - steering feedback */
  uint8_t  controller_id;         /* 28: which VESC */
  uint8_t  pad[3];                /* 29 */
};
```

Add `ORB_DECLARE(vesc_status);` to the declare block and the prototypes:

```c
int vesc_status_advertise(void);
int vesc_status_publish(int fd, FAR const struct vesc_status_s *msg);
```

- [ ] **Step 2: Add the assertions and definition**

In `apps/uorb_msgs/uorb_msgs.c`, add `ORB_NAME_FITS("vesc_status");` beside the others, then after the `external_pose_s` assertions:

```c
static_assert(offsetof(struct vesc_status_s, timestamp)        ==  0, "layout");
static_assert(offsetof(struct vesc_status_s, timestamp_sample) ==  8, "layout");
static_assert(offsetof(struct vesc_status_s, tachometer)       == 16, "layout");
static_assert(offsetof(struct vesc_status_s, current_a)        == 20, "layout");
static_assert(offsetof(struct vesc_status_s, adc_volts)        == 24, "layout");
static_assert(offsetof(struct vesc_status_s, controller_id)    == 28, "layout");
static_assert(sizeof(struct vesc_status_s)                     == 32, "layout");
```

Format string, inside the `CONFIG_DEBUG_UORB` block:

```c
static const char vesc_status_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",tachometer:%" PRIi32
  ",current_a:%hf,adc_volts:%hf"
  ",controller_id:%hhu";
```

Definition and accessors, copying `external_pose_publish`'s exact guard
(`fd < 0 || msg == NULL`):

```c
ORB_DEFINE(vesc_status, struct vesc_status_s, vesc_status_format);

int vesc_status_advertise(void)
{
  return orb_advertise(ORB_ID(vesc_status), NULL);
}

int vesc_status_publish(int fd, FAR const struct vesc_status_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(vesc_status), fd, msg);
}
```

- [ ] **Step 3: Prove the assertions run**

Change `sizeof(struct vesc_status_s) == 32` to `== 33` and:

```bash
./tools/build.sh 2>&1 | grep -i layout
```

Expected: build FAILS naming that line. Revert.

- [ ] **Step 4: Build and commit**

```bash
./tools/build.sh
git add apps/uorb_msgs/uorb_msgs.h apps/uorb_msgs/uorb_msgs.c
git commit -m "uorb_msgs: define the vesc_status topic

Raw units. No steering angle: converting the ADC voltage to an angle is a
calibration with its own zero and scale, and baking a guess into the topic
is the mistake SENS_MAG0_ROT already made once here.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Pins, parameters and configuration

**Files:**
- Modify: `boards/fmuv6c/include/board.h`, `boards/fmuv6c/configs/nsh/defconfig`
- Modify: `apps/param/param.c`, `tests/param_range_test.c`

**Interfaces:**
- Produces: `GPIO_CAN1_RX`, `GPIO_CAN1_TX`; parameters `VESC_EN`, `VESC_CAN_ID`, `VESC_BITRATE`.

- [ ] **Step 1: Write the failing test**

In `tests/param_range_test.c`, add above `main()` and register the call:

```c
/* The VESC link parameters.
 *
 * VESC_EN defaults OFF. A new driver touching a new peripheral does not
 * belong in the boot path until it has run at least once.
 *
 * VESC_CAN_ID defaults to 0 meaning accept-any, which is what makes the
 * first run a discovery rather than a guess.
 */

static void test_vesc_parameters(void)
{
  int32_t v;

  if (param_find("VESC_EN") < 0 ||
      param_find("VESC_CAN_ID") < 0 ||
      param_find("VESC_BITRATE") < 0)
    {
      fail("VESC schema is incomplete");
      return;
    }

  if (param_get_i32("VESC_EN", &v) < 0 || v != 0)
    {
      fail("VESC_EN does not default to off");
    }

  if (param_get_i32("VESC_CAN_ID", &v) < 0 || v != 0)
    {
      fail("VESC_CAN_ID does not default to accept-any");
    }

  if (param_get_i32("VESC_BITRATE", &v) < 0 || v != 1000000)
    {
      fail("VESC_BITRATE does not default to 1 Mbit/s");
    }

  /* A controller id is one byte on the wire. 256 cannot be expressed and
   * must be refused rather than truncated to 0, which would silently become
   * accept-any.
   */

  if (param_set_i32("VESC_CAN_ID", 256) != -ERANGE ||
      param_get_i32("VESC_CAN_ID", &v) < 0 || v != 255)
    {
      fail("VESC_CAN_ID did not clamp to a single byte");
    }

  if (param_set_i32("VESC_CAN_ID", 0) < 0)
    {
      fail("could not restore VESC_CAN_ID");
    }
}
```

- [ ] **Step 2: Run to verify it fails**

```bash
tools/test-param-range.sh
```

Expected: FAIL — `VESC schema is incomplete`.

- [ ] **Step 3: Add the parameters**

In `apps/param/param.c`, after the external-navigation block:

```c
  /* ---- VESC CAN link ----------------------------------------------------
   * VESC_EN is OFF by default. A new driver touching a new peripheral does
   * not belong in the boot path until it has run at least once.
   *
   * VESC_CAN_ID 0 means accept ANY controller id, which is what makes the
   * first run a discovery rather than a guess. Setting it to the id that
   * turns up narrows the HARDWARE filter, which matters more than it looks:
   * an unfiltered 1 Mbit/s bus can deliver over 8000 frames a second, and
   * discarding them in a task means waking up 8000 times a second to throw
   * work away.
   */

  { "VESC_EN", PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Start the VESC CAN link at boot" },
  { "VESC_CAN_ID", PARAM_TYPE_INT32, I32(0), I32(0), I32(255),
    "VESC controller id to accept (0 = any)" },
  { "VESC_BITRATE", PARAM_TYPE_INT32, I32(1000000), I32(125000),
    I32(1000000), "CAN bitrate (bit/s)" },
```

- [ ] **Step 4: Add the pins**

In `boards/fmuv6c/include/board.h`, beside the other GPIO alternate-function
definitions:

```c
/* FDCAN1 on the CAN1 connector.
 *
 * PD0/PD1 is the fmu-v6c assignment PX4 uses. The FDCAN kernel clock is
 * already selected as HSE above; these are the only pieces that were
 * missing.
 */

#define GPIO_CAN1_RX  GPIO_CAN1_RX_3   /* PD0 */
#define GPIO_CAN1_TX  GPIO_CAN1_TX_3   /* PD1 */
```

- [ ] **Step 5: Enable the peripheral**

In `boards/fmuv6c/configs/nsh/defconfig`:

```
# FDCAN1 for the VESC link. The peripheral only - NuttX's own CAN driver
# stays off, because the only one it ships for the H7 is SocketCAN and that
# needs CONFIG_NET. PX4 does the same on this silicon: their fmu-v6c has no
# CONFIG_NET, no NET_CAN and no CAN, and drives FDCAN directly.
CONFIG_STM32H7_FDCAN1=y
```

- [ ] **Step 6: Run to verify it passes**

```bash
tools/test-param-range.sh && RECONFIGURE=1 ./tools/build.sh
```

Expected: PASS, build exits 0.

Confirm NuttX's own CAN driver did *not* come along for the ride:

```bash
grep -cE "^CONFIG_(NET|CAN)=y" deps/nuttx/.config
```

Expected: `0`. If either appears, `STM32H7_FDCAN1` has selected something
that pulls the network stack in, and that needs understanding before
continuing — the whole point of this approach is that it does not.

- [ ] **Step 7: Commit**

```bash
git add apps/param/param.c tests/param_range_test.c \
        boards/fmuv6c/include/board.h boards/fmuv6c/configs/nsh/defconfig
git commit -m "board: enable FDCAN1 and add the VESC link parameters

The peripheral only. NuttX's own CAN driver stays off: the only one it ships
for the H7 is SocketCAN, which needs CONFIG_NET, and PX4 does not use it
either - their fmu-v6c has no CONFIG_NET, no NET_CAN and no CAN at all.

PD0/PD1 is PX4's fmu-v6c assignment. The FDCAN kernel clock was already
selected as HSE in board.h; the pins were the missing piece.

VESC_CAN_ID 0 means accept any, which makes the first run a discovery rather
than a guess, and it is clamped to one byte because that is what the
identifier carries - 256 truncating to 0 would silently mean accept-any.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: The FDCAN driver

The fiddliest part, and the one that cannot be tested on a host. Everything
here is verified against real bus traffic in Task 6.

**Files:**
- Create: `boards/fmuv6c/src/fdcan.h`, `boards/fmuv6c/src/fdcan.c`
- Modify: `boards/fmuv6c/src/Makefile`

**Interfaces:**
- Consumes: `GPIO_CAN1_RX` / `GPIO_CAN1_TX` (Task 3).
- Produces: `struct fdcan_frame_s`, `struct fdcan_stats_s`, `int fdcan_init(uint32_t bitrate)`, `int fdcan_receive(FAR struct fdcan_frame_s *frame)`, `void fdcan_set_filter(uint8_t controller_id)`, `void fdcan_stats(FAR struct fdcan_stats_s *out)`.

- [ ] **Step 1: Write the header**

Create `boards/fmuv6c/src/fdcan.h`:

```c
/****************************************************************************
 * boards/fmuv6c/src/fdcan.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal FDCAN1 receiver.
 *
 * Written rather than taken from NuttX because the only H7 CAN driver NuttX
 * ships is SocketCAN, which needs CONFIG_NET - a whole network stack to
 * carry two message types. PX4 reached the same conclusion on the same
 * silicon and drives the peripheral directly.
 *
 * RECEIVE ONLY. There is no transmit path here at all, which is deliberate:
 * a driver that can only listen cannot spin a motor while the bit timing is
 * still being proven.
 *
 * Classic CAN, 8-byte frames. CAN FD would change the message RAM element
 * size from four words to eighteen and invalidate the layout in fdcan.c.
 ****************************************************************************/

#ifndef __BOARDS_FMUV6C_SRC_FDCAN_H
#define __BOARDS_FMUV6C_SRC_FDCAN_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

struct fdcan_frame_s
{
  uint32_t id;              /* 29-bit extended identifier */
  uint8_t  dlc;             /* 0..8 */
  uint8_t  data[8];
};

struct fdcan_stats_s
{
  uint32_t rx;              /* frames handed out */
  uint32_t lost;            /* RX FIFO0 overruns - we did not drain fast enough */
  uint32_t rejected;        /* non-extended or remote frames */
  uint8_t  last_error;      /* PSR LEC */
  bool     bus_off;
  bool     error_passive;
};

/* Bring up the peripheral. Only 1000000 is supported today; anything else is
 * refused rather than silently mis-timed.
 */

int fdcan_init(uint32_t bitrate);

/* Take one frame. Returns OK, or -EAGAIN when the FIFO is empty.
 *
 * Non-blocking and polled. Much less code than an interrupt path and
 * adequate for telemetry, but it puts the caller's poll interval as jitter
 * on the data - worth revisiting if a control loop ever depends on it.
 */

int fdcan_receive(FAR struct fdcan_frame_s *frame);

/* Narrow the HARDWARE filter to one controller id; 0 accepts any.
 *
 * In hardware rather than in software because an unfiltered 1 Mbit/s bus can
 * deliver over 8000 frames a second, and rejecting them in a task means
 * waking up 8000 times a second to throw work away.
 */

void fdcan_set_filter(uint8_t controller_id);

void fdcan_stats(FAR struct fdcan_stats_s *out);

#endif /* __BOARDS_FMUV6C_SRC_FDCAN_H */
```

- [ ] **Step 2: Implement**

Create `boards/fmuv6c/src/fdcan.c`:

```c
/****************************************************************************
 * boards/fmuv6c/src/fdcan.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "arm_internal.h"
#include "stm32_gpio.h"
#include "hardware/stm32_fdcan.h"
#include "hardware/stm32_rcc.h"

#include "fdcan.h"

/* NuttX's stm32_fdcan.h defines NTSEG1, NBRP and NSJW but NOT NTSEG2.
 * Reference manual: NBTP bits [6:0].
 */

#define FDCAN_NBTP_NTSEG2_SHIFT   (0U)

/* Bit timing for 1 Mbit/s from the 16 MHz HSE kernel clock.
 *
 *   tq per bit  = 16 MHz / 1 Mbit/s = 16
 *   prescaler   = 1                       -> NBRP  = 0
 *   bs1_bs2_sum = 15
 *   bs1 = (7 * 15 - 1) / 8 = 13           -> NTSEG1 = 12
 *   bs2 = 15 - 13         =  2            -> NTSEG2 = 1
 *   sjw = 1                               -> NSJW   = 0
 *
 * Registers hold each value MINUS ONE, which is why these look off by one.
 * Sample point is (1 + 13) / 16 = 87.5%, the CiA recommendation, and PX4's
 * generic solver produces exactly these values for this clock and rate.
 */

#define FDCAN_BITRATE_SUPPORTED   1000000u
#define FDCAN_NBRP_1M             0u
#define FDCAN_NTSEG1_1M           12u
#define FDCAN_NTSEG2_1M           1u
#define FDCAN_NSJW_1M             0u

/* Message RAM layout, in WORDS from the start of the shared RAM.
 *
 * The 2560 words are shared between FDCAN1 and FDCAN2 and every region's
 * start address is assigned by software. Two regions that overlap corrupt
 * each other's frames and present as bus errors, so this table is the
 * authority and nothing computes an offset at runtime.
 *
 * Following PX4's allocation, which gives each interface the lower half.
 * Elements are four words: two header plus eight data bytes.
 *
 * THE TX FIFO IS RESERVED THOUGH NOTHING TRANSMITS. Laying it out later
 * would move RX FIFO0's start address, and that is a silent corruption
 * rather than a build error.
 */

#define FDCAN_RAM_STDFILT_OFF     0u
#define FDCAN_RAM_STDFILT_N       128u        /* 128 words */
#define FDCAN_RAM_EXTFILT_OFF     128u
#define FDCAN_RAM_EXTFILT_N       64u         /* 128 words */
#define FDCAN_RAM_RXF0_OFF        256u
#define FDCAN_RAM_RXF0_N          64u         /* 256 words */
#define FDCAN_RAM_TXF_OFF         512u
#define FDCAN_RAM_TXF_N           32u         /* 128 words */
#define FDCAN_RAM_USED            640u        /* of 1280 for FDCAN1 */

#define FDCAN_RAM_WORD(n)         (STM32_CANRAM_BASE + ((n) * 4u))

/* Extended filter element, two words:
 *   word 0: EFEC[31:29] | EFID1[28:0]
 *   word 1: EFT[31:30]  | EFID2[28:0]
 */

#define FDCAN_EFEC_STORE_FIFO0    (1u << 29)
#define FDCAN_EFT_CLASSIC         (2u << 30)

/* GFC: reject everything that does not match, including remote frames.
 * ANFE/ANFS 3 = reject non-matching.
 */

#define FDCAN_GFC_REJECT_ALL      ((3u << FDCAN_GFC_ANFE_SHIFT) | \
                                   (3u << FDCAN_GFC_ANFS_SHIFT) | \
                                   FDCAN_GFC_RRFE | FDCAN_GFC_RRFS)

/* ANFE 0 = accept non-matching extended into FIFO0. Standard frames are
 * still rejected - every frame in this protocol is extended.
 */

#define FDCAN_GFC_ACCEPT_EXT      ((0u << FDCAN_GFC_ANFE_SHIFT) | \
                                   (3u << FDCAN_GFC_ANFS_SHIFT) | \
                                   FDCAN_GFC_RRFE | FDCAN_GFC_RRFS)

static struct fdcan_stats_s g_stats;
static bool g_ready;

static void fdcan_ram_clear(void)
{
  uint32_t i;

  /* Message RAM is NOT cleared by a peripheral reset and comes up holding
   * whatever was there before. An uninitialised filter element is a filter,
   * and it will happily accept or reject traffic nobody asked it to.
   */

  for (i = 0; i < FDCAN_RAM_USED; i++)
    {
      putreg32(0, FDCAN_RAM_WORD(i));
    }
}

void fdcan_set_filter(uint8_t controller_id)
{
  if (controller_id == 0)
    {
      /* Discovery: take every extended frame, whatever its id. */

      putreg32(0, STM32_FDCAN1_XIDFC);
      putreg32(FDCAN_GFC_ACCEPT_EXT, STM32_FDCAN1_GFC);
      return;
    }

  /* One classic filter: match the low byte, ignore the packet id above it,
   * so every packet type from this one node arrives and nothing else does.
   */

  putreg32(FDCAN_EFEC_STORE_FIFO0 | controller_id,
           FDCAN_RAM_WORD(FDCAN_RAM_EXTFILT_OFF));
  putreg32(FDCAN_EFT_CLASSIC | 0xffu,
           FDCAN_RAM_WORD(FDCAN_RAM_EXTFILT_OFF + 1));

  putreg32((1u << FDCAN_XIDFC_LSE_SHIFT) |
           (FDCAN_RAM_EXTFILT_OFF << FDCAN_XIDFC_FLESA_SHIFT),
           STM32_FDCAN1_XIDFC);
  putreg32(FDCAN_GFC_REJECT_ALL, STM32_FDCAN1_GFC);
}

int fdcan_init(uint32_t bitrate)
{
  uint32_t regval;
  int guard;

  /* Only the timing that has been derived and checked. A different bitrate
   * would need different NBTP values, and quietly using 1 Mbit/s timing for
   * a 500 kbit/s bus produces a link that half works.
   */

  if (bitrate != FDCAN_BITRATE_SUPPORTED)
    {
      return -ENOTSUP;
    }

  regval = getreg32(STM32_RCC_APB1HENR);
  regval |= RCC_APB1HENR_FDCANEN;
  putreg32(regval, STM32_RCC_APB1HENR);

  stm32_configgpio(GPIO_CAN1_RX);
  stm32_configgpio(GPIO_CAN1_TX);

  /* INIT is not immediate. Configuring before it latches silently does
   * nothing, which is the classic way to get a peripheral that looks
   * configured and is not.
   */

  regval = getreg32(STM32_FDCAN1_CCCR);
  regval |= FDCAN_CCCR_INIT;
  putreg32(regval, STM32_FDCAN1_CCCR);

  for (guard = 0; guard < 100000; guard++)
    {
      if ((getreg32(STM32_FDCAN1_CCCR) & FDCAN_CCCR_INIT) != 0)
        {
          break;
        }
    }

  if ((getreg32(STM32_FDCAN1_CCCR) & FDCAN_CCCR_INIT) == 0)
    {
      return -ETIMEDOUT;
    }

  regval = getreg32(STM32_FDCAN1_CCCR);
  regval |= FDCAN_CCCR_CCE;
  putreg32(regval, STM32_FDCAN1_CCCR);

  fdcan_ram_clear();

  putreg32((FDCAN_NSJW_1M   << FDCAN_NBTP_NSJW_SHIFT)   |
           (FDCAN_NBRP_1M   << FDCAN_NBTP_NBRP_SHIFT)   |
           (FDCAN_NTSEG1_1M << FDCAN_NBTP_NTSEG1_SHIFT) |
           (FDCAN_NTSEG2_1M << FDCAN_NBTP_NTSEG2_SHIFT),
           STM32_FDCAN1_NBTP);

  /* The start-address fields sit at bit 2, which turns a word offset into
   * the byte address the hardware wants. Writing a byte offset here would
   * place every region four times too far out.
   */

  putreg32((FDCAN_RAM_STDFILT_N << FDCAN_SIDFC_LSS_SHIFT) |
           (FDCAN_RAM_STDFILT_OFF << FDCAN_SIDFC_FLSSA_SHIFT),
           STM32_FDCAN1_SIDFC);

  putreg32((FDCAN_RAM_RXF0_N << FDCAN_RXF0C_F0S_SHIFT) |
           (FDCAN_RAM_RXF0_OFF << FDCAN_RXF0C_F0SA_SHIFT),
           STM32_FDCAN1_RXF0C);

  putreg32((FDCAN_RAM_TXF_N << FDCAN_TXBC_TFQS_SHIFT) |
           (FDCAN_RAM_TXF_OFF << FDCAN_TXBC_TBSA_SHIFT),
           STM32_FDCAN1_TXBC);

  /* RXESC 0 in every field: 8-byte data. Explicit rather than relying on
   * reset value, because it is what makes the element four words.
   */

  putreg32(0, STM32_FDCAN1_RXESC);

  fdcan_set_filter(0);

  regval = getreg32(STM32_FDCAN1_CCCR);
  regval &= ~FDCAN_CCCR_INIT;
  putreg32(regval, STM32_FDCAN1_CCCR);

  memset(&g_stats, 0, sizeof(g_stats));
  g_ready = true;
  return OK;
}

int fdcan_receive(FAR struct fdcan_frame_s *frame)
{
  uint32_t status;
  uint32_t index;
  uint32_t element;
  uint32_t w0;
  uint32_t w1;
  uint32_t i;

  if (!g_ready || frame == NULL)
    {
      return -EINVAL;
    }

  status = getreg32(STM32_FDCAN1_RXF0S);

  /* F0FL is the fill level. Anything else about this register is about
   * where, not whether.
   */

  if ((status & FDCAN_RXF0S_F0FL_MASK) == 0)
    {
      return -EAGAIN;
    }

  index = (status & FDCAN_RXF0S_F0GI_MASK) >> FDCAN_RXF0S_F0GI_SHIFT;
  element = FDCAN_RAM_RXF0_OFF + (index * 4u);

  w0 = getreg32(FDCAN_RAM_WORD(element));
  w1 = getreg32(FDCAN_RAM_WORD(element + 1));

  frame->id = w0 & 0x1fffffffu;
  frame->dlc = (uint8_t)((w1 >> 16) & 0xfu);

  if (frame->dlc > 8)
    {
      frame->dlc = 8;
    }

  for (i = 0; i < frame->dlc; i++)
    {
      uint32_t word = getreg32(FDCAN_RAM_WORD(element + 2 + (i / 4u)));

      frame->data[i] = (uint8_t)((word >> ((i % 4u) * 8u)) & 0xffu);
    }

  /* Acknowledge BEFORE deciding whether we want it: the FIFO slot has to be
   * released either way, and returning early without acknowledging wedges
   * the FIFO one frame at a time until it overruns.
   */

  putreg32(index, STM32_FDCAN1_RXF0A);

  /* Bit 30 is XTD, bit 29 is RTR. Every frame in this protocol is an
   * extended data frame; a standard or remote frame here is somebody else's
   * traffic.
   */

  if ((w0 & (1u << 30)) == 0 || (w0 & (1u << 29)) != 0)
    {
      g_stats.rejected++;
      return -EAGAIN;
    }

  g_stats.rx++;
  return OK;
}

void fdcan_stats(FAR struct fdcan_stats_s *out)
{
  uint32_t psr;

  if (out == NULL)
    {
      return;
    }

  psr = getreg32(STM32_FDCAN1_PSR);
  g_stats.last_error = (uint8_t)(psr & FDCAN_PSR_LEC_MASK);
  g_stats.bus_off = (psr & FDCAN_PSR_BO_MASK) != 0;
  g_stats.error_passive = (psr & FDCAN_PSR_EP_MASK) != 0;

  if ((getreg32(STM32_FDCAN1_RXF0S) & (1u << 25)) != 0)   /* RF0L */
    {
      g_stats.lost++;
    }

  *out = g_stats;
}
```

- [ ] **Step 3: Add to the board build**

In `boards/fmuv6c/src/Makefile`, add `fdcan.c` to `CSRCS` alongside the
existing drivers, guarded so it only builds when the peripheral is enabled:

```make
ifeq ($(CONFIG_STM32H7_FDCAN1),y)
CSRCS += fdcan.c
endif
```

Read the file first and match how `ist8310.c` and `ms5611.c` are added — if
they are unconditional, follow that and drop the guard.

- [ ] **Step 4: Build**

```bash
./tools/build.sh
```

Expected: exits 0, no warnings. A missing register name here means
`stm32_fdcan.h` calls it something else — grep the header rather than
guessing, as `NTSEG2` already showed.

- [ ] **Step 5: Verify the RAM layout arithmetic**

The one part that can be checked without hardware:

```bash
python3 - <<'EOF'
regions = [("stdfilt", 0, 128), ("extfilt", 128, 128),
           ("rxfifo0", 256, 256), ("txfifo", 512, 128)]
end = 0
for name, off, words in regions:
    assert off >= end, f"{name} overlaps the previous region"
    end = off + words
assert end <= 1280, "FDCAN1 exceeds its half of the message RAM"
print(f"regions OK, {end} of 1280 words")
EOF
```

Expected: `regions OK, 640 of 1280 words`.

- [ ] **Step 6: Commit**

```bash
git add boards/fmuv6c/src/fdcan.h boards/fmuv6c/src/fdcan.c \
        boards/fmuv6c/src/Makefile
git commit -m "board: add a receive-only FDCAN1 driver

Written rather than taken from NuttX: the only H7 CAN driver it ships is
SocketCAN, which needs a network stack to carry two message types. PX4
reached the same conclusion on the same silicon.

Receive only, deliberately - a driver that cannot transmit cannot spin a
motor while the bit timing is still being proven.

The message RAM layout follows PX4's and is a table rather than runtime
arithmetic: two regions that overlap corrupt each other's frames and present
as bus errors. The TX FIFO is reserved though nothing transmits, because
laying it out later would move RX FIFO0 - a silent corruption rather than a
build error.

Three things that bite and are commented where they bite: NuttX's header
omits NBTP_NTSEG2 so it is defined here; the start-address fields sit at bit
2, which converts a word offset to the byte address the hardware wants; and
INIT is not immediate, so it is waited on rather than assumed.

The FIFO slot is acknowledged BEFORE deciding whether we want the frame -
returning early without acknowledging wedges the FIFO one frame at a time.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: The daemon

**Files:**
- Create: `apps/vesc/vesc.h`, `apps/vesc/vesc.c`, `apps/vesc/vesc_main.c`
- Create: `apps/vesc/Makefile`, `apps/vesc/Make.defs`, `apps/vesc/Kconfig`
- Modify: `boards/fmuv6c/configs/nsh/defconfig`

**Interfaces:**
- Consumes: `fdcan_*` (Task 4), `vesc_proto` (Task 1), `vesc_status_advertise/_publish` (Task 2), `VESC_CAN_ID` / `VESC_BITRATE` (Task 3).
- Produces: `struct vesc_daemon_status_s`, `int vesc_start(void)`, `int vesc_stop(void)`, `void vesc_status(FAR struct vesc_daemon_status_s *out)`.

- [ ] **Step 1: Write the header**

Create `apps/vesc/vesc.h`:

```c
/****************************************************************************
 * apps/vesc/vesc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Drains FDCAN1, decodes VESC telemetry, publishes vesc_status.
 *
 * Also the discovery tool: it counts every (packet id, controller id) pair
 * it sees, decoded or not. A VESC emitting something docs/can_packet.md does
 * not describe is a fact worth seeing rather than an error worth hiding.
 ****************************************************************************/

#ifndef __APPS_VESC_VESC_H
#define __APPS_VESC_VESC_H

#include <stdbool.h>
#include <stdint.h>

#include "vesc_proto.h"
#include "../../boards/fmuv6c/src/fdcan.h"

#ifndef FAR
#  define FAR
#endif

/* Enough for every packet id in can_packet.md plus room for whatever else
 * turns up, which is the point of discovery.
 */

#define VESC_SEEN_MAX 12

struct vesc_seen_s
{
  uint8_t  packet_id;
  uint8_t  controller_id;
  uint32_t count;
  uint64_t first_us;
  uint64_t last_us;
};

struct vesc_daemon_status_s
{
  bool     running;
  uint32_t bitrate;
  uint8_t  filter_id;         /* 0 = accept any */

  uint32_t decoded;           /* STATUS_5 frames decoded */
  uint32_t bad_dlc;           /* known packet id, wrong length */
  uint32_t publish_errors;

  struct vesc_status5_s last;
  uint64_t last_us;

  uint8_t  nseen;
  struct vesc_seen_s seen[VESC_SEEN_MAX];

  struct fdcan_stats_s bus;
};

int  vesc_start(void);
int  vesc_stop(void);
void vesc_status(FAR struct vesc_daemon_status_s *out);

#endif /* __APPS_VESC_VESC_H */
```

- [ ] **Step 2: Implement the daemon**

Create `apps/vesc/vesc.c`. Copy the lifecycle from `apps/companion/companion.c`
exactly — the `g_lock` mutex, the `volatile bool g_running` / `g_should_stop`
pair, `task_create` plus the 1-second spin-wait in start, and the mirrored
wait in stop.

```c
/****************************************************************************
 * apps/vesc/vesc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/uorb.h>
#include <uORB/uORB.h>

#include "vesc.h"
#include "../param/param.h"
#include "../uorb_msgs/uorb_msgs.h"

#define VESC_PRIORITY   (SCHED_PRIORITY_DEFAULT + 10)
#define VESC_STACK      2048

/* Poll interval. STATUS_5 arrives at tens of hertz, so 2 ms drains the FIFO
 * far faster than it fills while costing almost nothing.
 *
 * This interval IS the jitter on the telemetry - see the note in fdcan.h
 * about polling being a deliberate trade.
 */

#define VESC_POLL_US    2000

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool g_running;
static volatile bool g_should_stop;
static struct vesc_daemon_status_s g_status;

static uint64_t vesc_now_us(void)
{
  struct timespec t;

  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000ull + (uint64_t)t.tv_nsec / 1000ull;
}

static void status_publish(FAR const struct vesc_daemon_status_s *s)
{
  pthread_mutex_lock(&g_lock);
  g_status = *s;
  pthread_mutex_unlock(&g_lock);
}

/* Record that this (packet, controller) pair exists. Linear scan over at
 * most twelve entries, called at frame rate - a hash would be more code for
 * a table that never gets big.
 */

static void vesc_note_seen(FAR struct vesc_daemon_status_s *s,
                           uint8_t packet_id, uint8_t controller_id,
                           uint64_t now)
{
  int i;

  for (i = 0; i < s->nseen; i++)
    {
      if (s->seen[i].packet_id == packet_id &&
          s->seen[i].controller_id == controller_id)
        {
          s->seen[i].count++;
          s->seen[i].last_us = now;
          return;
        }
    }

  if (s->nseen >= VESC_SEEN_MAX)
    {
      return;
    }

  s->seen[s->nseen].packet_id = packet_id;
  s->seen[s->nseen].controller_id = controller_id;
  s->seen[s->nseen].count = 1;
  s->seen[s->nseen].first_us = now;
  s->seen[s->nseen].last_us = now;
  s->nseen++;
}

static void vesc_handle(FAR const struct fdcan_frame_s *frame, int pub,
                        FAR struct vesc_daemon_status_s *s)
{
  uint8_t packet_id = vesc_packet_id(frame->id);
  uint8_t controller_id = vesc_controller_id(frame->id);
  uint64_t now = vesc_now_us();
  struct vesc_status5_s decoded;
  struct vesc_status_s out;

  vesc_note_seen(s, packet_id, controller_id, now);

  if (packet_id != VESC_PACKET_STATUS_5)
    {
      /* Not an error. Counted above so it shows up in discovery, and
       * ignored here because nothing else is decoded yet.
       */

      return;
    }

  if (!vesc_decode_status5(frame->data, frame->dlc, &decoded))
    {
      /* A known packet id with the wrong length is the two ends disagreeing
       * about a format, which is a different thing from a packet id we do
       * not know - and is worth its own counter.
       */

      s->bad_dlc++;
      return;
    }

  memset(&out, 0, sizeof(out));
  out.timestamp = now;
  out.timestamp_sample = now;
  out.tachometer = decoded.tachometer;
  out.current_a = decoded.current_a;
  out.adc_volts = decoded.adc_volts;
  out.controller_id = controller_id;

  if (vesc_status_publish(pub, &out) < 0)
    {
      s->publish_errors++;
      return;
    }

  s->last = decoded;
  s->last_us = now;
  s->decoded++;
}

static int vesc_daemon(int argc, FAR char *argv[])
{
  struct vesc_daemon_status_s status;
  int pub = -1;
  int result = EXIT_FAILURE;
  int ret;

  memset(&status, 0, sizeof(status));
  status.bitrate = (uint32_t)param_i32("VESC_BITRATE");
  status.filter_id = (uint8_t)param_i32("VESC_CAN_ID");

  ret = fdcan_init(status.bitrate);

  if (ret < 0)
    {
      syslog(LOG_ERR, "[vesc] FDCAN1 init failed: %d%s\n", -ret,
             ret == -ENOTSUP ? " (only 1000000 bit/s is implemented)" : "");
      goto out;
    }

  fdcan_set_filter(status.filter_id);

  pub = vesc_status_advertise();

  if (pub < 0)
    {
      syslog(LOG_ERR, "[vesc] cannot advertise vesc_status (%d)\n", errno);
      goto out;
    }

  g_running = true;
  status.running = true;
  status_publish(&status);

  syslog(LOG_INFO, "[vesc] FDCAN1 at %" PRIu32 " bit/s, filter %s\n",
         status.bitrate,
         status.filter_id == 0 ? "accept-any" : "one id");

  while (!g_should_stop)
    {
      struct fdcan_frame_s frame;
      int drained = 0;

      /* Drain everything pending before sleeping. A burst arrives faster
       * than the poll interval, and leaving frames in the FIFO to collect
       * one per wake-up is how an overrun happens on a bus that is not even
       * busy.
       */

      while (drained++ < 32 && fdcan_receive(&frame) == OK)
        {
          vesc_handle(&frame, pub, &status);
        }

      fdcan_stats(&status.bus);
      status_publish(&status);
      usleep(VESC_POLL_US);
    }

  result = EXIT_SUCCESS;

out:
  status.running = false;
  status_publish(&status);

  if (pub >= 0)
    {
      orb_unadvertise(pub);
    }

  g_running = false;
  return result;
}

int vesc_start(void)
{
  int task;
  int wait;

  if (g_running)
    {
      return -EALREADY;
    }

  g_should_stop = false;
  task = task_create("vesc", VESC_PRIORITY, VESC_STACK, vesc_daemon, NULL);

  if (task < 0)
    {
      return -errno;
    }

  for (wait = 0; wait < 100 && !g_running; wait++)
    {
      usleep(10000);
    }

  return g_running ? 0 : -EIO;
}

int vesc_stop(void)
{
  int wait;

  if (!g_running)
    {
      return -ESRCH;
    }

  g_should_stop = true;

  for (wait = 0; wait < 100 && g_running; wait++)
    {
      usleep(10000);
    }

  return g_running ? -ETIMEDOUT : 0;
}

void vesc_status(FAR struct vesc_daemon_status_s *out)
{
  pthread_mutex_lock(&g_lock);
  *out = g_status;
  pthread_mutex_unlock(&g_lock);
}
```

Add `#include <inttypes.h>` if `PRIu32` does not resolve.

- [ ] **Step 3: Write the command**

Create `apps/vesc/vesc_main.c`, following `apps/companion/companion_main.c`'s
dispatch shape:

```c
/****************************************************************************
 * apps/vesc/vesc_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `vesc start | stop | status`
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vesc.h"

static void usage(void)
{
  printf("Usage: vesc start | stop | status\n"
         "\n"
         "  Receives VESC telemetry on FDCAN1 and publishes vesc_status.\n"
         "  Receive only - nothing here commands a motor.\n"
         "\n"
         "  VESC_CAN_ID   0 accepts any controller id (discovery)\n"
         "  VESC_BITRATE  bus bitrate; only 1000000 is implemented\n"
         "  VESC_EN       start at boot\n");
}

static FAR const char *vesc_packet_name(uint8_t id)
{
  switch (id)
    {
      case VESC_PACKET_SET_DUTY:          return "SET_DUTY";
      case VESC_PACKET_SET_CURRENT:       return "SET_CURRENT";
      case VESC_PACKET_PROCESS_SHORT_BUF: return "PROCESS_SHORT_BUFFER";
      case VESC_PACKET_STATUS_5:          return "STATUS_5";
      case VESC_PACKET_SET_CURRENT_SERVO: return "SET_CURRENT_SERVO";
      case VESC_PACKET_SET_DUTY_SERVO:    return "SET_DUTY_SERVO";
      default:                            return "unknown";
    }
}

static void print_status(void)
{
  struct vesc_daemon_status_s s;
  int i;

  vesc_status(&s);

  if (!s.running)
    {
      printf("vesc: stopped\n");
      return;
    }

  printf("vesc: running on FDCAN1 at %" PRIu32 " bit/s, filter %s\n",
         s.bitrate,
         s.filter_id == 0 ? "accept-any" : "one id");

  printf("  bus     rx %" PRIu32 "  lost %" PRIu32 "  rejected %" PRIu32
         "  state %s\n",
         s.bus.rx, s.bus.lost, s.bus.rejected,
         s.bus.bus_off ? "BUS_OFF" :
         s.bus.error_passive ? "ERROR_PASSIVE" : "ERROR_ACTIVE");

  /* Nothing at all is a distinct condition from frames arriving badly, and
   * saying so beats printing a screen of zeros and leaving the reader to
   * work out which.
   */

  if (s.bus.rx == 0)
    {
      printf("  seen    NOTHING - check wiring, termination and bitrate\n");
    }

  for (i = 0; i < s.nseen; i++)
    {
      FAR const struct vesc_seen_s *e = &s.seen[i];
      double span = (double)(e->last_us - e->first_us) / 1000000.0;

      printf("  seen    packet 0x%02x %-20s id 0x%02x  %" PRIu32
             " frames  %.1f Hz\n",
             e->packet_id, vesc_packet_name(e->packet_id),
             e->controller_id, e->count,
             span > 0.0 ? (double)(e->count - 1) / span : 0.0);
    }

  if (s.decoded > 0)
    {
      printf("  status5 tach %" PRIi32 "  current %.2f A  adc %.3f V\n",
             s.last.tachometer, (double)s.last.current_a,
             (double)s.last.adc_volts);
    }

  printf("  decoded %" PRIu32 "  bad_dlc %" PRIu32 "  publish_err %" PRIu32
         "\n", s.decoded, s.bad_dlc, s.publish_errors);
}

int main(int argc, FAR char *argv[])
{
  int ret;

  if (argc < 2)
    {
      usage();
      return EXIT_FAILURE;
    }

  if (strcmp(argv[1], "start") == 0)
    {
      ret = vesc_start();

      if (ret == -EALREADY)
        {
          printf("vesc: already running\n");
          return EXIT_FAILURE;
        }

      if (ret < 0)
        {
          printf("vesc: failed to start (%d) - check the syslog\n", ret);
          return EXIT_FAILURE;
        }

      print_status();
      return EXIT_SUCCESS;
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      ret = vesc_stop();

      if (ret == -ESRCH)
        {
          printf("vesc: not running\n");
          return EXIT_FAILURE;
        }

      printf("vesc: %s\n", ret == OK ? "stopped" : "did not stop");
      return ret == OK ? EXIT_SUCCESS : EXIT_FAILURE;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      print_status();
      return EXIT_SUCCESS;
    }

  usage();
  return EXIT_FAILURE;
}
```

- [ ] **Step 4: Build wiring**

`apps/vesc/Make.defs`:

```make
############################################################################
# apps/vesc/Make.defs
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

ifneq ($(CONFIG_XXCAR_VESC),)
CONFIGURED_APPS += $(APPDIR)/xxcar/vesc
endif
```

`apps/vesc/Makefile`:

```make
############################################################################
# apps/vesc/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_VESC_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_VESC_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_VESC_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_VESC)

# vesc_proto.c is the wire format, with no I/O so it can be host-tested;
# vesc.c is the daemon; vesc_main.c is the command.
CSRCS   = vesc.c vesc_proto.c
MAINSRC = vesc_main.c

include $(APPDIR)/Application.mk
```

`apps/vesc/Kconfig`:

```
#
# For a description of the syntax of this configuration file,
# see the file kconfig-language.txt in the NuttX tools repository.
#

config XXCAR_VESC
	tristate "vesc: VESC CAN telemetry"
	default n
	depends on UORB && STM32H7_FDCAN1
	select XXCAR_PARAM
	select XXCAR_UORB_MSGS
	---help---
		Receives VESC telemetry on FDCAN1 and publishes vesc_status:
		tachometer, filtered motor current, and the ADC1 voltage used as
		steering feedback.

		Receive only. Nothing here commands a motor or a servo.

		With VESC_CAN_ID at 0 it accepts any controller id and `vesc
		status` lists every packet id seen, which is how the node id and
		the packet set are discovered in the first place.

if XXCAR_VESC

config XXCAR_VESC_PROGNAME
	string "Program name"
	default "vesc"

config XXCAR_VESC_PRIORITY
	int "command priority"
	default 100

config XXCAR_VESC_STACKSIZE
	int "command stack size"
	default 2048

endif
```

Enable it in `boards/fmuv6c/configs/nsh/defconfig`:

```
CONFIG_XXCAR_VESC=y
```

- [ ] **Step 5: Build and gate**

```bash
RECONFIGURE=1 ./tools/build.sh && ./tools/verify.sh 2>&1 | grep -E "FAIL|exited|rebuilt|over 80"
```

Expected: build exits 0; only `test-cpu-runtime` fails.

- [ ] **Step 6: Commit**

```bash
git add apps/vesc boards/fmuv6c/configs/nsh/defconfig
git commit -m "vesc: add the telemetry daemon

Drains FDCAN1, decodes STATUS_5, publishes vesc_status. Receive only.

It is also the discovery tool: every (packet id, controller id) pair is
counted whether it decodes or not, because a VESC emitting something
can_packet.md does not describe is a fact worth seeing rather than an error
worth hiding. That is what answers the node id and the packet set on a
bench, without transmitting anything.

The FIFO is drained fully before sleeping. A burst arrives faster than the
poll interval, and taking one frame per wake-up is how an overrun happens on
a bus that is not even busy.

A known packet id with the wrong length is counted separately from an
unknown packet id - one is the two ends disagreeing about a format, the
other is a VESC newer than this firmware.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Boot integration and hardware verification

**Files:**
- Modify: `boards/fmuv6c/src/stm32_bringup.c`

**Interfaces:**
- Consumes: `vesc_start()` (Task 5), `VESC_EN` (Task 3).
- Produces: nothing.

- [ ] **Step 1: Start it at boot when asked**

In `boards/fmuv6c/src/stm32_bringup.c`, add the include beside the others:

```c
#ifdef CONFIG_XXCAR_VESC
#  include "../../../apps/vesc/vesc.h"
#endif
```

And in the services block, after the estimator:

```c
#ifdef CONFIG_XXCAR_VESC
  /* Off unless asked. A driver touching a new peripheral does not belong in
   * the boot path until it has run, and this one is still receive-only, so
   * there is nothing lost by starting it by hand at first.
   */

  if (param_i32("VESC_EN") != 0)
    {
      if (vesc_start() < 0)
        {
          syslog(LOG_ERR, "[vesc] boot start failed\n");
          fmuv6c_boot_optional_failure(&boot);
        }
      else
        {
          syslog(LOG_INFO, "[vesc] started at boot (VESC_EN=1)\n");
        }
    }
#endif
```

- [ ] **Step 2: Full gate**

```bash
./tools/verify.sh 2>&1 | grep -E "FAIL|exited|rebuilt|over 80"
```

Expected: only `test-cpu-runtime`.

- [ ] **Step 3: Commit**

```bash
git add boards/fmuv6c/src/stm32_bringup.c
git commit -m "vesc: start the CAN link at boot when VESC_EN is set

Off by default and an optional failure when on. A driver touching a new
peripheral does not belong in the boot path until it has run, and a board
that will not reach a shell because CAN failed is worse than one without
telemetry.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

## Hardware verification

Flash, then run these in order. **Stop at the first step that does not
match** — each isolates a different part, and continuing past a failure
gives up that isolation.

**1. The peripheral initialises without a bus.** Nothing connected:

```
vesc start
vesc status
```

Expect `state ERROR_ACTIVE`, `rx 0`, and `seen NOTHING`. This proves the
clock, pins and message RAM configuration are accepted by the hardware
independently of anything being attached. If `vesc start` fails here, no
amount of wiring will help.

**2. Frames arrive.** VESC connected, bus terminated at both ends:

```
vesc status
```

`rx` climbing is necessary but not sufficient — **`state ERROR_ACTIVE` with
`lost 0` is the real check.** A bitrate mismatch or missing termination
usually shows as frames arriving *with* errors, or as `ERROR_PASSIVE`,
rather than as silence.

**3. Discovery.** Read the `seen` list. Note:

- Is `0x1B STATUS_5` present, and at what rate?
- What controller id is it using? That is the value for `VESC_CAN_ID`.
- Is anything else there? If `0x09` appears, the VESC is also sending the
  stock status frame, which `can_packet.md` does not mention and which
  carries ERPM directly - worth knowing before the control phase.

**4. The decode is right, and this needs no transmit.** Turn the steering by
hand: `adc` moves, and should sit somewhere in 0-3.3 V. Turn the motor by
hand: `tach` moves, and moves *negative* if you turn it the other way. That
negative case is the one that proves sign extension across the byte swap,
which is the failure this subsystem is most likely to have.

**5. The hardware filter accepts rather than rejects.**

```
param set VESC_CAN_ID <the id from step 3>
param save
vesc stop && vesc start
vesc status
```

`rx` must keep climbing. A filter that rejects everything looks exactly like
a disconnected bus, so this step is checking the filter polarity, not the
wiring.

**6. Overrun behaviour**, if you want it: `vesc stop`, wait, `vesc start`.
`lost` may tick once as the FIFO drains a backlog. It should not climb in
steady state; if it does, the poll interval is too slow for this bus.

## Known limitations

Carried from the spec, because they shape what a failure means:

- **Polled receive.** Telemetry jitter is bounded by `VESC_POLL_US`, not by
  the bus. Fine for feedback, revisit for control.
- **No transmit.** The motor cannot be commanded from this firmware at all
  after this work. That is the point of it.
- **One VESC.** The filter accepts a single controller id; multiple nodes
  need a filter and a topic instance each.
- **Bus-off is reported, not recovered.** Automatic recovery would mask a
  wiring or termination fault as intermittent behaviour. It becomes worth
  adding when transmit exists and a fault can be provoked deliberately.
- **Only 1 Mbit/s.** `VESC_BITRATE` exists and other values are refused
  rather than mis-timed, because the NBTP constants are derived for this
  clock and rate only.
