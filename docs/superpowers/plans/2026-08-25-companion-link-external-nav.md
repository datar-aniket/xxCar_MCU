# Companion Link and External Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A packet link to the companion computer that routes messages to uORB topics by id, and external position fusion in EKF3 that bounds horizontal drift with ArduPilot-grade gating.

**Architecture:** `apps/companion` owns one serial port and knows nothing about navigation — it frames, CRCs and routes by message id, publishing `external_pose`. EKF3 subscribes to that topic exactly as it subscribes to `vehicle_mag`, queues samples into the existing delayed horizon, and fuses x/y as sequential scalar updates with yaw going through a `fuse_yaw()` shared with the magnetometer.

**Tech Stack:** C11, NuttX, uORB, existing `ekf_delay` horizon and `measurement_update_1d`.

**Spec:** `docs/superpowers/specs/2026-08-25-companion-link-external-nav-design.md`

## Global Constraints

- **Two phases.** Tasks 1–4 are the link and leave estimator behaviour completely unchanged — `external_pose` is published and consumed by nothing. Only Tasks 5–8 change fusion. Flash and verify the link before any estimator behaviour moves.
- **uORB topic names are limited to 20 characters.** `/dev/uorb/<name><instance>` must fit `NAME_MAX` (32); exceeding it truncates silently and `orb_advertise()` returns −1 with no explanation. Enforce with `ORB_NAME_FITS()`.
- **Every new uORB struct and every wire struct needs `static_assert` offset and size checks.** A uint64 first member forces 8-byte alignment; declare padding explicitly rather than letting the compiler add it.
- **`PARAM_NAME_MAX` is 16**, including the NUL.
- **The noise parameter is a FLOOR, not a default:** `noise = max(reported_sigma, EK3_EXT_M_NSE)`. ArduPilot does this with `posErr`; a source claiming millimetre accuracy must not talk the filter into trusting it more than configured.
- **Gates are plain sigma.** ArduPilot stores its as integer sigma×100; this codebase uses floats and has no reason to copy that.
- **Timestamp sanity:** refuse and count when ahead of board time, or older than 500 ms (`EKF3_EXT_MAX_AGE_US`), matching the baro and mag queue bounds.
- **C style:** NuttX kernel style as used throughout `apps/` — two-space indent, braces on their own line, `FAR` on pointer parameters, `/* */` comments only. Lines ≤ 80 columns (`verify.sh` checks this).
- **Host tests compile with** `-std=c11 -Wall -Wextra -Werror -DFAR=`.
- **No dynamic allocation** in the daemon or the estimator.
- **Do not modify** `apps/cal/`, `apps/sensors/`, or any driver in `boards/fmuv6c/src/`.
- **Commit after every task**, trailer `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`.
- **`tools/verify.sh` is the gate.** `test-cpu-runtime` is a known pre-existing failure and must not be "fixed" here.

## File Structure

**Created:**

| File | Responsibility |
|---|---|
| `apps/companion/comp_proto.h` / `.c` | Framing, CRC, id/length table, byte-at-a-time parser. No I/O, no uORB. |
| `apps/companion/companion.h` / `.c` | Daemon: port, route-and-publish, subscribe-and-transmit. |
| `apps/companion/companion_main.c` | `companion start \| stop \| status` |
| `apps/companion/Makefile`, `Make.defs`, `Kconfig` | Build wiring, following `apps/ekf3`. |
| `tests/comp_proto_test.c`, `tools/test-comp-proto.sh` | Codec tests. |

**Modified:**

| File | Change |
|---|---|
| `apps/uorb_msgs/uorb_msgs.h` / `.c` | `external_pose` topic. |
| `apps/param/param.h` | `SER_FUNC_COMPANION 6`. |
| `apps/param/param.c` | `SER_*_FUNC` range 5→6; `EXT_TX_RATE`, `EK3_EXT_*`. |
| `apps/serial/serial.c` | Reserve the port, start nothing. |
| `apps/ekf3/ekf_delay.h` / `.c` | `ekf_extnav_sample_s` queue. |
| `apps/ekf3/ekf_core.h` / `.c` | `fuse_yaw()` extraction, position reset, extnav fusion. |
| `apps/ekf3/ekf3.h` / `.c` | Subscribe, queue, fuse at the horizon. |
| `apps/ekf3/ekf3_main.c` | External-nav status line. |
| `tests/ekf_core_test.c` | Datum, fusion, floor, timestamp tests. |

`comp_proto` being free of I/O and uORB is what lets the packet format be tested while it is still being iterated on, which is the stated plan for it.

---

## Phase 1 — the link

### Task 1: The codec

**Files:**
- Create: `apps/companion/comp_proto.h`, `apps/companion/comp_proto.c`
- Create: `tests/comp_proto_test.c`, `tools/test-comp-proto.sh`

**Interfaces:**
- Consumes: nothing.
- Produces: `COMP_SYNC`, message id defines, `struct comp_external_pose_s`, `struct comp_estimator_pose_s`, `struct comp_parser_s`, `uint16_t comp_crc16(FAR const uint8_t *d, size_t n)`, `uint8_t comp_payload_len(uint8_t id)`, `void comp_parser_init(FAR struct comp_parser_s *p)`, `int comp_parser_byte(FAR struct comp_parser_s *p, uint8_t b)`, `int comp_encode(uint8_t id, FAR const void *payload, uint8_t len, FAR uint8_t *out, size_t out_size)`.

- [ ] **Step 1: Write the header**

Create `apps/companion/comp_proto.h`:

```c
/****************************************************************************
 * apps/companion/comp_proto.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wire format for the companion computer link.
 *
 *   0xFE | id | len | payload[len] | crc16
 *
 * The CRC covers id, len and payload. Anything that is not the sync byte is
 * discarded, so a lost byte costs one message rather than wedging the
 * stream.
 *
 * Inbound ids are low and outbound ids are high. A message sent in the wrong
 * direction then fails to route at all, instead of half-working.
 *
 * No I/O and no uORB in this file. That is what lets the format be tested on
 * the host while it is still being iterated on.
 ****************************************************************************/

#ifndef __APPS_COMPANION_COMP_PROTO_H
#define __APPS_COMPANION_COMP_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

#define COMP_SYNC              0xfe
#define COMP_MAX_PAYLOAD       64
#define COMP_FRAME_OVERHEAD    5      /* sync + id + len + crc16 */

/* Inbound: companion -> board. */

#define COMP_MSG_EXTERNAL_POSE   1
#define COMP_MSG_CONTROL_TRAJ    2    /* reserved, not yet defined */
#define COMP_MSG_TIMESYNC_REQ    3    /* reserved */

/* Outbound: board -> companion. */

#define COMP_MSG_TIMESYNC_REP    4    /* reserved */
#define COMP_MSG_ESTIMATOR_POSE 16

/* Absolute pose in the companion's map frame. Only x, y and yaw are fused;
 * height stays with the barometer.
 *
 * cov is the UPPER TRIANGLE of the 3x3 for (x, y, yaw):
 *   [0] xx  [1] xy  [2] x-yaw  [3] yy  [4] y-yaw  [5] yaw-yaw
 *
 * A zero variance means "no estimate supplied, use the parameter", so the
 * companion side can start by sending zeros and tighten later without a
 * format change.
 */

struct comp_external_pose_s
{
  uint64_t timestamp_us;    /*  0: board timebase, per the operator's sync */
  float    x;               /*  8: m, map frame */
  float    y;               /* 12 */
  float    yaw;             /* 16: rad, map frame */
  float    cov[6];          /* 20: see above */
  uint8_t  flags;           /* 44: bit 0 = pose valid */
  uint8_t  reset_counter;   /* 45: the SOURCE's frame-reset generation */
  uint8_t  pad[2];          /* 46 */
};

#define COMP_POSE_FLAG_VALID (1u << 0)

/* The estimator's pose, sent at EXT_TX_RATE.
 *
 * reset_counter matters more than it looks: the datum reset moves position
 * discontinuously, and anything differentiating position on the companion
 * side needs to know that happened rather than seeing a spike.
 */

struct comp_estimator_pose_s
{
  uint64_t timestamp_us;    /*  0: board monotonic */
  float    position[3];     /*  8: m, local NWU */
  float    quaternion[4];   /* 20: w x y z, body to nav */
  float    velocity[3];     /* 36: m/s, local NWU */
  uint8_t  solution_status; /* 48: ESTIMATOR_* validity bits */
  uint8_t  reset_counter;   /* 49: estimator reset generation */
  uint8_t  pad[6];          /* 50: a uint64 first member forces 8-byte
                             *     alignment, so 50 pads to 56 whatever this
                             *     says. Declared, so the wire format is what
                             *     the struct says rather than what the
                             *     compiler decided. */
};

enum comp_parse_state_e
{
  COMP_WAIT_SYNC = 0,
  COMP_WAIT_ID,
  COMP_WAIT_LEN,
  COMP_WAIT_PAYLOAD,
  COMP_WAIT_CRC_LO,
  COMP_WAIT_CRC_HI
};

struct comp_parser_s
{
  uint8_t  state;
  uint8_t  id;
  uint8_t  len;
  uint8_t  fill;
  uint16_t crc_rx;
  uint8_t  payload[COMP_MAX_PAYLOAD];

  uint32_t frames;          /* accepted */
  uint32_t crc_errors;
  uint32_t unknown_id;      /* benign: a companion newer than this firmware */
  uint32_t bad_length;      /* NOT benign: the two ends disagree on a format */
  uint32_t resyncs;
};

/* CRC16-CCITT-FALSE. Same polynomial and seed the cal protocol uses, so a
 * host implementing one has implemented both.
 */

uint16_t comp_crc16(FAR const uint8_t *d, size_t n);

/* Expected payload length for a known id, or 0 when the id is unknown.
 *
 * Knowing the length is what separates "a companion newer than this
 * firmware", which is fine and ignorable, from "the two ends disagree about
 * a format", which is not.
 */

uint8_t comp_payload_len(uint8_t id);

void comp_parser_init(FAR struct comp_parser_s *p);

/* Feed exactly one byte. Returns the message id when a complete, CRC-valid
 * frame with the right length just completed, and 0 otherwise. The payload is
 * then in p->payload for p->len bytes, valid until the next call.
 *
 * One byte at a time, deliberately. A parser that only works on whole frames
 * passes every other test and fails the first time a UART splits one, which
 * it will.
 */

int comp_parser_byte(FAR struct comp_parser_s *p, uint8_t b);

/* Frame payload into out. Returns the number of bytes written, or a negated
 * errno.
 */

int comp_encode(uint8_t id, FAR const void *payload, uint8_t len,
                FAR uint8_t *out, size_t out_size);

#endif /* __APPS_COMPANION_COMP_PROTO_H */
```

- [ ] **Step 2: Write the failing test**

Create `tests/comp_proto_test.c`:

```c
/****************************************************************************
 * tests/comp_proto_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "comp_proto.h"

static struct comp_parser_s g_parser;

/* Feed a buffer one byte at a time and return the last id produced. */

static int feed(FAR const uint8_t *d, size_t n)
{
  int last = 0;
  size_t i;

  for (i = 0; i < n; i++)
    {
      int got = comp_parser_byte(&g_parser, d[i]);

      if (got != 0)
        {
          last = got;
        }
    }

  return last;
}

static struct comp_external_pose_s sample_pose(void)
{
  struct comp_external_pose_s p;
  int i;

  memset(&p, 0, sizeof(p));
  p.timestamp_us = 1234567890123ull;
  p.x = -12.5f;
  p.y = 3.25f;
  p.yaw = 1.5707963f;

  for (i = 0; i < 6; i++)
    {
      p.cov[i] = 0.01f * (float)(i + 1);
    }

  p.flags = COMP_POSE_FLAG_VALID;
  p.reset_counter = 7;
  return p;
}

/* The layout is the wire format. If these ever disagree the companion reads
 * convincing nonsense, so the compiler proves them instead.
 */

static void test_layout(void)
{
  assert(sizeof(struct comp_external_pose_s) == 48);
  assert(sizeof(struct comp_estimator_pose_s) == 56);
  assert(comp_payload_len(COMP_MSG_EXTERNAL_POSE) == 48);
  assert(comp_payload_len(COMP_MSG_ESTIMATOR_POSE) == 56);
  assert(comp_payload_len(200) == 0);
}

static void test_round_trip(void)
{
  struct comp_external_pose_s in = sample_pose();
  struct comp_external_pose_s out;
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int n;

  n = comp_encode(COMP_MSG_EXTERNAL_POSE, &in, sizeof(in), frame,
                  sizeof(frame));
  assert(n == (int)sizeof(in) + COMP_FRAME_OVERHEAD);

  comp_parser_init(&g_parser);
  assert(feed(frame, (size_t)n) == COMP_MSG_EXTERNAL_POSE);
  assert(g_parser.len == sizeof(in));

  memcpy(&out, g_parser.payload, sizeof(out));
  assert(out.timestamp_us == in.timestamp_us);
  assert(out.x == in.x && out.y == in.y && out.yaw == in.yaw);
  assert(out.cov[0] == in.cov[0] && out.cov[5] == in.cov[5]);
  assert(out.flags == in.flags);
  assert(out.reset_counter == in.reset_counter);
  assert(g_parser.frames == 1);
}

static void test_corrupt_byte_fails_crc(void)
{
  struct comp_external_pose_s in = sample_pose();
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int n = comp_encode(COMP_MSG_EXTERNAL_POSE, &in, sizeof(in), frame,
                      sizeof(frame));

  frame[10] ^= 0x20;

  comp_parser_init(&g_parser);
  assert(feed(frame, (size_t)n) == 0);
  assert(g_parser.crc_errors == 1);
  assert(g_parser.frames == 0);
}

/* A truncated frame must be HELD, not misparsed. The rest arrives next
 * read(); a parser that gave up here would drop every message a UART split.
 */

static void test_truncated_is_held_then_completed(void)
{
  struct comp_external_pose_s in = sample_pose();
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int n = comp_encode(COMP_MSG_EXTERNAL_POSE, &in, sizeof(in), frame,
                      sizeof(frame));

  comp_parser_init(&g_parser);
  assert(feed(frame, (size_t)n - 3) == 0);
  assert(g_parser.frames == 0);
  assert(g_parser.crc_errors == 0);

  assert(feed(frame + n - 3, 3) == COMP_MSG_EXTERNAL_POSE);
  assert(g_parser.frames == 1);
}

static void test_garbage_then_resync(void)
{
  struct comp_external_pose_s in = sample_pose();
  uint8_t junk[] = {0x00, 0x11, 0x22, 0x7b, 0xff, 0x01};
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int n = comp_encode(COMP_MSG_EXTERNAL_POSE, &in, sizeof(in), frame,
                      sizeof(frame));

  comp_parser_init(&g_parser);
  assert(feed(junk, sizeof(junk)) == 0);
  assert(feed(frame, (size_t)n) == COMP_MSG_EXTERNAL_POSE);
}

/* A sync byte INSIDE the payload must not restart the parse. The length
 * field is authoritative; hunting for sync mid-frame would desynchronise on
 * any pose whose bytes happen to contain 0xFE, which is common for floats.
 */

static void test_sync_byte_in_payload(void)
{
  struct comp_external_pose_s in = sample_pose();
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int n;

  in.x = -1.0f;   /* 0xBF800000: no 0xFE */
  memcpy((uint8_t *)&in.y, "\xfe\xfe\xfe\xfe", 4);

  n = comp_encode(COMP_MSG_EXTERNAL_POSE, &in, sizeof(in), frame,
                  sizeof(frame));

  comp_parser_init(&g_parser);
  assert(feed(frame, (size_t)n) == COMP_MSG_EXTERNAL_POSE);
  assert(memcmp(g_parser.payload, &in, sizeof(in)) == 0);
}

/* An unknown id is a companion newer than this firmware: skip it and carry
 * on. A KNOWN id with the wrong length is the two ends disagreeing about a
 * format, which is not benign and is counted separately.
 */

static void test_unknown_id_versus_bad_length(void)
{
  uint8_t body[8] = {0};
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int n;

  comp_parser_init(&g_parser);

  n = comp_encode(99, body, sizeof(body), frame, sizeof(frame));
  assert(feed(frame, (size_t)n) == 0);
  assert(g_parser.unknown_id == 1);
  assert(g_parser.bad_length == 0);

  n = comp_encode(COMP_MSG_EXTERNAL_POSE, body, sizeof(body), frame,
                  sizeof(frame));
  assert(feed(frame, (size_t)n) == 0);
  assert(g_parser.bad_length == 1);
  assert(g_parser.unknown_id == 1);
}

/* Two frames back to back, with no gap, must both come out. */

static void test_back_to_back(void)
{
  struct comp_external_pose_s in = sample_pose();
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int n = comp_encode(COMP_MSG_EXTERNAL_POSE, &in, sizeof(in), frame,
                      sizeof(frame));

  comp_parser_init(&g_parser);
  assert(feed(frame, (size_t)n) == COMP_MSG_EXTERNAL_POSE);
  assert(feed(frame, (size_t)n) == COMP_MSG_EXTERNAL_POSE);
  assert(g_parser.frames == 2);
}

static void test_encode_refuses_a_short_buffer(void)
{
  struct comp_external_pose_s in = sample_pose();
  uint8_t small[8];

  assert(comp_encode(COMP_MSG_EXTERNAL_POSE, &in, sizeof(in), small,
                     sizeof(small)) < 0);
}

int main(void)
{
  test_layout();
  test_round_trip();
  test_corrupt_byte_fails_crc();
  test_truncated_is_held_then_completed();
  test_garbage_then_resync();
  test_sync_byte_in_payload();
  test_unknown_id_versus_bad_length();
  test_back_to_back();
  test_encode_refuses_a_short_buffer();

  puts("comp_proto: framing, CRC, resync and length checks verified - OK");
  return 0;
}
```

- [ ] **Step 3: Write the runner**

Create `tools/test-comp-proto.sh`:

```bash
#!/usr/bin/env bash
# Host-side test for the companion link codec: framing, CRC, resynchronisation
# and the id/length checks. Byte-at-a-time feeding is the one that earns its
# keep - a parser that only works on whole frames passes everything else and
# fails the first time a UART splits one.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -I"$REPO/apps/companion" \
   -o "$OUT/test" "$REPO/tests/comp_proto_test.c" \
   "$REPO/apps/companion/comp_proto.c"
"$OUT/test"

cc -std=c11 -Wall -Wextra -Werror -DFAR= \
   -fsanitize=undefined,address -fno-sanitize-recover=all \
   -I"$REPO/apps/companion" \
   -o "$OUT/test-san" "$REPO/tests/comp_proto_test.c" \
   "$REPO/apps/companion/comp_proto.c"
"$OUT/test-san"
```

Then `chmod +x tools/test-comp-proto.sh`.

The sanitiser pass matters here specifically: a parser indexing a payload
buffer from a length byte the wire supplied is exactly where an overrun
lives, and ASan is what proves the bound is real.

- [ ] **Step 4: Run to verify it fails**

```bash
tools/test-comp-proto.sh
```

Expected: FAIL at compile — `comp_proto.c: No such file or directory`.

- [ ] **Step 5: Implement the codec**

Create `apps/companion/comp_proto.c`:

```c
/****************************************************************************
 * apps/companion/comp_proto.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <errno.h>
#include <string.h>

#include "comp_proto.h"

/* CRC16-CCITT-FALSE, nibble table. Same polynomial and seed as the cal
 * protocol, so a host that implemented one has implemented both.
 */

static const uint16_t g_crc_tab[16] =
{
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
};

uint16_t comp_crc16(FAR const uint8_t *d, size_t n)
{
  uint16_t crc = 0xffff;
  size_t i;

  for (i = 0; i < n; i++)
    {
      uint8_t hi = (uint8_t)((crc >> 12) ^ (d[i] >> 4)) & 0xf;

      crc = (uint16_t)((crc << 4) ^ g_crc_tab[hi]);
      hi  = (uint8_t)((crc >> 12) ^ (d[i] & 0xf)) & 0xf;
      crc = (uint16_t)((crc << 4) ^ g_crc_tab[hi]);
    }

  return crc;
}

uint8_t comp_payload_len(uint8_t id)
{
  switch (id)
    {
      case COMP_MSG_EXTERNAL_POSE:
        return (uint8_t)sizeof(struct comp_external_pose_s);

      case COMP_MSG_ESTIMATOR_POSE:
        return (uint8_t)sizeof(struct comp_estimator_pose_s);

      default:
        return 0;
    }
}

void comp_parser_init(FAR struct comp_parser_s *p)
{
  if (p != NULL)
    {
      memset(p, 0, sizeof(*p));
    }
}

int comp_parser_byte(FAR struct comp_parser_s *p, uint8_t b)
{
  if (p == NULL)
    {
      return 0;
    }

  switch (p->state)
    {
      case COMP_WAIT_SYNC:
        if (b == COMP_SYNC)
          {
            p->state = COMP_WAIT_ID;
          }
        else
          {
            p->resyncs++;
          }

        return 0;

      case COMP_WAIT_ID:
        p->id = b;
        p->state = COMP_WAIT_LEN;
        return 0;

      case COMP_WAIT_LEN:

        /* Bound before trusting: the length came off the wire. */

        if (b > COMP_MAX_PAYLOAD)
          {
            p->bad_length++;
            p->state = COMP_WAIT_SYNC;
            return 0;
          }

        p->len = b;
        p->fill = 0;
        p->state = p->len > 0 ? COMP_WAIT_PAYLOAD : COMP_WAIT_CRC_LO;
        return 0;

      case COMP_WAIT_PAYLOAD:

        /* The length field is authoritative. A 0xFE inside the payload must
         * NOT restart the parse - float bytes contain it routinely, and
         * hunting for sync mid-frame would desynchronise on ordinary data.
         */

        p->payload[p->fill++] = b;

        if (p->fill >= p->len)
          {
            p->state = COMP_WAIT_CRC_LO;
          }

        return 0;

      case COMP_WAIT_CRC_LO:
        p->crc_rx = b;
        p->state = COMP_WAIT_CRC_HI;
        return 0;

      case COMP_WAIT_CRC_HI:
        {
          uint8_t header[2];
          uint16_t crc;
          uint8_t expect;

          p->crc_rx |= (uint16_t)b << 8;
          p->state = COMP_WAIT_SYNC;

          header[0] = p->id;
          header[1] = p->len;
          crc = comp_crc16(header, 2);
          crc = comp_crc16_update(crc, p->payload, p->len);

          if (crc != p->crc_rx)
            {
              p->crc_errors++;
              return 0;
            }

          expect = comp_payload_len(p->id);

          if (expect == 0)
            {
              /* A companion newer than this firmware. Benign. */

              p->unknown_id++;
              return 0;
            }

          if (expect != p->len)
            {
              /* The two ends disagree about a format. Not benign. */

              p->bad_length++;
              return 0;
            }

          p->frames++;
          return p->id;
        }

      default:
        p->state = COMP_WAIT_SYNC;
        return 0;
    }
}

int comp_encode(uint8_t id, FAR const void *payload, uint8_t len,
                FAR uint8_t *out, size_t out_size)
{
  uint16_t crc;

  if (out == NULL || len > COMP_MAX_PAYLOAD ||
      (len > 0 && payload == NULL) ||
      out_size < (size_t)len + COMP_FRAME_OVERHEAD)
    {
      return -EINVAL;
    }

  out[0] = COMP_SYNC;
  out[1] = id;
  out[2] = len;

  if (len > 0)
    {
      memcpy(out + 3, payload, len);
    }

  crc = comp_crc16(out + 1, (size_t)len + 2);
  out[3 + len] = (uint8_t)(crc & 0xff);
  out[4 + len] = (uint8_t)(crc >> 8);

  return (int)len + COMP_FRAME_OVERHEAD;
}
```

The parser needs a resumable CRC. Add alongside `comp_crc16` in the same
file, and declare it in the header next to it:

```c
uint16_t comp_crc16_update(uint16_t crc, FAR const uint8_t *d, size_t n)
{
  size_t i;

  for (i = 0; i < n; i++)
    {
      uint8_t hi = (uint8_t)((crc >> 12) ^ (d[i] >> 4)) & 0xf;

      crc = (uint16_t)((crc << 4) ^ g_crc_tab[hi]);
      hi  = (uint8_t)((crc >> 12) ^ (d[i] & 0xf)) & 0xf;
      crc = (uint16_t)((crc << 4) ^ g_crc_tab[hi]);
    }

  return crc;
}
```

and rewrite `comp_crc16` as `return comp_crc16_update(0xffff, d, n);` so
there is one loop, not two that can diverge.

Header declaration to add:

```c
uint16_t comp_crc16_update(uint16_t crc, FAR const uint8_t *d, size_t n);
```

- [ ] **Step 6: Run to verify it passes**

```bash
tools/test-comp-proto.sh
```

Expected: `comp_proto: framing, CRC, resync and length checks verified - OK`, twice (plain and sanitised).

If `test_layout` fails, the padding is wrong — re-read the `pad[6]` comment
in the header: a `uint64_t` first member forces 8-byte alignment.

- [ ] **Step 7: Commit**

```bash
git add apps/companion/comp_proto.h apps/companion/comp_proto.c \
        tests/comp_proto_test.c tools/test-comp-proto.sh
git commit -m "companion: add the link codec

Framing, CRC and an id/length table, with no I/O and no uORB - which is what
lets the packet format be tested on the host while it is still being
iterated on.

The parser takes one byte at a time. A parser that only works on whole
frames passes every other test and fails the first time a UART splits one,
which it will. The length field is authoritative rather than hunting for the
sync byte, because float payloads contain 0xFE routinely.

An unknown id is counted separately from a known id with the wrong length:
the first is a companion newer than this firmware and is ignorable, the
second is the two ends disagreeing about a format and is not.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: The `external_pose` topic

**Files:**
- Modify: `apps/uorb_msgs/uorb_msgs.h`, `apps/uorb_msgs/uorb_msgs.c`

**Interfaces:**
- Consumes: `struct comp_external_pose_s` field semantics (not the type — the topic is independent of the wire).
- Produces: `struct external_pose_s`, `ORB_ID(external_pose)`, `int external_pose_advertise(void)`, `int external_pose_publish(int fd, FAR const struct external_pose_s *msg)`.

- [ ] **Step 1: Add the struct**

In `apps/uorb_msgs/uorb_msgs.h`, after `struct vehicle_baro_s`:

```c
/* An absolute pose from the companion computer, in ITS map frame.
 *
 * Deliberately a separate type from the wire struct in comp_proto.h. They
 * happen to carry the same fields today; tying the topic to the wire would
 * mean every future format change was also a change to everything
 * subscribed.
 *
 * Only x, y and yaw are fused - height stays with the barometer. cov is the
 * upper triangle of the 3x3 for (x, y, yaw); a zero entry means the source
 * supplied no estimate and the parameter is used instead.
 */

struct external_pose_s
{
  uint64_t timestamp;             /*  0: us, publication time */
  uint64_t timestamp_sample;      /*  8: us, board timebase, from the source */
  float    x;                     /* 16: m, map frame */
  float    y;                     /* 20 */
  float    yaw;                   /* 24: rad, map frame */
  float    cov[6];                /* 28: xx xy xyaw yy yyaw yawyaw */
  uint8_t  flags;                 /* 52: bit 0 = pose valid */
  uint8_t  reset_counter;         /* 53: source's frame-reset generation */
  uint8_t  pad[2];                /* 54 */
};

#define EXTERNAL_POSE_VALID (1u << 0)
```

Add `ORB_DECLARE(external_pose);` to the declare block and the two prototypes
to the prototype block:

```c
int external_pose_advertise(void);
int external_pose_publish(int fd, FAR const struct external_pose_s *msg);
```

- [ ] **Step 2: Add the assertions and definition**

In `apps/uorb_msgs/uorb_msgs.c`, add `ORB_NAME_FITS("external_pose");` beside
the others, then after the `vehicle_baro_s` assertions:

```c
static_assert(offsetof(struct external_pose_s, timestamp)        ==  0, "layout");
static_assert(offsetof(struct external_pose_s, timestamp_sample) ==  8, "layout");
static_assert(offsetof(struct external_pose_s, x)                == 16, "layout");
static_assert(offsetof(struct external_pose_s, y)                == 20, "layout");
static_assert(offsetof(struct external_pose_s, yaw)              == 24, "layout");
static_assert(offsetof(struct external_pose_s, cov)              == 28, "layout");
static_assert(offsetof(struct external_pose_s, flags)            == 52, "layout");
static_assert(offsetof(struct external_pose_s, reset_counter)    == 53, "layout");
static_assert(sizeof(struct external_pose_s)                     == 56, "layout");
```

Format string, inside the `CONFIG_DEBUG_UORB` block:

```c
static const char external_pose_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",x:%hf,y:%hf,yaw:%hf"
  ",cov[0]:%hf,cov[1]:%hf,cov[2]:%hf"
  ",cov[3]:%hf,cov[4]:%hf,cov[5]:%hf"
  ",flags:%hhu,reset_counter:%hhu";
```

Definition and accessors, copying `vehicle_baro_publish`'s exact guard
(`fd < 0 || msg == NULL`):

```c
ORB_DEFINE(external_pose, struct external_pose_s, external_pose_format);

int external_pose_advertise(void)
{
  return orb_advertise(ORB_ID(external_pose), NULL);
}

int external_pose_publish(int fd, FAR const struct external_pose_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(external_pose), fd, msg);
}
```

- [ ] **Step 3: Prove the assertions run**

Change `sizeof(struct external_pose_s) == 56` to `== 57` and:

```bash
./tools/build.sh 2>&1 | grep -i layout
```

Expected: build FAILS naming that line. Revert.

- [ ] **Step 4: Build clean and commit**

```bash
./tools/build.sh
git add apps/uorb_msgs/uorb_msgs.h apps/uorb_msgs/uorb_msgs.c
git commit -m "uorb_msgs: define the external_pose topic

A separate type from the wire struct in comp_proto.h even though they carry
the same fields today. Tying the topic to the wire format would make every
future protocol change a change to everything subscribed to it.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Port reservation and parameters

**Files:**
- Modify: `apps/param/param.h`, `apps/param/param.c`, `apps/serial/serial.c`
- Modify: `tests/param_range_test.c`

**Interfaces:**
- Produces: `SER_FUNC_COMPANION` (6); parameters `EXT_TX_RATE`, `EK3_EXT_M_NSE`, `EK3_EXT_I_GATE`, `EK3_EXT_YAW_NSE`, `EK3_EXT_TIMEOUT`.

- [ ] **Step 1: Write the failing test**

In `tests/param_range_test.c`, add above `main()` and register the call:

```c
/* The companion port and the external-navigation parameters.
 *
 * SER_*_FUNC's range has to grow with the enum. A value the table refuses is
 * indistinguishable from a typo at the shell, and PARAM_RANGE_ENUM means it
 * is REFUSED rather than clamped to a neighbouring function - which is the
 * whole reason that range exists.
 */

static void test_companion_parameters(void)
{
  float value;
  int32_t v;

  if (param_find("EXT_TX_RATE") < 0 ||
      param_find("EK3_EXT_M_NSE") < 0 ||
      param_find("EK3_EXT_I_GATE") < 0 ||
      param_find("EK3_EXT_YAW_NSE") < 0 ||
      param_find("EK3_EXT_TIMEOUT") < 0)
    {
      fail("external navigation schema is incomplete");
      return;
    }

  if (param_set_i32("SER_TEL2_FUNC", SER_FUNC_COMPANION) < 0 ||
      param_get_i32("SER_TEL2_FUNC", &v) < 0 || v != SER_FUNC_COMPANION)
    {
      fail("SER_*_FUNC does not accept the companion function");
    }

  if (param_set_i32("SER_TEL2_FUNC", SER_FUNC_COMPANION + 1) >= 0)
    {
      fail("SER_*_FUNC accepted a function that does not exist");
    }

  if (param_set_i32("SER_TEL2_FUNC", SER_FUNC_DISABLED) < 0)
    {
      fail("could not restore SER_TEL2_FUNC");
    }

  if (param_get_i32("EXT_TX_RATE", &v) < 0 || v != 50)
    {
      fail("EXT_TX_RATE does not default to 50 Hz");
    }

  if (param_get_f32("EK3_EXT_M_NSE", &value) < 0 ||
      fabsf(value - 0.10f) > 1.0e-6f)
    {
      fail("external position noise floor is not 0.10 m");
    }

  if (param_get_f32("EK3_EXT_YAW_NSE", &value) < 0 ||
      fabsf(value - 0.05f) > 1.0e-6f)
    {
      fail("external yaw noise floor is not 0.05 rad");
    }

  if (param_get_i32("EK3_EXT_TIMEOUT", &v) < 0 || v != 1000)
    {
      fail("external navigation timeout is not 1000 ms");
    }
}
```

- [ ] **Step 2: Run to verify it fails**

```bash
tools/test-param-range.sh
```

Expected: FAIL — `external navigation schema is incomplete`.

- [ ] **Step 3: Add the function value**

In `apps/param/param.h`, after the `SER_FUNC_CAL` block:

```c
/* Reserve the port for the companion computer link. Like SER_FUNC_CAL this
 * starts nothing at boot - `companion start` opens it - and for the same
 * reason: a shell here would sit blocked in read() stealing the link's
 * input.
 */

#define SER_FUNC_COMPANION 6
```

- [ ] **Step 4: Widen the range and add the parameters**

In `apps/param/param.c`, change **every** `SER_*_FUNC` entry's maximum from
`I32(5)` to `I32(6)` — there are seven: `TEL1`, `TEL2`, `TEL3`, `GPS1`,
`GPS2`, `DBG`, `USB`. Update `SER_TEL1_FUNC`'s description string to
`"TELEM1 func (0=off 1=NSH 2=MAVLink 3=GPS 4=RC 5=CAL 6=COMP)"`.

Missing one is silent: that port simply refuses the companion function with a
range error at the shell, which reads like a typo.

Then add, after the `EK3_YAW_I_GATE` entry:

```c
  /* ---- External navigation ----------------------------------------------
   * EK3_EXT_M_NSE and EK3_EXT_YAW_NSE are FLOORS, not defaults. The fused
   * noise is max(what the source reported, this) - which is what ArduPilot
   * does with posErr. A source claiming millimetre accuracy must not be able
   * to talk the filter into trusting it more than the operator configured.
   */

  { "EXT_TX_RATE", PARAM_TYPE_INT32, I32(50), I32(1), I32(400),
    "Companion pose transmit rate (Hz)" },
  { "EK3_EXT_M_NSE", PARAM_TYPE_FLOAT, F32(0.10f), F32(0.01f), F32(10.0f),
    "External position measurement noise floor (m)" },
  { "EK3_EXT_I_GATE", PARAM_TYPE_FLOAT, F32(5.0f), F32(1.0f), F32(100.0f),
    "External position innovation gate (sigma)" },
  { "EK3_EXT_YAW_NSE", PARAM_TYPE_FLOAT, F32(0.05f), F32(0.01f), F32(1.5f),
    "External yaw measurement noise floor (rad)" },
  { "EK3_EXT_TIMEOUT", PARAM_TYPE_INT32, I32(1000), I32(100), I32(10000),
    "External navigation dropout before position is dropped (ms)" },
```

- [ ] **Step 5: Reserve the port in the manager**

In `apps/serial/serial.c`, beside `case SER_FUNC_CAL:`:

```c
          case SER_FUNC_COMPANION:

            /* Deliberately starts nothing, exactly as SER_FUNC_CAL does. The
             * link needs the port to itself and a shell here would sit
             * blocked in read() stealing its input. `companion start` opens
             * it on demand.
             */

            syslog(LOG_INFO,
                   "serial: %s (%s) reserved for the companion link - "
                   "no shell\n", p->name, p->devpath);
            break;
```

- [ ] **Step 6: Run to verify it passes**

```bash
tools/test-param-range.sh && ./tools/build.sh
```

Expected: PASS, build exits 0.

- [ ] **Step 7: Commit**

```bash
git add apps/param/param.h apps/param/param.c apps/serial/serial.c \
        tests/param_range_test.c
git commit -m "param: reserve a port for the companion link

SER_FUNC_COMPANION starts nothing at boot, exactly as SER_FUNC_CAL does and
for the same reason: a shell on the port would sit blocked in read()
stealing the link's input.

Every SER_*_FUNC maximum moves from 5 to 6 together. Missing one is silent -
that port would simply refuse the companion function with a range error that
reads like a typo at the shell.

EK3_EXT_M_NSE and EK3_EXT_YAW_NSE are floors under whatever the source
reports, not defaults, matching ArduPilot's treatment of posErr.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: The daemon

**Files:**
- Create: `apps/companion/companion.h`, `apps/companion/companion.c`, `apps/companion/companion_main.c`
- Create: `apps/companion/Makefile`, `apps/companion/Make.defs`, `apps/companion/Kconfig`
- Modify: `boards/fmuv6c/configs/nsh/defconfig`

**Interfaces:**
- Consumes: `comp_proto.h` (Task 1), `external_pose_advertise/_publish` (Task 2), `SER_FUNC_COMPANION` and `EXT_TX_RATE` (Task 3), `serial_ports()` / `serial_port_count()` from `apps/serial/serial.h`.
- Produces: `struct companion_status_s`, `int companion_start(void)`, `int companion_stop(void)`, `void companion_status(FAR struct companion_status_s *out)`.

- [ ] **Step 1: Write the header**

Create `apps/companion/companion.h`:

```c
/****************************************************************************
 * apps/companion/companion.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The link to the companion computer.
 *
 * Owns one serial port - whichever SER_*_FUNC names SER_FUNC_COMPANION - and
 * routes framed packets to uORB topics by message id. It knows nothing about
 * navigation: adding a message is a row in the routing table plus a topic,
 * not a new code path, which is what the control trajectory will need.
 ****************************************************************************/

#ifndef __APPS_COMPANION_COMPANION_H
#define __APPS_COMPANION_COMPANION_H

#include <stdbool.h>
#include <stdint.h>

#include "comp_proto.h"

#ifndef FAR
#  define FAR
#endif

struct companion_status_s
{
  bool     running;
  char     port[16];          /* connector name, as silkscreened */
  uint32_t baud;
  uint32_t tx_rate_hz;

  uint64_t bytes_in;
  uint64_t bytes_out;
  uint32_t tx_frames;
  uint32_t tx_errors;

  uint32_t rx_pose;           /* EXTERNAL_POSE routed and published */
  uint32_t rx_publish_errors;
  uint64_t last_rx_us;        /* board time of the last accepted frame */

  struct comp_parser_s parser;  /* frames, crc_errors, unknown_id, ... */
};

int  companion_start(void);
int  companion_stop(void);
void companion_status(FAR struct companion_status_s *out);

#endif /* __APPS_COMPANION_COMPANION_H */
```

- [ ] **Step 2: Implement the daemon**

Create `apps/companion/companion.c`. Copy the lifecycle from
`apps/ekf3/ekf3.c` lines 26–50 and 233–284 exactly — the `g_lock` mutex, the
`volatile bool g_running` / `g_should_stop` pair, `task_create` plus the
1-second spin-wait in start, and the mirrored wait in stop.

```c
/****************************************************************************
 * apps/companion/companion.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/uorb.h>
#include <uORB/uORB.h>

#include "companion.h"
#include "../param/param.h"
#include "../serial/serial.h"
#include "../uorb_msgs/uorb_msgs.h"

#define COMP_PRIORITY  (SCHED_PRIORITY_DEFAULT + 12)
#define COMP_STACK     2560
#define COMP_READ_MAX  256

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool g_running;
static volatile bool g_should_stop;
static struct companion_status_s g_status;

static uint64_t comp_now_us(void)
{
  struct timespec t;

  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000ull + (uint64_t)t.tv_nsec / 1000ull;
}

static void status_publish(FAR const struct companion_status_s *s)
{
  pthread_mutex_lock(&g_lock);
  g_status = *s;
  pthread_mutex_unlock(&g_lock);
}

/* Find the port whose SER_*_FUNC names the companion link.
 *
 * Scanned rather than hardcoded the way cal does it: the companion is a real
 * peripheral that could be on any connector, and cal's fixed devpath is a
 * consequence of the GUI always being on USB.
 */

static int comp_find_port(FAR const struct serial_port_s **out)
{
  FAR const struct serial_port_s *ports = serial_ports();
  int n = serial_port_count();
  int i;

  for (i = 0; i < n; i++)
    {
      if (param_i32(ports[i].func_param) == SER_FUNC_COMPANION)
        {
          *out = &ports[i];
          return i;
        }
    }

  return -ENOENT;
}

static int comp_open(FAR const struct serial_port_s *p, uint32_t baud)
{
  struct termios tio;
  int fd = open(p->devpath, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd < 0)
    {
      return -errno;
    }

  if (tcgetattr(fd, &tio) == 0)
    {
      /* Raw, both directions. Canonical mode would buffer by line, echo
       * would feed our own frames back to us, and CR/LF translation would
       * corrupt any frame containing 0x0d - which a float payload does
       * routinely. The cal protocol makes the same argument.
       */

      cfmakeraw(&tio);
      cfsetispeed(&tio, baud);
      cfsetospeed(&tio, baud);
      tcsetattr(fd, TCSANOW, &tio);
    }

  return fd;
}

/* Route one decoded frame. Adding a message means adding a case and a topic;
 * nothing else in this file changes.
 */

static void comp_route(int id, FAR const struct comp_parser_s *parser,
                       int pose_pub, FAR struct companion_status_s *s)
{
  if (id == COMP_MSG_EXTERNAL_POSE)
    {
      struct comp_external_pose_s wire;
      struct external_pose_s out;

      memcpy(&wire, parser->payload, sizeof(wire));
      memset(&out, 0, sizeof(out));

      out.timestamp = comp_now_us();
      out.timestamp_sample = wire.timestamp_us;
      out.x = wire.x;
      out.y = wire.y;
      out.yaw = wire.yaw;
      memcpy(out.cov, wire.cov, sizeof(out.cov));
      out.flags = wire.flags;
      out.reset_counter = wire.reset_counter;

      if (external_pose_publish(pose_pub, &out) < 0)
        {
          s->rx_publish_errors++;
          return;
        }

      s->rx_pose++;
      s->last_rx_us = out.timestamp;
    }

  /* COMP_MSG_CONTROL_TRAJ and the timesync ids are reserved. The parser
   * already counted them as unknown; nothing to do here until they are
   * defined.
   */
}

static void comp_transmit(int fd, int est_sub,
                          FAR struct companion_status_s *s)
{
  struct estimator_state_s est;
  struct comp_estimator_pose_s wire;
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int n;

  if (orb_copy(ORB_ID(estimator_state), est_sub, &est) < 0)
    {
      return;
    }

  memset(&wire, 0, sizeof(wire));
  wire.timestamp_us = est.timestamp_sample;
  memcpy(wire.position, est.position, sizeof(wire.position));
  memcpy(wire.quaternion, est.quaternion, sizeof(wire.quaternion));
  memcpy(wire.velocity, est.velocity, sizeof(wire.velocity));
  wire.solution_status = est.solution_status;
  wire.reset_counter = (uint8_t)est.reset_counter;

  n = comp_encode(COMP_MSG_ESTIMATOR_POSE, &wire, sizeof(wire), frame,
                  sizeof(frame));

  if (n < 0)
    {
      s->tx_errors++;
      return;
    }

  if (write(fd, frame, (size_t)n) != n)
    {
      /* A companion that stopped reading backs the port up. Count it and
       * carry on: dropping a pose is correct, blocking the daemon is not.
       */

      s->tx_errors++;
      return;
    }

  s->bytes_out += (uint64_t)n;
  s->tx_frames++;
}

static int companion_daemon(int argc, FAR char *argv[])
{
  struct companion_status_s status;
  FAR const struct serial_port_s *port = NULL;
  struct pollfd pfd;
  uint64_t next_tx;
  uint64_t tx_interval;
  int fd = -1;
  int pose_pub = -1;
  int est_sub = -1;
  int result = EXIT_FAILURE;

  memset(&status, 0, sizeof(status));
  comp_parser_init(&status.parser);

  if (comp_find_port(&port) < 0)
    {
      syslog(LOG_ERR,
             "[companion] no port reserved; set a SER_*_FUNC to %d\n",
             SER_FUNC_COMPANION);
      goto out;
    }

  strncpy(status.port, port->name, sizeof(status.port) - 1);
  status.baud = (uint32_t)param_i32(port->baud_param);
  status.tx_rate_hz = (uint32_t)param_i32("EXT_TX_RATE");
  tx_interval = 1000000ull / (status.tx_rate_hz > 0 ?
                              status.tx_rate_hz : 1);

  fd = comp_open(port, status.baud);

  if (fd < 0)
    {
      syslog(LOG_ERR, "[companion] cannot open %s: %d\n",
             port->devpath, -fd);
      goto out;
    }

  pose_pub = external_pose_advertise();

  if (pose_pub < 0)
    {
      syslog(LOG_ERR, "[companion] cannot advertise external_pose (%d)\n",
             errno);
      goto out;
    }

  /* The estimator may not be running. That is not a failure: the link still
   * receives, and starts transmitting when ekf3 comes up.
   */

  est_sub = orb_subscribe(ORB_ID(estimator_state));

  pfd.fd = fd;
  pfd.events = POLLIN;
  next_tx = comp_now_us();

  g_running = true;
  status.running = true;
  status_publish(&status);

  syslog(LOG_INFO, "[companion] %s at %" PRIu32 " baud, pose out at "
                   "%" PRIu32 " Hz\n",
         port->name, status.baud, status.tx_rate_hz);

  while (!g_should_stop)
    {
      uint8_t buf[COMP_READ_MAX];
      uint64_t now;
      ssize_t got;
      int ready = poll(&pfd, 1, 20);

      if (ready < 0 && errno != EINTR)
        {
          break;
        }

      if ((pfd.revents & POLLIN) != 0)
        {
          got = read(fd, buf, sizeof(buf));

          if (got > 0)
            {
              ssize_t i;

              status.bytes_in += (uint64_t)got;

              for (i = 0; i < got; i++)
                {
                  int id = comp_parser_byte(&status.parser, buf[i]);

                  if (id != 0)
                    {
                      comp_route(id, &status.parser, pose_pub, &status);
                    }
                }
            }
        }

      now = comp_now_us();

      if (est_sub >= 0 && now >= next_tx)
        {
          comp_transmit(fd, est_sub, &status);

          /* Advance from the deadline, not from now, so the rate does not
           * drift with scheduling. Resynchronise after a long stall rather
           * than bursting to catch up.
           */

          next_tx += tx_interval;

          if (next_tx < now)
            {
              next_tx = now + tx_interval;
            }
        }

      status_publish(&status);
    }

  result = EXIT_SUCCESS;

out:
  status.running = false;
  status_publish(&status);

  if (est_sub >= 0)
    {
      orb_unsubscribe(est_sub);
    }

  if (pose_pub >= 0)
    {
      orb_unadvertise(pose_pub);
    }

  if (fd >= 0)
    {
      close(fd);
    }

  g_running = false;
  return result;
}

int companion_start(void)
{
  int task;
  int wait;

  if (g_running)
    {
      return -EALREADY;
    }

  g_should_stop = false;
  task = task_create("companion", COMP_PRIORITY, COMP_STACK,
                     companion_daemon, NULL);

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

int companion_stop(void)
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

void companion_status(FAR struct companion_status_s *out)
{
  pthread_mutex_lock(&g_lock);
  *out = g_status;
  pthread_mutex_unlock(&g_lock);
}
```

- [ ] **Step 3: Write the command**

Create `apps/companion/companion_main.c`, following
`apps/sensors/sensors_main.c`'s dispatch shape:

```c
/****************************************************************************
 * apps/companion/companion_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `companion start | stop | status`
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "companion.h"
#include "../param/param.h"

static void usage(void)
{
  printf("Usage: companion start | stop | status\n"
         "\n"
         "  Links the companion computer over the port whose SER_*_FUNC is\n"
         "  %d. Routes framed packets to uORB topics by message id and\n"
         "  sends the estimator pose back at EXT_TX_RATE Hz.\n"
         "\n"
         "  The port must be reserved BEFORE boot - a shell started on it\n"
         "  at boot outlives the parameter change:\n"
         "    param set SER_TEL2_FUNC %d\n"
         "    param save\n"
         "    reboot\n", SER_FUNC_COMPANION, SER_FUNC_COMPANION);
}

static void print_status(void)
{
  struct companion_status_s s;

  companion_status(&s);

  if (!s.running)
    {
      printf("companion: stopped\n");
      return;
    }

  printf("companion: running on %s at %" PRIu32 " baud\n", s.port, s.baud);
  printf("  in   %" PRIu64 " bytes  frames %" PRIu32
         "  pose %" PRIu32 "\n",
         s.bytes_in, s.parser.frames, s.rx_pose);
  printf("  out  %" PRIu64 " bytes  frames %" PRIu32
         "  at %" PRIu32 " Hz  errors %" PRIu32 "\n",
         s.bytes_out, s.tx_frames, s.tx_rate_hz, s.tx_errors);

  /* Unknown id and bad length are shown apart deliberately: the first is a
   * companion newer than this firmware and is fine, the second is the two
   * ends disagreeing about a format and is not.
   */

  printf("  rx faults  crc %" PRIu32 "  unknown_id %" PRIu32
         "  bad_length %" PRIu32 "  publish %" PRIu32 "\n",
         s.parser.crc_errors, s.parser.unknown_id, s.parser.bad_length,
         s.rx_publish_errors);
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
      ret = companion_start();

      if (ret == -EALREADY)
        {
          printf("companion: already running\n");
          return EXIT_FAILURE;
        }

      if (ret < 0)
        {
          printf("companion: failed to start (%d) - check the syslog; no\n"
                 "           reserved port is the usual cause\n", ret);
          return EXIT_FAILURE;
        }

      print_status();
      return EXIT_SUCCESS;
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      ret = companion_stop();

      if (ret == -ESRCH)
        {
          printf("companion: not running\n");
          return EXIT_FAILURE;
        }

      printf("companion: %s\n", ret == OK ? "stopped" : "did not stop");
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

`apps/companion/Make.defs`:

```make
############################################################################
# apps/companion/Make.defs
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

ifneq ($(CONFIG_XXCAR_COMPANION),)
CONFIGURED_APPS += $(APPDIR)/xxcar/companion
endif
```

`apps/companion/Makefile`:

```make
############################################################################
# apps/companion/Makefile
#
# SPDX-License-Identifier: Apache-2.0
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_XXCAR_COMPANION_PROGNAME)
PRIORITY  = $(CONFIG_XXCAR_COMPANION_PRIORITY)
STACKSIZE = $(CONFIG_XXCAR_COMPANION_STACKSIZE)
MODULE    = $(CONFIG_XXCAR_COMPANION)

# comp_proto.c is the wire format, with no I/O so it can be host-tested;
# companion.c is the daemon; companion_main.c is the command.
CSRCS   = companion.c comp_proto.c
MAINSRC = companion_main.c

include $(APPDIR)/Application.mk
```

`apps/companion/Kconfig`:

```
#
# For a description of the syntax of this configuration file,
# see the file kconfig-language.txt in the NuttX tools repository.
#

config XXCAR_COMPANION
	tristate "companion: companion computer link"
	default n
	depends on UORB
	select XXCAR_PARAM
	select XXCAR_UORB_MSGS
	---help---
		Owns the serial port whose SER_*_FUNC is 6 and routes framed
		packets to uORB topics by message id, publishing external_pose.
		The estimator pose is sent back at EXT_TX_RATE Hz.

		The router knows nothing about navigation: adding a message is a
		row in the routing table plus a topic, not a new code path.

if XXCAR_COMPANION

config XXCAR_COMPANION_PROGNAME
	string "Program name"
	default "companion"

config XXCAR_COMPANION_PRIORITY
	int "command priority"
	default 100

config XXCAR_COMPANION_STACKSIZE
	int "command stack size"
	default 2560

endif
```

Enable it in `boards/fmuv6c/configs/nsh/defconfig`, keeping the file's
alphabetical ordering:

```
CONFIG_XXCAR_COMPANION=y
```

- [ ] **Step 5: Build and gate**

```bash
./tools/build.sh && ./tools/verify.sh 2>&1 | grep -E "FAIL|exited|rebuilt|over 80"
```

Expected: build exits 0; only `test-cpu-runtime` fails.

- [ ] **Step 6: Commit**

```bash
git add apps/companion boards/fmuv6c/configs/nsh/defconfig
git commit -m "companion: add the link daemon

Owns whichever port SER_*_FUNC names, and routes by message id. The port is
scanned rather than hardcoded the way cal does it - cal's fixed devpath is a
consequence of the GUI always being on USB, and the companion is a real
peripheral that could be on any connector.

The estimator subscription is optional: ekf3 may not be running, and the
link should still receive and start transmitting when it comes up.

A write that does not complete counts an error and carries on. A companion
that stopped reading backs the port up, and dropping a pose is correct where
blocking the daemon is not.

Nothing consumes external_pose yet - estimator behaviour is unchanged by
this commit.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

**This is the Phase 1 flash.** Verify on hardware before starting Task 5.

---

## Phase 2 — fusion

### Task 5: Extract the shared yaw update

A pure refactor. The existing tests must pass unchanged before and after,
which is what proves it changed nothing.

**Files:**
- Modify: `apps/ekf3/ekf_core.c`

**Interfaces:**
- Produces: `static int fuse_yaw(FAR struct ekf_core_s *ekf, float yaw_meas, float noise, float gate_sigma, FAR float *nis)` — returns 1 accepted, 0 gated, −1 numerical failure, matching the other updates.

- [ ] **Step 1: Confirm the baseline passes**

```bash
tools/test-ekf-core.sh
```

Expected: PASS. If it does not, stop — the refactor's only evidence is that
this keeps passing.

- [ ] **Step 2: Add the shared function**

In `apps/ekf3/ekf_core.c`, immediately before `ekf_core_fuse_mag`:

```c
/* Fuse an absolute yaw measurement.
 *
 * Shared by the magnetometer and by external navigation. Two copies of this
 * would be two ideas about what a yaw update is, free to drift apart while
 * both look correct.
 *
 * A yaw error is a rotation about the navigation UP axis. The state is a
 * BODY-frame rotation vector, so the observation is that axis expressed in
 * body - the third row of the body-to-nav rotation, which is exactly the
 * gauge direction the gravity update projects OUT.
 */

static int fuse_yaw(FAR struct ekf_core_s *ekf, float yaw_meas, float noise,
                    float gate_sigma, FAR float *nis)
{
  float h[EKF_STATE_DIM];
  float rotation[3][3];
  float euler[3];
  float residual;
  int axis;

  if (!isfinite(yaw_meas) || !isfinite(noise) || noise <= 0.0f)
    {
      return -1;
    }

  ekf_core_euler(ekf, euler);
  residual = wrap_pi(yaw_meas - euler[2]);

  quaternion_to_rotation(ekf->quaternion, rotation);
  memset(h, 0, sizeof(h));

  for (axis = 0; axis < 3; axis++)
    {
      h[axis] = rotation[2][axis];
    }

  return measurement_update_1d(ekf, h, residual, noise * noise, gate_sigma,
                               nis);
}
```

- [ ] **Step 3: Call it from the magnetometer path**

In `ekf_core_fuse_mag`, replace everything from `ekf_core_euler(ekf, euler);`
down to and including the `measurement_update_1d(...)` call with:

```c
  result = fuse_yaw(ekf, heading, noise, gate_sigma, &ekf->last_mag_nis);
```

Keep `ekf->last_mag_heading = heading;` above it. Delete the now-unused
`euler`, `rotation`, `h`, `residual` and `axis` locals from
`ekf_core_fuse_mag` — the compiler will name them.

- [ ] **Step 4: Prove it changed nothing**

```bash
tools/test-ekf-core.sh && ./tools/build.sh
```

Expected: PASS, build exits 0. The magnetometer tests exercise the yaw path
end to end; passing unchanged is the evidence the extraction was
behaviour-preserving.

- [ ] **Step 5: Commit**

```bash
git add apps/ekf3/ekf_core.c
git commit -m "ekf3: share one yaw update between magnetometer and external nav

Extracted unchanged from ekf_core_fuse_mag. Two copies would be two ideas
about what a yaw update is, free to drift apart while both look correct.

The existing magnetometer tests exercise this path end to end and pass
unchanged, which is the evidence the extraction was behaviour-preserving.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: The external navigation sample and queue

**Files:**
- Modify: `apps/ekf3/ekf_delay.h`, `apps/ekf3/ekf_delay.c`
- Modify: `tests/ekf_delay_test.c`

**Interfaces:**
- Consumes: the existing ring and queue machinery.
- Produces: `struct ekf_extnav_sample_s`, `EKF_EXTNAV_QUEUE_SIZE`, `bool ekf_delay_push_extnav(FAR struct ekf_delay_s *d, FAR const struct ekf_extnav_sample_s *s)`, `bool ekf_delay_next_extnav(FAR struct ekf_delay_s *d, uint64_t horizon_time, uint64_t max_age_us, FAR struct ekf_extnav_sample_s *out)`, and `d->extnav_overflow_count`.

- [ ] **Step 1: Write the failing test**

Add to `tests/ekf_delay_test.c` and register in `main()`:

```c
/* External navigation arrives slowly and matters a lot, so the queue is
 * short and dropping the OLDEST is right - a stale absolute fix is worth
 * less than a fresh one.
 */

static void test_extnav_queue(void)
{
  struct ekf_extnav_sample_s out;
  struct ekf_extnav_sample_s s;
  int i;

  ekf_delay_init(&g_delay, 0);

  for (i = 0; i < EKF_EXTNAV_QUEUE_SIZE + 2; i++)
    {
      memset(&s, 0, sizeof(s));
      s.timestamp_sample = (uint64_t)(i + 1) * 20000;
      s.x = (float)i;
      s.valid = true;
      ekf_delay_push_extnav(&g_delay, &s);
    }

  assert(g_delay.extnav_overflow_count == 2);
  assert(ekf_delay_next_extnav(&g_delay, 10000000, 10000000, &out));
  assert(out.timestamp_sample == 3 * 20000);   /* first two dropped */
  assert(out.valid);
}

static void test_extnav_respects_the_horizon_and_age(void)
{
  struct ekf_extnav_sample_s out;
  struct ekf_extnav_sample_s old;
  struct ekf_extnav_sample_s fresh;

  memset(&old, 0, sizeof(old));
  memset(&fresh, 0, sizeof(fresh));
  old.timestamp_sample = 1000;
  fresh.timestamp_sample = 900000;

  ekf_delay_init(&g_delay, 0);
  assert(ekf_delay_push_extnav(&g_delay, &old));
  assert(ekf_delay_push_extnav(&g_delay, &fresh));

  /* Nothing is due before the horizon reaches it. */

  assert(!ekf_delay_next_extnav(&g_delay, 500, 500000, &out));

  /* The 1000 sample is 999 ms old at a horizon of 1000000 - past the bound,
   * so it is discarded rather than fused where the filter now is.
   */

  assert(ekf_delay_next_extnav(&g_delay, 1000000, 500000, &out));
  assert(out.timestamp_sample == 900000);
  assert(!ekf_delay_next_extnav(&g_delay, 1000000, 500000, &out));
}
```

- [ ] **Step 2: Run to verify it fails**

```bash
tools/test-ekf-delay.sh
```

Expected: FAIL — `unknown type name 'struct ekf_extnav_sample_s'`.

- [ ] **Step 3: Implement**

In `apps/ekf3/ekf_delay.h`, beside the baro and mag sample types:

```c
#define EKF_EXTNAV_QUEUE_SIZE     4

/* An absolute pose from the companion computer.
 *
 * pos_sigma and yaw_sigma are the source's own reported standard deviations,
 * already square-rooted from the covariance diagonal. Zero means the source
 * supplied no estimate; the parameter floor applies either way.
 */

struct ekf_extnav_sample_s
{
  uint64_t timestamp_sample;
  float    x;
  float    y;
  float    yaw;
  float    pos_sigma[2];    /* x, y */
  float    yaw_sigma;
  uint8_t  reset_counter;   /* the SOURCE's frame-reset generation */
  bool     valid;
};
```

Add to `struct ekf_delay_s`:

```c
  struct ekf_extnav_sample_s extnav[EKF_EXTNAV_QUEUE_SIZE];
  uint16_t extnav_head;
  uint16_t extnav_count;
  uint32_t extnav_overflow_count;
```

and the two prototypes:

```c
bool ekf_delay_push_extnav(FAR struct ekf_delay_s *d,
                           FAR const struct ekf_extnav_sample_s *sample);
bool ekf_delay_next_extnav(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                           uint64_t max_age_us,
                           FAR struct ekf_extnav_sample_s *out);
```

In `apps/ekf3/ekf_delay.c`, beside the magnetometer pair:

```c
bool ekf_delay_push_extnav(FAR struct ekf_delay_s *d,
                           FAR const struct ekf_extnav_sample_s *sample)
{
  bool lost = false;

  if (d == NULL || sample == NULL)
    {
      return false;
    }

  if (d->extnav_count == EKF_EXTNAV_QUEUE_SIZE)
    {
      d->extnav_overflow_count++;
      d->extnav_count--;
      lost = true;
    }

  d->extnav[d->extnav_head] = *sample;
  d->extnav_head = (uint16_t)((d->extnav_head + 1) %
                              EKF_EXTNAV_QUEUE_SIZE);
  d->extnav_count++;
  return !lost;
}

bool ekf_delay_next_extnav(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                           uint64_t max_age_us,
                           FAR struct ekf_extnav_sample_s *out)
{
  if (d == NULL || out == NULL)
    {
      return false;
    }

  while (d->extnav_count > 0)
    {
      uint16_t index = ring_oldest(d->extnav_head, d->extnav_count,
                                   EKF_EXTNAV_QUEUE_SIZE);
      uint64_t stamp = d->extnav[index].timestamp_sample;

      if (stamp > horizon_time)
        {
          return false;
        }

      d->extnav_count--;

      if (horizon_time - stamp <= max_age_us)
        {
          *out = d->extnav[index];
          return true;
        }

      /* Too old to fuse where the filter now is. Drop and look at the next. */
    }

  return false;
}
```

`ring_oldest()` is the existing static helper in that file; it needs no
change.

- [ ] **Step 4: Run to verify it passes**

```bash
tools/test-ekf-delay.sh
```

Expected: `ekf_delay: horizon, ring ordering and overflow verified - OK`

- [ ] **Step 5: Commit**

```bash
git add apps/ekf3/ekf_delay.h apps/ekf3/ekf_delay.c tests/ekf_delay_test.c
git commit -m "ekf3: queue external navigation samples at the horizon

Same ring as the magnetometer and barometer, so an absolute pose is fused at
the point on the trajectory where it was actually taken rather than wherever
the filter has since propagated to.

The queue is short and drops the oldest on overflow: a stale absolute fix is
worth less than a fresh one.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: Datum and external navigation fusion

**Files:**
- Modify: `apps/ekf3/ekf_core.h`, `apps/ekf3/ekf_core.c`
- Modify: `tests/ekf_core_test.c`

**Interfaces:**
- Consumes: `fuse_yaw()` (Task 5), `struct ekf_extnav_sample_s` (Task 6), `measurement_update_1d`, `covariance_reset_attitude`.
- Produces: `int ekf_core_fuse_extnav(FAR struct ekf_core_s *ekf, FAR const struct ekf_extnav_sample_s *s, float pos_noise_floor, float pos_gate, float yaw_noise_floor, float yaw_gate, bool want_position, bool want_yaw)` returning 1 accepted, 0 gated, −1 rejected, −2 datum set; plus `ekf_core_s` fields `extnav_datum_set`, `extnav_source_reset`, `have_extnav_reset`, `last_extnav_innov[2]`, `last_extnav_nis[2]`, `last_extnav_noise`, `last_extnav_timestamp`, `extnav_accept_count`, `extnav_reject_count`, `extnav_consecutive_rejects`, `extnav_datum_count`, `extnav_stale_count`.

- [ ] **Step 1: Extend the state and declare the interface**

In `apps/ekf3/ekf_core.h`, add to `struct ekf_core_s` beside the barometer
block:

```c
  bool     extnav_datum_set;
  bool     have_extnav_reset;
  uint8_t  extnav_source_reset;    /* last seen source reset generation */
  float    last_extnav_innov[2];   /* m, x and y */
  float    last_extnav_nis[2];
  float    last_extnav_noise;      /* m, AFTER the floor was applied */
  uint64_t last_extnav_timestamp;  /* filter time of the last acceptance */

  uint32_t extnav_accept_count;
  uint32_t extnav_reject_count;
  uint32_t extnav_consecutive_rejects;
  uint32_t extnav_datum_count;
  uint32_t extnav_stale_count;
  uint32_t extnav_timeout_us;      /* EK3_EXT_TIMEOUT, via the setter */
```

and the constants and prototype:

```c
/* A rejection run this long means the filter and the source disagree about
 * where the vehicle is, not that one reading was bad. Re-datum rather than
 * going on rejecting every pose forever - ArduPilot's ResetPositionNE().
 */

#define EKF_EXTNAV_REJECT_RUN_MAX   20u

/* Set the dropout after which horizontal validity is withdrawn.
 *
 * A setter rather than a parameter read inside the core, matching
 * ekf_core_set_mag_config: the core reads no parameters, so it stays testable
 * without a parameter file.
 */

void ekf_core_set_extnav_config(FAR struct ekf_core_s *ekf,
                                uint32_t timeout_us);

/* Fuse an absolute pose from the companion computer.
 *
 * Returns 1 accepted, 0 gated, -1 rejected as unusable, and -2 when this
 * sample BECAME the datum and the filter was set rather than corrected.
 *
 * pos_noise_floor and yaw_noise_floor are FLOORS under whatever the source
 * reported, not defaults - the fused noise is the larger of the two. A source
 * claiming millimetre accuracy must not be able to talk the filter into
 * trusting it more than the operator configured. ArduPilot does the same with
 * posErr.
 */

int ekf_core_fuse_extnav(FAR struct ekf_core_s *ekf,
                         FAR const struct ekf_extnav_sample_s *s,
                         float pos_noise_floor, float pos_gate,
                         float yaw_noise_floor, float yaw_gate,
                         bool want_position, bool want_yaw);
```

Add `#include "ekf_delay.h"` to `ekf_core.h`? **No** — `ekf_delay.h` already
includes `ekf_core.h`, so that would be circular. Forward-declare instead,
above the prototype:

```c
struct ekf_extnav_sample_s;
```

- [ ] **Step 2: Write the failing test**

Add `#include "ekf_delay.h"` to `tests/ekf_core_test.c`'s includes. The test
uses `struct ekf_extnav_sample_s` BY VALUE, and `ekf_core.h` only
forward-declares it — a forward declaration is enough for the pointer in the
prototype and not enough to declare a local.

`tools/test-ekf-core.sh` compiles only `ekf_core.c`, so also add
`"$REPO/apps/ekf3/ekf_delay.c"` to both `cc` invocations in that script,
after `ekf_core.c`.

Then add to `tests/ekf_core_test.c` and register each in `main()`:

```c
static struct ekf_extnav_sample_s extnav_at(float x, float y, float yaw)
{
  struct ekf_extnav_sample_s s;

  memset(&s, 0, sizeof(s));
  s.x = x;
  s.y = y;
  s.yaw = yaw;
  s.valid = true;
  return s;
}

/* The first pose SETS the filter rather than correcting it.
 *
 * The map origin may be tens of metres from where the filter aligned. Fusing
 * would make the first innovation enormous, the gate would reject it, and it
 * would go on rejecting every subsequent pose forever.
 */

static void test_extnav_first_pose_sets_the_datum(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s ekf;
  uint64_t timestamp = 1000000ull;
  struct ekf_extnav_sample_s s = extnav_at(120.0f, -45.0f, 0.7f);
  float euler[3];

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);

  assert(!ekf.extnav_datum_set);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, true) == -2);
  assert(ekf.extnav_datum_set);
  assert(ekf.extnav_datum_count == 1);

  assert_near(ekf.position[0], 120.0f, 1.0e-4f);
  assert_near(ekf.position[1], -45.0f, 1.0e-4f);
  ekf_core_euler(&ekf, euler);
  assert_near(euler[2], 0.7f, 1.0e-4f);

  /* The datum is horizontal only. Height belongs to the barometer, and
   * roll and pitch to gravity.
   */

  assert_near(ekf.position[2], 0.0f, 1.0e-6f);
  assert_near(euler[0], 0.0f, 1.0e-3f);
  assert_near(euler[1], 0.0f, 1.0e-3f);
  assert(ekf.extnav_accept_count == 0);
}

static void test_extnav_fuses_after_the_datum(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s ekf;
  uint64_t timestamp = 1000000ull;
  struct ekf_extnav_sample_s s = extnav_at(10.0f, 5.0f, 0.0f);
  float before;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, true) == -2);

  before = ekf.covariance[EKF_P_INDEX(6, 6)];
  s.x = 10.05f;
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, true) == 1);
  assert(ekf.extnav_accept_count == 1);
  assert(ekf.covariance[EKF_P_INDEX(6, 6)] < before);
  assert_covariance_positive_definite(&ekf);
}

/* The parameter is a FLOOR. A source claiming 1 mm must be fused at the
 * configured 0.1 m, not at what it claimed.
 */

static void test_extnav_noise_floor_wins(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s ekf;
  uint64_t timestamp = 1000000ull;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  s.pos_sigma[0] = 0.001f;
  s.pos_sigma[1] = 0.001f;

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == 1);
  assert_near(ekf.last_extnav_noise, 0.1f, 1.0e-6f);
}

/* A source reporting WORSE than the floor is believed - the floor is a
 * minimum, not a fixed value.
 */

static void test_extnav_honours_a_worse_reported_sigma(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s ekf;
  uint64_t timestamp = 1000000ull;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  s.pos_sigma[0] = 2.0f;
  s.pos_sigma[1] = 2.0f;

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == 1);
  assert_near(ekf.last_extnav_noise, 2.0f, 1.0e-6f);
}

static void test_extnav_refuses_an_invalid_pose(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s ekf;
  uint64_t timestamp = 1000000ull;
  struct ekf_extnav_sample_s s = extnav_at(1.0f, 2.0f, 0.0f);

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  s.valid = false;

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, true) == -1);
  assert(!ekf.extnav_datum_set);
}

/* A sustained rejection run re-datums rather than rejecting forever.
 *
 * After a dropout the filter has drifted, so every incoming pose looks
 * impossible and the gate rejects it. Only a reset recovers - ArduPilot's
 * ResetPositionNE().
 */

static void test_extnav_rejection_run_forces_a_redatum(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s ekf;
  uint64_t timestamp = 1000000ull;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  unsigned i;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);

  /* Tighten the position variance so a distant pose is genuinely outside
   * the gate rather than merely surprising.
   */

  for (i = 0; i < 30; i++)
    {
      ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);
    }

  s.x = 500.0f;

  for (i = 0; i < EKF_EXTNAV_REJECT_RUN_MAX; i++)
    {
      assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                                  true, false) == 0);
    }

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert_near(ekf.position[0], 500.0f, 1.0e-3f);
  assert(ekf.extnav_datum_count == 2);
  assert(ekf.extnav_consecutive_rejects == 0);
}

/* The source telling us it relocalised is worth more than twenty gated
 * innovations. Re-datum immediately.
 */

static void test_extnav_source_reset_forces_a_redatum(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s ekf;
  uint64_t timestamp = 1000000ull;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);

  s.reset_counter = 1;
  s.x = 77.0f;
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert_near(ekf.position[0], 77.0f, 1.0e-3f);
  assert(ekf.extnav_datum_count == 2);
}

/* Silence withdraws horizontal validity.
 *
 * A source that simply stops talking leaves no rejections behind, so without
 * an age check the claim would stand for ever on a dead link - the worst
 * kind of stale, because everything downstream still believes it.
 */

static void test_extnav_timeout_drops_horizontal_validity(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s ekf;
  uint64_t timestamp = 1000000ull;
  struct ekf_extnav_sample_s s = extnav_at(1.0f, 2.0f, 0.0f);
  struct ekf_imu_sample_s sample;
  float accel[3];
  int i;

  ekf_core_init(&ekf);
  ekf_core_set_extnav_config(&ekf, 1000000u);      /* 1 s */
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == 1);
  assert((ekf_core_solution_status(&ekf) &
          EKF_SOLUTION_POSITION_HORIZ) != 0);

  /* Propagate two seconds with no external pose at all. */

  rest_accel(0.0f, 0.0f, accel);

  for (i = 0; i < 800; i++)
    {
      timestamp += TEST_DT_US;
      make_sample(&sample, timestamp, accel, zero_gyro);
      ekf_core_process(&ekf, &sample);
    }

  assert((ekf_core_solution_status(&ekf) &
          EKF_SOLUTION_POSITION_HORIZ) == 0);
  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_ATTITUDE) != 0);
}

/* An alignment restart discards the datum. The local frame is gone, so
 * claiming map coordinates for it would be a lie.
 */

static void test_extnav_datum_clears_on_restart(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s ekf;
  uint64_t timestamp = 1000000ull;
  struct ekf_extnav_sample_s s = extnav_at(3.0f, 4.0f, 0.0f);
  struct ekf_imu_sample_s sample;
  float accel[3];

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert(ekf.extnav_datum_set);

  /* An uncalibrated packet restarts alignment. */

  rest_accel(0.0f, 0.0f, accel);
  timestamp += TEST_DT_US;
  make_sample(&sample, timestamp, accel, zero_gyro);
  sample.accel_calibrated = false;
  ekf_core_process(&ekf, &sample);

  assert(!ekf.extnav_datum_set);
}
```

- [ ] **Step 3: Run to verify it fails**

```bash
tools/test-ekf-core.sh
```

Expected: FAIL — `implicit declaration of function 'ekf_core_fuse_extnav'`.

- [ ] **Step 4: Add the position covariance reset**

In `apps/ekf3/ekf_core.c`, beside `covariance_reset_attitude`:

```c
/* Reset the horizontal position states and their covariance.
 *
 * Zeroing the row and column, not just the diagonal: a cross-covariance to
 * velocity or attitude describes a correlation with the OLD position, and
 * keeping it would let the next update correct velocity using a relationship
 * that no longer exists.
 */

static void covariance_reset_position_xy(FAR struct ekf_core_s *ekf,
                                         float variance)
{
  int axis;
  int i;

  for (axis = 6; axis <= 7; axis++)
    {
      for (i = 0; i < EKF_STATE_DIM; i++)
        {
          ekf->covariance[EKF_P_INDEX(axis, i)] = 0.0f;
          ekf->covariance[EKF_P_INDEX(i, axis)] = 0.0f;
        }

      ekf->covariance[EKF_P_INDEX(axis, axis)] = variance;
    }
}
```

Add the equivalent for yaw. A yaw datum is a finite rotation, not a small
error, so `covariance_reset_attitude` — which linearises about a small
correction — is the wrong tool:

```c
/* Reset yaw to an absolute value and its covariance with it.
 *
 * NOT covariance_reset_attitude: that linearises about a small correction,
 * and a datum yaw is a finite rotation. Roll and pitch are preserved
 * exactly - they come from gravity and the external source says nothing
 * about them.
 */

static void reset_yaw_absolute(FAR struct ekf_core_s *ekf, float yaw,
                               float variance)
{
  float euler[3];
  int i;

  ekf_core_euler(ekf, euler);
  quaternion_from_euler(euler[0], euler[1], yaw, ekf->quaternion);
  quaternion_normalize(ekf->quaternion);

  for (i = 0; i < EKF_STATE_DIM; i++)
    {
      ekf->covariance[EKF_P_INDEX(2, i)] = 0.0f;
      ekf->covariance[EKF_P_INDEX(i, 2)] = 0.0f;
    }

  ekf->covariance[EKF_P_INDEX(2, 2)] = variance;
}
```

- [ ] **Step 5: Implement the fusion**

Append to `apps/ekf3/ekf_core.c`:

```c
static void extnav_set_datum(FAR struct ekf_core_s *ekf,
                             FAR const struct ekf_extnav_sample_s *s,
                             float pos_noise, float yaw_noise,
                             bool want_position, bool want_yaw)
{
  if (want_position)
    {
      ekf->position[0] = s->x;
      ekf->position[1] = s->y;
      covariance_reset_position_xy(ekf, pos_noise * pos_noise);
    }

  if (want_yaw)
    {
      reset_yaw_absolute(ekf, s->yaw, yaw_noise * yaw_noise);
      ekf->yaw_absolute = true;
    }

  ekf->extnav_datum_set = true;
  ekf->extnav_datum_count++;
  ekf->extnav_consecutive_rejects = 0;
  ekf->last_extnav_noise = pos_noise;
  ekf->last_extnav_timestamp = ekf->last_timestamp_sample;
}

int ekf_core_fuse_extnav(FAR struct ekf_core_s *ekf,
                         FAR const struct ekf_extnav_sample_s *s,
                         float pos_noise_floor, float pos_gate,
                         float yaw_noise_floor, float yaw_gate,
                         bool want_position, bool want_yaw)
{
  float pos_noise;
  float yaw_noise;
  float h[EKF_STATE_DIM];
  int accepted = 0;
  int gated = 0;
  int axis;

  if (ekf == NULL || s == NULL || !ekf->initialized || !s->valid ||
      !isfinite(s->x) || !isfinite(s->y) || !isfinite(s->yaw) ||
      !isfinite(pos_noise_floor) || pos_noise_floor <= 0.0f ||
      !isfinite(yaw_noise_floor) || yaw_noise_floor <= 0.0f)
    {
      return -1;
    }

  /* The parameter is a floor under whatever the source reported, not a
   * default. ArduPilot does the same with posErr: a source claiming
   * millimetre accuracy must not talk the filter into trusting it more than
   * the operator configured.
   */

  pos_noise = pos_noise_floor;

  for (axis = 0; axis < 2; axis++)
    {
      if (isfinite(s->pos_sigma[axis]) && s->pos_sigma[axis] > pos_noise)
        {
          pos_noise = s->pos_sigma[axis];
        }
    }

  yaw_noise = isfinite(s->yaw_sigma) && s->yaw_sigma > yaw_noise_floor ?
              s->yaw_sigma : yaw_noise_floor;

  /* The source relocalised. That is worth more than twenty gated
   * innovations telling us the same thing more slowly.
   */

  if (!ekf->have_extnav_reset)
    {
      ekf->extnav_source_reset = s->reset_counter;
      ekf->have_extnav_reset = true;
    }
  else if (s->reset_counter != ekf->extnav_source_reset)
    {
      ekf->extnav_source_reset = s->reset_counter;
      extnav_set_datum(ekf, s, pos_noise, yaw_noise, want_position,
                       want_yaw);
      return -2;
    }

  if (!ekf->extnav_datum_set ||
      ekf->extnav_consecutive_rejects >= EKF_EXTNAV_REJECT_RUN_MAX)
    {
      extnav_set_datum(ekf, s, pos_noise, yaw_noise, want_position,
                       want_yaw);
      return -2;
    }

  ekf->last_extnav_noise = pos_noise;

  /* North then East as sequential SCALAR updates, ignoring the measurement
   * cross-covariance. This is what ArduPilot's FuseVelPosNED does; carrying
   * cov's off-diagonals without using them is deliberate, not an oversight.
   */

  if (want_position)
    {
      const float measured[2] = {s->x, s->y};

      for (axis = 0; axis < 2; axis++)
        {
          int result;

          memset(h, 0, sizeof(h));
          h[6 + axis] = 1.0f;
          ekf->last_extnav_innov[axis] =
            measured[axis] - ekf->position[axis];

          result = measurement_update_1d(ekf, h,
                                         ekf->last_extnav_innov[axis],
                                         pos_noise * pos_noise, pos_gate,
                                         &ekf->last_extnav_nis[axis]);

          if (result < 0)
            {
              return -1;
            }

          if (result == 0)
            {
              gated++;
            }
          else
            {
              accepted++;
            }
        }
    }

  if (want_yaw)
    {
      float nis = 0.0f;
      int result = fuse_yaw(ekf, s->yaw, yaw_noise, yaw_gate, &nis);

      if (result < 0)
        {
          return -1;
        }

      if (result == 0)
        {
          gated++;
        }
      else
        {
          accepted++;
          ekf->yaw_absolute = true;
        }
    }

  if (accepted > 0)
    {
      ekf->extnav_accept_count++;
      ekf->extnav_consecutive_rejects = 0;
      ekf->last_extnav_timestamp = ekf->last_timestamp_sample;
      return 1;
    }

  if (gated > 0)
    {
      ekf->extnav_reject_count++;
      ekf->extnav_consecutive_rejects++;
      return 0;
    }

  return -1;
}
```

- [ ] **Step 6: Clear the datum on an alignment restart**

In `restart_alignment()`, beside the barometer reference reset:

```c
  /* The local frame is gone, so map coordinates for it would be a lie. */

  ekf->extnav_datum_set = false;
  ekf->have_extnav_reset = false;
  ekf->extnav_consecutive_rejects = 0;
```

- [ ] **Step 7: Report horizontal validity**

In `ekf_core_solution_status()`, after the vertical block:

```c
  /* Horizontal validity is a claim about external navigation actually
   * correcting, not about it being selected. A sustained rejection run
   * withdraws it, and so does silence: a source that simply stopped talking
   * leaves no rejections behind, so without the age check the claim would
   * stand for ever on a dead link.
   */

  if (ekf->extnav_datum_set && ekf->extnav_accept_count > 0 &&
      ekf->extnav_consecutive_rejects < EKF_EXTNAV_REJECT_RUN_MAX &&
      ekf->extnav_timeout_us > 0 &&
      ekf->last_timestamp_sample >= ekf->last_extnav_timestamp &&
      ekf->last_timestamp_sample - ekf->last_extnav_timestamp <
        (uint64_t)ekf->extnav_timeout_us)
    {
      status |= EKF_SOLUTION_POSITION_HORIZ | EKF_SOLUTION_VELOCITY_HORIZ;
    }
```

And the setter itself, beside `ekf_core_set_mag_config`:

```c
void ekf_core_set_extnav_config(FAR struct ekf_core_s *ekf,
                                uint32_t timeout_us)
{
  if (ekf != NULL)
    {
      ekf->extnav_timeout_us = timeout_us;
    }
}
```

Delete the comment above the old `return status;` claiming nothing makes
horizontal states observable — it is no longer true.

- [ ] **Step 8: Run to verify it passes**

```bash
tools/test-ekf-core.sh && tools/test-ekf-baro.sh && tools/test-ekf-output.sh
```

Expected: all PASS, including the UBSan pass.

If `test_extnav_rejection_run_forces_a_redatum` fails because the 500 m pose
is *accepted*, the thirty priming updates did not tighten the variance
enough — raise the count rather than loosening the gate, because the gate
value is the thing under test.

- [ ] **Step 9: Commit**

```bash
git add apps/ekf3/ekf_core.h apps/ekf3/ekf_core.c tests/ekf_core_test.c
git commit -m "ekf3: fuse the companion computer's absolute pose

The first pose SETS position and yaw rather than correcting them. The map
origin may be tens of metres from where the filter aligned, so fusing would
make the first innovation enormous, the gate would reject it, and it would
go on rejecting every pose after it forever.

The noise parameter is a floor under the source's reported sigma, not a
default - ArduPilot's treatment of posErr. x and y are sequential scalar
updates that ignore the measurement cross-covariance, which is what
FuseVelPosNED does; carrying cov's off-diagonals unused is deliberate.

Two things force a re-datum: a sustained rejection run, which after a
dropout is the only way out of a gate deadlock, and the source telling us
its own frame reset, which is worth more than twenty gated innovations
saying the same thing more slowly.

A yaw datum uses a finite-rotation reset rather than
covariance_reset_attitude, which linearises about a small correction. Roll
and pitch are preserved exactly - they come from gravity and the external
source says nothing about them.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 8: Wire it into the daemon and report

**Files:**
- Modify: `apps/ekf3/ekf3.h`, `apps/ekf3/ekf3.c`, `apps/ekf3/ekf3_main.c`

**Interfaces:**
- Consumes: everything above.
- Produces: no new API.

- [ ] **Step 1: Extend the status**

In `apps/ekf3/ekf3.h`, add to `struct ekf3_status_s`:

```c
  uint32_t extnav_in;         /* external_pose messages queued */
  uint32_t extnav_overflow;
  uint32_t extnav_bad_time;   /* refused on the timestamp check */
  bool     extnav_available;  /* external_pose subscribed */
  float    ext_noise;         /* EK3_EXT_M_NSE as read at start */
  float    ext_gate;
  float    ext_yaw_noise;
  float    yaw_gate;          /* EK3_YAW_I_GATE, shared by mag and extnav */
  uint32_t ext_timeout_ms;
```

- [ ] **Step 2: Subscribe and drain**

In `apps/ekf3/ekf3.c`, add the age bound beside the others:

```c
/* Matching the barometer and magnetometer bounds. Past this the filter has
 * propagated somewhere else and the correction would land on the wrong part
 * of the trajectory.
 */

#define EKF3_EXT_MAX_AGE_US   500000ull
```

Add a drain helper beside `drain_baro`:

```c
static void drain_extnav(int sub, FAR struct ekf3_status_s *status)
{
  int drained = 0;

  if (sub < 0)
    {
      return;
    }

  while (drained++ < EKF3_DRAIN_MAX)
    {
      struct external_pose_s message;
      struct ekf_extnav_sample_s sample;
      uint64_t now;

      if (orb_copy(ORB_ID(external_pose), sub, &message) < 0)
        {
          return;
        }

      now = now_us();

      /* The canary for a timesync that is not working. A clock that is
       * grossly wrong would otherwise corrupt position silently, which is
       * the worst failure available here.
       *
       * The two bounds do different jobs. The age bound is the queue's - past
       * it the correction lands on the wrong part of the trajectory. A
       * timestamp AHEAD of our own clock has no such excuse: it means the
       * timesync is wrong, full stop.
       */

      if (message.timestamp_sample > now + EKF3_EXT_MAX_AGE_US ||
          (now > message.timestamp_sample &&
           now - message.timestamp_sample > EKF3_EXT_MAX_AGE_US))
        {
          status->extnav_bad_time++;
          continue;
        }

      memset(&sample, 0, sizeof(sample));
      sample.timestamp_sample = message.timestamp_sample;
      sample.x = message.x;
      sample.y = message.y;
      sample.yaw = message.yaw;

      /* cov holds variances; the sample carries sigmas. Zero means the
       * source supplied no estimate, and the floor applies either way.
       */

      sample.pos_sigma[0] = message.cov[0] > 0.0f ?
                            sqrtf(message.cov[0]) : 0.0f;
      sample.pos_sigma[1] = message.cov[3] > 0.0f ?
                            sqrtf(message.cov[3]) : 0.0f;
      sample.yaw_sigma = message.cov[5] > 0.0f ?
                         sqrtf(message.cov[5]) : 0.0f;
      sample.reset_counter = message.reset_counter;
      sample.valid = (message.flags & EXTERNAL_POSE_VALID) != 0;

      ekf_delay_push_extnav(&g_delay, &sample);
      status->extnav_in++;
    }
}
```

Add `#include <math.h>` if `sqrtf` does not resolve.

Subscribe alongside the mag and baro subscriptions, following their exact
optional pattern (`extnav_sub = orb_subscribe(ORB_ID(external_pose));`, add
to `fds` when non-negative, set `status.extnav_available`, unsubscribe in
`out:`), read the parameters where the others are read:

```c
  status.ext_noise = param_f32("EK3_EXT_M_NSE");
  status.ext_gate = param_f32("EK3_EXT_I_GATE");
  status.ext_yaw_noise = param_f32("EK3_EXT_YAW_NSE");
  status.yaw_gate = param_f32("EK3_YAW_I_GATE");
  status.ext_timeout_ms = (uint32_t)param_i32("EK3_EXT_TIMEOUT");
  ekf_core_set_extnav_config(&status.core,
                             status.ext_timeout_ms * 1000u);
```

and call `drain_extnav(extnav_sub, &status);` beside the other drains.

- [ ] **Step 3: Fuse at the horizon**

Inside the `while (ekf_delay_next_imu(...))` loop, after the barometer block:

```c
          {
            FAR const struct ekf_source_set_s *src =
              &status.sources.set[status.sources.active_set];
            bool want_position =
              src->position_xy == EKF_SOURCE_EXTERNAL_NAV;
            bool want_yaw = src->yaw == EKF_SOURCE_EXTERNAL_NAV;
            struct ekf_extnav_sample_s ext;

            while (ekf_delay_next_extnav(&g_delay, sample.timestamp_sample,
                                         EKF3_EXT_MAX_AGE_US, &ext))
              {
                /* Source selection makes a measurement ELIGIBLE. Health
                 * gating inside the fusion decides whether it is USED.
                 */

                if (want_position || want_yaw)
                  {
                    ekf_core_fuse_extnav(&status.core, &ext,
                                         status.ext_noise,
                                         status.ext_gate,
                                         status.ext_yaw_noise,
                                         status.yaw_gate,
                                         want_position, want_yaw);
                  }
              }
          }
```

Mirror the overflow counter beside the existing three:

```c
      status.extnav_overflow = g_delay.extnav_overflow_count;
```

- [ ] **Step 4: Report**

In `apps/ekf3/ekf3_main.c`, after the barometer block:

```c
  if (core->extnav_datum_set)
    {
      printf("  extnav datum set  innov %+.3f %+.3f m  NIS %.3f %.3f\n",
             (double)core->last_extnav_innov[0],
             (double)core->last_extnav_innov[1],
             (double)core->last_extnav_nis[0],
             (double)core->last_extnav_nis[1]);

      /* The noise ACTUALLY used, after the floor. A source under-reporting
       * its error is invisible otherwise.
       */

      printf("    accept %" PRIu32 " reject %" PRIu32 " (run %" PRIu32
             ")  redatum %" PRIu32 "  noise %.3f m\n",
             core->extnav_accept_count, core->extnav_reject_count,
             core->extnav_consecutive_rejects, core->extnav_datum_count,
             (double)core->last_extnav_noise);
    }
  else
    {
      printf("  extnav no datum yet (queued %" PRIu32 ", bad time %" PRIu32
             ")\n", status.extnav_in, status.extnav_bad_time);
    }
```

Extend the ring overflow line to include `status.extnav_overflow`.

- [ ] **Step 5: Full gate**

```bash
./tools/verify.sh 2>&1 | grep -E "FAIL|exited|rebuilt|over 80"
```

Expected: only `test-cpu-runtime`.

- [ ] **Step 6: Commit**

```bash
git add apps/ekf3/ekf3.h apps/ekf3/ekf3.c apps/ekf3/ekf3_main.c
git commit -m "ekf3: fuse external navigation at the horizon

Subscribes external_pose optionally, exactly as it does vehicle_mag: its
absence means no aiding, not a failure to start.

A timestamp ahead of board time, or older than 500 ms, is refused and
counted before it reaches the queue. That is the canary for a timesync that
is not working - without it a wrong clock corrupts position silently.

The status line reports the noise ACTUALLY used after the floor, because a
source under-reporting its error is otherwise invisible.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Hardware verification

### After Task 4 — the link, estimator unchanged

```
param set SER_TEL2_FUNC 6
param set SER_TEL2_BAUD 921600
param save
reboot
companion start
companion status
```

1. `ekf3 status` must be **identical to before the flash** — no aiding is
   wired up yet. If anything moved, stop.
2. With the Jetson sending poses: `frames` climbs, `crc 0`, `pose` climbs.
   `uorb_listener external_pose -n 5` shows sensible x, y, yaw.
3. With the Jetson silent but the port connected: `out` bytes climb at
   `EXT_TX_RATE`, `tx errors 0`.
4. **Deliberately corrupt one byte** on the companion side. `crc` increments
   and `frames` keeps climbing — the parser resynchronised rather than
   wedging.
5. Send an unknown message id. `unknown_id` climbs; `bad_length` stays 0.
   Then send `EXTERNAL_POSE` with a wrong length: `bad_length` climbs. These
   must be distinguishable, because one is benign and the other is not.

### After Task 8 — fusion

```
param set EK3_SRC1_POSXY 6
param set EK3_SRC1_YAW 6
param save
ekf3 stop && ekf3 start
ekf3 status
```

1. `extnav no datum yet` until the first pose, then `extnav datum set` and
   `position NED` jumps to the map coordinates. That jump is expected —
   confirm the Jetson sees `reset_counter` increment.
2. Innovations settle near zero, `reject 0`, `run 0`. `POSXY` appears in the
   solution line.
3. `noise` shows `0.100` while the Jetson sends zero covariance. Have it send
   a **1 mm** sigma: `noise` must stay `0.100`. That is the floor working,
   and it is the detail most likely to surprise the Jetson team.
4. **Unplug the Jetson** for longer than `EK3_EXT_TIMEOUT`. `POSXY` drops
   from the solution line, attitude and height are unaffected, and
   reconnecting re-datums rather than sitting in a reject run.
5. Drive a known path. Position should track the map rather than drifting.

## Known limitations

Carried from the spec, because they shape what a failure means:

- **Horizontal velocity is still unaided.** External position bounds
  position; velocity integrates from accelerometers between updates. Fine at
  50 Hz, visibly worse if the companion is slow. Optical flow is what fixes
  that, and is a separate subsystem.
- **The datum reset is a discontinuity**, by design. `reset_counter` in the
  outgoing pose exists so the companion can see it.
- **A clock off by 50 ms passes every sanity check**, showing up only as
  position error proportional to speed.
