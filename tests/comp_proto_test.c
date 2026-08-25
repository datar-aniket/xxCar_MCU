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
