/****************************************************************************
 * tests/comp_proto_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <math.h>
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

static size_t sample_trajectory(uint8_t *payload)
{
  uint64_t timestamp = 1234567890123ull;
  uint64_t solution = 1234567880000ull;
  float values[8] = {1.0f, 2.0f, 1.5f, 2.5f,
                     -0.25f, 0.2f, 0.5f, -0.1f};

  memset(payload, 0, COMP_MAX_PAYLOAD);
  memcpy(payload + COMP_TRAJ_TIMESTAMP_OFS, &timestamp, sizeof(timestamp));
  memcpy(payload + COMP_TRAJ_SOLUTION_OFS, &solution, sizeof(solution));
  payload[COMP_TRAJ_HORIZON_OFS] = 2;
  payload[COMP_TRAJ_DT_OFS] = 0x66;       /* binary16 0.05 = 0x2a66 */
  payload[COMP_TRAJ_DT_OFS + 1] = 0x2a;
  payload[COMP_TRAJ_METHOD_OFS] = COMP_THROTTLE_DUTY;
  memcpy(payload + COMP_TRAJ_DATA_OFS, values, sizeof(values));
  return comp_control_trajectory_payload_size(2);
}

/* The layout is the wire format. If these ever disagree the companion reads
 * convincing nonsense, so the compiler proves them instead.
 */

static void test_layout(void)
{
  assert(sizeof(struct comp_external_pose_s) == 48);
  assert(sizeof(struct comp_vehicle_state_s) == 96);
  assert(offsetof(struct comp_vehicle_state_s, rc_status) == 92);
  assert(comp_payload_len(COMP_MSG_EXTERNAL_POSE) == 48);
  assert(comp_payload_len(COMP_MSG_VEHICLE_STATE) == 96);
  assert(comp_payload_len(COMP_MSG_TIMESYNC_REQ) == 8);
  assert(comp_payload_len(COMP_MSG_TIMESYNC_REP) == 24);
  assert(sizeof(struct comp_timesync_req_s) == 8);
  assert(sizeof(struct comp_timesync_rep_s) == 24);
  assert(sizeof(struct comp_direct_control_s) == 24);
  assert(comp_payload_len(COMP_MSG_DIRECT_CONTROL) == 24);
  assert(sizeof(struct comp_datum_reset_s) == 4);
  assert(comp_payload_len(COMP_MSG_DATUM_RESET) == 4);
  assert(comp_payload_len(COMP_MSG_CONTROL_TRAJ) == 0);
  assert(comp_control_trajectory_payload_size(1) == 36);
  assert(comp_control_trajectory_payload_size(14) == 244);
  assert(comp_control_trajectory_payload_size(0) == 0);
  assert(comp_control_trajectory_payload_size(15) == 0);
  assert(comp_payload_len(200) == 0);
}

static void test_datum_reset_round_trip(void)
{
  struct comp_datum_reset_s in;
  struct comp_datum_reset_s out;
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int n;

  in.request_counter = 0x12345678u;
  n = comp_encode(COMP_MSG_DATUM_RESET, &in, sizeof(in), frame,
                  sizeof(frame));
  assert(n == (int)sizeof(in) + COMP_FRAME_OVERHEAD);

  comp_parser_init(&g_parser);
  assert(feed(frame, (size_t)n) == COMP_MSG_DATUM_RESET);
  assert(g_parser.len == sizeof(in));
  memcpy(&out, g_parser.payload, sizeof(out));
  assert(out.request_counter == in.request_counter);
}

static void test_control_trajectory_round_trip(void)
{
  struct comp_control_trajectory_s decoded;
  uint8_t payload[COMP_MAX_PAYLOAD];
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  size_t len = sample_trajectory(payload);
  int n = comp_encode(COMP_MSG_CONTROL_TRAJ, payload, (uint8_t)len,
                      frame, sizeof(frame));

  comp_parser_init(&g_parser);
  assert(feed(frame, (size_t)n) == COMP_MSG_CONTROL_TRAJ);
  assert(g_parser.len == len);
  assert(comp_control_trajectory_decode(g_parser.payload, g_parser.len,
                                        &decoded));
  assert(decoded.timestamp_us == 1234567890123ull);
  assert(decoded.solution_time_us == 1234567880000ull);
  assert(decoded.horizon == 2);
  assert(fabsf(decoded.dt - 0.05f) < 0.0001f);
  assert(decoded.poses[0][0] == 1.0f);
  assert(decoded.poses[1][1] == 2.5f);
  assert(decoded.controls[0][0] == -0.25f);
  assert(decoded.controls[1][1] == -0.1f);
}

static void test_control_trajectory_rejects_bad_length_and_values(void)
{
  struct comp_control_trajectory_s decoded;
  uint8_t payload[COMP_MAX_PAYLOAD];
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  size_t len = sample_trajectory(payload);
  float invalid = 1.1f;
  int n;

  n = comp_encode(COMP_MSG_CONTROL_TRAJ, payload, (uint8_t)(len - 1),
                  frame, sizeof(frame));
  comp_parser_init(&g_parser);
  assert(feed(frame, (size_t)n) == 0);
  assert(g_parser.bad_length == 1);

  memcpy(payload + COMP_TRAJ_DATA_OFS + 4u * sizeof(float),
         &invalid, sizeof(invalid));
  assert(!comp_control_trajectory_decode(payload, len, &decoded));

  sample_trajectory(payload);
  payload[COMP_TRAJ_DT_OFS] = 0;
  payload[COMP_TRAJ_DT_OFS + 1] = 0;
  assert(!comp_control_trajectory_decode(payload, len, &decoded));
}

static struct comp_direct_control_s sample_command(void)
{
  struct comp_direct_control_s c;

  memset(&c, 0, sizeof(c));
  c.timestamp_us = 1234567890123ull;
  c.steering = -0.25f;
  c.throttle = 12.5f;
  c.throttle_type = COMP_THROTTLE_CURRENT;
  return c;
}

static void test_direct_control_round_trip(void)
{
  struct comp_direct_control_s in = sample_command();
  struct comp_direct_control_s out;
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int n = comp_encode(COMP_MSG_DIRECT_CONTROL, &in, sizeof(in),
                      frame, sizeof(frame));

  assert(n == (int)sizeof(in) + COMP_FRAME_OVERHEAD);

  comp_parser_init(&g_parser);
  assert(feed(frame, (size_t)n) == COMP_MSG_DIRECT_CONTROL);
  assert(g_parser.len == sizeof(in));

  memcpy(&out, g_parser.payload, sizeof(out));
  assert(out.timestamp_us == in.timestamp_us);
  assert(out.steering == in.steering);
  assert(out.throttle == in.throttle);
  assert(out.throttle_type == in.throttle_type);
}

/* The wire enum must be the topic enum.
 *
 * Not a tautology: if somebody renumbers one of these, the value that used to
 * mean amps starts meaning duty ratio, and 20 amps becomes duty 20 clamped to
 * full throttle. companion.c static_asserts the same pair against
 * ACTUATOR_MODE_*, which this cannot see from the host.
 */

static void test_throttle_modes_are_the_documented_numbers(void)
{
  assert(COMP_THROTTLE_DUTY == 0);
  assert(COMP_THROTTLE_CURRENT == 1);
}

static void test_direct_control_accepts_the_full_range(void)
{
  struct comp_direct_control_s c = sample_command();

  c.throttle_type = COMP_THROTTLE_DUTY;
  c.throttle = 0.0f;
  c.steering = 0.0f;
  assert(comp_direct_control_valid(&c));

  c.throttle = 1.0f;
  c.steering = 1.0f;
  assert(comp_direct_control_valid(&c));

  c.throttle = -1.0f;
  c.steering = -1.0f;
  assert(comp_direct_control_valid(&c));

  c.throttle_type = COMP_THROTTLE_CURRENT;
  c.throttle = 50.0f;
  assert(comp_direct_control_valid(&c));

  c.throttle = -50.0f;
  assert(comp_direct_control_valid(&c));
}

/* The limits are per mode, and swapping them is the mistake worth catching:
 * a duty of 12.5 is nine times full throttle, and a current of 0.9 A is a
 * command the vehicle would simply ignore.
 */

static void test_direct_control_limits_are_per_mode(void)
{
  struct comp_direct_control_s c = sample_command();

  c.throttle_type = COMP_THROTTLE_DUTY;
  c.throttle = 1.001f;
  assert(!comp_direct_control_valid(&c));

  c.throttle = -1.001f;
  assert(!comp_direct_control_valid(&c));

  /* Legal as amps, and it must not become legal as duty. */

  c.throttle = 12.5f;
  assert(!comp_direct_control_valid(&c));

  c.throttle_type = COMP_THROTTLE_CURRENT;
  assert(comp_direct_control_valid(&c));

  c.throttle = 50.001f;
  assert(!comp_direct_control_valid(&c));

  c.throttle = -50.001f;
  assert(!comp_direct_control_valid(&c));
}

/* The throttle value here is deliberately legal in BOTH modes.
 *
 * With the sample command's 12.5 the assertion still holds when the mode
 * check is deleted - 12.5 fails the duty limit an unknown mode falls back to
 * - so the test would pass while the rule it names had gone.
 */

static void test_direct_control_rejects_an_unknown_mode(void)
{
  struct comp_direct_control_s c = sample_command();

  c.throttle = 0.5f;

  c.throttle_type = COMP_THROTTLE_DUTY;
  assert(comp_direct_control_valid(&c));

  c.throttle_type = COMP_THROTTLE_CURRENT;
  assert(comp_direct_control_valid(&c));

  c.throttle_type = 2;
  assert(!comp_direct_control_valid(&c));

  c.throttle_type = 255;
  assert(!comp_direct_control_valid(&c));
}

static void test_direct_control_rejects_steering_out_of_range(void)
{
  struct comp_direct_control_s c = sample_command();

  c.steering = 1.001f;
  assert(!comp_direct_control_valid(&c));

  c.steering = -1.001f;
  assert(!comp_direct_control_valid(&c));
}

/* NaN compares false against everything, so the readable form of these range
 * tests - reject when x < lo || x > hi - lets a NaN through as "not out of
 * range" and puts it on the wire to the motor.
 */

static void test_direct_control_rejects_nan_and_infinity(void)
{
  struct comp_direct_control_s c = sample_command();
  const float nan_value = 0.0f / 0.0f;
  const float inf_value = 1.0f / 0.0f;

  c.throttle = nan_value;
  assert(!comp_direct_control_valid(&c));

  c.throttle = inf_value;
  assert(!comp_direct_control_valid(&c));

  c = sample_command();
  c.steering = nan_value;
  assert(!comp_direct_control_valid(&c));

  c.steering = -inf_value;
  assert(!comp_direct_control_valid(&c));

  assert(!comp_direct_control_valid(NULL));
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

/* A reply must carry the request's own host_tx back untouched, or a
 * companion with several requests in flight cannot tell which reply is
 * which - and would pair a reply with the wrong send time, producing an
 * offset that is confidently wrong.
 */

static void test_timesync_round_trip(void)
{
  struct comp_timesync_rep_s in;
  struct comp_timesync_rep_s out;
  uint8_t frame[COMP_MAX_PAYLOAD + COMP_FRAME_OVERHEAD];
  int n;

  memset(&in, 0, sizeof(in));
  in.host_tx_us = 111111111111ull;
  in.board_rx_us = 222222222222ull;
  in.board_tx_us = 333333333333ull;

  n = comp_encode(COMP_MSG_TIMESYNC_REP, &in, sizeof(in), frame,
                  sizeof(frame));

  comp_parser_init(&g_parser);
  assert(feed(frame, (size_t)n) == COMP_MSG_TIMESYNC_REP);
  memcpy(&out, g_parser.payload, sizeof(out));
  assert(out.host_tx_us == in.host_tx_us);
  assert(out.board_rx_us == in.board_rx_us);
  assert(out.board_tx_us == in.board_tx_us);
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
  test_timesync_round_trip();
  test_datum_reset_round_trip();
  test_direct_control_round_trip();
  test_control_trajectory_round_trip();
  test_control_trajectory_rejects_bad_length_and_values();
  test_throttle_modes_are_the_documented_numbers();
  test_direct_control_accepts_the_full_range();
  test_direct_control_limits_are_per_mode();
  test_direct_control_rejects_an_unknown_mode();
  test_direct_control_rejects_steering_out_of_range();
  test_direct_control_rejects_nan_and_infinity();
  test_encode_refuses_a_short_buffer();

  puts("comp_proto: framing, CRC, resync and length checks verified - OK");
  return 0;
}
