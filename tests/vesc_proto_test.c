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
  *   tachometer 0x0002CA7B = 182907
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
  assert(out.tachometer == 182907);
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
  *   0xFFFD3585 = -182907
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
  assert(out.tachometer == -182907);
}

/* The extremes of both signed fields, so a decoder that is right in the
 * middle of the range and wrong at the ends is caught.
 *
 * The ADC field is int16 per docs/can_packet.md, so its maximum is 0x7FFF
 * and not 0xFFFF - which decodes to -1, not 65535. A real steering voltage
 * never goes near either end, which is exactly why the type has to be
 * pinned here rather than inferred from plausible readings.
 */

static void test_status5_extremes(void)
{
  const uint8_t max_data[8] =
  {
    0x7f, 0xff, 0xff, 0xff,
    0x7f, 0xff,
    0x7f, 0xff
  };
  const uint8_t min_data[8] =
  {
    0x80, 0x00, 0x00, 0x00,
    0x80, 0x00,
    0x80, 0x00
  };
  struct vesc_status5_s out;

  assert(vesc_decode_status5(max_data, VESC_STATUS_5_DLC, &out));
  assert(out.tachometer == 2147483647);
  assert(CLOSE(out.current_a, 3276.7f));
  assert(CLOSE(out.adc_volts, 32.767f));

  assert(vesc_decode_status5(min_data, VESC_STATUS_5_DLC, &out));
  assert(out.tachometer == -2147483647 - 1);
  assert(CLOSE(out.current_a, -3276.8f));
  assert(CLOSE(out.adc_volts, -32.768f));
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
 *   800 us clamps to 900   = 0x0384
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
  assert(out[4] == 0x03 && out[5] == 0x84);
}

/*  0.53 duty x 100000 = 53000 = 0x0000CF08
 *  2200 us clamps to 2100     = 0x0834
 *
 * THIS TEST IS ABOUT ROUNDING, and the value is not arbitrary. The nearest
 * float to 0.53 is 0.52999997138977, and 0.53f * 100000.0f lands on the
 * float below 53000, so a truncating cast ships 52999 while rounding ships
 * 53000.
 *
 * Most values do NOT expose this. 0.29f * 100000.0f, for instance, rounds
 * up to exactly 29000.0f during the multiply, so truncation gets the right
 * answer there. The value had to be searched for rather than guessed - and
 * the bug it catches is a 1-count error that appears at some duty settings
 * and not others, which is the hardest kind to notice on a bench.
 */

static void test_encode_duty_rounds(void)
{
  uint8_t out[VESC_CMD_SERVO_DLC];

  assert(vesc_encode_duty_servo(0.53f, 2200, out));
  assert(out[0] == 0x00 && out[1] == 0x00 &&
         out[2] == 0xcf && out[3] == 0x08);
  assert(out[4] == 0x08 && out[5] == 0x34);
}

/*  8.03 A x 1000 = 8030 = 0x00001F5E
 *
 * The same trap on the current scale, which has its own factor and so its
 * own set of values that expose it.
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
  assert(out[4] == 0x03 && out[5] == 0x84);      /* 900 */

  assert(vesc_encode_duty_servo(0.0f, 5000, out));
  assert(out[4] == 0x08 && out[5] == 0x34);      /* 2100 */
}

static void test_encode_null(void)
{
  assert(!vesc_encode_current_servo(1.0f, 1500, NULL));
  assert(!vesc_encode_duty_servo(1.0f, 1500, NULL));
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

  puts("vesc_proto: decode and encode, byte order, scaling, sign - OK");
  return 0;
}
