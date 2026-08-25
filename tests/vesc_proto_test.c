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
