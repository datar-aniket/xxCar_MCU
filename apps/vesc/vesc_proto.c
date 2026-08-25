/****************************************************************************
 * apps/vesc/vesc_proto.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <math.h>

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

/* Shared tail for both command frames.
 *
 * The scale ROUNDS, it does not truncate. The nearest float to 0.29 is
 * 0.28999999165, so 0.29f * 100000.0f is 28999.999 and a plain cast ships
 * 28999. That is a 3% error that appears at some duty values and not others,
 * which is the hardest kind to notice on a bench.
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
      if (!isfinite(motor))
        {
          vesc_put_be32(&out[0], 0);
          vesc_put_be16(&out[4], (int16_t)vesc_clamp_servo(servo_us));
          return false;
        }

      motor = vesc_clampf(motor, -limit, limit);
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
