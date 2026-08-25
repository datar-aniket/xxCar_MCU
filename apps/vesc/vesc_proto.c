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
