/****************************************************************************
 * apps/companion/comp_proto.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <errno.h>
#include <math.h>
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

/* One loop, not two that can diverge: the parser needs a resumable CRC and
 * the encoder needs a one-shot, and they must agree byte for byte.
 */

uint16_t comp_crc16(FAR const uint8_t *d, size_t n)
{
  return comp_crc16_update(0xffff, d, n);
}

uint8_t comp_payload_len(uint8_t id)
{
  switch (id)
    {
      case COMP_MSG_EXTERNAL_POSE:
        return (uint8_t)sizeof(struct comp_external_pose_s);

      case COMP_MSG_VEHICLE_STATE:
        return (uint8_t)sizeof(struct comp_vehicle_state_s);

      case COMP_MSG_TIMESYNC_REQ:
        return (uint8_t)sizeof(struct comp_timesync_req_s);

      case COMP_MSG_TIMESYNC_REP:
        return (uint8_t)sizeof(struct comp_timesync_rep_s);

      case COMP_MSG_TIMESYNC_START:
        return (uint8_t)sizeof(struct comp_timesync_start_s);

      case COMP_MSG_TIMESYNC_END:
        return (uint8_t)sizeof(struct comp_timesync_end_s);

      case COMP_MSG_DIRECT_CONTROL:
        return (uint8_t)sizeof(struct comp_direct_control_s);

      case COMP_MSG_DATUM_RESET:
        return (uint8_t)sizeof(struct comp_datum_reset_s);

      case COMP_MSG_CONTROL_TRAJ:
        return 0; /* variable; validated against its horizon in the parser */

      default:
        return 0;
    }
}

size_t comp_control_trajectory_payload_size(uint8_t horizon)
{
  if (horizon == 0 || horizon > COMP_TRAJ_MAX_HORIZON)
    {
      return 0;
    }

  return COMP_TRAJ_HEADER_SIZE + (size_t)horizon * COMP_TRAJ_STEP_SIZE;
}

static uint64_t get_u64_le(FAR const uint8_t *p)
{
  uint64_t value = 0;
  unsigned i;

  for (i = 0; i < 8; i++)
    {
      value |= (uint64_t)p[i] << (8u * i);
    }

  return value;
}

static float get_f32_le(FAR const uint8_t *p)
{
  uint32_t bits = (uint32_t)p[0] | (uint32_t)p[1] << 8 |
                  (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
  float value;

  memcpy(&value, &bits, sizeof(value));
  return value;
}

static float half_to_float(uint16_t half)
{
  uint32_t sign = (uint32_t)(half & 0x8000u) << 16;
  uint32_t exponent = (half >> 10) & 0x1fu;
  uint32_t fraction = half & 0x03ffu;
  uint32_t bits;
  float value;

  if (exponent == 0)
    {
      if (fraction == 0)
        {
          bits = sign;
        }
      else
        {
          int shift = 0;

          while ((fraction & 0x0400u) == 0)
            {
              fraction <<= 1;
              shift++;
            }

          fraction &= 0x03ffu;
          bits = sign | (uint32_t)(113 - shift) << 23 | fraction << 13;
        }
    }
  else if (exponent == 0x1fu)
    {
      bits = sign | 0x7f800000u | fraction << 13;
    }
  else
    {
      bits = sign | (exponent + 112u) << 23 | fraction << 13;
    }

  memcpy(&value, &bits, sizeof(value));
  return value;
}

bool comp_control_trajectory_decode(FAR const uint8_t *payload, size_t len,
                                    FAR struct comp_control_trajectory_s *out)
{
  size_t controls_offset;
  float limit;
  uint8_t horizon;
  unsigned i;

  if (payload == NULL || out == NULL || len < COMP_TRAJ_HEADER_SIZE)
    {
      return false;
    }

  horizon = payload[COMP_TRAJ_HORIZON_OFS];

  if (len != comp_control_trajectory_payload_size(horizon))
    {
      return false;
    }

  memset(out, 0, sizeof(*out));
  out->timestamp_us = get_u64_le(payload + COMP_TRAJ_TIMESTAMP_OFS);
  out->solution_time_us = get_u64_le(payload + COMP_TRAJ_SOLUTION_OFS);
  out->horizon = horizon;
  out->dt = half_to_float((uint16_t)payload[COMP_TRAJ_DT_OFS] |
                          (uint16_t)payload[COMP_TRAJ_DT_OFS + 1] << 8);
  out->control_method = payload[COMP_TRAJ_METHOD_OFS];

  if (out->timestamp_us == 0 || out->solution_time_us == 0 ||
      !(out->dt > 0.0f && isfinite(out->dt)) ||
      (out->control_method != COMP_THROTTLE_DUTY &&
       out->control_method != COMP_THROTTLE_CURRENT))
    {
      return false;
    }

  controls_offset = COMP_TRAJ_DATA_OFS + (size_t)horizon * 2u * sizeof(float);
  limit = out->control_method == COMP_THROTTLE_CURRENT ?
          COMP_DIRECT_CURRENT_MAX : COMP_DIRECT_DUTY_MAX;

  for (i = 0; i < horizon; i++)
    {
      size_t pose = COMP_TRAJ_DATA_OFS + (size_t)i * 2u * sizeof(float);
      size_t control = controls_offset + (size_t)i * 2u * sizeof(float);

      out->poses[i][0] = get_f32_le(payload + pose);
      out->poses[i][1] = get_f32_le(payload + pose + sizeof(float));
      out->controls[i][0] = get_f32_le(payload + control);
      out->controls[i][1] = get_f32_le(payload + control + sizeof(float));

      if (!isfinite(out->poses[i][0]) || !isfinite(out->poses[i][1]) ||
          !(out->controls[i][0] >= -COMP_DIRECT_STEER_MAX &&
            out->controls[i][0] <= COMP_DIRECT_STEER_MAX) ||
          !(out->controls[i][1] >= -limit &&
            out->controls[i][1] <= limit))
        {
          return false;
        }
    }

  return true;
}

/* Every test is written as !(x >= lo && x <= hi) rather than the readable
 * inversion, because NaN compares false against everything: the readable
 * form lets a NaN through as "not out of range" and puts it straight on the
 * wire to the motor.
 */

bool comp_direct_control_valid(FAR const struct comp_direct_control_s *cmd)
{
  float limit;

  if (cmd == NULL)
    {
      return false;
    }

  if (cmd->throttle_type != COMP_THROTTLE_DUTY &&
      cmd->throttle_type != COMP_THROTTLE_CURRENT)
    {
      return false;
    }

  limit = cmd->throttle_type == COMP_THROTTLE_CURRENT ?
          COMP_DIRECT_CURRENT_MAX : COMP_DIRECT_DUTY_MAX;

  if (!(cmd->throttle >= -limit && cmd->throttle <= limit))
    {
      return false;
    }

  if (!(cmd->steering >= -COMP_DIRECT_STEER_MAX &&
        cmd->steering <= COMP_DIRECT_STEER_MAX))
    {
      return false;
    }

  return true;
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

          if (p->id == COMP_MSG_CONTROL_TRAJ)
            {
              if (p->len < COMP_TRAJ_HEADER_SIZE ||
                  p->len != comp_control_trajectory_payload_size(
                    p->payload[COMP_TRAJ_HORIZON_OFS]))
                {
                  p->bad_length++;
                  return 0;
                }

              p->frames++;
              return p->id;
            }

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
