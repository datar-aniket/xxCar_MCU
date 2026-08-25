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

      case COMP_MSG_ESTIMATOR_POSE:
        return (uint8_t)sizeof(struct comp_estimator_pose_s);

      case COMP_MSG_TIMESYNC_REQ:
        return (uint8_t)sizeof(struct comp_timesync_req_s);

      case COMP_MSG_TIMESYNC_REP:
        return (uint8_t)sizeof(struct comp_timesync_rep_s);

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
