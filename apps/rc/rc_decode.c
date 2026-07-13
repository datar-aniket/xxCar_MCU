/****************************************************************************
 * apps/rc/rc_decode.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SBUS and CRSF frame decoders.
 *
 * Both protocols pack 16 channels into 22 bytes as 11-bit little-endian
 * bitfields, so they share the unpacker. What differs is the framing, the
 * scaling, and the line settings - and the line settings differ so completely
 * (baud, parity, stop bits, polarity) that a port can only ever be listening
 * for one of them at a time.
 *
 * Constants and scaling are cross-checked against PX4's src/lib/rc/sbus.cpp and
 * crsf.cpp. This matters: the two use *different* raw ranges, and getting that
 * wrong gives channel values that look plausible and are wrong.
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>

#include "rc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Unpack 16 x 11-bit little-endian channels out of 22 bytes.
 *
 * Channel n occupies bits [11n, 11n+11) of the byte string, LSB first. Shared
 * by SBUS and CRSF, which pack identically even though they scale differently.
 */

static void rc_unpack11(FAR const uint8_t *d, FAR uint16_t *raw)
{
  unsigned bit = 0;
  unsigned i;

  for (i = 0; i < 16; i++)
    {
      unsigned byte = bit >> 3;
      unsigned shift = bit & 7;
      uint32_t word = (uint32_t)d[byte] | ((uint32_t)d[byte + 1] << 8);

      /* An 11-bit field starting at bit `shift` reaches into a third byte only
       * when shift > 5. Reading it unconditionally would run one byte past the
       * 22-byte payload on the last channel.
       */

      if (shift > 5)
        {
          word |= (uint32_t)d[byte + 2] << 16;
        }

      raw[i] = (uint16_t)((word >> shift) & 0x7ff);
      bit += 11;
    }
}

/* SBUS raw -> microseconds.
 *
 * PX4 maps 200..1800 onto 1000..2000us: scale 0.625 (= 5/8), offset 874. Done
 * in integer arithmetic - there is no reason to pull the FPU into an interrupt-
 * rate decode path.
 */

static uint16_t sbus_to_us(uint16_t raw)
{
  return (uint16_t)(((raw * 5) + 4) / 8 + 874);
}

/* CRSF raw -> microseconds.
 *
 * A DIFFERENT mapping from SBUS, despite the identical bit packing: CRSF is
 * defined as 172 -> 988us, 992 -> 1500us, 1811 -> 2012us. Scale is 1024/1639.
 */

static uint16_t crsf_to_us(uint16_t raw)
{
  return (uint16_t)(((uint32_t)raw * 1024u) / 1639u + 881u);
}

/* CRC-8/DVB-S2, polynomial 0xd5. Covers CRSF's type byte and payload. */

static uint8_t crsf_crc8(FAR const uint8_t *p, size_t len)
{
  uint8_t crc = 0;
  size_t i;
  int bit;

  for (i = 0; i < len; i++)
    {
      crc ^= p[i];

      for (bit = 0; bit < 8; bit++)
        {
          crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xd5)
                             : (uint8_t)(crc << 1);
        }
    }

  return crc;
}

/* One complete 25-byte SBUS frame. */

static bool sbus_frame(FAR const uint8_t *f, FAR struct rc_frame_s *out)
{
  uint16_t raw[16];
  unsigned i;

  /* The trailing byte distinguishes SBUS1 from the SBUS2 telemetry slots. We do
   * not use telemetry, but it is a useful sanity check on the framing: a byte
   * outside this set means we synchronised on a 0x0f that was really channel
   * data, and the "frame" is garbage.
   */

  if (f[24] != 0x00 && f[24] != 0x04 && f[24] != 0x14 &&
      f[24] != 0x24 && f[24] != 0x34)
    {
      return false;
    }

  rc_unpack11(&f[1], raw);

  memset(out, 0, sizeof(*out));
  out->count = SBUS_CHANNELS;

  for (i = 0; i < SBUS_CHANNELS; i++)
    {
      out->channel[i] = sbus_to_us(raw[i]);
    }

  out->failsafe   = (f[SBUS_FLAGS_BYTE] & SBUS_FAILSAFE_BIT) != 0;
  out->frame_lost = (f[SBUS_FLAGS_BYTE] & SBUS_FRAMELOST_BIT) != 0;

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void rc_decoder_reset(FAR struct rc_decoder_s *d, uint8_t proto)
{
  memset(d, 0, sizeof(*d));
  d->proto = proto;
}

bool rc_decode(FAR struct rc_decoder_s *d, FAR const uint8_t *data, size_t len,
               FAR struct rc_frame_s *out)
{
  bool got = false;
  size_t i;

  for (i = 0; i < len; i++)
    {
      uint8_t ch = data[i];

      if (d->proto == RC_PROTO_SBUS)
        {
          /* Only ever start a frame on the start byte. Anything else while
           * unsynchronised is thrown away, one byte at a time, until a 0x0f
           * turns up - which is also how we resynchronise after noise, and how
           * probing the WRONG protocol reliably fails instead of producing
           * plausible nonsense.
           */

          if (d->nbuf == 0 && ch != SBUS_START_BYTE)
            {
              continue;
            }

          d->buf[d->nbuf++] = ch;

          if (d->nbuf < SBUS_FRAME_SIZE)
            {
              continue;
            }

          d->nbuf = 0;

          if (sbus_frame(d->buf, out))
            {
              got = true;
            }
          else
            {
              /* Synchronised on a 0x0f that was really channel data. */

              d->errors++;
            }
        }
      else if (d->proto == RC_PROTO_CRSF)
        {
          /* [sync][len][type][payload...][crc8]
           *
           * `len` counts everything after itself, i.e. type + payload + crc.
           */

          if (d->nbuf == 0)
            {
              if (ch != CRSF_SYNC_BYTE)
                {
                  continue;
                }

              d->buf[d->nbuf++] = ch;
              continue;
            }

          if (d->nbuf == 1)
            {
              /* A length outside this range cannot be a real frame, so the
               * "sync" byte was payload. Drop it and hunt again.
               */

              if (ch < 2 || ch > CRSF_PAYLOAD_MAX + 2)
                {
                  d->nbuf = 0;
                  continue;
                }

              d->buf[d->nbuf++] = ch;
              d->want = ch;
              continue;
            }

          d->buf[d->nbuf++] = ch;

          if (--d->want > 0)
            {
              continue;
            }

          /* Frame complete: buf = [sync][len][type ... crc]. The CRC covers the
           * type and payload, not the sync or length bytes.
           */

          {
            unsigned n = d->nbuf;
            uint8_t crc = d->buf[n - 1];
            unsigned body = n - 3;   /* type + payload */

            d->nbuf = 0;

            if (crsf_crc8(&d->buf[2], body) != crc)
              {
                d->errors++;
                continue;
              }

            if (d->buf[2] != CRSF_TYPE_RC_PACKED || body != 23)
              {
                /* A valid frame, but telemetry/link-stats rather than channels.
                 * Not an error - just not ours.
                 */

                continue;
              }

            {
              uint16_t raw[16];
              unsigned c;

              rc_unpack11(&d->buf[3], raw);

              memset(out, 0, sizeof(*out));
              out->count = CRSF_CHANNELS;

              for (c = 0; c < CRSF_CHANNELS; c++)
                {
                  out->channel[c] = crsf_to_us(raw[c]);
                }

              /* CRSF's channel frame carries no failsafe bit. Loss of link is
               * detected by the frame simply not arriving - see RC_TIMEOUT_US.
               */

              got = true;
            }
          }
        }
    }

  return got;
}
