/* SPDX-License-Identifier: Apache-2.0 */
#ifndef XXCAR_PPM_DECODE_H
#define XXCAR_PPM_DECODE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Fixed receiver channel count: do not learn a shortened frame after a
 * missing edge. Two complete frames qualify initial/recovered framing.
 */
struct ppm_decoder_s
{
  uint16_t channel[18];
  uint8_t expected;
  uint8_t count;
  uint8_t good;
  bool collecting;
};

static inline void ppm_reset(struct ppm_decoder_s *d, unsigned channels)
{
  memset(d, 0, sizeof(*d));
  d->expected = channels;
}

/* -1 rejected frame, 0 collecting/qualifying, +1 complete qualified frame.
 * Caller copies channel[] immediately on +1, before the next edge.
 */
static inline int ppm_interval(struct ppm_decoder_s *d, uint32_t us)
{
  int result = 0;
  if (us >= 2700)
    {
      if (d->collecting)
        {
          if (d->count == d->expected && us < 30000)
            {
              if (d->good < 2) d->good++;
              result = d->good == 2 ? 1 : 0;
            }
          else
            {
              d->good = 0;
              result = -1;
            }
        }
      d->count = 0;
      d->collecting = true;
    }
  else if (d->collecting)
    {
      if (us >= 750 && us <= 2250 && d->count < d->expected)
        {
          d->channel[d->count++] = us;
        }
      else
        {
          d->good = 0;
          d->collecting = false;
          d->count = 0;
          result = -1;
        }
    }
  return result;
}
#endif
