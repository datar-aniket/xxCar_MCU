/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <stdio.h>
#include "../boards/fmuv6c/src/ppm_decode.h"

static int frame(struct ppm_decoder_s *d, unsigned count)
{
  for (unsigned i = 0; i < count; i++)
    assert(ppm_interval(d, 1000 + i * 50) == 0);
  return ppm_interval(d, 4000);
}

int main(void)
{
  for (unsigned channels = 4; channels <= 18; channels++)
    {
      struct ppm_decoder_s d;
      ppm_reset(&d, channels);
      assert(ppm_interval(&d, 4000) == 0);
      assert(frame(&d, channels) == 0);
      assert(frame(&d, channels) == 1);
      for (unsigned i = 0; i < channels; i++)
        assert(d.channel[i] == 1000 + i * 50);
      /* One missing edge merging two 1500 us intervals into a false sync. */
      for (unsigned lost = 0; lost + 1 < channels; lost++)
        {
          for (unsigned i = 0; i < lost; i++)
            assert(ppm_interval(&d, 1500) == 0);
          assert(ppm_interval(&d, 3000) == -1);
          for (unsigned i = lost + 2; i < channels; i++)
            assert(ppm_interval(&d, 1500) == 0);
          assert(ppm_interval(&d, 4000) == -1);
          assert(frame(&d, channels) == 0);
          assert(frame(&d, channels) == 1);
        }
      assert(ppm_interval(&d, 100) == -1); /* Noise: reject, don't shift. */
      assert(ppm_interval(&d, 1000) == 0);
      assert(ppm_interval(&d, 4000) == 0);
      assert(frame(&d, channels) == 0);
      assert(frame(&d, channels) == 1);
      assert(frame(&d, channels - 1) == -1); /* Count never relearned. */
      assert(frame(&d, channels - 1) == -1);
    }
  puts("PPM: counts 4..18, every merged-edge position, noise, recovery PASS");
  return 0;
}
