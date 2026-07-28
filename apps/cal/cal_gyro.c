/****************************************************************************
 * apps/cal/cal_gyro.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See cal_gyro.h.
 ****************************************************************************/

#ifndef CAL_GYRO_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <string.h>
#include <math.h>

#include "cal_gyro.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void cal_bias_reset(FAR struct cal_bias_s *b)
{
  memset(b, 0, sizeof(*b));
}

void cal_bias_add(FAR struct cal_bias_s *b, FAR const float v[3])
{
  int k;

  b->n++;

  for (k = 0; k < 3; k++)
    {
      /* Welford. Each sample updates the mean, and the squared-deviation sum
       * is accumulated about the RUNNING mean rather than about zero - so
       * there is never a large quantity to subtract away. See cal_gyro.h for
       * the measurement that made this necessary rather than tidy.
       */

      double x = (double)v[k];
      double d = x - b->mean[k];

      b->mean[k] += d / b->n;
      b->m2[k]   += d * (x - b->mean[k]);
    }
}

int cal_bias_result(FAR const struct cal_bias_s *b, FAR float mean[3],
                    FAR float sd[3])
{
  int k;

  if (b->n <= 0)
    {
      return 0;
    }

  for (k = 0; k < 3; k++)
    {
      double var = b->m2[k] / b->n;

      /* m2 is a sum of products of like-signed deviations and cannot be
       * negative, but rounding at the last bit can still produce -0.0.
       */

      if (var < 0.0)
        {
          var = 0.0;
        }

      mean[k] = (float)b->mean[k];
      sd[k]   = (float)sqrt(var);
    }

  return b->n;
}

enum cal_gyro_verdict_e cal_gyro_judge(int n, FAR const float mean[3],
                                       FAR const float sd[3])
{
  int k;

  if (n < CAL_GYRO_MIN_N)
    {
      return CAL_GYRO_TOO_FEW;
    }

  for (k = 0; k < 3; k++)
    {
      /* Written as !(x <= limit) so a NaN - which compares false against
       * everything, and would otherwise sail through - is refused.
       */

      if (!(sd[k] <= CAL_GYRO_SD_MAX))
        {
          return CAL_GYRO_NOT_STILL;
        }
    }

  for (k = 0; k < 3; k++)
    {
      if (!(fabsf(mean[k]) <= CAL_GYRO_BIAS_MAX))
        {
          return CAL_GYRO_TOO_LARGE;
        }
    }

  return CAL_GYRO_OK;
}
