/****************************************************************************
 * apps/cal/cal_still.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See cal_still.h.
 ****************************************************************************/

#ifndef CAL_STILL_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <string.h>
#include <math.h>

#include "cal_still.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void cal_still_reset(FAR struct cal_still_s *s, float gyro_thresh,
                     float accel_thresh, int min_samples)
{
  memset(s, 0, sizeof(*s));
  s->gyro_thresh  = gyro_thresh;
  s->accel_thresh = accel_thresh;
  s->min_samples  = min_samples;
}

bool cal_still_update(FAR struct cal_still_s *s, FAR const float acc[3],
                      FAR const float gyr[3])
{
  bool moving = false;
  int i;

  /* Rotation shows up in the gyro immediately and unambiguously, so it is the
   * primary test.
   */

  for (i = 0; i < 3; i++)
    {
      if (fabsf(gyr[i]) > s->gyro_thresh)
        {
          moving = true;
        }
    }

  /* Pure translation - a slide or a tap - barely moves the gyro but does move
   * the accelerometer away from the running mean. Checking against the mean
   * rather than against a fixed value is what makes this work in any
   * orientation.
   */

  if (!moving && s->count > 0)
    {
      for (i = 0; i < 3; i++)
        {
          if (fabsf(acc[i] - s->acc_mean[i]) > s->accel_thresh)
            {
              moving = true;
            }
        }
    }

  if (moving)
    {
      /* Reset, do not merely pause: a window that spanned the movement would
       * average two different orientations into one meaningless vector.
       */

      s->count = 0;
      memset(s->acc_sum, 0, sizeof(s->acc_sum));
      memset(s->gyr_sum, 0, sizeof(s->gyr_sum));
      return false;
    }

  for (i = 0; i < 3; i++)
    {
      s->acc_sum[i] += acc[i];
      s->gyr_sum[i] += gyr[i];
    }

  s->count++;

  for (i = 0; i < 3; i++)
    {
      s->acc_mean[i] = s->acc_sum[i] / (float)s->count;
    }

  return s->count >= s->min_samples;
}

void cal_still_mean(FAR const struct cal_still_s *s, FAR float acc_out[3],
                    FAR float gyr_out[3])
{
  int i;

  if (s->count <= 0)
    {
      for (i = 0; i < 3; i++)
        {
          acc_out[i] = 0.0f;
          gyr_out[i] = 0.0f;
        }

      return;
    }

  for (i = 0; i < 3; i++)
    {
      acc_out[i] = s->acc_sum[i] / (float)s->count;
      gyr_out[i] = s->gyr_sum[i] / (float)s->count;
    }
}
