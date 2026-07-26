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

  if (moving)
    {
      /* Reset, do not merely pause: a window that spanned the movement would
       * average two different orientations into one meaningless vector.
       */

      s->gyro_resets++;
      s->count = 0;
      memset(s->acc_sum, 0, sizeof(s->acc_sum));
      memset(s->acc_sq,  0, sizeof(s->acc_sq));
      memset(s->gyr_sum, 0, sizeof(s->gyr_sum));
      return false;
    }

  for (i = 0; i < 3; i++)
    {
      s->acc_sum[i] += acc[i];
      s->gyr_sum[i] += gyr[i];
    }

  for (i = 0; i < 3; i++)
    {
      s->acc_sq[i] += acc[i] * acc[i];
    }

  s->count++;

  if (s->count > s->best_count)
    {
      s->best_count = s->count;
    }

  if (s->count < s->min_samples)
    {
      return false;
    }

  /* The window is full. Judge translation by the DISPERSION of the whole
   * window, not by whether any single sample strayed from the running mean.
   *
   * The instantaneous test this replaces could not work: at +/-16 g the
   * ICM-42688's own noise is about 0.02 m/s^2 RMS, so a 0.05 m/s^2 band sits at
   * roughly 2.5 sigma and about 1.2% of samples per axis fall outside it by
   * chance. Requiring 500 consecutive samples inside the band therefore
   * succeeded with probability ~1e-8 - the board could be sitting on granite
   * and never be declared still.
   *
   * Dispersion is the right measure anyway: a single noisy sample is not
   * motion, and motion is what we are trying to exclude. A slide, a tap or a
   * hand tremor all raise the standard deviation over the window, while white
   * noise stays at the sensor's floor.
   */

  {
    bool spread = false;

    for (i = 0; i < 3; i++)
      {
        float mean = s->acc_sum[i] / (float)s->count;
        float var  = s->acc_sq[i] / (float)s->count - mean * mean;

        /* Cancellation can drive a genuinely zero variance slightly negative. */

        s->last_sd[i] = var > 0.0f ? sqrtf(var) : 0.0f;

        if (s->last_sd[i] > s->accel_thresh)
          {
            spread = true;
          }
      }

    if (spread)
      {
        s->accel_resets++;
        s->count = 0;
        memset(s->acc_sum, 0, sizeof(s->acc_sum));
        memset(s->acc_sq,  0, sizeof(s->acc_sq));
        memset(s->gyr_sum, 0, sizeof(s->gyr_sum));
        return false;
      }
  }

  return true;
}

void cal_still_report(FAR const struct cal_still_s *s, FAR int *best_count,
                      FAR float sd_out[3], FAR int *gyro_resets,
                      FAR int *accel_resets)
{
  int i;

  *best_count   = s->best_count;
  *gyro_resets  = s->gyro_resets;
  *accel_resets = s->accel_resets;

  for (i = 0; i < 3; i++)
    {
      sd_out[i] = s->last_sd[i];
    }
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
