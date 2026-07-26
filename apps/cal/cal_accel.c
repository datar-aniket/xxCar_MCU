/****************************************************************************
 * apps/cal/cal_accel.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See cal_accel.h.
 ****************************************************************************/

#ifndef CAL_ACCEL_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <string.h>
#include <math.h>
#include <errno.h>

#include "cal_accel.h"

/* How far from axis-aligned a reading may be and still count as a position.
 *
 * 0.94 of the vector on one axis is about 20 degrees, which a hand-placed
 * board on a flat surface clears easily while a board resting on a corner or
 * an edge does not. The magnitude window catches a board that was moving, or a
 * sensor whose scale is so far out that the whole procedure is pointless.
 */

#define CAL_AXIS_FRAC   0.94f
#define CAL_G_LO        (0.85f * CAL_G)
#define CAL_G_HI        (1.15f * CAL_G)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void cal_accel_reset(FAR struct cal_accel_s *s)
{
  memset(s, 0, sizeof(*s));
}

int cal_accel_classify(FAR const float a[3])
{
  float norm;
  float best = 0.0f;
  int axis = -1;
  int i;

  norm = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);

  if (norm < CAL_G_LO || norm > CAL_G_HI)
    {
      return -1;
    }

  for (i = 0; i < 3; i++)
    {
      float m = fabsf(a[i]);

      if (m > best)
        {
          best = m;
          axis = i;
        }
    }

  if (axis < 0 || best < CAL_AXIS_FRAC * norm)
    {
      return -1;
    }

  return axis * 2 + (a[axis] > 0.0f ? 0 : 1);
}

int cal_accel_add(FAR struct cal_accel_s *s, FAR const float a[3])
{
  int pos = cal_accel_classify(a);

  if (pos < 0)
    {
      return -1;
    }

  s->mean[pos][0] = a[0];
  s->mean[pos][1] = a[1];
  s->mean[pos][2] = a[2];
  s->have[pos]    = true;

  return pos;
}

int cal_accel_count(FAR const struct cal_accel_s *s)
{
  int n = 0;
  int i;

  for (i = 0; i < CAL_NPOS; i++)
    {
      if (s->have[i])
        {
          n++;
        }
    }

  return n;
}

bool cal_accel_complete(FAR const struct cal_accel_s *s)
{
  return cal_accel_count(s) == CAL_NPOS;
}

int cal_accel_solve(FAR const struct cal_accel_s *s, FAR float off[3],
                    FAR float scl[3], FAR float *residual)
{
  double err = 0.0;
  int i;
  int p;

  if (!cal_accel_complete(s))
    {
      return -EAGAIN;
    }

  for (i = 0; i < 3; i++)
    {
      /* The axis-up reading and the axis-down reading of the SAME axis. */

      float plus  = s->mean[i * 2][i];
      float minus = s->mean[i * 2 + 1][i];
      float span  = plus - minus;

      /* Up minus down should be two gravities. Anything near zero means the
       * two positions were not opposites, and dividing by it would produce a
       * huge scale that still looks like a number.
       */

      if (span < CAL_G || span > 4.0f * CAL_G)
        {
          return -EINVAL;
        }

      off[i] = 0.5f * (plus + minus);
      scl[i] = 2.0f * CAL_G / span;
    }

  /* Residual: apply the result to all six readings and see how far the
   * corrected magnitude sits from gravity. This is the number that says
   * whether the calibration is trustworthy - offsets alone never can.
   */

  for (p = 0; p < CAL_NPOS; p++)
    {
      double n2 = 0.0;

      for (i = 0; i < 3; i++)
        {
          double c = ((double)s->mean[p][i] - off[i]) * scl[i];

          n2 += c * c;
        }

      {
        double d = sqrt(n2) - (double)CAL_G;

        err += d * d;
      }
    }

  if (residual != NULL)
    {
      *residual = (float)sqrt(err / CAL_NPOS);
    }

  return 0;
}

void cal_stats(FAR const float *samples, int n, int nv, FAR float *mean,
               FAR float *sd)
{
  int i;
  int k;

  if (n <= 0)
    {
      for (i = 0; i < nv; i++)
        {
          mean[i] = 0.0f;
          sd[i]   = 0.0f;
        }

      return;
    }

  for (i = 0; i < nv; i++)
    {
      double sum = 0.0;
      double sq  = 0.0;
      double m;
      double var;

      for (k = 0; k < n; k++)
        {
          double v = samples[(size_t)k * nv + i];

          sum += v;
          sq  += v * v;
        }

      m   = sum / n;
      var = sq / n - m * m;

      mean[i] = (float)m;
      sd[i]   = (float)(var > 0.0 ? sqrt(var) : 0.0);
    }
}
