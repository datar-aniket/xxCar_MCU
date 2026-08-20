/****************************************************************************
 * apps/sensors/dsp_filter.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef DSP_FILTER_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <math.h>
#include <string.h>

#include "dsp_filter.h"

#define DSP_PI_F       3.14159265358979323846f
#define DSP_SQRT2_F    1.41421356237309504880f

static bool dsp_filter_valid_frequency(float sample_rate_hz,
                                       float frequency_hz)
{
  return isfinite(sample_rate_hz) && isfinite(frequency_hz) &&
         sample_rate_hz >= 100.0f && frequency_hz > 0.0f &&
         frequency_hz < sample_rate_hz * 0.45f;
}

static void dsp_biquad3_disable(FAR struct dsp_biquad3_s *filter,
                                float sample_rate_hz)
{
  memset(filter, 0, sizeof(*filter));
  filter->sample_rate_hz = sample_rate_hz;
}

bool dsp_biquad3_lowpass(FAR struct dsp_biquad3_s *filter,
                         float sample_rate_hz, float cutoff_hz)
{
  float omega;
  float omega2;
  float scale;

  if (filter == NULL)
    {
      return false;
    }

  if (cutoff_hz == 0.0f)
    {
      dsp_biquad3_disable(filter, sample_rate_hz);
      return true;
    }

  if (!dsp_filter_valid_frequency(sample_rate_hz, cutoff_hz))
    {
      return false;
    }

  omega = tanf(DSP_PI_F * cutoff_hz / sample_rate_hz);
  omega2 = omega * omega;
  scale = 1.0f / (1.0f + DSP_SQRT2_F * omega + omega2);

  memset(filter, 0, sizeof(*filter));
  filter->b0 = omega2 * scale;
  filter->b1 = 2.0f * filter->b0;
  filter->b2 = filter->b0;
  filter->a1 = 2.0f * (omega2 - 1.0f) * scale;
  filter->a2 = (1.0f - DSP_SQRT2_F * omega + omega2) * scale;
  filter->sample_rate_hz = sample_rate_hz;
  filter->frequency_hz = cutoff_hz;
  filter->enabled = true;
  return true;
}

bool dsp_biquad3_notch(FAR struct dsp_biquad3_s *filter,
                       float sample_rate_hz, float centre_hz,
                       float bandwidth_hz)
{
  float alpha;
  float beta;
  float scale;

  if (filter == NULL)
    {
      return false;
    }

  if (centre_hz == 0.0f)
    {
      dsp_biquad3_disable(filter, sample_rate_hz);
      return true;
    }

  if (!dsp_filter_valid_frequency(sample_rate_hz, centre_hz) ||
      !isfinite(bandwidth_hz) || bandwidth_hz <= 0.0f ||
      bandwidth_hz >= centre_hz ||
      bandwidth_hz >= sample_rate_hz * 0.45f)
    {
      return false;
    }

  alpha = tanf(DSP_PI_F * bandwidth_hz / sample_rate_hz);
  beta = -cosf(2.0f * DSP_PI_F * centre_hz / sample_rate_hz);
  scale = 1.0f / (1.0f + alpha);

  memset(filter, 0, sizeof(*filter));
  filter->b0 = scale;
  filter->b1 = 2.0f * beta * scale;
  filter->b2 = scale;
  filter->a1 = filter->b1;
  filter->a2 = (1.0f - alpha) * scale;
  filter->sample_rate_hz = sample_rate_hz;
  filter->frequency_hz = centre_hz;
  filter->bandwidth_hz = bandwidth_hz;
  filter->enabled = true;
  return true;
}

void dsp_biquad3_reset(FAR struct dsp_biquad3_s *filter,
                       FAR const float value[3])
{
  int axis;

  if (filter == NULL || value == NULL)
    {
      return;
    }

  for (axis = 0; axis < 3; axis++)
    {
      /* Both filters have unity DC gain.  Set the two delay elements to the
       * corresponding steady state rather than injecting a startup step.
       */

      filter->state[axis][0] = (1.0f - filter->b0) * value[axis];
      filter->state[axis][1] = (filter->b2 - filter->a2) * value[axis];
    }
}
