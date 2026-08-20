/****************************************************************************
 * apps/sensors/dsp_filter.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Small native-rate IMU filters for the Cortex-M7.  The state is arranged as
 * three independent direct-form-II-transposed biquads.  Processing the XYZ
 * vector together avoids de-interleaving samples for a block-DSP API and lets
 * GCC keep the two states and five coefficients in FP registers.
 ****************************************************************************/

#ifndef __APPS_SENSORS_DSP_FILTER_H
#define __APPS_SENSORS_DSP_FILTER_H

#include <stdbool.h>

#ifndef FAR
#  define FAR
#endif

struct dsp_biquad3_s
{
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
  float state[3][2];
  float sample_rate_hz;
  float frequency_hz;
  float bandwidth_hz;
  bool  enabled;
} __attribute__((aligned(32)));

/* Configure a second-order Butterworth low-pass.  A zero cutoff disables the
 * filter.  Invalid or numerically unsafe settings are rejected.
 */

bool dsp_biquad3_lowpass(FAR struct dsp_biquad3_s *filter,
                         float sample_rate_hz, float cutoff_hz);

/* Configure a second-order notch.  A zero centre frequency disables it. */

bool dsp_biquad3_notch(FAR struct dsp_biquad3_s *filter,
                       float sample_rate_hz, float centre_hz,
                       float bandwidth_hz);

/* Put the filter into the steady state corresponding to value.  This avoids
 * the false step transient produced by clearing a running low-pass to zero.
 */

void dsp_biquad3_reset(FAR struct dsp_biquad3_s *filter,
                       FAR const float value[3]);

/* Hot path.  Coefficients are immutable between parameter/rate updates and
 * all validation is done there, not for every multiply.
 */

static inline void dsp_biquad3_apply(FAR struct dsp_biquad3_s *filter,
                                     FAR float value[3])
{
  int axis;

  if (!filter->enabled)
    {
      return;
    }

  for (axis = 0; axis < 3; axis++)
    {
      const float input = value[axis];
      const float output = filter->b0 * input + filter->state[axis][0];

      filter->state[axis][0] = filter->b1 * input -
                               filter->a1 * output +
                               filter->state[axis][1];
      filter->state[axis][1] = filter->b2 * input -
                               filter->a2 * output;
      value[axis] = output;
    }
}

#endif /* __APPS_SENSORS_DSP_FILTER_H */
