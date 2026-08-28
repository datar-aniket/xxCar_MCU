/****************************************************************************
 * apps/imu_delta/imu_lpf.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "imu_lpf.h"

#define IMU_LPF_PI_F       3.14159265358979323846f
#define IMU_LPF_SQRT2_F    1.41421356237309504880f

bool imu_lpf3_configure(FAR struct imu_lpf3_s *filter,
                        float sample_rate_hz, float cutoff_hz)
{
  float omega;
  float omega2;
  float scale;
  float delay_samples;

  if (filter == NULL || !isfinite(sample_rate_hz) ||
      !isfinite(cutoff_hz) || sample_rate_hz < 100.0f || cutoff_hz < 0.0f ||
      cutoff_hz >= sample_rate_hz * 0.45f)
    {
      return false;
    }

  memset(filter, 0, sizeof(*filter));
  filter->sample_rate_hz = sample_rate_hz;
  filter->cutoff_hz = cutoff_hz;

  if (cutoff_hz == 0.0f)
    {
      return true;
    }

  /* Bilinear-transform, pre-warped two-pole Butterworth. */

  omega = tanf(IMU_LPF_PI_F * cutoff_hz / sample_rate_hz);
  omega2 = omega * omega;
  scale = 1.0f / (1.0f + IMU_LPF_SQRT2_F * omega + omega2);

  filter->b0 = omega2 * scale;
  filter->b1 = 2.0f * filter->b0;
  filter->b2 = filter->b0;
  filter->a1 = 2.0f * (omega2 - 1.0f) * scale;
  filter->a2 = (1.0f - IMU_LPF_SQRT2_F * omega + omega2) * scale;

  /* Exact DC group delay of this digital biquad. It is 4.464 samples,
   * 2232 us, at 2 kHz/100 Hz. A causal IIR has frequency-dependent delay;
   * low-frequency delay is the right timestamp correction for vehicle
   * dynamics, while accel and gyro retain identical residual phase.
   */

  delay_samples = IMU_LPF_SQRT2_F / (2.0f * omega);
  filter->group_delay_us =
    (uint32_t)(delay_samples * 1000000.0f / sample_rate_hz + 0.5f);
  filter->enabled = true;
  return true;
}

void imu_lpf3_reset(FAR struct imu_lpf3_s *filter,
                    FAR const float value[3])
{
  int axis;

  if (filter == NULL || value == NULL)
    {
      return;
    }

  memset(filter->state, 0, sizeof(filter->state));

  if (!filter->enabled)
    {
      return;
    }

  /* Seed the delay elements to the constant-input steady state. */

  for (axis = 0; axis < 3; axis++)
    {
      filter->state[axis][0] = value[axis] * (1.0f - filter->b0);
      filter->state[axis][1] = value[axis] * (filter->b2 - filter->a2);
    }
}

bool imu_lpf3_apply(FAR struct imu_lpf3_s *filter, FAR float value[3])
{
  int axis;

  if (filter == NULL || value == NULL)
    {
      return false;
    }

  if (!isfinite(value[0]) || !isfinite(value[1]) || !isfinite(value[2]))
    {
      return false;
    }

  if (!filter->enabled)
    {
      return true;
    }

  for (axis = 0; axis < 3; axis++)
    {
      float input = value[axis];
      float output = filter->b0 * input + filter->state[axis][0];

      filter->state[axis][0] = filter->b1 * input -
                               filter->a1 * output +
                               filter->state[axis][1];
      filter->state[axis][1] = filter->b2 * input -
                               filter->a2 * output;
      value[axis] = output;
    }

  return isfinite(value[0]) && isfinite(value[1]) && isfinite(value[2]);
}

uint64_t imu_lpf_compensate_timestamp(uint64_t timestamp_us,
                                      uint32_t group_delay_us)
{
  if (timestamp_us <= (uint64_t)group_delay_us)
    {
      return 1;
    }

  return timestamp_us - (uint64_t)group_delay_us;
}
