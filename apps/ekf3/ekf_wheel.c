/****************************************************************************
 * apps/ekf3/ekf_wheel.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef EKF_CORE_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <math.h>
#include <string.h>

#include "ekf_wheel.h"

static float filter_alpha(float dt_s, float tau_s)
{
  if (!(dt_s > 0.0f) || !isfinite(dt_s))
    {
      return 0.0f;
    }

  if (!(tau_s > 0.0f) || !isfinite(tau_s))
    {
      return 1.0f;
    }

  return dt_s / (tau_s + dt_s);
}

void ekf_wheel_accel_init(FAR struct ekf_wheel_accel_filter_s *filter)
{
  if (filter != NULL)
    {
      memset(filter, 0, sizeof(*filter));
    }
}

bool ekf_wheel_accel_update(FAR struct ekf_wheel_accel_filter_s *filter,
                            float speed_mps, uint64_t timestamp_us,
                            float tau_s, FAR float *raw_accel_mps2)
{
  float dt_s;
  float raw;
  float alpha;

  if (filter == NULL || !isfinite(speed_mps) || timestamp_us == 0)
    {
      return false;
    }

  if (!filter->initialized || timestamp_us <= filter->last_timestamp_us ||
      timestamp_us - filter->last_timestamp_us >
        EKF_WHEEL_ACCEL_MAX_GAP_US)
    {
      filter->initialized = true;
      filter->last_timestamp_us = timestamp_us;
      filter->last_speed_mps = speed_mps;
      filter->accel_mps2 = 0.0f;

      if (raw_accel_mps2 != NULL)
        {
          *raw_accel_mps2 = 0.0f;
        }

      return false;
    }

  dt_s = (float)(timestamp_us - filter->last_timestamp_us) * 1.0e-6f;
  raw = (speed_mps - filter->last_speed_mps) / dt_s;
  filter->last_timestamp_us = timestamp_us;
  filter->last_speed_mps = speed_mps;

  if (!isfinite(raw))
    {
      filter->accel_mps2 = 0.0f;
      return false;
    }

  alpha = filter_alpha(dt_s, tau_s);
  filter->accel_mps2 += alpha * (raw - filter->accel_mps2);

  if (raw_accel_mps2 != NULL)
    {
      *raw_accel_mps2 = raw;
    }

  return true;
}

float ekf_wheel_lpf_update(FAR struct ekf_wheel_lpf_s *filter, float input,
                           float dt_s, float tau_s)
{
  float alpha;

  if (filter == NULL || !isfinite(input))
    {
      return 0.0f;
    }

  if (!filter->initialized)
    {
      filter->value = input;
      filter->initialized = true;
      return filter->value;
    }

  alpha = filter_alpha(dt_s, tau_s);
  filter->value += alpha * (input - filter->value);
  return filter->value;
}

bool ekf_wheel_slipping(float wheel_accel_mps2, float imu_accel_mps2,
                        float margin_mps2)
{
  if (!isfinite(wheel_accel_mps2) || !isfinite(imu_accel_mps2) ||
      !isfinite(margin_mps2) || margin_mps2 < 0.0f)
    {
      return true;
    }

  return fabsf(wheel_accel_mps2) >
         fabsf(imu_accel_mps2) + margin_mps2;
}
