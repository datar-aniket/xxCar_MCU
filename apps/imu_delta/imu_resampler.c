/****************************************************************************
 * apps/imu_delta/imu_resampler.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <math.h>
#include <stddef.h>

#include "imu_resampler.h"

bool imu_resample3(uint64_t before_timestamp,
                   FAR const float before[3],
                   uint64_t after_timestamp,
                   FAR const float after[3],
                   uint64_t target_timestamp,
                   FAR float output[3])
{
  float fraction;
  int axis;

  if (before == NULL || after == NULL || output == NULL ||
      after_timestamp <= before_timestamp ||
      after_timestamp - before_timestamp > IMU_RESAMPLE_MAX_BRACKET_US ||
      target_timestamp < before_timestamp ||
      target_timestamp > after_timestamp)
    {
      return false;
    }

  for (axis = 0; axis < 3; axis++)
    {
      if (!isfinite(before[axis]) || !isfinite(after[axis]))
        {
          return false;
        }
    }

  fraction = (float)(target_timestamp - before_timestamp) /
             (float)(after_timestamp - before_timestamp);

  for (axis = 0; axis < 3; axis++)
    {
      output[axis] = before[axis] + fraction * (after[axis] - before[axis]);
    }

  return isfinite(output[0]) && isfinite(output[1]) && isfinite(output[2]);
}
