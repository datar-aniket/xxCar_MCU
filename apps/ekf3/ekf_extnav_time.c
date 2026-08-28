/****************************************************************************
 * apps/ekf3/ekf_extnav_time.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>

#include "ekf_extnav_time.h"

int ekf_extnav_time_prepare(uint64_t source_time_us,
                            uint64_t receive_time_us,
                            uint64_t filter_time_us,
                            int32_t delay_us,
                            uint32_t jitter_us,
                            FAR uint64_t *corrected_time_us,
                            FAR int64_t *signed_age_us)
{
  int64_t stamp;
  int64_t age;
  uint64_t tolerance_us = (uint64_t)jitter_us * 3ull;
  int result = 0;

  if (source_time_us == 0 || receive_time_us == 0 ||
      corrected_time_us == NULL || source_time_us > INT64_MAX ||
      receive_time_us > INT64_MAX || filter_time_us > receive_time_us)
    {
      return -1;
    }

  stamp = (int64_t)source_time_us - (int64_t)delay_us;

  if (stamp <= 0)
    {
      return -1;
    }

  age = (int64_t)receive_time_us - stamp;

  if (signed_age_us != NULL)
    {
      *signed_age_us = age;
    }

  if (stamp > (int64_t)receive_time_us)
    {
      uint64_t future_us = (uint64_t)(stamp - (int64_t)receive_time_us);

      if (future_us > tolerance_us)
        {
          return -1;
        }

      stamp = (int64_t)receive_time_us;
      result = 1;
    }

  if (filter_time_us != 0 && stamp < (int64_t)filter_time_us)
    {
      uint64_t late_us = filter_time_us - (uint64_t)stamp;

      if (late_us > tolerance_us)
        {
          return -1;
        }

      stamp = (int64_t)filter_time_us;
      result = 1;
    }

  *corrected_time_us = (uint64_t)stamp;
  return result;
}
