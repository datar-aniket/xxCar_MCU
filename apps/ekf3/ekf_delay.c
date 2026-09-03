/****************************************************************************
 * apps/ekf3/ekf_delay.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef EKF_CORE_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <string.h>

#include "ekf_delay.h"

/* head is the next write slot, so the oldest valid entry sits count places
 * behind it.
 */

static uint16_t ring_oldest(uint16_t head, uint16_t count, uint16_t size)
{
  return (uint16_t)((head + size - count) % size);
}

void ekf_delay_init(FAR struct ekf_delay_s *d, uint32_t horizon_ms)
{
  if (d == NULL)
    {
      return;
    }

  memset(d, 0, sizeof(*d));
  ekf_delay_set_horizon(d, horizon_ms);
}

void ekf_delay_set_horizon(FAR struct ekf_delay_s *d, uint32_t horizon_ms)
{
  if (d == NULL)
    {
      return;
    }

  if (horizon_ms > (uint32_t)EKF_DELAY_MAX_MS)
    {
      horizon_ms = (uint32_t)EKF_DELAY_MAX_MS;
    }

  d->horizon_us = horizon_ms * 1000u;
}

bool ekf_delay_push_imu(FAR struct ekf_delay_s *d,
                        FAR const struct ekf_imu_sample_s *sample)
{
  bool lost = false;

  if (d == NULL || sample == NULL)
    {
      return false;
    }

  /* The new sample always goes in. When that costs an entry the filter has
   * not consumed, say so and count it - but do not drop the newest data to
   * keep the oldest unprocessed data, which is the wrong trade for an
   * estimator. A full ring whose front entries are already consumed loses
   * nothing: those slots are free.
   */

  if (d->imu_count == EKF_IMU_RING_SIZE && d->imu_consumed == 0)
    {
      d->imu_overflow_count++;
      lost = true;
    }

  d->imu[d->imu_head] = *sample;
  d->imu_head = (uint16_t)((d->imu_head + 1) % EKF_IMU_RING_SIZE);

  if (d->imu_count < EKF_IMU_RING_SIZE)
    {
      d->imu_count++;
    }
  else if (d->imu_consumed > 0)
    {
      d->imu_consumed--;
    }

  return !lost;
}

bool ekf_delay_push_baro(FAR struct ekf_delay_s *d,
                         FAR const struct ekf_baro_sample_s *sample)
{
  bool lost = false;

  if (d == NULL || sample == NULL)
    {
      return false;
    }

  if (d->baro_count == EKF_BARO_QUEUE_SIZE)
    {
      d->baro_overflow_count++;
      d->baro_count--;
      lost = true;
    }

  d->baro[d->baro_head] = *sample;
  d->baro_head = (uint16_t)((d->baro_head + 1) % EKF_BARO_QUEUE_SIZE);
  d->baro_count++;
  return !lost;
}

bool ekf_delay_push_mag(FAR struct ekf_delay_s *d,
                        FAR const struct ekf_mag_sample_s *sample)
{
  bool lost = false;

  if (d == NULL || sample == NULL)
    {
      return false;
    }

  if (d->mag_count == EKF_MAG_QUEUE_SIZE)
    {
      d->mag_overflow_count++;
      d->mag_count--;
      lost = true;
    }

  d->mag[d->mag_head] = *sample;
  d->mag_head = (uint16_t)((d->mag_head + 1) % EKF_MAG_QUEUE_SIZE);
  d->mag_count++;
  return !lost;
}

bool ekf_delay_push_wheel(FAR struct ekf_delay_s *d,
                          FAR const struct ekf_wheel_sample_s *sample)
{
  bool lost = false;

  if (d == NULL || sample == NULL)
    {
      return false;
    }

  if (d->wheel_count == EKF_WHEEL_QUEUE_SIZE)
    {
      d->wheel_overflow_count++;
      d->wheel_count--;
      lost = true;
    }

  d->wheel[d->wheel_head] = *sample;
  d->wheel_head = (uint16_t)((d->wheel_head + 1) % EKF_WHEEL_QUEUE_SIZE);
  d->wheel_count++;
  return !lost;
}

uint64_t ekf_delay_horizon_time(FAR const struct ekf_delay_s *d,
                                uint64_t newest_imu_time_us)
{
  if (d == NULL || newest_imu_time_us <= (uint64_t)d->horizon_us)
    {
      return 0;
    }

  return newest_imu_time_us - (uint64_t)d->horizon_us;
}

bool ekf_delay_next_imu(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                        FAR struct ekf_imu_sample_s *out)
{
  uint16_t index;

  if (d == NULL || out == NULL || d->imu_consumed >= d->imu_count)
    {
      return false;
    }

  index = (uint16_t)((ring_oldest(d->imu_head, d->imu_count,
                                  EKF_IMU_RING_SIZE) + d->imu_consumed) %
                     EKF_IMU_RING_SIZE);

  if (d->imu[index].timestamp_sample > horizon_time)
    {
      return false;
    }

  *out = d->imu[index];
  d->imu_consumed++;
  return true;
}

bool ekf_delay_next_baro(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                         uint64_t max_age_us,
                         FAR struct ekf_baro_sample_s *out)
{
  if (d == NULL || out == NULL)
    {
      return false;
    }

  while (d->baro_count > 0)
    {
      uint16_t index = ring_oldest(d->baro_head, d->baro_count,
                                   EKF_BARO_QUEUE_SIZE);
      uint64_t stamp = d->baro[index].timestamp_sample;

      if (stamp > horizon_time)
        {
          return false;
        }

      d->baro_count--;

      if (horizon_time - stamp <= max_age_us)
        {
          *out = d->baro[index];
          return true;
        }

      /* Too old to fuse where the filter now is. Drop it and look at the
       * next one rather than returning a correction for a point on the
       * trajectory the filter has already left.
       */
    }

  return false;
}

bool ekf_delay_next_mag(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                        uint64_t max_age_us,
                        FAR struct ekf_mag_sample_s *out)
{
  if (d == NULL || out == NULL)
    {
      return false;
    }

  while (d->mag_count > 0)
    {
      uint16_t index = ring_oldest(d->mag_head, d->mag_count,
                                   EKF_MAG_QUEUE_SIZE);
      uint64_t stamp = d->mag[index].timestamp_sample;

      if (stamp > horizon_time)
        {
          return false;
        }

      d->mag_count--;

      if (horizon_time - stamp <= max_age_us)
        {
          *out = d->mag[index];
          return true;
        }
    }

  return false;
}

bool ekf_delay_next_wheel(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                          uint64_t max_age_us,
                          FAR struct ekf_wheel_sample_s *out)
{
  if (d == NULL || out == NULL)
    {
      return false;
    }

  while (d->wheel_count > 0)
    {
      uint16_t index = ring_oldest(d->wheel_head, d->wheel_count,
                                   EKF_WHEEL_QUEUE_SIZE);
      uint64_t stamp = d->wheel[index].timestamp_sample;

      if (stamp > horizon_time)
        {
          return false;
        }

      d->wheel_count--;

      if (horizon_time - stamp <= max_age_us)
        {
          *out = d->wheel[index];
          return true;
        }
    }

  return false;
}

uint16_t ekf_delay_output_count(FAR const struct ekf_delay_s *d)
{
  if (d == NULL || d->imu_consumed >= d->imu_count)
    {
      return 0;
    }

  return (uint16_t)(d->imu_count - d->imu_consumed);
}

FAR const struct ekf_imu_sample_s *
  ekf_delay_output_at(FAR const struct ekf_delay_s *d, uint16_t index)
{
  uint16_t slot;

  if (d == NULL || index >= ekf_delay_output_count(d))
    {
      return NULL;
    }

  slot = (uint16_t)((ring_oldest(d->imu_head, d->imu_count,
                                 EKF_IMU_RING_SIZE) + d->imu_consumed +
                     index) % EKF_IMU_RING_SIZE);
  return &d->imu[slot];
}

bool ekf_delay_push_extnav(FAR struct ekf_delay_s *d,
                           FAR const struct ekf_extnav_sample_s *sample)
{
  bool lost = false;

  if (d == NULL || sample == NULL)
    {
      return false;
    }

  /* The queue is short and the OLDEST goes: a stale absolute fix is worth
   * less than a fresh one.
   */

  if (d->extnav_count == EKF_EXTNAV_QUEUE_SIZE)
    {
      d->extnav_overflow_count++;
      d->extnav_count--;
      lost = true;
    }

  d->extnav[d->extnav_head] = *sample;
  d->extnav_head = (uint16_t)((d->extnav_head + 1) %
                              EKF_EXTNAV_QUEUE_SIZE);
  d->extnav_count++;
  return !lost;
}

bool ekf_delay_next_extnav(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                           uint64_t max_age_us,
                           FAR struct ekf_extnav_sample_s *out)
{
  if (d == NULL || out == NULL)
    {
      return false;
    }

  while (d->extnav_count > 0)
    {
      uint16_t index = ring_oldest(d->extnav_head, d->extnav_count,
                                   EKF_EXTNAV_QUEUE_SIZE);
      uint64_t stamp = d->extnav[index].timestamp_sample;

      if (stamp > horizon_time)
        {
          return false;
        }

      d->extnav_count--;

      if (horizon_time - stamp <= max_age_us)
        {
          *out = d->extnav[index];
          return true;
        }

      /* Too old to fuse where the filter now is. Drop and look at the next. */
    }

  return false;
}
