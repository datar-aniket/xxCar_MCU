/****************************************************************************
 * apps/sensor_status/timing_stats.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_SENSOR_STATUS_TIMING_STATS_H
#define __APPS_SENSOR_STATUS_TIMING_STATS_H

#include <stdint.h>

struct timing_stats_s
{
  uint64_t samples;
  uint64_t intervals;
  uint64_t first_sample_us;
  uint64_t last_sample_us;
  uint64_t min_dt_us;
  uint64_t max_dt_us;
  uint64_t duplicates;
  uint64_t backwards;
  uint64_t gaps;
  uint32_t expected_period_us;

  double   mean_dt_us;
  double   m2_dt_us;

  uint64_t age_samples;
  int64_t  min_age_us;
  int64_t  max_age_us;
  double   mean_age_us;

  uint64_t clock_samples;
  uint64_t clock_sample_origin_us;
  uint64_t clock_arrival_origin_us;
  double   clock_mean_x;
  double   clock_mean_y;
  double   clock_sxx;
  double   clock_sxy;
};

void timing_stats_init(struct timing_stats_s *stats,
                       uint32_t expected_period_us);
void timing_stats_add(struct timing_stats_s *stats, uint64_t sample_us,
                      uint64_t arrival_us);
double timing_stats_rate_hz(const struct timing_stats_s *stats);
double timing_stats_stddev_us(const struct timing_stats_s *stats);
double timing_stats_clock_drift_ppm(const struct timing_stats_s *stats);

#endif /* __APPS_SENSOR_STATUS_TIMING_STATS_H */
