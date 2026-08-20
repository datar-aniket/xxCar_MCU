/****************************************************************************
 * apps/sensor_status/timing_stats.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "timing_stats.h"

void timing_stats_init(struct timing_stats_s *stats,
                       uint32_t expected_period_us)
{
  memset(stats, 0, sizeof(*stats));
  stats->expected_period_us = expected_period_us;
  stats->min_dt_us = UINT64_MAX;
  stats->min_age_us = INT64_MAX;
  stats->max_age_us = INT64_MIN;
}

static void timing_stats_add_age(struct timing_stats_s *stats,
                                 int64_t age_us)
{
  double delta;

  stats->age_samples++;
  delta = (double)age_us - stats->mean_age_us;
  stats->mean_age_us += delta / (double)stats->age_samples;

  if (age_us < stats->min_age_us)
    {
      stats->min_age_us = age_us;
    }

  if (age_us > stats->max_age_us)
    {
      stats->max_age_us = age_us;
    }
}

static void timing_stats_add_clock(struct timing_stats_s *stats,
                                   uint64_t sample_us, uint64_t arrival_us)
{
  double x;
  double y;
  double dx;
  double dy;

  if (stats->clock_samples == 0)
    {
      stats->clock_sample_origin_us = sample_us;
      stats->clock_arrival_origin_us = arrival_us;
    }

  x = (double)(sample_us - stats->clock_sample_origin_us);
  y = (double)(arrival_us - stats->clock_arrival_origin_us);

  stats->clock_samples++;
  dx = x - stats->clock_mean_x;
  stats->clock_mean_x += dx / (double)stats->clock_samples;
  dy = y - stats->clock_mean_y;
  stats->clock_mean_y += dy / (double)stats->clock_samples;
  stats->clock_sxx += dx * (x - stats->clock_mean_x);
  stats->clock_sxy += dx * (y - stats->clock_mean_y);
}

void timing_stats_add(struct timing_stats_s *stats, uint64_t sample_us,
                      uint64_t arrival_us)
{
  bool monotonic = false;
  int64_t age_us;

  stats->samples++;
  age_us = arrival_us >= sample_us ?
           (int64_t)(arrival_us - sample_us) :
           -(int64_t)(sample_us - arrival_us);
  timing_stats_add_age(stats, age_us);

  if (stats->samples == 1)
    {
      stats->first_sample_us = sample_us;
      stats->last_sample_us = sample_us;
      monotonic = true;
    }
  else if (sample_us == stats->last_sample_us)
    {
      stats->duplicates++;
    }
  else if (sample_us < stats->last_sample_us)
    {
      stats->backwards++;
    }
  else
    {
      uint64_t dt_us = sample_us - stats->last_sample_us;
      double delta;

      stats->intervals++;
      delta = (double)dt_us - stats->mean_dt_us;
      stats->mean_dt_us += delta / (double)stats->intervals;
      stats->m2_dt_us += delta * ((double)dt_us - stats->mean_dt_us);

      if (dt_us < stats->min_dt_us)
        {
          stats->min_dt_us = dt_us;
        }

      if (dt_us > stats->max_dt_us)
        {
          stats->max_dt_us = dt_us;
        }

      if (stats->expected_period_us > 0 &&
          dt_us > (uint64_t)stats->expected_period_us * 3ull / 2ull)
        {
          uint64_t periods =
            (dt_us + stats->expected_period_us / 2u) /
            stats->expected_period_us;

          if (periods > 1)
            {
              stats->gaps += periods - 1;
            }
        }

      stats->last_sample_us = sample_us;
      monotonic = true;
    }

  if (monotonic)
    {
      timing_stats_add_clock(stats, sample_us, arrival_us);
    }
}

double timing_stats_rate_hz(const struct timing_stats_s *stats)
{
  uint64_t span_us;

  if (stats->intervals == 0)
    {
      return 0.0;
    }

  span_us = stats->last_sample_us - stats->first_sample_us;
  return span_us > 0 ? (double)stats->intervals * 1000000.0 /
                       (double)span_us : 0.0;
}

double timing_stats_stddev_us(const struct timing_stats_s *stats)
{
  return stats->intervals > 0 ?
         sqrt(stats->m2_dt_us / (double)stats->intervals) : 0.0;
}

double timing_stats_clock_drift_ppm(const struct timing_stats_s *stats)
{
  if (stats->clock_samples < 2 || stats->clock_sxx <= 0.0)
    {
      return 0.0;
    }

  return (stats->clock_sxy / stats->clock_sxx - 1.0) * 1000000.0;
}
