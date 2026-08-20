/****************************************************************************
 * tests/timing_stats_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "timing_stats.h"

static void assert_near(double actual, double expected, double tolerance)
{
  assert(fabs(actual - expected) <= tolerance);
}

static void test_ideal_stream(void)
{
  struct timing_stats_s stats;
  int i;

  timing_stats_init(&stats, 500);
  for (i = 0; i < 20; i++)
    {
      timing_stats_add(&stats, 1000000 + (uint64_t)i * 500,
                       1000100 + (uint64_t)i * 500);
    }

  assert(stats.samples == 20);
  assert(stats.intervals == 19);
  assert(stats.min_dt_us == 500);
  assert(stats.max_dt_us == 500);
  assert(stats.gaps == 0);
  assert(stats.duplicates == 0);
  assert(stats.backwards == 0);
  assert(stats.min_age_us == 100);
  assert(stats.max_age_us == 100);
  assert_near(stats.mean_dt_us, 500.0, 0.001);
  assert_near(timing_stats_stddev_us(&stats), 0.0, 0.001);
  assert_near(timing_stats_rate_hz(&stats), 2000.0, 0.001);
  assert_near(timing_stats_clock_drift_ppm(&stats), 0.0, 0.001);
}

static void test_fault_accounting(void)
{
  struct timing_stats_s stats;

  timing_stats_init(&stats, 500);
  timing_stats_add(&stats, 1000, 1100);
  timing_stats_add(&stats, 1490, 1600);
  timing_stats_add(&stats, 2000, 2120);
  timing_stats_add(&stats, 3500, 3630); /* two missing 500 us samples */
  timing_stats_add(&stats, 3500, 3640); /* duplicate */
  timing_stats_add(&stats, 3400, 3650); /* backwards */

  assert(stats.intervals == 3);
  assert(stats.min_dt_us == 490);
  assert(stats.max_dt_us == 1500);
  assert(stats.gaps == 2);
  assert(stats.duplicates == 1);
  assert(stats.backwards == 1);
  assert(stats.min_age_us == 100);
  assert(stats.max_age_us == 250);
}

static void test_clock_drift(void)
{
  struct timing_stats_s stats;
  int i;

  timing_stats_init(&stats, 5000);
  for (i = 0; i < 100; i++)
    {
      timing_stats_add(&stats, 1000000 + (uint64_t)i * 5000,
                       1000100 + (uint64_t)i * 5001);
    }

  assert_near(timing_stats_clock_drift_ppm(&stats), 200.0, 0.01);
}

int main(void)
{
  test_ideal_stream();
  test_fault_accounting();
  test_clock_drift();
  puts("timing_stats_test: PASS");
  return 0;
}
