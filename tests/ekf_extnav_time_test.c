/****************************************************************************
 * tests/ekf_extnav_time_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "ekf_extnav_time.h"

static void test_delay_is_signed(void)
{
  uint64_t corrected;
  int64_t age;

  assert(ekf_extnav_time_prepare(1000000, 1020000, 900000,
                                 1600, 2000, &corrected, &age) == 0);
  assert(corrected == 998400);
  assert(age == 21600);

  assert(ekf_extnav_time_prepare(1000000, 1020000, 900000,
                                 -1600, 2000, &corrected, &age) == 0);
  assert(corrected == 1001600);
  assert(age == 18400);
}

static void test_small_future_jitter_clamps(void)
{
  uint64_t corrected;
  int64_t age;

  assert(ekf_extnav_time_prepare(1002000, 1000000, 900000,
                                 0, 2000, &corrected, &age) == 1);
  assert(corrected == 1000000);
  assert(age == -2000);
}

static void test_large_future_error_rejects(void)
{
  uint64_t corrected = 123;

  assert(ekf_extnav_time_prepare(1007000, 1000000, 900000,
                                 0, 2000, &corrected, NULL) == -1);
  assert(corrected == 123);
}

static void test_small_horizon_overrun_clamps(void)
{
  uint64_t corrected;

  assert(ekf_extnav_time_prepare(899000, 1000000, 900000,
                                 0, 2000, &corrected, NULL) == 1);
  assert(corrected == 900000);
}

static void test_large_horizon_overrun_rejects(void)
{
  uint64_t corrected = 123;

  assert(ekf_extnav_time_prepare(893000, 1000000, 900000,
                                 0, 2000, &corrected, NULL) == -1);
  assert(corrected == 123);
}

int main(void)
{
  test_delay_is_signed();
  test_small_future_jitter_clamps();
  test_large_future_error_rejects();
  test_small_horizon_overrun_clamps();
  test_large_horizon_overrun_rejects();
  puts("ekf_extnav_time: delay, jitter clamps and rejection verified - OK");
  return 0;
}
