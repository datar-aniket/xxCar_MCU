/****************************************************************************
 * tests/imu_resampler_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "imu_resampler.h"

static void near_value(float actual, float expected)
{
  assert(fabsf(actual - expected) < 1.0e-6f);
}

static void test_bracket_interpolation(void)
{
  const float before[3] = {-1.0f, 2.0f, 9.0f};
  const float after[3] = {3.0f, -2.0f, 11.0f};
  float output[3];

  assert(imu_resample3(1000u, before, 1500u, after, 1250u, output));
  near_value(output[0], 1.0f);
  near_value(output[1], 0.0f);
  near_value(output[2], 10.0f);

  assert(imu_resample3(1000u, before, 1500u, after, 1000u, output));
  near_value(output[0], before[0]);
  near_value(output[1], before[1]);
  near_value(output[2], before[2]);

  assert(imu_resample3(1000u, before, 1500u, after, 1500u, output));
  near_value(output[0], after[0]);
  near_value(output[1], after[1]);
  near_value(output[2], after[2]);
}

static void test_invalid_brackets(void)
{
  const float good[3] = {0.0f, 0.0f, 9.8f};
  const float bad[3] = {NAN, 0.0f, 9.8f};
  float output[3];

  assert(!imu_resample3(1000u, good, 1000u, good, 1000u, output));
  assert(!imu_resample3(1500u, good, 1000u, good, 1250u, output));
  assert(!imu_resample3(1000u, good, 2500u, good, 1500u, output));
  assert(!imu_resample3(1000u, good, 1500u, good, 999u, output));
  assert(!imu_resample3(1000u, good, 1500u, good, 1501u, output));
  assert(!imu_resample3(1000u, bad, 1500u, good, 1250u, output));
}

int main(void)
{
  test_bracket_interpolation();
  test_invalid_brackets();
  puts("imu resampler tests: PASS");
  return 0;
}
