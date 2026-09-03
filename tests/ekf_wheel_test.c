/****************************************************************************
 * tests/ekf_wheel_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "ekf_wheel.h"

static void test_acceleration_is_delayed(void)
{
  struct ekf_wheel_accel_filter_s filter;
  float raw;

  ekf_wheel_accel_init(&filter);
  assert(!ekf_wheel_accel_update(&filter, 0.0f, 1000000, 0.2f, &raw));
  assert(ekf_wheel_accel_update(&filter, 1.0f, 1100000, 0.2f, &raw));
  assert(fabsf(raw - 10.0f) < 1.0e-5f);

  /* alpha = 0.1 / (0.2 + 0.1): a 10 m/s2 one-sample step is only 3.33. */

  assert(filter.accel_mps2 > 3.3f && filter.accel_mps2 < 3.4f);
}

static void test_gap_reseeds_derivative(void)
{
  struct ekf_wheel_accel_filter_s filter;
  float raw = 99.0f;

  ekf_wheel_accel_init(&filter);
  ekf_wheel_accel_update(&filter, 0.0f, 1000000, 0.2f, NULL);
  assert(!ekf_wheel_accel_update(
    &filter, 100.0f, 1000000 + EKF_WHEEL_ACCEL_MAX_GAP_US + 1,
    0.2f, &raw));
  assert(raw == 0.0f);
  assert(filter.accel_mps2 == 0.0f);
}

static void test_slip_gate_uses_filtered_magnitudes(void)
{
  assert(!ekf_wheel_slipping(2.0f, 1.2f, 1.0f));
  assert(ekf_wheel_slipping(2.3f, 1.2f, 1.0f));
  assert(ekf_wheel_slipping(-4.0f, -1.0f, 1.0f));
  assert(ekf_wheel_slipping(NAN, 0.0f, 1.0f));
}

int main(void)
{
  test_acceleration_is_delayed();
  test_gap_reseeds_derivative();
  test_slip_gate_uses_filtered_magnitudes();
  puts("ekf_wheel: acceleration delay and slip gate verified - OK");
  return 0;
}
