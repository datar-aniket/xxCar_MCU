/****************************************************************************
 * tests/ekf_baro_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Barometric height fusion: the pressure-to-height conversion, the reference
 * captured at alignment, the NED sign convention, and the gating.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ekf_core.h"

#define CLOSE(a, b, eps) (fabsf((a) - (b)) < (eps))
#define TEST_DT     0.0025f
#define TEST_DT_US  2500ull
#define TEST_G      9.80665f

static struct ekf_core_s g_core;

static void align_core(void)
{
  struct ekf_imu_sample_s s;
  uint64_t t = 0;
  int i;

  ekf_core_init(&g_core);

  for (i = 0; i < 500; i++)
    {
      memset(&s, 0, sizeof(s));
      t += TEST_DT_US;
      s.timestamp_sample = t;
      s.timestamp_first = t - TEST_DT_US;
      s.delta_angle_dt = TEST_DT;
      s.delta_velocity_dt = TEST_DT;
      s.delta_velocity[2] = -TEST_G * TEST_DT;
      s.samples = 5;
      s.accel_calibrated = true;
      s.gyro_calibrated = true;
      ekf_core_process(&g_core, &s);
    }

  assert(g_core.initialized);
}

static void test_height_at_reference_is_zero(void)
{
  assert(CLOSE(ekf_baro_height(1013.25f, 1013.25f), 0.0f, 1.0e-3f));
  assert(CLOSE(ekf_baro_height(950.0f, 950.0f), 0.0f, 1.0e-3f));
}

/* Lower pressure means higher up. Near sea level the gradient is about
 * 8.4 m per hPa, so one hPa below the reference is roughly 8.4 m up.
 */

static void test_height_sign_and_scale(void)
{
  float h = ekf_baro_height(1012.25f, 1013.25f);

  assert(h > 0.0f);
  assert(CLOSE(h, 8.4f, 0.5f));
  assert(ekf_baro_height(1014.25f, 1013.25f) < 0.0f);
}

/* The first sample becomes the reference and does not correct the filter.
 * Correcting against a reference that does not exist yet would inject the
 * whole altitude of the site as an error.
 */

static void test_first_sample_sets_reference(void)
{
  align_core();

  assert(!g_core.baro_have_reference);
  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);
  assert(g_core.baro_have_reference);
  assert(CLOSE(g_core.baro_reference_hpa, 1013.25f, 1.0e-3f));
  assert(g_core.baro_accept_count == 0);
  assert(CLOSE(g_core.position[2], 0.0f, 1.0e-3f));
}

/* NED position[2] is down-positive; barometric height is up-positive. A
 * measured RISE must drive position[2] NEGATIVE.
 *
 * This is the single most likely mistake in the whole change, and getting it
 * backwards would look like a working filter that drives into the ground.
 */

static void test_rise_drives_position_down_negative(void)
{
  align_core();

  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);

  /* About 8.4 m up. */

  assert(ekf_core_fuse_baro(&g_core, 1012.25f, 2.0f, 5.0f) == 1);
  assert(g_core.position[2] < 0.0f);
  assert(g_core.baro_accept_count == 1);
  assert(g_core.last_baro_height > 0.0f);
}

/* Pressure outside the physical range is refused before it reaches the
 * filter, and does not become a reference either.
 */

static void test_insane_pressure_refused(void)
{
  align_core();

  assert(ekf_core_fuse_baro(&g_core, 5.0f, 2.0f, 5.0f) == -1);
  assert(!g_core.baro_have_reference);
  assert(ekf_core_fuse_baro(&g_core, 5000.0f, 2.0f, 5.0f) == -1);
  assert(!g_core.baro_have_reference);
  assert(ekf_core_fuse_baro(&g_core, NAN, 2.0f, 5.0f) == -1);
  assert(!g_core.baro_have_reference);
}

/* A gated innovation increments the run and leaves the state alone. */

static void test_gate_rejects_and_counts(void)
{
  struct ekf_core_s before;
  int i;

  align_core();
  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);

  /* Agreeing measurements first, so a wild one is genuinely outside the gate
   * rather than merely surprising a wide initial covariance.
   */

  for (i = 0; i < 20; i++)
    {
      ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f);
    }

  before = g_core;

  /* 100 hPa below the reference is roughly 900 m. */

  assert(ekf_core_fuse_baro(&g_core, 913.25f, 2.0f, 5.0f) == 0);
  assert(g_core.baro_reject_count == before.baro_reject_count + 1);
  assert(g_core.baro_consecutive_rejects ==
         before.baro_consecutive_rejects + 1);
  assert(CLOSE(g_core.position[2], before.position[2], 1.0e-6f));
}

static void test_accept_clears_run(void)
{
  align_core();
  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);
  assert(ekf_core_fuse_baro(&g_core, 913.25f, 2.0f, 5.0f) == 0);
  assert(g_core.baro_consecutive_rejects == 1);
  assert(ekf_core_fuse_baro(&g_core, 1013.20f, 2.0f, 5.0f) == 1);
  assert(g_core.baro_consecutive_rejects == 0);
}

/* Height observations make vertical velocity observable through the
 * covariance cross-terms. This is why EK3_SRC1_VELZ=0 is still correct: the
 * barometer is not a velocity sensor, but it does constrain velocity.
 */

static void test_reduces_vertical_velocity_variance(void)
{
  float before;
  int i;

  align_core();
  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);
  before = g_core.covariance[EKF_P_INDEX(5, 5)];

  for (i = 0; i < 50; i++)
    {
      ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f);
    }

  assert(g_core.covariance[EKF_P_INDEX(5, 5)] < before);
}

/* The reference is tied to the alignment point, so losing alignment must
 * discard it. Keeping it would silently re-datum the height to whatever the
 * pressure was when the filter last aligned somewhere else.
 */

static void test_realignment_discards_reference(void)
{
  struct ekf_imu_sample_s s;
  uint64_t t;

  align_core();
  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);
  assert(g_core.baro_have_reference);

  /* An uncalibrated sample forces a restart. */

  t = g_core.last_timestamp_sample + TEST_DT_US;
  memset(&s, 0, sizeof(s));
  s.timestamp_sample = t;
  s.timestamp_first = t - TEST_DT_US;
  s.delta_angle_dt = TEST_DT;
  s.delta_velocity_dt = TEST_DT;
  s.delta_velocity[2] = -TEST_G * TEST_DT;
  s.samples = 5;
  s.accel_calibrated = false;
  s.gyro_calibrated = false;

  assert(ekf_core_process(&g_core, &s) == EKF_PROCESS_REJECTED);
  assert(!g_core.baro_have_reference);
  assert(g_core.baro_consecutive_rejects == 0);
}

/* An unaligned filter has no trajectory to correct. */

static void test_uninitialised_refuses(void)
{
  struct ekf_core_s fresh;

  ekf_core_init(&fresh);
  assert(ekf_core_fuse_baro(&fresh, 1013.25f, 2.0f, 5.0f) == -1);
  assert(!fresh.baro_have_reference);
}

int main(void)
{
  test_height_at_reference_is_zero();
  test_height_sign_and_scale();
  test_first_sample_sets_reference();
  test_rise_drives_position_down_negative();
  test_insane_pressure_refused();
  test_gate_rejects_and_counts();
  test_accept_clears_run();
  test_reduces_vertical_velocity_variance();
  test_realignment_discards_reference();
  test_uninitialised_refuses();

  puts("ekf_baro: height sign, reference capture and gating verified - OK");
  return 0;
}
