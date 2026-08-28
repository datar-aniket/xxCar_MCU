/****************************************************************************
 * tests/ekf_baro_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Barometric height fusion: the pressure-to-height conversion, the reference
 * captured at alignment, the z-UP sign convention, and the gating.
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

  /* Keep 200 prediction samples after the ten-second alignment, matching the
   * old fixture's deliberate post-alignment covariance buildup.
   */

  for (i = 0; i < 4200; i++)
    {
      memset(&s, 0, sizeof(s));
      t += TEST_DT_US;
      s.timestamp_sample = t;
      s.timestamp_first = t - TEST_DT_US;
      s.delta_angle_dt = TEST_DT;
      s.delta_velocity_dt = TEST_DT;
      s.delta_velocity[2] = TEST_G * TEST_DT;   /* at rest: specific force is UP */
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

/* The navigation frame's z is UP, so a measured RISE must drive position[2]
 * POSITIVE.
 *
 * This is the single most likely mistake in the whole change, and a
 * stationary bench test cannot catch it: with the vehicle still, both signs
 * leave the innovation at zero and the barometer looks equally healthy
 * either way. Only a deliberate height change discriminates, so it is
 * pinned here.
 */

static void test_rise_drives_position_up_positive(void)
{
  align_core();

  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);

  /* About 8.4 m up. */

  assert(ekf_core_fuse_baro(&g_core, 1012.25f, 2.0f, 5.0f) == 1);
  assert(g_core.position[2] > 0.0f);
  assert(g_core.baro_accept_count == 1);
  assert(g_core.last_baro_height > 0.0f);
}

/* And a descent drives it the other way. Deliberately a SEPARATE alignment
 * rather than a reversal of the test above: the first accepted update
 * collapses P[8][8] from 100 to about 4, so an immediate 17 m reversal is
 * legitimately outside a 5-sigma gate and would fail for a reason that has
 * nothing to do with the sign.
 */

static void test_descent_drives_position_up_negative(void)
{
  align_core();

  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);
  assert(ekf_core_fuse_baro(&g_core, 1014.25f, 2.0f, 5.0f) == 1);
  assert(g_core.position[2] < 0.0f);
  assert(g_core.last_baro_height < 0.0f);
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

/* A height observation may correct only the vertical state family. Seed
 * covariance from height into every other state so this verifies the gain
 * mask rather than relying on naturally small cross-terms.
 */

static void test_height_masks_attitude_and_horizontal_gain(void)
{
  struct ekf_core_s before;
  int state;
  int axis;

  align_core();
  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);

  for (state = 0; state < EKF_STATE_DIM; state++)
    {
      float cross;

      if (state == 8)
        {
          continue;
        }

      cross = 0.10f * sqrtf(
        g_core.covariance[EKF_P_INDEX(state, state)] *
        g_core.covariance[EKF_P_INDEX(8, 8)]);
      g_core.covariance[EKF_P_INDEX(state, 8)] = cross;
      g_core.covariance[EKF_P_INDEX(8, state)] = cross;
    }

  before = g_core;
  assert(ekf_core_fuse_baro(&g_core, 1013.20f, 2.0f, 5.0f) == 1);
  assert(fabsf(g_core.position[2] - before.position[2]) > 1.0e-4f);

  for (state = 0; state < 4; state++)
    {
      assert(CLOSE(g_core.quaternion[state], before.quaternion[state],
                   1.0e-7f));
    }

  for (axis = 0; axis < 2; axis++)
    {
      assert(CLOSE(g_core.velocity[axis], before.velocity[axis], 1.0e-7f));
      assert(CLOSE(g_core.position[axis], before.position[axis], 1.0e-7f));
      assert(CLOSE(g_core.accel_bias[axis], before.accel_bias[axis],
                   1.0e-7f));
    }

  for (axis = 0; axis < 3; axis++)
    {
      assert(CLOSE(g_core.gyro_bias[axis], before.gyro_bias[axis], 1.0e-7f));
    }
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
  s.delta_velocity[2] = TEST_G * TEST_DT;   /* at rest: specific force is UP */
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

/* Vertical validity appears only once the barometer is actually correcting,
 * and disappears on a sustained rejection run. Horizontal validity never
 * appears: nothing in this stage makes it observable, and claiming it would
 * be worse than claiming nothing.
 */

static void test_solution_status_vertical(void)
{
  uint8_t status;
  int i;

  align_core();

  status = ekf_core_solution_status(&g_core);
  assert((status & EKF_SOLUTION_ATTITUDE) != 0);
  assert((status & EKF_SOLUTION_YAW_RELATIVE) != 0);
  assert((status & EKF_SOLUTION_YAW_ABSOLUTE) == 0);
  assert((status & EKF_SOLUTION_POSITION_VERT) == 0);

  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);

  for (i = 0; i < 10; i++)
    {
      assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == 1);
    }

  status = ekf_core_solution_status(&g_core);
  assert((status & EKF_SOLUTION_POSITION_VERT) != 0);
  assert((status & EKF_SOLUTION_VELOCITY_VERT) != 0);
  assert((status & EKF_SOLUTION_POSITION_HORIZ) == 0);
  assert((status & EKF_SOLUTION_VELOCITY_HORIZ) == 0);

  /* A sustained rejection run withdraws the claim, and attitude survives it. */

  for (i = 0; i < (int)EKF_BARO_REJECT_RUN_MAX + 1; i++)
    {
      ekf_core_fuse_baro(&g_core, 913.25f, 2.0f, 5.0f);
    }

  status = ekf_core_solution_status(&g_core);
  assert((status & EKF_SOLUTION_POSITION_VERT) == 0);
  assert((status & EKF_SOLUTION_ATTITUDE) != 0);
}

int main(void)
{
  test_height_at_reference_is_zero();
  test_height_sign_and_scale();
  test_first_sample_sets_reference();
  test_rise_drives_position_up_positive();
  test_descent_drives_position_up_negative();
  test_insane_pressure_refused();
  test_gate_rejects_and_counts();
  test_accept_clears_run();
  test_reduces_vertical_velocity_variance();
  test_height_masks_attitude_and_horizontal_gain();
  test_realignment_discards_reference();
  test_uninitialised_refuses();
  test_solution_status_vertical();

  puts("ekf_baro: height sign, reference capture and gating verified - OK");
  return 0;
}
