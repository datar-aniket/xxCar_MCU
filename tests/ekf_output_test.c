/****************************************************************************
 * tests/ekf_output_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The output predictor: re-propagation from the delayed filter state to the
 * present. It must agree with the filter's own integration, must not touch
 * the filter, and must be a pure function of its inputs.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ekf_core.h"

#define CLOSE(a, b) (fabsf((a) - (b)) < 1.0e-5f)
#define TEST_DT     0.0025f
#define TEST_DT_US  2500ull
#define TEST_G      9.80665f

static struct ekf_core_s g_core;

/* Drive the core through alignment with a clean stationary sequence, so the
 * tests start from an initialised filter rather than an identity guess.
 */

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

/* Zero samples must reproduce the filter state exactly. This is what makes
 * "EK3_DELAY_MS=0 is inert" true at the publication end as well.
 */

static void test_empty_is_identity(void)
{
  struct ekf_output_s out;

  align_core();
  ekf_core_output_predict(&g_core, NULL, 0, &out);

  assert(out.valid);
  assert(out.samples_replayed == 0);
  assert(CLOSE(out.quaternion[0], g_core.quaternion[0]));
  assert(CLOSE(out.quaternion[1], g_core.quaternion[1]));
  assert(CLOSE(out.quaternion[2], g_core.quaternion[2]));
  assert(CLOSE(out.quaternion[3], g_core.quaternion[3]));
  assert(CLOSE(out.velocity[2], g_core.velocity[2]));
  assert(CLOSE(out.position[2], g_core.position[2]));
  assert(out.timestamp_sample == g_core.last_timestamp_sample);
}

/* Replaying N samples through the predictor must land where the filter lands
 * after processing the same N samples. If these two integrations disagree,
 * the published attitude is not the attitude the filter believes.
 *
 * The motion is deliberately energetic - 0.5 rad/s, above the 0.20 rad/s
 * EKF_DYNAMICS_GYRO_OUT threshold - so the filter leaves low-dynamics on the
 * first sample and its gravity update stops firing. The predictor only does
 * strapdown; comparing it against a filter that is also applying gravity
 * corrections would be comparing two different things, and the test would
 * fail for a reason that is not a bug.
 */

static void test_matches_filter_propagation(void)
{
  struct ekf_imu_sample_s seq[8];
  FAR const struct ekf_imu_sample_s *ptr[8];
  struct ekf_core_s reference;
  struct ekf_output_s out;
  uint64_t t;
  int i;

  align_core();
  t = g_core.last_timestamp_sample;

  for (i = 0; i < 8; i++)
    {
      memset(&seq[i], 0, sizeof(seq[i]));
      t += TEST_DT_US;
      seq[i].timestamp_sample = t;
      seq[i].timestamp_first = t - TEST_DT_US;
      seq[i].delta_angle_dt = TEST_DT;
      seq[i].delta_velocity_dt = TEST_DT;
      seq[i].delta_angle[0] = 0.5f * TEST_DT;    /* out of low-dynamics */
      seq[i].delta_angle[2] = 0.3f * TEST_DT;
      seq[i].delta_velocity[0] = 0.5f * TEST_DT;
      seq[i].delta_velocity[2] = -TEST_G * TEST_DT;
      seq[i].samples = 5;
      seq[i].accel_calibrated = true;
      seq[i].gyro_calibrated = true;
      ptr[i] = &seq[i];
    }

  ekf_core_output_predict(&g_core, ptr, 8, &out);

  reference = g_core;

  for (i = 0; i < 8; i++)
    {
      assert(ekf_core_process(&reference, &seq[i]) == EKF_PROCESS_PREDICTED);
    }

  /* If either of these trips, the sequence was not energetic enough to leave
   * low-dynamics and the gravity update polluted the comparison.
   */

  assert(!reference.low_dynamics);
  assert(reference.gravity_accept_count == g_core.gravity_accept_count);

  assert(out.samples_replayed == 8);
  assert(CLOSE(out.quaternion[0], reference.quaternion[0]));
  assert(CLOSE(out.quaternion[1], reference.quaternion[1]));
  assert(CLOSE(out.quaternion[2], reference.quaternion[2]));
  assert(CLOSE(out.quaternion[3], reference.quaternion[3]));
  assert(CLOSE(out.velocity[0], reference.velocity[0]));
  assert(CLOSE(out.velocity[1], reference.velocity[1]));
  assert(CLOSE(out.velocity[2], reference.velocity[2]));
  assert(CLOSE(out.position[0], reference.position[0]));
  assert(CLOSE(out.position[2], reference.position[2]));
  assert(out.timestamp_sample == seq[7].timestamp_sample);
}

/* The predictor must not modify the filter. It runs on every publication; a
 * predictor that mutated the core would integrate the same samples twice.
 */

static void test_does_not_mutate_core(void)
{
  struct ekf_imu_sample_s s;
  FAR const struct ekf_imu_sample_s *ptr[1];
  struct ekf_core_s before;
  struct ekf_output_s out;

  align_core();
  before = g_core;

  memset(&s, 0, sizeof(s));
  s.timestamp_sample = g_core.last_timestamp_sample + TEST_DT_US;
  s.timestamp_first = g_core.last_timestamp_sample;
  s.delta_angle_dt = TEST_DT;
  s.delta_velocity_dt = TEST_DT;
  s.delta_angle[0] = 0.05f;
  s.samples = 5;
  s.accel_calibrated = true;
  s.gyro_calibrated = true;
  ptr[0] = &s;

  ekf_core_output_predict(&g_core, ptr, 1, &out);

  assert(memcmp(&before, &g_core, sizeof(before)) == 0);
}

/* Repeated calls with the same input give the same answer. The predictor is
 * a pure function of (filter state, samples); anything else means it is
 * carrying hidden state between publications.
 */

static void test_deterministic(void)
{
  struct ekf_imu_sample_s s;
  FAR const struct ekf_imu_sample_s *ptr[1];
  struct ekf_output_s first;
  struct ekf_output_s second;

  align_core();

  memset(&s, 0, sizeof(s));
  s.timestamp_sample = g_core.last_timestamp_sample + TEST_DT_US;
  s.timestamp_first = g_core.last_timestamp_sample;
  s.delta_angle_dt = TEST_DT;
  s.delta_velocity_dt = TEST_DT;
  s.delta_angle[1] = 0.02f;
  s.samples = 5;
  s.accel_calibrated = true;
  s.gyro_calibrated = true;
  ptr[0] = &s;

  ekf_core_output_predict(&g_core, ptr, 1, &first);
  ekf_core_output_predict(&g_core, ptr, 1, &second);

  assert(memcmp(&first, &second, sizeof(first)) == 0);
}

/* An uninitialised filter has no state worth propagating, and must say so
 * rather than publishing an identity attitude as though it were a solution.
 */

static void test_uninitialised_is_invalid(void)
{
  struct ekf_core_s fresh;
  struct ekf_output_s out;

  ekf_core_init(&fresh);
  ekf_core_output_predict(&fresh, NULL, 0, &out);

  assert(!out.valid);
  assert(out.samples_replayed == 0);
}

int main(void)
{
  test_empty_is_identity();
  test_matches_filter_propagation();
  test_does_not_mutate_core();
  test_deterministic();
  test_uninitialised_is_invalid();

  puts("ekf_output: re-propagation matches the filter and is pure - OK");
  return 0;
}
