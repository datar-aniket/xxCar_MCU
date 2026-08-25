/****************************************************************************
 * tests/ekf_core_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ekf_core.h"
#include "ekf_delay.h"

#define TEST_DT       0.0025f
#define TEST_DT_US    2500ull
#define TEST_GRAVITY  9.80665f
#define TEST_PI       3.14159265358979323846f

static void assert_near(float actual, float expected, float tolerance)
{
  if (fabsf(actual - expected) > tolerance)
    {
      fprintf(stderr, "actual %.9g expected %.9g tolerance %.9g\n",
              (double)actual, (double)expected, (double)tolerance);
      assert(false);
    }
}

static void assert_covariance_positive_definite(const struct ekf_core_s *ekf)
{
  double lower[EKF_STATE_DIM][EKF_STATE_DIM] = {{0.0}};
  int row;
  int column;
  int inner;

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      for (column = 0; column <= row; column++)
        {
          double value = ekf->covariance[EKF_P_INDEX(row, column)];

          for (inner = 0; inner < column; inner++)
            {
              value -= lower[row][inner] * lower[column][inner];
            }

          if (row == column)
            {
              assert(value > 0.0);
              lower[row][column] = sqrt(value);
            }
          else
            {
              lower[row][column] = value / lower[column][column];
            }
        }
    }
}

static void rest_accel(float roll, float pitch, float accel[3])
{
  accel[0] = -sinf(pitch) * TEST_GRAVITY;
  accel[1] = sinf(roll) * cosf(pitch) * TEST_GRAVITY;
  accel[2] = cosf(roll) * cosf(pitch) * TEST_GRAVITY;
}

static void make_sample(struct ekf_imu_sample_s *sample, uint64_t timestamp,
                        const float accel[3], const float gyro[3])
{
  int axis;

  memset(sample, 0, sizeof(*sample));
  sample->timestamp_sample = timestamp;
  sample->timestamp_first = timestamp - TEST_DT_US;
  sample->delta_angle_dt = TEST_DT;
  sample->delta_velocity_dt = TEST_DT;
  sample->samples = 5;
  sample->accel_calibrated = true;
  sample->gyro_calibrated = true;

  for (axis = 0; axis < 3; axis++)
    {
      sample->delta_angle[axis] = gyro[axis] * TEST_DT;
      sample->delta_velocity[axis] = accel[axis] * TEST_DT;
    }
}

static void initialize_tilted(struct ekf_core_s *ekf, uint64_t *timestamp,
                              float roll, float pitch,
                              const float gyro_bias[3])
{
  struct ekf_imu_sample_s sample;
  float accel[3];
  int result = EKF_PROCESS_ALIGNING;
  int count;

  rest_accel(roll, pitch, accel);

  for (count = 0; count < 450 && !ekf->initialized; count++)
    {
      *timestamp += TEST_DT_US;
      make_sample(&sample, *timestamp, accel, gyro_bias);
      result = ekf_core_process(ekf, &sample);
      assert(result >= EKF_PROCESS_ALIGNING);
    }

  assert(result == EKF_PROCESS_INITIALIZED);
  assert(ekf->initialized);
  assert(ekf->align_samples >= 400);
}

static void test_initialization_and_static_prediction(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s sample;
  const float roll = 10.0f * TEST_PI / 180.0f;
  const float pitch = -5.0f * TEST_PI / 180.0f;
  const float gyro_bias[3] = {0.010f, -0.012f, 0.004f};
  float accel[3];
  float euler[3];
  uint64_t timestamp = 1000000ull;
  int row;
  int column;
  int index;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, roll, pitch, gyro_bias);
  ekf_core_euler(&ekf, euler);
  assert_near(euler[0], roll, 2.0e-5f);
  assert_near(euler[1], pitch, 2.0e-5f);
  assert_near(euler[2], 0.0f, 2.0e-5f);
  assert(ekf_core_solution_status(&ekf) ==
         (EKF_SOLUTION_ATTITUDE | EKF_SOLUTION_YAW_RELATIVE));

  for (index = 0; index < 3; index++)
    {
      assert_near(ekf.gyro_bias[index], gyro_bias[index], 2.0e-6f);
    }

  rest_accel(roll, pitch, accel);

  for (index = 0; index < 400; index++)
    {
      timestamp += TEST_DT_US;
      make_sample(&sample, timestamp, accel, gyro_bias);
      assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_PREDICTED);
    }

  assert(ekf.predict_count == 400);
  assert(ekf.covariance_count == 100);
  assert(ekf.low_dynamics);
  assert(ekf.low_dynamics_entry_count == 1);
  assert(ekf.gravity_accept_count == 100);
  assert(ekf.gravity_reject_count == 0);

  for (index = 0; index < 3; index++)
    {
      assert_near(ekf.velocity[index], 0.0f, 2.0e-4f);
      assert_near(ekf.position[index], 0.0f, 2.0e-4f);
    }

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      assert(isfinite(ekf.covariance[EKF_P_INDEX(row, row)]));
      assert(ekf.covariance[EKF_P_INDEX(row, row)] > 0.0f);

      for (column = 0; column < EKF_STATE_DIM; column++)
        {
          assert_near(ekf.covariance[EKF_P_INDEX(row, column)],
                      ekf.covariance[EKF_P_INDEX(column, row)], 1.0e-6f);
        }
    }

  assert_covariance_positive_definite(&ekf);
}

static void test_yaw_prediction(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s sample;
  const float gyro_bias[3] = {0.002f, -0.003f, 0.001f};
  const float accel[3] = {0.0f, 0.0f, TEST_GRAVITY};
  float moving_gyro[3] = {gyro_bias[0], gyro_bias[1],
                          gyro_bias[2] + 0.5f};
  float euler[3];
  uint64_t timestamp = 2000000ull;
  int index;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, gyro_bias);

  for (index = 0; index < 400; index++)
    {
      timestamp += TEST_DT_US;
      make_sample(&sample, timestamp, accel, moving_gyro);
      assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_PREDICTED);
    }

  ekf_core_euler(&ekf, euler);
  assert_near(euler[0], 0.0f, 2.0e-4f);
  assert_near(euler[1], 0.0f, 2.0e-4f);
  assert_near(euler[2], 0.5f, 3.0e-4f);
  assert(!ekf.low_dynamics);
  assert(ekf.low_dynamics_exit_count == 1);
  assert(ekf.gravity_accept_count == 0);
}

static void test_low_dynamics_updates(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s sample;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  float biased_gyro[3] = {0.006f, -0.004f, 0.0f};
  float biased_accel[3] = {0.0f, 0.0f, TEST_GRAVITY + 0.20f};
  float transient_accel[3] = {1.6f, 0.0f, TEST_GRAVITY};
  float euler[3];
  uint64_t timestamp = 6000000ull;
  uint32_t rejected_before;
  int index;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  assert(ekf.low_dynamics);

  for (index = 0; index < 2000; index++)
    {
      timestamp += TEST_DT_US;
      make_sample(&sample, timestamp, biased_accel, biased_gyro);
      assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_PREDICTED);
    }

  ekf_core_euler(&ekf, euler);
  assert_near(ekf.gyro_bias[0], biased_gyro[0], 1.5e-3f);
  assert_near(ekf.gyro_bias[1], biased_gyro[1], 1.5e-3f);
  assert_near(ekf.accel_bias[2], 0.20f, 3.0e-2f);
  assert_near(euler[0], 0.0f, 4.0e-3f);
  assert_near(euler[1], 0.0f, 4.0e-3f);
  assert(ekf.gravity_accept_count > 450);
  assert(ekf.gravity_reject_count == 0);
  assert(ekf.bias_limit_count == 0);
  assert_covariance_positive_definite(&ekf);

  rejected_before = ekf.gravity_reject_count;

  for (index = 0; index < EKF_COVARIANCE_INTERVAL; index++)
    {
      timestamp += TEST_DT_US;
      make_sample(&sample, timestamp, transient_accel, biased_gyro);
      assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_PREDICTED);
    }

  assert(ekf.low_dynamics);
  assert(ekf.gravity_reject_count == rejected_before + 1);
  assert(ekf.last_gravity_nis > 16.3f);
}

static void test_gravity_preserves_yaw_gauge(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s sample;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float yaw = 0.70f;
  float accel[3] = {0.20f, 0.0f, TEST_GRAVITY};
  float euler[3];
  float gyro_bias_z;
  uint64_t timestamp = 8000000ull;
  int index;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);

  ekf.quaternion[0] = cosf(0.5f * yaw);
  ekf.quaternion[1] = 0.0f;
  ekf.quaternion[2] = 0.0f;
  ekf.quaternion[3] = sinf(0.5f * yaw);

  /* Deliberately couple the unobservable yaw and Z gyro bias states to an
   * observed accel-bias state. An unconstrained Kalman gain would use this
   * correlation to alter both states during the gravity update.
   */

  ekf.covariance[EKF_P_INDEX(2, 12)] = 0.10f;
  ekf.covariance[EKF_P_INDEX(12, 2)] = 0.10f;
  ekf.covariance[EKF_P_INDEX(11, 12)] = 0.001f;
  ekf.covariance[EKF_P_INDEX(12, 11)] = 0.001f;
  assert_covariance_positive_definite(&ekf);
  gyro_bias_z = ekf.gyro_bias[2];

  for (index = 0; index < EKF_COVARIANCE_INTERVAL; index++)
    {
      timestamp += TEST_DT_US;
      make_sample(&sample, timestamp, accel, zero_gyro);
      assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_PREDICTED);
    }

  ekf_core_euler(&ekf, euler);
  assert_near(euler[2], yaw, 2.0e-6f);
  assert_near(ekf.gyro_bias[2], gyro_bias_z, 2.0e-8f);
  assert(ekf.gravity_accept_count == 1);
  assert(ekf.gravity_yaw_projection_count == 1);
  assert(fabsf(ekf.last_gravity_yaw_suppressed) > 1.0e-3f);
  assert(ekf.max_gravity_yaw_suppressed >=
         fabsf(ekf.last_gravity_yaw_suppressed));
  assert_covariance_positive_definite(&ekf);
}

static void test_fault_resets(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s sample;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float accel[3] = {0.0f, 0.0f, TEST_GRAVITY};
  uint64_t timestamp = 3000000ull;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);

  timestamp += TEST_DT_US;
  make_sample(&sample, timestamp, accel, zero_gyro);
  assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_PREDICTED);
  assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_REJECTED);
  assert(ekf.duplicate_count == 1);
  assert(ekf.initialized);

  /* Skip exactly one 2.5 ms packet. Boundary continuity must catch this even
   * though a 5 ms end-to-end interval is not an extreme wall-clock gap.
   */

  timestamp += 2ull * TEST_DT_US;
  make_sample(&sample, timestamp, accel, zero_gyro);
  assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_ALIGNING);
  assert(ekf.gap_count == 1);
  assert(!ekf.initialized);

  ekf_core_init(&ekf);
  timestamp = 5000000ull;
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  timestamp += TEST_DT_US;
  make_sample(&sample, timestamp, accel, zero_gyro);
  sample.reset_counter = 1;
  assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_ALIGNING);
  assert(ekf.source_reset_count == 1);
  assert(!ekf.initialized);

  timestamp += TEST_DT_US;
  make_sample(&sample, timestamp, accel, zero_gyro);
  sample.reset_counter = 1;
  sample.accel_calibrated = false;
  assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_REJECTED);
  assert(ekf.uncalibrated_count == 1);
}

/* The scalar update must agree exactly with the 3-D update on a measurement
 * that observes only one state.
 *
 * Rows 1 and 2 of H are all zero, so PH' has zero columns there and the gain
 * in those directions is zero: the 3-D update reduces algebraically to the
 * scalar one on row 0. Two independently written Kalman updates that disagree
 * is the kind of bug that shows up as a filter subtly wrong for months, so
 * pin them against each other rather than trusting the derivation.
 */

static void test_update_1d_matches_3d(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s scalar;
  struct ekf_core_s vector;
  uint64_t timestamp = 1000000ull;
  float h1[EKF_STATE_DIM];
  float h3[3][EKF_STATE_DIM];
  float residual3[3];
  float nis1 = 0.0f;
  float nis3 = 0.0f;
  int i;

  ekf_core_init(&scalar);
  initialize_tilted(&scalar, &timestamp, 0.0f, 0.0f, zero_gyro);
  vector = scalar;

  memset(h1, 0, sizeof(h1));
  h1[8] = 1.0f;                 /* observe position down only */

  memset(h3, 0, sizeof(h3));
  memset(residual3, 0, sizeof(residual3));
  h3[0][8] = 1.0f;
  residual3[0] = 1.5f;

  /* Gate generously on the scalar side: the 3-D update uses the fixed
   * EKF_MEASUREMENT_NIS_GATE, so the comparison must not be decided by a
   * difference in gating.
   */

  assert(ekf_core_test_update_1d(&scalar, h1, 1.5f, 4.0f, 1000.0f,
                                 &nis1) == 1);
  assert(ekf_core_test_update_3d(&vector, h3, residual3, 4.0f, &nis3) == 1);

  assert_near(nis1, nis3, 1.0e-4f);
  assert_near(scalar.position[2], vector.position[2], 1.0e-5f);
  assert_near(scalar.velocity[2], vector.velocity[2], 1.0e-5f);
  assert_near(scalar.accel_bias[2], vector.accel_bias[2], 1.0e-6f);

  for (i = 0; i < EKF_STATE_DIM; i++)
    {
      assert_near(scalar.covariance[EKF_P_INDEX(i, i)],
                  vector.covariance[EKF_P_INDEX(i, i)], 1.0e-5f);
    }
}

/* An innovation beyond the gate must leave the state completely untouched. A
 * partially applied correction would be worse than either accepting or
 * rejecting, because it is neither the old estimate nor the new one.
 */

static void test_update_1d_gate_rejects_cleanly(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s before;
  struct ekf_core_s ekf;
  uint64_t timestamp = 1000000ull;
  float h[EKF_STATE_DIM];
  float nis = 0.0f;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  before = ekf;

  memset(h, 0, sizeof(h));
  h[8] = 1.0f;

  /* 1000 m against a metre-scale variance is outside any sane gate. */

  assert(ekf_core_test_update_1d(&ekf, h, 1000.0f, 4.0f, 5.0f, &nis) == 0);
  assert(nis > 25.0f);
  assert(memcmp(&before, &ekf, sizeof(before)) == 0);
}

/* A correction must reduce the variance of the state it observed, without
 * driving it to zero or negative.
 */

static void test_update_1d_reduces_variance(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s ekf;
  uint64_t timestamp = 1000000ull;
  float h[EKF_STATE_DIM];
  float before;
  float nis = 0.0f;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  before = ekf.covariance[EKF_P_INDEX(8, 8)];

  memset(h, 0, sizeof(h));
  h[8] = 1.0f;

  assert(ekf_core_test_update_1d(&ekf, h, 0.5f, 4.0f, 1000.0f, &nis) == 1);
  assert(ekf.covariance[EKF_P_INDEX(8, 8)] < before);
  assert(ekf.covariance[EKF_P_INDEX(8, 8)] > 0.0f);
  assert_covariance_positive_definite(&ekf);
}


static struct ekf_extnav_sample_s extnav_at(float x, float y, float yaw)
{
  struct ekf_extnav_sample_s s;

  memset(&s, 0, sizeof(s));
  s.x = x;
  s.y = y;
  s.yaw = yaw;
  s.valid = true;
  return s;
}

static void extnav_align(struct ekf_core_s *ekf, uint64_t *timestamp)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};

  *timestamp = 1000000ull;
  ekf_core_init(ekf);
  ekf_core_set_extnav_config(ekf, 1000000u);
  initialize_tilted(ekf, timestamp, 0.0f, 0.0f, zero_gyro);
}

/* The first pose SETS the filter rather than correcting it.
 *
 * The map origin may be tens of metres from where the filter aligned. Fusing
 * would make the first innovation enormous, the gate would reject it, and it
 * would go on rejecting every pose after it for ever.
 */

static void test_extnav_first_pose_sets_the_datum(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(120.0f, -45.0f, 0.7f);
  float euler[3];

  extnav_align(&ekf, &timestamp);

  assert(!ekf.extnav_datum_set);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, true) == -2);
  assert(ekf.extnav_datum_set);
  assert(ekf.extnav_datum_count == 1);

  assert_near(ekf.position[0], 120.0f, 1.0e-4f);
  assert_near(ekf.position[1], -45.0f, 1.0e-4f);
  ekf_core_euler(&ekf, euler);
  assert_near(euler[2], 0.7f, 1.0e-4f);

  /* Horizontal only. Height belongs to the barometer, roll and pitch to
   * gravity, and the external source says nothing about either.
   */

  assert_near(ekf.position[2], 0.0f, 1.0e-6f);
  assert_near(euler[0], 0.0f, 1.0e-3f);
  assert_near(euler[1], 0.0f, 1.0e-3f);
  assert(ekf.extnav_accept_count == 0);
}

static void test_extnav_fuses_after_the_datum(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(10.0f, 5.0f, 0.0f);
  float before;

  extnav_align(&ekf, &timestamp);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, true) == -2);

  before = ekf.covariance[EKF_P_INDEX(6, 6)];
  s.x = 10.05f;
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, true) == 1);
  assert(ekf.extnav_accept_count == 1);
  assert(ekf.covariance[EKF_P_INDEX(6, 6)] < before);
  assert_covariance_positive_definite(&ekf);
}

/* The parameter is a FLOOR. A source claiming 1 mm is fused at the
 * configured 0.1 m, not at what it claimed.
 */

static void test_extnav_noise_floor_wins(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);

  extnav_align(&ekf, &timestamp);
  s.pos_sigma[0] = 0.001f;
  s.pos_sigma[1] = 0.001f;

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == 1);
  assert_near(ekf.last_extnav_noise, 0.1f, 1.0e-6f);
}

/* A source reporting WORSE than the floor is believed - it is a minimum,
 * not a fixed value.
 */

static void test_extnav_honours_a_worse_reported_sigma(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);

  extnav_align(&ekf, &timestamp);
  s.pos_sigma[0] = 2.0f;
  s.pos_sigma[1] = 2.0f;

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == 1);
  assert_near(ekf.last_extnav_noise, 2.0f, 1.0e-6f);
}

static void test_extnav_refuses_an_invalid_pose(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(1.0f, 2.0f, 0.0f);

  extnav_align(&ekf, &timestamp);
  s.valid = false;

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, true) == -1);
  assert(!ekf.extnav_datum_set);
}

/* A sustained rejection run re-datums rather than rejecting for ever.
 *
 * After a dropout the filter has drifted, so every incoming pose looks
 * impossible and the gate rejects it. Only a reset recovers - ArduPilot's
 * ResetPositionNE().
 */

static void test_extnav_rejection_run_forces_a_redatum(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  unsigned i;

  extnav_align(&ekf, &timestamp);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);

  /* Tighten the position variance so a distant pose is genuinely outside
   * the gate rather than merely surprising.
   */

  for (i = 0; i < 40; i++)
    {
      ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);
    }

  s.x = 500.0f;

  for (i = 0; i < EKF_EXTNAV_REJECT_RUN_MAX; i++)
    {
      assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                                  true, false) == 0);
    }

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert_near(ekf.position[0], 500.0f, 1.0e-3f);
  assert(ekf.extnav_datum_count == 2);
  assert(ekf.extnav_consecutive_rejects == 0);
}

/* The source telling us it relocalised is worth more than twenty gated
 * innovations saying the same thing more slowly.
 */

static void test_extnav_source_reset_forces_a_redatum(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);

  extnav_align(&ekf, &timestamp);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);

  s.reset_counter = 1;
  s.x = 77.0f;
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert_near(ekf.position[0], 77.0f, 1.0e-3f);
  assert(ekf.extnav_datum_count == 2);
}

/* Silence withdraws horizontal validity.
 *
 * A source that simply stops talking leaves no rejections behind, so without
 * an age check the claim would stand for ever on a dead link - the worst
 * kind of stale, because everything downstream still believes it.
 */

static void test_extnav_timeout_drops_horizontal_validity(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(1.0f, 2.0f, 0.0f);
  struct ekf_imu_sample_s sample;
  float accel[3];
  int i;

  extnav_align(&ekf, &timestamp);

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == 1);
  assert((ekf_core_solution_status(&ekf) &
          EKF_SOLUTION_POSITION_HORIZ) != 0);

  rest_accel(0.0f, 0.0f, accel);

  for (i = 0; i < 800; i++)          /* two seconds of silence */
    {
      timestamp += TEST_DT_US;
      make_sample(&sample, timestamp, accel, zero_gyro);
      ekf_core_process(&ekf, &sample);
    }

  assert((ekf_core_solution_status(&ekf) &
          EKF_SOLUTION_POSITION_HORIZ) == 0);
  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_ATTITUDE) != 0);
}

/* An alignment restart discards the datum. The local frame is gone, so
 * claiming map coordinates for it would be a lie.
 */

static void test_extnav_datum_clears_on_restart(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(3.0f, 4.0f, 0.0f);
  struct ekf_imu_sample_s sample;
  float accel[3];

  extnav_align(&ekf, &timestamp);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert(ekf.extnav_datum_set);

  rest_accel(0.0f, 0.0f, accel);
  timestamp += TEST_DT_US;
  make_sample(&sample, timestamp, accel, zero_gyro);
  sample.accel_calibrated = false;
  ekf_core_process(&ekf, &sample);

  assert(!ekf.extnav_datum_set);
}

int main(void)
{
  test_initialization_and_static_prediction();
  test_yaw_prediction();
  test_low_dynamics_updates();
  test_gravity_preserves_yaw_gauge();
  test_fault_resets();
  test_update_1d_matches_3d();
  test_update_1d_gate_rejects_cleanly();
  test_update_1d_reduces_variance();
  test_extnav_first_pose_sets_the_datum();
  test_extnav_fuses_after_the_datum();
  test_extnav_noise_floor_wins();
  test_extnav_honours_a_worse_reported_sigma();
  test_extnav_refuses_an_invalid_pose();
  test_extnav_rejection_run_forces_a_redatum();
  test_extnav_source_reset_forces_a_redatum();
  test_extnav_timeout_drops_horizontal_validity();
  test_extnav_datum_clears_on_restart();
  puts("ekf core tests: PASS");
  return 0;
}
