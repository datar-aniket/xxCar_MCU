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

  for (count = 0; count < 4200 && !ekf->initialized; count++)
    {
      *timestamp += TEST_DT_US;
      make_sample(&sample, *timestamp, accel, gyro_bias);
      result = ekf_core_process(ekf, &sample);
      assert(result >= EKF_PROCESS_ALIGNING);
    }

  assert(result == EKF_PROCESS_INITIALIZED);
  assert(ekf->initialized);
  assert(ekf->align_time_s >= EKF_ALIGN_TIME_S);
  assert(ekf->align_samples >= 4000);
}

static void test_startup_estimates_uncalibrated_gyro_bias(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s sample;
  const float accel[3] = {0.0f, 0.0f, TEST_GRAVITY};
  const float gyro_bias[3] = {0.024f, -0.015f, 0.009f};
  uint64_t timestamp = 1000000ull;
  int count;

  ekf_core_init(&ekf);

  for (count = 0; count < 3999; count++)
    {
      timestamp += TEST_DT_US;
      make_sample(&sample, timestamp, accel, gyro_bias);
      sample.gyro_calibrated = false;
      assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_ALIGNING);
    }

  assert(!ekf.initialized);

  while (!ekf.initialized && count++ < 4200)
    {
      timestamp += TEST_DT_US;
      make_sample(&sample, timestamp, accel, gyro_bias);
      sample.gyro_calibrated = false;
      assert(ekf_core_process(&ekf, &sample) >= EKF_PROCESS_ALIGNING);
    }

  assert(ekf.initialized);
  assert(ekf.gyro_bias_from_uncalibrated);
  assert_near(ekf.gyro_bias[0], gyro_bias[0], 1.0e-5f);
  assert_near(ekf.gyro_bias[1], gyro_bias[1], 1.0e-5f);
  assert_near(ekf.gyro_bias[2], gyro_bias[2], 1.0e-5f);
}

static void test_startup_requires_ten_continuous_stationary_seconds(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s sample;
  const float accel[3] = {0.0f, 0.0f, TEST_GRAVITY};
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float moving_gyro[3] = {0.5f, 0.0f, 0.0f};
  uint64_t timestamp = 1000000ull;
  int count;

  ekf_core_init(&ekf);

  for (count = 0; count < 2000; count++)
    {
      timestamp += TEST_DT_US;
      make_sample(&sample, timestamp, accel, zero_gyro);
      assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_ALIGNING);
    }

  timestamp += TEST_DT_US;
  make_sample(&sample, timestamp, accel, moving_gyro);
  assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_ALIGNING);
  assert(ekf.align_samples == 0);
  assert(ekf.alignment_restart_count == 1);

  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  assert(ekf.initialized);
}

static void test_zupt_requires_stationary_imu(void)
{
  struct ekf_core_s ekf;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  uint64_t timestamp = 1000000ull;
  float gravity_deviation;
  float accel_variance;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);

  assert(ekf_core_zupt_stationary(&ekf, 0.25f, 0.10f,
                                  &gravity_deviation, &accel_variance));
  assert(gravity_deviation < 1.0e-4f);
  assert(accel_variance < 1.0e-4f);

  ekf.zupt_accel_mean[0] = 1.0f;
  assert(!ekf_core_zupt_stationary(&ekf, 0.02f, 0.10f, NULL, NULL));

  ekf.zupt_accel_mean[0] = 0.0f;
  ekf.zupt_accel_variance[0] = 0.11f;
  assert(!ekf_core_zupt_stationary(&ekf, 0.25f, 0.10f, NULL, NULL));

  ekf.zupt_accel_variance[0] = 0.0f;
  ekf.low_dynamics = false;
  assert(ekf_core_zupt_stationary(&ekf, 0.25f, 0.10f, NULL, NULL));

  ekf.zupt_gyro_mean[2] = 0.10f;
  assert(!ekf_core_zupt_stationary(&ekf, 0.25f, 0.10f, NULL, NULL));
  ekf.zupt_gyro_mean[2] = 0.0f;
  ekf.zupt_last_clipped = true;
  assert(!ekf_core_zupt_stationary(&ekf, 0.25f, 0.10f, NULL, NULL));
}

/* Braking history must leave the wheel-stop detector in hundreds of
 * milliseconds, not the 1.1-1.5 seconds measured with the general
 * low-dynamics EWMA and dwell.  The general detector deliberately remains
 * false here: ZUPT eligibility is independent of it.
 */

static void test_zupt_short_window_recovers_after_braking(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s sample;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float braking[3] = {2.0f, 0.0f, TEST_GRAVITY};
  const float rest[3] = {0.0f, 0.0f, TEST_GRAVITY};
  uint64_t timestamp = 1000000ull;
  unsigned i;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);

  for (i = 0; i < 40; i++)
    {
      timestamp += TEST_DT_US;
      make_sample(&sample, timestamp, braking, zero_gyro);
      assert(ekf_core_process(&ekf, &sample) >= EKF_PROCESS_PREDICTED);
    }

  assert(!ekf_core_zupt_stationary(&ekf, 0.25f, 0.10f, NULL, NULL));

  /* Model the general detector having rejected the stop on independent
   * position/linear-acceleration evidence. Its 0.5 s re-entry dwell must not
   * become the ZUPT's dwell as well.
   */

  ekf.low_dynamics = false;
  ekf.low_dynamics_dwell_s = 0.0f;

  for (i = 0; i < 100; i++)
    {
      timestamp += TEST_DT_US;
      make_sample(&sample, timestamp, rest, zero_gyro);
      assert(ekf_core_process(&ekf, &sample) >= EKF_PROCESS_PREDICTED);
    }

  assert(!ekf.low_dynamics);
  assert(ekf_core_zupt_stationary(&ekf, 0.25f, 0.10f, NULL, NULL));
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

static void test_acceleration_audit_matches_strapdown(void)
{
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float measured[3] = {0.13f, -0.16f, TEST_GRAVITY + 0.02f};
  struct ekf_imu_sample_s sample;
  struct ekf_core_s ekf;
  uint64_t timestamp = 1000000ull;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  ekf.accel_bias[0] = 0.10f;
  ekf.accel_bias[1] = -0.20f;
  ekf.accel_bias[2] = 0.05f;

  timestamp += TEST_DT_US;
  make_sample(&sample, timestamp, measured, zero_gyro);
  assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_PREDICTED);

  assert(ekf.last_predict_timestamp == timestamp);
  assert_near(ekf.last_specific_force[0], 0.13f, 1.0e-5f);
  assert_near(ekf.last_corrected_force[0], 0.03f, 1.0e-5f);
  assert_near(ekf.last_corrected_force[1], 0.04f, 1.0e-5f);
  assert_near(ekf.last_corrected_force[2],
              TEST_GRAVITY - 0.03f, 1.0e-5f);
  assert_near(ekf.last_gravity_body[0], 0.0f, 1.0e-5f);
  assert_near(ekf.last_gravity_body[1], 0.0f, 1.0e-5f);
  assert_near(ekf.last_gravity_body[2], TEST_GRAVITY, 1.0e-5f);
  assert_near(ekf.last_residual_accel_body[0], 0.03f, 1.0e-5f);
  assert_near(ekf.last_residual_accel_body[1], 0.04f, 1.0e-5f);
  assert_near(ekf.last_residual_accel_body[2], -0.03f, 1.0e-5f);
  assert_near(ekf.last_nav_accel[0], 0.03f, 2.0e-5f);
  assert_near(ekf.last_nav_accel[1], 0.04f, 2.0e-5f);
  assert_near(ekf.last_nav_accel[2], -0.03f, 2.0e-5f);
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

/* imu_integrator has already rotated every native delta velocity into the
 * packet-start body frame. The EKF must apply the packet-start attitude once,
 * not a midpoint attitude that rotates the sculling correction a second time.
 */

static void test_sculling_delta_uses_packet_start_attitude(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s sample;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float rest[3] = {0.0f, 0.0f, TEST_GRAVITY};
  const float angle = 0.25f;
  const float rate = 100.0f;
  uint64_t timestamp = 5000000ull;
  float expected_x;
  float expected_y;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);

  timestamp += TEST_DT_US;
  make_sample(&sample, timestamp, rest, zero_gyro);
  sample.delta_angle[2] = angle;
  expected_x = sinf(angle) / rate;
  expected_y = (1.0f - cosf(angle)) / rate;
  sample.delta_velocity[0] = expected_x;
  sample.delta_velocity[1] = expected_y;

  assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_PREDICTED);
  assert_near(ekf.velocity[0], expected_x, 2.0e-7f);
  assert_near(ekf.velocity[1], expected_y, 2.0e-7f);
  assert_near(ekf.velocity[2], 0.0f, 2.0e-7f);
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

/* The core injects attitude errors by right-multiplying the quaternion, so
 * covariance propagation must use -R*skew(f). A sign error here reverses the
 * attitude/velocity cross-covariance and makes position innovations drive
 * tilt in the wrong direction.
 */

static void test_covariance_attitude_velocity_sign(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s sample;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float accel[3] = {1.0f, 0.0f, TEST_GRAVITY};
  uint64_t timestamp = 7000000ull;
  int index;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);

  for (index = 0; index < EKF_COVARIANCE_INTERVAL; index++)
    {
      timestamp += TEST_DT_US;
      make_sample(&sample, timestamp, accel, zero_gyro);

      /* Keep this a propagation-only check. */

      sample.clipping = 1;
      assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_PREDICTED);
    }

  assert(ekf.covariance[EKF_P_INDEX(3, 1)] > 0.0f); /* vx vs pitch */
  assert(ekf.covariance[EKF_P_INDEX(4, 0)] < 0.0f); /* vy vs roll */
  assert(ekf.covariance[EKF_P_INDEX(5, 1)] < 0.0f); /* vz vs pitch */
  assert_covariance_positive_definite(&ekf);
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

static void test_secondary_imu_instance_is_valid(void)
{
  struct ekf_core_s ekf;
  struct ekf_imu_sample_s sample;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float accel[3] = {0.0f, 0.0f, TEST_GRAVITY};

  ekf_core_init(&ekf);
  make_sample(&sample, 1000000ull, accel, zero_gyro);
  sample.instance = 1;
  assert(ekf_core_process(&ekf, &sample) == EKF_PROCESS_ALIGNING);
  assert(ekf.align_samples == 1);
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

  /* A datum COUNTS as position aiding, which reverses what this used to
   * assert.
   *
   * It is the strongest position information the filter ever receives - the
   * state is set to the fix outright rather than nudged towards it - so
   * reporting "not aided" until some later pose happened to pass a gate had
   * it backwards. It mattered once the position hold existed: the hold
   * engaged on the step after a datum and floored the very covariance the
   * datum had just reset.
   */

  assert(ekf.extnav_accept_count == 1);
  assert(ekf_core_position_aided(&ekf));
  assert(ekf_core_observability(&ekf) == EKF_OBS_POSITION);
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
  /* Two now, not one: the datum itself counts as the first. */

  assert(ekf.extnav_accept_count == 2);
  assert(ekf.covariance[EKF_P_INDEX(6, 6)] < before);
  assert_covariance_positive_definite(&ekf);
}

/* Horizontal position is allowed to correct horizontal velocity and
 * position only. Deliberately seed cross-covariance to every other state so
 * this cannot pass merely because the unwanted gains happened to be zero.
 */

static void test_extnav_position_masks_attitude_and_vertical_gain(void)
{
  struct ekf_core_s ekf;
  struct ekf_core_s before;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  uint64_t timestamp;
  int state;

  extnav_align(&ekf, &timestamp);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);

  for (state = 0; state < EKF_STATE_DIM; state++)
    {
      float cross;

      if (state == 6)
        {
          continue;
        }

      cross = 0.10f * sqrtf(
        ekf.covariance[EKF_P_INDEX(state, state)] *
        ekf.covariance[EKF_P_INDEX(6, 6)]);
      ekf.covariance[EKF_P_INDEX(state, 6)] = cross;
      ekf.covariance[EKF_P_INDEX(6, state)] = cross;
    }

  assert_covariance_positive_definite(&ekf);
  before = ekf;
  s.x = 0.02f;

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == 1);

  /* The supported derivative state and measured state do move. */

  assert(fabsf(ekf.velocity[0] - before.velocity[0]) > 1.0e-6f);
  assert(fabsf(ekf.position[0] - before.position[0]) > 1.0e-4f);

  /* No attitude, vertical state, or bias is injected by x/y position. */

  for (state = 0; state < 4; state++)
    {
      assert_near(ekf.quaternion[state], before.quaternion[state], 1.0e-9f);
    }

  assert_near(ekf.velocity[2], before.velocity[2], 1.0e-9f);
  assert_near(ekf.position[2], before.position[2], 1.0e-9f);

  for (state = 0; state < 3; state++)
    {
      assert_near(ekf.gyro_bias[state], before.gyro_bias[state], 1.0e-9f);
      assert_near(ekf.accel_bias[state], before.accel_bias[state], 1.0e-9f);
    }

  assert_covariance_positive_definite(&ekf);
}

/* Validation follows source selection. A position-only fusion must not be
 * rejected because yaw is unavailable, and yaw-only must not create or keep
 * a horizontal-position datum alive.
 */

static void test_extnav_validates_only_requested_observations(void)
{
  struct ekf_core_s ekf;
  struct ekf_extnav_sample_s s;
  uint64_t timestamp;

  extnav_align(&ekf, &timestamp);
  s = extnav_at(NAN, NAN, 0.3f);
  assert(ekf_core_fuse_extnav(&ekf, &s, NAN, NAN, 0.05f, 5.0f,
                              false, true) == -2);
  assert(!ekf.extnav_datum_set);
  assert(ekf.extnav_accept_count == 0);

  extnav_align(&ekf, &timestamp);
  s = extnav_at(2.0f, -1.0f, NAN);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, NAN, NAN,
                              true, false) == -2);
  assert(ekf.extnav_datum_set);
  assert(ekf.extnav_accept_count == 1);
}

/* Accepted yaw and rejected x/y can arrive in the same pose. Yaw must not
 * clear the horizontal rejection run or refresh the horizontal aiding time.
 */

static void test_extnav_yaw_cannot_hide_position_rejection(void)
{
  struct ekf_core_s ekf;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  uint64_t timestamp;
  uint64_t last_position_accept;

  extnav_align(&ekf, &timestamp);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, true) == -2);
  last_position_accept = ekf.last_extnav_timestamp;

  ekf.last_timestamp_sample += 10000ull;
  s.x = 100.0f;

  /* Overall return is accepted because yaw passed; position bookkeeping
   * still records its own rejection.
   */

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, true) == 1);
  assert(ekf.extnav_consecutive_rejects == 1);
  assert(ekf.extnav_reject_count == 1);
  assert(ekf.last_extnav_timestamp == last_position_accept);
}

/* Regression for the vehicle failure mode: smooth acceleration with a good
 * horizontal position source must not pull roll/pitch away from gravity.
 *
 * Smooth acceleration used to be classified as "low dynamics" because its
 * IMU variance and one-g norm were small. At the same time, the position
 * innovation had unrestricted attitude gain with the wrong propagated cross-
 * covariance sign. Both paths could therefore reinforce an impossible tilt.
 */

static void test_moving_extnav_does_not_corrupt_tilt(void)
{
  struct ekf_core_s ekf;
  struct ekf_extnav_sample_s pose = extnav_at(0.0f, 0.0f, 0.0f);
  struct ekf_imu_sample_s imu;
  const float accel[3] = {2.0f, 0.0f, TEST_GRAVITY};
  const float gyro[3] = {0.0f, 0.0f, 0.0f};
  float euler[3];
  uint64_t timestamp;
  unsigned i;

  extnav_align(&ekf, &timestamp);
  ekf_core_set_position_hold(&ekf, 2.0f);
  assert(ekf_core_fuse_extnav(&ekf, &pose, 0.10f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);

  for (i = 1; i <= 1200; i++)
    {
      float elapsed = (float)i * TEST_DT;

      timestamp += TEST_DT_US;
      make_sample(&imu, timestamp, accel, gyro);
      assert(ekf_core_process(&ekf, &imu) == EKF_PROCESS_PREDICTED);

      if ((i % 4u) == 0u)
        {
          pose.x = 0.5f * 2.0f * elapsed * elapsed;
          assert(ekf_core_fuse_extnav(&ekf, &pose, 0.10f, 5.0f,
                                      0.05f, 5.0f, true, false) == 1);
        }
    }

  ekf_core_euler(&ekf, euler);
  assert_near(euler[0], 0.0f, 0.02f);
  assert_near(euler[1], 0.0f, 0.05f);
  assert(ekf.velocity[0] > 5.0f);
  assert(!ekf.low_dynamics);
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

static void test_extnav_timestamp_jitter_becomes_motion_noise(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);

  extnav_align(&ekf, &timestamp);
  ekf.velocity[0] = 10.0f;
  s.time_sigma = 0.002f;

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.01f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert_near(ekf.last_extnav_noise, hypotf(0.01f, 0.02f), 1.0e-6f);
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

/* A source that keeps talking and keeps disagreeing must NOT be given a
 * datum, however long it goes on.
 *
 * This is the inverse of what this test used to assert, and the change is
 * the reason for it. Re-datuming on disagreement is what turned a companion
 * publishing a frozen position into a diverged filter: the reset snapped
 * position back to the stale value, the strapdown kept integrating real
 * motion, and the only way left to reconcile them was to grow accel bias
 * until it hit its limit - taking attitude with it, because accel bias
 * couples straight into the gravity reference.
 */

static void test_extnav_disagreement_never_redatums(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  unsigned i;

  extnav_align(&ekf, &timestamp);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);

  for (i = 0; i < 40; i++)
    {
      ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);
    }

  assert(ekf.extnav_datum_count == 1);

  /* Now it starts lying, and never stops. */

  s.x = 500.0f;

  for (i = 0; i < EKF_EXTNAV_REJECT_RUN_MAX * 4; i++)
    {
      int result = ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                                        true, false);

      /* Rejected every time. Never -2, which would mean a new datum. */

      assert(result == 0);
    }

  assert(ekf.extnav_datum_count == 1);

  /* And the filter has not moved to where the source claims. */

  assert(fabsf(ekf.position[0]) < 1.0f);
}

/* Silence is different from disagreement, and only silence earns a datum.
 *
 * A source that stopped and came back may legitimately have a new origin.
 * This is ArduPilot's posTimeout, measured from the last pose that PASSED.
 */

static void test_extnav_dropout_does_redatum(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  unsigned i;

  extnav_align(&ekf, &timestamp);
  ekf_core_set_extnav_config(&ekf, 1000000u);
  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);

  for (i = 0; i < 40; i++)
    {
      ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);
    }

  assert(ekf.extnav_datum_count == 1);

  /* Nothing arrives for longer than the timeout, then it comes back
   * somewhere else entirely.
   */

  ekf.last_timestamp_sample = ekf.last_extnav_timestamp + 2000000ull;
  s.x = 500.0f;

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);
  assert(ekf.extnav_datum_count == 2);
  assert_near(ekf.position[0], 500.0f, 1.0e-3f);
}

/* Persistent disagreement condemns the source, and withdrawing
 * POSITION_HORIZ is what tells autonomy to stop.
 */

static void test_extnav_persistent_disagreement_withdraws_position(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  unsigned i;

  extnav_align(&ekf, &timestamp);
  ekf_core_set_extnav_config(&ekf, 1000000u);
  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

  for (i = 0; i < 60; i++)
    {
      ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);
    }

  assert(ekf.extnav_healthy);
  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_POSITION_HORIZ) != 0);

  /* The vehicle moves; the source insists it has not. Advance filter time so
   * the fault timer can expire.
   */

  s.x = 50.0f;

  for (i = 0; i < 400; i++)
    {
      ekf.last_timestamp_sample += 10000ull;
      ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);
    }

  assert(!ekf.extnav_healthy);
  assert(ekf.extnav_fault_count > 0);
  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_POSITION_HORIZ) == 0);

  /* Attitude survives. Losing position must not cost roll and pitch, or a
   * bad companion would take the whole solution with it.
   */

  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_ATTITUDE) != 0);
}

/* Horizontal position has zero accelerometer-bias gain at all times, not
 * only after disagreement activates the redundant inhibit guard. Seed a
 * strong bias/position cross-covariance on every update so an unmasked gain
 * would be visible immediately.
 */

static void test_extnav_disagreement_never_updates_accel_bias(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  float bias_before[3];
  unsigned i;
  int axis;
  bool fused_while_inhibited = false;

  extnav_align(&ekf, &timestamp);
  ekf_core_set_extnav_config(&ekf, 1000000u);

  for (i = 0; i < 60; i++)
    {
      ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);
    }

  assert(!ekf.extnav_bias_inhibited);
  ekf.accel_bias[0] = 0.0f;
  ekf.accel_bias[1] = 0.0f;
  ekf.accel_bias[2] = 0.0f;

  memcpy(bias_before, ekf.accel_bias, sizeof(bias_before));

  for (i = 0; i < 300; i++)
    {
      int result;

      ekf.last_timestamp_sample += 10000ull;
      ekf.position[0] += 0.004f;

      for (axis = 0; axis < 3; axis++)
        {
          ekf.covariance[EKF_P_INDEX(12 + axis, 6)] = 0.05f;
          ekf.covariance[EKF_P_INDEX(6, 12 + axis)] = 0.05f;
        }

      result = ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                                    true, false);

      if (ekf.extnav_bias_inhibited && result == 1)
        {
          fused_while_inhibited = true;
        }

      for (axis = 0; axis < 3; axis++)
        {
          assert_near(ekf.accel_bias[axis], bias_before[axis], 1.0e-9f);
        }
    }

  /* The window this test exists for actually happened. */

  assert(fused_while_inhibited);
  assert(ekf.extnav_inhibit_count > 0);

  /* The redundant disagreement inhibit also reached accepted updates. */

  assert(ekf.inhibit_applied_count > 0);
}

/* A condemned source is not fused even by a pose that would sail through the
 * gate. Health is a separate verdict from the per-pose test, and it has to
 * outrank it - otherwise a lying source that happens to drift back past the
 * filter's position gets believed again on the strength of one agreement.
 */

static void test_unhealthy_source_is_not_fused_even_when_plausible(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  unsigned i;

  extnav_align(&ekf, &timestamp);
  ekf_core_set_extnav_config(&ekf, 1000000u);
  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

  s.x = 50.0f;

  for (i = 0; i < 400; i++)
    {
      ekf.last_timestamp_sample += 10000ull;
      ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);
    }

  assert(!ekf.extnav_healthy);

  /* Now hand it a pose exactly where the filter already thinks it is. The
   * per-pose gate would pass this instantly.
   */

  s.x = ekf.position[0];
  s.y = ekf.position[1];
  ekf.last_timestamp_sample += 10000ull;

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == 0);
}

/* Health withdraws POSITION_HORIZ on its own, not merely as a side effect of
 * the rejection counter also being high.
 */

static void test_health_alone_withdraws_position(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  unsigned i;

  extnav_align(&ekf, &timestamp);
  ekf_core_set_extnav_config(&ekf, 1000000u);

  for (i = 0; i < 60; i++)
    {
      ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);
    }

  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_POSITION_HORIZ) != 0);

  /* Condemn the source without letting the rejection counter be the reason:
   * clear it, so only health can withdraw the claim.
   */

  ekf.extnav_healthy = false;
  ekf.extnav_consecutive_rejects = 0;

  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_POSITION_HORIZ) == 0);
  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_ATTITUDE) != 0);
}

/* The inhibit mask itself, tested on the mechanism rather than through a
 * scenario.
 *
 * A position measurement has no direct coupling to accel bias - h is 1 on
 * the position row and 0 everywhere else - so the bias only moves through
 * the P[bias][position] cross-covariance that strapdown propagation builds
 * up. That cross term is seeded explicitly here, because without it the
 * bias gain is zero and the test would pass with the guard deleted.
 */

static void test_inhibit_mask_freezes_the_states_it_names(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  float h[EKF_STATE_DIM];
  float moved[3];
  float frozen[3];
  int axis;

  memset(h, 0, sizeof(h));
  h[6] = 1.0f;

  /* Free to learn. */

  extnav_align(&ekf, &timestamp);

  for (axis = 0; axis < 3; axis++)
    {
      ekf.covariance[EKF_P_INDEX(12 + axis, 6)] = 0.05f;
      ekf.covariance[EKF_P_INDEX(6, 12 + axis)] = 0.05f;
      ekf.accel_bias[axis] = 0.0f;
    }

  ekf.inhibit_mask = 0;
  assert(ekf_core_test_update_1d(&ekf, h, 1.0f, 0.01f, 1.0e6f, NULL) > 0);
  memcpy(moved, ekf.accel_bias, sizeof(moved));

  /* The cross term has to actually move it, or the comparison below proves
   * nothing.
   */

  assert(fabsf(moved[0]) > 1.0e-4f);

  /* Same setup, same update, bias inhibited. */

  extnav_align(&ekf, &timestamp);

  for (axis = 0; axis < 3; axis++)
    {
      ekf.covariance[EKF_P_INDEX(12 + axis, 6)] = 0.05f;
      ekf.covariance[EKF_P_INDEX(6, 12 + axis)] = 0.05f;
      ekf.accel_bias[axis] = 0.0f;
    }

  ekf.inhibit_mask = EKF_INHIBIT_ACCEL_BIAS;
  assert(ekf_core_test_update_1d(&ekf, h, 1.0f, 0.01f, 1.0e6f, NULL) > 0);
  memcpy(frozen, ekf.accel_bias, sizeof(frozen));

  for (axis = 0; axis < 3; axis++)
    {
      assert_near(frozen[axis], 0.0f, 1.0e-9f);
    }

  /* And the gyro bias mask names different states. */

  extnav_align(&ekf, &timestamp);

  for (axis = 0; axis < 3; axis++)
    {
      ekf.covariance[EKF_P_INDEX(9 + axis, 6)] = 0.05f;
      ekf.covariance[EKF_P_INDEX(6, 9 + axis)] = 0.05f;
      ekf.gyro_bias[axis] = 0.0f;
    }

  ekf.inhibit_mask = EKF_INHIBIT_GYRO_BIAS;
  assert(ekf_core_test_update_1d(&ekf, h, 1.0f, 0.01f, 1.0e6f, NULL) > 0);

  for (axis = 0; axis < 3; axis++)
    {
      assert_near(ekf.gyro_bias[axis], 0.0f, 1.0e-9f);
    }
}

/* The comparison metric is YAW-FREE. This is the property the whole
 * three-lane monitor rests on: EKF2 and EKF3 pin yaw at zero while EKF1 does
 * not, so any metric that moved with yaw would report a fault the instant
 * the vehicle turned.
 */

static void quat_ypr(float yaw, float pitch, float roll, float q[4])
{
  float cy = cosf(0.5f * yaw), sy = sinf(0.5f * yaw);
  float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
  float cr = cosf(0.5f * roll), sr = sinf(0.5f * roll);

  q[0] = cy * cp * cr + sy * sp * sr;
  q[1] = cy * cp * sr - sy * sp * cr;
  q[2] = cy * sp * cr + sy * cp * sr;
  q[3] = sy * cp * cr - cy * sp * sr;
}

static void test_tilt_difference_ignores_yaw(void)
{
  float a[4];
  float b[4];
  float err[3];
  int i;

  /* Same tilt, wildly different yaw. Must read as zero disagreement at
   * every heading, or a turning vehicle would trip the monitor.
   */

  for (i = 0; i < 12; i++)
    {
      float yaw = (float)i * 0.5f - 3.0f;

      quat_ypr(yaw, 0.15f, -0.2f, a);
      quat_ypr(0.0f, 0.15f, -0.2f, b);

      assert_near(ekf_core_tilt_difference(a, b, err), 0.0f, 1.0e-4f);
      assert_near(err[0], 0.0f, 1.0e-4f);
      assert_near(err[1], 0.0f, 1.0e-4f);
    }
}

/* A real tilt disagreement is reported at its true magnitude, and is not
 * diluted by the lanes being at different headings.
 */

static void test_tilt_difference_measures_real_disagreement(void)
{
  float a[4];
  float b[4];
  float err[3];

  quat_ypr(0.0f, 0.0f, 0.0f, a);
  quat_ypr(0.0f, 0.0f, 0.1f, b);
  assert_near(ekf_core_tilt_difference(a, b, err), 0.1f, 1.0e-4f);

  /* Same 0.1 rad of roll error, but the lanes are 2 rad apart in yaw. */

  quat_ypr(2.0f, 0.0f, 0.0f, a);
  quat_ypr(0.0f, 0.0f, 0.1f, b);
  assert_near(ekf_core_tilt_difference(a, b, err), 0.1f, 1.0e-4f);
}

/* Roll and pitch disagreement land on the axes they belong to, and z is
 * never reported - gravity cannot see yaw.
 */

static void test_tilt_difference_axes(void)
{
  float a[4];
  float b[4];
  float err[3];

  quat_ypr(0.0f, 0.0f, 0.0f, a);
  quat_ypr(0.0f, 0.0f, 0.12f, b);
  ekf_core_tilt_difference(a, b, err);
  assert(fabsf(err[0]) > 0.1f);
  assert(fabsf(err[1]) < 1.0e-4f);
  assert_near(err[2], 0.0f, 1.0e-9f);

  quat_ypr(0.0f, 0.0f, 0.0f, a);
  quat_ypr(0.0f, 0.12f, 0.0f, b);
  ekf_core_tilt_difference(a, b, err);
  assert(fabsf(err[1]) > 0.1f);
  assert(fabsf(err[0]) < 1.0e-4f);
  assert_near(err[2], 0.0f, 1.0e-9f);
}

/* A small disagreement must still be measurable.
 *
 * acos(dot) cannot do this: near zero the dot product is 1 - a^2/2, and
 * float resolution around 1.0 puts the floor at about 5e-4 rad. A monitor
 * spends its whole life near zero and has to see a trend before it becomes a
 * fault, so the angle is taken as atan2(|cross|, dot) instead.
 */

static void test_tilt_difference_resolves_small_angles(void)
{
  float a[4];
  float b[4];

  quat_ypr(0.0f, 0.0f, 0.0f, a);
  quat_ypr(0.0f, 0.0f, 1.0e-4f, b);

  assert_near(ekf_core_tilt_difference(a, b, NULL), 1.0e-4f, 1.0e-6f);

  quat_ypr(0.0f, 0.0f, 0.0f, a);
  quat_ypr(0.0f, 1.0e-5f, 0.0f, b);
  assert(ekf_core_tilt_difference(a, b, NULL) > 5.0e-6f);
}

/* The z component is never reported, even when the cross product has a real
 * one. Roll on one lane against pitch on the other gives cross_z = 0.0395 -
 * plenty to notice if it leaked through - and it means nothing, because
 * gravity carries no yaw information.
 */

static void test_tilt_difference_never_reports_yaw(void)
{
  float a[4];
  float b[4];
  float err[3];

  quat_ypr(0.0f, 0.0f, 0.2f, a);      /* pure roll  */
  quat_ypr(0.0f, 0.2f, 0.0f, b);      /* pure pitch */

  ekf_core_tilt_difference(a, b, err);

  assert(fabsf(err[0]) > 0.1f);
  assert(fabsf(err[1]) > 0.1f);
  assert_near(err[2], 0.0f, 1.0e-9f);
}

/* up_in_body must be the third ROW of the rotation, not the third column.
 * The column is where body z points in the nav frame, which is a different
 * vector and is NOT yaw-free - using it would silently reintroduce the
 * dependence this whole design removes.
 */

static void test_up_in_body_is_yaw_free(void)
{
  float q[4];
  float up_a[3];
  float up_b[3];

  quat_ypr(0.0f, 0.3f, 0.0f, q);
  ekf_core_up_in_body(q, up_a);

  quat_ypr(1.7f, 0.3f, 0.0f, q);
  ekf_core_up_in_body(q, up_b);

  assert_near(up_a[0], up_b[0], 1.0e-5f);
  assert_near(up_a[1], up_b[1], 1.0e-5f);
  assert_near(up_a[2], up_b[2], 1.0e-5f);

  /* Level: up is straight up the body z axis. */

  quat_ypr(0.9f, 0.0f, 0.0f, q);
  ekf_core_up_in_body(q, up_a);
  assert_near(up_a[0], 0.0f, 1.0e-5f);
  assert_near(up_a[1], 0.0f, 1.0e-5f);
  assert_near(up_a[2], 1.0f, 1.0e-5f);
}

/* The height bound holds the vertical state, and says so in the covariance.
 *
 * Clamping the number alone would be a lie: the filter would go on reporting
 * a confident height while it was being held in place by hand, and anything
 * reading position_variance to decide whether to trust the solution would be
 * misled by exactly the fault the clamp exists for.
 */

static void test_height_limit_clamps_and_admits_it(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);

  extnav_align(&ekf, &timestamp);
  ekf_core_set_height_limit(&ekf, 2.0f);

  /* The first pose only establishes the datum and returns before any
   * update runs, so nothing is constrained on that call. Poking the state
   * before it would be testing the datum path, not the bound.
   */

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);

  ekf.covariance[EKF_P_INDEX(8, 8)] = 0.01f;

  /* Inside the bound: untouched, and no admission made. */

  ekf.position[2] = 1.5f;
  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);
  assert_near(ekf.position[2], 1.5f, 1.0e-6f);
  assert(ekf.height_clamp_count == 0);

  /* Outside: clamped to the bound, and the variance floored. */

  ekf.position[2] = 9.0f;
  ekf.velocity[2] = 3.0f;
  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

  assert_near(ekf.position[2], 2.0f, 1.0e-6f);
  assert(ekf.height_clamp_count > 0);
  assert(ekf.covariance[EKF_P_INDEX(8, 8)] >= EKF_HEIGHT_LIMIT_VAR);

  /* Velocity still pushing outwards is stopped, or the clamp would be
   * re-violated on the very next step and achieve nothing.
   */

  assert_near(ekf.velocity[2], 0.0f, 1.0e-6f);
}

/* Both signs. A bound that only holds upwards is half a bound. */

static void test_height_limit_is_symmetric(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);

  extnav_align(&ekf, &timestamp);
  ekf_core_set_height_limit(&ekf, 2.0f);

  /* Establish the datum first: that call returns before any update. */

  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

  ekf.position[2] = -9.0f;
  ekf.velocity[2] = -3.0f;
  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

  assert_near(ekf.position[2], -2.0f, 1.0e-6f);
  assert_near(ekf.velocity[2], 0.0f, 1.0e-6f);
}

/* Velocity bringing the vehicle BACK inside must not be zeroed. A car at the
 * bound on a slope may legitimately be recovering, and stopping that would
 * fight the recovery rather than the fault.
 */

static void test_height_limit_allows_recovery(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);

  extnav_align(&ekf, &timestamp);
  ekf_core_set_height_limit(&ekf, 2.0f);

  /* Establish the datum first: that call returns before any update. */

  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

  ekf.position[2] = 9.0f;
  ekf.velocity[2] = -3.0f;            /* heading back down */
  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

  assert_near(ekf.position[2], 2.0f, 1.0e-6f);
  assert_near(ekf.velocity[2], -3.0f, 1.0e-6f);
}

/* Zero disables it, which is what a filter with no idea what vehicle it is
 * flying should do.
 */

static void test_height_limit_zero_disables(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);

  extnav_align(&ekf, &timestamp);
  ekf_core_set_height_limit(&ekf, 0.0f);

  /* Establish the datum first: that call returns before any update. */

  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

  ekf.position[2] = 900.0f;
  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

  assert_near(ekf.position[2], 900.0f, 1.0e-3f);
  assert(ekf.height_clamp_count == 0);

  /* And a nonsense limit is refused rather than believed. */

  ekf_core_set_height_limit(&ekf, -5.0f);
  assert(ekf.height_limit == 0.0f);
}

/* With no fix, position is bounded rather than integrated.
 *
 * Free dead reckoning on an IMU leaves quadratically and never returns, so
 * the useful property is not accuracy - the answer is known to be wrong -
 * but that the error stays finite and recoverable.
 */

static void test_position_held_when_unaided(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  unsigned i;

  extnav_align(&ekf, &timestamp);
  ekf_core_set_extnav_config(&ekf, 1000000u);
  ekf_core_set_position_hold(&ekf, 2.0f);

  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

  for (i = 0; i < 40; i++)
    {
      ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);
    }

  assert(ekf_core_position_aided(&ekf));

  /* Evaluated while aided: it must decline to engage, and must not freeze
   * accel bias. A hold that engaged whenever it was asked would bound a
   * perfectly good position and stop the bias ever being learnt.
   */

  ekf.inhibit_mask = 0;
  constrain_position_for_test(&ekf);
  assert(!ekf.position_holding);
  assert((ekf.inhibit_mask & EKF_INHIBIT_ACCEL_BIAS) == 0);

  /* The fix stops arriving. Everything past the timeout is unaided. */

  ekf.last_timestamp_sample += 5000000ull;
  assert(!ekf_core_position_aided(&ekf));

  /* Evaluated the moment aiding is lost, which is what latches the last
   * good position. It runs every sample in the real loop, so the latch is
   * always taken before drift can accumulate - poking the state first would
   * latch the runaway and test nothing.
   */

  ekf.inhibit_mask = 0;
  constrain_position_for_test(&ekf);
  assert(ekf.position_holding);

  /* Accel bias is unobservable without a fix, and it is the state whose
   * corruption takes attitude with it. Frozen for the duration.
   */

  assert((ekf.inhibit_mask & EKF_INHIBIT_ACCEL_BIAS) != 0);

  /* Now the strapdown tries to run away with it. */

  ekf.position[0] = 500.0f;
  ekf.position[1] = -300.0f;
  ekf.velocity[0] = 40.0f;
  constrain_position_for_test(&ekf);

  assert(ekf.position_holding);
  assert(fabsf(ekf.position[0]) <= 2.0f + 1.0e-3f);
  assert(fabsf(ekf.position[1]) <= 2.0f + 1.0e-3f);

  /* Outward velocity stopped, so the bound is not re-violated every step. */

  assert_near(ekf.velocity[0], 0.0f, 1.0e-6f);

  /* And the covariance admits the position is held, not known. */

  assert(ekf.covariance[EKF_P_INDEX(6, 6)] >= EKF_POSITION_HOLD_VAR);
}

/* Recovery is a SNAP, not a convergence.
 *
 * A position bounded around an old fix can be far from a returning source.
 * Waiting for agreement would lock the filter out forever; the fix is simply
 * adopted.
 */

static void test_position_snaps_back_on_recovery(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  unsigned i;

  extnav_align(&ekf, &timestamp);
  ekf_core_set_extnav_config(&ekf, 1000000u);
  ekf_core_set_position_hold(&ekf, 2.0f);
  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

  for (i = 0; i < 40; i++)
    {
      ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);
    }

  /* Lose the fix, and let the hold engage. */

  ekf.last_timestamp_sample += 5000000ull;
  constrain_position_for_test(&ekf);
  ekf.position[0] = 500.0f;
  constrain_position_for_test(&ekf);
  assert(ekf.position_holding);

  /* It comes back, somewhere else entirely - which is the normal case, since
   * the vehicle has been driving the whole time.
   */

  s.x = 137.0f;
  s.y = -42.0f;

  assert(ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f,
                              true, false) == -2);

  assert_near(ekf.position[0], 137.0f, 1.0e-3f);
  assert_near(ekf.position[1], -42.0f, 1.0e-3f);
  assert(!ekf.position_holding);
  assert(ekf.position_snap_count > 0);

  /* Health is restored with the datum, or the source would be condemned on
   * arrival by a ratio it had no chance to earn down.
   */

  assert(ekf.extnav_healthy);
}

/* A source that keeps arriving and keeps disagreeing is FOLLOWED, not
 * refused - and this is a deliberate reversal worth being explicit about.
 *
 * Before the hold existed, a disagreeing source was condemned and ignored,
 * because adopting it meant snapping position onto a value the strapdown
 * was busy contradicting, and the only way to reconcile the two was
 * accelerometer bias - which corrupted attitude.
 *
 * With position bounded and horizontal fusion unable to touch bias or tilt,
 * the filter cannot corrupt attitude to reconcile the two. On a vehicle
 * whose only absolute reference IS the external fix, following it is right:
 * dead reckoning is the thing that cannot be trusted, not the fix.
 *
 * So the contract this checks is not "the bad source is rejected" but "the
 * position never runs away, and ends up where the source says".
 */

static void test_disagreeing_source_is_followed_not_chased(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  float worst = 0.0f;
  unsigned i;

  extnav_align(&ekf, &timestamp);
  ekf_core_set_extnav_config(&ekf, 1000000u);
  ekf_core_set_position_hold(&ekf, 2.0f);
  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

  for (i = 0; i < 40; i++)
    {
      ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);
    }

  /* The source jumps far away and stays there, while the strapdown keeps
   * trying to drag position off in the other direction.
   */

  s.x = 50.0f;

  for (i = 0; i < 400; i++)
    {
      float excursion;

      ekf.last_timestamp_sample += 10000ull;
      ekf.position[0] += 0.5f;          /* the strapdown running away */

      constrain_position_for_test(&ekf);
      ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

      excursion = fabsf(ekf.position[0]);

      if (excursion > worst)
        {
          worst = excursion;
        }
    }

  /* Bounded. The strapdown was pushing 0.5 m per step for 400 steps, so
   * free dead reckoning would have ended 200 m out; what actually happens
   * is a drift of at most one aiding timeout, then a snap onto the source.
   */

  assert(worst < 70.0f);

  /* And it stays near where the source says, not where the IMU thought.
   *
   * The result is a SAWTOOTH, not convergence: position drifts for one
   * aiding timeout, snaps back onto the source, and drifts again. The
   * amplitude is EK3_EXT_TIMEOUT multiplied by the drift rate - which is
   * the number to reach for if the excursions matter, because shortening
   * the timeout shortens them directly.
   */

  assert(fabsf(ekf.position[0] - 50.0f) < 12.0f);
  assert(ekf.position_snap_count > 1);
}

/* Measurement availability must not change the inertial process model.
 *
 * ArduPilot continues propagating velocity and position when aiding is lost;
 * it changes which innovations may be fused and marks the unaided solution
 * invalid. Freezing the nominal navigation states while covariance still
 * propagates F gives the two different dynamics. The optional car hold is a
 * bound around propagation, not a replacement for it.
 */

static void test_unaided_states_still_propagate(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_extnav_sample_s s = extnav_at(0.0f, 0.0f, 0.0f);
  struct ekf_imu_sample_s imu;
  float held_position[3];
  float held_velocity[3];
  unsigned i;

  extnav_align(&ekf, &timestamp);
  ekf_core_set_extnav_config(&ekf, 1000000u);
  ekf_core_set_position_hold(&ekf, 10.0f);
  ekf_core_fuse_extnav(&ekf, &s, 0.1f, 5.0f, 0.05f, 5.0f, true, false);

  assert(ekf_core_observability(&ekf) == EKF_OBS_POSITION);

  /* Lose the fix. */

  ekf.last_timestamp_sample += 5000000ull;
  assert(ekf_core_observability(&ekf) == EKF_OBS_ATTITUDE);

  /* Carry the sample clock forward with the filter's, or the samples read
   * as going backwards and are refused before they reach the strapdown -
   * which would leave the states unchanged for the wrong reason entirely.
   */

  timestamp = ekf.last_timestamp_sample;

  memcpy(held_position, ekf.position, sizeof(held_position));
  memcpy(held_velocity, ekf.velocity, sizeof(held_velocity));

  /* Feed a full second of samples with a large lateral specific force. It is
   * intentionally outside the low-dynamics gravity gate, leaving this as a
   * process-model test rather than an attitude pseudo-measurement test.
   */

  for (i = 0; i < 400; i++)
    {
      const float accel[3] = {6.0f, 0.0f, TEST_GRAVITY};
      const float gyro[3] = {0.0f, 0.0f, 0.0f};

      timestamp += TEST_DT_US;
      make_sample(&imu, timestamp, accel, gyro);
      ekf_core_process(&ekf, &imu);
    }

  assert(ekf.velocity[0] - held_velocity[0] > 5.0f);
  assert(ekf.position[0] - held_position[0] > 2.5f);
  assert(ekf.position[0] - held_position[0] < 10.0f);

  /* And the covariance says so, rather than reporting the confidence it had
   * when the fix was lost.
   */

  assert(ekf.covariance[EKF_P_INDEX(6, 6)] >= EKF_POSITION_HOLD_VAR);
  assert(ekf.covariance[EKF_P_INDEX(3, 3)] >= EKF_VELOCITY_HOLD_VAR);
}

/* Attitude is still observable and must still track, or the monitor lanes
 * and everything downstream would freeze with it.
 */

static void test_attitude_still_propagates_when_unaided(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_imu_sample_s imu;
  float euler[3];
  unsigned i;

  extnav_align(&ekf, &timestamp);
  ekf_core_set_position_hold(&ekf, 2.0f);

  assert(ekf_core_observability(&ekf) == EKF_OBS_ATTITUDE);

  for (i = 0; i < 400; i++)
    {
      const float accel[3] = {0.0f, 0.0f, TEST_GRAVITY};
      const float gyro[3] = {0.0f, 0.0f, 0.5f};    /* yawing */

      timestamp += TEST_DT_US;
      make_sample(&imu, timestamp, accel, gyro);
      ekf_core_process(&ekf, &imu);
    }

  ekf_core_euler(&ekf, euler);
  assert(fabsf(euler[2]) > 0.2f);
}

/* Bias learning can be switched off, and the switch must reach BOTH update
 * paths.
 *
 * The 3D path is the one that matters: low_dynamics_updates observes
 * accelerometer bias directly through h[axis][12 + axis], so it is the
 * dominant way the bias is learned. An inhibit covering only the scalar
 * update - which is what existed - left the main path open, and every
 * earlier "bias is frozen" claim was true only of the path that mattered
 * least.
 */

static void test_bias_learning_can_be_frozen(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  struct ekf_imu_sample_s imu;
  float bias_before[3];
  unsigned i;
  int axis;

  /* Learning ON: the gravity update moves accel bias. */

  extnav_align(&ekf, &timestamp);
  ekf_core_set_bias_learning(&ekf, true, true);
  memset(ekf.accel_bias, 0, sizeof(ekf.accel_bias));

  for (i = 0; i < 600; i++)
    {
      const float accel[3] = {0.3f, 0.0f, TEST_GRAVITY};
      const float gyro[3] = {0.0f, 0.0f, 0.0f};

      timestamp += TEST_DT_US;
      make_sample(&imu, timestamp, accel, gyro);
      ekf_core_process(&ekf, &imu);
    }

  assert(fabsf(ekf.accel_bias[0]) > 1.0e-4f);

  /* Learning OFF: the same stimulus moves nothing. */

  extnav_align(&ekf, &timestamp);
  ekf_core_set_bias_learning(&ekf, false, false);
  memset(ekf.accel_bias, 0, sizeof(ekf.accel_bias));
  memset(ekf.gyro_bias, 0, sizeof(ekf.gyro_bias));
  memcpy(bias_before, ekf.accel_bias, sizeof(bias_before));

  for (i = 0; i < 600; i++)
    {
      const float accel[3] = {0.3f, 0.0f, TEST_GRAVITY};
      const float gyro[3] = {0.0f, 0.0f, 0.0f};

      timestamp += TEST_DT_US;
      make_sample(&imu, timestamp, accel, gyro);
      ekf_core_process(&ekf, &imu);
    }

  for (axis = 0; axis < 3; axis++)
    {
      assert_near(ekf.accel_bias[axis], bias_before[axis], 1.0e-9f);
      assert_near(ekf.gyro_bias[axis], 0.0f, 1.0e-9f);
    }

  assert(ekf.inhibit_applied_count > 0);
}

/* Freezing HOLDS the value rather than zeroing it - a converged bias is
 * worth keeping, and clearing one is `ekf3 reset`, an explicit action.
 */

static void test_freezing_holds_rather_than_zeroes(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;

  extnav_align(&ekf, &timestamp);
  ekf.accel_bias[2] = 0.25f;

  ekf_core_set_bias_learning(&ekf, true, false);
  assert_near(ekf.accel_bias[2], 0.25f, 1.0e-9f);

  /* And the two switches are independent. */

  assert((ekf.bias_learn_inhibit & EKF_INHIBIT_ACCEL_BIAS) != 0);
  assert((ekf.bias_learn_inhibit & EKF_INHIBIT_GYRO_BIAS) == 0);

  ekf_core_set_bias_learning(&ekf, false, true);
  assert((ekf.bias_learn_inhibit & EKF_INHIBIT_ACCEL_BIAS) == 0);
  assert((ekf.bias_learn_inhibit & EKF_INHIBIT_GYRO_BIAS) != 0);
}

/* Process noise is settable, and a nonsense value falls back to the
 * compiled default rather than being taken literally.
 *
 * Zero matters most: it would tell the filter its own propagation is
 * perfect, the covariance would stop growing, and every measurement would
 * eventually be gated out as the gain went to nothing. A parameter left
 * unset must not be able to do that.
 */

static void test_process_noise_is_settable_and_guarded(void)
{
  struct ekf_core_s ekf;

  ekf_core_init(&ekf);

  assert_near(ekf.gyro_noise, EKF_GYRO_NOISE, 1.0e-9f);
  assert_near(ekf.accel_noise, EKF_ACCEL_NOISE, 1.0e-9f);

  ekf_core_set_process_noise(&ekf, 0.02f, 0.5f, 0.001f, 0.02f);
  assert_near(ekf.gyro_noise, 0.02f, 1.0e-9f);
  assert_near(ekf.accel_noise, 0.5f, 1.0e-9f);
  assert_near(ekf.gyro_bias_rw, 0.001f, 1.0e-9f);
  assert_near(ekf.accel_bias_rw, 0.02f, 1.0e-9f);

  ekf_core_set_process_noise(&ekf, 0.0f, -1.0f, 0.0f, 0.0f);
  assert_near(ekf.gyro_noise, EKF_GYRO_NOISE, 1.0e-9f);
  assert_near(ekf.accel_noise, EKF_ACCEL_NOISE, 1.0e-9f);
  assert_near(ekf.gyro_bias_rw, EKF_GYRO_BIAS_RW, 1.0e-9f);
  assert_near(ekf.accel_bias_rw, EKF_ACCEL_BIAS_RW, 1.0e-9f);
}

/* More process noise must actually grow the covariance faster - the setter
 * storing a number is not the same as the propagation using it.
 */

static void test_process_noise_reaches_the_propagation(void)
{
  float grown[2];
  int mode;

  for (mode = 0; mode < 2; mode++)
    {
      struct ekf_core_s ekf;
      struct ekf_imu_sample_s imu;
      uint64_t timestamp = 1000000ull;
      const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
      const float accel[3] = {0.0f, 0.0f, TEST_GRAVITY};
      unsigned i;

      ekf_core_init(&ekf);
      initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
      ekf_core_set_process_noise(&ekf, 0.015f,
                                 mode ? 1.0f : 0.1f, 0.001f, 0.001f);

      for (i = 0; i < 200; i++)
        {
          timestamp += TEST_DT_US;
          make_sample(&imu, timestamp, accel, zero_gyro);
          ekf_core_process(&ekf, &imu);
        }

      grown[mode] = ekf.covariance[EKF_P_INDEX(3, 3)];
    }

  assert(grown[1] > grown[0]);
}

/* A zero-velocity update pulls velocity to zero and needs no calibration.
 *
 * This is the aid that bounds the drift an unaided inertial solution
 * accumulates. The vehicle below is stationary but the filter believes it is
 * moving, which is exactly the state a stop arrives in.
 */

static void test_zero_velocity_update_pulls_velocity_down(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  unsigned i;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);

  ekf.velocity[0] = 2.0f;
  ekf.velocity[1] = -1.0f;
  ekf.velocity[2] = 0.5f;

  for (i = 0; i < 50; i++)
    {
      assert(ekf_core_fuse_zero_velocity(&ekf, 0.05f, 5.0f) >= 0);
    }

  assert(fabsf(ekf.velocity[0]) < 0.05f);
  assert(fabsf(ekf.velocity[1]) < 0.05f);
  assert(fabsf(ekf.velocity[2]) < 0.05f);
  assert(ekf.zupt_accept_count > 0);
}

/* It must not move position or attitude.
 *
 * Stationary wheels do not always mean a stationary vehicle - a skid, a tow
 * or a jacked-up axle all read the same - so the one lie this measurement
 * can tell must stay out of roll and pitch. Gravity is the better tilt
 * reference at a standstill anyway, and the low-dynamics update already
 * takes it at exactly that moment.
 */

static void test_zero_velocity_update_leaves_attitude_and_position(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  float euler_before[3];
  float euler_after[3];
  float position_before[3];
  unsigned i;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.05f, -0.03f, zero_gyro);

  ekf.velocity[0] = 2.0f;
  ekf.position[0] = 17.0f;
  ekf.position[1] = -4.0f;

  /* Give attitude and position a path to move through, or this passes with
   * the gain mask deleted - a state with no cross-covariance to the
   * measurement sits still whether it is permitted to move or not.
   */

  ekf.covariance[EKF_P_INDEX(1, 3)] = 0.02f;   /* pitch <-> vx */
  ekf.covariance[EKF_P_INDEX(3, 1)] = 0.02f;
  ekf.covariance[EKF_P_INDEX(0, 4)] = 0.02f;   /* roll  <-> vy */
  ekf.covariance[EKF_P_INDEX(4, 0)] = 0.02f;
  ekf.covariance[EKF_P_INDEX(6, 3)] = 0.02f;   /* posE  <-> vx */
  ekf.covariance[EKF_P_INDEX(3, 6)] = 0.02f;

  ekf_core_euler(&ekf, euler_before);
  memcpy(position_before, ekf.position, sizeof(position_before));

  for (i = 0; i < 50; i++)
    {
      ekf.covariance[EKF_P_INDEX(1, 3)] = 0.02f;
      ekf.covariance[EKF_P_INDEX(3, 1)] = 0.02f;
      ekf.covariance[EKF_P_INDEX(0, 4)] = 0.02f;
      ekf.covariance[EKF_P_INDEX(4, 0)] = 0.02f;
      ekf.covariance[EKF_P_INDEX(6, 3)] = 0.02f;
      ekf.covariance[EKF_P_INDEX(3, 6)] = 0.02f;

      ekf_core_fuse_zero_velocity(&ekf, 0.05f, 5.0f);
    }

  ekf_core_euler(&ekf, euler_after);

  assert_near(euler_after[0], euler_before[0], 1.0e-9f);
  assert_near(euler_after[1], euler_before[1], 1.0e-9f);
  assert_near(ekf.position[0], position_before[0], 1.0e-9f);
  assert_near(ekf.position[1], position_before[1], 1.0e-9f);
}

/* A recent stop makes velocity observable, which is the middle tier that
 * had no source until now.
 */

static void test_zero_velocity_raises_observability(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  ekf_core_set_extnav_config(&ekf, 1000000u);
  ekf_core_set_position_hold(&ekf, 2.0f);

  assert(ekf_core_observability(&ekf) == EKF_OBS_ATTITUDE);

  ekf_core_fuse_zero_velocity(&ekf, 0.05f, 5.0f);
  assert(ekf_core_observability(&ekf) == EKF_OBS_VELOCITY);

  /* And it goes stale like any other aid. */

  ekf.last_timestamp_sample += 5000000ull;
  assert(ekf_core_observability(&ekf) == EKF_OBS_ATTITUDE);
}

static void test_wheel_velocity_constrains_planar_motion(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float wheel_to_imu[2] = {0.0f, 0.0f};
  unsigned i;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
  ekf_core_set_wheel_config(&ekf, 200000u);

  for (i = 0; i < 20; i++)
    {
      assert(ekf_core_fuse_wheel_velocity(
        &ekf, 2.0f, 0.0f, wheel_to_imu, 0.2f, 0.3f, 5.0f) >= 0);
    }

  assert(ekf.velocity[0] > 1.8f);
  assert_near(ekf.velocity[1], 0.0f, 1.0e-6f);
  assert(ekf.wheel_accept_count > 0);
  assert(ekf_core_observability(&ekf) == EKF_OBS_VELOCITY);
  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_VELOCITY_HORIZ) != 0);
  assert((ekf_core_solution_status(&ekf) & EKF_SOLUTION_POSITION_HORIZ) == 0);
}

static void test_wheel_yaw_rate_compensates_imu_lever_arm(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float wheel_to_imu[2] = {0.5f, 0.0f};
  unsigned i;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);

  for (i = 0; i < 20; i++)
    {
      ekf_core_fuse_wheel_velocity(&ekf, 2.0f, 2.0f, wheel_to_imu,
                                   0.2f, 0.2f, 5.0f);
    }

  /* At yaw zero, wheel->IMU x=0.5 m and wz=2 rad/s adds +1 m/s left. */

  assert(ekf.velocity[0] > 1.8f);
  assert(ekf.velocity[1] > 0.8f);
}

static void test_wheel_velocity_rotates_into_navigation_xy(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float wheel_to_imu[2] = {0.0f, 0.0f};
  unsigned i;

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);

  /* At +90 degrees yaw, body forward is navigation +Y. */

  ekf.quaternion[0] = cosf(TEST_PI * 0.25f);
  ekf.quaternion[1] = 0.0f;
  ekf.quaternion[2] = 0.0f;
  ekf.quaternion[3] = sinf(TEST_PI * 0.25f);

  for (i = 0; i < 20; i++)
    {
      ekf_core_fuse_wheel_velocity(&ekf, 2.0f, 0.0f, wheel_to_imu,
                                   0.2f, 0.2f, 5.0f);
    }

  assert_near(ekf.velocity[0], 0.0f, 1.0e-5f);
  assert(ekf.velocity[1] > 1.8f);
}

static void test_wheel_velocity_masks_unmeasured_states(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float wheel_to_imu[2] = {0.0f, 0.0f};
  float quaternion[4];
  float position[3];
  float biases[6];

  ekf_core_init(&ekf);
  initialize_tilted(&ekf, &timestamp, 0.04f, -0.03f, zero_gyro);
  ekf.velocity[0] = 1.0f;
  ekf.velocity[2] = 0.7f;

  /* Explicit cross-covariance makes this test fail if the gain mask is
   * removed; diagonal covariance alone cannot move the unmeasured states.
   */

  ekf.covariance[EKF_P_INDEX(0, 3)] = 0.01f;
  ekf.covariance[EKF_P_INDEX(3, 0)] = 0.01f;
  ekf.covariance[EKF_P_INDEX(6, 3)] = 0.01f;
  ekf.covariance[EKF_P_INDEX(3, 6)] = 0.01f;
  ekf.covariance[EKF_P_INDEX(12, 3)] = 0.01f;
  ekf.covariance[EKF_P_INDEX(3, 12)] = 0.01f;
  memcpy(quaternion, ekf.quaternion, sizeof(quaternion));
  memcpy(position, ekf.position, sizeof(position));
  memcpy(biases, ekf.gyro_bias, sizeof(ekf.gyro_bias));
  memcpy(&biases[3], ekf.accel_bias, sizeof(ekf.accel_bias));

  assert(ekf_core_fuse_wheel_velocity(
    &ekf, 1.5f, 0.0f, wheel_to_imu, 0.2f, 0.3f, 5.0f) == 1);
  assert(memcmp(quaternion, ekf.quaternion, sizeof(quaternion)) == 0);
  assert(memcmp(position, ekf.position, sizeof(position)) == 0);
  assert_near(ekf.velocity[2], 0.7f, 1.0e-9f);
  assert(memcmp(biases, ekf.gyro_bias, sizeof(ekf.gyro_bias)) == 0);
  assert(memcmp(&biases[3], ekf.accel_bias, sizeof(ekf.accel_bias)) == 0);
}

/* A moving vehicle must keep a tilt reference.
 *
 * The standstill update does not run while driving, so without this roll and
 * pitch are pure gyro integration between stops. Half a degree of tilt is
 * 0.086 m/s^2 the filter reads as real acceleration, which is what ramps
 * velocity in dead reckoning and makes each position fix land as a large
 * correction that does nothing about the cause.
 */

static void test_tilt_reference_while_moving(void)
{
  float drift[2];
  int mode;

  for (mode = 0; mode < 2; mode++)
    {
      struct ekf_core_s ekf;
      struct ekf_imu_sample_s imu;
      uint64_t timestamp = 1000000ull;
      const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
      unsigned i;

      ekf_core_init(&ekf);
      initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f, zero_gyro);
      ekf_core_set_tilt_fusion_moving(&ekf, mode != 0);

      for (i = 0; i < 4000; i++)
        {
          const float accel[3] = {0.0f, 0.0f, TEST_GRAVITY};
          const float gyro[3] = {0.004f, 0.0f, 0.30f};

          timestamp += TEST_DT_US;
          make_sample(&imu, timestamp, accel, gyro);
          ekf_core_process(&ekf, &imu);
        }

      drift[mode] = sqrtf(ekf.velocity[0] * ekf.velocity[0] +
                          ekf.velocity[1] * ekf.velocity[1]);
    }

  /* The vehicle is coasting level throughout, so any velocity at all is
   * error. An order of magnitude is the claim; measured is nearly two.
   */

  assert(drift[1] < drift[0] * 0.1f);
  assert(drift[1] < 0.2f);
}

/* The tiers nest, which is the property the gating relies on: asking
 * "observability >= VELOCITY" has to be true whenever position is available.
 */

static void test_observability_tiers_nest(void)
{
  assert(EKF_OBS_ATTITUDE < EKF_OBS_VELOCITY);
  assert(EKF_OBS_VELOCITY < EKF_OBS_POSITION);
}

/* Zero restores free dead reckoning, for anything that is not a car. */

static void test_position_hold_zero_disables(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;

  extnav_align(&ekf, &timestamp);
  ekf_core_set_position_hold(&ekf, 0.0f);

  ekf.position[0] = 500.0f;
  constrain_position_for_test(&ekf);

  assert_near(ekf.position[0], 500.0f, 1.0e-3f);
  assert(!ekf.position_holding);
  assert(ekf_core_observability(&ekf) == EKF_OBS_ATTITUDE);
  assert((ekf_core_solution_status(&ekf) &
          EKF_SOLUTION_POSITION_HORIZ) == 0);
}

/* The bias limit is a safety bound, not a tuning value: 2.0 m/s^2 is enough
 * to hide 0.2 g of real acceleration.
 */

static void test_accel_bias_limit_is_tight(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp;

  /* The compiled ceilings are a safety bound, not tuning: 2.0 m/s^2 is
   * enough bias to hide a fifth of a g of real acceleration.
   */

  assert(EKF_ACCEL_BIAS_LIMIT <= 1.0f);
  assert(EKF_GYRO_BIAS_LIMIT <= 0.2f);

  /* A parameter may tighten the bound. */

  extnav_align(&ekf, &timestamp);
  ekf_core_set_bias_limits(&ekf, 0.05f, 0.4f);
  assert_near(ekf.accel_bias_limit, 0.4f, 1.0e-6f);
  assert_near(ekf.gyro_bias_limit, 0.05f, 1.0e-6f);

  /* It may NOT loosen it past the ceiling - beyond that the "bias" is big
   * enough to be hiding a real acceleration, and a parameter is not
   * evidence that it is not.
   */

  ekf_core_set_bias_limits(&ekf, 99.0f, 99.0f);
  assert_near(ekf.accel_bias_limit, EKF_ACCEL_BIAS_LIMIT, 1.0e-6f);
  assert_near(ekf.gyro_bias_limit, EKF_GYRO_BIAS_LIMIT, 1.0e-6f);

  /* Nor does zero remove the bound, which would be the worst reading of
   * "0 means unlimited" available here.
   */

  ekf_core_set_bias_limits(&ekf, 0.0f, 0.0f);
  assert_near(ekf.accel_bias_limit, EKF_ACCEL_BIAS_LIMIT, 1.0e-6f);
  assert_near(ekf.gyro_bias_limit, EKF_GYRO_BIAS_LIMIT, 1.0e-6f);

  /* And the tightened bound is the one actually enforced. */

  ekf_core_set_bias_limits(&ekf, 0.05f, 0.4f);
  ekf.accel_bias[2] = 5.0f;
  ekf.gyro_bias[0] = 5.0f;
  constrain_position_for_test(&ekf);
  ekf_core_test_update_1d(&ekf, (float[EKF_STATE_DIM]){0}, 0.0f, 1.0f,
                          1.0e6f, NULL);
  assert(fabsf(ekf.accel_bias[2]) <= 0.4f + 1.0e-4f);
  assert(fabsf(ekf.gyro_bias[0]) <= 0.05f + 1.0e-6f);
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
  test_startup_estimates_uncalibrated_gyro_bias();
  test_startup_requires_ten_continuous_stationary_seconds();
  test_zupt_requires_stationary_imu();
  test_zupt_short_window_recovers_after_braking();
  test_initialization_and_static_prediction();
  test_acceleration_audit_matches_strapdown();
  test_yaw_prediction();
  test_sculling_delta_uses_packet_start_attitude();
  test_low_dynamics_updates();
  test_covariance_attitude_velocity_sign();
  test_gravity_preserves_yaw_gauge();
  test_fault_resets();
  test_secondary_imu_instance_is_valid();
  test_update_1d_matches_3d();
  test_update_1d_gate_rejects_cleanly();
  test_update_1d_reduces_variance();
  test_extnav_first_pose_sets_the_datum();
  test_extnav_fuses_after_the_datum();
  test_extnav_position_masks_attitude_and_vertical_gain();
  test_extnav_validates_only_requested_observations();
  test_extnav_yaw_cannot_hide_position_rejection();
  test_moving_extnav_does_not_corrupt_tilt();
  test_extnav_noise_floor_wins();
  test_extnav_honours_a_worse_reported_sigma();
  test_extnav_timestamp_jitter_becomes_motion_noise();
  test_extnav_refuses_an_invalid_pose();
  test_extnav_disagreement_never_redatums();
  test_extnav_dropout_does_redatum();
  test_extnav_persistent_disagreement_withdraws_position();
  test_extnav_disagreement_never_updates_accel_bias();
  test_unhealthy_source_is_not_fused_even_when_plausible();
  test_health_alone_withdraws_position();
  test_inhibit_mask_freezes_the_states_it_names();
  test_tilt_difference_ignores_yaw();
  test_tilt_difference_measures_real_disagreement();
  test_tilt_difference_axes();
  test_tilt_difference_resolves_small_angles();
  test_tilt_difference_never_reports_yaw();
  test_up_in_body_is_yaw_free();
  test_position_held_when_unaided();
  test_position_snaps_back_on_recovery();
  test_disagreeing_source_is_followed_not_chased();
  test_bias_learning_can_be_frozen();
  test_freezing_holds_rather_than_zeroes();
  test_process_noise_is_settable_and_guarded();
  test_process_noise_reaches_the_propagation();
  test_zero_velocity_update_pulls_velocity_down();
  test_zero_velocity_update_leaves_attitude_and_position();
  test_zero_velocity_raises_observability();
  test_wheel_velocity_constrains_planar_motion();
  test_wheel_yaw_rate_compensates_imu_lever_arm();
  test_wheel_velocity_rotates_into_navigation_xy();
  test_wheel_velocity_masks_unmeasured_states();
  test_tilt_reference_while_moving();
  test_observability_tiers_nest();
  test_unaided_states_still_propagate();
  test_attitude_still_propagates_when_unaided();
  test_position_hold_zero_disables();
  test_height_limit_clamps_and_admits_it();
  test_height_limit_is_symmetric();
  test_height_limit_allows_recovery();
  test_height_limit_zero_disables();
  test_accel_bias_limit_is_tight();
  test_extnav_source_reset_forces_a_redatum();
  test_extnav_timeout_drops_horizontal_validity();
  test_extnav_datum_clears_on_restart();
  puts("ekf core tests: PASS");
  return 0;
}
