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

int main(void)
{
  test_initialization_and_static_prediction();
  test_yaw_prediction();
  test_fault_resets();
  puts("ekf core tests: PASS");
  return 0;
}
