/****************************************************************************
 * tests/imu_integrator_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "imu_integrator.h"

static void near_value(float actual, float expected, float tolerance)
{
  assert(fabsf(actual - expected) <= tolerance);
}

static int add_sample(struct imu_integrator_s *integrator, uint64_t time,
                      float ax, float ay, float az,
                      float gx, float gy, float gz, uint8_t clipping,
                      struct imu_delta_output_s *output)
{
  float accel[3] = {ax, ay, az};
  float gyro[3] = {gx, gy, gz};

  return imu_integrator_add(integrator, time, accel, gyro, clipping, output);
}

static void test_constant_inputs_and_64_bit_time(void)
{
  struct imu_integrator_s integrator;
  struct imu_delta_output_s output;
  uint64_t start = (uint64_t)UINT32_MAX + 1000000ull;
  int i;

  imu_integrator_init(&integrator);

  for (i = 0; i <= 5; i++)
    {
      int result = add_sample(&integrator, start + (uint64_t)i * 500u,
                              1.0f, -2.0f, 9.0f,
                              0.0f, 0.0f, 2.0f, 0, &output);

      assert(result == (i == 5 ? 1 : 0));
    }

  assert(output.first_timestamp_us == start);
  assert(output.last_timestamp_us == start + 2500u);
  assert(output.samples == 5);
  near_value(output.delta_angle[0], 0.0f, 1.0e-7f);
  near_value(output.delta_angle[1], 0.0f, 1.0e-7f);
  near_value(output.delta_angle[2], 0.005f, 1.0e-7f);
  near_value(output.delta_angle_dt, 0.0025f, 1.0e-8f);
  near_value(output.delta_velocity_dt, 0.0025f, 1.0e-8f);

  /* Acceleration is rotated through the small yaw motion. The Z integral is
   * unaffected; XY are validated separately by the sculling test below.
   */

  near_value(output.delta_velocity[2], 0.0225f, 2.0e-7f);
  assert(integrator.packets == 1);
}

static void test_sculling_rotation(void)
{
  struct imu_integrator_s integrator;
  struct imu_delta_output_s output;
  const float rate = 100.0f;
  const float duration = 0.0025f;
  int i;

  imu_integrator_init(&integrator);

  for (i = 0; i <= 5; i++)
    {
      int result = add_sample(&integrator, 100000u + (uint64_t)i * 500u,
                              1.0f, 0.0f, 0.0f,
                              0.0f, 0.0f, rate, 0, &output);

      assert(result == (i == 5 ? 1 : 0));
    }

  /* Integral of a unit body-X vector rotated into the window-start frame. */

  near_value(output.delta_velocity[0], sinf(rate * duration) / rate,
             3.0e-6f);
  near_value(output.delta_velocity[1],
             (1.0f - cosf(rate * duration)) / rate, 3.0e-6f);
  near_value(output.delta_velocity[2], 0.0f, 1.0e-7f);
}

static void quaternion_multiply_double(const double a[4], const double b[4],
                                       double result[4])
{
  result[0] = a[0] * b[0] - a[1] * b[1] -
              a[2] * b[2] - a[3] * b[3];
  result[1] = a[0] * b[1] + a[1] * b[0] +
              a[2] * b[3] - a[3] * b[2];
  result[2] = a[0] * b[2] - a[1] * b[3] +
              a[2] * b[0] + a[3] * b[1];
  result[3] = a[0] * b[3] + a[1] * b[2] -
              a[2] * b[1] + a[3] * b[0];
}

static void reference_add(double quaternion[4], const float previous[3],
                          const float current[3], double dt)
{
  double vector[3];
  double magnitude;
  double scale;
  double increment[4];
  double next[4];
  int axis;

  for (axis = 0; axis < 3; axis++)
    {
      vector[axis] = 0.5 * (previous[axis] + current[axis]) * dt;
    }

  magnitude = sqrt(vector[0] * vector[0] + vector[1] * vector[1] +
                   vector[2] * vector[2]);
  scale = magnitude > 0.0 ? sin(0.5 * magnitude) / magnitude : 0.5;
  increment[0] = cos(0.5 * magnitude);
  increment[1] = vector[0] * scale;
  increment[2] = vector[1] * scale;
  increment[3] = vector[2] * scale;
  quaternion_multiply_double(quaternion, increment, next);

  for (axis = 0; axis < 4; axis++)
    {
      quaternion[axis] = next[axis];
    }
}

static void reference_rotation_vector(const double quaternion[4],
                                      float result[3])
{
  double sine_half = sqrt(quaternion[1] * quaternion[1] +
                          quaternion[2] * quaternion[2] +
                          quaternion[3] * quaternion[3]);
  double scale = sine_half > 0.0 ?
                 2.0 * atan2(sine_half, quaternion[0]) / sine_half : 2.0;

  result[0] = (float)(quaternion[1] * scale);
  result[1] = (float)(quaternion[2] * scale);
  result[2] = (float)(quaternion[3] * scale);
}

static void test_coning_sequence(void)
{
  static const float gyro[6][3] =
  {
    { 20.0f,   0.0f,  0.0f},
    {  0.0f,  20.0f,  0.0f},
    {-20.0f,   0.0f,  0.0f},
    {  0.0f, -20.0f,  0.0f},
    { 20.0f,   0.0f,  0.0f},
    {  0.0f,  20.0f,  0.0f}
  };

  struct imu_integrator_s integrator;
  struct imu_delta_output_s output;
  double reference_q[4] = {1.0, 0.0, 0.0, 0.0};
  float reference_vector[3];
  int i;

  imu_integrator_init(&integrator);

  for (i = 0; i <= 5; i++)
    {
      int result = add_sample(&integrator, 200000u + (uint64_t)i * 500u,
                              0.0f, 0.0f, 0.0f,
                              gyro[i][0], gyro[i][1], gyro[i][2],
                              i == 3 ? 2u : 0u, &output);

      assert(result == (i == 5 ? 1 : 0));

      if (i > 0)
        {
          reference_add(reference_q, gyro[i - 1], gyro[i], 0.0005);
        }
    }

  reference_rotation_vector(reference_q, reference_vector);
  near_value(output.delta_angle[0], reference_vector[0], 2.0e-7f);
  near_value(output.delta_angle[1], reference_vector[1], 2.0e-7f);
  near_value(output.delta_angle[2], reference_vector[2], 2.0e-7f);
  assert(fabsf(output.delta_angle[2]) > 1.0e-5f);
  assert(output.clipping == 2u);
}

static void test_timestamp_and_invalid_resets(void)
{
  struct imu_integrator_s integrator;
  struct imu_delta_output_s output;
  float good[3] = {0.0f, 0.0f, 0.0f};
  float bad[3] = {NAN, 0.0f, 0.0f};

  imu_integrator_init(&integrator);
  assert(imu_integrator_add(&integrator, 1000, good, good, 0, &output) == 0);
  assert(imu_integrator_add(&integrator, 1000, good, good, 0, &output) < 0);
  assert(integrator.duplicates == 1 && integrator.resets == 1);
  assert(imu_integrator_add(&integrator, 900, good, good, 0, &output) < 0);
  assert(integrator.backwards == 1 && integrator.resets == 2);
  assert(imu_integrator_add(&integrator, 2200, good, good, 0, &output) < 0);
  assert(integrator.gaps == 1 && integrator.resets == 3);
  assert(imu_integrator_add(&integrator, 2700, bad, good, 0, &output) < 0);
  assert(integrator.invalid == 1 && integrator.resets == 4);
  assert(!integrator.seeded);
  assert(imu_integrator_add(&integrator, 3200, good, good, 0, &output) == 0);
}

int main(void)
{
  test_constant_inputs_and_64_bit_time();
  test_sculling_rotation();
  test_coning_sequence();
  test_timestamp_and_invalid_resets();
  puts("imu integrator tests: PASS");
  return 0;
}
