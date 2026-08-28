/****************************************************************************
 * tests/imu_lpf_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "imu_lpf.h"

#define TEST_PI_F 3.14159265358979323846f

static void near_value(float actual, float expected, float tolerance)
{
  if (fabsf(actual - expected) > tolerance)
    {
      fprintf(stderr, "near_value: actual %.9g expected %.9g tolerance %.9g\n",
              (double)actual, (double)expected, (double)tolerance);
    }

  assert(fabsf(actual - expected) <= tolerance);
}

static void test_configuration_and_timestamp(void)
{
  struct imu_lpf3_s filter;

  assert(imu_lpf3_configure(&filter, 2000.0f, 100.0f));
  assert(filter.enabled);
  assert(filter.group_delay_us == 2232u);
  near_value(filter.b0 + filter.b1 + filter.b2,
             1.0f + filter.a1 + filter.a2, 1.0e-6f);
  assert(imu_lpf_compensate_timestamp(10000u,
                                      filter.group_delay_us) == 7768u);
  assert(imu_lpf_compensate_timestamp(100u,
                                      filter.group_delay_us) == 1u);

  assert(imu_lpf3_configure(&filter, 2000.0f, 0.0f));
  assert(!filter.enabled);
  assert(filter.group_delay_us == 0u);
  assert(!imu_lpf3_configure(&filter, 2000.0f, 900.0f));
  assert(!imu_lpf3_configure(&filter, 0.0f, 100.0f));
}

static void test_steady_state_reset(void)
{
  struct imu_lpf3_s filter;
  float value[3] = {1.25f, -3.5f, 9.80665f};
  int sample;

  assert(imu_lpf3_configure(&filter, 2000.0f, 100.0f));
  imu_lpf3_reset(&filter, value);

  for (sample = 0; sample < 100; sample++)
    {
      float output[3] = {value[0], value[1], value[2]};

      assert(imu_lpf3_apply(&filter, output));
      near_value(output[0], value[0], 2.0e-5f);
      near_value(output[1], value[1], 2.0e-5f);
      near_value(output[2], value[2], 2.0e-5f);
    }
}

static float sine_gain(float frequency_hz)
{
  struct imu_lpf3_s filter;
  float seed[3] = {0.0f, 0.0f, 0.0f};
  double input_energy = 0.0;
  double output_energy = 0.0;
  int sample;

  assert(imu_lpf3_configure(&filter, 2000.0f, 100.0f));
  imu_lpf3_reset(&filter, seed);

  for (sample = 0; sample < 8000; sample++)
    {
      float input = sinf(2.0f * TEST_PI_F * frequency_hz *
                         (float)sample / 2000.0f);
      float output[3] = {input, 0.0f, 0.0f};

      assert(imu_lpf3_apply(&filter, output));

      if (sample >= 4000)
        {
          input_energy += (double)input * input;
          output_energy += (double)output[0] * output[0];
        }
    }

  return (float)sqrt(output_energy / input_energy);
}

static void test_response_and_matching(void)
{
  struct imu_lpf3_s accel;
  struct imu_lpf3_s gyro;
  float seed[3] = {0.0f, 0.0f, 9.80665f};
  int sample;

  near_value(sine_gain(100.0f), 0.7071f, 0.002f);
  assert(sine_gain(200.0f) > 0.20f);
  assert(sine_gain(200.0f) < 0.26f);

  assert(imu_lpf3_configure(&accel, 2000.0f, 100.0f));
  assert(imu_lpf3_configure(&gyro, 2000.0f, 100.0f));
  imu_lpf3_reset(&accel, seed);
  imu_lpf3_reset(&gyro, seed);

  for (sample = 0; sample < 1000; sample++)
    {
      float accel_value[3];
      float gyro_value[3];

      accel_value[0] = sinf((float)sample * 0.031f);
      accel_value[1] = cosf((float)sample * 0.017f);
      accel_value[2] = 9.8f + sinf((float)sample * 0.13f);
      memcpy(gyro_value, accel_value, sizeof(gyro_value));

      assert(imu_lpf3_apply(&accel, accel_value));
      assert(imu_lpf3_apply(&gyro, gyro_value));
      assert(memcmp(accel_value, gyro_value, sizeof(accel_value)) == 0);
    }
}

static void test_invalid_input(void)
{
  struct imu_lpf3_s filter;
  float seed[3] = {0.0f, 0.0f, 0.0f};
  float invalid[3] = {NAN, 0.0f, 0.0f};

  assert(imu_lpf3_configure(&filter, 2000.0f, 100.0f));
  imu_lpf3_reset(&filter, seed);
  assert(!imu_lpf3_apply(&filter, invalid));
}

int main(void)
{
  test_configuration_and_timestamp();
  test_steady_state_reset();
  test_response_and_matching();
  test_invalid_input();
  puts("imu LPF tests: PASS");
  return 0;
}
