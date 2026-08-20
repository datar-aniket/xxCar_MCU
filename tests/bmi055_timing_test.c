/****************************************************************************
 * tests/bmi055_timing_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "bmi055_timing.h"

static uint64_t edge_time(uint64_t origin_us, uint64_t samples,
                          uint64_t period_q5)
{
  return origin_us +
         ((samples * period_q5 +
           (1ull << (BMI055_TIME_FRAC_BITS - 1))) >>
          BMI055_TIME_FRAC_BITS);
}

static void test_fast_acquisition(void)
{
  const uint64_t actual_q5 = 15668; /* 489.625 us, about 2042.4 Hz */
  const uint64_t origin_us = 1000000;
  struct bmi055_timing_s timing;
  enum bmi055_timing_update_e update;
  bool saw_provisional = false;
  bool saw_acquired = false;
  int edge;
  int i;

  bmi055_timing_init(&timing);
  assert(timing.period_q5 == BMI055_TIME_NOMINAL_PERIOD_Q5);
  assert(!timing.acquired);

  update = bmi055_timing_update(&timing, origin_us, 7);
  assert(update == BMI055_TIMING_NO_UPDATE);

  for (edge = 1; edge <= 20; edge++)
    {
      uint64_t samples = (uint64_t)edge * 8;

      update = bmi055_timing_update(&timing,
                                    edge_time(origin_us, samples,
                                              actual_q5),
                                    7 + samples);
      if (update == BMI055_TIMING_PROVISIONAL)
        {
          saw_provisional = true;
          assert(timing.period_q5 >= actual_q5 - 1);
          assert(timing.period_q5 <= actual_q5 + 1);
        }
      else if (update == BMI055_TIMING_ACQUIRED)
        {
          saw_acquired = true;
          break;
        }
    }

  assert(saw_provisional);
  assert(saw_acquired);
  assert(timing.acquired);
  assert(timing.period_q5 >= actual_q5 - 1);
  assert(timing.period_q5 <= actual_q5 + 1);

  for (i = 0; i < BMI055_TIME_HISTORY_SAMPLES; i++)
    {
      assert(timing.history_q5[i] == timing.period_q5);
    }
}

static void test_reject_invalid_observation(void)
{
  struct bmi055_timing_s timing;
  enum bmi055_timing_update_e update;

  bmi055_timing_init(&timing);
  bmi055_timing_update(&timing, 1000000, 7);
  update = bmi055_timing_update(&timing, 1070000, 71);

  assert(update == BMI055_TIMING_NO_UPDATE);
  assert(!timing.acquired);
  assert(timing.period_q5 == BMI055_TIME_NOMINAL_PERIOD_Q5);
  assert(timing.anchor_timestamp == 1070000);
  assert(timing.anchor_sample == 71);
}

static void test_steady_tracking(void)
{
  const uint64_t acquired_q5 = 15668;
  const uint64_t tracked_q5 = 15680;
  const uint64_t delta_samples = 2048;
  struct bmi055_timing_s timing;
  enum bmi055_timing_update_e update;
  uint64_t expected_q5;
  int i;

  bmi055_timing_init(&timing);
  timing.acquired = true;
  timing.period_q5 = acquired_q5;
  timing.anchor_timestamp = 2000000;
  timing.anchor_sample = 1000;
  timing.history_sum_q5 = 0;

  for (i = 0; i < BMI055_TIME_HISTORY_SAMPLES; i++)
    {
      timing.history_q5[i] = acquired_q5;
      timing.history_sum_q5 += acquired_q5;
    }

  update = bmi055_timing_update(
    &timing,
    2000000 + (delta_samples * tracked_q5 >> BMI055_TIME_FRAC_BITS),
    1000 + delta_samples);

  expected_q5 =
    (acquired_q5 * (BMI055_TIME_HISTORY_SAMPLES - 1) + tracked_q5 +
     BMI055_TIME_HISTORY_SAMPLES / 2) /
    BMI055_TIME_HISTORY_SAMPLES;
  assert(update == BMI055_TIMING_TRACKED);
  assert(timing.period_q5 == expected_q5);

  bmi055_timing_reset_anchor(&timing);
  assert(timing.anchor_timestamp == 0);
  assert(timing.anchor_sample == 0);
  assert(timing.acquired);
  assert(timing.period_q5 == expected_q5);
}

int main(void)
{
  test_fast_acquisition();
  test_reject_invalid_observation();
  test_steady_tracking();
  puts("bmi055_timing_test: PASS");
  return 0;
}
