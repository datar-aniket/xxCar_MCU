/****************************************************************************
 * boards/fmuv6c/src/bmi055_timing.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <stdint.h>
#include <string.h>

#include "bmi055_timing.h"

#define BMI055_TIME_ACQUIRE_WINDOW_US  64000ull
#define BMI055_TIME_TRACK_WINDOW_US  1000000ull
#define BMI055_TIME_PERIOD_MIN_Q5  (450ull << BMI055_TIME_FRAC_BITS)
#define BMI055_TIME_PERIOD_MAX_Q5  (550ull << BMI055_TIME_FRAC_BITS)

void bmi055_timing_init(struct bmi055_timing_s *timing)
{
  int i;

  memset(timing, 0, sizeof(*timing));
  timing->period_q5 = BMI055_TIME_NOMINAL_PERIOD_Q5;

  for (i = 0; i < BMI055_TIME_HISTORY_SAMPLES; i++)
    {
      timing->history_q5[i] = BMI055_TIME_NOMINAL_PERIOD_Q5;
      timing->history_sum_q5 += BMI055_TIME_NOMINAL_PERIOD_Q5;
    }
}

void bmi055_timing_reset_anchor(struct bmi055_timing_s *timing)
{
  timing->anchor_sample = 0;
  timing->anchor_timestamp = 0;
}

static bool bmi055_timing_observe(uint64_t delta_us, uint64_t delta_samples,
                                  uint64_t *observed_q5)
{
  *observed_q5 = ((delta_us << BMI055_TIME_FRAC_BITS) +
                  delta_samples / 2) / delta_samples;

  return *observed_q5 >= BMI055_TIME_PERIOD_MIN_Q5 &&
         *observed_q5 <= BMI055_TIME_PERIOD_MAX_Q5;
}

static void bmi055_timing_seed_history(struct bmi055_timing_s *timing,
                                       uint64_t observed_q5)
{
  int i;

  timing->history_sum_q5 = 0;
  timing->history_index = 0;

  for (i = 0; i < BMI055_TIME_HISTORY_SAMPLES; i++)
    {
      timing->history_q5[i] = observed_q5;
      timing->history_sum_q5 += observed_q5;
    }

  timing->period_q5 = observed_q5;
  timing->acquired = true;
}

enum bmi055_timing_update_e
bmi055_timing_update(struct bmi055_timing_s *timing,
                     uint64_t edge_timestamp, uint64_t edge_sample)
{
  uint64_t delta_samples;
  uint64_t delta_us;
  uint64_t observed_q5;
  uint64_t window_us;
  bool valid;

  if (timing->anchor_timestamp == 0)
    {
      timing->anchor_timestamp = edge_timestamp;
      timing->anchor_sample = edge_sample;
      return BMI055_TIMING_NO_UPDATE;
    }

  if (edge_timestamp <= timing->anchor_timestamp ||
      edge_sample <= timing->anchor_sample)
    {
      timing->anchor_timestamp = edge_timestamp;
      timing->anchor_sample = edge_sample;
      return BMI055_TIMING_NO_UPDATE;
    }

  delta_us = edge_timestamp - timing->anchor_timestamp;
  delta_samples = edge_sample - timing->anchor_sample;
  valid = bmi055_timing_observe(delta_us, delta_samples, &observed_q5);
  window_us = timing->acquired ? BMI055_TIME_TRACK_WINDOW_US :
                                 BMI055_TIME_ACQUIRE_WINDOW_US;

  if (!timing->acquired && valid)
    {
      /* A provisional edge-to-edge estimate prevents the nominal 500 us
       * period from accumulating phase error while the longer acquisition
       * window rejects individual ISR-latency outliers.
       */

      timing->period_q5 = observed_q5;
    }

  if (delta_us < window_us)
    {
      return !timing->acquired && valid ? BMI055_TIMING_PROVISIONAL :
                                          BMI055_TIMING_NO_UPDATE;
    }

  timing->anchor_timestamp = edge_timestamp;
  timing->anchor_sample = edge_sample;

  if (!valid)
    {
      return BMI055_TIMING_NO_UPDATE;
    }

  if (!timing->acquired)
    {
      bmi055_timing_seed_history(timing, observed_q5);
      return BMI055_TIMING_ACQUIRED;
    }
  else
    {
      uint64_t replaced_q5 = timing->history_q5[timing->history_index];

      timing->history_sum_q5 -= replaced_q5;
      timing->history_q5[timing->history_index] = observed_q5;
      timing->history_sum_q5 += observed_q5;
      timing->history_index =
        (timing->history_index + 1) % BMI055_TIME_HISTORY_SAMPLES;
      timing->period_q5 =
        (timing->history_sum_q5 + BMI055_TIME_HISTORY_SAMPLES / 2) /
        BMI055_TIME_HISTORY_SAMPLES;
      return BMI055_TIMING_TRACKED;
    }
}
