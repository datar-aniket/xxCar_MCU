/****************************************************************************
 * boards/fmuv6c/src/bmi055_timing.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __BOARDS_FMUV6C_SRC_BMI055_TIMING_H
#define __BOARDS_FMUV6C_SRC_BMI055_TIMING_H

#include <stdbool.h>
#include <stdint.h>

#define BMI055_TIME_FRAC_BITS          5
#define BMI055_TIME_NOMINAL_PERIOD_Q5  (500ull << BMI055_TIME_FRAC_BITS)
#define BMI055_TIME_HISTORY_SAMPLES    4

enum bmi055_timing_update_e
{
  BMI055_TIMING_NO_UPDATE = 0,
  BMI055_TIMING_PROVISIONAL,
  BMI055_TIMING_ACQUIRED,
  BMI055_TIMING_TRACKED
};

struct bmi055_timing_s
{
  uint64_t period_q5;
  uint64_t anchor_sample;
  uint64_t anchor_timestamp;
  uint64_t history_q5[BMI055_TIME_HISTORY_SAMPLES];
  uint64_t history_sum_q5;
  uint8_t  history_index;
  bool     acquired;
};

void bmi055_timing_init(struct bmi055_timing_s *timing);
void bmi055_timing_reset_anchor(struct bmi055_timing_s *timing);
enum bmi055_timing_update_e
bmi055_timing_update(struct bmi055_timing_s *timing,
                     uint64_t edge_timestamp, uint64_t edge_sample);

#endif /* __BOARDS_FMUV6C_SRC_BMI055_TIMING_H */
