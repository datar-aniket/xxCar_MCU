/****************************************************************************
 * apps/companion/comp_clock.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_COMPANION_COMP_CLOCK_H
#define __APPS_COMPANION_COMP_CLOCK_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

/* UTC_c = a * TIM5 + b, represented about a recent anchor so epoch-sized
 * values are never multiplied. rate_ppb is (a - 1) * 1e9 during a finite
 * phase slew; afterward the continuous mapping uses base_rate_ppb.
 */

#define COMP_CLOCK_OBSERVATIONS       8
#define COMP_CLOCK_MIN_SYNC_US  1000000ull
#define COMP_CLOCK_MAX_RATE_PPB   5000000ll  /* +/-0.5% relative rate; warn >1000 ppm */
#define COMP_CLOCK_MAX_SLEW_PPB    200000ll  /* +/- 200 ppm phase slew */
#define COMP_CLOCK_ACQUIRE_WINDOW_US 10000000ll
#define COMP_CLOCK_ACQUIRE_STEP_US      50000ll
#define COMP_CLOCK_MAX_SLEW_TIME_US 30000000ll

enum comp_clock_sync_result_e
{
  COMP_CLOCK_SYNC_REJECTED = -1,
  COMP_CLOCK_SYNC_FIRST = 1,
  COMP_CLOCK_SYNC_UPDATED = 2,
  COMP_CLOCK_SYNC_ACQUIRED = 3
};

struct comp_clock_s
{
  uint64_t mono_anchor_us;
  int64_t  utc_anchor_us;
  int64_t  rate_ppb;              /* applied affine scale minus one */
  int64_t  base_rate_ppb;         /* regression estimate, no phase slew */
  uint64_t slew_duration_us;      /* then continue at base rate, not forever */

  uint64_t observation_mono_us[COMP_CLOCK_OBSERVATIONS];
  int64_t  observation_offset_us[COMP_CLOCK_OBSERVATIONS];
  uint8_t  observation_count;

  uint64_t last_sync_mono_us;
  int64_t  last_sync_offset_us;
  int64_t  last_phase_error_us;
  uint32_t sync_updates;
  uint32_t rejected_observations;
  bool     valid;
  bool     synchronized;
  bool     rate_acquired;
  int64_t  acquisition_step_us;
};

void comp_clock_init(FAR struct comp_clock_s *clock);
int64_t comp_clock_rate_at(FAR const struct comp_clock_s *clock,
                          uint64_t mono_us);
bool comp_clock_seed(FAR struct comp_clock_s *clock, uint64_t mono_us,
                     int64_t utc_us);
bool comp_clock_to_utc(FAR const struct comp_clock_s *clock,
                       uint64_t mono_us, FAR uint64_t *utc_us);
bool comp_clock_from_utc(FAR const struct comp_clock_s *clock,
                         uint64_t utc_us, FAR uint64_t *mono_us);
int comp_clock_observe_sync(FAR struct comp_clock_s *clock,
                            uint64_t mono_us, int64_t utc_offset_us);
int comp_clock_observe_sync_at(FAR struct comp_clock_s *clock,
                              uint64_t sample_us, int64_t utc_offset_us,
                              uint64_t apply_us);
bool comp_clock_adjust_phase(FAR struct comp_clock_s *clock,
                             uint64_t mono_us, int64_t phase_error_us,
                             uint64_t correction_time_us);

#endif /* __APPS_COMPANION_COMP_CLOCK_H */
