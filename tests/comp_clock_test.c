/****************************************************************************
 * Host tests for the affine companion UTC clock.
 ****************************************************************************/

#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>

#include "comp_clock.h"

static int64_t absolute64(int64_t value)
{
  return value < 0 ? -value : value;
}

static void assert_near(int64_t actual, int64_t expected, int64_t limit)
{
  assert(absolute64(actual - expected) <= limit);
}

static void test_seed_and_inverse(void)
{
  struct comp_clock_s clock;
  uint64_t utc;
  uint64_t mono;

  comp_clock_init(&clock);
  assert(!comp_clock_to_utc(&clock, 100, &utc));
  assert(!comp_clock_from_utc(&clock, 100, &mono));

  assert(comp_clock_seed(&clock, 1000000, 1700000000000000ll));
  assert(comp_clock_to_utc(&clock, 1123456, &utc));
  assert(utc == 1700000000123456ull);
  assert(comp_clock_from_utc(&clock, utc, &mono));
  assert(mono == 1123456);
  assert(clock.valid && !clock.synchronized);
}

/* The board clock is 37 ppm slow against UTC. At the first periodic update
 * the rate becomes 74 ppm: 37 ppm is the measured oscillator correction and
 * another 37 ppm smoothly pays back the 1110 us accumulated before rate was
 * known. At the following update the phase is caught up and the model
 * settles to 37 ppm.
 */

static void test_periodic_sync_learns_rate_without_steps(void)
{
  struct comp_clock_s clock;
  const uint64_t t0 = 1000000ull;
  const int64_t utc0 = 1700000000000000ll;
  const int64_t offset0 = utc0 - (int64_t)t0;
  uint64_t before;
  uint64_t after;
  uint64_t target;
  uint64_t mono;

  comp_clock_init(&clock);
  assert(comp_clock_observe_sync(&clock, t0, offset0) ==
         COMP_CLOCK_SYNC_FIRST);
  assert(comp_clock_to_utc(&clock, t0, &after));
  assert(after == (uint64_t)utc0);

  assert(comp_clock_to_utc(&clock, t0 + 30000000ull, &before));
  assert(comp_clock_observe_sync(&clock, t0 + 30000000ull,
                                 offset0 + 1110) ==
         COMP_CLOCK_SYNC_UPDATED);
  assert(comp_clock_to_utc(&clock, t0 + 30000000ull, &after));
  assert(after == before);                 /* the update cannot step UTC */
  assert_near(clock.base_rate_ppb, 37000, 1);
  assert_near(clock.rate_ppb, 74000, 1);
  assert(clock.last_phase_error_us == 1110);

  target = (uint64_t)(utc0 + 60000000ll + 2220ll);
  assert(comp_clock_to_utc(&clock, t0 + 60000000ull, &before));
  assert_near((int64_t)before, (int64_t)target, 1);
  assert(comp_clock_observe_sync(&clock, t0 + 60000000ull,
                                 offset0 + 2220) ==
         COMP_CLOCK_SYNC_UPDATED);
  assert(comp_clock_to_utc(&clock, t0 + 60000000ull, &after));
  assert(after == before);
  assert_near(clock.base_rate_ppb, 37000, 1);
  assert_near(clock.rate_ppb, 37000, 2);
  assert_near(clock.last_phase_error_us, 0, 1);

  target = (uint64_t)(utc0 + 90000000ll + 3330ll);
  assert(comp_clock_to_utc(&clock, t0 + 90000000ull, &after));
  assert_near((int64_t)after, (int64_t)target, 2);
  assert(comp_clock_from_utc(&clock, after, &mono));
  assert_near((int64_t)mono, (int64_t)(t0 + 90000000ull), 1);
}

static void test_noisy_observation_is_slewed_not_stepped(void)
{
  struct comp_clock_s clock;
  const uint64_t t0 = 2000000ull;
  const int64_t utc0 = 1700001000000000ll;
  const int64_t offset0 = utc0 - (int64_t)t0;
  uint64_t before;
  uint64_t after;

  comp_clock_init(&clock);
  assert(comp_clock_observe_sync(&clock, t0, offset0) ==
         COMP_CLOCK_SYNC_FIRST);
  assert(comp_clock_observe_sync(&clock, t0 + 30000000ull,
                                 offset0 + 900) ==
         COMP_CLOCK_SYNC_UPDATED);

  assert(comp_clock_to_utc(&clock, t0 + 60000000ull, &before));
  assert(comp_clock_observe_sync(&clock, t0 + 60000000ull,
                                 offset0 + 2100) ==
         COMP_CLOCK_SYNC_UPDATED);
  assert(comp_clock_to_utc(&clock, t0 + 60000000ull, &after));
  assert(after == before);
  assert(clock.last_phase_error_us != 0);
  assert(clock.rate_ppb <= COMP_CLOCK_MAX_RATE_PPB);
  assert(clock.rate_ppb >= -COMP_CLOCK_MAX_RATE_PPB);
}

static void test_host_clock_step_is_bounded_and_continuous(void)
{
  struct comp_clock_s clock;
  const uint64_t t0 = 3000000ull;
  const int64_t utc0 = 1700002000000000ll;
  const int64_t offset0 = utc0 - (int64_t)t0;
  uint64_t before;
  uint64_t after;

  comp_clock_init(&clock);
  assert(comp_clock_observe_sync(&clock, t0, offset0) ==
         COMP_CLOCK_SYNC_FIRST);
  assert(comp_clock_observe_sync(&clock, t0 + 30000000ull,
                                 offset0 + 900) ==
         COMP_CLOCK_SYNC_UPDATED);

  assert(comp_clock_to_utc(&clock, t0 + 60000000ull, &before));
  assert(comp_clock_observe_sync(&clock, t0 + 60000000ull,
                                 offset0 + 1001800) ==
         COMP_CLOCK_SYNC_UPDATED);
  assert(comp_clock_to_utc(&clock, t0 + 60000000ull, &after));
  assert(after == before);
  assert(clock.rejected_observations == 1);
  assert(clock.rate_ppb <= COMP_CLOCK_MAX_RATE_PPB);
  assert(clock.rate_ppb >= -COMP_CLOCK_MAX_RATE_PPB);
}

static void test_phase_adjustment_is_continuous(void)
{
  struct comp_clock_s clock;
  uint64_t before;
  uint64_t after;

  comp_clock_init(&clock);
  assert(comp_clock_seed(&clock, 1000000, 1700003000000000ll));
  assert(comp_clock_to_utc(&clock, 2000000, &before));
  assert(comp_clock_adjust_phase(&clock, 2000000, -40, 1000000));
  assert(comp_clock_to_utc(&clock, 2000000, &after));
  assert(after == before);
  assert(clock.rate_ppb == -40000);
  assert(comp_clock_to_utc(&clock, 3000000, &after));
  assert(after == before + 1000000ull - 40ull);
}

static void test_extreme_remote_epoch_is_safely_slewed(void)
{
  struct comp_clock_s clock;
  const uint64_t t0 = 1000000ull;
  const int64_t utc0 = 1700004000000000ll;
  const int64_t offset0 = utc0 - (int64_t)t0;
  uint64_t before;
  uint64_t after;

  comp_clock_init(&clock);
  assert(comp_clock_observe_sync(&clock, t0, offset0) ==
         COMP_CLOCK_SYNC_FIRST);
  assert(comp_clock_to_utc(&clock, t0 + 30000000ull, &before));
  assert(comp_clock_observe_sync(&clock, t0 + 30000000ull,
                                 INT64_MAX - (int64_t)(t0 + 30000000ull)) ==
         COMP_CLOCK_SYNC_UPDATED);
  assert(comp_clock_to_utc(&clock, t0 + 30000000ull, &after));
  assert(after == before);
  assert(clock.rejected_observations == 1);
  assert(clock.rate_ppb == COMP_CLOCK_MAX_SLEW_PPB);
}

int main(void)
{
  test_seed_and_inverse();
  test_periodic_sync_learns_rate_without_steps();
  test_noisy_observation_is_slewed_not_stepped();
  test_host_clock_step_is_bounded_and_continuous();
  test_phase_adjustment_is_continuous();
  test_extreme_remote_epoch_is_safely_slewed();
  puts("comp_clock: affine UTC rate, inverse and no-jump updates - OK");
  return 0;
}
