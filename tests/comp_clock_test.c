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

static void test_large_measured_rate_and_delayed_end(void)
{
  /* Bench reproduced approximately +2600 ppm UTC/TIM5. Old +/-1000 ppm
   * bound reset every fit and left the age growing despite repeated syncs.
   * END arrives 0.4 s after the selected exchange, with alternating delay.
   */
  for (int sign = -1; sign <= 1; sign += 2)
    {
      struct comp_clock_s c;
      const int64_t offset = 1700000000000000ll;
      const uint64_t start = 1000000;
      uint64_t before, after, back;
      comp_clock_init(&c);
      assert(comp_clock_observe_sync_at(&c, start, offset, start + 400000) == COMP_CLOCK_SYNC_FIRST);
      for (unsigned i = 1; i <= 20; i++)
        {
          uint64_t sample = start + (uint64_t)i * 30000000;
          uint64_t apply = sample + (i % 2 ? 400000 : 80000);
          int64_t observed = offset + sign * (int64_t)i * 78000;
          assert(comp_clock_to_utc(&c, apply, &before));
          assert(comp_clock_observe_sync_at(&c, sample, observed, apply) == COMP_CLOCK_SYNC_UPDATED);
          assert(comp_clock_to_utc(&c, apply, &after));
          assert(before == after);
          assert_near(c.base_rate_ppb, sign * 2600000, 1);
          assert(comp_clock_from_utc(&c, after, &back));
          assert_near(back, apply, 1);
        }
      assert(c.rejected_observations == 0);
      assert_near(c.last_phase_error_us, 0, 5);
      assert_near(c.rate_ppb, sign * 2600000, 200);
      assert(comp_clock_observe_sync_at(&c, start, offset, start - 1) == COMP_CLOCK_SYNC_REJECTED);
    }
}

static void test_prompt_acquisition_corrects_phase_once(void)
{
  for (int sign = -1; sign <= 1; sign += 2)
    {
      struct comp_clock_s c;
      const uint64_t t0 = 1000000;
      const int64_t offset = 1700000000000000ll;
      uint64_t before, after, mono;
      comp_clock_init(&c);
      assert(comp_clock_observe_sync_at(&c, t0, offset, t0 + 300000) == COMP_CLOCK_SYNC_FIRST);
      assert(!c.rate_acquired);
      /* Prompt followup: both signs of the bench's 2600 ppm rate error. */
      uint64_t sample = t0 + 1600000;
      uint64_t apply = sample + 300000;
      assert(comp_clock_to_utc(&c, apply, &before));
      assert(comp_clock_observe_sync_at(&c, sample, offset + sign * 4160,
                                        apply) == COMP_CLOCK_SYNC_ACQUIRED);
      assert(comp_clock_to_utc(&c, apply, &after));
      assert_near((int64_t)after, offset + (int64_t)apply + sign * 4940, 1);
      assert_near((int64_t)after - (int64_t)before, sign * 4940, 1);
      assert_near(c.acquisition_step_us, sign * 4940, 1);
      assert(c.rate_acquired && c.last_phase_error_us == 0);
      assert_near(c.rate_ppb, sign * 2600000, 1);
      assert(comp_clock_from_utc(&c, after, &mono));
      assert_near(mono, apply, 1);
      /* Already aligned immediately, not 1-2 minutes later. */
      assert(comp_clock_to_utc(&c, apply + 1000000, &after));
      assert_near((int64_t)after, offset + (int64_t)apply + 1000000 + sign * 7540, 1);
      /* Subsequent noisy measurement cannot phase-step or reacquire. */
      sample += 1600000;
      apply = sample + 80000;
      assert(comp_clock_to_utc(&c, apply, &before));
      assert(comp_clock_observe_sync_at(&c, sample, offset + sign * 8320 + 100,
                                        apply) == COMP_CLOCK_SYNC_UPDATED);
      assert(comp_clock_to_utc(&c, apply, &after));
      assert(before == after);
      assert_near(c.acquisition_step_us, sign * 4940, 1);
    }
}

static void test_short_slew_expires_before_next_slow_sync(void)
{
  for (int sign = -1; sign <= 1; sign += 2)
    {
      struct comp_clock_s c;
      uint64_t utc, earlier, mono;
      const uint64_t start = 1000000;
      const int64_t offset = 1700000000000000ll;
      comp_clock_init(&c);
      assert(comp_clock_observe_sync(&c, start, offset) == COMP_CLOCK_SYNC_FIRST);
      assert(comp_clock_observe_sync(&c, start + 1600000, offset + 4160) == COMP_CLOCK_SYNC_ACQUIRED);
      assert(comp_clock_observe_sync(&c, start + 3200000, offset + 8320 + sign * 160) == COMP_CLOCK_SYNC_UPDATED);
      assert(c.slew_duration_us == 1600000);
      uint64_t end = c.mono_anchor_us + c.slew_duration_us;
      assert(comp_clock_rate_at(&c, end - 1) == c.rate_ppb);
      assert(comp_clock_rate_at(&c, end) == c.base_rate_ppb);
      assert(comp_clock_to_utc(&c, end, &earlier));
      assert(comp_clock_to_utc(&c, end + 30000000, &utc));
      assert_near((int64_t)(utc - earlier), 30000000 + c.base_rate_ppb * 30 / 1000, 1);
      /* Both sides of the rate transition must be continuous/invertible. */
      for (int64_t d = -5; d <= 5; d++)
        {
          assert(comp_clock_to_utc(&c, end + d, &utc));
          assert(comp_clock_from_utc(&c, utc, &mono));
          assert_near(mono, end + d, 2);
        }
      assert(comp_clock_to_utc(&c, end + 30000000, &utc));
      assert(comp_clock_from_utc(&c, utc, &mono));
      assert_near(mono, end + 30000000, 2);
    }
}

int main(void)
{
  test_seed_and_inverse();
  test_periodic_sync_learns_rate_without_steps();
  test_noisy_observation_is_slewed_not_stepped();
  test_host_clock_step_is_bounded_and_continuous();
  test_phase_adjustment_is_continuous();
  test_extreme_remote_epoch_is_safely_slewed();
  test_large_measured_rate_and_delayed_end();
  test_prompt_acquisition_corrects_phase_once();
  test_short_slew_expires_before_next_slow_sync();
  puts("comp_clock: affine UTC rate, inverse and no-jump updates - OK");
  return 0;
}
