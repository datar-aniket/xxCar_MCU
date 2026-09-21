/****************************************************************************
 * apps/companion/comp_clock.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include "comp_clock.h"

#define COMP_CLOCK_ONE_BILLION 1000000000ll

static int64_t clamp_rate(int64_t value, int64_t limit)
{
  if (value > limit)
    {
      return limit;
    }

  if (value < -limit)
    {
      return -limit;
    }

  return value;
}

/* Convert a timestamp/phase difference to ppb without first subtracting or
 * multiplying int64_t values.  A bad remote epoch can otherwise overflow
 * before the normal rate plausibility check gets a chance to reject it.
 */

static bool difference_rate(int64_t newer, int64_t older,
                            int64_t interval_us, int64_t limit_ppb,
                            FAR int64_t *rate_ppb)
{
  double estimate;

  if (interval_us <= 0 || limit_ppb < 0 || rate_ppb == NULL)
    {
      return false;
    }

  estimate = ((double)newer - (double)older) *
             (double)COMP_CLOCK_ONE_BILLION / (double)interval_us;

  if (estimate > (double)limit_ppb || estimate < -(double)limit_ppb)
    {
      return false;
    }

  *rate_ppb = (int64_t)(estimate >= 0.0 ? estimate + 0.5 : estimate - 0.5);
  return true;
}

static int64_t bounded_phase_rate(int64_t phase_error_us,
                                  int64_t interval_us)
{
  int64_t rate_ppb;

  if (!difference_rate(phase_error_us, 0, interval_us,
                       COMP_CLOCK_MAX_SLEW_PPB, &rate_ppb))
    {
      return phase_error_us < 0 ? -COMP_CLOCK_MAX_SLEW_PPB :
                                  COMP_CLOCK_MAX_SLEW_PPB;
    }

  return rate_ppb;
}

/* Signed multiply/divide with the ranges this clock permits. Anchors are
 * refreshed every sync, so delta is normally only tens of seconds; the
 * explicit quotient also keeps an unusually long unsynchronised uptime from
 * overflowing delta * rate.
 */

static int64_t scale_correction(int64_t delta, int64_t rate_ppb)
{
  int64_t quotient = delta / COMP_CLOCK_ONE_BILLION;
  int64_t remainder = delta % COMP_CLOCK_ONE_BILLION;

  return quotient * rate_ppb +
         (remainder * rate_ppb) / COMP_CLOCK_ONE_BILLION;
}

int64_t comp_clock_rate_at(FAR const struct comp_clock_s *clock,
                          uint64_t mono_us)
{
  if (clock->slew_duration_us && mono_us >= clock->mono_anchor_us &&
      mono_us - clock->mono_anchor_us >= clock->slew_duration_us)
    return clock->base_rate_ppb;
  return clock->rate_ppb;
}

static bool utc_at(FAR const struct comp_clock_s *clock, uint64_t mono_us,
                   FAR int64_t *utc_us)
{
  int64_t delta;
  int64_t scaled;

  if (clock == NULL || utc_us == NULL || !clock->valid ||
      mono_us > INT64_MAX)
    {
      return false;
    }

  delta = (int64_t)mono_us - (int64_t)clock->mono_anchor_us;
  scaled = delta + scale_correction(delta, clock->rate_ppb);
  if (clock->slew_duration_us && delta > (int64_t)clock->slew_duration_us)
    {
      int64_t duration = (int64_t)clock->slew_duration_us;
      scaled = delta + scale_correction(duration, clock->rate_ppb) +
               scale_correction(delta - duration, clock->base_rate_ppb);
    }

  if ((scaled > 0 && clock->utc_anchor_us > INT64_MAX - scaled) ||
      (scaled < 0 && clock->utc_anchor_us < INT64_MIN - scaled))
    {
      return false;
    }

  *utc_us = clock->utc_anchor_us + scaled;
  return *utc_us > 0;
}

static void reanchor(FAR struct comp_clock_s *clock, uint64_t mono_us,
                     int64_t utc_us, int64_t rate_ppb)
{
  clock->mono_anchor_us = mono_us;
  clock->utc_anchor_us = utc_us;
  clock->rate_ppb = clamp_rate(rate_ppb, COMP_CLOCK_MAX_RATE_PPB);
  clock->slew_duration_us = 0;
}

static void observations_reset(FAR struct comp_clock_s *clock,
                               uint64_t mono_us, int64_t offset_us)
{
  clock->observation_mono_us[0] = mono_us;
  clock->observation_offset_us[0] = offset_us;
  clock->observation_count = 1;
}

static void observation_append(FAR struct comp_clock_s *clock,
                               uint64_t mono_us, int64_t offset_us)
{
  uint8_t count = clock->observation_count;

  if (count == COMP_CLOCK_OBSERVATIONS)
    {
      memmove(&clock->observation_mono_us[0],
              &clock->observation_mono_us[1],
              sizeof(clock->observation_mono_us[0]) *
              (COMP_CLOCK_OBSERVATIONS - 1));
      memmove(&clock->observation_offset_us[0],
              &clock->observation_offset_us[1],
              sizeof(clock->observation_offset_us[0]) *
              (COMP_CLOCK_OBSERVATIONS - 1));
      count--;
    }

  clock->observation_mono_us[count] = mono_us;
  clock->observation_offset_us[count] = offset_us;
  clock->observation_count = count + 1;
}

/* Least-squares slope of (UTC - TIM5) against TIM5. Since
 * UTC = TIM5 + offset(TIM5), this slope is exactly a - 1.
 */

static bool observation_rate(FAR const struct comp_clock_s *clock,
                             FAR int64_t *rate_ppb)
{
  double mean_x = 0.0;
  double mean_y = 0.0;
  double numerator = 0.0;
  double denominator = 0.0;
  uint64_t x0;
  int64_t y0;
  uint8_t i;

  if (clock == NULL || rate_ppb == NULL || clock->observation_count < 2)
    {
      return false;
    }

  x0 = clock->observation_mono_us[0];
  y0 = clock->observation_offset_us[0];

  for (i = 0; i < clock->observation_count; i++)
    {
      mean_x += (double)(clock->observation_mono_us[i] - x0);
      mean_y += (double)(clock->observation_offset_us[i] - y0);
    }

  mean_x /= (double)clock->observation_count;
  mean_y /= (double)clock->observation_count;

  for (i = 0; i < clock->observation_count; i++)
    {
      double x = (double)(clock->observation_mono_us[i] - x0) - mean_x;
      double y = (double)(clock->observation_offset_us[i] - y0) - mean_y;

      numerator += x * y;
      denominator += x * x;
    }

  if (!(denominator > 0.0))
    {
      return false;
    }

  {
    double estimate = numerator * (double)COMP_CLOCK_ONE_BILLION /
                      denominator;

    if (estimate > (double)COMP_CLOCK_MAX_RATE_PPB ||
        estimate < -(double)COMP_CLOCK_MAX_RATE_PPB)
      {
        return false;
      }

    *rate_ppb = (int64_t)(estimate >= 0.0 ? estimate + 0.5 :
                                            estimate - 0.5);
  }

  return true;
}

void comp_clock_init(FAR struct comp_clock_s *clock)
{
  if (clock != NULL)
    {
      memset(clock, 0, sizeof(*clock));
    }
}

bool comp_clock_seed(FAR struct comp_clock_s *clock, uint64_t mono_us,
                     int64_t utc_us)
{
  if (clock == NULL || mono_us > INT64_MAX || utc_us <= 0)
    {
      return false;
    }

  comp_clock_init(clock);
  reanchor(clock, mono_us, utc_us, 0);
  clock->valid = true;
  return true;
}

bool comp_clock_to_utc(FAR const struct comp_clock_s *clock,
                       uint64_t mono_us, FAR uint64_t *utc_us)
{
  int64_t value;

  if (utc_us == NULL || !utc_at(clock, mono_us, &value))
    {
      return false;
    }

  *utc_us = (uint64_t)value;
  return true;
}

bool comp_clock_from_utc(FAR const struct comp_clock_s *clock,
                         uint64_t utc_us, FAR uint64_t *mono_us)
{
  int64_t utc_delta;
  int64_t denominator;
  int64_t mono_delta;
  int64_t value;
  uint64_t anchor;

  if (clock == NULL || mono_us == NULL || !clock->valid ||
      utc_us == 0 || utc_us > INT64_MAX)
    {
      return false;
    }

  utc_delta = (int64_t)utc_us - clock->utc_anchor_us;
  denominator = COMP_CLOCK_ONE_BILLION + clock->rate_ppb;
  anchor = clock->mono_anchor_us;
  if (clock->slew_duration_us)
    {
      int64_t duration = (int64_t)clock->slew_duration_us;
      int64_t boundary = duration + scale_correction(duration, clock->rate_ppb);
      if (utc_delta > boundary)
        {
          if (anchor > (uint64_t)(INT64_MAX - duration)) return false;
          anchor += duration;
          utc_delta -= boundary;
          denominator = COMP_CLOCK_ONE_BILLION + clock->base_rate_ppb;
        }
    }

  if (denominator <= 0 ||
      (utc_delta > 0 && utc_delta > INT64_MAX / COMP_CLOCK_ONE_BILLION) ||
      (utc_delta < 0 && utc_delta < INT64_MIN / COMP_CLOCK_ONE_BILLION))
    {
      return false;
    }

  mono_delta = utc_delta * COMP_CLOCK_ONE_BILLION / denominator;

  if ((mono_delta > 0 &&
       anchor > (uint64_t)(INT64_MAX - mono_delta)) ||
      (mono_delta < 0 &&
       (int64_t)anchor < -mono_delta))
    {
      return false;
    }

  value = (int64_t)anchor + mono_delta;

  if (value < 0)
    {
      return false;
    }

  *mono_us = (uint64_t)value;
  return true;
}

int comp_clock_observe_sync(FAR struct comp_clock_s *clock,
                            uint64_t mono_us, int64_t utc_offset_us)
{
  return comp_clock_observe_sync_at(clock, mono_us, utc_offset_us, mono_us);
}

int comp_clock_observe_sync_at(FAR struct comp_clock_s *clock,
                              uint64_t mono_us, int64_t utc_offset_us,
                              uint64_t apply_us)
{
  int64_t observed_utc;
  int64_t predicted_utc;
  int64_t phase_rate_ppb;
  int64_t interval_us;
  int64_t segment_rate_ppb;
  int64_t fitted_rate_ppb;
  int64_t correction_time_us;
  bool first_rate = false;
  int result = COMP_CLOCK_SYNC_UPDATED;

  if (clock == NULL || mono_us > INT64_MAX || apply_us > INT64_MAX ||
      apply_us < mono_us || apply_us - mono_us > 10000000ull ||
      (utc_offset_us > 0 &&
       (int64_t)mono_us > INT64_MAX - utc_offset_us) ||
      (utc_offset_us < 0 &&
       (int64_t)mono_us < INT64_MIN - utc_offset_us))
    {
      return COMP_CLOCK_SYNC_REJECTED;
    }

  observed_utc = (int64_t)mono_us + utc_offset_us;

  if (observed_utc <= 0)
    {
      return COMP_CLOCK_SYNC_REJECTED;
    }

  /* The first authoritative sync is allowed to establish absolute UTC.
   * It is provisional until the first rate estimate. Startup acquisition
   * may correct phase once; steady-state updates remain continuous.
   */

  if (!clock->synchronized)
    {
      if (observed_utc > INT64_MAX - (int64_t)(apply_us - mono_us))
        return COMP_CLOCK_SYNC_REJECTED;
      reanchor(clock, apply_us, observed_utc + (apply_us - mono_us), 0);
      clock->valid = true;
      clock->synchronized = true;
      clock->last_sync_mono_us = mono_us;
      clock->last_sync_offset_us = utc_offset_us;
      clock->last_phase_error_us = 0;
      observations_reset(clock, mono_us, utc_offset_us);
      clock->sync_updates++;
      return COMP_CLOCK_SYNC_FIRST;
    }

  if (mono_us <= clock->last_sync_mono_us ||
      mono_us - clock->last_sync_mono_us < COMP_CLOCK_MIN_SYNC_US ||
      !utc_at(clock, apply_us, &predicted_utc))
    {
      clock->rejected_observations++;
      return COMP_CLOCK_SYNC_REJECTED;
    }

  interval_us = (int64_t)(mono_us - clock->last_sync_mono_us);
  segment_rate_ppb = 0;

  /* A host clock step is not oscillator drift. Start a fresh regression at
   * the new epoch and slew toward its phase; never put the discontinuity on
   * the wire or into an incoming estimator timestamp.
   */

  if (!difference_rate(utc_offset_us, clock->last_sync_offset_us,
                       interval_us, COMP_CLOCK_MAX_RATE_PPB,
                       &segment_rate_ppb))
    {
      observations_reset(clock, mono_us, utc_offset_us);
      clock->rejected_observations++;
    }
  else
    {
      observation_append(clock, mono_us, utc_offset_us);

      if (observation_rate(clock, &fitted_rate_ppb))
        {
          clock->base_rate_ppb = fitted_rate_ppb;
          first_rate = !clock->rate_acquired;
          clock->rate_acquired = true;
        }
    }

  /* Extrapolate the selected exchange to arrival using the fitted rate.
   * Fit at the measurement epoch, but reanchor at NOW: a delayed END must
   * not introduce a jump by retroactively applying a new derivative.
   */
  {
    int64_t elapsed = (int64_t)(apply_us - mono_us);
    int64_t advance = elapsed + scale_correction(elapsed, clock->base_rate_ppb);
    if (observed_utc > INT64_MAX - advance) return COMP_CLOCK_SYNC_REJECTED;
    clock->last_phase_error_us = observed_utc + advance - predicted_utc;
  }
  correction_time_us = interval_us < COMP_CLOCK_MAX_SLEW_TIME_US ?
                       interval_us : COMP_CLOCK_MAX_SLEW_TIME_US;
  phase_rate_ppb = bounded_phase_rate(clock->last_phase_error_us,
                                      correction_time_us);

  /* The first rate observation must remove the phase accumulated while
   * rate was unknown. Paying 14 ms back at 200 ppm takes 70 seconds.
   * Permit ONE bounded startup correction, for either drift sign, only
   * when a plausible fit follows promptly. START messages cannot re-enable
   * it. No TIM5, OS, sensor or actuator-clock value is changed.
   * After acquisition, including host steps/reconnects, retain the no-jump
   * steady-state contract. Late/large acquisition also takes that path.
   */
  if (first_rate && interval_us <= COMP_CLOCK_ACQUIRE_WINDOW_US &&
      clock->last_phase_error_us >= -COMP_CLOCK_ACQUIRE_STEP_US &&
      clock->last_phase_error_us <= COMP_CLOCK_ACQUIRE_STEP_US)
    {
      clock->acquisition_step_us = clock->last_phase_error_us;
      predicted_utc += clock->last_phase_error_us;
      clock->last_phase_error_us = 0;
      phase_rate_ppb = 0;
      result = COMP_CLOCK_SYNC_ACQUIRED;
    }

  /* Outside the explicit startup exception above, anchor at the OLD model
   * value: only the derivative changes at a periodic sync.
   */

  reanchor(clock, apply_us, predicted_utc,
           clock->base_rate_ppb + phase_rate_ppb);
  /* Expire phase slew mathematically, without a task/IRQ. In particular a
   * 1.6 s acquisition correction must not keep running for the next 30 s.
   * utc_at/from_utc use the same continuous two-segment mapping.
   */
  clock->slew_duration_us = phase_rate_ppb ? correction_time_us : 0;
  clock->last_sync_mono_us = mono_us;
  clock->last_sync_offset_us = utc_offset_us;
  clock->sync_updates++;
  return result;
}

bool comp_clock_adjust_phase(FAR struct comp_clock_s *clock,
                             uint64_t mono_us, int64_t phase_error_us,
                             uint64_t correction_time_us)
{
  int64_t utc_us;
  int64_t phase_rate_ppb;

  if (clock == NULL || !clock->valid || correction_time_us == 0 ||
      correction_time_us > INT64_MAX || !utc_at(clock, mono_us, &utc_us))
    {
      return false;
    }

  phase_rate_ppb = bounded_phase_rate(phase_error_us,
                                      (int64_t)correction_time_us);
  reanchor(clock, mono_us, utc_us,
           comp_clock_rate_at(clock, mono_us) + phase_rate_ppb);
  return true;
}
