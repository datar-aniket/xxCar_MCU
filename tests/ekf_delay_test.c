/****************************************************************************
 * tests/ekf_delay_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Horizon arithmetic, ring ordering and overflow. No filter mathematics is
 * involved, which is the point: an off-by-one here would otherwise hide
 * behind plausible-looking attitude.
 ****************************************************************************/

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ekf_delay.h"

static struct ekf_delay_s g_delay;

static struct ekf_imu_sample_s imu_at(uint64_t t)
{
  struct ekf_imu_sample_s s;

  memset(&s, 0, sizeof(s));
  s.timestamp_sample = t;
  s.timestamp_first = t - 2500;
  s.delta_angle_dt = 0.0025f;
  s.delta_velocity_dt = 0.0025f;
  s.samples = 5;
  return s;
}

/* A zero horizon makes every sample immediately due. The whole "the rewrite
 * is inert at EK3_DELAY_MS=0" argument rests on this: the ring becomes a
 * pass-through and the filter sees exactly what it saw before.
 */

static void test_zero_horizon_is_passthrough(void)
{
  struct ekf_imu_sample_s out;
  uint64_t t;

  ekf_delay_init(&g_delay, 0);

  for (t = 1000; t <= 5000; t += 1000)
    {
      struct ekf_imu_sample_s s = imu_at(t);
      assert(ekf_delay_push_imu(&g_delay, &s));
    }

  for (t = 1000; t <= 5000; t += 1000)
    {
      assert(ekf_delay_next_imu(&g_delay,
                                ekf_delay_horizon_time(&g_delay, 5000),
                                &out));
      assert(out.timestamp_sample == t);
    }

  assert(!ekf_delay_next_imu(&g_delay,
                             ekf_delay_horizon_time(&g_delay, 5000), &out));
  assert(ekf_delay_output_count(&g_delay) == 0);
}

/* With a 10 ms horizon and "now" at 20 ms, only samples at or before 10 ms
 * are due. The rest stay for the output predictor.
 */

static void test_horizon_withholds_recent(void)
{
  struct ekf_imu_sample_s out;
  uint64_t horizon;
  uint64_t t;

  ekf_delay_init(&g_delay, 10);

  for (t = 2500; t <= 20000; t += 2500)
    {
      struct ekf_imu_sample_s s = imu_at(t);
      assert(ekf_delay_push_imu(&g_delay, &s));
    }

  horizon = ekf_delay_horizon_time(&g_delay, 20000);
  assert(horizon == 10000);

  for (t = 2500; t <= 10000; t += 2500)
    {
      assert(ekf_delay_next_imu(&g_delay, horizon, &out));
      assert(out.timestamp_sample == t);
    }

  assert(!ekf_delay_next_imu(&g_delay, horizon, &out));

  /* 12500, 15000, 17500, 20000 remain for re-propagation, oldest first. */

  assert(ekf_delay_output_count(&g_delay) == 4);
  assert(ekf_delay_output_at(&g_delay, 0)->timestamp_sample == 12500);
  assert(ekf_delay_output_at(&g_delay, 3)->timestamp_sample == 20000);
  assert(ekf_delay_output_at(&g_delay, 4) == NULL);
}

static void test_horizon_saturates(void)
{
  ekf_delay_init(&g_delay, 100);
  assert(ekf_delay_horizon_time(&g_delay, 0) == 0);
  assert(ekf_delay_horizon_time(&g_delay, 50000) == 0);
  assert(ekf_delay_horizon_time(&g_delay, 150000) == 50000);
}

static void test_horizon_clamped(void)
{
  ekf_delay_init(&g_delay, 5000);
  assert(g_delay.horizon_us == (uint32_t)EKF_DELAY_MAX_MS * 1000u);
}

/* Overwriting an unconsumed sample is counted and reported, and the NEW
 * sample survives. Losing the newest data to preserve the oldest unprocessed
 * data would be the wrong trade for an estimator.
 */

static void test_overflow_counted(void)
{
  bool saw_false = false;
  int i;

  ekf_delay_init(&g_delay, 100);

  for (i = 0; i < EKF_IMU_RING_SIZE + 5; i++)
    {
      struct ekf_imu_sample_s s = imu_at((uint64_t)(i + 1) * 2500);

      if (!ekf_delay_push_imu(&g_delay, &s))
        {
          saw_false = true;
        }
    }

  assert(saw_false);
  assert(g_delay.imu_overflow_count == 5);
  assert(g_delay.imu_count == EKF_IMU_RING_SIZE);

  /* The newest sample is present; the oldest five are gone. */

  assert(ekf_delay_output_at(&g_delay, EKF_IMU_RING_SIZE - 1)
           ->timestamp_sample == (uint64_t)(EKF_IMU_RING_SIZE + 5) * 2500);
}

/* Consumed entries are what a full ring overwrites first, so a filter that is
 * keeping up never loses anything.
 */

static void test_consumed_entries_recycle_without_loss(void)
{
  struct ekf_imu_sample_s out;
  int i;

  ekf_delay_init(&g_delay, 0);

  for (i = 0; i < EKF_IMU_RING_SIZE * 3; i++)
    {
      struct ekf_imu_sample_s s = imu_at((uint64_t)(i + 1) * 2500);

      assert(ekf_delay_push_imu(&g_delay, &s));
      assert(ekf_delay_next_imu(&g_delay,
                                ekf_delay_horizon_time(&g_delay,
                                                       (uint64_t)(i + 1) *
                                                       2500),
                                &out));
      assert(out.timestamp_sample == (uint64_t)(i + 1) * 2500);
    }

  assert(g_delay.imu_overflow_count == 0);
}

/* A measurement older than the age bound is discarded, not fused late. The
 * filter has already propagated past it, so the correction would land at the
 * wrong point on the trajectory.
 */

static void test_stale_measurement_discarded(void)
{
  struct ekf_baro_sample_s out;
  struct ekf_baro_sample_s old;
  struct ekf_baro_sample_s fresh;

  memset(&old, 0, sizeof(old));
  old.timestamp_sample = 1000;
  old.pressure = 1000.0f;

  memset(&fresh, 0, sizeof(fresh));
  fresh.timestamp_sample = 900000;
  fresh.pressure = 1013.0f;

  ekf_delay_init(&g_delay, 0);
  assert(ekf_delay_push_baro(&g_delay, &old));
  assert(ekf_delay_push_baro(&g_delay, &fresh));

  /* horizon 1000000, age bound 500000: the 1000 sample is 999 ms old. */

  assert(ekf_delay_next_baro(&g_delay, 1000000, 500000, &out));
  assert(out.timestamp_sample == 900000);
  assert(!ekf_delay_next_baro(&g_delay, 1000000, 500000, &out));
}

/* A measurement from the future relative to the horizon waits its turn. */

static void test_future_measurement_waits(void)
{
  struct ekf_baro_sample_s out;
  struct ekf_baro_sample_s s;

  memset(&s, 0, sizeof(s));
  s.timestamp_sample = 50000;
  s.pressure = 1013.0f;

  ekf_delay_init(&g_delay, 0);
  assert(ekf_delay_push_baro(&g_delay, &s));
  assert(!ekf_delay_next_baro(&g_delay, 10000, 500000, &out));
  assert(ekf_delay_next_baro(&g_delay, 60000, 500000, &out));
  assert(out.timestamp_sample == 50000);
}

static void test_mag_queue(void)
{
  struct ekf_mag_sample_s out;
  struct ekf_mag_sample_s s;
  int i;

  ekf_delay_init(&g_delay, 0);

  for (i = 0; i < EKF_MAG_QUEUE_SIZE + 2; i++)
    {
      memset(&s, 0, sizeof(s));
      s.timestamp_sample = (uint64_t)(i + 1) * 20000;
      s.field[0] = 0.25f;
      s.calibrated = true;
      ekf_delay_push_mag(&g_delay, &s);
    }

  assert(g_delay.mag_overflow_count == 2);
  assert(ekf_delay_next_mag(&g_delay, 10000000, 10000000, &out));
  assert(out.timestamp_sample == 3 * 20000);   /* first two dropped */
  assert(out.calibrated);
}

/* NULL must be refused rather than dereferenced. These are called from a
 * daemon loop where a failed subscription can leave a pointer unset.
 */

static void test_null_is_refused(void)
{
  struct ekf_imu_sample_s imu = imu_at(2500);
  struct ekf_imu_sample_s out;

  ekf_delay_init(NULL, 10);
  assert(!ekf_delay_push_imu(NULL, &imu));

  ekf_delay_init(&g_delay, 0);
  assert(!ekf_delay_push_imu(&g_delay, NULL));
  assert(!ekf_delay_next_imu(&g_delay, 10000, NULL));
  assert(ekf_delay_output_count(NULL) == 0);
  assert(ekf_delay_output_at(NULL, 0) == NULL);
  (void)out;
}

/* External navigation arrives slowly and matters a lot, so the queue is
 * short and dropping the OLDEST is right - a stale absolute fix is worth
 * less than a fresh one.
 */

static void test_extnav_queue(void)
{
  struct ekf_extnav_sample_s out;
  struct ekf_extnav_sample_s s;
  int i;

  ekf_delay_init(&g_delay, 0);

  for (i = 0; i < EKF_EXTNAV_QUEUE_SIZE + 2; i++)
    {
      memset(&s, 0, sizeof(s));
      s.timestamp_sample = (uint64_t)(i + 1) * 20000;
      s.x = (float)i;
      s.valid = true;
      ekf_delay_push_extnav(&g_delay, &s);
    }

  assert(g_delay.extnav_overflow_count == 2);
  assert(ekf_delay_next_extnav(&g_delay, 10000000, 10000000, &out));
  assert(out.timestamp_sample == 3 * 20000);   /* first two dropped */
  assert(out.valid);
}

static void test_extnav_respects_the_horizon_and_age(void)
{
  struct ekf_extnav_sample_s out;
  struct ekf_extnav_sample_s old;
  struct ekf_extnav_sample_s fresh;

  memset(&old, 0, sizeof(old));
  memset(&fresh, 0, sizeof(fresh));
  old.timestamp_sample = 1000;
  fresh.timestamp_sample = 900000;

  ekf_delay_init(&g_delay, 0);
  assert(ekf_delay_push_extnav(&g_delay, &old));
  assert(ekf_delay_push_extnav(&g_delay, &fresh));

  /* Nothing is due before the horizon reaches it. */

  assert(!ekf_delay_next_extnav(&g_delay, 500, 500000, &out));

  /* The 1000 sample is 999 ms old at a horizon of 1000000 - past the bound,
   * so it is discarded rather than fused where the filter now is.
   */

  assert(ekf_delay_next_extnav(&g_delay, 1000000, 500000, &out));
  assert(out.timestamp_sample == 900000);
  assert(!ekf_delay_next_extnav(&g_delay, 1000000, 500000, &out));
}

int main(void)
{
  test_zero_horizon_is_passthrough();
  test_horizon_withholds_recent();
  test_horizon_saturates();
  test_horizon_clamped();
  test_overflow_counted();
  test_consumed_entries_recycle_without_loss();
  test_stale_measurement_discarded();
  test_future_measurement_waits();
  test_mag_queue();
  test_extnav_queue();
  test_extnav_respects_the_horizon_and_age();
  test_null_is_refused();

  puts("ekf_delay: horizon, ring ordering and overflow verified - OK");
  return 0;
}
