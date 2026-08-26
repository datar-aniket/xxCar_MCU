/****************************************************************************
 * tests/vesc_speed_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Motor speed from the tachometer.
 *
 * This filter is the ANTI-ALIAS filter for the companion downlink: STATUS_5
 * arrives at 400 Hz and the downlink samples at 200, so anything between 100
 * and 200 Hz folds down into the band the consumer cares about unless it is
 * stopped here. That makes the rolloff a correctness property rather than a
 * tuning preference, and it is what most of these tests measure.
 *
 * Every interval below is the real 2.5 ms of a 400 Hz stream, because the
 * dt bounds are relative to the nominal rate and a test at some other rate
 * would exercise the outlier path instead of the filter.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "vesc_speed.h"

#define CLOSE(a, b) (fabsf((a) - (b)) < 1.0e-4f)

#define TLM_HZ    400.0f
#define TLM_DT_US 2500ull

static void test_first_reading_is_zero(void)
{
  struct vesc_speed_s f;

  vesc_speed_init(&f, TLM_HZ, 100.0f);

  /* The first sample only establishes a reference. Emitting a rate would
   * divide the whole accumulated count by the time since boot.
   */

  assert(CLOSE(vesc_speed_update(&f, 100000, 1000000), 0.0f));
}

/* Seeded from the nominal rate, so the very first differentiated sample
 * divides by something sensible instead of whatever the first two
 * timestamps happened to be.
 */

static void test_dt_seeded_from_nominal(void)
{
  struct vesc_speed_s f;

  vesc_speed_init(&f, TLM_HZ, 100.0f);
  assert(CLOSE(f.dt_ema, 1.0f / TLM_HZ));

  vesc_speed_init(&f, 200.0f, 100.0f);
  assert(CLOSE(f.dt_ema, 1.0f / 200.0f));
}

/* A steady 2000 counts/s at 400 Hz: 5 counts every 2.5 ms. */

static void test_converges(void)
{
  struct vesc_speed_s f;
  int32_t tach = 0;
  uint64_t t = 1000000;
  float v = 0.0f;
  int i;

  vesc_speed_init(&f, TLM_HZ, 100.0f);
  vesc_speed_update(&f, tach, t);

  for (i = 0; i < 400; i++)
    {
      t += TLM_DT_US;
      tach += 5;
      v = vesc_speed_update(&f, tach, t);
    }

  assert(fabsf(v - 2000.0f) < 1.0f);
}

static void test_sign(void)
{
  struct vesc_speed_s f;
  int32_t tach = 0;
  uint64_t t = 1000000;
  float v = 0.0f;
  int i;

  vesc_speed_init(&f, TLM_HZ, 100.0f);
  vesc_speed_update(&f, tach, t);

  for (i = 0; i < 400; i++)
    {
      t += TLM_DT_US;
      tach -= 5;
      v = vesc_speed_update(&f, tach, t);
    }

  assert(v < 0.0f);
  assert(fabsf(v + 2000.0f) < 1.0f);
}

/* The tachometer is a 32-bit accumulator and it wraps. A finite difference
 * across the wrap is still the correct small step - the same as it would be
 * for an angle - provided the subtraction is unsigned. The SIGNED difference
 * of the raw values is about -4.3 billion.
 */

static void test_wraps(void)
{
  struct vesc_speed_s f;
  uint64_t t = 1000000;
  float v;

  vesc_speed_init(&f, TLM_HZ, 100.0f);
  vesc_speed_update(&f, INT32_MAX - 2, t);

  t += TLM_DT_US;
  v = vesc_speed_update(&f, INT32_MIN + 3, t);

  /* 5 counts forward across the wrap over 2.5 ms is 2000 counts/s raw, then
   * two sections attenuate it. Sign and order are what matter.
   */

  assert(v > 0.0f);
  assert(v < 2000.0f);
}

/* The rolloff must be TWO poles, not one.
 *
 * A step separates them. At a 100 Hz cutoff and 2.5 ms intervals,
 * alpha = dt/(tau+dt) = 0.6110, so one section passes 0.611 of a step on the
 * first sample and two sections pass alpha^2 = 0.373.
 */

static void test_is_two_pole(void)
{
  struct vesc_speed_s f;
  uint64_t t = 1000000;
  float first;

  vesc_speed_init(&f, TLM_HZ, 100.0f);
  vesc_speed_update(&f, 0, t);

  t += TLM_DT_US;
  first = vesc_speed_update(&f, 5, t);      /* a 2000 counts/s step */

  assert(first > 600.0f);
  assert(first < 900.0f);                   /* one pole would give 1222 */
}

/* A cutoff above what the arrival rate supports is clamped to 0.4*fs.
 *
 * NOT because the filter would go unstable - alpha = dt/(tau+dt) is below 1
 * for any cutoff - but because it stops filtering. At 400 Hz requested
 * against 400 Hz arrivals alpha reaches 0.863 and 0.744 of a step passes on
 * the first sample; clamped to 160 Hz it is 0.512.
 */

static void test_clamps_cutoff(void)
{
  struct vesc_speed_s f;
  uint64_t t = 1000000;
  float first;

  vesc_speed_init(&f, TLM_HZ, 400.0f);      /* at Nyquist, not a filter */
  vesc_speed_update(&f, 0, t);

  t += TLM_DT_US;
  first = vesc_speed_update(&f, 5, t);      /* a 2000 counts/s step */

  assert(first < 1240.0f);                  /* unclamped would give 1488 */
  assert(first > 800.0f);
}

/* A zero cutoff means "no filtering" and must pass the raw rate through,
 * not silently apply some default.
 */

static void test_cutoff_zero_passes_through(void)
{
  struct vesc_speed_s f;
  uint64_t t = 1000000;
  float v;

  vesc_speed_init(&f, TLM_HZ, 0.0f);
  vesc_speed_update(&f, 0, t);

  t += TLM_DT_US;
  v = vesc_speed_update(&f, 5, t);

  assert(CLOSE(v, 2000.0f));
}

/* The timebase follows the stream. Told 400 Hz but fed 500, it must end up
 * dividing by the real interval rather than the assumed one - otherwise
 * every speed would be 25% high.
 */

static void test_dt_follows_the_stream(void)
{
  struct vesc_speed_s f;
  int32_t tach = 0;
  uint64_t t = 1000000;
  float v = 0.0f;
  int i;

  vesc_speed_init(&f, TLM_HZ, 100.0f);
  vesc_speed_update(&f, tach, t);

  for (i = 0; i < 600; i++)
    {
      t += 2000;                 /* 500 Hz, not the 400 it was told */
      tach += 4;                 /* 4 counts per 2 ms = 2000 counts/s */
      v = vesc_speed_update(&f, tach, t);
    }

  assert(fabsf(f.dt_ema - 0.002f) < 1.0e-5f);
  assert(fabsf(v - 2000.0f) < 5.0f);
}

/* The averaged interval is what rejects TIMESTAMP jitter.
 *
 * Frames arrive regularly at 400 Hz and the motor turns at a constant speed,
 * so the count advances by exactly 5 every time. But if the timestamps are
 * noisy the measured interval alternates - and dividing a constant delta by
 * a jittering dt manufactures a speed ripple that is not there.
 *
 * Using the raw interval gives 5/0.002 = 2500 and 5/0.003 = 1667 in turn.
 * Using the average gives 2000 throughout, which is the truth.
 */

static void test_dt_average_rejects_timestamp_jitter(void)
{
  struct vesc_speed_s f;
  int32_t tach = 0;
  uint64_t t = 1000000;
  float lo = 1.0e9f;
  float hi = -1.0e9f;
  int i;

  vesc_speed_init(&f, TLM_HZ, 0.0f);     /* no output filter: isolate dt */
  vesc_speed_update(&f, tach, t);

  for (i = 0; i < 400; i++)
    {
      float v;

      t += (i & 1) ? 3000 : 2000;        /* jittered, 2.5 ms on average */
      tach += 5;                          /* constant true speed */
      v = vesc_speed_update(&f, tach, t);

      if (i > 100)
        {
          if (v < lo)
            {
              lo = v;
            }

          if (v > hi)
            {
              hi = v;
            }
        }
    }

  /* Raw dt would swing 1667..2500, a spread of 833. */

  assert(hi - lo < 100.0f);
  assert(fabsf((hi + lo) * 0.5f - 2000.0f) < 50.0f);
}

/* One late frame must not drag the timebase. Jitter is exactly what the
 * average exists to reject, so an interval outside the bounds is refused
 * rather than averaged in.
 */

static void test_dt_rejects_outliers(void)
{
  struct vesc_speed_s f;
  int32_t tach = 0;
  uint64_t t = 1000000;
  float settled;
  int i;

  vesc_speed_init(&f, TLM_HZ, 100.0f);
  vesc_speed_update(&f, tach, t);

  for (i = 0; i < 200; i++)
    {
      t += TLM_DT_US;
      tach += 5;
      vesc_speed_update(&f, tach, t);
    }

  settled = f.dt_ema;

  /* 20 ms: well inside the 500 ms gap limit, so it does not reset - but
   * eight times the nominal interval, so it must not move the average.
   */

  t += 20000;
  tach += 40;
  vesc_speed_update(&f, tach, t);

  assert(CLOSE(f.dt_ema, settled));
}

/* A gap so long the count cannot be related to the previous one restarts
 * the filter rather than emitting one enormous value.
 */

static void test_restarts_after_gap(void)
{
  struct vesc_speed_s f;
  uint64_t t = 1000000;

  vesc_speed_init(&f, TLM_HZ, 100.0f);
  vesc_speed_update(&f, 0, t);

  t += VESC_SPEED_MAX_GAP_US + 1;
  assert(CLOSE(vesc_speed_update(&f, 5000000, t), 0.0f));

  /* And the timebase goes back to the seed, not to whatever the gap was. */

  assert(CLOSE(f.dt_ema, 1.0f / TLM_HZ));
}

static void test_rejects_backwards_time(void)
{
  struct vesc_speed_s f;

  vesc_speed_init(&f, TLM_HZ, 100.0f);
  vesc_speed_update(&f, 0, 2000000);

  /* Same timestamp would divide by zero; an earlier one is nonsense. */

  assert(CLOSE(vesc_speed_update(&f, 100, 2000000), 0.0f));
  assert(CLOSE(vesc_speed_update(&f, 200, 1000000), 0.0f));
}

int main(void)
{
  test_first_reading_is_zero();
  test_dt_seeded_from_nominal();
  test_converges();
  test_sign();
  test_wraps();
  test_is_two_pole();
  test_clamps_cutoff();
  test_cutoff_zero_passes_through();
  test_dt_follows_the_stream();
  test_dt_average_rejects_timestamp_jitter();
  test_dt_rejects_outliers();
  test_restarts_after_gap();
  test_rejects_backwards_time();

  printf("vesc_speed: derivative, dt averaging and rolloff - OK\n");
  return 0;
}
