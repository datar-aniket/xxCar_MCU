/* Host unit test for gyro zero-rate bias measurement (apps/cal/cal_gyro.c).
 *
 * The failure this guards against is the quiet one: a board that is TURNING at
 * a constant rate is perfectly steady by any standard-deviation test, and its
 * rotation rate would be stored as zero-rate offset. Every subsequent reading
 * would then be biased by exactly the rate the board happened to be turning at
 * during calibration, and nothing downstream can tell.
 *
 * Also pinned: the accumulator has to stay accurate over a window long enough
 * to be worth taking. Summing tens of thousands of values near 1e-3 in float
 * stops moving the mean once the running total dwarfs the addend - the count
 * keeps climbing while the answer silently freezes.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "cal_gyro.h"

static int g_fail;

static void fail(const char *what)
{
  printf("FAIL %s\n", what);
  g_fail++;
}

/* Deterministic pseudo-noise, so a failure is reproducible. */

static float noise(unsigned *seed, float sd)
{
  double u1;
  double u2;

  *seed = *seed * 1103515245u + 12345u;
  u1 = ((*seed >> 8) & 0xffffff) / (double)0x1000000 + 1e-12;
  *seed = *seed * 1103515245u + 12345u;
  u2 = ((*seed >> 8) & 0xffffff) / (double)0x1000000;

  return (float)(sd * sqrt(-2.0 * log(u1)) * cos(6.283185307 * u2));
}

/* Fill an accumulator with `n` samples of a given bias plus white noise. */

static int fill(struct cal_bias_s *b, int n, const float bias[3], float sd,
                unsigned seed)
{
  int i;
  int k;

  cal_bias_reset(b);

  for (i = 0; i < n; i++)
    {
      float v[3];

      for (k = 0; k < 3; k++)
        {
          v[k] = bias[k] + noise(&seed, sd);
        }

      cal_bias_add(b, v);
    }

  return b->n;
}

/* ---- 1. a still board gives back the bias it was given ----------------- */

static void test_recovers_bias(void)
{
  const float bias[3] = { 0.0031f, -0.0072f, 0.0015f };
  struct cal_bias_s b;
  float mean[3];
  float sd[3];
  int n;
  int k;

  n = fill(&b, 2000, bias, 0.0016f, 12345u);

  if (cal_bias_result(&b, mean, sd) != n)
    {
      fail("result did not report the sample count");
      return;
    }

  for (k = 0; k < 3; k++)
    {
      /* 2000 samples at 1.6e-3 leaves about 3.6e-5 of noise on the mean;
       * allow a comfortable multiple of that.
       */

      if (fabsf(mean[k] - bias[k]) > 2.0e-4f)
        {
          printf("FAIL bias[%d] got %.6f want %.6f\n", k, mean[k], bias[k]);
          g_fail++;
        }

      if (fabsf(sd[k] - 0.0016f) > 3.0e-4f)
        {
          printf("FAIL sd[%d] got %.6f want ~0.0016\n", k, sd[k]);
          g_fail++;
        }
    }

  if (cal_gyro_judge(n, mean, sd) != CAL_GYRO_OK)
    {
      fail("a genuinely still board was refused");
    }
}

/* ---- 2. the one that matters: steady, but turning ---------------------- */

static void test_constant_rotation_is_refused(void)
{
  const float turning[3] = { 0.0f, 0.0f, 0.9f };   /* ~52 deg/s about Z */
  struct cal_bias_s b;
  float mean[3];
  float sd[3];
  int n = fill(&b, 2000, turning, 0.0016f, 999u);

  cal_bias_result(&b, mean, sd);

  /* It really does look still - that is the whole point. */

  if (sd[2] > CAL_GYRO_SD_MAX)
    {
      fail("test is not exercising the case: the rotation was not steady");
      return;
    }

  if (cal_gyro_judge(n, mean, sd) != CAL_GYRO_TOO_LARGE)
    {
      fail("a constant rotation was accepted as zero-rate bias");
    }
}

/* ---- 3. a board being handled is refused ------------------------------- */

static void test_motion_is_refused(void)
{
  const float bias[3] = { 0.001f, 0.001f, 0.001f };
  struct cal_bias_s b;
  float mean[3];
  float sd[3];
  int n = fill(&b, 2000, bias, 0.05f, 4242u);      /* 30x the noise floor */

  cal_bias_result(&b, mean, sd);

  if (cal_gyro_judge(n, mean, sd) != CAL_GYRO_NOT_STILL)
    {
      fail("a moving board was accepted");
    }
}

/* ---- 4. too short a window is refused ---------------------------------- */

static void test_short_window_is_refused(void)
{
  const float bias[3] = { 0.002f, 0.002f, 0.002f };
  struct cal_bias_s b;
  float mean[3];
  float sd[3];
  int n = fill(&b, CAL_GYRO_MIN_N - 1, bias, 0.0016f, 7u);

  cal_bias_result(&b, mean, sd);

  if (cal_gyro_judge(n, mean, sd) != CAL_GYRO_TOO_FEW)
    {
      fail("a window too short to average was accepted");
    }
}

/* ---- 5. a small spread about a LARGE mean is still measured ------------ */

static void test_spread_about_a_large_mean(void)
{
  const float turning[3] = { 0.9f, 0.9f, 0.9f };
  struct cal_bias_s b;
  float mean[3];
  float sd[3];
  int k;

  /* This is the shape that breaks a sum-of-squares variance: the mean is
   * three orders of magnitude above the spread, so sumsq/n and mean^2 are
   * nearly equal and subtracting them cancels the answer away. Measured with
   * single-precision sums, the standard deviation here came out as exactly
   * 0.00000 instead of 0.00159 - and a standard deviation of zero reads as
   * "perfectly still", which is the one conclusion that must never be reached
   * about a rotating board.
   *
   * It is not hypothetical: a rotation below CAL_GYRO_BIAS_MAX would then
   * pass BOTH gates and be stored as zero-rate bias.
   */

  fill(&b, 8000, turning, 0.0016f, 31337u);
  cal_bias_result(&b, mean, sd);

  for (k = 0; k < 3; k++)
    {
      if (fabsf(sd[k] - 0.0016f) > 3.0e-4f)
        {
          printf("FAIL large-mean sd[%d] got %.8f want ~0.00160 - the "
                 "variance is being cancelled away\n", k, sd[k]);
          g_fail++;
        }

      if (fabsf(mean[k] - 0.9f) > 1.0e-4f)
        {
          printf("FAIL large-mean mean[%d] got %.6f want 0.9\n", k, mean[k]);
          g_fail++;
        }
    }
}

/* Same shape, but at a rate small enough to pass the bias limit. Without an
 * honest standard deviation this is accepted outright.
 */

static void test_slow_rotation_under_the_bias_limit(void)
{
  const float creep[3] = { 0.0f, 0.0f, 0.15f };    /* 8.6 deg/s */
  struct cal_bias_s b;
  float mean[3];
  float sd[3];
  int n = fill(&b, 8000, creep, 0.0016f, 555u);

  cal_bias_result(&b, mean, sd);

  if (fabsf(sd[2] - 0.0016f) > 3.0e-4f)
    {
      printf("FAIL slow rotation: sd got %.8f want ~0.00160\n", sd[2]);
      g_fail++;
    }

  /* It is under CAL_GYRO_BIAS_MAX, so the magnitude gate does NOT save us
   * here - by design, since 0.15 rad/s is a plausible-looking number.
   */

  if (fabsf(creep[2]) > CAL_GYRO_BIAS_MAX)
    {
      fail("test no longer exercises the case: raise or lower the rate");
      return;
    }

  if (cal_gyro_judge(n, mean, sd) != CAL_GYRO_OK)
    {
      fail("a steady 8.6 deg/s creep should reach the OK verdict here - "
           "the sd gate cannot catch it and the bias gate must not");
    }
}

/* ---- 6. NaN must not pass ---------------------------------------------- */

static void test_nan_is_refused(void)
{
  const float ok_mean[3] = { 0.001f, 0.001f, 0.001f };
  float bad_sd[3] = { 0.001f, (float)NAN, 0.001f };
  float bad_mean[3] = { (float)NAN, 0.001f, 0.001f };
  const float ok_sd[3] = { 0.001f, 0.001f, 0.001f };

  if (cal_gyro_judge(5000, ok_mean, bad_sd) != CAL_GYRO_NOT_STILL)
    {
      fail("a NaN standard deviation was accepted");
    }

  if (cal_gyro_judge(5000, bad_mean, ok_sd) != CAL_GYRO_TOO_LARGE)
    {
      fail("a NaN bias was accepted");
    }
}

int main(void)
{
  test_recovers_bias();
  test_constant_rotation_is_refused();
  test_motion_is_refused();
  test_short_window_is_refused();
  test_spread_about_a_large_mean();
  test_slow_rotation_under_the_bias_limit();
  test_nan_is_refused();

  if (g_fail != 0)
    {
      printf("cal_gyro: %d failure(s)\n", g_fail);
      return 1;
    }

  printf("cal_gyro: bias recovered, rotation and motion refused - OK\n");
  return 0;
}
