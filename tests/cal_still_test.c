/* Host unit test for the stillness detector (apps/cal/cal_still.c).
 *
 * This is the gate on every captured orientation, so its two failure modes are
 * both expensive: declaring still while the board is drifting poisons the fit
 * with a smeared vector, and never declaring still makes calibration impossible
 * to complete. Both are cheap to provoke here and painful to diagnose on a
 * bench with a board in your hand.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "cal_still.h"

static int g_fail;

static void feed(struct cal_still_s *s, int n,
                 float ax, float ay, float az,
                 float gx, float gy, float gz, bool *saw_still)
{
  int i;

  for (i = 0; i < n; i++)
    {
      float a[3];
      float g[3];

      a[0] = ax; a[1] = ay; a[2] = az;
      g[0] = gx; g[1] = gy; g[2] = gz;

      if (cal_still_update(s, a, g) && saw_still != NULL)
        {
          *saw_still = true;
        }
    }
}

static void test_perfectly_still_is_still(void)
{
  struct cal_still_s s;
  bool still = false;

  cal_still_reset(&s, 0.02f, 0.05f, 50);
  feed(&s, 60, 0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, &still);

  if (!still)
    {
      printf("FAIL: a motionless board was never declared still\n");
      g_fail++;
    }
}

/* A board being rotated has gyro rate well above threshold. Declaring that
 * still would smear a moving gravity vector into the average.
 */

static void test_rotating_is_not_still(void)
{
  struct cal_still_s s;
  bool still = false;

  cal_still_reset(&s, 0.02f, 0.05f, 50);
  feed(&s, 200, 0.0f, 0.0f, 9.81f, 0.5f, 0.0f, 0.0f, &still);

  if (still)
    {
      printf("FAIL: a rotating board was declared still\n");
      g_fail++;
    }
}

/* Not enough samples yet, however quiet they are: an average over a handful of
 * samples has none of the noise rejection the whole exercise depends on.
 */

static void test_short_window_is_not_still(void)
{
  struct cal_still_s s;
  bool still = false;

  cal_still_reset(&s, 0.02f, 0.05f, 50);
  feed(&s, 10, 0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, &still);

  if (still)
    {
      printf("FAIL: declared still after only 10 of 50 samples\n");
      g_fail++;
    }
}

/* Motion must reset progress, not merely pause it - otherwise a board that was
 * still, moved, and stopped mid-swing could satisfy the count with a window
 * spanning the movement.
 */

static void test_motion_resets_progress(void)
{
  struct cal_still_s s;
  bool still = false;

  cal_still_reset(&s, 0.02f, 0.05f, 50);
  feed(&s, 40, 0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, NULL);
  feed(&s,  1, 0.0f, 0.0f, 9.81f, 1.0f, 0.0f, 0.0f, NULL);   /* a jolt */
  feed(&s, 40, 0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, &still);

  if (still)
    {
      printf("FAIL: a jolt did not reset the still window\n");
      g_fail++;
    }
}

/* A slide or a tap moves the accelerometer without turning the board, so the
 * gyro alone would call it still. This is the pure-translation branch in
 * cal_still_update() (checked only once count > 0, against the running mean
 * rather than a fixed value so it works in any orientation) - without it, a
 * hand nudging the board mid-capture would smear a moving gravity vector into
 * the average with no outward sign anything was wrong.
 */

static void test_accel_jolt_resets_progress(void)
{
  struct cal_still_s s;
  bool still = false;

  cal_still_reset(&s, 0.02f, 0.05f, 50);
  feed(&s, 40, 0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, NULL);
  feed(&s,  1, 0.0f, 0.0f, 10.5f, 0.0f, 0.0f, 0.0f, NULL);   /* a nudge, gyro quiet */
  feed(&s, 40, 0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, &still);

  if (still)
    {
      printf("FAIL: an accel jolt (gyro quiet) did not reset the still window\n");
      g_fail++;
    }
}

static void test_mean_is_the_average(void)
{
  struct cal_still_s s;
  float a[3];
  float g[3];

  cal_still_reset(&s, 0.02f, 0.05f, 50);
  feed(&s, 60, 1.0f, 2.0f, 9.81f, 0.001f, 0.0f, 0.0f, NULL);
  cal_still_mean(&s, a, g);

  if (fabsf(a[0] - 1.0f) > 1e-4f || fabsf(a[1] - 2.0f) > 1e-4f ||
      fabsf(a[2] - 9.81f) > 1e-4f)
    {
      printf("FAIL mean accel: (%f, %f, %f)\n", a[0], a[1], a[2]);
      g_fail++;
    }

  if (fabsf(g[0] - 0.001f) > 1e-5f)
    {
      printf("FAIL mean gyro x: %f\n", g[0]);
      g_fail++;
    }
}

int main(void)
{
  test_perfectly_still_is_still();
  test_rotating_is_not_still();
  test_short_window_is_not_still();
  test_motion_resets_progress();
  test_accel_jolt_resets_progress();
  test_mean_is_the_average();

  if (g_fail != 0)
    {
      printf("cal_still: %d failure(s)\n", g_fail);
      return 1;
    }

  printf("cal_still: stillness detection verified - OK\n");
  return 0;
}
