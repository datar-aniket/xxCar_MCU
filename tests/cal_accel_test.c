/* Host unit test for six-position accelerometer calibration.
 *
 * This is the part of the procedure that fails silently. A swapped position, a
 * sign error in the offset, or an inverted scale all produce numbers of
 * entirely plausible magnitude - a few hundredths for an offset, something
 * near 1.0 for a scale - and the only way to notice on hardware is that the
 * vehicle slowly drifts. So the solver is driven here with a KNOWN error
 * injected, and checked for recovering exactly that error.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "cal_accel.h"

static int g_fail;

static void fail(const char *what)
{
  printf("FAIL %s\n", what);
  g_fail++;
}

/* Simulate a sensor with a known offset and scale, held in position `pos`.
 *
 * The sensor reports raw = true/scale + offset, which is the inverse of what
 * the calibration applies - so recovering (offset, scale) proves the solver
 * inverts the model in the right direction rather than merely producing
 * something self-consistent.
 */

static void fake(int pos, const float off[3], const float scl[3],
                 float tilt, float out[3])
{
  float truth[3] = { 0.0f, 0.0f, 0.0f };
  int axis = pos / 2;
  int i;

  truth[axis] = (pos & 1) ? -CAL_G : CAL_G;

  /* A little cross-axis gravity, as a hand-placed board always has. */

  truth[(axis + 1) % 3] = tilt;

  for (i = 0; i < 3; i++)
    {
      out[i] = truth[i] / scl[i] + off[i];
    }
}

static void test_classify_axes(void)
{
  const float pos_x[3] = {  CAL_G, 0.1f, 0.1f };
  const float neg_z[3] = { 0.05f, -0.05f, -CAL_G };

  if (cal_accel_classify(pos_x) != 0)
    {
      fail("classify: +X should be position 0");
    }

  if (cal_accel_classify(neg_z) != 5)
    {
      fail("classify: -Z should be position 5");
    }
}

/* A board on a corner reads a confident vector that means nothing. Accepting
 * it would poison the fit with no downstream way to notice.
 */

static void test_classify_rejects_tilt(void)
{
  const float diag[3] = { 5.66f, 5.66f, 4.0f };     /* ~45 degrees */
  const float small[3] = { 0.1f, 0.1f, 2.0f };      /* moving, or falling */

  if (cal_accel_classify(diag) >= 0)
    {
      fail("classify: accepted a 45-degree tilt");
    }

  if (cal_accel_classify(small) >= 0)
    {
      fail("classify: accepted a vector that is not 1 g");
    }
}

static void test_solve_recovers_known_error(void)
{
  const float off[3] = { 0.25f, -0.40f, 0.10f };
  const float scl[3] = { 1.02f, 0.97f, 1.01f };
  struct cal_accel_s s;
  float go[3];
  float gs[3];
  float res = -1.0f;
  int p;
  int i;

  cal_accel_reset(&s);

  for (p = 0; p < CAL_NPOS; p++)
    {
      float a[3];

      fake(p, off, scl, 0.15f, a);

      if (cal_accel_add(&s, a) != p)
        {
          printf("FAIL solve: position %d misclassified\n", p);
          g_fail++;
          return;
        }
    }

  if (!cal_accel_complete(&s) || cal_accel_count(&s) != 6)
    {
      fail("solve: six positions not registered");
      return;
    }

  if (cal_accel_solve(&s, go, gs, &res) != 0)
    {
      fail("solve: returned an error on good data");
      return;
    }

  for (i = 0; i < 3; i++)
    {
      if (fabsf(go[i] - off[i]) > 0.02f)
        {
          printf("FAIL solve: offset[%d] got %.4f want %.4f\n",
                 i, go[i], off[i]);
          g_fail++;
        }

      if (fabsf(gs[i] - scl[i]) > 0.01f)
        {
          printf("FAIL solve: scale[%d] got %.4f want %.4f\n",
                 i, gs[i], scl[i]);
          g_fail++;
        }
    }

  if (res < 0.0f || res > 0.25f)
    {
      printf("FAIL solve: residual %.4f is not small on clean data\n", res);
      g_fail++;
    }
}

/* Applying the result must actually restore gravity. Recovering the injected
 * constants could still be self-consistent nonsense if the apply convention
 * were the other way round.
 */

static void test_applied_result_reads_one_g(void)
{
  const float off[3] = { 0.5f, -0.3f, 0.2f };
  const float scl[3] = { 1.05f, 0.95f, 1.00f };
  struct cal_accel_s s;
  float go[3];
  float gs[3];
  int p;

  cal_accel_reset(&s);
  for (p = 0; p < CAL_NPOS; p++)
    {
      float a[3];

      fake(p, off, scl, 0.0f, a);
      cal_accel_add(&s, a);
    }

  if (cal_accel_solve(&s, go, gs, NULL) != 0)
    {
      fail("apply: solve failed");
      return;
    }

  for (p = 0; p < CAL_NPOS; p++)
    {
      float a[3];
      float n = 0.0f;
      int i;

      fake(p, off, scl, 0.0f, a);

      for (i = 0; i < 3; i++)
        {
          float c = (a[i] - go[i]) * gs[i];

          n += c * c;
        }

      n = sqrtf(n);

      if (fabsf(n - CAL_G) > 0.05f)
        {
          printf("FAIL apply: position %d corrects to %.3f, want %.3f\n",
                 p, n, CAL_G);
          g_fail++;
        }
    }
}

static void test_incomplete_is_refused(void)
{
  const float off[3] = { 0.0f, 0.0f, 0.0f };
  const float scl[3] = { 1.0f, 1.0f, 1.0f };
  struct cal_accel_s s;
  float go[3];
  float gs[3];
  int p;

  cal_accel_reset(&s);

  for (p = 0; p < 5; p++)                 /* one short */
    {
      float a[3];

      fake(p, off, scl, 0.0f, a);
      cal_accel_add(&s, a);
    }

  if (cal_accel_solve(&s, go, gs, NULL) == 0)
    {
      fail("solve: accepted five positions as complete");
    }
}

/* Standard deviation, not excursion, is what decides stillness - a noisy
 * sample is not motion. This checks the statistic is right on data whose
 * answer is known exactly.
 */

static void test_stats(void)
{
  float samples[10 * 2];
  float mean[2];
  float sd[2];
  int k;

  for (k = 0; k < 10; k++)
    {
      samples[k * 2]     = 5.0f;                      /* constant */
      samples[k * 2 + 1] = (k % 2) ? 1.0f : -1.0f;    /* +/-1, sd = 1 */
    }

  cal_stats(samples, 10, 2, mean, sd);

  if (fabsf(mean[0] - 5.0f) > 1e-5f || fabsf(sd[0]) > 1e-4f)
    {
      printf("FAIL stats: constant gave mean %.4f sd %.4f\n", mean[0], sd[0]);
      g_fail++;
    }

  if (fabsf(mean[1]) > 1e-5f || fabsf(sd[1] - 1.0f) > 1e-4f)
    {
      printf("FAIL stats: alternating gave mean %.4f sd %.4f\n",
             mean[1], sd[1]);
      g_fail++;
    }
}

int main(void)
{
  test_classify_axes();
  test_classify_rejects_tilt();
  test_solve_recovers_known_error();
  test_applied_result_reads_one_g();
  test_incomplete_is_refused();
  test_stats();

  if (g_fail != 0)
    {
      printf("cal_accel: %d failure(s)\n", g_fail);
      return 1;
    }

  printf("cal_accel: six-position solver verified - OK\n");
  return 0;
}
