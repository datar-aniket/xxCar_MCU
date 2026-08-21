/* Host unit test for out-of-range parameter handling (apps/param/param.c).
 *
 * The distinction under test is not cosmetic. Clamping an out-of-range SCALAR
 * to the nearest bound is a fair reading of the request. Clamping an
 * out-of-range SELECTOR is not, because neighbouring values are unrelated
 * functions: SER_USB_FUNC=5 clamped to 4 does not mean "nearly calibration",
 * it means RC_IN, and it silently started an SBUS decoder on the USB CDC port
 * - which has no UART to invert and so failed forever with ENOTTY.
 *
 * That happened on hardware, from a params.txt that outlived the firmware
 * which understood the value. These tests pin the fix so it cannot come back.
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include "param.h"

static int g_fail;

static void fail(const char *what)
{
  printf("FAIL %s\n", what);
  g_fail++;
}

/* A selector given a value it does not define must be REFUSED, and the stored
 * value must not move. Coercing it to a neighbour is the actual bug.
 */

static void test_selector_rejects_and_does_not_move(void)
{
  FAR const struct param_def_s *d;
  int32_t before;
  int32_t after;
  int32_t bad;
  int idx;
  int ret;

  /* Derive the invalid value from the table rather than hard-coding it, so
   * adding a serial function does not turn this test red for the wrong reason.
   */

  idx = param_find("SER_USB_FUNC");
  if (idx < 0)
    {
      fail("selector: SER_USB_FUNC is missing");
      return;
    }

  d   = param_def(idx);
  bad = d->max.i + 1;

  if (param_get_i32("SER_USB_FUNC", &before) < 0)
    {
      fail("selector: cannot read SER_USB_FUNC");
      return;
    }

  ret = param_set_i32("SER_USB_FUNC", bad);

  if (ret >= 0)
    {
      printf("FAIL selector: out-of-range set (%d) was accepted\n", (int)bad);
      g_fail++;
    }

  /* The whole point: a refused selector must not move at all. The original bug
   * clamped it to the maximum, which is a DIFFERENT function - that is how an
   * SBUS decoder ended up on the USB port.
   */

  if (param_get_i32("SER_USB_FUNC", &after) < 0 || after != before)
    {
      printf("FAIL selector: value moved %d -> %d on a refused set\n",
             (int)before, (int)after);
      g_fail++;
    }
}

static void test_selector_accepts_a_valid_choice(void)
{
  int32_t v;

  if (param_set_i32("SER_USB_FUNC", 2) < 0)     /* MAVLink, in range */
    {
      fail("selector: a valid choice was refused");
      return;
    }

  if (param_get_i32("SER_USB_FUNC", &v) < 0 || v != 2)
    {
      fail("selector: a valid choice did not stick");
    }
}

/* A scalar keeps the old behaviour: coerced to the nearest bound, applied, and
 * the caller told. Someone asking for a faster log rate than exists should get
 * the fastest, not a silent reset to the default.
 */

static void test_scalar_still_clamps_and_applies(void)
{
  int32_t v;
  int ret;

  ret = param_set_i32("LOG_RATE", 999999);      /* max is 2000 */

  if (ret != -ERANGE)
    {
      printf("FAIL scalar: expected -ERANGE, got %d\n", ret);
      g_fail++;
    }

  if (param_get_i32("LOG_RATE", &v) < 0 || v != 2000)
    {
      printf("FAIL scalar: expected clamp to 2000, got %d\n", (int)v);
      g_fail++;
    }
}

static void test_scalar_in_range_is_untouched(void)
{
  int32_t v;

  if (param_set_i32("LOG_RATE", 200) < 0)
    {
      fail("scalar: an in-range value was refused");
      return;
    }

  if (param_get_i32("LOG_RATE", &v) < 0 || v != 200)
    {
      fail("scalar: an in-range value did not stick");
    }
}

/* RC_PROT is the other selector, and it has a different range (0-3) - so this
 * catches a fix that hard-coded the serial function range.
 */

static void test_other_selector_also_protected(void)
{
  int32_t before;
  int32_t after;

  param_get_i32("RC_PROT", &before);

  if (param_set_i32("RC_PROT", 99) >= 0)
    {
      fail("RC_PROT: out-of-range set was accepted");
    }

  param_get_i32("RC_PROT", &after);

  if (after != before)
    {
      fail("RC_PROT: value moved on a refused set");
    }
}

/* A NaN passes every range comparison, so without an explicit guard it is
 * stored, written to params.txt, and read back for ever. An Allan run too
 * short to reach its curve minimum produces exactly one.
 */

static void test_nonfinite_float_is_refused(void)
{
  float before;
  float after;

  if (param_get_f32("CAL_ACC0_XOFF", &before) < 0)
    {
      fail("nonfinite: cannot read CAL_ACC0_XOFF");
      return;
    }

  if (param_set_f32("CAL_ACC0_XOFF", (float)NAN) >= 0)
    {
      fail("nonfinite: NaN was accepted");
    }

  if (param_get_f32("CAL_ACC0_XOFF", &after) < 0 || after != before)
    {
      fail("nonfinite: NaN moved the stored value");
    }

  if (param_get_f32("CAL_ACC0_XOFF", &after) == 0 && after != after)
    {
      fail("nonfinite: stored value is itself NaN");
    }

  if (param_set_f32("CAL_ACC0_XOFF", (float)INFINITY) >= 0)
    {
      fail("nonfinite: infinity was accepted");
    }
}

static void test_extrinsic_parameter_bounds(void)
{
  float value;
  int32_t valid;

  if (param_find("SENS_IMU1_POS_X") < 0 ||
      param_find("CAL_IMU1_RVX") < 0 ||
      param_find("CAL_MAG0_EXT_OK") < 0)
    {
      fail("extrinsic parameter schema is incomplete");
      return;
    }

  if (param_set_f32("SENS_IMU1_POS_X", 2.0f) != -ERANGE ||
      param_get_f32("SENS_IMU1_POS_X", &value) < 0 || value != 0.5f)
    {
      fail("extrinsic position did not clamp to its physical bound");
    }

  if (param_set_f32("CAL_IMU1_RVX", 0.1f) < 0 ||
      param_get_f32("CAL_IMU1_RVX", &value) < 0 ||
      fabsf(value - 0.1f) > 1.0e-6f)
    {
      fail("valid fine rotation was not retained");
    }

  if (param_set_i32("CAL_MAG0_EXT_OK", 2) >= 0 ||
      param_get_i32("CAL_MAG0_EXT_OK", &valid) < 0 || valid != 0)
    {
      fail("extrinsic validity selector accepted an invalid value");
    }
}

/* The horizon and barometer parameters Flash A introduces.
 *
 * EK3_DELAY_MS defaults to zero so a fresh flash reproduces the pre-horizon
 * attitude exactly and the timing rewrite can be proven inert before any
 * measurement starts correcting. Its maximum is bounded by the IMU ring, so a
 * larger value must clamp rather than index past the end of it.
 */

static void test_estimator_horizon_and_baro_bounds(void)
{
  float value;
  int32_t horizon;

  if (param_find("EK3_DELAY_MS") < 0 ||
      param_find("EK3_ALT_M_NSE") < 0 ||
      param_find("EK3_ALT_I_GATE") < 0)
    {
      fail("estimator horizon/barometer schema is incomplete");
      return;
    }

  if (param_get_i32("EK3_DELAY_MS", &horizon) < 0 || horizon != 0)
    {
      fail("EK3_DELAY_MS does not default to an inert zero horizon");
    }

  if (param_get_f32("EK3_ALT_M_NSE", &value) < 0 ||
      fabsf(value - 2.0f) > 1.0e-6f)
    {
      fail("barometer measurement noise default is not 2.0 m");
    }

  if (param_get_f32("EK3_ALT_I_GATE", &value) < 0 ||
      fabsf(value - 5.0f) > 1.0e-6f)
    {
      fail("barometer innovation gate default is not 5 sigma");
    }

  if (param_set_i32("EK3_DELAY_MS", 500) != -ERANGE ||
      param_get_i32("EK3_DELAY_MS", &horizon) < 0 || horizon != 100)
    {
      fail("EK3_DELAY_MS did not clamp to the IMU ring's bound");
    }

  if (param_set_i32("EK3_DELAY_MS", 30) < 0 ||
      param_get_i32("EK3_DELAY_MS", &horizon) < 0 || horizon != 30)
    {
      fail("a valid horizon was not retained");
    }

  if (param_set_i32("EK3_DELAY_MS", 0) < 0)
    {
      fail("could not restore the zero horizon");
    }
}

int main(void)
{
  param_init();
  test_nonfinite_float_is_refused();

  test_selector_rejects_and_does_not_move();
  test_selector_accepts_a_valid_choice();
  test_scalar_still_clamps_and_applies();
  test_scalar_in_range_is_untouched();
  test_other_selector_also_protected();
  test_extrinsic_parameter_bounds();
  test_estimator_horizon_and_baro_bounds();

  if (g_fail != 0)
    {
      printf("param_range: %d failure(s)\n", g_fail);
      return 1;
    }

  printf("param_range: selector vs scalar coercion verified - OK\n");
  return 0;
}
