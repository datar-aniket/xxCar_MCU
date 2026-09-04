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

/* Magnetic heading. Declination defaults to zero, which yields MAGNETIC
 * heading rather than true - correct as a default, because there is no GPS
 * to look the real value up from and guessing one would be worse than
 * reporting the raw magnetic reference.
 */

static void test_magnetic_heading_bounds(void)
{
  float value;

  if (param_find("EK3_MAG_DEC") < 0 ||
      param_find("EK3_YAW_M_NSE") < 0 ||
      param_find("EK3_YAW_I_GATE") < 0)
    {
      fail("magnetic heading schema is incomplete");
      return;
    }

  if (param_get_f32("EK3_MAG_DEC", &value) < 0 || value != 0.0f)
    {
      fail("declination does not default to zero");
    }

  if (param_get_f32("EK3_YAW_M_NSE", &value) < 0 ||
      fabsf(value - 0.5f) > 1.0e-6f)
    {
      fail("yaw measurement noise default is not 0.5 rad");
    }

  /* Declination is an angle: both signs are meaningful and both extremes
   * are reachable. A clamp at zero would silently make west declination
   * unusable.
   */

  if (param_set_f32("EK3_MAG_DEC", -13.5f) < 0 ||
      param_get_f32("EK3_MAG_DEC", &value) < 0 ||
      fabsf(value + 13.5f) > 1.0e-6f)
    {
      fail("west declination was not retained");
    }

  if (param_set_f32("EK3_MAG_DEC", 400.0f) != -ERANGE ||
      param_get_f32("EK3_MAG_DEC", &value) < 0 || value != 180.0f)
    {
      fail("declination did not clamp to +/-180 degrees");
    }

  if (param_set_f32("EK3_MAG_DEC", 0.0f) < 0)
    {
      fail("could not restore zero declination");
    }
}

/* The companion port and the external-navigation parameters.
 *
 * SER_*_FUNC's range has to grow with the enum. PARAM_RANGE_ENUM means an
 * out-of-range value is REFUSED rather than clamped to a neighbouring
 * function - which is the whole reason that range exists - so a maximum left
 * at 5 makes the companion function indistinguishable from a typo.
 */

static void test_companion_parameters(void)
{
  float value;
  int32_t v;

  if (param_find("EXT_TX_RATE") < 0 ||
      param_find("EK3_EXT_M_NSE") < 0 ||
      param_find("EK3_EXT_I_GATE") < 0 ||
      param_find("EK3_EXT_YAW_NSE") < 0 ||
      param_find("EK3_EXT_POS_X") < 0 ||
      param_find("EK3_EXT_POS_Y") < 0 ||
      param_find("EK3_EXT_POS_Z") < 0 ||
      param_find("EK3_EXT_ROLL") < 0 ||
      param_find("EK3_EXT_PITCH") < 0 ||
      param_find("EK3_EXT_YAW") < 0 ||
      param_find("EK3_ZUPT_GDEV") < 0 ||
      param_find("EK3_ZUPT_AVAR") < 0 ||
      param_find("EK3_ZUPT_DW_MS") < 0 ||
      param_find("EK3_EXT_TIMEOUT") < 0)
    {
      fail("external navigation schema is incomplete");
      return;
    }

  /* EVERY port, not one. The maxima are widened by hand and missing one is
   * silent: that port simply refuses the companion function with a range
   * error that reads exactly like a typo at the shell. Two were in fact
   * missed the first time this was written, and a single-port check passed
   * anyway.
   */

  {
    static const char *const ports[] =
    {
      "SER_TEL1_FUNC", "SER_TEL2_FUNC", "SER_TEL3_FUNC", "SER_GPS1_FUNC",
      "SER_GPS2_FUNC", "SER_DBG_FUNC",  "SER_USB_FUNC"
    };
    size_t i;

    for (i = 0; i < sizeof(ports) / sizeof(ports[0]); i++)
      {
        int32_t saved;

        if (param_get_i32(ports[i], &saved) < 0)
          {
            fail(ports[i]);
            continue;
          }

        if (param_set_i32(ports[i], SER_FUNC_COMPANION) < 0 ||
            param_get_i32(ports[i], &v) < 0 || v != SER_FUNC_COMPANION)
          {
            printf("FAIL %s does not accept the companion function\n",
                   ports[i]);
            g_fail++;
          }

        if (param_set_i32(ports[i], SER_FUNC_COMPANION + 1) >= 0)
          {
            printf("FAIL %s accepted a function that does not exist\n",
                   ports[i]);
            g_fail++;
          }

        param_set_i32(ports[i], saved);
      }
  }

  /* Both corrected-stream filters must be ON.
   *
   * This is the latent bug the check exists for: SENS_GYR_LPF defaulted to
   * 0, so the "filtered" gyro was not filtered at all and the VEHICLE_STATE
   * twist carried the raw signal with only the ICM42688's hardware AAF.
   *
   * The CUTOFF is a tuning choice and is not asserted - 100 Hz trades
   * anti-aliasing at the downlink's 100 Hz Nyquist for control bandwidth,
   * which is the operator's call. That it is switched on is not.
   *
   * Neither filter reaches the estimator: imu_delta subscribes to
   * sensor_gyro and sensor_accel directly, so raising or lowering these
   * cannot affect EKF3.
   */

  {
    float lpf = 0.0f;

    if (param_get_f32("SENS_GYR_LPF", &lpf) < 0)
      {
        fail("SENS_GYR_LPF is missing");
      }
    else if (!(lpf > 0.0f && lpf <= 400.0f))
      {
        fail("SENS_GYR_LPF must be on and within the filter's design range");
      }

    if (param_get_f32("SENS_ACC_LPF", &lpf) < 0)
      {
        fail("SENS_ACC_LPF is missing");
      }
    else if (!(lpf > 0.0f && lpf <= 400.0f))
      {
        fail("SENS_ACC_LPF must be on and within the filter's design range");
      }
  }

  if (param_get_i32("EXT_TX_RATE", &v) < 0 || v != 200)
    {
      fail("EXT_TX_RATE does not default to 200 Hz");
    }

  /* The rate has to fit down the wire, and that is not obvious by eye.
   *
   * An ESTIMATOR_POSE frame is 56 payload bytes plus 5 of framing, and a
   * UART spends 10 bits on each byte once the start and stop bits are
   * counted. At 200 Hz that is 122 kbit/s - which does NOT fit in the
   * 115200 this port would run at if the baud were ever lowered to the
   * common default, and the failure mode is not a clean error but a port
   * that backs up until writes start failing.
   *
   * Checked at 2x so the reverse channel and any other message have room.
   */

  {
    const int32_t frame_bits = (56 + 5) * 10;
    int32_t baud = 0;

    if (param_get_i32("SER_TEL2_BAUD", &baud) < 0)
      {
        fail("SER_TEL2_BAUD is missing");
      }
    else if (v * frame_bits * 2 > baud)
      {
        fail("EXT_TX_RATE does not fit in SER_TEL2_BAUD with 2x headroom");
      }
  }

  if (param_get_f32("EK3_EXT_M_NSE", &value) < 0 ||
      fabsf(value - 0.10f) > 1.0e-6f)
    {
      fail("external position noise floor is not 0.10 m");
    }

  if (param_get_f32("EK3_EXT_YAW_NSE", &value) < 0 ||
      fabsf(value - 0.05f) > 1.0e-6f)
    {
      fail("external yaw noise floor is not 0.05 rad");
    }

  if (param_get_f32("EK3_EXT_POS_X", &value) < 0 || value != 0.0f ||
      param_get_f32("EK3_EXT_POS_Y", &value) < 0 || value != 0.0f ||
      param_get_f32("EK3_EXT_POS_Z", &value) < 0 || value != 0.0f ||
      param_get_f32("EK3_EXT_ROLL", &value) < 0 || value != 0.0f ||
      param_get_f32("EK3_EXT_PITCH", &value) < 0 || value != 0.0f ||
      param_get_f32("EK3_EXT_YAW", &value) < 0 || value != 0.0f)
    {
      fail("external navigation extrinsics do not default to identity");
    }

  if (param_get_f32("EK3_ZUPT_GDEV", &value) < 0 ||
      fabsf(value - 0.25f) > 1.0e-6f ||
      param_get_f32("EK3_ZUPT_AVAR", &value) < 0 ||
      fabsf(value - 0.10f) > 1.0e-6f ||
      param_get_i32("EK3_ZUPT_DW_MS", &v) < 0 || v != 150)
    {
      fail("zero-velocity IMU gates have incorrect defaults");
    }

  if (param_get_i32("EK3_EXT_TIMEOUT", &v) < 0 || v != 1000)
    {
      fail("external navigation timeout is not 1000 ms");
    }
}

/* The VESC link parameters.
 *
 * VESC_EN defaults OFF. A new driver touching a new peripheral does not
 * belong in the boot path until it has run at least once.
 *
 * VESC_CAN_ID defaults to 0 meaning accept-any, which is what makes the
 * first run a discovery rather than a guess.
 */

static void test_vesc_parameters(void)
{
  int32_t v;
  float ekf_speed_k;
  float state_speed_k;

  if (param_find("VESC_EN") < 0 ||
      param_find("VESC_CAN_ID") < 0 ||
      param_find("VESC_BITRATE") < 0)
    {
      fail("VESC schema is incomplete");
      return;
    }

  /* VESC_EN now defaults ON, which it did not when this link was new.
   *
   * What makes that safe is not the flag but the two properties below, so
   * they are what gets asserted rather than the flag alone.
   */

  if (param_get_i32("VESC_EN", &v) < 0 || v != 1)
    {
      fail("VESC_EN does not default to on");
    }

  /* Transmit is suppressed entirely while VESC_CAN_ID is 0, so a board that
   * boots with the link up and no node id configured commands nothing. If
   * this ever defaults to a real id, starting at boot starts commanding.
   */

  if (param_get_i32("VESC_CAN_ID", &v) < 0 || v != 0)
    {
      fail("VESC_CAN_ID must default to 0 or boot-start begins commanding");
    }

  /* The telemetry watchdog must be armed by default. Zero disables it, and a
   * vehicle that boots armed-capable with no watchdog is the combination
   * worth refusing to ship.
   */

  if (param_get_i32("VESC_TLM_TO_MS", &v) < 0 || v <= 0 || v > 500)
    {
      fail("VESC_TLM_TO_MS must default to a live, short timeout");
    }

  if (param_get_i32("VESC_CAN_ID", &v) < 0 || v != 0)
    {
      fail("VESC_CAN_ID does not default to accept-any");
    }

  if (param_get_i32("VESC_BITRATE", &v) < 0 || v != 1000000)
    {
      fail("VESC_BITRATE does not default to 1 Mbit/s");
    }

  if (param_get_f32("VESC_SPEED_K", &ekf_speed_k) < 0 ||
      param_get_f32("VESC_STATE_K", &state_speed_k) < 0 ||
      fabsf(ekf_speed_k - 1.0f) > 1.0e-6f ||
      fabsf(state_speed_k - 1.0f) > 1.0e-6f)
    {
      fail("VESC EKF/state speed scales are missing or have bad defaults");
    }

  if (param_set_f32("VESC_STATE_K", 10.0f) < 0 ||
      param_get_f32("VESC_SPEED_K", &ekf_speed_k) < 0 ||
      fabsf(ekf_speed_k - 1.0f) > 1.0e-6f)
    {
      fail("VESC state speed scale changed the independent EKF scale");
    }

  /* A controller id is one byte on the wire. 256 cannot be expressed and
   * must be refused rather than truncated to 0, which would silently become
   * accept-any.
   */

  if (param_set_i32("VESC_CAN_ID", 256) != -ERANGE ||
      param_get_i32("VESC_CAN_ID", &v) < 0 || v != 255)
    {
      fail("VESC_CAN_ID did not clamp to a single byte");
    }

  if (param_set_i32("VESC_CAN_ID", 0) < 0)
    {
      fail("could not restore VESC_CAN_ID");
    }
}

static void test_control_router_parameters(void)
{
  static const struct
  {
    const char *name;
    int32_t expected;
  } maps[] =
  {
    { "RC_MAP_STEERING", 1 },
    { "RC_MAP_THROTTLE", 3 },
    { "RC_MAP_SOURCE", 5 },
    { "RC_MAP_MODE", 6 },
    { "RC_MAP_ARM", 7 }
  };
  size_t i;
  int32_t value;

  if (param_get_i32("CTRL_ROUTER_EN", &value) < 0 || value != 1)
    {
      fail("control router does not default to boot enabled");
    }

  for (i = 0; i < sizeof(maps) / sizeof(maps[0]); i++)
    {
      if (param_get_i32(maps[i].name, &value) < 0 ||
          value != maps[i].expected)
        {
          fail(maps[i].name);
        }

      if (param_set_i32(maps[i].name, 19) >= 0 ||
          param_get_i32(maps[i].name, &value) < 0 ||
          value != maps[i].expected)
        {
          fail("RC channel selector accepted an invalid channel");
        }
    }

  if (param_get_i32("RC_SW_LOW", &value) < 0 || value != 1300 ||
      param_get_i32("RC_SW_HIGH", &value) < 0 || value != 1700)
    {
      fail("RC switch thresholds do not match the safe defaults");
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
  test_magnetic_heading_bounds();
  test_companion_parameters();
  test_vesc_parameters();
  test_control_router_parameters();

  if (g_fail != 0)
    {
      printf("param_range: %d failure(s)\n", g_fail);
      return 1;
    }

  printf("param_range: selector vs scalar coercion verified - OK\n");
  return 0;
}
