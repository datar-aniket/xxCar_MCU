/****************************************************************************
 * tests/vesc_cmd_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The arm gate and the failsafe. This is the part of the VESC link that is
 * dangerous when it is wrong, and none of it needs hardware to exercise, so
 * every state is driven here rather than discovered on a bench.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "vesc_cmd.h"
#include "vesc_proto.h"

static const struct vesc_limits_s g_lim =
{
  .cur_max    = 20.0f,
  .duty_max   = 0.30f,
  .steer_min  = 1100,
  .steer_trim = 1500,
  .steer_max  = 1900,
  .steer_offset = 0,
};

#define CLOSE(a, b) (fabsf((a) - (b)) < 1.0e-4f)

/* Neutral is zero motor at the trim microseconds. Every failsafe path below
 * has to land here, so it gets its own check.
 */

static void expect_neutral(FAR const struct vesc_cmd_out_s *o,
                           uint8_t reason)
{
  assert(o->reason == reason);
  assert(CLOSE(o->motor, 0.0f));
  assert(o->servo_us == 1500);
}

static void test_disarmed_is_neutral(void)
{
  struct vesc_cmd_out_s o;

  /* A live, in-range, perfectly valid setpoint. Disarmed still wins. */

  vesc_cmd_resolve(false, true, VESC_MODE_DUTY, 0.25f, 1.0f,
                   0, 200, &g_lim, &o);
  expect_neutral(&o, VESC_CMD_DISARMED);
}

static void test_no_setpoint_is_neutral(void)
{
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, false, VESC_MODE_DUTY, 0.25f, 1.0f,
                   0, 200, &g_lim, &o);
  expect_neutral(&o, VESC_CMD_NO_SETPOINT);

  /* With nothing ever received there is no mode to honour, so it falls back
   * to duty rather than guessing current.
   */

  assert(o.packet_id == VESC_PACKET_SET_DUTY_SERVO);
}

static void test_stale_is_neutral(void)
{
  struct vesc_cmd_out_s o;

  /* Exactly at the timeout is still live; one microsecond past is not. */

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.25f, 1.0f,
                   200000, 200, &g_lim, &o);
  assert(o.reason == VESC_CMD_ARMED);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.25f, 1.0f,
                   200001, 200, &g_lim, &o);
  expect_neutral(&o, VESC_CMD_STALE);

  /* A stale setpoint still selects the frame it asked for: the VESC should
   * keep seeing the same packet id, carrying zeros.
   */

  vesc_cmd_resolve(true, true, VESC_MODE_CURRENT, 5.0f, 0.0f,
                   1000000, 200, &g_lim, &o);
  expect_neutral(&o, VESC_CMD_STALE);
  assert(o.packet_id == VESC_PACKET_SET_CURRENT_SERVO);
}

static void test_bad_mode_is_neutral(void)
{
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, 7, 0.25f, 1.0f, 0, 200, &g_lim, &o);
  expect_neutral(&o, VESC_CMD_BAD_MODE);
  assert(o.packet_id == VESC_PACKET_SET_DUTY_SERVO);
}

static void test_armed_passes_through(void)
{
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.25f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(o.reason == VESC_CMD_ARMED);
  assert(CLOSE(o.motor, 0.25f));
  assert(o.servo_us == 1500);
  assert(o.packet_id == VESC_PACKET_SET_DUTY_SERVO);
  assert(!o.clamped);

  vesc_cmd_resolve(true, true, VESC_MODE_CURRENT, -8.0f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(o.reason == VESC_CMD_ARMED);
  assert(CLOSE(o.motor, -8.0f));
  assert(o.packet_id == VESC_PACKET_SET_CURRENT_SERVO);
}

/* The two motor limits are different numbers on purpose: a mix-up would
 * clamp duty at 20 and current at 0.3, and both are catastrophic in
 * opposite directions.
 */

static void test_motor_clamps_per_mode(void)
{
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.95f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(CLOSE(o.motor, 0.30f));
  assert(o.clamped);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, -0.95f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(CLOSE(o.motor, -0.30f));
  assert(o.clamped);

  vesc_cmd_resolve(true, true, VESC_MODE_CURRENT, 100.0f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(CLOSE(o.motor, 20.0f));
  assert(o.clamped);

  vesc_cmd_resolve(true, true, VESC_MODE_CURRENT, -100.0f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(CLOSE(o.motor, -20.0f));
  assert(o.clamped);
}

static void test_steering_maps_to_endpoints(void)
{
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 0.0f,
                   0, 200, &g_lim, &o);
  assert(o.servo_us == 1500);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 1.0f,
                   0, 200, &g_lim, &o);
  assert(o.servo_us == 1900);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, -1.0f,
                   0, 200, &g_lim, &o);
  assert(o.servo_us == 1100);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 0.5f,
                   0, 200, &g_lim, &o);
  assert(o.servo_us == 1700);

  /* Beyond full authority clamps to the endpoint, and is counted. */

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 3.0f,
                   0, 200, &g_lim, &o);
  assert(o.servo_us == 1900);
  assert(o.clamped);
}

/* Asymmetric travel is the normal case, not the exception: a linkage rarely
 * gives the same microseconds either side of straight. Each half is scaled
 * against its own endpoint, so trimming one side does not steal travel from
 * the other.
 */

static void test_steering_asymmetric(void)
{
  const struct vesc_limits_s lim =
  {
    .cur_max = 20.0f, .duty_max = 0.30f,
    .steer_min = 1200, .steer_trim = 1480, .steer_max = 1900,
  };
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 1.0f, 0, 200, &lim, &o);
  assert(o.servo_us == 1900);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, -1.0f, 0, 200, &lim, &o);
  assert(o.servo_us == 1200);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 0.0f, 0, 200, &lim, &o);
  assert(o.servo_us == 1480);

  /* Half left travels half of 280 us, not half of 420. */

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, -0.5f, 0, 200, &lim, &o);
  assert(o.servo_us == 1340);
}

/* A reversed linkage is expressed by MIN above MAX. Handling it here keeps
 * the reversal out of every controller that ever publishes a command.
 */

static void test_steering_reversed(void)
{
  const struct vesc_limits_s lim =
  {
    .cur_max = 20.0f, .duty_max = 0.30f,
    .steer_min = 1900, .steer_trim = 1500, .steer_max = 1100,
  };
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 1.0f, 0, 200, &lim, &o);
  assert(o.servo_us == 1100);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, -1.0f, 0, 200, &lim, &o);
  assert(o.servo_us == 1900);
}

static void test_steering_offset_and_final_bounds(void)
{
  struct vesc_limits_s lim = g_lim;
  struct vesc_cmd_out_s o;

  /* Offset is applied after the asymmetric steering map. */

  lim.steer_offset = 125;
  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 0.5f,
                   0, 200, &lim, &o);
  assert(o.servo_us == 1825); /* mapped 1700 + 125 */
  assert(!o.clamped);

  /* Neutral and every failsafe path receive the same centre correction. */

  vesc_cmd_resolve(false, true, VESC_MODE_DUTY, 0.2f, -1.0f,
                   0, 200, &lim, &o);
  assert(o.reason == VESC_CMD_DISARMED);
  assert(o.servo_us == 1625);
  assert(!o.clamped);

  lim.steer_offset = -200;
  vesc_cmd_resolve(true, false, VESC_MODE_DUTY, 0.0f, 0.0f,
                   0, 200, &lim, &o);
  assert(o.reason == VESC_CMD_NO_SETPOINT);
  assert(o.servo_us == 1300);

  /* The value handed to the encoder can never leave 900..2100 us. */

  lim.steer_offset = 300;
  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, 1.0f,
                   0, 200, &lim, &o);
  assert(o.servo_us == 2100);
  assert(o.clamped);

  lim.steer_offset = -300;
  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, -1.0f,
                   0, 200, &lim, &o);
  assert(o.servo_us == 900);
  assert(o.clamped);
}

static void test_rc_live_trim_map(void)
{
  assert(vesc_cmd_rc_trim(1000) == -100);
  assert(vesc_cmd_rc_trim(1250) == -50);
  assert(vesc_cmd_rc_trim(1500) == 0);
  assert(vesc_cmd_rc_trim(1750) == 50);
  assert(vesc_cmd_rc_trim(2000) == 100);

  /* Receiver values beyond the nominal stick/knob range saturate. */

  assert(vesc_cmd_rc_trim(750) == -100);
  assert(vesc_cmd_rc_trim(2250) == 100);
}

/* NaN compares false against every bound. Written the wrong way round, the
 * range check passes it straight through to an undefined cast.
 */

static void test_non_finite_is_neutral(void)
{
  struct vesc_cmd_out_s o;

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, NAN, 0.0f,
                   0, 200, &g_lim, &o);
  assert(CLOSE(o.motor, 0.0f));
  assert(o.clamped);

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, INFINITY, 0.0f,
                   0, 200, &g_lim, &o);
  assert(CLOSE(o.motor, 0.0f));
  assert(o.clamped);

  /* A NaN steering command goes to trim, not to an endpoint. */

  vesc_cmd_resolve(true, true, VESC_MODE_DUTY, 0.0f, NAN,
                   0, 200, &g_lim, &o);
  assert(o.servo_us == 1500);
  assert(o.clamped);
}

/* The arm gate. It refuses exactly one thing: arming into a live throttle
 * demand. Everything else must be permitted, or the daemon cannot be armed
 * by hand at all - `vesc set` publishes once and is stale 200 ms later.
 */

static void test_may_arm(void)
{
  assert(vesc_cmd_may_arm(false, 0.0f, 0, 200));        /* nothing yet */
  assert(vesc_cmd_may_arm(false, 0.9f, 0, 200));        /* nothing yet */
  assert(vesc_cmd_may_arm(true, 0.0f, 0, 200));         /* fresh, zero */
  assert(vesc_cmd_may_arm(true, 0.9f, 200001, 200));    /* stale */
  assert(vesc_cmd_may_arm(true, 0.0005f, 0, 200));      /* below epsilon */

  assert(!vesc_cmd_may_arm(true, 0.9f, 0, 200));        /* fresh throttle */
  assert(!vesc_cmd_may_arm(true, -0.9f, 0, 200));       /* either sign */
  assert(!vesc_cmd_may_arm(true, 0.9f, 200000, 200));   /* exactly fresh */
  assert(!vesc_cmd_may_arm(true, NAN, 0, 200));         /* not trustworthy */
}

/* The telemetry watchdog. It disarms the vehicle, so every edge matters. */

static void test_telemetry_lost(void)
{
  const uint64_t last = 1000000;

  /* Inside the window is fine, one microsecond past it is not. */

  assert(!vesc_cmd_telemetry_lost(last, last + 100000, 100));
  assert(vesc_cmd_telemetry_lost(last, last + 100000 + 1, 100));

  /* A timeout of zero disables the check entirely. */

  assert(!vesc_cmd_telemetry_lost(last, last + 10000000, 0));

  /* Nothing has ever arrived: that is start-up, not a drop-out. Disarming
   * here would mean a board that can never be armed before the first frame.
   */

  assert(!vesc_cmd_telemetry_lost(0, 10000000, 100));

  /* A timestamp at or ahead of now is a clock problem, not a comms failure.
   * Unsigned subtraction would otherwise underflow to something enormous and
   * disarm the vehicle for it.
   */

  assert(!vesc_cmd_telemetry_lost(last, last, 100));
  assert(!vesc_cmd_telemetry_lost(last, last - 500000, 100));
}

static void test_reason_names(void)
{
  int i;

  for (i = 0; i < VESC_CMD_NREASON; i++)
    {
      assert(vesc_cmd_reason_name((uint8_t)i) != NULL);
      assert(strlen(vesc_cmd_reason_name((uint8_t)i)) > 0);
    }

  assert(vesc_cmd_reason_name(200) != NULL);
}

int main(void)
{
  test_disarmed_is_neutral();
  test_no_setpoint_is_neutral();
  test_stale_is_neutral();
  test_bad_mode_is_neutral();
  test_armed_passes_through();
  test_motor_clamps_per_mode();
  test_steering_maps_to_endpoints();
  test_steering_asymmetric();
  test_steering_reversed();
  test_steering_offset_and_final_bounds();
  test_rc_live_trim_map();
  test_non_finite_is_neutral();
  test_may_arm();
  test_telemetry_lost();
  test_reason_names();

  printf("vesc_cmd: arm gate, failsafe, clamping and steering map - OK\n");
  return 0;
}
