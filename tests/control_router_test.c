/****************************************************************************
 * tests/control_router_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "control_router_policy.h"

static struct router_config_s config_default(void)
{
  struct router_config_s c;

  memset(&c, 0, sizeof(c));
  c.map_steering = 1;
  c.map_throttle = 3;
  c.map_source = 5;
  c.map_mode = 6;
  c.map_arm = 7;
  c.switch_low = 1300;
  c.switch_high = 1700;
  c.rc_timeout_us = 150000;
  c.auto_timeout_us = 200000;
  c.duty_max = 0.3f;
  c.current_max = 20.0f;
  c.arm_motor_max = 0.05f;
  c.steering.negative = 1000;
  c.steering.trim = 1500;
  c.steering.positive = 2000;
  c.steering.deadzone = 30;
  c.throttle = c.steering;
  return c;
}

static struct router_input_s input_default(uint64_t now)
{
  struct router_input_s in;
  unsigned i;

  memset(&in, 0, sizeof(in));
  in.now_us = now;
  in.rc_timestamp = now;
  in.rc_count = 8;
  in.rc_ok = true;

  for (i = 0; i < ROUTER_RC_CHANNELS; i++)
    {
      in.rc_channel[i] = 1500;
    }

  in.rc_channel[4] = 1000; /* manual */
  in.rc_channel[5] = 1000; /* duty */
  in.rc_channel[6] = 1000; /* disarm */
  return in;
}

static void step(const struct router_config_s *c, struct router_state_s *s,
                 struct router_input_s *in, struct router_output_s *out)
{
  in->rc_timestamp = in->now_us;
  router_policy_step(c, s, in, out);
}

static void arm_manual(const struct router_config_s *c,
                       struct router_state_s *s,
                       struct router_input_s *in,
                       struct router_output_s *out)
{
  step(c, s, in, out);                 /* observe low */
  assert(!s->actual_armed);
  in->rc_channel[6] = 2000;
  in->now_us += 1000;
  step(c, s, in, out);                 /* neutral hold begins */
  assert(out->reason == ROUTER_REASON_ARM_HOLD);
  assert(!out->request_arm);
  in->now_us += ROUTER_NEUTRAL_HOLD_US;
  step(c, s, in, out);
  assert(out->request_arm);
  s->actual_armed = true;              /* daemon reports vesc_arm success */
  in->now_us += 1000;
  step(c, s, in, out);
  assert(out->reason == ROUTER_REASON_OK);
}

static void test_mapping_and_reversal(void)
{
  struct router_config_s c = config_default();
  struct router_state_s s;
  struct router_input_s in = input_default(1000000);
  struct router_output_s out;

  router_state_init(&s);
  arm_manual(&c, &s, &in, &out);

  in.rc_channel[0] = 2000;
  in.rc_channel[2] = 2000;
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  assert(fabsf(out.steering - 1.0f) < 1e-6f);
  assert(fabsf(out.motor - 0.3f) < 1e-6f);

  c.steering.negative = 2000;
  c.steering.positive = 1000;
  in.rc_channel[0] = 1000;
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  assert(fabsf(out.steering - 1.0f) < 1e-6f);

  in.rc_channel[0] = 1510;             /* inside deadzone */
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  assert(out.steering == 0.0f);
}

static void test_arm_requires_low_and_neutral(void)
{
  struct router_config_s c = config_default();
  struct router_state_s s;
  struct router_input_s in = input_default(2000000);
  struct router_output_s out;

  router_state_init(&s);
  in.rc_channel[6] = 2000;             /* booted high */
  step(&c, &s, &in, &out);
  assert(out.request_disarm);
  assert(out.reason == ROUTER_REASON_ARM_CYCLE);

  in.rc_channel[6] = 1000;
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  in.rc_channel[2] = 1800;
  in.rc_channel[6] = 2000;
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  assert(out.reason == ROUTER_REASON_NOT_NEUTRAL);
  assert(!out.request_arm);
}

static void test_mode_toggle_and_hysteresis(void)
{
  struct router_config_s c = config_default();
  struct router_state_s s;
  struct router_input_s in = input_default(3000000);
  struct router_output_s out;

  router_state_init(&s);
  arm_manual(&c, &s, &in, &out);
  in.rc_channel[5] = 2000;             /* rising edge: current */
  in.rc_channel[2] = 2000;
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  assert(out.reason == ROUTER_REASON_SOURCE_HOLD);
  in.now_us += ROUTER_NEUTRAL_HOLD_US;
  step(&c, &s, &in, &out);
  assert(out.mode == ROUTER_MODE_CURRENT);
  assert(fabsf(out.motor - 20.0f) < 1e-6f);

  in.rc_channel[5] = 1500;             /* hysteresis retains current */
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  assert(out.mode == ROUTER_MODE_CURRENT);

  in.rc_channel[5] = 1000;             /* release does not change mode */
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  assert(out.mode == ROUTER_MODE_CURRENT);

  in.rc_channel[5] = 2000;             /* next rising edge: duty */
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  assert(out.mode == ROUTER_MODE_DUTY);
  assert(out.reason == ROUTER_REASON_SOURCE_HOLD);
}

static void test_mode_boot_high_does_not_toggle(void)
{
  struct router_config_s c = config_default();
  struct router_state_s s;
  struct router_input_s in = input_default(3500000);
  struct router_output_s out;

  router_state_init(&s);
  in.rc_channel[5] = 2000;
  step(&c, &s, &in, &out);
  assert(out.mode == ROUTER_MODE_DUTY);

  in.rc_channel[5] = 1000;
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  in.rc_channel[5] = 2000;
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  assert(out.mode == ROUTER_MODE_CURRENT);
}

static void test_auto_and_source_hold(void)
{
  struct router_config_s c = config_default();
  struct router_state_s s;
  struct router_input_s in = input_default(4000000);
  struct router_output_s out;

  router_state_init(&s);
  arm_manual(&c, &s, &in, &out);
  in.auto_present = true;
  in.auto_timestamp = in.now_us;
  in.auto_motor = 0.1f;
  in.auto_steering = -0.25f;
  in.auto_mode = ROUTER_MODE_DUTY;
  in.rc_channel[4] = 2000;             /* auto */
  in.now_us += 1000;
  in.auto_timestamp = in.now_us;
  step(&c, &s, &in, &out);
  assert(out.reason == ROUTER_REASON_SOURCE_HOLD);
  assert(out.motor == 0.0f);

  in.now_us += ROUTER_NEUTRAL_HOLD_US;
  in.auto_timestamp = in.now_us;
  step(&c, &s, &in, &out);
  assert(out.source == ROUTER_SOURCE_AUTO);
  assert(fabsf(out.motor - 0.1f) < 1e-6f);
  assert(fabsf(out.steering + 0.25f) < 1e-6f);

  in.now_us += c.auto_timeout_us + 1u;
  step(&c, &s, &in, &out);
  assert(out.reason == ROUTER_REASON_AUTO_STALE);
  assert(out.motor == 0.0f);
  /* Stale auto neutralises; RC remains the independent kill authority. */

  assert(s.actual_armed);
}

static void test_rc_loss_disarms_and_requires_recycle(void)
{
  struct router_config_s c = config_default();
  struct router_state_s s;
  struct router_input_s in = input_default(5000000);
  struct router_output_s out;

  router_state_init(&s);
  arm_manual(&c, &s, &in, &out);
  in.rc_ok = false;
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  assert(out.request_disarm);
  assert(out.reason == ROUTER_REASON_RC_LOST);
  assert(!s.arm_low_seen);

  in.rc_ok = true;
  in.rc_channel[6] = 2000;
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  assert(out.reason == ROUTER_REASON_ARM_CYCLE);
}

static void test_rc_safety_overrides_external_arm(void)
{
  struct router_config_s c = config_default();
  struct router_state_s s;
  struct router_input_s in = input_default(6000000);
  struct router_output_s out;

  router_state_init(&s);

  /* The policy cannot assume that it is the only caller of vesc_arm(). An
   * arm switch held low must therefore assert disarm even when its local
   * armed state is false.
   */

  step(&c, &s, &in, &out);
  assert(out.request_disarm);
  assert(out.reason == ROUTER_REASON_DISARMED);

  in.rc_ok = false;
  in.now_us += 1000;
  step(&c, &s, &in, &out);
  assert(out.request_disarm);
  assert(out.reason == ROUTER_REASON_RC_LOST);
}

static void test_unused_channel_may_be_absent(void)
{
  struct router_config_s c = config_default();
  struct router_state_s s;
  struct router_input_s in = input_default(7000000);
  struct router_output_s out;

  router_state_init(&s);
  in.rc_channel[1] = 0; /* CH2 is not mapped to any router function. */
  step(&c, &s, &in, &out);
  assert(out.rc_valid);
}

int main(void)
{
  struct router_config_s c = config_default();

  assert(router_config_valid(&c));
  test_mapping_and_reversal();
  test_arm_requires_low_and_neutral();
  test_mode_toggle_and_hysteresis();
  test_mode_boot_high_does_not_toggle();
  test_auto_and_source_hold();
  test_rc_loss_disarms_and_requires_recycle();
  test_rc_safety_overrides_external_arm();
  test_unused_channel_may_be_absent();
  puts("control_router: mapping, selection and safety transitions - OK");
  return 0;
}
