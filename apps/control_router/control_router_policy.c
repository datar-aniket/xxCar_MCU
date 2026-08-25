/****************************************************************************
 * apps/control_router/control_router_policy.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "control_router_policy.h"

static float clampf(float value, float low, float high)
{
  return value < low ? low : value > high ? high : value;
}

static bool channel_valid(uint16_t pwm)
{
  return pwm >= 750u && pwm <= 2250u;
}

static bool age_valid(uint64_t now, uint64_t stamp, uint32_t timeout)
{
  return stamp != 0 && stamp <= now && now - stamp <= timeout;
}

static bool axis_config_valid(const struct router_axis_config_s *axis)
{
  int32_t neg;
  int32_t pos;

  if (axis == NULL || axis->deadzone > 400u)
    {
      return false;
    }

  neg = (int32_t)axis->negative - (int32_t)axis->trim;
  pos = (int32_t)axis->positive - (int32_t)axis->trim;
  return neg != 0 && pos != 0 && ((neg < 0) != (pos < 0)) &&
         (uint32_t)abs(neg) > axis->deadzone &&
         (uint32_t)abs(pos) > axis->deadzone;
}

static float axis_map(uint16_t pwm, const struct router_axis_config_s *axis)
{
  float delta = (float)((int32_t)pwm - (int32_t)axis->trim);
  float neg = (float)((int32_t)axis->negative - (int32_t)axis->trim);
  float pos = (float)((int32_t)axis->positive - (int32_t)axis->trim);
  float projected_pos = delta * (pos > 0.0f ? 1.0f : -1.0f);
  float projected_neg = delta * (neg > 0.0f ? 1.0f : -1.0f);
  float dz = (float)axis->deadzone;

  if (projected_pos > dz)
    {
      return clampf((projected_pos - dz) / (fabsf(pos) - dz), 0.0f, 1.0f);
    }

  if (projected_neg > dz)
    {
      return -clampf((projected_neg - dz) / (fabsf(neg) - dz), 0.0f, 1.0f);
    }

  return 0.0f;
}

static bool switch_update(uint16_t pwm, uint16_t low, uint16_t high,
                          bool *value, bool *initialized)
{
  if (pwm <= low)
    {
      *value = false;
      *initialized = true;
      return true;
    }

  if (pwm >= high)
    {
      *value = true;
      *initialized = true;
      return true;
    }

  return false;
}

bool router_config_valid(const struct router_config_s *config)
{
  uint8_t maps[5];
  unsigned i;

  if (config == NULL || config->switch_low >= config->switch_high ||
      config->duty_max < 0.0f || config->duty_max > 1.0f ||
      config->current_max < 0.0f || config->arm_motor_max < 0.0f ||
      !axis_config_valid(&config->steering) ||
      !axis_config_valid(&config->throttle))
    {
      return false;
    }

  maps[0] = config->map_steering;
  maps[1] = config->map_throttle;
  maps[2] = config->map_source;
  maps[3] = config->map_mode;
  maps[4] = config->map_arm;

  for (i = 0; i < 5; i++)
    {
      if (maps[i] < 1 || maps[i] > ROUTER_RC_CHANNELS)
        {
          return false;
        }
    }

  return true;
}

void router_state_init(struct router_state_s *state)
{
  memset(state, 0, sizeof(*state));
}

void router_policy_step(const struct router_config_s *config,
                        struct router_state_s *state,
                        const struct router_input_s *input,
                        struct router_output_s *output)
{
  uint8_t required;
  bool selected_valid;
  bool source_changed = false;
  bool mode_changed = false;
  bool arm_initialized = false;
  bool old_source;
  bool old_mode;
  float selected_motor = 0.0f;
  float selected_steering = 0.0f;
  float arm_motor_fraction = 0.0f;
  uint8_t selected_mode;

  memset(output, 0, sizeof(*output));
  output->reason = ROUTER_REASON_INVALID;

  if (!router_config_valid(config) || state == NULL || input == NULL)
    {
      return;
    }

  required = config->map_steering;
  if (config->map_throttle > required) required = config->map_throttle;
  if (config->map_source > required) required = config->map_source;
  if (config->map_mode > required) required = config->map_mode;
  if (config->map_arm > required) required = config->map_arm;

  output->rc_valid = input->rc_ok && !input->rc_failsafe &&
                     input->rc_count >= required &&
                     age_valid(input->now_us, input->rc_timestamp,
                               config->rc_timeout_us);

  if (output->rc_valid)
    {
      uint8_t maps[5];
      unsigned i;

      maps[0] = config->map_steering;
      maps[1] = config->map_throttle;
      maps[2] = config->map_source;
      maps[3] = config->map_mode;
      maps[4] = config->map_arm;

      /* A receiver may leave unused channels at zero. Only inputs which can
       * affect routing or actuation are required to contain valid PWM.
       */

      for (i = 0; i < 5; i++)
        {
          if (!channel_valid(input->rc_channel[maps[i] - 1]))
            {
              output->rc_valid = false;
              break;
            }
        }
    }

  output->auto_valid = input->auto_present &&
                       age_valid(input->now_us, input->auto_timestamp,
                                 config->auto_timeout_us) &&
                       isfinite(input->auto_motor) &&
                       isfinite(input->auto_steering) &&
                       input->auto_mode <= ROUTER_MODE_CURRENT;

  if (!output->rc_valid)
    {
      /* Assert disarm continuously while RC is unhealthy. This makes the RC
       * safety authority dominate even if somebody armed VESC from the CLI.
       */

      output->request_disarm = true;
      state->actual_armed = false;
      state->arm_holding = false;
      state->arm_high = false;
      state->arm_low_seen = false;
      output->source = state->source_auto ? ROUTER_SOURCE_AUTO
                                          : ROUTER_SOURCE_RC;
      output->mode = state->mode_current ? ROUTER_MODE_CURRENT
                                         : ROUTER_MODE_DUTY;
      output->reason = ROUTER_REASON_RC_LOST;
      return;
    }

  old_source = state->source_auto;
  old_mode = state->mode_current;
  switch_update(input->rc_channel[config->map_source - 1],
                config->switch_low, config->switch_high,
                &state->source_auto, &state->source_initialized);
  switch_update(input->rc_channel[config->map_mode - 1],
                config->switch_low, config->switch_high,
                &state->mode_current, &state->mode_initialized);
  source_changed = state->source_initialized &&
                   old_source != state->source_auto;
  mode_changed = state->mode_initialized && old_mode != state->mode_current;

  switch_update(input->rc_channel[config->map_arm - 1],
                config->switch_low, config->switch_high,
                &state->arm_high, &arm_initialized);

  if (arm_initialized && !state->arm_high)
    {
      state->arm_low_seen = true;
      state->arm_holding = false;
      output->request_disarm = true;
      state->actual_armed = false;
    }

  output->rc_throttle = axis_map(
    input->rc_channel[config->map_throttle - 1], &config->throttle);
  output->rc_steering = axis_map(
    input->rc_channel[config->map_steering - 1], &config->steering);
  output->source = state->source_auto ? ROUTER_SOURCE_AUTO : ROUTER_SOURCE_RC;
  selected_mode = state->source_auto && output->auto_valid ? input->auto_mode :
                  state->mode_current ? ROUTER_MODE_CURRENT : ROUTER_MODE_DUTY;
  output->mode = selected_mode;

  if (state->source_auto)
    {
      selected_valid = output->auto_valid;
      selected_motor = clampf(input->auto_motor,
                              selected_mode == ROUTER_MODE_CURRENT ?
                                -config->current_max : -config->duty_max,
                              selected_mode == ROUTER_MODE_CURRENT ?
                                config->current_max : config->duty_max);
      selected_steering = clampf(input->auto_steering, -1.0f, 1.0f);
      if (selected_mode == ROUTER_MODE_CURRENT && config->current_max > 0.0f)
        {
          arm_motor_fraction = fabsf(selected_motor) / config->current_max;
        }
      else if (selected_mode == ROUTER_MODE_DUTY && config->duty_max > 0.0f)
        {
          arm_motor_fraction = fabsf(selected_motor) / config->duty_max;
        }
    }
  else
    {
      selected_valid = true;
      selected_motor = output->rc_throttle *
                       (selected_mode == ROUTER_MODE_CURRENT ?
                          config->current_max : config->duty_max);
      selected_steering = output->rc_steering;
      arm_motor_fraction = fabsf(output->rc_throttle);
    }

  /* Keep the hardware disarmed whenever the policy has not completed its
   * own arm sequence. This also cancels an out-of-band `vesc arm` command;
   * publishing neutral alone is not a substitute for removing arm state.
   */

  if (!state->actual_armed)
    {
      output->request_disarm = true;
    }

  if ((source_changed || (!state->source_auto && mode_changed)) &&
      state->actual_armed)
    {
      state->source_hold_until = input->now_us + ROUTER_NEUTRAL_HOLD_US;
    }

  if (!state->arm_high)
    {
      output->reason = ROUTER_REASON_DISARMED;
      return;
    }

  if (!state->arm_low_seen)
    {
      output->reason = ROUTER_REASON_ARM_CYCLE;
      return;
    }

  if (!selected_valid)
    {
      output->reason = state->source_auto ? ROUTER_REASON_AUTO_STALE
                                          : ROUTER_REASON_INVALID;
      return;
    }

  if (!state->actual_armed)
    {
      if (arm_motor_fraction > config->arm_motor_max)
        {
          state->arm_holding = false;
          output->reason = ROUTER_REASON_NOT_NEUTRAL;
          return;
        }

      if (!state->arm_holding)
        {
          state->arm_holding = true;
          state->arm_hold_until = input->now_us + ROUTER_NEUTRAL_HOLD_US;
        }

      if (input->now_us < state->arm_hold_until)
        {
          output->reason = ROUTER_REASON_ARM_HOLD;
          return;
        }

      output->request_arm = true;
      output->reason = ROUTER_REASON_ARM_HOLD;
      return;
    }

  if (input->now_us < state->source_hold_until)
    {
      output->reason = ROUTER_REASON_SOURCE_HOLD;
      return;
    }

  output->motor = selected_motor;
  output->steering = selected_steering;
  output->reason = ROUTER_REASON_OK;
}
