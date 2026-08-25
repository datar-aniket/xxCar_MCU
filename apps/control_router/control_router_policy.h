/****************************************************************************
 * apps/control_router/control_router_policy.h
 *
 * Pure RC/source-selection policy. No NuttX or uORB dependencies so every
 * safety transition can be exercised on the host.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_CONTROL_ROUTER_CONTROL_ROUTER_POLICY_H
#define __APPS_CONTROL_ROUTER_CONTROL_ROUTER_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#define ROUTER_RC_CHANNELS       18
#define ROUTER_NEUTRAL_HOLD_US   50000u

#define ROUTER_SOURCE_RC         0
#define ROUTER_SOURCE_AUTO       1

#define ROUTER_MODE_DUTY         0
#define ROUTER_MODE_CURRENT      1

enum router_reason_e
{
  ROUTER_REASON_OK = 0,
  ROUTER_REASON_DISARMED,
  ROUTER_REASON_RC_LOST,
  ROUTER_REASON_ARM_CYCLE,
  ROUTER_REASON_NOT_NEUTRAL,
  ROUTER_REASON_ARM_HOLD,
  ROUTER_REASON_SOURCE_HOLD,
  ROUTER_REASON_AUTO_STALE,
  ROUTER_REASON_INVALID,
  ROUTER_REASON_COUNT
};

struct router_axis_config_s
{
  uint16_t negative;
  uint16_t trim;
  uint16_t positive;
  uint16_t deadzone;
};

struct router_config_s
{
  uint8_t map_steering;          /* one-based RC channel */
  uint8_t map_throttle;
  uint8_t map_source;
  uint8_t map_mode;
  uint8_t map_arm;
  uint16_t switch_low;
  uint16_t switch_high;
  uint32_t rc_timeout_us;
  uint32_t auto_timeout_us;
  float duty_max;
  float current_max;
  float arm_motor_max;
  struct router_axis_config_s steering;
  struct router_axis_config_s throttle;
};

struct router_input_s
{
  uint64_t now_us;

  uint64_t rc_timestamp;
  uint16_t rc_channel[ROUTER_RC_CHANNELS];
  uint8_t rc_count;
  bool rc_ok;
  bool rc_failsafe;

  uint64_t auto_timestamp;
  float auto_motor;
  float auto_steering;
  uint8_t auto_mode;
  bool auto_present;
};

struct router_state_s
{
  bool source_auto;
  bool mode_current;
  bool mode_switch_high;
  bool source_initialized;
  bool mode_initialized;
  bool arm_low_seen;
  bool arm_high;
  bool actual_armed;
  bool arm_holding;
  uint64_t arm_hold_until;
  uint64_t source_hold_until;
};

struct router_output_s
{
  float motor;
  float steering;
  float rc_throttle;
  float rc_steering;
  uint8_t source;
  uint8_t mode;
  uint8_t reason;
  bool rc_valid;
  bool auto_valid;
  bool request_arm;
  bool request_disarm;
};

bool router_config_valid(const struct router_config_s *config);
void router_state_init(struct router_state_s *state);
void router_policy_step(const struct router_config_s *config,
                        struct router_state_s *state,
                        const struct router_input_s *input,
                        struct router_output_s *output);

#endif
