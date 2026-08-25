/****************************************************************************
 * apps/control_router/control_router.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_CONTROL_ROUTER_CONTROL_ROUTER_H
#define __APPS_CONTROL_ROUTER_CONTROL_ROUTER_H

#include <stdbool.h>
#include <stdint.h>

#include "control_router_policy.h"

#ifndef FAR
#  define FAR
#endif

struct control_router_status_s
{
  bool running;
  bool armed;
  bool rc_valid;
  bool auto_valid;
  uint8_t source;
  uint8_t mode;
  uint8_t reason;
  uint8_t rc_source;
  uint64_t rc_age_us;
  uint64_t auto_age_us;
  float rc_throttle;
  float rc_steering;
  float output_motor;
  float output_steering;
  uint32_t publications;
  uint32_t publish_errors;
  uint32_t arm_success;
  uint32_t arm_refused;
  uint32_t disarms;
  uint32_t rc_losses;
  uint32_t auto_stale;
};

int control_router_start(void);
int control_router_stop(void);
void control_router_status(FAR struct control_router_status_s *status);
FAR const char *control_router_reason_name(uint8_t reason);

#endif
