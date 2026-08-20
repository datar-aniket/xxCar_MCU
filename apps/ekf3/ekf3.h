/****************************************************************************
 * apps/ekf3/ekf3.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_EKF3_EKF3_H
#define __APPS_EKF3_EKF3_H

#include <stdbool.h>
#include <stdint.h>

#include "ekf_core.h"

struct ekf3_status_s
{
  struct ekf_core_s core;
  uint64_t first_output_us;
  uint64_t last_output_us;
  uint32_t publish_count;
  uint32_t publish_errors;
  uint32_t stale_count;
  bool running;
};

int ekf3_start(void);
int ekf3_stop(void);
void ekf3_status(struct ekf3_status_s *status);

#endif /* __APPS_EKF3_EKF3_H */
