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
#include "ekf_delay.h"
#include "ekf_sources.h"

/* The ring itself is NOT a member here. It is ~3 kB, this struct is copied
 * wholesale under a mutex by ekf3_status(), and the task runs on a
 * 6144-byte stack. Only its counters are mirrored.
 */

struct ekf3_status_s
{
  struct ekf_core_s core;
  uint64_t first_output_us;
  uint64_t last_output_us;
  uint32_t publish_count;
  uint32_t publish_errors;
  uint32_t stale_count;
  uint32_t horizon_ms;        /* EK3_DELAY_MS as read at start */
  float    alt_noise;         /* EK3_ALT_M_NSE as read at start */
  float    alt_gate;          /* EK3_ALT_I_GATE as read at start */
  float    declination;       /* EK3_MAG_DEC, radians */
  float    yaw_noise;         /* EK3_YAW_M_NSE as read at start */
  float    yaw_gate;          /* EK3_YAW_I_GATE as read at start */
  float    mag_expected;      /* CAL_MAG0_FIELD, Gauss */
  uint32_t mag_align_used;    /* fields handed to alignment */
  uint32_t mag_in;            /* vehicle_mag messages queued */
  uint32_t baro_in;           /* vehicle_baro messages queued */
  uint32_t imu_overflow;      /* mirrored from the ring */
  uint32_t mag_overflow;
  uint32_t baro_overflow;
  uint16_t output_replay;     /* samples in the last re-propagation */
  bool     mag_available;     /* vehicle_mag subscribed */
  bool     baro_available;    /* vehicle_baro subscribed */
  struct ekf_source_config_s sources;
  bool running;
};

int ekf3_start(void);
int ekf3_stop(void);
void ekf3_status(struct ekf3_status_s *status);

#endif /* __APPS_EKF3_EKF3_H */
