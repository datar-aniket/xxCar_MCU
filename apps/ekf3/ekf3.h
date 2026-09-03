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
#include "ekf_extnav_frame.h"
#include "ekf_sources.h"

/* The ring itself is NOT a member here. It is ~3 kB, this struct is copied
 * wholesale under a mutex by ekf3_status(), and the task runs on a
 * 6144-byte stack. Only its counters are mirrored.
 */

struct ekf3_status_s
{
  struct ekf_core_s core;

  /* The monitor CORES are not members here, for the same reason the delay
   * ring is not: this struct is copied wholesale under a mutex by
   * ekf3_status(), and two more ekf_core_s would add 2800 bytes to every
   * copy on a 6144-byte stack. Only the derived numbers are mirrored.
   *
   * mon_secondary runs the SECONDARY IMU; mon_primary runs the PRIMARY, the
   * same one the main core uses. That pairing is what separates a sensor
   * fault from an aiding fault: primary-vs-monitor_primary shares the IMU,
   * so a difference there is the aiding; monitor_primary-vs-monitor_secondary
   * removes the aiding from both, so a difference there is the IMU.
   */

  bool     mon_enabled;
  bool     mon_act;              /* act on a fault, or only report it */
  float    mon_tilt_limit;       /* rad */
  uint32_t mon_hold_ms;

  bool     mon_subscribed;       /* vehicle_imu instance 1 was found */
  bool     mon_secondary_live;   /* the second IMU is actually publishing */
  uint8_t  mon_primary_status;   /* solution bits of the primary monitor */
  uint8_t  mon_secondary_status;

  /* Tilt disagreements, radians, and the per-axis body-frame errors.
   * "aiding" is primary vs monitor_primary; "imu" is the two monitors.
   */

  float    mon_aiding_tilt;
  float    mon_aiding_err[3];
  float    mon_imu_tilt;
  float    mon_imu_err[3];

  bool     mon_aiding_fault;
  bool     mon_imu_fault;
  uint32_t mon_aiding_faults;
  uint32_t mon_imu_faults;
  uint64_t mon_aiding_since;
  uint64_t mon_imu_since;
  uint64_t first_output_us;
  uint64_t last_output_us;
  uint32_t publish_count;
  uint32_t publish_errors;
  uint32_t stale_count;
  uint32_t reset_requests;
  uint32_t horizon_ms;        /* EK3_DELAY_MS as read at start */

  /* The estimator's sample-time clock is TIM5.  CLOCK_MONOTONIC is retained
   * only as a diagnostic reference so hardware can show whether the two
   * counters actually stay together instead of assuming that they do because
   * they ultimately share an oscillator.
   */

  int64_t  clock_skew_start_us; /* TIM5 - CLOCK_MONOTONIC at EKF start */
  int64_t  clock_skew_us;       /* latest TIM5 - CLOCK_MONOTONIC */
  bool     zupt_enabled;
  float    zupt_threshold_cps;
  float    zupt_noise;
  float    zupt_gate;
  float    zupt_gravity_limit;
  float    zupt_variance_limit;
  uint32_t zupt_dwell_us;
  float    zupt_gravity_deviation;
  float    zupt_accel_variance;
  bool     zupt_imu_stationary;
  bool     zupt_dwell_complete;
  uint32_t zupt_motion_block_count;
  bool     zupt_stopped;      /* wheels currently reading stopped */
  uint64_t zupt_stop_since_us;/* source sample time of stop transition */
  bool     wheel_available;   /* vesc_status is being received */
  bool     wheel_fresh;       /* newest wheel sample is <= 200 ms old */
  uint64_t last_wheel_sample_us;
  float    last_wheel_cps;

  /* Moving wheel velocity, selected by EK3_SRCn_VELXY=7. */

  bool     wheel_fusion_selected;
  float    wheel_speed_k;
  float    wheel_noise;
  float    wheel_lateral_noise;
  float    wheel_gate;
  float    wheel_accel_tau;
  float    wheel_slip_margin;
  float    wheel_position[2];       /* wheel reference -> IMU, body XY */
  uint32_t wheel_fusion_rate_hz;
  uint32_t wheel_delay_us;
  uint32_t wheel_in;
  uint32_t wheel_decimated;
  uint32_t wheel_bad;
  uint32_t wheel_overflow;
  uint32_t wheel_slip_block_count;
  uint64_t last_wheel_queued_us;
  float    wheel_speed_mps;
  float    wheel_accel_raw;
  float    wheel_accel_filtered;
  float    imu_accel_filtered;
  float    wheel_yaw_rate;
  bool     wheel_slipping;

  /* Exact delayed sample attempted on the most recent filter step. These
   * feed estimator_diag; the live fields above feed `ekf3 status`.
   */

  uint64_t wheel_diag_timestamp;
  float    wheel_diag_speed_mps;
  float    wheel_diag_accel_mps2;
  float    wheel_diag_imu_accel_mps2;

  float    height_limit;      /* EK3_HGT_LIM, m; 0 = unbounded */
  float    position_hold_limit; /* EK3_POSHOLD_M, m; 0 = free */
  float    alt_noise;         /* EK3_ALT_M_NSE as read at start */
  float    alt_gate;          /* EK3_ALT_I_GATE as read at start */
  float    declination;       /* EK3_MAG_DEC, radians */
  float    yaw_noise;         /* EK3_YAW_M_NSE as read at start */
  float    yaw_gate;          /* EK3_YAW_I_GATE as read at start */
  float    mag_expected;      /* CAL_MAG0_FIELD, Gauss */
  uint32_t mag_align_used;    /* fields handed to alignment */
  uint32_t extnav_in;         /* external_pose messages queued */
  uint32_t extnav_overflow;
  uint32_t extnav_bad_time;   /* refused on the timestamp check */
  uint32_t extnav_bad_frame;  /* non-finite pose/extrinsic transform */
  uint32_t extnav_untimed;    /* arrival-stamped: source sent zero */
  uint32_t extnav_time_clamped; /* small future/late timestamp corrected */
  uint32_t extnav_age_samples;
  int64_t  extnav_age_us;       /* receive minus corrected source time */
  int64_t  extnav_age_min_us;
  int64_t  extnav_age_max_us;
  bool     extnav_available;  /* external_pose subscribed */
  float    ext_noise;         /* EK3_EXT_M_NSE as read at start */
  float    ext_gate;
  float    ext_yaw_noise;
  int32_t  ext_delay_us;      /* EK3_EXT_DLY_MS; positive means older */
  uint32_t ext_jitter_us;     /* EK3_EXT_JIT_MS, one sigma */
  uint32_t ext_timeout_ms;
  struct ekf_extnav_extrinsics_s ext_extrinsics;
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

/* Drop the filter back to alignment.
 *
 * Requests it; the daemon performs it on its own thread. Doing it from the
 * caller's would race ekf_core_process() halfway through a propagation, and
 * a half-reset filter is worse than either state.
 *
 * Returns 0 once the reset has been observed, -ESRCH when not running, and
 * -ETIMEDOUT if the daemon did not act on it.
 */

int ekf3_reset(void);
void ekf3_status(struct ekf3_status_s *status);

#endif /* __APPS_EKF3_EKF3_H */
