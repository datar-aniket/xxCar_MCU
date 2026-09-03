/****************************************************************************
 * apps/ekf3/ekf_wheel.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wheel-speed preprocessing kept separate from the EKF mathematics so the
 * derivative, first-order delay and slip decision are host-testable.
 ****************************************************************************/

#ifndef __APPS_EKF3_EKF_WHEEL_H
#define __APPS_EKF3_EKF_WHEEL_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

/* A longer gap no longer describes one continuous speed derivative. */

#define EKF_WHEEL_ACCEL_MAX_GAP_US  100000ull

struct ekf_wheel_accel_filter_s
{
  uint64_t last_timestamp_us;
  float    last_speed_mps;
  float    accel_mps2;
  bool     initialized;
};

struct ekf_wheel_lpf_s
{
  float value;
  bool  initialized;
};

void ekf_wheel_accel_init(FAR struct ekf_wheel_accel_filter_s *filter);

/* Differentiate speed and apply a first-order LPF with time constant tau_s.
 * Returns true only when an actual derivative was produced; the first sample
 * and a sample after a gap establish a new reference and return false.
 */

bool ekf_wheel_accel_update(FAR struct ekf_wheel_accel_filter_s *filter,
                            float speed_mps, uint64_t timestamp_us,
                            float tau_s, FAR float *raw_accel_mps2);

float ekf_wheel_lpf_update(FAR struct ekf_wheel_lpf_s *filter, float input,
                           float dt_s, float tau_s);

/* Wheel acceleration larger than the IMU acceleration by margin_mps2 is the
 * signature used here for wheel spin or lock. Both inputs are already
 * first-order filtered, so a one-sample speed spike cannot flip the gate.
 */

bool ekf_wheel_slipping(float wheel_accel_mps2, float imu_accel_mps2,
                        float margin_mps2);

#endif /* __APPS_EKF3_EKF_WHEEL_H */
