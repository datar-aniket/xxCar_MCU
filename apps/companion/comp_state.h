/****************************************************************************
 * apps/companion/comp_state.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Assembling the VEHICLE_STATE downlink from the topics that feed it.
 *
 * Pure: no uORB, no hardware, no I/O. The frame conversions and the
 * tachometer differentiator are the parts that are wrong in ways a bench
 * cannot show you - a velocity rotated with the conjugate instead of the
 * quaternion still looks like a velocity - so they live here where a host
 * test can pin them against known rotations.
 ****************************************************************************/

#ifndef __APPS_COMPANION_COMP_STATE_H
#define __APPS_COMPANION_COMP_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "comp_proto.h"

#ifndef FAR
#  define FAR
#endif

#define COMP_STATE_GRAVITY   9.80665f

/* Below this the direction of travel is noise, so side slip is reported as
 * NaN rather than whatever atan2 makes of two near-zero numbers.
 */

#define COMP_SLIP_MIN_SPEED  0.3f

struct comp_state_inputs_s
{
  bool     est_valid;
  float    position[3];       /* local ENU, m */
  float    quaternion[4];     /* w x y z, body FLU to local ENU */
  float    velocity_enu[3];   /* m/s, ENU */
  float    accel_bias[3];     /* body m/s^2, EKF calibration residual */
  uint8_t  solution_status;
  uint8_t  reset_counter;

  bool     gyro_valid;
  float    gyro[3];           /* body rad/s, filtered */

  bool     accel_valid;
  float    accel[3];          /* body m/s^2, specific force */

  bool     vesc_valid;
  float    current_a;
  float    adc_volts;
  float    motor_counts_per_s;

  float    torque_k;
  float    steer_k;
  float    speed_k;
};

/* cutoff_hz of zero disables the filter and passes the raw rate through. */

/* Rotate a local-ENU vector into the body frame. `q` maps body to ENU, so
 * this applies its transpose.
 */

void comp_state_enu_to_body(FAR const float q[4], FAR const float v[3],
                            FAR float out[3]);

/* Remove gravity from a measured specific force, leaving linear
 * acceleration in the body frame. At rest this returns zero.
 */

void comp_state_remove_gravity(FAR const float q[4], FAR const float accel[3],
                               FAR float out[3]);

void comp_state_build(FAR const struct comp_state_inputs_s *in,
                      uint64_t timestamp_us,
                      FAR struct comp_vehicle_state_s *out);

#endif /* __APPS_COMPANION_COMP_STATE_H */
