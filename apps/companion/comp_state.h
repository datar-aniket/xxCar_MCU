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

/* Time constant of the motor-speed low pass, and the longest gap that is
 * still treated as a continuous measurement.
 *
 * The tachometer is an accumulated count arriving in STATUS_5 at tens of
 * hertz. Differentiating it produces a quantised, noisy rate, so it is
 * filtered; and after a long gap - the VESC rebooted, the bus dropped - the
 * count is no longer relatable to the previous one and the filter restarts
 * rather than emitting one enormous spike.
 */

#define COMP_SPEED_TAU_S     0.05f
#define COMP_SPEED_MAX_GAP_US 500000ull

struct comp_speed_filter_s
{
  bool     primed;
  int32_t  last_tach;
  uint64_t last_us;
  float    value;             /* filtered counts per second */
};

struct comp_state_inputs_s
{
  bool     est_valid;
  float    position[3];       /* local ENU, m */
  float    quaternion[4];     /* w x y z, body FLU to local ENU */
  float    velocity_enu[3];   /* m/s, ENU */
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

void comp_speed_reset(FAR struct comp_speed_filter_s *f);

/* Feed one tachometer reading. Returns the filtered rate in counts per
 * second.
 *
 * Call it when a NEW reading arrives, not on the downlink tick. STATUS_5
 * comes in at tens of hertz while the downlink runs at 200, so sampling this
 * on the tick would differentiate an unchanged count most of the time and
 * produce zeros punctuated by spikes - aliasing that looks exactly like
 * noise.
 */

float comp_speed_update(FAR struct comp_speed_filter_s *f,
                        int32_t tachometer, uint64_t timestamp_us);

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
