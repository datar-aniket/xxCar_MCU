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

/* Motor-speed filtering.
 *
 * The tachometer is an accumulated count arriving in STATUS_5 at the VESC's
 * own rate - tens of hertz - and the difference of two integers over a short
 * interval is heavily quantised. So the rate is differentiated on ARRIVAL
 * and low-passed here.
 *
 * TWO cascaded one-pole sections rather than one, for a -40 dB/decade
 * rolloff instead of -20.
 *
 * Not the dsp_biquad3 filters the sensors use: those are designed for a
 * FIXED sample rate, and CAN telemetry does not arrive on a fixed one. A
 * biquad designed for 50 Hz and fed at 43 Hz has neither the cutoff nor the
 * damping it was built for. These sections recompute their coefficient from
 * the measured interval on every sample instead.
 *
 * WHAT THIS CANNOT DO: the tachometer is already sampled by the VESC at the
 * VESC's rate, so anything on the wheel above that Nyquist has ALREADY
 * folded down before the count reaches us. No filter on this side can undo
 * it; that would have to happen inside the VESC.
 *
 * What it can do is stop the differentiator's own quantisation noise from
 * reaching the downlink. Note the 200 Hz downlink cannot itself alias this:
 * holding a slower signal at a faster rate produces images, not aliases, and
 * the output can carry no content above the VESC Nyquist regardless.
 */

#define COMP_SPEED_MAX_GAP_US 500000ull

/* Ceiling on the cutoff as a fraction of the MEASURED arrival rate. Asking
 * for 100 Hz from a 50 Hz stream is not a filter, it is a pass-through with
 * extra steps; this makes that impossible rather than merely inadvisable.
 */

#define COMP_SPEED_MAX_FS_FRACTION 0.4f

struct comp_speed_filter_s
{
  bool     primed;
  int32_t  last_tach;
  uint64_t last_us;
  float    cutoff_hz;         /* requested, before the Nyquist clamp */
  float    stage[2];          /* the two one-pole sections */
  float    value;             /* filtered counts per second */
  float    rate_hz;           /* observed arrival rate, for diagnosis */
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

/* cutoff_hz of zero disables the filter and passes the raw rate through. */

void comp_speed_init(FAR struct comp_speed_filter_s *f, float cutoff_hz);
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
