/****************************************************************************
 * apps/vesc/vesc_speed.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Motor speed from the tachometer: differentiate, then filter.
 *
 * THIS LIVES IN THE VESC DAEMON, not in the consumer, and that placement is
 * the whole point. STATUS_5 arrives at 400 Hz while the companion downlink
 * runs at 200, and vesc_status is advertised without a queue - so a
 * subscriber reading at 200 Hz sees only the newest message and every other
 * sample is gone before it arrives. Anti-aliasing has to happen where every
 * sample still exists, which is here.
 *
 * The order matters too: the DERIVATIVE comes first and the filter second.
 * Filtering the accumulated count would smooth a position, which is not what
 * anybody wants smoothed.
 *
 * Pure - no uORB, no hardware - so the filter response is host-testable.
 ****************************************************************************/

#ifndef __APPS_VESC_VESC_SPEED_H
#define __APPS_VESC_VESC_SPEED_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

/* Longest gap still treated as one continuous measurement. Past this the
 * count cannot be related to the previous one - the VESC rebooted, the bus
 * dropped - and carrying on would emit a single enormous spike.
 */

#define VESC_SPEED_MAX_GAP_US   500000ull

/* How far a single interval may stray from the running average before it is
 * refused as an outlier. One late frame should not drag the timebase.
 */

#define VESC_SPEED_DT_MIN_RATIO 0.25f
#define VESC_SPEED_DT_MAX_RATIO 4.0f

/* Weight of one interval in the dt average. Slow, because dt is nearly
 * constant and it is the jitter that is being rejected.
 */

#define VESC_SPEED_DT_BETA      0.05f

/* Ceiling on the cutoff as a fraction of the arrival rate. A cutoff at or
 * above Nyquist is not a filter.
 */

#define VESC_SPEED_MAX_FS_FRACTION 0.4f

struct vesc_speed_s
{
  bool     primed;
  int32_t  last_tach;
  uint64_t last_us;

  float    dt_nominal;      /* seeded from the expected telemetry rate */
  float    dt_ema;          /* smoothed interval, the timebase actually used */

  float    cutoff_hz;
  float    stage[2];        /* two one-pole sections, -40 dB/decade */
  float    value;           /* filtered counts per second */
};

/* nominal_hz seeds the interval average - 400 for stock STATUS_5 - so the
 * first samples are differentiated against a sensible dt rather than
 * whatever the first two timestamps happened to be.
 *
 * cutoff_hz of zero disables filtering and passes the raw rate through.
 */

void vesc_speed_init(FAR struct vesc_speed_s *f, float nominal_hz,
                     float cutoff_hz);
void vesc_speed_reset(FAR struct vesc_speed_s *f);

/* Feed one tachometer reading, at the rate STATUS_5 actually arrives.
 * Returns the filtered rate in counts per second.
 */

float vesc_speed_update(FAR struct vesc_speed_s *f, int32_t tachometer,
                        uint64_t timestamp_us);

#endif /* __APPS_VESC_VESC_SPEED_H */
