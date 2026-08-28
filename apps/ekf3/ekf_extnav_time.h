/****************************************************************************
 * apps/ekf3/ekf_extnav_time.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_EKF3_EKF_EXTNAV_TIME_H
#define __APPS_EKF3_EKF_EXTNAV_TIME_H

#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

/* Prepare an external-navigation sample time for the delayed EKF timeline.
 *
 * delay_us follows ArduPilot's visual-odometry convention: a positive value
 * says the measurement is older than its supplied timestamp and is therefore
 * subtracted.  It is signed so a known remote-clock bias can be corrected in
 * either direction.
 *
 * A physical measurement cannot be newer than its receive time, nor older
 * than the state the delayed filter has already processed.  Errors within
 * three timestamp-jitter sigmas are clamped to those boundaries; larger
 * errors are rejected instead of being fused at the wrong trajectory point.
 *
 * Returns 0 unchanged, 1 clamped, or -1 rejected.  signed_age_us is the
 * receive time minus the delay-corrected source time before boundary clamps.
 */

int ekf_extnav_time_prepare(uint64_t source_time_us,
                            uint64_t receive_time_us,
                            uint64_t filter_time_us,
                            int32_t delay_us,
                            uint32_t jitter_us,
                            FAR uint64_t *corrected_time_us,
                            FAR int64_t *signed_age_us);

#endif /* __APPS_EKF3_EKF_EXTNAV_TIME_H */
