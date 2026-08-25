/****************************************************************************
 * apps/ekf3/ekf_delay.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The fusion horizon: an IMU ring and timestamped measurement queues.
 *
 * The filter runs EK3_DELAY_MS behind real time so a measurement can be fused
 * against the state as it was when the measurement was taken, rather than
 * against a state that has since moved on. Publication re-propagates from the
 * horizon state forward over the samples the filter has not consumed yet, so
 * estimator_state remains a current-time topic.
 *
 * Samples are NOT removed when the filter consumes them. They stay in the
 * ring because the output predictor has to replay them. A separate index
 * tracks how far the filter has got; entries are only lost when the ring
 * wraps around, which is counted.
 *
 * This file holds no filter mathematics. That is what makes the horizon
 * arithmetic testable on its own, and the horizon arithmetic is exactly where
 * an off-by-one would otherwise hide behind plausible-looking attitude.
 ****************************************************************************/

#ifndef __APPS_EKF3_EKF_DELAY_H
#define __APPS_EKF3_EKF_DELAY_H

#include <stdbool.h>
#include <stdint.h>

#include "ekf_core.h"

/* 100 ms at 400 Hz is 40 samples. 48 leaves margin for jitter and for a
 * publication that arrives between two IMU packets. EK3_DELAY_MS is bounded
 * at 100 in the parameter table to match.
 */

#define EKF_DELAY_MAX_MS        100
#define EKF_IMU_RING_SIZE        48
#define EKF_MAG_QUEUE_SIZE        8
#define EKF_BARO_QUEUE_SIZE       4
#define EKF_EXTNAV_QUEUE_SIZE     4

struct ekf_baro_sample_s
{
  uint64_t timestamp_sample;
  float    pressure;        /* hPa */
  float    temperature;     /* degrees C */
};

struct ekf_mag_sample_s
{
  uint64_t timestamp_sample;
  float    field[3];        /* Gauss, body frame */
  bool     calibrated;
};

/* An absolute pose from the companion computer.
 *
 * pos_sigma and yaw_sigma are the source's OWN reported standard deviations,
 * already square-rooted from the covariance diagonal. Zero means the source
 * supplied no estimate; the parameter floor applies either way.
 */

struct ekf_extnav_sample_s
{
  uint64_t timestamp_sample;
  float    x;
  float    y;
  float    yaw;
  float    pos_sigma[2];    /* x, y */
  float    yaw_sigma;
  uint8_t  reset_counter;   /* the SOURCE's frame-reset generation */
  bool     valid;
};

struct ekf_delay_s
{
  struct ekf_imu_sample_s imu[EKF_IMU_RING_SIZE];
  uint16_t imu_head;        /* next write slot */
  uint16_t imu_count;       /* valid entries, <= EKF_IMU_RING_SIZE */
  uint16_t imu_consumed;    /* entries already given to the filter */

  struct ekf_baro_sample_s baro[EKF_BARO_QUEUE_SIZE];
  uint16_t baro_head;
  uint16_t baro_count;

  struct ekf_mag_sample_s mag[EKF_MAG_QUEUE_SIZE];
  uint16_t mag_head;
  uint16_t mag_count;

  uint32_t horizon_us;

  uint32_t imu_overflow_count;   /* unconsumed sample overwritten */
  uint32_t baro_overflow_count;
  uint32_t mag_overflow_count;

  struct ekf_extnav_sample_s extnav[EKF_EXTNAV_QUEUE_SIZE];
  uint16_t extnav_head;
  uint16_t extnav_count;
  uint32_t extnav_overflow_count;
};

/* horizon_ms is clamped to EKF_DELAY_MAX_MS. */

void ekf_delay_init(FAR struct ekf_delay_s *d, uint32_t horizon_ms);

/* Change the horizon without discarding buffered data. */

void ekf_delay_set_horizon(FAR struct ekf_delay_s *d, uint32_t horizon_ms);

/* Append. Returns false when the append cost an entry the filter had not
 * consumed yet, having still stored the new sample and counted the loss: the
 * newest data is always worth more than the oldest unprocessed data.
 */

bool ekf_delay_push_imu(FAR struct ekf_delay_s *d,
                        FAR const struct ekf_imu_sample_s *sample);
bool ekf_delay_push_baro(FAR struct ekf_delay_s *d,
                         FAR const struct ekf_baro_sample_s *sample);
bool ekf_delay_push_mag(FAR struct ekf_delay_s *d,
                        FAR const struct ekf_mag_sample_s *sample);

/* The absolute time the filter is allowed to advance to: now_us minus the
 * horizon, saturating at zero rather than wrapping. now_us < horizon happens
 * for the first few milliseconds after boot, and an unsigned wrap there would
 * produce a horizon far in the future and drain the ring in one go.
 */

uint64_t ekf_delay_horizon_time(FAR const struct ekf_delay_s *d,
                                uint64_t now_us);

/* Copy the oldest unconsumed IMU sample into out and mark it consumed, if its
 * timestamp_sample is at or before horizon_time. Returns false when there is
 * nothing due, leaving out untouched.
 */

bool ekf_delay_next_imu(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                        FAR struct ekf_imu_sample_s *out);

/* Pop the oldest measurement whose sample time is at or before horizon_time.
 * Measurements older than max_age_us before horizon_time are DISCARDED rather
 * than returned: the filter has already propagated past them, so applying one
 * would correct the wrong point on the trajectory.
 */

bool ekf_delay_next_baro(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                         uint64_t max_age_us,
                         FAR struct ekf_baro_sample_s *out);
bool ekf_delay_next_mag(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                        uint64_t max_age_us,
                        FAR struct ekf_mag_sample_s *out);
bool ekf_delay_push_extnav(FAR struct ekf_delay_s *d,
                           FAR const struct ekf_extnav_sample_s *sample);
bool ekf_delay_next_extnav(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                           uint64_t max_age_us,
                           FAR struct ekf_extnav_sample_s *out);

/* The samples the filter has not consumed: what the output predictor replays
 * to get from the horizon state to the present. index 0 is the oldest.
 */

uint16_t ekf_delay_output_count(FAR const struct ekf_delay_s *d);
FAR const struct ekf_imu_sample_s *
  ekf_delay_output_at(FAR const struct ekf_delay_s *d, uint16_t index);

#endif /* __APPS_EKF3_EKF_DELAY_H */
