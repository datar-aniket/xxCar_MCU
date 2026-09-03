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
#define EKF_WHEEL_QUEUE_SIZE     16
/* 100 ms at a 240 Hz motion-capture rate is 24 poses. 32 leaves room for
 * arrival jitter and task scheduling. The old four-entry queue represented
 * only 40 ms at 100 Hz, so EK3_DELAY_MS=100 overwrote every pose before the
 * delayed horizon could reach it.
 */

#define EKF_EXTNAV_QUEUE_SIZE    32

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

/* VESC longitudinal velocity after conversion to m/s. accel_mps2 is the
 * first-order-filtered derivative at this same sample time and is retained
 * with the measurement so the delayed slip gate compares like with like.
 */

struct ekf_wheel_sample_s
{
  uint64_t timestamp_sample;
  float    speed_mps;
  float    accel_mps2;
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

  struct ekf_wheel_sample_s wheel[EKF_WHEEL_QUEUE_SIZE];
  uint16_t wheel_head;
  uint16_t wheel_count;

  uint32_t horizon_us;

  uint32_t imu_overflow_count;   /* unconsumed sample overwritten */
  uint32_t baro_overflow_count;
  uint32_t mag_overflow_count;
  uint32_t wheel_overflow_count;

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
bool ekf_delay_push_wheel(FAR struct ekf_delay_s *d,
                          FAR const struct ekf_wheel_sample_s *sample);

/* The absolute time the filter is allowed to advance to: the newest IMU sample
 * time minus the horizon, saturating at zero rather than wrapping.  Using the
 * sample time (rather than task wake-up time) keeps scheduling latency out of
 * the fusion timeline.  A sample time below the horizon occurs for the first
 * few milliseconds after boot; unsigned wrap there would otherwise produce a
 * horizon far in the future and drain the ring in one go.
 */

uint64_t ekf_delay_horizon_time(FAR const struct ekf_delay_s *d,
                                uint64_t newest_imu_time_us);

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
bool ekf_delay_next_wheel(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                          uint64_t max_age_us,
                          FAR struct ekf_wheel_sample_s *out);
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
