/****************************************************************************
 * apps/imu_delta/imu_delta.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_IMU_DELTA_IMU_DELTA_H
#define __APPS_IMU_DELTA_IMU_DELTA_H

#include <stdbool.h>
#include <stdint.h>

struct imu_delta_status_s
{
  uint64_t first_packet_us;
  uint64_t last_packet_us;
  uint64_t total_window_us;
  float    total_delta_angle[3];
  float    total_delta_velocity[3];
  uint32_t packets;
  uint32_t accel_samples;
  uint32_t gyro_samples;
  uint32_t paired_samples;
  uint32_t sync_drops;
  uint32_t resample_drops;
  uint32_t queue_overruns;
  uint32_t publish_errors;
  uint32_t clipped_packets;
  uint32_t resets;
  uint32_t gaps;
  uint32_t duplicates;
  uint32_t backwards;
  uint32_t invalid;
  uint32_t lpf_resets;
  uint32_t lpf_invalid;
  uint32_t min_window_us;
  uint32_t max_window_us;
  uint32_t lpf_delay_us;
  float    lpf_hz;
  uint16_t min_samples;
  uint16_t max_samples;
  uint8_t  sensor_rotation;
  uint8_t  board_rotation;
  uint8_t  instance;
  bool     accel_calibrated;
  bool     gyro_calibrated;
  bool     running;
};

/* Instance 0 is the primary IMU and feeds the estimator; instance 1 is the
 * secondary and feeds the monitor lane.
 */

#define IMU_DELTA_INSTANCES 2

int imu_delta_start(int instance);
int imu_delta_stop(int instance);
void imu_delta_status(int instance,
                      struct imu_delta_status_s *status);

#endif /* __APPS_IMU_DELTA_IMU_DELTA_H */
