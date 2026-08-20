/****************************************************************************
 * apps/imu_delta/imu_integrator.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_IMU_DELTA_IMU_INTEGRATOR_H
#define __APPS_IMU_DELTA_IMU_INTEGRATOR_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

#define IMU_DELTA_TARGET_US 2500u
#define IMU_DELTA_MIN_DT_US  250u
#define IMU_DELTA_MAX_DT_US 1000u

struct imu_delta_output_s
{
  uint64_t first_timestamp_us;
  uint64_t last_timestamp_us;
  float    delta_angle[3];
  float    delta_velocity[3];
  float    delta_angle_dt;
  float    delta_velocity_dt;
  uint16_t samples;
  uint8_t  clipping;
};

struct imu_integrator_s
{
  uint64_t last_timestamp_us;
  uint64_t first_timestamp_us;
  uint64_t integration_us;
  float    previous_accel[3];
  float    previous_gyro[3];
  float    quaternion[4];
  float    delta_velocity[3];
  uint32_t packets;
  uint32_t resets;
  uint32_t gaps;
  uint32_t duplicates;
  uint32_t backwards;
  uint32_t invalid;
  uint16_t samples;
  uint8_t  clipping;
  bool     seeded;
};

/* Return 1 when output contains a complete packet, zero when the sample was
 * accepted but the window is incomplete, and -1 when the sample was rejected.
 */

void imu_integrator_init(FAR struct imu_integrator_s *integrator);
int imu_integrator_add(FAR struct imu_integrator_s *integrator,
                       uint64_t timestamp_us, FAR const float accel[3],
                       FAR const float gyro[3], uint8_t clipping,
                       FAR struct imu_delta_output_s *output);

#endif /* __APPS_IMU_DELTA_IMU_INTEGRATOR_H */
