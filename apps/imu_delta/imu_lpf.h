/****************************************************************************
 * apps/imu_delta/imu_lpf.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_IMU_DELTA_IMU_LPF_H
#define __APPS_IMU_DELTA_IMU_LPF_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

/* One native-rate, three-axis, second-order Butterworth low-pass.  Accel and
 * gyro use separate instances configured with the same coefficients so their
 * phase delay remains matched before coning/sculling integration.
 */

struct imu_lpf3_s
{
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
  float state[3][2];
  float sample_rate_hz;
  float cutoff_hz;
  uint32_t group_delay_us; /* low-frequency delay used for sample time */
  bool enabled;
};

bool imu_lpf3_configure(FAR struct imu_lpf3_s *filter,
                        float sample_rate_hz, float cutoff_hz);
void imu_lpf3_reset(FAR struct imu_lpf3_s *filter,
                    FAR const float value[3]);
bool imu_lpf3_apply(FAR struct imu_lpf3_s *filter, FAR float value[3]);

uint64_t imu_lpf_compensate_timestamp(uint64_t timestamp_us,
                                      uint32_t group_delay_us);

#endif /* __APPS_IMU_DELTA_IMU_LPF_H */
