/****************************************************************************
 * apps/ekf3/ekf_core.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_EKF3_EKF_CORE_H
#define __APPS_EKF3_EKF_CORE_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

#define EKF_STATE_DIM             15
#define EKF_COVARIANCE_INTERVAL    4

#define EKF_SOLUTION_ATTITUDE     (1u << 0)
#define EKF_SOLUTION_YAW_RELATIVE (1u << 1)

#define EKF_P_INDEX(row, column) \
  ((row) * EKF_STATE_DIM + (column))

enum ekf_process_result_e
{
  EKF_PROCESS_REJECTED = -1,
  EKF_PROCESS_ALIGNING = 0,
  EKF_PROCESS_PREDICTED = 1,
  EKF_PROCESS_INITIALIZED = 2
};

struct ekf_imu_sample_s
{
  uint64_t timestamp_sample;
  uint64_t timestamp_first;
  float delta_angle[3];
  float delta_velocity[3];
  float delta_angle_dt;
  float delta_velocity_dt;
  uint16_t samples;
  uint16_t reset_counter;
  uint8_t instance;
  uint8_t clipping;
  bool accel_calibrated;
  bool gyro_calibrated;
};

/* Error-state ordering: attitude, velocity, position, gyro bias, accel bias.
 * The quaternion and other physical quantities below are the nominal state;
 * covariance tracks the 15 small errors around that nominal trajectory.
 */

struct ekf_core_s
{
  float quaternion[4];
  float velocity[3];
  float position[3];
  float gyro_bias[3];
  float accel_bias[3];
  float covariance[EKF_STATE_DIM * EKF_STATE_DIM];

  float align_accel_sum[3];
  float align_gyro_sum[3];
  float align_accel_norm2_sum;
  float align_gyro_norm2_sum;
  float align_time_s;
  uint32_t align_samples;

  float covariance_delta_angle[3];
  float covariance_delta_velocity[3];
  float covariance_dt;
  uint8_t covariance_phase;
  uint8_t covariance_clipping;

  uint64_t last_timestamp_sample;
  uint64_t first_predict_timestamp;
  uint16_t source_reset_counter;
  uint16_t reset_counter;
  bool have_source_reset;
  bool initialized;

  uint32_t input_count;
  uint32_t predict_count;
  uint32_t covariance_count;
  uint32_t rejected_count;
  uint32_t uncalibrated_count;
  uint32_t clipping_count;
  uint32_t duplicate_count;
  uint32_t backward_count;
  uint32_t gap_count;
  uint32_t source_reset_count;
  uint32_t numerical_reset_count;
  uint32_t alignment_restart_count;
};

void ekf_core_init(FAR struct ekf_core_s *ekf);
int ekf_core_process(FAR struct ekf_core_s *ekf,
                     FAR const struct ekf_imu_sample_s *sample);
uint8_t ekf_core_solution_status(FAR const struct ekf_core_s *ekf);
void ekf_core_euler(FAR const struct ekf_core_s *ekf, FAR float euler[3]);

#endif /* __APPS_EKF3_EKF_CORE_H */
