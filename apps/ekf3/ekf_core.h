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

  float dynamics_accel_mean[3];
  float dynamics_gyro_mean[3];
  float dynamics_accel_variance[3];
  float dynamics_gyro_variance[3];
  float low_dynamics_dwell_s;
  bool dynamics_seeded;
  bool low_dynamics;

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
  uint32_t low_dynamics_entry_count;
  uint32_t low_dynamics_exit_count;
  uint32_t gravity_accept_count;
  uint32_t gravity_reject_count;
  uint32_t gravity_yaw_projection_count;
  uint32_t bias_limit_count;
  float last_gravity_nis;
  float last_gravity_yaw_suppressed;
  float max_gravity_yaw_suppressed;
};

/* Current-time state, re-propagated from the delayed filter state.
 *
 * Deliberately small: quaternion, velocity, position and nothing else. The
 * alternative - copying the whole ~1.2 kB ekf_core_s and propagating that -
 * would move covariance the predictor never touches, 400 times a second.
 */

struct ekf_output_s
{
  float    quaternion[4];
  float    velocity[3];
  float    position[3];
  uint64_t timestamp_sample;
  uint16_t samples_replayed;
  bool     valid;
};

void ekf_core_init(FAR struct ekf_core_s *ekf);

/* Replay count samples forward from the filter state.
 *
 * samples is an array of POINTERS because the source is a ring buffer and the
 * entries are not contiguous. With count == 0 the result is the filter state
 * itself, which is what makes a zero horizon inert at the publication end as
 * well as at the input end.
 *
 * Does not modify ekf. It is called on every publication; a predictor that
 * mutated the core would integrate the same samples twice.
 */

void ekf_core_output_predict(FAR const struct ekf_core_s *ekf,
                             FAR const struct ekf_imu_sample_s *const *samples,
                             uint16_t count,
                             FAR struct ekf_output_s *out);
int ekf_core_process(FAR struct ekf_core_s *ekf,
                     FAR const struct ekf_imu_sample_s *sample);
uint8_t ekf_core_solution_status(FAR const struct ekf_core_s *ekf);
void ekf_core_euler(FAR const struct ekf_core_s *ekf, FAR float euler[3]);

#endif /* __APPS_EKF3_EKF_CORE_H */
