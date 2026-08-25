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

/* Mirrors the ESTIMATOR_* bits in uorb_msgs.h. */

#define EKF_SOLUTION_ATTITUDE       (1u << 0)
#define EKF_SOLUTION_YAW_RELATIVE   (1u << 1)
#define EKF_SOLUTION_YAW_ABSOLUTE   (1u << 2)
#define EKF_SOLUTION_VELOCITY_HORIZ (1u << 3)
#define EKF_SOLUTION_VELOCITY_VERT  (1u << 4)
#define EKF_SOLUTION_POSITION_HORIZ (1u << 5)
#define EKF_SOLUTION_POSITION_VERT  (1u << 6)

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

  float align_mag_sum[3];
  uint32_t align_mag_samples;
  float align_declination;     /* rad, EK3_MAG_DEC at start */
  float align_yaw_variance;    /* rad^2, EK3_YAW_M_NSE^2; 0 leaves 180 deg */

  float last_mag_heading;      /* rad, true, last accepted measurement */
  float last_mag_nis;
  float last_mag_field;        /* Gauss, magnitude of the last sample */
  uint64_t last_mag_timestamp; /* filter time of the last accepted fusion */
  bool  yaw_absolute;          /* heading is north-referenced */

  uint32_t mag_accept_count;
  uint32_t mag_reject_count;
  uint32_t mag_consecutive_rejects;
  uint32_t mag_unhealthy_count;

  float baro_reference_hpa;
  float last_baro_height;
  float last_baro_nis;
  bool  baro_have_reference;

  uint32_t baro_accept_count;
  uint32_t baro_reject_count;
  uint32_t baro_consecutive_rejects;

  bool     extnav_datum_set;
  bool     have_extnav_reset;
  uint8_t  extnav_source_reset;    /* last seen source reset generation */
  float    last_extnav_innov[2];   /* m, x and y */
  float    last_extnav_nis[2];
  float    last_extnav_noise;      /* m, AFTER the floor was applied */
  uint64_t last_extnav_timestamp;  /* filter time of the last acceptance */
  uint32_t extnav_timeout_us;      /* EK3_EXT_TIMEOUT, via the setter */

  uint32_t extnav_accept_count;
  uint32_t extnav_reject_count;
  uint32_t extnav_consecutive_rejects;
  uint32_t extnav_datum_count;

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
  uint32_t commanded_reset_count;
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

/* Pressure sanity, and the rejection run that withdraws vertical validity.
 * Fixed rather than parameters: these are not tuning decisions, they are the
 * boundary between a reading and a fault.
 */

#define EKF_BARO_PRESSURE_MIN      500.0f
#define EKF_BARO_PRESSURE_MAX     1200.0f
#define EKF_BARO_REJECT_RUN_MAX      20u

/* Magnetometer health. The field band is wide enough to tolerate ordinary
 * vehicle-borne distortion and narrow enough to catch a magnet, a motor or a
 * failed sensor. Staleness is measured against the filter's own clock.
 */

#define EKF_MAG_FIELD_TOLERANCE     0.30f
#define EKF_MAG_MAX_AGE_US       500000ull
#define EKF_MAG_REJECT_RUN_MAX       20u

/* A rejection run this long means the filter and the source disagree about
 * where the vehicle IS, not that one reading was bad. Re-datum rather than
 * going on rejecting every pose for ever - ArduPilot's ResetPositionNE().
 */

#define EKF_EXTNAV_REJECT_RUN_MAX    20u

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

/* Dropout after which horizontal validity is withdrawn. A setter rather than
 * a parameter read inside the core, matching ekf_core_set_mag_config: the
 * core reads no parameters, which is what keeps it testable without one.
 */

void ekf_core_set_extnav_config(FAR struct ekf_core_s *ekf,
                                uint32_t timeout_us);

/* Fuse an absolute pose from the companion computer.
 *
 * Returns 1 accepted, 0 gated, -1 rejected as unusable, and -2 when this
 * sample BECAME the datum and the filter was set rather than corrected.
 *
 * pos_noise_floor and yaw_noise_floor are FLOORS under whatever the source
 * reported, not defaults - the fused noise is the larger of the two. A
 * source claiming millimetre accuracy must not be able to talk the filter
 * into trusting it more than the operator configured. ArduPilot does the
 * same with posErr.
 */

int ekf_core_fuse_extnav(FAR struct ekf_core_s *ekf,
                         FAR const struct ekf_extnav_sample_s *s,
                         float pos_noise_floor, float pos_gate,
                         float yaw_noise_floor, float yaw_gate,
                         bool want_position, bool want_yaw);

/* Tilt-compensated magnetic heading, radians, from a body-frame field.
 *
 * Uses roll and pitch ONLY - never the current yaw, which is the quantity
 * being measured. Returns false when the field is not finite or its
 * horizontal projection is too short to give a direction, which is what
 * happens when the field is nearly parallel to the tilt axis.
 */

bool ekf_mag_heading(FAR const float quaternion[4],
                     FAR const float field[3], float declination,
                     FAR float *heading);

/* Declination and initial yaw uncertainty used at alignment. The core reads
 * no parameters itself, so the daemon hands these in once at start. Survives
 * a re-alignment, because it is configuration rather than state.
 */

void ekf_core_set_mag_config(FAR struct ekf_core_s *ekf, float declination,
                             float yaw_variance);

/* Accumulate a body-frame field for heading initialisation. Ignored once the
 * filter is aligned; the running estimate is maintained by fusion instead.
 */

void ekf_core_add_align_mag(FAR struct ekf_core_s *ekf,
                            FAR const float field[3]);

/* Fuse a body-frame field as a heading observation.
 *
 * Returns 1 accepted, 0 gated, -1 unhealthy or numerically refused, and -2
 * when the filter's heading has no absolute datum - alignment never got a
 * usable field - in which case fusing would be a yaw JUMP rather than a
 * correction, so it is refused.
 *
 * expected_field is CAL_MAG0_FIELD; pass <= 0 to skip the magnitude check.
 */

int ekf_core_fuse_mag(FAR struct ekf_core_s *ekf,
                      FAR const float field[3], float declination,
                      float expected_field, float noise,
                      float gate_sigma);

/* Height above the reference pressure, ISA. Positive is UP. */

float ekf_baro_height(float pressure_hpa, float reference_hpa);

/* Fuse a barometric height. Returns 1 accepted, 0 gated, -1 numerical
 * failure or an insane pressure, and -2 when there is no reference yet - in
 * which case this sample BECOMES the reference and the filter is left alone.
 */

int ekf_core_fuse_baro(FAR struct ekf_core_s *ekf, float pressure_hpa,
                       float noise, float gate_sigma);

void ekf_core_init(FAR struct ekf_core_s *ekf);
void ekf_core_reset(FAR struct ekf_core_s *ekf);

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

#ifdef EKF_CORE_HOST_TEST

/* Test access to the static measurement updates. Compiled only for the host
 * test - the firmware never sees these. The alternative, making the updates
 * non-static, would widen the interface permanently for a test-only need.
 */

int ekf_core_test_update_1d(FAR struct ekf_core_s *ekf,
                            FAR const float h[EKF_STATE_DIM],
                            float residual, float noise_variance,
                            float gate_sigma, FAR float *nis);
int ekf_core_test_update_3d(FAR struct ekf_core_s *ekf,
                            FAR const float h[3][EKF_STATE_DIM],
                            FAR const float residual[3],
                            float noise_variance, FAR float *nis);
#endif

#endif /* __APPS_EKF3_EKF_CORE_H */
