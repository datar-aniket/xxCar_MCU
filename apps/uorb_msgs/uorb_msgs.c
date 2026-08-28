/****************************************************************************
 * apps/uorb_msgs/uorb_msgs.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Definitions of the optical_flow and distance_sensor uORB topics. See
 * uorb_msgs.h.
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <errno.h>
#include <assert.h>
#include <stddef.h>
#include <limits.h>

#include "uorb_msgs.h"

/****************************************************************************
 * Layout checks
 *
 * uORB decodes a topic by walking o_format and advancing the read offset by
 * each conversion's size, with no realignment. A stray padding byte would shift
 * every later field and print convincing nonsense, so make the compiler prove
 * the fields sit exactly where the format strings expect.
 ****************************************************************************/

/****************************************************************************
 * Topic-name length
 *
 * uORB registers a topic by building "/dev/uorb/<name><instance>" and copying
 * that WHOLE PATH into struct sensor_reginfo_s, whose path field is only
 * NAME_MAX bytes (nuttx/uorb.h) - while the string it is copied from is
 * ORB_PATH_MAX = NAME_MAX + 16 (nuttx-apps system/uorb/uORB/uORB.c:79). A name
 * that overruns is silently TRUNCATED, so the node is registered under a
 * shortened path and the open() that immediately follows - using the full path
 * - fails. orb_advertise() then returns -1 with nothing to say why.
 *
 * That cost a flash cycle: vehicle_angular_velocity produced a 35-character
 * path against CONFIG_NAME_MAX=32 and `sensors start` reported only "cannot
 * advertise". vehicle_acceleration, at 31, had fitted by a single byte.
 *
 * So the budget is checked here instead. With CONFIG_NAME_MAX=32 a topic name
 * may be 20 characters: 32 - strlen("/dev/uorb/") - 1 instance digit - NUL.
 ****************************************************************************/

#define ORB_NAME_FITS(name)                                                   \
  static_assert(sizeof(ORB_SENSOR_PATH name "0") <= NAME_MAX,                 \
                "uORB topic name too long: /dev/uorb/<name><instance> must "  \
                "fit in NAME_MAX, or the node is registered truncated and "   \
                "orb_advertise() fails at runtime")

ORB_NAME_FITS("optical_flow");
ORB_NAME_FITS("distance_sensor");
ORB_NAME_FITS("vehicle_accel");
ORB_NAME_FITS("vehicle_gyro");
ORB_NAME_FITS("vehicle_mag");
ORB_NAME_FITS("vehicle_baro");
ORB_NAME_FITS("external_pose");
ORB_NAME_FITS("vehicle_state_tx");
ORB_NAME_FITS("vesc_status");
ORB_NAME_FITS("actuator_command");
ORB_NAME_FITS("control_cmd");
ORB_NAME_FITS("vehicle_imu");
ORB_NAME_FITS("estimator_state");
ORB_NAME_FITS("estimator_diag");

static_assert(offsetof(struct optical_flow_s, timestamp)             ==  0, "layout");
static_assert(offsetof(struct optical_flow_s, integration_time_us)   ==  8, "layout");
static_assert(offsetof(struct optical_flow_s, time_delta_distance_us)== 12, "layout");
static_assert(offsetof(struct optical_flow_s, integrated_x)          == 16, "layout");
static_assert(offsetof(struct optical_flow_s, distance)              == 36, "layout");
static_assert(offsetof(struct optical_flow_s, temperature)           == 40, "layout");
static_assert(offsetof(struct optical_flow_s, quality)               == 42, "layout");
static_assert(offsetof(struct optical_flow_s, sensor_id)             == 43, "layout");
static_assert(sizeof(struct optical_flow_s)                          == 48, "layout");

static_assert(offsetof(struct distance_sensor_s, timestamp)          ==  0, "layout");
static_assert(offsetof(struct distance_sensor_s, current_distance)   ==  8, "layout");
static_assert(offsetof(struct distance_sensor_s, min_distance)       == 12, "layout");
static_assert(offsetof(struct distance_sensor_s, max_distance)       == 16, "layout");
static_assert(offsetof(struct distance_sensor_s, type)               == 20, "layout");
static_assert(offsetof(struct distance_sensor_s, signal_quality)     == 23, "layout");
static_assert(sizeof(struct distance_sensor_s)                       == 24, "layout");

static_assert(offsetof(struct vehicle_accel_s, timestamp)        ==  0, "layout");
static_assert(offsetof(struct vehicle_accel_s, timestamp_sample) ==  8, "layout");
static_assert(offsetof(struct vehicle_accel_s, x)                == 16, "layout");
static_assert(offsetof(struct vehicle_accel_s, y)                == 20, "layout");
static_assert(offsetof(struct vehicle_accel_s, z)                == 24, "layout");
static_assert(offsetof(struct vehicle_accel_s, instance)         == 28, "layout");
static_assert(offsetof(struct vehicle_accel_s, calibrated)       == 29, "layout");
static_assert(sizeof(struct vehicle_accel_s)                     == 32, "layout");

/* The two are deliberately the same shape, and the code that fills them relies
 * on that only through the field names - never by casting one to the other.
 */

static_assert(offsetof(struct vehicle_gyro_s, timestamp)        ==  0, "layout");
static_assert(offsetof(struct vehicle_gyro_s, timestamp_sample) ==  8, "layout");
static_assert(offsetof(struct vehicle_gyro_s, x)                == 16, "layout");
static_assert(offsetof(struct vehicle_gyro_s, y)                == 20, "layout");
static_assert(offsetof(struct vehicle_gyro_s, z)                == 24, "layout");
static_assert(offsetof(struct vehicle_gyro_s, instance)         == 28, "layout");
static_assert(offsetof(struct vehicle_gyro_s, calibrated)       == 29, "layout");
static_assert(sizeof(struct vehicle_gyro_s)                     == 32, "layout");

static_assert(offsetof(struct vehicle_mag_s, timestamp)        ==  0, "layout");
static_assert(offsetof(struct vehicle_mag_s, timestamp_sample) ==  8, "layout");
static_assert(offsetof(struct vehicle_mag_s, field)            == 16, "layout");
static_assert(offsetof(struct vehicle_mag_s, temperature)      == 28, "layout");
static_assert(offsetof(struct vehicle_mag_s, calibrated)       == 32, "layout");
static_assert(offsetof(struct vehicle_mag_s, instance)         == 33, "layout");
static_assert(sizeof(struct vehicle_mag_s)                     == 40, "layout");

static_assert(offsetof(struct vehicle_baro_s, timestamp)        ==  0, "layout");
static_assert(offsetof(struct vehicle_baro_s, timestamp_sample) ==  8, "layout");
static_assert(offsetof(struct vehicle_baro_s, pressure)         == 16, "layout");
static_assert(offsetof(struct vehicle_baro_s, temperature)      == 20, "layout");
static_assert(sizeof(struct vehicle_baro_s)                     == 24, "layout");

static_assert(offsetof(struct external_pose_s, timestamp)        ==  0, "layout");
static_assert(offsetof(struct external_pose_s, timestamp_sample) ==  8, "layout");
static_assert(offsetof(struct external_pose_s, x)                == 16, "layout");
static_assert(offsetof(struct external_pose_s, y)                == 20, "layout");
static_assert(offsetof(struct external_pose_s, yaw)              == 24, "layout");
static_assert(offsetof(struct external_pose_s, cov)              == 28, "layout");
static_assert(offsetof(struct external_pose_s, flags)            == 52, "layout");
static_assert(offsetof(struct external_pose_s, reset_counter)    == 53, "layout");
static_assert(sizeof(struct external_pose_s)                     == 56, "layout");

static_assert(offsetof(struct vehicle_state_tx_s, timestamp) == 0, "layout");
static_assert(offsetof(struct vehicle_state_tx_s, timestamp_sample) == 8,
              "layout");
static_assert(offsetof(struct vehicle_state_tx_s, accel_timestamp_sample) == 16,
              "layout");
static_assert(offsetof(struct vehicle_state_tx_s, wire_timestamp_us) == 24,
              "layout");
static_assert(offsetof(struct vehicle_state_tx_s, position) == 32, "layout");
static_assert(offsetof(struct vehicle_state_tx_s, quaternion) == 44, "layout");
static_assert(offsetof(struct vehicle_state_tx_s, velocity) == 60, "layout");
static_assert(offsetof(struct vehicle_state_tx_s, angular_velocity) == 72,
              "layout");
static_assert(offsetof(struct vehicle_state_tx_s, side_slip_rad) == 84,
              "layout");
static_assert(offsetof(struct vehicle_state_tx_s, accel) == 88, "layout");
static_assert(offsetof(struct vehicle_state_tx_s, wheel_torque_nm) == 100,
              "layout");
static_assert(offsetof(struct vehicle_state_tx_s, solution_status) == 112,
              "layout");
static_assert(sizeof(struct vehicle_state_tx_s) == 120, "layout");
static_assert(VEHICLE_STATE_TX_QUEUE_SIZE >= 32u,
              "vehicle state TX queue must absorb ordinary SD stalls");

static_assert(offsetof(struct vesc_status_s, timestamp)        ==  0, "layout");
static_assert(offsetof(struct vesc_status_s, timestamp_sample) ==  8, "layout");
static_assert(offsetof(struct vesc_status_s, tachometer)       == 16, "layout");
static_assert(offsetof(struct vesc_status_s, current_a)        == 20, "layout");
static_assert(offsetof(struct vesc_status_s, adc_volts)        == 24, "layout");
static_assert(offsetof(struct vesc_status_s, controller_id)    == 28, "layout");
static_assert(offsetof(struct vesc_status_s, speed_cps)        == 32, "layout");
static_assert(sizeof(struct vesc_status_s)                     == 40, "layout");

static_assert(offsetof(struct actuator_command_s, timestamp) ==  0, "layout");
static_assert(offsetof(struct actuator_command_s, motor)     ==  8, "layout");
static_assert(offsetof(struct actuator_command_s, steering)  == 12, "layout");
static_assert(offsetof(struct actuator_command_s, mode)      == 16, "layout");
static_assert(sizeof(struct actuator_command_s)              == 24, "layout");

static_assert(offsetof(struct control_cmd_s, timestamp) ==  0, "layout");
static_assert(offsetof(struct control_cmd_s, motor)     ==  8, "layout");
static_assert(offsetof(struct control_cmd_s, steering)  == 12, "layout");
static_assert(offsetof(struct control_cmd_s, mode)      == 16, "layout");
static_assert(sizeof(struct control_cmd_s)              == 24, "layout");

static_assert(offsetof(struct vehicle_imu_s, timestamp)         ==  0, "layout");
static_assert(offsetof(struct vehicle_imu_s, timestamp_sample)  ==  8, "layout");
static_assert(offsetof(struct vehicle_imu_s, timestamp_first)   == 16, "layout");
static_assert(offsetof(struct vehicle_imu_s, delta_angle)       == 24, "layout");
static_assert(offsetof(struct vehicle_imu_s, delta_velocity)    == 36, "layout");
static_assert(offsetof(struct vehicle_imu_s, delta_angle_dt)    == 48, "layout");
static_assert(offsetof(struct vehicle_imu_s, samples)           == 56, "layout");
static_assert(offsetof(struct vehicle_imu_s, instance)          == 60, "layout");
static_assert(sizeof(struct vehicle_imu_s)                      == 64, "layout");
static_assert(VEHICLE_IMU_QUEUE_SIZE >= 4u,
              "vehicle_imu must be queued; delta packets cannot be overwritten");

static_assert(offsetof(struct estimator_state_s, timestamp)          ==   0, "layout");
static_assert(offsetof(struct estimator_state_s, timestamp_sample)   ==   8, "layout");
static_assert(offsetof(struct estimator_state_s, quaternion)         ==  16, "layout");
static_assert(offsetof(struct estimator_state_s, velocity)           ==  32, "layout");
static_assert(offsetof(struct estimator_state_s, position)           ==  44, "layout");
static_assert(offsetof(struct estimator_state_s, gyro_bias)          ==  56, "layout");
static_assert(offsetof(struct estimator_state_s, accel_bias)         ==  68, "layout");
static_assert(offsetof(struct estimator_state_s, angle_variance)     ==  80, "layout");
static_assert(offsetof(struct estimator_state_s, velocity_variance)  ==  92, "layout");
static_assert(offsetof(struct estimator_state_s, position_variance)  == 104, "layout");
static_assert(offsetof(struct estimator_state_s, predict_count)      == 116, "layout");
static_assert(offsetof(struct estimator_state_s, covariance_count)   == 120, "layout");
static_assert(offsetof(struct estimator_state_s, reset_counter)      == 124, "layout");
static_assert(offsetof(struct estimator_state_s, solution_status)    == 126, "layout");
static_assert(sizeof(struct estimator_state_s)                       == 128, "layout");

static_assert(offsetof(struct estimator_diag_s, timestamp)            ==   0, "layout");
static_assert(offsetof(struct estimator_diag_s, timestamp_sample)     ==   8, "layout");
static_assert(offsetof(struct estimator_diag_s, extnav_timestamp)     ==  16, "layout");
static_assert(offsetof(struct estimator_diag_s, specific_force)       ==  24, "layout");
static_assert(offsetof(struct estimator_diag_s, corrected_force)      ==  36, "layout");
static_assert(offsetof(struct estimator_diag_s, gravity_body)         ==  48, "layout");
static_assert(offsetof(struct estimator_diag_s, residual_accel_body)  ==  60, "layout");
static_assert(offsetof(struct estimator_diag_s, nav_accel)            ==  72, "layout");
static_assert(offsetof(struct estimator_diag_s, quaternion)           ==  84, "layout");
static_assert(offsetof(struct estimator_diag_s, velocity)             == 100, "layout");
static_assert(offsetof(struct estimator_diag_s, position)             == 112, "layout");
static_assert(offsetof(struct estimator_diag_s, gyro_bias)            == 124, "layout");
static_assert(offsetof(struct estimator_diag_s, accel_bias)           == 136, "layout");
static_assert(offsetof(struct estimator_diag_s, extnav_innov)         == 148, "layout");
static_assert(offsetof(struct estimator_diag_s, extnav_nis)           == 156, "layout");
static_assert(offsetof(struct estimator_diag_s, extnav_measurement)   == 164, "layout");
static_assert(offsetof(struct estimator_diag_s, zupt_nis)             == 176, "layout");
static_assert(offsetof(struct estimator_diag_s, gravity_nis)          == 188, "layout");
static_assert(offsetof(struct estimator_diag_s, extnav_test_ratio)    == 204, "layout");
static_assert(offsetof(struct estimator_diag_s, wheel_speed_cps)      == 208, "layout");
static_assert(offsetof(struct estimator_diag_s, extnav_accept_count)  == 212, "layout");
static_assert(offsetof(struct estimator_diag_s, reset_counter)        == 236, "layout");
static_assert(offsetof(struct estimator_diag_s, flags)                == 238, "layout");
static_assert(offsetof(struct estimator_diag_s, instance)             == 240, "layout");
static_assert(sizeof(struct estimator_diag_s)                         == 248, "layout");
static_assert(ESTIMATOR_DIAG_QUEUE_SIZE >= 64u,
              "estimator_diag queue must absorb ordinary SD stalls");

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_DEBUG_UORB
static const char optical_flow_format[] =
  "timestamp:%" PRIu64
  ",integration_time_us:%" PRIu32
  ",time_delta_distance_us:%" PRIu32
  ",integrated_x:%hf,integrated_y:%hf"
  ",integrated_xgyro:%hf,integrated_ygyro:%hf,integrated_zgyro:%hf"
  ",distance:%hf,temperature:%hi"
  ",quality:%hhu,sensor_id:%hhu";

static const char distance_sensor_format[] =
  "timestamp:%" PRIu64
  ",current_distance:%hf,min_distance:%hf,max_distance:%hf"
  ",type:%hhu,orientation:%hhu,covariance:%hhu,signal_quality:%hhu";

static const char vehicle_accel_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",extnav_timestamp:%" PRIu64
  ",x:%hf,y:%hf,z:%hf"
  ",instance:%hhu,calibrated:%hhu";

static const char vehicle_gyro_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",x:%hf,y:%hf,z:%hf"
  ",instance:%hhu,calibrated:%hhu";

static const char vehicle_mag_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",field[0]:%hf,field[1]:%hf,field[2]:%hf"
  ",temperature:%hf"
  ",calibrated:%hhu,instance:%hhu";

static const char vehicle_baro_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",pressure:%hf,temperature:%hf";

static const char external_pose_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",x:%hf,y:%hf,yaw:%hf"
  ",cov[0]:%hf,cov[1]:%hf,cov[2]:%hf"
  ",cov[3]:%hf,cov[4]:%hf,cov[5]:%hf"
  ",flags:%hhu,reset_counter:%hhu";

static const char vehicle_state_tx_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",accel_timestamp_sample:%" PRIu64
  ",wire_timestamp_us:%" PRIu64
  ",position[0]:%hf,position[1]:%hf,position[2]:%hf"
  ",quaternion[0]:%hf,quaternion[1]:%hf"
  ",quaternion[2]:%hf,quaternion[3]:%hf"
  ",velocity[0]:%hf,velocity[1]:%hf,velocity[2]:%hf"
  ",angular_velocity[0]:%hf,angular_velocity[1]:%hf"
  ",angular_velocity[2]:%hf,side_slip_rad:%hf"
  ",accel[0]:%hf,accel[1]:%hf,accel[2]:%hf"
  ",wheel_torque_nm:%hf,steering_angle:%hf,motor_speed_ms:%hf"
  ",solution_status:%hhu,reset_counter:%hhu,source_valid:%hhu";

static const char vesc_status_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",tachometer:%" PRIi32
  ",current_a:%hf,adc_volts:%hf"
  ",controller_id:%hhu"
  ",speed_cps:%hf";

static const char actuator_command_format[] =
  "timestamp:%" PRIu64
  ",motor:%hf,steering:%hf"
  ",mode:%hhu";

static const char control_cmd_format[] =
  "timestamp:%" PRIu64
  ",motor:%hf,steering:%hf"
  ",mode:%hhu";

static const char vehicle_imu_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",timestamp_first:%" PRIu64
  ",delta_angle[0]:%hf,delta_angle[1]:%hf,delta_angle[2]:%hf"
  ",delta_velocity[0]:%hf,delta_velocity[1]:%hf,delta_velocity[2]:%hf"
  ",delta_angle_dt:%hf,delta_velocity_dt:%hf"
  ",samples:%hu,reset_counter:%hu"
  ",instance:%hhu,clipping:%hhu"
  ",accel_calibrated:%hhu,gyro_calibrated:%hhu";

static const char estimator_state_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",quaternion[0]:%hf,quaternion[1]:%hf"
  ",quaternion[2]:%hf,quaternion[3]:%hf"
  ",velocity[0]:%hf,velocity[1]:%hf,velocity[2]:%hf"
  ",position[0]:%hf,position[1]:%hf,position[2]:%hf"
  ",gyro_bias[0]:%hf,gyro_bias[1]:%hf,gyro_bias[2]:%hf"
  ",accel_bias[0]:%hf,accel_bias[1]:%hf,accel_bias[2]:%hf"
  ",angle_variance[0]:%hf,angle_variance[1]:%hf"
  ",angle_variance[2]:%hf"
  ",velocity_variance[0]:%hf,velocity_variance[1]:%hf"
  ",velocity_variance[2]:%hf"
  ",position_variance[0]:%hf,position_variance[1]:%hf"
  ",position_variance[2]:%hf"
  ",predict_count:%" PRIu32 ",covariance_count:%" PRIu32
  ",reset_counter:%hu,solution_status:%hhu,instance:%hhu";

static const char estimator_diag_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",specific_force[0]:%hf,specific_force[1]:%hf,specific_force[2]:%hf"
  ",corrected_force[0]:%hf,corrected_force[1]:%hf,corrected_force[2]:%hf"
  ",gravity_body[0]:%hf,gravity_body[1]:%hf,gravity_body[2]:%hf"
  ",residual_accel_body[0]:%hf,residual_accel_body[1]:%hf"
  ",residual_accel_body[2]:%hf"
  ",nav_accel[0]:%hf,nav_accel[1]:%hf,nav_accel[2]:%hf"
  ",quaternion[0]:%hf,quaternion[1]:%hf"
  ",quaternion[2]:%hf,quaternion[3]:%hf"
  ",velocity[0]:%hf,velocity[1]:%hf,velocity[2]:%hf"
  ",position[0]:%hf,position[1]:%hf,position[2]:%hf"
  ",gyro_bias[0]:%hf,gyro_bias[1]:%hf,gyro_bias[2]:%hf"
  ",accel_bias[0]:%hf,accel_bias[1]:%hf,accel_bias[2]:%hf"
  ",extnav_innov[0]:%hf,extnav_innov[1]:%hf"
  ",extnav_nis[0]:%hf,extnav_nis[1]:%hf"
  ",extnav_measurement[0]:%hf,extnav_measurement[1]:%hf"
  ",extnav_measurement[2]:%hf"
  ",zupt_nis[0]:%hf,zupt_nis[1]:%hf,zupt_nis[2]:%hf"
  ",gravity_nis:%hf,accel_norm:%hf,accel_variance:%hf"
  ",gravity_deviation:%hf,extnav_test_ratio:%hf,wheel_speed_cps:%hf"
  ",extnav_accept_count:%" PRIu32 ",extnav_reject_count:%" PRIu32
  ",zupt_accept_count:%" PRIu32 ",zupt_reject_count:%" PRIu32
  ",gravity_accept_count:%" PRIu32 ",gravity_reject_count:%" PRIu32
  ",reset_counter:%hu,flags:%hu,instance:%hhu";
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DEFINE(optical_flow, struct optical_flow_s, optical_flow_format);
ORB_DEFINE(distance_sensor, struct distance_sensor_s, distance_sensor_format);
ORB_DEFINE(vehicle_accel, struct vehicle_accel_s, vehicle_accel_format);
ORB_DEFINE(vehicle_gyro, struct vehicle_gyro_s, vehicle_gyro_format);
ORB_DEFINE(vehicle_mag, struct vehicle_mag_s, vehicle_mag_format);
ORB_DEFINE(vehicle_baro, struct vehicle_baro_s, vehicle_baro_format);
ORB_DEFINE(external_pose, struct external_pose_s, external_pose_format);
ORB_DEFINE(vehicle_state_tx, struct vehicle_state_tx_s,
           vehicle_state_tx_format);
ORB_DEFINE(vesc_status, struct vesc_status_s, vesc_status_format);
ORB_DEFINE(actuator_command, struct actuator_command_s,
           actuator_command_format);
ORB_DEFINE(control_cmd, struct control_cmd_s, control_cmd_format);
ORB_DEFINE(vehicle_imu, struct vehicle_imu_s, vehicle_imu_format);
ORB_DEFINE(estimator_state, struct estimator_state_s,
           estimator_state_format);
ORB_DEFINE(estimator_diag, struct estimator_diag_s,
           estimator_diag_format);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int optical_flow_advertise(void)
{
  return orb_advertise(ORB_ID(optical_flow), NULL);
}

int optical_flow_publish(int fd, FAR const struct optical_flow_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(optical_flow), fd, msg);
}

int distance_sensor_advertise(void)
{
  return orb_advertise(ORB_ID(distance_sensor), NULL);
}

int distance_sensor_publish(int fd, FAR const struct distance_sensor_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(distance_sensor), fd, msg);
}

int vehicle_accel_advertise(void)
{
  return orb_advertise(ORB_ID(vehicle_accel), NULL);
}

int vehicle_accel_publish(int fd, FAR const struct vehicle_accel_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(vehicle_accel), fd, msg);
}

int vehicle_gyro_advertise(void)
{
  return orb_advertise(ORB_ID(vehicle_gyro), NULL);
}

int vehicle_gyro_publish(int fd, FAR const struct vehicle_gyro_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(vehicle_gyro), fd, msg);
}

int vehicle_mag_advertise(void)
{
  return orb_advertise(ORB_ID(vehicle_mag), NULL);
}

int vehicle_mag_publish(int fd, FAR const struct vehicle_mag_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(vehicle_mag), fd, msg);
}

int vehicle_baro_advertise(void)
{
  return orb_advertise(ORB_ID(vehicle_baro), NULL);
}

int vehicle_baro_publish(int fd, FAR const struct vehicle_baro_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(vehicle_baro), fd, msg);
}

int external_pose_advertise(void)
{
  return orb_advertise(ORB_ID(external_pose), NULL);
}

int external_pose_publish(int fd, FAR const struct external_pose_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(external_pose), fd, msg);
}

int vehicle_state_tx_advertise(void)
{
  return orb_advertise_queue(ORB_ID(vehicle_state_tx), NULL,
                             VEHICLE_STATE_TX_QUEUE_SIZE);
}

int vehicle_state_tx_publish(int fd,
                             FAR const struct vehicle_state_tx_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(vehicle_state_tx), fd, msg);
}

int vesc_status_advertise(void)
{
  return orb_advertise(ORB_ID(vesc_status), NULL);
}

int vesc_status_publish(int fd, FAR const struct vesc_status_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(vesc_status), fd, msg);
}

int actuator_command_advertise(void)
{
  return orb_advertise(ORB_ID(actuator_command), NULL);
}

int actuator_command_publish(int fd,
                             FAR const struct actuator_command_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(actuator_command), fd, msg);
}

int control_cmd_advertise(void)
{
  return orb_advertise(ORB_ID(control_cmd), NULL);
}

int control_cmd_publish(int fd, FAR const struct control_cmd_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(control_cmd), fd, msg);
}

int vehicle_imu_advertise(int instance)
{
  /* Multi-instance: 0 is the primary IMU feeding the estimator, 1 the
   * secondary feeding the monitor lane.
   *
   * The instance is an INPUT here and is never written back -
   * orb_advertise_multi_queue_flags does `inst = instance ? *instance :
   * orb_group_count(meta)`, so passing a pointer requests that exact
   * instance. Checking it afterwards would be checking a value nothing
   * changed.
   */

  int want = instance;

  return orb_advertise_multi_queue(ORB_ID(vehicle_imu), NULL, &want,
                                   VEHICLE_IMU_QUEUE_SIZE);
}

int vehicle_imu_publish(int fd, FAR const struct vehicle_imu_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(vehicle_imu), fd, msg);
}

int estimator_state_advertise(void)
{
  return orb_advertise(ORB_ID(estimator_state), NULL);
}

int estimator_state_publish(int fd, FAR const struct estimator_state_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(estimator_state), fd, msg);
}

int estimator_diag_advertise(void)
{
  return orb_advertise_queue(ORB_ID(estimator_diag), NULL,
                             ESTIMATOR_DIAG_QUEUE_SIZE);
}

int estimator_diag_publish(int fd, FAR const struct estimator_diag_s *msg)
{
  if (fd < 0 || msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(estimator_diag), fd, msg);
}
