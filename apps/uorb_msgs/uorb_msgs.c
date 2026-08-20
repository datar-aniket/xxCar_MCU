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
ORB_NAME_FITS("vehicle_imu");
ORB_NAME_FITS("estimator_state");

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
  ",x:%hf,y:%hf,z:%hf"
  ",instance:%hhu,calibrated:%hhu";

static const char vehicle_gyro_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",x:%hf,y:%hf,z:%hf"
  ",instance:%hhu,calibrated:%hhu";

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
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DEFINE(optical_flow, struct optical_flow_s, optical_flow_format);
ORB_DEFINE(distance_sensor, struct distance_sensor_s, distance_sensor_format);
ORB_DEFINE(vehicle_accel, struct vehicle_accel_s, vehicle_accel_format);
ORB_DEFINE(vehicle_gyro, struct vehicle_gyro_s, vehicle_gyro_format);
ORB_DEFINE(vehicle_imu, struct vehicle_imu_s, vehicle_imu_format);
ORB_DEFINE(estimator_state, struct estimator_state_s,
           estimator_state_format);

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

int vehicle_imu_advertise(void)
{
  return orb_advertise_queue(ORB_ID(vehicle_imu), NULL,
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
