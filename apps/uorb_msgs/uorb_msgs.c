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
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DEFINE(optical_flow, struct optical_flow_s, optical_flow_format);
ORB_DEFINE(distance_sensor, struct distance_sensor_s, distance_sensor_format);
ORB_DEFINE(vehicle_accel, struct vehicle_accel_s, vehicle_accel_format);
ORB_DEFINE(vehicle_gyro, struct vehicle_gyro_s, vehicle_gyro_format);

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
