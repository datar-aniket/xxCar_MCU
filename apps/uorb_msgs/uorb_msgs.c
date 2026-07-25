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

#include "uorb_msgs.h"

/****************************************************************************
 * Layout checks
 *
 * uORB decodes a topic by walking o_format and advancing the read offset by
 * each conversion's size, with no realignment. A stray padding byte would shift
 * every later field and print convincing nonsense, so make the compiler prove
 * the fields sit exactly where the format strings expect.
 ****************************************************************************/

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
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DEFINE(optical_flow, struct optical_flow_s, optical_flow_format);
ORB_DEFINE(distance_sensor, struct distance_sensor_s, distance_sensor_format);

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
