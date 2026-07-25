/****************************************************************************
 * apps/uorb_msgs/uorb_msgs.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Custom uORB topics that are produced by one subsystem and consumed by
 * another, so they cannot live inside either. Same reasoning as rc_in: a topic
 * belongs to nobody in particular.
 *
 *   optical_flow     integrated optical flow + height, from the MTF-02 over
 *                    MAVLink today, from DroneCAN tomorrow.
 *   distance_sensor  a single ranged distance (lidar/sonar/ToF).
 *
 * The publisher is whatever driver has the data (apps/mavlink for now); the
 * subscriber is the navigation/fusion code. Neither should have to link the
 * other, which is the whole point of putting the definition here.
 *
 * Layout warning (same as rc_in): uORB prints a topic by walking o_format and
 * stepping the read offset by each conversion's size, with NO realignment
 * (lib_libbsprintf.c). So these structs must have no internal padding, the
 * format strings must list every field in order, and both are pinned with
 * static_asserts in the .c file. Get it wrong and `uorb_listener` prints
 * plausible garbage rather than failing.
 ****************************************************************************/

#ifndef __APPS_UORB_MSGS_UORB_MSGS_H
#define __APPS_UORB_MSGS_UORB_MSGS_H

#include <nuttx/config.h>

#include <stdint.h>

#include <uORB/uORB.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Integrated optical flow. Fields mirror MAVLink OPTICAL_FLOW_RAD, which is how
 * the MTF-02 reports: flow is INTEGRATED over integration_time_us, so average
 * angular rate is integrated_x / integration_time_us. The gyro fields are the
 * sensor's own rate over the same window, for de-rotation.
 */

struct optical_flow_s
{
  uint64_t timestamp;              /*  0: us, board time when received */
  uint32_t integration_time_us;   /*  8: us, the flow integration window */
  uint32_t time_delta_distance_us;/* 12: us since the distance was sampled */
  float    integrated_x;          /* 16: rad, flow about X over the window */
  float    integrated_y;          /* 20: rad, flow about Y over the window */
  float    integrated_xgyro;      /* 24: rad, gyro about X over the window */
  float    integrated_ygyro;      /* 28: rad */
  float    integrated_zgyro;      /* 32: rad */
  float    distance;              /* 36: m, height to the flow field; <0 = n/a */
  int16_t  temperature;           /* 40: cdegC */
  uint8_t  quality;               /* 42: 0 = no flow, 255 = best */
  uint8_t  sensor_id;             /* 43 */
  uint8_t  pad[4];                /* 44: keep size a multiple of 8, no internal
                                   *     padding */
};

/* A single ranged distance. Distances are in METRES here, converted from the
 * centimetres MAVLink DISTANCE_SENSOR uses, so a consumer never has to remember
 * which unit a given source spoke.
 */

struct distance_sensor_s
{
  uint64_t timestamp;             /*  0: us */
  float    current_distance;      /*  8: m */
  float    min_distance;          /* 12: m, sensor floor */
  float    max_distance;          /* 16: m, sensor ceiling */
  uint8_t  type;                  /* 20: MAV_DISTANCE_SENSOR_* */
  uint8_t  orientation;           /* 21: MAV_SENSOR_ROTATION_* */
  uint8_t  covariance;            /* 22: cm^2, 255 = unknown */
  uint8_t  signal_quality;        /* 23: %, 0 = unknown */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DECLARE(optical_flow);
ORB_DECLARE(distance_sensor);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int optical_flow_advertise(void);
int optical_flow_publish(int fd, FAR const struct optical_flow_s *msg);

int distance_sensor_advertise(void);
int distance_sensor_publish(int fd, FAR const struct distance_sensor_s *msg);

#endif /* __APPS_UORB_MSGS_UORB_MSGS_H */
