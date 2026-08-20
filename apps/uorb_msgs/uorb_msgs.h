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
 *   vehicle_accel    corrected, body-frame accelerometer
 *   vehicle_gyro     corrected, body-frame gyroscope
 *   vehicle_imu      unfiltered coning/sculling-corrected IMU deltas
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

/* The corrected IMU, in the vehicle's own frame.
 *
 * sensor_accel/sensor_gyro stay exactly as the drivers publish them: raw
 * counts scaled to SI, in the chip's own axes, uncorrected. Those are what the
 * logger records, and keeping them untouched is what lets a calibration be
 * re-derived from a recording, or checked against one taken before it. These
 * two topics are the same data after the stored offset and scale have been
 * applied and the sensor has been rotated into the body frame - the form a
 * state estimator wants and the form nothing should have to reconstruct.
 *
 * Splitting it this way is PX4's arrangement (sensor_accel ->
 * vehicle_acceleration) and it exists because the alternative - correcting
 * inside the driver - destroys the raw data at the only point where it is
 * still available.
 *
 * The names are SHORTER than PX4's. uORB builds a device node at
 * /dev/uorb/<name><instance> and registers it through a NAME_MAX-sized field,
 * so with CONFIG_NAME_MAX=32 a topic name may be 20 characters;
 * vehicle_angular_velocity is 24 and was silently truncated, leaving
 * orb_advertise() failing with nothing to say why. Unlike the rotation enum -
 * where a value is copied between configs by hand and MUST match PX4 - nothing
 * outside this firmware depends on a topic's name, so shortening both keeps
 * the pair symmetric and leaves margin. uorb_msgs.c checks the budget at
 * compile time.
 *
 * TWO timestamps, and the distinction is the entire point of the DRDY-edge
 * work in the IMU drivers:
 *
 *   timestamp_sample  when the measurement was TAKEN, carried through from
 *                     the driver's TIM5 edge estimate
 *   timestamp         when this message was produced
 *
 * An estimator must integrate against the first. Publishing only the second
 * would throw away microsecond-accurate sample timing and replace it with
 * whenever a task happened to be scheduled - which is what the shared 1 MHz
 * timebase exists to avoid.
 */

struct vehicle_accel_s
{
  uint64_t timestamp;             /*  0: us, when this was published */
  uint64_t timestamp_sample;      /*  8: us, when it was measured */
  float    x;                     /* 16: m/s^2, body frame */
  float    y;                     /* 20 */
  float    z;                     /* 24 */
  uint8_t  instance;              /* 28: which sensor_accel it came from */

  /* 0 means the values above are RAW passthrough: the source has never been
   * calibrated, so there was nothing to apply. The topic is still published,
   * because a consumer that has to guess whether a silent topic means
   * "uncalibrated" or "broken" will guess wrong - but it must be able to tell,
   * and with a bias near zero the numbers alone cannot say.
   */

  uint8_t  calibrated;            /* 29: 0 = raw passthrough, 1 = corrected */
  uint8_t  pad[2];                /* 30: no internal padding, size mult. of 8 */
};

struct vehicle_gyro_s
{
  uint64_t timestamp;             /*  0: us, when this was published */
  uint64_t timestamp_sample;      /*  8: us, when it was measured */
  float    x;                     /* 16: rad/s, body frame */
  float    y;                     /* 20 */
  float    z;                     /* 24 */
  uint8_t  instance;              /* 28: which sensor_gyro it came from */
  uint8_t  calibrated;            /* 29: 0 = raw passthrough, 1 = corrected */
  uint8_t  pad[2];                /* 30 */
};

/* Calibrated body-frame increments for strapdown propagation. Unlike the
 * corrected controller topics above, this path bypasses every configurable
 * software LPF/notch. The hardware anti-alias filters remain part of the
 * physical measurement chain.
 */

struct vehicle_imu_s
{
  uint64_t timestamp;             /*  0: us, publication time */
  uint64_t timestamp_sample;      /*  8: us, end of integration window */
  uint64_t timestamp_first;       /* 16: us, start of integration window */
  float    delta_angle[3];        /* 24: rad, coning corrected */
  float    delta_velocity[3];     /* 36: m/s, sculling corrected */
  float    delta_angle_dt;        /* 48: s */
  float    delta_velocity_dt;     /* 52: s */
  uint16_t samples;               /* 56: native intervals in this packet */
  uint16_t reset_counter;         /* 58: discontinuity reset generation */
  uint8_t  instance;              /* 60: source IMU */
  uint8_t  clipping;              /* 61: accel XYZ bits 0..2, gyro bits 3..5 */
  uint8_t  accel_calibrated;      /* 62 */
  uint8_t  gyro_calibrated;       /* 63 */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DECLARE(optical_flow);
ORB_DECLARE(distance_sensor);
ORB_DECLARE(vehicle_accel);
ORB_DECLARE(vehicle_gyro);
ORB_DECLARE(vehicle_imu);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int optical_flow_advertise(void);
int optical_flow_publish(int fd, FAR const struct optical_flow_s *msg);

int distance_sensor_advertise(void);
int distance_sensor_publish(int fd, FAR const struct distance_sensor_s *msg);

int vehicle_accel_advertise(void);
int vehicle_accel_publish(int fd, FAR const struct vehicle_accel_s *msg);

int vehicle_gyro_advertise(void);
int vehicle_gyro_publish(int fd, FAR const struct vehicle_gyro_s *msg);

int vehicle_imu_advertise(void);
int vehicle_imu_publish(int fd, FAR const struct vehicle_imu_s *msg);

#endif /* __APPS_UORB_MSGS_UORB_MSGS_H */
