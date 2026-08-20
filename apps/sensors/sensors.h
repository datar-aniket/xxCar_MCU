/****************************************************************************
 * apps/sensors/sensors.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Turns one raw IMU into the corrected, body-frame topics an estimator wants.
 *
 * Subscribes to sensor_accel/sensor_gyro at the instance SENS_IMU_SEL names,
 * applies the stored calibration, rotates into the vehicle frame, and
 * publishes vehicle_acceleration and vehicle_angular_velocity. The raw topics
 * are not touched - they stay exactly as the drivers produced them, which is
 * what lets a calibration be re-derived from a recording later.
 *
 *     corrected = (raw - OFF) * SCL          accel: offset and scale
 *     corrected = raw - OFF                  gyro: bias only, no scale
 *                                            without a rate table
 *     body      = BOARD_ROT( IMUn_ROT( corrected ) )
 *
 * The order matters and is not arbitrary. Calibration is measured in the
 * SENSOR's own axes - the six-position procedure moves the board and records
 * what each chip axis reads - so it has to be applied before any rotation. A
 * diagonal scale means nothing once the axes have been mixed.
 ****************************************************************************/

#ifndef __APPS_SENSORS_SENSORS_H
#define __APPS_SENSORS_SENSORS_H

#include <stdbool.h>
#include <stdint.h>

struct sensors_status_s
{
  bool     running;
  uint8_t  instance;        /* SENS_IMU_SEL as it was read at start */
  uint8_t  accel_rot;       /* composed sensor+board rotation actually used */
  uint8_t  gyro_rot;
  bool     accel_calibrated;
  bool     gyro_calibrated;
  uint32_t accel_out;       /* messages published */
  uint32_t gyro_out;
  uint32_t accel_skipped;   /* samples that could not be published */
  uint32_t gyro_skipped;
  float    accel_off[3];
  float    accel_scl[3];
  float    gyro_off[3];
  float    accel_filter_rate_hz;
  float    gyro_filter_rate_hz;
  float    accel_lpf_hz;
  float    gyro_lpf_hz;
  float    gyro_notch_hz;
  float    gyro_notch_bw_hz;
  float    accel_raw_rms[3]; /* Latest stationary-sized AC RMS window */
  float    accel_filt_rms[3];
  float    gyro_raw_rms[3];
  float    gyro_filt_rms[3];
  uint32_t filter_resets;
  uint32_t filter_timestamp_errors;
  uint32_t filter_invalid;
};

int  sensors_start(void);
int  sensors_stop(void);
void sensors_status(struct sensors_status_s *out);

#endif /* __APPS_SENSORS_SENSORS_H */
