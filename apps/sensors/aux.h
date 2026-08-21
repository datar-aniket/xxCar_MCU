/****************************************************************************
 * apps/sensors/aux.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The low-rate aiding sensors: magnetometer and barometer.
 *
 * Separate from sensors.c, which turns one raw IMU into corrected body-frame
 * topics at 400 Hz and above. These two run at 50 Hz and 10 Hz and answer a
 * different question, so they get their own daemon rather than being folded
 * into a file whose stated purpose is the IMU.
 *
 * One thread polls both. At these rates two threads would buy nothing but
 * context switches.
 *
 * This is also the first code to honour SENS_MAG_RATE and SENS_BARO_RATE.
 * Both parameters have existed since sensor bring-up and been read by
 * nothing: every driver implements set_interval, but the only caller of
 * orb_set_interval was apps/cal, with hardcoded values.
 ****************************************************************************/

#ifndef __APPS_SENSORS_AUX_H
#define __APPS_SENSORS_AUX_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

struct sensors_aux_status_s
{
  bool     running;
  bool     mag_calibrated;    /* CAL_MAG0_OK, a sane fit, rotations supported */
  uint8_t  mag_rot;           /* SENS_MAG0_ROT actually used */
  uint8_t  board_rot;         /* SENS_BOARD_ROT actually used */
  uint32_t mag_rate_hz;       /* SENS_MAG_RATE as requested */
  uint32_t baro_rate_hz;      /* SENS_BARO_RATE as requested */
  uint32_t mag_out;           /* published */
  uint32_t baro_out;
  uint32_t mag_skipped;       /* correction or publication failed */
  uint32_t baro_skipped;
  float    mag_field[3];      /* last corrected body-frame field, Gauss */
  float    mag_magnitude;     /* |field|, Gauss */
  float    mag_expected;      /* CAL_MAG0_FIELD */
  float    baro_pressure;     /* last pressure, hPa */
  float    baro_temperature;  /* degrees C */
};

int  sensors_aux_start(void);
int  sensors_aux_stop(void);
void sensors_aux_status(FAR struct sensors_aux_status_s *out);

#endif /* __APPS_SENSORS_AUX_H */
