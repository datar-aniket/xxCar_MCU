/****************************************************************************
 * boards/fmuv6c/src/fmuv6c_sensors.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Stage 2 - Task 2: register onboard sensors on the NuttX uorb framework so
 * they can be streamed and consumed (uorb_listener now, DDS/fusion later).
 *
 * Starting with the MS5611 barometer using NuttX's stock ms56xx_uorb driver
 * -> /dev/uorb/sensor_baro0. The uorb model samples on demand: the driver's
 * kthread starts when a subscriber opens the topic, and the sampling rate is
 * set by that subscriber (e.g. `uorb_listener sensor_baro -r 20` for 20 Hz;
 * the driver default is 1 Hz).
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>

#include "stm32_i2c.h"
#include "stm32_spi.h"
#include "stm32_gpio.h"
#include "fmuv6c.h"
#include "ms5611.h"
#include "dps368.h"
#include "ist8310.h"
#include "icm42688.h"
#include "bmi055.h"

/****************************************************************************
 * Name: fmuv6c_sensors_initialize
 *
 * Description:
 *   Register the onboard sensors on the uorb framework. Each failure is
 *   logged but non-fatal. Called from board bring-up under CONFIG_SENSORS.
 ****************************************************************************/

int fmuv6c_sensors_initialize(
  FAR const struct fmuv6c_sensor_probe_s *probe)
{
  int first_error = OK;
  int failures = 0;
  int registered = 0;
  int ret;

  /* MS5611 barometer on the internal I2C bus (I2C4) @0x77 -> sensor_baro0.
   * Uses our lean driver (ms5611.c), not NuttX's high-CPU ms56xx_uorb.
   */

  {
    FAR struct i2c_master_s *i2c;

    i2c = stm32_i2cbus_initialize(FMUV6C_I2C_INTERNAL);
    if (i2c == NULL)
      {
        syslog(LOG_ERR, "[sensors] i2c%d init failed for MS5611\n",
               FMUV6C_I2C_INTERNAL);
        ret = -ENODEV;
        first_error = ret;
        failures += 2;
      }
    else
      {
#ifdef CONFIG_XXCAR_BOARD_MATEKH743
        ret = dps368_register(i2c, 0, DPS368_I2C_ADDR);
#else
        ret = ms5611_register(i2c, 0, MS5611_I2C_ADDR);
#endif
        if (ret < 0)
          {
#ifdef CONFIG_XXCAR_BOARD_MATEKH743
            syslog(LOG_ERR, "[sensors] dps368_register failed: %d\n", ret);
#else
            syslog(LOG_ERR, "[sensors] ms5611_register failed: %d\n", ret);
#endif
            first_error = ret;
            failures++;
          }
        else
          {
            registered++;
            syslog(LOG_INFO, "[sensors] %s baro on uorb -> "
                             "/dev/uorb/sensor_baro0\n",
#ifdef CONFIG_XXCAR_BOARD_MATEKH743
                   "DPS368"
#else
                   "MS5611"
#endif
                   );
          }

#ifndef CONFIG_XXCAR_BOARD_MATEKH743
        /* IST8310 magnetometer on the same internal I2C bus @0x0c ->
         * sensor_mag0. Reuse the already-initialized bus handle.
         */

        ret = ist8310_register(i2c, 0, IST8310_I2C_ADDR);
        if (ret < 0)
          {
            syslog(LOG_ERR, "[sensors] ist8310_register failed: %d\n", ret);
            if (first_error == OK)
              {
                first_error = ret;
              }

            failures++;
          }
        else
          {
            registered++;
            syslog(LOG_INFO,
                   "[sensors] IST8310 mag on uorb -> /dev/uorb/sensor_mag0\n");
          }
#endif
      }
  }

#ifdef CONFIG_XXCAR_BOARD_MATEKH743
  /* This board has no onboard compass.  If an IST8310 is connected to the
   * external I2C pads, expose it as sensor_mag0; absence is expected.
   */

  {
    FAR struct i2c_master_s *i2c;

    i2c = stm32_i2cbus_initialize(FMUV6C_I2C_EXTERNAL);
    if (i2c != NULL && ist8310_register(i2c, 0, IST8310_I2C_ADDR) == OK)
      {
        registered++;
        syslog(LOG_INFO,
               "[sensors] external IST8310 -> /dev/uorb/sensor_mag0\n");
      }
    else
      {
        syslog(LOG_INFO,
               "[sensors] no onboard mag; external I2C1 compass not found\n");
      }
  }
#endif

  /* ICM-42688-P primary IMU on SPI1 -> sensor_accel0 + sensor_gyro0.
   * Polled bring-up driver (FIFO/DRDY 2 kHz streaming is a later stage).
   */

  {
    FAR struct spi_dev_s *spi;

    spi = stm32_spibus_initialize(1);
    if (spi == NULL)
      {
        syslog(LOG_ERR, "[sensors] SPI1 init failed for ICM-42688\n");
        ret = -ENODEV;
        if (first_error == OK)
          {
            first_error = ret;
          }

        failures += 2;
      }
    else
      {
        ret = icm42688_register(spi, 0,
                                SPIDEV_IMU(FMUV6C_SPIDEV_ICM42688),
                                GPIO_DRDY_ICM42688);
        if (ret < 0)
          {
            syslog(LOG_ERR, "[sensors] icm42688_register failed: %d\n", ret);
            if (first_error == OK)
              {
                first_error = ret;
              }

            failures++;
          }
        else
          {
            registered++;
            syslog(LOG_INFO, "[sensors] ICM-42688-P IMU on uorb -> "
                             "sensor_accel0 + sensor_gyro0\n");
          }

#ifdef CONFIG_XXCAR_BOARD_MATEKH743
        /* Secondary ICM-42688-P on SPI4 has no usable DRDY connection on
         * H743-SLIM-V4, so use fixed 250 Hz FIFO-drain deadlines.
         */

        spi = stm32_spibus_initialize(4);
        if (spi == NULL)
          {
            ret = -ENODEV;
          }
        else
          {
            ret = icm42688_register(
              spi, 1, SPIDEV_IMU(MATEKH743_SPIDEV_ICM42688_2),
              GPIO_DRDY_ICM42688_2);
          }

        if (ret < 0)
          {
            syslog(LOG_ERR,
                   "[sensors] SPI4 ICM-42688 register failed: %d\n", ret);
            if (first_error == OK)
              {
                first_error = ret;
              }

            failures++;
          }
        else
          {
            registered++;
            syslog(LOG_INFO, "[sensors] SPI4 ICM-42688-P -> "
                             "sensor_accel1 + sensor_gyro1\n");
          }
#else
        /* BMI055 secondary IMU on the same SPI1 bus (accel CS PC15, gyro
         * CS PC14) -> sensor_accel1 + sensor_gyro1. Also 2 kHz FIFO+INT.
         */

        if (probe != NULL &&
            probe->secondary_imu == FMUV6C_SECONDARY_IMU_BMI088)
          {
            ret = -ENOTSUP;
            syslog(LOG_WARNING,
                   "[sensors] BMI088 detected; runtime driver is deferred"
                   " to Step 2\n");
            if (first_error == OK)
              {
                first_error = ret;
              }

            failures++;
          }
        else
          {
            if (probe != NULL &&
                probe->secondary_imu == FMUV6C_SECONDARY_IMU_UNKNOWN)
              {
                syslog(LOG_WARNING,
                       "[sensors] secondary IMU identity unknown;"
                       " attempting the existing BMI055 driver\n");
              }

            ret = bmi055_register(spi, 1);
            if (ret < 0)
              {
                syslog(LOG_ERR, "[sensors] bmi055_register failed: %d\n", ret);
                if (first_error == OK)
                  {
                    first_error = ret;
                  }

                failures++;
              }
            else
              {
                registered++;
                syslog(LOG_INFO, "[sensors] BMI055 2nd IMU on uorb -> "
                                 "sensor_accel1 + sensor_gyro1\n");
            }
          }
#endif
      }
  }

  syslog(failures == 0 ? LOG_INFO : LOG_WARNING,
         "[sensors] registration summary: %d ready, %d failed,"
         " secondary=%s\n",
         registered, failures,
         probe == NULL ? "not-probed" :
         fmuv6c_secondary_imu_name(probe->secondary_imu));
  return first_error;
}
