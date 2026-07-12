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
#include "fmuv6c.h"
#include "ms5611.h"
#include "ist8310.h"
#include "icm42688.h"

/****************************************************************************
 * Name: fmuv6c_sensors_initialize
 *
 * Description:
 *   Register the onboard sensors on the uorb framework. Each failure is
 *   logged but non-fatal. Called from board bring-up under CONFIG_SENSORS.
 ****************************************************************************/

int fmuv6c_sensors_initialize(void)
{
  int ret = OK;

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
      }
    else
      {
        ret = ms5611_register(i2c, 0, MS5611_I2C_ADDR);
        if (ret < 0)
          {
            syslog(LOG_ERR, "[sensors] ms5611_register failed: %d\n", ret);
          }
        else
          {
            syslog(LOG_INFO,
                   "[sensors] MS5611 baro on uorb -> /dev/uorb/sensor_baro0\n");
          }

        /* IST8310 magnetometer on the same internal I2C bus @0x0c ->
         * sensor_mag0. Reuse the already-initialized bus handle.
         */

        ret = ist8310_register(i2c, 0, IST8310_I2C_ADDR);
        if (ret < 0)
          {
            syslog(LOG_ERR, "[sensors] ist8310_register failed: %d\n", ret);
          }
        else
          {
            syslog(LOG_INFO,
                   "[sensors] IST8310 mag on uorb -> /dev/uorb/sensor_mag0\n");
          }
      }
  }

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
      }
    else
      {
        ret = icm42688_register(spi, 0);
        if (ret < 0)
          {
            syslog(LOG_ERR, "[sensors] icm42688_register failed: %d\n", ret);
          }
        else
          {
            syslog(LOG_INFO, "[sensors] ICM-42688-P IMU on uorb -> "
                             "sensor_accel0 + sensor_gyro0\n");
          }
      }
  }

  return ret;
}
