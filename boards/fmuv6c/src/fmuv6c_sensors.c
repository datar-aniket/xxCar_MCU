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

#include "stm32_i2c.h"
#include "fmuv6c.h"

#ifdef CONFIG_SENSORS_MS56XX
#  include <nuttx/sensors/ms56xx.h>
#endif

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

#ifdef CONFIG_SENSORS_MS56XX
  /* MS5611 barometer on the internal I2C bus (I2C4) @0x77 -> sensor_baro0 */

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
        ret = ms56xx_register(i2c, 0, MS56XX_ADDR0, MS56XX_MODEL_MS5611);
        if (ret < 0)
          {
            syslog(LOG_ERR, "[sensors] ms56xx_register failed: %d\n", ret);
          }
        else
          {
            syslog(LOG_INFO,
                   "[sensors] MS5611 baro on uorb -> /dev/uorb/sensor_baro0\n");
          }
      }
  }
#endif /* CONFIG_SENSORS_MS56XX */

  return ret;
}
