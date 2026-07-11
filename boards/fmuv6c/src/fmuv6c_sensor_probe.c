/****************************************************************************
 * boards/fmuv6c/src/fmuv6c_sensor_probe.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Stage 2 - Task 1: synchronous discovery of every onboard sensor. For each
 * device we assert its chip-select / I2C address, read its ID register (or
 * probe an ACK) and log one PASS/FAIL line via syslog (-> TELEM1 console AND
 * /dev/ramlog). Deliberately NO threads, NO uorb, NO floating point - a pure
 * register-read health check that verifies the SPI1 + I2C4 buses and confirms
 * the Holybro Pixhawk 6C sensor set.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>

#include "stm32_spi.h"
#include "stm32_i2c.h"
#include "fmuv6c.h"

#if defined(CONFIG_SPI) && defined(CONFIG_I2C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PROBE_SPI_FREQ   1000000
#define PROBE_I2C_FREQ   400000
#define SPI_READ_BIT     0x80

/* Expected IDs */

#define ICM42688_WHOAMI_REG   0x75
#define ICM42688_WHOAMI_VAL   0x47
#define BMI088_CHIPID_REG     0x00
#define BMI088_ACC_CHIPID_VAL 0x1e
#define BMI088_GYR_CHIPID_VAL 0x0f
#define IST8310_WAI_REG       0x00
#define IST8310_WAI_VAL       0x10
#define MS5611_RESET_CMD      0x1e

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t spi_read_id(FAR struct spi_dev_s *spi, uint32_t devid,
                           uint8_t reg, uint8_t mode, bool dummy)
{
  uint8_t val = 0;

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, mode);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, PROBE_SPI_FREQ);

  SPI_SELECT(spi, devid, true);
  SPI_SEND(spi, reg | SPI_READ_BIT);
  if (dummy)
    {
      SPI_SEND(spi, 0);          /* BMI088 accel returns a dummy byte first */
    }

  SPI_RECVBLOCK(spi, &val, 1);
  SPI_SELECT(spi, devid, false);
  SPI_LOCK(spi, false);
  return val;
}

static int i2c_read_reg(FAR struct i2c_master_s *i2c, uint8_t addr,
                        uint8_t reg, FAR uint8_t *val)
{
  struct i2c_msg_s msg[2];

  msg[0].frequency = PROBE_I2C_FREQ;
  msg[0].addr      = addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = PROBE_I2C_FREQ;
  msg[1].addr      = addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = val;
  msg[1].length    = 1;

  return I2C_TRANSFER(i2c, msg, 2);
}

static int i2c_write_cmd(FAR struct i2c_master_s *i2c, uint8_t addr,
                         uint8_t cmd)
{
  struct i2c_msg_s msg;

  msg.frequency = PROBE_I2C_FREQ;
  msg.addr      = addr;
  msg.flags     = 0;
  msg.buffer    = &cmd;
  msg.length    = 1;

  return I2C_TRANSFER(i2c, &msg, 1);
}

#define PF(ok) ((ok) ? "PASS" : "FAIL")

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int fmuv6c_sensor_probe(void)
{
  FAR struct spi_dev_s     *spi;
  FAR struct i2c_master_s  *i2c;
  uint8_t id;
  uint8_t val;
  int fail = 0;

  syslog(LOG_INFO, "==== FMUv6C sensor discovery ====\n");

  /* ---- SPI1 IMUs ---- */

  spi = stm32_spibus_initialize(1);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "[probe] SPI1 init FAILED\n");
      fail++;
    }
  else
    {
      /* ICM-42688-P (CS PC13), SPI mode 3 */

      id = spi_read_id(spi, SPIDEV_IMU(FMUV6C_SPIDEV_ICM42688),
                       ICM42688_WHOAMI_REG, SPIDEV_MODE3, false);
      syslog(LOG_INFO, "[probe] ICM-42688-P  SPI1 CS PC13  WHOAMI=0x%02x  %s\n",
             id, PF(id == ICM42688_WHOAMI_VAL));
      fail += (id != ICM42688_WHOAMI_VAL);

      /* BMI088 accel (CS PC15), SPI mode 0. First read switches I2C->SPI mode
       * (garbage), so read twice and check the second; needs the dummy byte.
       */

      spi_read_id(spi, SPIDEV_ACCELEROMETER(FMUV6C_SPIDEV_BMI088_ACCEL),
                  BMI088_CHIPID_REG, SPIDEV_MODE0, true);
      id = spi_read_id(spi, SPIDEV_ACCELEROMETER(FMUV6C_SPIDEV_BMI088_ACCEL),
                       BMI088_CHIPID_REG, SPIDEV_MODE0, true);
      syslog(LOG_INFO, "[probe] BMI088-accel SPI1 CS PC15  CHIPID=0x%02x  %s\n",
             id, PF(id == BMI088_ACC_CHIPID_VAL));
      fail += (id != BMI088_ACC_CHIPID_VAL);

      /* BMI088 gyro (CS PC14), SPI mode 0, no dummy byte */

      id = spi_read_id(spi, SPIDEV_ACCELEROMETER(FMUV6C_SPIDEV_BMI088_GYRO),
                       BMI088_CHIPID_REG, SPIDEV_MODE0, false);
      syslog(LOG_INFO, "[probe] BMI088-gyro  SPI1 CS PC14  CHIPID=0x%02x  %s\n",
             id, PF(id == BMI088_GYR_CHIPID_VAL));
      fail += (id != BMI088_GYR_CHIPID_VAL);
    }

  /* ---- I2C4 internal sensor module ---- */

  i2c = stm32_i2cbus_initialize(FMUV6C_I2C_INTERNAL);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "[probe] I2C%d init FAILED\n", FMUV6C_I2C_INTERNAL);
      fail++;
    }
  else
    {
      /* IST8310 magnetometer @0x0c */

      val = 0;
      if (i2c_read_reg(i2c, 0x0c, IST8310_WAI_REG, &val) == OK &&
          val == IST8310_WAI_VAL)
        {
          syslog(LOG_INFO, "[probe] IST8310     I2C4 0x0c    WHOAMI=0x%02x  PASS\n", val);
        }
      else
        {
          syslog(LOG_INFO, "[probe] IST8310     I2C4 0x0c    WHOAMI=0x%02x  FAIL\n", val);
          fail++;
        }

      /* MS5611 barometer @0x77 - reset command must ACK */

      if (i2c_write_cmd(i2c, 0x77, MS5611_RESET_CMD) == OK)
        {
          syslog(LOG_INFO, "[probe] MS5611      I2C4 0x77    ACK          PASS\n");
        }
      else
        {
          syslog(LOG_INFO, "[probe] MS5611      I2C4 0x77    no-ACK       FAIL\n");
          fail++;
        }

      /* Calibration EEPROM @0x50 - 1-byte read must ACK */

      val = 0;
      if (i2c_read_reg(i2c, 0x50, 0x00, &val) == OK)
        {
          syslog(LOG_INFO, "[probe] EEPROM      I2C4 0x50    ACK          PASS\n");
        }
      else
        {
          syslog(LOG_INFO, "[probe] EEPROM      I2C4 0x50    no-ACK       FAIL\n");
          fail++;
        }
    }

  syslog(LOG_INFO, "==== sensor discovery: %s (%d fail) ====\n",
         fail ? "INCOMPLETE" : "ALL PASS", fail);
  return fail ? -ENODEV : OK;
}

#endif /* CONFIG_SPI && CONFIG_I2C */
