/****************************************************************************
 * boards/fmuv6c/src/ms5611.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Lean MS5611 barometer driver on the NuttX uorb framework (sensor_baro).
 *
 * Written to replace NuttX's stock ms56xx_uorb, which burns ~48% CPU at 20 Hz
 * because it busy-waits (up_udelay) for each conversion and reads temperature
 * every sample. This driver instead:
 *   - waits for conversions with nxsig_usleep only (no CPU spin),
 *   - reads pressure every sample but temperature only every Nth sample
 *     (temperature drifts slowly and only feeds the compensation),
 * so a sampling cycle is essentially all sleep -> near-0% CPU.
 *
 * Compensation math is the first-order algorithm from the MS5611 datasheet.
 * Output: pressure in hPa/mbar, temperature in degrees C (sensor_baro units).
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>

#include "ms5611.h"

#if defined(CONFIG_SENSORS) && defined(CONFIG_I2C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MS5611_CMD_RESET      0x1e
#define MS5611_CMD_CONV_D1    0x48   /* pressure,    OSR 4096 (~9.04 ms) */
#define MS5611_CMD_CONV_D2    0x58   /* temperature, OSR 4096 (~9.04 ms) */
#define MS5611_CMD_ADC_READ   0x00
#define MS5611_CMD_PROM_READ  0xa0   /* + (coef << 1); coef 0..7 */

#define MS5611_CONV_WAIT_US   10000  /* >= 9.04 ms OSR4096 conv, 1 tick, no spin */
#define MS5611_TEMP_DECIMATE  10     /* read temperature every Nth sample */
#define MS5611_I2C_FREQ       400000
#define MS5611_MIN_INTERVAL   10000  /* us -> 100 Hz cap (one conversion/tick) */
#define MS5611_DEFAULT_INTERVAL 100000 /* us -> 10 Hz default */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ms5611_dev_s
{
  struct sensor_lowerhalf_s lower;
  FAR struct i2c_master_s  *i2c;
  uint8_t                   addr;
  uint16_t                  c[7];      /* PROM: c[1..6] = C1..C6 */
  int32_t                   dt;        /* cached dT (from last temp read) */
  int32_t                   temp;      /* cached TEMP in 0.01 C */
  uint32_t                  count;     /* sample counter for temp decimation */
  uint32_t                  interval;  /* us */
  mutex_t                   lock;
  sem_t                     run;
  bool                      enabled;
};

/****************************************************************************
 * Private Functions - I2C
 ****************************************************************************/

static int ms5611_sendcmd(FAR struct ms5611_dev_s *dev, uint8_t cmd)
{
  struct i2c_msg_s msg;

  msg.frequency = MS5611_I2C_FREQ;
  msg.addr      = dev->addr;
  msg.flags     = 0;
  msg.buffer    = &cmd;
  msg.length    = 1;
  return I2C_TRANSFER(dev->i2c, &msg, 1);
}

static int ms5611_read(FAR struct ms5611_dev_s *dev, uint8_t cmd,
                       FAR uint8_t *buf, size_t len)
{
  struct i2c_msg_s msg[2];

  msg[0].frequency = MS5611_I2C_FREQ;
  msg[0].addr      = dev->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &cmd;
  msg[0].length    = 1;

  msg[1].frequency = MS5611_I2C_FREQ;
  msg[1].addr      = dev->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buf;
  msg[1].length    = len;

  return I2C_TRANSFER(dev->i2c, msg, 2);
}

/* Read a 24-bit ADC result (after a conversion + settle) */

static int ms5611_read_adc(FAR struct ms5611_dev_s *dev, FAR uint32_t *val)
{
  uint8_t buf[3];
  int ret = ms5611_read(dev, MS5611_CMD_ADC_READ, buf, 3);
  if (ret >= 0)
    {
      *val = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    }

  return ret;
}

/****************************************************************************
 * Private Functions - device
 ****************************************************************************/

static int ms5611_configure(FAR struct ms5611_dev_s *dev)
{
  uint8_t buf[2];
  int i;
  int ret;

  ms5611_sendcmd(dev, MS5611_CMD_RESET);
  nxsig_usleep(5000);

  /* Read PROM coefficients C1..C6 (words 1..6) */

  for (i = 1; i <= 6; i++)
    {
      ret = ms5611_read(dev, MS5611_CMD_PROM_READ + (i << 1), buf, 2);
      if (ret < 0)
        {
          snerr("ERROR: MS5611 PROM read %d failed: %d\n", i, ret);
          return ret;
        }

      dev->c[i] = ((uint16_t)buf[0] << 8) | buf[1];
    }

  if (dev->c[1] == 0 || dev->c[1] == 0xffff)
    {
      snerr("ERROR: MS5611 bad PROM (C1=0x%04x)\n", dev->c[1]);
      return -ENODEV;
    }

  sninfo("MS5611 PROM C1..C6: %u %u %u %u %u %u\n",
         dev->c[1], dev->c[2], dev->c[3], dev->c[4], dev->c[5], dev->c[6]);
  return OK;
}

/* Read temperature (D2), update cached dT + TEMP. Costs one conversion. */

static void ms5611_update_temp(FAR struct ms5611_dev_s *dev)
{
  uint32_t d2 = 0;

  if (ms5611_sendcmd(dev, MS5611_CMD_CONV_D2) < 0)
    {
      return;
    }

  nxsig_usleep(MS5611_CONV_WAIT_US);

  if (ms5611_read_adc(dev, &d2) < 0)
    {
      return;
    }

  dev->dt   = (int32_t)d2 - ((int32_t)dev->c[5] << 8);
  dev->temp = 2000 + (int32_t)(((int64_t)dev->dt * dev->c[6]) >> 23);
}

/* Read pressure (D1) and push a compensated sample using the cached dT/TEMP */

static void ms5611_sample(FAR struct ms5611_dev_s *dev)
{
  struct sensor_baro baro;
  uint32_t d1 = 0;
  int64_t  off;
  int64_t  sens;
  int32_t  p;

  /* Refresh temperature occasionally (slow-changing, feeds compensation) */

  if ((dev->count++ % MS5611_TEMP_DECIMATE) == 0)
    {
      ms5611_update_temp(dev);
    }

  if (ms5611_sendcmd(dev, MS5611_CMD_CONV_D1) < 0)
    {
      return;
    }

  nxsig_usleep(MS5611_CONV_WAIT_US);

  if (ms5611_read_adc(dev, &d1) < 0)
    {
      return;
    }

  off  = ((int64_t)dev->c[2] << 16) + (((int64_t)dev->c[4] * dev->dt) >> 7);
  sens = ((int64_t)dev->c[1] << 15) + (((int64_t)dev->c[3] * dev->dt) >> 8);
  p    = (int32_t)((((int64_t)d1 * sens) >> 21) - off) >> 15;   /* 0.01 mbar */

  baro.timestamp   = sensor_get_timestamp();
  baro.pressure    = (float)p / 100.0f;          /* hPa / mbar */
  baro.temperature = (float)dev->temp / 100.0f;  /* deg C */
  dev->lower.push_event(dev->lower.priv, &baro, sizeof(baro));
}

static int ms5611_thread(int argc, FAR char **argv)
{
  FAR struct ms5611_dev_s *dev =
      (FAR struct ms5611_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  while (true)
    {
      if (!dev->enabled)
        {
          nxsem_wait(&dev->run);
          continue;
        }

      ms5611_sample(dev);
      nxsig_usleep(dev->interval);
    }

  return 0;
}

/****************************************************************************
 * Private Functions - uorb ops
 ****************************************************************************/

static int ms5611_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  FAR struct ms5611_dev_s *dev = (FAR struct ms5611_dev_s *)lower;
  bool start = false;

  nxmutex_lock(&dev->lock);
  if (enable && !dev->enabled)
    {
      dev->enabled = true;
      dev->count   = 0;   /* force a temperature read on the first sample */
      start        = true;
    }
  else
    {
      dev->enabled = enable;
    }

  nxmutex_unlock(&dev->lock);

  if (start)
    {
      nxsem_post(&dev->run);
    }

  return OK;
}

static int ms5611_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us)
{
  FAR struct ms5611_dev_s *dev = (FAR struct ms5611_dev_s *)lower;

  if (*period_us < MS5611_MIN_INTERVAL)
    {
      *period_us = MS5611_MIN_INTERVAL;
    }

  dev->interval = *period_us;
  return OK;
}

static const struct sensor_ops_s g_ms5611_ops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  ms5611_activate,
  ms5611_set_interval,
  NULL,                 /* batch */
  NULL,                 /* fetch */
  NULL,                 /* flush */
  NULL,                 /* selftest */
  NULL,                 /* set_calibvalue */
  NULL,                 /* calibrate */
  NULL,                 /* get_info */
  NULL,                 /* set_nonwakeup */
  NULL,                 /* control */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ms5611_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr)
{
  FAR struct ms5611_dev_s *dev;
  FAR char *argv[2];
  char arg1[16];
  int ret;

  dev = kmm_zalloc(sizeof(struct ms5611_dev_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->i2c        = i2c;
  dev->addr       = addr;
  dev->interval   = MS5611_DEFAULT_INTERVAL;
  dev->lower.ops  = &g_ms5611_ops;
  dev->lower.type = SENSOR_TYPE_BAROMETER;
  nxmutex_init(&dev->lock);
  nxsem_init(&dev->run, 0, 0);

  ret = ms5611_configure(dev);
  if (ret < 0)
    {
      goto errout;
    }

  ret = sensor_register(&dev->lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: MS5611 sensor_register failed: %d\n", ret);
      goto errout;
    }

  snprintf(arg1, sizeof(arg1), "%p", dev);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("ms5611", SCHED_PRIORITY_DEFAULT, 1024,
                       ms5611_thread, argv);
  if (ret < 0)
    {
      snerr("ERROR: MS5611 thread create failed: %d\n", ret);
      sensor_unregister(&dev->lower, devno);
      goto errout;
    }

  return OK;

errout:
  nxmutex_destroy(&dev->lock);
  nxsem_destroy(&dev->run);
  kmm_free(dev);
  return ret;
}

#endif /* CONFIG_SENSORS && CONFIG_I2C */
