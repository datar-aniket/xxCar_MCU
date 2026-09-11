/****************************************************************************
 * Lean Infineon DPS368 barometer driver on the NuttX uORB sensor framework.
 * Register layout, scaling, and compensation follow DPS368 datasheet v1.1.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>

#include "fmuv6c.h"
#include "dps368.h"

#if defined(CONFIG_SENSORS) && defined(CONFIG_I2C)

#define DPS_REG_PRESS          0x00
#define DPS_REG_PRS_CFG        0x06
#define DPS_REG_TMP_CFG        0x07
#define DPS_REG_MEAS_CFG       0x08
#define DPS_REG_CFG            0x09
#define DPS_REG_RESET          0x0c
#define DPS_REG_ID             0x0d
#define DPS_REG_COEF           0x10
#define DPS_REG_COEF_SOURCE    0x28

#define DPS_PRODUCT_ID         0x00
#define DPS_PRODUCT_ID_MASK    0x0f
#define DPS_RESET              0x09
#define DPS_READY_MASK         0xc0
#define DPS_DATA_READY_MASK    0x30
#define DPS_CONTINUOUS_PT      0x07
#define DPS_RATE_32_OSR_16     0x54
#define DPS_SHIFT_PT           0x0c
#define DPS_SCALE_OSR_16       253952.0f
#define DPS_I2C_FREQ           400000
#define DPS_MIN_INTERVAL_US    31250
#define DPS_DEFAULT_INTERVAL_US 50000

struct dps368_cal_s
{
  int16_t c0;
  int16_t c1;
  int32_t c00;
  int32_t c10;
  int16_t c01;
  int16_t c11;
  int16_t c20;
  int16_t c21;
  int16_t c30;
  uint8_t temp_source;
};

struct dps368_dev_s
{
  struct sensor_lowerhalf_s lower;
  FAR struct i2c_master_s *i2c;
  struct dps368_cal_s cal;
  uint8_t addr;
  uint32_t interval;
  mutex_t lock;
  sem_t run;
  bool enabled;
};

static int dps_transfer(FAR struct dps368_dev_s *dev, uint8_t reg,
                        FAR uint8_t *buf, size_t len)
{
  struct i2c_msg_s msg[2];

  msg[0].frequency = DPS_I2C_FREQ;
  msg[0].addr = dev->addr;
  msg[0].flags = 0;
  msg[0].buffer = &reg;
  msg[0].length = 1;
  msg[1].frequency = DPS_I2C_FREQ;
  msg[1].addr = dev->addr;
  msg[1].flags = I2C_M_READ;
  msg[1].buffer = buf;
  msg[1].length = len;
  return I2C_TRANSFER(dev->i2c, msg, 2);
}

static int dps_write(FAR struct dps368_dev_s *dev, uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = {reg, val};
  struct i2c_msg_s msg;

  msg.frequency = DPS_I2C_FREQ;
  msg.addr = dev->addr;
  msg.flags = 0;
  msg.buffer = buf;
  msg.length = sizeof(buf);
  return I2C_TRANSFER(dev->i2c, &msg, 1);
}

static int32_t dps_sign_extend(uint32_t value, unsigned bits)
{
  uint32_t sign = 1u << (bits - 1u);
  return (int32_t)((value ^ sign) - sign);
}

static int dps_configure(FAR struct dps368_dev_s *dev)
{
  uint8_t coef[18];
  uint8_t source;
  uint8_t status;
  uint8_t id;
  int retry;

  if (dps_transfer(dev, DPS_REG_ID, &id, 1) < 0 ||
      (id & DPS_PRODUCT_ID_MASK) != DPS_PRODUCT_ID)
    {
      return -ENODEV;
    }

  if (dps_write(dev, DPS_REG_RESET, DPS_RESET) < 0)
    {
      return -EIO;
    }

  nxsig_usleep(12000);
  for (retry = 0; retry < 20; retry++)
    {
      if (dps_transfer(dev, DPS_REG_MEAS_CFG, &status, 1) >= 0 &&
          (status & DPS_READY_MASK) == DPS_READY_MASK)
        {
          break;
        }

      nxsig_usleep(2000);
    }

  if (retry == 20 || dps_transfer(dev, DPS_REG_COEF, coef, sizeof(coef)) < 0 ||
      dps_transfer(dev, DPS_REG_COEF_SOURCE, &source, 1) < 0)
    {
      return -EIO;
    }

  dev->cal.c0 = (int16_t)dps_sign_extend(((uint32_t)coef[0] << 4) |
                                         (coef[1] >> 4), 12);
  dev->cal.c1 = (int16_t)dps_sign_extend(((uint32_t)(coef[1] & 0x0f) << 8) |
                                         coef[2], 12);
  dev->cal.c00 = dps_sign_extend(((uint32_t)coef[3] << 12) |
                                 ((uint32_t)coef[4] << 4) |
                                 (coef[5] >> 4), 20);
  dev->cal.c10 = dps_sign_extend(((uint32_t)(coef[5] & 0x0f) << 16) |
                                 ((uint32_t)coef[6] << 8) | coef[7], 20);
  dev->cal.c01 = (int16_t)(((uint16_t)coef[8] << 8) | coef[9]);
  dev->cal.c11 = (int16_t)(((uint16_t)coef[10] << 8) | coef[11]);
  dev->cal.c20 = (int16_t)(((uint16_t)coef[12] << 8) | coef[13]);
  dev->cal.c21 = (int16_t)(((uint16_t)coef[14] << 8) | coef[15]);
  dev->cal.c30 = (int16_t)(((uint16_t)coef[16] << 8) | coef[17]);
  dev->cal.temp_source = source & 0x80;

  if (dps_write(dev, DPS_REG_CFG, DPS_SHIFT_PT) < 0 ||
      dps_write(dev, DPS_REG_PRS_CFG, DPS_RATE_32_OSR_16) < 0 ||
      dps_write(dev, DPS_REG_TMP_CFG,
                DPS_RATE_32_OSR_16 | dev->cal.temp_source) < 0 ||
      dps_write(dev, DPS_REG_MEAS_CFG, DPS_CONTINUOUS_PT) < 0)
    {
      return -EIO;
    }

  sninfo("DPS368 ID=%02x configured 32 Hz, 16x oversampling\n", id);
  return OK;
}

static void dps_sample(FAR struct dps368_dev_s *dev)
{
  struct sensor_baro baro;
  uint8_t buf[6];
  uint8_t ready;
  int32_t raw_p;
  int32_t raw_t;
  float p;
  float t;

  if (dps_transfer(dev, DPS_REG_MEAS_CFG, &ready, 1) < 0 ||
      (ready & DPS_DATA_READY_MASK) != DPS_DATA_READY_MASK ||
      dps_transfer(dev, DPS_REG_PRESS, buf, sizeof(buf)) < 0)
    {
      return;
    }

  raw_p = dps_sign_extend(((uint32_t)buf[0] << 16) |
                          ((uint32_t)buf[1] << 8) | buf[2], 24);
  raw_t = dps_sign_extend(((uint32_t)buf[3] << 16) |
                          ((uint32_t)buf[4] << 8) | buf[5], 24);
  p = (float)raw_p / DPS_SCALE_OSR_16;
  t = (float)raw_t / DPS_SCALE_OSR_16;

  baro.pressure = (dev->cal.c00 +
                   p * (dev->cal.c10 +
                        p * (dev->cal.c20 + p * dev->cal.c30)) +
                   t * dev->cal.c01 +
                   t * p * (dev->cal.c11 + p * dev->cal.c21)) / 100.0f;
  baro.temperature = dev->cal.c0 * 0.5f + dev->cal.c1 * t;
  baro.timestamp = fmuv6c_imu_time_now() - DPS_MIN_INTERVAL_US / 2u;

  if (isfinite(baro.pressure) && isfinite(baro.temperature) &&
      baro.pressure >= 10.0f && baro.pressure <= 1200.0f &&
      baro.temperature >= -40.0f && baro.temperature <= 120.0f)
    {
      dev->lower.push_event(dev->lower.priv, &baro, sizeof(baro));
    }
}

static int dps_thread(int argc, FAR char **argv)
{
  FAR struct dps368_dev_s *dev =
    (FAR struct dps368_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  for (; ; )
    {
      if (!dev->enabled)
        {
          nxsem_wait(&dev->run);
        }

      nxsig_usleep(dev->interval);
      if (dev->enabled)
        {
          dps_sample(dev);
        }
    }

  return 0;
}

static int dps_activate(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, bool enable)
{
  FAR struct dps368_dev_s *dev = (FAR struct dps368_dev_s *)lower;
  bool wake;

  nxmutex_lock(&dev->lock);
  wake = enable && !dev->enabled;
  dev->enabled = enable;
  nxmutex_unlock(&dev->lock);
  if (wake)
    {
      nxsem_post(&dev->run);
    }

  return OK;
}

static int dps_set_interval(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, FAR uint32_t *period_us)
{
  FAR struct dps368_dev_s *dev = (FAR struct dps368_dev_s *)lower;

  if (*period_us < DPS_MIN_INTERVAL_US)
    {
      *period_us = DPS_MIN_INTERVAL_US;
    }

  dev->interval = *period_us;
  return OK;
}

static const struct sensor_ops_s g_dps_ops =
{
  NULL, NULL, dps_activate, dps_set_interval, NULL, NULL, NULL, NULL,
  NULL, NULL, NULL, NULL, NULL
};

int dps368_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr)
{
  FAR struct dps368_dev_s *dev;
  FAR char *argv[2];
  char arg1[16];
  int ret;

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->i2c = i2c;
  dev->addr = addr;
  dev->interval = DPS_DEFAULT_INTERVAL_US;
  dev->lower.ops = &g_dps_ops;
  dev->lower.type = SENSOR_TYPE_BAROMETER;
  nxmutex_init(&dev->lock);
  nxsem_init(&dev->run, 0, 0);

  ret = dps_configure(dev);
  if (ret < 0)
    {
      goto fail;
    }

  ret = sensor_register(&dev->lower, devno);
  if (ret < 0)
    {
      goto fail;
    }

  snprintf(arg1, sizeof(arg1), "%p", dev);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("dps368", FMUV6C_SENSOR_PRIO, 2048,
                       dps_thread, argv);
  if (ret < 0)
    {
      sensor_unregister(&dev->lower, devno);
      goto fail;
    }

  return OK;

fail:
  nxmutex_destroy(&dev->lock);
  nxsem_destroy(&dev->run);
  kmm_free(dev);
  return ret;
}

#endif
