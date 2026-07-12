/****************************************************************************
 * boards/fmuv6c/src/icm42688.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ICM-42688-P primary IMU on the NuttX uorb framework (sensor_accel +
 * sensor_gyro). This is the POLLED bring-up driver: the device runs its
 * internal ODR (1 kHz low-noise) and a single kthread reads one accel+gyro
 * burst per sample, sleeping the sample interval in between (no CPU spin).
 *
 * The poll interval is clamped to >= 1 system tick: nxsig_usleep of a sub-tick
 * value rounds toward a busy 0-tick spin that starves the console, so polled
 * sampling is capped at the tick rate (100 Hz). High-rate (2 kHz) streaming
 * will use the FIFO + DRDY interrupt in a later stage, not this poll loop.
 *
 * Config: accel +/-16 g, gyro +/-2000 dps (both default full scale). Output in
 * SI units: accel m/s^2, gyro rad/s, raw sensor axes (board-frame rotation is
 * applied later in fusion).
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
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/sensor.h>

#include "fmuv6c.h"
#include "icm42688.h"

#if defined(CONFIG_SENSORS) && defined(CONFIG_SPI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register map (bank 0) */

#define ICM_REG_DEVICE_CONFIG 0x11   /* bit0 SOFT_RESET                     */
#define ICM_REG_TEMP_DATA1    0x1d   /* burst start: temp,accel,gyro (14 B) */
#define ICM_REG_PWR_MGMT0     0x4e
#define ICM_REG_GYRO_CONFIG0  0x4f
#define ICM_REG_ACCEL_CONFIG0 0x50
#define ICM_REG_WHO_AM_I      0x75

#define ICM_WHO_AM_I_VAL      0x47
#define ICM_SOFT_RESET        0x01
#define ICM_PWR_ALL_LOWNOISE  0x0f   /* gyro LN (11) + accel LN (11)        */

/* GYRO_CONFIG0 / ACCEL_CONFIG0: FS_SEL[7:5]=000 (max FS), ODR[3:0]=0110=1kHz */

#define ICM_GYRO_CFG_2000DPS_1KHZ  0x06   /* +/-2000 dps, 1 kHz             */
#define ICM_ACCEL_CFG_16G_1KHZ     0x06   /* +/-16 g,     1 kHz             */

#define ICM_READ_BIT         0x80
#define ICM_SPI_MODE         SPIDEV_MODE3
#define ICM_SPI_FREQ         8000000      /* 8 MHz (device max 24 MHz)      */

/* Scale factors (full-scale sensitivities from the datasheet) */

#define ICM_ACCEL_LSB_PER_G  2048.0f      /* +/-16 g                        */
#define ICM_GYRO_LSB_PER_DPS 16.4f        /* +/-2000 dps                    */
#define ICM_DPS_TO_RADS      0.017453292519943295f   /* pi/180             */
#define ICM_G_MSS            9.80665f

#define ICM_ACCEL_SCALE      (ICM_G_MSS / ICM_ACCEL_LSB_PER_G)
#define ICM_GYRO_SCALE       (ICM_DPS_TO_RADS / ICM_GYRO_LSB_PER_DPS)

/* Temperature: T[C] = raw/132.48 + 25 */

#define ICM_TEMP_SCALE       (1.0f / 132.48f)
#define ICM_TEMP_OFFSET      25.0f

#define ICM_MIN_INTERVAL     (USEC_PER_TICK)   /* >= 1 tick (see file head)  */
#define ICM_DEFAULT_INTERVAL 10000             /* us -> 100 Hz               */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One uorb lowerhalf per topic (accel, gyro), each with a backpointer to the
 * shared device so the ops can reach the common SPI handle and thread state.
 */

struct icm42688_dev_s;

struct icm42688_sensor_s
{
  struct sensor_lowerhalf_s     lower;
  FAR struct icm42688_dev_s    *dev;
};

struct icm42688_dev_s
{
  struct icm42688_sensor_s  accel;
  struct icm42688_sensor_s  gyro;
  FAR struct spi_dev_s     *spi;
  uint32_t                  devid;
  uint32_t                  interval;   /* us */
  bool                      accel_en;
  bool                      gyro_en;
  bool                      running;    /* thread past its initial wait     */
  mutex_t                   lock;
  sem_t                     run;
};

/****************************************************************************
 * Private Functions - SPI
 ****************************************************************************/

static void icm42688_lock(FAR struct icm42688_dev_s *dev)
{
  SPI_LOCK(dev->spi, true);
  SPI_SETMODE(dev->spi, ICM_SPI_MODE);
  SPI_SETBITS(dev->spi, 8);
  SPI_SETFREQUENCY(dev->spi, ICM_SPI_FREQ);
}

static void icm42688_unlock(FAR struct icm42688_dev_s *dev)
{
  SPI_LOCK(dev->spi, false);
}

static void icm42688_write_reg(FAR struct icm42688_dev_s *dev, uint8_t reg,
                               uint8_t val)
{
  icm42688_lock(dev);
  SPI_SELECT(dev->spi, dev->devid, true);
  SPI_SEND(dev->spi, reg & 0x7f);
  SPI_SEND(dev->spi, val);
  SPI_SELECT(dev->spi, dev->devid, false);
  icm42688_unlock(dev);
}

static uint8_t icm42688_read_reg(FAR struct icm42688_dev_s *dev, uint8_t reg)
{
  uint8_t val = 0;

  icm42688_lock(dev);
  SPI_SELECT(dev->spi, dev->devid, true);
  SPI_SEND(dev->spi, reg | ICM_READ_BIT);
  SPI_RECVBLOCK(dev->spi, &val, 1);
  SPI_SELECT(dev->spi, dev->devid, false);
  icm42688_unlock(dev);
  return val;
}

static void icm42688_read_burst(FAR struct icm42688_dev_s *dev, uint8_t reg,
                                FAR uint8_t *buf, size_t len)
{
  icm42688_lock(dev);
  SPI_SELECT(dev->spi, dev->devid, true);
  SPI_SEND(dev->spi, reg | ICM_READ_BIT);
  SPI_RECVBLOCK(dev->spi, buf, len);
  SPI_SELECT(dev->spi, dev->devid, false);
  icm42688_unlock(dev);
}

/****************************************************************************
 * Private Functions - device
 ****************************************************************************/

static int icm42688_configure(FAR struct icm42688_dev_s *dev)
{
  uint8_t id;

  /* Soft reset and let the device re-initialize */

  icm42688_write_reg(dev, ICM_REG_DEVICE_CONFIG, ICM_SOFT_RESET);
  nxsig_usleep(2000);

  id = icm42688_read_reg(dev, ICM_REG_WHO_AM_I);
  if (id != ICM_WHO_AM_I_VAL)
    {
      snerr("ERROR: ICM-42688 WHO_AM_I=0x%02x (want 0x%02x)\n",
            id, ICM_WHO_AM_I_VAL);
      return -ENODEV;
    }

  /* Set full-scale + ODR before powering the sensors on (datasheet order) */

  icm42688_write_reg(dev, ICM_REG_GYRO_CONFIG0, ICM_GYRO_CFG_2000DPS_1KHZ);
  icm42688_write_reg(dev, ICM_REG_ACCEL_CONFIG0, ICM_ACCEL_CFG_16G_1KHZ);

  /* Accel + gyro to low-noise mode; wait for them to spin up */

  icm42688_write_reg(dev, ICM_REG_PWR_MGMT0, ICM_PWR_ALL_LOWNOISE);
  nxsig_usleep(1000);

  sninfo("ICM-42688-P configured (WHO_AM_I=0x%02x)\n", id);
  return OK;
}

static void icm42688_sample(FAR struct icm42688_dev_s *dev)
{
  uint8_t buf[14];
  float   temp;
  int16_t raw;

  /* Burst: TEMP(2), ACCEL X/Y/Z(6), GYRO X/Y/Z(6), all big-endian */

  icm42688_read_burst(dev, ICM_REG_TEMP_DATA1, buf, sizeof(buf));

  raw  = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
  temp = (float)raw * ICM_TEMP_SCALE + ICM_TEMP_OFFSET;

  if (dev->accel_en)
    {
      struct sensor_accel a;
      a.timestamp   = sensor_get_timestamp();
      a.x = (float)(int16_t)((uint16_t)buf[2] << 8 | buf[3]) * ICM_ACCEL_SCALE;
      a.y = (float)(int16_t)((uint16_t)buf[4] << 8 | buf[5]) * ICM_ACCEL_SCALE;
      a.z = (float)(int16_t)((uint16_t)buf[6] << 8 | buf[7]) * ICM_ACCEL_SCALE;
      a.temperature = temp;
      dev->accel.lower.push_event(dev->accel.lower.priv, &a, sizeof(a));
    }

  if (dev->gyro_en)
    {
      struct sensor_gyro g;
      g.timestamp   = sensor_get_timestamp();
      g.x = (float)(int16_t)((uint16_t)buf[8]  << 8 | buf[9])  * ICM_GYRO_SCALE;
      g.y = (float)(int16_t)((uint16_t)buf[10] << 8 | buf[11]) * ICM_GYRO_SCALE;
      g.z = (float)(int16_t)((uint16_t)buf[12] << 8 | buf[13]) * ICM_GYRO_SCALE;
      g.temperature = temp;
      dev->gyro.lower.push_event(dev->gyro.lower.priv, &g, sizeof(g));
    }
}

static int icm42688_thread(int argc, FAR char **argv)
{
  FAR struct icm42688_dev_s *dev =
      (FAR struct icm42688_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  while (true)
    {
      if (!dev->accel_en && !dev->gyro_en)
        {
          nxsem_wait(&dev->run);
          continue;
        }

      nxsig_usleep(dev->interval);
      icm42688_sample(dev);
    }

  return 0;
}

/****************************************************************************
 * Private Functions - uorb ops
 ****************************************************************************/

static int icm42688_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable)
{
  FAR struct icm42688_sensor_s *s = (FAR struct icm42688_sensor_s *)lower;
  FAR struct icm42688_dev_s *dev = s->dev;
  bool wake = false;

  nxmutex_lock(&dev->lock);

  /* Was the thread idle (both topics off) before this change? */

  bool was_idle = (!dev->accel_en && !dev->gyro_en);

  if (s == &dev->accel)
    {
      dev->accel_en = enable;
    }
  else
    {
      dev->gyro_en = enable;
    }

  if (was_idle && (dev->accel_en || dev->gyro_en))
    {
      wake = true;
    }

  nxmutex_unlock(&dev->lock);

  if (wake)
    {
      nxsem_post(&dev->run);
    }

  return OK;
}

static int icm42688_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us)
{
  FAR struct icm42688_sensor_s *s = (FAR struct icm42688_sensor_s *)lower;
  FAR struct icm42688_dev_s *dev = s->dev;

  if (*period_us < ICM_MIN_INTERVAL)
    {
      *period_us = ICM_MIN_INTERVAL;
    }

  /* Accel and gyro are read from one shared burst, so they share the poll
   * interval; the most recent request wins.
   */

  dev->interval = *period_us;
  return OK;
}

static const struct sensor_ops_s g_icm42688_ops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  icm42688_activate,
  icm42688_set_interval,
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

int icm42688_register(FAR struct spi_dev_s *spi, int devno)
{
  FAR struct icm42688_dev_s *dev;
  FAR char *argv[2];
  char arg1[16];
  int ret;

  dev = kmm_zalloc(sizeof(struct icm42688_dev_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->spi      = spi;
  dev->devid    = SPIDEV_IMU(FMUV6C_SPIDEV_ICM42688);
  dev->interval = ICM_DEFAULT_INTERVAL;

  dev->accel.dev        = dev;
  dev->accel.lower.ops  = &g_icm42688_ops;
  dev->accel.lower.type = SENSOR_TYPE_ACCELEROMETER;

  dev->gyro.dev         = dev;
  dev->gyro.lower.ops   = &g_icm42688_ops;
  dev->gyro.lower.type  = SENSOR_TYPE_GYROSCOPE;

  nxmutex_init(&dev->lock);
  nxsem_init(&dev->run, 0, 0);

  ret = icm42688_configure(dev);
  if (ret < 0)
    {
      goto errout;
    }

  ret = sensor_register(&dev->accel.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: ICM-42688 accel sensor_register failed: %d\n", ret);
      goto errout;
    }

  ret = sensor_register(&dev->gyro.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: ICM-42688 gyro sensor_register failed: %d\n", ret);
      sensor_unregister(&dev->accel.lower, devno);
      goto errout;
    }

  snprintf(arg1, sizeof(arg1), "%p", dev);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("icm42688", SCHED_PRIORITY_DEFAULT, 1024,
                       icm42688_thread, argv);
  if (ret < 0)
    {
      snerr("ERROR: ICM-42688 thread create failed: %d\n", ret);
      sensor_unregister(&dev->gyro.lower, devno);
      sensor_unregister(&dev->accel.lower, devno);
      goto errout;
    }

  return OK;

errout:
  nxmutex_destroy(&dev->lock);
  nxsem_destroy(&dev->run);
  kmm_free(dev);
  return ret;
}

#endif /* CONFIG_SENSORS && CONFIG_SPI */
