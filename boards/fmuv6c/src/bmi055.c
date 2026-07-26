/****************************************************************************
 * boards/fmuv6c/src/bmi055.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BMI055 secondary IMU on the NuttX uorb framework (sensor_accel1 +
 * sensor_gyro1), FIFO + interrupt streaming at 2 kHz.
 *
 * The BMI055 is two independent dies in one package, each with its own chip
 * select and its own INT1 line:
 *
 *   accel (BMA2x2 class, chip-id 0xFA) : CS PC15, INT1 -> PE4
 *   gyro  (BMG160 class,  chip-id 0x0F) : CS PC14, INT1 -> PE5
 *
 * So this driver runs them as two separate devices that happen to share code:
 * each has its own FIFO, watermark interrupt, drain kthread and uorb topic.
 * The architecture matches the primary ICM-42688-P: the die buffers samples in
 * its hardware FIFO, pulses INT1 (active-low) at an 8-sample watermark, the
 * falling-edge ISR posts a semaphore, and a kthread drains the batch in one
 * SPI-DMA burst. No polling, no OS-tick coupling.
 *
 * Register maps, reset sequence and FIFO/INT setup are cross-checked against
 * PX4's hardware-proven bmi055 driver.
 *
 * Configured to match the primary IMU where the silicon allows:
 *   rate  : 2 kHz         (same as primary)
 *   range : +/-16 g, +/-2000 dps (same as primary)
 *   filter: unfiltered / high-bandwidth (no extra group delay)
 *
 * RESOLUTION IS A HARDWARE LIMIT, NOT A CONFIG CHOICE. The BMI055 accel ADC is
 * 12-bit: at +/-16 g that is 128 LSB/g (7.8 mg/LSB) versus the ICM-42688-P's
 * 20-bit hi-res 32768 LSB/g. The gyro is 16-bit: 16.384 LSB/dps versus the
 * ICM's 262 LSB/dps. The BMI055 therefore cannot match the primary's
 * resolution - it is a coarser part by design. It matches on rate and range so
 * the two IMUs are directly comparable for cross-checking/voting, and it will
 * not clip on the shocks that a ground robot sees.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/sensor.h>

#include "stm32_gpio.h"
#include "fmuv6c.h"
#include "bmi055.h"

#if defined(CONFIG_SENSORS) && defined(CONFIG_SPI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers common to both dies */

#define BMI_REG_CHIPID          0x00
#define BMI_REG_TEMP            0x08
#define BMI_REG_FIFO_STATUS     0x0e
#define BMI_REG_SOFTRESET       0x14
#define BMI_REG_FIFO_CONFIG_1   0x3e
#define BMI_REG_FIFO_DATA       0x3f

#define BMI_SOFTRESET_VAL       0xb6
#define BMI_FIFO_OVERRUN        0x80
#define BMI_FIFO_FRAME_MASK     0x7f
#define BMI_FIFO_MODE           0x40   /* FIFO_CONFIG_1: fifo_mode */

/* Accel (BMA2x2 class) */

#define ACC_CHIPID_VAL          0xfa
#define ACC_REG_PMU_RANGE       0x0f
#define ACC_REG_ACCD_HBW        0x13
#define ACC_REG_INT_EN_1        0x17
#define ACC_REG_INT_MAP_1       0x1a
#define ACC_REG_INT_OUT_CTRL    0x20
#define ACC_REG_FIFO_CONFIG_0   0x30

#define ACC_RANGE_16G_SET       0x0c   /* range<3:0> = 1100b */
#define ACC_RANGE_16G_CLR       0x03
#define ACC_HBW_UNFILTERED      0x80
#define ACC_INT_FWM_EN          0x40
#define ACC_INT1_FWM            0x02
#define ACC_INT1_OD_LVL         0x03   /* clear -> push-pull, active low */
#define ACC_FIFO_DEPTH          32     /* frames */

/* Gyro (BMG160 class) */

#define GYR_CHIPID_VAL          0x0f
#define GYR_REG_RANGE           0x0f
#define GYR_REG_RATE_HBW        0x13
#define GYR_REG_INT_EN_0        0x15
#define GYR_REG_INT_EN_1        0x16
#define GYR_REG_INT_MAP_1       0x18
#define GYR_REG_FIFO_WM_ENABLE  0x1e
#define GYR_REG_FIFO_CONFIG_0   0x3d

#define GYR_RANGE_2000DPS       0x00
#define GYR_HBW_UNFILTERED      0x80
#define GYR_INT_FIFO_EN         0x40   /* INT_EN_0: fifo_en */
#define GYR_INT1_OD_LVL         0x03   /* clear -> push-pull, active low */
#define GYR_INT1_FIFO           0x04   /* INT_MAP_1: int1_fifo */
#define GYR_FIFO_WM_ENABLE      0x80
#define GYR_FIFO_CFG0_TAG       0x80   /* keep clear */
#define GYR_FIFO_DEPTH          100    /* frames */

/* FIFO framing / timing (both dies: 6-byte XYZ frames at 2 kHz) */

#define BMI_FIFO_FRAME          6
#define BMI_FIFO_WM_SAMPLES     8      /* watermark -> ~250 Hz interrupt */
#define BMI_FIFO_MAX_READ       64     /* per DMA read (*6=384 <= 512) */
#define BMI_SAMPLE_PERIOD_US    500    /* 2 kHz */

#define BMI_READ_BIT            0x80
#define BMI_SPI_MODE            SPIDEV_MODE0
#define BMI_SPI_FREQ            10000000   /* 10 MHz (PX4 uses the same) */

/* Scales. See the resolution note in the file header. */

#define BMI_ACCEL_SCALE         (9.80665f / 128.0f)              /* +/-16 g   */
#define BMI_GYRO_SCALE          (0.017453292519943295f / 16.384f) /* +/-2000dps */
#define BMI_TEMP_SCALE          0.5f                             /* 0.5 K/LSB */
#define BMI_TEMP_OFFSET         23.0f

#define BMI_UORB_NBUFFER        256    /* ~128 ms at 2 kHz - see icm42688.c */
#define BMI_WATCHDOG_MS         20     /* fallback drain if an INT is missed */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bmi055_dev_s
{
  struct sensor_lowerhalf_s lower;
  FAR struct spi_dev_s     *spi;
  uint32_t                  devid;    /* SPI chip select id  */
  uint32_t                  drdy;     /* INT1 GPIO pinset    */
  bool                      is_gyro;
  uint8_t                   depth;    /* FIFO depth (frames) */
  bool                      enabled;
  bool                      streaming;
  mutex_t                   lock;
  sem_t                     run;
  uint8_t                   fifobuf[BMI_FIFO_MAX_READ * BMI_FIFO_FRAME];
};

/****************************************************************************
 * Private Functions - SPI
 ****************************************************************************/

static void bmi055_lock(FAR struct bmi055_dev_s *dev)
{
  SPI_LOCK(dev->spi, true);
  SPI_SETMODE(dev->spi, BMI_SPI_MODE);
  SPI_SETBITS(dev->spi, 8);
  SPI_SETFREQUENCY(dev->spi, BMI_SPI_FREQ);
}

static void bmi055_write_reg(FAR struct bmi055_dev_s *dev, uint8_t reg,
                             uint8_t val)
{
  bmi055_lock(dev);
  SPI_SELECT(dev->spi, dev->devid, true);
  SPI_SEND(dev->spi, reg & 0x7f);
  SPI_SEND(dev->spi, val);
  SPI_SELECT(dev->spi, dev->devid, false);
  SPI_LOCK(dev->spi, false);
}

static uint8_t bmi055_read_reg(FAR struct bmi055_dev_s *dev, uint8_t reg)
{
  uint8_t val = 0;

  bmi055_lock(dev);
  SPI_SELECT(dev->spi, dev->devid, true);
  SPI_SEND(dev->spi, reg | BMI_READ_BIT);
  SPI_RECVBLOCK(dev->spi, &val, 1);
  SPI_SELECT(dev->spi, dev->devid, false);
  SPI_LOCK(dev->spi, false);
  return val;
}

static void bmi055_read_burst(FAR struct bmi055_dev_s *dev, uint8_t reg,
                              FAR uint8_t *buf, size_t len)
{
  bmi055_lock(dev);
  SPI_SELECT(dev->spi, dev->devid, true);
  SPI_SEND(dev->spi, reg | BMI_READ_BIT);
  SPI_RECVBLOCK(dev->spi, buf, len);   /* DMA for len over the threshold */
  SPI_SELECT(dev->spi, dev->devid, false);
  SPI_LOCK(dev->spi, false);
}

static void bmi055_modify(FAR struct bmi055_dev_s *dev, uint8_t reg,
                          uint8_t set, uint8_t clear)
{
  uint8_t v = bmi055_read_reg(dev, reg);
  uint8_t n = (v & ~clear) | set;

  if (n != v)
    {
      bmi055_write_reg(dev, reg, n);
    }
}

/****************************************************************************
 * Private Functions - configuration
 ****************************************************************************/

/* The BMI055 accel powers up in I2C mode; the first SPI transaction after
 * power-on (and after every soft reset) is consumed switching it to SPI and
 * returns garbage. Issue a throwaway read so the next one is valid.
 */

static void bmi055_spi_mode_switch(FAR struct bmi055_dev_s *dev)
{
  bmi055_read_reg(dev, BMI_REG_CHIPID);
  nxsig_usleep(1000);
}

static void bmi055_fifo_flush(FAR struct bmi055_dev_s *dev)
{
  uint8_t cfg0 = dev->is_gyro ? GYR_REG_FIFO_CONFIG_0 : ACC_REG_FIFO_CONFIG_0;

  /* A FIFO overrun can only be cleared by writing FIFO_CONFIG_1; disable the
   * FIFO, then re-arm the watermark and FIFO mode.
   */

  bmi055_write_reg(dev, BMI_REG_FIFO_CONFIG_1, 0x00);
  bmi055_write_reg(dev, cfg0, BMI_FIFO_WM_SAMPLES);
  bmi055_write_reg(dev, BMI_REG_FIFO_CONFIG_1, BMI_FIFO_MODE);
}

static int bmi055_configure(FAR struct bmi055_dev_s *dev)
{
  uint8_t want = dev->is_gyro ? GYR_CHIPID_VAL : ACC_CHIPID_VAL;
  uint8_t id;

  bmi055_spi_mode_switch(dev);

  bmi055_write_reg(dev, BMI_REG_SOFTRESET, BMI_SOFTRESET_VAL);
  nxsig_usleep(30000);              /* 25 ms+ for the die to come back */

  bmi055_spi_mode_switch(dev);      /* reset drops it back to I2C mode */

  id = bmi055_read_reg(dev, BMI_REG_CHIPID);
  if (id != want)
    {
      snerr("ERROR: BMI055 %s chip-id=0x%02x (want 0x%02x)\n",
            dev->is_gyro ? "gyro" : "accel", id, want);
      return -ENODEV;
    }

  if (dev->is_gyro)
    {
      /* +/-2000 dps, unfiltered, FIFO watermark -> INT1 (push-pull, low) */

      bmi055_write_reg(dev, GYR_REG_RANGE, GYR_RANGE_2000DPS);
      bmi055_modify(dev, GYR_REG_RATE_HBW, GYR_HBW_UNFILTERED, 0x00);
      bmi055_modify(dev, GYR_REG_INT_EN_0, GYR_INT_FIFO_EN, 0x00);
      bmi055_modify(dev, GYR_REG_INT_EN_1, 0x00, GYR_INT1_OD_LVL);
      bmi055_modify(dev, GYR_REG_INT_MAP_1, GYR_INT1_FIFO, 0x00);
      bmi055_modify(dev, GYR_REG_FIFO_WM_ENABLE, GYR_FIFO_WM_ENABLE, 0x00);
      bmi055_write_reg(dev, GYR_REG_FIFO_CONFIG_0,
                       BMI_FIFO_WM_SAMPLES & ~GYR_FIFO_CFG0_TAG);
    }
  else
    {
      /* +/-16 g, unfiltered, FIFO watermark -> INT1 (push-pull, low) */

      bmi055_modify(dev, ACC_REG_PMU_RANGE, ACC_RANGE_16G_SET,
                    ACC_RANGE_16G_CLR);
      bmi055_modify(dev, ACC_REG_ACCD_HBW, ACC_HBW_UNFILTERED, 0x00);
      bmi055_modify(dev, ACC_REG_INT_EN_1, ACC_INT_FWM_EN, 0x00);
      bmi055_modify(dev, ACC_REG_INT_MAP_1, ACC_INT1_FWM, 0x00);
      bmi055_modify(dev, ACC_REG_INT_OUT_CTRL, 0x00, ACC_INT1_OD_LVL);
      bmi055_write_reg(dev, ACC_REG_FIFO_CONFIG_0, BMI_FIFO_WM_SAMPLES);
    }

  bmi055_write_reg(dev, BMI_REG_FIFO_CONFIG_1, BMI_FIFO_MODE);

  sninfo("BMI055 %s configured for 2 kHz FIFO streaming\n",
         dev->is_gyro ? "gyro" : "accel");
  return OK;
}

/****************************************************************************
 * Private Functions - sampling
 ****************************************************************************/

static void bmi055_publish(FAR struct bmi055_dev_s *dev,
                           FAR const uint8_t *f, float temp, uint64_t ts)
{
  /* Both dies emit 6-byte little-endian XYZ frames. The accel is 12-bit,
   * left-justified in 16 bits, so it needs an arithmetic >> 4 to sign-extend.
   */

  int16_t x = (int16_t)((uint16_t)f[1] << 8 | f[0]);
  int16_t y = (int16_t)((uint16_t)f[3] << 8 | f[2]);
  int16_t z = (int16_t)((uint16_t)f[5] << 8 | f[4]);

  if (dev->is_gyro)
    {
      struct sensor_gyro g;

      g.timestamp   = ts;
      g.x           = (float)x * BMI_GYRO_SCALE;
      g.y           = (float)y * BMI_GYRO_SCALE;
      g.z           = (float)z * BMI_GYRO_SCALE;
      g.temperature = temp;
      dev->lower.push_event(dev->lower.priv, &g, sizeof(g));
    }
  else
    {
      struct sensor_accel a;

      a.timestamp   = ts;
      a.x           = (float)(x >> 4) * BMI_ACCEL_SCALE;
      a.y           = (float)(y >> 4) * BMI_ACCEL_SCALE;
      a.z           = (float)(z >> 4) * BMI_ACCEL_SCALE;
      a.temperature = temp;
      dev->lower.push_event(dev->lower.priv, &a, sizeof(a));
    }
}

static void bmi055_drain_fifo(FAR struct bmi055_dev_s *dev)
{
  uint8_t  status = bmi055_read_reg(dev, BMI_REG_FIFO_STATUS);
  uint8_t  total;
  uint8_t  remaining;
  uint16_t idx = 0;
  uint64_t base;
  float    temp;

  if (status & BMI_FIFO_OVERRUN)
    {
      bmi055_fifo_flush(dev);
      return;
    }

  total = status & BMI_FIFO_FRAME_MASK;
  if (total == 0)
    {
      return;
    }

  if (total > dev->depth)
    {
      /* Impossible frame count - the FIFO state is bad, resync */

      bmi055_fifo_flush(dev);
      return;
    }

  /* Temperature is not carried in the FIFO; sample it once per batch */

  temp = (float)(int8_t)bmi055_read_reg(dev, BMI_REG_TEMP) * BMI_TEMP_SCALE +
         BMI_TEMP_OFFSET;

  /* Back-date each sample from the read time so every one carries its own
   * monotonic timestamp at the 2 kHz period (correct dt for fusion).
   */

  base = sensor_get_timestamp() -
         (uint64_t)(total - 1) * BMI_SAMPLE_PERIOD_US;

  remaining = total;
  while (remaining > 0)
    {
      uint8_t n = remaining > BMI_FIFO_MAX_READ ? BMI_FIFO_MAX_READ :
                                                  remaining;
      int i;

      bmi055_read_burst(dev, BMI_REG_FIFO_DATA, dev->fifobuf,
                        (size_t)n * BMI_FIFO_FRAME);

      for (i = 0; i < n; i++, idx++)
        {
          bmi055_publish(dev, dev->fifobuf + i * BMI_FIFO_FRAME, temp,
                         base + (uint64_t)idx * BMI_SAMPLE_PERIOD_US);
        }

      remaining -= n;
    }
}

/****************************************************************************
 * Private Functions - interrupt + thread
 ****************************************************************************/

static int bmi055_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct bmi055_dev_s *dev = (FAR struct bmi055_dev_s *)arg;

  nxsem_post(&dev->run);
  return OK;
}

static int bmi055_thread(int argc, FAR char **argv)
{
  FAR struct bmi055_dev_s *dev =
      (FAR struct bmi055_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  for (; ; )
    {
      /* Wait for the watermark interrupt; the timeout is a watchdog so a
       * missed edge still gets drained rather than stalling forever.
       */

      nxsem_tickwait(&dev->run, MSEC2TICK(BMI_WATCHDOG_MS));

      if (dev->streaming)
        {
          bmi055_drain_fifo(dev);
        }
    }

  return 0;
}

static void bmi055_stream_start(FAR struct bmi055_dev_s *dev)
{
  bmi055_fifo_flush(dev);
  dev->streaming = true;
  stm32_gpiosetevent(dev->drdy, false, true, true, bmi055_isr, dev);
}

static void bmi055_stream_stop(FAR struct bmi055_dev_s *dev)
{
  stm32_gpiosetevent(dev->drdy, false, false, false, NULL, NULL);
  dev->streaming = false;
}

/****************************************************************************
 * Private Functions - uorb ops
 ****************************************************************************/

static int bmi055_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  FAR struct bmi055_dev_s *dev = (FAR struct bmi055_dev_s *)lower;

  nxmutex_lock(&dev->lock);

  if (enable && !dev->streaming)
    {
      dev->enabled = true;
      bmi055_stream_start(dev);
    }
  else if (!enable && dev->streaming)
    {
      dev->enabled = false;
      bmi055_stream_stop(dev);
    }

  nxmutex_unlock(&dev->lock);
  return OK;
}

static int bmi055_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us)
{
  /* Hardware-timed 2 kHz FIFO stream: the rate is fixed. Report the true
   * sample period.
   */

  *period_us = BMI_SAMPLE_PERIOD_US;
  return OK;
}

static const struct sensor_ops_s g_bmi055_ops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  bmi055_activate,
  bmi055_set_interval,
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
 * Private Functions - per-die construction
 ****************************************************************************/

static int bmi055_init_one(FAR struct spi_dev_s *spi, int devno, bool is_gyro,
                           FAR const char *name,
                           FAR struct bmi055_dev_s **out)
{
  FAR struct bmi055_dev_s *dev;
  FAR char *argv[2];
  char arg1[16];
  int ret;

  dev = kmm_zalloc(sizeof(struct bmi055_dev_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->spi     = spi;
  dev->is_gyro = is_gyro;

  if (is_gyro)
    {
      dev->devid       = SPIDEV_ACCELEROMETER(FMUV6C_SPIDEV_BMI088_GYRO);
      dev->drdy        = GPIO_DRDY_BMI088_GYRO;
      dev->depth       = GYR_FIFO_DEPTH;
      dev->lower.type  = SENSOR_TYPE_GYROSCOPE;
    }
  else
    {
      dev->devid       = SPIDEV_ACCELEROMETER(FMUV6C_SPIDEV_BMI088_ACCEL);
      dev->drdy        = GPIO_DRDY_BMI088_ACCEL;
      dev->depth       = ACC_FIFO_DEPTH;
      dev->lower.type  = SENSOR_TYPE_ACCELEROMETER;
    }

  dev->lower.ops     = &g_bmi055_ops;
  dev->lower.nbuffer = BMI_UORB_NBUFFER;

  nxmutex_init(&dev->lock);
  nxsem_init(&dev->run, 0, 0);

  ret = bmi055_configure(dev);
  if (ret < 0)
    {
      goto errout;
    }

  ret = sensor_register(&dev->lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: BMI055 %s sensor_register failed: %d\n", name, ret);
      goto errout;
    }

  snprintf(arg1, sizeof(arg1), "%p", dev);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create(name, SCHED_PRIORITY_DEFAULT, 2048,
                       bmi055_thread, argv);
  if (ret < 0)
    {
      snerr("ERROR: BMI055 %s thread create failed: %d\n", name, ret);
      sensor_unregister(&dev->lower, devno);
      goto errout;
    }

  *out = dev;
  return OK;

errout:
  nxmutex_destroy(&dev->lock);
  nxsem_destroy(&dev->run);
  kmm_free(dev);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bmi055_register(FAR struct spi_dev_s *spi, int devno)
{
  FAR struct bmi055_dev_s *accel = NULL;
  FAR struct bmi055_dev_s *gyro  = NULL;
  int ret;

  ret = bmi055_init_one(spi, devno, false, "bmi055_acc", &accel);
  if (ret < 0)
    {
      return ret;
    }

  ret = bmi055_init_one(spi, devno, true, "bmi055_gyr", &gyro);
  if (ret < 0)
    {
      /* The accel is up and useful on its own; report the gyro failure but
       * leave the accel streaming.
       */

      snerr("ERROR: BMI055 gyro init failed (%d), accel still active\n", ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_SENSORS && CONFIG_SPI */
