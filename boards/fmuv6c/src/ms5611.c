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
 * Compensation uses the datasheet's 64-bit first- and second-order algorithm.
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

#include "fmuv6c.h"
#include "ms5611.h"
#include "ms5611_comp.h"

#if defined(CONFIG_SENSORS) && defined(CONFIG_I2C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MS5611_CMD_RESET      0x1e
#define MS5611_CMD_CONV_D1    0x48   /* pressure,    OSR 4096 (~9.04 ms) */
#define MS5611_CMD_CONV_D2    0x58   /* temperature, OSR 4096 (~9.04 ms) */
#define MS5611_CMD_ADC_READ   0x00
#define MS5611_CMD_PROM_READ  0xa0   /* + (coef << 1); coef 0..7 */

#define MS5611_TEMP_DECIMATE  10     /* read temperature every Nth sample */
#define MS5611_I2C_FREQ       400000
#define MS5611_MIN_INTERVAL   20000  /* us -> 50 Hz cap; interval >= conv (~9ms) */
#define MS5611_DEFAULT_INTERVAL 100000 /* us -> 10 Hz default */
#define MS5611_CONVERSION_US  9040
#define MS5611_PRESSURE_MIN_HPA 10.0f
#define MS5611_PRESSURE_MAX_HPA 1200.0f
#define MS5611_TEMP_MIN_C      -40.0f
#define MS5611_TEMP_MAX_C       85.0f

/* Which conversion is currently in flight (pipelined with the interval sleep) */

#define MS5611_PENDING_D1     1       /* pressure    */
#define MS5611_PENDING_D2     2       /* temperature */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ms5611_dev_s
{
  struct sensor_lowerhalf_s lower;
  FAR struct i2c_master_s  *i2c;
  uint8_t                   addr;
  uint16_t                  c[8];      /* complete PROM including CRC */
  uint32_t                  raw_temperature;
  uint64_t                  pending_timestamp;
  uint32_t                  count;     /* sample counter for temp decimation */
  uint8_t                   pending;   /* conversion in flight (D1/D2) */
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

  /* Read the complete PROM: C1..C6 and the factory CRC nibble. */

  for (i = 0; i <= 7; i++)
    {
      ret = ms5611_read(dev, MS5611_CMD_PROM_READ + (i << 1), buf, 2);
      if (ret < 0)
        {
          snerr("ERROR: MS5611 PROM read %d failed: %d\n", i, ret);
          return ret;
        }

      dev->c[i] = ((uint16_t)buf[0] << 8) | buf[1];
    }

  if (!ms5611_prom_valid(dev->c))
    {
      snerr("ERROR: MS5611 bad PROM or CRC (stored=%x computed=%x)\n",
            dev->c[7] & 0x0f, ms5611_prom_crc4(dev->c));
      return -ENODEV;
    }

  sninfo("MS5611 PROM C1..C6: %u %u %u %u %u %u\n",
         dev->c[1], dev->c[2], dev->c[3], dev->c[4], dev->c[5], dev->c[6]);
  return OK;
}

/* Start a conversion (non-blocking) and remember which one is in flight. */

static void ms5611_start(FAR struct ms5611_dev_s *dev, uint8_t which)
{
  /* EKF sample timestamps use the shared TIM5 domain.  The conversion's
   * centre time must use that same clock; sensor_get_timestamp() is the
   * tick-quantized NuttX monotonic counter and slowly changes phase against
   * TIM5 between its 71.6-minute wrap re-anchors.
   */

  uint64_t trigger_timestamp = fmuv6c_imu_time_now();

  ms5611_sendcmd(dev, which == MS5611_PENDING_D2 ?
                      MS5611_CMD_CONV_D2 : MS5611_CMD_CONV_D1);
  dev->pending = which;

  if (which == MS5611_PENDING_D1)
    {
      /* Timestamp the centre of the ADC conversion, not the much later I2C
       * read. At 10 Hz the old read timestamp was about 95 ms too new. */

      dev->pending_timestamp = trigger_timestamp + MS5611_CONVERSION_US / 2;
    }
}

static void ms5611_process(FAR struct ms5611_dev_s *dev, uint8_t was,
                           uint32_t raw)
{
  if (was == MS5611_PENDING_D2)
    {
      dev->raw_temperature = raw;
    }
  else
    {
      struct sensor_baro baro;
      struct ms5611_compensated_s compensated;

      if (!ms5611_compensate(dev->c, raw, dev->raw_temperature,
                             &compensated))
        {
          return;
        }

      baro.timestamp = dev->pending_timestamp;
      baro.pressure = (float)compensated.pressure_centi_hpa / 100.0f;
      baro.temperature = (float)compensated.temperature_centi_c / 100.0f;

      if (baro.pressure < MS5611_PRESSURE_MIN_HPA ||
          baro.pressure > MS5611_PRESSURE_MAX_HPA ||
          baro.temperature < MS5611_TEMP_MIN_C ||
          baro.temperature > MS5611_TEMP_MAX_C)
        {
          return;
        }

      dev->lower.push_event(dev->lower.priv, &baro, sizeof(baro));
    }
}

/* Pipelined sampler: the ADC conversion (~9 ms) runs while the thread SLEEPS
 * for the sample interval, so there is exactly one sleep per sample and no
 * busy/extra conversion wait -> the thread is essentially always sleeping.
 */

static int ms5611_thread(int argc, FAR char **argv)
{
  FAR struct ms5611_dev_s *dev =
      (FAR struct ms5611_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  uint32_t raw;
  uint8_t  was;

  /* Prime with a temperature conversion so dT is valid for the 1st pressure */

  ms5611_start(dev, MS5611_PENDING_D2);

  while (true)
    {
      if (!dev->enabled)
        {
          nxsem_wait(&dev->run);
          ms5611_start(dev, MS5611_PENDING_D2);   /* re-prime on re-activate */
          continue;
        }

      /* The pending conversion completes during this sleep */

      nxsig_usleep(dev->interval);

      raw = 0;
      if (ms5611_read_adc(dev, &raw) < 0)
        {
          continue;
        }

      was = dev->pending;

      /* Start the next conversion immediately (overlaps the next interval):
       * temperature every Nth sample, pressure otherwise.
       */

      if (++dev->count >= MS5611_TEMP_DECIMATE)
        {
          dev->count = 0;
          ms5611_start(dev, MS5611_PENDING_D2);
        }
      else
        {
          ms5611_start(dev, MS5611_PENDING_D1);
        }

      ms5611_process(dev, was, raw);
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
  /* 2048, not 1024: with CONFIG_STACK_COLORATION on, `ps` showed this thread at
   * 62.8% of a 1024-byte stack - ~360 bytes of headroom, and a stack overflow
   * here would take the board down with no crash dump. Matches the IMU threads.
   */

  ret = kthread_create("ms5611", FMUV6C_SENSOR_PRIO, 2048,
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
