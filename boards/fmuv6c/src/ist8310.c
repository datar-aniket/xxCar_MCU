/****************************************************************************
 * boards/fmuv6c/src/ist8310.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Lean IST8310 magnetometer driver on the NuttX uorb framework (sensor_mag).
 *
 * The IST8310 has no free-running mode: each sample is a single-shot
 * measurement (write CNTL1, wait ~6 ms, read 6 data bytes). To keep CPU near
 * zero we mirror the MS5611 driver's pipeline: trigger the next measurement,
 * then sleep the whole sample interval (the conversion completes during the
 * sleep), then read + publish. Exactly one nxsig_usleep per sample, no spin.
 *
 * Output: x/y/z in Gauss (sensor_mag units), RAW chip axes - the IST8310's
 * own +x forward, +y right, +z up. Nothing is flipped, scaled away or
 * rotated here.
 *
 * That is deliberate and not laziness. A calibration is fitted against this
 * stream, so anything this driver changes about the axes invalidates every
 * stored CAL_MAG0_* value. Handedness and mounting are both dealt with in
 * apps/sensors/mag_frame.c, AFTER the calibration has been applied, which is
 * what lets the frame be corrected without recalibrating the sensor.
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
#include "ist8310.h"

#if defined(CONFIG_SENSORS) && defined(CONFIG_I2C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IST8310_REG_WAI       0x00   /* WHO_AM_I -> 0x10                     */
#define IST8310_REG_STAT1     0x02   /* bit0 = DRDY                         */
#define IST8310_REG_DATA_XL   0x03   /* X L,H, Y L,H, Z L,H (auto-increment) */
#define IST8310_REG_CNTL1     0x0a   /* 0x01 = single measurement           */
#define IST8310_REG_CNTL2     0x0b   /* bit0 = SRST (soft reset)            */
#define IST8310_REG_AVGCNTL   0x41   /* sample averaging control            */
#define IST8310_REG_PDCNTL    0x42   /* pulse-duration control              */

#define IST8310_WAI_VAL       0x10
#define IST8310_STAT1_DRDY    0x01
#define IST8310_CNTL1_SINGLE  0x01
#define IST8310_CNTL2_SRST    0x01
#define IST8310_AVGCNTL_16X   0x24   /* 16x average on X/Y and Z (per PX4)  */
#define IST8310_PDCNTL_NORMAL 0xc0   /* normal pulse duration (datasheet)   */

/* 1 LSB = 0.3 uT = 0.003 Gauss (1 Gauss = 100 uT). */

#define IST8310_GAUSS_PER_LSB (0.3f / 100.0f)

#define IST8310_I2C_FREQ      400000
#define IST8310_MIN_INTERVAL  10000  /* us -> 100 Hz cap; measure ~6 ms     */
#define IST8310_DEFAULT_INTERVAL 20000 /* us -> 50 Hz default               */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ist8310_dev_s
{
  struct sensor_lowerhalf_s lower;
  FAR struct i2c_master_s  *i2c;
  uint8_t                   addr;
  uint32_t                  interval;  /* us */
  mutex_t                   lock;
  sem_t                     run;
  bool                      enabled;
};

/****************************************************************************
 * Private Functions - I2C
 ****************************************************************************/

static int ist8310_write_reg(FAR struct ist8310_dev_s *dev, uint8_t reg,
                             uint8_t val)
{
  struct i2c_msg_s msg;
  uint8_t buf[2];

  buf[0]        = reg;
  buf[1]        = val;
  msg.frequency = IST8310_I2C_FREQ;
  msg.addr      = dev->addr;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 2;
  return I2C_TRANSFER(dev->i2c, &msg, 1);
}

static int ist8310_read_regs(FAR struct ist8310_dev_s *dev, uint8_t reg,
                             FAR uint8_t *buf, size_t len)
{
  struct i2c_msg_s msg[2];

  msg[0].frequency = IST8310_I2C_FREQ;
  msg[0].addr      = dev->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = IST8310_I2C_FREQ;
  msg[1].addr      = dev->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buf;
  msg[1].length    = len;

  return I2C_TRANSFER(dev->i2c, msg, 2);
}

/****************************************************************************
 * Private Functions - device
 ****************************************************************************/

static int ist8310_configure(FAR struct ist8310_dev_s *dev)
{
  uint8_t val = 0;
  int ret;

  /* Soft reset, then let the device settle */

  ist8310_write_reg(dev, IST8310_REG_CNTL2, IST8310_CNTL2_SRST);
  nxsig_usleep(10000);

  ret = ist8310_read_regs(dev, IST8310_REG_WAI, &val, 1);
  if (ret < 0 || val != IST8310_WAI_VAL)
    {
      snerr("ERROR: IST8310 WHO_AM_I=0x%02x (want 0x%02x) ret=%d\n",
            val, IST8310_WAI_VAL, ret);
      return ret < 0 ? ret : -ENODEV;
    }

  /* Averaging + pulse-duration: reduces noise; values proven by PX4. */

  ist8310_write_reg(dev, IST8310_REG_AVGCNTL, IST8310_AVGCNTL_16X);
  ist8310_write_reg(dev, IST8310_REG_PDCNTL, IST8310_PDCNTL_NORMAL);

  sninfo("IST8310 configured (WHO_AM_I=0x%02x)\n", val);
  return OK;
}

/* Trigger a single measurement (non-blocking; completes in ~6 ms). */

static void ist8310_trigger(FAR struct ist8310_dev_s *dev)
{
  ist8310_write_reg(dev, IST8310_REG_CNTL1, IST8310_CNTL1_SINGLE);
}

static void ist8310_process(FAR struct ist8310_dev_s *dev)
{
  struct sensor_mag mag;
  uint8_t buf[6];
  int16_t x;
  int16_t y;
  int16_t z;

  if (ist8310_read_regs(dev, IST8310_REG_DATA_XL, buf, 6) < 0)
    {
      return;
    }

  x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
  y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
  z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

  /* Keep aiding sample time in the same TIM5 domain as the IMU trajectory
   * against which the EKF will fuse it.  Publication scheduling remains a
   * separate concern in the sensors daemon.
   */

  mag.timestamp   = fmuv6c_imu_time_now();
  mag.x           = (float)x * IST8310_GAUSS_PER_LSB;
  mag.y           = (float)y * IST8310_GAUSS_PER_LSB;
  mag.z           = (float)z * IST8310_GAUSS_PER_LSB;
  mag.temperature = 0.0f;   /* IST8310 temp is uncalibrated; not published */
  mag.status      = 0;
  dev->lower.push_event(dev->lower.priv, &mag, sizeof(mag));
}

/* Pipelined sampler: the single-shot measurement (~6 ms) runs while the
 * thread SLEEPS for the sample interval, so there is exactly one sleep per
 * sample -> the thread is essentially always sleeping (near-0% CPU).
 */

static int ist8310_thread(int argc, FAR char **argv)
{
  FAR struct ist8310_dev_s *dev =
      (FAR struct ist8310_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  /* Prime the first measurement so data is ready after the first sleep */

  ist8310_trigger(dev);

  while (true)
    {
      if (!dev->enabled)
        {
          nxsem_wait(&dev->run);
          ist8310_trigger(dev);   /* re-prime on re-activate */
          continue;
        }

      /* The pending measurement completes during this sleep */

      nxsig_usleep(dev->interval);

      /* Read the completed sample, then immediately start the next one so it
       * overlaps the next interval sleep.
       */

      ist8310_process(dev);
      ist8310_trigger(dev);
    }

  return 0;
}

/****************************************************************************
 * Private Functions - uorb ops
 ****************************************************************************/

static int ist8310_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct ist8310_dev_s *dev = (FAR struct ist8310_dev_s *)lower;
  bool start = false;

  nxmutex_lock(&dev->lock);
  if (enable && !dev->enabled)
    {
      dev->enabled = true;
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

static int ist8310_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us)
{
  FAR struct ist8310_dev_s *dev = (FAR struct ist8310_dev_s *)lower;

  if (*period_us < IST8310_MIN_INTERVAL)
    {
      *period_us = IST8310_MIN_INTERVAL;
    }

  dev->interval = *period_us;
  return OK;
}

static const struct sensor_ops_s g_ist8310_ops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  ist8310_activate,
  ist8310_set_interval,
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

int ist8310_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr)
{
  FAR struct ist8310_dev_s *dev;
  FAR char *argv[2];
  char arg1[16];
  int ret;

  dev = kmm_zalloc(sizeof(struct ist8310_dev_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->i2c        = i2c;
  dev->addr       = addr;
  dev->interval   = IST8310_DEFAULT_INTERVAL;
  dev->lower.ops  = &g_ist8310_ops;
  dev->lower.type = SENSOR_TYPE_MAGNETIC_FIELD;
  nxmutex_init(&dev->lock);
  nxsem_init(&dev->run, 0, 0);

  ret = ist8310_configure(dev);
  if (ret < 0)
    {
      goto errout;
    }

  ret = sensor_register(&dev->lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: IST8310 sensor_register failed: %d\n", ret);
      goto errout;
    }

  snprintf(arg1, sizeof(arg1), "%p", dev);
  argv[0] = arg1;
  argv[1] = NULL;
  /* 2048, not 1024: with CONFIG_STACK_COLORATION on, `ps` showed this thread at
   * 62.8% of a 1024-byte stack - ~360 bytes of headroom, and a stack overflow
   * here would take the board down with no crash dump. Matches the IMU threads.
   */

  ret = kthread_create("ist8310", FMUV6C_SENSOR_PRIO, 2048,
                       ist8310_thread, argv);
  if (ret < 0)
    {
      snerr("ERROR: IST8310 thread create failed: %d\n", ret);
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
