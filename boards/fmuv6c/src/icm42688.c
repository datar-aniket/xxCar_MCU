/****************************************************************************
 * boards/fmuv6c/src/icm42688.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ICM-42688-P primary IMU on the NuttX uorb framework (sensor_accel +
 * sensor_gyro), FIFO + DRDY-interrupt streaming.
 *
 * The device runs its internal 2 kHz ODR and buffers accel+gyro+temp samples
 * in its hardware FIFO. When the FIFO reaches the watermark it pulses INT1
 * (active-low), wired to PE6 (GPIO_DRDY_ICM42688). The falling edge fires a
 * GPIO interrupt whose ISR just posts a semaphore; a kthread then drains the
 * FIFO in one DMA burst and publishes each sample. This decouples sampling
 * from the OS tick entirely (no polling, no phase-locking) and amortises the
 * per-read overhead across a whole watermark of samples -> true 2 kHz at low
 * CPU.
 *
 * Register configuration and reset sequence mirror PX4's hardware-proven
 * icm42688p driver, including its 20-byte high-res FIFO packet (20-bit
 * accel/gyro). Hi-res is deliberate for a ground robot: sharp mechanical
 * shocks (curbs, potholes, hard contacts) produce much larger instantaneous
 * peaks than a drone's smooth flight, and the extended range avoids clipping
 * the accelerometer while preserving resolution during quiet motion.
 *
 * Output is SI (accel m/s^2, gyro rad/s), raw sensor axes; board-frame
 * rotation is applied later in fusion.
 *
 * Config: accel +/-16 g, gyro +/-2000 dps. In 20-bit hi-res the raw counts are
 * 32768 LSB/g and 262 LSB/dps.
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
#include "icm42688.h"

#if defined(CONFIG_SENSORS) && defined(CONFIG_SPI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register map (bank 0) - subset used here */

#define ICM_REG_DEVICE_CONFIG   0x11
#define ICM_REG_INT_CONFIG      0x14
#define ICM_REG_FIFO_CONFIG     0x16
#define ICM_REG_INT_STATUS      0x2d
#define ICM_REG_FIFO_COUNTH     0x2e
#define ICM_REG_FIFO_DATA       0x30
#define ICM_REG_SIGNAL_PATH_RESET 0x4b
#define ICM_REG_INTF_CONFIG0    0x4c
#define ICM_REG_INTF_CONFIG1    0x4d
#define ICM_REG_PWR_MGMT0       0x4e
#define ICM_REG_GYRO_CONFIG0    0x4f
#define ICM_REG_ACCEL_CONFIG0   0x50
#define ICM_REG_GYRO_CONFIG1    0x51
#define ICM_REG_GYRO_ACCEL_CONFIG0 0x52
#define ICM_REG_ACCEL_CONFIG1   0x53
#define ICM_REG_TMST_CONFIG     0x54
#define ICM_REG_FIFO_CONFIG1    0x5f
#define ICM_REG_FIFO_CONFIG2    0x60
#define ICM_REG_FIFO_CONFIG3    0x61
#define ICM_REG_INT_CONFIG0     0x63
#define ICM_REG_INT_CONFIG1     0x64
#define ICM_REG_INT_SOURCE0     0x65
#define ICM_REG_WHO_AM_I        0x75
#define ICM_REG_BANK_SEL        0x76

#define ICM_WHO_AM_I_VAL        0x47
#define ICM_SOFT_RESET          0x01
#define ICM_RESET_DONE_INT      0x10   /* INT_STATUS bit4 */
#define ICM_PWR_ALL_LOWNOISE    0x0f   /* gyro LN | accel LN */
#define ICM_FIFO_FLUSH          0x02   /* SIGNAL_PATH_RESET bit1 */

#define ICM_READ_BIT            0x80
#define ICM_SPI_MODE            SPIDEV_MODE3
#define ICM_SPI_FREQ            8000000     /* 8 MHz (device max 24 MHz) */

/* FIFO 20-byte high-res packet (PX4 layout):
 *   [0]      header (accel+gyro+20bit+timestamp -> 0b0110_10xx)
 *   [1..6]   accel X/Y/Z high 16 bits (X1,X0,Y1,Y0,Z1,Z0)
 *   [7..12]  gyro  X/Y/Z high 16 bits
 *   [13..14] temperature (16-bit)
 *   [15..16] FIFO timestamp
 *   [17..19] extension: [accel[3:0]:gyro[3:0]] low nibbles for X, Y, Z
 */

#define ICM_FIFO_PACKET         20
#define ICM_FIFO_HW_SIZE        2048        /* device FIFO size (bytes) */
#define ICM_FIFO_WM_SAMPLES     8           /* watermark -> ~250 Hz INT */
#define ICM_FIFO_MAX_READ       25          /* per DMA read (*20=500 <= 512) */

#define ICM_HDR_MSG             0x80
#define ICM_HDR_ACCEL           0x40
#define ICM_HDR_GYRO            0x20
#define ICM_HDR_20BIT           0x10
#define ICM_HDR_TS_MASK         0x0c        /* timestamp/fsync field */
#define ICM_HDR_TS_ODR          0x08        /* ODR timestamp present */
#define ICM_ACCEL_INVALID       (-524288)   /* reassemble sentinel */

/* Scale factors for the 20-bit hi-res raw counts:
 *   accel +/-16 g  -> 32768 LSB/g   (524288 = 16 g)
 *   gyro  +/-2000dps -> 262 LSB/dps (524288 = 2000 dps)
 */

#define ICM_ACCEL_SCALE         (9.80665f / 32768.0f)
#define ICM_GYRO_SCALE          (0.017453292519943295f / 262.0f)
#define ICM_TEMP_SCALE          (1.0f / 132.48f)              /* 16-bit temp */
#define ICM_TEMP_OFFSET         25.0f

#define ICM_UORB_NBUFFER        32          /* holds several watermark bursts */
#define ICM_WATCHDOG_MS         20          /* fallback drain if an INT is missed */
#define ICM_SAMPLE_PERIOD_US    500         /* 2 kHz ODR -> 500 us per sample */

/****************************************************************************
 * Private Types
 ****************************************************************************/

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
  bool                      accel_en;
  bool                      gyro_en;
  bool                      streaming;  /* DRDY interrupt armed */
  mutex_t                   lock;
  sem_t                     run;
  uint8_t                   fifobuf[ICM_FIFO_MAX_READ * ICM_FIFO_PACKET];
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

static void icm42688_write_reg(FAR struct icm42688_dev_s *dev, uint8_t reg,
                               uint8_t val)
{
  icm42688_lock(dev);
  SPI_SELECT(dev->spi, dev->devid, true);
  SPI_SEND(dev->spi, reg & 0x7f);
  SPI_SEND(dev->spi, val);
  SPI_SELECT(dev->spi, dev->devid, false);
  SPI_LOCK(dev->spi, false);
}

static uint8_t icm42688_read_reg(FAR struct icm42688_dev_s *dev, uint8_t reg)
{
  uint8_t val = 0;

  icm42688_lock(dev);
  SPI_SELECT(dev->spi, dev->devid, true);
  SPI_SEND(dev->spi, reg | ICM_READ_BIT);
  SPI_RECVBLOCK(dev->spi, &val, 1);
  SPI_SELECT(dev->spi, dev->devid, false);
  SPI_LOCK(dev->spi, false);
  return val;
}

static void icm42688_read_burst(FAR struct icm42688_dev_s *dev, uint8_t reg,
                                FAR uint8_t *buf, size_t len)
{
  icm42688_lock(dev);
  SPI_SELECT(dev->spi, dev->devid, true);
  SPI_SEND(dev->spi, reg | ICM_READ_BIT);
  SPI_RECVBLOCK(dev->spi, buf, len);   /* DMA for len over the threshold */
  SPI_SELECT(dev->spi, dev->devid, false);
  SPI_LOCK(dev->spi, false);
}

/* Read-modify-write, preserving reserved/default bits (mirrors PX4). */

static void icm42688_modify(FAR struct icm42688_dev_s *dev, uint8_t reg,
                            uint8_t set, uint8_t clear)
{
  uint8_t v = icm42688_read_reg(dev, reg);
  uint8_t n = (v & ~clear) | set;

  if (n != v)
    {
      icm42688_write_reg(dev, reg, n);
    }
}

/****************************************************************************
 * Private Functions - configuration (register values from PX4 icm42688p)
 ****************************************************************************/

static int icm42688_configure(FAR struct icm42688_dev_s *dev)
{
  uint8_t id;

  /* Soft reset and let the device re-initialise */

  icm42688_write_reg(dev, ICM_REG_DEVICE_CONFIG, ICM_SOFT_RESET);
  nxsig_usleep(2000);

  id = icm42688_read_reg(dev, ICM_REG_WHO_AM_I);
  if (id != ICM_WHO_AM_I_VAL)
    {
      snerr("ERROR: ICM-42688 WHO_AM_I=0x%02x (want 0x%02x)\n",
            id, ICM_WHO_AM_I_VAL);
      return -ENODEV;
    }

  /* Wake accel + gyro (low-noise), then wait for the gyro to start (30 ms) */

  icm42688_write_reg(dev, ICM_REG_PWR_MGMT0, ICM_PWR_ALL_LOWNOISE);
  nxsig_usleep(30000);

  /* INT1: pulse->latched, push-pull, active-low (clear polarity) */

  icm42688_modify(dev, ICM_REG_INT_CONFIG, 0x06, 0x01);

  /* FIFO: stop-on-full mode */

  icm42688_modify(dev, ICM_REG_FIFO_CONFIG, 0xc0, 0x00);

  /* Big-endian FIFO count + sensor data, disable I2C interface */

  icm42688_modify(dev, ICM_REG_INTF_CONFIG0, 0x33, 0x00);

  /* Disable adaptive full-scale range (AFSR off) */

  icm42688_modify(dev, ICM_REG_INTF_CONFIG1, 0x40, 0x80);

  /* Full scale + ODR: gyro +/-2000 dps @2 kHz, accel +/-16 g @2 kHz.
   * ODR field [3:0]=0101 (2 kHz); FS field [7:5]=000 (max FS).
   */

  icm42688_modify(dev, ICM_REG_GYRO_CONFIG0,  0x05, 0xea);
  icm42688_modify(dev, ICM_REG_ACCEL_CONFIG0, 0x05, 0xea);

  /* UI filters: 1st order, BW = ODR/2 */

  icm42688_modify(dev, ICM_REG_GYRO_CONFIG1,        0x00, 0x0c);
  icm42688_modify(dev, ICM_REG_GYRO_ACCEL_CONFIG0,  0x00, 0xff);
  icm42688_modify(dev, ICM_REG_ACCEL_CONFIG1,       0x00, 0x18);

  /* Timestamp: enable (register + delta + to-regs, 16 us res), no FSYNC */

  icm42688_modify(dev, ICM_REG_TMST_CONFIG, 0x1d, 0x02);

  /* FIFO contents: watermark-gt-threshold, high-res, temp+gyro+accel;
   * no FSYNC tag -> 20-byte hi-res packets.
   */

  icm42688_modify(dev, ICM_REG_FIFO_CONFIG1, 0x37, 0x08);

  /* FIFO watermark (bytes) = samples * packet size */

  icm42688_write_reg(dev, ICM_REG_FIFO_CONFIG2,
                     (ICM_FIFO_WM_SAMPLES * ICM_FIFO_PACKET) & 0xff);
  icm42688_write_reg(dev, ICM_REG_FIFO_CONFIG3,
                     ((ICM_FIFO_WM_SAMPLES * ICM_FIFO_PACKET) >> 8) & 0x0f);

  /* Watermark interrupt clears on FIFO read; async-reset off; route to INT1 */

  icm42688_modify(dev, ICM_REG_INT_CONFIG0, 0x08, 0x00);
  icm42688_modify(dev, ICM_REG_INT_CONFIG1, 0x00, 0x10);
  icm42688_modify(dev, ICM_REG_INT_SOURCE0, 0x04, 0x00);

  sninfo("ICM-42688-P configured for 2 kHz FIFO streaming\n");
  return OK;
}

static void icm42688_fifo_flush(FAR struct icm42688_dev_s *dev)
{
  icm42688_modify(dev, ICM_REG_SIGNAL_PATH_RESET, ICM_FIFO_FLUSH, 0x00);
  nxsig_usleep(1000);
}

static uint16_t icm42688_fifo_count(FAR struct icm42688_dev_s *dev)
{
  uint8_t buf[2];

  icm42688_read_burst(dev, ICM_REG_FIFO_COUNTH, buf, 2);  /* big-endian */
  return ((uint16_t)buf[0] << 8) | buf[1];
}

/* Reassemble a 20-bit signed sample from its high byte [19:12], mid byte
 * [11:4] and low nibble [3:0] (PX4 icm42688p algorithm).
 */

static int32_t icm42688_reassemble20(uint8_t hi, uint8_t mid, uint8_t lo)
{
  uint32_t x = (((uint32_t)hi << 12) & 0x000ff000) |
               (((uint32_t)mid << 4) & 0x00000ff0) |
               ((uint32_t)lo & 0x0000000f);

  if (hi & 0x80)                 /* sign-extend from bit 19 */
    {
      x |= 0xfff00000u;
    }

  return (int32_t)x;
}

/* Parse and publish one 20-byte hi-res FIFO packet. Returns false only on a
 * bad header (framing lost -> caller should flush and resync). A single
 * data-invalid sample is skipped (returns true).
 */

static bool icm42688_publish(FAR struct icm42688_dev_s *dev,
                             FAR const uint8_t *p, uint64_t ts)
{
  uint8_t hdr = p[0];
  int32_t ax;
  int32_t ay;
  int32_t az;
  float   temp;

  if ((hdr & ICM_HDR_MSG) ||
      !(hdr & ICM_HDR_ACCEL) || !(hdr & ICM_HDR_GYRO) ||
      !(hdr & ICM_HDR_20BIT) ||
      ((hdr & ICM_HDR_TS_MASK) != ICM_HDR_TS_ODR))
    {
      return false;
    }

  /* accel low nibble = high nibble of the extension byte; gyro = low nibble */

  ax = icm42688_reassemble20(p[1], p[2], (p[17] >> 4) & 0x0f);
  ay = icm42688_reassemble20(p[3], p[4], (p[18] >> 4) & 0x0f);
  az = icm42688_reassemble20(p[5], p[6], (p[19] >> 4) & 0x0f);

  if (ax == ICM_ACCEL_INVALID || ay == ICM_ACCEL_INVALID ||
      az == ICM_ACCEL_INVALID)
    {
      return true;               /* invalid sample, skip but keep framing */
    }

  temp = (float)(int16_t)((uint16_t)p[13] << 8 | p[14]) * ICM_TEMP_SCALE +
         ICM_TEMP_OFFSET;

  if (dev->accel_en)
    {
      struct sensor_accel a;
      a.timestamp   = ts;
      a.x = (float)ax * ICM_ACCEL_SCALE;
      a.y = (float)ay * ICM_ACCEL_SCALE;
      a.z = (float)az * ICM_ACCEL_SCALE;
      a.temperature = temp;
      dev->accel.lower.push_event(dev->accel.lower.priv, &a, sizeof(a));
    }

  if (dev->gyro_en)
    {
      struct sensor_gyro g;
      g.timestamp   = ts;
      g.x = (float)icm42688_reassemble20(p[7],  p[8],  p[17] & 0x0f) *
            ICM_GYRO_SCALE;
      g.y = (float)icm42688_reassemble20(p[9],  p[10], p[18] & 0x0f) *
            ICM_GYRO_SCALE;
      g.z = (float)icm42688_reassemble20(p[11], p[12], p[19] & 0x0f) *
            ICM_GYRO_SCALE;
      g.temperature = temp;
      dev->gyro.lower.push_event(dev->gyro.lower.priv, &g, sizeof(g));
    }

  return true;
}

static void icm42688_drain_fifo(FAR struct icm42688_dev_s *dev)
{
  uint16_t count = icm42688_fifo_count(dev);
  uint16_t total;
  uint16_t remaining;
  uint16_t idx = 0;
  uint64_t base;

  if (count >= ICM_FIFO_HW_SIZE)
    {
      /* Overflow - discard and resync */

      icm42688_fifo_flush(dev);
      return;
    }

  total = count / ICM_FIFO_PACKET;
  if (total == 0)
    {
      return;
    }

  /* The batch was just read, so the newest sample is ~now and the oldest is
   * (total-1) sample periods earlier. Back-date each sample from that base so
   * every sample carries its own monotonic timestamp at the 2 kHz period.
   */

  base = sensor_get_timestamp() -
         (uint64_t)(total - 1) * ICM_SAMPLE_PERIOD_US;

  remaining = total;
  while (remaining > 0)
    {
      uint16_t n = remaining > ICM_FIFO_MAX_READ ? ICM_FIFO_MAX_READ :
                                                   remaining;
      int i;

      icm42688_read_burst(dev, ICM_REG_FIFO_DATA, dev->fifobuf,
                          (size_t)n * ICM_FIFO_PACKET);

      for (i = 0; i < n; i++, idx++)
        {
          uint64_t ts = base + (uint64_t)idx * ICM_SAMPLE_PERIOD_US;

          if (!icm42688_publish(dev, dev->fifobuf + i * ICM_FIFO_PACKET, ts))
            {
              /* Lost framing - flush and wait for the next watermark */

              icm42688_fifo_flush(dev);
              return;
            }
        }

      remaining -= n;
    }
}

/****************************************************************************
 * Private Functions - interrupt + thread
 ****************************************************************************/

static int icm42688_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct icm42688_dev_s *dev = (FAR struct icm42688_dev_s *)arg;

  nxsem_post(&dev->run);
  return OK;
}

static int icm42688_thread(int argc, FAR char **argv)
{
  FAR struct icm42688_dev_s *dev =
      (FAR struct icm42688_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  for (; ; )
    {
      /* Wait for the FIFO-watermark interrupt; the timeout is a watchdog so a
       * missed edge still gets drained (~50 Hz fallback), never a permanent
       * stall.
       */

      nxsem_tickwait(&dev->run, MSEC2TICK(ICM_WATCHDOG_MS));

      if (dev->streaming)
        {
          icm42688_drain_fifo(dev);
        }
    }

  return 0;
}

static void icm42688_stream_start(FAR struct icm42688_dev_s *dev)
{
  icm42688_fifo_flush(dev);
  dev->streaming = true;
  stm32_gpiosetevent(GPIO_DRDY_ICM42688, false, true, true,
                     icm42688_isr, dev);
}

static void icm42688_stream_stop(FAR struct icm42688_dev_s *dev)
{
  stm32_gpiosetevent(GPIO_DRDY_ICM42688, false, false, false, NULL, NULL);
  dev->streaming = false;
}

/****************************************************************************
 * Private Functions - uorb ops
 ****************************************************************************/

static int icm42688_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable)
{
  FAR struct icm42688_sensor_s *s = (FAR struct icm42688_sensor_s *)lower;
  FAR struct icm42688_dev_s *dev = s->dev;

  nxmutex_lock(&dev->lock);

  bool was_idle = (!dev->accel_en && !dev->gyro_en);

  if (s == &dev->accel)
    {
      dev->accel_en = enable;
    }
  else
    {
      dev->gyro_en = enable;
    }

  if (was_idle && (dev->accel_en || dev->gyro_en) && !dev->streaming)
    {
      icm42688_stream_start(dev);
    }
  else if (!dev->accel_en && !dev->gyro_en && dev->streaming)
    {
      icm42688_stream_stop(dev);
    }

  nxmutex_unlock(&dev->lock);
  return OK;
}

static int icm42688_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us)
{
  /* Hardware-timed 2 kHz FIFO stream: the ODR is fixed. Report the true
   * sample period so the upper half accounts for it correctly.
   */

  *period_us = 500;   /* 2 kHz */
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

  dev->spi   = spi;
  dev->devid = SPIDEV_IMU(FMUV6C_SPIDEV_ICM42688);

  dev->accel.dev            = dev;
  dev->accel.lower.ops      = &g_icm42688_ops;
  dev->accel.lower.type     = SENSOR_TYPE_ACCELEROMETER;
  dev->accel.lower.nbuffer  = ICM_UORB_NBUFFER;

  dev->gyro.dev             = dev;
  dev->gyro.lower.ops       = &g_icm42688_ops;
  dev->gyro.lower.type      = SENSOR_TYPE_GYROSCOPE;
  dev->gyro.lower.nbuffer   = ICM_UORB_NBUFFER;

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
  ret = kthread_create("icm42688", SCHED_PRIORITY_DEFAULT, 2048,
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
