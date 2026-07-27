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
#define ICM_FIFO_CONFIG_BITS    0x17   /* hires | temp | gyro | accel */
#define ICM_FIFO_WM_GT_TH       0x20   /* repeat threshold event per ODR */
#define ICM_FIFO_THS_INT1_EN    0x04   /* route FIFO threshold to INT1 */

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
/* Scale factors for the 20-bit hi-res raw counts:
 *   accel +/-16 g  -> 32768 LSB/g   (524288 = 16 g)
 *   gyro  +/-2000dps -> 262 LSB/dps (524288 = 2000 dps)
 */

#define ICM_ACCEL_SCALE         (9.80665f / 32768.0f)
#define ICM_GYRO_SCALE          (0.017453292519943295f / 262.0f)
#define ICM_TEMP_SCALE          (1.0f / 132.48f)              /* 16-bit temp */
#define ICM_TEMP_OFFSET         25.0f

#define ICM_UORB_NBUFFER        1280        /* 640 ms at 2 kHz. Routine full-rate
                                             * SD flushes reach the old 128 ms
                                             * limit, and observed card stalls
                                             * reach about 500 ms. */
#define ICM_WATCHDOG_MS         20          /* fallback drain if an INT is missed */
#define ICM_TIMESTAMP_FRAC_BITS 5
#define ICM_NOMINAL_PERIOD_Q5   (500ull << ICM_TIMESTAMP_FRAC_BITS)
#define ICM_RATE_WINDOW_US      1000000ull   /* update once per TIM5 second */
#define ICM_PERIOD_AVG_SAMPLES  4           /* four-second moving average */
#define ICM_PERIOD_MIN_Q5       (450ull << ICM_TIMESTAMP_FRAC_BITS)
#define ICM_PERIOD_MAX_Q5       (550ull << ICM_TIMESTAMP_FRAC_BITS)

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
  uint64_t                  last_timestamp_q5;
  uint64_t                  sample_period_q5;
  uint64_t                  sample_count;
  uint64_t                  rate_anchor_sample;
  uint64_t                  rate_anchor_timestamp;
  uint64_t                  period_history_q5[ICM_PERIOD_AVG_SAMPLES];
  uint64_t                  period_history_sum_q5;
  uint8_t                   period_history_index;
  volatile uint64_t         drdy_timestamp;
  volatile uint32_t         drdy_sequence;
  volatile uint32_t         consumed_drdy_sequence;
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

static bool icm42688_check_bits(FAR struct icm42688_dev_s *dev,
                                uint8_t reg, uint8_t set, uint8_t clear)
{
  uint8_t value = icm42688_read_reg(dev, reg);

  if ((value & set) != set || (value & clear) != 0)
    {
      snerr("ERROR: ICM-42688 reg 0x%02x=0x%02x"
            " (set 0x%02x clear 0x%02x)\n",
            reg, value, set, clear);
      return false;
    }

  return true;
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

  /* Big-endian FIFO count + sensor data, disable I2C interface.
   *
   * Hold the last valid value when one accel/gyro ODR slot is invalid. In
   * 20-bit FIFO mode the alternative is the -524288 sentinel. With this bit
   * set -524288 is also a legitimate full-scale value, so the packet parser
   * must not treat that value as an invalid marker.
   */

  icm42688_modify(dev, ICM_REG_INTF_CONFIG0, 0xb3, 0x00);

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

  /* Leave FIFO packet generation disabled until a subscriber starts the
   * stream. Board probe runs well before the logger opens the uORB nodes. If
   * packet generation and the latched INT1 route are enabled here, the FIFO
   * reaches its threshold while PE6 EXTI is still detached and holds INT1
   * low. Attaching a falling-edge interrupt later cannot observe that already
   * asserted level, so every drain comes from the 20 ms watchdog.
   *
   * FIFO_WM_GT_TH can be configured while idle. The packet-content bits and
   * INT1 route are enabled only after PE6 has been armed in stream_start().
   */

  icm42688_modify(dev, ICM_REG_FIFO_CONFIG1, ICM_FIFO_WM_GT_TH, 0x1f);

  /* FIFO watermark (bytes) = samples * packet size */

  icm42688_write_reg(dev, ICM_REG_FIFO_CONFIG2,
                     (ICM_FIFO_WM_SAMPLES * ICM_FIFO_PACKET) & 0xff);
  icm42688_write_reg(dev, ICM_REG_FIFO_CONFIG3,
                     ((ICM_FIFO_WM_SAMPLES * ICM_FIFO_PACKET) >> 8) & 0x0f);

  /* Watermark interrupt clears on FIFO read and async-reset is off. Keep the
   * INT1 route disabled until PE6 EXTI is armed by stream_start().
   */

  icm42688_modify(dev, ICM_REG_INT_CONFIG0, 0x08, 0x00);
  icm42688_modify(dev, ICM_REG_INT_CONFIG1, 0x00, 0x10);
  /* INT_SOURCE0 resets to 0x10, which routes RESET_DONE to INT1. Do not use
   * read-modify-write here: preserving that reset default can leave the
   * latched active-low INT1 asserted before EXTI is armed. No INT1 source is
   * wanted while idle.
   */

  icm42688_write_reg(dev, ICM_REG_INT_SOURCE0, 0x00);

  /* PX4 verifies its register configuration instead of assuming an SPI write
   * stuck. Do the same for the timing-critical bank-0 registers before the
   * driver is exposed to applications.
   */

  if (!icm42688_check_bits(dev, ICM_REG_INT_CONFIG, 0x06, 0x01) ||
      !icm42688_check_bits(dev, ICM_REG_FIFO_CONFIG, 0xc0, 0x00) ||
      !icm42688_check_bits(dev, ICM_REG_FIFO_CONFIG1,
                           ICM_FIFO_WM_GT_TH, 0x1f) ||
      icm42688_read_reg(dev, ICM_REG_FIFO_CONFIG2) !=
        ICM_FIFO_WM_SAMPLES * ICM_FIFO_PACKET ||
      (icm42688_read_reg(dev, ICM_REG_FIFO_CONFIG3) & 0x0f) != 0 ||
      !icm42688_check_bits(dev, ICM_REG_INT_CONFIG0, 0x08, 0x04) ||
      !icm42688_check_bits(dev, ICM_REG_INT_CONFIG1, 0x00, 0x10) ||
      icm42688_read_reg(dev, ICM_REG_INT_SOURCE0) != 0x00)
    {
      snerr("ERROR: ICM-42688 FIFO/interrupt configuration verification"
            " failed\n");
      return -EIO;
    }

  sninfo("ICM-42688-P configured for 2 kHz FIFO streaming\n");
  return OK;
}

static void icm42688_fifo_flush(FAR struct icm42688_dev_s *dev)
{
  irqstate_t flags;

  icm42688_modify(dev, ICM_REG_SIGNAL_PATH_RESET, ICM_FIFO_FLUSH, 0x00);
  nxsig_usleep(1000);

  /* A pre-flush edge cannot be associated with the new FIFO epoch. Consume
   * the sequence atomically so the next watermark is treated as one fresh
   * hardware anchor rather than a coalesced stale edge.
   */

  flags = enter_critical_section();
  dev->consumed_drdy_sequence = dev->drdy_sequence;
  leave_critical_section(flags);

  dev->rate_anchor_timestamp = 0;
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
 * bad header (framing lost -> caller should flush and resync).
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

/* Measure the primary IMU's actual FIFO period from watermark edges captured
 * by the same TIM5 clock used by the BMI055. The ICM accel and gyro share one
 * FIFO packet and one sample clock, so one absolute packet counter represents
 * both sensor streams.
 *
 * Update only after one elapsed TIM5 second. Accepted observations replace
 * one entry in a four-value moving average seeded at the nominal 500 us
 * period. Out-of-order anchors and observations outside the bounded 2 kHz
 * neighborhood are ignored without disturbing the active timestamp period.
 */

static void icm42688_update_period(FAR struct icm42688_dev_s *dev,
                                   uint64_t edge_timestamp,
                                   uint64_t edge_sample)
{
  uint64_t delta_samples;
  uint64_t delta_us;
  uint64_t observed_q5;
  uint64_t replaced_q5;

  if (dev->rate_anchor_timestamp == 0)
    {
      dev->rate_anchor_timestamp = edge_timestamp;
      dev->rate_anchor_sample = edge_sample;
      return;
    }

  if (edge_timestamp <= dev->rate_anchor_timestamp ||
      edge_sample <= dev->rate_anchor_sample)
    {
      dev->rate_anchor_timestamp = edge_timestamp;
      dev->rate_anchor_sample = edge_sample;
      return;
    }

  delta_us = edge_timestamp - dev->rate_anchor_timestamp;
  if (delta_us < ICM_RATE_WINDOW_US)
    {
      return;
    }

  delta_samples = edge_sample - dev->rate_anchor_sample;
  observed_q5 = ((delta_us << ICM_TIMESTAMP_FRAC_BITS) +
                 delta_samples / 2) / delta_samples;

  if (observed_q5 >= ICM_PERIOD_MIN_Q5 &&
      observed_q5 <= ICM_PERIOD_MAX_Q5)
    {
      replaced_q5 =
        dev->period_history_q5[dev->period_history_index];
      dev->period_history_sum_q5 -= replaced_q5;
      dev->period_history_q5[dev->period_history_index] = observed_q5;
      dev->period_history_sum_q5 += observed_q5;
      dev->period_history_index =
        (dev->period_history_index + 1) % ICM_PERIOD_AVG_SAMPLES;
      dev->sample_period_q5 =
        (dev->period_history_sum_q5 + ICM_PERIOD_AVG_SAMPLES / 2) /
        ICM_PERIOD_AVG_SAMPLES;
    }

  dev->rate_anchor_timestamp = edge_timestamp;
  dev->rate_anchor_sample = edge_sample;
}

static void icm42688_drain_fifo(FAR struct icm42688_dev_s *dev)
{
  uint16_t count = icm42688_fifo_count(dev);
  uint16_t total;
  uint16_t remaining;
  uint16_t idx = 0;
  uint64_t base_q5;
  uint64_t period_q5;
  uint64_t batch_now_q5;
  uint64_t batch_span_q5;
  uint64_t causal_base_q5;
  uint64_t drdy_timestamp;
  uint64_t watermark_edge_sample;
  uint32_t drdy_sequence;
  uint32_t drdy_events;
  irqstate_t flags;

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

  /* Snapshot the ISR-written 64-bit timestamp with interrupts excluded so a
   * 32-bit core cannot observe a torn value.
   */

  flags = enter_critical_section();
  drdy_timestamp = dev->drdy_timestamp;
  drdy_sequence  = dev->drdy_sequence;
  drdy_events = drdy_sequence - dev->consumed_drdy_sequence;
  if (drdy_events != 0)
    {
      dev->consumed_drdy_sequence = drdy_sequence;
    }

  leave_critical_section(flags);

  /* A fresh watermark edge identifies the absolute packet at the programmed
   * FIFO threshold. Use that physical edge/sample pair for the one-second
   * period estimator before reconstructing this batch.
   */

  if (drdy_events != 0 && total >= ICM_FIFO_WM_SAMPLES)
    {
      watermark_edge_sample = dev->sample_count +
                              ICM_FIFO_WM_SAMPLES - 1;
      icm42688_update_period(dev, drdy_timestamp,
                             watermark_edge_sample);
    }

  period_q5 = dev->sample_period_q5;

  /* Build this batch from a physical TIM5 reference, never from the preceding
   * batch. A unique watermark edge identifies the packet at the FIFO threshold
   * and removes worker scheduling latency from its timestamps. The current
   * TIM5 snapshot remains the causal upper bound: the newest packet must be at
   * least one measured period old.
   */

  batch_now_q5 =
    fmuv6c_imu_time_now() << ICM_TIMESTAMP_FRAC_BITS;
  batch_span_q5 = (uint64_t)total * period_q5;

  if (batch_now_q5 <= batch_span_q5)
    {
      icm42688_fifo_flush(dev);
      return;
    }

  causal_base_q5 = batch_now_q5 - batch_span_q5;
  base_q5 = causal_base_q5;

  if (drdy_events != 0 && total >= ICM_FIFO_WM_SAMPLES)
    {
      uint64_t edge_q5 =
        drdy_timestamp << ICM_TIMESTAMP_FRAC_BITS;
      uint64_t edge_span_q5 =
        (uint64_t)ICM_FIFO_WM_SAMPLES * period_q5;

      if (edge_q5 > edge_span_q5)
        {
          uint64_t edge_base_q5 = edge_q5 - edge_span_q5;

          /* Accept only an edge that lies between the prior published packet
           * and the current-time causal bound.
           */

          if (edge_base_q5 <= causal_base_q5 &&
              (dev->last_timestamp_q5 == 0 ||
               edge_base_q5 > dev->last_timestamp_q5))
            {
              base_q5 = edge_base_q5;
            }
        }
    }

  /* The causal fallback normally advances beyond the preceding batch. If it
   * does not, no fixed-period assignment can be both causal and monotonic, so
   * discard rather than emit impossible timestamps.
   */

  if (dev->last_timestamp_q5 != 0 &&
      base_q5 <= dev->last_timestamp_q5)
    {
      icm42688_fifo_flush(dev);
      return;
    }

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
          uint64_t ts_q5 = base_q5 + (uint64_t)idx * period_q5;
          uint64_t ts =
            (ts_q5 + (1ull << (ICM_TIMESTAMP_FRAC_BITS - 1))) >>
            ICM_TIMESTAMP_FRAC_BITS;

          if (!icm42688_publish(dev, dev->fifobuf + i * ICM_FIFO_PACKET, ts))
            {
              /* Lost framing - flush and wait for the next watermark */

              icm42688_fifo_flush(dev);
              return;
            }

          dev->last_timestamp_q5 = ts_q5;
        }

      remaining -= n;
    }

  dev->sample_count += total;
}

/****************************************************************************
 * Private Functions - interrupt + thread
 ****************************************************************************/

static int icm42688_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct icm42688_dev_s *dev = (FAR struct icm42688_dev_s *)arg;

  /* Preserve the first edge that has not yet been consumed. In latched mode
   * the first falling edge is the physical FIFO-threshold crossing. Replacing
   * it with a later retrigger would associate the timestamp with the wrong
   * packet, while rejecting a coalesced sequence would throw the only useful
   * anchor away.
   */

  if (dev->drdy_sequence == dev->consumed_drdy_sequence)
    {
      dev->drdy_timestamp = fmuv6c_imu_time_now();
      dev->drdy_sequence++;
      nxsem_post(&dev->run);
    }
  else
    {
      dev->drdy_sequence++;
    }

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

static int icm42688_stream_start(FAR struct icm42688_dev_s *dev)
{
  int ret;
  int i;

  /* First force INT1 inactive and stop all FIFO writes. This clears the idle
   * state left by board probe or a previous subscriber without depending on
   * a falling edge that may already have happened.
   */

  icm42688_write_reg(dev, ICM_REG_INT_SOURCE0, 0x00);
  icm42688_modify(dev, ICM_REG_FIFO_CONFIG1, ICM_FIFO_WM_GT_TH, 0x1f);

  dev->last_timestamp_q5 = 0;
  dev->sample_period_q5 = ICM_NOMINAL_PERIOD_Q5;
  dev->sample_count = 0;
  dev->rate_anchor_sample = 0;
  dev->rate_anchor_timestamp = 0;
  dev->period_history_sum_q5 = 0;
  dev->period_history_index = 0;
  for (i = 0; i < ICM_PERIOD_AVG_SAMPLES; i++)
    {
      dev->period_history_q5[i] = ICM_NOMINAL_PERIOD_Q5;
      dev->period_history_sum_q5 += ICM_NOMINAL_PERIOD_Q5;
    }

  dev->drdy_timestamp = 0;
  dev->drdy_sequence = 0;
  dev->consumed_drdy_sequence = 0;
  icm42688_fifo_flush(dev);

  /* Arm PE6 before packet generation or INT1 routing. The first physical
   * threshold crossing therefore always creates a new falling edge.
   */

  ret = stm32_gpiosetevent(GPIO_DRDY_ICM42688, false, true, true,
                          icm42688_isr, dev);
  if (ret < 0)
    {
      snerr("ERROR: ICM-42688 PE6 EXTI setup failed: %d\n", ret);
      return ret;
    }

  dev->streaming = true;
  icm42688_modify(dev, ICM_REG_FIFO_CONFIG1,
                  ICM_FIFO_WM_GT_TH | ICM_FIFO_CONFIG_BITS, 0x08);
  /* Route exactly FIFO_THS to INT1. In particular, do not preserve the
   * register's 0x10 reset default (RESET_DONE_INT1_EN).
   */

  icm42688_write_reg(dev, ICM_REG_INT_SOURCE0,
                     ICM_FIFO_THS_INT1_EN);

  if (!icm42688_check_bits(dev, ICM_REG_FIFO_CONFIG1,
                           ICM_FIFO_WM_GT_TH | ICM_FIFO_CONFIG_BITS, 0x08) ||
      icm42688_read_reg(dev, ICM_REG_INT_SOURCE0) !=
        ICM_FIFO_THS_INT1_EN)
    {
      icm42688_write_reg(dev, ICM_REG_INT_SOURCE0, 0x00);
      icm42688_modify(dev, ICM_REG_FIFO_CONFIG1,
                      ICM_FIFO_WM_GT_TH, 0x1f);
      stm32_gpiosetevent(GPIO_DRDY_ICM42688, false, false, false,
                         NULL, NULL);
      dev->streaming = false;
      snerr("ERROR: ICM-42688 stream enable verification failed\n");
      return -EIO;
    }

  return OK;
}

static void icm42688_stream_stop(FAR struct icm42688_dev_s *dev)
{
  /* Make a concurrent watchdog wake a no-op before changing FIFO state. */

  dev->streaming = false;
  icm42688_write_reg(dev, ICM_REG_INT_SOURCE0, 0x00);
  icm42688_modify(dev, ICM_REG_FIFO_CONFIG1, ICM_FIFO_WM_GT_TH, 0x1f);
  stm32_gpiosetevent(GPIO_DRDY_ICM42688, false, false, false, NULL, NULL);
  icm42688_fifo_flush(dev);
}

/****************************************************************************
 * Private Functions - uorb ops
 ****************************************************************************/

static int icm42688_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable)
{
  FAR struct icm42688_sensor_s *s = (FAR struct icm42688_sensor_s *)lower;
  FAR struct icm42688_dev_s *dev = s->dev;
  int ret = OK;

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
      ret = icm42688_stream_start(dev);
      if (ret < 0)
        {
          if (s == &dev->accel)
            {
              dev->accel_en = false;
            }
          else
            {
              dev->gyro_en = false;
            }
        }
    }
  else if (!dev->accel_en && !dev->gyro_en && dev->streaming)
    {
      icm42688_stream_stop(dev);
    }

  nxmutex_unlock(&dev->lock);
  return ret;
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
  ret = kthread_create("icm42688", FMUV6C_SENSOR_PRIO, 2048,
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
