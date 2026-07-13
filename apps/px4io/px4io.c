/****************************************************************************
 * apps/px4io/px4io.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * PX4IO co-processor client. See px4io.h for what this is and why it exists.
 *
 * Transport note
 * --------------
 * PX4 drives this link with raw USART registers, DMA, and the IDLE-line
 * interrupt, because it pushes actuator updates at 400 Hz and cares about
 * microseconds of jitter. We do not: RC arrives at 50 Hz and a ground robot's
 * steering servo is happy at the same rate. So this uses the ordinary NuttX
 * serial driver, which is far less code and much easier to reason about.
 *
 * That works because a reply is self-describing. We read the 4-byte header
 * first, and its count field tells us exactly how many register bytes follow -
 * so there is never any guessing about where the packet ends, and we never need
 * the IDLE-line trick to find the boundary.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <termios.h>
#include <pthread.h>
#include <sched.h>
#include <syslog.h>

#include "px4io.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A 64-byte reply at 1.5 Mbaud takes ~426us on the wire. 10ms is the same
 * deadline PX4 uses: generous enough that a busy IO is never mistaken for a
 * dead one, short enough that a dead one is noticed promptly.
 */

#define PX4IO_TIMEOUT_MS   10

/* How many times to retry a failed exchange before giving up. A single CRC
 * error is not interesting (the line is fast and unshielded); a persistent one
 * is.
 */

#define PX4IO_RETRIES      3

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int             g_fd = -1;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

/* Number of RC channels this particular IO reports it can decode. Read from
 * PAGE_CONFIG at probe time rather than assumed, so we never read off the end
 * of the RC page.
 */

static uint8_t         g_rc_channels;

/* Poller state. */

static pthread_t       g_poller;
static volatile bool   g_running;
static volatile bool   g_should_stop;
static int             g_rate_hz = 50;

/* The PWM setpoint the poller keeps re-sending. IO failsafes the outputs if the
 * FMU goes quiet for 500ms, so a setpoint has to be refreshed, not just sent.
 */

static uint16_t        g_pwm[PX4IO_SERVO_COUNT];
static bool            g_pwm_valid;

/* Last RC frame the poller saw. */

static struct px4io_rc_s g_rc;
static bool            g_rc_valid;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* CRC-8, polynomial 0x07, initial value 0, MSB first.
 *
 * PX4 ships this as a 256-entry lookup table (crc8_tab in protocol.h). This
 * loop produces byte-for-byte identical results for all 256 inputs - it was
 * checked against PX4's table before being written - and costs 256 fewer bytes
 * of flash.
 */

static uint8_t px4io_crc8(FAR const uint8_t *p, size_t len)
{
  uint8_t crc = 0;
  size_t i;
  int bit;

  for (i = 0; i < len; i++)
    {
      crc ^= p[i];

      for (bit = 0; bit < 8; bit++)
        {
          crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
                             : (uint8_t)(crc << 1);
        }
    }

  return crc;
}

/* Read exactly len bytes, or fail. Once the first byte of a reply lands the
 * rest arrive back-to-back (it is one uninterrupted burst), so applying the
 * timeout per poll() rather than as one deadline across the whole read costs
 * nothing and keeps this simple.
 */

static int px4io_read_exact(FAR uint8_t *buf, size_t len)
{
  struct pollfd pfd;
  size_t got = 0;

  pfd.fd     = g_fd;
  pfd.events = POLLIN;

  while (got < len)
    {
      ssize_t n;
      int ret = poll(&pfd, 1, PX4IO_TIMEOUT_MS);

      if (ret == 0)
        {
          return -ETIMEDOUT;
        }

      if (ret < 0)
        {
          if (errno == EINTR)
            {
              continue;
            }

          return -errno;
        }

      n = read(g_fd, buf + got, len - got);
      if (n < 0)
        {
          if (errno == EINTR)
            {
              continue;
            }

          return -errno;
        }

      if (n == 0)
        {
          return -EIO;
        }

      got += (size_t)n;
    }

  return OK;
}

/* One request/reply round trip. pkt must already have count_code, page, offset
 * and any outgoing regs filled in; on return it holds IO's reply.
 */

static int px4io_exchange(FAR struct px4io_packet_s *pkt)
{
  size_t txlen = PKT_SIZE(*pkt);
  unsigned count;
  uint8_t crc;
  int ret;

  /* CRC covers the whole packet with the crc field itself zeroed. */

  pkt->crc = 0;
  pkt->crc = px4io_crc8((FAR const uint8_t *)pkt, txlen);

  /* Drop anything stale in the RX buffer - after a timeout there may be a
   * late reply sitting there, and consuming it as the *next* reply would
   * desynchronise every subsequent exchange.
   */

  tcflush(g_fd, TCIFLUSH);

  if (write(g_fd, pkt, txlen) != (ssize_t)txlen)
    {
      return -errno;
    }

  /* Header first. Its count field says how much payload follows, so the
   * packet boundary is never in doubt.
   */

  ret = px4io_read_exact((FAR uint8_t *)pkt, 4);
  if (ret < 0)
    {
      return ret;
    }

  count = PKT_COUNT(*pkt);
  if (count > PKT_MAX_REGS)
    {
      return -EIO;
    }

  if (count > 0)
    {
      ret = px4io_read_exact((FAR uint8_t *)pkt->regs, count * 2);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Verify: same rule as the request - zero the crc field, recompute. */

  crc = pkt->crc;
  pkt->crc = 0;

  if (crc != px4io_crc8((FAR const uint8_t *)pkt, PKT_SIZE(*pkt)))
    {
      return -EIO;
    }

  /* CORRUPT means IO received *our* packet badly; ERROR means it understood
   * us but the register operation itself was rejected.
   */

  if (PKT_CODE(*pkt) == PKT_CODE_CORRUPT)
    {
      return -EIO;
    }

  if (PKT_CODE(*pkt) == PKT_CODE_ERROR)
    {
      return -EINVAL;
    }

  return OK;
}

/* Retry wrapper. Only transport failures are retried - a register op that IO
 * actively rejected (-EINVAL) will be rejected again, so it fails straight out.
 */

static int px4io_exchange_retry(FAR struct px4io_packet_s *pkt,
                                FAR const struct px4io_packet_s *req)
{
  int ret = -EIO;
  int i;

  for (i = 0; i < PX4IO_RETRIES; i++)
    {
      *pkt = *req;

      ret = px4io_exchange(pkt);
      if (ret == OK || ret == -EINVAL)
        {
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int px4io_reg_read(uint8_t page, uint8_t offset,
                   FAR uint16_t *values, unsigned count)
{
  struct px4io_packet_s req;
  struct px4io_packet_s pkt;
  int ret;

  if (count == 0 || count > PKT_MAX_REGS || values == NULL)
    {
      return -EINVAL;
    }

  if (g_fd < 0)
    {
      return -ENODEV;
    }

  /* For a read, the count field says how many registers we *want*; no payload
   * travels in the request itself.
   */

  memset(&req, 0, sizeof(req));
  req.count_code = (uint8_t)(count | PKT_CODE_READ);
  req.page       = page;
  req.offset     = offset;

  pthread_mutex_lock(&g_lock);
  ret = px4io_exchange_retry(&pkt, &req);
  pthread_mutex_unlock(&g_lock);

  if (ret < 0)
    {
      return ret;
    }

  /* IO is allowed to return fewer registers than asked for. Returning stale
   * stack contents for the rest would be worse than failing.
   */

  if (PKT_COUNT(pkt) != count)
    {
      return -EIO;
    }

  memcpy(values, pkt.regs, count * sizeof(uint16_t));
  return OK;
}

int px4io_reg_write(uint8_t page, uint8_t offset,
                    FAR const uint16_t *values, unsigned count)
{
  struct px4io_packet_s req;
  struct px4io_packet_s pkt;
  int ret;

  if (count == 0 || count > PKT_MAX_REGS || values == NULL)
    {
      return -EINVAL;
    }

  if (g_fd < 0)
    {
      return -ENODEV;
    }

  memset(&req, 0, sizeof(req));
  req.count_code = (uint8_t)(count | PKT_CODE_WRITE);
  req.page       = page;
  req.offset     = offset;
  memcpy(req.regs, values, count * sizeof(uint16_t));

  pthread_mutex_lock(&g_lock);
  ret = px4io_exchange_retry(&pkt, &req);
  pthread_mutex_unlock(&g_lock);

  return ret;
}

int px4io_reg_get(uint8_t page, uint8_t offset, FAR uint16_t *value)
{
  return px4io_reg_read(page, offset, value, 1);
}

int px4io_reg_set(uint8_t page, uint8_t offset, uint16_t value)
{
  return px4io_reg_write(page, offset, &value, 1);
}

int px4io_reg_modify(uint8_t page, uint8_t offset,
                     uint16_t clearbits, uint16_t setbits)
{
  uint16_t value;
  int ret;

  ret = px4io_reg_get(page, offset, &value);
  if (ret < 0)
    {
      return ret;
    }

  value &= ~clearbits;
  value |= setbits;

  return px4io_reg_set(page, offset, value);
}

int px4io_probe(void)
{
  uint16_t config[8];
  struct termios tio;
  int ret;

  if (g_fd >= 0)
    {
      return OK;
    }

  g_fd = open(PX4IO_DEVPATH, O_RDWR | O_NOCTTY);
  if (g_fd < 0)
    {
      return -errno;
    }

  /* Raw, 8N1, no flow control, 1.5 Mbaud. The rate is dictated by the firmware
   * already running on the IO chip; it is not negotiable.
   *
   * cfmakeraw() matters: in canonical mode the driver would try to interpret
   * the binary packet as lines of text.
   */

  if (tcgetattr(g_fd, &tio) < 0)
    {
      ret = -errno;
      goto err;
    }

  cfmakeraw(&tio);
  tio.c_cflag &= ~(CSTOPB | PARENB | CSIZE);
  tio.c_cflag |= CS8;
  cfsetspeed(&tio, B1500000);

  if (tcsetattr(g_fd, TCSANOW, &tio) < 0)
    {
      ret = -errno;
      goto err;
    }

  /* Ask IO who it is. This is the real test: on a board with no IO chip fitted
   * (PX4's board_config.h lists NO-PX4IO hardware revisions) nothing answers
   * and we time out here, which is exactly the outcome we want - better than
   * silently half-working.
   */

  ret = px4io_reg_read(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION,
                       config, 8);
  if (ret < 0)
    {
      goto err;
    }

  if (config[PX4IO_P_CONFIG_PROTOCOL_VERSION] != PX4IO_PROTOCOL_VERSION)
    {
      syslog(LOG_ERR, "px4io: protocol version %u, expected %u\n",
             config[PX4IO_P_CONFIG_PROTOCOL_VERSION], PX4IO_PROTOCOL_VERSION);
      ret = -EPROTO;
      goto err;
    }

  g_rc_channels = (uint8_t)config[PX4IO_P_CONFIG_RC_INPUT_COUNT];
  if (g_rc_channels > PX4IO_RC_CHANNELS)
    {
      g_rc_channels = PX4IO_RC_CHANNELS;
    }

  return OK;

err:
  close(g_fd);
  g_fd = -1;
  return ret;
}

void px4io_close(void)
{
  px4io_stop();

  if (g_fd >= 0)
    {
      close(g_fd);
      g_fd = -1;
    }
}

int px4io_get_status(FAR struct px4io_status_s *status)
{
  uint16_t config[8];
  uint16_t stat[8];
  int ret;

  if (status == NULL)
    {
      return -EINVAL;
    }

  ret = px4io_reg_read(PX4IO_PAGE_CONFIG, 0, config, 8);
  if (ret < 0)
    {
      return ret;
    }

  ret = px4io_reg_read(PX4IO_PAGE_STATUS, 0, stat, 8);
  if (ret < 0)
    {
      return ret;
    }

  status->protocol_version = config[PX4IO_P_CONFIG_PROTOCOL_VERSION];
  status->hardware_version = config[PX4IO_P_CONFIG_HARDWARE_VERSION];
  status->max_transfer     = config[PX4IO_P_CONFIG_MAX_TRANSFER];
  status->actuator_count   = config[PX4IO_P_CONFIG_ACTUATOR_COUNT];
  status->rc_input_count   = config[PX4IO_P_CONFIG_RC_INPUT_COUNT];

  status->freemem          = stat[PX4IO_P_STATUS_FREEMEM];
  status->cpuload          = stat[PX4IO_P_STATUS_CPULOAD];
  status->flags            = stat[PX4IO_P_STATUS_FLAGS];
  status->alarms           = stat[PX4IO_P_STATUS_ALARMS];
  status->vservo_mv        = stat[PX4IO_P_STATUS_VSERVO];

  return OK;
}

int px4io_get_rc(FAR struct px4io_rc_s *rc)
{
  uint16_t regs[PX4IO_P_RAW_RC_BASE + PX4IO_RC_CHANNELS];
  unsigned count = PX4IO_P_RAW_RC_BASE + g_rc_channels;
  uint16_t flags;
  unsigned i;
  int ret;

  if (rc == NULL)
    {
      return -EINVAL;
    }

  if (count > PKT_MAX_REGS)
    {
      count = PKT_MAX_REGS;
    }

  /* One read pulls the header and every channel together, so the channels can
   * never be torn across two frames.
   */

  ret = px4io_reg_read(PX4IO_PAGE_RAW_RC_INPUT, 0, regs, count);
  if (ret < 0)
    {
      return ret;
    }

  memset(rc, 0, sizeof(*rc));

  flags           = regs[PX4IO_P_RAW_RC_FLAGS];
  rc->count       = (uint8_t)regs[PX4IO_P_RAW_RC_COUNT];
  rc->rssi        = (uint8_t)regs[PX4IO_P_RAW_RC_NRSSI];
  rc->frames      = regs[PX4IO_P_RAW_FRAME_COUNT];
  rc->lost_frames = regs[PX4IO_P_RAW_LOST_FRAME_COUNT];
  rc->ok          = (flags & PX4IO_P_RAW_RC_FLAGS_RC_OK) != 0;
  rc->failsafe    = (flags & PX4IO_P_RAW_RC_FLAGS_FAILSAFE) != 0;

  if (rc->count > g_rc_channels)
    {
      rc->count = g_rc_channels;
    }

  for (i = 0; i < rc->count && (PX4IO_P_RAW_RC_BASE + i) < count; i++)
    {
      rc->channel[i] = regs[PX4IO_P_RAW_RC_BASE + i];
    }

  return OK;
}

int px4io_set_pwm(FAR const uint16_t *values, unsigned count)
{
  int ret;

  if (values == NULL || count == 0 || count > PX4IO_SERVO_COUNT)
    {
      return -EINVAL;
    }

  ret = px4io_reg_write(PX4IO_PAGE_DIRECT_PWM, 0, values, count);
  if (ret < 0)
    {
      return ret;
    }

  /* Remember it: IO drops the outputs to failsafe if we go quiet for 500ms, so
   * the poller has to keep re-sending this.
   */

  pthread_mutex_lock(&g_lock);
  memcpy(g_pwm, values, count * sizeof(uint16_t));
  g_pwm_valid = true;
  pthread_mutex_unlock(&g_lock);

  return OK;
}

int px4io_arm(bool armed)
{
  if (armed)
    {
      /* IO_ARM_OK lets IO arm at all; FMU_ARMED says we want it now.
       *
       * Also tell IO that safety is off. On current IO firmware that flag only
       * drives the safety LED (the outputs are gated by INIT_OK && (FMU_ARMED
       * || RAW_PWM), not by the button) - but leaving it unset would mean a
       * blinking LED saying "safe" while the servos move, which is a lie worth
       * avoiding.
       */

      int ret = px4io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SAFETY_OFF, 1);
      if (ret < 0)
        {
          return ret;
        }

      return px4io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING,
                              PX4IO_P_SETUP_ARMING_LOCKDOWN,
                              PX4IO_P_SETUP_ARMING_IO_ARM_OK |
                              PX4IO_P_SETUP_ARMING_FMU_ARMED);
    }

  px4io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SAFETY_OFF, 0);

  return px4io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING,
                          PX4IO_P_SETUP_ARMING_IO_ARM_OK |
                          PX4IO_P_SETUP_ARMING_FMU_ARMED,
                          0);
}

int px4io_set_pwm_rate(uint16_t rate_hz)
{
  int ret;

  /* PWM_RATES is a per-channel bitmask choosing between the "default" and
   * "alt" rate. Clearing it puts all 8 channels on the default rate, and then
   * one register sets that rate - which is all a ground robot needs.
   */

  ret = px4io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES, 0);
  if (ret < 0)
    {
      return ret;
    }

  return px4io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_DEFAULTRATE,
                       rate_hz);
}

int px4io_set_failsafe_pwm(FAR const uint16_t *values, unsigned count)
{
  if (values == NULL || count == 0 || count > PX4IO_SERVO_COUNT)
    {
      return -EINVAL;
    }

  return px4io_reg_write(PX4IO_PAGE_FAILSAFE_PWM, 0, values, count);
}

int px4io_set_disarmed_pwm(FAR const uint16_t *values, unsigned count)
{
  if (values == NULL || count == 0 || count > PX4IO_SERVO_COUNT)
    {
      return -EINVAL;
    }

  return px4io_reg_write(PX4IO_PAGE_DISARMED_PWM, 0, values, count);
}

int px4io_rc_latest(FAR struct px4io_rc_s *rc)
{
  int ret = -EAGAIN;

  if (rc == NULL)
    {
      return -EINVAL;
    }

  pthread_mutex_lock(&g_lock);

  if (g_rc_valid)
    {
      *rc = g_rc;
      ret = OK;
    }

  pthread_mutex_unlock(&g_lock);
  return ret;
}

/* The poller. Two jobs, and the second one is not optional:
 *
 *  1. Refresh the RC snapshot.
 *  2. Keep talking to IO. IO failsafes the servo rails if the FMU is silent for
 *     500ms (PX4IO_FMU_DROP_LIMIT_US), so even when nothing is steering, this
 *     loop is what keeps the outputs alive.
 */

static FAR void *px4io_poller(FAR void *arg)
{
  useconds_t period_us = (useconds_t)(1000000 / g_rate_hz);

  UNUSED(arg);

  while (!g_should_stop)
    {
      struct px4io_rc_s rc;

      if (px4io_get_rc(&rc) == OK)
        {
          pthread_mutex_lock(&g_lock);
          g_rc = rc;
          g_rc_valid = true;
          pthread_mutex_unlock(&g_lock);
        }

      /* Re-send the setpoint. Writing DIRECT_PWM is also what raises IO's
       * RAW_PWM status flag, which is half of what keeps the outputs armed.
       */

      pthread_mutex_lock(&g_lock);
      bool have_pwm = g_pwm_valid;
      uint16_t pwm[PX4IO_SERVO_COUNT];
      memcpy(pwm, g_pwm, sizeof(pwm));
      pthread_mutex_unlock(&g_lock);

      if (have_pwm)
        {
          px4io_reg_write(PX4IO_PAGE_DIRECT_PWM, 0, pwm, PX4IO_SERVO_COUNT);
        }

      usleep(period_us);
    }

  g_running = false;
  return NULL;
}

int px4io_start(int rate_hz)
{
  pthread_attr_t attr;
  struct sched_param param;
  int ret;

  if (g_running)
    {
      return OK;
    }

  if (g_fd < 0)
    {
      return -ENODEV;
    }

  /* Must stay comfortably above IO's 500ms failsafe timeout. */

  if (rate_hz < 5 || rate_hz > 400)
    {
      return -EINVAL;
    }

  g_rate_hz     = rate_hz;
  g_should_stop = false;

  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, 2048);
  param.sched_priority = SCHED_PRIORITY_DEFAULT + 10;
  pthread_attr_setschedparam(&attr, &param);

  ret = pthread_create(&g_poller, &attr, px4io_poller, NULL);
  pthread_attr_destroy(&attr);

  if (ret != 0)
    {
      return -ret;
    }

  pthread_setname_np(g_poller, "px4io");
  g_running = true;
  return OK;
}

void px4io_stop(void)
{
  if (!g_running)
    {
      return;
    }

  g_should_stop = true;
  pthread_join(g_poller, NULL);
  g_running = false;
}

bool px4io_is_running(void)
{
  return g_running;
}
