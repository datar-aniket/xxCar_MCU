/****************************************************************************
 * apps/px4io/px4io.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * PX4IO co-processor client. See px4io.h for what this is and why it exists.
 *
 * Transport note
 * --------------
 * PX4 drives this link by hand: raw USART registers, its own DMA setup, and the
 * IDLE-line interrupt to find the end of a reply. We get the same three things
 * from the ordinary NuttX serial driver instead, with none of the code -
 * CONFIG_USART6_RXDMA turns on exactly that machinery inside stm32_serial.c
 * (circular RX DMA, drained on the IDLE-line interrupt), and the H7's 8-deep
 * hardware RX FIFO is always on. So a ~52-byte reply arriving as one 350us burst
 * costs about one interrupt, not fifty, and a 400 Hz servo loop stays cheap.
 *
 * We can also be simpler than PX4 in one place: a reply is self-describing. The
 * 4-byte header's count field says exactly how many register bytes follow, so we
 * read the header, then the payload, and the packet boundary is never in doubt.
 * PX4 needs the IDLE edge to delimit packets; we only need it to keep the
 * interrupt rate down.
 *
 * Concurrency note
 * ----------------
 * The file descriptor lives in a caller-owned handle, never in a global. In a
 * flat build every task shares .bss but file descriptors are per task group, so
 * a cached global fd goes stale (EBADF) the instant the task that opened it
 * exits. The daemon opens its own; each NSH invocation opens its own.
 *
 * Plain data - the RC snapshot, the PWM setpoint - IS legitimately shared
 * through .bss, guarded by g_lock. g_lock also serialises the wire itself, so
 * an NSH command and the daemon can both hold the port open without their
 * request/reply pairs interleaving.
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
#include <sys/time.h>

#include "px4io.h"
#include "../rc_in/rc_in.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A 64-byte reply at 1.5 Mbaud takes ~426us on the wire. 10ms is the same
 * deadline PX4 uses: generous enough that a busy IO is never mistaken for a
 * dead one, short enough that a dead one is noticed promptly.
 */

#define PX4IO_TIMEOUT_MS   10

/* Retry transport failures. A single CRC error on a fast unshielded line is not
 * interesting; a persistent one is.
 */

#define PX4IO_RETRIES      3

#define PX4IO_DAEMON_STACK 2048
#define PX4IO_DAEMON_PRIO  (SCHED_PRIORITY_DEFAULT + 10)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serialises the wire, and protects the shared snapshots below. */

static pthread_mutex_t   g_lock = PTHREAD_MUTEX_INITIALIZER;

/* Daemon state. */

static volatile bool     g_running;
static volatile bool     g_should_stop;
static int               g_rate_hz = 50;

/* The PWM setpoint the daemon keeps re-sending. IO failsafes the outputs if the
 * FMU goes quiet for 500ms, so a setpoint has to be refreshed, not just sent.
 */

static uint16_t          g_pwm[PX4IO_SERVO_COUNT];
static bool              g_pwm_valid;

/* Last RC frame the daemon saw. */

static struct px4io_rc_s g_rc;
static bool              g_rc_valid;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* CRC-8, polynomial 0x07, initial value 0, MSB first.
 *
 * PX4 ships this as a 256-entry lookup table (crc8_tab in protocol.h). This
 * loop reproduces that table byte-for-byte across all 256 inputs - it was
 * checked against PX4's before being written - and costs 256 fewer bytes of
 * flash.
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

static int px4io_read_exact(int fd, FAR uint8_t *buf, size_t len)
{
  struct pollfd pfd;
  size_t got = 0;

  pfd.fd     = fd;
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

      n = read(fd, buf + got, len - got);
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

/* Throw away anything sitting in the RX path.
 *
 * tcflush(TCIFLUSH) alone is not enough once RX DMA is on: it only resets the
 * upper-half ring (dev->recv.tail = dev->recv.head, see drivers/serial/
 * serial.c). Bytes that the DMA controller has already written to its own
 * buffer but that have not yet been handed up - they are handed up on the
 * IDLE-line interrupt - survive it, and would then be consumed as the *next*
 * reply. So drain the readable side as well.
 */

static void px4io_drain(int fd)
{
  struct pollfd pfd;
  uint8_t junk[64];

  tcflush(fd, TCIFLUSH);

  pfd.fd     = fd;
  pfd.events = POLLIN;

  while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN) != 0)
    {
      if (read(fd, junk, sizeof(junk)) <= 0)
        {
          break;
        }
    }
}

/* One request/reply round trip. Caller holds g_lock. */

static int px4io_exchange(int fd, FAR struct px4io_packet_s *pkt)
{
  size_t txlen = PKT_SIZE(*pkt);
  uint8_t page = pkt->page;
  uint8_t offset = pkt->offset;
  unsigned count;
  uint8_t crc;
  int ret;

  /* The CRC covers the whole packet with the crc field itself zeroed. */

  pkt->crc = 0;
  pkt->crc = px4io_crc8((FAR const uint8_t *)pkt, txlen);

  /* After a timeout a late reply may still be in flight or buffered, and
   * consuming it as the next reply would desynchronise every exchange after it.
   */

  px4io_drain(fd);

  if (write(fd, pkt, txlen) != (ssize_t)txlen)
    {
      return -errno;
    }

  /* Header first: its count field says how much payload follows. */

  ret = px4io_read_exact(fd, (FAR uint8_t *)pkt, 4);
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
      ret = px4io_read_exact(fd, (FAR uint8_t *)pkt->regs, count * 2);
      if (ret < 0)
        {
          return ret;
        }
    }

  crc = pkt->crc;
  pkt->crc = 0;

  if (crc != px4io_crc8((FAR const uint8_t *)pkt, PKT_SIZE(*pkt)))
    {
      return -EIO;
    }

  /* Is this a reply to the request we just sent, or a leftover?
   *
   * IO builds its reply by overwriting count_code and regs in the packet it
   * received, in place (px4iofirmware/serial.cpp) - so page and offset come
   * back exactly as we sent them. That makes the reply self-identifying, and it
   * is worth checking: a stale reply from an earlier, timed-out request carries
   * a perfectly valid CRC, so the CRC alone cannot tell the two apart. No amount
   * of flushing can either, because such a reply may still be in flight at the
   * moment we flush. Matching page/offset is what actually resynchronises us.
   */

  if (pkt->page != page || pkt->offset != offset)
    {
      return -EIO;
    }

  /* CORRUPT means IO received *our* packet badly; ERROR means it understood us
   * but rejected the register operation.
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

/* Retry transport failures only: a register op IO actively rejected (-EINVAL)
 * will be rejected again.
 */

static int px4io_exchange_locked(int fd, FAR struct px4io_packet_s *pkt,
                                 FAR const struct px4io_packet_s *req)
{
  int ret = -EIO;
  int i;

  pthread_mutex_lock(&g_lock);

  for (i = 0; i < PX4IO_RETRIES; i++)
    {
      *pkt = *req;

      ret = px4io_exchange(fd, pkt);
      if (ret == OK || ret == -EINVAL)
        {
          break;
        }
    }

  pthread_mutex_unlock(&g_lock);
  return ret;
}

static uint64_t px4io_now_us(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int px4io_reg_read(FAR struct px4io_s *io, uint8_t page, uint8_t offset,
                   FAR uint16_t *values, unsigned count)
{
  struct px4io_packet_s req;
  struct px4io_packet_s pkt;
  int ret;

  if (io == NULL || io->fd < 0 || values == NULL ||
      count == 0 || count > PKT_MAX_REGS)
    {
      return -EINVAL;
    }

  /* On a read, count says how many registers we *want*; the request carries no
   * payload of its own.
   */

  memset(&req, 0, sizeof(req));
  req.count_code = (uint8_t)(count | PKT_CODE_READ);
  req.page       = page;
  req.offset     = offset;

  ret = px4io_exchange_locked(io->fd, &pkt, &req);
  if (ret < 0)
    {
      return ret;
    }

  /* IO may legally return fewer registers than asked for. Handing back stale
   * stack contents for the remainder would be worse than failing.
   */

  if (PKT_COUNT(pkt) != count)
    {
      return -EIO;
    }

  memcpy(values, pkt.regs, count * sizeof(uint16_t));
  return OK;
}

int px4io_reg_write(FAR struct px4io_s *io, uint8_t page, uint8_t offset,
                    FAR const uint16_t *values, unsigned count)
{
  struct px4io_packet_s req;
  struct px4io_packet_s pkt;

  if (io == NULL || io->fd < 0 || values == NULL ||
      count == 0 || count > PKT_MAX_REGS)
    {
      return -EINVAL;
    }

  memset(&req, 0, sizeof(req));
  req.count_code = (uint8_t)(count | PKT_CODE_WRITE);
  req.page       = page;
  req.offset     = offset;
  memcpy(req.regs, values, count * sizeof(uint16_t));

  return px4io_exchange_locked(io->fd, &pkt, &req);
}

int px4io_reg_get(FAR struct px4io_s *io, uint8_t page, uint8_t offset,
                  FAR uint16_t *value)
{
  return px4io_reg_read(io, page, offset, value, 1);
}

int px4io_reg_set(FAR struct px4io_s *io, uint8_t page, uint8_t offset,
                  uint16_t value)
{
  return px4io_reg_write(io, page, offset, &value, 1);
}

int px4io_reg_modify(FAR struct px4io_s *io, uint8_t page, uint8_t offset,
                     uint16_t clearbits, uint16_t setbits)
{
  uint16_t value;
  int ret;

  ret = px4io_reg_get(io, page, offset, &value);
  if (ret < 0)
    {
      return ret;
    }

  value &= ~clearbits;
  value |= setbits;

  return px4io_reg_set(io, page, offset, value);
}

int px4io_open(FAR struct px4io_s *io)
{
  uint16_t config[8];
  struct termios tio;
  int ret;

  if (io == NULL)
    {
      return -EINVAL;
    }

  io->fd = open(PX4IO_DEVPATH, O_RDWR | O_NOCTTY);
  if (io->fd < 0)
    {
      return -errno;
    }

  /* Raw, 8N1, no flow control, 1.5 Mbaud. The rate is dictated by the firmware
   * already running on the IO chip; it is not negotiable.
   *
   * cfmakeraw() matters: in canonical mode the driver would try to read the
   * binary packet as lines of text.
   */

  if (tcgetattr(io->fd, &tio) < 0)
    {
      ret = -errno;
      goto err;
    }

  cfmakeraw(&tio);
  tio.c_cflag &= ~(CSTOPB | PARENB | CSIZE);
  tio.c_cflag |= CS8;
  cfsetspeed(&tio, B1500000);

  if (tcsetattr(io->fd, TCSANOW, &tio) < 0)
    {
      ret = -errno;
      goto err;
    }

  /* Ask IO who it is. This is the real test: on a board with no IO chip fitted
   * nothing answers and we time out here, which is exactly what we want -
   * better than silently half-working.
   */

  ret = px4io_reg_read(io, PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION,
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

  io->rc_channels = (uint8_t)config[PX4IO_P_CONFIG_RC_INPUT_COUNT];
  if (io->rc_channels > PX4IO_RC_CHANNELS)
    {
      io->rc_channels = PX4IO_RC_CHANNELS;
    }

  return OK;

err:
  close(io->fd);
  io->fd = -1;
  return ret;
}

void px4io_close(FAR struct px4io_s *io)
{
  if (io != NULL && io->fd >= 0)
    {
      close(io->fd);
      io->fd = -1;
    }
}

int px4io_init(FAR struct px4io_s *io)
{
  uint16_t disarmed[PX4IO_SERVO_COUNT];
  unsigned i;

  /* Writing PAGE_DISARMED_PWM is what makes IO raise INIT_OK, and INIT_OK is
   * half the gate on the servo rails (INIT_OK && (FMU_ARMED || RAW_PWM)).
   * Skip this and the outputs can never come on, whatever else we write.
   *
   * Zero means "emit no pulses at all" on the disarmed page (it is explicitly
   * special-cased in IO's registers.c), so a disarmed rail is genuinely dead
   * rather than parked at some pulse width. For a ground robot that is the
   * right default: no signal beats a wrong signal into an ESC.
   */

  for (i = 0; i < PX4IO_SERVO_COUNT; i++)
    {
      disarmed[i] = 0;
    }

  return px4io_reg_write(io, PX4IO_PAGE_DISARMED_PWM, 0,
                         disarmed, PX4IO_SERVO_COUNT);
}

int px4io_get_status(FAR struct px4io_s *io, FAR struct px4io_status_s *status)
{
  uint16_t config[8];
  uint16_t stat[8];
  int ret;

  if (status == NULL)
    {
      return -EINVAL;
    }

  ret = px4io_reg_read(io, PX4IO_PAGE_CONFIG, 0, config, 8);
  if (ret < 0)
    {
      return ret;
    }

  ret = px4io_reg_read(io, PX4IO_PAGE_STATUS, 0, stat, 8);
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

int px4io_get_rc(FAR struct px4io_s *io, FAR struct px4io_rc_s *rc)
{
  uint16_t regs[PX4IO_P_RAW_RC_BASE + PX4IO_RC_CHANNELS];
  unsigned count;
  uint16_t flags;
  unsigned i;
  int ret;

  if (io == NULL || rc == NULL)
    {
      return -EINVAL;
    }

  count = PX4IO_P_RAW_RC_BASE + io->rc_channels;
  if (count > PKT_MAX_REGS)
    {
      count = PKT_MAX_REGS;
    }

  /* One read pulls the header and every channel together, so the channels can
   * never be torn across two RC frames.
   */

  ret = px4io_reg_read(io, PX4IO_PAGE_RAW_RC_INPUT, 0, regs, count);
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

  if (rc->count > io->rc_channels)
    {
      rc->count = io->rc_channels;
    }

  for (i = 0; i < rc->count && (PX4IO_P_RAW_RC_BASE + i) < count; i++)
    {
      rc->channel[i] = regs[PX4IO_P_RAW_RC_BASE + i];
    }

  return OK;
}

int px4io_set_pwm(FAR struct px4io_s *io, FAR const uint16_t *values,
                  unsigned count)
{
  int ret;

  if (values == NULL || count == 0 || count > PX4IO_SERVO_COUNT)
    {
      return -EINVAL;
    }

  ret = px4io_reg_write(io, PX4IO_PAGE_DIRECT_PWM, 0, values, count);
  if (ret < 0)
    {
      return ret;
    }

  /* Hand it to the daemon as well: IO drops the outputs to failsafe if we go
   * quiet for 500ms, so this has to keep being re-sent.
   */

  return px4io_set_setpoint(values, count);
}

int px4io_set_setpoint(FAR const uint16_t *values, unsigned count)
{
  if (values == NULL || count == 0 || count > PX4IO_SERVO_COUNT)
    {
      return -EINVAL;
    }

  pthread_mutex_lock(&g_lock);
  memset(g_pwm, 0, sizeof(g_pwm));
  memcpy(g_pwm, values, count * sizeof(uint16_t));
  g_pwm_valid = true;
  pthread_mutex_unlock(&g_lock);

  return OK;
}

int px4io_arm(FAR struct px4io_s *io, bool armed)
{
  if (armed)
    {
      /* IO_ARM_OK lets IO arm at all; FMU_ARMED says we want it armed now.
       *
       * SAFETY_OFF is also set. On current IO firmware that flag only drives
       * the safety LED - the outputs are gated by INIT_OK && (FMU_ARMED ||
       * RAW_PWM), not by the button - but leaving it clear would mean an LED
       * blinking "safe" above moving servos, which is a lie worth avoiding.
       */

      int ret = px4io_reg_set(io, PX4IO_PAGE_SETUP,
                              PX4IO_P_SETUP_SAFETY_OFF, 1);
      if (ret < 0)
        {
          return ret;
        }

      return px4io_reg_modify(io, PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING,
                              PX4IO_P_SETUP_ARMING_LOCKDOWN,
                              PX4IO_P_SETUP_ARMING_IO_ARM_OK |
                              PX4IO_P_SETUP_ARMING_FMU_ARMED);
    }

  px4io_reg_set(io, PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SAFETY_OFF, 0);

  return px4io_reg_modify(io, PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING,
                          PX4IO_P_SETUP_ARMING_IO_ARM_OK |
                          PX4IO_P_SETUP_ARMING_FMU_ARMED,
                          0);
}

int px4io_set_pwm_rate(FAR struct px4io_s *io, uint16_t rate_hz)
{
  uint16_t readback;
  int ret;

  /* IO clamps this to [25, 400] Hz *silently* (registers.c), so a typo would
   * otherwise be accepted and quietly do something else. Reject it here instead.
   */

  if (rate_hz < PX4IO_PWM_RATE_MIN || rate_hz > PX4IO_PWM_RATE_MAX)
    {
      return -ERANGE;
    }

  /* PWM_RATES is a per-channel bitmask choosing between the "default" and the
   * "alt" rate. Clearing it puts all 8 channels on the default rate, and then
   * one register sets what that rate is - all a ground robot needs.
   */

  ret = px4io_reg_set(io, PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES, 0);
  if (ret < 0)
    {
      return ret;
    }

  ret = px4io_reg_set(io, PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_DEFAULTRATE,
                      rate_hz);
  if (ret < 0)
    {
      return ret;
    }

  /* Read it back. The rate the timers are actually running at is the one that
   * matters, and IO is the only one who knows it.
   */

  ret = px4io_reg_get(io, PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_DEFAULTRATE,
                      &readback);
  if (ret < 0)
    {
      return ret;
    }

  return (readback == rate_hz) ? OK : -EIO;
}

int px4io_set_failsafe_pwm(FAR struct px4io_s *io, FAR const uint16_t *values,
                           unsigned count)
{
  if (values == NULL || count == 0 || count > PX4IO_SERVO_COUNT)
    {
      return -EINVAL;
    }

  return px4io_reg_write(io, PX4IO_PAGE_FAILSAFE_PWM, 0, values, count);
}

int px4io_set_disarmed_pwm(FAR struct px4io_s *io, FAR const uint16_t *values,
                           unsigned count)
{
  if (values == NULL || count == 0 || count > PX4IO_SERVO_COUNT)
    {
      return -EINVAL;
    }

  return px4io_reg_write(io, PX4IO_PAGE_DISARMED_PWM, 0, values, count);
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

/****************************************************************************
 * The daemon
 *
 * A task rather than a pthread: it must outlive the NSH command that starts it,
 * and it needs its own file-descriptor table (a pthread would share the command
 * task's, and die with it).
 *
 * Three jobs, and the third is not optional:
 *   1. refresh the RC snapshot,
 *   2. publish RC to uORB as rc_in,
 *   3. keep talking to IO at all. IO failsafes the servo rails if the FMU is
 *      silent for 500ms (PX4IO_FMU_DROP_LIMIT_US), so even when nothing is
 *      steering, this loop is what keeps the outputs alive.
 ****************************************************************************/

static int px4io_daemon(int argc, FAR char *argv[])
{
  struct px4io_s io;
  useconds_t period_us;
  int period_ms;
  int rc_divisor;
  int cycle = 0;
  int rcfd;
  int ret;

  UNUSED(argc);
  UNUSED(argv);

  /* Our own connection: file descriptors do not cross task groups. */

  ret = px4io_open(&io);
  if (ret < 0)
    {
      syslog(LOG_ERR, "px4io: daemon cannot open IO: %d\n", ret);
      g_running = false;
      return EXIT_FAILURE;
    }

  /* Raise INIT_OK, without which the outputs can never arm. */

  px4io_init(&io);

  rcfd = rc_in_advertise();
  if (rcfd < 0)
    {
      syslog(LOG_ERR, "px4io: cannot advertise rc_in: %d\n", rcfd);
    }

  /* Snap the period to whole system ticks, and publish the rate we can actually
   * hold rather than the one that was asked for.
   *
   * The tick is 1 kHz (CONFIG_USEC_PER_TICK=1000), so usleep() can only express
   * whole milliseconds. Naively sleeping 1000000/rate microseconds goes wrong in
   * a way that is easy to miss: for 333 Hz that is 3003us, which rounds UP to 4
   * ticks and quietly runs the loop at 250 Hz. 400 Hz (2.5 ms) is not expressible
   * at all. So round to the nearest tick and tell the truth about the result.
   *
   * This limits how often we hand IO a NEW setpoint. It does NOT limit the PWM
   * frame rate the servo sees - IO keeps pulsing the rails at
   * P_SETUP_PWM_DEFAULTRATE (up to 400 Hz) from the last value it was given,
   * whatever we do here.
   */

  period_ms = (1000 + g_rate_hz / 2) / g_rate_hz;
  if (period_ms < 1)
    {
      period_ms = 1;
    }

  g_rate_hz = 1000 / period_ms;
  period_us = (useconds_t)(period_ms * 1000);
  g_running = true;

  /* Actuators are refreshed every cycle, but RC is not.
   *
   * A receiver emits frames at 50-150 Hz; asking IO for RC at 400 Hz would just
   * re-read the same frame several times over, and an RC read is by far the
   * bigger of the two transfers (a ~52-byte reply against 4 bytes for a PWM
   * write). So RC is polled at roughly PX4IO_RC_POLL_HZ regardless of how fast
   * the servo loop runs, which keeps a 400 Hz output rate cheap on the wire.
   */

  rc_divisor = g_rate_hz / PX4IO_RC_POLL_HZ;
  if (rc_divisor < 1)
    {
      rc_divisor = 1;
    }

  while (!g_should_stop)
    {
      struct px4io_rc_s rc;
      uint16_t pwm[PX4IO_SERVO_COUNT];
      bool have_pwm;

      /* Re-send the setpoint first: it is what the servos are waiting on, and
       * writing DIRECT_PWM also raises IO's RAW_PWM status flag, which is the
       * other half of what keeps the outputs armed.
       */

      pthread_mutex_lock(&g_lock);
      have_pwm = g_pwm_valid;
      memcpy(pwm, g_pwm, sizeof(pwm));
      pthread_mutex_unlock(&g_lock);

      if (have_pwm)
        {
          px4io_reg_write(&io, PX4IO_PAGE_DIRECT_PWM, 0,
                          pwm, PX4IO_SERVO_COUNT);
        }

      if (++cycle >= rc_divisor)
        {
          cycle = 0;

          if (px4io_get_rc(&io, &rc) == OK)
            {
              pthread_mutex_lock(&g_lock);
              g_rc = rc;
              g_rc_valid = true;
              pthread_mutex_unlock(&g_lock);

              if (rcfd >= 0)
                {
                  struct rc_in_s msg;
                  unsigned i;

                  memset(&msg, 0, sizeof(msg));
                  msg.timestamp   = px4io_now_us();
                  msg.count       = rc.count;
                  msg.rssi        = rc.rssi;
                  msg.ok          = rc.ok;
                  msg.failsafe    = rc.failsafe;
                  msg.frames      = rc.frames;
                  msg.lost_frames = rc.lost_frames;
                  msg.source      = RC_IN_SRC_PX4IO;

                  for (i = 0; i < RC_IN_MAX_CHANNELS; i++)
                    {
                      msg.channel[i] = rc.channel[i];
                    }

                  rc_in_publish(rcfd, &msg);
                }
            }
        }

      usleep(period_us);
    }

  px4io_close(&io);
  g_running = false;
  return EXIT_SUCCESS;
}

int px4io_start(int rate_hz)
{
  int pid;
  int i;

  if (g_running)
    {
      return OK;
    }

  /* Must stay comfortably above IO's 500ms failsafe timeout at the bottom, and
   * within the 1 kHz tick at the top. The daemon snaps the period to whole ticks
   * and republishes the rate it can actually hold, so a request in between is
   * always honoured, just possibly rounded.
   */

  if (rate_hz < 5 || rate_hz > 1000)
    {
      return -EINVAL;
    }

  g_rate_hz     = rate_hz;
  g_should_stop = false;

  pid = task_create("px4io", PX4IO_DAEMON_PRIO, PX4IO_DAEMON_STACK,
                    px4io_daemon, NULL);
  if (pid < 0)
    {
      return -errno;
    }

  /* Wait for it to come up (or fail), so the caller gets a real answer rather
   * than a hopeful one.
   */

  for (i = 0; i < 100 && !g_running; i++)
    {
      usleep(10000);
    }

  return g_running ? OK : -EIO;
}

void px4io_stop(void)
{
  int i;

  if (!g_running)
    {
      return;
    }

  g_should_stop = true;

  for (i = 0; i < 100 && g_running; i++)
    {
      usleep(10000);
    }
}

bool px4io_is_running(void)
{
  return g_running;
}

int px4io_daemon_rate(void)
{
  return g_running ? g_rate_hz : 0;
}
