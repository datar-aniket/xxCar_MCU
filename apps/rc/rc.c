/****************************************************************************
 * apps/rc/rc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * RC receiver driver for a plain FMU serial port. See rc.h.
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
#include <sys/ioctl.h>
#include <sys/time.h>

#include <nuttx/serial/tioctl.h>
#include <arch/board/board.h>

#include "rc.h"
#include "../param/param.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RC_STACK  3072
#define RC_PRIO   (SCHED_PRIORITY_DEFAULT + 5)

/* How long to listen on one protocol's line settings before concluding the
 * receiver is not speaking it. A receiver sends every 7-20 ms, so 300 ms is a
 * dozen or more frames - if none of them decoded, we have the wrong settings.
 */

#define RC_PROBE_MS  300

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_mutex_t   g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool     g_running;
static volatile bool     g_should_stop;

static char              g_devpath[16];
static int32_t           g_proto_param;

static struct rc_status_s g_status;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rc_configure_port(int fd, uint8_t proto)
{
  struct termios tio;
  int invert = 0;

  if (tcgetattr(fd, &tio) < 0)
    {
      return -errno;
    }

  cfmakeraw(&tio);
  tio.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB);
  tio.c_cflag |= CS8;
  tio.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
  tio.c_cflag &= ~CRTSCTS;
#endif

  if (proto == RC_PROTO_SBUS)
    {
      /* 100000 baud, 8 data bits, EVEN parity, TWO stop bits. All three are
       * part of the protocol - a receiver that is speaking SBUS into a port set
       * to 8N1 produces framing errors, not data.
       */

      cfsetspeed(&tio, SBUS_BAUD);
      tio.c_cflag |= PARENB;   /* even: PARENB set, PARODD clear */
      tio.c_cflag |= CSTOPB;   /* two stop bits */

      /* And the signal is INVERTED. There is no inverter on the FMU's serial
       * connectors - the one on this board sits in front of PX4IO, on the RC IN
       * line. So the STM32's own RXINV has to do it.
       */

      invert = SER_INVERT_ENABLED_RX;
    }
  else
    {
      /* CRSF/ELRS: 420000 baud, 8N1, not inverted. */

      cfsetspeed(&tio, CRSF_BAUD);
    }

  if (tcsetattr(fd, TCSANOW, &tio) < 0)
    {
      return -errno;
    }

  /* Set the polarity AFTER the line settings: TCSETS reconfigures the USART,
   * and doing it the other way round would throw the inversion away.
   */

  if (ioctl(fd, TIOCSINVERT, invert) < 0)
    {
      /* Without inversion SBUS cannot work at all, so say so plainly rather
       * than leave someone staring at a silent receiver.
       */

      if (proto == RC_PROTO_SBUS)
        {
          syslog(LOG_ERR,
                 "rc: cannot invert RX (%d) - SBUS needs it. Is "
                 "CONFIG_STM32H7_USART_INVERT enabled?\n", errno);
          return -errno;
        }
      return -errno; /* Cannot assume a previous RX inversion was cleared. */
    }

  tcflush(fd, TCIFLUSH);
  return OK;
}

int rc_get_status(FAR struct rc_status_s *status)
{
  if (status == NULL)
    {
      return -EINVAL;
    }

  pthread_mutex_lock(&g_lock);
  *status = g_status;
  status->running = g_running;
  pthread_mutex_unlock(&g_lock);

  return OK;
}

bool rc_is_running(void)
{
  return g_running;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t rc_now_us(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

#ifdef CONFIG_XXCAR_BOARD_MATEKH743
static int rc_ppm_daemon(int rcfd)
{
  struct board_rc_ppm_frame_s frame;
  struct rc_in_s held;
  uint32_t last_attempts = 0;
  uint32_t last_sequence = 0;
  uint64_t last_packet = 0;
  bool have_valid = false;
  bool link_ok = false;
  int32_t channels = 8;
  int ret;

  memset(&held, 0, sizeof(held));
  held.source = RC_IN_SRC_PPM;

  param_get_i32("RC_PPM_CH", &channels);
  ret = board_matek_rc_ppm_start((unsigned)channels);
  if (ret < 0)
    {
      syslog(LOG_ERR, "rc: cannot capture PPM on R6/PC7: %d\n", ret);
      return ret;
    }

  pthread_mutex_lock(&g_lock);
  g_status.proto = RC_PROTO_PPM;
  g_status.locked = true;
  pthread_mutex_unlock(&g_lock);
  syslog(LOG_INFO, "rc: R6/PC7 PPM capture (TIM3_CH2)\n");

  while (!g_should_stop)
    {
      memset(&frame, 0, sizeof(frame));
      if (board_matek_rc_ppm_latest(&frame) &&
          frame.attempts != last_attempts)
        {
          unsigned i;

          if (frame.sequence != last_sequence)
            {
              held.count = frame.count;
              for (i = 0; i < frame.count; i++)
                {
                  held.channel[i] = frame.channel[i];
                }

              have_valid = true;
              last_sequence = frame.sequence;
              last_packet = rc_now_us();
            }

          last_attempts = frame.attempts;

          /* frame.timestamp_us is the edge time from the dedicated TIM5
           * sensor clock.  rc_in timestamps are consumed by the control
           * router and its timeout logic in CLOCK_MONOTONIC.  Mixing the
           * two made only PPM (not SBUS/CRSF) age, glitch, and eventually
           * disarm as the clocks separated.
           */

          held.timestamp = last_packet;
          held.frames = (uint16_t)frame.sequence;
          held.lost_frames = (uint16_t)frame.errors;
          held.ok = have_valid &&
                    frame.invalid_streak < RC_INVALID_LIMIT &&
                    rc_now_us() - last_packet < RC_TIMEOUT_US;
          held.failsafe = false;
          held.rssi = held.ok ? 255 : 0;

          pthread_mutex_lock(&g_lock);
          g_status.frames = frame.sequence;
          g_status.last_valid_us = last_packet;
          g_status.errors = frame.errors;
          g_status.lost_frames = (uint16_t)frame.errors;
          g_status.invalid_streak = frame.invalid_streak;
          g_status.ok = held.ok;
          g_status.failsafe = false;
          g_status.last.count = held.count;
          g_status.last.failsafe = false;
          g_status.last.frame_lost = frame.invalid_streak != 0;
          for (i = 0; i < held.count; i++)
            {
              g_status.last.channel[i] = held.channel[i];
            }
          pthread_mutex_unlock(&g_lock);

          if (link_ok && !held.ok)
            {
              syslog(LOG_WARNING,
                     "rc: R6 PPM lost after %u consecutive invalid frames\n",
                     frame.invalid_streak);
            }

          link_ok = held.ok;
          if (rcfd >= 0)
            {
              rc_in_publish(rcfd, &held);
            }
        }

      if (last_packet != 0 && rc_now_us() - last_packet > RC_TIMEOUT_US)
        {
          pthread_mutex_lock(&g_lock);
          if (g_status.ok)
            {
              g_status.timeouts++;
              syslog(LOG_WARNING, "rc: R6 PPM link lost\n");
            }
          g_status.ok = false;
          pthread_mutex_unlock(&g_lock);
          if (held.ok)
            {
              held.ok = false;
              held.rssi = 0;
              if (rcfd >= 0)
                {
                  rc_in_publish(rcfd, &held);
                }
            }
          link_ok = false;
        }

      usleep(2000);
    }

  board_matek_rc_ppm_stop();
  held.ok = false;
  held.rssi = 0;
  if (rcfd >= 0) rc_in_publish(rcfd, &held);
  pthread_mutex_lock(&g_lock);
  g_status.ok = false;
  pthread_mutex_unlock(&g_lock);
  return OK;
}
#endif

/* Listen on one protocol's line settings for RC_PROBE_MS and report whether a
 * valid frame turned up.
 *
 * This IS the autodetection. SBUS and CRSF differ in baud, parity, stop bits
 * and polarity, so a port can only listen for one at a time - there is no way
 * to sniff both at once and decide afterwards. Trying one and seeing whether it
 * decodes is not a heuristic; it is the only thing that can work.
 */

static bool rc_probe(int fd, uint8_t proto, FAR struct rc_frame_s *out)
{
  struct rc_decoder_s dec;
  struct pollfd pfd;
  uint64_t deadline;

  if (rc_configure_port(fd, proto) < 0)
    {
      return false;
    }

  rc_decoder_reset(&dec, proto);

  pfd.fd     = fd;
  pfd.events = POLLIN;
  deadline   = rc_now_us() + (uint64_t)RC_PROBE_MS * 1000;

  while (rc_now_us() < deadline)
    {
      uint8_t buf[64];
      ssize_t n;

      if (poll(&pfd, 1, 20) <= 0)
        {
          rc_decoder_idle(&dec);
          continue;
        }

      n = read(fd, buf, sizeof(buf));
      if (n <= 0)
        {
          continue;
        }

      if (rc_decode(&dec, buf, (size_t)n, out))
        {
          return true;
        }
    }

  return false;
}

static int rc_daemon(int argc, FAR char *argv[])
{
  struct rc_decoder_s dec;
  struct rc_frame_s frame;
  struct rc_link_quality_s quality;
  struct rc_in_s held;
  struct pollfd pfd;
  uint8_t proto;
  int rcfd;
  int fd;

  UNUSED(argc);
  UNUSED(argv);
  memset(&held, 0, sizeof(held));
  rc_link_quality_reset(&quality);

#ifdef CONFIG_XXCAR_BOARD_MATEKH743
  if (g_proto_param == RC_PROT_PPM)
    {
      int result;

      rcfd = rc_in_advertise();
      result = rc_ppm_daemon(rcfd);
      if (rcfd >= 0)
        {
          orb_unadvertise(rcfd);
        }
      g_running = false;
      return result < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
    }
#endif

  fd = open(g_devpath, O_RDWR | O_NOCTTY);
  if (fd < 0)
    {
      syslog(LOG_ERR, "rc: cannot open %s: %d\n", g_devpath, errno);
      g_running = false;
      return EXIT_FAILURE;
    }

  rcfd = rc_in_advertise();

  /* Work out what we are listening to. */

  if (g_proto_param == RC_PROT_SBUS)
    {
      proto = RC_PROTO_SBUS;
      if (rc_configure_port(fd, proto) < 0)
        {
          goto config_failed;
        }
      syslog(LOG_INFO, "rc: %s SBUS (forced)\n", g_devpath);
    }
  else if (g_proto_param == RC_PROT_CRSF)
    {
      proto = RC_PROTO_CRSF;
      if (rc_configure_port(fd, proto) < 0)
        {
          goto config_failed;
        }
      syslog(LOG_INFO, "rc: %s CRSF (forced)\n", g_devpath);
    }
  else
    {
      /* Autodetect: alternate between the two until one of them decodes. Keep
       * alternating rather than giving up, so a receiver that is powered on
       * after the FMU still gets picked up.
       */

      proto = RC_PROTO_NONE;

      syslog(LOG_INFO, "rc: %s probing for SBUS / CRSF\n", g_devpath);

      while (!g_should_stop && proto == RC_PROTO_NONE)
        {
          if (rc_probe(fd, RC_PROTO_SBUS, &frame))
            {
              proto = RC_PROTO_SBUS;
            }
          else if (rc_probe(fd, RC_PROTO_CRSF, &frame))
            {
              proto = RC_PROTO_CRSF;
            }
        }

      if (g_should_stop)
        {
          goto out;
        }

      syslog(LOG_INFO, "rc: %s detected %s\n", g_devpath,
             proto == RC_PROTO_SBUS ? "SBUS" : "CRSF");
    }

  pthread_mutex_lock(&g_lock);
  g_status.proto  = proto;
  g_status.locked = true;
  pthread_mutex_unlock(&g_lock);

  rc_decoder_reset(&dec, proto);

  pfd.fd     = fd;
  pfd.events = POLLIN;

  uint64_t last_service = rc_now_us();
  while (!g_should_stop)
    {
      uint8_t buf[64];
      ssize_t n;
      int ret;

      ret = poll(&pfd, 1, 20);
      uint64_t service = rc_now_us();
      if (service - last_service >= RC_TIMEOUT_US)
        {
          /* UART bytes have no arrival timestamps. After task starvation,
           * queued old channels must not be relabelled as fresh commands.
           */
          tcflush(fd, TCIFLUSH);
          rc_decoder_idle(&dec);
          quality.have_valid = false;
          held.ok = false;
          held.rssi = 0;
          pthread_mutex_lock(&g_lock);
          g_status.ok = false;
          g_status.timeouts++;
          pthread_mutex_unlock(&g_lock);
          if (rcfd >= 0) rc_in_publish(rcfd, &held);
          last_service = service;
          continue;
        }
      last_service = service;

      if (ret == 0 && rc_decoder_idle(&dec))
        {
          rc_link_note_invalid(&quality, 1);
        }

      if (ret > 0 && (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)))
        {
          syslog(LOG_ERR, "rc: UART poll fault: %lx\n",
                 (unsigned long)pfd.revents);
          break;
        }

      if (ret > 0 && (pfd.revents & POLLIN))
        {
          n = read(fd, buf, sizeof(buf));

          /* Feed one byte at a time so every complete valid or rejected
           * packet is observed in wire order. Feeding the whole read at once
           * collapses multiple packets into one boolean result and can make
           * an invalid streak depend on where the UART read was split.
           */

          for (ssize_t byte = 0; byte < n; byte++)
            {
              uint32_t errors_before = dec.errors;

              if (rc_decode(&dec, &buf[byte], 1, &frame))
                {
                  bool packet_valid;
                  bool link_ok;
                  bool was_ok;
                  uint32_t frames;
                  unsigned i;

                  packet_valid = !frame.failsafe && !frame.frame_lost;
                  link_ok = rc_link_frame(&quality, &frame, rc_now_us());

                  pthread_mutex_lock(&g_lock);
                  was_ok = g_status.ok;
                  g_status.frames++;
                  frames = g_status.frames;
                  g_status.errors = dec.errors;
                  g_status.invalid_streak = quality.invalid_streak;
                  g_status.ok = link_ok;
                  g_status.failsafe = quality.failsafe;
                  g_status.last_valid_us = quality.last_valid_us;
                  g_status.lost_frames = quality.lost_frames;
                  g_status.last = frame;
                  pthread_mutex_unlock(&g_lock);

                  if (packet_valid)
                    {
                      held.count = frame.count;
                      for (i = 0;
                           i < frame.count && i < RC_IN_MAX_CHANNELS;
                           i++)
                        {
                          held.channel[i] = frame.channel[i];
                        }
                    }

                  held.timestamp = quality.last_valid_us;
                  held.ok = link_ok;
                  held.failsafe = quality.failsafe;
                  held.frames = (uint16_t)frames;
                  held.lost_frames = quality.lost_frames;
                  held.rssi = link_ok ? 255 : 0;
                  held.source = (proto == RC_PROTO_SBUS) ? RC_IN_SRC_SBUS
                                                         : RC_IN_SRC_CRSF;

                  if (rcfd >= 0)
                    {
                      rc_in_publish(rcfd, &held);
                    }

                  if (was_ok && !link_ok)
                    {
                      syslog(LOG_WARNING,
                             "rc: %s lost after %u consecutive invalid "
                             "frames\n",
                             g_devpath, quality.invalid_streak);
                    }
                }
              else if (dec.errors != errors_before)
                {
                  bool link_ok;
                  bool was_ok;

                  rc_link_note_invalid(&quality, 1);
                  link_ok = rc_link_ok(&quality, rc_now_us());
                  pthread_mutex_lock(&g_lock);
                  was_ok = g_status.ok;
                  g_status.errors = dec.errors;
                  g_status.lost_frames = quality.lost_frames;
                  g_status.invalid_streak = quality.invalid_streak;
                  g_status.ok = link_ok;
                  g_status.failsafe = quality.failsafe;
                  pthread_mutex_unlock(&g_lock);

                  if (quality.have_valid)
                    {
                      held.ok = link_ok;
                      held.failsafe = quality.failsafe;
                      held.lost_frames = quality.lost_frames;
                      held.rssi = link_ok ? 255 : 0;
                      if (rcfd >= 0)
                        {
                          rc_in_publish(rcfd, &held);
                        }
                    }

                  if (was_ok && !link_ok)
                    {
                      syslog(LOG_WARNING,
                             "rc: %s lost after %u consecutive invalid "
                             "frames\n",
                             g_devpath, quality.invalid_streak);
                    }
                }
            }
        }

      /* A receiver that stops sending is a lost link. Neither protocol
       * announces that - SBUS's failsafe bit only fires if the RECEIVER still
       * has power and knows it lost the transmitter. An unplugged cable just
       * goes quiet, so silence has to be treated as loss.
       */

      if (!rc_link_ok(&quality, rc_now_us()))
        {
          pthread_mutex_lock(&g_lock);

          if (g_status.ok)
            {
              g_status.timeouts++;
              syslog(LOG_WARNING, "rc: %s link lost\n", g_devpath);
            }

          g_status.ok = false;
          g_status.errors = dec.errors;
          g_status.invalid_streak = quality.invalid_streak;
          pthread_mutex_unlock(&g_lock);
          if (held.ok)
            {
              held.ok = false;
              held.rssi = 0;
              if (rcfd >= 0)
                {
                  rc_in_publish(rcfd, &held);
                }
            }
        }
    }

out:
  held.ok = false;
  held.rssi = 0;
  if (rcfd >= 0)
    {
      rc_in_publish(rcfd, &held);
      orb_unadvertise(rcfd);
    }
  pthread_mutex_lock(&g_lock);
  g_status.ok = false;
  pthread_mutex_unlock(&g_lock);
  close(fd);
  g_running = false;
  return EXIT_SUCCESS;

config_failed:
  syslog(LOG_ERR, "rc: line configuration failed on %s\n", g_devpath);
  if (rcfd >= 0)
    {
      orb_unadvertise(rcfd);
    }
  close(fd);
  g_running = false;
  return EXIT_FAILURE;
}

int rc_start(FAR const char *devpath, int32_t proto_param)
{
  int pid;

  if (g_running)
    {
      return -EALREADY;
    }

  if (devpath == NULL)
    {
      return -EINVAL;
    }

  if (proto_param == RC_PROT_PPM)
    {
#ifdef CONFIG_XXCAR_BOARD_MATEKH743
      if (strcmp(devpath, "/dev/ttyS4") != 0)
        {
          syslog(LOG_ERR,
                 "rc: Matek PPM is physically available only on R6/PC7 "
                 "(RCIN)\n");
          return -ENOTSUP;
        }
#else
      syslog(LOG_ERR,
             "rc: PPM cannot be decoded on a serial port - it is a pulse "
             "train, not a byte stream.\n");
      syslog(LOG_ERR,
             "rc: plug the receiver into RC IN; PX4IO decodes PPM already "
             "(see `px4io status`).\n");
      return -ENOTSUP;
#endif
    }

  strlcpy(g_devpath, devpath, sizeof(g_devpath));
  g_proto_param = proto_param;
  g_should_stop = false;

  memset(&g_status, 0, sizeof(g_status));
  g_running = true;

  /* A task, not a pthread: the driver must outlive whatever started it. Started
   * from the `rc start` command, a detached pthread would be killed the instant
   * that command returned (its task group is torn down on exit). A task is its
   * own group. See apps/px4io and apps/logger for the same reason.
   */

  pid = task_create("rc", RC_PRIO, RC_STACK, rc_daemon, NULL);
  if (pid < 0)
    {
      g_running = false;
      return -errno;
    }

  return OK;
}

void rc_stop(void)
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
