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
  struct pollfd pfd;
  uint64_t last_frame = 0;
  uint8_t proto;
  int rcfd;
  int fd;

  UNUSED(argc);
  UNUSED(argv);

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
      rc_configure_port(fd, proto);
      syslog(LOG_INFO, "rc: %s SBUS (forced)\n", g_devpath);
    }
  else if (g_proto_param == RC_PROT_CRSF)
    {
      proto = RC_PROTO_CRSF;
      rc_configure_port(fd, proto);
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

  while (!g_should_stop)
    {
      uint8_t buf[64];
      ssize_t n;
      int ret;

      ret = poll(&pfd, 1, 20);

      if (ret > 0)
        {
          n = read(fd, buf, sizeof(buf));

          if (n > 0 && rc_decode(&dec, buf, (size_t)n, &frame))
            {
              struct rc_in_s msg;
              unsigned i;

              last_frame = rc_now_us();

              pthread_mutex_lock(&g_lock);
              g_status.frames++;
              g_status.errors   = dec.errors;
              g_status.ok       = !frame.failsafe;
              g_status.failsafe = frame.failsafe;
              g_status.last     = frame;
              pthread_mutex_unlock(&g_lock);

              if (rcfd >= 0)
                {
                  memset(&msg, 0, sizeof(msg));
                  msg.timestamp = last_frame;
                  msg.count     = frame.count;
                  msg.ok        = !frame.failsafe;
                  msg.failsafe  = frame.failsafe;
                  msg.frames    = (uint16_t)g_status.frames;
                  msg.rssi      = frame.failsafe ? 0 : 255;
                  msg.source    = (proto == RC_PROTO_SBUS) ? RC_IN_SRC_SBUS
                                                           : RC_IN_SRC_CRSF;

                  for (i = 0; i < frame.count && i < RC_IN_MAX_CHANNELS; i++)
                    {
                      msg.channel[i] = frame.channel[i];
                    }

                  rc_in_publish(rcfd, &msg);
                }
            }
        }

      /* A receiver that stops sending is a lost link. Neither protocol
       * announces that - SBUS's failsafe bit only fires if the RECEIVER still
       * has power and knows it lost the transmitter. An unplugged cable just
       * goes quiet, so silence has to be treated as loss.
       */

      if (last_frame != 0 && (rc_now_us() - last_frame) > RC_TIMEOUT_US)
        {
          pthread_mutex_lock(&g_lock);

          if (g_status.ok)
            {
              g_status.timeouts++;
              syslog(LOG_WARNING, "rc: %s link lost\n", g_devpath);
            }

          g_status.ok = false;
          pthread_mutex_unlock(&g_lock);
        }
    }

out:
  close(fd);
  g_running = false;
  return EXIT_SUCCESS;
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

  /* PPM cannot be done here, and saying so is more useful than trying.
   *
   * PPM is a pulse train measured with a timer's input capture, not a byte
   * stream - a UART cannot see it at all. On this board the PPM/SBUS RC IN
   * connector is wired to the PX4IO co-processor, which decodes PPM itself and
   * hands over channels (px4io status shows RC_PPM when it does). So the answer
   * to "I want PPM" is apps/px4io, not this driver.
   */

  if (proto_param == RC_PROT_PPM)
    {
      syslog(LOG_ERR,
             "rc: PPM cannot be decoded on a serial port - it is a pulse "
             "train, not a byte stream.\n");
      syslog(LOG_ERR,
             "rc: plug the receiver into RC IN; PX4IO decodes PPM already "
             "(see `px4io status`).\n");
      return -ENOTSUP;
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
