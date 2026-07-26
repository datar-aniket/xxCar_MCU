/****************************************************************************
 * apps/cal/cal.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See cal.h.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <termios.h>
#include <sys/time.h>

#include <uORB/uORB.h>
#include <nuttx/uorb.h>

#include "cal.h"
#include "../param/param.h"
#include "../serial/serial.h"
#include "../uorb_msgs/uorb_msgs.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum cal_kind_e
{
  CAL_KIND_ACCEL = 0,
  CAL_KIND_GYRO,
  CAL_KIND_MAG,
  CAL_KIND_BARO,
  CAL_KIND_FLOW,
  CAL_KIND_DIST
};

/* One offerable sensor.
 *
 * The GUI is told labels and units rather than deducing them from the name, so
 * adding a sensor here is the only change needed to make it appear, selectable
 * and correctly plotted, in the GUI.
 */

struct cal_sensor_s
{
  FAR const char                *name;    /* what the GUI shows and selects */
  FAR const char                *orb;     /* orb_get_meta() name, or NULL */
  FAR const struct orb_metadata *direct;  /* for our own topics, which
                                           * orb_get_meta cannot find by name */
  enum cal_kind_e                kind;
  uint8_t                        nvalues;
  FAR const char                *labels;  /* JSON array, ready to embed */
  FAR const char                *unit;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct cal_sensor_s g_sensors[] =
{
  { "accel0", "sensor_accel0", NULL, CAL_KIND_ACCEL, 3,
    "[\"x\",\"y\",\"z\"]", "m/s^2" },
  { "gyro0",  "sensor_gyro0",  NULL, CAL_KIND_GYRO,  3,
    "[\"x\",\"y\",\"z\"]", "rad/s" },
  { "accel1", "sensor_accel1", NULL, CAL_KIND_ACCEL, 3,
    "[\"x\",\"y\",\"z\"]", "m/s^2" },
  { "gyro1",  "sensor_gyro1",  NULL, CAL_KIND_GYRO,  3,
    "[\"x\",\"y\",\"z\"]", "rad/s" },
  { "mag0",   "sensor_mag0",   NULL, CAL_KIND_MAG,   3,
    "[\"x\",\"y\",\"z\"]", "gauss" },
  { "baro0",  "sensor_baro0",  NULL, CAL_KIND_BARO,  2,
    "[\"pressure\",\"temperature\"]", "hPa,degC" },
  { "flow",   NULL, ORB_ID(optical_flow),    CAL_KIND_FLOW, 4,
    "[\"int_x\",\"int_y\",\"distance\",\"quality\"]", "rad,rad,m,-" },
  { "dist",   NULL, ORB_ID(distance_sensor), CAL_KIND_DIST, 2,
    "[\"distance\",\"quality\"]", "m,%" },
};

#define CAL_NSENSORS ((int)(sizeof(g_sensors) / sizeof(g_sensors[0])))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t cal_now_us(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

static FAR const struct orb_metadata *cal_meta(int i)
{
  return g_sensors[i].direct != NULL ? g_sensors[i].direct
                                     : orb_get_meta(g_sensors[i].orb);
}

/* Write a whole buffer.
 *
 * The fd is non-blocking, so a single write() can come up short or return
 * EAGAIN when the host stalls. Left unchecked that silently truncates a JSON
 * line and desynchronises the reader, which is a miserable thing to debug from
 * the far end of a cable.
 */

static int cal_write(int fd, FAR const void *buf, size_t len)
{
  FAR const uint8_t *p = buf;
  size_t off = 0;
  int spins = 0;

  while (off < len)
    {
      ssize_t n = write(fd, p + off, len - off);

      if (n > 0)
        {
          off += (size_t)n;
          spins = 0;
          continue;
        }

      if (n < 0 && errno != EAGAIN)
        {
          return -errno;
        }

      if (++spins > 200)
        {
          return -ETIMEDOUT;       /* host is not draining; give up */
        }

      usleep(1000);
    }

  return OK;
}

static int cal_emit(int fd, FAR const char *fmt, ...)
{
  char buf[192];
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n < 0 || (size_t)n >= sizeof(buf))
    {
      return -EINVAL;              /* never emit a partial line */
    }

  return cal_write(fd, buf, (size_t)n);
}

/* Report every sensor, whether or not it is publishing.
 *
 * Presence is measured, not assumed: a topic can be advertised and silent (a
 * MAVLink sensor with nothing plugged in does exactly that). The generation
 * counter advances once per published sample, so sampling it across a short
 * window says whether data is actually flowing - and gives the rate for free.
 * The GUI greys out what is absent rather than offering a plot that will never
 * move.
 */

static int cal_cmd_list(int fd)
{
  uint64_t gen[CAL_NSENSORS];
  int      sub[CAL_NSENSORS];
  uint64_t t0;
  int      i;

  for (i = 0; i < CAL_NSENSORS; i++)
    {
      FAR const struct orb_metadata *meta = cal_meta(i);
      struct orb_state st;

      sub[i] = meta != NULL ? orb_subscribe(meta) : -1;
      gen[i] = 0;

      if (sub[i] >= 0 && orb_get_state(sub[i], &st) == 0)
        {
          gen[i] = st.generation;
        }
    }

  t0 = cal_now_us();
  usleep(200000);

  for (i = 0; i < CAL_NSENSORS; i++)
    {
      uint64_t dt = cal_now_us() - t0;
      struct orb_state st;
      unsigned rate = 0;
      bool present = false;

      if (sub[i] >= 0 && orb_get_state(sub[i], &st) == 0 && dt > 0)
        {
          uint64_t d = st.generation - gen[i];

          present = d > 0;
          rate    = (unsigned)((d * 1000000ull) / dt);
        }

      cal_emit(fd,
               "{\"evt\":\"sensor\",\"id\":%d,\"name\":\"%s\",\"n\":%u,"
               "\"labels\":%s,\"unit\":\"%s\",\"present\":%s,\"rate\":%u}\n",
               i, g_sensors[i].name, g_sensors[i].nvalues,
               g_sensors[i].labels, g_sensors[i].unit,
               present ? "true" : "false", rate);

      if (sub[i] >= 0)
        {
          orb_unsubscribe(sub[i]);
        }
    }

  return cal_emit(fd, "{\"evt\":\"ok\",\"what\":\"list\"}\n");
}

/* CRC16-CCITT-FALSE. Small and bit-serial: a frame is at most 24 bytes and
 * these go out at tens of hertz, so a lookup table would cost 512 bytes of
 * flash to save time nobody is waiting for.
 */

static uint16_t cal_crc16(FAR const uint8_t *d, size_t n)
{
  uint16_t crc = 0xffff;
  size_t i;
  int b;

  for (i = 0; i < n; i++)
    {
      crc ^= (uint16_t)d[i] << 8;

      for (b = 0; b < 8; b++)
        {
          crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                               : (uint16_t)(crc << 1);
        }
    }

  return crc;
}

/* Copy the fields the GUI plots out of whichever sample struct this sensor
 * uses. The order here must match the `labels` in g_sensors - that pairing is
 * the whole contract, and it is why labels live in the table beside the sensor
 * rather than being inferred anywhere.
 */

static int cal_read_values(int i, int sub, FAR float *out,
                           FAR uint32_t *t_us)
{
  FAR const struct orb_metadata *meta = cal_meta(i);
  union
  {
    struct sensor_accel      accel;
    struct sensor_gyro       gyro;
    struct sensor_mag        mag;
    struct sensor_baro       baro;
    struct optical_flow_s    flow;
    struct distance_sensor_s dist;
  } d;

  if (meta == NULL || orb_copy(meta, sub, &d) < 0)
    {
      return -ENODATA;
    }

  switch (g_sensors[i].kind)
    {
      case CAL_KIND_ACCEL:
        out[0] = d.accel.x; out[1] = d.accel.y; out[2] = d.accel.z;
        *t_us = (uint32_t)d.accel.timestamp;
        break;

      case CAL_KIND_GYRO:
        out[0] = d.gyro.x; out[1] = d.gyro.y; out[2] = d.gyro.z;
        *t_us = (uint32_t)d.gyro.timestamp;
        break;

      case CAL_KIND_MAG:
        out[0] = d.mag.x; out[1] = d.mag.y; out[2] = d.mag.z;
        *t_us = (uint32_t)d.mag.timestamp;
        break;

      case CAL_KIND_BARO:
        out[0] = d.baro.pressure; out[1] = d.baro.temperature;
        *t_us = (uint32_t)d.baro.timestamp;
        break;

      case CAL_KIND_FLOW:
        out[0] = d.flow.integrated_x;
        out[1] = d.flow.integrated_y;
        out[2] = d.flow.distance;
        out[3] = (float)d.flow.quality;
        *t_us = (uint32_t)d.flow.timestamp;
        break;

      case CAL_KIND_DIST:
        out[0] = d.dist.current_distance;
        out[1] = (float)d.dist.signal_quality;
        *t_us = (uint32_t)d.dist.timestamp;
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

static int cal_send_sample(int fd, int i, uint8_t seq, uint32_t t_us,
                           FAR const float *v)
{
  uint8_t frame[4 + 4 + 4 * CAL_MAX_VALUES + 2];
  uint8_t n = g_sensors[i].nvalues;
  uint8_t len = (uint8_t)(6 + 4 * n);   /* id, seq, t_us, values */
  uint16_t crc;
  size_t off;

  frame[0] = CAL_SYNC;
  frame[1] = len;
  frame[2] = (uint8_t)i;
  frame[3] = seq;
  memcpy(frame + 4, &t_us, 4);
  memcpy(frame + 8, v, (size_t)4 * n);

  off = 8 + (size_t)4 * n;
  crc = cal_crc16(frame + 1, off - 1);   /* over len .. last value */
  memcpy(frame + off, &crc, 2);

  return cal_write(fd, frame, off + 2);
}

/* Raw mode, saving what was there so it can be put back.
 *
 * The protocol frames itself: canonical mode would buffer by line, echo would
 * feed our own output back to us, and CR/LF translation would corrupt a binary
 * frame the moment one contained 0x0d.
 */

static int cal_raw_mode(int fd, FAR struct termios *saved)
{
  struct termios raw;

  if (tcgetattr(fd, saved) < 0)
    {
      return -errno;
    }

  raw = *saved;
  cfmakeraw(&raw);

  return tcsetattr(fd, TCSANOW, &raw) < 0 ? -errno : OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cal_session(void)
{
  struct termios saved;
  char line[96];
  size_t fill = 0;
  bool overlong = false;
  bool done = false;
  int ret;
  int fd;

  /* Streaming state. One sensor at a time: the GUI plots one at a time, and a
   * single subscription keeps the frame format free of any "which sensor is
   * this" ambiguity beyond the id byte.
   */

  int      st_idx = -1;          /* index into g_sensors, -1 = idle */
  int      st_sub = -1;
  uint32_t st_period_us = 0;
  uint64_t st_next = 0;
  uint8_t  st_seq = 0;

  /* The port must be reserved for us. Checking the parameter is not paranoia:
   * a shell on this port would sit blocked in read() and steal our input, and
   * NuttX offers no way to detect that - uart_open() is refcounted with no
   * exclusivity check and TIOCEXCL is declared but unimplemented.
   */

  if (param_i32("SER_USB_FUNC") != SER_FUNC_CAL)
    {
      fprintf(stderr,
              "cal: %s is not reserved for calibration.\n"
              "  param set SER_USB_FUNC %d\n"
              "  param save\n"
              "  reboot          <- required: the shell on this port was\n"
              "                     started at boot and outlives the change\n",
              CAL_DEVPATH, SER_FUNC_CAL);
      return -EBUSY;
    }

  fd = open(CAL_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "cal: cannot open %s: %d%s\n", CAL_DEVPATH, errno,
              errno == ENOTCONN ? " (no USB host attached)" : "");
      return -errno;
    }

  ret = cal_raw_mode(fd, &saved);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  printf("cal: session open on %s - drive it from the GUI\n", CAL_DEVPATH);

  while (!done)
    {
      struct pollfd pfd;
      char ch;

      pfd.fd     = fd;
      pfd.events = POLLIN;

      /* Poll briefly rather than blocking, so streaming stays on schedule
       * while the session is still responsive to commands. 5 ms is well under
       * the shortest period we offer.
       */

      ret = poll(&pfd, 1, st_idx >= 0 ? 5 : 200);

      if (st_idx >= 0 && cal_now_us() >= st_next)
        {
          float v[CAL_MAX_VALUES];
          uint32_t t_us = 0;

          if (cal_read_values(st_idx, st_sub, v, &t_us) == OK)
            {
              if (cal_send_sample(fd, st_idx, st_seq++, t_us, v) < 0)
                {
                  /* The host stopped draining. Stop streaming rather than
                   * spin on a blocked pipe.
                   */

                  orb_unsubscribe(st_sub);
                  st_sub = -1;
                  st_idx = -1;
                }
            }

          st_next += st_period_us;

          /* If we fell behind - a long write, a busy card - resync rather than
           * chase the backlog with a burst.
           */

          if (st_idx >= 0 && cal_now_us() > st_next + st_period_us)
            {
              st_next = cal_now_us() + st_period_us;
            }
        }

      if (ret <= 0)
        {
          continue;
        }

      if ((pfd.revents & (POLLHUP | POLLERR)) != 0)
        {
          ret = -ENOTCONN;         /* cable pulled */
          break;
        }

      if (read(fd, &ch, 1) != 1)
        {
          continue;
        }

      /* CR and LF both end a line. A terminal sends a bare CR on Enter, and
       * accepting only LF would make this unreachable by hand.
       */

      if (ch != '\n' && ch != '\r')
        {
          if (fill < sizeof(line) - 1)
            {
              line[fill++] = ch;
            }
          else
            {
              /* Drop the rest of an over-long line rather than dispatch its
               * truncated prefix as if it were a whole command.
               */

              overlong = true;
            }

          continue;
        }

      line[fill] = '\0';
      fill = 0;

      if (overlong)
        {
          overlong = false;
          cal_emit(fd, "{\"evt\":\"error\",\"msg\":\"line too long\"}\n");
          continue;
        }

      if (line[0] == '\0')
        {
          continue;                /* bare newline */
        }

      if (strcmp(line, "hello") == 0)
        {
          cal_emit(fd,
                   "{\"evt\":\"hello\",\"proto\":%d,\"board\":\"fmuv6c\"}\n",
                   CAL_PROTO_VERSION);
        }
      else if (strcmp(line, "list") == 0)
        {
          cal_cmd_list(fd);
        }
      else if (strncmp(line, "stream ", 7) == 0)
        {
          FAR char *arg = line + 7;
          FAR char *sp = strchr(arg, ' ');
          long hz = 50;
          int i;

          if (sp != NULL)
            {
              *sp = '\0';
              hz = strtol(sp + 1, NULL, 10);
            }

          /* 1..200 Hz. The ceiling is not the link's limit - it is what a plot
           * can show. High-rate capture belongs in the SD logger, which does
           * not have to survive a USB stall.
           */

          if (hz < 1 || hz > 200)
            {
              cal_emit(fd,
                       "{\"evt\":\"error\",\"msg\":\"rate must be 1-200\"}\n");
              continue;
            }

          for (i = 0; i < CAL_NSENSORS; i++)
            {
              if (strcmp(arg, g_sensors[i].name) == 0)
                {
                  break;
                }
            }

          if (i == CAL_NSENSORS)
            {
              cal_emit(fd,
                       "{\"evt\":\"error\",\"msg\":\"no such sensor\"}\n");
              continue;
            }

          if (st_sub >= 0)
            {
              orb_unsubscribe(st_sub);
              st_sub = -1;
              st_idx = -1;
            }

          {
            FAR const struct orb_metadata *meta = cal_meta(i);

            st_sub = meta != NULL ? orb_subscribe(meta) : -1;
          }

          if (st_sub < 0)
            {
              cal_emit(fd, "{\"evt\":\"error\","
                           "\"msg\":\"sensor not available\"}\n");
              continue;
            }

          st_idx       = i;
          st_seq       = 0;
          st_period_us = (uint32_t)(1000000 / hz);
          st_next      = cal_now_us();

          cal_emit(fd,
                   "{\"evt\":\"ok\",\"what\":\"stream\",\"name\":\"%s\","
                   "\"id\":%d,\"hz\":%ld}\n", g_sensors[i].name, i, hz);
        }
      else if (strcmp(line, "stop") == 0)
        {
          if (st_sub >= 0)
            {
              orb_unsubscribe(st_sub);
            }

          st_sub = -1;
          st_idx = -1;
          cal_emit(fd, "{\"evt\":\"ok\",\"what\":\"stop\"}\n");
        }
      else if (strcmp(line, "quit") == 0)
        {
          cal_emit(fd, "{\"evt\":\"ok\",\"what\":\"bye\"}\n");
          done = true;
        }
      else
        {
          cal_emit(fd, "{\"evt\":\"error\",\"msg\":\"unknown command\"}\n");
        }
    }

  if (st_sub >= 0)
    {
      orb_unsubscribe(st_sub);
    }

  tcsetattr(fd, TCSANOW, &saved);
  close(fd);
  return ret;
}
