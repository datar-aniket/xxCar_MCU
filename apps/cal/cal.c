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
#include <math.h>
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
 * The GUI is told labels, units, encoding and scale rather than deducing them,
 * so adding a sensor here is the only change needed to make it appear,
 * selectable and correctly plotted.
 *
 * `scale` is the value of one integer step for CAL_ENC_I16 sensors, chosen to
 * match the driver's own full-scale range so the quantisation is the sensor's,
 * not ours. It is unused for CAL_ENC_F32.
 */

struct cal_sensor_s
{
  FAR const char                *name;
  FAR const char                *orb;
  FAR const struct orb_metadata *direct;
  enum cal_kind_e                kind;
  uint8_t                        nvalues;
  uint8_t                        enc;
  float                          scale;
  FAR const char                *labels;   /* JSON array, ready to embed */
  FAR const char                *unit;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Scales are full-scale-range / 32767, matching how the drivers configure the
 * parts: accel +/-16 g, gyro +/-2000 dps, mag +/-8 gauss.
 */

#define CAL_ACC_SCALE  (16.0f * 9.80665f / 32767.0f)
#define CAL_GYR_SCALE  (2000.0f * 0.017453292519943295f / 32767.0f)
#define CAL_MAG_SCALE  (8.0f / 32767.0f)

static const struct cal_sensor_s g_sensors[] =
{
  { "accel0", "sensor_accel0", NULL, CAL_KIND_ACCEL, 3,
    CAL_ENC_I16, CAL_ACC_SCALE, "[\"x\",\"y\",\"z\"]", "m/s^2" },
  { "gyro0",  "sensor_gyro0",  NULL, CAL_KIND_GYRO,  3,
    CAL_ENC_I16, CAL_GYR_SCALE, "[\"x\",\"y\",\"z\"]", "rad/s" },
  { "accel1", "sensor_accel1", NULL, CAL_KIND_ACCEL, 3,
    CAL_ENC_I16, CAL_ACC_SCALE, "[\"x\",\"y\",\"z\"]", "m/s^2" },
  { "gyro1",  "sensor_gyro1",  NULL, CAL_KIND_GYRO,  3,
    CAL_ENC_I16, CAL_GYR_SCALE, "[\"x\",\"y\",\"z\"]", "rad/s" },
  { "mag0",   "sensor_mag0",   NULL, CAL_KIND_MAG,   3,
    CAL_ENC_I16, CAL_MAG_SCALE, "[\"x\",\"y\",\"z\"]", "gauss" },

  /* Pressure near 1013 hPa beside a temperature near 40 degC do not share a
   * symmetric range, so a single integer scale cannot serve both. These are
   * slow enough that the extra bytes are free.
   */

  { "baro0",  "sensor_baro0",  NULL, CAL_KIND_BARO,  2,
    CAL_ENC_F32, 0.0f, "[\"pressure\",\"temperature\"]", "hPa | degC" },
  { "flow",   NULL, ORB_ID(optical_flow),    CAL_KIND_FLOW, 4,
    CAL_ENC_F32, 0.0f,
    "[\"int_x\",\"int_y\",\"distance\",\"quality\"]", "rad | m" },
  { "dist",   NULL, ORB_ID(distance_sensor), CAL_KIND_DIST, 2,
    CAL_ENC_F32, 0.0f, "[\"distance\",\"quality\"]", "m | %" },
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
 * EAGAIN when the host stalls. Left unchecked that silently truncates a frame
 * and desynchronises the reader, which is a miserable thing to debug from the
 * far end of a cable.
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
  char buf[224];
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

/* CRC16-CCITT-FALSE, table-driven on the high nibble.
 *
 * A full-rate frame is ~300 bytes and there are tens per second, so the
 * bit-serial version was costing real cycles on the path we are trying to make
 * fast. Sixteen entries is 32 bytes of flash.
 */

static const uint16_t g_crc_tab[16] =
{
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
};

static uint16_t cal_crc16(FAR const uint8_t *d, size_t n)
{
  uint16_t crc = 0xffff;
  size_t i;

  for (i = 0; i < n; i++)
    {
      uint8_t hi = (uint8_t)((crc >> 12) ^ (d[i] >> 4)) & 0xf;

      crc = (uint16_t)((crc << 4) ^ g_crc_tab[hi]);

      hi  = (uint8_t)((crc >> 12) ^ (d[i] & 0xf)) & 0xf;
      crc = (uint16_t)((crc << 4) ^ g_crc_tab[hi]);
    }

  return crc;
}

/* Copy the fields the GUI plots out of whichever sample struct this sensor
 * uses. The order must match the `labels` in g_sensors - that pairing is the
 * whole contract, and it is why labels live beside the sensor rather than being
 * inferred anywhere.
 */

static int cal_read_values(int i, int sub, FAR float *out, FAR uint32_t *t_us)
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

/* Report every sensor, whether or not it is publishing.
 *
 * Presence is measured, not assumed: a topic can be advertised and silent (a
 * MAVLink sensor with nothing plugged in does exactly that). The generation
 * counter advances once per published sample, so sampling it across a window
 * says whether data is actually flowing - and gives the rate for free.
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

      if (sub[i] >= 0)
        {
          /* Ask for a rate, or the on-demand sensors publish nothing.
           *
           * The IMUs free-run off a hardware FIFO and are always producing.
           * The MS5611 baro and IST8310 mag use NuttX's polled uorb drivers,
           * whose kthread samples only at the interval a SUBSCRIBER asks for -
           * default 1 Hz. Without this a perfectly healthy baro reports zero
           * samples in the window below and is shown as absent.
           */

          orb_set_interval(sub[i], 20000);        /* 50 Hz */

          if (orb_get_state(sub[i], &st) == 0)
            {
              gen[i] = st.generation;
            }
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
               "\"enc\":%u,\"scale\":%.9g,\"labels\":%s,\"unit\":\"%s\","
               "\"present\":%s,\"rate\":%u}\n",
               i, g_sensors[i].name, g_sensors[i].nvalues,
               g_sensors[i].enc, (double)g_sensors[i].scale,
               g_sensors[i].labels, g_sensors[i].unit,
               present ? "true" : "false", rate);

      if (sub[i] >= 0)
        {
          orb_unsubscribe(sub[i]);
        }
    }

  return cal_emit(fd, "{\"evt\":\"ok\",\"what\":\"list\"}\n");
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
   * single subscription keeps the frame free of any "which sensor is this"
   * ambiguity beyond the id byte.
   */

  int      st_idx = -1;              /* index into g_sensors, -1 = idle */
  int      st_sub = -1;
  uint32_t st_dt_us = 0;             /* nominal spacing, for the frame header */
  uint8_t  st_seq = 0;

  /* Batch accumulator. */

  static uint8_t  frame[16 + CAL_BATCH_MAX * CAL_MAX_VALUES * 4 + 2];
  float    acc_f[CAL_BATCH_MAX * CAL_MAX_VALUES];
  uint8_t  nbatch = 0;
  uint32_t batch_t0 = 0;
  uint64_t batch_since = 0;

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
  ret = OK;

  while (!done)
    {
      struct pollfd pfd[2];
      nfds_t nfds = 1;
      char ch;

      pfd[0].fd     = fd;
      pfd[0].events = POLLIN;

      /* Poll the subscription too, rather than waking on a timer.
       *
       * The timer version could not exceed 1/timeout samples a second no
       * matter what rate was asked for - a 5 ms tick capped the stream at
       * ~166 Hz. Waiting on the uORB fd instead means we wake exactly when
       * there is data, so the rate is the sensor's, not the loop's.
       */

      if (st_sub >= 0)
        {
          pfd[1].fd     = st_sub;
          pfd[1].events = POLLIN;
          nfds = 2;
        }

      if (poll(pfd, nfds, 200) < 0 && errno != EINTR)
        {
          break;
        }

      if ((pfd[0].revents & (POLLHUP | POLLERR)) != 0)
        {
          ret = -ENOTCONN;           /* cable pulled */
          break;
        }

      /* ---- drain samples into the batch ---------------------------- */

      if (st_sub >= 0 && nfds == 2 && (pfd[1].revents & POLLIN) != 0)
        {
          while (nbatch < CAL_BATCH_MAX)
            {
              uint32_t t_us = 0;
              FAR float *slot = &acc_f[(size_t)nbatch *
                                       g_sensors[st_idx].nvalues];

              if (cal_read_values(st_idx, st_sub, slot, &t_us) != OK)
                {
                  break;             /* nothing more queued */
                }

              if (nbatch == 0)
                {
                  batch_t0    = t_us;
                  batch_since = cal_now_us();
                }

              nbatch++;
            }
        }

      /* ---- flush when full, or when it has been waiting too long ---- */

      if (nbatch > 0 &&
          (nbatch >= CAL_BATCH_MAX || cal_now_us() - batch_since >= 20000))
        {
          FAR const struct cal_sensor_s *s = &g_sensors[st_idx];
          size_t nv = (size_t)nbatch * s->nvalues;
          size_t bytes;
          size_t off;
          uint16_t len;
          uint16_t crc;
          size_t k;

          if (s->enc == CAL_ENC_I16)
            {
              FAR int16_t *p = (FAR int16_t *)(frame + 14);
              float inv = 1.0f / s->scale;

              for (k = 0; k < nv; k++)
                {
                  float q = acc_f[k] * inv;

                  /* Saturate rather than wrap: a clipped reading is obvious on
                   * a plot, a wrapped one looks like a real excursion in the
                   * opposite direction.
                   */

                  p[k] = q >= 32767.0f ? 32767 :
                         q <= -32768.0f ? -32768 : (int16_t)lrintf(q);
                }

              bytes = nv * 2;
            }
          else
            {
              memcpy(frame + 14, acc_f, nv * sizeof(float));
              bytes = nv * 4;
            }

          len = (uint16_t)(11 + bytes);

          frame[0] = CAL_SYNC;
          frame[1] = (uint8_t)(len & 0xff);
          frame[2] = (uint8_t)(len >> 8);
          frame[3] = (uint8_t)st_idx;
          frame[4] = st_seq++;
          memcpy(frame + 5, &batch_t0, 4);
          frame[9]  = (uint8_t)(st_dt_us & 0xff);
          frame[10] = (uint8_t)(st_dt_us >> 8);
          frame[11] = nbatch;
          frame[12] = s->nvalues;
          frame[13] = s->enc;

          off = 14 + bytes;
          crc = cal_crc16(frame + 1, off - 1);
          frame[off]     = (uint8_t)(crc & 0xff);
          frame[off + 1] = (uint8_t)(crc >> 8);

          if (cal_write(fd, frame, off + 2) < 0)
            {
              orb_unsubscribe(st_sub);
              st_sub = -1;
              st_idx = -1;
            }

          nbatch = 0;
        }

      /* ---- commands ------------------------------------------------ */

      if ((pfd[0].revents & POLLIN) == 0 || read(fd, &ch, 1) != 1)
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
              overlong = true;       /* drop to EOL, do not dispatch a stub */
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
          continue;
        }

      if (strcmp(line, "hello") == 0)
        {
          cal_emit(fd,
                   "{\"evt\":\"hello\",\"proto\":%d,\"board\":\"fmuv6c\","
                   "\"batch\":%d}\n", CAL_PROTO_VERSION, CAL_BATCH_MAX);
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

          if (hz < 1 || hz > 2000)
            {
              cal_emit(fd,
                       "{\"evt\":\"error\",\"msg\":\"rate must be 1-2000\"}\n");
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
              cal_emit(fd, "{\"evt\":\"error\","
                           "\"msg\":\"no such sensor\"}\n");
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

          /* Drive the sensor at the rate we intend to send. For the polled
           * drivers this is what makes them sample at all; for the free-running
           * IMUs it throttles delivery to what was asked for, instead of
           * copying 2000 samples a second to send 50.
           */

          st_dt_us = (uint32_t)(1000000 / hz);
          orb_set_interval(st_sub, st_dt_us);

          st_idx = i;
          st_seq = 0;
          nbatch = 0;

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
          nbatch = 0;
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
