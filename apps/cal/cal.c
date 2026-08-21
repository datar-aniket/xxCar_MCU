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
#include <inttypes.h>
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
#include "cal_accel.h"
#include "cal_gyro.h"
#include "cal_mag.h"
#include "../param/param.h"
#include "../serial/serial.h"
#include "../uorb_msgs/uorb_msgs.h"

#ifdef CONFIG_XXCAR_LOGGER
#  include "../logger/logger.h"
#endif

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
  uint8_t                        orb_instance;
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
  { "accel0", "sensor_accel0", NULL, 0, CAL_KIND_ACCEL, 3,
    CAL_ENC_I16, CAL_ACC_SCALE, "[\"x\",\"y\",\"z\"]", "m/s^2" },
  { "gyro0",  "sensor_gyro0",  NULL, 0, CAL_KIND_GYRO,  3,
    CAL_ENC_I16, CAL_GYR_SCALE, "[\"x\",\"y\",\"z\"]", "rad/s" },
  { "accel1", "sensor_accel1", NULL, 1, CAL_KIND_ACCEL, 3,
    CAL_ENC_I16, CAL_ACC_SCALE, "[\"x\",\"y\",\"z\"]", "m/s^2" },
  { "gyro1",  "sensor_gyro1",  NULL, 1, CAL_KIND_GYRO,  3,
    CAL_ENC_I16, CAL_GYR_SCALE, "[\"x\",\"y\",\"z\"]", "rad/s" },
  { "mag0",   "sensor_mag0",   NULL, 0, CAL_KIND_MAG,   3,
    CAL_ENC_I16, CAL_MAG_SCALE, "[\"x\",\"y\",\"z\"]", "gauss" },

  /* Pressure near 1013 hPa beside a temperature near 40 degC do not share a
   * symmetric range, so a single integer scale cannot serve both. These are
   * slow enough that the extra bytes are free.
   */

  { "baro0",  "sensor_baro0",  NULL, 0, CAL_KIND_BARO,  2,
    CAL_ENC_F32, 0.0f, "[\"pressure\",\"temperature\"]", "hPa | degC" },
  { "flow",   NULL, ORB_ID(optical_flow),    0, CAL_KIND_FLOW, 4,
    CAL_ENC_F32, 0.0f,
    "[\"int_x\",\"int_y\",\"distance\",\"quality\"]", "rad | m" },
  { "dist",   NULL, ORB_ID(distance_sensor), 0, CAL_KIND_DIST, 2,
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

static int cal_subscribe(int i)
{
  FAR const struct orb_metadata *meta = cal_meta(i);

  return meta != NULL
         ? orb_subscribe_multi(meta, g_sensors[i].orb_instance) : -1;
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
  char buf[512];
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

/****************************************************************************
 * Alignment streaming
 *
 * A SECOND streaming path, deliberately separate from the single-sensor one
 * below rather than a generalisation of it.
 *
 * Alignment needs several sensors at once on a shared timebase - correlating
 * a magnetometer against a gyro is the whole procedure - and the existing
 * stream is single-sensor by construction. Generalising it looked like the
 * obvious move until the call sites were counted: st_idx is threaded through
 * the calibration preview, the six-position accelerometer run and the
 * magnetometer staging, none of which have anything to do with alignment and
 * all of which would have to be re-reasoned about. That is a large risk to
 * the path you calibrate THROUGH, for no benefit.
 *
 * So this path shares the frame format and nothing else. It always sends raw
 * floats: no calibration is applied, and no integer quantisation, because a
 * rotation solve wants the sensor's own numbers and the bandwidth is trivial
 * at these rates.
 ****************************************************************************/

#define CAL_ALIGN_MAX 6

struct cal_align_s
{
  int      idx;                    /* index into g_sensors, -1 = unused */
  int      sub;
  uint8_t  seq;
  uint8_t  nbatch;
  uint32_t batch_t0;
  uint64_t batch_since;
  float    acc[CAL_BATCH_MAX * CAL_MAX_VALUES];
};

/* File scope, not on the stack: CAL_BATCH_MAX * CAL_MAX_VALUES floats times
 * six entries is several kilobytes and the cal task's stack cannot hold it.
 */

static struct cal_align_s g_align[CAL_ALIGN_MAX];
static int      g_align_n;
static uint32_t g_align_dt_us;

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
      struct orb_state st;

      sub[i] = cal_subscribe(i);
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

/* Per-axis offset and scale for one sensor, as stored in the parameters.
 *
 *     corrected = (raw - off) * scl
 *
 * A gyro has offsets only; its scale stays 1 because nothing here can measure
 * it - that needs a known rotation, which a desk cannot provide.
 */

struct cal_apply_s
{
  float off[3];
  float scl[3];
  float matrix[3][3];
  bool  on;
  bool  full_matrix;
};

static const char *const g_cal_prefix[] =
{
  "CAL_ACC0", "CAL_GYRO0", "CAL_ACC1", "CAL_GYRO1", "CAL_MAG0"
};

/* Load the stored calibration for a streaming sensor, if it has one and the
 * host asked for corrected data. Sensors with no calibration model (baro,
 * flow, distance) simply stay off.
 */

static void cal_load_apply(int i, bool want, FAR struct cal_apply_s *a)
{
  char name[PARAM_NAME_MAX + 1];
  FAR const char *pfx = NULL;
  bool is_accel;
  bool is_mag;
  int k;

  memset(a, 0, sizeof(*a));
  a->scl[0] = a->scl[1] = a->scl[2] = 1.0f;
  a->matrix[0][0] = 1.0f;
  a->matrix[1][1] = 1.0f;
  a->matrix[2][2] = 1.0f;

  if (!want || i < 0 || i > 4)
    {
      return;
    }

  pfx      = g_cal_prefix[i];
  is_accel = (g_sensors[i].kind == CAL_KIND_ACCEL);
  is_mag   = (g_sensors[i].kind == CAL_KIND_MAG);

  /* Both kinds carry a validity flag now. The gyro used to have none, so its
   * stored offsets were applied whether or not anything had ever measured
   * them - which was harmless only because nothing ever wrote them.
   */

  snprintf(name, sizeof(name), "%s_OK", pfx);

  if (param_i32(name) != 1)
    {
      return;                    /* never calibrated: pass raw through */
    }

  for (k = 0; k < 3; k++)
    {
      const char axis[3] = { 'X', 'Y', 'Z' };

      snprintf(name, sizeof(name), "%s_%cOFF", pfx, axis[k]);
      a->off[k] = param_f32(name);

      if (is_accel)
        {
          snprintf(name, sizeof(name), "%s_%cSCL", pfx, axis[k]);
          a->scl[k] = param_f32(name);
        }
    }

  if (is_mag)
    {
      a->matrix[0][0] = param_f32("CAL_MAG0_XX");
      a->matrix[1][1] = param_f32("CAL_MAG0_YY");
      a->matrix[2][2] = param_f32("CAL_MAG0_ZZ");
      a->matrix[0][1] = a->matrix[1][0] = param_f32("CAL_MAG0_XY");
      a->matrix[0][2] = a->matrix[2][0] = param_f32("CAL_MAG0_XZ");
      a->matrix[1][2] = a->matrix[2][1] = param_f32("CAL_MAG0_YZ");
      a->full_matrix = true;
    }

  a->on = true;
}

static void cal_apply_mag_candidate(FAR const struct cal_mag_fit_s *fit,
                                    FAR struct cal_apply_s *apply)
{
  int row;

  memset(apply, 0, sizeof(*apply));

  for (row = 0; row < 3; row++)
    {
      apply->off[row] = fit->offset[row];
      memcpy(apply->matrix[row], fit->matrix[row],
             sizeof(apply->matrix[row]));
    }

  apply->on = true;
  apply->full_matrix = true;
}

/* Strictly parse the ten finite host-fit values. strtof also accepts nan and
 * inf, so numerical safety remains the independent validator's job. */

static bool cal_parse_mag_candidate(FAR const char *text,
                                    FAR struct cal_mag_fit_s *fit)
{
  float value[10];
  FAR char *end;
  int k;

  for (k = 0; k < 10; k++)
    {
      while (*text == ' ' || *text == '\t')
        {
          text++;
        }

      if (*text == '\0')
        {
          return false;
        }

      errno = 0;
      value[k] = strtof(text, &end);

      if (end == text || errno == ERANGE)
        {
          return false;
        }

      text = end;
    }

  while (*text == ' ' || *text == '\t')
    {
      text++;
    }

  if (*text != '\0')
    {
      return false;
    }

  memset(fit, 0, sizeof(*fit));
  memcpy(fit->offset, value, 3 * sizeof(float));
  fit->matrix[0][0] = value[3];
  fit->matrix[1][1] = value[4];
  fit->matrix[2][2] = value[5];
  fit->matrix[0][1] = fit->matrix[1][0] = value[6];
  fit->matrix[0][2] = fit->matrix[2][0] = value[7];
  fit->matrix[1][2] = fit->matrix[2][1] = value[8];
  fit->field = value[9];
  return true;
}

/* Index of a sensor by name, or -1. */

static int cal_find(FAR const char *name)
{
  int i;

  for (i = 0; i < CAL_NSENSORS; i++)
    {
      if (strcmp(name, g_sensors[i].name) == 0)
        {
          return i;
        }
    }

  return -1;
}

static void cal_align_stop(void)
{
  int i;

  for (i = 0; i < g_align_n; i++)
    {
      if (g_align[i].sub >= 0)
        {
          orb_unsubscribe(g_align[i].sub);
        }

      g_align[i].sub = -1;
      g_align[i].idx = -1;
    }

  g_align_n = 0;
}

/* Emit one alignment frame. Same wire format as the single-sensor stream -
 * the host decoder already routes on the sensor id byte - but always float
 * encoded, so the host receives exactly what the driver produced.
 */

static int cal_align_flush(int fd, FAR struct cal_align_s *a,
                           FAR uint8_t *frame)
{
  FAR const struct cal_sensor_s *s = &g_sensors[a->idx];
  size_t nv = (size_t)a->nbatch * s->nvalues;
  size_t bytes = nv * sizeof(float);
  size_t off;
  uint16_t len;
  uint16_t crc;

  memcpy(frame + 14, a->acc, bytes);
  len = (uint16_t)(11 + bytes);

  frame[0] = CAL_SYNC;
  frame[1] = (uint8_t)(len & 0xff);
  frame[2] = (uint8_t)(len >> 8);
  frame[3] = (uint8_t)a->idx;
  frame[4] = a->seq++;
  memcpy(frame + 5, &a->batch_t0, 4);
  frame[9]  = (uint8_t)(g_align_dt_us & 0xff);
  frame[10] = (uint8_t)(g_align_dt_us >> 8);
  frame[11] = a->nbatch;
  frame[12] = s->nvalues;
  frame[13] = CAL_ENC_F32;

  off = 14 + bytes;
  crc = cal_crc16(frame + 1, off - 1);
  frame[off]     = (uint8_t)(crc & 0xff);
  frame[off + 1] = (uint8_t)(crc >> 8);

  a->nbatch = 0;
  return cal_write(fd, frame, off + 2);
}

/* Drain what one alignment stream has ready, flushing when the batch is full
 * or has waited long enough.
 */

static int cal_align_service(int fd, FAR struct cal_align_s *a,
                             FAR uint8_t *frame)
{
  if (a->idx < 0 || a->sub < 0)
    {
      return OK;
    }

  while (a->nbatch < CAL_BATCH_MAX)
    {
      uint32_t t_us = 0;
      FAR float *slot = &a->acc[(size_t)a->nbatch *
                                g_sensors[a->idx].nvalues];

      if (cal_read_values(a->idx, a->sub, slot, &t_us) != OK)
        {
          break;
        }

      if (a->nbatch == 0)
        {
          a->batch_t0 = t_us;
          a->batch_since = cal_now_us();
        }

      a->nbatch++;
    }

  if (a->nbatch > 0 &&
      (a->nbatch >= CAL_BATCH_MAX || cal_now_us() - a->batch_since >= 20000))
    {
      return cal_align_flush(fd, a, frame);
    }

  return OK;
}

/* Average a stretch of samples and say whether the board was actually still.
 *
 * Stillness is judged by the standard deviation over the window, not by
 * whether any single sample strayed: at +/-16 g the accelerometer's own noise
 * is around 0.02 m/s^2 RMS, so any excursion threshold tight enough to catch
 * real motion is tripped constantly by noise alone.
 *
 * Blocks for up to `ms`. That is deliberate - the operator is holding a board
 * still and there is nothing else for the session to do.
 */

static int cal_capture_still(int i, int sub, int ms, FAR float mean[3],
                             FAR float sd[3])
{
  static float buf[256 * 3];
  const int want = 200;
  int n = 0;
  uint64_t deadline = cal_now_us() + (uint64_t)ms * 1000;

  while (n < want && cal_now_us() < deadline)
    {
      float v[CAL_MAX_VALUES];
      uint32_t t;

      if (cal_read_values(i, sub, v, &t) == OK)
        {
          buf[n * 3 + 0] = v[0];
          buf[n * 3 + 1] = v[1];
          buf[n * 3 + 2] = v[2];
          n++;
        }
      else
        {
          usleep(1000);
        }
    }

  if (n < want / 2)
    {
      return -ETIMEDOUT;         /* sensor is not producing */
    }

  cal_stats(buf, n, 3, mean, sd);
  return n;
}

/* Average a gyro for the whole window rather than for a fixed sample count.
 *
 * The accel capture above stops at 200 samples, which at 500 Hz is 0.4 s. That
 * is ample against gravity, and short for a bias: see the arithmetic in
 * cal_gyro.h, where 0.4 s leaves ten times more white noise in the estimate
 * than the bias instability it is trying to measure. Running the clock out
 * instead needs no buffer, since cal_bias_s keeps sums.
 *
 * Returns the sample count, or a negative errno.
 */

static int cal_capture_bias(int i, int sub, int ms, FAR float mean[3],
                            FAR float sd[3])
{
  struct cal_bias_s acc;
  uint64_t deadline = cal_now_us() + (uint64_t)ms * 1000;

  cal_bias_reset(&acc);

  while (cal_now_us() < deadline)
    {
      float v[CAL_MAX_VALUES];
      uint32_t t;

      if (cal_read_values(i, sub, v, &t) == OK)
        {
          cal_bias_add(&acc, v);
        }
      else
        {
          usleep(1000);
        }
    }

  if (cal_bias_result(&acc, mean, sd) == 0)
    {
      return -ETIMEDOUT;         /* sensor is not producing */
    }

  return acc.n;
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
  char line[256];
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

  /* Six-position accelerometer calibration, and whether the stream should send
   * corrected values. Streaming calibrated is how you confirm a calibration
   * actually took: a corrected accel reads 9.81 in every orientation.
   */

  struct cal_accel_s cal6;
  struct cal_apply_s apply;
  int  cal6_idx = -1;                /* which sensor is being calibrated */
  bool want_cal = false;

  /* The host owns the large sample cloud and solver. The board retains only a
   * validated candidate for live preview; nothing is persistent until commit.
   */

  struct cal_mag_fit_s mag_stage;
  int  mag_idx = -1;
  bool mag_stage_valid = false;

  cal_accel_reset(&cal6);
  memset(&mag_stage, 0, sizeof(mag_stage));
  cal_load_apply(-1, false, &apply);

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
      struct pollfd pfd[2 + CAL_ALIGN_MAX];
      nfds_t nfds = 1;
      char ch;
      int ai;

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

      /* Alignment streams wake the loop the same way, so their rate is the
       * sensors' rather than the loop's.
       */

      for (ai = 0; ai < g_align_n; ai++)
        {
          if (g_align[ai].sub >= 0)
            {
              pfd[nfds].fd     = g_align[ai].sub;
              pfd[nfds].events = POLLIN;
              nfds++;
            }
        }

      if (poll(pfd, nfds, 200) < 0 && errno != EINTR)
        {
          break;
        }

      for (ai = 0; ai < g_align_n; ai++)
        {
          if (cal_align_service(fd, &g_align[ai], frame) < 0)
            {
              cal_align_stop();
              break;
            }
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

              if (apply.on)
                {
                  int k;

                  if (apply.full_matrix)
                    {
                      float raw[3] = {slot[0], slot[1], slot[2]};

                      for (k = 0; k < 3; k++)
                        {
                          slot[k] =
                            apply.matrix[k][0] * (raw[0] - apply.off[0]) +
                            apply.matrix[k][1] * (raw[1] - apply.off[1]) +
                            apply.matrix[k][2] * (raw[2] - apply.off[2]);
                        }
                    }
                  else
                    {
                      for (k = 0; k < 3; k++)
                        {
                          slot[k] =
                            (slot[k] - apply.off[k]) * apply.scl[k];
                        }
                    }
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

                  /* lroundf, not lrintf: this tree declares lrintf in
                   * math.h but does not provide it in libm, so lrintf links
                   * only by accident of nothing calling it. The difference -
                   * half-away-from-zero versus the current rounding mode - is
                   * immaterial when quantising to the sensor's own LSB.
                   */

                  p[k] = q >= 32767.0f ? 32767 :
                         q <= -32768.0f ? -32768 : (int16_t)lroundf(q);
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

          st_sub = cal_subscribe(i);

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
          cal_load_apply(i, want_cal, &apply);

          if (mag_stage_valid && i == mag_idx)
            {
              cal_apply_mag_candidate(&mag_stage, &apply);
            }

          cal_emit(fd,
                   "{\"evt\":\"ok\",\"what\":\"stream\",\"name\":\"%s\","
                   "\"id\":%d,\"hz\":%ld,\"cal\":%s}\n",
                   g_sensors[i].name, i, hz, apply.on ? "true" : "false");
        }
      else if (strncmp(line, "calib ", 6) == 0)
        {
          want_cal = strcmp(line + 6, "on") == 0;

          if (st_idx >= 0)
            {
              cal_load_apply(st_idx, want_cal, &apply);

              if (mag_stage_valid && st_idx == mag_idx)
                {
                  cal_apply_mag_candidate(&mag_stage, &apply);
                }
            }

          cal_emit(fd, "{\"evt\":\"ok\",\"what\":\"calib\",\"on\":%s}\n",
                   apply.on ? "true" : "false");
        }
      /* `align <hz> <name> [<name> ...]` - stream several sensors at once on
       * a shared timebase, raw. `align stop` ends it.
       */

      else if (strncmp(line, "align", 5) == 0 &&
               (line[5] == '\0' || line[5] == ' '))
        {
          FAR char *arg = line[5] == ' ' ? line + 6 : NULL;
          FAR char *save = NULL;
          FAR char *tok;
          long hz;
          int started = 0;
          int slot[CAL_ALIGN_MAX];
          int n = 0;
          int k;

          if (arg == NULL || strcmp(arg, "stop") == 0)
            {
              cal_align_stop();
              cal_emit(fd, "{\"evt\":\"ok\",\"what\":\"align stop\"}\n");
              continue;
            }

          tok = strtok_r(arg, " ", &save);
          hz = tok != NULL ? strtol(tok, NULL, 10) : 0;

          if (hz < 1 || hz > 400)
            {
              cal_emit(fd, "{\"evt\":\"error\","
                           "\"msg\":\"rate must be 1-400\"}\n");
              continue;
            }

          /* Resolve every name BEFORE subscribing to any of them. A partially
           * started set would leave the host solving against a sensor that is
           * silently absent, which is worse than refusing outright.
           */

          while ((tok = strtok_r(NULL, " ", &save)) != NULL)
            {
              int idx = cal_find(tok);

              if (idx < 0)
                {
                  cal_emit(fd, "{\"evt\":\"error\",\"msg\":\"no such "
                               "sensor\",\"name\":\"%s\"}\n", tok);
                  n = -1;
                  break;
                }

              if (n >= CAL_ALIGN_MAX)
                {
                  cal_emit(fd, "{\"evt\":\"error\",\"msg\":\"at most %d "
                               "sensors at once\"}\n", CAL_ALIGN_MAX);
                  n = -1;
                  break;
                }

              slot[n++] = idx;
            }

          if (n <= 0)
            {
              if (n == 0)
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"msg\":\"no sensors named\"}\n");
                }

              continue;
            }

          cal_align_stop();
          g_align_dt_us = (uint32_t)(1000000 / hz);

          for (k = 0; k < n; k++)
            {
              int sub = cal_subscribe(slot[k]);

              if (sub < 0)
                {
                  cal_emit(fd, "{\"evt\":\"error\",\"msg\":\"sensor not "
                               "available\",\"name\":\"%s\"}\n",
                           g_sensors[slot[k]].name);
                  cal_align_stop();
                  started = -1;
                  break;
                }

              orb_set_interval(sub, g_align_dt_us);
              g_align[g_align_n].idx = slot[k];
              g_align[g_align_n].sub = sub;
              g_align[g_align_n].seq = 0;
              g_align[g_align_n].nbatch = 0;
              g_align_n++;
              started++;
            }

          if (started < 0)
            {
              continue;
            }

          cal_emit(fd, "{\"evt\":\"ok\",\"what\":\"align\",\"n\":%d,"
                       "\"hz\":%ld}\n", started, hz);
        }

      /* `still <name>` - a stillness-checked average of one sensor.
       *
       * The deviation is REPORTED, not judged here. Alignment tolerates far
       * more movement than a calibration does - it only has to pick among 24
       * discrete rotations - so cal6's threshold would reject positions
       * alignment is perfectly happy with.
       */

      else if (strncmp(line, "still ", 6) == 0)
        {
          float mean[3];
          float sd[3];
          int idx = cal_find(line + 6);
          int sub;
          int n;

          if (idx < 0)
            {
              cal_emit(fd, "{\"evt\":\"error\","
                           "\"msg\":\"no such sensor\"}\n");
              continue;
            }

          sub = cal_subscribe(idx);

          if (sub < 0)
            {
              cal_emit(fd, "{\"evt\":\"error\","
                           "\"msg\":\"sensor not available\"}\n");
              continue;
            }

          orb_set_interval(sub, 2000);         /* 500 Hz is ample */
          n = cal_capture_still(idx, sub, 4000, mean, sd);
          orb_unsubscribe(sub);

          if (n < 0)
            {
              cal_emit(fd, "{\"evt\":\"error\","
                           "\"msg\":\"sensor produced nothing\"}\n");
              continue;
            }

          cal_emit(fd, "{\"evt\":\"still\",\"sensor\":\"%s\","
                       "\"mean\":[%.5f,%.5f,%.5f],"
                       "\"sd\":[%.5f,%.5f,%.5f],\"n\":%d}\n",
                   g_sensors[idx].name,
                   (double)mean[0], (double)mean[1], (double)mean[2],
                   (double)sd[0], (double)sd[1], (double)sd[2], n);
        }

      else if (strncmp(line, "record ", 7) == 0)
        {
#ifdef CONFIG_XXCAR_LOGGER
          FAR char *what = line + 7;

          if (strncmp(what, "start", 5) == 0)
            {
              struct logger_status_s ls;
              long hz = 200;

              if (what[5] == ' ')
                {
                  hz = strtol(what + 6, NULL, 10);
                }

              if (hz < 0 || hz > 2000)
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"msg\":\"rate must be 0-2000\"}\n");
                  continue;
                }

              /* Allan variance wants both IMUs, raw, for hours - but NOT
               * necessarily at the native rate, and defaulting to it was a
               * trap.
               *
               * Both IMUs at 2 kHz is 227 KB/s, which reaches FAT32's 4 GB
               * per-file ceiling in 5.1 hours: an overnight run would stop
               * before morning with no warning. It is also only 16 ms ahead of
               * the uORB queue, so an SD housekeeping stall punches a hole -
               * and a gap corrupts the long-tau end, which is the part being
               * measured.
               *
               * 200 Hz is the useful long run: 52 hours of headroom on the
               * file size, ten times the stall tolerance, and it still reaches
               * the bias-instability knee. Full rate is for a ~20 minute run
               * to characterise the white-noise region, where a stall costs
               * little. Hence the default, with 0 available for native when
               * that is what is wanted.
               */

              param_set_i32("LOG_IMU0", 1);
              param_set_i32("LOG_IMU1", 1);
              param_set_i32("LOG_RATE", (int32_t)hz);

              if (logger_is_running())
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"msg\":\"already recording\"}\n");
                  continue;
                }

              if (logger_start() < 0)
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"msg\":\"logger would not start\"}\n");
                  continue;
                }

              logger_get_status(&ls);
              cal_emit(fd,
                       "{\"evt\":\"ok\",\"what\":\"record\","
                       "\"path\":\"%s\",\"topics\":%" PRIu32 ","
                       "\"hz\":%ld}\n",
                       ls.path, ls.topics, hz);
            }
          else if (strcmp(what, "stop") == 0)
            {
              struct logger_status_s ls;

              logger_get_status(&ls);
              logger_stop();
              cal_emit(fd,
                       "{\"evt\":\"ok\",\"what\":\"record stop\","
                       "\"path\":\"%s\",\"samples\":%" PRIu32 ","
                       "\"bytes\":%" PRIu64 ",\"dropped\":%" PRIu32 "}\n",
                       ls.path, ls.samples, ls.bytes, ls.dropped);
            }
          else
            {
              struct logger_status_s ls;

              logger_get_status(&ls);
              cal_emit(fd,
                       "{\"evt\":\"record\",\"running\":%s,\"path\":\"%s\","
                       "\"samples\":%" PRIu32 ",\"bytes\":%" PRIu64 ","
                       "\"dropped\":%" PRIu32 "}\n",
                       ls.running ? "true" : "false", ls.path,
                       ls.samples, ls.bytes, ls.dropped);
            }
#else
          cal_emit(fd, "{\"evt\":\"error\","
                       "\"msg\":\"logger not built in\"}\n");
#endif
        }
        else if (strncmp(line, "cal6", 4) == 0)
        {
          FAR char *what = line[4] == ' ' ? line + 5 : (FAR char *)"";

          if (strncmp(what, "start ", 6) == 0)
            {
              int i;

              for (i = 0; i < CAL_NSENSORS; i++)
                {
                  if (strcmp(what + 6, g_sensors[i].name) == 0 &&
                      g_sensors[i].kind == CAL_KIND_ACCEL)
                    {
                      break;
                    }
                }

              if (i == CAL_NSENSORS)
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"msg\":\"not an accelerometer\"}\n");
                  continue;
                }

              cal_accel_reset(&cal6);
              cal6_idx = i;
              cal_emit(fd,
                       "{\"evt\":\"ok\",\"what\":\"cal6 start\","
                       "\"name\":\"%s\",\"need\":%d}\n",
                       g_sensors[i].name, CAL_NPOS);
            }
          else if (strcmp(what, "capture") == 0)
            {
              float mean[3];
              float sd[3];
              int sub;
              int n;
              int pos;

              if (cal6_idx < 0)
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"msg\":\"cal6 not started\"}\n");
                  continue;
                }

              sub = cal_subscribe(cal6_idx);

              if (sub < 0)
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"msg\":\"sensor not available\"}\n");
                  continue;
                }

              orb_set_interval(sub, 2000);         /* 500 Hz is ample */
              n = cal_capture_still(cal6_idx, sub, 4000, mean, sd);
              orb_unsubscribe(sub);

              if (n < 0)
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"msg\":\"sensor produced nothing\"}\n");
                  continue;
                }

              /* 0.08 m/s^2 is several times the sensor's own noise and well
               * under what a hand resting on the bench produces.
               */

              if (sd[0] > 0.08f || sd[1] > 0.08f || sd[2] > 0.08f)
                {
                  cal_emit(fd,
                           "{\"evt\":\"error\",\"msg\":\"not steady\","
                           "\"sd\":[%.4f,%.4f,%.4f]}\n",
                           (double)sd[0], (double)sd[1], (double)sd[2]);
                  continue;
                }

              pos = cal_accel_add(&cal6, mean);

              if (pos < 0)
                {
                  cal_emit(fd,
                           "{\"evt\":\"error\","
                           "\"msg\":\"not square to an axis\","
                           "\"a\":[%.3f,%.3f,%.3f]}\n",
                           (double)mean[0], (double)mean[1], (double)mean[2]);
                  continue;
                }

              cal_emit(fd,
                       "{\"evt\":\"cal6\",\"pos\":%d,\"have\":%d,"
                       "\"need\":%d,\"a\":[%.4f,%.4f,%.4f],\"n\":%d}\n",
                       pos, cal_accel_count(&cal6), CAL_NPOS,
                       (double)mean[0], (double)mean[1], (double)mean[2], n);
            }
          else if (strcmp(what, "save") == 0)
            {
              float off[3];
              float scl[3];
              float res = 0.0f;
              char nm[PARAM_NAME_MAX + 1];
              const char axis[3] = { 'X', 'Y', 'Z' };
              FAR const char *pfx;
              int k;

              int bad = 0;

              if (cal6_idx < 0 || cal_accel_solve(&cal6, off, scl, &res) != 0)
                {
                  cal_emit(fd,
                           "{\"evt\":\"error\",\"msg\":\"need all six\","
                           "\"have\":%d}\n", cal_accel_count(&cal6));
                  continue;
                }

              /* Judge the fit before storing it. The residual was already
               * being computed and shown; without a threshold it was decoration
               * on a result that gets applied either way.
               *
               * Written as !(res <= limit) rather than (res > limit) so a NaN
               * residual - every comparison with which is false - is refused
               * instead of accepted. That is the same trap that once let a NaN
               * through param_set_f32().
               */

              if (!(res <= CAL_RESIDUAL_MAX))
                {
                  cal_emit(fd,
                           "{\"evt\":\"error\",\"msg\":\"fit rejected\","
                           "\"residual\":%.4f,\"limit\":%.4f}\n",
                           (double)res, (double)CAL_RESIDUAL_MAX);
                  continue;
                }

              pfx = g_cal_prefix[cal6_idx];

              /* Clear the validity flag FIRST. If a set below fails we stop
               * with the live offsets half-updated, and a stale OK=1 from a
               * previous good calibration would make that mixture look
               * measured. Nothing is written to the card unless every step
               * here succeeds, so params.txt still holds the old set.
               */

              snprintf(nm, sizeof(nm), "%s_OK", pfx);
              if (param_set_i32(nm, 0) < 0)
                {
                  cal_emit(fd, "{\"evt\":\"error\",\"msg\":\"cannot clear\","
                               "\"param\":\"%s\"}\n", nm);
                  continue;
                }

              /* param_set_f32() returns -ERANGE when it CLAMPS as well as when
               * it refuses, so ignoring it meant a scale outside 0.8-1.2 was
               * silently stored as the bound while the GUI displayed the value
               * the solver produced. Report the parameter and the value that
               * would not fit rather than pretending either number is real.
               */

              for (k = 0; k < 3 && bad == 0; k++)
                {
                  snprintf(nm, sizeof(nm), "%s_%cOFF", pfx, axis[k]);
                  if (param_set_f32(nm, off[k]) < 0)
                    {
                      cal_emit(fd,
                               "{\"evt\":\"error\",\"msg\":\"out of range\","
                               "\"param\":\"%s\",\"value\":%.5f}\n",
                               nm, (double)off[k]);
                      bad = 1;
                      break;
                    }

                  snprintf(nm, sizeof(nm), "%s_%cSCL", pfx, axis[k]);
                  if (param_set_f32(nm, scl[k]) < 0)
                    {
                      cal_emit(fd,
                               "{\"evt\":\"error\",\"msg\":\"out of range\","
                               "\"param\":\"%s\",\"value\":%.5f}\n",
                               nm, (double)scl[k]);
                      bad = 1;
                      break;
                    }
                }

              if (bad != 0)
                {
                  continue;
                }

              snprintf(nm, sizeof(nm), "%s_OK", pfx);
              if (param_set_i32(nm, 1) < 0)
                {
                  cal_emit(fd, "{\"evt\":\"error\",\"msg\":\"cannot mark ok\","
                               "\"param\":\"%s\"}\n", nm);
                  continue;
                }

              /* One write, at the end. The USB port dies on cable pull, and a
               * half-written calibration is worse than none.
               */

              if (param_save() < 0)
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"msg\":\"param save failed\"}\n");
                  continue;
                }

              if (st_idx == cal6_idx)
                {
                  cal_load_apply(st_idx, want_cal, &apply);
                }

              cal_emit(fd,
                       "{\"evt\":\"ok\",\"what\":\"cal6 save\","
                       "\"off\":[%.5f,%.5f,%.5f],\"scl\":[%.5f,%.5f,%.5f],"
                       "\"residual\":%.4f}\n",
                       (double)off[0], (double)off[1], (double)off[2],
                       (double)scl[0], (double)scl[1], (double)scl[2],
                       (double)res);
            }
          else
            {
              cal_accel_reset(&cal6);
              cal6_idx = -1;
              cal_emit(fd, "{\"evt\":\"ok\",\"what\":\"cal6 abort\"}\n");
            }
        }
      else if (strncmp(line, "mag", 3) == 0)
        {
          FAR const char *what = line[3] == ' ' ? line + 4 : "";

          if (strncmp(what, "stage ", 6) == 0)
            {
              enum cal_mag_result_e result;
              struct cal_mag_fit_s candidate;

              if (st_idx < 0 || st_sub < 0 ||
                  g_sensors[st_idx].kind != CAL_KIND_MAG)
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"what\":\"mag stage\","
                               "\"msg\":\"stream magnetometer first\"}\n");
                  continue;
                }

              if (!cal_parse_mag_candidate(what + 6, &candidate))
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"what\":\"mag stage\","
                               "\"msg\":\"expected ten numeric values\"}\n");
                  continue;
                }

              result = cal_mag_validate(&candidate);

              if (result != CAL_MAG_OK)
                {
                  cal_emit(fd,
                           "{\"evt\":\"error\",\"what\":\"mag stage\","
                           "\"msg\":\"%s\",\"code\":%d}\n",
                           cal_mag_result_string(result), result);
                  continue;
                }

              mag_stage = candidate;
              mag_idx = st_idx;
              mag_stage_valid = true;
              cal_apply_mag_candidate(&mag_stage, &apply);
              cal_emit(fd,
                       "{\"evt\":\"ok\",\"what\":\"mag stage\","
                       "\"field\":%.6f}\n", (double)mag_stage.field);
            }
          else if (strcmp(what, "commit") == 0)
            {
              static const char *const off_name[3] =
              {
                "CAL_MAG0_XOFF", "CAL_MAG0_YOFF", "CAL_MAG0_ZOFF"
              };
              static const char *const matrix_name[6] =
              {
                "CAL_MAG0_XX", "CAL_MAG0_YY", "CAL_MAG0_ZZ",
                "CAL_MAG0_XY", "CAL_MAG0_XZ", "CAL_MAG0_YZ"
              };
              float matrix_value[6];
              bool failed = false;
              int k;

              if (!mag_stage_valid || mag_idx < 0)
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"what\":\"mag commit\","
                               "\"msg\":\"stage and validate first\"}\n");
                  continue;
                }

              matrix_value[0] = mag_stage.matrix[0][0];
              matrix_value[1] = mag_stage.matrix[1][1];
              matrix_value[2] = mag_stage.matrix[2][2];
              matrix_value[3] = mag_stage.matrix[0][1];
              matrix_value[4] = mag_stage.matrix[0][2];
              matrix_value[5] = mag_stage.matrix[1][2];

              if (param_set_i32("CAL_MAG0_OK", 0) < 0)
                {
                  failed = true;
                }

              for (k = 0; k < 3 && !failed; k++)
                {
                  if (param_set_f32(off_name[k], mag_stage.offset[k]) < 0)
                    {
                      failed = true;
                    }
                }

              for (k = 0; k < 6 && !failed; k++)
                {
                  if (param_set_f32(matrix_name[k], matrix_value[k]) < 0)
                    {
                      failed = true;
                    }
                }

              if (!failed &&
                  param_set_f32("CAL_MAG0_FIELD", mag_stage.field) < 0)
                {
                  failed = true;
                }

              if (!failed && param_set_i32("CAL_MAG0_OK", 1) < 0)
                {
                  failed = true;
                }

              if (failed)
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"msg\":\"mag parameter out of range\"}\n");
                  continue;
                }

              if (param_save() < 0)
                {
                  cal_emit(fd, "{\"evt\":\"error\","
                               "\"msg\":\"param save failed\"}\n");
                  continue;
                }

              want_cal = true;
              cal_load_apply(st_idx, true, &apply);

              cal_emit(fd,
                       "{\"evt\":\"ok\",\"what\":\"mag commit\","
                       "\"field\":%.6f}\n", (double)mag_stage.field);
              mag_stage_valid = false;
            }
          else if (strcmp(what, "abort") == 0)
            {
              memset(&mag_stage, 0, sizeof(mag_stage));
              mag_idx = -1;
              mag_stage_valid = false;
              cal_load_apply(st_idx, want_cal, &apply);
              cal_emit(fd,
                       "{\"evt\":\"ok\",\"what\":\"mag abort\"}\n");
            }
          else
            {
              cal_emit(fd, "{\"evt\":\"error\","
                           "\"msg\":\"use mag stage/commit/abort\"}\n");
            }
        }
      else if (strncmp(line, "gyro ", 5) == 0)
        {
          /* Zero-rate bias: hold the board still, average, store. One shot -
           * there is nothing for the operator to reposition between, which is
           * the whole difference from the six-position accel procedure.
           */

          char nm[PARAM_NAME_MAX + 1];
          const char axis[3] = { 'X', 'Y', 'Z' };
          float mean[3];
          float sd[3];
          FAR const char *pfx;
          int bad = 0;
          int sub;
          int n;
          int i;
          int k;

          for (i = 0; i < CAL_NSENSORS; i++)
            {
              if (strcmp(line + 5, g_sensors[i].name) == 0 &&
                  g_sensors[i].kind == CAL_KIND_GYRO)
                {
                  break;
                }
            }

          if (i == CAL_NSENSORS)
            {
              cal_emit(fd, "{\"evt\":\"error\","
                           "\"msg\":\"not a gyroscope\"}\n");
              continue;
            }

          sub = cal_subscribe(i);

          if (sub < 0)
            {
              cal_emit(fd, "{\"evt\":\"error\","
                           "\"msg\":\"sensor not available\"}\n");
              continue;
            }

          cal_emit(fd, "{\"evt\":\"ok\",\"what\":\"gyro start\","
                       "\"name\":\"%s\",\"secs\":4}\n", g_sensors[i].name);

          orb_set_interval(sub, 2000);           /* 500 Hz is ample */
          n = cal_capture_bias(i, sub, 4000, mean, sd);
          orb_unsubscribe(sub);

          if (n < 0)
            {
              cal_emit(fd, "{\"evt\":\"error\","
                           "\"msg\":\"sensor produced nothing\"}\n");
              continue;
            }

          switch (cal_gyro_judge(n, mean, sd))
            {
              case CAL_GYRO_TOO_FEW:
                cal_emit(fd,
                         "{\"evt\":\"error\",\"msg\":\"too few samples\","
                         "\"n\":%d,\"need\":%d}\n", n, CAL_GYRO_MIN_N);
                continue;

              case CAL_GYRO_NOT_STILL:
                cal_emit(fd,
                         "{\"evt\":\"error\",\"msg\":\"not steady\","
                         "\"sd\":[%.5f,%.5f,%.5f],\"limit\":%.5f}\n",
                         (double)sd[0], (double)sd[1], (double)sd[2],
                         (double)CAL_GYRO_SD_MAX);
                continue;

              case CAL_GYRO_TOO_LARGE:

                /* Steady but turning: a constant rotation has zero standard
                 * deviation and would otherwise be stored as bias.
                 */

                cal_emit(fd,
                         "{\"evt\":\"error\",\"msg\":\"still turning\","
                         "\"bias\":[%.5f,%.5f,%.5f],\"limit\":%.4f}\n",
                         (double)mean[0], (double)mean[1], (double)mean[2],
                         (double)CAL_GYRO_BIAS_MAX);
                continue;

              default:
                break;
            }

          pfx = g_cal_prefix[i];

          /* Same order as the accel save: invalidate first, so a failure
           * partway cannot leave new offsets under an old OK=1.
           */

          snprintf(nm, sizeof(nm), "%s_OK", pfx);
          if (param_set_i32(nm, 0) < 0)
            {
              cal_emit(fd, "{\"evt\":\"error\",\"msg\":\"cannot clear\","
                           "\"param\":\"%s\"}\n", nm);
              continue;
            }

          for (k = 0; k < 3; k++)
            {
              snprintf(nm, sizeof(nm), "%s_%cOFF", pfx, axis[k]);
              if (param_set_f32(nm, mean[k]) < 0)
                {
                  cal_emit(fd,
                           "{\"evt\":\"error\",\"msg\":\"out of range\","
                           "\"param\":\"%s\",\"value\":%.5f}\n",
                           nm, (double)mean[k]);
                  bad = 1;
                  break;
                }
            }

          if (bad != 0)
            {
              continue;
            }

          snprintf(nm, sizeof(nm), "%s_OK", pfx);
          if (param_set_i32(nm, 1) < 0)
            {
              cal_emit(fd, "{\"evt\":\"error\",\"msg\":\"cannot mark ok\","
                           "\"param\":\"%s\"}\n", nm);
              continue;
            }

          if (param_save() < 0)
            {
              cal_emit(fd, "{\"evt\":\"error\","
                           "\"msg\":\"param save failed\"}\n");
              continue;
            }

          if (st_idx == i)
            {
              cal_load_apply(st_idx, want_cal, &apply);
            }

          cal_emit(fd,
                   "{\"evt\":\"ok\",\"what\":\"gyro save\",\"name\":\"%s\","
                   "\"bias\":[%.6f,%.6f,%.6f],\"sd\":[%.6f,%.6f,%.6f],"
                   "\"n\":%d}\n",
                   g_sensors[i].name,
                   (double)mean[0], (double)mean[1], (double)mean[2],
                   (double)sd[0], (double)sd[1], (double)sd[2], n);
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
