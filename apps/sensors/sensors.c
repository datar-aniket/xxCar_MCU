/****************************************************************************
 * apps/sensors/sensors.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See sensors.h.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <sched.h>
#include <pthread.h>
#include <syslog.h>
#include <time.h>
#include <math.h>

#include <uORB/uORB.h>
#include <nuttx/uorb.h>

#include "sensors.h"
#include "rotation.h"
#include "dsp_filter.h"
#include "../param/param.h"
#include "../uorb_msgs/uorb_msgs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SENSORS_STACK  2048

/* Above the logger, mavlink, rc and px4io, below the sensor drivers.
 *
 * This task sits between the drivers and everything that will consume an
 * attitude estimate, so a lower priority would simply move the delay the
 * driver priority fix removed one step downstream. It stays below
 * FMUV6C_SENSOR_PRIO (150) because a driver that cannot drain its hardware
 * FIFO loses samples outright, and no amount of promptness here recovers that.
 *
 *   224  HPWORK
 *   150  sensor drivers
 *   120  sensors        <- this
 *   110  px4io
 *   105  rc
 *   104  mavlink
 *   102  logger
 *   100  NSH, LPWORK
 */

#define SENSORS_PRIO   (SCHED_PRIORITY_DEFAULT + 20)

/* One poll wakeup can stand for several queued samples at 2 kHz. Bounded so a
 * single topic cannot monopolise the loop if its producer runs concurrently -
 * the same reasoning as the logger's drain bound.
 */

#define SENSORS_DRAIN_MAX  64
#define FILTER_NOMINAL_RATE_HZ 2000.0f
#define FILTER_RATE_SAMPLES    512
#define FILTER_GAP_US          2000
#define FILTER_RMS_SAMPLES     4096
#define FILTER_MEAN_ALPHA      (1.0f / 1024.0f)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A rotation reduced to what it does per axis: out[i] = sgn[i]*in[idx[i]].
 *
 * Composing the sensor and board rotations once at start, into this, keeps the
 * per-sample cost to three loads and three sign flips - and, more usefully,
 * makes the composed result something that can be named and printed rather
 * than two switch statements whose combined effect nobody can read off.
 */

struct axis_map_s
{
  int8_t idx[3];
  int8_t sgn[3];
};

struct filter_rate_s
{
  uint64_t first_timestamp;
  uint64_t last_timestamp;
  uint32_t samples;
  float    rate_hz;
  bool     locked;
};

/* AC RMS measurement with a slow DC tracker.  Unlike plain RMS this does not
 * report gravity as "noise" on the vertical accelerometer axis.  The mean is
 * retained between reporting windows; only the energy accumulator is reset.
 */

struct filter_rms_s
{
  float raw_mean[3];
  float filtered_mean[3];
  float raw_energy[3];
  float filtered_energy[3];
  uint32_t samples;
  bool initialized;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool   g_running;
static volatile bool   g_should_stop;
static struct sensors_status_s g_status;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t sensors_now_us(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

static bool vector_finite(FAR const float value[3])
{
  return isfinite(value[0]) && isfinite(value[1]) && isfinite(value[2]);
}

/* Returns 1 when a measured rate first becomes available, -1 for a timestamp
 * discontinuity, and zero otherwise.  Once locked, the measured rate stays
 * fixed until a discontinuity; this avoids repeatedly perturbing live biquad
 * state for the sub-percent wander already handled by the IMU timebase.
 */

static int filter_rate_update(FAR struct filter_rate_s *rate,
                              uint64_t timestamp)
{
  if (rate->last_timestamp != 0 &&
      (timestamp <= rate->last_timestamp ||
       timestamp - rate->last_timestamp > FILTER_GAP_US))
    {
      rate->first_timestamp = timestamp;
      rate->last_timestamp  = timestamp;
      rate->samples         = 1;
      rate->locked          = false;
      return -1;
    }

  if (rate->first_timestamp == 0)
    {
      rate->first_timestamp = timestamp;
      rate->last_timestamp  = timestamp;
      rate->samples         = 1;
      return 0;
    }

  rate->last_timestamp = timestamp;

  if (!rate->locked && ++rate->samples >= FILTER_RATE_SAMPLES)
    {
      uint64_t span = timestamp - rate->first_timestamp;

      if (span > 0)
        {
          rate->rate_hz = (float)(rate->samples - 1) * 1000000.0f /
                          (float)span;
          rate->locked = true;
          return 1;
        }
    }

  return 0;
}

static bool filter_rms_update(FAR struct filter_rms_s *stats,
                              FAR const float raw[3],
                              FAR const float filtered[3],
                              FAR float raw_rms[3],
                              FAR float filtered_rms[3])
{
  int axis;

  if (!stats->initialized)
    {
      memcpy(stats->raw_mean, raw, sizeof(stats->raw_mean));
      memcpy(stats->filtered_mean, filtered, sizeof(stats->filtered_mean));
      stats->initialized = true;
    }

  for (axis = 0; axis < 3; axis++)
    {
      float raw_ac;
      float filtered_ac;

      stats->raw_mean[axis] +=
        FILTER_MEAN_ALPHA * (raw[axis] - stats->raw_mean[axis]);
      stats->filtered_mean[axis] +=
        FILTER_MEAN_ALPHA * (filtered[axis] - stats->filtered_mean[axis]);
      raw_ac = raw[axis] - stats->raw_mean[axis];
      filtered_ac = filtered[axis] - stats->filtered_mean[axis];
      stats->raw_energy[axis] += raw_ac * raw_ac;
      stats->filtered_energy[axis] += filtered_ac * filtered_ac;
    }

  if (++stats->samples < FILTER_RMS_SAMPLES)
    {
      return false;
    }

  for (axis = 0; axis < 3; axis++)
    {
      raw_rms[axis] = sqrtf(stats->raw_energy[axis] /
                            (float)stats->samples);
      filtered_rms[axis] = sqrtf(stats->filtered_energy[axis] /
                                 (float)stats->samples);
      stats->raw_energy[axis] = 0.0f;
      stats->filtered_energy[axis] = 0.0f;
    }

  stats->samples = 0;
  return true;
}

/* Reduce a rotation to its per-axis form by rotating the basis vectors.
 * Returns false if the rotation is one this build cannot perform exactly.
 */

static bool map_of_rotation(uint8_t rot, FAR struct axis_map_s *m)
{
  int j;

  memset(m, 0, sizeof(*m));

  for (j = 0; j < 3; j++)
    {
      float v[3] =
      {
        0.0f, 0.0f, 0.0f
      };

      int i;

      v[j] = 1.0f;

      if (!rotation_apply(rot, v))
        {
          return false;
        }

      for (i = 0; i < 3; i++)
        {
          if (v[i] != 0.0f)
            {
              m->idx[i] = (int8_t)j;
              m->sgn[i] = v[i] > 0.0f ? 1 : -1;
            }
        }
    }

  return true;
}

static void map_apply(FAR const struct axis_map_s *m, FAR float v[3])
{
  float in[3];

  in[0] = v[0];
  in[1] = v[1];
  in[2] = v[2];

  v[0] = m->sgn[0] * in[m->idx[0]];
  v[1] = m->sgn[1] * in[m->idx[1]];
  v[2] = m->sgn[2] * in[m->idx[2]];
}

/* Compose sensor-then-board into one map, and find the enum value that names
 * the result.
 *
 * The 24 axis permutations are a group, so a composition of two supported
 * rotations is always another supported rotation - the search cannot fail
 * unless the table itself is wrong, which is worth finding out at start rather
 * than never. `named` is for the status line; the map is what does the work.
 */

static bool compose_rotations(uint8_t sensor_rot, uint8_t board_rot,
                              FAR struct axis_map_s *m, FAR uint8_t *named)
{
  struct axis_map_s s;
  struct axis_map_s b;
  int rot;
  int i;

  if (!map_of_rotation(sensor_rot, &s) || !map_of_rotation(board_rot, &b))
    {
      return false;
    }

  /* board(sensor(v)): the sensor's rotation first, because the calibration and
   * the sensor rotation both live in the chip's frame.
   */

  for (i = 0; i < 3; i++)
    {
      m->idx[i] = s.idx[b.idx[i]];
      m->sgn[i] = (int8_t)(b.sgn[i] * s.sgn[b.idx[i]]);
    }

  *named = 0xff;

  for (rot = 0; rot < ROTATION_MAX_SUPPORTED; rot++)
    {
      struct axis_map_s c;

      if (map_of_rotation((uint8_t)rot, &c) &&
          memcmp(&c, m, sizeof(c)) == 0)
        {
          *named = (uint8_t)rot;
          break;
        }
    }

  return true;
}

/* Read one calibration set. `have` says whether it was ever measured; when it
 * was not, the identity is loaded so the pipeline still runs and the published
 * message says calibrated=0 rather than going silent.
 */

static void load_cal(FAR const char *pfx, bool is_accel, FAR float off[3],
                     FAR float scl[3], FAR bool *have)
{
  const char axis[3] =
  {
    'X', 'Y', 'Z'
  };

  char name[PARAM_NAME_MAX + 1];
  int k;

  for (k = 0; k < 3; k++)
    {
      off[k] = 0.0f;
      scl[k] = 1.0f;
    }

  snprintf(name, sizeof(name), "%s_OK", pfx);
  *have = param_i32(name) == 1;

  if (!*have)
    {
      return;
    }

  for (k = 0; k < 3; k++)
    {
      snprintf(name, sizeof(name), "%s_%cOFF", pfx, axis[k]);
      off[k] = param_f32(name);

      if (is_accel)
        {
          snprintf(name, sizeof(name), "%s_%cSCL", pfx, axis[k]);
          scl[k] = param_f32(name);
        }
    }
}

static int sensors_daemon(int argc, FAR char *argv[])
{
  struct axis_map_s amap;
  struct axis_map_s gmap;
  struct pollfd pfd[2];
  struct sensor_accel araw;
  struct sensor_gyro graw;
  struct vehicle_accel_s aout;
  struct vehicle_gyro_s gout;
  FAR const struct orb_metadata *ameta;
  FAR const struct orb_metadata *gmeta;
  float aoff[3];
  float ascl[3];
  float goff[3];
  float gscl[3];
  float accel_lpf_hz;
  float gyro_lpf_hz;
  float gyro_notch_hz;
  float gyro_notch_bw_hz;
  bool  acal;
  bool  gcal;
  bool  accel_filter_started = false;
  bool  gyro_filter_started = false;
  uint8_t anamed;
  uint8_t gnamed;
  uint8_t board_rot;
  uint8_t sensor_rot;
  int32_t sel;
  int asub = -1;
  int gsub = -1;
  int apub = -1;
  int gpub = -1;
  struct dsp_biquad3_s accel_lpf;
  struct dsp_biquad3_s gyro_lpf;
  struct dsp_biquad3_s gyro_notch;
  struct filter_rate_s accel_rate;
  struct filter_rate_s gyro_rate;
  struct filter_rms_s accel_rms;
  struct filter_rms_s gyro_rms;

  memset(&accel_rate, 0, sizeof(accel_rate));
  memset(&gyro_rate, 0, sizeof(gyro_rate));
  memset(&accel_rms, 0, sizeof(accel_rms));
  memset(&gyro_rms, 0, sizeof(gyro_rms));

  sel = param_i32("SENS_IMU_SEL");
  if (sel < 0 || sel > 1)
    {
      sel = 0;
    }

  accel_lpf_hz     = param_f32("SENS_ACC_LPF");
  gyro_lpf_hz      = param_f32("SENS_GYR_LPF");
  gyro_notch_hz    = param_f32("SENS_GYR_NF_FRQ");
  gyro_notch_bw_hz = param_f32("SENS_GYR_NF_BW");

  if (!dsp_biquad3_lowpass(&accel_lpf, FILTER_NOMINAL_RATE_HZ,
                           accel_lpf_hz) ||
      !dsp_biquad3_lowpass(&gyro_lpf, FILTER_NOMINAL_RATE_HZ,
                           gyro_lpf_hz))
    {
      syslog(LOG_ERR, "[sensors] invalid LPF configuration\n");
      return EXIT_FAILURE;
    }

  if (!dsp_biquad3_notch(&gyro_notch, FILTER_NOMINAL_RATE_HZ,
                         gyro_notch_hz, gyro_notch_bw_hz))
    {
      syslog(LOG_WARNING,
             "[sensors] invalid gyro notch %.1f/%.1f Hz; disabled\n",
             (double)gyro_notch_hz, (double)gyro_notch_bw_hz);
      dsp_biquad3_notch(&gyro_notch, FILTER_NOMINAL_RATE_HZ, 0.0f,
                        gyro_notch_bw_hz);
      gyro_notch_hz = 0.0f;
    }

  board_rot  = (uint8_t)param_i32("SENS_BOARD_ROT");
  sensor_rot = (uint8_t)param_i32(sel == 0 ? "SENS_IMU0_ROT"
                                           : "SENS_IMU1_ROT");

  /* Refuse to run rather than publish data in an unknown frame. A rotation
   * this build cannot perform is not "close to none" - the parameter says the
   * sensor is mounted somewhere the code cannot express, and quietly using the
   * identity would label sensor-frame data as body-frame.
   */

  if (!rotation_supported(board_rot) || !rotation_supported(sensor_rot))
    {
      syslog(LOG_ERR,
             "[sensors] unsupported rotation: SENS_BOARD_ROT=%u "
             "SENS_IMU%d_ROT=%u (45-degree rotations are not implemented)\n",
             board_rot, (int)sel, sensor_rot);
      return EXIT_FAILURE;
    }

  if (!compose_rotations(sensor_rot, board_rot, &amap, &anamed))
    {
      syslog(LOG_ERR, "[sensors] could not compose the rotations\n");
      return EXIT_FAILURE;
    }

  /* Both dies sit on the same package, so they share the rotation. Kept as two
   * maps anyway: the moment a board appears with the gyro mounted differently,
   * that is a parameter change and not a rewrite.
   */

  gmap   = amap;
  gnamed = anamed;

  load_cal(sel == 0 ? "CAL_ACC0" : "CAL_ACC1", true, aoff, ascl, &acal);
  load_cal(sel == 0 ? "CAL_GYRO0" : "CAL_GYRO1", false, goff, gscl, &gcal);

  ameta = orb_get_meta("sensor_accel");
  gmeta = orb_get_meta("sensor_gyro");

  if (ameta == NULL || gmeta == NULL)
    {
      syslog(LOG_ERR, "[sensors] no sensor_accel/sensor_gyro metadata\n");
      return EXIT_FAILURE;
    }

  /* orb_subscribe() is orb_subscribe_multi(meta, 0), and the instance is NOT
   * carried in the metadata - which is how every consumer on this board ended
   * up reading IMU0 twice and calling one of them IMU1.
   */

  asub = orb_subscribe_multi(ameta, (unsigned)sel);
  gsub = orb_subscribe_multi(gmeta, (unsigned)sel);

  if (asub < 0 || gsub < 0)
    {
      syslog(LOG_ERR, "[sensors] cannot subscribe to IMU%d\n", (int)sel);
      goto out;
    }

  apub = vehicle_accel_advertise();
  gpub = vehicle_gyro_advertise();

  /* Name the topic that failed. "cannot advertise the corrected topics" cost a
   * flash cycle to diagnose: both are advertised together, only one had a name
   * too long for uORB's node path, and the message could not say which.
   */

  if (apub < 0 || gpub < 0)
    {
      syslog(LOG_ERR, "[sensors] cannot advertise %s%s%s (errno %d)\n",
             apub < 0 ? "vehicle_accel" : "",
             (apub < 0 && gpub < 0) ? " and " : "",
             gpub < 0 ? "vehicle_gyro" : "", errno);
      goto out;
    }

  pthread_mutex_lock(&g_lock);
  memset(&g_status, 0, sizeof(g_status));
  g_status.instance         = (uint8_t)sel;
  g_status.accel_rot        = anamed;
  g_status.gyro_rot         = gnamed;
  g_status.accel_calibrated = acal;
  g_status.gyro_calibrated  = gcal;
  g_status.accel_filter_rate_hz = FILTER_NOMINAL_RATE_HZ;
  g_status.gyro_filter_rate_hz  = FILTER_NOMINAL_RATE_HZ;
  g_status.accel_lpf_hz          = accel_lpf_hz;
  g_status.gyro_lpf_hz           = gyro_lpf_hz;
  g_status.gyro_notch_hz         = gyro_notch_hz;
  g_status.gyro_notch_bw_hz      = gyro_notch_bw_hz;
  memcpy(g_status.accel_off, aoff, sizeof(aoff));
  memcpy(g_status.accel_scl, ascl, sizeof(ascl));
  memcpy(g_status.gyro_off, goff, sizeof(goff));
  g_status.running = true;
  pthread_mutex_unlock(&g_lock);

  syslog(LOG_INFO,
         "[sensors] IMU%d -> body, rotation %s, accel cal %s, gyro cal %s\n",
         (int)sel, rotation_name(anamed),
         acal ? "on" : "NONE (raw passthrough)",
         gcal ? "on" : "NONE (raw passthrough)");

  syslog(LOG_INFO,
         "[sensors] corrected filters accel LPF %.1f Hz; gyro notch "
         "%.1f/%.1f Hz -> LPF %.1f Hz\n",
         (double)accel_lpf_hz, (double)gyro_notch_hz,
         (double)gyro_notch_bw_hz, (double)gyro_lpf_hz);

  pfd[0].fd     = asub;
  pfd[0].events = POLLIN;
  pfd[1].fd     = gsub;
  pfd[1].events = POLLIN;

  g_running = true;

  while (!g_should_stop)
    {
      int drained;

      if (poll(pfd, 2, 100) <= 0)
        {
          continue;
        }

      if ((pfd[0].revents & POLLIN) != 0)
        {
          drained = 0;

          while (drained++ < SENSORS_DRAIN_MAX &&
                 orb_copy(ameta, asub, &araw) == 0)
            {
              float v[3];
              float raw[3];
              float raw_rms[3];
              float filtered_rms[3];
              int rate_event;

              v[0] = (araw.x - aoff[0]) * ascl[0];
              v[1] = (araw.y - aoff[1]) * ascl[1];
              v[2] = (araw.z - aoff[2]) * ascl[2];
              map_apply(&amap, v);

              /* Reject before a bad sample can seed either the biquad
               * history or the measured-rate state.  A later valid sample
               * continues from the last valid filter state.
               */

              if (!vector_finite(v))
                {
                  g_status.filter_invalid++;
                  g_status.accel_skipped++;
                  continue;
                }

              memcpy(raw, v, sizeof(raw));
              rate_event = filter_rate_update(&accel_rate, araw.timestamp);

              if (!accel_filter_started || rate_event < 0)
                {
                  dsp_biquad3_reset(&accel_lpf, v);
                  accel_filter_started = true;

                  if (rate_event < 0)
                    {
                      g_status.filter_resets++;
                      g_status.filter_timestamp_errors++;
                    }
                }

              if (rate_event > 0)
                {
                  dsp_biquad3_lowpass(&accel_lpf, accel_rate.rate_hz,
                                      accel_lpf_hz);
                  dsp_biquad3_reset(&accel_lpf, v);
                  g_status.accel_filter_rate_hz = accel_rate.rate_hz;
                  g_status.filter_resets++;
                }

              dsp_biquad3_apply(&accel_lpf, v);

              if (!vector_finite(v))
                {
                  dsp_biquad3_reset(&accel_lpf, raw);
                  memcpy(v, raw, sizeof(v));
                  g_status.filter_invalid++;
                  g_status.filter_resets++;
                }

              if (filter_rms_update(&accel_rms, raw, v, raw_rms,
                                    filtered_rms))
                {
                  pthread_mutex_lock(&g_lock);
                  memcpy(g_status.accel_raw_rms, raw_rms, sizeof(raw_rms));
                  memcpy(g_status.accel_filt_rms, filtered_rms,
                         sizeof(filtered_rms));
                  pthread_mutex_unlock(&g_lock);
                }

              memset(&aout, 0, sizeof(aout));
              aout.timestamp_sample = araw.timestamp;
              aout.timestamp        = sensors_now_us();
              aout.x                = v[0];
              aout.y                = v[1];
              aout.z                = v[2];
              aout.instance         = (uint8_t)sel;
              aout.calibrated       = acal ? 1 : 0;

              if (vehicle_accel_publish(apub, &aout) == 0)
                {
                  g_status.accel_out++;
                }
              else
                {
                  g_status.accel_skipped++;
                }
            }
        }

      if ((pfd[1].revents & POLLIN) != 0)
        {
          drained = 0;

          while (drained++ < SENSORS_DRAIN_MAX &&
                 orb_copy(gmeta, gsub, &graw) == 0)
            {
              float v[3];
              float raw[3];
              float raw_rms[3];
              float filtered_rms[3];
              int rate_event;

              /* No scale: a gyro's sensitivity cannot be measured without a
               * rate table, so gscl stays 1.0 and is not applied at all
               * rather than multiplied by one in the hot loop.
               */

              v[0] = graw.x - goff[0];
              v[1] = graw.y - goff[1];
              v[2] = graw.z - goff[2];
              map_apply(&gmap, v);

              if (!vector_finite(v))
                {
                  g_status.filter_invalid++;
                  g_status.gyro_skipped++;
                  continue;
                }

              memcpy(raw, v, sizeof(raw));
              rate_event = filter_rate_update(&gyro_rate, graw.timestamp);

              if (!gyro_filter_started || rate_event < 0)
                {
                  dsp_biquad3_reset(&gyro_notch, v);
                  dsp_biquad3_reset(&gyro_lpf, v);
                  gyro_filter_started = true;

                  if (rate_event < 0)
                    {
                      g_status.filter_resets++;
                      g_status.filter_timestamp_errors++;
                    }
                }

              if (rate_event > 0)
                {
                  dsp_biquad3_notch(&gyro_notch, gyro_rate.rate_hz,
                                    gyro_notch_hz, gyro_notch_bw_hz);
                  dsp_biquad3_lowpass(&gyro_lpf, gyro_rate.rate_hz,
                                      gyro_lpf_hz);
                  dsp_biquad3_reset(&gyro_notch, v);
                  dsp_biquad3_reset(&gyro_lpf, v);
                  g_status.gyro_filter_rate_hz = gyro_rate.rate_hz;
                  g_status.filter_resets++;
                }

              dsp_biquad3_apply(&gyro_notch, v);
              dsp_biquad3_apply(&gyro_lpf, v);

              if (!vector_finite(v))
                {
                  dsp_biquad3_reset(&gyro_notch, raw);
                  dsp_biquad3_reset(&gyro_lpf, raw);
                  memcpy(v, raw, sizeof(v));
                  g_status.filter_invalid++;
                  g_status.filter_resets++;
                }

              if (filter_rms_update(&gyro_rms, raw, v, raw_rms,
                                    filtered_rms))
                {
                  pthread_mutex_lock(&g_lock);
                  memcpy(g_status.gyro_raw_rms, raw_rms, sizeof(raw_rms));
                  memcpy(g_status.gyro_filt_rms, filtered_rms,
                         sizeof(filtered_rms));
                  pthread_mutex_unlock(&g_lock);
                }

              memset(&gout, 0, sizeof(gout));
              gout.timestamp_sample = graw.timestamp;
              gout.timestamp        = sensors_now_us();
              gout.x                = v[0];
              gout.y                = v[1];
              gout.z                = v[2];
              gout.instance         = (uint8_t)sel;
              gout.calibrated       = gcal ? 1 : 0;

              if (vehicle_gyro_publish(gpub, &gout) == 0)
                {
                  g_status.gyro_out++;
                }
              else
                {
                  g_status.gyro_skipped++;
                }
            }
        }
    }

out:
  if (asub >= 0)
    {
      orb_unsubscribe(asub);
    }

  if (gsub >= 0)
    {
      orb_unsubscribe(gsub);
    }

  if (apub >= 0)
    {
      orb_unadvertise(apub);
    }

  if (gpub >= 0)
    {
      orb_unadvertise(gpub);
    }

  pthread_mutex_lock(&g_lock);
  g_status.running = false;
  pthread_mutex_unlock(&g_lock);

  g_running     = false;
  g_should_stop = false;
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sensors_start(void)
{
  int pid;
  int spin;

  if (g_running)
    {
      return -EALREADY;
    }

  g_should_stop = false;

  pid = task_create("sensors", SENSORS_PRIO, SENSORS_STACK,
                    sensors_daemon, NULL);
  if (pid < 0)
    {
      return -errno;
    }

  /* Report what actually happened. A start that returns OK because a second
   * elapsed - while the task exited on an unsupported rotation or a failed
   * subscription - is the logger's old bug, and it is not worth repeating.
   */

  for (spin = 0; spin < 100; spin++)
    {
      if (g_running)
        {
          return OK;
        }

      usleep(10000);
    }

  return -EIO;
}

int sensors_stop(void)
{
  int spin;

  if (!g_running)
    {
      return -ESRCH;
    }

  g_should_stop = true;

  for (spin = 0; spin < 100; spin++)
    {
      if (!g_running)
        {
          return OK;
        }

      usleep(10000);
    }

  return -ETIMEDOUT;
}

void sensors_status(FAR struct sensors_status_s *out)
{
  pthread_mutex_lock(&g_lock);
  *out = g_status;
  pthread_mutex_unlock(&g_lock);
}
