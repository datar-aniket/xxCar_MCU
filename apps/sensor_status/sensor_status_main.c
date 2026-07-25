/****************************************************************************
 * apps/sensor_status/sensor_status_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `sensor_status` - one line per onboard sensor: is it streaming, at what rate,
 * and its current reading.
 *
 * This exists because neither uorb_listener mode answers the plain question
 * "what rate is each sensor running at?". With CONFIG_DEBUG_UORB off, the
 * listener prints nothing; with it on, it dumps every field of every sample - a
 * flood. Here the rate is measured the same way the listener does internally,
 * from the topic's generation counter over a fixed window, which needs no debug
 * build and produces exactly one line per sensor.
 *
 *   sensor_status            measure over ~1 s and print once
 *   sensor_status -w         repeat until Ctrl-C
 *   sensor_status -t <ms>    measurement window (default 1000 ms)
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <uORB/uORB.h>
#include <nuttx/uorb.h>

#include "../uorb_msgs/uorb_msgs.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* What kind of payload a topic carries, so we can print one sensible line of
 * values for it.
 */

enum sens_kind_e
{
  KIND_ACCEL,   /* x y z (m/s^2) */
  KIND_GYRO,    /* x y z (rad/s) */
  KIND_MAG,     /* x y z (gauss) */
  KIND_BARO,    /* pressure (hPa) */
  KIND_FLOW,    /* optical flow (MTF-02 over MAVLink) */
  KIND_DIST,    /* ranged distance */
};

struct sens_row_s
{
  FAR const char                *name;  /* uORB topic name */
  FAR const char                *label; /* human label + which chip */
  enum sens_kind_e               kind;
  FAR const struct orb_metadata *meta;  /* non-NULL for our own topics, which
                                         * are not in uORB's built-in name list;
                                         * NULL means look up by name */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Every sensor topic on the vehicle, onboard and off. Instance is encoded in
 * the name (…0 primary, …1 secondary) as uORB registers them.
 *
 * The onboard sensors are built-in uORB topics, found by name. The MAVLink
 * sensors (optical_flow, distance_sensor) are ours, so they carry a direct
 * ORB_ID - orb_get_meta() only knows the built-in list. They read "absent"
 * until the MAVLink daemon advertises them, and "stalled" if it is running but
 * the MTF-02 is not actually sending.
 */

static const struct sens_row_s g_rows[] =
{
  { "sensor_accel0", "accel0  ICM-42688", KIND_ACCEL, NULL },
  { "sensor_gyro0",  "gyro0   ICM-42688", KIND_GYRO,  NULL },
  { "sensor_accel1", "accel1  BMI055",    KIND_ACCEL, NULL },
  { "sensor_gyro1",  "gyro1   BMI055",    KIND_GYRO,  NULL },
  { "sensor_mag0",   "mag0    IST8310",   KIND_MAG,   NULL },
  { "sensor_baro0",  "baro0   MS5611",    KIND_BARO,  NULL },
  { "optical_flow",  "flow    MTF-02",    KIND_FLOW,  ORB_ID(optical_flow) },
  { "distance_sensor", "range MTF-02",    KIND_DIST,  ORB_ID(distance_sensor) },
};

#define NROWS ((int)(sizeof(g_rows) / sizeof(g_rows[0])))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sensor_status_values(int fd, enum sens_kind_e kind,
                                 FAR const struct orb_metadata *meta)
{
  union
  {
    struct sensor_accel      accel;
    struct sensor_gyro       gyro;
    struct sensor_mag        mag;
    struct sensor_baro       baro;
    struct optical_flow_s    flow;
    struct distance_sensor_s dist;
  } d;

  if (orb_copy(meta, fd, &d) < 0)
    {
      return;
    }

  switch (kind)
    {
      case KIND_ACCEL:
        printf("  % 8.3f % 8.3f % 8.3f m/s2  %5.1fC",
               d.accel.x, d.accel.y, d.accel.z, d.accel.temperature);
        break;

      case KIND_GYRO:
        printf("  % 8.3f % 8.3f % 8.3f rad/s %5.1fC",
               d.gyro.x, d.gyro.y, d.gyro.z, d.gyro.temperature);
        break;

      case KIND_MAG:
        printf("  % 8.3f % 8.3f % 8.3f gauss %5.1fC",
               d.mag.x, d.mag.y, d.mag.z, d.mag.temperature);
        break;

      case KIND_BARO:
        printf("  %9.2f hPa            %5.1fC",
               d.baro.pressure, d.baro.temperature);
        break;

      case KIND_FLOW:

        /* Flow is integrated; divide by the window to get an average rate. A
         * negative distance means the sensor could not range the surface.
         */

        printf("  q=%-3u fx=% .3f fy=% .3f  h=%.2fm",
               d.flow.quality, d.flow.integrated_x, d.flow.integrated_y,
               d.flow.distance);
        break;

      case KIND_DIST:
        printf("  %.2f m", d.dist.current_distance);
        break;
    }
}

/* Measure every sensor's rate over `window_ms` and print the table.
 *
 * Rate comes from the generation counter, which advances once per published
 * sample: read it, wait, read it again, divide the delta by the elapsed time.
 * This is exactly what uorb_listener does, and it does not depend on the topic
 * ever printing a value, so CONFIG_DEBUG_UORB is irrelevant.
 */

static void sensor_status_run(int window_ms)
{
  int      fd[NROWS];
  uint64_t gen0[NROWS];
  uint64_t t0[NROWS];
  int      i;

  printf("%-18s %-8s %7s   %s\n", "SENSOR", "STATE", "RATE", "READING");

  /* First pass: subscribe and snapshot the starting generation. */

  for (i = 0; i < NROWS; i++)
    {
      FAR const struct orb_metadata *meta =
        g_rows[i].meta ? g_rows[i].meta : orb_get_meta(g_rows[i].name);
      struct orb_state st;

      fd[i]   = -1;
      gen0[i] = 0;

      if (meta == NULL)
        {
          continue;
        }

      fd[i] = orb_subscribe(meta);
      if (fd[i] < 0)
        {
          continue;
        }

      if (orb_get_state(fd[i], &st) == 0)
        {
          gen0[i] = st.generation;
          t0[i]   = orb_absolute_time();
        }
    }

  usleep((useconds_t)window_ms * 1000);

  /* Second pass: measure and print. */

  for (i = 0; i < NROWS; i++)
    {
      FAR const struct orb_metadata *meta =
        g_rows[i].meta ? g_rows[i].meta : orb_get_meta(g_rows[i].name);
      struct orb_state st;
      double hz = 0.0;

      if (fd[i] < 0 || meta == NULL)
        {
          printf("%-18s %-8s\n", g_rows[i].label, "absent");
          continue;
        }

      if (orb_get_state(fd[i], &st) == 0)
        {
          uint64_t dgen = st.generation - gen0[i];
          uint64_t dt   = orb_absolute_time() - t0[i];

          if (dt > 0)
            {
              hz = (double)dgen * 1000000.0 / (double)dt;
            }
        }

      if (hz < 1.0)
        {
          /* Subscribed, but nothing came through in the window. */

          printf("%-18s %-8s %7s\n", g_rows[i].label, "STALLED", "-");
        }
      else
        {
          printf("%-18s %-8s %6.1f", g_rows[i].label, "stream", hz);
          sensor_status_values(fd[i], g_rows[i].kind, meta);
          printf("\n");
        }

      orb_unsubscribe(fd[i]);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int window_ms = 1000;
  bool watch = false;
  int i;

  for (i = 1; i < argc; i++)
    {
      if (strcmp(argv[i], "-w") == 0)
        {
          watch = true;
        }
      else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc)
        {
          window_ms = atoi(argv[++i]);
          if (window_ms < 100)
            {
              window_ms = 100;   /* below this the count is too small to trust */
            }
        }
      else
        {
          printf("Usage: sensor_status [-w] [-t <ms>]\n"
                 "  -w        repeat until Ctrl-C\n"
                 "  -t <ms>   measurement window (default 1000)\n");
          return 1;
        }
    }

  do
    {
      sensor_status_run(window_ms);

      if (watch)
        {
          printf("\n");
        }
    }
  while (watch);

  return 0;
}
