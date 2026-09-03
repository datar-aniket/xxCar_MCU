/****************************************************************************
 * apps/logger/logger.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * On-request ULog logger. See logger.h.
 *
 * ULog in brief: a 16-byte header (magic + start timestamp), then a definition
 * section of 'F' (format) messages, then a data section of 'A' (subscription)
 * and 'D' (data) messages. Every message is prefixed with a 3-byte header:
 * uint16 payload-size, uint8 type. Every logged message's first field must be
 * uint64 timestamp - all our topics satisfy that. The reader (pyulog) recovers
 * everything from the 'F' strings, so the field lists here must match the C
 * structs byte-for-byte, padding included.
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <syslog.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <uORB/uORB.h>

#include "logger.h"
#include "log_batch.h"
#include "log_write.h"
#include "../param/param.h"
#include "../rc_in/rc_in.h"
#include "../uorb_msgs/uorb_msgs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LOG_STACK     4096
/* Below the sensor threads, which run at SCHED_PRIORITY_DEFAULT + 50 (see
 * FMUV6C_SENSOR_PRIO). That ordering is load-bearing, not a preference: this
 * task blocks for 50-200 ms on a stalling card, and the ICM-42688's FIFO only
 * holds about 51 ms at 2 kHz. Preempting the sensor threads for a card write
 * loses samples in hardware, where nothing downstream can see it happen.
 *
 * The comment here used to claim it was below the sensors while the numbers
 * said otherwise - the sensors were at DEFAULT and this was DEFAULT + 2.
 */

#define LOG_PRIO      (SCHED_PRIORITY_DEFAULT + 2)

#define LOG_DIR       "/fs/microsd/log"

/* Write buffer. At 2 kHz across the IMUs the logger produces a record every few
 * hundred microseconds; a write() per record would swamp the FAT layer. Records
 * accumulate here and go to the card in one write when the buffer fills.
 *
 * 64 KB, not the 8 KB this started with, because the buffer's real job is not
 * batching - it is absorbing SD stalls. Cards routinely pause 50-200 ms for
 * wear levelling and internal housekeeping, and 8 KB is about 36 ms of
 * full-rate IMU data. 64 KB is roughly 290 ms, which covers the stalls
 * actually observed.
 *
 * A bigger buffer is only safe because log_flush() rolls an ambiguous partial
 * write back to the preceding complete flush boundary and stops. Continuing
 * after a partial write made the corruption larger - see the comment there.
 */

#define LOG_BUFSIZE   65536
#define LOG_SECTOR_SIZE 512

#if LOG_BUFSIZE % LOG_SECTOR_SIZE != 0
#  error LOG_BUFSIZE must be a whole number of SD sectors
#endif

/* Roll over to a new file at this size.
 *
 * FAT32 cannot hold a file above 4 GB, which at full rate arrives in five
 * hours - long enough to look fine and short enough to end an overnight run
 * before morning. Splitting also means a card pulled or a power loss costs one
 * part rather than the whole night.
 *
 * Each part is a COMPLETE ULog file: header, format definitions and
 * subscriptions are written afresh at the start of every one. Splitting the
 * byte stream alone would leave every part after the first unreadable, since
 * the definitions live only at the front.
 */

#define LOG_PART_BYTES  (100u * 1024u * 1024u)

/* Largest record we serialise (estimator_diag, 288 bytes including trailing
 * C padding; 285 meaningful bytes are written).
 */

#define LOG_RECMAX    320
#define LOG_DRAIN_MAX 1024
#define LOG_READ_BATCH 32

/* ULog message types. */

#define ULOG_MSG_FORMAT  'F'
#define ULOG_MSG_ADD_SUB 'A'
#define ULOG_MSG_DATA    'D'

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A loggable topic: how to find it, its ULog identity, how many bytes of the
 * sample go into the file, and the LOG_* parameter that turns it on.
 *
 * Adding a sensor is one row here plus one LOG_* parameter - nothing else in the
 * logger changes. That is the whole reason the gate is a parameter name rather
 * than a hard-coded group.
 */

struct log_topic_s
{
  FAR const char                *orb_name;    /* orb_get_meta() name, or NULL */
  FAR const struct orb_metadata *direct_meta; /* non-NULL for our own topics
                                               * (rc_in, optical_flow, ...),
                                               * which orb_get_meta cannot find
                                               * by name */
  uint8_t                        orb_instance; /* uORB device instance */
  FAR const char                *ulog_name;   /* ULog message name */
  uint8_t                        multi_id;    /* ULog instance */
  uint16_t                       rec_size;    /* meaningful bytes to write */
  FAR const char                *param;       /* "LOG_IMU0", ... */
};

/* Per-subscription runtime state. */

struct log_sub_s
{
  FAR const struct log_topic_s *topic;
  FAR const struct orb_metadata *meta;
  int      fd;
  uint16_t msg_id;
  uint32_t min_interval_us; /* zero for lossless EKF/event streams */
  uint64_t next_us;   /* next LOG_RATE sample-time boundary */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The ULog format strings, one per distinct message name. Field lists mirror
 * the C structs exactly (see nuttx/uorb.h and rc_in.h), padding included, or
 * pyulog would mis-decode every field after the gap.
 */

static const struct
{
  FAR const char *name;
  FAR const char *fields;
}
g_formats[] =
{
  { "sensor_accel",
    "uint64_t timestamp;float x;float y;float z;float temperature;" },
  { "sensor_gyro",
    "uint64_t timestamp;float x;float y;float z;float temperature;" },
  { "sensor_baro",
    "uint64_t timestamp;float pressure;float temperature;" },
  { "sensor_mag",
    "uint64_t timestamp;float x;float y;float z;float temperature;"
    "int32_t status;" },
  { "rc_input",
    "uint64_t timestamp;uint16_t[18] channel;uint16_t frames;"
    "uint16_t lost_frames;uint8_t count;uint8_t rssi;uint8_t ok;"
    "uint8_t failsafe;uint8_t source;" },
  { "optical_flow",
    "uint64_t timestamp;uint32_t integration_time_us;"
    "uint32_t time_delta_distance_us;float integrated_x;float integrated_y;"
    "float integrated_xgyro;float integrated_ygyro;float integrated_zgyro;"
    "float distance;int16_t temperature;uint8_t quality;uint8_t sensor_id;" },
  { "distance_sensor",
    "uint64_t timestamp;float current_distance;float min_distance;"
    "float max_distance;uint8_t type;uint8_t orientation;uint8_t covariance;"
    "uint8_t signal_quality;" },
  { "vehicle_imu",
    "uint64_t timestamp;uint64_t timestamp_sample;"
    "uint64_t timestamp_first;float[3] delta_angle;"
    "float[3] delta_velocity;float delta_angle_dt;"
    "float delta_velocity_dt;uint16_t samples;uint16_t reset_counter;"
    "uint8_t instance;uint8_t clipping;uint8_t accel_calibrated;"
    "uint8_t gyro_calibrated;" },
  { "estimator_state",
    "uint64_t timestamp;uint64_t timestamp_sample;float[4] quaternion;"
    "float[3] velocity;float[3] position;float[3] gyro_bias;"
    "float[3] accel_bias;float[3] angle_variance;"
    "float[3] velocity_variance;float[3] position_variance;"
    "uint32_t predict_count;uint32_t covariance_count;"
    "uint16_t reset_counter;uint8_t solution_status;uint8_t instance;" },
  { "external_pose",
    "uint64_t timestamp;uint64_t timestamp_sample;float x;float y;float yaw;"
    "float[6] cov;uint8_t flags;uint8_t reset_counter;" },
  { "vehicle_accel",
    "uint64_t timestamp;uint64_t timestamp_sample;float x;float y;float z;"
    "uint8_t instance;uint8_t calibrated;" },
  { "vehicle_state_tx",
    "uint64_t timestamp;uint64_t timestamp_sample;"
    "uint64_t accel_timestamp_sample;uint64_t wire_timestamp_us;"
    "float[3] position;float[4] quaternion;float[3] velocity;"
    "float[3] angular_velocity;float side_slip_rad;float[3] accel;"
    "float wheel_torque_nm;float steering_angle;float motor_speed_ms;"
    "uint32_t rc_status;uint8_t solution_status;uint8_t reset_counter;"
    "uint8_t source_valid;" },
  { "estimator_diag",
    "uint64_t timestamp;uint64_t timestamp_sample;"
    "uint64_t extnav_timestamp;float[3] specific_force;"
    "float[3] corrected_force;float[3] gravity_body;"
    "float[3] residual_accel_body;float[3] nav_accel;"
    "float[4] quaternion;float[3] velocity;float[3] position;"
    "float[3] gyro_bias;float[3] accel_bias;float[2] extnav_innov;"
    "float[2] extnav_nis;float[3] extnav_measurement;float[3] zupt_nis;"
    "float gravity_nis;float accel_norm;float accel_variance;"
    "float gravity_deviation;float extnav_test_ratio;"
    "float wheel_speed_cps;float wheel_speed_mps;"
    "float wheel_accel_mps2;float imu_accel_mps2;"
    "float[2] wheel_innov;float[2] wheel_nis;uint64_t wheel_timestamp;"
    "uint32_t extnav_accept_count;"
    "uint32_t extnav_reject_count;uint32_t zupt_accept_count;"
    "uint32_t zupt_reject_count;uint32_t gravity_accept_count;"
    "uint32_t gravity_reject_count;uint32_t wheel_accept_count;"
    "uint32_t wheel_reject_count;uint16_t reset_counter;uint16_t flags;"
    "uint8_t instance;" },
};

#define NFORMATS ((int)(sizeof(g_formats) / sizeof(g_formats[0])))

/* The candidate topics. rec_size is the number of MEANINGFUL bytes to write -
 * the fields, without any trailing C padding.
 *
 * This is not sizeof for the padded structs, and that is deliberate: pyulog
 * strips a trailing _padding field from both the record's expected size and its
 * maximum, so it wants the bytes without the pad. sensor_mag is 28 (sizeof 32),
 * rc_in is 53 (sizeof 56). Neither struct has *internal* padding, so the first
 * rec_size bytes are exactly the fields, contiguous. accel/gyro/baro have no
 * padding at all, so there rec_size == sizeof.
 */

static const struct log_topic_s g_topics[] =
{
  /* orb_name        direct_meta              inst ulog_name       mid sz param */
  { "sensor_accel0", NULL,                       0, "sensor_accel",    0, 24, "LOG_IMU0" },
  { "sensor_gyro0",  NULL,                       0, "sensor_gyro",     0, 24, "LOG_IMU0" },
  { "sensor_accel1", NULL,                       1, "sensor_accel",    1, 24, "LOG_IMU1" },
  { "sensor_gyro1",  NULL,                       1, "sensor_gyro",     1, 24, "LOG_IMU1" },
  { "sensor_mag0",   NULL,                       0, "sensor_mag",      0, 28, "LOG_MAG"  },
  { "sensor_baro0",  NULL,                       0, "sensor_baro",     0, 16, "LOG_BARO" },
  { NULL,            ORB_ID(rc_in),              0, "rc_input",        0, 53, "LOG_RC"   },
  { NULL,            ORB_ID(optical_flow),       0, "optical_flow",    0, 44, "LOG_FLOW" },
  { NULL,            ORB_ID(distance_sensor),    0, "distance_sensor", 0, 24, "LOG_DIST" },
  { NULL,            ORB_ID(vehicle_imu),        0, "vehicle_imu",     0, 64, "LOG_EKF"  },
  { NULL,            ORB_ID(estimator_state),    0, "estimator_state", 0, 128,"LOG_EKF"  },
  { NULL,            ORB_ID(external_pose),      0, "external_pose",   0, 54, "LOG_EKF"  },
  { NULL,            ORB_ID(vehicle_accel),      0, "vehicle_accel",   0, 30, "LOG_EKF"  },
  { NULL,            ORB_ID(vehicle_state_tx),   0, "vehicle_state_tx",0, 119,"LOG_EKF"  },
  { NULL,            ORB_ID(estimator_diag),     0, "estimator_diag",  0, 285,"LOG_EKF"  },
};

#define NTOPICS ((int)(sizeof(g_topics) / sizeof(g_topics[0])))

static pthread_mutex_t   g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool     g_running;
static volatile bool     g_should_stop;
static struct logger_status_s g_status;

/* Writer state. */

static int      g_fd = -1;

/* Bytes written to the CURRENT part, which is not g_status.bytes: that one
 * counts the whole session and must keep climbing across a rollover.
 */

static uint32_t g_part_bytes;
/* Cache-line aligned, because FAT hands this buffer straight to the SDMMC
 * IDMA for full-sector writes rather than copying it through its own sector
 * buffer. The stock driver aligns its internal buffers the same way
 * (arch/arm/src/stm32h7/stm32_sdmmc.c).
 *
 * It was aligned only by accident of where .bss happened to place it, and it
 * stopped being aligned the moment an unrelated array was added ahead of it -
 * which is exactly the kind of dependency that produces corruption nobody can
 * reproduce.
 */

static uint8_t  g_buf[LOG_BUFSIZE] aligned_data(32);
static size_t   g_buflen;

/* uORB can return multiple queued events in one read. Keep this out of the
 * logger's 4 KB stack and reuse it for one subscription at a time.
 */

static uint8_t  g_read_buf[LOG_RECMAX * LOG_READ_BATCH];

/****************************************************************************
 * Private Functions - the buffered ULog writer
 ****************************************************************************/

static uint64_t log_now_us(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

/* Push the buffer to the card, all of it.
 *
 * write() is allowed to be short, and the version of this that treated a short
 * write as "the card cannot keep up, drop the rest" corrupted an entire
 * overnight recording. Dropping the tail of a buffer does not lose a tidy
 * whole number of records: it cuts one in half, and every byte after that is
 * read against the wrong frame boundary. The file stays exactly as long, still
 * opens, still looks plausible - and the stream is desynchronised from that
 * point until the next part.
 *
 * It got worse with the buffer, not better. At 8 KB a dropped flush cost about
 * 70 ms of data; at 64 KB it costs 565 ms, and a 7.7 hour run came back as
 * 26,783 fragments averaging 0.42 s - fine for white noise, useless for the
 * bias instability the run existed to measure.
 *
 * Resuming at FAT's reported position also proved unsafe: a real recording
 * showed that position disagreeing with the data boundary by one byte, first
 * inserting a byte and later deleting one. Retry only when absolutely no file
 * progress occurred. For any partial progress, truncate to the preceding flush
 * boundary and stop - a shorter readable log is better than a longer corrupt
 * one.
 */

static bool log_flush(bool all)
{
  off_t flush_start;
  size_t written = 0;
  size_t pending;
  size_t remaining;
  int ret;

  if (g_buflen == 0)
    {
      return true;
    }

  /* Keep periodic writes sector-aligned at both ends. The file starts at
   * offset zero and g_buf is cache-line aligned, so writing only complete
   * 512-byte prefixes preserves that alignment for every later flush. This
   * lets FAT use one fast direct multi-sector IDMA transfer.
   *
   * Writing the arbitrary-length buffer on every 250 ms tick left the file at
   * an arbitrary offset. FAT then had to finish that partial sector before
   * starting its next direct transfer at g_buf + 1/2/3. The SDMMC preflight
   * now correctly rejects that unaligned pointer, but its safe one-sector
   * fallback can stall long enough to overrun the 128 ms uORB history at
   * 2 kHz. Retain the sub-sector tail in RAM instead. Stop and rollover force
   * the final tail so every completed file is whole.
   */

  pending = all ? g_buflen :
                  g_buflen & ~((size_t)LOG_SECTOR_SIZE - 1);
  if (pending == 0)
    {
      return true;
    }

  flush_start = lseek(g_fd, 0, SEEK_CUR);
  ret = log_write_all(g_fd, g_buf, pending, log_io_default(), 1000,
                      &written);

  if (ret < 0)
    {
      bool rolled_back = false;
      int rollback_errno = 0;

      /* A failed FAT write may already have advanced the file through part of
       * this buffer even when write() and lseek() claim zero progress. Always
       * end the file at the previous complete flush boundary so it stays
       * structurally valid rather than leaving a torn ULog record.
       */

      if (flush_start >= 0)
        {
          if (ftruncate(g_fd, flush_start) == 0)
            {
              rolled_back = true;

              /* ftruncate() does not move the open file position. Nothing
               * else will be written after this failure, but restore it so a
               * future caller cannot accidentally create a hole.
               */

              if (lseek(g_fd, flush_start, SEEK_SET) < 0)
                {
                  syslog(LOG_ERR,
                         "logger: truncated failed flush but could not restore "
                         "the file position: %d\n", errno);
                }
            }
          else
            {
              rollback_errno = errno;
            }
        }

      if (!rolled_back)
        {
          pthread_mutex_lock(&g_lock);
          g_status.bytes += written;
          pthread_mutex_unlock(&g_lock);
          g_part_bytes += (uint32_t)written;
        }

      if (rollback_errno != 0)
        {
          syslog(LOG_ERR,
                 "logger: flush failed: %d after %zu/%zu bytes; partial write "
                 "remains (truncate errno %d)\n",
                 ret, written, pending, rollback_errno);
        }
      else
        {
          syslog(LOG_ERR,
                 "logger: flush failed: %d after %zu/%zu bytes; partial write "
                 "%s\n",
                 ret, written, pending,
                 rolled_back ? "removed" : "remains");
        }

      g_buflen = 0;
      return false;
    }

  pthread_mutex_lock(&g_lock);
  g_status.bytes += written;
  pthread_mutex_unlock(&g_lock);
  g_part_bytes += (uint32_t)written;

  remaining = g_buflen - written;
  if (remaining > 0)
    {
      memmove(g_buf, g_buf + written, remaining);
    }

  g_buflen = remaining;
  return true;
}

/* Append bytes, flushing first if they will not fit. */

static bool log_put(FAR const void *data, size_t len)
{
  if (len > LOG_BUFSIZE)
    {
      return false;
    }

  if (g_buflen + len > LOG_BUFSIZE && !log_flush(false))
    {
      return false;
    }

  memcpy(g_buf + g_buflen, data, len);
  g_buflen += len;
  return true;
}

/* Write one ULog message: the 3-byte header then the payload. */

static bool log_msg(uint8_t type, FAR const void *payload, uint16_t len)
{
  uint8_t hdr[3];

  hdr[0] = (uint8_t)(len & 0xff);
  hdr[1] = (uint8_t)(len >> 8);
  hdr[2] = type;

  return log_put(hdr, sizeof(hdr)) && log_put(payload, len);
}

/* Write a 'D' data message: header, then uint16 msg_id, then the record. */

static bool log_data(uint16_t msg_id, FAR const void *rec, uint16_t rec_size)
{
  uint8_t hdr[3];
  uint8_t id[2];
  uint16_t len = (uint16_t)(2 + rec_size);

  hdr[0] = (uint8_t)(len & 0xff);
  hdr[1] = (uint8_t)(len >> 8);
  hdr[2] = ULOG_MSG_DATA;

  id[0] = (uint8_t)(msg_id & 0xff);
  id[1] = (uint8_t)(msg_id >> 8);

  return log_put(hdr, sizeof(hdr)) && log_put(id, sizeof(id)) &&
         log_put(rec, rec_size);
}

static bool log_write_header(void)
{
  static const uint8_t magic[8] =
  {
    0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35, 0x01
  };

  /* The flag-bits message: compat[8], incompat[8], appended_offsets[3]*u64, all
   * zero - no optional features, no crash data appended. pyulog parses without
   * it, but every real ULog carries it and the stricter readers (FlightPlot,
   * QGC) expect it, so it is cheap insurance. It must be the first message after
   * the 16-byte header.
   */

  uint8_t flags[40];
  uint64_t start = log_now_us();

  memset(flags, 0, sizeof(flags));

  return log_put(magic, sizeof(magic)) &&
         log_put(&start, sizeof(start)) &&
         log_msg('B', flags, sizeof(flags));
}

static bool log_write_formats(void)
{
  int i;

  for (i = 0; i < NFORMATS; i++)
    {
      size_t namelen = strlen(g_formats[i].name);
      size_t fieldlen = strlen(g_formats[i].fields);
      size_t payload_len;
      uint8_t hdr[3];

      /* FORMAT payloads are not bounded to 255 bytes. optical_flow is already
       * 269 bytes, so formatting it through a 256-byte stack array truncated
       * the string while snprintf() returned 269. log_msg() then read 13 bytes
       * beyond that array and wrote an invalid definition section.
       *
       * Emit the payload directly into the logger buffer instead. The ULog
       * length field is uint16_t, so reject a future definition that cannot be
       * represented rather than silently narrowing it.
       */

      if (namelen >= UINT16_MAX ||
          fieldlen > UINT16_MAX - namelen - 1)
        {
          syslog(LOG_ERR, "logger: ULog format definition is too long: %s\n",
                 g_formats[i].name);
          return false;
        }

      payload_len = namelen + 1 + fieldlen;
      hdr[0] = (uint8_t)(payload_len & 0xff);
      hdr[1] = (uint8_t)(payload_len >> 8);
      hdr[2] = ULOG_MSG_FORMAT;

      if (!log_put(hdr, sizeof(hdr)) ||
          !log_put(g_formats[i].name, namelen) ||
          !log_put(":", 1) ||
          !log_put(g_formats[i].fields, fieldlen))
        {
          return false;
        }
    }

  return true;
}

static bool log_write_subscription(FAR const struct log_sub_s *sub)
{
  uint8_t payload[64];
  size_t namelen = strlen(sub->topic->ulog_name);
  size_t len = 0;

  payload[len++] = sub->topic->multi_id;
  payload[len++] = (uint8_t)(sub->msg_id & 0xff);
  payload[len++] = (uint8_t)(sub->msg_id >> 8);
  memcpy(payload + len, sub->topic->ulog_name, namelen);
  len += namelen;

  return log_msg(ULOG_MSG_ADD_SUB, payload, (uint16_t)len);
}

/****************************************************************************
 * Private Functions - session setup
 ****************************************************************************/

/* Pick the next unused session number.
 *
 * sscanf("log_007_02.ulg", "log_%d...") still yields 7, because %d stops at the
 * underscore and the trailing literal failing to match does not undo the
 * assignment. So this finds the highest session across both the old one-file
 * names and the split ones.
 */

static int log_next_session(void)
{
  FAR DIR *dir;
  FAR struct dirent *ent;
  int next = 1;

  mkdir(LOG_DIR, 0777);   /* harmless if it already exists */

  dir = opendir(LOG_DIR);
  if (dir != NULL)
    {
      while ((ent = readdir(dir)) != NULL)
        {
          int n;

          if (sscanf(ent->d_name, "log_%d", &n) == 1 && n >= next)
            {
              next = n + 1;
            }
        }

      closedir(dir);
    }

  return next;
}

static int log_open_part(int session, int part, FAR char *path,
                         size_t pathlen)
{
  snprintf(path, pathlen, "%s/log_%03d_%02d.ulg", LOG_DIR, session, part);

  return open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
}

/****************************************************************************
 * The daemon
 ****************************************************************************/

static int log_daemon(int argc, FAR char *argv[])
{
  struct log_sub_s subs[NTOPICS];
  struct pollfd    pfd[NTOPICS];
  char             path[48];
  uint64_t         last_flush;
  int32_t          rate;
  uint32_t         min_interval;
  int              nsubs = 0;
  int              session = 1;
  int              part = 0;
  int              i;

  UNUSED(argc);
  UNUSED(argv);

  /* LOG_RATE 0 means every sample - full native rate. Otherwise it is a
   * per-topic ceiling, applied as a minimum spacing between logged samples.
   */

  rate = param_i32("LOG_RATE");
  min_interval = (rate > 0) ? (uint32_t)(1000000 / rate) : 0;

  /* Subscribe to each enabled topic that actually exists. Each topic's own
   * LOG_* parameter decides whether it is in. A blocking subscribe so poll()
   * wakes the moment a sample is published.
   */

  for (i = 0; i < NTOPICS; i++)
    {
      FAR const struct log_topic_s *t = &g_topics[i];
      FAR const struct orb_metadata *meta;
      int fd;

      if (param_i32(t->param) == 0)
        {
          continue;
        }

      meta = t->direct_meta ? t->direct_meta : orb_get_meta(t->orb_name);
      if (meta == NULL)
        {
          continue;
        }

      if (meta->o_size > LOG_RECMAX || t->rec_size > meta->o_size)
        {
          syslog(LOG_ERR, "logger: invalid record size for %s\n",
                 t->ulog_name);
          continue;
        }

      fd = orb_subscribe_multi(meta, t->orb_instance);
      if (fd < 0)
        {
          continue;
        }

      subs[nsubs].topic   = t;
      subs[nsubs].meta    = meta;
      subs[nsubs].fd      = fd;
      subs[nsubs].msg_id  = (uint16_t)nsubs;
      /* LOG_RATE is useful for the 2 kHz raw sensors, but decimating an EKF
       * delta packet or a fusion event makes propagation impossible to replay
       * and can hide the one innovation that caused a jump. LOG_EKF topics
       * are therefore always lossless; `log ekf 400` caps only faster raw
       * streams while retaining every native estimator record.
       */

      subs[nsubs].min_interval_us =
        strcmp(t->param, "LOG_EKF") == 0 ? 0 : min_interval;
      subs[nsubs].next_us = 0;
      nsubs++;
    }

  if (nsubs == 0)
    {
      syslog(LOG_WARNING,
             "logger: nothing selected (set LOG_IMU0 / LOG_EKF / ...)\n");
      g_running = false;
      return EXIT_FAILURE;
    }

  session = log_next_session();
  g_fd = log_open_part(session, part, path, sizeof(path));
  if (g_fd < 0)
    {
      syslog(LOG_ERR, "logger: cannot create log in %s: %d\n", LOG_DIR, errno);

      for (i = 0; i < nsubs; i++)
        {
          orb_unsubscribe(subs[i].fd);
        }

      g_running = false;
      return EXIT_FAILURE;
    }

  g_buflen     = 0;
  g_part_bytes = 0;

  /* Definition section, then the subscriptions. Repeated at the head of every
   * part - see LOG_PART_BYTES.
   */

  log_write_header();
  log_write_formats();

  for (i = 0; i < nsubs; i++)
    {
      log_write_subscription(&subs[i]);
      pfd[i].fd     = subs[i].fd;
      pfd[i].events = POLLIN;
    }

  pthread_mutex_lock(&g_lock);
  strlcpy(g_status.path, path, sizeof(g_status.path));
  g_status.topics = nsubs;
  g_status.rate   = rate;
  pthread_mutex_unlock(&g_lock);

  syslog(LOG_INFO, "logger: %s, %d topic(s), rate %s\n",
         path, nsubs, rate > 0 ? "capped" : "native");

  last_flush = log_now_us();
  g_running  = true;

  while (!g_should_stop)
    {
      uint64_t now;

      if (poll(pfd, nsubs, 100) > 0)
        {
          for (i = 0; i < nsubs; i++)
            {
              int drained = 0;

              if ((pfd[i].revents & POLLIN) == 0)
                {
                  continue;
                }

              /* One poll wakeup can represent many queued samples after an SD
               * write. Reading only one and polling again made the logger
               * fractionally slower than four native 2 kHz subscriptions. At
               * 2 kHz it lost one paired ICM sample about every 0.35 seconds
               * even though the card was no longer stalling.
               *
               * Drain the available history in a bounded batch. The bound is
               * the sensor queue depth, so no topic can monopolize this loop
               * indefinitely if its producer happens to run concurrently.
               */

              while (drained < LOG_DRAIN_MAX)
                {
                  size_t stride = subs[i].meta->o_size;
                  size_t request = LOG_DRAIN_MAX - drained;
                  size_t count;
                  size_t j;
                  ssize_t copied;

                  if (request > LOG_READ_BATCH)
                    {
                      request = LOG_READ_BATCH;
                    }

                  copied = orb_copy_multi(subs[i].fd, g_read_buf,
                                          request * stride);
                  count = log_batch_count(copied, stride);
                  if (count == 0)
                    {
                      break;
                    }

                  drained += count;
                  for (j = 0; j < count; j++)
                    {
                      FAR const uint8_t *rec =
                        log_batch_record(g_read_buf, j, stride);

                      /* The sample's own timestamp is its first 8 bytes. Use
                       * it for decimation so spacing is measured in sample
                       * time, not loop-wakeup time.
                       */

                      if (subs[i].min_interval_us > 0)
                        {
                          uint64_t ts;

                          memcpy(&ts, rec, sizeof(ts));

                          /* Advance an ideal output schedule rather than
                           * accepted-sample time, avoiding accumulated ODR
                           * phase error.
                           */

                          if (!log_sample_due(ts,
                                              subs[i].min_interval_us,
                                              &subs[i].next_us))
                            {
                              continue;
                            }
                        }

                      if (log_data(subs[i].msg_id, rec,
                                   subs[i].topic->rec_size))
                        {
                          pthread_mutex_lock(&g_lock);
                          g_status.samples++;
                          pthread_mutex_unlock(&g_lock);
                        }
                      else
                        {
                          pthread_mutex_lock(&g_lock);
                          g_status.dropped++;
                          pthread_mutex_unlock(&g_lock);
                        }
                    }
                }
            }
        }

      /* Flush at least a few times a second, so a short session or a sudden
       * power loss keeps most of what was recorded.
       */

      now = log_now_us();
      if (now - last_flush >= 250000)
        {
          if (!log_flush(false))
            {
              /* Stop rather than carry on. Anything written after a partial
               * flush lands against the wrong frame boundary, and the result
               * is a file that opens, looks plausible, and is wrong.
               */

              break;
            }

          last_flush = now;
        }

      /* Roll over to the next part once this one is big enough.
       *
       * The buffer is flushed and the file closed before the new one opens, so
       * a part on the card is always complete and readable on its own - which
       * is the point of splitting at all.
       */

      if (g_part_bytes >= LOG_PART_BYTES)
        {
          if (!log_flush(true))
            {
              break;
            }

          fsync(g_fd);
          close(g_fd);

          part++;
          g_fd = log_open_part(session, part, path, sizeof(path));

          if (g_fd < 0)
            {
              syslog(LOG_ERR, "logger: cannot open part %d: %d\n",
                     part, errno);
              break;
            }

          g_buflen     = 0;
          g_part_bytes = 0;

          log_write_header();
          log_write_formats();

          for (i = 0; i < nsubs; i++)
            {
              log_write_subscription(&subs[i]);
            }

          pthread_mutex_lock(&g_lock);
          strlcpy(g_status.path, path, sizeof(g_status.path));
          pthread_mutex_unlock(&g_lock);

          syslog(LOG_INFO, "logger: rolled to %s\n", path);
        }
    }

  /* Guarded: the loop can exit with no file open, if opening the next part
   * failed. Flushing to -1 would silently discard the buffer, and fsync/close
   * on -1 are simply errors.
   */

  if (g_fd >= 0)
    {
      log_flush(true);
      fsync(g_fd);
      close(g_fd);
      g_fd = -1;
    }

  for (i = 0; i < nsubs; i++)
    {
      orb_unsubscribe(subs[i].fd);
    }

  syslog(LOG_INFO, "logger: stopped (%" PRIu32 " samples, %" PRIu64 " bytes, "
                   "%" PRIu32 " dropped)\n",
         g_status.samples, g_status.bytes, g_status.dropped);

  g_running = false;
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int logger_start(void)
{
  int pid;
  int i;

  if (g_running)
    {
      return -EALREADY;
    }

  g_should_stop = false;
  memset(&g_status, 0, sizeof(g_status));

  /* task_create, NOT pthread_create. The logger has to outlive whatever started
   * it - typically the short-lived `log start` command - and a detached pthread
   * is a child of that command's task group, so it is killed the instant the
   * command returns (group_kill_children on task exit). A task is its own group
   * and survives. Same reason apps/px4io runs its daemon as a task.
   */

  pid = task_create("logger", LOG_PRIO, LOG_STACK, log_daemon, NULL);
  if (pid < 0)
    {
      return -errno;
    }

  /* Wait for it to open the file (or fail), so `log start` can report the real
   * outcome rather than a hopeful one.
   */

  for (i = 0; i < 100 && !g_running; i++)
    {
      usleep(10000);
    }

  return OK;
}

void logger_stop(void)
{
  int i;

  if (!g_running)
    {
      return;
    }

  g_should_stop = true;

  for (i = 0; i < 200 && g_running; i++)
    {
      usleep(10000);
    }
}

bool logger_is_running(void)
{
  return g_running;
}

int logger_get_status(FAR struct logger_status_s *status)
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
