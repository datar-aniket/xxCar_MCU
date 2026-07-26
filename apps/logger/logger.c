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
#include "../param/param.h"
#include "../rc_in/rc_in.h"
#include "../uorb_msgs/uorb_msgs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LOG_STACK     4096
#define LOG_PRIO      (SCHED_PRIORITY_DEFAULT + 2)   /* below the sensors, so
                                                      * logging never delays a
                                                      * sample being produced */

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
 * A bigger buffer is only safe because log_flush() now writes all of it. While
 * a short write discarded the remainder, enlarging this made things worse
 * rather than better - see the comment there.
 */

#define LOG_BUFSIZE   65536

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

/* Largest record we serialise (rc_in, 56 bytes). */

#define LOG_RECMAX    64

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
  uint64_t last_us;   /* for LOG_RATE decimation */
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
  /* orb_name        direct_meta                 ulog_name         mid sz  param */
  { "sensor_accel0", NULL,                       "sensor_accel",    0, 24, "LOG_IMU0" },
  { "sensor_gyro0",  NULL,                       "sensor_gyro",     0, 24, "LOG_IMU0" },
  { "sensor_accel1", NULL,                       "sensor_accel",    1, 24, "LOG_IMU1" },
  { "sensor_gyro1",  NULL,                       "sensor_gyro",     1, 24, "LOG_IMU1" },
  { "sensor_mag0",   NULL,                       "sensor_mag",      0, 28, "LOG_MAG"  },
  { "sensor_baro0",  NULL,                       "sensor_baro",     0, 16, "LOG_BARO" },
  { NULL,            ORB_ID(rc_in),              "rc_input",        0, 53, "LOG_RC"   },
  { NULL,            ORB_ID(optical_flow),       "optical_flow",    0, 44, "LOG_FLOW" },
  { NULL,            ORB_ID(distance_sensor),    "distance_sensor", 0, 24, "LOG_DIST" },
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

/****************************************************************************
 * Private Functions - the buffered ULog writer
 ****************************************************************************/

static uint64_t log_now_us(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

/* Push the buffer to the card. Returns false if the write fell short - a full
 * or failing card - so the caller can count the loss.
 */

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
 * So: retry the remainder. A card that stalls for wear levelling comes back
 * within tens of milliseconds, and blocking the logger for that long is
 * cheaper than a corrupt file. If it truly will not take the data, say so and
 * let the caller end the session - a short recording beats a desynchronised
 * one.
 */

static bool log_flush(void)
{
  size_t off = 0;
  int spins = 0;

  if (g_buflen == 0)
    {
      return true;
    }

  while (off < g_buflen)
    {
      ssize_t n = write(g_fd, g_buf + off, g_buflen - off);

      if (n > 0)
        {
          off += (size_t)n;
          spins = 0;
          continue;
        }

      if (n < 0 && errno != EINTR && errno != EAGAIN)
        {
          break;                       /* a real error, not back-pressure */
        }

      /* Ten seconds of patience. Beyond that the card is not coming back and
       * waiting longer only delays the bad news.
       */

      if (++spins > 1000)
        {
          break;
        }

      usleep(10000);
    }

  pthread_mutex_lock(&g_lock);
  g_status.bytes += off;
  pthread_mutex_unlock(&g_lock);
  g_part_bytes += (uint32_t)off;

  if (off < g_buflen)
    {
      syslog(LOG_ERR,
             "logger: card took only %zu of %zu bytes - ending the session "
             "rather than writing a torn record\n", off, g_buflen);
      g_buflen = 0;
      return false;
    }

  g_buflen = 0;
  return true;
}

/* Append bytes, flushing first if they will not fit. */

static bool log_put(FAR const void *data, size_t len)
{
  if (len > LOG_BUFSIZE)
    {
      return false;
    }

  if (g_buflen + len > LOG_BUFSIZE && !log_flush())
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
      char fmt[256];
      int n = snprintf(fmt, sizeof(fmt), "%s:%s",
                       g_formats[i].name, g_formats[i].fields);

      if (!log_msg(ULOG_MSG_FORMAT, fmt, (uint16_t)n))
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
  uint8_t          rec[LOG_RECMAX];
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

      fd = orb_subscribe(meta);
      if (fd < 0)
        {
          continue;
        }

      subs[nsubs].topic   = t;
      subs[nsubs].meta    = meta;
      subs[nsubs].fd      = fd;
      subs[nsubs].msg_id  = (uint16_t)nsubs;
      subs[nsubs].last_us = 0;
      nsubs++;
    }

  if (nsubs == 0)
    {
      syslog(LOG_WARNING,
             "logger: nothing selected (set LOG_IMU0 / LOG_MAG / ...)\n");
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
              if ((pfd[i].revents & POLLIN) == 0)
                {
                  continue;
                }

              if (orb_copy(subs[i].meta, subs[i].fd, rec) < 0)
                {
                  continue;
                }

              /* The sample's own timestamp is its first 8 bytes. Use it for
               * decimation so the spacing is measured in sample time, not
               * loop-wakeup time.
               */

              if (min_interval > 0)
                {
                  uint64_t ts;

                  memcpy(&ts, rec, sizeof(ts));

                  if (ts - subs[i].last_us < min_interval)
                    {
                      continue;
                    }

                  subs[i].last_us = ts;
                }

              if (log_data(subs[i].msg_id, rec, subs[i].topic->rec_size))
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

      /* Flush at least a few times a second, so a short session or a sudden
       * power loss keeps most of what was recorded.
       */

      now = log_now_us();
      if (now - last_flush >= 250000)
        {
          if (!log_flush())
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
          if (!log_flush())
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
      log_flush();
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
