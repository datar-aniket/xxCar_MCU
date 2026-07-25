/****************************************************************************
 * apps/mavlink/mavlink.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MAVLink daemon. See mavlink.h.
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

/* The generated MAVLink codec. It is full of casts through packed structs, so
 * the warnings it trips are the library's, not ours - quiet just this include.
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include <mavlink.h>
#pragma GCC diagnostic pop

#include "mavlink.h"
#include "../param/param.h"
#include "../uorb_msgs/uorb_msgs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAV_STACK  4096   /* the codec builds messages on the stack, and this
                           * task also does uORB publishes and param lookups */
#define MAV_PRIO   (SCHED_PRIORITY_DEFAULT + 4)

#define MAV_CHAN   MAVLINK_COMM_0

/* PARAM_VALUE messages to emit per loop iteration while streaming the whole
 * table. The loop runs at ~50 Hz, so this is a few thousand params/sec - the
 * table is tens of params, streamed in a fraction of a second - without ever
 * hogging the link away from RX.
 */

#define MAV_PARAM_BURST  8

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_mutex_t   g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool     g_running;
static volatile bool     g_should_stop;

static char              g_devpath[16];
static int32_t           g_baud;

static struct mavlink_status_s g_status;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t mav_now_us(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

/* Serialise a built message and write it. */

static void mav_send(int fd, FAR const mavlink_message_t *msg)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

  if (write(fd, buf, len) == (ssize_t)len)
    {
      g_status.tx_frames++;
    }
}

static void mav_send_heartbeat(int fd, uint8_t sysid, uint8_t compid)
{
  mavlink_message_t msg;

  /* A ground rover running its own (generic) stack. base_mode/custom_mode are
   * left at 0: we are not pretending to be an ArduPilot/PX4 flight mode.
   */

  mavlink_msg_heartbeat_pack(sysid, compid, &msg,
                             MAV_TYPE_GROUND_ROVER,
                             MAV_AUTOPILOT_GENERIC,
                             0, 0, MAV_STATE_ACTIVE);
  mav_send(fd, &msg);
}

/* Send one parameter as PARAM_VALUE.
 *
 * MAVLink carries every value in a float32 field. We use the "bytewise"
 * convention (as PX4 and ArduPilot do, and QGroundControl expects): an int32 is
 * reinterpreted into those 4 bytes rather than numerically cast, so large
 * integers survive intact. param_type tells the other end which it is.
 */

static void mav_send_param(int fd, uint8_t sysid, uint8_t compid, int index)
{
  FAR const struct param_def_s *d = param_def(index);
  mavlink_message_t msg;
  char name[16];
  union
  {
    float   f;
    int32_t i;
  } u;
  uint8_t type;

  if (d == NULL)
    {
      return;
    }

  /* param_id is 16 chars, NUL-padded only if shorter than 16. strncpy gives
   * exactly that.
   */

  strncpy(name, d->name, sizeof(name));

  if (d->type == PARAM_TYPE_INT32)
    {
      u.i  = param_i32(d->name);
      type = MAV_PARAM_TYPE_INT32;
    }
  else
    {
      u.f  = param_f32(d->name);
      type = MAV_PARAM_TYPE_REAL32;
    }

  mavlink_msg_param_value_pack(sysid, compid, &msg,
                               name, u.f, type,
                               (uint16_t)param_count(),
                               (uint16_t)index);
  mav_send(fd, &msg);
  g_status.param_tx++;
}

/* PARAM_SET: write one parameter and echo it back as PARAM_VALUE (the ack the
 * protocol expects).
 */

static void mav_handle_param_set(int fd, FAR const mavlink_message_t *msg)
{
  mavlink_param_set_t set;
  char name[17];
  int index;
  union
  {
    float   f;
    int32_t i;
  } u;

  mavlink_msg_param_set_decode(msg, &set);

  /* param_id may not be NUL-terminated if it is a full 16 chars. */

  memcpy(name, set.param_id, 16);
  name[16] = '\0';

  index = param_find(name);
  if (index < 0)
    {
      return;   /* unknown parameter: silently ignore, as GCS clients expect */
    }

  u.f = set.param_value;

  if (param_def(index)->type == PARAM_TYPE_INT32)
    {
      param_set_i32(name, (set.param_type == MAV_PARAM_TYPE_INT32) ?
                          u.i : (int32_t)set.param_value);
    }
  else
    {
      param_set_f32(name, (set.param_type == MAV_PARAM_TYPE_REAL32) ?
                          u.f : (float)u.i);
    }

  /* Ack with the value that actually stuck (it may have been clamped). Reply to
   * the sender.
   */

  mav_send_param(fd, g_status.sysid, g_status.compid, index);
}

static void mav_handle_param_request_read(int fd,
                                          FAR const mavlink_message_t *msg)
{
  mavlink_param_request_read_t req;
  int index;

  mavlink_msg_param_request_read_decode(msg, &req);

  if (req.param_index >= 0)
    {
      index = req.param_index;
    }
  else
    {
      char name[17];

      memcpy(name, req.param_id, 16);
      name[16] = '\0';
      index = param_find(name);
    }

  if (index >= 0 && index < param_count())
    {
      mav_send_param(fd, g_status.sysid, g_status.compid, index);
    }
}

/* Decode the MTF-02's two messages into uORB. */

static void mav_handle_optical_flow(FAR const mavlink_message_t *msg,
                                    int of_fd)
{
  mavlink_optical_flow_rad_t f;
  struct optical_flow_s out;

  if (of_fd < 0)
    {
      return;
    }

  mavlink_msg_optical_flow_rad_decode(msg, &f);

  memset(&out, 0, sizeof(out));
  out.timestamp              = mav_now_us();
  out.integration_time_us    = f.integration_time_us;
  out.time_delta_distance_us = f.time_delta_distance_us;
  out.integrated_x           = f.integrated_x;
  out.integrated_y           = f.integrated_y;
  out.integrated_xgyro       = f.integrated_xgyro;
  out.integrated_ygyro       = f.integrated_ygyro;
  out.integrated_zgyro       = f.integrated_zgyro;
  out.distance               = f.distance;
  out.temperature            = f.temperature;
  out.quality                = f.quality;
  out.sensor_id              = f.sensor_id;

  optical_flow_publish(of_fd, &out);
  g_status.flow_msgs++;
}

static void mav_handle_distance(FAR const mavlink_message_t *msg, int ds_fd)
{
  mavlink_distance_sensor_t d;
  struct distance_sensor_s out;

  if (ds_fd < 0)
    {
      return;
    }

  mavlink_msg_distance_sensor_decode(msg, &d);

  memset(&out, 0, sizeof(out));
  out.timestamp        = mav_now_us();

  /* MAVLink DISTANCE_SENSOR is in centimetres; we publish metres so a consumer
   * never has to track which unit a given source used.
   */

  out.current_distance = d.current_distance * 0.01f;
  out.min_distance     = d.min_distance * 0.01f;
  out.max_distance     = d.max_distance * 0.01f;
  out.type             = d.type;
  out.orientation      = d.orientation;
  out.covariance       = d.covariance;
  out.signal_quality   = d.signal_quality;

  distance_sensor_publish(ds_fd, &out);
  g_status.dist_msgs++;
}

/****************************************************************************
 * The daemon
 ****************************************************************************/

static FAR void *mav_thread(FAR void *arg)
{
  mavlink_message_t msg;
  mavlink_status_t  parse;
  struct pollfd     pfd;
  uint64_t          last_hb = 0;
  bool              stream_active = false;
  int               stream_index = 0;
  int               of_fd;
  int               ds_fd;
  int               fd;

  UNUSED(arg);

  fd = open(g_devpath, O_RDWR | O_NOCTTY);
  if (fd < 0)
    {
      syslog(LOG_ERR, "mavlink: cannot open %s: %d\n", g_devpath, errno);
      g_running = false;
      return NULL;
    }

  /* Raw 8N1 at the requested baud. The framing lives entirely in the MAVLink
   * bytes, so the tty must not touch them - no canonical mode, no CR/LF
   * mangling, no parity.
   */

  {
    struct termios tio;

    if (tcgetattr(fd, &tio) == 0)
      {
        cfmakeraw(&tio);
        tio.c_cflag &= ~(CSTOPB | PARENB | CSIZE);
        tio.c_cflag |= CS8;

        if (g_baud > 0)
          {
            cfsetspeed(&tio, g_baud);
          }

        tcsetattr(fd, TCSANOW, &tio);
      }
  }

  of_fd = optical_flow_advertise();
  ds_fd = distance_sensor_advertise();

  pfd.fd     = fd;
  pfd.events = POLLIN;

  g_running = true;

  while (!g_should_stop)
    {
      uint64_t now;

      if (poll(&pfd, 1, 20) > 0 && (pfd.revents & POLLIN) != 0)
        {
          uint8_t buf[128];
          ssize_t n = read(fd, buf, sizeof(buf));
          ssize_t i;

          for (i = 0; i < n; i++)
            {
              if (!mavlink_parse_char(MAV_CHAN, buf[i], &msg, &parse))
                {
                  continue;
                }

              pthread_mutex_lock(&g_lock);
              g_status.rx_frames++;
              g_status.peer_seen  = true;
              g_status.peer_sysid = msg.sysid;

              switch (msg.msgid)
                {
                  case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
                    mav_handle_optical_flow(&msg, of_fd);
                    break;

                  case MAVLINK_MSG_ID_DISTANCE_SENSOR:
                    mav_handle_distance(&msg, ds_fd);
                    break;

                  case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                    stream_active = true;
                    stream_index  = 0;
                    break;

                  case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                    mav_handle_param_request_read(fd, &msg);
                    break;

                  case MAVLINK_MSG_ID_PARAM_SET:
                    mav_handle_param_set(fd, &msg);
                    break;

                  default:
                    /* A frame we do not act on. It still decoded, which is the
                     * point of carrying the full codec - a new sensor can be
                     * added by handling one more case, not by re-deriving the
                     * wire format.
                     */

                    break;
                }

              pthread_mutex_unlock(&g_lock);
            }
        }

      /* Track parser drops for `mav status`. */

      pthread_mutex_lock(&g_lock);
      g_status.rx_dropped = parse.packet_rx_drop_count;
      pthread_mutex_unlock(&g_lock);

      now = mav_now_us();

      if (now - last_hb >= 1000000)
        {
          mav_send_heartbeat(fd, g_status.sysid, g_status.compid);
          last_hb = now;
        }

      /* Feed the parameter stream a burst at a time, so a full-table read never
       * starves RX.
       */

      if (stream_active)
        {
          int sent;

          for (sent = 0; sent < MAV_PARAM_BURST &&
                         stream_index < param_count(); sent++)
            {
              mav_send_param(fd, g_status.sysid, g_status.compid,
                             stream_index++);
            }

          if (stream_index >= param_count())
            {
              stream_active = false;
            }
        }
    }

  close(fd);
  g_running = false;
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mavlink_start(FAR const char *devpath, int32_t baud)
{
  pthread_attr_t attr;
  struct sched_param sparam;
  pthread_t tid;
  int ret;

  if (g_running)
    {
      return -EALREADY;
    }

  if (devpath == NULL)
    {
      return -EINVAL;
    }

  strlcpy(g_devpath, devpath, sizeof(g_devpath));
  g_baud        = baud;
  g_should_stop = false;

  memset(&g_status, 0, sizeof(g_status));
  strlcpy(g_status.devpath, devpath, sizeof(g_status.devpath));
  g_status.baud   = baud;
  g_status.sysid  = (uint8_t)param_i32("MAV_SYS_ID");
  g_status.compid = (uint8_t)param_i32("MAV_COMP_ID");

  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, MAV_STACK);
  sparam.sched_priority = MAV_PRIO;
  pthread_attr_setschedparam(&attr, &sparam);

  g_running = true;
  ret = pthread_create(&tid, &attr, mav_thread, NULL);
  pthread_attr_destroy(&attr);

  if (ret != 0)
    {
      g_running = false;
      return -ret;
    }

  pthread_setname_np(tid, "mavlink");
  pthread_detach(tid);

  return OK;
}

void mavlink_stop(void)
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

bool mavlink_is_running(void)
{
  return g_running;
}

int mavlink_get_status(FAR struct mavlink_status_s *status)
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
