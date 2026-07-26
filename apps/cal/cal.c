/****************************************************************************
 * apps/cal/cal.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See cal.h.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <inttypes.h>
#include <poll.h>
#include <stdbool.h>
#include <math.h>

#include <uORB/uORB.h>

#include "cal.h"
#include "cal_proto.h"
#include "cal_still.h"
#include "../imu_cal/imu_cal.h"
#include "../param/param.h"
#include "../serial/serial.h"

/* CAL_PROTO_NAME_MAX is duplicated in cal_proto.h rather than including
 * param.h there, so the protocol builds standalone in the host test. This
 * file includes both, so it is the one place that can catch the two
 * silently drifting apart - a name that fits the protocol's buffer but not
 * the parameter store's (or vice versa) would truncate or misaddress
 * instead of failing loudly.
 */

#if CAL_PROTO_NAME_MAX != PARAM_NAME_MAX
#  error "CAL_PROTO_NAME_MAX (cal_proto.h) must match PARAM_NAME_MAX (param.h)"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *const g_sensor_name[IMU_CAL_NSENSORS] =
{
  "accel0", "gyro0 ", "accel1", "gyro1 "
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Write a complete event line to the (non-blocking) session fd, retrying
 * through EAGAIN and a short count instead of trusting a bare write().
 *
 * fd is O_NONBLOCK and the CDC TX ring (CDCACM_TXBUFSIZE) is finite, so a
 * host that is not draining it - GUI hung, USB stalled - makes write() return
 * -EAGAIN (the whole line silently vanishes) or a short count (the host gets
 * a truncated JSON line and desynchronises, since every reply here is
 * supposed to be exactly one line). Retrying is bounded, not indefinite: a
 * host that has genuinely stopped reading must not wedge the session, so
 * persistent failure ends it instead of looping forever.
 */

static int cal_write_all(int fd, FAR const char *buf, size_t len)
{
  size_t off = 0;
  int stalled = 0;

  while (off < len)
    {
      ssize_t got = write(fd, buf + off, len - off);

      if (got > 0)
        {
          off += (size_t)got;
          stalled = 0;
          continue;
        }

      if (got < 0 && errno == EAGAIN)
        {
          if (++stalled > 200)          /* ~200ms of a fully backed-up ring */
            {
              return -ETIMEDOUT;
            }

          usleep(1000);
          continue;
        }

      return got < 0 ? -errno : -EIO;
    }

  return OK;
}

/* Put the port in raw mode and hand back the previous settings so the caller
 * can restore them. The protocol frames itself; canonical mode would buffer by
 * line, echo would feed our own output back to us, and CR/LF translation would
 * corrupt binary frames.
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

  if (tcsetattr(fd, TCSANOW, &raw) < 0)
    {
      return -errno;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cal_print_status(void)
{
  int32_t mode = param_i32("CAL_MODE");
  int i;

  printf("stored IMU calibration (CAL_MODE=%" PRId32 ", %s)\n",
         mode,
         mode == 1 ? "jig: alignment observable"
                   : "desk: norm-only, alignment NOT observable");

  for (i = 0; i < IMU_CAL_NSENSORS; i++)
    {
      struct imu_cal_s cal;

      /* imu_cal_load() only fails on an out-of-range sensor id, which cannot
       * happen here - i is bounded by IMU_CAL_NSENSORS, the same enum this
       * loop walks. Kept anyway: it is a one-line guard against this loop
       * being copied somewhere its bound and the enum have drifted apart.
       */

      if (imu_cal_load((enum imu_cal_sensor_e)i, &cal) < 0)
        {
          continue;
        }

      if (!cal.valid)
        {
          printf("  %s  uncalibrated (raw passthrough)\n", g_sensor_name[i]);
          continue;
        }

      printf("  %s  bias % .5f % .5f % .5f\n",
             g_sensor_name[i], cal.b[0], cal.b[1], cal.b[2]);
      printf("          diag % .5f % .5f % .5f\n",
             cal.M[0], cal.M[4], cal.M[8]);
    }

  return OK;
}

int cal_session(void)
{
  struct termios saved;
  int ret;
  int fd;

  /* The kernel gives us nothing to lean on here: /dev/ttyACM0's open() is
   * refcounted, not exclusive (any number of opens succeed), TIOCEXCL is
   * declared but not implemented for this port, and termios state lives on
   * the shared uart_dev_t rather than per-fd - so if a shell is already on
   * this port, opening it out from under it and calling cfmakeraw() would
   * silently strip ICANON/ECHO from that shell's line discipline and the two
   * would race for every input byte with no diagnostic anywhere. This check
   * is therefore the ONLY thing standing between an operator who forgets to
   * set SER_USB_FUNC first and that exact race - not a courtesy, load-bearing.
   */

  if (param_i32("SER_USB_FUNC") != SER_FUNC_CAL)
    {
      fprintf(stderr,
              "cal: %s is not reserved for calibration (SER_USB_FUNC != CAL).\n"
              "  Set SER_USB_FUNC=5 (CAL) and reboot, so no shell is\n"
              "  started on it - a shell there races the GUI for input.\n",
              CAL_DEVPATH);
      return -EBUSY;
    }

  /* The parameter check above is necessary but not sufficient: SER_USB_FUNC
   * is read live, but the NSH task on this port (if any) was started once at
   * boot from whatever that parameter said THEN, and then runs - or, being
   * removable, keeps re-arming after every unplug - until the board reboots.
   * An operator who runs `param set SER_USB_FUNC 5` without rebooting passes
   * the check above while that boot-time shell is still very much alive,
   * still looping open()/read() on this exact device every 250ms. The two
   * would then race for every byte with no diagnostic anywhere. Rebooting is
   * the only real fix, so that is what this tells the operator to do.
   */

  {
    int usb_port = serial_find("USB");

    if (usb_port >= 0 && serial_port_has_nsh(usb_port))
      {
        fprintf(stderr,
                "cal: a shell is still attached to %s from before\n"
                "  SER_USB_FUNC was set. Reboot the board so no NSH task is\n"
                "  left racing the GUI for input on this port.\n",
                CAL_DEVPATH);
        return -EBUSY;
      }
  }

  /* O_NONBLOCK on open: the CDC port only exists while a host is attached, and
   * a blocking open would hang the shell until someone plugged in a cable.
   */

  fd = open(CAL_DEVPATH, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    {
      if (errno == ENOTCONN)
        {
          fprintf(stderr, "cal: no USB host attached to %s\n", CAL_DEVPATH);
          return -ENOTCONN;
        }

      if (errno == EBUSY)
        {
          fprintf(stderr,
                  "cal: %s is already in use.\n"
                  "  Set SER_USB_FUNC=5 (CAL) and reboot, so no shell is\n"
                  "  started on it - a shell there races the GUI for input.\n",
                  CAL_DEVPATH);
          return -EBUSY;
        }

      fprintf(stderr, "cal: cannot open %s: %d\n", CAL_DEVPATH, errno);
      return -errno;
    }

  ret = cal_raw_mode(fd, &saved);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  printf("cal: session open on %s - drive it from the host\n", CAL_DEVPATH);

  /* Read a line at a time. poll() rather than a blocking read so a cable pull
   * (POLLHUP) ends the session instead of hanging the shell forever.
   */

  {
    char line[96];
    char evt[256];
    size_t fill = 0;
    bool done = false;
    bool overlong = false;

    FAR const struct orb_metadata *acc_meta;
    FAR const struct orb_metadata *gyr_meta;
    int acc_fd;
    int gyr_fd;

    /* A missing sensor must not crash the session - get/set/commit still
     * work without one. CAPTURE alone needs both fds and reports an error
     * event if either is unavailable.
     */

    acc_meta = orb_get_meta("sensor_accel0");
    acc_fd   = acc_meta != NULL ? orb_subscribe(acc_meta) : -1;

    gyr_meta = orb_get_meta("sensor_gyro0");
    gyr_fd   = gyr_meta != NULL ? orb_subscribe(gyr_meta) : -1;

    ret = OK;

    while (!done)
      {
        struct pollfd pfd;
        char ch;
        ssize_t got;

        pfd.fd     = fd;
        pfd.events = POLLIN;

        if (poll(&pfd, 1, 500) <= 0)
          {
            continue;
          }

        if ((pfd.revents & (POLLHUP | POLLERR)) != 0)
          {
            /* Cable pulled. Nothing has been written to the parameter store,
             * so there is nothing to unwind.
             */

            ret = -ENOTCONN;
            break;
          }

        got = read(fd, &ch, 1);
        if (got <= 0)
          {
            continue;
          }

        if (fill >= sizeof(line) - 1 && ch != '\n')
          {
            /* Buffer is full and the line has not ended yet. Drop bytes until
             * the real newline instead of quietly dispatching the truncated
             * prefix as a complete command and the remainder as a second one
             * - that would fragment one over-long line into two commands
             * with a byte silently missing at the seam.
             */

            overlong = true;
            continue;
          }

        if (ch != '\n')
          {
            line[fill++] = ch;
            continue;
          }

        line[fill] = '\0';
        fill = 0;

        {
          struct cal_cmd_s c;
          int n = 0;

          if (overlong)
            {
              /* One error for the whole oversized line, then resynchronise
               * on the next line.
               */

              overlong = false;
              n = cal_proto_error(evt, sizeof(evt), "line too long");
            }
          else
            {
              cal_proto_parse(line, &c);

              switch (c.cmd)
                {
                  case CAL_CMD_NONE:
                    continue;

                  case CAL_CMD_HELLO:
                    n = cal_proto_hello(evt, sizeof(evt));
                    break;

                  case CAL_CMD_QUIT:
                  case CAL_CMD_ABORT:
                    n = cal_proto_ok(evt, sizeof(evt), "bye");
                    done = true;
                    break;

                  case CAL_CMD_CAPTURE:
                    if (acc_fd < 0 || gyr_fd < 0)
                      {
                        n = cal_proto_error(evt, sizeof(evt),
                                            "sensor not available");
                      }
                    else
                      {
                        struct cal_still_s still;
                        struct sensor_accel a;
                        struct sensor_gyro  g;
                        float acc[3];
                        float gyr[3];
                        int   waited = 0;

                        /* 0.02 rad/s is about a degree per second - below
                         * what a hand on a bench can hold steady, above the
                         * gyro's own noise. This loop usleep(1000)s between
                         * samples, i.e. it samples at ~1kHz, so the
                         * 500-sample window is ~500ms, not a quarter second
                         * at 2kHz.
                         */

                        cal_still_reset(&still, 0.02f, 0.05f, 500);

                        while (waited < 10000)
                          {
                            if (orb_copy(acc_meta, acc_fd, &a) < 0 ||
                                orb_copy(gyr_meta, gyr_fd, &g) < 0)
                              {
                                usleep(1000);
                                waited++;
                                continue;
                              }

                            acc[0] = a.x; acc[1] = a.y; acc[2] = a.z;
                            gyr[0] = g.x; gyr[1] = g.y; gyr[2] = g.z;

                            if (cal_still_update(&still, acc, gyr))
                              {
                                break;
                              }

                            usleep(1000);
                            waited++;
                          }

                        if (waited >= 10000)
                          {
                            n = cal_proto_error(evt, sizeof(evt),
                                    "still not steady - hold it and retry");
                          }
                        else
                          {
                            float macc[3];
                            float mgyr[3];

                            cal_still_mean(&still, macc, mgyr);
                            n = cal_proto_captured(evt, sizeof(evt),
                                                   still.count, macc, mgyr,
                                                   a.temperature);
                          }
                      }
                    break;

                  case CAL_CMD_GET:
                    {
                      int idx = param_find(c.name);
                      FAR const struct param_def_s *def = param_def(idx);

                      /* param_f32()/param_i32() fall back to a default value
                       * for an unknown name (0.0 for f32) and, on a type
                       * mismatch, reinterpret the OTHER union member's raw
                       * bits - neither is a real reading, and both would
                       * come back wrapped in an "ok" event indistinguishable
                       * from a genuine one. Look the name up and format by
                       * its actual type instead of guessing.
                       */

                      if (def == NULL)
                        {
                          n = cal_proto_error(evt, sizeof(evt),
                                              "no such parameter");
                        }
                      else
                        {
                          char msg[64];

                          if (def->type == PARAM_TYPE_INT32)
                            {
                              snprintf(msg, sizeof(msg), "%s=%" PRId32,
                                       c.name, param_i32(c.name));
                            }
                          else
                            {
                              snprintf(msg, sizeof(msg), "%s=%.6f", c.name,
                                       (double)param_f32(c.name));
                            }

                          n = cal_proto_ok(evt, sizeof(evt), msg);
                        }
                    }
                    break;

                  case CAL_CMD_SET:
                    {
                      int idx = param_find(c.name);
                      FAR const struct param_def_s *def = param_def(idx);

                      /* Three distinguishable failures, not one. Before this,
                       * every one of them - an unknown name, a name that
                       * exists but is the wrong type for param_set_f32(), and
                       * a value outside the parameter's bounds - collapsed
                       * onto "no such parameter", which hid F-1 (an INT32
                       * calibration flag could never be set at all) and F-4
                       * (an out-of-range value was clamped and applied to RAM
                       * while being reported as if nothing had been touched).
                       */

                      if (def == NULL)
                        {
                          n = cal_proto_error(evt, sizeof(evt),
                                              "no such parameter");
                        }
                      else if (def->type == PARAM_TYPE_INT32)
                        {
                          /* lroundf, not a cast: the host GUI has no reason
                           * to send an INT32 parameter as anything but a
                           * whole number, but round-to-nearest is the honest
                           * way to turn whatever float arrived into the
                           * integer that's actually stored, rather than
                           * truncating toward zero. (lrintf is declared in
                           * this NuttX tree's math.h but not linked into
                           * libm; lroundf is, and round-half-away-from-zero
                           * vs. lrintf's round-half-to-even makes no
                           * difference for values that are meant to already
                           * be integral.)
                           */

                          int32_t ival = (int32_t)lroundf(c.fval);

                          if (ival < def->min.i || ival > def->max.i)
                            {
                              n = cal_proto_error(evt, sizeof(evt),
                                                  "value out of range");
                            }
                          else
                            {
                              param_set_i32(c.name, ival);
                              n = cal_proto_ok(evt, sizeof(evt), "set");
                            }
                        }
                      else
                        {
                          if (c.fval < def->min.f || c.fval > def->max.f)
                            {
                              n = cal_proto_error(evt, sizeof(evt),
                                                  "value out of range");
                            }
                          else
                            {
                              param_set_f32(c.name, c.fval);
                              n = cal_proto_ok(evt, sizeof(evt), "set");
                            }
                        }
                    }
                    break;

                  case CAL_CMD_COMMIT:

                    /* One write, at the end. The USB port dies on cable
                     * pull, and a half-written calibration is worse than
                     * none.
                     */

                    n = param_save() < 0
                          ? cal_proto_error(evt, sizeof(evt),
                                            "param save failed")
                          : cal_proto_ok(evt, sizeof(evt), "committed");
                    break;

                  default:
                    n = cal_proto_error(evt, sizeof(evt), "unknown command");
                    break;
                }
            }

          if (n > 0)
            {
              if (cal_write_all(fd, evt, (size_t)n) < 0)
                {
                  /* The host is not draining the port. Continuing would just
                   * mean replying to further commands that also cannot be
                   * delivered - end the session instead of spinning blind.
                   * Nothing has been written to the parameter store here
                   * that needs unwinding.
                   */

                  ret = -EIO;
                  done = true;
                }
            }
        }
      }

    if (acc_fd >= 0)
      {
        orb_unsubscribe(acc_fd);
      }

    if (gyr_fd >= 0)
      {
        orb_unsubscribe(gyr_fd);
      }
  }

  tcsetattr(fd, TCSANOW, &saved);
  close(fd);
  return ret;
}
