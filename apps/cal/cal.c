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

#include "cal.h"
#include "../imu_cal/imu_cal.h"
#include "../param/param.h"

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

  /* Task 4 replaces this with the protocol event loop. */

  ret = OK;

  tcsetattr(fd, TCSANOW, &saved);
  close(fd);
  return ret;
}
