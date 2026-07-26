/****************************************************************************
 * apps/cal/cal_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * cal - sensor calibration.
 *
 *   cal status    show the stored calibration for every sensor
 *   cal session   open the USB port and let the calibration GUI drive
 *
 * Run this from the shell on TELEM1/DEBUG, not from a shell on USB: the session
 * takes the USB port, and a shell sitting on the same port would race it.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>

#include "cal.h"

static void cal_usage(void)
{
  printf("Usage: cal <status|session>\n"
         "  status   show stored calibration for every sensor\n"
         "  session  open %s for the calibration GUI\n"
         "\n"
         "Run from the TELEM1/DEBUG shell. USB needs SER_USB_FUNC=5 (CAL)\n"
         "so no shell is started there.\n", CAL_DEVPATH);
}

int main(int argc, FAR char *argv[])
{
  if (argc != 2)
    {
      cal_usage();
      return 1;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      return cal_print_status() < 0 ? 1 : 0;
    }

  if (strcmp(argv[1], "session") == 0)
    {
      return cal_session() < 0 ? 1 : 0;
    }

  cal_usage();
  return 1;
}
