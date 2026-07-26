/****************************************************************************
 * apps/cal/cal_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * cal - sensor calibration session for the host GUI.
 *
 * Run this from the shell on TELEM1/DEBUG. It takes over the USB CDC port,
 * which is where the GUI is listening, so running it from a shell on USB would
 * mean the shell and the session both reading the same bytes.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>

#include "cal.h"
#include "../param/param.h"      /* SER_FUNC_CAL, for the usage hint */

int main(int argc, FAR char *argv[])
{
  if (argc == 2 && strcmp(argv[1], "session") == 0)
    {
      return cal_session() < 0 ? 1 : 0;
    }

  printf("Usage: cal session\n"
         "  Opens %s and waits for the calibration GUI.\n"
         "  Needs SER_USB_FUNC=%d and a reboot, so no shell holds the port.\n",
         CAL_DEVPATH, SER_FUNC_CAL);
  return 1;
}
