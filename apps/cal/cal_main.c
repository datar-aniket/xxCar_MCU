/****************************************************************************
 * apps/cal/cal_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * cal - sensor calibration session for the host GUI.
 *
 * Run this from a shell on a port other than the USB CDC port reserved for
 * CAL. The default is NSH on USB0 and calibration on USB1.
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
         "  Opens the USB port assigned to CAL and waits for the GUI.\n"
         "  Default: USB1 (/dev/ttyACM1), SER_USB2_FUNC=%d.\n"
         "  Reboot after changing the assignment so no shell holds it.\n",
         SER_FUNC_CAL);
  return 1;
}
