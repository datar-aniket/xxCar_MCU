/****************************************************************************
 * apps/reboot/reboot_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `reboot` - restart the board, optionally into the bootloader.
 *
 *   reboot        ordinary restart
 *   reboot -b     restart and stay in the bootloader, so the next flash needs
 *                 no unplug/replug: the board comes up in px_uploader's
 *                 serial-upload window on its own.
 *
 * This replaces NuttX's builtin reboot (disabled in the defconfig) only to add
 * the -b flag; a plain `reboot` behaves exactly as before.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <sys/boardctl.h>

#include <arch/board/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int target = BOARD_REBOOT_NORMAL;

  if (argc > 1)
    {
      if (strcmp(argv[1], "-b") == 0)
        {
          target = BOARD_REBOOT_TO_BOOTLOADER;
          printf("reboot: entering the bootloader "
                 "(flash now; no replug needed)\n");
        }
      else
        {
          printf("Usage: reboot [-b]\n"
                 "  (no arg)  ordinary restart\n"
                 "  -b        restart into the bootloader for flashing\n");
          return 1;
        }
    }

  fflush(stdout);

  /* Does not return on success. */

  boardctl(BOARDIOC_RESET, target);

  fprintf(stderr, "reboot: boardctl(RESET) returned - reset not supported?\n");
  return 1;
}
