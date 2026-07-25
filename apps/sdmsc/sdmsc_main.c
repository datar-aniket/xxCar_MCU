/****************************************************************************
 * apps/sdmsc/sdmsc_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * sdmsc - hand the microSD card to the USB host, or take it back.
 *
 * The USB device is a permanent CDC/ACM + Mass Storage composite, so the
 * serial data port is always present. This command only toggles which side
 * owns the SD card:
 *
 *   sdmsc on      unmount /fs/microsd locally, then export it to the host.
 *                 The card appears as a USB drive on Linux - edit params and
 *                 config there.
 *   sdmsc off     take the card back: unbind from USB, remount /fs/microsd.
 *   sdmsc status  show who currently owns the card.
 *
 * Exactly one side owns the card at any time - that is the point. Never let
 * NuttX and the host write the same FAT.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <arch/board/board.h>

#ifdef CONFIG_XXCAR_LOGGER
#  include "../logger/logger.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sdmsc_usage(void)
{
  printf("Usage: sdmsc <on|off|status>\n"
         "  on      unmount /fs/microsd and export the card to the USB host\n"
         "  off     reclaim the card from the host and remount /fs/microsd\n"
         "  status  show which side currently owns the card\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  if (argc != 2)
    {
      sdmsc_usage();
      return 1;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      printf("microSD owner: %s\n",
             fmuv6c_msc_is_exported() ? "USB host (exported)"
                                      : "board (/fs/microsd mounted)");
      return 0;
    }

  if (strcmp(argv[1], "on") == 0)
    {
#ifdef CONFIG_XXCAR_LOGGER
      /* The logger holds a file open on the card, which blocks the unmount that
       * export needs (nx_umount2 returns -EBUSY). Stop it first so the card can
       * be handed over cleanly - and so everything it wrote is flushed and the
       * host sees the finished .ulg.
       */

      if (logger_is_running())
        {
          printf("sdmsc: stopping the logger to release the card\n");
          logger_stop();
        }
#endif

      ret = fmuv6c_msc_export();
      if (ret < 0)
        {
          fprintf(stderr, "sdmsc: export failed: %d\n", ret);
          if (ret == -EBUSY)
            {
              fprintf(stderr,
                      "  something still has /fs/microsd open "
                      "(logger running? cwd inside it?)\n");
            }
          else if (ret == -ENODEV)
            {
              fprintf(stderr, "  USB not connected\n");
            }

          return 1;
        }

      printf("microSD exported to USB host. "
             "It should appear as a drive on the host.\n"
             "Run 'sdmsc off' to take it back "
             "(eject it on the host first).\n");
      return 0;
    }

  if (strcmp(argv[1], "off") == 0)
    {
      ret = fmuv6c_msc_release();
      if (ret < 0)
        {
          fprintf(stderr, "sdmsc: release failed: %d\n", ret);
          return 1;
        }

      printf("microSD reclaimed and remounted at /fs/microsd\n");
      return 0;
    }

  sdmsc_usage();
  return 1;
}
