/****************************************************************************
 * apps/logger/log_write.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See log_write.h.
 ****************************************************************************/

#ifndef LOG_WRITE_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <unistd.h>
#include <errno.h>

#include "log_write.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int log_sleep_us(unsigned us)
{
  return usleep(us);
}

static const struct log_io_s g_real_io =
{
  .write    = write,
  .lseek    = lseek,
  .sleep_us = log_sleep_us,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR const struct log_io_s *log_io_default(void)
{
  return &g_real_io;
}

int log_write_all(int fd, FAR const uint8_t *buf, size_t len,
                  FAR const struct log_io_s *io, int max_spins,
                  FAR size_t *written)
{
  off_t start;
  size_t off = 0;
  int spins = 0;
  int err = -EIO;

  if (written != NULL)
    {
      *written = 0;
    }

  if (len == 0)
    {
      return 0;
    }

  /* Where the file is before we touch it. Everything below is measured
   * against this rather than against what write() claims.
   */

  start = io->lseek(fd, 0, SEEK_CUR);
  if (start < 0)
    {
      return -errno;
    }

  while (off < len)
    {
      off_t now;
      size_t done;
      ssize_t n = io->write(fd, buf + off, len - off);
      int write_errno = n < 0 ? errno : 0;

      if (n == (ssize_t)(len - off))
        {
          off = len;                     /* the whole remainder landed */
          break;
        }

      /* Anything else - a short count OR an error - means the returned value
       * cannot be trusted to describe what reached the file. Ask the file.
       */

      now = io->lseek(fd, 0, SEEK_CUR);
      if (now < start)
        {
          err = -EIO;                    /* position went backwards; give up */
          break;
        }

      done = (size_t)(now - start);
      if (done > len)
        {
          err = -EIO;                    /* more than we asked for; nonsense */
          break;
        }

      if (done > off)
        {
          /* Real progress, however it was reported. Carry on from where the
           * file actually is - never from where we assumed it would be.
           */

          off = done;
          spins = 0;
          continue;
        }

      /* No progress at all. A card stalling for wear levelling comes back
       * within tens of milliseconds; a card that never does is not going to.
       */

      /* lseek() above is allowed to change errno. Classify the write using
       * the errno captured immediately after it, never the current errno.
       */

      if (n < 0 && write_errno != EINTR && write_errno != EAGAIN)
        {
          err = write_errno != 0 ? -write_errno : -EIO;
          break;
        }

      if (++spins > max_spins)
        {
          err = -ETIMEDOUT;
          break;
        }

      io->sleep_us(10000);
    }

  if (written != NULL)
    {
      *written = off;
    }

  return off == len ? 0 : err;
}
