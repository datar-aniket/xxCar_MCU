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
  int spins = 0;

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

  for (;;)
    {
      off_t now;
      size_t done;
      ssize_t n = io->write(fd, buf, len);
      int write_errno = n < 0 ? errno : 0;

      /* Always verify the position, including after an apparently complete
       * write. The hardware failure this protects against is precisely a
       * disagreement between the returned count and the FAT file position.
       */

      now = io->lseek(fd, 0, SEEK_CUR);
      if (now < 0)
        {
          return -EIO;
        }

      if (now < start)
        {
          return -EIO;
        }

      done = (size_t)(now - start);
      if (done > len)
        {
          return -EIO;
        }

      if (written != NULL)
        {
          *written = done;
        }

      if (n == (ssize_t)len && done == len)
        {
          return 0;
        }

      /* Once any part of the request may have landed, its exact boundary is
       * ambiguous. Real recordings have shown the FAT position disagree with
       * the data by one byte: resuming at that reported position inserted or
       * deleted a byte and made the rest of the ULog unreadable. Do not resume.
       * The caller truncates the file back to the pre-flush boundary.
       */

      if (done != 0 || n > 0)
        {
          return -EIO;
        }

      /* With zero progress it is safe to retry the original buffer. lseek()
       * may change errno, so classify the write using the saved value.
       */

      if (n < 0 && write_errno != EINTR && write_errno != EAGAIN)
        {
          return write_errno != 0 ? -write_errno : -EIO;
        }

      if (++spins > max_spins)
        {
          return -ETIMEDOUT;
        }

      io->sleep_us(10000);
    }
}
