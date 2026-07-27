/****************************************************************************
 * apps/logger/log_write.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Write a whole buffer to a file without trusting the returned byte count.
 *
 * NuttX's FAT driver can write part of a buffer and then report an error
 * instead of the partial count. fat_write() accumulates `byteswritten` and
 * advances filep->f_pos for every sector it lands, but its error path is
 *
 *     errout_with_lock:
 *       nxmutex_unlock(&fs->fs_lock);
 *       return ret;
 *
 * which discards `byteswritten` entirely (fs/fat/fs_fat32.c). So a caller sees
 * a negative errno while the file position has already moved, and has no way
 * to learn by how much.
 *
 * Both obvious strategies are wrong, and both were tried on real hardware:
 *
 *   drop the rest of the buffer   -> a torn record, and every byte after it in
 *                                    the file is framed against the wrong
 *                                    boundary
 *   retry from the same offset    -> the bytes that DID land get written twice,
 *                                    shifting the stream out of phase
 *
 * The fix is to stop guessing and ask: lseek(fd, 0, SEEK_CUR) reports where the
 * file actually is, so the amount truly written is the difference from where it
 * started. That is the one number nobody has to infer.
 ****************************************************************************/

#ifndef __APPS_LOGGER_LOG_WRITE_H
#define __APPS_LOGGER_LOG_WRITE_H

#include <sys/types.h>
#include <stdint.h>
#include <stddef.h>

#ifndef FAR
#  define FAR
#endif

/* Indirection so the retry logic can be exercised on the host against a stub
 * that reproduces FAT's behaviour. There is no other way to test the case that
 * matters: a write that half-succeeds and then lies about it.
 */

struct log_io_s
{
  ssize_t (*write)(int fd, FAR const void *buf, size_t len);
  off_t   (*lseek)(int fd, off_t offset, int whence);
  int     (*sleep_us)(unsigned us);
};

/* The default vtable, wired to the real system calls. */

FAR const struct log_io_s *log_io_default(void);

/* Write len bytes of buf to fd.
 *
 * Returns 0 when everything landed. On failure returns a negative errno and
 * sets *written to how many bytes actually reached the file - derived from the
 * file position, not from any returned count - so the caller knows whether the
 * file now ends on a record boundary or part way through one.
 *
 * max_spins bounds how long it will wait on a stalling card before giving up.
 */

int log_write_all(int fd, FAR const uint8_t *buf, size_t len,
                  FAR const struct log_io_s *io, int max_spins,
                  FAR size_t *written);

#endif /* __APPS_LOGGER_LOG_WRITE_H */
