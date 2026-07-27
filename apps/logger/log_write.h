/****************************************************************************
 * apps/logger/log_write.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Write one whole buffer to a file and reject ambiguous partial progress.
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
 * All attempts to continue an ambiguous partial write are unsafe:
 *
 *   drop the rest of the buffer   -> a torn record, and every byte after it in
 *                                    the file is framed against the wrong
 *                                    boundary
 *   retry from the same offset    -> the bytes that DID land get written twice,
 *                                    shifting the stream out of phase
 *   resume from reported f_pos    -> real hardware has reported a position one
 *                                    byte before or after the data boundary,
 *                                    inserting or deleting one byte
 *
 * The only safe recovery is for the caller to truncate to the position before
 * the flush and stop recording. A zero-progress EINTR/EAGAIN/stall is safe to
 * retry because no boundary needs to be inferred.
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

/* Write len bytes of buf to fd as one indivisible application-level request.
 *
 * Returns 0 only when write() reports the full count and the file position
 * advances by exactly len. A zero-progress transient failure is retried. Any
 * partial or contradictory progress returns a negative errno immediately and
 * sets *written from the reported file position; the caller must roll the file
 * back to its position before this call rather than attempting to resume.
 *
 * max_spins bounds zero-progress retries.
 */

int log_write_all(int fd, FAR const uint8_t *buf, size_t len,
                  FAR const struct log_io_s *io, int max_spins,
                  FAR size_t *written);

#endif /* __APPS_LOGGER_LOG_WRITE_H */
