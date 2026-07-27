/* Host unit test for apps/logger/log_write.c.
 *
 * The case that matters cannot be produced by a well-behaved write(): NuttX's
 * FAT driver writes part of a buffer, advances the file position, and then
 * returns a negative errno with the partial count discarded
 * (fs/fat/fs_fat32.c, errout_with_lock). A caller that believes the return
 * value either loses the remainder or writes it twice.
 *
 * Both mistakes shipped. Dropping the remainder tore records and desynchronised
 * every byte after them; retrying from the same offset duplicated the bytes
 * that had landed and shifted the stream out of phase. Real recordings were
 * ruined by each in turn, so the stub below reproduces exactly that behaviour
 * and the tests assert on the bytes that reach the "file".
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

#include "log_write.h"

#define FILESZ (1 << 20)

static unsigned char g_file[FILESZ];
static size_t g_pos;                  /* the file's real position */
static int    g_fail_after;           /* bytes to accept before misbehaving */
static int    g_fail_errno;
static int    g_calls;
static int    g_short_only;           /* short count, no error */
static int    g_stall;                /* refuse everything, no progress */
static size_t g_accept_total;         /* accept at most this many bytes ever */
static size_t g_accepted;
static int    g_lseek_errno;          /* simulate lseek() clobbering errno */
static int    g_fails;

/* Behaves like fat_write(): may land some bytes and STILL return an error,
 * with the count discarded. The file position advances regardless.
 */

static ssize_t stub_write(int fd, const void *buf, size_t len)
{
  size_t take = len;

  (void)fd;
  g_calls++;

  if (g_stall)
    {
      errno = EAGAIN;
      return -1;
    }

  if (g_fail_after >= 0 && (size_t)g_fail_after < len)
    {
      take = (size_t)g_fail_after;
    }

  /* A card that has stopped accepting data for good: it takes a first bite and
   * then nothing more, however many times it is asked.
   */

  if (g_accept_total > 0)
    {
      size_t room = g_accept_total > g_accepted
                    ? g_accept_total - g_accepted : 0;
      if (take > room)
        {
          take = room;
        }
    }

  if (take > 0)
    {
      memcpy(g_file + g_pos, buf, take);
      g_pos += take;
      g_accepted += take;
    }

  if (take == len)
    {
      return (ssize_t)len;
    }

  if (g_short_only)
    {
      return (ssize_t)take;          /* honest short write */
    }

  errno = g_fail_errno;              /* the FAT case: wrote some, reports error */
  return -1;
}

static off_t stub_lseek(int fd, off_t offset, int whence)
{
  (void)fd; (void)offset; (void)whence;

  if (g_lseek_errno != 0)
    {
      errno = g_lseek_errno;
    }

  return (off_t)g_pos;
}

static int stub_sleep(unsigned us)
{
  (void)us;
  /* After one stall, let the next write succeed - a card coming back. */
  g_stall = 0;
  return 0;
}

static const struct log_io_s g_stub_io =
{
  .write = stub_write, .lseek = stub_lseek, .sleep_us = stub_sleep,
};

static void reset(void)
{
  memset(g_file, 0, sizeof(g_file));
  g_pos = 0; g_calls = 0; g_fail_after = -1;
  g_fail_errno = EIO; g_short_only = 0; g_stall = 0;
  g_accept_total = 0; g_accepted = 0;
  g_lseek_errno = 0;
}

static void fail(const char *what)
{
  printf("FAIL %s\n", what);
  g_fails++;
}

static void fill(unsigned char *b, size_t n)
{
  size_t i;
  for (i = 0; i < n; i++)
    {
      b[i] = (unsigned char)(i * 7 + 3);
    }
}

/* The whole point: after a partial-write-then-error, the bytes in the file
 * must be the input exactly once - not truncated, not repeated.
 */

static void test_partial_then_error_is_not_duplicated(void)
{
  unsigned char src[4096];
  size_t written = 0;
  int ret;

  reset();
  fill(src, sizeof(src));
  g_fail_after = 1500;               /* land 1500, then claim -EIO */

  ret = log_write_all(3, src, sizeof(src), &g_stub_io, 100, &written);

  if (ret != 0)
    {
      printf("FAIL partial: returned %d, expected success after resuming\n",
             ret);
      g_fails++;
    }

  if (written != sizeof(src) || g_pos != sizeof(src))
    {
      printf("FAIL partial: wrote %zu, file has %zu, want %zu\n",
             written, g_pos, sizeof(src));
      g_fails++;
      return;
    }

  if (memcmp(g_file, src, sizeof(src)) != 0)
    {
      size_t i;
      for (i = 0; i < sizeof(src); i++)
        {
          if (g_file[i] != src[i])
            {
              printf("FAIL partial: file differs at byte %zu "
                     "(got 0x%02x want 0x%02x) - duplicated or lost\n",
                     i, g_file[i], src[i]);
              break;
            }
        }

      g_fails++;
    }
}

/* An honest short write must also work, and must not re-send what landed. */

static void test_honest_short_write(void)
{
  unsigned char src[8192];
  size_t written = 0;

  reset();
  fill(src, sizeof(src));
  g_fail_after = 700;
  g_short_only = 1;

  if (log_write_all(3, src, sizeof(src), &g_stub_io, 100, &written) != 0 ||
      written != sizeof(src) || memcmp(g_file, src, sizeof(src)) != 0)
    {
      fail("short write: file does not match the input exactly once");
    }
}

/* A card that stalls and then recovers must not lose or repeat anything. */

static void test_stall_then_recover(void)
{
  unsigned char src[2048];
  size_t written = 0;

  reset();
  fill(src, sizeof(src));
  g_stall = 1;                       /* stub_sleep clears it */

  if (log_write_all(3, src, sizeof(src), &g_stub_io, 100, &written) != 0 ||
      written != sizeof(src) || memcmp(g_file, src, sizeof(src)) != 0)
    {
      fail("stall: did not recover cleanly");
    }
}

/* A card that never comes back must give up and report how much really landed,
 * so the caller can tell whether the file ends on a record boundary.
 */

static void test_permanent_stall_reports_progress(void)
{
  unsigned char src[2048];
  size_t written = 12345;
  int ret;

  reset();
  fill(src, sizeof(src));
  g_accept_total = 600;              /* takes 600 bytes and never any more */
  g_fail_errno = EIO;

  /* land 600 bytes then fail forever: lseek shows no further progress */
  ret = log_write_all(3, src, sizeof(src), &g_stub_io, 3, &written);

  if (ret >= 0)
    {
      fail("permanent failure: reported success");
    }

  if (written != 600)
    {
      printf("FAIL permanent failure: reported %zu written, want 600\n",
             written);
      g_fails++;
    }

  if (memcmp(g_file, src, 600) != 0)
    {
      fail("permanent failure: the bytes that landed are wrong");
    }
}

/* The position check follows write(), so it may overwrite errno even when it
 * succeeds. The returned failure must still be the write failure.
 */

static void test_write_errno_survives_lseek(void)
{
  unsigned char src[128];
  size_t written = 12345;
  int ret;

  reset();
  fill(src, sizeof(src));
  g_fail_after = 0;
  g_fail_errno = ENOSPC;
  g_lseek_errno = EINTR;

  ret = log_write_all(3, src, sizeof(src), &g_stub_io, 3, &written);

  if (ret != -ENOSPC)
    {
      printf("FAIL errno preservation: returned %d, want %d\n",
             ret, -ENOSPC);
      g_fails++;
    }

  if (written != 0 || g_pos != 0)
    {
      fail("errno preservation: a failed write made progress");
    }
}

static void test_zero_length(void)
{
  reset();
  if (log_write_all(3, (const unsigned char *)"", 0, &g_stub_io, 10, NULL) != 0)
    {
      fail("zero length should succeed trivially");
    }
}

int main(void)
{
  test_partial_then_error_is_not_duplicated();
  test_honest_short_write();
  test_stall_then_recover();
  test_permanent_stall_reports_progress();
  test_write_errno_survives_lseek();
  test_zero_length();

  if (g_fails != 0)
    {
      printf("log_write: %d failure(s)\n", g_fails);
      return 1;
    }

  printf("log_write: partial writes neither lost nor duplicated - OK\n");
  return 0;
}
