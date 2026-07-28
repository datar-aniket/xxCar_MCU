/* Host unit test for parameter persistence (apps/param/param.c).
 *
 * The bug under test destroyed data rather than failing to store it. The old
 * param_save() opened the LIVE params.txt with "w" - truncating it before one
 * new byte was known to be storable - ignored every fprintf() result, never
 * flushed, and returned a count of parameters visited. So a save onto a full
 * card, or a card yanked or exported over USB mid-write, left an empty or
 * half-written file while the caller was told it had succeeded. The only copy
 * of a calibration was destroyed by the act of saving it.
 *
 * Nothing about that is visible from a passing return code, which is why it is
 * pinned here: every case below asserts on the CONTENT of the live file after
 * a failed save, not on what save() returned.
 *
 * The write failure is injected by making the staging path unwritable, which is
 * what a full card, a read-only mount and an exported card all look like from
 * fopen()/fprintf().
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>

#include "param.h"

static int g_fail;

static void fail(const char *what)
{
  printf("FAIL %s\n", what);
  g_fail++;
}

static void check(int cond, const char *what)
{
  if (!cond)
    {
      fail(what);
    }
}

/* Read a whole file. Returns length, or -1 if it does not exist. */

static long slurp(const char *path, char *buf, size_t len)
{
  FILE *fp = fopen(path, "r");
  size_t n;

  if (fp == NULL)
    {
      return -1;
    }

  n = fread(buf, 1, len - 1, fp);
  buf[n] = '\0';
  fclose(fp);
  return (long)n;
}

/* A known-modified parameter to look for in the file. LOG_IMU0 defaults to 1,
 * so 0 is a genuine change and must be listed.
 */

#define MARK_NAME "LOG_IMU0"

static void set_mark(int32_t v)
{
  if (param_set_i32(MARK_NAME, v) != 0)
    {
      fail("could not set " MARK_NAME);
    }
}

/* ---- 1. a normal save round-trips ------------------------------------- */

static void test_save_load_roundtrip(void)
{
  char buf[4096];
  int32_t got = -1;
  int ret;

  param_reset();
  set_mark(0);

  ret = param_save();
  check(ret >= 1, "save reported no parameters written");
  check(slurp(PARAM_FILE, buf, sizeof(buf)) > 0, "no live file after save");
  check(strstr(buf, MARK_NAME) != NULL, "modified parameter not in the file");

  /* The staging file must not be left lying around on the happy path. */

  check(slurp(PARAM_TMPFILE, buf, sizeof(buf)) < 0,
        "staging file survived a successful save");

  /* Wipe the live value, reload, and it must come back. */

  param_reset();
  check(param_load() >= 1, "load found nothing");
  param_get_i32(MARK_NAME, &got);
  check(got == 0, "value did not survive the round trip");
}

/* ---- 2. a failed save must not touch the live file --------------------- */

static void test_failed_save_preserves_live_file(void)
{
  char before[4096];
  char after[4096];
  long nbefore;
  long nafter;
  int ret;

  /* Establish a good file, and remember it byte for byte. */

  param_reset();
  set_mark(0);
  check(param_save() >= 1, "setup save failed");
  nbefore = slurp(PARAM_FILE, before, sizeof(before));
  check(nbefore > 0, "setup produced no file");

  /* Now make the staging path impossible to create: put a DIRECTORY where the
   * staging file goes, so fopen(PARAM_TMPFILE, "w") fails the way it does on a
   * full or read-only card.
   */

  if (mkdir(PARAM_TMPFILE, 0777) != 0)
    {
      fail("could not stage the write failure");
      return;
    }

  set_mark(1);
  ret = param_save();
  rmdir(PARAM_TMPFILE);

  check(ret < 0, "save reported success when it could not write");

  /* This is the assertion the whole file exists for. */

  nafter = slurp(PARAM_FILE, after, sizeof(after));
  check(nafter == nbefore && memcmp(before, after, (size_t)nbefore) == 0,
        "the live file was damaged by a save that could not complete");
}

/* ---- 3. a save interrupted mid-commit is recovered --------------------- */

static void test_interrupted_commit_is_recovered(void)
{
  char buf[4096];
  int32_t got = -1;

  /* Reproduce the state power loss can leave behind: the staging file has been
   * written and fsync'ed, the live file has been unlinked, and the rename has
   * not happened yet. param_save() is the only thing that creates this, so
   * build it the same way it does.
   */

  param_reset();
  set_mark(0);
  check(param_save() >= 1, "setup save failed");
  check(rename(PARAM_FILE, PARAM_TMPFILE) == 0, "could not stage the crash");
  check(slurp(PARAM_FILE, buf, sizeof(buf)) < 0, "live file still present");

  /* A boot in this state must not silently fall back to defaults - the newest
   * values are sitting right there.
   */

  param_reset();
  check(param_load() >= 1, "load did not recover the interrupted save");
  param_get_i32(MARK_NAME, &got);
  check(got == 0, "recovered the file but not the value");

  check(slurp(PARAM_FILE, buf, sizeof(buf)) > 0,
        "recovery did not restore the live name");
  check(slurp(PARAM_TMPFILE, buf, sizeof(buf)) < 0,
        "staging file left behind after recovery");
}

/* ---- 4. no file at all is not an error -------------------------------- */

static void test_missing_file_is_not_a_failure(void)
{
  unlink(PARAM_FILE);
  unlink(PARAM_TMPFILE);

  param_reset();
  check(param_load() == -ENOENT, "a fresh card should report ENOENT");
}

int main(void)
{
  test_save_load_roundtrip();
  test_failed_save_preserves_live_file();
  test_interrupted_commit_is_recovered();
  test_missing_file_is_not_a_failure();

  unlink(PARAM_FILE);
  unlink(PARAM_TMPFILE);

  if (g_fail != 0)
    {
      printf("param_save: %d failure(s)\n", g_fail);
      return 1;
    }

  printf("param_save: commit, rollback and crash recovery verified - OK\n");
  return 0;
}
