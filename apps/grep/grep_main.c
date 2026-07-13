/****************************************************************************
 * apps/grep/grep_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `grep` - print the lines that contain a string.
 *
 *   grep [-i] [-v] [-n] [-c] [-q] PATTERN [FILE...]
 *
 *     -i  ignore case
 *     -v  invert: print the lines that do NOT match
 *     -n  prefix each line with its line number
 *     -c  print only a count of matching lines
 *     -q  print nothing; exit 0 if anything matched
 *
 * With no FILE, reads standard input - which is the point of it:
 *
 *   dmesg | grep serial
 *   ps | grep px4io
 *   cat /fs/microsd/params.txt | grep SER_
 *
 * PATTERN is a plain string, not a regular expression. That is a deliberate
 * limit rather than an oversight: a regex engine is a lot of flash for a shell
 * tool whose whole job here is to find a word in a log.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Long enough for a syslog line or a `ps` row without truncating it. A line
 * longer than this is split, which can only cost a match that straddles the
 * split - acceptable for a shell tool, and far better than a heap allocation
 * per line.
 */

#define GREP_LINE_MAX 512

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct grep_opts_s
{
  bool ignore_case;
  bool invert;
  bool number;
  bool count_only;
  bool quiet;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void grep_usage(void)
{
  printf("Usage: grep [-i] [-v] [-n] [-c] [-q] PATTERN [FILE...]\n"
         "  -i  ignore case\n"
         "  -v  print the lines that do NOT match\n"
         "  -n  prefix each line with its line number\n"
         "  -c  print only the number of matching lines\n"
         "  -q  print nothing; exit status says whether anything matched\n"
         "\n"
         "With no FILE, reads standard input:\n"
         "  dmesg | grep serial\n"
         "\n"
         "PATTERN is a plain string, not a regular expression.\n");
}

/* Does this line contain the pattern? */

static bool grep_matches(FAR const char *line, FAR const char *pattern,
                         bool ignore_case)
{
  if (ignore_case)
    {
      return strcasestr(line, pattern) != NULL;
    }

  return strstr(line, pattern) != NULL;
}

/* Scan one already-open stream. Returns the number of matching lines. */

static int grep_stream(FAR FILE *stream, FAR const char *pattern,
                       FAR const struct grep_opts_s *opts,
                       FAR const char *label)
{
  char line[GREP_LINE_MAX];
  int matches = 0;
  int lineno = 0;

  while (fgets(line, sizeof(line), stream) != NULL)
    {
      size_t len = strlen(line);

      lineno++;

      /* fgets keeps the newline; drop it so we control the formatting. */

      while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r'))
        {
          line[--len] = '\0';
        }

      if (grep_matches(line, pattern, opts->ignore_case) == opts->invert)
        {
          continue;
        }

      matches++;

      if (opts->quiet || opts->count_only)
        {
          continue;
        }

      /* Name the file only when there is more than one, as grep does -
       * otherwise every line of a pipeline would be prefixed with junk.
       */

      if (label != NULL)
        {
          printf("%s:", label);
        }

      if (opts->number)
        {
          printf("%d:", lineno);
        }

      printf("%s\n", line);
    }

  return matches;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct grep_opts_s opts;
  FAR const char *pattern;
  int total = 0;
  int nfiles;
  int i;

  memset(&opts, 0, sizeof(opts));

  /* Options first, then the pattern, then the files. */

  for (i = 1; i < argc && argv[i][0] == '-' && argv[i][1] != '\0'; i++)
    {
      FAR const char *o;

      for (o = &argv[i][1]; *o != '\0'; o++)
        {
          switch (*o)
            {
              case 'i': opts.ignore_case = true; break;
              case 'v': opts.invert      = true; break;
              case 'n': opts.number      = true; break;
              case 'c': opts.count_only  = true; break;
              case 'q': opts.quiet       = true; break;

              default:
                fprintf(stderr, "grep: unknown option '-%c'\n", *o);
                grep_usage();
                return 2;
            }
        }
    }

  if (i >= argc)
    {
      grep_usage();
      return 2;
    }

  pattern = argv[i++];
  nfiles  = argc - i;

  if (nfiles == 0)
    {
      /* No files: read standard input. This is the pipeline case. */

      total = grep_stream(stdin, pattern, &opts, NULL);
    }
  else
    {
      for (; i < argc; i++)
        {
          FAR FILE *stream = fopen(argv[i], "r");

          if (stream == NULL)
            {
              fprintf(stderr, "grep: %s: %s\n", argv[i], strerror(errno));
              continue;
            }

          /* Prefix lines with the filename only when several files are being
           * searched, so a single file reads cleanly.
           */

          total += grep_stream(stream, pattern, &opts,
                               nfiles > 1 ? argv[i] : NULL);
          fclose(stream);
        }
    }

  if (opts.count_only && !opts.quiet)
    {
      printf("%d\n", total);
    }

  /* Same convention as grep: 0 if anything matched, 1 if nothing did. It is
   * what makes `if grep -q ...` usable in a script.
   */

  return total > 0 ? 0 : 1;
}
