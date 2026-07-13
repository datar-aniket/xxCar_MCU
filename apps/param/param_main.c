/****************************************************************************
 * apps/param/param_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `param` - inspect and change configuration.
 *
 *   param show [pattern]   list params (pattern is a name prefix)
 *   param get <name>       print one value
 *   param set <name> <v>   change a value (in RAM; use 'param save' to keep)
 *   param save             write /fs/microsd/params.txt
 *   param load             re-read it (e.g. after editing on a Linux host)
 *   param reset            restore defaults in RAM
 *
 * Typical Linux-host workflow:
 *   sdmsc on               export the card
 *   (edit params.txt on the host, then sync + eject)
 *   sdmsc off
 *   param load
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "param.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void param_usage(void)
{
  printf("Usage: param <command>\n"
         "  show [pattern]    list parameters (pattern = name prefix)\n"
         "  get <name>        print one value\n"
         "  set <name> <val>  change a value (RAM only until 'save')\n"
         "  save              write %s\n"
         "  load              re-read it (after editing on a host)\n"
         "  reset             restore defaults (RAM only until 'save')\n",
         PARAM_FILE);
}

static int param_do_show(FAR const char *pattern)
{
  int n = param_count();
  int shown = 0;
  int i;

  printf("%-16s %-12s %s\n", "NAME", "VALUE", "DESCRIPTION");

  for (i = 0; i < n; i++)
    {
      FAR const struct param_def_s *d = param_def(i);
      char val[32];

      if (pattern != NULL &&
          strncmp(d->name, pattern, strlen(pattern)) != 0)
        {
          continue;
        }

      param_value_str(i, val, sizeof(val));

      /* A '*' marks a value that differs from the default - i.e. exactly what
       * 'param save' will write to the card.
       */

      printf("%-16s %-12s %s%s\n", d->name, val, d->desc,
             param_is_modified(i) ? "  *" : "");
      shown++;
    }

  printf("\n%d parameter(s); * = changed from default\n", shown);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  if (argc < 2)
    {
      param_usage();
      return 1;
    }

  if (strcmp(argv[1], "show") == 0)
    {
      return param_do_show(argc > 2 ? argv[2] : NULL);
    }

  if (strcmp(argv[1], "get") == 0 && argc == 3)
    {
      int idx = param_find(argv[2]);
      char val[32];

      if (idx < 0)
        {
          fprintf(stderr, "param: unknown parameter '%s'\n", argv[2]);
          return 1;
        }

      param_value_str(idx, val, sizeof(val));
      printf("%s %s\n", argv[2], val);
      return 0;
    }

  if (strcmp(argv[1], "set") == 0 && argc == 4)
    {
      FAR const struct param_def_s *d;
      int idx = param_find(argv[2]);

      if (idx < 0)
        {
          fprintf(stderr, "param: unknown parameter '%s'\n", argv[2]);
          return 1;
        }

      d = param_def(idx);

      if (d->type == PARAM_TYPE_INT32)
        {
          ret = param_set_i32(argv[2], strtol(argv[3], NULL, 0));
        }
      else
        {
          ret = param_set_f32(argv[2], strtof(argv[3], NULL));
        }

      if (ret == -ERANGE)
        {
          char val[32];
          param_value_str(idx, val, sizeof(val));
          fprintf(stderr, "param: out of range, clamped to %s\n", val);
          return 1;
        }
      else if (ret < 0)
        {
          fprintf(stderr, "param: set failed: %d\n", ret);
          return 1;
        }

      printf("%s = %s  (not saved yet - run 'param save')\n",
             argv[2], argv[3]);
      return 0;
    }

  if (strcmp(argv[1], "save") == 0)
    {
      ret = param_save();
      if (ret < 0)
        {
          fprintf(stderr, "param: save failed: %d\n", ret);
          fprintf(stderr, "  is the card present and not exported "
                          "over USB (sdmsc off)?\n");
          return 1;
        }

      printf("saved %d changed parameter(s) to %s\n", ret, PARAM_FILE);
      return 0;
    }

  if (strcmp(argv[1], "load") == 0)
    {
      ret = param_load();
      if (ret < 0)
        {
          fprintf(stderr, "param: load failed: %d (no %s?)\n",
                  ret, PARAM_FILE);
          return 1;
        }

      printf("loaded %d parameter(s) from %s\n", ret, PARAM_FILE);
      return 0;
    }

  if (strcmp(argv[1], "reset") == 0)
    {
      param_reset();
      printf("defaults restored (RAM only - run 'param save' to keep)\n");
      return 0;
    }

  param_usage();
  return 1;
}
