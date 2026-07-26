/****************************************************************************
 * apps/cal/cal_proto.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * See cal_proto.h.
 ****************************************************************************/

#ifndef CAL_PROTO_HOST_TEST
#  include <nuttx/config.h>
#else
/* Host build only: glibc hides strtok_r's declaration under -std=c11 (strict
 * ANSI) unless a POSIX feature test macro is requested first. NuttX's libc
 * declares it unconditionally, so this is not needed for the target build.
 */
#  define _POSIX_C_SOURCE 200809L
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#include "cal_proto.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Copy a JSON string body into buf, escaping what JSON requires. Error text can
 * contain a parameter name or a stray quote; without this one of those could
 * close the string early and hand the GUI malformed JSON.
 */

static int cal_json_escape(FAR char *buf, size_t len, FAR const char *src)
{
  size_t o = 0;
  size_t i;

  for (i = 0; src[i] != '\0'; i++)
    {
      char c = src[i];

      if (c == '"' || c == '\\')
        {
          if (o + 2 >= len)
            {
              return -1;
            }

          buf[o++] = '\\';
          buf[o++] = c;
        }
      else if ((unsigned char)c < 0x20)
        {
          /* Control characters are not legal raw inside a JSON string. Drop
           * them rather than emit an invalid document.
           */

          continue;
        }
      else
        {
          if (o + 1 >= len)
            {
              return -1;
            }

          buf[o++] = c;
        }
    }

  buf[o] = '\0';
  return (int)o;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cal_proto_parse(FAR const char *line, FAR struct cal_cmd_s *out)
{
  char work[96];
  FAR char *tok;
  FAR char *save = NULL;
  size_t i;

  memset(out, 0, sizeof(*out));
  out->cmd = CAL_CMD_NONE;

  if (line == NULL)
    {
      return 0;
    }

  strncpy(work, line, sizeof(work) - 1);
  work[sizeof(work) - 1] = '\0';

  /* A terminal sends CRLF and people leave trailing spaces. Neither is an
   * error worth reporting.
   */

  for (i = strlen(work); i > 0; i--)
    {
      char c = work[i - 1];

      if (c == '\r' || c == '\n' || c == ' ' || c == '\t')
        {
          work[i - 1] = '\0';
        }
      else
        {
          break;
        }
    }

  tok = strtok_r(work, " \t", &save);
  if (tok == NULL)
    {
      return 0;                        /* blank line */
    }

  for (i = 0; tok[i] != '\0'; i++)
    {
      tok[i] = (char)tolower((unsigned char)tok[i]);
    }

  if (strcmp(tok, "hello") == 0)
    {
      out->cmd = CAL_CMD_HELLO;
    }
  else if (strcmp(tok, "capture") == 0)
    {
      out->cmd = CAL_CMD_CAPTURE;
    }
  else if (strcmp(tok, "commit") == 0)
    {
      out->cmd = CAL_CMD_COMMIT;
    }
  else if (strcmp(tok, "abort") == 0)
    {
      out->cmd = CAL_CMD_ABORT;
    }
  else if (strcmp(tok, "quit") == 0)
    {
      out->cmd = CAL_CMD_QUIT;
    }
  else if (strcmp(tok, "list") == 0)
    {
      out->cmd = CAL_CMD_LIST;
    }
  else if (strcmp(tok, "stop") == 0)
    {
      out->cmd = CAL_CMD_STOP;
    }
  else if (strcmp(tok, "stream") == 0)
    {
      /* stream <sensor> [hz] - hz defaults to 50, which is well above what a
       * plot can show and far below what the link or the parser strain at.
       */

      tok = strtok_r(NULL, " \t", &save);
      if (tok == NULL || strlen(tok) > CAL_PROTO_NAME_MAX)
        {
          out->cmd = CAL_CMD_UNKNOWN;
          return 0;
        }

      strncpy(out->name, tok, sizeof(out->name) - 1);

      tok = strtok_r(NULL, " \t", &save);
      if (tok == NULL)
        {
          out->fval = 50.0f;
        }
      else
        {
          FAR char *end = NULL;

          out->fval = strtof(tok, &end);

          if (end == tok || *end != '\0')
            {
              out->cmd = CAL_CMD_UNKNOWN;
              return 0;
            }
        }

      out->cmd = CAL_CMD_STREAM;
    }
  else if (strcmp(tok, "get") == 0 || strcmp(tok, "set") == 0)
    {
      bool is_set = (tok[0] == 's');

      tok = strtok_r(NULL, " \t", &save);
      if (tok == NULL || strlen(tok) > CAL_PROTO_NAME_MAX)
        {
          /* Truncating an over-long name would silently address a DIFFERENT
           * parameter, so refuse it instead.
           */

          out->cmd = CAL_CMD_UNKNOWN;
          return 0;
        }

      strncpy(out->name, tok, sizeof(out->name) - 1);

      if (is_set)
        {
          FAR char *endptr;

          tok = strtok_r(NULL, " \t", &save);
          if (tok == NULL)
            {
              out->cmd = CAL_CMD_UNKNOWN;
              return 0;
            }

          out->fval = strtof(tok, &endptr);

          /* No digits converted, or trailing junk after the number: refuse
           * rather than silently store 0.0, which would be indistinguishable
           * from someone deliberately setting zero.
           */

          if (endptr == tok || *endptr != '\0')
            {
              out->cmd = CAL_CMD_UNKNOWN;
              return 0;
            }
        }

      out->cmd = is_set ? CAL_CMD_SET : CAL_CMD_GET;
    }
  else
    {
      out->cmd = CAL_CMD_UNKNOWN;
    }

  return 0;
}

int cal_proto_hello(FAR char *buf, size_t len)
{
  int n = snprintf(buf, len,
                   "{\"evt\":\"hello\",\"proto\":%d,\"board\":\"fmuv6c\"}\n",
                   CAL_PROTO_VERSION);

  return (n < 0 || (size_t)n >= len) ? -1 : n;
}

int cal_proto_ok(FAR char *buf, size_t len, FAR const char *what)
{
  char esc[64];
  int n;

  if (cal_json_escape(esc, sizeof(esc), what) < 0)
    {
      return -1;
    }

  n = snprintf(buf, len, "{\"evt\":\"ok\",\"what\":\"%s\"}\n", esc);
  return (n < 0 || (size_t)n >= len) ? -1 : n;
}

int cal_proto_error(FAR char *buf, size_t len, FAR const char *msg)
{
  char esc[128];
  int n;

  if (cal_json_escape(esc, sizeof(esc), msg) < 0)
    {
      return -1;
    }

  n = snprintf(buf, len, "{\"evt\":\"error\",\"msg\":\"%s\"}\n", esc);
  return (n < 0 || (size_t)n >= len) ? -1 : n;
}

int cal_proto_captured(FAR char *buf, size_t len, int n_samples,
                       FAR const float acc[3], FAR const float gyr[3],
                       float temp)
{
  int n = snprintf(buf, len,
                   "{\"evt\":\"captured\",\"n\":%d,"
                   "\"acc\":[%.6f,%.6f,%.6f],"
                   "\"gyr\":[%.6f,%.6f,%.6f],"
                   "\"temp\":%.2f}\n",
                   n_samples,
                   (double)acc[0], (double)acc[1], (double)acc[2],
                   (double)gyr[0], (double)gyr[1], (double)gyr[2],
                   (double)temp);

  return (n < 0 || (size_t)n >= len) ? -1 : n;
}
