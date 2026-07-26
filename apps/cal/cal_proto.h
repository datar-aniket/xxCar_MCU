/****************************************************************************
 * apps/cal/cal_proto.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The calibration wire protocol: text commands in, JSON lines out.
 *
 * Asymmetric on purpose. Commands are simple enough to parse with strtok, which
 * saves carrying a JSON parser on the MCU; replies are JSON so the host gets
 * structured data for free. The whole control path stays readable, which means a
 * calibration can be driven by hand from a plain terminal when the GUI is the
 * thing that is broken.
 *
 * These functions touch no file descriptors. That is what lets the protocol be
 * tested on the host, before any hardware is involved.
 ****************************************************************************/

#ifndef __APPS_CAL_CAL_PROTO_H
#define __APPS_CAL_CAL_PROTO_H

#include <stddef.h>

#ifndef FAR
#  define FAR
#endif

/* Must match PARAM_NAME_MAX in apps/param/param.h. Duplicated rather than
 * included so this file builds standalone in the host test.
 */

#define CAL_PROTO_NAME_MAX 16

/* Bump when the message set changes incompatibly; reported in the hello event
 * so a mismatched GUI can say so instead of misbehaving.
 */

#define CAL_PROTO_VERSION 1

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum cal_cmd_e
{
  CAL_CMD_NONE = 0,      /* blank line */
  CAL_CMD_HELLO,
  CAL_CMD_CAPTURE,
  CAL_CMD_GET,
  CAL_CMD_SET,
  CAL_CMD_COMMIT,
  CAL_CMD_ABORT,
  CAL_CMD_QUIT,

  /* Live view. The GUI needs to SEE a sensor before it can decide anything
   * about it - whether it is present, whether the bench is quiet enough to
   * calibrate on, whether an axis is dead. `capture` answers none of that: it
   * returns one averaged vector after the fact.
   */

  CAL_CMD_LIST,          /* enumerate sensors the GUI can offer */
  CAL_CMD_STREAM,        /* stream <sensor> <hz> */
  CAL_CMD_STOP,          /* stop streaming */

  CAL_CMD_UNKNOWN
};

struct cal_cmd_s
{
  enum cal_cmd_e cmd;
  char           name[CAL_PROTO_NAME_MAX + 1];  /* get/set: parameter name */
  float          fval;                          /* set: value */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Parse one input line. Never fails: an unrecognised line yields
 * CAL_CMD_UNKNOWN so the caller can answer with an error event rather than
 * dropping the host's message silently. Tolerates leading and trailing
 * whitespace, a trailing CR, and any case.
 */

int cal_proto_parse(FAR const char *line, FAR struct cal_cmd_s *out);

/* Event emitters. Each writes one newline-terminated JSON line and returns its
 * length, or -1 if it would not fit (never a partial line).
 */

int cal_proto_hello(FAR char *buf, size_t len);
int cal_proto_ok(FAR char *buf, size_t len, FAR const char *what);
int cal_proto_error(FAR char *buf, size_t len, FAR const char *msg);
int cal_proto_captured(FAR char *buf, size_t len, int n,
                       FAR const float acc[3], FAR const float gyr[3],
                       float temp);

#endif /* __APPS_CAL_CAL_PROTO_H */
