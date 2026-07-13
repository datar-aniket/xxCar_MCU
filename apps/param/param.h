/****************************************************************************
 * apps/param/param.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * xxCar parameter system.
 *
 * Typed, named, bounded parameters with defaults, persisted to the microSD as
 * HUMAN-READABLE TEXT (/fs/microsd/params.txt). The text format is deliberate:
 * the card can be exported to a Linux host with `sdmsc on`, so the file must be
 * editable in any text editor, not a binary blob.
 *
 * Everything configurable hangs off this: serial port functions and baud
 * (including which port runs NSH), RC protocol, sensor rates, MAVLink IDs,
 * what gets logged, and sensor calibration.
 *
 * The library self-initialises on first use, so callers never have to remember
 * to call param_init().
 ****************************************************************************/

#ifndef __APPS_PARAM_PARAM_H
#define __APPS_PARAM_PARAM_H

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PARAM_NAME_MAX   16
#define PARAM_FILE       "/fs/microsd/params.txt"

/* Serial port function (SER_*_FUNC). Which physical port runs NSH is just a
 * parameter like any other.
 */

#define SER_FUNC_DISABLED 0
#define SER_FUNC_NSH      1
#define SER_FUNC_MAVLINK  2
#define SER_FUNC_GPS      3
#define SER_FUNC_RC_IN    4

/* RC protocol (RC_PROT). Auto only spans the two UART protocols; PPM is a
 * pulse train on a timer-capture pin, so it must be selected explicitly.
 */

#define RC_PROT_AUTO      0   /* autodetect SBUS <-> CRSF/ELRS */
#define RC_PROT_SBUS      1
#define RC_PROT_CRSF      2   /* ELRS */
#define RC_PROT_PPM       3   /* explicit only - never autodetected */

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum param_type_e
{
  PARAM_TYPE_INT32 = 0,
  PARAM_TYPE_FLOAT
};

union param_value_u
{
  int32_t i;
  float   f;
};

struct param_def_s
{
  FAR const char     *name;
  enum param_type_e   type;
  union param_value_u def;
  union param_value_u min;
  union param_value_u max;
  FAR const char     *desc;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Load params from the SD card (defaults for anything missing). Safe to call
 * repeatedly; called automatically on first access.
 */

int param_init(void);

/* Look up a parameter index by name; <0 if unknown. */

int param_find(FAR const char *name);

/* Typed accessors. Return <0 on unknown name or type mismatch. Out-of-range
 * sets are clamped and still reported as an error so the caller can complain.
 */

int param_get_i32(FAR const char *name, FAR int32_t *value);
int param_get_f32(FAR const char *name, FAR float *value);
int param_set_i32(FAR const char *name, int32_t value);
int param_set_f32(FAR const char *name, float value);

/* Convenience: return the value or the default if anything goes wrong. Handy
 * for drivers that just want a number and cannot fail.
 */

int32_t param_i32(FAR const char *name);
float   param_f32(FAR const char *name);

/* Persistence. save() writes PARAM_FILE; load() re-reads it; reset() restores
 * defaults in RAM (call save() to make it stick).
 */

int param_save(void);
int param_load(void);
int param_reset(void);

/* Introspection, for the `param` command and the MAVLink PARAM protocol. */

int  param_count(void);
FAR const struct param_def_s *param_def(int index);
bool param_is_modified(int index);
int  param_value_str(int index, FAR char *buf, size_t len);

#endif /* __APPS_PARAM_PARAM_H */
