/****************************************************************************
 * apps/param/param.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * xxCar parameter system - see param.h.
 *
 * Values live in RAM (g_values) alongside a static table of definitions
 * (g_params). Persistence is a plain-text "NAME VALUE" file on the microSD so
 * it can be edited from a Linux host over USB mass storage.
 *
 * Only parameters that DIFFER from their default are written, and unknown
 * names in the file are ignored with a warning. That means a params.txt from
 * an older firmware still loads, and hand-editing it cannot brick the board.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <syslog.h>

#include "param.h"

/****************************************************************************
 * Private Data - the parameter table
 ****************************************************************************/

#define I32(v) { .i = (v) }
#define F32(v) { .f = (v) }

static const struct param_def_s g_params[] =
{
  /* ---- Serial ports -----------------------------------------------------
   * One FUNC + one BAUD per FMU connector. Function:
   *   0=disabled 1=NSH 2=MAVLink 3=GPS 4=RC_IN
   *
   * NSH is not special - it is a function you assign to a port like any other,
   * and it can be moved. TELEM1 has it by default only because that is where
   * the boot console lives.
   *
   * Every connector below is an FMU UART. The RC IN connector is deliberately
   * absent: on the 6C it is wired to the PX4IO co-processor, not to the FMU, so
   * it is not a port you can assign a function to - see PX4IO_* below.
   * USART6 is likewise absent; it IS the link to PX4IO.
   */

  { "SER_TEL1_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_NSH),      I32(0), I32(4),
    "TELEM1 function (0=off 1=NSH 2=MAVLink 3=GPS 4=RC)" },
  { "SER_TEL1_BAUD", PARAM_TYPE_INT32, I32(115200), I32(1200), I32(3000000),
    "TELEM1 baud rate" },
  { "SER_TEL2_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(4),
    "TELEM2 function" },
  { "SER_TEL2_BAUD", PARAM_TYPE_INT32, I32(57600),  I32(1200), I32(3000000),
    "TELEM2 baud rate" },
  { "SER_TEL3_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(4),
    "TELEM3 function" },
  { "SER_TEL3_BAUD", PARAM_TYPE_INT32, I32(57600),  I32(1200), I32(3000000),
    "TELEM3 baud rate" },
  { "SER_GPS1_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(4),
    "GPS1 function" },
  { "SER_GPS1_BAUD", PARAM_TYPE_INT32, I32(38400),  I32(1200), I32(3000000),
    "GPS1 baud rate" },
  { "SER_GPS2_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(4),
    "GPS2 function" },
  { "SER_GPS2_BAUD", PARAM_TYPE_INT32, I32(38400),  I32(1200), I32(3000000),
    "GPS2 baud rate" },
  { "SER_DBG_FUNC",  PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(4),
    "FMU DEBUG connector function" },
  { "SER_DBG_BAUD",  PARAM_TYPE_INT32, I32(115200), I32(1200), I32(3000000),
    "FMU DEBUG baud rate" },

  /* ---- RC input ---------------------------------------------------------
   * RC_PROT applies only to a receiver wired to an FMU UART (SER_*_FUNC=4).
   * A receiver in the RC IN connector is decoded by PX4IO, which works out the
   * protocol itself and hands over channels - nothing to configure.
   *
   * PPM is a pulse train on a timer-capture pin, not a UART protocol, so it can
   * never be autodetected alongside SBUS/CRSF: select it explicitly.
   */

  { "RC_PROT", PARAM_TYPE_INT32, I32(RC_PROT_AUTO), I32(0), I32(3),
    "RC protocol on an FMU UART (0=auto SBUS/CRSF 1=SBUS 2=CRSF 3=PPM)" },

  /* ---- PX4IO co-processor ------------------------------------------------
   * Owns the RC IN connector and the 8 PWM servo rails. Not all FMUv6C boards
   * are fitted with the chip, so this is allowed to fail harmlessly.
   */

  { "PX4IO_EN",   PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Start the PX4IO client at boot (RC in + PWM out)" },
  { "PX4IO_RATE", PARAM_TYPE_INT32, I32(50), I32(5), I32(400),
    "PX4IO setpoint refresh rate (Hz)" },
  { "PX4IO_PWM_HZ", PARAM_TYPE_INT32, I32(50), I32(25), I32(400),
    "PWM frame rate the servos see (50=analog, 333/400=digital)" },

  /* ---- MAVLink ---------------------------------------------------------- */

  { "MAV_SYS_ID",  PARAM_TYPE_INT32, I32(1),  I32(1), I32(255),
    "MAVLink system ID" },
  { "MAV_COMP_ID", PARAM_TYPE_INT32, I32(1),  I32(1), I32(255),
    "MAVLink component ID" },
  { "MAV_RATE",    PARAM_TYPE_INT32, I32(10), I32(1), I32(200),
    "MAVLink stream rate (Hz)" },

  /* ---- Sensors ---------------------------------------------------------
   * The IMUs are hardware-timed off their FIFOs at 2 kHz; these set the rate
   * that is PUBLISHED to uorb subscribers.
   */

  { "SENS_IMU_RATE",  PARAM_TYPE_INT32, I32(2000), I32(50), I32(2000),
    "IMU publish rate (Hz)" },
  { "SENS_MAG_RATE",  PARAM_TYPE_INT32, I32(50),   I32(1),  I32(100),
    "Magnetometer rate (Hz)" },
  { "SENS_BARO_RATE", PARAM_TYPE_INT32, I32(10),   I32(1),  I32(50),
    "Barometer rate (Hz)" },

  /* ---- Logging (on request only; these choose WHAT gets logged) ---------- */

  { "LOG_ENABLE", PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Start logging at boot" },
  { "LOG_RATE",   PARAM_TYPE_INT32, I32(50), I32(1), I32(500),
    "Log sample rate (Hz)" },
  { "LOG_IMU",    PARAM_TYPE_INT32, I32(1), I32(0), I32(1), "Log IMU" },
  { "LOG_MAG",    PARAM_TYPE_INT32, I32(1), I32(0), I32(1), "Log magnetometer" },
  { "LOG_BARO",   PARAM_TYPE_INT32, I32(1), I32(0), I32(1), "Log barometer" },
  { "LOG_RC",     PARAM_TYPE_INT32, I32(0), I32(0), I32(1), "Log RC input" },

  /* ---- Calibration ------------------------------------------------------
   * Written by the calibration app. Gyro offsets are rad/s, accel offsets
   * m/s^2, accel scales dimensionless, mag offsets Gauss.
   */

  { "CAL_GYRO0_XOFF", PARAM_TYPE_FLOAT, F32(0.0f), F32(-1.0f), F32(1.0f),
    "Gyro 0 X offset (rad/s)" },
  { "CAL_GYRO0_YOFF", PARAM_TYPE_FLOAT, F32(0.0f), F32(-1.0f), F32(1.0f),
    "Gyro 0 Y offset (rad/s)" },
  { "CAL_GYRO0_ZOFF", PARAM_TYPE_FLOAT, F32(0.0f), F32(-1.0f), F32(1.0f),
    "Gyro 0 Z offset (rad/s)" },
  { "CAL_ACC0_XOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-10.0f), F32(10.0f),
    "Accel 0 X offset (m/s2)" },
  { "CAL_ACC0_YOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-10.0f), F32(10.0f),
    "Accel 0 Y offset (m/s2)" },
  { "CAL_ACC0_ZOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-10.0f), F32(10.0f),
    "Accel 0 Z offset (m/s2)" },
  { "CAL_ACC0_XSCL",  PARAM_TYPE_FLOAT, F32(1.0f), F32(0.8f),  F32(1.2f),
    "Accel 0 X scale" },
  { "CAL_ACC0_YSCL",  PARAM_TYPE_FLOAT, F32(1.0f), F32(0.8f),  F32(1.2f),
    "Accel 0 Y scale" },
  { "CAL_ACC0_ZSCL",  PARAM_TYPE_FLOAT, F32(1.0f), F32(0.8f),  F32(1.2f),
    "Accel 0 Z scale" },
  { "CAL_MAG0_XOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-2.0f), F32(2.0f),
    "Mag 0 X offset (Gauss)" },
  { "CAL_MAG0_YOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-2.0f), F32(2.0f),
    "Mag 0 Y offset (Gauss)" },
  { "CAL_MAG0_ZOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-2.0f), F32(2.0f),
    "Mag 0 Z offset (Gauss)" },
};

#define PARAM_COUNT ((int)(sizeof(g_params) / sizeof(g_params[0])))

/* Live values, and whether the SD file has been read yet. */

static union param_value_u g_values[PARAM_COUNT];
static bool                g_initialised;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void param_apply_defaults(void)
{
  int i;

  for (i = 0; i < PARAM_COUNT; i++)
    {
      g_values[i] = g_params[i].def;
    }
}

static void param_ensure_init(void)
{
  if (!g_initialised)
    {
      param_init();
    }
}

/* Clamp a value into the definition's range. Returns true if it had to. */

static bool param_clamp(int idx, FAR union param_value_u *v)
{
  FAR const struct param_def_s *d = &g_params[idx];

  if (d->type == PARAM_TYPE_INT32)
    {
      if (v->i < d->min.i)
        {
          v->i = d->min.i;
          return true;
        }

      if (v->i > d->max.i)
        {
          v->i = d->max.i;
          return true;
        }
    }
  else
    {
      if (v->f < d->min.f)
        {
          v->f = d->min.f;
          return true;
        }

      if (v->f > d->max.f)
        {
          v->f = d->max.f;
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int param_find(FAR const char *name)
{
  int i;

  for (i = 0; i < PARAM_COUNT; i++)
    {
      if (strcmp(g_params[i].name, name) == 0)
        {
          return i;
        }
    }

  return -ENOENT;
}

int param_init(void)
{
  param_apply_defaults();
  g_initialised = true;

  /* Missing file is normal on a fresh card - defaults stand. */

  param_load();
  return OK;
}

int param_load(void)
{
  FAR FILE *fp;
  char line[96];
  int loaded = 0;
  int unknown = 0;

  if (!g_initialised)
    {
      param_apply_defaults();
      g_initialised = true;
    }

  fp = fopen(PARAM_FILE, "r");
  if (fp == NULL)
    {
      return -ENOENT;          /* no file yet: defaults are in effect */
    }

  while (fgets(line, sizeof(line), fp) != NULL)
    {
      char name[PARAM_NAME_MAX + 1];
      char valstr[32];
      union param_value_u v;
      int idx;

      /* Skip comments and blanks */

      if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
        {
          continue;
        }

      if (sscanf(line, "%16s %31s", name, valstr) != 2)
        {
          continue;
        }

      idx = param_find(name);
      if (idx < 0)
        {
          /* An old or hand-typed name. Ignore it rather than fail the whole
           * file - a stale params.txt must never brick the board.
           */

          syslog(LOG_WARNING, "[param] unknown '%s' in %s (ignored)\n",
                 name, PARAM_FILE);
          unknown++;
          continue;
        }

      if (g_params[idx].type == PARAM_TYPE_INT32)
        {
          v.i = strtol(valstr, NULL, 0);
        }
      else
        {
          v.f = strtof(valstr, NULL);
        }

      if (param_clamp(idx, &v))
        {
          syslog(LOG_WARNING, "[param] %s out of range, clamped\n", name);
        }

      g_values[idx] = v;
      loaded++;
    }

  fclose(fp);
  syslog(LOG_INFO, "[param] loaded %d from %s (%d unknown)\n",
         loaded, PARAM_FILE, unknown);
  return loaded;
}

int param_save(void)
{
  FAR FILE *fp;
  int i;
  int written = 0;

  param_ensure_init();

  fp = fopen(PARAM_FILE, "w");
  if (fp == NULL)
    {
      /* No card, or it is currently exported to the USB host */

      return -errno;
    }

  fprintf(fp, "# xxCar parameters\n");
  fprintf(fp, "# Edit and save, then run 'param load' or reboot.\n");
  fprintf(fp, "# Only values that differ from the default are listed.\n");

  for (i = 0; i < PARAM_COUNT; i++)
    {
      if (!param_is_modified(i))
        {
          continue;
        }

      if (g_params[i].type == PARAM_TYPE_INT32)
        {
          fprintf(fp, "%s %" PRId32 "\n", g_params[i].name, g_values[i].i);
        }
      else
        {
          fprintf(fp, "%s %.6f\n", g_params[i].name, (double)g_values[i].f);
        }

      written++;
    }

  fclose(fp);
  syslog(LOG_INFO, "[param] saved %d changed to %s\n", written, PARAM_FILE);
  return written;
}

int param_reset(void)
{
  param_apply_defaults();
  g_initialised = true;
  return OK;
}

int param_get_i32(FAR const char *name, FAR int32_t *value)
{
  int idx;

  param_ensure_init();

  idx = param_find(name);
  if (idx < 0)
    {
      return idx;
    }

  if (g_params[idx].type != PARAM_TYPE_INT32)
    {
      return -EINVAL;
    }

  *value = g_values[idx].i;
  return OK;
}

int param_get_f32(FAR const char *name, FAR float *value)
{
  int idx;

  param_ensure_init();

  idx = param_find(name);
  if (idx < 0)
    {
      return idx;
    }

  if (g_params[idx].type != PARAM_TYPE_FLOAT)
    {
      return -EINVAL;
    }

  *value = g_values[idx].f;
  return OK;
}

int param_set_i32(FAR const char *name, int32_t value)
{
  union param_value_u v;
  int idx;

  param_ensure_init();

  idx = param_find(name);
  if (idx < 0)
    {
      return idx;
    }

  if (g_params[idx].type != PARAM_TYPE_INT32)
    {
      return -EINVAL;
    }

  v.i = value;
  if (param_clamp(idx, &v))
    {
      g_values[idx] = v;
      return -ERANGE;          /* clamped: applied, but tell the caller */
    }

  g_values[idx] = v;
  return OK;
}

int param_set_f32(FAR const char *name, float value)
{
  union param_value_u v;
  int idx;

  param_ensure_init();

  idx = param_find(name);
  if (idx < 0)
    {
      return idx;
    }

  if (g_params[idx].type != PARAM_TYPE_FLOAT)
    {
      return -EINVAL;
    }

  v.f = value;
  if (param_clamp(idx, &v))
    {
      g_values[idx] = v;
      return -ERANGE;
    }

  g_values[idx] = v;
  return OK;
}

int32_t param_i32(FAR const char *name)
{
  int32_t v = 0;
  int idx;

  if (param_get_i32(name, &v) < 0)
    {
      idx = param_find(name);
      return idx >= 0 ? g_params[idx].def.i : 0;
    }

  return v;
}

float param_f32(FAR const char *name)
{
  float v = 0.0f;
  int idx;

  if (param_get_f32(name, &v) < 0)
    {
      idx = param_find(name);
      return idx >= 0 ? g_params[idx].def.f : 0.0f;
    }

  return v;
}

int param_count(void)
{
  return PARAM_COUNT;
}

FAR const struct param_def_s *param_def(int index)
{
  if (index < 0 || index >= PARAM_COUNT)
    {
      return NULL;
    }

  return &g_params[index];
}

bool param_is_modified(int index)
{
  if (index < 0 || index >= PARAM_COUNT)
    {
      return false;
    }

  param_ensure_init();

  if (g_params[index].type == PARAM_TYPE_INT32)
    {
      return g_values[index].i != g_params[index].def.i;
    }

  return fabsf(g_values[index].f - g_params[index].def.f) > 1e-9f;
}

int param_value_str(int index, FAR char *buf, size_t len)
{
  if (index < 0 || index >= PARAM_COUNT)
    {
      return -EINVAL;
    }

  param_ensure_init();

  if (g_params[index].type == PARAM_TYPE_INT32)
    {
      return snprintf(buf, len, "%" PRId32, g_values[index].i);
    }

  return snprintf(buf, len, "%.6f", (double)g_values[index].f);
}
