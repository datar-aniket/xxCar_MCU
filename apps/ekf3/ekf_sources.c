/****************************************************************************
 * Category validation prevents an in-range but meaningless source selection.
 ****************************************************************************/

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ekf_sources.h"
#include "../param/param.h"

static bool valid_position_xy(int value)
{
  return value == EKF_SOURCE_NONE || value == EKF_SOURCE_GPS ||
         value == EKF_SOURCE_BEACON || value == EKF_SOURCE_EXTERNAL_NAV;
}

static bool valid_velocity_xy(int value)
{
  return value == EKF_SOURCE_NONE || value == EKF_SOURCE_GPS ||
         value == EKF_SOURCE_OPTICAL_FLOW ||
         value == EKF_SOURCE_EXTERNAL_NAV || value == EKF_SOURCE_WHEEL;
}

static bool valid_position_z(int value)
{
  return value == EKF_SOURCE_NONE || value == EKF_SOURCE_BARO_OR_COMPASS ||
         value == EKF_SOURCE_RANGE_OR_GPS_YAW || value == EKF_SOURCE_GPS ||
         value == EKF_SOURCE_BEACON || value == EKF_SOURCE_EXTERNAL_NAV;
}

static bool valid_velocity_z(int value)
{
  return value == EKF_SOURCE_NONE || value == EKF_SOURCE_GPS ||
         value == EKF_SOURCE_EXTERNAL_NAV;
}

static bool valid_yaw(int value)
{
  return value == EKF_SOURCE_NONE || value == EKF_SOURCE_BARO_OR_COMPASS ||
         value == EKF_SOURCE_RANGE_OR_GPS_YAW || value == EKF_SOURCE_GPS ||
         value == EKF_SOURCE_EXTERNAL_NAV || value == EKF_SOURCE_GSF_YAW;
}

static int load_one(FAR const char *name, FAR uint8_t *destination,
                    bool (*valid)(int), FAR char *error, size_t error_length)
{
  int value = param_i32(name);

  if (!valid(value))
    {
      if (error != NULL && error_length > 0)
        {
          snprintf(error, error_length, "%s=%d is invalid for this state",
                   name, value);
        }

      return -EINVAL;
    }

  *destination = (uint8_t)value;
  return 0;
}

int ekf_sources_load(FAR struct ekf_source_config_s *config,
                     FAR char *error, size_t error_length)
{
  char name[PARAM_NAME_MAX + 1];
  int active;
  int set;

  if (config == NULL)
    {
      return -EINVAL;
    }

  memset(config, 0, sizeof(*config));

  for (set = 0; set < EKF_SOURCE_SET_COUNT; set++)
    {
      int number = set + 1;

#define LOAD_SOURCE(suffix, member, validator) \
      do \
        { \
          snprintf(name, sizeof(name), "EK3_SRC%d_" suffix, number); \
          if (load_one(name, &config->set[set].member, validator, error, \
                       error_length) < 0) \
            { \
              return -EINVAL; \
            } \
        } \
      while (0)

      LOAD_SOURCE("POSXY", position_xy, valid_position_xy);
      LOAD_SOURCE("VELXY", velocity_xy, valid_velocity_xy);
      LOAD_SOURCE("POSZ", position_z, valid_position_z);
      LOAD_SOURCE("VELZ", velocity_z, valid_velocity_z);
      LOAD_SOURCE("YAW", yaw, valid_yaw);
#undef LOAD_SOURCE
    }

  active = param_i32("EK3_SRC_SET");

  if (active < 1 || active > EKF_SOURCE_SET_COUNT)
    {
      if (error != NULL && error_length > 0)
        {
          snprintf(error, error_length, "EK3_SRC_SET=%d is invalid", active);
        }

      return -EINVAL;
    }

  config->active_set = (uint8_t)(active - 1);
  config->options = (uint8_t)param_i32("EK3_SRC_OPTIONS");

  if ((config->options & ~3u) != 0)
    {
      if (error != NULL && error_length > 0)
        {
          snprintf(error, error_length, "EK3_SRC_OPTIONS has unknown bits");
        }

      return -EINVAL;
    }

  if (error != NULL && error_length > 0)
    {
      error[0] = '\0';
    }

  return 0;
}
