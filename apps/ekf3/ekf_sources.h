/****************************************************************************
 * ArduPilot-compatible EKF aiding source-set configuration.
 ****************************************************************************/

#ifndef __APPS_EKF3_EKF_SOURCES_H
#define __APPS_EKF3_EKF_SOURCES_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

#define EKF_SOURCE_SET_COUNT 3

enum ekf_source_e
{
  EKF_SOURCE_NONE = 0,
  EKF_SOURCE_BARO_OR_COMPASS = 1,
  EKF_SOURCE_RANGE_OR_GPS_YAW = 2,
  EKF_SOURCE_GPS = 3,
  EKF_SOURCE_BEACON = 4,
  EKF_SOURCE_OPTICAL_FLOW = 5,
  EKF_SOURCE_EXTERNAL_NAV = 6,
  EKF_SOURCE_WHEEL = 7,
  EKF_SOURCE_GSF_YAW = 8
};

struct ekf_source_set_s
{
  uint8_t position_xy;
  uint8_t velocity_xy;
  uint8_t position_z;
  uint8_t velocity_z;
  uint8_t yaw;
};

struct ekf_source_config_s
{
  struct ekf_source_set_s set[EKF_SOURCE_SET_COUNT];
  uint8_t active_set;  /* zero based */
  uint8_t options;
};

int ekf_sources_load(FAR struct ekf_source_config_s *config,
                     FAR char *error, size_t error_length);

/* True when source is the active VELXY source, or appears in any source set
 * while the ArduPilot-compatible FUSE_ALL_VELOCITIES option is set.
 */

bool ekf_sources_use_velocity_xy(
  FAR const struct ekf_source_config_s *config, uint8_t source);

#endif /* __APPS_EKF3_EKF_SOURCES_H */
