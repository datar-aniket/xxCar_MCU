#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ekf_sources.h"
#include "param.h"

static void expect_rejected(const char *name, int value)
{
  struct ekf_source_config_s config;
  char error[80];

  assert(param_set_i32(name, value) == 0);
  assert(ekf_sources_load(&config, error, sizeof(error)) < 0);
  assert(strstr(error, name) != NULL);
}

int main(void)
{
  struct ekf_source_config_s config;
  char error[80];

  param_init();
  assert(ekf_sources_load(&config, error, sizeof(error)) == 0);
  assert(config.active_set == 0);
  assert(config.set[0].position_xy == EKF_SOURCE_NONE);
  assert(config.set[0].velocity_xy == EKF_SOURCE_OPTICAL_FLOW);
  assert(config.set[0].position_z == EKF_SOURCE_BARO_OR_COMPASS);
  assert(config.set[0].velocity_z == EKF_SOURCE_NONE);
  assert(config.set[0].yaw == EKF_SOURCE_BARO_OR_COMPASS);
  assert(config.set[1].position_xy == EKF_SOURCE_EXTERNAL_NAV);

  /* Values are within the generic parameter bounds but meaningless for the
   * requested state. The category layer, not range clamping, must reject. */

  expect_rejected("EK3_SRC1_POSXY", EKF_SOURCE_OPTICAL_FLOW);
  assert(param_set_i32("EK3_SRC1_POSXY", EKF_SOURCE_NONE) == 0);
  expect_rejected("EK3_SRC1_VELZ", EKF_SOURCE_BARO_OR_COMPASS);
  assert(param_set_i32("EK3_SRC1_VELZ", EKF_SOURCE_NONE) == 0);
  expect_rejected("EK3_SRC1_YAW", EKF_SOURCE_OPTICAL_FLOW);

  assert(param_set_i32("EK3_SRC1_YAW", EKF_SOURCE_BARO_OR_COMPASS) == 0);
  assert(param_set_i32("EK3_SRC_SET", 2) == 0);
  assert(ekf_sources_load(&config, error, sizeof(error)) == 0);
  assert(config.active_set == 1);

  puts("ekf_sources: defaults, source sets and category gates verified - OK");
  return 0;
}
