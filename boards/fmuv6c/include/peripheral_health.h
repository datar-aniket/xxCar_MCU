/* SPDX-License-Identifier: Apache-2.0 */
#ifndef XXCAR_PERIPHERAL_HEALTH_H
#define XXCAR_PERIPHERAL_HEALTH_H
#include <stdbool.h>
#include <stdint.h>

#define BOARD_HEALTH_IMU0 0
#define BOARD_HEALTH_IMU1 1
#define BOARD_HEALTH_BARO 2
#define BOARD_HEALTH_COUNT 3

struct board_sensor_health_s
{
  uint64_t last_sample_us; /* TIM5 domain; not CLOCK_MONOTONIC */
  uint32_t samples;
  uint32_t errors;
  uint32_t checks_failed;
  uint32_t fifo_flushes;
  uint32_t recovery_attempts;
  uint32_t recoveries;
  int last_error;
  bool registered;
  bool running;
};

int board_sensor_health(unsigned index, struct board_sensor_health_s *out);
void board_sensor_health_publish(unsigned index,
                                const struct board_sensor_health_s *status);
#endif
