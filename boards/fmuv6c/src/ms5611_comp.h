/****************************************************************************
 * Pure MS5611 PROM and compensation helpers, shared with host tests.
 ****************************************************************************/

#ifndef __BOARDS_FMUV6C_SRC_MS5611_COMP_H
#define __BOARDS_FMUV6C_SRC_MS5611_COMP_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

struct ms5611_compensated_s
{
  int32_t pressure_centi_hpa;
  int32_t temperature_centi_c;
};

uint8_t ms5611_prom_crc4(FAR const uint16_t prom[8]);
bool ms5611_prom_valid(FAR const uint16_t prom[8]);

bool ms5611_compensate(FAR const uint16_t prom[8], uint32_t raw_pressure,
                       uint32_t raw_temperature,
                       FAR struct ms5611_compensated_s *result);

#endif /* __BOARDS_FMUV6C_SRC_MS5611_COMP_H */
