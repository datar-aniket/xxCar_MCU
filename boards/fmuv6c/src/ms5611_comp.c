/****************************************************************************
 * MS5611 compensation from the datasheet, with 64-bit intermediates.
 ****************************************************************************/

#include <limits.h>
#include <stddef.h>

#include "ms5611_comp.h"

uint8_t ms5611_prom_crc4(FAR const uint16_t prom[8])
{
  uint16_t remainder = 0;
  int byte;

  for (byte = 0; byte < 16; byte++)
    {
      uint16_t word = prom[byte >> 1];
      int bit;

      if ((byte >> 1) == 7)
        {
          word &= 0xff00u;  /* stored CRC nibble is zero during calculation */
        }

      remainder ^= (byte & 1) != 0 ? word & 0x00ffu : word >> 8;

      for (bit = 0; bit < 8; bit++)
        {
          remainder = (remainder & 0x8000u) != 0
                      ? (uint16_t)((remainder << 1) ^ 0x3000u)
                      : (uint16_t)(remainder << 1);
        }
    }

  return (uint8_t)((remainder >> 12) & 0x0fu);
}

bool ms5611_prom_valid(FAR const uint16_t prom[8])
{
  int coefficient;

  if (prom == NULL)
    {
      return false;
    }

  for (coefficient = 1; coefficient <= 6; coefficient++)
    {
      if (prom[coefficient] == 0 || prom[coefficient] == 0xffffu)
        {
          return false;
        }
    }

  return ms5611_prom_crc4(prom) == (prom[7] & 0x0fu);
}

bool ms5611_compensate(FAR const uint16_t prom[8], uint32_t raw_pressure,
                       uint32_t raw_temperature,
                       FAR struct ms5611_compensated_s *result)
{
  int64_t delta_temperature;
  int64_t temperature;
  int64_t offset;
  int64_t sensitivity;
  int64_t temperature_second;
  int64_t offset_second;
  int64_t sensitivity_second;
  int64_t difference;
  int64_t pressure;

  if (prom == NULL || result == NULL || raw_pressure == 0 ||
      raw_temperature == 0 || raw_pressure >= 0x00ffffffu ||
      raw_temperature >= 0x00ffffffu)
    {
      return false;
    }

  delta_temperature = (int64_t)raw_temperature -
                      ((int64_t)prom[5] << 8);
  temperature = 2000 + ((delta_temperature * prom[6]) >> 23);
  offset = ((int64_t)prom[2] << 16) +
           ((prom[4] * delta_temperature) >> 7);
  sensitivity = ((int64_t)prom[1] << 15) +
                ((prom[3] * delta_temperature) >> 8);

  temperature_second = 0;
  offset_second = 0;
  sensitivity_second = 0;

  if (temperature < 2000)
    {
      difference = temperature - 2000;
      temperature_second = (delta_temperature * delta_temperature) >> 31;
      offset_second = (5 * difference * difference) >> 1;
      sensitivity_second = (5 * difference * difference) >> 2;

      if (temperature < -1500)
        {
          difference = temperature + 1500;
          offset_second += 7 * difference * difference;
          sensitivity_second += (11 * difference * difference) >> 1;
        }
    }
  temperature -= temperature_second;
  offset -= offset_second;
  sensitivity -= sensitivity_second;

  /* Do not narrow before the final shift. The pre-shift pressure term is
   * normally tens of billions and was previously wrapping int32_t here. */

  pressure = ((((int64_t)raw_pressure * sensitivity) >> 21) - offset) >> 15;

  if (temperature < INT32_MIN || temperature > INT32_MAX ||
      pressure < INT32_MIN || pressure > INT32_MAX)
    {
      return false;
    }

  result->temperature_centi_c = (int32_t)temperature;
  result->pressure_centi_hpa = (int32_t)pressure;
  return true;
}
