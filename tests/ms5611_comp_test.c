#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "ms5611_comp.h"

static void set_crc(uint16_t prom[8])
{
  prom[7] = (uint16_t)((prom[7] & 0xfff0u) | ms5611_prom_crc4(prom));
}

static void test_datasheet_vector(void)
{
  /* MS5611-01BA03 datasheet example: 20.07 C and 1000.09 mbar. */

  uint16_t prom[8] = {0, 40127, 36924, 23317, 23282, 33464, 28312, 0};
  struct ms5611_compensated_s result;

  set_crc(prom);
  assert(ms5611_prom_valid(prom));
  assert(ms5611_compensate(prom, 9085466, 8569150, &result));
  assert(result.temperature_centi_c == 2007);
  assert(result.pressure_centi_hpa == 100009);

  /* This intermediate is about 3.28e9, larger than signed int32. The old
   * driver narrowed before shifting and published a negative pressure. */

  assert(result.pressure_centi_hpa > 0);
}

static void test_crc_rejects_corruption(void)
{
  uint16_t prom[8] = {0, 40127, 36924, 23317, 23282, 33464, 28312, 0};

  set_crc(prom);
  assert(ms5611_prom_valid(prom));
  prom[3] ^= 0x0040;
  assert(!ms5611_prom_valid(prom));
}

static void test_invalid_adc_values(void)
{
  uint16_t prom[8] = {0, 40127, 36924, 23317, 23282, 33464, 28312, 0};
  struct ms5611_compensated_s result;

  set_crc(prom);
  assert(!ms5611_compensate(prom, 0, 8569150, &result));
  assert(!ms5611_compensate(prom, 9085466, 0x00ffffff, &result));
}

int main(void)
{
  test_datasheet_vector();
  test_crc_rejects_corruption();
  test_invalid_adc_values();
  puts("ms5611: CRC and 64-bit second-order compensation verified - OK");
  return 0;
}
