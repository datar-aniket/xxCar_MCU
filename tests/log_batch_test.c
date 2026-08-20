/****************************************************************************
 * tests/log_batch_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "log_batch.h"

struct padded_record_s
{
  uint64_t timestamp;
  uint32_t value;
  uint32_t padding;
};

int main(void)
{
  struct padded_record_s record[3];
  const uint8_t *buffer = (const uint8_t *)record;
  uint64_t next = 0;
  size_t i;

  memset(record, 0, sizeof(record));
  for (i = 0; i < 3; i++)
    {
      record[i].timestamp = 1000 + i * 500;
      record[i].value = 10 + i;
      record[i].padding = 0xa5a5a5a5u;
    }

  assert(log_batch_count(sizeof(record), sizeof(record[0])) == 3);
  assert(log_batch_count(sizeof(record) - 1, sizeof(record[0])) == 0);
  assert(log_batch_count(0, sizeof(record[0])) == 0);
  assert(log_batch_count(-1, sizeof(record[0])) == 0);
  assert(log_batch_count(sizeof(record), 0) == 0);

  for (i = 0; i < 3; i++)
    {
      const struct padded_record_s *item =
        (const struct padded_record_s *)
        log_batch_record(buffer, i, sizeof(record[0]));

      assert(item->timestamp == 1000 + i * 500);
      assert(item->value == 10 + i);
      assert(item->padding == 0xa5a5a5a5u);
    }

  assert(log_sample_due(1000, 1000, &next));
  assert(next == 2000);
  assert(!log_sample_due(1499, 1000, &next));
  assert(next == 2000);
  assert(log_sample_due(2001, 1000, &next));
  assert(next == 3000);
  assert(log_sample_due(5500, 1000, &next));
  assert(next == 6000);
  assert(!log_sample_due(5999, 1000, &next));
  assert(log_sample_due(5999, 0, &next));
  assert(next == 6000);

  puts("logger bulk record stride and decimation: OK");
  return 0;
}
