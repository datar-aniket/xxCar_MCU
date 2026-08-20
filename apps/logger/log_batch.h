/****************************************************************************
 * apps/logger/log_batch.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_XXCAR_LOGGER_LOG_BATCH_H
#define __APPS_XXCAR_LOGGER_LOG_BATCH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

static inline size_t log_batch_count(ssize_t bytes, size_t stride)
{
  if (bytes <= 0 || stride == 0 || (size_t)bytes % stride != 0)
    {
      return 0;
    }

  return (size_t)bytes / stride;
}

static inline const uint8_t *log_batch_record(const uint8_t *buffer,
                                               size_t index, size_t stride)
{
  return buffer + index * stride;
}

static inline bool log_sample_due(uint64_t timestamp, uint32_t interval,
                                  uint64_t *next_timestamp)
{
  if (interval == 0)
    {
      return true;
    }

  if (*next_timestamp == 0)
    {
      *next_timestamp = timestamp;
    }

  if (timestamp < *next_timestamp)
    {
      return false;
    }

  *next_timestamp +=
    ((timestamp - *next_timestamp) / interval + 1) * interval;
  return true;
}

#endif /* __APPS_XXCAR_LOGGER_LOG_BATCH_H */
