/****************************************************************************
 * apps/imu_delta/imu_resampler.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_IMU_DELTA_IMU_RESAMPLER_H
#define __APPS_IMU_DELTA_IMU_RESAMPLER_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

/* Resample an asynchronous three-axis stream at a timestamp bracketed by two
 * native samples.  The bracket is deliberately bounded: interpolation is a
 * timing alignment operation, not permission to bridge a sensor dropout.
 */

#define IMU_RESAMPLE_MAX_BRACKET_US 1000u

bool imu_resample3(uint64_t before_timestamp,
                   FAR const float before[3],
                   uint64_t after_timestamp,
                   FAR const float after[3],
                   uint64_t target_timestamp,
                   FAR float output[3]);

#endif /* __APPS_IMU_DELTA_IMU_RESAMPLER_H */
