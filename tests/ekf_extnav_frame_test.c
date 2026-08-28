/****************************************************************************
 * tests/ekf_extnav_frame_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ekf_extnav_frame.h"

#define PI 3.14159265358979323846f

static void near(float actual, float expected, float tolerance)
{
  assert(fabsf(actual - expected) <= tolerance);
}

static void test_identity(void)
{
  struct ekf_extnav_extrinsics_s extrinsics = {{0.0f}, {0.0f}};
  float marker_position[3] = {2.0f, -3.0f, 0.5f};
  float marker_rpy[3] = {0.1f, -0.2f, 0.3f};
  float body_position[3];
  float body_rpy[3];

  assert(ekf_extnav_apply_extrinsics(&extrinsics, marker_position,
                                     marker_rpy, body_position, body_rpy));
  near(body_position[0], marker_position[0], 1.0e-6f);
  near(body_position[1], marker_position[1], 1.0e-6f);
  near(body_position[2], marker_position[2], 1.0e-6f);
  near(body_rpy[0], marker_rpy[0], 1.0e-6f);
  near(body_rpy[1], marker_rpy[1], 1.0e-6f);
  near(body_rpy[2], marker_rpy[2], 1.0e-6f);
}

static void test_marker_to_body_direction_and_rotation(void)
{
  struct ekf_extnav_extrinsics_s extrinsics =
    {{1.0f, 0.0f, 0.25f}, {0.0f, 0.0f, 0.2f}};
  float marker_position[3] = {10.0f, 20.0f, 1.0f};
  float marker_rpy[3] = {0.0f, 0.0f, PI * 0.5f};
  float body_position[3];
  float body_rpy[3];

  assert(ekf_extnav_apply_extrinsics(&extrinsics, marker_position,
                                     marker_rpy, body_position, body_rpy));
  near(body_position[0], 10.0f, 1.0e-5f);
  near(body_position[1], 21.0f, 1.0e-5f);
  near(body_position[2], 1.25f, 1.0e-5f);
  near(body_rpy[2], PI * 0.5f + 0.2f, 1.0e-5f);
}

static void test_lever_arm_increases_position_variance(void)
{
  struct ekf_extnav_extrinsics_s extrinsics =
    {{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
  float input[6] = {0.01f, 0.0f, 0.0f, 0.01f, 0.0f, 0.04f};
  float output[6];

  assert(ekf_extnav_transform_planar_covariance(&extrinsics, 0.0f,
                                                 input, output));
  near(output[0], 0.01f, 1.0e-6f);
  near(output[3], 0.05f, 1.0e-6f);
  near(output[5], 0.04f, 1.0e-6f);
}

static void test_full_relative_rotation_is_finite(void)
{
  struct ekf_extnav_extrinsics_s extrinsics =
    {{0.1f, -0.2f, 0.3f}, {0.2f, -0.1f, 0.3f}};
  float marker_position[3] = {0.0f, 0.0f, 0.0f};
  float marker_rpy[3] = {-0.3f, 0.15f, 2.8f};
  float body_position[3];
  float body_rpy[3];

  assert(ekf_extnav_apply_extrinsics(&extrinsics, marker_position,
                                     marker_rpy, body_position, body_rpy));
  assert(isfinite(body_rpy[0]));
  assert(isfinite(body_rpy[1]));
  assert(isfinite(body_rpy[2]));
}

int main(void)
{
  test_identity();
  test_marker_to_body_direction_and_rotation();
  test_lever_arm_increases_position_variance();
  test_full_relative_rotation_is_finite();
  puts("ekf extnav frame tests: PASS");
  return 0;
}
