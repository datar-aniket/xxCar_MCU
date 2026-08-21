/****************************************************************************
 * tests/mag_frame_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Covers what mag_frame.c adds: parameter loading and body-frame rotation.
 * The calibration mathematics itself belongs to cal_mag_apply() and is
 * already covered by tools/test-cal-mag.sh - it is not retested here.
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mag_frame.h"
#include "param.h"

#define CLOSE(a, b) (fabsf((a) - (b)) < 1.0e-4f)
#define TEST_PI     3.14159265358979323846f

/* Build a frame directly, bypassing the parameters, so the rotation tests
 * are not also testing the loader.
 */

static void identity_frame(FAR struct mag_frame_s *frame)
{
  memset(frame, 0, sizeof(*frame));
  frame->fit.matrix[0][0] = 1.0f;
  frame->fit.matrix[1][1] = 1.0f;
  frame->fit.matrix[2][2] = 1.0f;
  frame->fit.field = 0.45f;
  frame->valid = true;
}

/* With no calibration and no mounting rotation, the ONE thing that still
 * happens is the handedness conversion: the IST8310 reports +y RIGHT and the
 * vehicle convention is +y LEFT.
 *
 * This is the correction that fixed a heading which ran backwards while roll
 * and pitch stayed perfect - heading is atan2(-y, x), and nothing else in
 * the solution touches mag y.
 */

static void test_identity_still_flips_handedness(void)
{
  struct mag_frame_s frame;
  float raw[3] = {0.20f, -0.10f, 0.40f};
  float out[3];

  identity_frame(&frame);

  assert(mag_frame_apply(&frame, raw, out));
  assert(CLOSE(out[0], 0.20f));
  assert(CLOSE(out[1], 0.10f));    /* negated: FRU -> FLU */
  assert(CLOSE(out[2], 0.40f));
}

/* The flip happens AFTER the calibration, which is what lets the frame be
 * corrected without refitting the sensor. With an offset of 1.0 on y, the
 * input 3.0 must give -(3.0 - 1.0) = -2.0. Flipping first would subtract the
 * offset from an already-negated axis and give -3.0 - 1.0 = -4.0.
 */

static void test_calibration_precedes_handedness(void)
{
  struct mag_frame_s frame;
  float raw[3] = {0.0f, 3.0f, 0.0f};
  float out[3];

  identity_frame(&frame);
  frame.fit.offset[1] = 1.0f;

  assert(mag_frame_apply(&frame, raw, out));
  assert(CLOSE(out[1], -2.0f));
}

/* THE test this file exists for.
 *
 * Calibration is applied in sensor axes, THEN the result is rotated. The
 * offset must hit the axis it was measured on, not the axis that ends up
 * there.
 *
 * ROTATION_YAW_90 maps (x,y,z) -> (-y,x,z). Removing an offset of 1.0 on
 * sensor X from the input (1,0,0) gives (0,0,0), which rotates to (0,0,0).
 * Rotating first would give (0,1,0), and subtracting X's offset from that
 * leaves (-1,1,0) - a wrong answer that still looks exactly like a field.
 */

static void test_calibration_precedes_rotation(void)
{
  struct mag_frame_s frame;
  float raw[3] = {1.0f, 0.0f, 0.0f};
  float out[3];

  identity_frame(&frame);
  frame.fit.offset[0] = 1.0f;
  frame.mag_rot = 2;              /* ROTATION_YAW_90 */

  assert(mag_frame_apply(&frame, raw, out));
  assert(CLOSE(out[0], 0.0f));
  assert(CLOSE(out[1], 0.0f));
  assert(CLOSE(out[2], 0.0f));
}

/* Sensor rotation and board rotation compose, in that order. Two YAW_90
 * rotations are a YAW_180: (1,0,0) -> (0,1,0) -> (-1,0,0).
 */

static void test_rotations_compose(void)
{
  struct mag_frame_s frame;
  float raw[3] = {1.0f, 0.0f, 0.0f};
  float out[3];

  identity_frame(&frame);
  frame.mag_rot = 2;              /* ROTATION_YAW_90 */
  frame.board_rot = 2;            /* ROTATION_YAW_90 */

  assert(mag_frame_apply(&frame, raw, out));
  assert(CLOSE(out[0], -1.0f));
  assert(CLOSE(out[1], 0.0f));
  assert(CLOSE(out[2], 0.0f));
}

/* The fine rotation is a full Rodrigues rotation, not the small-angle form.
 * CAL_MAG0_RV* is bounded at 0.35 rad, where first order is already about 2
 * percent wrong and no longer norm-preserving - which would corrupt the
 * field-magnitude health check the estimator uses to decide whether the
 * reading can be trusted at all.
 *
 * Tested well past the parameter bound so the distinction is unambiguous:
 * (1,0,0) rotated about Z by pi/2 is (0,1,0).
 */

static void test_fine_rotation_is_exact(void)
{
  struct mag_frame_s frame;
  float raw[3] = {1.0f, 0.0f, 0.0f};
  float out[3];

  identity_frame(&frame);
  frame.fine_rv[2] = TEST_PI / 2.0f;

  assert(mag_frame_apply(&frame, raw, out));
  assert(CLOSE(out[0], 0.0f));
  assert(CLOSE(out[1], 1.0f));
  assert(CLOSE(out[2], 0.0f));
}

/* A rotation preserves magnitude. If the fine rotation were applied as a
 * first-order approximation this would drift, and the magnitude health check
 * downstream would start rejecting good readings.
 */

static void test_fine_rotation_preserves_magnitude(void)
{
  struct mag_frame_s frame;
  float raw[3] = {0.20f, -0.10f, 0.40f};
  float out[3];
  float before;
  float after;

  identity_frame(&frame);
  frame.fine_rv[0] = 0.35f;       /* the parameter's upper bound */
  frame.fine_rv[1] = -0.35f;

  before = sqrtf(raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2]);
  assert(mag_frame_apply(&frame, raw, out));
  after = sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);

  assert(CLOSE(before, after));
}

/* An invalid calibration passes the reading through and says so. The topic
 * stays observable; the estimator is what declines to fuse it.
 */

static void test_invalid_passes_through(void)
{
  struct mag_frame_s frame;
  float raw[3] = {0.20f, -0.10f, 0.40f};
  float out[3];

  identity_frame(&frame);
  frame.fit.offset[0] = 99.0f;
  frame.mag_rot = 2;
  frame.valid = false;

  assert(!mag_frame_apply(&frame, raw, out));
  assert(CLOSE(out[0], 0.20f));
  assert(CLOSE(out[1], -0.10f));
  assert(CLOSE(out[2], 0.40f));
}

static void test_nonfinite_rejected(void)
{
  struct mag_frame_s frame;
  float raw[3] = {0.0f, NAN, 0.0f};
  float out[3] = {7.0f, 7.0f, 7.0f};

  identity_frame(&frame);

  assert(!mag_frame_apply(&frame, raw, out));
  assert(CLOSE(out[0], 7.0f));    /* untouched */
}

/* Defaults leave CAL_MAG0_OK clear, so a board that has never been
 * calibrated reports invalid rather than silently applying zeros.
 */

static void test_load_defaults_invalid(void)
{
  struct mag_frame_s frame;

  param_init();
  assert(!mag_frame_load(&frame));
  assert(!frame.valid);
}

/* A stored calibration is loaded into the same struct cal_mag_apply() takes,
 * with the symmetric soft-iron expanded from its six parameters.
 */

static void test_load_valid(void)
{
  struct mag_frame_s frame;

  param_init();
  assert(param_set_f32("CAL_MAG0_XOFF", 0.05f) == 0);
  assert(param_set_f32("CAL_MAG0_XY", 0.02f) == 0);
  assert(param_set_i32("CAL_MAG0_OK", 1) == 0);

  assert(mag_frame_load(&frame));
  assert(frame.valid);
  assert(CLOSE(frame.fit.offset[0], 0.05f));
  assert(CLOSE(frame.fit.matrix[0][0], 1.0f));
  assert(CLOSE(frame.fit.matrix[0][1], 0.02f));
  assert(CLOSE(frame.fit.matrix[1][0], 0.02f));   /* symmetric */
}

/* A 45-degree rotation is not an exact axis permutation. It must invalidate
 * rather than be approximated, for the reason rotation.h gives: an inexact
 * rotation changes the magnitude, and every value is a plausible orientation
 * so the mistake would be invisible.
 */

static void test_load_rejects_unsupported_rotation(void)
{
  struct mag_frame_s frame;

  param_init();
  assert(param_set_i32("CAL_MAG0_OK", 1) == 0);
  assert(param_set_i32("SENS_MAG0_ROT", 1) == 0);   /* ROTATION_YAW_45 */

  assert(!mag_frame_load(&frame));
  assert(!frame.valid);
}

/* A stored fit that cannot survive cal_mag_validate() is refused even with
 * CAL_MAG0_OK set. The flag says a calibration was written; it does not say
 * the values are still sane.
 *
 * The parameter range and the validator are NOT the same bound, which is why
 * this can happen at all: CAL_MAG0_XOFF permits +-2.0 Gauss, while
 * CAL_MAG_OFFSET_MAX rejects anything past 1.50.
 */

static void test_load_rejects_insane_fit(void)
{
  struct mag_frame_s frame;

  param_init();
  assert(param_set_i32("CAL_MAG0_OK", 1) == 0);
  assert(param_set_f32("CAL_MAG0_XOFF", 1.8f) == 0);   /* in range, insane */

  assert(!mag_frame_load(&frame));
  assert(!frame.valid);
}

int main(void)
{
  test_identity_still_flips_handedness();
  test_calibration_precedes_handedness();
  test_calibration_precedes_rotation();
  test_rotations_compose();
  test_fine_rotation_is_exact();
  test_fine_rotation_preserves_magnitude();
  test_invalid_passes_through();
  test_nonfinite_rejected();
  test_load_defaults_invalid();
  test_load_valid();
  test_load_rejects_unsupported_rotation();
  test_load_rejects_insane_fit();

  puts("mag_frame: loading, rotation order and composition verified - OK");
  return 0;
}
