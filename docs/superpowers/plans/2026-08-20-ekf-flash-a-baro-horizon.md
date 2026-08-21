# EKF3 Flash A — Delayed Horizon, Measurement Frontend, Barometer Fusion

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give EKF3 a delayed fusion horizon with a re-propagating output predictor, publish calibrated `vehicle_mag`/`vehicle_baro` topics, and fuse barometric height into vertical position — leaving attitude behaviour provably unchanged when `EK3_DELAY_MS` is 0.

**Architecture:** IMU samples enter a ring buffer instead of going straight to the core. The filter consumes only samples older than `now − EK3_DELAY_MS`; barometer measurements are fused at the horizon against the state as it was when they were sampled. Publication replays the ring forward from the horizon state to the present using a lightweight output state, so `estimator_state` stays a current-time 400 Hz topic. A separate low-rate daemon in `apps/sensors` applies the stored magnetometer calibration and republishes both aiding sensors in the body frame.

**Tech Stack:** C11, NuttX (out-of-tree apps under `apps/`, symlinked into `deps/nuttx-apps/xxcar`), NuttX uORB, STM32H7 hard-float. Host tests are plain C compiled with `cc` and run natively.

**Source spec:** `docs/superpowers/specs/2026-08-20-ekf-baro-mag-fusion-design.md`

## Global Constraints

- **Style:** NuttX kernel C style as used throughout `apps/`. Two-space indent, braces on their own line indented with the block, `FAR` on pointer parameters, `/* */` comments only. Match the surrounding file — do not reformat existing code.
- **uORB topic names are limited to 20 characters.** `/dev/uorb/<name><instance>` must fit `NAME_MAX` (32). Exceeding it truncates the registered path silently and `orb_advertise()` returns −1 with no explanation. Enforce with `ORB_NAME_FITS()` in `apps/uorb_msgs/uorb_msgs.c`.
- **Every new uORB struct needs `static_assert` offset and size checks** in `uorb_msgs.c`. uORB decodes by walking `o_format` and advancing by each conversion's size with no realignment; one stray pad byte shifts every later field and prints convincing nonsense.
- **`PARAM_NAME_MAX` is 16**, including the NUL. All new parameter names must fit.
- **Floats, not doubles.** `printf` of a float requires an explicit `(double)` cast — the codebase does this everywhere and `-Wall -Wextra -Werror` is on in host tests.
- **Host tests must compile with** `-std=c11 -Wall -Wextra -Werror -DFAR=`.
- **No dynamic allocation** in the estimator or sensor paths.
- **Large state lives in statics, not on the stack.** `ekf3` runs on a 6144-byte stack; the IMU ring is ~3 KB and must be a file-scope static.
- **Do not modify** `apps/sensors/sensors.c`, `apps/imu_delta/`, or any driver in `boards/fmuv6c/src/`. This plan does not touch the IMU path.
- **Commit after every task.** Sign commits with the trailer `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`.
- **`tools/verify.sh` is the gate.** It runs every `tools/test-*.sh`, then builds the firmware and proves the artifact is newer than its sources. `test-cpu-runtime` is a known pre-existing failure; it is unrelated to this work and must not be "fixed" here.

## File Structure

**Created:**

| File | Responsibility |
|---|---|
| `apps/sensors/mag_correct.h` / `.c` | Pure magnetometer calibration + rotation math. No I/O, host-testable. |
| `apps/sensors/aux.h` / `.c` | Low-rate daemon: polls `sensor_mag0`/`sensor_baro0`, publishes `vehicle_mag`/`vehicle_baro`. |
| `apps/ekf3/ekf_delay.h` / `.c` | IMU ring buffer and timestamped measurement queues. No filter math. |
| `tests/mag_correct_test.c` | Calibration application, rotation composition, ordering. |
| `tests/ekf_delay_test.c` | Ring ordering, horizon arithmetic, overflow, output window. |
| `tests/ekf_output_test.c` | Output predictor equivalence and determinism. |
| `tests/ekf_baro_test.c` | Height conversion, sign convention, gating, timeout. |
| `tools/test-mag-correct.sh`, `test-ekf-delay.sh`, `test-ekf-output.sh`, `test-ekf-baro.sh` | Host test runners. |

**Modified:**

| File | Change |
|---|---|
| `apps/uorb_msgs/uorb_msgs.h` / `.c` | `vehicle_mag`, `vehicle_baro`; granular `ESTIMATOR_*` validity bits. |
| `apps/sensors/Makefile`, `Kconfig`, `sensors_main.c` | Build new files; `sensors aux` subcommand. |
| `apps/ekf3/Makefile` | Build `ekf_delay.c`. |
| `apps/ekf3/ekf_core.h` / `.c` | `measurement_update_1d()`, output predictor, baro fusion, granular status. |
| `apps/ekf3/ekf3.h` / `.c` | Ring wiring, aiding subscriptions, horizon loop. |
| `apps/ekf3/ekf3_main.c` | Per-source status reporting. |
| `apps/param/param.c` | `EK3_DELAY_MS`, `EK3_ALT_M_NSE`, `EK3_ALT_I_GATE`. |
| `tests/param_range_test.c`, `tests/ekf_core_test.c` | Extend for new parameters and the 1-D update. |

---

### Task 1: `vehicle_mag` and `vehicle_baro` topics

The compiler is the test here. `static_assert` on every offset and the total size is what catches a layout mistake, and it fails the build — which is a stronger gate than a runtime test, because it cannot be skipped.

**Files:**
- Modify: `apps/uorb_msgs/uorb_msgs.h`
- Modify: `apps/uorb_msgs/uorb_msgs.c`

**Interfaces:**
- Consumes: nothing.
- Produces: `struct vehicle_mag_s`, `struct vehicle_baro_s`, `ORB_ID(vehicle_mag)`, `ORB_ID(vehicle_baro)`, `int vehicle_mag_advertise(void)`, `int vehicle_mag_publish(int fd, FAR const struct vehicle_mag_s *msg)`, `int vehicle_baro_advertise(void)`, `int vehicle_baro_publish(int fd, FAR const struct vehicle_baro_s *msg)`.

- [ ] **Step 1: Add the structs to `uorb_msgs.h`**

Insert after `struct vehicle_gyro_s` and before the `vehicle_imu_s` comment block:

```c
/* Calibrated body-frame aiding sensors. Both carry the driver's sample time
 * separately from publication time, because the EKF fuses them at the horizon
 * against the state as it was when the sample was taken - not when it arrived.
 *
 * vehicle_baro deliberately carries pressure, not height. Converting needs a
 * reference pressure, and that reference is captured at EKF alignment: it is
 * an estimator concern, and a height field here would be meaningless before
 * the estimator has aligned.
 */

struct vehicle_mag_s
{
  uint64_t timestamp;             /*  0: us, publication time */
  uint64_t timestamp_sample;      /*  8: us, driver sample time */
  float    field[3];              /* 16: Gauss, calibrated, body frame */
  float    temperature;           /* 28: degrees C */
  uint8_t  calibrated;            /* 32: 0 = raw passthrough, 1 = corrected */
  uint8_t  instance;              /* 33 */
  uint8_t  pad[6];                /* 34 */
};

struct vehicle_baro_s
{
  uint64_t timestamp;             /*  0: us, publication time */
  uint64_t timestamp_sample;      /*  8: us, driver sample time */
  float    pressure;              /* 16: hPa */
  float    temperature;           /* 20: degrees C */
};
```

Add to the `ORB_DECLARE` block:

```c
ORB_DECLARE(vehicle_mag);
ORB_DECLARE(vehicle_baro);
```

Add to the prototype block:

```c
int vehicle_mag_advertise(void);
int vehicle_mag_publish(int fd, FAR const struct vehicle_mag_s *msg);

int vehicle_baro_advertise(void);
int vehicle_baro_publish(int fd, FAR const struct vehicle_baro_s *msg);
```

- [ ] **Step 2: Add the name and layout assertions to `uorb_msgs.c`**

After `ORB_NAME_FITS("estimator_state");`:

```c
ORB_NAME_FITS("vehicle_mag");
ORB_NAME_FITS("vehicle_baro");
```

After the `vehicle_gyro_s` layout assertions:

```c
static_assert(offsetof(struct vehicle_mag_s, timestamp)        ==  0, "layout");
static_assert(offsetof(struct vehicle_mag_s, timestamp_sample) ==  8, "layout");
static_assert(offsetof(struct vehicle_mag_s, field)            == 16, "layout");
static_assert(offsetof(struct vehicle_mag_s, temperature)      == 28, "layout");
static_assert(offsetof(struct vehicle_mag_s, calibrated)       == 32, "layout");
static_assert(offsetof(struct vehicle_mag_s, instance)         == 33, "layout");
static_assert(sizeof(struct vehicle_mag_s)                     == 40, "layout");

static_assert(offsetof(struct vehicle_baro_s, timestamp)        ==  0, "layout");
static_assert(offsetof(struct vehicle_baro_s, timestamp_sample) ==  8, "layout");
static_assert(offsetof(struct vehicle_baro_s, pressure)         == 16, "layout");
static_assert(offsetof(struct vehicle_baro_s, temperature)      == 20, "layout");
static_assert(sizeof(struct vehicle_baro_s)                     == 24, "layout");
```

- [ ] **Step 3: Add the format strings**

Inside the `#ifdef CONFIG_DEBUG_UORB` block, after `vehicle_gyro_format`:

```c
static const char vehicle_mag_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",field[0]:%hf,field[1]:%hf,field[2]:%hf"
  ",temperature:%hf"
  ",calibrated:%hhu,instance:%hhu";

static const char vehicle_baro_format[] =
  "timestamp:%" PRIu64
  ",timestamp_sample:%" PRIu64
  ",pressure:%hf,temperature:%hf";
```

- [ ] **Step 4: Define the topics and the accessors**

After `ORB_DEFINE(vehicle_gyro, ...)`:

```c
ORB_DEFINE(vehicle_mag, struct vehicle_mag_s, vehicle_mag_format);
ORB_DEFINE(vehicle_baro, struct vehicle_baro_s, vehicle_baro_format);
```

In the public functions section, following the `vehicle_gyro_advertise`/`_publish` pattern exactly (including its `msg == NULL` guard returning `-EINVAL`):

```c
int vehicle_mag_advertise(void)
{
  return orb_advertise(ORB_ID(vehicle_mag), NULL);
}

int vehicle_mag_publish(int fd, FAR const struct vehicle_mag_s *msg)
{
  if (msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(vehicle_mag), fd, msg);
}

int vehicle_baro_advertise(void)
{
  return orb_advertise(ORB_ID(vehicle_baro), NULL);
}

int vehicle_baro_publish(int fd, FAR const struct vehicle_baro_s *msg)
{
  if (msg == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(vehicle_baro), fd, msg);
}
```

Read `vehicle_gyro_publish` first and copy its exact guard, so the two do not diverge.

- [ ] **Step 5: Prove the assertions actually run**

Temporarily change one assertion to a wrong value — e.g. `sizeof(struct vehicle_mag_s) == 41` — and run:

```bash
./tools/build.sh 2>&1 | grep -i "layout"
```

Expected: build FAILS with the "layout" message. This proves the check is compiled and not silently skipped. Revert the change.

- [ ] **Step 6: Build clean**

```bash
./tools/build.sh
```

Expected: exits 0, `build/xxcar.px4` produced.

- [ ] **Step 7: Commit**

```bash
git add apps/uorb_msgs/uorb_msgs.h apps/uorb_msgs/uorb_msgs.c
git commit -m "uorb_msgs: define the calibrated mag and baro aiding topics

The EKF fuses these at a delayed horizon, so each carries the driver's
sample time separately from publication time. vehicle_baro carries
pressure rather than height: the reference is captured at EKF alignment,
which makes height an estimator concern, not a sensor one.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Magnetometer calibration math

Pure functions, no I/O, fully host-tested. The ordering — calibrate in sensor axes, *then* rotate — is the part most likely to be got wrong, so it gets a dedicated test.

**Files:**
- Create: `apps/sensors/mag_correct.h`, `apps/sensors/mag_correct.c`
- Create: `tests/mag_correct_test.c`, `tools/test-mag-correct.sh`
- Modify: `apps/sensors/Makefile`

**Interfaces:**
- Consumes: `rotation_apply()`, `rotation_supported()` from `apps/sensors/rotation.h`; `param_f32()`, `param_i32()` from `apps/param/param.h`.
- Produces: `struct mag_cal_s`, `bool mag_correct_load(FAR struct mag_cal_s *cal)`, `bool mag_correct_apply(FAR const struct mag_cal_s *cal, FAR const float raw[3], FAR float out[3])`.

- [ ] **Step 1: Write the header**

Create `apps/sensors/mag_correct.h`:

```c
/****************************************************************************
 * apps/sensors/mag_correct.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Turns one raw magnetometer reading into a calibrated body-frame field.
 *
 *     corrected = SOFT_IRON * (raw - OFFSET)
 *     body      = BOARD_ROT( FINE_ROT( MAG_ROT( corrected ) ) )
 *
 * The order is not arbitrary, and is the same argument sensors.h makes for
 * the IMU: calibration is measured in the SENSOR's own axes - the sphere fit
 * records what each chip axis reads - so it must be applied before any
 * rotation. A soft-iron matrix means nothing once the axes have been mixed.
 *
 * FINE_ROT is CAL_MAG0_RV*, the small residual rotation a host extrinsic
 * calibration would solve for. It defaults to zero, which is identity, and
 * that is the correct assumption for a magnetometer soldered to a known board.
 ****************************************************************************/

#ifndef __APPS_SENSORS_MAG_CORRECT_H
#define __APPS_SENSORS_MAG_CORRECT_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

struct mag_cal_s
{
  float   offset[3];      /* CAL_MAG0_{X,Y,Z}OFF, Gauss */
  float   matrix[3][3];   /* CAL_MAG0_{XX,YY,ZZ,XY,XZ,YZ}, symmetric */
  float   fine_rv[3];     /* CAL_MAG0_RV{X,Y,Z}, rotation vector, rad */
  float   field;          /* CAL_MAG0_FIELD, expected magnitude, Gauss */
  uint8_t mag_rot;        /* SENS_MAG0_ROT */
  uint8_t board_rot;      /* SENS_BOARD_ROT */
  bool    valid;          /* CAL_MAG0_OK and both rotations supported */
};

/* Read CAL_MAG0_* / SENS_MAG0_ROT / SENS_BOARD_ROT into cal.
 *
 * Always fills cal, and always returns having set cal->valid. Returns the
 * same value as cal->valid, which is false when CAL_MAG0_OK is clear or
 * either rotation is one rotation_apply() cannot perform exactly. An invalid
 * calibration is not an error: the caller publishes the raw field with
 * calibrated=0 so the topic stays observable for diagnostics.
 */

bool mag_correct_load(FAR struct mag_cal_s *cal);

/* Apply cal to raw, writing the body-frame field to out.
 *
 * When cal->valid is false, copies raw to out unchanged and returns false:
 * the reading is still the right shape, just not in the right frame. Returns
 * false and leaves out untouched if raw is not finite.
 */

bool mag_correct_apply(FAR const struct mag_cal_s *cal,
                       FAR const float raw[3], FAR float out[3]);

#endif /* __APPS_SENSORS_MAG_CORRECT_H */
```

- [ ] **Step 2: Write the failing test**

Create `tests/mag_correct_test.c`:

```c
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mag_correct.h"
#include "param.h"

#define CLOSE(a, b) (fabsf((a) - (b)) < 1.0e-4f)

/* Identity calibration in every respect: no offset, unit matrix, no fine
 * rotation, no mounting rotation. Anything that changes the vector here is a
 * bug in the plumbing rather than in the mathematics.
 */

static void test_identity(void)
{
  struct mag_cal_s cal;
  float raw[3] = {0.20f, -0.10f, 0.40f};
  float out[3];

  memset(&cal, 0, sizeof(cal));
  cal.matrix[0][0] = 1.0f;
  cal.matrix[1][1] = 1.0f;
  cal.matrix[2][2] = 1.0f;
  cal.valid = true;

  assert(mag_correct_apply(&cal, raw, out));
  assert(CLOSE(out[0], 0.20f));
  assert(CLOSE(out[1], -0.10f));
  assert(CLOSE(out[2], 0.40f));
}

/* Hard iron is removed before soft iron is applied. With offset (1,2,3) and a
 * diagonal soft-iron of 2, the input (2,3,4) must give (2,2,2): 2*(2-1),
 * 2*(3-2), 2*(4-3). Applying the matrix first would give (3,4,5).
 */

static void test_offset_precedes_matrix(void)
{
  struct mag_cal_s cal;
  float raw[3] = {2.0f, 3.0f, 4.0f};
  float out[3];

  memset(&cal, 0, sizeof(cal));
  cal.offset[0] = 1.0f;
  cal.offset[1] = 2.0f;
  cal.offset[2] = 3.0f;
  cal.matrix[0][0] = 2.0f;
  cal.matrix[1][1] = 2.0f;
  cal.matrix[2][2] = 2.0f;
  cal.valid = true;

  assert(mag_correct_apply(&cal, raw, out));
  assert(CLOSE(out[0], 2.0f));
  assert(CLOSE(out[1], 2.0f));
  assert(CLOSE(out[2], 2.0f));
}

/* Calibration is applied in sensor axes, THEN rotated. The offset must hit
 * the axis it was measured on, not the axis that ends up there.
 *
 * ROTATION_YAW_90 maps (x,y,z) -> (-y,x,z). Removing an offset of 1.0 on
 * sensor X from the input (1,0,0) gives (0,0,0), which rotates to (0,0,0).
 * Rotating first would give (0,1,0), and subtracting X's offset from that
 * leaves (-1,1,0) - a wrong answer that still looks like a field.
 */

static void test_calibration_precedes_rotation(void)
{
  struct mag_cal_s cal;
  float raw[3] = {1.0f, 0.0f, 0.0f};
  float out[3];

  memset(&cal, 0, sizeof(cal));
  cal.offset[0] = 1.0f;
  cal.matrix[0][0] = 1.0f;
  cal.matrix[1][1] = 1.0f;
  cal.matrix[2][2] = 1.0f;
  cal.mag_rot = 2;   /* ROTATION_YAW_90 */
  cal.valid = true;

  assert(mag_correct_apply(&cal, raw, out));
  assert(CLOSE(out[0], 0.0f));
  assert(CLOSE(out[1], 0.0f));
  assert(CLOSE(out[2], 0.0f));
}

/* A fine rotation of 90 degrees about Z is a full Rodrigues rotation, not a
 * small-angle approximation. CAL_MAG0_RV* is bounded at 0.35 rad, where the
 * first-order error is already about 2 percent, so the implementation must
 * use the exact form. Test it well past the bound to make the distinction
 * unambiguous: (1,0,0) about Z by pi/2 is (0,1,0).
 */

static void test_fine_rotation_is_exact(void)
{
  struct mag_cal_s cal;
  float raw[3] = {1.0f, 0.0f, 0.0f};
  float out[3];

  memset(&cal, 0, sizeof(cal));
  cal.matrix[0][0] = 1.0f;
  cal.matrix[1][1] = 1.0f;
  cal.matrix[2][2] = 1.0f;
  cal.fine_rv[2] = (float)M_PI / 2.0f;
  cal.valid = true;

  assert(mag_correct_apply(&cal, raw, out));
  assert(CLOSE(out[0], 0.0f));
  assert(CLOSE(out[1], 1.0f));
  assert(CLOSE(out[2], 0.0f));
}

/* An invalid calibration passes the reading through unchanged and says so.
 * The topic stays observable; the EKF is what declines to fuse it.
 */

static void test_invalid_passes_through(void)
{
  struct mag_cal_s cal;
  float raw[3] = {0.20f, -0.10f, 0.40f};
  float out[3];

  memset(&cal, 0, sizeof(cal));
  cal.offset[0] = 99.0f;
  cal.valid = false;

  assert(!mag_correct_apply(&cal, raw, out));
  assert(CLOSE(out[0], 0.20f));
  assert(CLOSE(out[1], -0.10f));
  assert(CLOSE(out[2], 0.40f));
}

static void test_nonfinite_rejected(void)
{
  struct mag_cal_s cal;
  float raw[3] = {0.0f, NAN, 0.0f};
  float out[3] = {7.0f, 7.0f, 7.0f};

  memset(&cal, 0, sizeof(cal));
  cal.matrix[0][0] = 1.0f;
  cal.matrix[1][1] = 1.0f;
  cal.matrix[2][2] = 1.0f;
  cal.valid = true;

  assert(!mag_correct_apply(&cal, raw, out));
  assert(CLOSE(out[0], 7.0f));   /* untouched */
}

/* Defaults are identity plus CAL_MAG0_OK clear, so a board that has never
 * been calibrated reports invalid rather than silently applying zeros.
 */

static void test_load_defaults_invalid(void)
{
  struct mag_cal_s cal;

  param_init();
  assert(!mag_correct_load(&cal));
  assert(!cal.valid);
  assert(CLOSE(cal.matrix[0][0], 1.0f));
  assert(CLOSE(cal.offset[0], 0.0f));
}

static void test_load_valid(void)
{
  struct mag_cal_s cal;

  param_init();
  assert(param_set_f32("CAL_MAG0_XOFF", 0.05f) == 0);
  assert(param_set_i32("CAL_MAG0_OK", 1) == 0);
  assert(mag_correct_load(&cal));
  assert(cal.valid);
  assert(CLOSE(cal.offset[0], 0.05f));
}

/* A 45-degree rotation cannot be done as an exact axis permutation. It must
 * invalidate the calibration rather than be approximated, for the reason
 * rotation.h gives: an inexact rotation changes the magnitude, and every
 * value is a plausible orientation so the mistake would be invisible.
 */

static void test_load_rejects_unsupported_rotation(void)
{
  struct mag_cal_s cal;

  param_init();
  assert(param_set_i32("CAL_MAG0_OK", 1) == 0);
  assert(param_set_i32("SENS_MAG0_ROT", 1) == 0);  /* ROTATION_YAW_45 */
  assert(!mag_correct_load(&cal));
  assert(!cal.valid);
}

int main(void)
{
  test_identity();
  test_offset_precedes_matrix();
  test_calibration_precedes_rotation();
  test_fine_rotation_is_exact();
  test_invalid_passes_through();
  test_nonfinite_rejected();
  test_load_defaults_invalid();
  test_load_valid();
  test_load_rejects_unsupported_rotation();

  puts("mag_correct: calibration order, fine rotation and gating verified - OK");
  return 0;
}
```

- [ ] **Step 3: Write the test runner**

Create `tools/test-mag-correct.sh`, modelled on `tools/test-ekf-sources.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

mkdir -p "$OUT/nuttx"
cat > "$OUT/nuttx/config.h" <<'STUB'
#ifndef OK
#  define OK 0
#endif
STUB

cc -std=c11 -D_POSIX_C_SOURCE=200809L -Wall -Wextra -Werror \
  -Wno-unused-parameter \
  -Wno-missing-field-initializers -DFAR= -I"$OUT" \
  -I"$REPO/apps/sensors" -I"$REPO/apps/param" \
  -DPARAM_FILE='"/tmp/xxcar-mag-correct-no-file"' \
  -DPARAM_TMPFILE='"/tmp/xxcar-mag-correct-no-temp"' \
  "$REPO/tests/mag_correct_test.c" "$REPO/apps/sensors/mag_correct.c" \
  "$REPO/apps/sensors/rotation.c" \
  "$REPO/apps/param/param.c" -lm -o "$OUT/test"
"$OUT/test"
```

Then `chmod +x tools/test-mag-correct.sh`.

- [ ] **Step 4: Run the test to verify it fails**

```bash
tools/test-mag-correct.sh
```

Expected: FAIL at compile — `mag_correct.c: No such file or directory`.

- [ ] **Step 5: Implement `mag_correct.c`**

Create `apps/sensors/mag_correct.c`. `mag_correct.c` must not include `nuttx/config.h` unconditionally — the host test does not have it. Follow the `cal_mag.c` precedent and guard it.

```c
/****************************************************************************
 * apps/sensors/mag_correct.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef MAG_CORRECT_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "mag_correct.h"
#include "rotation.h"
#include "../param/param.h"

#define MAG_FINE_ROTATION_MIN 1.0e-9f

static bool vector_finite(FAR const float v[3])
{
  return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

/* Full Rodrigues rotation about the axis of rv by |rv|.
 *
 * Not the small-angle form. CAL_MAG0_RV* is bounded at 0.35 rad, where a
 * first-order approximation is already about 2 percent wrong AND no longer
 * norm-preserving - which would corrupt the field-magnitude health check the
 * EKF uses to decide whether the reading can be trusted at all.
 */

static void rotate_by_vector(FAR const float rv[3], FAR float v[3])
{
  float angle = sqrtf(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
  float axis[3];
  float cross[3];
  float dot;
  float sine;
  float cosine;
  int i;

  if (!isfinite(angle) || angle < MAG_FINE_ROTATION_MIN)
    {
      return;
    }

  for (i = 0; i < 3; i++)
    {
      axis[i] = rv[i] / angle;
    }

  sine = sinf(angle);
  cosine = cosf(angle);
  dot = axis[0] * v[0] + axis[1] * v[1] + axis[2] * v[2];
  cross[0] = axis[1] * v[2] - axis[2] * v[1];
  cross[1] = axis[2] * v[0] - axis[0] * v[2];
  cross[2] = axis[0] * v[1] - axis[1] * v[0];

  for (i = 0; i < 3; i++)
    {
      v[i] = v[i] * cosine + cross[i] * sine +
             axis[i] * dot * (1.0f - cosine);
    }
}

bool mag_correct_load(FAR struct mag_cal_s *cal)
{
  static const FAR char *offset_names[3] =
  {
    "CAL_MAG0_XOFF", "CAL_MAG0_YOFF", "CAL_MAG0_ZOFF"
  };

  static const FAR char *fine_names[3] =
  {
    "CAL_MAG0_RVX", "CAL_MAG0_RVY", "CAL_MAG0_RVZ"
  };

  float xy;
  float xz;
  float yz;
  int i;

  if (cal == NULL)
    {
      return false;
    }

  memset(cal, 0, sizeof(*cal));

  for (i = 0; i < 3; i++)
    {
      cal->offset[i] = param_f32(offset_names[i]);
      cal->fine_rv[i] = param_f32(fine_names[i]);
    }

  /* The stored soft-iron is symmetric: six parameters, nine entries. */

  xy = param_f32("CAL_MAG0_XY");
  xz = param_f32("CAL_MAG0_XZ");
  yz = param_f32("CAL_MAG0_YZ");

  cal->matrix[0][0] = param_f32("CAL_MAG0_XX");
  cal->matrix[1][1] = param_f32("CAL_MAG0_YY");
  cal->matrix[2][2] = param_f32("CAL_MAG0_ZZ");
  cal->matrix[0][1] = xy;
  cal->matrix[1][0] = xy;
  cal->matrix[0][2] = xz;
  cal->matrix[2][0] = xz;
  cal->matrix[1][2] = yz;
  cal->matrix[2][1] = yz;

  cal->field = param_f32("CAL_MAG0_FIELD");
  cal->mag_rot = (uint8_t)param_i32("SENS_MAG0_ROT");
  cal->board_rot = (uint8_t)param_i32("SENS_BOARD_ROT");

  cal->valid = param_i32("CAL_MAG0_OK") != 0 &&
               rotation_supported(cal->mag_rot) &&
               rotation_supported(cal->board_rot);

  return cal->valid;
}

bool mag_correct_apply(FAR const struct mag_cal_s *cal,
                       FAR const float raw[3], FAR float out[3])
{
  float centred[3];
  int row;
  int column;

  if (cal == NULL || raw == NULL || out == NULL || !vector_finite(raw))
    {
      return false;
    }

  if (!cal->valid)
    {
      memcpy(out, raw, 3 * sizeof(float));
      return false;
    }

  for (row = 0; row < 3; row++)
    {
      centred[row] = raw[row] - cal->offset[row];
    }

  for (row = 0; row < 3; row++)
    {
      out[row] = 0.0f;

      for (column = 0; column < 3; column++)
        {
          out[row] += cal->matrix[row][column] * centred[column];
        }
    }

  /* Sensor axes -> board axes -> vehicle axes. rotation_apply() returning
   * false means the value was validated at load and cannot fail here; treat
   * it as a failure anyway rather than publishing the wrong frame.
   */

  if (!rotation_apply(cal->mag_rot, out))
    {
      return false;
    }

  rotate_by_vector(cal->fine_rv, out);

  if (!rotation_apply(cal->board_rot, out))
    {
      return false;
    }

  return vector_finite(out);
}
```

- [ ] **Step 6: Add `-DMAG_CORRECT_HOST_TEST` to the runner**

In `tools/test-mag-correct.sh`, add `-DMAG_CORRECT_HOST_TEST` to the `cc` flags, after `-DFAR=`.

- [ ] **Step 7: Run the test to verify it passes**

```bash
tools/test-mag-correct.sh
```

Expected: `mag_correct: calibration order, fine rotation and gating verified - OK`

- [ ] **Step 8: Add to the sensors build**

In `apps/sensors/Makefile`, change the `CSRCS` line and its comment:

```make
# sensors.c is the daemon; rotation.c is the shared rotation table;
# dsp_filter.c is the native-rate three-axis FPU biquad;
# mag_correct.c is the magnetometer calibration and frame math;
# sensors_main.c is the `sensors` command.
CSRCS    += sensors.c rotation.c dsp_filter.c mag_correct.c
```

- [ ] **Step 9: Build and commit**

```bash
./tools/build.sh
git add apps/sensors/mag_correct.h apps/sensors/mag_correct.c \
        apps/sensors/Makefile tests/mag_correct_test.c \
        tools/test-mag-correct.sh
git commit -m "sensors: apply the magnetometer calibration in the sensor's own axes

Hard iron, then soft iron, then rotation - the same argument sensors.h
makes for the IMU. A sphere fit records what each chip axis reads, so
rotating first would subtract each offset from the wrong axis and still
produce something that looks like a field.

The fine CAL_MAG0_RV* rotation uses full Rodrigues rather than the
small-angle form: the parameter is bounded at 0.35 rad, where first order
is already ~2% wrong and no longer norm-preserving, which would corrupt
the field-magnitude health check downstream.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Flash A parameters

**Files:**
- Modify: `apps/param/param.c`
- Modify: `tests/param_range_test.c`

**Interfaces:**
- Produces: parameters `EK3_DELAY_MS` (int32), `EK3_ALT_M_NSE` (float), `EK3_ALT_I_GATE` (float), readable via `param_i32()` / `param_f32()`.

- [ ] **Step 1: Write the failing test**

In `tests/param_range_test.c`, read the file first and follow its existing assertion style. Add to `main()` before the final `puts`:

```c
  /* Flash A estimator parameters. The horizon defaults to zero so a fresh
   * flash reproduces the pre-horizon behaviour exactly, and raising it is a
   * deliberate act with a hardware check behind it.
   */

  assert(param_i32("EK3_DELAY_MS") == 0);
  assert(fabsf(param_f32("EK3_ALT_M_NSE") - 2.0f) < 1.0e-6f);
  assert(fabsf(param_f32("EK3_ALT_I_GATE") - 5.0f) < 1.0e-6f);

  /* The ring is sized for a bounded horizon; a larger value must clamp
   * rather than index past the end of it.
   */

  assert(param_set_i32("EK3_DELAY_MS", 500) < 0);
  assert(param_i32("EK3_DELAY_MS") == 100);
  assert(param_set_i32("EK3_DELAY_MS", 0) == 0);
```

If `math.h` is not already included in that file, add it.

- [ ] **Step 2: Run the test to verify it fails**

```bash
tools/test-param-range.sh
```

Expected: FAIL — assertion on `EK3_DELAY_MS`, because the parameter does not exist and `param_i32` returns 0 for an unknown name only by coincidence; the `param_set_i32` clamp assertion will fail definitively.

- [ ] **Step 3: Add the parameters**

In `apps/param/param.c`, immediately after the `EK3_SRC_OPTIONS` entry and before the `/* ---- Logging ... */` comment:

```c
  /* ---- EKF aiding: horizon and barometer ---------------------------------
   * EK3_DELAY_MS is how far behind real time the filter runs. Measurements
   * are fused against the state as it was when they were sampled, and the
   * output is re-propagated forward to the present for publication.
   *
   * It defaults to ZERO, which drains the ring every tick and reproduces the
   * pre-horizon behaviour exactly. That is deliberate: a default should
   * reproduce known-good behaviour, and it lets the timing change be proven
   * inert on hardware before any measurement starts correcting anything.
   *
   * The maximum is bounded by the IMU ring in ekf_delay.h. Gates are plain
   * sigma; ArduPilot stores its as integer sigma x 100, which this codebase
   * has no reason to copy.
   */

  { "EK3_DELAY_MS", PARAM_TYPE_INT32, I32(0), I32(0), I32(100),
    "EKF fusion horizon behind real time (ms)" },
  { "EK3_ALT_M_NSE", PARAM_TYPE_FLOAT, F32(2.0f), F32(0.1f), F32(100.0f),
    "Barometer height measurement noise (m)" },
  { "EK3_ALT_I_GATE", PARAM_TYPE_FLOAT, F32(5.0f), F32(1.0f), F32(100.0f),
    "Barometer height innovation gate (sigma)" },
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
tools/test-param-range.sh
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add apps/param/param.c tests/param_range_test.c
git commit -m "param: add the EKF horizon and barometer fusion parameters

EK3_DELAY_MS defaults to zero so a fresh flash reproduces today's
attitude exactly and the timing rewrite can be proven inert before
anything starts correcting. Its maximum is bounded by the IMU ring.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: `sensors aux` daemon

Publishes the two corrected aiding topics and applies the two rate parameters nothing has read until now.

**Files:**
- Create: `apps/sensors/aux.h`, `apps/sensors/aux.c`
- Modify: `apps/sensors/Makefile`, `apps/sensors/Kconfig`, `apps/sensors/sensors_main.c`

**Interfaces:**
- Consumes: `mag_correct_load()`, `mag_correct_apply()`, `struct mag_cal_s` from Task 2; `vehicle_mag_advertise/_publish`, `vehicle_baro_advertise/_publish` from Task 1.
- Produces: `struct sensors_aux_status_s`, `int sensors_aux_start(void)`, `int sensors_aux_stop(void)`, `void sensors_aux_status(FAR struct sensors_aux_status_s *out)`.

- [ ] **Step 1: Write the header**

Create `apps/sensors/aux.h`:

```c
/****************************************************************************
 * apps/sensors/aux.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The low-rate aiding sensors: magnetometer and barometer.
 *
 * Separate from sensors.c, which turns one raw IMU into corrected body-frame
 * topics at 400 Hz-plus. These two run at 50 Hz and 10 Hz and answer a
 * different question, so they get their own daemon rather than being folded
 * into a file whose stated purpose is the IMU.
 *
 * One thread polls both. At these rates two threads would buy nothing but
 * context switches.
 *
 * This is also the first code to honour SENS_MAG_RATE and SENS_BARO_RATE.
 * Both parameters have existed and been read by nothing; every driver
 * implements set_interval, and only apps/cal ever called orb_set_interval,
 * with hardcoded values.
 ****************************************************************************/

#ifndef __APPS_SENSORS_AUX_H
#define __APPS_SENSORS_AUX_H

#include <stdbool.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

struct sensors_aux_status_s
{
  bool     running;
  bool     mag_calibrated;    /* CAL_MAG0_OK and rotations supported */
  uint8_t  mag_rot;
  uint8_t  board_rot;
  uint32_t mag_rate_hz;       /* SENS_MAG_RATE as requested */
  uint32_t baro_rate_hz;      /* SENS_BARO_RATE as requested */
  uint32_t mag_out;           /* published */
  uint32_t baro_out;
  uint32_t mag_skipped;       /* correction or publication failed */
  uint32_t baro_skipped;
  float    mag_field[3];      /* last corrected body-frame field, Gauss */
  float    mag_magnitude;     /* |field|, Gauss */
  float    mag_expected;      /* CAL_MAG0_FIELD */
  float    baro_pressure;     /* last pressure, hPa */
  float    baro_temperature;  /* degrees C */
};

int  sensors_aux_start(void);
int  sensors_aux_stop(void);
void sensors_aux_status(FAR struct sensors_aux_status_s *out);

#endif /* __APPS_SENSORS_AUX_H */
```

- [ ] **Step 2: Implement the daemon**

Create `apps/sensors/aux.c`. Read `apps/ekf3/ekf3.c` lines 26–50 and 233–284 first and copy the start/stop/status lifecycle exactly — the `g_lock` mutex, the `volatile bool g_running` / `g_should_stop` pair, the `task_create` plus 1-second spin-wait in start, and the mirrored spin-wait in stop.

```c
/****************************************************************************
 * apps/sensors/aux.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/uorb.h>
#include <uORB/uORB.h>

#include "aux.h"
#include "mag_correct.h"
#include "../param/param.h"
#include "../uorb_msgs/uorb_msgs.h"

#define AUX_PRIORITY   (SCHED_PRIORITY_DEFAULT + 8)
#define AUX_STACK      2048
#define AUX_POLL_MS    200

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool g_running;
static volatile bool g_should_stop;
static struct sensors_aux_status_s g_status;

static uint64_t aux_now_us(void)
{
  struct timespec t;

  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000ull + (uint64_t)t.tv_nsec / 1000ull;
}

static void status_publish(FAR const struct sensors_aux_status_s *s)
{
  pthread_mutex_lock(&g_lock);
  g_status = *s;
  pthread_mutex_unlock(&g_lock);
}

/* Convert a rate in Hz to the microsecond interval orb_set_interval wants.
 * Clamped so a nonsense parameter cannot divide by zero.
 */

static unsigned rate_to_interval_us(int32_t hz)
{
  if (hz < 1)
    {
      hz = 1;
    }

  return (unsigned)(1000000 / hz);
}

static void handle_mag(int sub, int pub,
                       FAR const struct mag_cal_s *cal,
                       FAR struct sensors_aux_status_s *s)
{
  struct sensor_mag raw;
  struct vehicle_mag_s out;
  float in[3];
  float body[3];
  bool corrected;

  if (orb_copy(ORB_ID(sensor_mag), sub, &raw) < 0)
    {
      s->mag_skipped++;
      return;
    }

  in[0] = raw.x;
  in[1] = raw.y;
  in[2] = raw.z;

  corrected = mag_correct_apply(cal, in, body);

  /* mag_correct_apply() returns false both for "not calibrated, passed
   * through" and for "could not be corrected at all". The first leaves body
   * usable; the second does not. cal->valid tells them apart.
   */

  if (!corrected && cal->valid)
    {
      s->mag_skipped++;
      return;
    }

  memset(&out, 0, sizeof(out));
  out.timestamp = aux_now_us();
  out.timestamp_sample = raw.timestamp;
  memcpy(out.field, body, sizeof(out.field));
  out.temperature = raw.temperature;
  out.calibrated = corrected ? 1 : 0;
  out.instance = 0;

  if (vehicle_mag_publish(pub, &out) < 0)
    {
      s->mag_skipped++;
      return;
    }

  memcpy(s->mag_field, body, sizeof(s->mag_field));
  s->mag_magnitude = sqrtf(body[0] * body[0] + body[1] * body[1] +
                           body[2] * body[2]);
  s->mag_out++;
}

static void handle_baro(int sub, int pub,
                        FAR struct sensors_aux_status_s *s)
{
  struct sensor_baro raw;
  struct vehicle_baro_s out;

  if (orb_copy(ORB_ID(sensor_baro), sub, &raw) < 0)
    {
      s->baro_skipped++;
      return;
    }

  if (!isfinite(raw.pressure) || !isfinite(raw.temperature))
    {
      s->baro_skipped++;
      return;
    }

  memset(&out, 0, sizeof(out));
  out.timestamp = aux_now_us();
  out.timestamp_sample = raw.timestamp;
  out.pressure = raw.pressure;
  out.temperature = raw.temperature;

  if (vehicle_baro_publish(pub, &out) < 0)
    {
      s->baro_skipped++;
      return;
    }

  s->baro_pressure = raw.pressure;
  s->baro_temperature = raw.temperature;
  s->baro_out++;
}

static int aux_daemon(int argc, FAR char *argv[])
{
  struct sensors_aux_status_s status;
  struct mag_cal_s cal;
  struct pollfd fds[2];
  int mag_sub = -1;
  int baro_sub = -1;
  int mag_pub = -1;
  int baro_pub = -1;
  int result = EXIT_FAILURE;

  memset(&status, 0, sizeof(status));

  status.mag_calibrated = mag_correct_load(&cal);
  status.mag_rot = cal.mag_rot;
  status.board_rot = cal.board_rot;
  status.mag_expected = cal.field;
  status.mag_rate_hz = (uint32_t)param_i32("SENS_MAG_RATE");
  status.baro_rate_hz = (uint32_t)param_i32("SENS_BARO_RATE");

  mag_sub = orb_subscribe(ORB_ID(sensor_mag));
  baro_sub = orb_subscribe(ORB_ID(sensor_baro));

  if (mag_sub < 0 || baro_sub < 0)
    {
      syslog(LOG_ERR,
             "[aux] cannot subscribe %s%s%s (errno %d)\n",
             mag_sub < 0 ? "sensor_mag0" : "",
             mag_sub < 0 && baro_sub < 0 ? " and " : "",
             baro_sub < 0 ? "sensor_baro0" : "", errno);
      goto out;
    }

  /* First code in the tree to honour these two parameters. */

  orb_set_interval(mag_sub,
                   rate_to_interval_us((int32_t)status.mag_rate_hz));
  orb_set_interval(baro_sub,
                   rate_to_interval_us((int32_t)status.baro_rate_hz));

  mag_pub = vehicle_mag_advertise();
  baro_pub = vehicle_baro_advertise();

  /* Name the topic that failed. uorb_msgs.c records that "cannot advertise"
   * without a name cost a flash cycle to diagnose.
   */

  if (mag_pub < 0 || baro_pub < 0)
    {
      syslog(LOG_ERR, "[aux] cannot advertise %s%s%s (errno %d)\n",
             mag_pub < 0 ? "vehicle_mag" : "",
             mag_pub < 0 && baro_pub < 0 ? " and " : "",
             baro_pub < 0 ? "vehicle_baro" : "", errno);
      goto out;
    }

  fds[0].fd = mag_sub;
  fds[0].events = POLLIN;
  fds[1].fd = baro_sub;
  fds[1].events = POLLIN;

  g_running = true;
  status.running = true;
  status_publish(&status);

  syslog(LOG_INFO,
         "[aux] mag %" PRIu32 " Hz (%s), baro %" PRIu32 " Hz\n",
         status.mag_rate_hz,
         status.mag_calibrated ? "calibrated" : "RAW - not calibrated",
         status.baro_rate_hz);

  while (!g_should_stop)
    {
      int ready = poll(fds, 2, AUX_POLL_MS);

      if (ready < 0 && errno != EINTR)
        {
          break;
        }

      if ((fds[0].revents & POLLIN) != 0)
        {
          handle_mag(mag_sub, mag_pub, &cal, &status);
        }

      if ((fds[1].revents & POLLIN) != 0)
        {
          handle_baro(baro_sub, baro_pub, &status);
        }

      status_publish(&status);
    }

  result = EXIT_SUCCESS;

out:
  status.running = false;
  status_publish(&status);

  if (mag_sub >= 0)
    {
      orb_unsubscribe(mag_sub);
    }

  if (baro_sub >= 0)
    {
      orb_unsubscribe(baro_sub);
    }

  if (mag_pub >= 0)
    {
      orb_unadvertise(mag_pub);
    }

  if (baro_pub >= 0)
    {
      orb_unadvertise(baro_pub);
    }

  g_running = false;
  return result;
}

int sensors_aux_start(void)
{
  int task;
  int wait;

  if (g_running)
    {
      return -EALREADY;
    }

  g_should_stop = false;
  task = task_create("sensors_aux", AUX_PRIORITY, AUX_STACK,
                     aux_daemon, NULL);

  if (task < 0)
    {
      return -errno;
    }

  for (wait = 0; wait < 100 && !g_running; wait++)
    {
      usleep(10000);
    }

  return g_running ? 0 : -EIO;
}

int sensors_aux_stop(void)
{
  int wait;

  if (!g_running)
    {
      return -ESRCH;
    }

  g_should_stop = true;

  for (wait = 0; wait < 100 && g_running; wait++)
    {
      usleep(10000);
    }

  return g_running ? -ETIMEDOUT : 0;
}

void sensors_aux_status(FAR struct sensors_aux_status_s *out)
{
  pthread_mutex_lock(&g_lock);
  *out = g_status;
  pthread_mutex_unlock(&g_lock);
}
```

Add `#include <inttypes.h>` if the `PRIu32` in the syslog call does not resolve.

- [ ] **Step 3: Add the `sensors aux` subcommand**

In `apps/sensors/sensors_main.c`, add `#include "aux.h"` alongside the existing includes, extend `usage()`, add a `print_aux_status()`, and dispatch `argv[1] == "aux"` on `argv[2]`.

```c
static void print_aux_status(void)
{
  struct sensors_aux_status_s s;

  sensors_aux_status(&s);

  if (!s.running)
    {
      printf("sensors aux: stopped\n");
      return;
    }

  printf("sensors aux: running, mag %s\n",
         s.mag_calibrated ? "calibrated" : "NOT CALIBRATED - raw passthrough");
  printf("  rates  mag %" PRIu32 " Hz  baro %" PRIu32 " Hz"
         "  (SENS_MAG_RATE / SENS_BARO_RATE)\n",
         s.mag_rate_hz, s.baro_rate_hz);
  printf("  mag    %+.4f %+.4f %+.4f Gauss  |B| %.4f expected %.4f\n",
         (double)s.mag_field[0], (double)s.mag_field[1],
         (double)s.mag_field[2], (double)s.mag_magnitude,
         (double)s.mag_expected);
  printf("  baro   %.2f hPa  %.1f C\n",
         (double)s.baro_pressure, (double)s.baro_temperature);
  printf("  published  mag %" PRIu32 " (%" PRIu32 " skipped)"
         "  baro %" PRIu32 " (%" PRIu32 " skipped)\n",
         s.mag_out, s.mag_skipped, s.baro_out, s.baro_skipped);
}
```

In `main()`, before the `strcmp(argv[1], "start")` branch:

```c
  if (strcmp(argv[1], "aux") == 0)
    {
      if (argc < 3)
        {
          usage();
          return EXIT_FAILURE;
        }

      if (strcmp(argv[2], "start") == 0)
        {
          ret = sensors_aux_start();

          if (ret == -EALREADY)
            {
              printf("sensors aux: already running\n");
              return EXIT_FAILURE;
            }

          if (ret < 0)
            {
              printf("sensors aux: failed to start (%d) - check the syslog\n",
                     ret);
              return EXIT_FAILURE;
            }

          print_aux_status();
          return EXIT_SUCCESS;
        }

      if (strcmp(argv[2], "stop") == 0)
        {
          ret = sensors_aux_stop();

          if (ret == -ESRCH)
            {
              printf("sensors aux: not running\n");
              return EXIT_FAILURE;
            }

          printf("sensors aux: %s\n", ret == OK ? "stopped" : "did not stop");
          return ret == OK ? EXIT_SUCCESS : EXIT_FAILURE;
        }

      if (strcmp(argv[2], "status") == 0)
        {
          print_aux_status();
          return EXIT_SUCCESS;
        }

      usage();
      return EXIT_FAILURE;
    }
```

Extend `usage()`'s first line to `"Usage: sensors start | stop | status\n"
         "       sensors aux start | stop | status\n"` and add a short
paragraph describing the aux daemon and the two rate parameters.

- [ ] **Step 4: Add to the build**

`apps/sensors/Makefile` — extend `CSRCS`:

```make
CSRCS    += sensors.c rotation.c dsp_filter.c mag_correct.c aux.c
```

`apps/sensors/Kconfig` — append to the `XXCAR_SENSORS` help text:

```
		`sensors aux` is a second, low-rate daemon for the aiding sensors.
		It applies the stored magnetometer calibration and publishes
		vehicle_mag and vehicle_baro for the estimator. It is the first
		code to honour SENS_MAG_RATE and SENS_BARO_RATE, which until now
		were read by nothing.
```

- [ ] **Step 5: Build**

```bash
./tools/build.sh
```

Expected: exits 0.

- [ ] **Step 6: Commit**

```bash
git add apps/sensors/aux.h apps/sensors/aux.c apps/sensors/Makefile \
        apps/sensors/Kconfig apps/sensors/sensors_main.c
git commit -m "sensors: publish the calibrated aiding sensors as vehicle_mag/baro

One low-rate thread polls both; at 50 Hz and 10 Hz two threads would buy
nothing but context switches. Kept out of sensors.c, whose stated purpose
is turning one raw IMU into corrected body-frame topics.

This is also the first code to honour SENS_MAG_RATE and SENS_BARO_RATE.
Both parameters have existed since the sensor bring-up and been read by
nothing: every driver implements set_interval, but the only caller of
orb_set_interval was apps/cal, with hardcoded values.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: IMU ring and measurement queues

Pure data structure. No filter mathematics, no I/O — which is what makes the horizon arithmetic testable in isolation, and the horizon arithmetic is where off-by-one errors hide.

**Files:**
- Create: `apps/ekf3/ekf_delay.h`, `apps/ekf3/ekf_delay.c`
- Create: `tests/ekf_delay_test.c`, `tools/test-ekf-delay.sh`
- Modify: `apps/ekf3/Makefile`

**Interfaces:**
- Consumes: `struct ekf_imu_sample_s` from `apps/ekf3/ekf_core.h`.
- Produces: `struct ekf_baro_sample_s`, `struct ekf_mag_sample_s`, `struct ekf_delay_s`, and the functions listed in the header below.

- [ ] **Step 1: Write the header**

Create `apps/ekf3/ekf_delay.h`:

```c
/****************************************************************************
 * apps/ekf3/ekf_delay.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The fusion horizon: an IMU ring and timestamped measurement queues.
 *
 * The filter runs EK3_DELAY_MS behind real time so a measurement can be fused
 * against the state as it was when the measurement was taken, rather than
 * against a state that has since moved on. Publication re-propagates from the
 * horizon state forward over the samples the filter has not consumed yet, so
 * estimator_state remains a current-time topic.
 *
 * Samples are NOT removed when the filter consumes them. They stay in the ring
 * because the output predictor has to replay them. A separate index tracks how
 * far the filter has got; entries are only lost when the ring wraps around,
 * which is counted.
 *
 * This file holds no filter mathematics. That is what makes the horizon
 * arithmetic testable on its own, and the horizon arithmetic is exactly where
 * an off-by-one would otherwise hide behind plausible-looking attitude.
 ****************************************************************************/

#ifndef __APPS_EKF3_EKF_DELAY_H
#define __APPS_EKF3_EKF_DELAY_H

#include <stdbool.h>
#include <stdint.h>

#include "ekf_core.h"

/* 100 ms at 400 Hz is 40 samples. 48 leaves margin for jitter and for a
 * publication that arrives between two IMU packets. EK3_DELAY_MS is bounded
 * at 100 in the parameter table to match.
 */

#define EKF_DELAY_MAX_MS        100
#define EKF_IMU_RING_SIZE        48
#define EKF_MAG_QUEUE_SIZE        8
#define EKF_BARO_QUEUE_SIZE       4

struct ekf_baro_sample_s
{
  uint64_t timestamp_sample;
  float    pressure;        /* hPa */
  float    temperature;     /* degrees C */
};

struct ekf_mag_sample_s
{
  uint64_t timestamp_sample;
  float    field[3];        /* Gauss, body frame */
  bool     calibrated;
};

struct ekf_delay_s
{
  struct ekf_imu_sample_s imu[EKF_IMU_RING_SIZE];
  uint16_t imu_head;        /* next write slot */
  uint16_t imu_count;       /* valid entries, <= EKF_IMU_RING_SIZE */
  uint16_t imu_consumed;    /* entries already given to the filter */

  struct ekf_baro_sample_s baro[EKF_BARO_QUEUE_SIZE];
  uint16_t baro_head;
  uint16_t baro_count;

  struct ekf_mag_sample_s mag[EKF_MAG_QUEUE_SIZE];
  uint16_t mag_head;
  uint16_t mag_count;

  uint32_t horizon_us;

  uint32_t imu_overflow_count;   /* unconsumed sample overwritten */
  uint32_t baro_overflow_count;
  uint32_t mag_overflow_count;
};

/* horizon_ms is clamped to EKF_DELAY_MAX_MS. */

void ekf_delay_init(FAR struct ekf_delay_s *d, uint32_t horizon_ms);

/* Change the horizon without discarding buffered data. */

void ekf_delay_set_horizon(FAR struct ekf_delay_s *d, uint32_t horizon_ms);

/* Append. Returns false when the append overwrote an entry the filter had not
 * consumed yet, having still stored the new sample and counted the loss: the
 * newest data is always worth more than the oldest unprocessed data.
 */

bool ekf_delay_push_imu(FAR struct ekf_delay_s *d,
                        FAR const struct ekf_imu_sample_s *sample);
bool ekf_delay_push_baro(FAR struct ekf_delay_s *d,
                         FAR const struct ekf_baro_sample_s *sample);
bool ekf_delay_push_mag(FAR struct ekf_delay_s *d,
                        FAR const struct ekf_mag_sample_s *sample);

/* The absolute time the filter is allowed to advance to: now_us minus the
 * horizon, saturating at zero rather than wrapping.
 */

uint64_t ekf_delay_horizon_time(FAR const struct ekf_delay_s *d,
                                uint64_t now_us);

/* Copy the oldest unconsumed IMU sample into out and mark it consumed, if its
 * timestamp_sample is at or before horizon_time. Returns false when there is
 * nothing due, leaving out untouched.
 */

bool ekf_delay_next_imu(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                        FAR struct ekf_imu_sample_s *out);

/* Pop the oldest measurement whose sample time is at or before horizon_time.
 * Measurements older than max_age_us before horizon_time are DISCARDED rather
 * than returned: fusing a measurement the filter has already propagated past
 * would apply a correction at the wrong point on the trajectory.
 */

bool ekf_delay_next_baro(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                         uint64_t max_age_us,
                         FAR struct ekf_baro_sample_s *out);
bool ekf_delay_next_mag(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                        uint64_t max_age_us,
                        FAR struct ekf_mag_sample_s *out);

/* The samples the filter has not consumed: what the output predictor replays
 * to get from the horizon state to the present. index 0 is the oldest.
 */

uint16_t ekf_delay_output_count(FAR const struct ekf_delay_s *d);
FAR const struct ekf_imu_sample_s *
  ekf_delay_output_at(FAR const struct ekf_delay_s *d, uint16_t index);

#endif /* __APPS_EKF3_EKF_DELAY_H */
```

- [ ] **Step 2: Write the failing test**

Create `tests/ekf_delay_test.c`:

```c
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ekf_delay.h"

static struct ekf_delay_s g_delay;

static struct ekf_imu_sample_s imu_at(uint64_t t)
{
  struct ekf_imu_sample_s s;

  memset(&s, 0, sizeof(s));
  s.timestamp_sample = t;
  s.timestamp_first = t - 2500;
  s.delta_angle_dt = 0.0025f;
  s.delta_velocity_dt = 0.0025f;
  s.samples = 5;
  return s;
}

/* A zero horizon must make every sample immediately due. This is the property
 * the whole "prove the rewrite is inert" argument rests on: at EK3_DELAY_MS=0
 * the ring is a pass-through and the filter sees exactly what it saw before.
 */

static void test_zero_horizon_is_passthrough(void)
{
  struct ekf_imu_sample_s out;

  ekf_delay_init(&g_delay, 0);

  for (uint64_t t = 1000; t <= 5000; t += 1000)
    {
      struct ekf_imu_sample_s s = imu_at(t);
      assert(ekf_delay_push_imu(&g_delay, &s));
    }

  for (uint64_t t = 1000; t <= 5000; t += 1000)
    {
      assert(ekf_delay_next_imu(&g_delay, ekf_delay_horizon_time(&g_delay,
                                                                 5000), &out));
      assert(out.timestamp_sample == t);
    }

  assert(!ekf_delay_next_imu(&g_delay,
                             ekf_delay_horizon_time(&g_delay, 5000), &out));
  assert(ekf_delay_output_count(&g_delay) == 0);
}

/* With a 10 ms horizon and "now" at 20 ms, only samples at or before 10 ms are
 * due. The rest stay for the output predictor.
 */

static void test_horizon_withholds_recent(void)
{
  struct ekf_imu_sample_s out;
  uint64_t horizon;

  ekf_delay_init(&g_delay, 10);

  for (uint64_t t = 2500; t <= 20000; t += 2500)
    {
      struct ekf_imu_sample_s s = imu_at(t);
      assert(ekf_delay_push_imu(&g_delay, &s));
    }

  horizon = ekf_delay_horizon_time(&g_delay, 20000);
  assert(horizon == 10000);

  for (uint64_t t = 2500; t <= 10000; t += 2500)
    {
      assert(ekf_delay_next_imu(&g_delay, horizon, &out));
      assert(out.timestamp_sample == t);
    }

  assert(!ekf_delay_next_imu(&g_delay, horizon, &out));

  /* 12500, 15000, 17500, 20000 remain for re-propagation, oldest first. */

  assert(ekf_delay_output_count(&g_delay) == 4);
  assert(ekf_delay_output_at(&g_delay, 0)->timestamp_sample == 12500);
  assert(ekf_delay_output_at(&g_delay, 3)->timestamp_sample == 20000);
}

/* The horizon must saturate rather than wrap. now_us < horizon happens for
 * the first few milliseconds after boot, and an unsigned wrap there would
 * produce a horizon far in the future and drain the entire ring at once.
 */

static void test_horizon_saturates(void)
{
  ekf_delay_init(&g_delay, 100);
  assert(ekf_delay_horizon_time(&g_delay, 0) == 0);
  assert(ekf_delay_horizon_time(&g_delay, 50000) == 0);
  assert(ekf_delay_horizon_time(&g_delay, 150000) == 50000);
}

static void test_horizon_clamped(void)
{
  ekf_delay_init(&g_delay, 5000);
  assert(g_delay.horizon_us == (uint32_t)EKF_DELAY_MAX_MS * 1000u);
}

/* Overwriting an unconsumed sample is counted and reported, and the NEW
 * sample survives. Losing the newest data to preserve the oldest unprocessed
 * data would be the wrong trade for an estimator.
 */

static void test_overflow_counted(void)
{
  int i;
  bool saw_false = false;

  ekf_delay_init(&g_delay, 100);

  for (i = 0; i < EKF_IMU_RING_SIZE + 5; i++)
    {
      struct ekf_imu_sample_s s = imu_at((uint64_t)(i + 1) * 2500);

      if (!ekf_delay_push_imu(&g_delay, &s))
        {
          saw_false = true;
        }
    }

  assert(saw_false);
  assert(g_delay.imu_overflow_count == 5);
  assert(g_delay.imu_count == EKF_IMU_RING_SIZE);

  /* The newest sample is present; the oldest five are gone. */

  assert(ekf_delay_output_at(&g_delay, EKF_IMU_RING_SIZE - 1)
           ->timestamp_sample == (uint64_t)(EKF_IMU_RING_SIZE + 5) * 2500);
}

/* A measurement older than the age bound is discarded, not fused late. The
 * filter has already propagated past it, so applying it would correct the
 * wrong point on the trajectory.
 */

static void test_stale_measurement_discarded(void)
{
  struct ekf_baro_sample_s out;
  struct ekf_baro_sample_s old = {.timestamp_sample = 1000,
                                  .pressure = 1000.0f};
  struct ekf_baro_sample_s fresh = {.timestamp_sample = 900000,
                                    .pressure = 1013.0f};

  ekf_delay_init(&g_delay, 0);
  assert(ekf_delay_push_baro(&g_delay, &old));
  assert(ekf_delay_push_baro(&g_delay, &fresh));

  /* horizon 1000000, age bound 500000: the 1000 sample is 999 ms old. */

  assert(ekf_delay_next_baro(&g_delay, 1000000, 500000, &out));
  assert(out.timestamp_sample == 900000);
  assert(!ekf_delay_next_baro(&g_delay, 1000000, 500000, &out));
}

/* A measurement from the future relative to the horizon waits its turn. */

static void test_future_measurement_waits(void)
{
  struct ekf_baro_sample_s out;
  struct ekf_baro_sample_s s = {.timestamp_sample = 50000,
                                .pressure = 1013.0f};

  ekf_delay_init(&g_delay, 0);
  assert(ekf_delay_push_baro(&g_delay, &s));
  assert(!ekf_delay_next_baro(&g_delay, 10000, 500000, &out));
  assert(ekf_delay_next_baro(&g_delay, 60000, 500000, &out));
  assert(out.timestamp_sample == 50000);
}

static void test_mag_queue(void)
{
  struct ekf_mag_sample_s out;
  struct ekf_mag_sample_s s;
  int i;

  ekf_delay_init(&g_delay, 0);

  for (i = 0; i < EKF_MAG_QUEUE_SIZE + 2; i++)
    {
      memset(&s, 0, sizeof(s));
      s.timestamp_sample = (uint64_t)(i + 1) * 20000;
      s.field[0] = 0.25f;
      s.calibrated = true;
      ekf_delay_push_mag(&g_delay, &s);
    }

  assert(g_delay.mag_overflow_count == 2);
  assert(ekf_delay_next_mag(&g_delay, 10000000, 10000000, &out));
  assert(out.timestamp_sample == 3 * 20000);   /* first two dropped */
  assert(out.calibrated);
}

int main(void)
{
  test_zero_horizon_is_passthrough();
  test_horizon_withholds_recent();
  test_horizon_saturates();
  test_horizon_clamped();
  test_overflow_counted();
  test_stale_measurement_discarded();
  test_future_measurement_waits();
  test_mag_queue();

  puts("ekf_delay: horizon, ring ordering and overflow verified - OK");
  return 0;
}
```

- [ ] **Step 3: Write the test runner**

Create `tools/test-ekf-delay.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -D_POSIX_C_SOURCE=200809L -Wall -Wextra -Werror \
  -Wno-unused-parameter -Wno-missing-field-initializers \
  -DFAR= -DEKF_CORE_HOST_TEST \
  -I"$REPO/apps/ekf3" \
  "$REPO/tests/ekf_delay_test.c" "$REPO/apps/ekf3/ekf_delay.c" \
  -lm -o "$OUT/test"
"$OUT/test"
```

Then `chmod +x tools/test-ekf-delay.sh`.

- [ ] **Step 4: Run the test to verify it fails**

```bash
tools/test-ekf-delay.sh
```

Expected: FAIL at compile — `ekf_delay.c: No such file or directory`.

- [ ] **Step 5: Implement `ekf_delay.c`**

Create `apps/ekf3/ekf_delay.c`. Ring indexing convention: `imu_head` is the next write slot; the oldest valid entry is at `(imu_head - imu_count + EKF_IMU_RING_SIZE) % EKF_IMU_RING_SIZE`.

```c
/****************************************************************************
 * apps/ekf3/ekf_delay.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef EKF_CORE_HOST_TEST
#  include <nuttx/config.h>
#endif

#include <string.h>

#include "ekf_delay.h"

static uint16_t ring_oldest(uint16_t head, uint16_t count, uint16_t size)
{
  return (uint16_t)((head + size - count) % size);
}

void ekf_delay_init(FAR struct ekf_delay_s *d, uint32_t horizon_ms)
{
  if (d == NULL)
    {
      return;
    }

  memset(d, 0, sizeof(*d));
  ekf_delay_set_horizon(d, horizon_ms);
}

void ekf_delay_set_horizon(FAR struct ekf_delay_s *d, uint32_t horizon_ms)
{
  if (d == NULL)
    {
      return;
    }

  if (horizon_ms > (uint32_t)EKF_DELAY_MAX_MS)
    {
      horizon_ms = (uint32_t)EKF_DELAY_MAX_MS;
    }

  d->horizon_us = horizon_ms * 1000u;
}

bool ekf_delay_push_imu(FAR struct ekf_delay_s *d,
                        FAR const struct ekf_imu_sample_s *sample)
{
  bool lost = false;

  if (d == NULL || sample == NULL)
    {
      return false;
    }

  /* The new sample always goes in. When that costs an unconsumed entry, say
   * so and count it - but do not drop the newest data to keep the oldest
   * unprocessed data, which is the wrong trade for an estimator.
   */

  if (d->imu_count == EKF_IMU_RING_SIZE && d->imu_consumed == 0)
    {
      d->imu_overflow_count++;
      lost = true;
    }

  d->imu[d->imu_head] = *sample;
  d->imu_head = (uint16_t)((d->imu_head + 1) % EKF_IMU_RING_SIZE);

  if (d->imu_count < EKF_IMU_RING_SIZE)
    {
      d->imu_count++;
    }
  else if (d->imu_consumed > 0)
    {
      d->imu_consumed--;
    }

  return !lost;
}

bool ekf_delay_push_baro(FAR struct ekf_delay_s *d,
                         FAR const struct ekf_baro_sample_s *sample)
{
  bool lost = false;

  if (d == NULL || sample == NULL)
    {
      return false;
    }

  if (d->baro_count == EKF_BARO_QUEUE_SIZE)
    {
      d->baro_overflow_count++;
      d->baro_count--;
      lost = true;
    }

  d->baro[d->baro_head] = *sample;
  d->baro_head = (uint16_t)((d->baro_head + 1) % EKF_BARO_QUEUE_SIZE);
  d->baro_count++;
  return !lost;
}

bool ekf_delay_push_mag(FAR struct ekf_delay_s *d,
                        FAR const struct ekf_mag_sample_s *sample)
{
  bool lost = false;

  if (d == NULL || sample == NULL)
    {
      return false;
    }

  if (d->mag_count == EKF_MAG_QUEUE_SIZE)
    {
      d->mag_overflow_count++;
      d->mag_count--;
      lost = true;
    }

  d->mag[d->mag_head] = *sample;
  d->mag_head = (uint16_t)((d->mag_head + 1) % EKF_MAG_QUEUE_SIZE);
  d->mag_count++;
  return !lost;
}

uint64_t ekf_delay_horizon_time(FAR const struct ekf_delay_s *d,
                                uint64_t now_us)
{
  if (d == NULL || now_us <= (uint64_t)d->horizon_us)
    {
      return 0;
    }

  return now_us - (uint64_t)d->horizon_us;
}

bool ekf_delay_next_imu(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                        FAR struct ekf_imu_sample_s *out)
{
  uint16_t index;

  if (d == NULL || out == NULL || d->imu_consumed >= d->imu_count)
    {
      return false;
    }

  index = (uint16_t)((ring_oldest(d->imu_head, d->imu_count,
                                  EKF_IMU_RING_SIZE) + d->imu_consumed) %
                     EKF_IMU_RING_SIZE);

  if (d->imu[index].timestamp_sample > horizon_time)
    {
      return false;
    }

  *out = d->imu[index];
  d->imu_consumed++;
  return true;
}

bool ekf_delay_next_baro(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                         uint64_t max_age_us,
                         FAR struct ekf_baro_sample_s *out)
{
  if (d == NULL || out == NULL)
    {
      return false;
    }

  while (d->baro_count > 0)
    {
      uint16_t index = ring_oldest(d->baro_head, d->baro_count,
                                   EKF_BARO_QUEUE_SIZE);
      uint64_t stamp = d->baro[index].timestamp_sample;

      if (stamp > horizon_time)
        {
          return false;
        }

      d->baro_count--;

      if (horizon_time - stamp <= max_age_us)
        {
          *out = d->baro[index];
          return true;
        }

      /* Too old to fuse where the filter now is. Drop and look at the next. */
    }

  return false;
}

bool ekf_delay_next_mag(FAR struct ekf_delay_s *d, uint64_t horizon_time,
                        uint64_t max_age_us,
                        FAR struct ekf_mag_sample_s *out)
{
  if (d == NULL || out == NULL)
    {
      return false;
    }

  while (d->mag_count > 0)
    {
      uint16_t index = ring_oldest(d->mag_head, d->mag_count,
                                   EKF_MAG_QUEUE_SIZE);
      uint64_t stamp = d->mag[index].timestamp_sample;

      if (stamp > horizon_time)
        {
          return false;
        }

      d->mag_count--;

      if (horizon_time - stamp <= max_age_us)
        {
          *out = d->mag[index];
          return true;
        }
    }

  return false;
}

uint16_t ekf_delay_output_count(FAR const struct ekf_delay_s *d)
{
  if (d == NULL || d->imu_consumed >= d->imu_count)
    {
      return 0;
    }

  return (uint16_t)(d->imu_count - d->imu_consumed);
}

FAR const struct ekf_imu_sample_s *
  ekf_delay_output_at(FAR const struct ekf_delay_s *d, uint16_t index)
{
  uint16_t slot;

  if (d == NULL || index >= ekf_delay_output_count(d))
    {
      return NULL;
    }

  slot = (uint16_t)((ring_oldest(d->imu_head, d->imu_count,
                                 EKF_IMU_RING_SIZE) + d->imu_consumed +
                     index) % EKF_IMU_RING_SIZE);
  return &d->imu[slot];
}
```

- [ ] **Step 6: Run the test to verify it passes**

```bash
tools/test-ekf-delay.sh
```

Expected: `ekf_delay: horizon, ring ordering and overflow verified - OK`

If `test_overflow_counted` fails, the likely cause is the interaction between `imu_consumed` and wrap. Re-read `ekf_delay_push_imu`: when the ring is full and some entries have been consumed, the overwritten entry is a consumed one, so `imu_consumed` decrements and nothing is lost.

- [ ] **Step 7: Add to the build and commit**

`apps/ekf3/Makefile`:

```make
CSRCS   = ekf3.c ekf_core.c ekf_sources.c ekf_delay.c
```

```bash
./tools/build.sh
git add apps/ekf3/ekf_delay.h apps/ekf3/ekf_delay.c apps/ekf3/Makefile \
        tests/ekf_delay_test.c tools/test-ekf-delay.sh
git commit -m "ekf3: add the fusion horizon ring and measurement queues

Samples are not removed when the filter consumes them - the output
predictor has to replay them - so a separate index tracks how far the
filter has got and entries are only lost on wrap, which is counted.

A measurement older than the age bound is discarded rather than fused
late: the filter has already propagated past it, so the correction would
land at the wrong point on the trajectory.

No filter mathematics here, which is what makes the horizon arithmetic
testable on its own.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Output predictor

The nominal strapdown step is factored out of `nominal_predict()` so the filter and the predictor cannot drift apart. Two copies of this integration that were meant to be identical would be a slow, silent divergence between what the filter believes and what the vehicle is told.

**Files:**
- Modify: `apps/ekf3/ekf_core.h`, `apps/ekf3/ekf_core.c`
- Create: `tests/ekf_output_test.c`, `tools/test-ekf-output.sh`

**Interfaces:**
- Consumes: `struct ekf_core_s`, `struct ekf_imu_sample_s`.
- Produces: `struct ekf_output_s`, `void ekf_core_output_predict(FAR const struct ekf_core_s *ekf, FAR const struct ekf_imu_sample_s *const *samples, uint16_t count, FAR struct ekf_output_s *out)`.

- [ ] **Step 1: Add the output type to `ekf_core.h`**

After the `ekf_core_s` definition:

```c
/* Current-time state, re-propagated from the delayed filter state.
 *
 * Deliberately small: quaternion, velocity, position and nothing else. The
 * alternative - copying the whole 1.2 kB ekf_core_s and propagating that -
 * would move covariance the predictor never touches, 400 times a second.
 */

struct ekf_output_s
{
  float    quaternion[4];
  float    velocity[3];
  float    position[3];
  uint64_t timestamp_sample;
  uint16_t samples_replayed;
  bool     valid;
};

/* Replay count samples forward from the filter state. samples is an array of
 * POINTERS because the source is a ring buffer and the entries are not
 * contiguous. With count == 0 the result is the filter state itself.
 */

void ekf_core_output_predict(FAR const struct ekf_core_s *ekf,
                             FAR const struct ekf_imu_sample_s *const *samples,
                             uint16_t count,
                             FAR struct ekf_output_s *out);
```

- [ ] **Step 2: Write the failing test**

Create `tests/ekf_output_test.c`:

```c
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ekf_core.h"

#define CLOSE(a, b) (fabsf((a) - (b)) < 1.0e-5f)

static struct ekf_core_s g_core;

/* Drive the core through alignment with a clean stationary sequence, so the
 * tests below start from an initialised filter rather than an identity guess.
 */

static void align_core(void)
{
  struct ekf_imu_sample_s s;
  uint64_t t = 0;
  int i;

  ekf_core_init(&g_core);

  for (i = 0; i < 500; i++)
    {
      memset(&s, 0, sizeof(s));
      t += 2500;
      s.timestamp_sample = t;
      s.timestamp_first = t - 2500;
      s.delta_angle_dt = 0.0025f;
      s.delta_velocity_dt = 0.0025f;
      s.delta_velocity[2] = -9.80665f * 0.0025f;
      s.samples = 5;
      s.accel_calibrated = true;
      s.gyro_calibrated = true;
      ekf_core_process(&g_core, &s);
    }

  assert(g_core.initialized);
}

/* Zero samples must reproduce the filter state exactly. This is what makes
 * "EK3_DELAY_MS=0 is inert" true at the publication end as well.
 */

static void test_empty_is_identity(void)
{
  struct ekf_output_s out;

  align_core();
  ekf_core_output_predict(&g_core, NULL, 0, &out);

  assert(out.valid);
  assert(out.samples_replayed == 0);
  assert(CLOSE(out.quaternion[0], g_core.quaternion[0]));
  assert(CLOSE(out.quaternion[3], g_core.quaternion[3]));
  assert(CLOSE(out.velocity[2], g_core.velocity[2]));
  assert(CLOSE(out.position[2], g_core.position[2]));
  assert(out.timestamp_sample == g_core.last_timestamp_sample);
}

/* Replaying N samples through the predictor must land where the filter lands
 * after processing the same N samples. If these two integrations disagree,
 * the published attitude is not the attitude the filter believes.
 *
 * The motion here is deliberately energetic - 0.5 rad/s, above the 0.20 rad/s
 * EKF_DYNAMICS_GYRO_OUT threshold - so the filter leaves low-dynamics on the
 * first sample and its gravity update stops firing. The predictor only does
 * strapdown; comparing it against a filter that is also applying gravity
 * corrections would be comparing two different things and the test would fail
 * for a reason that is not a bug.
 */

static void test_matches_filter_propagation(void)
{
  struct ekf_imu_sample_s seq[8];
  FAR const struct ekf_imu_sample_s *ptr[8];
  struct ekf_core_s reference;
  struct ekf_output_s out;
  uint64_t t;
  int i;

  align_core();
  t = g_core.last_timestamp_sample;

  for (i = 0; i < 8; i++)
    {
      memset(&seq[i], 0, sizeof(seq[i]));
      t += 2500;
      seq[i].timestamp_sample = t;
      seq[i].timestamp_first = t - 2500;
      seq[i].delta_angle_dt = 0.0025f;
      seq[i].delta_velocity_dt = 0.0025f;
      seq[i].delta_angle[0] = 0.5f * 0.0025f;    /* rolling, out of low-dyn */
      seq[i].delta_angle[2] = 0.3f * 0.0025f;    /* yawing */
      seq[i].delta_velocity[0] = 0.5f * 0.0025f; /* accelerating */
      seq[i].delta_velocity[2] = -9.80665f * 0.0025f;
      seq[i].samples = 5;
      seq[i].accel_calibrated = true;
      seq[i].gyro_calibrated = true;
      ptr[i] = &seq[i];
    }

  ekf_core_output_predict(&g_core, ptr, 8, &out);

  reference = g_core;

  for (i = 0; i < 8; i++)
    {
      assert(ekf_core_process(&reference, &seq[i]) ==
             EKF_PROCESS_PREDICTED);
    }

  /* If this trips, the sequence was not energetic enough to leave
   * low-dynamics and the gravity update polluted the comparison.
   */

  assert(!reference.low_dynamics);
  assert(reference.gravity_accept_count == g_core.gravity_accept_count);

  assert(out.samples_replayed == 8);
  assert(CLOSE(out.quaternion[0], reference.quaternion[0]));
  assert(CLOSE(out.quaternion[1], reference.quaternion[1]));
  assert(CLOSE(out.quaternion[2], reference.quaternion[2]));
  assert(CLOSE(out.quaternion[3], reference.quaternion[3]));
  assert(CLOSE(out.velocity[0], reference.velocity[0]));
  assert(CLOSE(out.velocity[2], reference.velocity[2]));
  assert(CLOSE(out.position[0], reference.position[0]));
  assert(out.timestamp_sample == seq[7].timestamp_sample);
}

/* The predictor must not modify the filter. It is called on every
 * publication; a predictor that mutated the core would integrate the same
 * samples twice.
 */

static void test_does_not_mutate_core(void)
{
  struct ekf_imu_sample_s s;
  FAR const struct ekf_imu_sample_s *ptr[1];
  struct ekf_core_s before;
  struct ekf_output_s out;

  align_core();
  before = g_core;

  memset(&s, 0, sizeof(s));
  s.timestamp_sample = g_core.last_timestamp_sample + 2500;
  s.timestamp_first = g_core.last_timestamp_sample;
  s.delta_angle_dt = 0.0025f;
  s.delta_velocity_dt = 0.0025f;
  s.delta_angle[0] = 0.05f;
  s.samples = 5;
  s.accel_calibrated = true;
  s.gyro_calibrated = true;
  ptr[0] = &s;

  ekf_core_output_predict(&g_core, ptr, 1, &out);

  assert(memcmp(&before, &g_core, sizeof(before)) == 0);
}

/* Repeated calls with the same input give the same answer. The predictor is a
 * pure function of (filter state, samples); anything else means hidden state.
 */

static void test_deterministic(void)
{
  struct ekf_imu_sample_s s;
  FAR const struct ekf_imu_sample_s *ptr[1];
  struct ekf_output_s first;
  struct ekf_output_s second;

  align_core();

  memset(&s, 0, sizeof(s));
  s.timestamp_sample = g_core.last_timestamp_sample + 2500;
  s.timestamp_first = g_core.last_timestamp_sample;
  s.delta_angle_dt = 0.0025f;
  s.delta_velocity_dt = 0.0025f;
  s.delta_angle[1] = 0.02f;
  s.samples = 5;
  s.accel_calibrated = true;
  s.gyro_calibrated = true;
  ptr[0] = &s;

  ekf_core_output_predict(&g_core, ptr, 1, &first);
  ekf_core_output_predict(&g_core, ptr, 1, &second);

  assert(memcmp(&first, &second, sizeof(first)) == 0);
}

int main(void)
{
  test_empty_is_identity();
  test_matches_filter_propagation();
  test_does_not_mutate_core();
  test_deterministic();

  puts("ekf_output: re-propagation matches the filter and is pure - OK");
  return 0;
}
```

- [ ] **Step 3: Write the test runner**

Create `tools/test-ekf-output.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -D_POSIX_C_SOURCE=200809L -Wall -Wextra -Werror \
  -Wno-unused-parameter -Wno-missing-field-initializers \
  -DFAR= -DEKF_CORE_HOST_TEST \
  -I"$REPO/apps/ekf3" \
  "$REPO/tests/ekf_output_test.c" "$REPO/apps/ekf3/ekf_core.c" \
  -lm -o "$OUT/test"
"$OUT/test"
```

Then `chmod +x tools/test-ekf-output.sh`.

- [ ] **Step 4: Run the test to verify it fails**

```bash
tools/test-ekf-output.sh
```

Expected: FAIL at link — `undefined reference to 'ekf_core_output_predict'`.

- [ ] **Step 5: Factor the strapdown step out of `nominal_predict()`**

In `apps/ekf3/ekf_core.c`, insert this immediately before `nominal_predict()`:

```c
/* One strapdown integration step, shared by the filter and the output
 * predictor.
 *
 * Factored out rather than duplicated because two copies of this that were
 * meant to be identical would be a slow, silent divergence between what the
 * filter believes and what the vehicle is told. Returns false when the step
 * produces a non-finite state.
 */

static bool strapdown_step(FAR float quaternion[4], FAR float velocity[3],
                           FAR float position[3],
                           FAR const float gyro_bias[3],
                           FAR const float accel_bias[3],
                           FAR const struct ekf_imu_sample_s *sample)
{
  float corrected_angle[3];
  float corrected_velocity[3];
  float half_angle[3];
  float increment[4];
  float half_increment[4];
  float midpoint[4];
  float next_quaternion[4];
  float rotation[3][3];
  float nav_delta_velocity[3];
  float old_velocity[3];
  float dt = sample->delta_angle_dt;
  int row;
  int axis;

  for (axis = 0; axis < 3; axis++)
    {
      corrected_angle[axis] = sample->delta_angle[axis] -
                              gyro_bias[axis] * dt;
      corrected_velocity[axis] = sample->delta_velocity[axis] -
                                 accel_bias[axis] * dt;
      half_angle[axis] = 0.5f * corrected_angle[axis];
      old_velocity[axis] = velocity[axis];
    }

  rotation_vector_quaternion(half_angle, half_increment);
  quaternion_multiply(quaternion, half_increment, midpoint);

  if (!quaternion_normalize(midpoint))
    {
      return false;
    }

  quaternion_to_rotation(midpoint, rotation);

  for (row = 0; row < 3; row++)
    {
      nav_delta_velocity[row] =
        rotation[row][0] * corrected_velocity[0] +
        rotation[row][1] * corrected_velocity[1] +
        rotation[row][2] * corrected_velocity[2];
    }

  nav_delta_velocity[2] -= EKF_GRAVITY * dt;

  for (axis = 0; axis < 3; axis++)
    {
      velocity[axis] += nav_delta_velocity[axis];
      position[axis] +=
        (old_velocity[axis] + 0.5f * nav_delta_velocity[axis]) * dt;
    }

  rotation_vector_quaternion(corrected_angle, increment);
  quaternion_multiply(quaternion, increment, next_quaternion);
  memcpy(quaternion, next_quaternion, 4 * sizeof(float));

  return quaternion_normalize(quaternion) &&
         vector_finite(velocity) && vector_finite(position);
}
```

- [ ] **Step 6: Rewrite `nominal_predict()` to use it**

Replace the body of `nominal_predict()` down to (but not including) the `ekf->covariance_dt += dt;` line with:

```c
static bool nominal_predict(FAR struct ekf_core_s *ekf,
                            FAR const struct ekf_imu_sample_s *sample)
{
  float dt = sample->delta_angle_dt;
  int axis;

  if (!strapdown_step(ekf->quaternion, ekf->velocity, ekf->position,
                      ekf->gyro_bias, ekf->accel_bias, sample))
    {
      return false;
    }

  for (axis = 0; axis < 3; axis++)
    {
      ekf->covariance_delta_angle[axis] += sample->delta_angle[axis];
      ekf->covariance_delta_velocity[axis] +=
        sample->delta_velocity[axis];
    }

  ekf->covariance_dt += dt;
```

Keep the remainder of the function — the `covariance_phase++`, clipping OR, `predict_count++`, and the `EKF_COVARIANCE_INTERVAL` block — exactly as it is.

- [ ] **Step 7: Verify the refactor changed nothing**

```bash
tools/test-ekf-core.sh
```

Expected: PASS. This is the existing suite; it must still pass before the new function is added, which is what proves the extraction was behaviour-preserving.

- [ ] **Step 8: Implement `ekf_core_output_predict()`**

Add at the end of `ekf_core.c`:

```c
void ekf_core_output_predict(FAR const struct ekf_core_s *ekf,
                             FAR const struct ekf_imu_sample_s *const *samples,
                             uint16_t count,
                             FAR struct ekf_output_s *out)
{
  uint16_t index;

  if (ekf == NULL || out == NULL)
    {
      return;
    }

  memset(out, 0, sizeof(*out));
  memcpy(out->quaternion, ekf->quaternion, sizeof(out->quaternion));
  memcpy(out->velocity, ekf->velocity, sizeof(out->velocity));
  memcpy(out->position, ekf->position, sizeof(out->position));
  out->timestamp_sample = ekf->last_timestamp_sample;
  out->valid = ekf->initialized;

  if (!ekf->initialized || samples == NULL)
    {
      return;
    }

  for (index = 0; index < count; index++)
    {
      FAR const struct ekf_imu_sample_s *sample = samples[index];

      if (sample == NULL)
        {
          break;
        }

      /* The bias estimates are the filter's, held fixed across the replay.
       * They change on the scale of minutes; the replay spans milliseconds.
       */

      if (!strapdown_step(out->quaternion, out->velocity, out->position,
                          ekf->gyro_bias, ekf->accel_bias, sample))
        {
          out->valid = false;
          return;
        }

      out->timestamp_sample = sample->timestamp_sample;
      out->samples_replayed++;
    }
}
```

- [ ] **Step 9: Run the test to verify it passes**

```bash
tools/test-ekf-output.sh && tools/test-ekf-core.sh
```

Expected: both PASS.

- [ ] **Step 10: Commit**

```bash
./tools/build.sh
git add apps/ekf3/ekf_core.h apps/ekf3/ekf_core.c \
        tests/ekf_output_test.c tools/test-ekf-output.sh
git commit -m "ekf3: re-propagate the published state from the delayed filter state

The strapdown step is factored out of nominal_predict() and shared. Two
copies of this integration that were meant to be identical would be a
slow, silent divergence between what the filter believes and what the
vehicle is told.

Chosen over an ArduPilot-style output observer because it is
deterministic, cannot drift, and has no tuning constants - so it adds no
new failure mode to debug alongside first-ever aiding. The output state
is quaternion/velocity/position only; copying the whole 1.2 kB core 400
times a second to move covariance the predictor never touches would not
be worth it.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: Wire the horizon into the daemon

This is the flash-candidate task for the "prove it inert" step. With `EK3_DELAY_MS=0` the observable behaviour must be indistinguishable from the previous firmware.

**Files:**
- Modify: `apps/ekf3/ekf3.h`, `apps/ekf3/ekf3.c`

**Interfaces:**
- Consumes: `ekf_delay_*` from Task 5, `ekf_core_output_predict()` from Task 6, `vehicle_mag`/`vehicle_baro` from Task 1, `EK3_DELAY_MS` from Task 3.
- Produces: `struct ekf3_status_s` extended with `struct ekf_delay_s delay`, `uint32_t horizon_ms`, `uint32_t mag_in`, `uint32_t baro_in`.

- [ ] **Step 1: Extend `ekf3_status_s`**

In `apps/ekf3/ekf3.h`, add `#include "ekf_delay.h"` and extend the struct:

```c
struct ekf3_status_s
{
  struct ekf_core_s core;
  uint64_t first_output_us;
  uint64_t last_output_us;
  uint32_t publish_count;
  uint32_t publish_errors;
  uint32_t stale_count;
  uint32_t horizon_ms;        /* EK3_DELAY_MS as read at start */
  float    alt_noise;         /* EK3_ALT_M_NSE as read at start */
  float    alt_gate;          /* EK3_ALT_I_GATE as read at start */
  uint32_t mag_in;            /* vehicle_mag messages queued */
  uint32_t baro_in;           /* vehicle_baro messages queued */
  uint32_t imu_overflow;      /* mirrored from the ring */
  uint32_t mag_overflow;
  uint32_t baro_overflow;
  uint16_t output_replay;     /* samples in the last re-propagation */
  struct ekf_source_config_s sources;
  bool running;
};
```

`struct ekf_delay_s` is ~3 KB and must NOT go in here — `g_status` is copied under a mutex by `ekf3_status()` and lives alongside a 6144-byte task stack. Keep the ring as its own file-scope static in `ekf3.c` and mirror only the counters.

- [ ] **Step 2: Add the ring and the aiding subscriptions**

In `apps/ekf3/ekf3.c`, add includes for `ekf_delay.h`, `../param/param.h`, and after the existing statics:

```c
/* ~3 kB. A file-scope static, not a member of g_status and not on the stack:
 * ekf3 runs on 6144 bytes and g_status is copied wholesale under a mutex.
 */

static struct ekf_delay_s g_delay;
```

Add the constants:

```c
#define EKF3_BARO_MAX_AGE_US  500000ull
#define EKF3_MAG_MAX_AGE_US   500000ull
```

- [ ] **Step 3: Restructure the daemon loop**

Replace the body of `ekf3_daemon()` between the source-configuration block and the `out:` label. The new shape: subscribe to all three topics, poll all three, drain each into the ring/queues, then run the horizon loop, then publish.

```c
  subscriber = orb_subscribe(ORB_ID(vehicle_imu));
  mag_sub = orb_subscribe(ORB_ID(vehicle_mag));
  baro_sub = orb_subscribe(ORB_ID(vehicle_baro));
  publisher = estimator_state_advertise();

  if (subscriber < 0 || publisher < 0)
    {
      syslog(LOG_ERR,
             "[ekf3] vehicle_imu unavailable; start imu_delta first\n");
      goto out;
    }

  /* The aiding topics are optional. Their absence means no aiding, not a
   * failure to start: attitude from the IMU alone is still a useful output,
   * and it is what this estimator produced before there was any aiding.
   */

  if (mag_sub < 0 || baro_sub < 0)
    {
      syslog(LOG_WARNING,
             "[ekf3] %s%s%s unavailable; run `sensors aux start` for aiding\n",
             mag_sub < 0 ? "vehicle_mag" : "",
             mag_sub < 0 && baro_sub < 0 ? " and " : "",
             baro_sub < 0 ? "vehicle_baro" : "");
    }

  status.horizon_ms = (uint32_t)param_i32("EK3_DELAY_MS");
  status.alt_noise = param_f32("EK3_ALT_M_NSE");
  status.alt_gate = param_f32("EK3_ALT_I_GATE");
  ekf_delay_init(&g_delay, status.horizon_ms);

  fds[0].fd = subscriber;
  fds[0].events = POLLIN;
  nfds = 1;

  if (mag_sub >= 0)
    {
      fds[nfds].fd = mag_sub;
      fds[nfds].events = POLLIN;
      nfds++;
    }

  if (baro_sub >= 0)
    {
      fds[nfds].fd = baro_sub;
      fds[nfds].events = POLLIN;
      nfds++;
    }

  g_running = true;
  status.running = true;
  status_publish(&status);

  syslog(LOG_INFO,
         "[ekf3] 15-state, horizon %" PRIu32 " ms, sources %u\n",
         status.horizon_ms, status.sources.active_set + 1u);
```

Then the loop body, replacing the current drain-and-process block:

```c
  while (!g_should_stop)
    {
      int ready = poll(fds, nfds, 100);
      uint64_t now;
      uint64_t horizon;
      struct ekf_imu_sample_s sample;
      bool advanced = false;

      if (ready < 0 && errno != EINTR)
        {
          break;
        }

      drain_imu(subscriber, &status);
      drain_mag(mag_sub, &status);
      drain_baro(baro_sub, &status);

      now = now_us();
      horizon = ekf_delay_horizon_time(&g_delay, now);

      /* Advance the filter to the horizon, fusing each measurement at the
       * point on the trajectory where it was actually taken.
       */

      while (ekf_delay_next_imu(&g_delay, horizon, &sample))
        {
          if (ekf_core_process(&status.core, &sample) !=
              EKF_PROCESS_REJECTED)
            {
              advanced = true;
            }
        }

      if (advanced)
        {
          publish_output(publisher, &status, now);
        }

      status.imu_overflow = g_delay.imu_overflow_count;
      status.mag_overflow = g_delay.mag_overflow_count;
      status.baro_overflow = g_delay.baro_overflow_count;
      status_publish(&status);
    }
```

Declare `struct pollfd fds[3]; int nfds; int mag_sub = -1; int baro_sub = -1;` at the top of the function and unsubscribe both in the `out:` block alongside the existing cleanup.

- [ ] **Step 4: Write the three drain helpers and the publisher**

Add above `ekf3_daemon()`:

```c
static void drain_imu(int sub, FAR struct ekf3_status_s *status)
{
  int drained = 0;

  while (drained++ < EKF3_DRAIN_MAX)
    {
      struct vehicle_imu_s message;
      struct ekf_imu_sample_s sample;
      uint64_t now;

      if (orb_copy(ORB_ID(vehicle_imu), sub, &message) < 0)
        {
          return;
        }

      now = now_us();

      if (now > message.timestamp_sample &&
          now - message.timestamp_sample > EKF3_MAX_INPUT_AGE_US)
        {
          status->stale_count++;
          continue;
        }

      fill_core_sample(&message, &sample);
      ekf_delay_push_imu(&g_delay, &sample);
    }
}

static void drain_mag(int sub, FAR struct ekf3_status_s *status)
{
  int drained = 0;

  if (sub < 0)
    {
      return;
    }

  while (drained++ < EKF3_DRAIN_MAX)
    {
      struct vehicle_mag_s message;
      struct ekf_mag_sample_s sample;

      if (orb_copy(ORB_ID(vehicle_mag), sub, &message) < 0)
        {
          return;
        }

      memset(&sample, 0, sizeof(sample));
      sample.timestamp_sample = message.timestamp_sample;
      memcpy(sample.field, message.field, sizeof(sample.field));
      sample.calibrated = message.calibrated != 0;

      ekf_delay_push_mag(&g_delay, &sample);
      status->mag_in++;
    }
}

static void drain_baro(int sub, FAR struct ekf3_status_s *status)
{
  int drained = 0;

  if (sub < 0)
    {
      return;
    }

  while (drained++ < EKF3_DRAIN_MAX)
    {
      struct vehicle_baro_s message;
      struct ekf_baro_sample_s sample;

      if (orb_copy(ORB_ID(vehicle_baro), sub, &message) < 0)
        {
          return;
        }

      memset(&sample, 0, sizeof(sample));
      sample.timestamp_sample = message.timestamp_sample;
      sample.pressure = message.pressure;
      sample.temperature = message.temperature;

      ekf_delay_push_baro(&g_delay, &sample);
      status->baro_in++;
    }
}

static void publish_output(int publisher, FAR struct ekf3_status_s *status,
                           uint64_t now)
{
  FAR const struct ekf_imu_sample_s *replay[EKF_IMU_RING_SIZE];
  struct ekf_output_s output;
  struct estimator_state_s message;
  uint16_t count = ekf_delay_output_count(&g_delay);
  uint16_t i;

  for (i = 0; i < count; i++)
    {
      replay[i] = ekf_delay_output_at(&g_delay, i);
    }

  ekf_core_output_predict(&status->core, replay, count, &output);
  status->output_replay = output.samples_replayed;

  fill_output(&status->core, &output, now, &message);

  if (estimator_state_publish(publisher, &message) < 0)
    {
      status->publish_errors++;
    }

  if (status->publish_count == 0)
    {
      status->first_output_us = output.timestamp_sample;
    }

  status->publish_count++;
  status->last_output_us = output.timestamp_sample;
}
```

- [ ] **Step 5: Change `fill_output()` to take the predicted state**

Modify the existing `fill_output()` signature to
`static void fill_output(FAR const struct ekf_core_s *core, FAR const struct ekf_output_s *predicted, uint64_t publication_time, FAR struct estimator_state_s *output)`.

Take `quaternion`, `velocity`, `position` and `timestamp_sample` from `predicted`; take biases, variances, counts, `reset_counter` and `solution_status` from `core` as it does now. The covariance belongs to the filter state at the horizon, and there is no meaningful way to forward-propagate it here — that is the point of separating them.

- [ ] **Step 6: Build and run the full gate**

```bash
./tools/verify.sh
```

Expected: every test PASS except the known pre-existing `test-cpu-runtime` failure; firmware builds and `build/xxcar.px4` is newer than its sources.

- [ ] **Step 7: Commit**

```bash
git add apps/ekf3/ekf3.h apps/ekf3/ekf3.c
git commit -m "ekf3: run the filter at a horizon and publish a re-propagated state

IMU samples enter a ring; the filter advances only to now - EK3_DELAY_MS
so a measurement can be fused where it was actually taken. Publication
replays the unconsumed samples forward, so estimator_state stays a
current-time topic.

EK3_DELAY_MS defaults to 0, which drains the ring every tick and makes
this change inert - deliberately, so the timing rewrite can be confirmed
on hardware before any measurement starts correcting anything.

The ring is a file-scope static, not part of ekf3_status_s: it is ~3 kB,
the status struct is copied wholesale under a mutex, and the task runs on
a 6144-byte stack.

vehicle_mag and vehicle_baro are optional. Their absence means no aiding,
not a failure to start.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 8: Scalar measurement update

**Files:**
- Modify: `apps/ekf3/ekf_core.c`
- Modify: `tests/ekf_core_test.c`

**Interfaces:**
- Produces: `static int measurement_update_1d(FAR struct ekf_core_s *ekf, FAR const float h[EKF_STATE_DIM], float residual, float noise_variance, float gate_sigma, FAR float *nis)` — returns 1 accepted, 0 gated, −1 numerical failure, matching `measurement_update_3d()`.

- [ ] **Step 1: Add the host-test shims**

`tests/ekf_core_test.c` links against `ekf_core.c` as a separate translation unit (see `tools/test-ekf-core.sh`) rather than `#include`-ing it, so the static updates are not reachable from the test. Both need a shim.

At the end of `apps/ekf3/ekf_core.c`:

```c
#ifdef EKF_CORE_HOST_TEST

/* Test access to the static measurement updates. Compiled only for the host
 * test - the firmware never sees these. The alternative, making the updates
 * non-static, would widen the interface permanently for a test-only need.
 */

int ekf_core_test_update_1d(FAR struct ekf_core_s *ekf,
                            FAR const float h[EKF_STATE_DIM],
                            float residual, float noise_variance,
                            float gate_sigma, FAR float *nis)
{
  return measurement_update_1d(ekf, h, residual, noise_variance,
                               gate_sigma, nis);
}

int ekf_core_test_update_3d(FAR struct ekf_core_s *ekf,
                            FAR const float h[3][EKF_STATE_DIM],
                            FAR const float residual[3],
                            float noise_variance, FAR float *nis)
{
  return measurement_update_3d(ekf, h, residual, noise_variance, nis,
                               NULL, NULL);
}

#endif
```

And at the end of `apps/ekf3/ekf_core.h`, before the closing `#endif`:

```c
#ifdef EKF_CORE_HOST_TEST
int ekf_core_test_update_1d(FAR struct ekf_core_s *ekf,
                            FAR const float h[EKF_STATE_DIM],
                            float residual, float noise_variance,
                            float gate_sigma, FAR float *nis);
int ekf_core_test_update_3d(FAR struct ekf_core_s *ekf,
                            FAR const float h[3][EKF_STATE_DIM],
                            FAR const float residual[3],
                            float noise_variance, FAR float *nis);
#endif
```

- [ ] **Step 2: Write the failing test**

Add to `tests/ekf_core_test.c`. Reuse the file's existing `initialize_tilted(struct ekf_core_s *ekf, uint64_t *timestamp, float roll, float pitch)` helper — do not add another alignment routine.

```c
/* The scalar update must agree exactly with the 3-D update on a measurement
 * that only observes one state.
 *
 * Rows 1 and 2 of H are all zero, so PH' has zero columns there and the gain
 * in those directions is zero: the 3-D update reduces algebraically to the
 * scalar one on row 0. Two independently written Kalman updates that disagree
 * is the kind of bug that shows up as a filter that is subtly wrong for
 * months, so pin them against each other rather than trusting the derivation.
 */

static void test_update_1d_matches_3d(void)
{
  struct ekf_core_s scalar;
  struct ekf_core_s vector;
  uint64_t timestamp = 0;
  float h1[EKF_STATE_DIM];
  float h3[3][EKF_STATE_DIM];
  float residual3[3];
  float nis1 = 0.0f;
  float nis3 = 0.0f;
  int i;

  initialize_tilted(&scalar, &timestamp, 0.0f, 0.0f);
  vector = scalar;

  memset(h1, 0, sizeof(h1));
  h1[8] = 1.0f;                 /* observe position down only */

  memset(h3, 0, sizeof(h3));
  memset(residual3, 0, sizeof(residual3));
  h3[0][8] = 1.0f;
  residual3[0] = 1.5f;

  /* Gate generously on the scalar side: the 3-D update uses the fixed
   * EKF_MEASUREMENT_NIS_GATE, so the comparison must not be decided by a
   * difference in gating.
   */

  assert(ekf_core_test_update_1d(&scalar, h1, 1.5f, 4.0f, 1000.0f,
                                 &nis1) == 1);
  assert(ekf_core_test_update_3d(&vector, h3, residual3, 4.0f, &nis3) == 1);

  assert_near(nis1, nis3, 1.0e-4f);
  assert_near(scalar.position[2], vector.position[2], 1.0e-5f);
  assert_near(scalar.velocity[2], vector.velocity[2], 1.0e-5f);
  assert_near(scalar.accel_bias[2], vector.accel_bias[2], 1.0e-6f);

  for (i = 0; i < EKF_STATE_DIM; i++)
    {
      assert_near(scalar.covariance[EKF_P_INDEX(i, i)],
                  vector.covariance[EKF_P_INDEX(i, i)], 1.0e-5f);
    }
}

/* An innovation beyond the gate must leave the state completely untouched. A
 * partially applied correction would be worse than either accepting or
 * rejecting, because it is neither the old estimate nor the new one.
 */

static void test_update_1d_gate_rejects_cleanly(void)
{
  struct ekf_core_s before;
  struct ekf_core_s ekf;
  uint64_t timestamp = 0;
  float h[EKF_STATE_DIM];
  float nis = 0.0f;

  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f);
  before = ekf;

  memset(h, 0, sizeof(h));
  h[8] = 1.0f;

  /* 1000 m against a metre-scale variance is outside any sane gate. */

  assert(ekf_core_test_update_1d(&ekf, h, 1000.0f, 4.0f, 5.0f, &nis) == 0);
  assert(nis > 25.0f);
  assert(memcmp(&before, &ekf, sizeof(before)) == 0);
}

/* A correction must reduce the variance of the state it observed, and must
 * not drive it to zero or negative.
 */

static void test_update_1d_reduces_variance(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp = 0;
  float h[EKF_STATE_DIM];
  float before;
  float nis = 0.0f;

  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f);
  before = ekf.covariance[EKF_P_INDEX(8, 8)];

  memset(h, 0, sizeof(h));
  h[8] = 1.0f;

  assert(ekf_core_test_update_1d(&ekf, h, 0.5f, 4.0f, 1000.0f, &nis) == 1);
  assert(ekf.covariance[EKF_P_INDEX(8, 8)] < before);
  assert(ekf.covariance[EKF_P_INDEX(8, 8)] > 0.0f);
  assert_covariance_positive_definite(&ekf);
}
```

Add `test_update_1d_matches_3d();`, `test_update_1d_gate_rejects_cleanly();` and `test_update_1d_reduces_variance();` to `main()`.

- [ ] **Step 3: Run the test to verify it fails**

```bash
tools/test-ekf-core.sh
```

Expected: FAIL at compile — `implicit declaration of function 'measurement_update_1d'` in the shim, because the function does not exist yet.

- [ ] **Step 4: Implement `measurement_update_1d()`**

Add to `ekf_core.c` immediately after `measurement_update_3d()` and before the shims from Step 1:

```c
/* Scalar form of measurement_update_3d(). Same expanded Joseph covariance
 * form, same numerical guards, same post-update attitude covariance reset and
 * bias constraint - so the two cannot develop different ideas about what a
 * safe update is.
 *
 * Returns one for an accepted update, zero for a gated innovation, and minus
 * one for a numerical failure. On a gated innovation NOTHING is modified: a
 * partially applied correction would be worse than either outcome.
 */

static int measurement_update_1d(FAR struct ekf_core_s *ekf,
                                 FAR const float h[EKF_STATE_DIM],
                                 float residual, float noise_variance,
                                 float gate_sigma, FAR float *nis)
{
  float pht[EKF_STATE_DIM];
  float gain[EKF_STATE_DIM];
  float correction[EKF_STATE_DIM];
  float delta_quaternion[4];
  float next_quaternion[4];
  float innovation = noise_variance;
  float local_nis;
  int row;
  int column;
  int inner;

  if (!isfinite(residual) || !isfinite(noise_variance) ||
      noise_variance <= 0.0f)
    {
      return -1;
    }

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      pht[row] = 0.0f;

      for (inner = 0; inner < EKF_STATE_DIM; inner++)
        {
          pht[row] += ekf->covariance[EKF_P_INDEX(row, inner)] * h[inner];
        }
    }

  for (inner = 0; inner < EKF_STATE_DIM; inner++)
    {
      innovation += h[inner] * pht[inner];
    }

  if (!isfinite(innovation) || innovation <= 0.0f)
    {
      return -1;
    }

  local_nis = residual * residual / innovation;

  if (nis != NULL)
    {
      *nis = local_nis;
    }

  if (!isfinite(local_nis))
    {
      return -1;
    }

  if (local_nis > gate_sigma * gate_sigma)
    {
      return 0;
    }

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      gain[row] = pht[row] / innovation;
      correction[row] = gain[row] * residual;
    }

  /* P - K H P - P H' K' + K S K', evaluated on the upper triangle from the
   * unchanged P/PHt workspaces and mirrored exactly.
   */

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      for (column = row; column < EKF_STATE_DIM; column++)
        {
          float value = ekf->covariance[EKF_P_INDEX(row, column)] -
                        gain[row] * pht[column] -
                        pht[row] * gain[column] +
                        gain[row] * innovation * gain[column];

          if (!isfinite(value))
            {
              return -1;
            }

          ekf->covariance[EKF_P_INDEX(row, column)] = value;
          ekf->covariance[EKF_P_INDEX(column, row)] = value;
        }
    }

  rotation_vector_quaternion(correction, delta_quaternion);
  quaternion_multiply(ekf->quaternion, delta_quaternion, next_quaternion);
  memcpy(ekf->quaternion, next_quaternion, sizeof(ekf->quaternion));

  if (!quaternion_normalize(ekf->quaternion))
    {
      return -1;
    }

  for (row = 0; row < 3; row++)
    {
      ekf->velocity[row] += correction[3 + row];
      ekf->position[row] += correction[6 + row];
      ekf->gyro_bias[row] += correction[9 + row];
      ekf->accel_bias[row] += correction[12 + row];
    }

  if (!covariance_reset_attitude(ekf, correction))
    {
      return -1;
    }

  constrain_biases(ekf);

  for (row = 0; row < EKF_STATE_DIM; row++)
    {
      FAR float *diagonal = &ekf->covariance[EKF_P_INDEX(row, row)];

      if (!isfinite(*diagonal))
        {
          return -1;
        }

      if (*diagonal < EKF_MIN_VARIANCE)
        {
          *diagonal = EKF_MIN_VARIANCE;
        }
    }

  return 1;
}
```

- [ ] **Step 5: Run the test to verify it passes**

```bash
tools/test-ekf-core.sh
```

Expected: PASS, including the UBSan pass that `test-ekf-core.sh` runs second.

- [ ] **Step 6: Commit**

```bash
git add apps/ekf3/ekf_core.c apps/ekf3/ekf_core.h tests/ekf_core_test.c
git commit -m "ekf3: add the scalar measurement update

Same expanded Joseph form, numerical guards, attitude covariance reset
and bias constraint as the 3-D update, so the two cannot develop
different ideas about what a safe update is. A gated innovation modifies
nothing: a partially applied correction is worse than either accepting or
rejecting.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 9: Barometer height fusion

**Files:**
- Modify: `apps/ekf3/ekf_core.h`, `apps/ekf3/ekf_core.c`, `apps/ekf3/ekf3.c`
- Create: `tests/ekf_baro_test.c`, `tools/test-ekf-baro.sh`

**Interfaces:**
- Consumes: `measurement_update_1d()` (Task 8), `struct ekf_baro_sample_s` (Task 5), `EK3_ALT_M_NSE`/`EK3_ALT_I_GATE` (Task 3).
- Produces: `float ekf_baro_height(float pressure_hpa, float reference_hpa)`, `int ekf_core_fuse_baro(FAR struct ekf_core_s *ekf, float pressure_hpa, float noise, float gate_sigma)`, and `ekf_core_s` fields `baro_reference_hpa`, `baro_have_reference`, `baro_accept_count`, `baro_reject_count`, `baro_consecutive_rejects`, `last_baro_nis`, `last_baro_height`.

- [ ] **Step 1: Extend `ekf_core_s` and declare the interface**

In `apps/ekf3/ekf_core.h`, add to `struct ekf_core_s` before the counters:

```c
  float baro_reference_hpa;
  float last_baro_height;
  float last_baro_nis;
  bool  baro_have_reference;

  uint32_t baro_accept_count;
  uint32_t baro_reject_count;
  uint32_t baro_consecutive_rejects;
```

Add the constants and prototypes:

```c
/* Pressure sanity, staleness and the rejection run that drops validity. These
 * are fixed rather than parameters: they are not tuning decisions, they are
 * the boundary between a reading and a fault.
 */

#define EKF_BARO_PRESSURE_MIN      500.0f
#define EKF_BARO_PRESSURE_MAX     1200.0f
#define EKF_BARO_REJECT_RUN_MAX      20u

/* Height above the reference pressure, ISA. Positive is up. */

float ekf_baro_height(float pressure_hpa, float reference_hpa);

/* Fuse a height. Returns 1 accepted, 0 gated, -1 numerical failure, and -2
 * when there is no reference yet - in which case this sample BECOMES the
 * reference and the filter is left alone.
 */

int ekf_core_fuse_baro(FAR struct ekf_core_s *ekf, float pressure_hpa,
                       float noise, float gate_sigma);
```

- [ ] **Step 2: Write the failing test**

Create `tests/ekf_baro_test.c`:

```c
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ekf_core.h"

#define CLOSE(a, b, eps) (fabsf((a) - (b)) < (eps))

static struct ekf_core_s g_core;

static void align_core(void)
{
  struct ekf_imu_sample_s s;
  uint64_t t = 0;
  int i;

  ekf_core_init(&g_core);

  for (i = 0; i < 500; i++)
    {
      memset(&s, 0, sizeof(s));
      t += 2500;
      s.timestamp_sample = t;
      s.timestamp_first = t - 2500;
      s.delta_angle_dt = 0.0025f;
      s.delta_velocity_dt = 0.0025f;
      s.delta_velocity[2] = -9.80665f * 0.0025f;
      s.samples = 5;
      s.accel_calibrated = true;
      s.gyro_calibrated = true;
      ekf_core_process(&g_core, &s);
    }

  assert(g_core.initialized);
}

/* At the reference pressure the height is exactly zero. */

static void test_height_at_reference_is_zero(void)
{
  assert(CLOSE(ekf_baro_height(1013.25f, 1013.25f), 0.0f, 1.0e-4f));
  assert(CLOSE(ekf_baro_height(950.0f, 950.0f), 0.0f, 1.0e-4f));
}

/* Lower pressure means higher up. Near sea level the gradient is about
 * 8.4 m per hPa, so one hPa below the reference is roughly 8.4 m up.
 */

static void test_height_sign_and_scale(void)
{
  float h = ekf_baro_height(1012.25f, 1013.25f);

  assert(h > 0.0f);
  assert(CLOSE(h, 8.4f, 0.5f));
  assert(ekf_baro_height(1014.25f, 1013.25f) < 0.0f);
}

/* The first sample becomes the reference and does not correct the filter. */

static void test_first_sample_sets_reference(void)
{
  align_core();

  assert(!g_core.baro_have_reference);
  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);
  assert(g_core.baro_have_reference);
  assert(CLOSE(g_core.baro_reference_hpa, 1013.25f, 1.0e-4f));
  assert(g_core.baro_accept_count == 0);
  assert(CLOSE(g_core.position[2], 0.0f, 1.0e-3f));
}

/* NED position[2] is down-positive; barometric height is up-positive. A
 * measured rise must drive position[2] NEGATIVE. Getting this backwards is
 * the single most likely mistake here and it would look like a working
 * filter that flies into the ground.
 */

static void test_rise_drives_position_down_negative(void)
{
  align_core();

  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);

  /* About 8.4 m up. */

  assert(ekf_core_fuse_baro(&g_core, 1012.25f, 2.0f, 5.0f) == 1);
  assert(g_core.position[2] < 0.0f);
  assert(g_core.baro_accept_count == 1);
  assert(g_core.last_baro_height > 0.0f);
}

/* Pressure outside the physical range is refused before it reaches the
 * filter, and does not become a reference either.
 */

static void test_insane_pressure_refused(void)
{
  align_core();

  assert(ekf_core_fuse_baro(&g_core, 5.0f, 2.0f, 5.0f) == -1);
  assert(!g_core.baro_have_reference);
  assert(ekf_core_fuse_baro(&g_core, 5000.0f, 2.0f, 5.0f) == -1);
  assert(!g_core.baro_have_reference);
  assert(ekf_core_fuse_baro(&g_core, NAN, 2.0f, 5.0f) == -1);
  assert(!g_core.baro_have_reference);
}

/* A gated innovation increments the run and leaves the state alone. */

static void test_gate_rejects_and_counts(void)
{
  struct ekf_core_s before;

  align_core();
  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);

  /* Drive the variance down with agreeing measurements first, so a wild one
   * is genuinely outside the gate rather than merely surprising.
   */

  for (int i = 0; i < 20; i++)
    {
      ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f);
    }

  before = g_core;

  /* 100 hPa below the reference is roughly 900 m. */

  assert(ekf_core_fuse_baro(&g_core, 913.25f, 2.0f, 5.0f) == 0);
  assert(g_core.baro_reject_count == before.baro_reject_count + 1);
  assert(g_core.baro_consecutive_rejects ==
         before.baro_consecutive_rejects + 1);
  assert(CLOSE(g_core.position[2], before.position[2], 1.0e-6f));
}

/* An accepted update clears the rejection run. */

static void test_accept_clears_run(void)
{
  align_core();
  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);
  assert(ekf_core_fuse_baro(&g_core, 913.25f, 2.0f, 5.0f) == 0);
  assert(g_core.baro_consecutive_rejects == 1);
  assert(ekf_core_fuse_baro(&g_core, 1013.20f, 2.0f, 5.0f) == 1);
  assert(g_core.baro_consecutive_rejects == 0);
}

/* Height observations make vertical velocity observable through the
 * covariance cross-terms. This is why EK3_SRC1_VELZ=0 is still correct: the
 * barometer is not a velocity sensor, but it does constrain velocity.
 */

static void test_reduces_vertical_velocity_variance(void)
{
  float before;
  int i;

  align_core();
  assert(ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f) == -2);
  before = g_core.covariance[EKF_P_INDEX(5, 5)];

  for (i = 0; i < 50; i++)
    {
      ekf_core_fuse_baro(&g_core, 1013.25f, 2.0f, 5.0f);
    }

  assert(g_core.covariance[EKF_P_INDEX(5, 5)] < before);
}

int main(void)
{
  test_height_at_reference_is_zero();
  test_height_sign_and_scale();
  test_first_sample_sets_reference();
  test_rise_drives_position_down_negative();
  test_insane_pressure_refused();
  test_gate_rejects_and_counts();
  test_accept_clears_run();
  test_reduces_vertical_velocity_variance();

  puts("ekf_baro: height sign, reference capture and gating verified - OK");
  return 0;
}
```

- [ ] **Step 3: Write the test runner**

Create `tools/test-ekf-baro.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cc -std=c11 -D_POSIX_C_SOURCE=200809L -Wall -Wextra -Werror \
  -Wno-unused-parameter -Wno-missing-field-initializers \
  -DFAR= -DEKF_CORE_HOST_TEST \
  -I"$REPO/apps/ekf3" \
  "$REPO/tests/ekf_baro_test.c" "$REPO/apps/ekf3/ekf_core.c" \
  -lm -o "$OUT/test"
"$OUT/test"
```

Then `chmod +x tools/test-ekf-baro.sh`.

- [ ] **Step 4: Run the test to verify it fails**

```bash
tools/test-ekf-baro.sh
```

Expected: FAIL at link — `undefined reference to 'ekf_baro_height'`.

- [ ] **Step 5: Implement the height conversion and the fusion**

Add to `ekf_core.c`:

```c
/* ISA height above a reference pressure. Positive is UP.
 *
 * Relative to the reference captured at alignment rather than to a sea-level
 * constant, which is what makes this a height above the alignment point and
 * consistent with the filter's local-NED origin.
 */

float ekf_baro_height(float pressure_hpa, float reference_hpa)
{
  if (!isfinite(pressure_hpa) || !isfinite(reference_hpa) ||
      pressure_hpa <= 0.0f || reference_hpa <= 0.0f)
    {
      return 0.0f;
    }

  return 44330.77f * (1.0f - powf(pressure_hpa / reference_hpa,
                                  0.1902632f));
}

int ekf_core_fuse_baro(FAR struct ekf_core_s *ekf, float pressure_hpa,
                       float noise, float gate_sigma)
{
  float h[EKF_STATE_DIM];
  float height;
  float residual;
  int result;

  if (ekf == NULL || !ekf->initialized || !isfinite(pressure_hpa) ||
      pressure_hpa < EKF_BARO_PRESSURE_MIN ||
      pressure_hpa > EKF_BARO_PRESSURE_MAX ||
      !isfinite(noise) || noise <= 0.0f)
    {
      return -1;
    }

  /* The first good sample defines where zero is. Correcting against a
   * reference that does not exist yet would inject the whole altitude of the
   * site as an error.
   */

  if (!ekf->baro_have_reference)
    {
      ekf->baro_reference_hpa = pressure_hpa;
      ekf->baro_have_reference = true;
      ekf->last_baro_height = 0.0f;
      return -2;
    }

  height = ekf_baro_height(pressure_hpa, ekf->baro_reference_hpa);
  ekf->last_baro_height = height;

  /* NED down-positive against an up-positive measurement. */

  memset(h, 0, sizeof(h));
  h[8] = 1.0f;
  residual = -height - ekf->position[2];

  result = measurement_update_1d(ekf, h, residual, noise * noise,
                                 gate_sigma, &ekf->last_baro_nis);

  if (result == 1)
    {
      ekf->baro_accept_count++;
      ekf->baro_consecutive_rejects = 0;
    }
  else if (result == 0)
    {
      ekf->baro_reject_count++;
      ekf->baro_consecutive_rejects++;
    }

  return result;
}
```

Add `baro_reference_hpa`, `baro_have_reference` and the counters to the reset performed in `restart_alignment()` — the reference is tied to the alignment point, so a re-alignment must discard it:

```c
  ekf->baro_reference_hpa = 0.0f;
  ekf->baro_have_reference = false;
  ekf->baro_consecutive_rejects = 0;
  ekf->last_baro_height = 0.0f;
```

- [ ] **Step 6: Run the test to verify it passes**

```bash
tools/test-ekf-baro.sh && tools/test-ekf-core.sh && tools/test-ekf-output.sh
```

Expected: all PASS.

- [ ] **Step 7: Call it from the horizon loop**

In `apps/ekf3/ekf3.c`, inside the `while (ekf_delay_next_imu(...))` loop, after `ekf_core_process()` succeeds, fuse any measurements now due at this sample's time:

```c
          if (ekf_core_process(&status.core, &sample) !=
              EKF_PROCESS_REJECTED)
            {
              struct ekf_baro_sample_s baro;

              advanced = true;

              /* Source selection makes a measurement ELIGIBLE. Health gating
               * inside the fusion decides whether it is USED. A parameter
               * never makes a bad measurement good.
               */

              while (ekf_delay_next_baro(&g_delay, sample.timestamp_sample,
                                         EKF3_BARO_MAX_AGE_US, &baro))
                {
                  if (status.sources.set[status.sources.active_set]
                        .position_z == EKF_SOURCE_BARO_OR_COMPASS)
                    {
                      ekf_core_fuse_baro(&status.core, baro.pressure,
                                         status.alt_noise,
                                         status.alt_gate);
                    }
                }
            }
```

Read `EK3_ALT_M_NSE` and `EK3_ALT_I_GATE` once at daemon start into new `float alt_noise; float alt_gate;` members of `ekf3_status_s`, alongside `horizon_ms`.

Drain the mag queue the same way and discard the samples for now — Flash B fuses them. This keeps the queue from filling and the overflow counter from being misleading:

```c
              {
                struct ekf_mag_sample_s mag;

                while (ekf_delay_next_mag(&g_delay, sample.timestamp_sample,
                                          EKF3_MAG_MAX_AGE_US, &mag))
                  {
                    /* Flash B fuses this. Drained here so the queue does not
                     * fill and report a misleading overflow.
                     */
                  }
              }
```

- [ ] **Step 8: Build, verify, commit**

```bash
./tools/verify.sh
git add apps/ekf3/ekf_core.h apps/ekf3/ekf_core.c apps/ekf3/ekf3.c \
        apps/ekf3/ekf3.h tests/ekf_baro_test.c tools/test-ekf-baro.sh
git commit -m "ekf3: fuse barometric height into vertical position

The first good sample defines the reference; correcting against one that
does not exist yet would inject the site's whole altitude as an error. A
re-alignment discards it, because the reference is tied to the alignment
point.

NED position[2] is down-positive and barometric height is up-positive, so
the observation is -height. Getting that backwards would look like a
working filter that drives into the ground, which is why the sign has a
test of its own.

Height observations constrain vertical velocity through the covariance
cross-terms. That is correct, and is why EK3_SRC1_VELZ=0 remains right:
the barometer is not a velocity sensor.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 10: Granular output validity

**Files:**
- Modify: `apps/uorb_msgs/uorb_msgs.h`, `apps/ekf3/ekf_core.h`, `apps/ekf3/ekf_core.c`
- Modify: `tests/ekf_core_test.c`

**Interfaces:**
- Produces: `ESTIMATOR_*` bit definitions and the matching `EKF_SOLUTION_*`; `ekf_core_solution_status()` returning the granular set.

- [ ] **Step 1: Redefine the validity bits**

In `apps/uorb_msgs/uorb_msgs.h`, replace the four existing `ESTIMATOR_*` defines:

```c
/* Granular output validity. Bits 0 and 1 keep the values and meanings they
 * have always had.
 *
 * Bits 2 and 3 are REDEFINED: they were ESTIMATOR_VELOCITY_VALID and
 * ESTIMATOR_POSITION_VALID. That is a breaking change to this topic's
 * meaning, and it is safe only because velocity and position have been
 * advertised invalid since this estimator existed and no consumer reads them.
 */

#define ESTIMATOR_ATTITUDE_VALID  (1u << 0)  /* roll and pitch */
#define ESTIMATOR_YAW_RELATIVE    (1u << 1)  /* heading, arbitrary datum */
#define ESTIMATOR_YAW_ABSOLUTE    (1u << 2)  /* heading against north */
#define ESTIMATOR_VELOCITY_HORIZ  (1u << 3)
#define ESTIMATOR_VELOCITY_VERT   (1u << 4)
#define ESTIMATOR_POSITION_HORIZ  (1u << 5)
#define ESTIMATOR_POSITION_VERT   (1u << 6)
```

Mirror them in `apps/ekf3/ekf_core.h`, replacing the two existing `EKF_SOLUTION_*` defines:

```c
#define EKF_SOLUTION_ATTITUDE      (1u << 0)
#define EKF_SOLUTION_YAW_RELATIVE  (1u << 1)
#define EKF_SOLUTION_YAW_ABSOLUTE  (1u << 2)
#define EKF_SOLUTION_VELOCITY_HORIZ (1u << 3)
#define EKF_SOLUTION_VELOCITY_VERT  (1u << 4)
#define EKF_SOLUTION_POSITION_HORIZ (1u << 5)
#define EKF_SOLUTION_POSITION_VERT  (1u << 6)
```

- [ ] **Step 2: Write the failing test**

Add to `tests/ekf_core_test.c`:

```c
/* Vertical validity appears only once the barometer is actually correcting,
 * and disappears on a sustained rejection run. Horizontal validity never
 * appears: nothing in Flash A makes it observable, and claiming otherwise
 * would be worse than claiming nothing.
 */

static void test_solution_status_vertical(void)
{
  struct ekf_core_s ekf;
  uint64_t timestamp = 0;
  uint8_t status;
  int i;

  initialize_tilted(&ekf, &timestamp, 0.0f, 0.0f);

  status = ekf_core_solution_status(&ekf);
  assert((status & EKF_SOLUTION_ATTITUDE) != 0);
  assert((status & EKF_SOLUTION_YAW_RELATIVE) != 0);
  assert((status & EKF_SOLUTION_YAW_ABSOLUTE) == 0);
  assert((status & EKF_SOLUTION_POSITION_VERT) == 0);

  assert(ekf_core_fuse_baro(&ekf, 1013.25f, 2.0f, 5.0f) == -2);

  for (i = 0; i < 10; i++)
    {
      assert(ekf_core_fuse_baro(&ekf, 1013.25f, 2.0f, 5.0f) == 1);
    }

  status = ekf_core_solution_status(&ekf);
  assert((status & EKF_SOLUTION_POSITION_VERT) != 0);
  assert((status & EKF_SOLUTION_VELOCITY_VERT) != 0);
  assert((status & EKF_SOLUTION_POSITION_HORIZ) == 0);
  assert((status & EKF_SOLUTION_VELOCITY_HORIZ) == 0);

  /* A sustained rejection run withdraws the claim. */

  for (i = 0; i < EKF_BARO_REJECT_RUN_MAX + 1; i++)
    {
      ekf_core_fuse_baro(&ekf, 913.25f, 2.0f, 5.0f);
    }

  status = ekf_core_solution_status(&ekf);
  assert((status & EKF_SOLUTION_POSITION_VERT) == 0);
  assert((status & EKF_SOLUTION_ATTITUDE) != 0);   /* attitude survives */
}
```

Add `test_solution_status_vertical();` to `main()`.

- [ ] **Step 3: Run the test to verify it fails**

```bash
tools/test-ekf-core.sh
```

Expected: FAIL — `POSITION_VERT` is never set, because `ekf_core_solution_status()` still returns only the two attitude bits.

- [ ] **Step 4: Implement**

Replace `ekf_core_solution_status()` in `ekf_core.c`:

```c
uint8_t ekf_core_solution_status(FAR const struct ekf_core_s *ekf)
{
  uint8_t status;

  if (ekf == NULL || !ekf->initialized)
    {
      return 0;
    }

  /* Roll and pitch come from gravity, which is always available. Heading is
   * relative until a magnetometer makes it absolute - Flash B.
   */

  status = EKF_SOLUTION_ATTITUDE | EKF_SOLUTION_YAW_RELATIVE;

  /* Vertical validity is a claim about the barometer actually correcting,
   * not about it being selected. A sustained rejection run withdraws it
   * while leaving attitude alone.
   */

  if (ekf->baro_have_reference && ekf->baro_accept_count > 0 &&
      ekf->baro_consecutive_rejects < EKF_BARO_REJECT_RUN_MAX)
    {
      status |= EKF_SOLUTION_POSITION_VERT | EKF_SOLUTION_VELOCITY_VERT;
    }

  /* Nothing in this stage makes horizontal position or velocity observable.
   * Claiming them would be worse than claiming nothing.
   */

  return status;
}
```

- [ ] **Step 5: Run the test to verify it passes**

```bash
tools/test-ekf-core.sh && tools/test-ekf-baro.sh
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add apps/uorb_msgs/uorb_msgs.h apps/ekf3/ekf_core.h \
        apps/ekf3/ekf_core.c tests/ekf_core_test.c
git commit -m "uorb_msgs: report validity per state rather than per category

Bits 2 and 3 are redefined - they were VELOCITY_VALID and POSITION_VALID.
A breaking change to the topic's meaning, safe only because both have
been advertised invalid since this estimator existed and no consumer
reads them.

Vertical validity is a claim about the barometer actually correcting, not
about it being selected, so a sustained rejection run withdraws it while
attitude carries on.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 11: Status reporting and the flash gate

**Files:**
- Modify: `apps/ekf3/ekf3_main.c`

**Interfaces:**
- Consumes: everything above.
- Produces: no new API.

- [ ] **Step 1: Decode the solution flags**

In `apps/ekf3/ekf3_main.c`, add this helper above `print_status()`:

```c
/* The solution used to be one of two fixed strings. Now that validity is per
 * state, print what is actually valid: a barometer that is correcting has to
 * be distinguishable from one that is merely selected.
 */

static void print_solution(uint8_t status)
{
  if (status == 0)
    {
      printf("NONE");
      return;
    }

  printf("%s%s%s%s%s%s",
         (status & EKF_SOLUTION_ATTITUDE) ? "ATTITUDE " : "",
         (status & EKF_SOLUTION_YAW_ABSOLUTE) ? "YAW_ABS " :
           (status & EKF_SOLUTION_YAW_RELATIVE) ? "YAW_REL " : "",
         (status & EKF_SOLUTION_VELOCITY_HORIZ) ? "VELXY " : "",
         (status & EKF_SOLUTION_VELOCITY_VERT) ? "VELZ " : "",
         (status & EKF_SOLUTION_POSITION_HORIZ) ? "POSXY " : "",
         (status & EKF_SOLUTION_POSITION_VERT) ? "POSZ" : "");
}
```

Replace the existing three-argument header `printf` at `ekf3_main.c:69-72` with:

```c
  printf("ekf3: %s, %s, solution ",
         status.running ? "running" : "stopped",
         core->initialized ? "initialized" : "aligning");
  print_solution(ekf_core_solution_status(core));
  printf("\n");
```

- [ ] **Step 2: Report the horizon and the aiding sources**

Add after the existing `sources set` block:

```c
  printf("  horizon %" PRIu32 " ms  replay %u samples"
         "  ring overflow imu %" PRIu32 " mag %" PRIu32 " baro %" PRIu32
         "\n",
         status.horizon_ms, status.output_replay,
         status.imu_overflow, status.mag_overflow, status.baro_overflow);

  if (core->baro_have_reference)
    {
      printf("  baro ref %.2f hPa  height %+.3f m  accept %" PRIu32
             " reject %" PRIu32 " (run %" PRIu32 ") NIS %.3f\n",
             (double)core->baro_reference_hpa,
             (double)core->last_baro_height,
             core->baro_accept_count, core->baro_reject_count,
             core->baro_consecutive_rejects, (double)core->last_baro_nis);
    }
  else
    {
      printf("  baro no reference yet (queued %" PRIu32 ")\n",
             status.baro_in);
    }

  printf("  aiding in  mag %" PRIu32 "  baro %" PRIu32 "\n",
         status.mag_in, status.baro_in);
```

- [ ] **Step 3: Stop claiming everything is invalid**

At `ekf3_main.c:106-111` the velocity and position lines print `[INVALID]` unconditionally. Replace both:

```c
  printf("  velocity NED %+.4f %+.4f %+.4f m/s [%s]\n",
         (double)core->velocity[0], (double)core->velocity[1],
         (double)core->velocity[2],
         (ekf_core_solution_status(core) & EKF_SOLUTION_VELOCITY_VERT) ?
           "VERT ONLY" : "INVALID");
  printf("  position NED %+.4f %+.4f %+.4f m [%s]\n",
         (double)core->position[0], (double)core->position[1],
         (double)core->position[2],
         (ekf_core_solution_status(core) & EKF_SOLUTION_POSITION_VERT) ?
           "VERT ONLY" : "INVALID");
```

At `ekf3_main.c:99` the attitude line ends with a hardcoded `(yaw relative)`. Make it select, so Flash B does not have to touch this file:

```c
  printf("  attitude RPY %+.3f %+.3f %+.3f deg (yaw %s)\n",
         (double)(euler[0] * rad_to_deg),
         (double)(euler[1] * rad_to_deg),
         (double)(euler[2] * rad_to_deg),
         (ekf_core_solution_status(core) & EKF_SOLUTION_YAW_ABSOLUTE) ?
           "absolute" : "relative");
```

- [ ] **Step 4: Run the full gate**

```bash
./tools/verify.sh
```

Expected: every host test PASS except the known pre-existing `test-cpu-runtime`; firmware builds; `build/xxcar.px4` newer than sources.

- [ ] **Step 5: Confirm the new tests are actually in the gate**

```bash
tools/verify.sh 2>&1 | grep -E "mag-correct|ekf-delay|ekf-output|ekf-baro"
```

Expected: four lines, all `PASS`. `verify.sh` globs `tools/test-*.sh`, so a missing `chmod +x` or a misnamed file would silently drop a test from the gate rather than fail.

- [ ] **Step 6: Commit**

```bash
git add apps/ekf3/ekf3_main.c
git commit -m "ekf3: report the horizon, the aiding sources and real validity

The velocity and position lines said [INVALID] unconditionally. Now they
say what is actually true, so a barometer that is correcting is
distinguishable from one that is merely selected.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Hardware verification

Flash `build/xxcar.px4`, then run these in order. **Stop at the first step that does not match** — each one isolates a different part of the change, and continuing past a failure gives up that isolation.

**1. The horizon rewrite is inert.**

```
param show EK3_DELAY_MS      # expect 0
ekf3 start
ekf3 status
```

Attitude, predict/covariance rates, and every fault counter must match the pre-flash baseline. `horizon 0 ms  replay 0 samples`, and all three ring overflow counters at 0. If attitude differs here, the timing rewrite is wrong and nothing after this matters.

**2. The frontend publishes.**

```
sensors aux start
sensors aux status
uorb_listener vehicle_mag -n 5
uorb_listener vehicle_baro -n 5
```

Pressure 800–1100 hPa depending on altitude and weather. `|B|` near `CAL_MAG0_FIELD` if the magnetometer has been calibrated; `NOT CALIBRATED - raw passthrough` otherwise, which is expected and not a failure.

**3. The rate parameters are honoured.** These have been read by nothing until now, so this is a real check, not a formality.

```
param set SENS_BARO_RATE 25
sensors aux stop
sensors aux start
sensors aux status
```

The reported baro rate must be 25, and the observed publication rate must change with it. Restore to 10 afterwards.

**4. The horizon engages.**

```
param set EK3_DELAY_MS 30
ekf3 stop
ekf3 start
ekf3 status
top
```

Expect `horizon 30 ms  replay ~12 samples`. Attitude must still track normally. Record the ekf3 CPU figure from `top` — this is the number that decides whether the re-propagating predictor stays or gets replaced by an output observer later.

**5. The barometer corrects.**

```
param show EK3_SRC1_POSZ     # expect 1
ekf3 status
```

`baro ref` should show a plausible pressure, accept count climbing, reject count near zero, NIS small. Lift the board a known height — a metre of desk-to-floor is enough — and confirm `position NED` third component goes **negative** as it rises, and `[VERT ONLY]` appears on both lines.

**6. Rejection behaves.** Cover and uncover the barometer port, or breathe on it, to force a large innovation. The reject run must climb, `POSZ` must drop out of the solution flags, attitude must be unaffected, and it must recover once the disturbance stops.

If all six pass, save the parameters and this is the Flash A baseline:

```
param set EK3_DELAY_MS 30
param save
```

## Flash B

Not planned here. `EK3_MAG_DEC`, `EK3_YAW_M_NSE`, `EK3_YAW_I_GATE`, the tilt-compensated heading initialisation and the gated yaw update are specified in the design document and get their own plan once Flash A is verified on hardware — in particular once step 4 has produced a real CPU number and step 2 has shown whether the magnetometer calibration on this board is good enough to fuse.
