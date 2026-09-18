/****************************************************************************
 * apps/vesc/vesc_cmd.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The command policy: arm gate, failsafe, limits, steering map.
 *
 * Separate from vesc_proto.c because the two fail differently. vesc_proto
 * gets byte order and scaling wrong; this file gets SAFETY wrong. Keeping it
 * free of uORB and hardware is what lets a host test drive every state,
 * including the ones a bench cannot produce on demand.
 *
 * It takes plain scalars rather than a struct actuator_command_s because
 * that struct lives behind uORB headers that will not compile on a host. The
 * daemon unpacks the topic.
 ****************************************************************************/

#ifndef __APPS_VESC_VESC_CMD_H
#define __APPS_VESC_VESC_CMD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

/* Mirrors ACTUATOR_MODE_* in uorb_msgs.h. Duplicated rather than included
 * because that header needs uORB. vesc.c carries a static_assert that the
 * two agree, so drift is a build failure rather than a silent unit change.
 */

#define VESC_MODE_DUTY        0
#define VESC_MODE_CURRENT     1

/* Why the output is what it is. Every one of these except ARMED means the
 * wire is carrying neutral.
 */

#define VESC_CMD_ARMED        0
#define VESC_CMD_DISARMED     1
#define VESC_CMD_NO_SETPOINT  2
#define VESC_CMD_STALE        3
#define VESC_CMD_BAD_MODE     4
#define VESC_CMD_NREASON      5

/* Below this the motor command counts as zero for the arm gate. Floats
 * arriving from a controller are rarely exactly 0.0f.
 */

#define VESC_CMD_ZERO_EPS     0.001f

/* Live steering trim. The radio holds no trim state: a switch position is a
 * nudge request, not a value, and the accumulated offset lives here. That is
 * what stops a knob and a saved parameter from double-applying after a
 * reboot, which is the failure the absolute-channel trim it replaces had.
 *
 * Auto-repeat so one held switch can walk the offset across its range without
 * a hundred flicks, delayed so a deliberate single step is still possible.
 */

#define VESC_TRIM_LIMIT_US        300   /* matches VESC_STEER_OFS's range */
#define VESC_TRIM_REPEAT_DELAY_US 500000ull
#define VESC_TRIM_REPEAT_US       100000ull

struct vesc_trim_cfg_s
{
  uint16_t sw_low;       /* at or below: nudge negative */
  uint16_t sw_high;      /* at or above: nudge positive */
  int16_t  step_us;      /* per step */
  int16_t  limit_us;     /* magnitude ceiling on the accumulated offset */
};

struct vesc_trim_state_s
{
  int32_t  offset_us;    /* accumulated, not yet folded into a parameter */
  int8_t   dir;          /* -1/0/+1: which way the switch is held now */
  uint64_t next_step_us; /* when auto-repeat may fire again */
};

struct vesc_limits_s
{
  float    cur_max;      /* A, magnitude ceiling in current mode */
  float    duty_max;     /* 0..1, magnitude ceiling in duty mode */
  uint16_t steer_min;    /* us at steering -1 */
  uint16_t steer_trim;   /* us at steering 0 */
  uint16_t steer_max;    /* us at steering +1 */
  int16_t  steer_offset; /* us added after mapping, before final clamp */
};

struct vesc_cmd_out_s
{
  uint8_t  packet_id;    /* internal duty/current choice; transport selects
                         * combined or motor-only CAN packet */
  float    motor;        /* clamped, in the units the packet id implies */
  uint16_t servo_us;
  uint8_t  reason;       /* VESC_CMD_* */
  bool     clamped;      /* input was out of range or not finite */
};

/* Resolve one transmit period's output.
 *
 * `age_us` is how old the setpoint is; the caller computes it and must guard
 * against a timestamp in the future, since this takes an unsigned value.
 *
 * Always writes `out`. There is no failure return: every input produces
 * something safe to put on the wire, which is the entire point.
 */

void vesc_cmd_resolve(bool armed, bool have_setpoint,
                      uint8_t mode, float motor, float steering,
                      uint64_t age_us, uint32_t timeout_ms,
                      FAR const struct vesc_limits_s *lim,
                      FAR struct vesc_cmd_out_s *out);

/* Map one normalized steering command through an independently calibrated
 * servo. Safe for front or rear steering; non-finite input maps to trim.
 */

uint16_t vesc_cmd_steering_us(float steering,
                              FAR const struct vesc_limits_s *lim,
                              FAR bool *clamped);

/* Integrate one trim switch sample and return the accumulated offset.
 *
 * A step is applied on the edge that engages the switch, then repeated while
 * it is held. Returning to centre rearms the edge, so releasing and pressing
 * again always yields exactly one step.
 */

int16_t vesc_cmd_trim_nudge(FAR struct vesc_trim_state_s *state,
                            uint16_t pwm, uint64_t now_us,
                            FAR const struct vesc_trim_cfg_s *cfg);

/* Treat the switch as centred without a sample, keeping the accumulated
 * offset. Used when RC goes stale: a lost link must not leave a trim
 * creeping, and it must not silently throw away what the operator dialled in
 * either.
 */

void vesc_cmd_trim_idle(FAR struct vesc_trim_state_s *state);

/* The saved offset that results from folding a live nudge into it, bounded
 * to +/-limit_us.
 *
 * The caller subtracts (result - saved_us) from the live nudge rather than
 * zeroing it. That keeps saved + live - the offset the servo actually sees -
 * identical across a save, both when the sum overruns the limit (the excess
 * simply stays live) and when the operator nudges again while the save is
 * still writing to the card.
 */

int32_t vesc_cmd_trim_fold(int32_t saved_us, int32_t live_us,
                           int32_t limit_us);

/* Should this loop pass queue an automatic trim save?
 *
 * Only on the edge from armed to disarmed. The router asserts disarm on every
 * cycle while the arm switch is low, so a test of the level would ask for a
 * flash write fifty times a second. A zero live trim is skipped here as a
 * cheap filter; whether the saved values would actually change is decided
 * later, by the save itself.
 */

bool vesc_cmd_trim_save_due(bool was_armed, bool armed,
                            int32_t front_live_us, int32_t rear_live_us);

/* May the daemon be armed right now?
 *
 * Refuses exactly one thing: arming into a live non-zero motor demand. A
 * stale setpoint or none at all is permitted, because both already produce
 * neutral output - and refusing them would make the daemon impossible to arm
 * by hand, since `vesc set` publishes once and is stale 200 ms later.
 */

bool vesc_cmd_may_arm(bool have_setpoint, float motor,
                      uint64_t age_us, uint32_t timeout_ms);

/* Has telemetry stopped?
 *
 * A timeout of zero disables the check. A last_tlm_us of zero means nothing
 * has EVER arrived, which is start-up rather than a drop-out - the daemon
 * has no business disarming something it never saw working.
 */

bool vesc_cmd_telemetry_lost(uint64_t last_tlm_us, uint64_t now_us,
                             uint32_t timeout_ms);

FAR const char *vesc_cmd_reason_name(uint8_t reason);

#endif /* __APPS_VESC_VESC_CMD_H */
