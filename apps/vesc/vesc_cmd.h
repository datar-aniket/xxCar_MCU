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

struct vesc_limits_s
{
  float    cur_max;      /* A, magnitude ceiling in current mode */
  float    duty_max;     /* 0..1, magnitude ceiling in duty mode */
  uint16_t steer_min;    /* us at steering -1 */
  uint16_t steer_trim;   /* us at steering 0 */
  uint16_t steer_max;    /* us at steering +1 */
};

struct vesc_cmd_out_s
{
  uint8_t  packet_id;    /* VESC_PACKET_SET_DUTY_SERVO or _CURRENT_SERVO */
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

/* May the daemon be armed right now?
 *
 * Refuses exactly one thing: arming into a live non-zero motor demand. A
 * stale setpoint or none at all is permitted, because both already produce
 * neutral output - and refusing them would make the daemon impossible to arm
 * by hand, since `vesc set` publishes once and is stale 200 ms later.
 */

bool vesc_cmd_may_arm(bool have_setpoint, float motor,
                      uint64_t age_us, uint32_t timeout_ms);

FAR const char *vesc_cmd_reason_name(uint8_t reason);

#endif /* __APPS_VESC_VESC_CMD_H */
