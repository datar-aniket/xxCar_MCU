/****************************************************************************
 * apps/vesc/vesc_cmd.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <math.h>

#include "vesc_cmd.h"
#include "vesc_proto.h"

/* Every bound check in this file is written !(x >= lo && x <= hi) rather
 * than x < lo || x > hi. NaN compares false against everything, so the
 * second form passes it through untouched - and a NaN reaching the encoder
 * is an undefined cast on a motor command.
 */

static float cmd_clamp(float v, float lo, float hi, FAR bool *clamped)
{
  if (!(v >= lo && v <= hi))
    {
      *clamped = true;

      if (!isfinite(v))
        {
          /* Not finite: neither endpoint is the right answer, so fall to the
           * middle. For a motor limit that is zero; for steering it is the
           * fraction that maps to trim.
           */

          return 0.0f;
        }

      return v < lo ? lo : hi;
    }

  return v;
}

/* Each half of the travel is scaled against its own endpoint. A linkage
 * rarely gives equal microseconds either side of straight, and forcing
 * symmetry would mean giving up travel on one side to trim the other.
 *
 * steer_min ABOVE steer_max is legal and means a reversed linkage. The
 * arithmetic handles it without a special case, which keeps the reversal out
 * of every controller that publishes a command.
 */

static uint16_t cmd_steer_us(float steering,
                             FAR const struct vesc_limits_s *lim)
{
  float span;
  float us;

  if (steering >= 0.0f)
    {
      span = (float)lim->steer_max - (float)lim->steer_trim;
    }
  else
    {
      span = (float)lim->steer_trim - (float)lim->steer_min;
    }

  us = (float)lim->steer_trim + steering * span;

  /* Round rather than truncate: 1699.9997 is 1700 microseconds, not 1699. */

  return (uint16_t)lroundf(us);
}

static void cmd_neutral(uint8_t packet_id, uint8_t reason,
                        FAR const struct vesc_limits_s *lim,
                        FAR struct vesc_cmd_out_s *out)
{
  out->packet_id = packet_id;
  out->motor = 0.0f;
  out->servo_us = lim->steer_trim;
  out->reason = reason;
  out->clamped = false;
}

void vesc_cmd_resolve(bool armed, bool have_setpoint,
                      uint8_t mode, float motor, float steering,
                      uint64_t age_us, uint32_t timeout_ms,
                      FAR const struct vesc_limits_s *lim,
                      FAR struct vesc_cmd_out_s *out)
{
  uint8_t packet_id;
  float limit;

  if (lim == NULL || out == NULL)
    {
      return;
    }

  /* Pick the frame first, so that a failsafe keeps sending the same packet
   * id the VESC was already seeing - carrying zeros. Switching frame ids at
   * the moment of a failsafe would look, from the far end, like a different
   * controller taking over.
   */

  if (have_setpoint && mode == VESC_MODE_CURRENT)
    {
      packet_id = VESC_PACKET_SET_CURRENT_SERVO;
      limit = lim->cur_max;
    }
  else
    {
      packet_id = VESC_PACKET_SET_DUTY_SERVO;
      limit = lim->duty_max;
    }

  if (!armed)
    {
      cmd_neutral(packet_id, VESC_CMD_DISARMED, lim, out);
      return;
    }

  if (!have_setpoint)
    {
      cmd_neutral(packet_id, VESC_CMD_NO_SETPOINT, lim, out);
      return;
    }

  if (mode != VESC_MODE_DUTY && mode != VESC_MODE_CURRENT)
    {
      cmd_neutral(packet_id, VESC_CMD_BAD_MODE, lim, out);
      return;
    }

  if (age_us > (uint64_t)timeout_ms * 1000ull)
    {
      cmd_neutral(packet_id, VESC_CMD_STALE, lim, out);
      return;
    }

  out->packet_id = packet_id;
  out->reason = VESC_CMD_ARMED;
  out->clamped = false;
  out->motor = cmd_clamp(motor, -limit, limit, &out->clamped);
  out->servo_us =
    cmd_steer_us(cmd_clamp(steering, -1.0f, 1.0f, &out->clamped), lim);
}

bool vesc_cmd_may_arm(bool have_setpoint, float motor,
                      uint64_t age_us, uint32_t timeout_ms)
{
  if (!have_setpoint)
    {
      return true;
    }

  if (age_us > (uint64_t)timeout_ms * 1000ull)
    {
      return true;
    }

  /* Fresh. Refuse only if it is actually commanding the motor. A non-finite
   * value counts as commanding: it is not zero, and it is not trustworthy.
   */

  if (!isfinite(motor))
    {
      return false;
    }

  return fabsf(motor) <= VESC_CMD_ZERO_EPS;
}

FAR const char *vesc_cmd_reason_name(uint8_t reason)
{
  switch (reason)
    {
      case VESC_CMD_ARMED:       return "armed";
      case VESC_CMD_DISARMED:    return "disarmed";
      case VESC_CMD_NO_SETPOINT: return "no-setpoint";
      case VESC_CMD_STALE:       return "stale";
      case VESC_CMD_BAD_MODE:    return "bad-mode";
      default:                   return "unknown";
    }
}
