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

/* Apply the mechanical centre correction at the final pulse stage. Doing it
 * here makes the correction identical for live control, disarmed trim, and
 * every failsafe-neutral path. The encoder repeats this 900..2100 bound as a
 * last-resort wire-safety check.
 */

static uint16_t cmd_steer_offset(uint16_t servo_us,
                                 FAR const struct vesc_limits_s *lim,
                                 FAR bool *clamped)
{
  int32_t corrected = (int32_t)servo_us + (int32_t)lim->steer_offset;

  if (corrected < (int32_t)VESC_SERVO_US_MIN)
    {
      *clamped = true;
      corrected = VESC_SERVO_US_MIN;
    }
  else if (corrected > (int32_t)VESC_SERVO_US_MAX)
    {
      *clamped = true;
      corrected = VESC_SERVO_US_MAX;
    }

  return (uint16_t)corrected;
}

uint16_t vesc_cmd_steering_us(float steering,
                              FAR const struct vesc_limits_s *lim,
                              FAR bool *clamped)
{
  bool local_clamped = false;

  if (clamped == NULL)
    {
      clamped = &local_clamped;
    }

  if (lim == NULL)
    {
      *clamped = true;
      return 1500u;
    }

  return cmd_steer_offset(
    cmd_steer_us(cmd_clamp(steering, -1.0f, 1.0f, clamped), lim),
    lim, clamped);
}

void vesc_cmd_trim_idle(FAR struct vesc_trim_state_s *state)
{
  if (state != NULL)
    {
      state->dir = 0;
      state->next_step_us = 0;
    }
}

int32_t vesc_cmd_trim_fold(int32_t saved_us, int32_t live_us,
                           int32_t limit_us)
{
  int32_t sum = saved_us + live_us;

  if (limit_us < 0)
    {
      limit_us = 0;
    }

  if (sum > limit_us)
    {
      return limit_us;
    }

  return sum < -limit_us ? -limit_us : sum;
}

bool vesc_cmd_trim_save_due(bool was_armed, bool armed,
                            int32_t front_live_us, int32_t rear_live_us)
{
  return was_armed && !armed && (front_live_us != 0 || rear_live_us != 0);
}

int16_t vesc_cmd_trim_nudge(FAR struct vesc_trim_state_s *state,
                            uint16_t pwm, uint64_t now_us,
                            FAR const struct vesc_trim_cfg_s *cfg)
{
  int32_t limit;
  int8_t dir;

  if (state == NULL)
    {
      return 0;
    }

  if (cfg == NULL || cfg->step_us <= 0 || cfg->limit_us <= 0 ||
      cfg->sw_low >= cfg->sw_high)
    {
      vesc_cmd_trim_idle(state);
      return (int16_t)state->offset_us;
    }

  limit = cfg->limit_us;
  dir = pwm >= cfg->sw_high ? 1 : pwm <= cfg->sw_low ? -1 : 0;

  if (dir == 0)
    {
      vesc_cmd_trim_idle(state);
      return (int16_t)state->offset_us;
    }

  if (dir != state->dir)
    {
      /* The engaging edge, including a throw straight from one side to the
       * other. One step now, and the repeat clock starts from here so a
       * single deliberate flick cannot become two.
       */

      state->dir = dir;
      state->next_step_us = now_us + VESC_TRIM_REPEAT_DELAY_US;
    }
  else if (now_us >= state->next_step_us)
    {
      /* Rearmed from now rather than advanced by one period: after a
       * scheduling gap the offset should resume stepping, not catch up in a
       * burst the operator never asked for.
       */

      state->next_step_us = now_us + VESC_TRIM_REPEAT_US;
    }
  else
    {
      return (int16_t)state->offset_us;
    }

  state->offset_us += (int32_t)dir * (int32_t)cfg->step_us;

  if (state->offset_us > limit)
    {
      state->offset_us = limit;
    }
  else if (state->offset_us < -limit)
    {
      state->offset_us = -limit;
    }

  return (int16_t)state->offset_us;
}

static void cmd_neutral(uint8_t packet_id, uint8_t reason,
                        FAR const struct vesc_limits_s *lim,
                        FAR struct vesc_cmd_out_s *out)
{
  out->packet_id = packet_id;
  out->motor = 0.0f;
  out->reason = reason;
  out->clamped = false;
  out->servo_us = cmd_steer_offset(lim->steer_trim, lim, &out->clamped);
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
  out->servo_us = vesc_cmd_steering_us(steering, lim, &out->clamped);
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

bool vesc_cmd_telemetry_lost(uint64_t last_tlm_us, uint64_t now_us,
                             uint32_t timeout_ms)
{
  if (timeout_ms == 0 || last_tlm_us == 0)
    {
      return false;
    }

  /* A timestamp in the future is a clock problem, not a drop-out. Reporting
   * it as lost would disarm the vehicle for something that is not a comms
   * failure at all.
   */

  if (now_us <= last_tlm_us)
    {
      return false;
    }

  return now_us - last_tlm_us > (uint64_t)timeout_ms * 1000ull;
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
