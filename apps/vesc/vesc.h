/****************************************************************************
 * apps/vesc/vesc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Drains FDCAN1, decodes VESC telemetry, publishes vesc_status.
 *
 * Also the discovery tool: it counts every (packet id, controller id) pair
 * it sees, decoded or not. A VESC emitting something docs/can_packet.md does
 * not describe is a fact worth seeing rather than an error worth hiding.
 ****************************************************************************/

#ifndef __APPS_VESC_VESC_H
#define __APPS_VESC_VESC_H

#include <stdbool.h>
#include <stdint.h>

#include <arch/board/fdcan.h>

#include "vesc_cmd.h"
#include "vesc_proto.h"

#ifndef FAR
#  define FAR
#endif

/* Enough for every packet id in can_packet.md plus room for whatever else
 * turns up, which is the point of discovery.
 */

#define VESC_SEEN_MAX 12

struct vesc_seen_s
{
  uint8_t  packet_id;
  uint8_t  controller_id;
  uint32_t count;
  uint64_t first_us;
  uint64_t last_us;
};

struct vesc_daemon_status_s
{
  bool     running;
  uint32_t bitrate;
  uint8_t  filter_id;         /* 0 = accept any */

  uint32_t decoded;           /* STATUS_5 frames decoded */
  uint32_t bad_dlc;           /* known packet id, wrong length */
  uint32_t publish_errors;

  /* Transmit */

  bool     armed;
  uint32_t tx_rate;                     /* Hz, as read at start */
  uint32_t cmd_timeout_ms;
  struct vesc_limits_s limits;
  struct vesc_limits_s rear_limits;

  /* Live steering trim, added after control routing so it applies to both RC
   * and automatic steering. The switches request nudges; the accumulated
   * offsets live in the daemon until `vesc trim save` folds them into
   * VESC_STEER_OFS / REAR_ST_OFS. An unhealthy or stale RC sample freezes
   * them where they are rather than stepping or discarding them.
   */

  uint32_t rc_timeout_ms;
  uint64_t rc_trim_stamp_us;
  uint8_t  trim_front_channel;  /* one-based, 0 = unmapped */
  uint8_t  trim_rear_channel;
  uint16_t trim_front_pwm;
  uint16_t trim_rear_pwm;
  int16_t  trim_front_us;       /* live, not yet persisted */
  int16_t  trim_rear_us;
  int16_t  trim_step_us;
  uint16_t trim_sw_low;         /* RC_SW_LOW / RC_SW_HIGH, shared with the */
  uint16_t trim_sw_high;        /* router rather than duplicated as params */
  bool     rc_trim_input_valid;
  bool     rc_trim_active;

  /* Trim saves, manual and on disarm. Filled in by vesc_status() from the
   * save task's own counters. A skipped save (nothing changed) is neither.
   */

  uint32_t trim_saves;
  uint32_t trim_save_errors;
  int      trim_save_last;      /* 0, -EALREADY, or the failure */
  bool     trim_saving;

  uint32_t setpoints;                   /* actuator_command messages taken */
  uint32_t tx_sent;                     /* frames handed to the driver */
  uint32_t tx_errors;                   /* driver refused the frame */
  uint32_t tx_clamped;                  /* setpoint was out of range */
  uint32_t reason_count[VESC_CMD_NREASON];

  /* Telemetry watchdog. A VESC that has stopped reporting is one whose state
   * we cannot see, and commanding a motor blind is what this prevents.
   */

  uint32_t tlm_timeout_ms;
  uint32_t tlm_disarms;         /* times the watchdog disarmed the vehicle */
  bool     tlm_lost;            /* telemetry currently absent */
  uint8_t  last_reason;
  float    last_motor;                  /* what actually went out */
  uint16_t last_servo_us;
  uint16_t last_rear_servo_us;
  uint8_t  steer_output_source; /* 0 VESC, 1 PX4IO */
  uint8_t  steer_io_channel;    /* one-based PX4IO channel */
  uint8_t  rear_steer_io_channel;
  bool     steer_io_healthy;
  uint32_t steer_io_errors;

  struct vesc_status5_s last;
  uint64_t last_us;

  uint8_t  nseen;
  struct vesc_seen_s seen[VESC_SEEN_MAX];

  struct fdcan_stats_s bus;
};

/* Arm or disarm. Returns 0, -ESRCH when not running, or -EPERM when a live
 * non-zero motor command is already being published - arming into a throttle
 * demand somebody else set is the accident this refuses.
 */

int  vesc_arm(bool armed);

/* Fold the live trim nudges into VESC_STEER_OFS / REAR_ST_OFS and persist
 * them, or discard them.
 *
 * The daemon also does this on its own on every disarm. Either way the write
 * happens in a separate task, so the CAN transmit loop never blocks on the
 * card. Commit waits for that task and returns its result: -EPERM while
 * armed, -EALREADY when the saved values would not change (nothing is
 * written), -EBUSY while another save is running. Reset returns -EBUSY
 * during a save.
 */

int  vesc_trim_commit(void);
int  vesc_trim_reset(void);

int  vesc_start(void);
int  vesc_stop(void);
void vesc_status(FAR struct vesc_daemon_status_s *out);

#endif /* __APPS_VESC_VESC_H */
