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

int  vesc_start(void);
int  vesc_stop(void);
void vesc_status(FAR struct vesc_daemon_status_s *out);

#endif /* __APPS_VESC_VESC_H */
