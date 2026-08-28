/****************************************************************************
 * apps/companion/companion.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The link to the companion computer.
 *
 * Owns one serial port - whichever SER_*_FUNC names SER_FUNC_COMPANION - and
 * routes framed packets to uORB topics by message id. It knows nothing about
 * navigation: adding a message is a row in the routing table plus a topic,
 * not a new code path, which is what the control trajectory will need.
 ****************************************************************************/

#ifndef __APPS_COMPANION_COMPANION_H
#define __APPS_COMPANION_COMPANION_H

#include <stdbool.h>
#include <stdint.h>

#include "comp_proto.h"

#ifndef FAR
#  define FAR
#endif

struct companion_status_s
{
  bool     running;
  char     port[16];          /* connector name, as silkscreened */
  uint32_t baud;
  uint32_t tx_rate_hz;

  uint64_t bytes_in;
  uint64_t bytes_out;
  uint32_t tx_frames;
  uint32_t tx_errors;

  uint32_t rx_pose;           /* EXTERNAL_POSE routed and published */
  uint32_t rx_publish_errors;

  /* DIRECT_CONTROL, counted by outcome rather than in one total.
   *
   * The three failures mean different things to whoever is holding the
   * transmitter: stale is the link or the clock, invalid is the companion
   * sending something the format cannot mean, and a rising accepted count
   * with a still vehicle is the control router declining to act - the wrong
   * source switch or an incomplete arm sequence.
   */

  uint32_t rx_direct;           /* accepted and published to control_cmd */
  uint32_t rx_direct_stale;     /* no usable UTC, or older than the budget */
  uint32_t rx_direct_invalid;   /* out of range, bad mode, or not a number */
  uint64_t last_direct_us;      /* board time of the last accepted command */
  uint32_t last_direct_age_us;  /* how old that one was on arrival */
  uint32_t auto_timeout_us;     /* AUTO_CMD_TO_MS, the age budget above */
  uint32_t timesync_replies;
  uint32_t timesync_expected;   /* the burst the companion announced */
  uint32_t timesync_samples;    /* what came back, per the companion */
  int64_t  timesync_offset_us;  /* what the companion settled on */
  uint32_t timesync_trip_us;
  bool     timesync_synced;
  bool     utc_from_rtc;        /* offset came from the RTC, not a sync */
  bool     wall_clock_set;      /* CLOCK_REALTIME set from companion UTC */
  uint32_t rx_unsynced_stamp;   /* UTC arrived before a sync could use it */
  uint32_t est_seen;          /* estimator states actually read */
  uint32_t tx_no_state;       /* nothing new to send */

  /* The downlink tick free-runs on TIM6 and is NOT locked to the estimator,
   * so the two slip against each other. tx_repeat counts ticks that found
   * the same solution as the previous one; gap_min/gap_max bracket how far
   * the sample time moved between the solutions actually sent. At 200 Hz
   * against a 400 Hz estimator the gap sits at one estimator period either
   * side of 5 ms, and those numbers are how much slip is visible.
   */

  uint32_t tick_ticks;        /* TIM6 ticks raised */
  uint32_t tick_missed;       /* the downlink did not consume one in time */
  uint32_t tx_repeat;         /* same solution sent twice */
  uint32_t tx_gap_min_us;
  uint32_t tx_gap_max_us;

  /* UTC discipline. The PPS comes FROM the companion, so its rising edge is
   * the companion's own second boundary - which makes the residual below a
   * direct measurement against the very clock the companion will compare
   * our timestamps to.
   */

  uint8_t  pps_state;           /* enum fmuv6c_pps_state_e */
  uint32_t pps_corrections;
  uint32_t pps_rejected;        /* residual too large to be a refinement */
  uint32_t pps_max_correction_us;

  /* The pulse's own phase offset from the UTC second, established once per
   * sync. A free-running 1 Hz PWM has an arbitrary one; it is not an error
   * to be corrected, it is the thing drift is measured against.
   */

  bool     pps_absolute_phase;  /* PPS_ABS_PHASE: the pulse marks the second */
  bool     pps_phase_valid;
  int32_t  pps_phase_ref_us;
  int32_t  pps_drift_us;        /* movement away from that phase */
  int32_t  pps_worst_us;        /* largest residual seen, applied or not */
  int32_t  pps_residual_us;     /* last measured error, + means we ran fast */
  uint64_t pps_edge_used;
  uint32_t tx_future_clamped;   /* a stamp that would have led the clock */

  /* TIM5 minus CLOCK_MONOTONIC, microseconds.
   *
   * Both count from boot, so this should sit near zero. It does not have to:
   * they are independent counters and fmuv6c_imu_time_now() only re-anchors
   * to the coarse clock every 71.6 minutes, so a rate difference accumulates
   * between wraps. Reported because mixing the two was the cause of a UTC
   * that led the host by tens of milliseconds, and a number that grows here
   * is that same divergence made visible.
   */

  int64_t  clock_skew_us;
  uint32_t connects;          /* hosts attached */
  uint32_t disconnects;
  bool     waiting_for_host;  /* removable port, nobody plugged in */
  uint64_t last_rx_us;        /* board time of the last accepted frame */

  struct comp_parser_s parser;  /* frames, crc_errors, unknown_id, ... */
};

int  companion_start(void);
int  companion_stop(void);
void companion_status(FAR struct companion_status_s *out);

#endif /* __APPS_COMPANION_COMPANION_H */
