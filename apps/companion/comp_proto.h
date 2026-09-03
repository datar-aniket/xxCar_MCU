/****************************************************************************
 * apps/companion/comp_proto.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wire format for the companion computer link.
 *
 *   0xFE | id | len | payload[len] | crc16
 *
 * The CRC covers id, len and payload. Anything that is not the sync byte is
 * discarded, so a lost byte costs one message rather than wedging the
 * stream.
 *
 * Inbound ids are low and outbound ids are high. A message sent in the wrong
 * direction then fails to route at all, instead of half-working.
 *
 * No I/O and no uORB in this file. That is what lets the format be tested on
 * the host while it is still being iterated on.
 ****************************************************************************/

#ifndef __APPS_COMPANION_COMP_PROTO_H
#define __APPS_COMPANION_COMP_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

#define COMP_SYNC              0xfe
/* Sized for CONTROL_TRAJ at its 14-step maximum. The `len` field is a uint8,
 * but 244 is the largest defined payload; this buffer sits inside
 * comp_parser_s, which is copied under a mutex by companion_status().
 */

#define COMP_MAX_PAYLOAD       244
#define COMP_FRAME_OVERHEAD    5      /* sync + id + len + crc16 */

/* Inbound: companion -> board. */

#define COMP_MSG_EXTERNAL_POSE   1
#define COMP_MSG_CONTROL_TRAJ    2
#define COMP_MSG_TIMESYNC_REQ    3
#define COMP_MSG_TIMESYNC_START  5
#define COMP_MSG_TIMESYNC_END    6
#define COMP_MSG_DIRECT_CONTROL  7

/* Outbound: board -> companion. */

#define COMP_MSG_TIMESYNC_REP    4
#define COMP_MSG_VEHICLE_STATE  16

/* Absolute pose in the companion's map frame. Only x, y and yaw are fused;
 * height stays with the barometer.
 *
 * cov is the UPPER TRIANGLE of the 3x3 for (x, y, yaw):
 *   [0] xx  [1] xy  [2] x-yaw  [3] yy  [4] y-yaw  [5] yaw-yaw
 *
 * A zero variance means "no estimate supplied, use the parameter", so the
 * companion side can start by sending zeros and tighten later without a
 * format change.
 */

struct comp_external_pose_s
{
  uint64_t timestamp_us;    /*  0: board timebase, per the operator's sync.
                             *     ZERO means "not timestamped" and the board
                             *     stamps it on arrival - which costs the
                             *     whole link latency as position error, and
                             *     is counted separately so a working
                             *     timesync is never silently replaced. */
  float    x;               /*  8: m, map frame */
  float    y;               /* 12 */
  float    yaw;             /* 16: rad, map frame */
  float    cov[6];          /* 20: see above */
  uint8_t  flags;           /* 44: bit 0 = pose valid */
  uint8_t  reset_counter;   /* 45: the SOURCE's frame-reset generation */
  uint8_t  pad[2];          /* 46 */
};

#define COMP_POSE_FLAG_VALID (1u << 0)

/* Mirrors ACTUATOR_MODE_* in uorb_msgs.h. Duplicated rather than included
 * because that header needs uORB. companion.c carries a static_assert that
 * the two agree, so drift is a build failure rather than a silent unit
 * change - and the unit change this prevents is "20 amps" arriving as
 * "duty 20", which clamps to full throttle.
 */

#define COMP_THROTTLE_DUTY     0
#define COMP_THROTTLE_CURRENT  1

/* Protocol ceilings, not vehicle limits.
 *
 * The operator's ceilings are VESC_DUTY_MAX and VESC_CUR_MAX, applied by the
 * control router afterwards. These bound only what the FORMAT can mean, and a
 * value outside them is rejected rather than clamped: it says the sender is
 * wrong about the units or the mode, and clamping would turn that into a
 * command that looks deliberate.
 */

#define COMP_DIRECT_STEER_MAX    1.0f
#define COMP_DIRECT_DUTY_MAX     1.0f
#define COMP_DIRECT_CURRENT_MAX  50.0f

/* CONTROL_TRAJ has a variable payload. Its 20-byte header is followed by
 * `horizon` pose pairs and then `horizon` control pairs. Each pair is two
 * float32 values, so 14 steps exactly fit the protocol's 244-byte ceiling.
 */

#define COMP_TRAJ_HEADER_SIZE     20u
#define COMP_TRAJ_STEP_SIZE       16u
#define COMP_TRAJ_MAX_HORIZON     14u

#define COMP_TRAJ_TIMESTAMP_OFS    0u
#define COMP_TRAJ_SOLUTION_OFS     8u
#define COMP_TRAJ_HORIZON_OFS     16u
#define COMP_TRAJ_DT_OFS          17u
#define COMP_TRAJ_METHOD_OFS      19u
#define COMP_TRAJ_DATA_OFS        20u

/* Decoded representation. The arrays contain only the first `horizon`
 * entries. controls[][0] is steering and controls[][1] is duty/current.
 */

struct comp_control_trajectory_s
{
  uint64_t timestamp_us;
  uint64_t solution_time_us;
  float    dt;
  float    poses[COMP_TRAJ_MAX_HORIZON][2];
  float    controls[COMP_TRAJ_MAX_HORIZON][2];
  uint8_t  horizon;
  uint8_t  control_method;
};

/* An immediate actuator command from the companion.
 *
 * This is the immediate, actuating half of the autonomous input.
 * CONTROL_TRAJ publishes a plan and does not itself actuate anything.
 *
 * timestamp_us is UTC and is NOT optional here, unlike EXTERNAL_POSE where a
 * zero means "stamp it on arrival". A pose with no timestamp costs accuracy;
 * a command with no timestamp cannot have its freshness checked at all, and
 * an actuator command whose age is unknown is the one thing this link must
 * not act on.
 */

struct comp_direct_control_s
{
  uint64_t timestamp_us;   /*  0: UTC us, when the companion sent it */
  float    steering;       /*  8: -1..+1, left positive */
  float    throttle;       /* 12: duty -1..+1, or amps -50..+50 */
  uint8_t  throttle_type;  /* 16: COMP_THROTTLE_* */
  uint8_t  pad[7];         /* 17: a uint64 first member forces 8-byte
                            *     alignment, so this pads to 24 whatever
                            *     it says. Declared, so the wire format is
                            *     what the struct says rather than what the
                            *     compiler decided. */
};

/* The vehicle's full state, sent at EXT_TX_RATE.
 *
 * FRAMES, and they are not all the same one - this follows ROS
 * nav_msgs/Odometry, where the pose is in the world frame and the twist is
 * in the body frame:
 *
 *   position, quaternion   local ENU (x east, y north, z up)
 *   velocity               body FLU (x forward, y left, z up)
 *   angular_velocity       body FLU
 *   accel                  body FLU
 *
 * The estimator works in ENU velocity; this converts. Angular velocity is
 * body by construction because it is the gyro.
 *
 * reset_counter matters more than it looks: the datum reset moves position
 * discontinuously, and anything differentiating position on the companion
 * side needs to know that happened rather than seeing a spike.
 */

struct comp_vehicle_state_s
{
  uint64_t timestamp_us;      /*  0: UTC us once synced, else monotonic */

  float    position[3];       /*  8: m, local ENU */
  float    quaternion[4];     /* 20: w x y z, body FLU to local ENU */

  float    velocity[3];       /* 36: m/s, BODY frame */
  float    angular_velocity[3]; /* 48: rad/s, body - the filtered gyro */

  /* Angle between the velocity vector and the vehicle's heading.
   *
   * NaN until the estimator carries it as a state. NOT zero: zero is a
   * perfectly good slip angle meaning "travelling straight ahead", and a
   * consumer cannot tell that apart from "not computed". NaN can only mean
   * the latter. Check it with isnan() before use.
   */

  float    side_slip_rad;     /* 60 */

  /* Linear acceleration: calibrated specific force with the EKF's remaining
   * accelerometer bias and attitude-projected gravity removed. It therefore
   * reads zero-mean at rest rather than retaining either 9.8 m/s^2 or the
   * residual bias the EKF has already learned.
   */

  float    accel[3];          /* 64: m/s^2, body FLU */

  float    wheel_torque_nm;   /* 76: VESC current x VESC_TORQUE_K */
  float    steering_angle;    /* 80: VESC ADC x K, or mapped sent command */
  float    motor_speed_ms;    /* 84: tachometer rate x VESC_SPEED_K */

  uint8_t  solution_status;   /* 88: ESTIMATOR_* validity bits */
  uint8_t  reset_counter;     /* 89: estimator reset generation */
  uint8_t  source_valid;      /* 90: COMP_SRC_* - which inputs were fresh */
  uint8_t  pad;               /* 91: align the packed status */
  uint32_t rc_status;         /* 92: raw PWM and control state, bits below */
};

/* rc_status: the two raw PWM values consume 12 bits each; four booleans use
 * the high byte. A zero PWM means unavailable, and COMP_SRC_RC then stays
 * clear. CH6 is the physical trigger level; CURRENT is the router's latched
 * control mode, so releasing a momentary CH6 does not lose the selected mode.
 */

#define COMP_RC_PWM_MASK          0x0fffu
#define COMP_RC_STEER_SHIFT       0u
#define COMP_RC_THROTTLE_SHIFT   12u
#define COMP_RC_ARMED            (1u << 24)
#define COMP_RC_AUTO             (1u << 25)
#define COMP_RC_TRIGGER_HIGH     (1u << 26)
#define COMP_RC_CURRENT          (1u << 27)

/* Which inputs were actually fresh when this was assembled.
 *
 * Without this a consumer cannot tell a genuine zero from a source that is
 * not publishing: a stopped VESC and a stationary vehicle both report zero
 * wheel torque, and only one of them means the vehicle is under control.
 */

#define COMP_SRC_ESTIMATOR   (1u << 0)
#define COMP_SRC_GYRO        (1u << 1)
#define COMP_SRC_ACCEL       (1u << 2)
#define COMP_SRC_VESC        (1u << 3)
#define COMP_SRC_RC          (1u << 4)
#define COMP_SRC_STEERING    (1u << 5)

/* Clock synchronisation, request and reply.
 *
 * The ordinary ping-pong: the companion sends its own clock, the board
 * echoes it and adds when it saw the request and when it sent the reply.
 * The companion then has four timestamps and can solve for both the offset
 * and the round trip:
 *
 *   offset     = ((board_rx - host_tx) + (board_tx - host_rx)) / 2
 *   round_trip = (host_rx - host_tx) - (board_tx - board_rx)
 *
 * Both terms matter. Without board_tx the board's own processing delay is
 * indistinguishable from wire latency and lands entirely in the offset.
 *
 * Sending several and keeping the one with the SMALLEST round trip is the
 * standard move: the offset estimate is only as good as the path is
 * symmetric, and the least-delayed exchange is the least asymmetric one.
 */

/* A sync burst is bracketed: START, then `count` exchanges, then END
 * carrying the result the companion settled on.
 *
 * The bracket is not what makes the offset accurate - the exchanges do that.
 * What it buys is that the BOARD knows a sync is in progress and what the
 * companion concluded, so `companion status` can show the agreed offset
 * instead of the board having no idea whether its peer thinks the clocks
 * are aligned.
 */

struct comp_timesync_start_s
{
  uint32_t count;           /* 0: exchanges about to be sent */
  uint32_t pad;             /* 4 */
};

struct comp_timesync_end_s
{
  /* Observed UTC minus board TIM5, in microseconds, at this sync. The
   * companion computes it because only it sees all four timestamps of an
   * exchange. The first observation establishes UTC; later observations
   * estimate the affine UTC/TIM5 rate and remove phase error by slewing, not
   * by replacing this offset and stepping time.
   *
   * The board does not adopt UTC as its own timebase, and must not. Every
   * internal timestamp - IMU samples, the delay ring, the fusion horizon,
   * every staleness check - is monotonic, and monotonic is the only clock
   * that cannot step. Moving those to UTC would jump them from microseconds
   * since boot to microseconds since 1970, at which point every buffered
   * sample reads as impossibly old and the filter resets.
   *
   * So UTC is a WIRE format: converted on the way out, converted back on the
   * way in, and never seen by the estimator.
   */

  int64_t  utc_offset_us;   /*  0: observed UTC - board TIM5 */
  uint32_t trip_us;         /*  8: the round trip it was measured at */
  uint32_t samples;         /* 12: exchanges that came back */
};

struct comp_timesync_req_s
{
  uint64_t host_tx_us;      /* 0: companion UTC microseconds when asked */
};

struct comp_timesync_rep_s
{
  uint64_t host_tx_us;      /*  0: echoed, so replies cannot be mismatched */
  uint64_t board_rx_us;     /*  8: board clock when the request arrived */
  uint64_t board_tx_us;     /* 16: board clock when this reply was sent */
};

enum comp_parse_state_e
{
  COMP_WAIT_SYNC = 0,
  COMP_WAIT_ID,
  COMP_WAIT_LEN,
  COMP_WAIT_PAYLOAD,
  COMP_WAIT_CRC_LO,
  COMP_WAIT_CRC_HI
};

struct comp_parser_s
{
  uint8_t  state;
  uint8_t  id;
  uint8_t  len;
  uint8_t  fill;
  uint16_t crc_rx;
  uint8_t  payload[COMP_MAX_PAYLOAD];

  uint32_t frames;          /* accepted */
  uint32_t crc_errors;
  uint32_t unknown_id;      /* benign: a companion newer than this firmware */
  uint32_t bad_length;      /* NOT benign: the two ends disagree on a format */
  uint32_t resyncs;
};

/* CRC16-CCITT-FALSE. Same polynomial and seed the cal protocol uses, so a
 * host implementing one has implemented both.
 */

uint16_t comp_crc16(FAR const uint8_t *d, size_t n);
uint16_t comp_crc16_update(uint16_t crc, FAR const uint8_t *d, size_t n);

/* Expected FIXED payload length for a known id. Zero means either unknown or
 * variable-length CONTROL_TRAJ; the parser distinguishes that id and checks
 * its length against the embedded horizon.
 *
 * Knowing the length is what separates "a companion newer than this
 * firmware", which is fine and ignorable, from "the two ends disagree about
 * a format", which is not.
 */

uint8_t comp_payload_len(uint8_t id);

size_t comp_control_trajectory_payload_size(uint8_t horizon);
bool comp_control_trajectory_decode(FAR const uint8_t *payload, size_t len,
                                    FAR struct comp_control_trajectory_s *out);

void comp_parser_init(FAR struct comp_parser_s *p);

/* Feed exactly one byte. Returns the message id when a complete, CRC-valid
 * frame with the right length just completed, and 0 otherwise. The payload is
 * then in p->payload for p->len bytes, valid until the next call.
 *
 * One byte at a time, deliberately. A parser that only works on whole frames
 * passes every other test and fails the first time a UART splits one, which
 * it will.
 */

int comp_parser_byte(FAR struct comp_parser_s *p, uint8_t b);

/* Frame payload into out. Returns the number of bytes written, or a negated
 * errno.
 */

int comp_encode(uint8_t id, FAR const void *payload, uint8_t len,
                FAR uint8_t *out, size_t out_size);

/* Is this command one the actuators may be given?
 *
 * Range and mode only - freshness is the caller's job, because it needs a
 * clock. Kept here, next to the format it validates, so every rule can be
 * driven on the host rather than found on a bench with the wheels turning.
 */

bool comp_direct_control_valid(FAR const struct comp_direct_control_s *cmd);

#endif /* __APPS_COMPANION_COMP_PROTO_H */
