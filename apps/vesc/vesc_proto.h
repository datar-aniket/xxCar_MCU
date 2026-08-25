/****************************************************************************
 * apps/vesc/vesc_proto.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * VESC CAN payloads. See docs/can_packet.md.
 *
 *   29-bit extended ID = (packet_id << 8) | controller_id
 *
 * Every payload is BIG-ENDIAN, and this MCU is not. That is the whole
 * reason this file exists separately from the driver and the daemon: a byte
 * order mistake on the int32 tachometer is obvious - wrong by millions - but
 * on the signed int16 current it is occasionally PLAUSIBLE, which is far
 * worse. Being testable on a host is what catches that.
 *
 * No I/O, no uORB, no hardware.
 ****************************************************************************/

#ifndef __APPS_VESC_VESC_PROTO_H
#define __APPS_VESC_VESC_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef FAR
#  define FAR
#endif

/* Telemetry, VESC -> host. */

#define VESC_PACKET_STATUS_5          0x1b

/* Commands, host -> VESC.
 *
 * Only the two combined frames are encoded. They carry motor and steering in
 * one six-byte payload, which is the reason they exist: sending the standard
 * SET_DUTY plus a separate servo packet doubles the frame rate and puts a
 * variable skew between the two axes.
 *
 * The other three are defined so the discovery listing can name what it
 * sees.
 */

#define VESC_PACKET_SET_DUTY          0x00
#define VESC_PACKET_SET_CURRENT       0x01
#define VESC_PACKET_PROCESS_SHORT_BUF 0x08
#define VESC_PACKET_SET_CURRENT_SERVO 0x45
#define VESC_PACKET_SET_DUTY_SERVO    0x46

#define VESC_STATUS_5_DLC             8

/* Decoded CAN_PACKET_STATUS_5.
 *
 * The tachometer is an accumulated POSITION count, not a rate. Turning it
 * into a speed is a consumer's job.
 */

struct vesc_status5_s
{
  int32_t tachometer;   /* accumulated counts */
  float   current_a;    /* A, raw / 10 */
  float   adc_volts;    /* V, raw / 1000 - the steering feedback */
};

/* The two halves of the 29-bit identifier. */

uint8_t vesc_packet_id(uint32_t can_id);
uint8_t vesc_controller_id(uint32_t can_id);

/* Decode a STATUS_5 payload. Returns false when the DLC is not what this
 * packet is defined to carry - which is a firmware mismatch, and a different
 * thing from an unknown packet id.
 */

bool vesc_decode_status5(FAR const uint8_t *data, uint8_t dlc,
                         FAR struct vesc_status5_s *out);

#define VESC_CMD_SERVO_DLC            6

/* The microsecond range docs/can_packet.md gives for the servo field. */

#define VESC_SERVO_US_MIN             800u
#define VESC_SERVO_US_MAX             2200u

/* Protocol ceilings, NOT vehicle limits. The vehicle limits are parameters
 * and are applied before this. These exist so that a value the wire format
 * cannot represent is impossible rather than merely unlikely.
 */

#define VESC_PROTO_CUR_LIMIT_A        200.0f
#define VESC_PROTO_DUTY_LIMIT         1.0f

/* Build the 29-bit extended identifier. */

uint32_t vesc_can_id(uint8_t packet_id, uint8_t controller_id);

/* Encode a combined motor + steering command into VESC_CMD_SERVO_DLC bytes.
 *
 * Returns false, having written a ZERO motor value, when the motor argument
 * is not finite. A NaN cast to int32_t is undefined, and on a motor
 * controller "undefined" is a torque command nobody chose.
 *
 * Out-of-range motor and servo values are clamped silently: by the time a
 * value reaches here the daemon has already applied and counted the vehicle
 * limits, so this is a backstop, not a policy decision.
 */

bool vesc_encode_current_servo(float amps, uint16_t servo_us,
                               FAR uint8_t *out);
bool vesc_encode_duty_servo(float duty, uint16_t servo_us,
                            FAR uint8_t *out);

#endif /* __APPS_VESC_VESC_PROTO_H */
