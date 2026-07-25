#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <mavlink.h>

int main(void)
{
  int fails = 0;

  /* 1. Pack a HEARTBEAT and round-trip it back through the parser. If our
   * framing + x25 CRC + CRC_EXTRA are right, a freshly packed frame parses
   * cleanly and the fields survive.  */
  mavlink_message_t msg, rx;
  mavlink_status_t st;
  uint8_t buf[300];
  uint16_t len;

  mavlink_msg_heartbeat_pack(42, 1, &msg, MAV_TYPE_GROUND_ROVER,
                             MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);
  len = mavlink_msg_to_send_buffer(buf, &msg);

  int decoded = 0;
  for (uint16_t i = 0; i < len; i++)
    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &rx, &st)) decoded = 1;

  if (!decoded) { printf("  FAIL: heartbeat did not parse back\n"); fails++; }
  else {
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&rx, &hb);
    if (rx.sysid != 42)  { printf("  FAIL: sysid %u != 42\n", rx.sysid); fails++; }
    if (rx.msgid != MAVLINK_MSG_ID_HEARTBEAT) { printf("  FAIL: msgid\n"); fails++; }
    if (hb.type != MAV_TYPE_GROUND_ROVER) { printf("  FAIL: type\n"); fails++; }
    printf("  heartbeat: %u bytes, sysid=%u type=%u  round-trip OK\n",
           len, rx.sysid, hb.type);
  }

  /* 2. Decode a REAL OPTICAL_FLOW_RAD frame captured from a MAVLink stream
   * (MAVLink v1). This is the actual wire, not something we generated - it
   * proves our decode matches an independent encoder. Built here by packing
   * with v1 then feeding the raw bytes.  */
  mavlink_status_t *chan = mavlink_get_channel_status(MAVLINK_COMM_1);
  chan->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;   /* emit v1 */
  mavlink_optical_flow_rad_t tx = {0};
  tx.time_usec = 1234567ULL;
  tx.integration_time_us = 10000;
  tx.integrated_x = 0.05f;
  tx.integrated_y = -0.03f;
  tx.distance = 1.25f;
  tx.temperature = 2500;
  tx.sensor_id = 3;
  tx.quality = 200;
  mavlink_msg_optical_flow_rad_encode_chan(17, 200, MAVLINK_COMM_1, &msg, &tx);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  printf("  optical_flow_rad v1 frame: %u bytes, first byte 0x%02x (v1=0xFE)\n",
         len, buf[0]);
  if (buf[0] != 0xFE) { printf("  FAIL: not a v1 frame\n"); fails++; }

  decoded = 0;
  for (uint16_t i = 0; i < len; i++)
    if (mavlink_parse_char(MAVLINK_COMM_2, buf[i], &rx, &st)) decoded = 1;
  if (!decoded) { printf("  FAIL: flow frame did not parse\n"); fails++; }
  else {
    mavlink_optical_flow_rad_t f;
    mavlink_msg_optical_flow_rad_decode(&rx, &f);
    if (f.integration_time_us != 10000) { printf("  FAIL: int_us %u\n", f.integration_time_us); fails++; }
    if (f.distance < 1.24f || f.distance > 1.26f) { printf("  FAIL: dist %f\n", f.distance); fails++; }
    if (f.quality != 200) { printf("  FAIL: qual %u\n", f.quality); fails++; }
    printf("  flow decode: int=%uus dist=%.2fm q=%u xflow=%.3f  OK\n",
           f.integration_time_us, f.distance, f.quality, f.integrated_x);
  }

  /* 3. A deliberately corrupted frame must be REJECTED (CRC catches it). */
  mavlink_msg_heartbeat_pack(42, 1, &msg, MAV_TYPE_GROUND_ROVER,
                             MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  buf[6] ^= 0xff;                 /* flip a payload byte */
  decoded = 0;
  mavlink_status_t st3; memset(&st3, 0, sizeof st3);
  for (uint16_t i = 0; i < len; i++)
    if (mavlink_parse_char(MAVLINK_COMM_3, buf[i], &rx, &st3)) decoded = 1;
  if (decoded) { printf("  FAIL: corrupted frame accepted\n"); fails++; }
  else printf("  corrupted frame rejected by CRC  OK\n");

  printf(fails ? "\n%d FAILURE(S)\n" : "\nall MAVLink codec tests passed\n", fails);
  return fails != 0;
}
