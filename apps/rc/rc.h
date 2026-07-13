/****************************************************************************
 * apps/rc/rc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * RC receiver on a plain FMU serial port.
 *
 * This is the sibling of apps/px4io: a receiver plugged into the RC IN
 * connector is decoded by the PX4IO co-processor, which hands over finished
 * channels. A receiver plugged into TELEM/GPS instead arrives here as a raw
 * byte stream, and we decode it ourselves. Both publish the same `rc_in` uORB
 * topic and set `source`, so nothing downstream has to care which wire it came
 * in on.
 *
 * Protocols
 * ---------
 * SBUS  100000 baud, 8E2, and the signal is INVERTED. Inversion is not
 *       optional and there is no inverter on the FMU's serial connectors - the
 *       STM32's own RXINV does it, via TIOCSINVERT.
 * CRSF  420000 baud, 8N1, not inverted. This is what ELRS speaks.
 *
 * PPM is deliberately absent. It is a pulse train on a timer-capture pin, not a
 * UART protocol, so it cannot be decoded on a serial port at all. The 6C's
 * PPM/SBUS RC IN connector is wired to PX4IO, which already decodes PPM - see
 * apps/px4io. RC_PROT=PPM therefore says so rather than failing quietly.
 *
 * Autodetection (RC_PROT=0) only has to choose between SBUS and CRSF, and it
 * does so the only way that is actually reliable: configure the port for one,
 * listen, and see whether a valid frame turns up. The two differ in baud rate,
 * parity, stop bits AND signal polarity, so there is no way to listen for both
 * at once - a CRSF stream read with SBUS's line settings is noise, not data.
 ****************************************************************************/

#ifndef __APPS_RC_RC_H
#define __APPS_RC_RC_H

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "../rc_in/rc_in.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Which protocol the decoder is currently running. */

#define RC_PROTO_NONE  0
#define RC_PROTO_SBUS  1
#define RC_PROTO_CRSF  2

/* SBUS: 25-byte frame at 100000 baud, 8E2, inverted. */

#define SBUS_BAUD           100000
#define SBUS_FRAME_SIZE     25
#define SBUS_START_BYTE     0x0f
#define SBUS_FLAGS_BYTE     23
#define SBUS_FRAMELOST_BIT  (1 << 2)
#define SBUS_FAILSAFE_BIT   (1 << 3)
#define SBUS_CHANNELS       16

/* CRSF: 420000 baud, 8N1. */

#define CRSF_BAUD           420000
#define CRSF_SYNC_BYTE      0xc8
#define CRSF_TYPE_RC_PACKED 0x16
#define CRSF_PAYLOAD_MAX    60
#define CRSF_CHANNELS       16

/* No frame for this long and the link is considered lost. A receiver sends at
 * 50-150 Hz, so 100 ms is several missed frames - long enough not to twitch on
 * one dropped frame, short enough to notice a pulled cable.
 */

#define RC_TIMEOUT_US       100000

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* One decoded frame. Same shape for both protocols. */

struct rc_frame_s
{
  uint16_t channel[RC_IN_MAX_CHANNELS]; /* microseconds */
  uint8_t  count;
  bool     failsafe;                    /* receiver says failsafe (SBUS only) */
  bool     frame_lost;                  /* receiver dropped a frame (SBUS only) */
};

/* Byte-stream decoder state. Fed one buffer at a time; emits frames. */

struct rc_decoder_s
{
  uint8_t  proto;
  uint8_t  buf[64];
  unsigned nbuf;
  unsigned want;    /* CRSF: bytes still expected in the current frame */
  uint32_t errors;  /* frames rejected: bad CRC, or framing that did not hold */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Feed received bytes to the decoder. Returns true when `out` holds a new
 * frame. Resynchronises by itself on garbage - which is what makes probing for
 * the right protocol work.
 */

void rc_decoder_reset(FAR struct rc_decoder_s *d, uint8_t proto);
bool rc_decode(FAR struct rc_decoder_s *d, FAR const uint8_t *data, size_t len,
               FAR struct rc_frame_s *out);

/* Put a port into a protocol's line settings: baud, parity, stop bits and -
 * for SBUS - RX signal inversion.
 */

int rc_configure_port(int fd, uint8_t proto);

/* The driver. Starts a thread on `devpath` that decodes RC and publishes
 * `rc_in`. proto is RC_PROT_* from the parameter system (RC_PROT_AUTO probes).
 */

int  rc_start(FAR const char *devpath, int32_t proto_param);
void rc_stop(void);
bool rc_is_running(void);

/* What the driver is doing right now, for `rc status`. */

struct rc_status_s
{
  bool     running;
  uint8_t  proto;         /* RC_PROTO_* - what it actually locked on to */
  bool     locked;        /* has it seen a valid frame? */
  bool     ok;            /* link is live right now */
  bool     failsafe;
  uint32_t frames;        /* good frames decoded */
  uint32_t errors;        /* frames rejected (bad CRC / bad framing) */
  uint32_t timeouts;      /* times the link went quiet */
  struct rc_frame_s last;
};

int rc_get_status(FAR struct rc_status_s *status);

#endif /* __APPS_RC_RC_H */
