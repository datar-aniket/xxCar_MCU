/****************************************************************************
 * apps/rc_in/rc_in.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The `rc_in` uORB topic: one decoded RC frame, whatever wire it arrived on.
 *
 * This is deliberately its own tiny library rather than living inside a driver.
 * RC reaches this board two different ways - demodulated by the PX4IO
 * co-processor (the RC IN connector), or decoded by us from a raw SBUS/CRSF
 * byte stream on an FMU UART - and consumers should not care which. Both
 * publishers fill in the same struct and set `source` to say where it came
 * from.
 *
 * Subscribers:  uorb_listener rc_in
 *
 * ---------------------------------------------------------------------------
 * Layout warning
 *
 * uORB prints a topic by walking o_format and stepping the read offset by the
 * size of each conversion (see lib_libbsprintf.c: `offset += sizeof(...)`).
 * It applies NO alignment padding. So struct rc_in_s must have no *internal*
 * padding, and o_format must list every field in order. Reorder a field or drop
 * one from the format and uorb_listener silently prints garbage - it does not
 * fail. The trailing pad below exists to keep that property explicit.
 ****************************************************************************/

#ifndef __APPS_RC_IN_RC_IN_H
#define __APPS_RC_IN_RC_IN_H

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <uORB/uORB.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RC_IN_MAX_CHANNELS  18

/* Where the frame came from (struct rc_in_s.source). */

#define RC_IN_SRC_NONE      0
#define RC_IN_SRC_PX4IO     1  /* RC IN connector, demodulated by PX4IO */
#define RC_IN_SRC_SBUS      2  /* raw SBUS decoded by us on an FMU UART */
#define RC_IN_SRC_CRSF      3  /* CRSF / ELRS, ditto */
#define RC_IN_SRC_PPM       4

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rc_in_s
{
  uint64_t timestamp;                     /*  0: microseconds */
  uint16_t channel[RC_IN_MAX_CHANNELS];   /*  8: pulse widths, us */
  uint16_t frames;                        /* 44: wrapping counter */
  uint16_t lost_frames;                   /* 46: wrapping counter */
  uint8_t  count;                         /* 48: channels valid */
  uint8_t  rssi;                          /* 49: 0 = none, 255 = perfect */
  uint8_t  ok;                            /* 50: link is good */
  uint8_t  failsafe;                      /* 51: receiver in failsafe */
  uint8_t  source;                        /* 52: RC_IN_SRC_* */
  uint8_t  pad[3];                        /* 53: keep size a multiple of 8 and
                                           *     internal padding at zero */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DECLARE(rc_in);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Advertise the topic. Returns a uORB fd, or negative on failure. */

int rc_in_advertise(void);

/* Publish one frame. */

int rc_in_publish(int fd, FAR const struct rc_in_s *rc);

#endif /* __APPS_RC_IN_RC_IN_H */
