/****************************************************************************
 * apps/rc_in/rc_in.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Definition of the `rc_in` uORB topic. See rc_in.h.
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <errno.h>
#include <assert.h>
#include <stddef.h>

#include "rc_in.h"

/****************************************************************************
 * Layout checks
 *
 * uORB decodes a topic by walking o_format and advancing the read offset by the
 * size of each conversion, with no realignment. So any padding that creeps into
 * the middle of struct rc_in_s would silently shift every field after it and
 * uorb_listener would print plausible-looking nonsense - it would not fail.
 *
 * Rather than trust that the field order happens to pack, make the compiler
 * prove it: every field must sit exactly where the format string expects.
 ****************************************************************************/

static_assert(offsetof(struct rc_in_s, timestamp)   ==  0, "rc_in layout");
static_assert(offsetof(struct rc_in_s, channel)     ==  8, "rc_in layout");
static_assert(offsetof(struct rc_in_s, frames)      == 44, "rc_in layout");
static_assert(offsetof(struct rc_in_s, lost_frames) == 46, "rc_in layout");
static_assert(offsetof(struct rc_in_s, count)       == 48, "rc_in layout");
static_assert(offsetof(struct rc_in_s, rssi)        == 49, "rc_in layout");
static_assert(offsetof(struct rc_in_s, ok)          == 50, "rc_in layout");
static_assert(offsetof(struct rc_in_s, failsafe)    == 51, "rc_in layout");
static_assert(offsetof(struct rc_in_s, source)      == 52, "rc_in layout");
static_assert(sizeof(struct rc_in_s)                == 56, "rc_in layout");

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_DEBUG_UORB

/* Every field of struct rc_in_s, in order, with a conversion whose size matches
 * the field exactly. uORB steps the read offset by the size of each conversion
 * and does not realign, so this list and the struct have to agree field for
 * field or `uorb_listener rc_in` prints nonsense rather than failing.
 *
 *   %hhu -> uint8_t   %hu -> uint16_t   %"PRIu64" -> uint64_t
 *
 * The trailing pad[3] is deliberately not listed: printing stops after the last
 * conversion, and it carries no information.
 */

static const char rc_in_format[] =
  "timestamp:%" PRIu64
  ",ch1:%hu,ch2:%hu,ch3:%hu,ch4:%hu,ch5:%hu,ch6:%hu"
  ",ch7:%hu,ch8:%hu,ch9:%hu,ch10:%hu,ch11:%hu,ch12:%hu"
  ",ch13:%hu,ch14:%hu,ch15:%hu,ch16:%hu,ch17:%hu,ch18:%hu"
  ",frames:%hu,lost:%hu"
  ",count:%hhu,rssi:%hhu,ok:%hhu,failsafe:%hhu,source:%hhu";
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DEFINE(rc_in, struct rc_in_s, rc_in_format);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rc_in_advertise(void)
{
  return orb_advertise(ORB_ID(rc_in), NULL);
}

int rc_in_publish(int fd, FAR const struct rc_in_s *rc)
{
  if (fd < 0 || rc == NULL)
    {
      return -EINVAL;
    }

  return orb_publish(ORB_ID(rc_in), fd, rc);
}
