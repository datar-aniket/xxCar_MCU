/****************************************************************************
 * apps/control_router/control_router_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control_router.h"

static void usage(void)
{
  printf("Usage: control_router start | stop | status\n");
}

static FAR const char *source_name(uint8_t source)
{
  return source == ROUTER_SOURCE_AUTO ? "AUTO" : "RC";
}

static FAR const char *mode_name(uint8_t mode)
{
  return mode == ROUTER_MODE_CURRENT ? "CURRENT" : "DUTY";
}

static void print_age(FAR const char *name, uint64_t age)
{
  if (age == UINT64_MAX)
    {
      printf("  %-7s never\n", name);
    }
  else
    {
      printf("  %-7s %.1f ms old\n", name, (double)age / 1000.0);
    }
}

static int print_status(void)
{
  struct control_router_status_s s;

  control_router_status(&s);
  if (!s.running)
    {
      printf("control_router: stopped\n");
      return EXIT_FAILURE;
    }

  printf("control_router: %s %s, %s, reason %s\n",
         s.armed ? "ARMED" : "disarmed", source_name(s.source),
         mode_name(s.mode), control_router_reason_name(s.reason));
  printf("  rc      %s source=%u throttle=%+.3f steering=%+.3f\n",
         s.rc_valid ? "valid" : "INVALID", s.rc_source,
         (double)s.rc_throttle, (double)s.rc_steering);
  printf("  auto    %s%s\n", s.auto_valid ? "valid" : "INVALID",
         s.reason == ROUTER_REASON_AUTO_CYCLE ?
           "  - LOCKED OUT until the source switch is cycled to RC and back"
           : "");
  print_age("rc age", s.rc_age_us);
  print_age("auto age", s.auto_age_us);
  printf("  output  motor=%+.3f steering=%+.3f\n",
         (double)s.output_motor, (double)s.output_steering);
  printf("  counts  pub=%" PRIu32 " err=%" PRIu32
         " arm=%" PRIu32 " refused=%" PRIu32 " disarm=%" PRIu32 "\n",
         s.publications, s.publish_errors, s.arm_success, s.arm_refused,
         s.disarms);
  printf("  faults  rc_loss=%" PRIu32 " auto_stale=%" PRIu32 "\n",
         s.rc_losses, s.auto_stale);
  return EXIT_SUCCESS;
}

int main(int argc, FAR char *argv[])
{
  int ret;

  if (argc < 2)
    {
      usage();
      return EXIT_FAILURE;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      return print_status();
    }

  if (strcmp(argv[1], "start") == 0)
    {
      ret = control_router_start();
    }
  else if (strcmp(argv[1], "stop") == 0)
    {
      ret = control_router_stop();
    }
  else
    {
      usage();
      return EXIT_FAILURE;
    }

  if (ret < 0)
    {
      printf("control_router: %s failed: %d\n", argv[1], ret);
      return EXIT_FAILURE;
    }

  printf("control_router: %s\n",
         strcmp(argv[1], "start") == 0 ? "started" : "stopped");
  return EXIT_SUCCESS;
}
