/****************************************************************************
 * apps/logger/logger_auto.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Low-priority automatic logging supervisor. Logger start/stop may wait on
 * filesystem I/O, so this work must never run in the control-router task.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <sched.h>
#include <stdbool.h>
#include <stdlib.h>
#include <syslog.h>
#include <unistd.h>

#include "logger.h"
#include "../param/param.h"

#ifdef CONFIG_XXCAR_CONTROL_ROUTER
#  include "../control_router/control_router.h"
#endif

#define LOG_AUTO_PRIO       (SCHED_PRIORITY_DEFAULT + 1)
#define LOG_AUTO_STACK      2048
#define LOG_AUTO_POLL_US    100000
#define LOG_AUTO_RETRY_US  1000000

static volatile bool g_auto_running;
static volatile bool g_auto_suspended;

static int logger_auto_daemon(int argc, FAR char *argv[])
{
  bool owned = false;
  bool wanted = false;

  UNUSED(argc);
  UNUSED(argv);
  g_auto_running = true;

  for (;;)
    {
#ifdef CONFIG_XXCAR_CONTROL_ROUTER
      struct control_router_status_s router;

      control_router_status(&router);
      wanted = !g_auto_suspended && param_i32("LOG_AUTO") != 0 &&
               router.running &&
               router.source == ROUTER_SOURCE_AUTO;
#else
      wanted = false;
#endif

      if (wanted && !logger_is_running())
        {
          int ret = logger_start();

          if (ret == OK)
            {
              owned = true;
              syslog(LOG_INFO, "[log] AUTO selected: logging started\n");
            }
          else if (ret != -EALREADY)
            {
              syslog(LOG_WARNING,
                     "[log] AUTO logging start failed: %d; retrying\n", ret);
              usleep(LOG_AUTO_RETRY_US);
            }
        }
      else if (!wanted && owned)
        {
          logger_stop();
          owned = false;
          syslog(LOG_INFO, "[log] RC selected: AUTO log stopped\n");
        }

      usleep(LOG_AUTO_POLL_US);
    }

  return EXIT_SUCCESS;
}

int logger_auto_start(void)
{
  int pid;
  int wait;

#ifndef CONFIG_XXCAR_CONTROL_ROUTER
  return -ENOSYS;
#endif

  if (g_auto_running)
    {
      return -EALREADY;
    }

  pid = task_create("log_auto", LOG_AUTO_PRIO, LOG_AUTO_STACK,
                    logger_auto_daemon, NULL);
  if (pid < 0)
    {
      return -errno;
    }

  for (wait = 0; wait < 100 && !g_auto_running; wait++)
    {
      usleep(10000);
    }

  return g_auto_running ? OK : -EIO;
}

bool logger_auto_is_running(void)
{
  return g_auto_running;
}

void logger_auto_suspend(bool suspend)
{
  g_auto_suspended = suspend;
}
