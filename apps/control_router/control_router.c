/****************************************************************************
 * apps/control_router/control_router.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/uorb.h>
#include <uORB/uORB.h>

#include "control_router.h"
#include "../param/param.h"
#include "../rc_in/rc_in.h"
#include "../uorb_msgs/uorb_msgs.h"
#include "../vesc/vesc.h"

#define ROUTER_PRIORITY       (SCHED_PRIORITY_DEFAULT + 15)
#define ROUTER_STACK          4096
#define ROUTER_PERIOD_MS      20
#define ROUTER_ARM_RETRY_US   100000u

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile bool g_running;
static volatile bool g_should_stop;
static struct control_router_status_s g_status;

static uint64_t router_now_us(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static void status_publish(const struct control_router_status_s *status)
{
  pthread_mutex_lock(&g_lock);
  g_status = *status;
  pthread_mutex_unlock(&g_lock);
}

static void load_axis(struct router_axis_config_s *axis,
                      FAR const char *min_name, FAR const char *trim_name,
                      FAR const char *max_name, FAR const char *dz_name)
{
  axis->negative = (uint16_t)param_i32(min_name);
  axis->trim = (uint16_t)param_i32(trim_name);
  axis->positive = (uint16_t)param_i32(max_name);
  axis->deadzone = (uint16_t)param_i32(dz_name);
}

static void load_config(struct router_config_s *config)
{
  memset(config, 0, sizeof(*config));
  config->map_steering = (uint8_t)param_i32("RC_MAP_STEERING");
  config->map_throttle = (uint8_t)param_i32("RC_MAP_THROTTLE");
  config->map_source = (uint8_t)param_i32("RC_MAP_SOURCE");
  config->map_mode = (uint8_t)param_i32("RC_MAP_MODE");
  config->map_arm = (uint8_t)param_i32("RC_MAP_ARM");
  config->switch_low = (uint16_t)param_i32("RC_SW_LOW");
  config->switch_high = (uint16_t)param_i32("RC_SW_HIGH");
  config->rc_timeout_us = (uint32_t)param_i32("RC_INPUT_TO_MS") * 1000u;
  config->auto_timeout_us = (uint32_t)param_i32("AUTO_CMD_TO_MS") * 1000u;
  config->duty_max = param_f32("VESC_DUTY_MAX");
  config->current_max = param_f32("VESC_CUR_MAX");
  config->arm_motor_max = param_f32("RC_ARM_MAX");
  load_axis(&config->steering, "RC_ST_MIN", "RC_ST_TRIM", "RC_ST_MAX",
            "RC_ST_DZ");
  load_axis(&config->throttle, "RC_THR_MIN", "RC_THR_TRIM", "RC_THR_MAX",
            "RC_THR_DZ");
}

FAR const char *control_router_reason_name(uint8_t reason)
{
  static FAR const char *const names[ROUTER_REASON_COUNT] =
  {
    "OK", "DISARMED", "RC_LOST", "ARM_CYCLE", "NOT_NEUTRAL",
    "ARM_HOLD", "SOURCE_HOLD", "AUTO_STALE", "AUTO_CYCLE", "INVALID"
  };

  return reason < ROUTER_REASON_COUNT ? names[reason] : "UNKNOWN";
}

static int control_router_daemon(int argc, FAR char *argv[])
{
  struct router_config_s config;
  struct router_state_s policy;
  struct router_input_s input;
  struct control_router_status_s status;
  struct rc_in_s rc;
  struct control_cmd_s auto_cmd;
  struct pollfd pfd[2];
  uint64_t last_arm_attempt = 0;
  int rc_sub = -1;
  int auto_sub = -1;
  int output_pub = -1;
  int result = EXIT_FAILURE;

  UNUSED(argc);
  UNUSED(argv);
  memset(&status, 0, sizeof(status));
  memset(&input, 0, sizeof(input));
  memset(&rc, 0, sizeof(rc));
  memset(&auto_cmd, 0, sizeof(auto_cmd));
  load_config(&config);

  if (!router_config_valid(&config))
    {
      syslog(LOG_ERR, "[router] invalid RC mapping/calibration parameters\n");
      goto out;
    }

  rc_sub = orb_subscribe(ORB_ID(rc_in));
  auto_sub = orb_subscribe(ORB_ID(control_cmd));
  output_pub = actuator_command_advertise();

  if (rc_sub < 0 || auto_sub < 0 || output_pub < 0)
    {
      syslog(LOG_ERR, "[router] uORB setup failed rc=%d auto=%d out=%d\n",
             rc_sub, auto_sub, output_pub);
      goto out;
    }

  pfd[0].fd = rc_sub;
  pfd[0].events = POLLIN;
  pfd[1].fd = auto_sub;
  pfd[1].events = POLLIN;
  router_state_init(&policy);
  status.running = true;
  g_running = true;
  status_publish(&status);
  syslog(LOG_INFO,
         "[router] RC ch steer=%u throttle=%u source=%u mode=%u arm=%u\n",
         config.map_steering, config.map_throttle, config.map_source,
         config.map_mode, config.map_arm);

  while (!g_should_stop)
    {
      struct router_output_s routed;
      struct actuator_command_s output;
      bool updated = false;
      bool was_armed;
      uint64_t now;

      poll(pfd, 2, ROUTER_PERIOD_MS);

      if (orb_check(rc_sub, &updated) == OK && updated &&
          orb_copy(ORB_ID(rc_in), rc_sub, &rc) == OK)
        {
          input.rc_timestamp = rc.timestamp;
          input.rc_count = rc.count;
          input.rc_ok = rc.ok != 0;
          input.rc_failsafe = rc.failsafe != 0;
          memcpy(input.rc_channel, rc.channel, sizeof(input.rc_channel));
          status.rc_source = rc.source;
        }

      updated = false;
      if (orb_check(auto_sub, &updated) == OK && updated &&
          orb_copy(ORB_ID(control_cmd), auto_sub, &auto_cmd) == OK)
        {
          now = router_now_us();
          if (auto_cmd.timestamp == 0 || auto_cmd.timestamp > now)
            {
              auto_cmd.timestamp = now;
            }

          input.auto_timestamp = auto_cmd.timestamp;
          input.auto_motor = auto_cmd.motor;
          input.auto_steering = auto_cmd.steering;
          input.auto_mode = auto_cmd.mode;
          input.auto_present = true;
        }

      now = router_now_us();
      input.now_us = now;
      was_armed = policy.actual_armed;
      router_policy_step(&config, &policy, &input, &routed);

      memset(&output, 0, sizeof(output));
      output.timestamp = now;
      output.motor = routed.motor;
      output.steering = routed.steering;
      output.mode = routed.mode;

      if (actuator_command_publish(output_pub, &output) < 0)
        {
          status.publish_errors++;
        }
      else
        {
          status.publications++;
        }

      if (routed.request_disarm)
        {
          if (vesc_arm(false) == OK && was_armed)
            {
              status.disarms++;
            }
        }

      if (routed.request_arm &&
          (last_arm_attempt == 0 || now - last_arm_attempt >=
           ROUTER_ARM_RETRY_US))
        {
          last_arm_attempt = now;
          if (vesc_arm(true) == OK)
            {
              policy.actual_armed = true;
              policy.arm_holding = false;
              status.arm_success++;
            }
          else
            {
              status.arm_refused++;
            }
        }

      if (routed.reason == ROUTER_REASON_RC_LOST &&
          status.reason != ROUTER_REASON_RC_LOST)
        {
          status.rc_losses++;
        }

      if (routed.reason == ROUTER_REASON_AUTO_STALE &&
          status.reason != ROUTER_REASON_AUTO_STALE)
        {
          status.auto_stale++;
        }

      status.armed = policy.actual_armed;
      status.rc_valid = routed.rc_valid;
      status.auto_valid = routed.auto_valid;
      status.source = routed.source;
      status.mode = routed.mode;
      status.reason = routed.reason;
      status.rc_throttle = routed.rc_throttle;
      status.rc_steering = routed.rc_steering;
      status.output_motor = routed.motor;
      status.output_steering = routed.steering;
      status.rc_age_us = input.rc_timestamp != 0 && now >= input.rc_timestamp ?
                         now - input.rc_timestamp : UINT64_MAX;
      status.auto_age_us = input.auto_timestamp != 0 &&
                           now >= input.auto_timestamp ?
                           now - input.auto_timestamp : UINT64_MAX;
      status_publish(&status);
    }

  result = EXIT_SUCCESS;

out:
  if (output_pub >= 0)
    {
      struct actuator_command_s neutral;

      memset(&neutral, 0, sizeof(neutral));
      neutral.timestamp = router_now_us();
      actuator_command_publish(output_pub, &neutral);
    }

  vesc_arm(false);

  if (rc_sub >= 0) orb_unsubscribe(rc_sub);
  if (auto_sub >= 0) orb_unsubscribe(auto_sub);
  if (output_pub >= 0) orb_unadvertise(output_pub);
  status.running = false;
  status.armed = false;
  status_publish(&status);
  g_running = false;
  return result;
}

int control_router_start(void)
{
  int pid;
  int wait;

  if (g_running)
    {
      return -EALREADY;
    }

  g_should_stop = false;
  memset(&g_status, 0, sizeof(g_status));
  pid = task_create("control_router", ROUTER_PRIORITY, ROUTER_STACK,
                    control_router_daemon, NULL);

  if (pid < 0)
    {
      return -errno;
    }

  for (wait = 0; wait < 100 && !g_running; wait++)
    {
      usleep(10000);
    }

  return g_running ? OK : -EIO;
}

int control_router_stop(void)
{
  int wait;

  if (!g_running)
    {
      return -ESRCH;
    }

  g_should_stop = true;
  for (wait = 0; wait < 100 && g_running; wait++)
    {
      usleep(10000);
    }

  return g_running ? -ETIMEDOUT : OK;
}

void control_router_status(FAR struct control_router_status_s *status)
{
  if (status == NULL)
    {
      return;
    }

  pthread_mutex_lock(&g_lock);
  *status = g_status;
  pthread_mutex_unlock(&g_lock);
}
