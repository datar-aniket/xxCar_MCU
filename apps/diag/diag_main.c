/* SPDX-License-Identifier: Apache-2.0
 * Passive observations only: no raw bus access, subscription/activation,
 * reset, CAN transmission, RC consumption, or actuator movement.
 */
#include <nuttx/config.h>
#include <arch/board/board.h>
#include <arch/board/fdcan.h>
#include <arch/board/peripheral_health.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "../rc/rc.h"

static const char *const names[] = {"imu0", "imu1", "baro", "can", "rc"};
#define COMPONENTS 5

struct observation_s
{
  uint32_t progress;
  uint32_t errors;
  uint32_t recovered;
  uint32_t tx;
  uint32_t completed;
  bool ready;
  bool healthy;
};

static uint64_t now_us(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static struct observation_s observe(unsigned component, bool print)
{
  struct observation_s o = {0};
  if (component < BOARD_HEALTH_COUNT)
    {
      struct board_sensor_health_s s;
      board_sensor_health(component, &s);
      o.progress = s.samples;
      o.errors = s.errors + s.checks_failed;
      o.recovered = s.recoveries;
      o.ready = s.registered;
      o.healthy = s.registered && s.running && s.last_error == 0;
      if (print)
        printf("%-5s registered=%d running=%d samples=%" PRIu32
               " errors=%" PRIu32 " config_fail=%" PRIu32
               " flush=%" PRIu32 " recovery=%" PRIu32 "/%" PRIu32
               " last_errno=%d\n", names[component], s.registered, s.running,
               s.samples, s.errors, s.checks_failed, s.fifo_flushes,
               s.recoveries, s.recovery_attempts, s.last_error);
    }
  else if (component == 3)
    {
      struct fdcan_stats_s s;
      fdcan_stats(&s);
      o.progress = s.rx;
      o.errors = s.lost + s.ring_full + s.tx_expired + s.bus_off_count;
      o.recovered = s.recoveries;
      o.tx = s.tx;
      o.completed = s.tx_completed;
      o.ready = s.ready;
      o.healthy = s.ready && !s.bus_off && !s.error_passive;
      if (print)
        printf("can   ready=%d off=%d passive=%d LEC=%u rx=%" PRIu32
               " rx_lost=%" PRIu32 "/%" PRIu32
               " tx_queued=%" PRIu32 " completed=%" PRIu32
               " cancelled=%" PRIu32 " expired=%" PRIu32
               " busy=%" PRIu32 " pending=%08" PRIx32
               " bus_off=%" PRIu32 " recovered=%" PRIu32 "\n",
               s.ready, s.bus_off, s.error_passive, s.last_error, s.rx,
               s.lost, s.ring_full, s.tx, s.tx_completed, s.tx_cancelled,
               s.tx_expired, s.tx_full, s.pending, s.bus_off_count,
               s.recoveries);
    }
  else
    {
      struct rc_status_s s;
      rc_get_status(&s);
      o.progress = s.frames;
      o.errors = s.errors + s.timeouts + s.lost_frames;
      o.ready = s.running;
      o.healthy = s.running && s.ok && !s.failsafe &&
                  now_us() - s.last_valid_us < RC_TIMEOUT_US;
      if (print)
        printf("rc    running=%d proto=%u ok=%d failsafe=%d frames=%" PRIu32
               " errors=%" PRIu32 " lost=%u timeouts=%" PRIu32
               " streak=%u good_age_ms=%" PRIu64 "\n",
               s.running, s.proto, s.ok, s.failsafe, s.frames, s.errors,
               s.lost_frames, s.timeouts, s.invalid_streak,
               s.last_valid_us ? (now_us() - s.last_valid_us) / 1000 :
                                UINT64_MAX);
    }
  return o;
}

static int test(unsigned component, unsigned seconds)
{
  struct observation_s first = observe(component, false);
  struct observation_s prev = first;
  struct observation_s last = first;
  uint64_t start = now_us();
  uint64_t progress_at = start;
  uint64_t max_gap = 0;
  uint64_t gap_limit = component == 2 || component == 3 ? 500000 : 100000;
  unsigned faults = 0;

  if (!first.ready)
    {
      printf("%s: NOT TESTED (driver absent/stopped/uninstrumented)\n",
             names[component]);
      return 1;
    }

  while (now_us() - start < (uint64_t)seconds * 1000000)
    {
      usleep(20000);
      uint64_t now = now_us();
      last = observe(component, false);
      uint64_t gap = now - progress_at;
      if (gap > max_gap) max_gap = gap;
      if (!last.ready || !last.healthy || last.errors != prev.errors)
        faults++;
      if (last.progress != prev.progress) progress_at = now;
      prev = last;
    }

  bool passed = last.progress != first.progress && faults == 0 &&
                max_gap <= gap_limit;
  if (component == 3 && last.tx != first.tx &&
      last.completed == first.completed) passed = false;
  printf("%s: %s progress=%" PRIu32 " errors_delta=%" PRIu32
         " fault_observations=%u max_observed_gap_ms=%" PRIu64
         " recovery_delta=%" PRIu32 "\n", names[component],
         passed ? "PASS" : "FAIL", last.progress - first.progress,
         last.errors - first.errors, faults, max_gap / 1000,
         last.recovered - first.recovered);
  if (component == 3)
    printf("  TX queued/completed delta=%" PRIu32 "/%" PRIu32
           " (bus ACK is not proof the intended ESC applied the command)\n",
           last.tx - first.tx, last.completed - first.completed);
  return passed ? 0 : 1;
}

int main(int argc, char **argv)
{
#ifdef CONFIG_XXCAR_BOARD_MATEKH743
  if (argc == 2 && (strcmp(argv[1], "pwm") == 0 || strcmp(argv[1], "status") == 0))
    {
      struct board_matek_pwm_status_s s;
      board_matek_pwm_status(&s);
      printf("pwm   S1-S8 mask=0x%02x healthy=%d period=%" PRIu32
             " us watchdog=%" PRIu32 " last_errno=%d\n",
             s.mask, s.healthy, s.period_us, s.timeouts, s.last_error);
      for (unsigned i = 0; i < BOARD_MATEK_PWM_CHANNELS; i++)
        printf("  S%u %s requested=%u us\n", i + 1,
               s.mask & (1u << i) ? "PWM" : "off", s.pulse_us[i]);
      printf("  banks S1/S2 TIM8, S3-S6 TIM2, S7/S8 TIM4; independent phase.\n"
             "  Register health/requested pulses are not electrical feedback.\n");
      if (strcmp(argv[1], "pwm") == 0) return 0;
    }
#endif
  if (argc == 2 && strcmp(argv[1], "status") == 0)
    {
      for (unsigned i = 0; i < COMPONENTS; i++) observe(i, true);
      return 0;
    }
  if ((argc == 3 || argc == 4) && strcmp(argv[1], "test") == 0)
    {
      char *end;
      unsigned seconds = 5;
      if (argc == 4)
        {
          unsigned long value = strtoul(argv[3], &end, 10);
          if (*end || end == argv[3] || value < 1 || value > 60) goto usage;
          seconds = value;
        }
      bool all = strcmp(argv[2], "all") == 0;
      int failures = 0;
      bool found = false;
      printf("Passive progress test; no outputs moved or frames transmitted.\n");
      for (unsigned i = 0; i < COMPONENTS; i++)
        if (all || strcmp(argv[2], names[i]) == 0)
          {
            found = true;
            failures += test(i, seconds);
          }
      if (found) return failures ? 1 : 0;
    }
usage:
  printf("Usage: diag status | diag pwm (Matek) | diag test imu0|imu1|baro|can|rc|all [1..60 seconds]\n"
         "Tests require already-running drivers and connected devices.\n"
         "PASS means observed progress, not electrical or timing qualification.\n");
  return 1;
}
