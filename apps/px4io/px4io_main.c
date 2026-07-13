/****************************************************************************
 * apps/px4io/px4io_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `px4io` - talk to the PX4IO co-processor.
 *
 *   px4io status              identify IO and dump its status flags
 *   px4io start [rate_hz]     start the background poller (default 50 Hz)
 *   px4io stop                stop it
 *   px4io rc [-w]             show decoded RC channels (-w = watch)
 *   px4io arm | disarm        enable / disable the servo rails
 *   px4io pwm <ch> <us>       set one channel's pulse width
 *   px4io pwm all <us>        set every channel
 *   px4io rate <hz>           PWM frame rate (50 for an analog servo)
 *   px4io reg <page> <off> [n]        read registers
 *   px4io reg <page> <off> = <value>  write one register
 *
 * Bringing up a steering servo on the FMU PWM OUT connector:
 *
 *   px4io status              expect protocol 5
 *   px4io start               keeps the link alive - without this IO will
 *                             failsafe the outputs after 500 ms
 *   px4io rate 50
 *   px4io arm
 *   px4io pwm 1 1500          centre
 *   px4io pwm 1 1900          one side
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "px4io.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void px4io_usage(void)
{
  printf("Usage: px4io <command>\n"
         "  status              identify IO and show its status\n"
         "  start [rate_hz]     start the poller (default 50 Hz)\n"
         "  stop                stop the poller\n"
         "  rc [-w]             show RC channels (-w to watch)\n"
         "  arm | disarm        enable / disable the servo rails\n"
         "  pwm <ch> <us>       set one channel (ch = 1..%d)\n"
         "  pwm all <us>        set every channel\n"
         "  rate <hz>           PWM frame rate (50 for an analog servo)\n"
         "  reg <page> <off> [n]         read registers\n"
         "  reg <page> <off> = <value>   write one register\n"
         "\n"
         "Note: IO drops the outputs to failsafe if the FMU stops talking to\n"
         "it for 500 ms, so 'px4io start' must be running for PWM to hold.\n",
         PX4IO_SERVO_COUNT);
}

static void px4io_print_flags(uint16_t flags)
{
  static const struct
  {
    uint16_t bit;
    FAR const char *name;
  }
  names[] =
  {
    { PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED, "OUTPUTS_ARMED" },
    { PX4IO_P_STATUS_FLAGS_RC_OK,         "RC_OK"         },
    { PX4IO_P_STATUS_FLAGS_RC_PPM,        "RC_PPM"        },
    { PX4IO_P_STATUS_FLAGS_RC_DSM,        "RC_DSM"        },
    { PX4IO_P_STATUS_FLAGS_RC_SBUS,       "RC_SBUS"       },
    { PX4IO_P_STATUS_FLAGS_FMU_OK,        "FMU_OK"        },
    { PX4IO_P_STATUS_FLAGS_RAW_PWM,       "RAW_PWM"       },
    { PX4IO_P_STATUS_FLAGS_INIT_OK,       "INIT_OK"       },
    { PX4IO_P_STATUS_FLAGS_FAILSAFE,      "FAILSAFE"      },
    { PX4IO_P_STATUS_FLAGS_SAFETY_OFF,    "SAFETY_OFF"    },
    { PX4IO_P_STATUS_FLAGS_RC_ST24,       "RC_ST24"       },
    { PX4IO_P_STATUS_FLAGS_RC_SUMD,       "RC_SUMD"       },
  };

  size_t i;

  printf("  flags          0x%04x  ", flags);

  for (i = 0; i < sizeof(names) / sizeof(names[0]); i++)
    {
      if (flags & names[i].bit)
        {
          printf("%s ", names[i].name);
        }
    }

  printf("\n");
}

static int px4io_do_status(void)
{
  struct px4io_status_s s;
  int ret = px4io_get_status(&s);

  if (ret < 0)
    {
      fprintf(stderr, "px4io: cannot read status: %d\n", ret);
      return 1;
    }

  printf("PX4IO on %s @ %d baud\n", PX4IO_DEVPATH, PX4IO_BAUD);
  printf("  protocol       %u%s\n", s.protocol_version,
         s.protocol_version == PX4IO_PROTOCOL_VERSION ? " (ok)" : " (!)");
  printf("  hardware       0x%04x\n", s.hardware_version);
  printf("  max transfer   %u bytes\n", s.max_transfer);
  printf("  actuators      %u\n", s.actuator_count);
  printf("  rc channels    %u\n", s.rc_input_count);
  printf("  free memory    %u bytes\n", s.freemem);
  printf("  IO cpu load    %u%%\n", s.cpuload);
  printf("  servo rail     %u mV\n", s.vservo_mv);
  px4io_print_flags(s.flags);
  printf("  alarms         0x%04x\n", s.alarms);
  printf("  poller         %s\n", px4io_is_running() ? "running" : "stopped");

  if (!(s.flags & PX4IO_P_STATUS_FLAGS_FMU_OK))
    {
      printf("\nnote: FMU_OK is clear - IO thinks we are not talking to it.\n"
             "      run 'px4io start' or the outputs will stay in failsafe.\n");
    }

  return 0;
}

static int px4io_do_rc(bool watch)
{
  do
    {
      struct px4io_rc_s rc;
      int ret = px4io_get_rc(&rc);
      unsigned i;

      if (ret < 0)
        {
          fprintf(stderr, "px4io: cannot read RC: %d\n", ret);
          return 1;
        }

      printf("RC %s  channels=%u  rssi=%u  frames=%u  lost=%u%s\n",
             rc.ok ? "OK" : "--", rc.count, rc.rssi, rc.frames,
             rc.lost_frames, rc.failsafe ? "  FAILSAFE" : "");

      for (i = 0; i < rc.count; i++)
        {
          printf("  ch%-2u %4u us\n", i + 1, rc.channel[i]);
        }

      if (rc.count == 0)
        {
          printf("  (no channels - is a receiver plugged into RC IN?)\n");
        }

      if (watch)
        {
          usleep(200000);
        }
    }
  while (watch);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  if (argc < 2)
    {
      px4io_usage();
      return 1;
    }

  /* Everything needs the link up. probe() is idempotent, so just do it. */

  ret = px4io_probe();
  if (ret < 0)
    {
      fprintf(stderr, "px4io: no response from IO on %s (%d)\n",
              PX4IO_DEVPATH, ret);
      fprintf(stderr, "  not all FMUv6C boards are fitted with the IO chip.\n");
      return 1;
    }

  if (strcmp(argv[1], "status") == 0)
    {
      return px4io_do_status();
    }

  if (strcmp(argv[1], "start") == 0)
    {
      int rate = (argc > 2) ? atoi(argv[2]) : 50;

      ret = px4io_start(rate);
      if (ret < 0)
        {
          fprintf(stderr, "px4io: start failed: %d\n", ret);
          return 1;
        }

      printf("px4io: polling at %d Hz\n", rate);
      return 0;
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      px4io_stop();
      printf("px4io: stopped\n");
      return 0;
    }

  if (strcmp(argv[1], "rc") == 0)
    {
      bool watch = (argc > 2 && strcmp(argv[2], "-w") == 0);
      return px4io_do_rc(watch);
    }

  if (strcmp(argv[1], "arm") == 0 || strcmp(argv[1], "disarm") == 0)
    {
      bool arm = (strcmp(argv[1], "arm") == 0);

      ret = px4io_arm(arm);
      if (ret < 0)
        {
          fprintf(stderr, "px4io: %s failed: %d\n", argv[1], ret);
          return 1;
        }

      printf("px4io: outputs %s\n", arm ? "ARMED" : "disarmed");

      if (arm && !px4io_is_running())
        {
          printf("warning: the poller is not running, so IO will failsafe the\n"
                 "         outputs in 500 ms. run 'px4io start'.\n");
        }

      return 0;
    }

  if (strcmp(argv[1], "rate") == 0 && argc == 3)
    {
      ret = px4io_set_pwm_rate((uint16_t)atoi(argv[2]));
      if (ret < 0)
        {
          fprintf(stderr, "px4io: rate failed: %d\n", ret);
          return 1;
        }

      printf("px4io: PWM rate %s Hz\n", argv[2]);
      return 0;
    }

  if (strcmp(argv[1], "pwm") == 0 && argc == 4)
    {
      uint16_t pwm[PX4IO_SERVO_COUNT];
      int us = atoi(argv[3]);
      unsigned i;

      if (us != 0 && (us < 800 || us > 2200))
        {
          fprintf(stderr, "px4io: %d us is outside the sane servo range "
                          "(800-2200, or 0 to disable)\n", us);
          return 1;
        }

      /* DIRECT_PWM is written as a block, so start from what we last sent
       * rather than zeroing the channels we are not touching.
       */

      for (i = 0; i < PX4IO_SERVO_COUNT; i++)
        {
          pwm[i] = 0;
        }

      px4io_reg_read(PX4IO_PAGE_DIRECT_PWM, 0, pwm, PX4IO_SERVO_COUNT);

      if (strcmp(argv[2], "all") == 0)
        {
          for (i = 0; i < PX4IO_SERVO_COUNT; i++)
            {
              pwm[i] = (uint16_t)us;
            }
        }
      else
        {
          int ch = atoi(argv[2]);

          if (ch < 1 || ch > PX4IO_SERVO_COUNT)
            {
              fprintf(stderr, "px4io: channel must be 1..%d\n",
                      PX4IO_SERVO_COUNT);
              return 1;
            }

          pwm[ch - 1] = (uint16_t)us;
        }

      ret = px4io_set_pwm(pwm, PX4IO_SERVO_COUNT);
      if (ret < 0)
        {
          fprintf(stderr, "px4io: pwm failed: %d\n", ret);
          return 1;
        }

      printf("px4io: ch%s = %d us\n", argv[2], us);

      if (!px4io_is_running())
        {
          printf("warning: the poller is not running, so this will be held for\n"
                 "         only 500 ms. run 'px4io start'.\n");
        }

      return 0;
    }

  /* px4io reg <page> <offset> [count]
   * px4io reg <page> <offset> = <value>
   */

  if (strcmp(argv[1], "reg") == 0 && argc >= 4)
    {
      int page = (int)strtol(argv[2], NULL, 0);
      int off  = (int)strtol(argv[3], NULL, 0);

      if (argc == 6 && strcmp(argv[4], "=") == 0)
        {
          uint16_t value = (uint16_t)strtol(argv[5], NULL, 0);

          ret = px4io_reg_set((uint8_t)page, (uint8_t)off, value);
          if (ret < 0)
            {
              fprintf(stderr, "px4io: write failed: %d\n", ret);
              return 1;
            }

          printf("page %d offset %d = %u\n", page, off, value);
          return 0;
        }
      else
        {
          uint16_t regs[PKT_MAX_REGS];
          int count = (argc > 4) ? atoi(argv[4]) : 1;
          int i;

          if (count < 1 || count > PKT_MAX_REGS)
            {
              fprintf(stderr, "px4io: count must be 1..%d\n", PKT_MAX_REGS);
              return 1;
            }

          ret = px4io_reg_read((uint8_t)page, (uint8_t)off, regs, count);
          if (ret < 0)
            {
              fprintf(stderr, "px4io: read failed: %d\n", ret);
              return 1;
            }

          for (i = 0; i < count; i++)
            {
              printf("page %d offset %-3d = %5u  0x%04x\n",
                     page, off + i, regs[i], regs[i]);
            }

          return 0;
        }
    }

  px4io_usage();
  return 1;
}
