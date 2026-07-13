/****************************************************************************
 * apps/px4io/px4io.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Client for the PX4IO co-processor on the Pixhawk 6C.
 *
 * The 6C carries a second MCU (an STM32F103) that owns the RC IN connector and
 * the 8 PWM servo rails. It is NOT wired to the FMU's pins - the only way to
 * reach either is to ask IO over the USART6 link (/dev/ttyS4, 1.5 Mbaud). That
 * is what this client is for:
 *
 *   - RC input:  IO demodulates SBUS/DSM/PPM itself and hands us decoded
 *                channels in microseconds. There is no raw byte stream to
 *                parse, so the RC protocol autodetection question does not
 *                arise for this connector - IO already did it.
 *   - PWM out:   drive the 8 servo rails (steering servo, etc).
 *
 * Not every FMUv6C board is populated with an IO chip (PX4's board_config.h
 * lists NO-PX4IO hardware revisions), so px4io_open() is expected to fail
 * gracefully.
 *
 * RC on a *direct* FMU UART (a receiver plugged into TELEM/GPS rather than
 * RC IN) is a different code path and does not live here.
 *
 * ---------------------------------------------------------------------------
 * Why the handle
 *
 * The connection is a caller-owned struct rather than a hidden global, and that
 * is not a style choice. In a NuttX flat build every task shares one copy of
 * .bss, but file descriptors are per task group. A file descriptor cached in a
 * global is therefore only valid inside the task that opened it: the moment that
 * task exits, the fd is closed, and the next task to run reads the stale number
 * out of the global and gets EBADF. Each task must own its own connection.
 ****************************************************************************/

#ifndef __APPS_PX4IO_PX4IO_H
#define __APPS_PX4IO_PX4IO_H

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "px4io_protocol.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USART6. Fixed by the board wiring, not a preference. */

#define PX4IO_DEVPATH  "/dev/ttyS4"
#define PX4IO_BAUD     1500000

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* One task's connection to IO. Open it, use it, close it - do not share it
 * between tasks (see the note at the top of this file).
 */

struct px4io_s
{
  int     fd;
  uint8_t rc_channels;  /* how many channels this IO says it can decode */
};

/* A decoded RC frame as IO handed it to us. */

struct px4io_rc_s
{
  uint16_t channel[PX4IO_RC_CHANNELS]; /* pulse widths, microseconds */
  uint8_t  count;                      /* channels actually valid */
  uint8_t  rssi;                       /* 0 = nothing, 255 = perfect */
  bool     ok;                         /* IO says the link is good */
  bool     failsafe;                   /* receiver is in failsafe */
  uint16_t frames;                     /* wrapping counter */
  uint16_t lost_frames;                /* wrapping counter */
};

/* What IO reports about itself. */

struct px4io_status_s
{
  uint16_t protocol_version;           /* PX4IO_PROTOCOL_VERSION (5) */
  uint16_t hardware_version;
  uint16_t max_transfer;
  uint16_t actuator_count;
  uint16_t rc_input_count;
  uint16_t flags;                      /* PX4IO_P_STATUS_FLAGS_* */
  uint16_t alarms;
  uint16_t vservo_mv;                  /* servo rail voltage, mV */
  uint16_t freemem;
  uint16_t cpuload;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Open the link and confirm something is actually answering on it. Reads
 * PAGE_CONFIG and checks the protocol version, so a board with no IO chip
 * fitted fails here rather than half-working. Returns 0 on success.
 */

int px4io_open(FAR struct px4io_s *io);
void px4io_close(FAR struct px4io_s *io);

/* Tell IO the FMU has configured it.
 *
 * This is a real handshake, not a formality: IO only raises its INIT_OK status
 * flag when the FMU writes PAGE_DISARMED_PWM, and INIT_OK is half of what gates
 * the servo rails (INIT_OK && (FMU_ARMED || RAW_PWM)). Until this runs, the
 * outputs can never come on, no matter what else is written.
 *
 * The disarmed values are set to zero, which on IO means "emit no pulses at
 * all" - so a disarmed rail is genuinely dead rather than holding a position.
 */

int px4io_init(FAR struct px4io_s *io);

/* Raw register access, for anything this header does not wrap. count is in
 * 16-bit registers and must be <= PKT_MAX_REGS.
 */

int px4io_reg_read(FAR struct px4io_s *io, uint8_t page, uint8_t offset,
                   FAR uint16_t *values, unsigned count);
int px4io_reg_write(FAR struct px4io_s *io, uint8_t page, uint8_t offset,
                    FAR const uint16_t *values, unsigned count);

int px4io_reg_get(FAR struct px4io_s *io, uint8_t page, uint8_t offset,
                  FAR uint16_t *value);
int px4io_reg_set(FAR struct px4io_s *io, uint8_t page, uint8_t offset,
                  uint16_t value);
int px4io_reg_modify(FAR struct px4io_s *io, uint8_t page, uint8_t offset,
                     uint16_t clearbits, uint16_t setbits);

/* Fetch identification + live status. */

int px4io_get_status(FAR struct px4io_s *io, FAR struct px4io_status_s *status);

/* Fetch one decoded RC frame from PAGE_RAW_RC_INPUT. */

int px4io_get_rc(FAR struct px4io_s *io, FAR struct px4io_rc_s *rc);

/* Drive the servo rails. Values are pulse widths in microseconds; 0 disables
 * that channel. Writing here also raises IO's RAW_PWM flag, which is the other
 * half of what arms the outputs.
 *
 * IMPORTANT: IO drops the outputs to failsafe if the FMU goes quiet for
 * PX4IO_FMU_DROP_LIMIT_US (500 ms), so a one-shot call will NOT hold a servo
 * position. The daemon (px4io_start) is what keeps it alive.
 */

int px4io_set_pwm(FAR struct px4io_s *io, FAR const uint16_t *values,
                  unsigned count);

/* Arm or disarm the outputs. */

int px4io_arm(FAR struct px4io_s *io, bool armed);

/* PWM frame rate, Hz. 50 suits an analog steering servo. */

int px4io_set_pwm_rate(FAR struct px4io_s *io, uint16_t rate_hz);

/* Where the rails go when IO decides the FMU is gone, or when disarmed.
 * Note IO *ignores* a zero written to the failsafe page, but honours it on the
 * disarmed page (where it means "no pulses").
 */

int px4io_set_failsafe_pwm(FAR struct px4io_s *io, FAR const uint16_t *values,
                           unsigned count);
int px4io_set_disarmed_pwm(FAR struct px4io_s *io, FAR const uint16_t *values,
                           unsigned count);

/****************************************************************************
 * The daemon
 *
 * A background task - not a pthread - because it has to outlive the NSH command
 * that started it, and because it needs its own file descriptor table.
 *
 * It refreshes the RC snapshot and re-sends the PWM setpoint every cycle. That
 * second job is also what keeps the link alive: writing PAGE_DIRECT_PWM is the
 * only thing IO accepts as proof the FMU still exists, so without this daemon IO
 * clears FMU_OK after 500 ms and drops the rails to failsafe.
 *
 * Returns -EALREADY if a daemon is already running (the rate is not changed;
 * stop it first).
 ****************************************************************************/

int  px4io_start(int rate_hz);
void px4io_stop(void);
bool px4io_is_running(void);

/* The rate the daemon is refreshing setpoints at, Hz (0 if stopped). This is
 * NOT the PWM frame rate: IO pulses the servos at P_SETUP_PWM_DEFAULTRATE
 * whatever we do, and this is how often we give it a fresh value to pulse.
 */

int  px4io_daemon_rate(void);

/* Ask the daemon to hold this PWM setpoint. Takes effect on its next cycle. */

int px4io_set_setpoint(FAR const uint16_t *values, unsigned count);

/* Latest RC frame the daemon saw. -EAGAIN if it has not seen one yet. Cheap:
 * does not touch the wire.
 */

int px4io_rc_latest(FAR struct px4io_rc_s *rc);

#endif /* __APPS_PX4IO_PX4IO_H */
