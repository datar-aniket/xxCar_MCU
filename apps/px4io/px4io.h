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
 * lists NO-PX4IO hardware revisions), so px4io_probe() is the entry point and
 * it is expected to fail gracefully.
 *
 * RC on a *direct* FMU UART (a receiver plugged into TELEM/GPS rather than
 * RC IN) is a different code path and does not live here.
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

int px4io_probe(void);

/* Close the link. */

void px4io_close(void);

/* Raw register access, for anything this header does not wrap. count is in
 * 16-bit registers and must be <= PKT_MAX_REGS.
 */

int px4io_reg_read(uint8_t page, uint8_t offset,
                   FAR uint16_t *values, unsigned count);
int px4io_reg_write(uint8_t page, uint8_t offset,
                    FAR const uint16_t *values, unsigned count);

/* Convenience wrappers for the very common single-register case. */

int px4io_reg_get(uint8_t page, uint8_t offset, FAR uint16_t *value);
int px4io_reg_set(uint8_t page, uint8_t offset, uint16_t value);

/* Read-modify-write a setup register (clearbits then setbits). */

int px4io_reg_modify(uint8_t page, uint8_t offset,
                     uint16_t clearbits, uint16_t setbits);

/* Fetch identification + live status. */

int px4io_get_status(FAR struct px4io_status_s *status);

/* Fetch one decoded RC frame from PAGE_RAW_RC_INPUT. */

int px4io_get_rc(FAR struct px4io_rc_s *rc);

/* Drive the servo rails. Values are pulse widths in microseconds; a value of 0
 * disables that channel. Writing here is also what sets IO's RAW_PWM flag,
 * which is half of what arms the outputs.
 *
 * IMPORTANT: IO drops the outputs to failsafe if the FMU goes quiet for
 * PX4IO_FMU_DROP_LIMIT_US (500 ms), so a one-shot call will not hold a servo
 * position. Something has to keep talking - see px4io_start().
 */

int px4io_set_pwm(FAR const uint16_t *values, unsigned count);

/* Arm or disarm the outputs. Arming sets IO_ARM_OK|FMU_ARMED; disarming clears
 * them, which drops the rails to their PAGE_DISARMED_PWM values.
 */

int px4io_arm(bool armed);

/* Set the PWM frame rate (Hz) for all 8 channels. 50 Hz suits an analog
 * steering servo; ESCs generally accept much more.
 */

int px4io_set_pwm_rate(uint16_t rate_hz);

/* Set the values the rails fall back to when IO decides the FMU is gone
 * (failsafe) or when the outputs are disarmed.
 */

int px4io_set_failsafe_pwm(FAR const uint16_t *values, unsigned count);
int px4io_set_disarmed_pwm(FAR const uint16_t *values, unsigned count);

/* Background poller. Keeps the link alive (so IO does not failsafe), republishes
 * RC, and re-sends the current PWM setpoint at `rate_hz`. Idempotent.
 */

int  px4io_start(int rate_hz);
void px4io_stop(void);
bool px4io_is_running(void);

/* Latest RC frame seen by the poller. Returns -EAGAIN if it has not seen one
 * yet. Cheap - does not touch the wire.
 */

int px4io_rc_latest(FAR struct px4io_rc_s *rc);

#endif /* __APPS_PX4IO_PX4IO_H */
