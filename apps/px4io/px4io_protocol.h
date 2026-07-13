/****************************************************************************
 * apps/px4io/px4io_protocol.h
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * The FMU <-> PX4IO register protocol.
 *
 * These definitions describe firmware we do NOT build: the PX4IO co-processor
 * (an STM32F103) ships with PX4's IO firmware already flashed, and we talk to
 * it as-is. So this file is a transcription of PX4's
 * src/modules/px4iofirmware/protocol.h, not an invention - every constant here
 * has to match the other end of the wire exactly.
 *
 * Copyright (c) 2012-2023 PX4 Development Team. Redistribution and use in
 * source and binary forms, with or without modification, are permitted under
 * the terms of the BSD 3-Clause license.
 *
 * ---------------------------------------------------------------------------
 *
 * Protocol summary
 *
 * The FMU is the master; IO only ever answers. Registers are addressed as
 * (page, offset) and are all 16 bits wide. One request is one IOPacket, and IO
 * replies with one IOPacket:
 *
 *   read:   FMU sends count_code = PKT_CODE_READ  | <regs wanted>, no payload
 *           IO  sends count_code = PKT_CODE_SUCCESS | <regs returned> + regs
 *   write:  FMU sends count_code = PKT_CODE_WRITE | <regs sent> + regs
 *           IO  sends count_code = PKT_CODE_SUCCESS | 0, no payload
 *
 * The CRC covers the whole packet (header + the regs actually present) with the
 * crc field itself zeroed.
 ****************************************************************************/

#ifndef __APPS_PX4IO_PX4IO_PROTOCOL_H
#define __APPS_PX4IO_PX4IO_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PX4IO_PROTOCOL_VERSION      5

#define PX4IO_MAX_TRANSFER_LEN      64
#define PKT_MAX_REGS                32   /* by agreement with the IO firmware */

/* Packet codes. The same two bits mean different things depending on which way
 * the packet is travelling, which is why READ/SUCCESS and WRITE/CORRUPT share
 * values.
 */

#define PKT_CODE_READ               0x00 /* FMU->IO  read transaction   */
#define PKT_CODE_WRITE              0x40 /* FMU->IO  write transaction  */

#define PKT_CODE_SUCCESS            0x00 /* IO->FMU  success            */
#define PKT_CODE_CORRUPT            0x40 /* IO->FMU  bad packet         */
#define PKT_CODE_ERROR              0x80 /* IO->FMU  register op failed */

#define PKT_CODE_MASK               0xc0
#define PKT_COUNT_MASK              0x3f

#define PKT_COUNT(_p)  ((_p).count_code & PKT_COUNT_MASK)
#define PKT_CODE(_p)   ((_p).count_code & PKT_CODE_MASK)
#define PKT_SIZE(_p)   ((size_t)((uint8_t *)&((_p).regs[PKT_COUNT(_p)]) - \
                                 (uint8_t *)&(_p)))

/* Page 0: identification. Read-only. */

#define PX4IO_PAGE_CONFIG             0
#define PX4IO_P_CONFIG_PROTOCOL_VERSION   0  /* must read back as 5 */
#define PX4IO_P_CONFIG_HARDWARE_VERSION   1
#define PX4IO_P_CONFIG_BOOTLOADER_VERSION 2
#define PX4IO_P_CONFIG_MAX_TRANSFER       3
#define PX4IO_P_CONFIG_CONTROL_COUNT      4
#define PX4IO_P_CONFIG_ACTUATOR_COUNT     5
#define PX4IO_P_CONFIG_RC_INPUT_COUNT     6
#define PX4IO_P_CONFIG_ADC_INPUT_COUNT    7

/* Page 1: status. Read-only. */

#define PX4IO_PAGE_STATUS             1
#define PX4IO_P_STATUS_FREEMEM        0
#define PX4IO_P_STATUS_CPULOAD        1
#define PX4IO_P_STATUS_FLAGS          2
#define PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED  (1 << 0)
#define PX4IO_P_STATUS_FLAGS_RC_OK          (1 << 1)
#define PX4IO_P_STATUS_FLAGS_RC_PPM         (1 << 2)
#define PX4IO_P_STATUS_FLAGS_RC_DSM         (1 << 3)
#define PX4IO_P_STATUS_FLAGS_RC_SBUS        (1 << 4)
#define PX4IO_P_STATUS_FLAGS_FMU_OK         (1 << 5)
#define PX4IO_P_STATUS_FLAGS_RAW_PWM        (1 << 6)
#define PX4IO_P_STATUS_FLAGS_ARM_SYNC       (1 << 7)
#define PX4IO_P_STATUS_FLAGS_INIT_OK        (1 << 8)
#define PX4IO_P_STATUS_FLAGS_FAILSAFE       (1 << 9)
#define PX4IO_P_STATUS_FLAGS_SAFETY_OFF     (1 << 10)
#define PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED (1 << 11)
#define PX4IO_P_STATUS_FLAGS_RC_ST24        (1 << 12)
#define PX4IO_P_STATUS_FLAGS_RC_SUMD        (1 << 13)
#define PX4IO_P_STATUS_FLAGS_SAFETY_BUTTON_EVENT (1 << 14)
#define PX4IO_P_STATUS_ALARMS         3
#define PX4IO_P_STATUS_ALARMS_RC_LOST       (1 << 0)
#define PX4IO_P_STATUS_ALARMS_PWM_ERROR     (1 << 1)
#define PX4IO_P_STATUS_VSERVO         6  /* servo rail, mV */
#define PX4IO_P_STATUS_VRSSI          7  /* RSSI pin voltage */

/* Page 3: the PWM values IO is actually driving right now. Read-only. */

#define PX4IO_PAGE_SERVOS             3

/* Page 4: decoded RC. This is where the RC IN connector's channels surface,
 * already demodulated by IO (SBUS/DSM/PPM/ST24/SUMD).
 */

#define PX4IO_PAGE_RAW_RC_INPUT       4
#define PX4IO_P_RAW_RC_COUNT          0  /* number of valid channels */
#define PX4IO_P_RAW_RC_FLAGS          1
#define PX4IO_P_RAW_RC_FLAGS_FRAME_DROP (1 << 0)
#define PX4IO_P_RAW_RC_FLAGS_FAILSAFE   (1 << 1)
#define PX4IO_P_RAW_RC_FLAGS_RC_DSM11   (1 << 2)
#define PX4IO_P_RAW_RC_FLAGS_MAPPING_OK (1 << 3)
#define PX4IO_P_RAW_RC_FLAGS_RC_OK      (1 << 4)
#define PX4IO_P_RAW_RC_NRSSI          2  /* 0 = no reception, 255 = perfect */
#define PX4IO_P_RAW_RC_DATA           3
#define PX4IO_P_RAW_FRAME_COUNT       4  /* wrapping counter */
#define PX4IO_P_RAW_LOST_FRAME_COUNT  5  /* wrapping counter */
#define PX4IO_P_RAW_RC_BASE           6  /* channels start here, in us */

/* Page 6/7: ADC and PWM rate-group info. */

#define PX4IO_PAGE_RAW_ADC_INPUT      6
#define PX4IO_PAGE_PWM_INFO           7
#define PX4IO_RATE_MAP_BASE           0

/* Page 50: setup. Writable. */

#define PX4IO_PAGE_SETUP             50
#define PX4IO_P_SETUP_FEATURES        0
#define PX4IO_P_SETUP_FEATURES_SBUS1_OUT (1 << 0)
#define PX4IO_P_SETUP_FEATURES_SBUS2_OUT (1 << 1)
#define PX4IO_P_SETUP_FEATURES_ADC_RSSI  (1 << 2)
#define PX4IO_P_SETUP_ARMING          1
#define PX4IO_P_SETUP_ARMING_IO_ARM_OK            (1 << 0)
#define PX4IO_P_SETUP_ARMING_FMU_ARMED            (1 << 1)
#define PX4IO_P_SETUP_ARMING_FMU_PREARMED         (1 << 2)
#define PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM      (1 << 3)
#define PX4IO_P_SETUP_ARMING_LOCKDOWN             (1 << 4)
#define PX4IO_P_SETUP_ARMING_TERMINATION          (1 << 5)
#define PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE (1 << 6)
#define PX4IO_P_SETUP_PWM_RATES       2  /* bitmask: 0 = default rate, 1 = alt */
#define PX4IO_P_SETUP_PWM_DEFAULTRATE 3  /* Hz */
#define PX4IO_P_SETUP_PWM_ALTRATE     4  /* Hz */
#define PX4IO_P_SETUP_VSERVO_SCALE    5
#define PX4IO_P_SETUP_DSM             6
#define PX4IO_P_SETUP_SET_DEBUG       9
#define PX4IO_P_SETUP_REBOOT_BL      10
#define PX4IO_REBOOT_BL_MAGIC     14662  /* required argument for REBOOT_BL */
#define PX4IO_P_SETUP_CRC            11
#define PX4IO_P_SETUP_SAFETY_BUTTON_ACK 14
#define PX4IO_P_SETUP_SAFETY_OFF     15  /* FMU tells IO safety is off (LED) */
#define PX4IO_P_SETUP_SBUS_RATE      16
#define PX4IO_P_SETUP_THERMAL        17
#define PX4IO_P_SETUP_ENABLE_FLIGHTTERMINATION 18

/* Page 54/55/109: PWM output values, in microseconds. */

#define PX4IO_PAGE_DIRECT_PWM        54  /* what to output now  */
#define PX4IO_PAGE_FAILSAFE_PWM      55  /* output if FMU dies  */
#define PX4IO_PAGE_DISARMED_PWM     109  /* output when disarmed */

#define PX4IO_PAGE_TEST             127
#define PX4IO_P_TEST_LED              0

/* IO drives 8 PWM channels. */

#define PX4IO_SERVO_COUNT             8
#define PX4IO_RC_CHANNELS            18

/* IO clamps the PWM frame rate into this range, silently (registers.c). 50 Hz
 * suits an analog steering servo; 333/400 Hz suits a digital servo or an ESC.
 */

#define PX4IO_PWM_RATE_MIN           25
#define PX4IO_PWM_RATE_MAX          400

/* How often the daemon asks IO for RC, regardless of how fast the servo loop is
 * running. Receivers emit frames at 50-150 Hz, so polling faster just re-reads
 * the same frame - and an RC read is by far the larger transfer.
 */

#define PX4IO_RC_POLL_HZ             50

/* IO declares the FMU dead if it has not been spoken to for this long, and
 * drops the outputs to their failsafe values. Any client that wants IO to keep
 * driving the servos must poll faster than this - see PX4's
 * px4iofirmware/mixer.cpp FMU_INPUT_DROP_LIMIT_US.
 */

#define PX4IO_FMU_DROP_LIMIT_US  500000

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct px4io_packet_s
{
  uint8_t  count_code;
  uint8_t  crc;
  uint8_t  page;
  uint8_t  offset;
  uint16_t regs[PKT_MAX_REGS];
} end_packed_struct;

#endif /* __APPS_PX4IO_PX4IO_PROTOCOL_H */
