/****************************************************************************
 * apps/param/param.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * xxCar parameter system - see param.h.
 *
 * Values live in RAM (g_values) alongside a static table of definitions
 * (g_params). Persistence is a plain-text "NAME VALUE" file on the microSD so
 * it can be edited from a Linux host over USB mass storage.
 *
 * Only parameters that DIFFER from their default are written, and unknown
 * names in the file are ignored with a warning. That means a params.txt from
 * an older firmware still loads, and hand-editing it cannot brick the board.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <syslog.h>

#include "param.h"

/****************************************************************************
 * Private Data - the parameter table
 ****************************************************************************/

#define I32(v) { .i = (v) }
#define F32(v) { .f = (v) }

static const struct param_def_s g_params[] =
{
  /* ---- Serial ports -----------------------------------------------------
   * One FUNC + one BAUD per FMU connector. Function:
   *   0=disabled 1=NSH 2=MAVLink 3=GPS 4=RC_IN 5=CAL (GUI, no shell)
   *
   * NSH is not special - it is a function you assign to a port like any other,
   * and it can be moved. TELEM1 has it by default only because that is where
   * the boot console lives.
   *
   * Every connector below is an FMU UART. The RC IN connector is deliberately
   * absent: on the 6C it is wired to the PX4IO co-processor, not to the FMU, so
   * it is not a port you can assign a function to - see PX4IO_* below.
   * USART6 is likewise absent; it IS the link to PX4IO.
   */

  { "SER_TEL1_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_NSH),      I32(0), I32(5),
    "TELEM1 func (0=off 1=NSH 2=MAVLink 3=GPS 4=RC 5=CAL)", PARAM_RANGE_ENUM },
  { "SER_TEL1_BAUD", PARAM_TYPE_INT32, I32(115200), I32(1200), I32(3000000),
    "TELEM1 baud rate" },
  { "SER_TEL2_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(5),
    "TELEM2 function", PARAM_RANGE_ENUM },
  { "SER_TEL2_BAUD", PARAM_TYPE_INT32, I32(57600),  I32(1200), I32(3000000),
    "TELEM2 baud rate" },
  { "SER_TEL3_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(5),
    "TELEM3 function", PARAM_RANGE_ENUM },
  { "SER_TEL3_BAUD", PARAM_TYPE_INT32, I32(57600),  I32(1200), I32(3000000),
    "TELEM3 baud rate" },
  { "SER_GPS1_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(5),
    "GPS1 function", PARAM_RANGE_ENUM },
  { "SER_GPS1_BAUD", PARAM_TYPE_INT32, I32(38400),  I32(1200), I32(3000000),
    "GPS1 baud rate" },
  { "SER_GPS2_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(5),
    "GPS2 function", PARAM_RANGE_ENUM },
  { "SER_GPS2_BAUD", PARAM_TYPE_INT32, I32(38400),  I32(1200), I32(3000000),
    "GPS2 baud rate" },
  { "SER_DBG_FUNC",  PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(5),
    "FMU DEBUG connector function", PARAM_RANGE_ENUM },
  { "SER_DBG_BAUD",  PARAM_TYPE_INT32, I32(115200), I32(1200), I32(3000000),
    "FMU DEBUG baud rate" },

  /* The USB CDC/ACM port. No baud: the host owns the line coding on a USB
   * serial port and the device ignores it, so there is nothing to configure.
   * NSH by default - plug a cable in and you get a shell.
   */

  { "SER_USB_FUNC",  PARAM_TYPE_INT32, I32(SER_FUNC_NSH), I32(0), I32(5),
    "USB (/dev/ttyACM0) function - no baud, the host sets it",
    PARAM_RANGE_ENUM },

  /* ---- RC input ---------------------------------------------------------
   * RC_PROT applies only to a receiver wired to an FMU UART (SER_*_FUNC=4).
   * A receiver in the RC IN connector is decoded by PX4IO, which works out the
   * protocol itself and hands over channels - nothing to configure.
   *
   * PPM is a pulse train on a timer-capture pin, not a UART protocol, so it can
   * never be autodetected alongside SBUS/CRSF: select it explicitly.
   */

  { "RC_PROT", PARAM_TYPE_INT32, I32(RC_PROT_AUTO), I32(0), I32(3),
    "RC protocol on an FMU UART (0=auto SBUS/CRSF 1=SBUS 2=CRSF 3=PPM)",
    PARAM_RANGE_ENUM },

  /* ---- PX4IO co-processor ------------------------------------------------
   * Owns the RC IN connector and the 8 PWM servo rails. Not all FMUv6C boards
   * are fitted with the chip, so this is allowed to fail harmlessly.
   */

  { "PX4IO_EN",   PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Start the PX4IO client at boot (RC in + PWM out)" },
  { "PX4IO_RATE", PARAM_TYPE_INT32, I32(50), I32(5), I32(400),
    "PX4IO setpoint refresh rate (Hz)" },
  { "PX4IO_PWM_HZ", PARAM_TYPE_INT32, I32(50), I32(25), I32(400),
    "PWM frame rate the servos see (50=analog, 333/400=digital)" },

  /* ---- MAVLink ---------------------------------------------------------- */

  { "MAV_SYS_ID",  PARAM_TYPE_INT32, I32(1),  I32(1), I32(255),
    "MAVLink system ID" },
  { "MAV_COMP_ID", PARAM_TYPE_INT32, I32(1),  I32(1), I32(255),
    "MAVLink component ID" },
  { "MAV_RATE",    PARAM_TYPE_INT32, I32(10), I32(1), I32(200),
    "MAVLink stream rate (Hz)" },

  /* ---- Sensors ---------------------------------------------------------
   * The IMUs are hardware-timed off their FIFOs at 2 kHz; these set the rate
   * that is PUBLISHED to uorb subscribers.
   */

  { "SENS_IMU_RATE",  PARAM_TYPE_INT32, I32(2000), I32(50), I32(2000),
    "IMU publish rate (Hz)" },
  { "SENS_MAG_RATE",  PARAM_TYPE_INT32, I32(50),   I32(1),  I32(100),
    "Magnetometer rate (Hz)" },
  { "SENS_BARO_RATE", PARAM_TYPE_INT32, I32(10),   I32(1),  I32(50),
    "Barometer rate (Hz)" },

  /* ---- Sensor orientation ------------------------------------------------
   *
   * Values are PX4's enum Rotation, so a number copied from a PX4 config means
   * the same thing here. Only the 90-degree multiples are implemented - those
   * are exact axis permutations - and the 45-degree entries are REFUSED rather
   * than approximated. PARAM_RANGE_ENUM matters more here than almost anywhere:
   * clamping rotation 17 to 16 would not be "nearly right", it would silently
   * mount the sensor on a different face.
   *
   * Body frame = SENS_BOARD_ROT applied after SENS_IMUn_ROT. Splitting them is
   * what stops a change of vehicle mounting from having to be re-derived for
   * each IMU separately:
   *
   *   SENS_IMUn_ROT    where sensor n sits relative to the BOARD - a property
   *                    of the Pixhawk 6C, not of the vehicle
   *   SENS_BOARD_ROT   how the board is bolted into the vehicle
   *
   * IMU1 defaults to yaw 90 because that is what the hardware measures.
   * docs/imu-timestamp-audit-2026-07-26.md got, from motion correlation at
   * 0.999 across all three axes, ICM x = -BMI y, ICM y = BMI x, ICM z = BMI z -
   * exactly ROTATION_YAW_90 applied to the BMI. Confirm it after any change by
   * rotating the board and checking the two corrected streams agree; if they
   * only agree with a 90-degree swap still in them, this default is wrong.
   */

  { "SENS_BOARD_ROT", PARAM_TYPE_INT32, I32(0), I32(0), I32(37),
    "Board mounting rotation (PX4 enum Rotation; 45s unsupported)",
    PARAM_RANGE_ENUM },
  { "SENS_IMU0_ROT",  PARAM_TYPE_INT32, I32(0), I32(0), I32(37),
    "IMU0 (ICM-42688) rotation relative to the board", PARAM_RANGE_ENUM },
  { "SENS_IMU1_ROT",  PARAM_TYPE_INT32, I32(2), I32(0), I32(37),
    "IMU1 (BMI055) rotation relative to the board (2 = yaw 90, measured)",
    PARAM_RANGE_ENUM },
  { "SENS_MAG0_ROT",  PARAM_TYPE_INT32, I32(0), I32(0), I32(37),
    "IST8310 rotation relative to the board", PARAM_RANGE_ENUM },

  /* Sensor extrinsics in the body frame. The existing enum supplies the exact
   * coarse mounting rotation; calibration later supplies a small arbitrary
   * residual rotation vector. Positions are metres from the chosen vehicle
   * body origin. IMU0 is the default reference and therefore starts at zero.
   *
   * A magnetometer position is mechanical metadata, not a tumble-calibration
   * result: translation is unobservable in a uniform magnetic field.
   */

  { "SENS_IMU0_POS_X", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.5f), F32(0.5f),
    "IMU0 X position from body origin (m)" },
  { "SENS_IMU0_POS_Y", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.5f), F32(0.5f),
    "IMU0 Y position from body origin (m)" },
  { "SENS_IMU0_POS_Z", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.5f), F32(0.5f),
    "IMU0 Z position from body origin (m)" },
  { "SENS_IMU1_POS_X", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.5f), F32(0.5f),
    "IMU1 X position from body origin (m)" },
  { "SENS_IMU1_POS_Y", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.5f), F32(0.5f),
    "IMU1 Y position from body origin (m)" },
  { "SENS_IMU1_POS_Z", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.5f), F32(0.5f),
    "IMU1 Z position from body origin (m)" },
  { "SENS_MAG0_POS_X", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.5f), F32(0.5f),
    "Mag0 X position from body origin (m)" },
  { "SENS_MAG0_POS_Y", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.5f), F32(0.5f),
    "Mag0 Y position from body origin (m)" },
  { "SENS_MAG0_POS_Z", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.5f), F32(0.5f),
    "Mag0 Z position from body origin (m)" },

  { "CAL_IMU1_RVX", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.35f), F32(0.35f),
    "IMU1 fine rotation vector X after coarse mounting rotation (rad)" },
  { "CAL_IMU1_RVY", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.35f), F32(0.35f),
    "IMU1 fine rotation vector Y after coarse mounting rotation (rad)" },
  { "CAL_IMU1_RVZ", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.35f), F32(0.35f),
    "IMU1 fine rotation vector Z after coarse mounting rotation (rad)" },
  { "CAL_MAG0_RVX", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.35f), F32(0.35f),
    "Mag0 fine rotation vector X after coarse mounting rotation (rad)" },
  { "CAL_MAG0_RVY", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.35f), F32(0.35f),
    "Mag0 fine rotation vector Y after coarse mounting rotation (rad)" },
  { "CAL_MAG0_RVZ", PARAM_TYPE_FLOAT, F32(0.0f), F32(-0.35f), F32(0.35f),
    "Mag0 fine rotation vector Z after coarse mounting rotation (rad)" },
  { "CAL_IMU1_R_ERR", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "IMU1 extrinsic rotation fit RMS (rad)" },
  { "CAL_IMU1_P_ERR", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(0.5f),
    "IMU1 lever-arm fit uncertainty (m)" },
  { "CAL_MAG0_R_ERR", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "Mag0 extrinsic rotation fit RMS (rad)" },
  { "CAL_IMU1_EXT_OK", PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "IMU1 rotation and lever-arm calibration valid", PARAM_RANGE_ENUM },
  { "CAL_MAG0_EXT_OK", PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Mag0 extrinsic rotation calibration valid", PARAM_RANGE_ENUM },

  /* Which IMU feeds vehicle_acceleration / vehicle_angular_velocity. There is
   * no voting: with two sensors a disagreement cannot be resolved by majority,
   * and picking one deliberately is more honest than averaging two that may
   * not agree.
   */

  { "SENS_IMU_SEL",   PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "IMU feeding the corrected topics (0 = ICM-42688, 1 = BMI055)",
    PARAM_RANGE_ENUM },

  /* Native-rate software filters for the corrected/controller signal.  These
   * never alter sensor_accel/sensor_gyro, which remain the estimator/logger
   * source.  A zero cutoff/centre disables the corresponding filter.
   */

  { "SENS_ACC_LPF", PARAM_TYPE_FLOAT, F32(100.0f), F32(0.0f), F32(800.0f),
    "Corrected accel 2-pole low-pass cutoff (Hz, 0=off)" },
  { "SENS_GYR_LPF", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(800.0f),
    "Corrected gyro 2-pole low-pass cutoff (Hz, 0=off)" },
  { "SENS_GYR_NF_FRQ", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(800.0f),
    "Corrected gyro notch centre (Hz, 0=off)" },
  { "SENS_GYR_NF_BW", PARAM_TYPE_FLOAT, F32(20.0f), F32(1.0f), F32(400.0f),
    "Corrected gyro notch bandwidth (Hz)" },

  /* ---- EKF aiding source sets -------------------------------------------
   * Numeric values match ArduPilot EKF3 so configurations remain readable:
   * 0 none, 1 baro/compass, 2 range/GPS yaw, 3 GPS, 4 beacon,
   * 5 optical flow, 6 external navigation, 7 wheel speed, 8 GSF yaw.
   * Category-specific validation in ekf_sources.c rejects combinations that
   * have a number but no physical meaning (for example barometer VELZ).
   */

  { "EK3_SRC1_POSXY", PARAM_TYPE_INT32, I32(0), I32(0), I32(6),
    "Primary horizontal position source", PARAM_RANGE_ENUM },
  { "EK3_SRC1_VELXY", PARAM_TYPE_INT32, I32(5), I32(0), I32(7),
    "Primary horizontal velocity source", PARAM_RANGE_ENUM },
  { "EK3_SRC1_POSZ", PARAM_TYPE_INT32, I32(1), I32(0), I32(6),
    "Primary vertical position source", PARAM_RANGE_ENUM },
  { "EK3_SRC1_VELZ", PARAM_TYPE_INT32, I32(0), I32(0), I32(6),
    "Primary vertical velocity source", PARAM_RANGE_ENUM },
  { "EK3_SRC1_YAW", PARAM_TYPE_INT32, I32(1), I32(0), I32(8),
    "Primary yaw source", PARAM_RANGE_ENUM },

  { "EK3_SRC2_POSXY", PARAM_TYPE_INT32, I32(6), I32(0), I32(6),
    "Secondary horizontal position source", PARAM_RANGE_ENUM },
  { "EK3_SRC2_VELXY", PARAM_TYPE_INT32, I32(6), I32(0), I32(7),
    "Secondary horizontal velocity source", PARAM_RANGE_ENUM },
  { "EK3_SRC2_POSZ", PARAM_TYPE_INT32, I32(6), I32(0), I32(6),
    "Secondary vertical position source", PARAM_RANGE_ENUM },
  { "EK3_SRC2_VELZ", PARAM_TYPE_INT32, I32(6), I32(0), I32(6),
    "Secondary vertical velocity source", PARAM_RANGE_ENUM },
  { "EK3_SRC2_YAW", PARAM_TYPE_INT32, I32(6), I32(0), I32(8),
    "Secondary yaw source", PARAM_RANGE_ENUM },

  { "EK3_SRC3_POSXY", PARAM_TYPE_INT32, I32(0), I32(0), I32(6),
    "Tertiary horizontal position source", PARAM_RANGE_ENUM },
  { "EK3_SRC3_VELXY", PARAM_TYPE_INT32, I32(0), I32(0), I32(7),
    "Tertiary horizontal velocity source", PARAM_RANGE_ENUM },
  { "EK3_SRC3_POSZ", PARAM_TYPE_INT32, I32(0), I32(0), I32(6),
    "Tertiary vertical position source", PARAM_RANGE_ENUM },
  { "EK3_SRC3_VELZ", PARAM_TYPE_INT32, I32(0), I32(0), I32(6),
    "Tertiary vertical velocity source", PARAM_RANGE_ENUM },
  { "EK3_SRC3_YAW", PARAM_TYPE_INT32, I32(0), I32(0), I32(8),
    "Tertiary yaw source", PARAM_RANGE_ENUM },
  { "EK3_SRC_SET", PARAM_TYPE_INT32, I32(1), I32(1), I32(3),
    "Active EKF source set (1 primary, 2 secondary, 3 tertiary)",
    PARAM_RANGE_ENUM },
  { "EK3_SRC_OPTIONS", PARAM_TYPE_INT32, I32(0), I32(0), I32(3),
    "EKF sources bits: 0 fuse all velocity, 1 align extnav to flow",
    PARAM_RANGE_ENUM },

  /* ---- EKF aiding: fusion horizon and barometer -------------------------
   * EK3_DELAY_MS is how far behind real time the filter runs. Measurements
   * are fused against the state as it was when they were sampled, and the
   * published state is re-propagated forward to the present.
   *
   * It defaults to ZERO, which drains the ring every tick and reproduces the
   * pre-horizon behaviour exactly. That is deliberate: a default should
   * reproduce known-good behaviour, and it lets the timing change be proven
   * inert on hardware before any measurement starts correcting anything.
   *
   * The maximum is bounded by the IMU ring in ekf_delay.h - a larger horizon
   * would ask for samples the ring no longer holds. Gates are plain sigma;
   * ArduPilot stores its as integer sigma x 100, which is a historical
   * artefact this codebase has no reason to copy.
   */

  { "EK3_DELAY_MS", PARAM_TYPE_INT32, I32(0), I32(0), I32(100),
    "EKF fusion horizon behind real time (ms)" },
  { "EK3_ALT_M_NSE", PARAM_TYPE_FLOAT, F32(2.0f), F32(0.1f), F32(100.0f),
    "Barometer height measurement noise (m)" },
  { "EK3_ALT_I_GATE", PARAM_TYPE_FLOAT, F32(5.0f), F32(1.0f), F32(100.0f),
    "Barometer height innovation gate (sigma)" },

  /* ---- Logging (on request only; these choose WHAT gets logged) ---------- */

  { "LOG_ENABLE", PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Start logging at boot" },
  { "LOG_RATE",   PARAM_TYPE_INT32, I32(0), I32(0), I32(2000),
    "Log rate cap, Hz (0 = every sample / native)" },

  /* One switch per sensor, so a session can be exactly the data you want. Add a
   * new sensor here and in apps/logger's table; nothing else changes.
   */

  { "LOG_IMU0",   PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Log IMU0 (ICM-42688 accel+gyro)" },
  { "LOG_IMU1",   PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Log IMU1 (BMI055 accel+gyro)" },
  { "LOG_MAG",    PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Log magnetometer (IST8310)" },
  { "LOG_BARO",   PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Log barometer (MS5611)" },
  { "LOG_RC",     PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Log RC input" },
  { "LOG_FLOW",   PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Log optical flow (MTF-02)" },
  { "LOG_DIST",   PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Log distance sensor" },

  /* ---- Calibration ------------------------------------------------------
   * Written by the calibration app. Gyro offsets are rad/s, accel offsets
   * m/s^2, accel scales dimensionless, mag offsets Gauss.
   */

  { "CAL_GYRO0_XOFF", PARAM_TYPE_FLOAT, F32(0.0f), F32(-1.0f), F32(1.0f),
    "Gyro 0 X offset (rad/s)" },
  { "CAL_GYRO0_YOFF", PARAM_TYPE_FLOAT, F32(0.0f), F32(-1.0f), F32(1.0f),
    "Gyro 0 Y offset (rad/s)" },
  { "CAL_GYRO0_ZOFF", PARAM_TYPE_FLOAT, F32(0.0f), F32(-1.0f), F32(1.0f),
    "Gyro 0 Z offset (rad/s)" },
  { "CAL_ACC0_XOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-10.0f), F32(10.0f),
    "Accel 0 X offset (m/s2)" },
  { "CAL_ACC0_YOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-10.0f), F32(10.0f),
    "Accel 0 Y offset (m/s2)" },
  { "CAL_ACC0_ZOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-10.0f), F32(10.0f),
    "Accel 0 Z offset (m/s2)" },
  { "CAL_ACC0_XSCL",  PARAM_TYPE_FLOAT, F32(1.0f), F32(0.8f),  F32(1.2f),
    "Accel 0 X scale" },
  { "CAL_ACC0_YSCL",  PARAM_TYPE_FLOAT, F32(1.0f), F32(0.8f),  F32(1.2f),
    "Accel 0 Y scale" },
  { "CAL_ACC0_ZSCL",  PARAM_TYPE_FLOAT, F32(1.0f), F32(0.8f),  F32(1.2f),
    "Accel 0 Z scale" },
  /* IMU1 (BMI055). Same model and the same apply convention as IMU0:
   *
   *     corrected = (raw - OFF) * SCL
   *
   * Six-position calibration solves it in closed form per axis - with the axis
   * up and then down, OFF is the midpoint of the two readings and SCL is the
   * gravity span divided by their difference. No solver, and each axis is
   * independent of the others.
   */

  { "CAL_GYRO1_XOFF", PARAM_TYPE_FLOAT, F32(0.0f), F32(-1.0f), F32(1.0f),
    "Gyro 1 X offset (rad/s)" },
  { "CAL_GYRO1_YOFF", PARAM_TYPE_FLOAT, F32(0.0f), F32(-1.0f), F32(1.0f),
    "Gyro 1 Y offset (rad/s)" },
  { "CAL_GYRO1_ZOFF", PARAM_TYPE_FLOAT, F32(0.0f), F32(-1.0f), F32(1.0f),
    "Gyro 1 Z offset (rad/s)" },
  { "CAL_ACC1_XOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-10.0f), F32(10.0f),
    "Accel 1 X offset (m/s2)" },
  { "CAL_ACC1_YOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-10.0f), F32(10.0f),
    "Accel 1 Y offset (m/s2)" },
  { "CAL_ACC1_ZOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-10.0f), F32(10.0f),
    "Accel 1 Z offset (m/s2)" },
  { "CAL_ACC1_XSCL",  PARAM_TYPE_FLOAT, F32(1.0f), F32(0.8f),  F32(1.2f),
    "Accel 1 X scale" },
  { "CAL_ACC1_YSCL",  PARAM_TYPE_FLOAT, F32(1.0f), F32(0.8f),  F32(1.2f),
    "Accel 1 Y scale" },
  { "CAL_ACC1_ZSCL",  PARAM_TYPE_FLOAT, F32(1.0f), F32(0.8f),  F32(1.2f),
    "Accel 1 Z scale" },

  /* Whether the numbers above mean anything yet. A consumer must be able to
   * tell "calibrated, and the offsets happen to be near zero" from "never
   * calibrated", and defaults alone cannot say which.
   */

  { "CAL_ACC0_OK", PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Accel 0 calibrated (0 = raw passthrough)" },
  { "CAL_ACC1_OK", PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Accel 1 calibrated (0 = raw passthrough)" },

  /* The gyros need this flag for the same reason and more sharply: a gyro
   * bias is a small number, so "calibrated, and the bias came out near zero"
   * and "never calibrated" look identical in the offsets alone. Only a scale
   * factor would give it away, and a gyro has none to calibrate.
   */

  { "CAL_GYRO0_OK", PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Gyro 0 bias calibrated (0 = raw passthrough)" },
  { "CAL_GYRO1_OK", PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Gyro 1 bias calibrated (0 = raw passthrough)" },

  /* ---- IMU noise, from an Allan variance run ---------------------------
   * Not calibration - these do not change a reading. They are what an EKF
   * needs to know about how much to trust one: the white-noise density it
   * uses for the measurement, and the bias random walk it uses to let the
   * estimated bias move.
   *
   *   ND  noise density,      sigma(tau) = ND / sqrt(tau)
   *   RW  bias random walk,   sigma(tau) = RW * sqrt(tau / 3)
   *   BI  bias instability,   min of the curve / 0.664
   *
   * Bounds are wide because these span orders of magnitude between a MEMS
   * part and a good one, and a bound that rejects a real measurement is worse
   * than one that admits a bad one - the residual and the plot are what say
   * whether a run was any good.
   */

  { "IMU0_ACC_ND", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "Accel 0 noise density (m/s^2/sqrt(Hz))" },
  { "IMU0_ACC_RW", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "Accel 0 bias random walk (m/s^3*sqrt(s))" },
  { "IMU0_ACC_BI", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "Accel 0 bias instability (m/s^2)" },
  { "IMU0_GYR_ND", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "Gyro 0 noise density (rad/s/sqrt(Hz))" },
  { "IMU0_GYR_RW", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "Gyro 0 bias random walk (rad/s^2*sqrt(s))" },
  { "IMU0_GYR_BI", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "Gyro 0 bias instability (rad/s)" },
  { "IMU1_ACC_ND", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "Accel 1 noise density (m/s^2/sqrt(Hz))" },
  { "IMU1_ACC_RW", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "Accel 1 bias random walk (m/s^3*sqrt(s))" },
  { "IMU1_ACC_BI", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "Accel 1 bias instability (m/s^2)" },
  { "IMU1_GYR_ND", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "Gyro 1 noise density (rad/s/sqrt(Hz))" },
  { "IMU1_GYR_RW", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "Gyro 1 bias random walk (rad/s^2*sqrt(s))" },
  { "IMU1_GYR_BI", PARAM_TYPE_FLOAT, F32(0.0f), F32(0.0f), F32(1.0f),
    "Gyro 1 bias instability (rad/s)" },

  { "CAL_MAG0_XOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-2.0f), F32(2.0f),
    "Mag 0 X offset (Gauss)" },
  { "CAL_MAG0_YOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-2.0f), F32(2.0f),
    "Mag 0 Y offset (Gauss)" },
  { "CAL_MAG0_ZOFF",  PARAM_TYPE_FLOAT, F32(0.0f), F32(-2.0f), F32(2.0f),
    "Mag 0 Z offset (Gauss)" },
  { "CAL_MAG0_XX", PARAM_TYPE_FLOAT, F32(1.0f), F32(0.25f), F32(4.0f),
    "Mag 0 soft-iron matrix XX" },
  { "CAL_MAG0_YY", PARAM_TYPE_FLOAT, F32(1.0f), F32(0.25f), F32(4.0f),
    "Mag 0 soft-iron matrix YY" },
  { "CAL_MAG0_ZZ", PARAM_TYPE_FLOAT, F32(1.0f), F32(0.25f), F32(4.0f),
    "Mag 0 soft-iron matrix ZZ" },
  { "CAL_MAG0_XY", PARAM_TYPE_FLOAT, F32(0.0f), F32(-2.0f), F32(2.0f),
    "Mag 0 soft-iron matrix XY" },
  { "CAL_MAG0_XZ", PARAM_TYPE_FLOAT, F32(0.0f), F32(-2.0f), F32(2.0f),
    "Mag 0 soft-iron matrix XZ" },
  { "CAL_MAG0_YZ", PARAM_TYPE_FLOAT, F32(0.0f), F32(-2.0f), F32(2.0f),
    "Mag 0 soft-iron matrix YZ" },
  { "CAL_MAG0_FIELD", PARAM_TYPE_FLOAT, F32(0.45f), F32(0.15f), F32(0.8f),
    "Mag 0 calibrated field strength (Gauss)" },
  { "CAL_MAG0_OK", PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Mag 0 full ellipsoid calibrated" },
};

#define PARAM_COUNT ((int)(sizeof(g_params) / sizeof(g_params[0])))

/* Live values, and whether the SD file has been read yet. */

static union param_value_u g_values[PARAM_COUNT];
static bool                g_initialised;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void param_apply_defaults(void)
{
  int i;

  for (i = 0; i < PARAM_COUNT; i++)
    {
      g_values[i] = g_params[i].def;
    }
}

static void param_ensure_init(void)
{
  if (!g_initialised)
    {
      param_init();
    }
}

/* Bring a value into the definition's range, or report that it cannot be.
 *
 * A selector (PARAM_RANGE_ENUM) is never coerced into a neighbouring value -
 * see the rationale on enum param_range_e. It is reported REJECTED and left
 * untouched, so the caller can decide: param_load falls back to the default
 * because it must boot with something, param_set_* refuses because the caller
 * can simply be told it was wrong.
 */

enum param_fix_e
{
  PARAM_FIX_NONE = 0,      /* value was already in range */
  PARAM_FIX_CLAMPED,       /* scalar, coerced to the nearest bound */
  PARAM_FIX_REJECTED       /* selector, not a valid choice; *v is unchanged */
};

static enum param_fix_e param_clamp(int idx, FAR union param_value_u *v)
{
  FAR const struct param_def_s *d = &g_params[idx];

  /* Reject NaN and infinity before anything else.
   *
   * Every comparison against NaN is false, so a NaN sails through the range
   * test below as though it were in range and gets stored - then written to
   * params.txt as "nan" and read straight back on the next boot, permanently.
   * (Infinity is caught by the bounds, NaN is not.) It arrives more easily
   * than it looks: an Allan run too short to reach its curve minimum yields a
   * genuine NaN for rate random walk, and that number is on its way to a
   * parameter.
   */

  if (d->type == PARAM_TYPE_FLOAT && !isfinite(v->f))
    {
      return PARAM_FIX_REJECTED;
    }

  if (d->range == PARAM_RANGE_ENUM)
    {
      /* Selectors are always INT32; a float selector is meaningless. */

      if (v->i < d->min.i || v->i > d->max.i)
        {
          return PARAM_FIX_REJECTED;
        }

      return PARAM_FIX_NONE;
    }

  if (d->type == PARAM_TYPE_INT32)
    {
      if (v->i < d->min.i)
        {
          v->i = d->min.i;
          return PARAM_FIX_CLAMPED;
        }

      if (v->i > d->max.i)
        {
          v->i = d->max.i;
          return PARAM_FIX_CLAMPED;
        }
    }
  else
    {
      if (v->f < d->min.f)
        {
          v->f = d->min.f;
          return PARAM_FIX_CLAMPED;
        }

      if (v->f > d->max.f)
        {
          v->f = d->max.f;
          return PARAM_FIX_CLAMPED;
        }
    }

  return PARAM_FIX_NONE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int param_find(FAR const char *name)
{
  int i;

  for (i = 0; i < PARAM_COUNT; i++)
    {
      if (strcmp(g_params[i].name, name) == 0)
        {
          return i;
        }
    }

  return -ENOENT;
}

int param_init(void)
{
  param_apply_defaults();
  g_initialised = true;

  /* Missing file is normal on a fresh card - defaults stand. */

  param_load();
  return OK;
}

int param_load(void)
{
  FAR FILE *fp;
  char line[96];
  int loaded = 0;
  int unknown = 0;

  if (!g_initialised)
    {
      param_apply_defaults();
      g_initialised = true;
    }

  fp = fopen(PARAM_FILE, "r");
  if (fp == NULL)
    {
      /* Nothing under the live name. Either this is a fresh card, or power was
       * lost inside param_save()'s unlink/rename commit window. Tell those
       * apart by looking for the staging file: it exists only after fsync()
       * succeeded, so if it is here it is a complete set of values and it is
       * NEWER than whatever used to be at PARAM_FILE. Adopt it.
       */

      if (rename(PARAM_TMPFILE, PARAM_FILE) != 0)
        {
          return -ENOENT;      /* no file yet: defaults are in effect */
        }

      fp = fopen(PARAM_FILE, "r");
      if (fp == NULL)
        {
          return -ENOENT;
        }

      syslog(LOG_WARNING,
             "[param] recovered %s from %s - a save was interrupted\n",
             PARAM_FILE, PARAM_TMPFILE);
    }

  while (fgets(line, sizeof(line), fp) != NULL)
    {
      char name[PARAM_NAME_MAX + 1];
      char valstr[32];
      union param_value_u v;
      int idx;

      /* Skip comments and blanks */

      if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
        {
          continue;
        }

      if (sscanf(line, "%16s %31s", name, valstr) != 2)
        {
          continue;
        }

      idx = param_find(name);
      if (idx < 0)
        {
          /* An old or hand-typed name. Ignore it rather than fail the whole
           * file - a stale params.txt must never brick the board.
           */

          syslog(LOG_WARNING, "[param] unknown '%s' in %s (ignored)\n",
                 name, PARAM_FILE);
          unknown++;
          continue;
        }

      if (g_params[idx].type == PARAM_TYPE_INT32)
        {
          v.i = strtol(valstr, NULL, 0);
        }
      else
        {
          v.f = strtof(valstr, NULL);
        }

      switch (param_clamp(idx, &v))
        {
          case PARAM_FIX_CLAMPED:
            syslog(LOG_WARNING, "[param] %s out of range, clamped\n", name);
            break;

          case PARAM_FIX_REJECTED:

            /* A selector we do not recognise - typically params.txt outliving
             * the firmware that understood the value. Boot with the default
             * rather than whichever unrelated function happens to sit at the
             * nearest bound, and say so loudly: the operator's saved intent has
             * been dropped, and they need to know which parameter to re-set.
             */

            v = g_params[idx].def;
            syslog(LOG_ERR,
                   "[param] %s: '%s' is not a valid choice, using default %"
                   PRId32 "\n", name, valstr, v.i);
            break;

          default:
            break;
        }

      g_values[idx] = v;
      loaded++;
    }

  fclose(fp);
  syslog(LOG_INFO, "[param] loaded %d from %s (%d unknown)\n",
         loaded, PARAM_FILE, unknown);
  return loaded;
}

/* Emit the whole file into an already-open stream.
 *
 * Returns the number of parameters written, or a negated errno. Every
 * fprintf() result is checked: on a full card the first failing write is the
 * only warning there will be, and a save that ignores it reports success over
 * a truncated file.
 */

static int param_write_body(FAR FILE *fp, FAR size_t *bytes)
{
  int written = 0;
  int total = 0;
  int n;
  int i;

  n = fprintf(fp,
              "# xxCar parameters\n"
              "# Edit and save, then run 'param load' or reboot.\n"
              "# Only values that differ from the default are listed.\n");
  if (n < 0)
    {
      return -EIO;
    }

  total += n;

  for (i = 0; i < PARAM_COUNT; i++)
    {
      if (!param_is_modified(i))
        {
          continue;
        }

      if (g_params[i].type == PARAM_TYPE_INT32)
        {
          n = fprintf(fp, "%s %" PRId32 "\n",
                      g_params[i].name, g_values[i].i);
        }
      else
        {
          n = fprintf(fp, "%s %.6f\n",
                      g_params[i].name, (double)g_values[i].f);
        }

      if (n < 0)
        {
          return -EIO;
        }

      total += n;
      written++;
    }

  *bytes = (size_t)total;
  return written;
}

/* Save every modified parameter, or leave the previous file exactly as it was.
 *
 * The old version opened PARAM_FILE with "w". That truncates the live file
 * before a single new byte is known to be storable, ignored every fprintf()
 * result, never flushed, and returned a count of parameters VISITED. A full
 * card, a pulled card, or the card being exported over USB mid-write therefore
 * produced an empty or half-written params.txt while calibration reported that
 * it had saved - the operator's only copy of a calibration destroyed by the act
 * of storing it.
 *
 * So: build the new contents in PARAM_TMPFILE, prove every write landed and is
 * on the medium, and only then replace the live file.
 *
 * NuttX's FAT has no atomic replace - fat_rename() returns -EEXIST rather than
 * overwriting (fs/fat/fs_fat32.c) - so the commit is unlink-then-rename, and
 * there is a window where neither name is params.txt. That window is covered by
 * param_load() below, which adopts a leftover PARAM_TMPFILE. It is a complete
 * file by construction: it is only ever renamed after fsync() succeeded.
 */

int param_save(void)
{
  FAR FILE *fp;
  size_t bytes = 0;
  int written;
  int err;

  param_ensure_init();

  fp = fopen(PARAM_TMPFILE, "w");
  if (fp == NULL)
    {
      /* No card, or it is currently exported to the USB host */

      return -errno;
    }

  written = param_write_body(fp, &bytes);

  if (written >= 0 && fflush(fp) != 0)
    {
      written = -EIO;
    }

  /* fflush() only empties the stdio buffer into the file. fsync() is what puts
   * it on the card, and it is the call that reports a write the FAT layer
   * could not complete.
   */

  if (written >= 0 && fsync(fileno(fp)) != 0)
    {
      written = -errno;
    }

  /* A close can fail with data still unwritten, so its result matters as much
   * as any other. Take it before deciding the file is good.
   */

  if (fclose(fp) != 0 && written >= 0)
    {
      written = -EIO;
    }

  if (written < 0)
    {
      err = written;
      unlink(PARAM_TMPFILE);
      syslog(LOG_ERR, "[param] save failed: %d; %s is unchanged\n",
             err, PARAM_FILE);
      return err;
    }

  /* Commit. ENOENT from the unlink just means there was nothing to replace. */

  if (unlink(PARAM_FILE) != 0 && errno != ENOENT)
    {
      err = -errno;
      unlink(PARAM_TMPFILE);
      syslog(LOG_ERR, "[param] could not replace %s: %d\n", PARAM_FILE, err);
      return err;
    }

  if (rename(PARAM_TMPFILE, PARAM_FILE) != 0)
    {
      err = -errno;

      /* Deliberately keep PARAM_TMPFILE: it holds the only copy of the new
       * values, and param_load() will adopt it on the next boot.
       */

      syslog(LOG_ERR,
             "[param] wrote %s but could not rename it to %s: %d - "
             "the values are safe, run 'param save' again\n",
             PARAM_TMPFILE, PARAM_FILE, err);
      return err;
    }

  syslog(LOG_INFO, "[param] saved %d changed (%zu bytes) to %s\n",
         written, bytes, PARAM_FILE);
  return written;
}

int param_reset(void)
{
  param_apply_defaults();
  g_initialised = true;
  return OK;
}

int param_get_i32(FAR const char *name, FAR int32_t *value)
{
  int idx;

  param_ensure_init();

  idx = param_find(name);
  if (idx < 0)
    {
      return idx;
    }

  if (g_params[idx].type != PARAM_TYPE_INT32)
    {
      return -EINVAL;
    }

  *value = g_values[idx].i;
  return OK;
}

int param_get_f32(FAR const char *name, FAR float *value)
{
  int idx;

  param_ensure_init();

  idx = param_find(name);
  if (idx < 0)
    {
      return idx;
    }

  if (g_params[idx].type != PARAM_TYPE_FLOAT)
    {
      return -EINVAL;
    }

  *value = g_values[idx].f;
  return OK;
}

int param_set_i32(FAR const char *name, int32_t value)
{
  union param_value_u v;
  int idx;

  param_ensure_init();

  idx = param_find(name);
  if (idx < 0)
    {
      return idx;
    }

  if (g_params[idx].type != PARAM_TYPE_INT32)
    {
      return -EINVAL;
    }

  v.i = value;
  switch (param_clamp(idx, &v))
    {
      case PARAM_FIX_REJECTED:

        /* Not a valid choice for this selector. Leave the parameter alone -
         * quietly substituting a different function is how an RC decoder ends
         * up on the USB port.
         */

        return -ERANGE;

      case PARAM_FIX_CLAMPED:
        g_values[idx] = v;
        return -ERANGE;          /* clamped: applied, but tell the caller */

      default:
        break;
    }

  g_values[idx] = v;
  return OK;
}

int param_set_f32(FAR const char *name, float value)
{
  union param_value_u v;
  int idx;

  param_ensure_init();

  idx = param_find(name);
  if (idx < 0)
    {
      return idx;
    }

  if (g_params[idx].type != PARAM_TYPE_FLOAT)
    {
      return -EINVAL;
    }

  v.f = value;
  switch (param_clamp(idx, &v))
    {
      case PARAM_FIX_REJECTED:

        /* Non-finite. Leave the stored value alone: a NaN written here would
         * outlive the mistake in params.txt.
         */

        return -ERANGE;

      case PARAM_FIX_CLAMPED:
        g_values[idx] = v;
        return -ERANGE;

      default:
        break;
    }

  g_values[idx] = v;
  return OK;
}

int32_t param_i32(FAR const char *name)
{
  int32_t v = 0;
  int idx;

  if (param_get_i32(name, &v) < 0)
    {
      idx = param_find(name);
      return idx >= 0 ? g_params[idx].def.i : 0;
    }

  return v;
}

float param_f32(FAR const char *name)
{
  float v = 0.0f;
  int idx;

  if (param_get_f32(name, &v) < 0)
    {
      idx = param_find(name);
      return idx >= 0 ? g_params[idx].def.f : 0.0f;
    }

  return v;
}

int param_count(void)
{
  return PARAM_COUNT;
}

FAR const struct param_def_s *param_def(int index)
{
  if (index < 0 || index >= PARAM_COUNT)
    {
      return NULL;
    }

  return &g_params[index];
}

bool param_is_modified(int index)
{
  if (index < 0 || index >= PARAM_COUNT)
    {
      return false;
    }

  param_ensure_init();

  if (g_params[index].type == PARAM_TYPE_INT32)
    {
      return g_values[index].i != g_params[index].def.i;
    }

  return fabsf(g_values[index].f - g_params[index].def.f) > 1e-9f;
}

int param_value_str(int index, FAR char *buf, size_t len)
{
  if (index < 0 || index >= PARAM_COUNT)
    {
      return -EINVAL;
    }

  param_ensure_init();

  if (g_params[index].type == PARAM_TYPE_INT32)
    {
      return snprintf(buf, len, "%" PRId32, g_values[index].i);
    }

  return snprintf(buf, len, "%.6f", (double)g_values[index].f);
}
