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

  { "SER_TEL1_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_NSH),      I32(0), I32(6),
    "TELEM1 func (0=off 1=NSH 2=MAVLink 3=GPS 4=RC 5=CAL 6=COMP)", PARAM_RANGE_ENUM },
  { "SER_TEL1_BAUD", PARAM_TYPE_INT32, I32(115200), I32(1200), I32(3000000),
    "TELEM1 baud rate" },
  { "SER_TEL2_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_COMPANION), I32(0), I32(6),
    "TELEM2 function", PARAM_RANGE_ENUM },
  { "SER_TEL2_BAUD", PARAM_TYPE_INT32, I32(921600),  I32(1200), I32(3000000),
    "TELEM2 baud rate" },
  { "SER_TEL3_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(6),
    "TELEM3 function", PARAM_RANGE_ENUM },
  { "SER_TEL3_BAUD", PARAM_TYPE_INT32, I32(57600),  I32(1200), I32(3000000),
    "TELEM3 baud rate" },
  { "SER_GPS1_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(6),
    "GPS1 function", PARAM_RANGE_ENUM },
  { "SER_GPS1_BAUD", PARAM_TYPE_INT32, I32(38400),  I32(1200), I32(3000000),
    "GPS1 baud rate" },
  { "SER_GPS2_FUNC", PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(6),
    "GPS2 function", PARAM_RANGE_ENUM },
  { "SER_GPS2_BAUD", PARAM_TYPE_INT32, I32(38400),  I32(1200), I32(3000000),
    "GPS2 baud rate" },
  { "SER_DBG_FUNC",  PARAM_TYPE_INT32, I32(SER_FUNC_DISABLED), I32(0), I32(6),
    "FMU DEBUG connector function", PARAM_RANGE_ENUM },
  { "SER_DBG_BAUD",  PARAM_TYPE_INT32, I32(115200), I32(1200), I32(3000000),
    "FMU DEBUG baud rate" },

  /* The USB CDC/ACM port. No baud: the host owns the line coding on a USB
   * serial port and the device ignores it, so there is nothing to configure.
   * NSH by default - plug a cable in and you get a shell.
   */

  { "SER_USB_FUNC",  PARAM_TYPE_INT32, I32(SER_FUNC_NSH), I32(0), I32(6),
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

  /* RC safety router. Channel maps are one-based, matching transmitter and
   * `rc status` labels. The arm channel deliberately starts on channel 7:
   * channels 5 and 6 are source and motor-mode selection respectively.
   */

  { "CTRL_ROUTER_EN", PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Start RC/auto safety router at boot", PARAM_RANGE_ENUM },
  { "RC_MAP_STEERING", PARAM_TYPE_INT32, I32(1), I32(1), I32(18),
    "RC steering channel (one-based)", PARAM_RANGE_ENUM },
  { "RC_MAP_THROTTLE", PARAM_TYPE_INT32, I32(3), I32(1), I32(18),
    "RC throttle channel (one-based)", PARAM_RANGE_ENUM },
  { "RC_MAP_SOURCE", PARAM_TYPE_INT32, I32(5), I32(1), I32(18),
    "RC manual/auto switch channel", PARAM_RANGE_ENUM },
  { "RC_MAP_MODE", PARAM_TYPE_INT32, I32(6), I32(1), I32(18),
    "RC duty/current momentary toggle channel", PARAM_RANGE_ENUM },
  { "RC_MAP_ARM", PARAM_TYPE_INT32, I32(7), I32(1), I32(18),
    "RC arm/disarm switch channel", PARAM_RANGE_ENUM },

  { "RC_ST_MIN", PARAM_TYPE_INT32, I32(1000), I32(750), I32(2250),
    "Steering PWM at -1 (us)" },
  { "RC_ST_TRIM", PARAM_TYPE_INT32, I32(1500), I32(750), I32(2250),
    "Steering centre PWM (us)" },
  { "RC_ST_MAX", PARAM_TYPE_INT32, I32(2000), I32(750), I32(2250),
    "Steering PWM at +1 (us)" },
  { "RC_ST_DZ", PARAM_TYPE_INT32, I32(30), I32(0), I32(400),
    "Steering deadzone about trim (us)" },
  { "RC_THR_MIN", PARAM_TYPE_INT32, I32(1000), I32(750), I32(2250),
    "Throttle PWM at -1 (us)" },
  { "RC_THR_TRIM", PARAM_TYPE_INT32, I32(1500), I32(750), I32(2250),
    "Throttle neutral PWM (us)" },
  { "RC_THR_MAX", PARAM_TYPE_INT32, I32(2000), I32(750), I32(2250),
    "Throttle PWM at +1 (us)" },
  { "RC_THR_DZ", PARAM_TYPE_INT32, I32(30), I32(0), I32(400),
    "Throttle deadzone about trim (us)" },
  { "RC_SW_LOW", PARAM_TYPE_INT32, I32(1300), I32(750), I32(2250),
    "Switch low threshold (us)" },
  { "RC_SW_HIGH", PARAM_TYPE_INT32, I32(1700), I32(750), I32(2250),
    "Switch high threshold (us)" },
  { "RC_INPUT_TO_MS", PARAM_TYPE_INT32, I32(150), I32(50), I32(2000),
    "RC frame timeout in router (ms)" },
  { "AUTO_CMD_TO_MS", PARAM_TYPE_INT32, I32(200), I32(20), I32(5000),
    "Automatic control command timeout (ms)" },
  { "RC_ARM_MAX", PARAM_TYPE_FLOAT, F32(0.05f), F32(0.0f), F32(0.25f),
    "Maximum normalized motor demand when arming" },

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
   * THE VEHICLE CONVENTION IS +x FORWARD, +y LEFT, +z UP.
   *
   * Not PX4's FRD. It is what the parts themselves report: PX4's own drivers
   * describe both the ICM-42688-P and the BMI055 as "+x forward, +y left, +z
   * up" and then flip y and z to reach ITS board frame. This project does not
   * make that flip, so with SENS_IMU0_ROT=0 the vehicle frame IS the ICM's
   * own frame, and everything else is brought to match it.
   *
   * The estimator agrees, which is worth checking rather than assuming: at
   * rest a level FLU accelerometer reads +g on z, and ekf_core.c removes
   * gravity as `nav_delta_velocity[2] -= EKF_GRAVITY * dt`. Those cancel only
   * if the navigation frame's z is UP too. It is ENU - east, north, up - the ROS
   * REP-103 convention, and yaw is positive counter-clockwise seen from
   * above with ZERO AT EAST.
   *
   * A consequence worth knowing before comparing anything against PX4 or
   * ArduPilot: roll and yaw run in the opposite sense to FRD, because a
   * roll-180 conjugation negates both. That is also why SENS_IMU1_ROT is yaw
   * 90 here where PX4's fmu-v6c uses yaw 270 for the same physical parts -
   * the two are the same relationship expressed in mirrored frames.
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
  /* NONE, matching PX4 and ArduPilot, which both declare the 6C's internal
   * IST8310 as ROTATION_NONE. The part IS mounted square with the board.
   *
   * Its left-handedness against our +x fwd, +y left, +z up convention is a
   * separate matter and is handled in apps/sensors/mag_frame.c, because a
   * reflection is not a rotation and cannot be spelled with this enum.
   */

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
   *
   * THE SPLIT IS DELIBERATE, and it is ArduPilot's: the estimator runs on
   * the raw calibrated stream while the rate loop runs on a filtered one.
   *
   * imu_delta subscribes to sensor_gyro directly and applies its own
   * calibration, so nothing here reaches the EKF - a filter in the
   * estimator's path would add phase lag to the very signal the attitude
   * solution integrates, and the filter's own delayed-fusion horizon already
   * handles what the filter would be there to fix. The corrected topics feed
   * control and the companion's VEHICLE_STATE twist, where lag costs less
   * than noise does.
   *
   * Both default to 100 Hz, chosen for control bandwidth. That is well
   * inside the 2 kHz the filters are designed at, but note the consequence
   * downstream: the companion downlink samples these topics at 200 Hz, and
   * two poles at 100 Hz are only 3 dB down at that 100 Hz Nyquist, so the
   * VEHICLE_STATE twist and accel carry some folded content. That is a
   * deliberate bandwidth-versus-aliasing trade - drop the cutoffs to around
   * 30 Hz if those channels look noisy and the bandwidth is not needed.
   */

  { "SENS_ACC_LPF", PARAM_TYPE_FLOAT, F32(100.0f), F32(0.0f), F32(800.0f),
    "Corrected accel 2-pole low-pass cutoff (Hz, 0=off)" },
  { "SENS_GYR_LPF", PARAM_TYPE_FLOAT, F32(100.0f), F32(0.0f), F32(800.0f),
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

  /* ---- EKF aiding: magnetic heading -------------------------------------
   * EK3_MAG_DEC is the angle from TRUE north to MAGNETIC north, positive
   * east. There is no GPS to look it up from, so it is entered per location.
   * Left at zero the estimator produces magnetic heading, not true heading -
   * adequate if consumers need only a consistent absolute reference, wrong
   * if they need north.
   *
   * EK3_YAW_M_NSE takes ArduPilot's default of 0.5 rad, far looser than a
   * compass's actual accuracy. That is intentional on their part and worth
   * keeping: a loose measurement noise makes the filter lean on the gyro
   * between updates and limits how hard one disturbed reading can pull the
   * heading. It is the first thing to tune once the path is proven.
   */

  { "EK3_MAG_DEC", PARAM_TYPE_FLOAT, F32(0.0f), F32(-180.0f), F32(180.0f),
    "Magnetic declination, true to magnetic, +east (deg)" },
  { "EK3_YAW_M_NSE", PARAM_TYPE_FLOAT, F32(0.5f), F32(0.01f), F32(1.5f),
    "Yaw measurement noise (rad)" },
  { "EK3_YAW_I_GATE", PARAM_TYPE_FLOAT, F32(5.0f), F32(1.0f), F32(100.0f),
    "Yaw innovation gate (sigma)" },

  /* ---- External navigation ----------------------------------------------
   * EK3_EXT_M_NSE and EK3_EXT_YAW_NSE are FLOORS, not defaults. The fused
   * noise is max(what the source reported, this) - which is what ArduPilot
   * does with posErr. A source claiming millimetre accuracy must not be able
   * to talk the filter into trusting it more than the operator configured.
   */

  { "EXT_TX_RATE", PARAM_TYPE_INT32, I32(200), I32(16), I32(1000),
    "Companion pose transmit rate (Hz)" },
  /* ---- What starts at boot --------------------------------------------
   *
   * Every one of these is an OPTIONAL boot failure: a subsystem that will
   * not start is a serious problem, but wedging the boot is a worse one -
   * the board must still reach a shell, which is where it gets diagnosed.
   *
   * SENS_EN and SENS_AUX_EN were the gap. Nothing started either daemon at
   * boot, so a freshly booted board published no vehicle_accel, vehicle_gyro,
   * vehicle_mag or vehicle_baro at all - the estimator ran on the IMU alone
   * and the companion's VEHICLE_STATE reported gyro and accel permanently
   * absent. imu_delta was unaffected because it reads the NuttX driver
   * topics directly, which is why this stayed hidden.
   */

  { "SENS_EN", PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Start the IMU sensor daemon at boot" },
  { "SENS_AUX_EN", PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Start the magnetometer and barometer daemon at boot" },
  { "IMU_DELTA_EN", PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Start the IMU delta integrator at boot" },
  { "EKF3_EN", PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Start the estimator at boot" },

  /* The attitude monitor lanes.
   *
   * EKF1 is the primary: primary IMU plus every aiding source. EKF2 runs the
   * SECONDARY IMU, attitude only. EKF3 runs the PRIMARY IMU, attitude only.
   *
   * The third lane is what makes this diagnostic rather than merely a
   * disagreement detector. EKF3 shares EKF1's IMU exactly, so an attitude
   * difference between them cannot be the sensor - it is the aiding
   * corrupting EKF1. EKF2 against EKF3 is the same IMU question with the
   * aiding removed from both, so that difference IS the sensor.
   *
   * EKF_MON_ACT is OFF: the lanes run, the comparison is reported, and
   * nothing acts on it yet. Turn it on once the numbers have been watched on
   * a real vehicle and the threshold means something.
   */

  { "EKF_MON_EN", PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Run the attitude monitor lanes" },
  { "EKF_MON_ACT", PARAM_TYPE_INT32, I32(0), I32(0), I32(1),
    "Act on monitor faults (0 = report only)" },
  { "EKF_MON_TILT", PARAM_TYPE_FLOAT, F32(0.15f), F32(0.01f), F32(1.5f),
    "Tilt disagreement calling a lane faulted (rad)" },
  { "EKF_MON_MS", PARAM_TYPE_INT32, I32(2000), I32(100), I32(30000),
    "How long disagreement must persist (ms)" },

  { "COMP_EN", PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Start the companion serial link at boot" },
  /* Largest PPS residual that will be believed as a refinement.
   *
   * A timesync burst already puts the UTC offset within a few milliseconds,
   * so a residual far outside that is not imprecision - it is the pulse
   * arriving somewhere other than the second boundary it claims to mark. A
   * PPS generated from userspace on a non-realtime host does exactly that,
   * late by however long the scheduler took.
   *
   * Applying such a correction drags the board's clock away from UTC by the
   * pulse's own latency, which shows up as a solution time lagging the host
   * by tens of milliseconds - worse than no PPS at all. Beyond this bound
   * the edge is refused and the timesync offset stands.
   *
   * 10 ms: comfortably above timesync round-trip error, far below the
   * scheduling latency of a userspace pulse.
   */

  { "PPS_MAX_COR_US", PARAM_TYPE_INT32, I32(10000), I32(100), I32(500000),
    "Largest PPS correction believed as a refinement (us)" },

  { "PPS_EN", PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Capture Jetson PPS on TELEM2 CTS at boot" },
  { "EK3_EXT_M_NSE", PARAM_TYPE_FLOAT, F32(0.10f), F32(0.01f), F32(10.0f),
    "External position measurement noise floor (m)" },
  { "EK3_EXT_I_GATE", PARAM_TYPE_FLOAT, F32(5.0f), F32(1.0f), F32(100.0f),
    "External position innovation gate (sigma)" },
  { "EK3_EXT_YAW_NSE", PARAM_TYPE_FLOAT, F32(0.05f), F32(0.01f), F32(1.5f),
    "External yaw measurement noise floor (rad)" },
  /* Vertical bound, metres either side of where the filter aligned.
   *
   * A ground vehicle does not leave the ground, and saying so gives
   * divergence one less place to hide: a bad accel bias or a drifting baro
   * reference otherwise integrates into height unopposed.
   *
   * 50 m by default, which no road journey will reach in the local frame but
   * a runaway crosses in seconds - so it guards against divergence without
   * ever arguing with real terrain. Tighten it to a metre or so for flat or
   * indoor running and it becomes a genuine height lock. Zero disables it.
   *
   * It is a bound, not a measurement: when it engages, the vertical variance
   * is floored so the solution stops claiming a confidence it does not have.
   */

  { "EK3_HGT_LIM", PARAM_TYPE_FLOAT, F32(50.0f), F32(0.0f), F32(1000.0f),
    "Height bound above the alignment point (m, 0 = off)" },

  /* Horizontal bound applied while NOTHING is aiding position.
   *
   * Position is not observable from an IMU. Left free, the strapdown
   * integrates accelerometer error twice and leaves quadratically - metres
   * within seconds on a car, and it never comes back. It also gives the
   * filter a way to explain the excursion: it learns an accelerometer bias
   * to match, and that bias corrupts the gravity reference and takes
   * attitude with it.
   *
   * So while unaided the estimate is bounded to where the last fix left it
   * and accel bias learning is frozen. The result is wrong - the vehicle is
   * moving - but bounded, declared through POSITION_HORIZ, and recoverable:
   * the next valid fix is adopted outright rather than argued with.
   *
   * 2 m by default. Larger buys nothing, because the answer is already
   * known to be wrong; the point is that it stays finite. Zero restores
   * free dead reckoning.
   */

  { "EK3_POSHOLD_M", PARAM_TYPE_FLOAT, F32(2.0f), F32(0.0f), F32(1000.0f),
    "Position bound while unaided (m, 0 = free dead reckoning)" },

  /* Bias bounds. Each may be TIGHTENED from here but never loosened past
   * the ceiling compiled into ekf_core.h - beyond that a "bias" is large
   * enough to be hiding a real acceleration, and a parameter is not
   * evidence that it is not.
   *
   * ArduPilot's EK3_ABIAS_LIM defaults to 1.0 and PX4's acc_bias_lim to 0.4.
   * 0.4 is the better default for a ground vehicle: an accelerometer that
   * genuinely needs more than 0.4 m/s^2 of correction after calibration is
   * telling you the calibration is wrong, and letting the filter absorb it
   * hides that.
   *
   * A bias sitting exactly ON the limit is a fault report, not a converged
   * estimate - see the "AT LIMIT" marker in `ekf3 status`.
   */

  { "EK3_ABIAS_LIM", PARAM_TYPE_FLOAT, F32(0.4f), F32(0.05f), F32(1.0f),
    "Accelerometer bias bound (m/s^2)" },
  { "EK3_GBIAS_LIM", PARAM_TYPE_FLOAT, F32(0.1f), F32(0.01f), F32(0.1f),
    "Gyro bias bound (rad/s)" },

  /* Bias learning switches.
   *
   * Turning one OFF freezes that bias where it stands - it does not zero it.
   * A converged value is worth keeping; a bad one is cleared by `ekf3 reset`,
   * which is an explicit action rather than a side effect of editing a
   * parameter. So to pin a bias at zero: set the switch off, then reset.
   *
   * Both default ON. Freezing accelerometer bias is a reasonable response to
   * an estimate that will not settle - a saturated bias is worse than none,
   * because the filter is using it to explain something that is not bias -
   * but it is a diagnosis aid, not a fix: the underlying cause is usually a
   * calibration that needs redoing.
   *
   * Gyro bias is the one to leave alone. It is genuinely observable from
   * gravity, it really does drift with temperature, and freezing it costs
   * heading stability for nothing.
   */

  { "EK3_ABIAS_EN", PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Learn accelerometer bias (0 = frozen)" },
  { "EK3_GBIAS_EN", PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Learn gyro bias (0 = frozen)" },

  { "EK3_EXT_TIMEOUT", PARAM_TYPE_INT32, I32(1000), I32(100), I32(10000),
    "External nav dropout before position is dropped (ms)" },

  /* ---- VESC CAN link ----------------------------------------------------
   * VESC_EN is OFF by default. A new driver touching a new peripheral does
   * not belong in the boot path until it has run at least once.
   *
   * VESC_CAN_ID 0 means accept ANY controller id, which is what makes the
   * first run a discovery rather than a guess. Setting it to the id that
   * turns up narrows the HARDWARE filter, which matters more than it looks:
   * an unfiltered 1 Mbit/s bus can deliver over 8000 frames a second, and
   * discarding them in a task means waking up 8000 times a second to throw
   * work away.
   */

  { "VESC_EN", PARAM_TYPE_INT32, I32(1), I32(0), I32(1),
    "Start the VESC CAN link at boot" },
  { "VESC_CAN_ID", PARAM_TYPE_INT32, I32(0), I32(0), I32(255),
    "VESC controller id to accept (0 = any)" },
  { "VESC_BITRATE", PARAM_TYPE_INT32, I32(1000000), I32(125000),
    I32(1000000), "CAN bitrate (bit/s)" },

  /* Transmit. The two motor ceilings are deliberately timid: the first armed
   * run happens on a bench with a vehicle that may be on its wheels. Raising
   * them is a decision made with the hardware in front of you.
   *
   * VESC_STEER_MIN is allowed to exceed VESC_STEER_MAX. That is how a
   * reversed linkage is expressed, and rejecting it would push the reversal
   * into every controller that ever publishes a command.
   *
   * VESC_TX_RATE is capped at 500 because the scheduler tick is 1000 us
   * (CONFIG_USEC_PER_TICK) and the interrupt wait timeout uses whole ticks.
   * 500 Hz is exactly two ticks; above that the requested period is shorter
   * than the clock can express and the real rate would stop matching it.
   *
   * Rates that are not a whole number of ticks still average correctly, but
   * individual periods alternate between the ticks either side. 400 Hz is
   * 2.5 ticks, so frames go out 2 ms and 3 ms apart in turn. Fine for a
   * motor controller; worth knowing before reading a scope.
   */

  { "VESC_TX_RATE", PARAM_TYPE_INT32, I32(400), I32(1), I32(500),
    "Command frame rate (Hz)" },
  { "VESC_CMD_TO_MS", PARAM_TYPE_INT32, I32(200), I32(20), I32(5000),
    "Setpoint age before failsafe neutral (ms)" },
  { "VESC_CUR_MAX", PARAM_TYPE_FLOAT, F32(20.0f), F32(0.0f), F32(200.0f),
    "Motor current magnitude ceiling (A)" },
  { "VESC_DUTY_MAX", PARAM_TYPE_FLOAT, F32(0.30f), F32(0.0f), F32(1.0f),
    "Motor duty magnitude ceiling (0-1)" },
  { "VESC_STEER_MIN", PARAM_TYPE_INT32, I32(1100), I32(800), I32(2200),
    "Servo pulse at steering -1, full right (us)" },
  { "VESC_STEER_TRIM", PARAM_TYPE_INT32, I32(1500), I32(800), I32(2200),
    "Servo pulse at steering 0, straight (us)" },
  { "VESC_STEER_MAX", PARAM_TYPE_INT32, I32(1900), I32(800), I32(2200),
    "Servo pulse at steering +1, full left (us)" },

  /* Scalars turning VESC telemetry into the engineering units the companion
   * expects. All 1.0 until the vehicle is characterised - a guessed gear
   * ratio or torque constant is worse than an honest raw number, because it
   * looks calibrated.
   */

  { "VESC_TORQUE_K", PARAM_TYPE_FLOAT, F32(1.0f), F32(-1000.0f),
    F32(1000.0f), "Motor current to wheel torque (Nm per A)" },
  { "VESC_STEER_K", PARAM_TYPE_FLOAT, F32(1.0f), F32(-1000.0f), F32(1000.0f),
    "Steering ADC to angle (rad per V)" },
  { "VESC_SPEED_K", PARAM_TYPE_FLOAT, F32(1.0f), F32(-1000.0f), F32(1000.0f),
    "Tachometer rate to ground speed (m/s per count/s)" },

  /* Motor speed filtering, applied in the VESC daemon on every STATUS_5.
   *
   * VESC_TLM_HZ seeds the interval average so the first samples divide by a
   * sensible dt instead of whatever the first two timestamps happened to be.
   * It also sets the outlier bounds, so a wrong value here shows up as
   * refused intervals rather than a wrong speed.
   *
   * VESC_SPD_LPF is the ANTI-ALIAS filter for the 200 Hz downlink, which
   * samples this 400 Hz stream at half rate. 100 Hz is the downlink's
   * Nyquist; two poles put 150 Hz content roughly 14 dB down before it folds
   * to 50. Zero disables filtering.
   */

  { "VESC_TLM_HZ", PARAM_TYPE_INT32, I32(400), I32(1), I32(2000),
    "Expected STATUS_5 telemetry rate (Hz)" },
  { "VESC_SPD_LPF", PARAM_TYPE_FLOAT, F32(100.0f), F32(0.0f), F32(500.0f),
    "Motor speed low-pass cutoff (Hz, 0 = off)" },

  /* Telemetry drop-out before the vehicle is disarmed.
   *
   * A VESC that has stopped reporting is a VESC we cannot see the state of,
   * and continuing to command a motor blind is the failure this prevents.
   * 100 ms is 40 missed frames at the stock 400 Hz - long enough that
   * ordinary bus contention cannot trip it, short enough to matter.
   *
   * This is SEPARATE from VESC_CMD_TO_MS, which is about commands going
   * stale on the way OUT. This one is about telemetry stopping on the way
   * IN, and it disarms rather than merely commanding neutral.
   */

  { "VESC_TLM_TO_MS", PARAM_TYPE_INT32, I32(100), I32(0), I32(5000),
    "Telemetry dropout before disarm (ms, 0 = never)" },

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
