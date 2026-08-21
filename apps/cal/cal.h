/****************************************************************************
 * apps/cal/cal.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Calibration session, spoken over the USB CDC port to a host GUI.
 *
 * Two encodings share the one pipe, and the host tells them apart by the first
 * byte of each message:
 *
 *   '{'   an ASCII JSON line, terminated by '\n' - control and replies
 *   0xA5  a binary sample frame - see below
 *
 * Commands are ASCII because they are read by people far more often than they
 * are written; samples are binary because they are neither. 0xA5 is not
 * printable and cannot be the '{' that starts a JSON line, so a reader can
 * always resynchronise on it.
 *
 * `cal session` runs from the shell on TELEM1/DEBUG and opens /dev/ttyACM0.
 * The shell living on a different port is what makes this safe: nothing else
 * holds the USB port, so there is no handoff and no race for input bytes.
 ****************************************************************************/

#ifndef __APPS_CAL_CAL_H
#define __APPS_CAL_CAL_H

#include <nuttx/config.h>

#include <stdint.h>

/* The port the GUI listens on. Not configurable - it is the only USB serial
 * device on this board.
 */

#define CAL_DEVPATH  "/dev/ttyACM0"

/* Protocol version, reported in `hello` so a mismatched GUI can say so instead
 * of misbehaving. 2 added batched, integer-encoded sample frames; 3 added the
 * `gyro <sensor>` bias command and the CAL_GYROn_OK flag that goes with it.
 * Version 4 adds full 3D magnetometer ellipsoid calibration.
 */

#define CAL_PROTO_VERSION 4

/* Binary sample frame.
 *
 *   off  size  field
 *   0    1     0xA5    sync
 *   1    2     len     u16 LE, bytes after len excluding the CRC
 *   3    1     id      sensor id, as reported by `list`
 *   4    1     seq     per-frame, wraps at 256 - the GUI detects drops
 *   5    4     t0_us   u32 LE, timestamp of the FIRST sample in the frame
 *   9    2     dt_us   u16 LE, nominal spacing between samples
 *   11   1     count   samples in this frame
 *   12   1     nvals   values per sample
 *   13   1     enc     CAL_ENC_I16 or CAL_ENC_F32
 *   14   ...   data    count * nvals values, little-endian
 *   +    2     crc16   CCITT-FALSE over bytes 1 .. end-of-data
 *
 * Batching is what makes full-rate streaming possible. One frame per sample at
 * 2 kHz is 2000 writes per second, and the per-write cost - not the byte count
 * - is what caps throughput. Fifty samples per frame turns that into forty.
 *
 * Sending a whole timestamp per sample would then dominate the payload, so the
 * frame carries the first timestamp and the nominal spacing instead; a
 * fixed-rate stream is exactly what that describes.
 *
 * CAL_ENC_I16 sends the sensor's native integer resolution with a per-sensor
 * scale reported by `list` (value = raw * scale). For an IMU that is lossless -
 * the driver's own LSB - at half the bytes of a float. Sensors whose values do
 * not share one symmetric range, like a barometer reporting 1013 hPa beside
 * 40 degC, use CAL_ENC_F32; they are slow, so the bytes do not matter.
 */

#define CAL_SYNC        0xa5
#define CAL_ENC_I16     0
#define CAL_ENC_F32     1

#define CAL_MAX_VALUES  4        /* widest sensor we stream */
#define CAL_BATCH_MAX   50       /* samples per frame at full rate */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Run a session until the host quits or the cable is pulled. Returns OK, or a
 * negated errno (-ENOTCONN no host, -EBUSY port not reserved).
 */

int cal_session(void);

#endif /* __APPS_CAL_CAL_H */
