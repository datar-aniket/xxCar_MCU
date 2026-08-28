/****************************************************************************
 * apps/ekf3/ekf_extnav_frame.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __APPS_EKF3_EKF_EXTNAV_FRAME_H
#define __APPS_EKF3_EKF_EXTNAV_FRAME_H

#include <stdbool.h>

#ifndef FAR
#  define FAR
#endif

/* T_marker_body: the position and orientation of the IMU/body origin in the
 * mocap marker frame. Position is metres; rotation is ZYX roll/pitch/yaw in
 * radians. The reported body pose is T_map_marker * T_marker_body.
 */

struct ekf_extnav_extrinsics_s
{
  float position[3];
  float rotation[3];
};

bool ekf_extnav_apply_extrinsics(
  FAR const struct ekf_extnav_extrinsics_s *extrinsics,
  FAR const float marker_position[3], FAR const float marker_rpy[3],
  FAR float body_position[3], FAR float body_rpy[3]);

/* Transform the planar x/y/yaw covariance through the lever arm. Covariance
 * order is xx, xy, xyaw, yy, yyaw, yawyaw on both sides.
 */

bool ekf_extnav_transform_planar_covariance(
  FAR const struct ekf_extnav_extrinsics_s *extrinsics, float marker_yaw,
  FAR const float marker_covariance[6], FAR float body_covariance[6]);

#endif /* __APPS_EKF3_EKF_EXTNAV_FRAME_H */
