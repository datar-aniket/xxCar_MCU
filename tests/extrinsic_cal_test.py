"""Numerical and refusal tests for log-derived planar extrinsics."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from extrinsic_cal import (CalibrationError, estimate_planar_lever_arm,
                           estimate_time_offset, estimate_yaw)
from frame_gui import parameter_frames


def main():
    pose_t = np.arange(0.0, 30.0, 0.02)
    lag = 0.018
    pose_yaw = 0.7 * np.sin(0.43 * pose_t) + 0.2 * np.sin(1.17 * pose_t)
    imu_t = np.arange(0.0, 30.0, 0.0025)
    physical_t = imu_t - lag
    gyro = (0.7 * 0.43 * np.cos(0.43 * physical_t) +
            0.2 * 1.17 * np.cos(1.17 * physical_t))
    timing = estimate_time_offset(pose_t, pose_yaw, imu_t, gyro)
    assert abs(timing["imu_minus_pose_s"] - lag) <= 0.001
    assert timing["correlation"] > 0.99

    relative_yaw = math.radians(7.0)
    speed = 2.0 + 0.3 * np.sin(pose_t)
    course = pose_yaw + relative_yaw
    yaw = estimate_yaw(pose_yaw, speed * np.cos(course),
                       speed * np.sin(course), speed)
    assert abs(yaw["yaw_rad"] - relative_yaw) < math.radians(0.05)

    omega = 0.6 * np.sin(0.7 * pose_t)
    alpha = 0.42 * np.cos(0.7 * pose_t)
    marker_accel = np.column_stack((0.4 * np.sin(pose_t),
                                    0.3 * np.cos(0.8 * pose_t)))
    position = np.array([0.22, -0.08])
    c, s = math.cos(relative_yaw), math.sin(relative_yaw)
    rotation = np.array([[c, -s], [s, c]])
    body_accel = []
    for index in range(len(pose_t)):
        matrix = np.array([[-omega[index] ** 2, -alpha[index]],
                           [alpha[index], -omega[index] ** 2]])
        in_marker = marker_accel[index] + matrix @ position
        body_accel.append(rotation.T @ in_marker)
    lever = estimate_planar_lever_arm(marker_accel, np.asarray(body_accel),
                                      omega, alpha, relative_yaw)
    assert abs(lever["position_x_m"] - position[0]) < 1.0e-6
    assert abs(lever["position_y_m"] - position[1]) < 1.0e-6

    try:
        estimate_time_offset(pose_t, np.zeros_like(pose_t), imu_t, gyro)
        raise AssertionError("static data was accepted for time calibration")
    except CalibrationError:
        pass

    try:
        estimate_planar_lever_arm(marker_accel, np.asarray(body_accel),
                                  np.zeros_like(omega), np.zeros_like(alpha),
                                  relative_yaw)
        raise AssertionError("straight motion was accepted for lever arm")
    except CalibrationError:
        pass

    frames = parameter_frames({"EK3_EXT_POS_X": 1.0,
                               "EK3_EXT_YAW": 90.0})
    assert [frame[0] for frame in frames] == [
        "BODY FLU", "IMU0", "IMU1", "MAG0", "MOCAP"]
    # T_marker_body says body is +marker-X from the marker and rotated +90
    # degrees. Expressed in body axes, the marker origin is therefore +Y.
    assert np.allclose(frames[-1][1], [0.0, 1.0, 0.0], atol=1.0e-12)
    assert np.allclose(frames[-1][2][:, 0], [0.0, -1.0, 0.0],
                       atol=1.0e-12)

    print("extrinsic_cal: solve, refusal gates and frame display - OK")


if __name__ == "__main__":
    main()
