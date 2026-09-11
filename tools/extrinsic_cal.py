#!/usr/bin/env python3
"""Estimate planar mocap-marker to vehicle/IMU extrinsics from an xxCar ULog.

The external-pose packet contains x, y and yaw, so only planar x/y/yaw
extrinsics are observable from it. Roll, pitch and z translation are retained
from the logged parameters and explicitly reported as unobservable.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


class CalibrationError(ValueError):
    """The data cannot support a defensible calibration."""


def scalar(value):
    return value.item() if hasattr(value, "item") else value


def wrap_pi(value):
    return (np.asarray(value) + np.pi) % (2.0 * np.pi) - np.pi


def _series(ulog, name: str):
    matches = [dataset for dataset in ulog.data_list if dataset.name == name]
    if not matches:
        raise CalibrationError(f"log has no {name} topic")
    return matches[0]


def _sorted_unique(t, *columns):
    order = np.argsort(t, kind="stable")
    t = np.asarray(t, dtype=float)[order]
    values = [np.asarray(column, dtype=float)[order] for column in columns]
    keep = np.r_[True, np.diff(t) > 0]
    return (t[keep], *(value[keep] for value in values))


def local_polynomial(t, value, half_window=0.14, degree=3):
    """Return value, first and second derivative at irregular sample times."""
    t = np.asarray(t, dtype=float)
    value = np.asarray(value, dtype=float)
    estimate = np.full((3, len(t)), np.nan)
    minimum = degree + 2
    for index, center in enumerate(t):
        selected = np.flatnonzero(np.abs(t - center) <= half_window)
        if len(selected) < minimum:
            lo = max(0, index - minimum // 2)
            hi = min(len(t), lo + minimum)
            lo = max(0, hi - minimum)
            selected = np.arange(lo, hi)
        if len(selected) < minimum:
            continue
        relative = t[selected] - center
        scale = max(float(np.max(np.abs(relative))), 1.0e-6)
        weight = np.clip(1.0 - (np.abs(relative) / scale) ** 3, 0.05, 1.0) ** 3
        design = np.column_stack([relative ** power
                                  for power in range(degree + 1)])
        coefficient, *_ = np.linalg.lstsq(design * weight[:, None],
                                           value[selected] * weight,
                                           rcond=None)
        estimate[0, index] = coefficient[0]
        estimate[1, index] = coefficient[1]
        estimate[2, index] = 2.0 * coefficient[2]
    return estimate


def estimate_time_offset(pose_t, pose_yaw, imu_t, gyro_z,
                         limit_s=0.100, step_s=0.0005):
    """Cross-correlate yaw rates; result is IMU time minus pose time."""
    yaw_rate = local_polynomial(pose_t, np.unwrap(pose_yaw))[1]
    valid_pose = np.isfinite(yaw_rate)
    if np.std(yaw_rate[valid_pose]) < 0.08:
        raise CalibrationError("insufficient yaw-rate excitation for time offset")
    best = None
    scored = []
    for lag in np.arange(-limit_s, limit_s + step_s * 0.5, step_s):
        query = pose_t + lag
        valid = valid_pose & (query >= imu_t[0]) & (query <= imu_t[-1])
        if np.count_nonzero(valid) < 30:
            continue
        gyro = np.interp(query[valid], imu_t, gyro_z)
        reference = yaw_rate[valid]
        gyro -= np.mean(gyro)
        reference -= np.mean(reference)
        denominator = np.linalg.norm(gyro) * np.linalg.norm(reference)
        score = float(np.dot(gyro, reference) / denominator) if denominator else -1.0
        scored.append((float(lag), score))
        candidate = (score, -abs(lag), lag)
        if best is None or candidate > best:
            best = candidate
    if best is None or best[0] < 0.35:
        raise CalibrationError("yaw-rate time correlation is too weak")
    near_peak = [lag for lag, score in scored if score >= best[0] - 0.001]
    uncertainty = (0.5 * (max(near_peak) - min(near_peak))
                   if near_peak else 0.5 * step_s)
    pose_dt = np.diff(pose_t)
    return {"imu_minus_pose_s": float(best[2]), "correlation": float(best[0]),
            "peak_uncertainty_s": float(max(uncertainty, 0.5 * step_s)),
            "pose_rate_hz": float(1.0 / np.median(pose_dt[pose_dt > 0]))}


def circular_location(angles):
    angles = np.asarray(angles, dtype=float)
    if len(angles) == 0:
        raise CalibrationError("no angles for circular estimate")
    center = math.atan2(float(np.mean(np.sin(angles))),
                        float(np.mean(np.cos(angles))))
    for _ in range(3):
        residual = wrap_pi(angles - center)
        limit = max(float(np.percentile(np.abs(residual), 70)), math.radians(1.0))
        weight = np.minimum(1.0, limit / np.maximum(np.abs(residual), 1.0e-9))
        center = float(wrap_pi(center + np.sum(weight * residual) / np.sum(weight)))
    residual = wrap_pi(angles - center)
    return center, float(np.sqrt(np.mean(residual ** 2)))


def estimate_yaw(pose_yaw, velocity_x, velocity_y, wheel_speed=None,
                 yaw_rate=None, minimum_speed=0.50):
    speed = np.hypot(velocity_x, velocity_y)
    course = np.arctan2(velocity_y, velocity_x)
    wheel_valid = None
    if wheel_speed is not None:
        wheel_speed = np.asarray(wheel_speed)
        wheel_valid = np.abs(wheel_speed) >= 0.10
        course = course + np.where(wheel_speed < 0.0, np.pi, 0.0)
    usable = np.isfinite(course) & np.isfinite(pose_yaw) & (speed >= minimum_speed)
    if yaw_rate is not None:
        usable &= np.abs(np.asarray(yaw_rate)) <= 0.60
    if wheel_valid is not None and np.count_nonzero(usable & wheel_valid) >= 30:
        usable &= wheel_valid
    if np.count_nonzero(usable) < 30:
        raise CalibrationError("insufficient translating motion for yaw extrinsic")
    angles = wrap_pi(course[usable] - pose_yaw[usable])
    if wheel_valid is None or np.count_nonzero(usable & wheel_valid) < 30:
        # Position alone identifies a car's forward axis, but not which end is
        # the nose. Fold the 180-degree ambiguity and select the mounting near
        # the configured/default identity; the report makes that ambiguity
        # explicit through the input-source field.
        angles = 0.5 * wrap_pi(2.0 * angles)
        direction_source = "position course (180 deg ambiguity)"
    else:
        direction_source = "signed wheel speed + position course"
    yaw, rms = circular_location(angles)
    return {"yaw_rad": yaw, "yaw_deg": math.degrees(yaw),
            "rms_rad": rms, "samples": int(np.count_nonzero(usable)),
            "direction_source": direction_source}


def estimate_planar_lever_arm(marker_accel, body_accel, yaw_rate, yaw_accel,
                              relative_yaw):
    """Solve R_marker_body*a_body - a_marker = A(omega,alpha)*p + bias."""
    marker_accel = np.asarray(marker_accel, dtype=float)
    body_accel = np.asarray(body_accel, dtype=float)
    yaw_rate = np.asarray(yaw_rate, dtype=float)
    yaw_accel = np.asarray(yaw_accel, dtype=float)
    c, s = math.cos(relative_yaw), math.sin(relative_yaw)
    rotation = np.array([[c, -s], [s, c]])
    observed = (rotation @ body_accel.T).T - marker_accel
    blocks = []
    target = []
    for index, (omega, alpha) in enumerate(zip(yaw_rate, yaw_accel)):
        blocks.append([[-omega * omega, -alpha, 1.0, 0.0],
                       [alpha, -omega * omega, 0.0, 1.0]])
        target.extend(observed[index])
    design = np.asarray(blocks, dtype=float).reshape(-1, 4)
    target = np.asarray(target, dtype=float)
    finite_rows = np.all(np.isfinite(design), axis=1) & np.isfinite(target)
    design, target = design[finite_rows], target[finite_rows]
    if len(target) < 80:
        raise CalibrationError("not enough finite samples for lever arm")
    if np.std(yaw_rate) < 0.12:
        raise CalibrationError("insufficient turning excitation for lever arm")
    keep = np.ones(len(target), dtype=bool)
    solution = np.zeros(4)
    for _ in range(4):
        solution, *_ = np.linalg.lstsq(design[keep], target[keep], rcond=None)
        residual = target - design @ solution
        scale = max(float(np.percentile(np.abs(residual[keep]), 70)), 0.02)
        keep = np.abs(residual) <= 2.5 * scale
    singular = np.linalg.svd(design[keep, :2], compute_uv=False)
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else math.inf
    if condition > 100.0:
        raise CalibrationError(f"lever-arm excitation is ill-conditioned ({condition:.1f})")
    residual = target[keep] - design[keep] @ solution
    return {"position_x_m": float(solution[0]),
            "position_y_m": float(solution[1]),
            "accel_bias_x_mps2": float(solution[2]),
            "accel_bias_y_mps2": float(solution[3]),
            "rms_mps2": float(np.sqrt(np.mean(residual ** 2))),
            "condition": condition, "scalar_samples": int(np.count_nonzero(keep))}


def warmup_assessment(ulog):
    result: dict[str, Any] = {}
    try:
        accel = _series(ulog, "sensor_accel")
        time = np.asarray(accel.data["timestamp"], dtype=float) * 1.0e-6
        temperature = np.asarray(accel.data["temperature"], dtype=float)
        tail = time >= time[-1] - 30.0
        if np.count_nonzero(tail) >= 10:
            slope = np.polyfit(time[tail] - time[tail][0], temperature[tail], 1)[0]
            result["temperature_slope_c_per_min"] = float(slope * 60.0)
            result["thermal_stable"] = bool(abs(slope * 60.0) <= 0.10)
    except (CalibrationError, KeyError, ValueError):
        result["thermal_stable"] = None
    return result


def motion_assessment(t, velocity_x, velocity_y, yaw_rate):
    dt = np.diff(t, prepend=t[0])
    speed = np.hypot(velocity_x, velocity_y)
    stopped = speed < 0.08
    initial_end = 0
    while initial_end < len(stopped) and stopped[initial_end]:
        initial_end += 1
    stationary_start = (float(t[initial_end - 1] - t[0])
                        if initial_end > 1 else 0.0)
    left = float(np.sum(np.maximum(yaw_rate, 0.0) * dt))
    right = float(np.sum(np.maximum(-yaw_rate, 0.0) * dt))
    distance = float(np.sum(speed * dt))
    return {
        "initial_stationary_s": stationary_start,
        "translation_distance_m": distance,
        "left_turn_deg": math.degrees(left),
        "right_turn_deg": math.degrees(right),
        "stationary_alignment_pass": stationary_start >= 10.0,
        "translation_excitation_pass": distance >= 5.0,
        "bidirectional_turn_excitation_pass":
            math.degrees(left) >= 45.0 and math.degrees(right) >= 45.0,
    }


def calibrate_log(path: Path):
    try:
        from pyulog import ULog
    except ImportError as exc:
        raise CalibrationError("pyulog is required") from exc
    ulog = ULog(str(path))
    pose = _series(ulog, "external_pose")
    imu = _series(ulog, "vehicle_imu")
    diag = _series(ulog, "estimator_diag")

    pose_t, x, y, yaw = _sorted_unique(
        np.asarray(pose.data["timestamp_sample"]) * 1.0e-6,
        pose.data["x"], pose.data["y"], pose.data["yaw"])
    imu_t, gyro_z = _sorted_unique(
        np.asarray(imu.data["timestamp_sample"]) * 1.0e-6,
        np.asarray(imu.data["delta_angle[2]"]) /
        np.maximum(np.asarray(imu.data["delta_angle_dt"]), 1.0e-6))
    timing = estimate_time_offset(pose_t, yaw, imu_t, gyro_z)

    smooth_x = local_polynomial(pose_t, x)
    smooth_y = local_polynomial(pose_t, y)
    smooth_yaw = local_polynomial(pose_t, np.unwrap(yaw))
    wheel = None
    if "wheel_speed_mps" in diag.data:
        diag_t = np.asarray(diag.data["timestamp_sample"], dtype=float) * 1.0e-6
        wheel = np.interp(pose_t + timing["imu_minus_pose_s"], diag_t,
                          diag.data["wheel_speed_mps"])
    yaw_fit = estimate_yaw(yaw, smooth_x[1], smooth_y[1], wheel,
                           smooth_yaw[1])

    diag_t = np.asarray(diag.data["timestamp_sample"], dtype=float) * 1.0e-6
    body_accel = np.column_stack([
        np.interp(pose_t + timing["imu_minus_pose_s"], diag_t,
                  diag.data[f"residual_accel_body[{axis}]"])
        for axis in range(2)])
    marker_accel_nav = np.column_stack([smooth_x[2], smooth_y[2]])
    c, s = np.cos(yaw), np.sin(yaw)
    marker_accel = np.column_stack([
        c * marker_accel_nav[:, 0] + s * marker_accel_nav[:, 1],
        -s * marker_accel_nav[:, 0] + c * marker_accel_nav[:, 1]])
    usable = (np.isfinite(marker_accel).all(axis=1) &
              np.isfinite(body_accel).all(axis=1) &
              (np.abs(smooth_yaw[1]) < 4.0) &
              (np.linalg.norm(marker_accel, axis=1) < 30.0))
    lever = estimate_planar_lever_arm(marker_accel[usable], body_accel[usable],
                                      smooth_yaw[1, usable],
                                      smooth_yaw[2, usable], yaw_fit["yaw_rad"])
    parameters = {name: scalar(value)
                  for name, value in ulog.initial_parameters.items()}
    warmup = warmup_assessment(ulog)
    warmup.update(motion_assessment(pose_t, smooth_x[1], smooth_y[1],
                                    smooth_yaw[1]))
    quality = {
        "time_pass": timing["correlation"] >= 0.70 and
                     timing["peak_uncertainty_s"] <= 0.010,
        "yaw_pass": yaw_fit["rms_rad"] <= math.radians(10.0),
        "lever_arm_pass": lever["condition"] <= 30.0 and
                          lever["rms_mps2"] <= 0.50,
    }
    quality["spatial_pass"] = quality["yaw_pass"] and quality["lever_arm_pass"]
    return {
        "schema_version": 1,
        "source": str(path.resolve()),
        "observable": ["EK3_EXT_POS_X", "EK3_EXT_POS_Y", "EK3_EXT_YAW",
                       "EK3_EXT_DLY_MS"],
        "unobservable": ["EK3_EXT_POS_Z", "EK3_EXT_ROLL", "EK3_EXT_PITCH"],
        "timing": timing, "yaw": yaw_fit, "lever_arm": lever,
        "warmup": warmup, "quality": quality,
        "recommended_parameters": {
            "EK3_EXT_POS_X": lever["position_x_m"],
            "EK3_EXT_POS_Y": lever["position_y_m"],
            "EK3_EXT_YAW": yaw_fit["yaw_deg"],
            # Firmware applies corrected_time = supplied_time - delay.
            "EK3_EXT_DLY_MS": -1000.0 * timing["imu_minus_pose_s"],
        },
        "logged_parameters": parameters,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--nsh", action="store_true",
                        help="print reviewed param-set commands instead of JSON")
    args = parser.parse_args()
    try:
        result = calibrate_log(args.log)
    except CalibrationError as exc:
        parser.error(str(exc))
    if args.nsh:
        lines = ["# Review fit quality and physical dimensions before use."]
        for name, value in result["recommended_parameters"].items():
            quality_key = "time_pass" if name == "EK3_EXT_DLY_MS" \
                else "spatial_pass"
            if result["quality"][quality_key]:
                lines.append(f"param set {name} {value:.9g}")
            else:
                lines.append(f"# withheld {name} {value:.9g}: "
                             f"{quality_key} is false")
        encoded = "\n".join(lines) + "\n"
    else:
        encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(encoded, encoding="utf-8")
    else:
        print(encoded, end="")


if __name__ == "__main__":
    main()
