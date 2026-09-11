#!/usr/bin/env python3
"""Deterministic EKF ULog audit and canonical input-stream exporter.

This tool does not imitate the C filter in Python. It preserves every logged
input in a stable sample-time order so the firmware core can consume the same
stream bit-for-bit, and it produces a quantitative report from the recorded
state/innovation/health topics. Multiple rollover parts may be supplied in any
order; ordering is always (timestamp_sample, source priority, part, row).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Iterable

try:
    from pyulog import ULog
except ImportError as exc:  # pragma: no cover - depends on host setup
    raise SystemExit("pyulog is required: python3 -m pip install pyulog") from exc


INPUT_TOPICS = (
    "vehicle_imu",
    "vehicle_mag",
    "vehicle_baro",
    "external_pose",
    "vesc_status",
)
PRIORITY = {name: index for index, name in enumerate(INPUT_TOPICS)}


def scalar(value: Any) -> Any:
    """Convert numpy scalars to stable JSON primitives."""
    if hasattr(value, "item"):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def finite(values: Iterable[Any]) -> list[float]:
    result = []
    for value in values:
        number = float(value)
        if math.isfinite(number):
            result.append(number)
    return result


def percentile(values: Iterable[Any], fraction: float) -> float | None:
    ordered = sorted(finite(values))
    if not ordered:
        return None
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def stats(values: Iterable[Any]) -> dict[str, Any]:
    data = finite(values)
    if not data:
        return {"count": 0, "mean": None, "rms": None, "p95": None,
                "max_abs": None}
    return {
        "count": len(data),
        "mean": sum(data) / len(data),
        "rms": math.sqrt(sum(value * value for value in data) / len(data)),
        "p95": percentile(data, 0.95),
        "max_abs": max(abs(value) for value in data),
    }


def counter_increments(values: Iterable[Any]) -> int:
    sequence = [int(value) for value in values]
    return sum(max(0, current - previous)
               for previous, current in zip(sequence, sequence[1:]))


def timing(timestamps: Iterable[Any]) -> dict[str, Any]:
    values = [int(value) for value in timestamps]
    delta = [current - previous
             for previous, current in zip(values, values[1:])]
    positive = [value for value in delta if value > 0]
    duration = values[-1] - values[0] if len(values) > 1 else 0
    return {
        "count": len(values),
        "duration_s": duration * 1.0e-6,
        "rate_hz": ((len(values) - 1) * 1.0e6 / duration
                    if duration > 0 else None),
        "dt_us_p50": percentile(positive, 0.50),
        "dt_us_p95": percentile(positive, 0.95),
        "dt_us_max": max(positive) if positive else None,
        "duplicate_count": sum(value == 0 for value in delta),
        "backward_count": sum(value < 0 for value in delta),
    }


def fields(dataset: Any, names: list[str]) -> list[list[Any]] | None:
    if any(name not in dataset.data for name in names):
        return None
    return [dataset.data[name] for name in names]


def vector_norm(dataset: Any, names: list[str], mask: list[bool] | None = None
                ) -> list[float]:
    columns = fields(dataset, names)
    if columns is None:
        return []
    output = []
    for index, values in enumerate(zip(*columns)):
        if mask is None or mask[index]:
            output.append(math.sqrt(sum(float(value) ** 2 for value in values)))
    return output


def quaternion_tilt(dataset: Any) -> tuple[list[float], list[float]]:
    columns = fields(dataset, [f"quaternion[{i}]" for i in range(4)])
    if columns is None:
        return [], []
    roll = []
    pitch = []
    for w, x, y, z in zip(*columns):
        w, x, y, z = map(float, (w, x, y, z))
        roll.append(math.atan2(2.0 * (w * x + y * z),
                               1.0 - 2.0 * (x * x + y * y)))
        argument = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch.append(math.asin(argument))
    return roll, pitch


def load_logs(paths: list[Path]) -> tuple[list[tuple[int, Any]], dict[str, Any]]:
    logs = []
    manifest = {"files": [], "parameters": {}, "parameter_mismatches": {}}
    reference_params = None
    for part, path in enumerate(sorted(paths, key=lambda item: str(item))):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        ulog = ULog(str(path))
        logs.append((part, ulog))
        manifest["files"].append({
            "path": str(path.resolve()),
            "bytes": path.stat().st_size,
            "sha256": digest,
            "start_timestamp": int(ulog.start_timestamp),
        })
        params = {name: scalar(value)
                  for name, value in sorted(ulog.initial_parameters.items())}
        if reference_params is None:
            reference_params = params
            manifest["parameters"] = params
        elif params != reference_params:
            all_names = sorted(set(reference_params) | set(params))
            manifest["parameter_mismatches"][str(path)] = {
                name: [reference_params.get(name), params.get(name)]
                for name in all_names
                if reference_params.get(name) != params.get(name)
            }
    return logs, manifest


def collect(logs: list[tuple[int, Any]]) -> dict[str, list[tuple[int, Any]]]:
    topics: dict[str, list[tuple[int, Any]]] = {}
    for part, ulog in logs:
        for dataset in ulog.data_list:
            topics.setdefault(dataset.name, []).append((part, dataset))
    return topics


def canonical_events(logs: list[tuple[int, Any]]) -> list[tuple[Any, ...]]:
    events = []
    for part, ulog in logs:
        for dataset in ulog.data_list:
            if dataset.name not in PRIORITY:
                continue
            count = len(dataset.data.get("timestamp", []))
            sample_field = ("timestamp_sample"
                            if "timestamp_sample" in dataset.data
                            else "timestamp")
            names = sorted(dataset.data)
            for row in range(count):
                sample_time = int(dataset.data[sample_field][row])
                payload = {name: scalar(dataset.data[name][row])
                           for name in names
                           if not name.startswith("_padding")}
                events.append((sample_time, PRIORITY[dataset.name], part, row,
                               dataset.name, int(dataset.multi_id), payload))
    events.sort(key=lambda event: event[:4])
    return events


def build_report(topics: dict[str, list[tuple[int, Any]]],
                 manifest: dict[str, Any]) -> dict[str, Any]:
    report: dict[str, Any] = {"schema_version": 1, "manifest": manifest,
                              "topics": {}, "estimator": {}, "warnings": []}
    for name, parts in sorted(topics.items()):
        timestamps = []
        for _, dataset in parts:
            key = "timestamp_sample" if "timestamp_sample" in dataset.data else "timestamp"
            timestamps.extend(dataset.data.get(key, []))
        report["topics"][name] = timing(timestamps)

    def joined(name: str) -> list[Any]:
        result = []
        for _, dataset in topics.get(name, []):
            result.append(dataset)
        return result

    states = joined("estimator_state")
    if states:
        velocity = []
        rolls = []
        pitches = []
        covariance = []
        validity = []
        for dataset in states:
            velocity.extend(vector_norm(dataset,
                                        [f"velocity[{i}]" for i in range(3)]))
            roll, pitch = quaternion_tilt(dataset)
            rolls.extend(math.degrees(value) for value in roll)
            pitches.extend(math.degrees(value) for value in pitch)
            for prefix in ("angle_variance", "velocity_variance",
                           "position_variance"):
                for axis in range(3):
                    covariance.extend(dataset.data.get(f"{prefix}[{axis}]", []))
            validity.extend(dataset.data.get("solution_status", []))
        report["estimator"]["velocity_norm_mps"] = stats(velocity)
        report["estimator"]["roll_deg"] = stats(rolls)
        report["estimator"]["pitch_deg"] = stats(pitches)
        finite_covariance = finite(covariance)
        report["estimator"]["covariance"] = {
            "finite_fraction": (len(finite_covariance) / len(covariance)
                                if covariance else None),
            "negative_count": sum(float(value) < 0 for value in finite_covariance),
            "max": max(finite_covariance) if finite_covariance else None,
        }
        report["estimator"]["position_valid_fraction"] = (
            sum((int(value) & (1 << 5)) != 0 for value in validity) / len(validity)
            if validity else None)

    diagnostics = joined("estimator_diag")
    if diagnostics:
        ext_innov = []
        ext_nis = []
        stationary_speed = []
        residual = []
        accept = reject = 0
        for dataset in diagnostics:
            ext_innov.extend(vector_norm(dataset,
                                         ["extnav_innov[0]", "extnav_innov[1]"]))
            ext_nis.extend(vector_norm(dataset,
                                       ["extnav_nis[0]", "extnav_nis[1]"]))
            flags = [int(value) for value in dataset.data.get("flags", [])]
            mask = [bool(value & (1 << 3)) and bool(value & (1 << 5))
                    for value in flags]
            stationary_speed.extend(vector_norm(
                dataset, [f"velocity[{i}]" for i in range(3)], mask))
            residual.extend(vector_norm(
                dataset, [f"residual_accel_body[{i}]" for i in range(3)], mask))
            accept += counter_increments(dataset.data.get("extnav_accept_count", []))
            reject += counter_increments(dataset.data.get("extnav_reject_count", []))
        report["estimator"]["extnav_innovation_norm_m"] = stats(ext_innov)
        report["estimator"]["extnav_nis_norm"] = stats(ext_nis)
        report["estimator"]["stationary_velocity_norm_mps"] = stats(stationary_speed)
        report["estimator"]["stationary_residual_accel_norm_mps2"] = stats(residual)
        report["estimator"]["extnav_updates"] = {
            "accepted": accept, "rejected": reject,
            "accept_fraction": accept / (accept + reject) if accept + reject else None,
        }

    health = joined("estimator_health")
    if health:
        flags = []
        imu_age = []
        output_age = []
        replay = []
        ext_age = []
        for dataset in health:
            flags.extend(int(value) for value in dataset.data.get("flags", []))
            imu_age.extend(dataset.data.get("imu_age_us", []))
            output_age.extend(dataset.data.get("output_age_us", []))
            replay.extend(dataset.data.get("output_replay_samples", []))
            ext_age.extend(dataset.data.get("extnav_source_age_us", []))
        report["estimator"]["timing"] = {
            "imu_age_us": stats(imu_age),
            "output_age_us": stats(output_age),
            "extnav_source_age_us": stats(ext_age),
            "replay_samples": stats(replay),
        }
        report["estimator"]["health"] = {
            "samples": len(flags),
            "covariance_bad": sum((value & 0x0e) != 0x0e for value in flags),
            "imu_overflow_active": sum(bool(value & (1 << 11)) for value in flags),
            "aiding_overflow_active": sum(bool(value & (1 << 12)) for value in flags),
            "publish_error_active": sum(bool(value & (1 << 13)) for value in flags),
        }

    for required in ("vehicle_imu", "estimator_state", "estimator_diag"):
        if required not in topics:
            report["warnings"].append(f"missing required topic: {required}")
    if "estimator_health" not in topics:
        report["warnings"].append(
            "log has no estimator_health topic; it predates health logging")
    if not manifest["parameters"]:
        report["warnings"].append(
            "log has no parameter snapshot; it predates reproducible logging")
    if manifest["parameter_mismatches"]:
        report["warnings"].append("parameters differ between rollover parts")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path, help="ULog part(s)")
    parser.add_argument("--report", type=Path, help="write JSON report")
    parser.add_argument("--events", type=Path,
                        help="write canonical EKF input stream as JSONL")
    parser.add_argument("--strict", action="store_true",
                        help="fail on missing core topics or recorded health faults")
    args = parser.parse_args()

    missing = [str(path) for path in args.logs if not path.is_file()]
    if missing:
        parser.error("not a file: " + ", ".join(missing))

    logs, manifest = load_logs(args.logs)
    topics = collect(logs)
    report = build_report(topics, manifest)

    if args.events:
        with args.events.open("w", encoding="utf-8", newline="\n") as output:
            for sample_time, _, part, row, topic, instance, payload in canonical_events(logs):
                output.write(json.dumps({"timestamp_sample": sample_time,
                                         "topic": topic, "instance": instance,
                                         "part": part, "row": row,
                                         "data": payload},
                                        sort_keys=True, separators=(",", ":"),
                                        allow_nan=False) + "\n")

    encoded = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.report:
        args.report.write_text(encoded, encoding="utf-8")
    else:
        sys.stdout.write(encoded)

    if args.strict:
        health_faults = report.get("estimator", {}).get("health", {})
        if report["warnings"] or any(health_faults.get(name, 0) for name in (
                "covariance_bad", "imu_overflow_active",
                "aiding_overflow_active", "publish_error_active")):
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
