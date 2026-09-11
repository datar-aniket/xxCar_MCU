"""End-to-end check for the deterministic ULog replay/audit tool."""

import json
import struct
import subprocess
import sys
import tempfile
from pathlib import Path


MAGIC = bytes([0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35, 0x01])
FORMATS = {
    "vehicle_imu": "uint64_t timestamp;uint64_t timestamp_sample;float[3] delta_velocity;",
    "estimator_state": (
        "uint64_t timestamp;uint64_t timestamp_sample;float[4] quaternion;"
        "float[3] velocity;float[3] angle_variance;float[3] velocity_variance;"
        "float[3] position_variance;uint8_t solution_status;"),
    "estimator_diag": (
        "uint64_t timestamp;uint64_t timestamp_sample;float[3] velocity;"
        "float[3] residual_accel_body;float[2] extnav_innov;float[2] extnav_nis;"
        "uint32_t extnav_accept_count;uint32_t extnav_reject_count;uint16_t flags;"),
    "estimator_health": (
        "uint64_t timestamp;uint64_t timestamp_sample;int32_t imu_age_us;"
        "int32_t output_age_us;int32_t extnav_source_age_us;"
        "uint16_t output_replay_samples;uint16_t flags;"),
}


def message(kind: str, payload: bytes) -> bytes:
    return struct.pack("<HB", len(payload), ord(kind)) + payload


def make_log() -> bytes:
    output = bytearray(MAGIC + struct.pack("<Q", 1_000_000))
    output += message("B", bytes(40))
    for name, fields in FORMATS.items():
        output += message("F", f"{name}:{fields}".encode())
    key = b"int32_t EK3_DELAY_MS"
    output += message("P", bytes([len(key)]) + key + struct.pack("<i", 30))
    for msg_id, name in enumerate(FORMATS):
        output += message("A", struct.pack("<BH", 0, msg_id) + name.encode())

    for index in range(4):
        timestamp = 1_000_000 + index * 2_500
        imu = struct.pack("<QQfff", timestamp + 80, timestamp,
                          0.001 * index, 0.0, 0.024525)
        state = struct.pack("<QQ" + "f" * 16 + "B", timestamp + 100,
                            timestamp, 1.0, 0.0, 0.0, 0.0,
                            0.01 * index, 0.0, 0.0,
                            *([0.01] * 9), 0x21)
        diag = struct.pack("<QQ" + "f" * 10 + "IIH", timestamp, timestamp,
                           0.0, 0.0, 0.0, 0.01, 0.02, 0.01,
                           0.02, 0.01, 0.2, 0.1, index, 0, 0x0028)
        health = struct.pack("<QQiiiHH", timestamp + 100, timestamp,
                             100, 100, 1_000, 12, 0x000e)
        for msg_id, record in enumerate((imu, state, diag, health)):
            output += message("D", struct.pack("<H", msg_id) + record)
    return bytes(output)


def main() -> int:
    try:
        import pyulog  # noqa: F401
    except ImportError:
        print("ekf_replay: pyulog not installed - skipped")
        return 0

    repo = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        log = root / "input.ulg"
        report_a = root / "a.json"
        report_b = root / "b.json"
        events_a = root / "a.jsonl"
        events_b = root / "b.jsonl"
        log.write_bytes(make_log())
        command = [sys.executable, str(repo / "tools/ekf_replay.py"),
                   str(log), "--strict"]
        subprocess.run(command + ["--report", str(report_a),
                                  "--events", str(events_a)], check=True)
        subprocess.run(command + ["--report", str(report_b),
                                  "--events", str(events_b)], check=True)
        if report_a.read_bytes() != report_b.read_bytes():
            raise AssertionError("report is not deterministic")
        if events_a.read_bytes() != events_b.read_bytes():
            raise AssertionError("event stream is not deterministic")
        report = json.loads(report_a.read_text())
        assert report["manifest"]["parameters"]["EK3_DELAY_MS"] == 30
        assert report["topics"]["vehicle_imu"]["count"] == 4
        assert report["estimator"]["health"]["covariance_bad"] == 0
        lines = events_a.read_text().splitlines()
        assert len(lines) == 4
        assert all(json.loads(line)["topic"] == "vehicle_imu" for line in lines)
    print("ekf_replay: deterministic report and event stream - OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
