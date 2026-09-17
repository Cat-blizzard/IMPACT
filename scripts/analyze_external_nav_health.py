#!/usr/bin/env python3
"""Correlate ExternalNav rosbag evidence with ArduPilot DataFlash.

This report is deliberately diagnostic, not a pass/fail evaluator.  The two
logs use different clocks, so it reports each timeline and the rosbag record
time used for approximate cross-stream ordering instead of pretending that
DataFlash ``TimeUS`` is a ROS simulation timestamp.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import rosbag2_py
from pymavlink import DFReader
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BLOCKING_FAULT_TOKENS = (
    "visodom: not healthy",
    "visodom: roll/pitch diff",
    "ekf variance",
    "ekf failsafe",
    "potential thrust loss",
)

OBSERVATION_TOKENS = (
    "ekf3 lane switch",
    "ekf primary changed",
)


def classify_status_text(text: str) -> tuple[str | None, str | None]:
    lowered = text.strip().lower()
    fault = next((token for token in BLOCKING_FAULT_TOKENS if token in lowered), None)
    observation = next((token for token in OBSERVATION_TOKENS if token in lowered), None)
    return fault, observation


def _stamp(message: Any) -> float:
    header = getattr(message, "header", None)
    if header is None:
        return math.nan
    return float(header.stamp.sec) + 1e-9 * float(header.stamp.nanosec)


def _xyz(message: Any) -> list[float]:
    p = message.pose.pose.position
    return [float(p.x), float(p.y), float(p.z)]


def _quat(message: Any) -> list[float]:
    q = message.pose.pose.orientation
    return [float(q.w), float(q.x), float(q.y), float(q.z)]


def _rpy_deg(q: list[float]) -> list[float]:
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_arg = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_arg)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def _odom_summary(samples: list[dict[str, Any]]) -> dict[str, Any]:
    if not samples:
        return {"count": 0}
    stamps = [float(item["header_stamp_s"]) for item in samples]
    valid_stamps = [stamp for stamp in stamps if math.isfinite(stamp)]
    gaps = [b - a for a, b in zip(valid_stamps, valid_stamps[1:])]
    positions = [item["xyz_m"] for item in samples]
    steps = [
        math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
        for a, b in zip(positions, positions[1:])
    ]
    return {
        "count": len(samples),
        "header_stamp_start_s": valid_stamps[0] if valid_stamps else None,
        "header_stamp_end_s": valid_stamps[-1] if valid_stamps else None,
        "max_header_gap_s": max(gaps) if gaps else None,
        "nonmonotonic_header_stamps": sum(gap <= 0.0 for gap in gaps),
        "max_position_step_m": max(steps) if steps else 0.0,
        "first": samples[0],
        "last": samples[-1],
        "frame_ids": sorted({item["frame_id"] for item in samples}),
        "child_frame_ids": sorted({item["child_frame_id"] for item in samples}),
        "samples": samples,
    }


def _quat_angle_deg(first: list[float], second: list[float]) -> float:
    dot = sum(a * b for a, b in zip(first, second))
    dot = max(-1.0, min(1.0, abs(dot)))
    return math.degrees(2.0 * math.acos(dot))


def _orientation_comparison(
    first: list[dict[str, Any]], second: list[dict[str, Any]], max_record_gap_s: float = 0.25
) -> dict[str, Any]:
    """Compare nearest rosbag-recorded orientations without asserting frame equivalence."""
    if not first or not second:
        return {"matched_count": 0, "max_record_gap_s": max_record_gap_s}
    second_index = 0
    angles: list[float] = []
    gaps: list[float] = []
    for sample in first:
        record_s = float(sample["record_ns"]) / 1e9
        while second_index + 1 < len(second):
            current_gap = abs(float(second[second_index]["record_ns"]) / 1e9 - record_s)
            next_gap = abs(float(second[second_index + 1]["record_ns"]) / 1e9 - record_s)
            if next_gap <= current_gap:
                second_index += 1
            else:
                break
        candidate = second[second_index]
        gap = abs(float(candidate["record_ns"]) / 1e9 - record_s)
        if gap <= max_record_gap_s:
            gaps.append(gap)
            angles.append(_quat_angle_deg(sample["quaternion_wxyz"], candidate["quaternion_wxyz"]))
    return {
        "matched_count": len(angles),
        "max_record_gap_s": max(gaps) if gaps else None,
        "max_orientation_difference_deg": max(angles) if angles else None,
        "mean_orientation_difference_deg": sum(angles) / len(angles) if angles else None,
        "p95_orientation_difference_deg": sorted(angles)[max(0, math.ceil(0.95 * len(angles)) - 1)]
        if angles else None,
    }


def read_rosbag(run_dir: Path) -> dict[str, Any]:
    bag = run_dir / "rosbag"
    # Prefer a plain SQLite bag. It is deterministic and does not mutate the
    # run directory while rosbag2 attempts to decompress a compressed bag.
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""),
        )
    except RuntimeError as plain_error:
        compressed_reader_type = getattr(rosbag2_py, "SequentialCompressionReader", None)
        if compressed_reader_type is None:
            raise RuntimeError(f"Cannot open plain rosbag SQLite: {plain_error}") from plain_error
        reader = compressed_reader_type()
        try:
            reader.open(
                rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""),
            )
        except RuntimeError as compressed_error:
            raise RuntimeError(
                "Rosbag is neither readable SQLite nor readable rosbag2 compression; "
                "preserve the run as diagnostic evidence and rerun with compression disabled. "
                f"plain={plain_error}; compressed={compressed_error}"
            ) from compressed_error
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = {
        "/localization/odom",
        "/uav1/mavros/odometry/out",
        "/uav1/mavros/local_position/odom",
        "/uav1/mavros/state",
        "/uav1/mavros/statustext/recv",
        "/uav1/mavros/extended_state",
        "/xq/p4/extnav/status",
        "/clock",
    }
    odom: dict[str, list[dict[str, Any]]] = {topic: [] for topic in selected if "odom" in topic}
    statuses: list[dict[str, Any]] = []
    state_changes: list[dict[str, Any]] = []
    status_texts: list[dict[str, Any]] = []
    topic_counts: dict[str, int] = {}
    clock_samples: list[float] = []
    while reader.has_next():
        topic, raw, record_ns = reader.read_next()
        topic_counts[topic] = topic_counts.get(topic, 0) + 1
        if topic not in selected or topic not in topic_types:
            continue
        message = deserialize_message(raw, get_message(topic_types[topic]))
        if topic in odom:
            odom[topic].append(
                {
                    "record_ns": int(record_ns),
                    "header_stamp_s": _stamp(message),
                    "frame_id": str(message.header.frame_id),
                    "child_frame_id": str(message.child_frame_id),
                    "xyz_m": _xyz(message),
                    "quaternion_wxyz": _quat(message),
                    "rpy_deg": _rpy_deg(_quat(message)),
                }
            )
        elif topic == "/xq/p4/extnav/status":
            try:
                payload = json.loads(message.data)
            except (TypeError, json.JSONDecodeError):
                payload = {"invalid_json": True, "raw": str(message.data)}
            statuses.append({"record_ns": int(record_ns), "status": payload})
        elif topic == "/uav1/mavros/statustext/recv":
            text = str(message.text)
            fault, observation = classify_status_text(text)
            status_texts.append(
                {
                    "record_ns": int(record_ns),
                    "severity": int(message.severity),
                    "text": text,
                    "fault": fault,
                    "observation": observation,
                }
            )
        elif topic == "/uav1/mavros/state":
            state_changes.append(
                {
                    "record_ns": int(record_ns),
                    "connected": bool(message.connected),
                    "armed": bool(message.armed),
                    "guided": bool(message.guided),
                    "mode": str(message.mode),
                    "system_status": int(message.system_status),
                }
            )
        elif topic == "/clock":
            clock_samples.append(
                float(message.clock.sec) + 1e-9 * float(message.clock.nanosec)
            )
    return {
        "topic_counts": topic_counts,
        "odom": {topic: _odom_summary(samples) for topic, samples in odom.items()},
        "external_nav_status": {
            "count": len(statuses),
            "first": statuses[0] if statuses else None,
            "last": statuses[-1] if statuses else None,
            "unhealthy_samples": [
                item for item in statuses if item.get("status", {}).get("healthy") is not True
            ][:20],
        },
        "status_text": {
            "count": len(status_texts),
            "first_fault": next((item for item in status_texts if item["fault"]), None),
            "faults": [item for item in status_texts if item["fault"]][:50],
            "observations": [item for item in status_texts if item["observation"]][:50],
            "all": status_texts[-200:],
        },
        "state": {
            "count": len(state_changes),
            "mode_changes": [
                item for index, item in enumerate(state_changes)
                if index == 0 or item["mode"] != state_changes[index - 1]["mode"]
            ],
            "arm_changes": [
                item for index, item in enumerate(state_changes)
                if index == 0 or item["armed"] != state_changes[index - 1]["armed"]
            ],
        },
        "clock": {
            "count": len(clock_samples),
            "start_s": clock_samples[0] if clock_samples else None,
            "end_s": clock_samples[-1] if clock_samples else None,
        },
    }


def read_dataflash(run_dir: Path) -> dict[str, Any]:
    files = sorted((run_dir / "sitl_runtime" / "logs").glob("*.BIN"))
    result: dict[str, Any] = {
        "files": [],
        "first_critical_message": None,
        "events": [],
        "critical_events": [],
        "observation_events": [],
    }
    for path in files:
        reader = DFReader.DFReader_binary(str(path))
        messages: list[dict[str, Any]] = []
        visp: list[dict[str, Any]] = []
        while True:
            message = reader.recv_msg()
            if message is None:
                break
            data = message.to_dict()
            name = message.get_type()
            time_us = data.get("TimeUS")
            if time_us is None:
                continue
            time_s = float(time_us) / 1e6
            if name == "MSG":
                text = str(data.get("Message", ""))
                fault, observation = classify_status_text(text)
                if fault:
                    event = {"time_s": time_s, "type": name, "fault": fault, "text": text}
                    messages.append(event)
                    result["events"].append({"source": path.name, **event})
                    result["critical_events"].append({"source": path.name, **event})
                if observation:
                    event = {
                        "time_s": time_s,
                        "type": name,
                        "observation": observation,
                        "text": text,
                    }
                    result["observation_events"].append({"source": path.name, **event})
            elif name in ("ERR", "EV", "MODE"):
                event = {"time_s": time_s, "type": name, **data}
                messages.append(event)
                result["events"].append({"source": path.name, **event})
            elif name == "VISP":
                visp.append(
                    {
                        "time_s": time_s,
                        "remote_time_s": float(data.get("RTimeUS", math.nan)) / 1e6,
                        "corrected_time_s": float(data.get("CTimeMS", math.nan)) / 1e3,
                        "roll_deg": float(data.get("R", math.nan)),
                        "pitch_deg": float(data.get("P", math.nan)),
                        "yaw_deg": float(data.get("Y", math.nan)),
                        "position_error_m": float(data.get("PErr", math.nan)),
                        "attitude_error_deg": float(data.get("AErr", math.nan)),
                        "reset": int(data.get("Rst", 0)),
                        "ignored": int(data.get("Ign", 0)),
                    }
                )
        receive_gaps = [
            current["time_s"] - previous["time_s"]
            for previous, current in zip(visp, visp[1:])
        ]
        remote_gaps = [
            current["remote_time_s"] - previous["remote_time_s"]
            for previous, current in zip(visp, visp[1:])
            if math.isfinite(previous["remote_time_s"])
            and math.isfinite(current["remote_time_s"])
        ]
        correction_ages_ms = [
            1000.0 * (item["time_s"] - item["corrected_time_s"])
            for item in visp
            if math.isfinite(item["corrected_time_s"])
        ]
        timeout_gaps = [
            {
                "previous_receive_time_s": previous["time_s"],
                "receive_time_s": current["time_s"],
                "gap_s": current["time_s"] - previous["time_s"],
                "previous_remote_time_s": previous["remote_time_s"],
                "remote_time_s": current["remote_time_s"],
            }
            for previous, current in zip(visp, visp[1:])
            if current["time_s"] - previous["time_s"] >= 0.3
        ]
        result["files"].append(
            {
                "path": str(path),
                "size_bytes": path.stat().st_size,
                "critical_messages": messages,
                "first_critical_message": messages[0] if messages else None,
                "visp": {
                    "count": len(visp),
                    "first": visp[0] if visp else None,
                    "last": visp[-1] if visp else None,
                    "max_receive_gap_s": max(receive_gaps) if receive_gaps else None,
                    "max_remote_header_gap_s": max(remote_gaps) if remote_gaps else None,
                    "receive_gaps_at_least_300ms": len(timeout_gaps),
                    "timeout_gaps": timeout_gaps,
                    "max_correction_age_ms": max(correction_ages_ms)
                    if correction_ages_ms else None,
                    "max_abs_roll_deg": max((abs(item["roll_deg"]) for item in visp), default=None),
                    "max_abs_pitch_deg": max((abs(item["pitch_deg"]) for item in visp), default=None),
                    "max_position_error_m": max((item["position_error_m"] for item in visp), default=None),
                    "max_attitude_error_deg": max((item["attitude_error_deg"] for item in visp), default=None),
                },
            }
        )
    result["events"].sort(key=lambda item: (item.get("time_s", math.inf), item.get("type", "")))
    result["critical_events"].sort(key=lambda item: item.get("time_s", math.inf))
    result["observation_events"].sort(key=lambda item: item.get("time_s", math.inf))
    result["first_critical_message"] = (
        result["critical_events"][0] if result["critical_events"] else None
    )
    return result


def build_report(run_dir: Path) -> dict[str, Any]:
    ros = read_rosbag(run_dir)
    dataflash = read_dataflash(run_dir)
    raw = ros["odom"].get("/localization/odom", {})
    output = ros["odom"].get("/uav1/mavros/odometry/out", {})
    fcu = ros["odom"].get("/uav1/mavros/local_position/odom", {})
    raw_samples = raw.get("samples", [])
    output_samples = output.get("samples", [])
    fcu_samples = fcu.get("samples", [])
    visp_count = sum(item["visp"]["count"] for item in dataflash["files"])
    output_count = int(output.get("count", 0))
    visp_timeout_gaps = sum(
        item["visp"]["receive_gaps_at_least_300ms"] for item in dataflash["files"]
    )
    conclusion = []
    if dataflash["first_critical_message"]:
        conclusion.append(
            "DataFlash first critical event is recorded independently; do not use the ROS adapter healthy flag as proof of EKF acceptance."
        )
    if raw.get("frame_ids") != output.get("frame_ids"):
        conclusion.append(
            "Raw localization and MAVROS ExternalNav output use different frame IDs; compare through the declared TF/coordinate contract, not direct XYZ subtraction."
        )
    if raw.get("header_stamp_start_s") is not None and output.get("header_stamp_start_s") is not None:
        conclusion.append(
            "Raw odom uses simulation/source stamps while the ExternalNav adapter intentionally retimestamps output at the MAVROS boundary; record and audit arrival time separately."
        )
    return {
        "schema_version": 1,
        "diagnostic": "EXTERNAL_NAV_EKF_ROOT_CAUSE_AUDIT",
        "run_dir": str(run_dir),
        "rosbag": ros,
        "dataflash": dataflash,
        "comparison": {
            "raw_localization": raw,
            "external_nav_output": output,
            "fcu_local_odom": fcu,
            "fcu_vision_ingress": {
                "ros_external_nav_output_count": output_count,
                "dataflash_visp_count": visp_count,
                "dataflash_to_ros_output_ratio": visp_count / output_count
                if output_count else None,
                "dataflash_receive_gaps_at_least_300ms": visp_timeout_gaps,
                "interpretation": (
                    "VISP is written by ArduPilot when an ODOMETRY pose reaches the "
                    "VisualOdom backend. A ROS output count alone does not prove FCU ingress."
                ),
            },
            "first_ros_fcu_fault": ros["status_text"]["first_fault"],
            "first_dataflash_fault": dataflash["first_critical_message"],
            "attitude": {
                "raw_vs_external_nav": _orientation_comparison(raw_samples, output_samples),
                "raw_vs_fcu_local_odom": _orientation_comparison(raw_samples, fcu_samples),
                "interpretation": "Nearest record-time quaternion differences are diagnostic only; frame/axis equivalence must be established from TF and FCU configuration.",
            },
        },
        "conclusion": conclusion,
        "root_cause_status": "DIAGNOSTIC_EVIDENCE_ONLY",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = build_report(args.run_dir.resolve())
    output = args.output or (args.run_dir / "external-nav-health-diagnostic.json")
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(output),
        "first_dataflash_fault": report["comparison"]["first_dataflash_fault"],
        "first_ros_fcu_fault": report["comparison"]["first_ros_fcu_fault"],
        "raw_frames": report["comparison"]["raw_localization"].get("frame_ids", []),
        "external_nav_frames": report["comparison"]["external_nav_output"].get("frame_ids", []),
        "fcu_frames": report["comparison"]["fcu_local_odom"].get("frame_ids", []),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
