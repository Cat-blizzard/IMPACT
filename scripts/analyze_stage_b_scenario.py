#!/usr/bin/env python3
"""Audit whether recorded Stage B worlds produced their intended conditions."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = ("/localization/odom", "/localization/geometry", "/integrity/directional")
AXES = "xyz"


def _stamp(message) -> float:
    return float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)


def _axis(vector) -> str:
    values = np.abs(np.asarray(vector, dtype=float).reshape(3))
    return AXES[int(np.argmax(values))]


def _stats(values: list[float]) -> dict:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return {
        "samples": len(finite),
        "minimum": min(finite, default=None),
        "median": statistics.median(finite) if finite else None,
        "maximum": max(finite, default=None),
    }


def static_observability(scenario: dict, *, lidar_range_m: float,
                         vertical_min_rad: float, vertical_max_rad: float) -> dict:
    start = np.asarray(scenario["start_world_m"], dtype=float)
    goal = np.asarray(scenario["goal_lio_m"], dtype=float)
    low_x, high_x = sorted((float(start[0]), float(goal[0])))
    boxes = scenario["boxes"]
    caps = []
    ceilings = []
    floors = []
    for box in boxes:
        center = np.asarray(box["center"], dtype=float)
        size = np.asarray(box["size"], dtype=float)
        if size[0] <= 0.5 and size[1] >= 3.0 and size[2] >= 2.0:
            distance = max(low_x - center[0], 0.0, center[0] - high_x)
            caps.append({"name": box["name"], "minimum_path_distance_m": float(distance),
                         "within_lidar_range": bool(distance <= lidar_range_m)})
        if size[2] <= 0.5 and size[0] >= 10.0 and center[2] > goal[2]:
            ceilings.append(box["name"])
        if size[2] <= 0.5 and size[0] >= 10.0 and center[2] < start[2]:
            floors.append(box["name"])
    operational_height = max(float(goal[2]), float(start[2]))
    floor_drop = operational_height - max(
        (float(box["center"][2]) + 0.5 * float(box["size"][2])
         for box in boxes if box["name"] in floors),
        default=0.0,
    )
    downward = abs(min(float(vertical_min_rad), 0.0))
    floor_intersection = floor_drop / math.tan(downward) if downward > 0.0 else math.inf
    return {
        "lidar_range_m": float(lidar_range_m),
        "vertical_fov_rad": [float(vertical_min_rad), float(vertical_max_rad)],
        "longitudinal_caps": caps,
        "longitudinal_cap_visible_from_route": any(item["within_lidar_range"] for item in caps),
        "ceiling_surfaces": ceilings,
        "ceiling_present": bool(ceilings),
        "floor_surfaces": floors,
        "approximate_floor_intersection_range_m": float(floor_intersection),
        "floor_intersection_within_lidar_range": bool(floor_intersection <= lidar_range_m),
    }


def _sensor_contract(run: Path) -> tuple[float, float, float]:
    model = run / "models" / "xq_iris_mid360_ardupilot" / "model.sdf"
    if not model.is_file():
        raise FileNotFoundError(f"run-local LiDAR model is missing: {model}")
    root = ET.parse(model).getroot()
    lidar = root.find(".//sensor[@name='xq_mid360_lidar']/lidar")
    if lidar is None:
        raise ValueError("run-local Mid-360 lidar declaration is missing")
    return (float(lidar.findtext("range/max")),
            float(lidar.findtext("scan/vertical/min_angle")),
            float(lidar.findtext("scan/vertical/max_angle")))


def _events(run: Path) -> dict:
    rows = [json.loads(line) for line in (run / "events.jsonl").read_text(encoding="utf-8").splitlines()
            if line.strip()]
    certifications = [row for row in rows if row.get("event") == "CERTIFY"]
    margins = [float(row["margin"]) for row in certifications
               if isinstance(row.get("margin"), (int, float))]
    counts = Counter(str(row.get("event")) for row in rows)
    return {
        "certifications": len(certifications),
        "certifications_accepted": sum(bool(row.get("accepted")) for row in certifications),
        "certifications_rejected": sum(not bool(row.get("accepted")) for row in certifications),
        "certification_margin_m": _stats(margins),
        "recovery_forecasts": counts["RECOVERY_FORECAST"],
        "recovery_steps_completed": counts["RECOVERY_STEP_DONE"],
        "new_observations": counts["NEW_OBSERVATION"],
        "recovery_confirmed": counts["RECOVERY_CONFIRMED"],
    }


def _bag(run: Path) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run / "rosbag"), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in TOPICS if topic not in topic_types]
    if missing:
        raise RuntimeError(f"required Stage B topics missing: {missing}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    types = {topic: get_message(topic_types[topic]) for topic in TOPICS}
    odom = None
    geometry = []
    integrity = []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        message = deserialize_message(raw, types[topic])
        if topic == "/localization/odom":
            odom = message
            continue
        if odom is None or abs(_stamp(message) - _stamp(odom)) > 0.25:
            continue
        position = odom.pose.pose.position
        item = {"stamp_s": _stamp(message), "position": [position.x, position.y, position.z]}
        if topic == "/localization/geometry":
            information = np.asarray(message.information_matrix, dtype=float).reshape(3, 3)
            values, vectors = np.linalg.eigh(0.5 * (information + information.T))
            item.update(weak_axis=_axis(vectors[:, 0]), lambda_min=float(values[0]),
                        condition_number=float(values[-1] / max(values[0], 1.0e-9)),
                        effective_points=int(message.effective_points))
            geometry.append(item)
        else:
            item.update(weak_axis=_axis(message.weak_direction_map),
                        lambda_min=float(message.lambda_min),
                        condition_number=float(message.condition_number),
                        weak_pl_m=float(message.weak_direction_protection_level))
            integrity.append(item)

    def summarize(records: list[dict], include_pl: bool = False) -> dict:
        airborne = [row for row in records if float(row["position"][2]) >= 0.8]
        selected = airborne or records
        result = {
            "samples": len(records),
            "airborne_samples": len(airborne),
            "weak_axis_counts_xyz": {axis: sum(row["weak_axis"] == axis for row in selected)
                                     for axis in AXES},
            "lambda_min": _stats([row["lambda_min"] for row in selected]),
            "condition_number": _stats([row["condition_number"] for row in selected]),
        }
        if records and "effective_points" in records[0]:
            result["effective_points"] = _stats([row["effective_points"] for row in selected])
        if include_pl:
            result["weak_pl_m"] = _stats([row["weak_pl_m"] for row in selected])
        return result

    return {"geometry": summarize(geometry), "integrity": summarize(integrity, True)}


def analyze_run(run: Path) -> dict:
    scenario = json.loads((run / "scenario.json").read_text(encoding="utf-8"))
    lidar_range, vertical_min, vertical_max = _sensor_contract(run)
    streams = _bag(run)
    events = _events(run)
    axes = streams["geometry"]["weak_axis_counts_xyz"]
    dominant_axis = max(AXES, key=lambda axis: axes[axis]) if sum(axes.values()) else None
    hypothesis_checks = {
        "longitudinal_direction_is_empirically_weak": dominant_axis == "x",
        "mission_certification_rejected": events["certifications_rejected"] > 0,
        "online_recovery_triggered": events["recovery_forecasts"] > 0,
    }
    if scenario["scenario"] == "normal":
        relevant = {"mission_certification_never_rejected": events["certifications_rejected"] == 0}
    elif scenario["scenario"] == "recoverable":
        relevant = hypothesis_checks
    else:
        relevant = {key: hypothesis_checks[key] for key in
                    ("longitudinal_direction_is_empirically_weak", "mission_certification_rejected")}
    return {
        "schema_version": 1,
        "analysis": "STAGE_B_SCENARIO_CONDITION_AUDIT",
        "status": "DIAGNOSTIC_COMPLETE",
        "scenario_hypothesis": "SUPPORTED" if all(relevant.values()) else "CONTRADICTED",
        "run": str(run.resolve()),
        "scenario": scenario["scenario"],
        "seed": scenario["seed"],
        "static_observability": static_observability(
            scenario, lidar_range_m=lidar_range,
            vertical_min_rad=vertical_min, vertical_max_rad=vertical_max),
        "recorded_streams": streams,
        "recorded_events": events,
        "hypothesis_checks": relevant,
        "limitations": [
            "This is a diagnostic of recorded conditions, not a task or recovery acceptance gate.",
            "Weak-axis majority is categorical evidence; it does not replace calibrated PL and margin checks.",
            "Static visibility checks do not model occlusion or point density; recorded FAST-LIO geometry is authoritative.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    reports = [analyze_run(path.resolve()) for path in args.run]
    result = {"schema_version": 1, "analysis": "STAGE_B_SCENARIO_COMPARISON",
              "status": "DIAGNOSTIC_COMPLETE", "runs": reports}
    if args.output:
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
