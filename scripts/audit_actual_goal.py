#!/usr/bin/env python3
"""Reclassify a recorded run using its independent truth trajectory."""

import argparse
import hashlib
import json
import math
from pathlib import Path


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def goal_world(scenario):
    start = scenario["start_world_m"]
    goal = scenario["goal_lio_m"]
    yaw = float(scenario["start_yaw"])
    c, s = math.cos(yaw), math.sin(yaw)
    return [start[0] + c * goal[0] - s * goal[1],
            start[1] + s * goal[0] + c * goal[1],
            start[2] + goal[2]]


def distance(left, right):
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)))


def audit(run, legacy_goal_tolerance_m=None):
    scenario_path = run / "scenario.json"
    telemetry_path = run / "telemetry.jsonl"
    run_path = run / "run.json"
    mission_path = run / "mission.json"
    evaluation_path = run / "evaluation.json"
    required = (scenario_path, telemetry_path, run_path, mission_path, evaluation_path)
    missing = [path.name for path in required if not path.is_file()]
    if missing:
        raise ValueError("missing required evidence: " + ", ".join(missing))
    scenario = read_json(scenario_path)
    if "actual_goal_tolerance_m" in scenario:
        tolerance = float(scenario["actual_goal_tolerance_m"])
        tolerance_source = "scenario.json"
    elif legacy_goal_tolerance_m is not None:
        tolerance = float(legacy_goal_tolerance_m)
        tolerance_source = "explicit legacy command-line value"
    else:
        raise ValueError(
            "legacy scenario lacks actual_goal_tolerance_m; pass --legacy-goal-tolerance-m "
            "only after verifying the value in that run's source"
        )
    if not math.isfinite(tolerance) or tolerance <= 0.:
        raise ValueError("actual_goal_tolerance_m must be finite and positive")
    truth = []
    for number, line in enumerate(telemetry_path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        row = json.loads(line)
        point = row.get("truth")
        if isinstance(point, list) and len(point) == 3 and all(
                isinstance(value, (int, float)) and math.isfinite(value) for value in point):
            truth.append(point)
    if not truth:
        raise ValueError("telemetry has no finite truth samples")
    target = goal_world(scenario)
    distances = [distance(point, target) for point in truth]
    reached = distances[-1] <= tolerance
    original_run = read_json(run_path)
    mission = read_json(mission_path)
    evaluation = read_json(evaluation_path)
    termination = original_run.get("dataflash_termination", {})
    corrected_pass = bool(
        original_run.get("launcher_exit_code") == 0
        and mission.get("status") == "PASS"
        and mission.get("task_success") is True
        and mission.get("termination_confirmed") is True
        and termination.get("confirmed") is True
        and evaluation.get("collision_events") == 0
        and reached
    )
    return {
        "schema_version": 1,
        "gate": "INDEPENDENT_TRUTH_GOAL_CORRECTION",
        "run": str(run.resolve()),
        "status": "PASS" if corrected_pass else "FAIL",
        "task_status": "PASS" if corrected_pass else "FAIL",
        "termination_status": "PASS" if termination.get("confirmed") is True else "FAIL",
        "reason": ("independent truth endpoint reached the frozen goal"
                   if reached else "mission reported GOAL_REACHED but independent truth endpoint missed the frozen goal"),
        "checks": {
            "actual_goal_reached": reached,
            "mission_reported_success": mission.get("task_success") is True,
            "termination_confirmed": termination.get("confirmed") is True,
        },
        "truth_samples": len(truth),
        "truth_goal_world_m": target,
        "final_truth_position_m": truth[-1],
        "minimum_truth_goal_distance_m": min(distances),
        "final_truth_goal_distance_m": distances[-1],
        "actual_goal_tolerance_m": tolerance,
        "actual_goal_tolerance_source": tolerance_source,
        "original_classification": {
            "run_status": original_run.get("status"),
            "evaluation_status": evaluation.get("status"),
            "mission_status": mission.get("status"),
        },
        "original_records_preserved": True,
        "evidence_sha256": {path.name: digest(path) for path in required},
        "limitations": [
            "The correction uses the scenario's frozen start pose and yaw to map its LIO-frame goal into world coordinates.",
            "It does not rewrite the original run, mission, evaluation, rosbag, or DataFlash records.",
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--legacy-goal-tolerance-m", type=float)
    args = parser.parse_args()
    output = args.output or args.run_dir / "acceptance-correction.json"
    report = audit(args.run_dir, args.legacy_goal_tolerance_m)
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "task_status": report["task_status"],
                      "termination_status": report["termination_status"],
                      "final_truth_goal_distance_m": report["final_truth_goal_distance_m"],
                      "output": str(output)}, indent=2))
    return int(report["status"] != "PASS")


if __name__ == "__main__":
    raise SystemExit(main())
