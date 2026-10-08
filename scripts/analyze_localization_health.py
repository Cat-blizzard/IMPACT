#!/usr/bin/env python3
"""Offline phase-by-phase localization diagnosis, including before takeoff."""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from xq_autonomy.sitl_evaluator_node import rotation


ODOM = "/localization/odom"
TRUTH = "/xq/eval/p5/ground_truth"
GEOMETRY = "/localization/geometry"
INTEGRITY = "/integrity/directional"
STAGE = "/impact/mission_stage"
TOPICS = (ODOM, TRUTH, GEOMETRY, INTEGRITY, STAGE)


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def nearest(sequence, times, value, tolerance):
    index = bisect.bisect_left(times, value)
    candidates = sequence[max(0, index - 1):index + 1]
    selected = min(candidates, key=lambda row: abs(row[0] - value)) if candidates else None
    return selected if selected and abs(selected[0] - value) <= tolerance else None


def stats(values):
    finite = np.asarray([value for value in values if math.isfinite(value)], float)
    return dict(samples=len(finite), minimum=float(finite.min()) if len(finite) else None,
                median=float(np.median(finite)) if len(finite) else None,
                maximum=float(finite.max()) if len(finite) else None)


def matrix_metrics(message):
    matrix = np.asarray(message.information_matrix, float).reshape(3, 3)
    values, vectors = np.linalg.eigh(0.5 * (matrix + matrix.T))
    return dict(information_x=float(matrix[0, 0]), lambda_min=float(values[0]),
                weak_fraction=float(values[0] / max(float(np.trace(matrix)), 1e-9)),
                condition_number=float(values[-1] / max(values[0], 1e-9)),
                weak_direction=vectors[:, 0].tolist(), effective_points=int(message.effective_points))


def analyze(run):
    record = json.loads((run / "run.json").read_text())
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run / "rosbag"), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    declared = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = set(TOPICS) - set(declared)
    if missing:
        raise ValueError(f"missing diagnostic topics: {sorted(missing)}")
    types = {topic: get_message(declared[topic]) for topic in TOPICS}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    streams = {topic: [] for topic in TOPICS}
    while reader.has_next():
        topic, payload, received_ns = reader.read_next()
        message = deserialize_message(payload, types[topic])
        if topic == STAGE:
            data = json.loads(message.data)
            if data.get("session_id") != record["session_id"]:
                continue
            streams[topic].append((received_ns, data))
        else:
            streams[topic].append((stamp(message), message, received_ns))
    for sequence in streams.values():
        sequence.sort(key=lambda row: row[0])
    times = {topic: [row[0] for row in sequence] for topic, sequence in streams.items()}
    transform = None
    alignment = None
    samples = []
    dropped_truth_pairs = 0
    for observed_stamp, odom, received_ns in streams[ODOM]:
        truth = nearest(streams[TRUTH], times[TRUTH], observed_stamp, 0.05)
        if truth is None:
            dropped_truth_pairs += 1
            continue
        estimate = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y,
                             odom.pose.pose.position.z])
        pose = truth[1].pose.pose
        actual = np.array([pose.position.x, pose.position.y, pose.position.z])
        if transform is None:
            try:
                rot = rotation(pose.orientation) @ rotation(odom.pose.pose.orientation).T
            except ValueError:
                continue
            offset = actual - rot @ estimate
            transform = rot, offset
            alignment = dict(stamp_s=observed_stamp, rotation=rot.tolist(), offset_m=offset.tolist())
        stage_index = bisect.bisect_right(times[STAGE], received_ns) - 1
        phase = streams[STAGE][stage_index][1]["phase"] if stage_index >= 0 else "BEFORE_STAGE"
        rot, offset = transform
        covariance = np.asarray(odom.pose.covariance).reshape(6, 6)[:3, :3]
        item = dict(stamp_s=observed_stamp, phase=phase, truth=actual.tolist(),
                    estimated_world=(rot @ estimate + offset).tolist(),
                    truth_pair_gap_s=abs(truth[0] - observed_stamp),
                    error_3d_m=float(np.linalg.norm(rot @ estimate + offset - actual)),
                    lio_variance_x_m2=float(covariance[0, 0]))
        for topic, key in ((GEOMETRY, "raw"), (INTEGRITY, "memory")):
            matched = nearest(streams[topic], times[topic], observed_stamp, 0.25)
            if matched:
                metrics = matrix_metrics(matched[1])
                metrics["stamp_gap_s"] = abs(matched[0] - observed_stamp)
                if topic == INTEGRITY:
                    metrics["variance_x_m2"] = float(matched[1].integrity_covariance[0])
                item[key] = metrics
        samples.append(item)
    summaries = {}
    for phase in dict.fromkeys(row["phase"] for row in samples):
        selected = [row for row in samples if row["phase"] == phase]
        summary = dict(samples=len(selected), start_s=selected[0]["stamp_s"],
                       end_s=selected[-1]["stamp_s"],
                       error_3d_m=stats([row["error_3d_m"] for row in selected]),
                       truth_x_m=stats([row["truth"][0] for row in selected]))
        for key in ("raw", "memory"):
            summary[key] = {metric: stats([row[key][metric] for row in selected if key in row])
                            for metric in ("information_x", "lambda_min", "weak_fraction",
                                           "condition_number", "effective_points")}
        summaries[phase] = summary
    inputs = [run / name for name in ("run.json", "mission.json", "evaluation.json", "scenario.json")]
    inputs += sorted((run / "rosbag").glob("*.db3"))
    inputs += [run / "rosbag" / "metadata.yaml"]
    return dict(run=str(run), source_sha256=record["source_sha256"], status=record["status"],
                fixed_initial_pose_alignment=alignment, dropped_truth_pairs=dropped_truth_pairs,
                phase_summary=summaries,
                first_error_crossings={str(limit): next((row for row in samples
                    if row["error_3d_m"] > limit), None) for limit in (0.45, 1., 2., 10.)},
                input_sha256={str(path.relative_to(run)): digest(path) for path in inputs if path.is_file()})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = dict(analysis="PHASE_LOCALIZATION_HEALTH_DIAGNOSTIC", acceptance_evidence=False,
                  limitations=["Truth is used offline only, never as a controller input.",
                               "Phase labels follow recorded receipt order; header-stamp joins are bounded.",
                               "Fixed initial alignment may differ slightly from live callback ordering.",
                               "Information eigenvalues alone do not prove localization accuracy."],
                  runs=[analyze(run.resolve()) for run in args.runs])
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    for run in report["runs"]:
        print(Path(run["run"]).name, run["status"])
        for phase, summary in run["phase_summary"].items():
            print(phase, "error", summary["error_3d_m"],
                  "raw weak fraction", summary["raw"]["weak_fraction"])


if __name__ == "__main__":
    main()
