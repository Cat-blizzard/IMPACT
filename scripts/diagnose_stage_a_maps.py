#!/usr/bin/env python3
"""Inventory recorded Stage A map evidence; never certify map correctness.

Readable messages make evidence ready for content review. They do not establish
TF/time alignment, correct occupancy/inflation, or reachable free space.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import yaml

EXPECTED = {
    "/localization/odom": "nav_msgs/msg/Odometry",
    "/impact/information_cloud": "sensor_msgs/msg/PointCloud2",
    "/integrity/information_map": "xq_sim_interfaces/msg/InformationMap",
    "/cloud_registered": "sensor_msgs/msg/PointCloud2",
    "/xq/p5/navigation_map": "nav_msgs/msg/OccupancyGrid",
    "/grid_map/occupancy_inflate": "sensor_msgs/msg/PointCloud2",
    "/impact/position_cmd": "quadrotor_msgs/msg/PositionCommand",
    "/impact/authorization": "xq_sim_interfaces/msg/TrajectoryAuthorization",
    "/uav1/mavros/local_position/odom": "nav_msgs/msg/Odometry",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}
MAP_TOPICS = (
    "/impact/information_cloud", "/integrity/information_map",
    "/cloud_registered", "/xq/p5/navigation_map", "/grid_map/occupancy_inflate",
)


def file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1048576), b""):
            digest.update(block)
    return digest.hexdigest()


def audit(run_dir):
    run = Path(run_dir).resolve()
    bag = run / "rosbag"
    metadata = bag / "metadata.yaml"
    errors, files, topics = [], [], {}
    metadata_hash = None
    try:
        metadata_hash = file_hash(metadata)
        info = yaml.safe_load(metadata.read_text())["rosbag2_bagfile_information"]
        if info["storage_identifier"] != "sqlite3":
            raise ValueError("only sqlite3 bag evidence is supported by this inventory")
        relative_paths = info["relative_file_paths"]
        if not isinstance(relative_paths, list) or not relative_paths:
            raise ValueError("bag metadata has no storage files")
        seen = set()
        for relative in relative_paths:
            path = (bag / relative).resolve()
            if not path.is_relative_to(bag.resolve()) or path in seen:
                raise ValueError("bag metadata contains an escaping or duplicate path")
            seen.add(path)
            with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as connection:
                # Read actual stored counts/types; metadata text and arbitrary map
                # filenames are not evidence of recorded map messages.
                rows = connection.execute(
                    "SELECT t.name, t.type, COUNT(m.id) FROM topics t "
                    "LEFT JOIN messages m ON m.topic_id=t.id GROUP BY t.id"
                ).fetchall()
            for name, message_type, count in rows:
                entry = topics.setdefault(name, {"types": [], "message_count": 0})
                if message_type not in entry["types"]:
                    entry["types"].append(message_type)
                entry["message_count"] += count
            files.append({"path": path.relative_to(run).as_posix(),
                          "bytes": path.stat().st_size, "sha256": file_hash(path)})
    except (OSError, ValueError, KeyError, TypeError, sqlite3.Error, yaml.YAMLError) as error:
        errors.append(str(error))
    recorded = {name: topics.get(name, {}).get("message_count", 0) > 0
                and topics.get(name, {}).get("types") == [kind]
                for name, kind in EXPECTED.items()}
    ready = not errors and bool(files) and all(recorded.values())
    return {
        "schema_version": 2,
        "gate": "STAGE_A_SHARED_MAP_OFFLINE_AUDIT",
        "run": str(run),
        "status": "READY_FOR_REVIEW" if ready else "UNVERIFIED",
        "map_content_verified": False,
        "metadata_sha256": metadata_hash,
        "bag_files": files,
        "topics": topics,
        "topics_present": recorded,
        "artifacts": {
            "rosbag_metadata": metadata.is_file(),
            "information_map_artifact": recorded["/impact/information_cloud"]
                and recorded["/integrity/information_map"],
            "ego_collision_map_artifact": recorded["/grid_map/occupancy_inflate"],
        },
        "errors": errors,
        "limitations": [
            "Recorded messages are evidence for review, not proof of map correctness.",
            "Map contents, time/TF alignment, occupancy inflation and reachable components remain unverified.",
            "READY_FOR_REVIEW must not satisfy the Stage A map-content acceptance check.",
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--output")
    args = parser.parse_args()
    result = audit(args.run_dir)
    output = Path(args.output) if args.output else Path(args.run_dir) / "stage-a-map-audit.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
