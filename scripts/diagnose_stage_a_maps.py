#!/usr/bin/env python3
"""Audit Stage A map messages, alignment, inflation and reachable free space."""
from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
import yaml


EXPECTED = {
    "/localization/odom": "nav_msgs/msg/Odometry",
    "/impact/information_cloud": "sensor_msgs/msg/PointCloud2",
    "/integrity/information_map": "xq_sim_interfaces/msg/InformationMap",
    "/xq/p5/cloud_map": "sensor_msgs/msg/PointCloud2",
    "/xq/p5/navigation_map": "nav_msgs/msg/OccupancyGrid",
    "/grid_map/occupancy_inflate": "sensor_msgs/msg/PointCloud2",
    "/impact/position_cmd": "quadrotor_msgs/msg/PositionCommand",
    "/impact/authorization": "xq_sim_interfaces/msg/TrajectoryAuthorization",
    "/uav1/mavros/local_position/odom": "nav_msgs/msg/Odometry",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}
MAP_TOPICS = (
    "/impact/information_cloud", "/integrity/information_map",
    "/xq/p5/cloud_map", "/xq/p5/navigation_map", "/grid_map/occupancy_inflate",
)
MIN_SURFELS = 12
MIN_REACHABLE_FREE_CELLS = 100
MAX_MAP_ODOM_STAMP_GAP_S = 1.0


def file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1048576), b""):
            digest.update(block)
    return digest.hexdigest()


def _stamp(header):
    return float(header.stamp.sec) + 1.0e-9 * float(header.stamp.nanosec)


def _finite(values):
    return bool(np.isfinite(np.asarray(values, dtype=float)).all())


def _cloud_xyz(message, maximum=5000):
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=float), 0
    if any(int(fields[name].datatype) != 7 for name in ("x", "y", "z")):  # FLOAT32
        return np.empty((0, 3), dtype=float), 0
    count = int(message.width * message.height)
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype({"names": ("x", "y", "z"),
        "formats": (endian + "f4", endian + "f4", endian + "f4"),
        "offsets": tuple(fields[name].offset for name in ("x", "y", "z")),
        "itemsize": int(message.point_step)})
    records = np.frombuffer(message.data, dtype=dtype, count=count)
    points = np.column_stack((records["x"], records["y"], records["z"])).astype(float)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) > maximum:
        points = points[::max(1, math.ceil(len(points) / maximum))][:maximum]
    return points, count


def _cloud_summary(message):
    points, total = _cloud_xyz(message)
    return {
        "frame_id": str(message.header.frame_id),
        "stamp_s": _stamp(message.header),
        "total_points": total,
        "sampled_finite_points": len(points),
        "bounds_min": points.min(axis=0).tolist() if len(points) else None,
        "bounds_max": points.max(axis=0).tolist() if len(points) else None,
        "_points": points,
    }


def _information_summary(message):
    positions = np.asarray([(p.x, p.y, p.z) for p in message.positions], dtype=float).reshape((-1, 3))
    normals = np.asarray([(n.x, n.y, n.z) for n in message.normals], dtype=float).reshape((-1, 3))
    arrays = [positions, normals, np.asarray(message.static_confidence, dtype=float),
              np.asarray(message.geometry_quality, dtype=float), np.asarray(message.last_seen_s, dtype=float)]
    aligned = len({len(value) for value in arrays}) == 1
    finite = all(_finite(value) for value in arrays)
    valid = (bool(message.valid) and not bool(message.ground_truth_used) and aligned and finite
             and len(positions) >= MIN_SURFELS)
    return {"frame_id": str(message.header.frame_id), "stamp_s": _stamp(message.header),
        "surfels": len(positions), "arrays_aligned": aligned, "finite": finite,
        "valid_flag": bool(message.valid), "ground_truth_used": bool(message.ground_truth_used),
        "source": str(message.source), "content_valid": valid}


def _nearest_free(grid, ix, iy, maximum_radius=5):
    height, width = grid.shape
    candidates = []
    for radius in range(maximum_radius + 1):
        for y in range(max(0, iy-radius), min(height, iy+radius+1)):
            for x in range(max(0, ix-radius), min(width, ix+radius+1)):
                if grid[y, x] == 0:
                    candidates.append((abs(x-ix) + abs(y-iy), x, y))
        if candidates:
            return min(candidates)[1:]
    return None


def reachable_free_cells(data, start):
    """Return the four-connected component containing a free start cell."""
    grid = np.asarray(data)
    ix, iy = start
    if grid.ndim != 2 or not (0 <= iy < grid.shape[0] and 0 <= ix < grid.shape[1]):
        return set()
    if grid[iy, ix] != 0:
        return set()
    visited = {(ix, iy)}
    queue = deque([(ix, iy)])
    while queue:
        x, y = queue.popleft()
        for nx, ny in ((x-1, y), (x+1, y), (x, y-1), (x, y+1)):
            if (0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]
                    and grid[ny, nx] == 0 and (nx, ny) not in visited):
                visited.add((nx, ny))
                queue.append((nx, ny))
    return visited


def _grid_summary(message, record_ns, odometry, goal_xy):
    width, height = int(message.info.width), int(message.info.height)
    resolution = float(message.info.resolution)
    values = np.asarray(message.data, dtype=np.int16)
    structurally_valid = width > 0 and height > 0 and resolution > 0 and len(values) == width * height
    result = {"frame_id": str(message.header.frame_id), "stamp_s": _stamp(message.header),
        "width": width, "height": height, "resolution_m": resolution,
        "structurally_valid": structurally_valid}
    if not structurally_valid:
        return result
    grid = values.reshape((height, width))
    origin_x = float(message.info.origin.position.x)
    origin_y = float(message.info.origin.position.y)
    same_frame = [item for item in odometry if item["frame_id"] == result["frame_id"]]
    nearest = min(same_frame, key=lambda item: abs(item["record_ns"] - record_ns)) if same_frame else None
    result.update(free_cells=int(np.count_nonzero(grid == 0)),
                  occupied_cells=int(np.count_nonzero(grid >= 50)),
                  unknown_cells=int(np.count_nonzero(grid < 0)),
                  odom_record_gap_s=abs(nearest["record_ns"] - record_ns) / 1e9 if nearest else None)
    if nearest is None:
        result.update(start_cell=None, goal_cell=None, reachable_free_cells=0,
                      goal_reachable=False, start_snap_m=None, goal_snap_m=None)
        return result
    def cell(x, y):
        return int(math.floor((x-origin_x)/resolution)), int(math.floor((y-origin_y)/resolution))
    raw_start = cell(nearest["x"], nearest["y"])
    raw_goal = cell(*goal_xy)
    start = _nearest_free(grid, *raw_start)
    goal = _nearest_free(grid, *raw_goal)
    component = reachable_free_cells(grid, start) if start else set()
    result.update(start_cell=list(start) if start else None, goal_cell=list(goal) if goal else None,
        start_snap_m=resolution * math.dist(start, raw_start) if start else None,
        goal_snap_m=resolution * math.dist(goal, raw_goal) if goal else None,
        reachable_free_cells=len(component), goal_reachable=bool(goal and goal in component))
    return result


def _inflation_alignment(source, inflated):
    if source is None or inflated is None or not len(source) or not len(inflated):
        return {"aligned": False, "aligned_fraction_within_1m": 0.0,
                "expanded_points_beyond_0_15m": 0}
    source = source[:2500]
    inflated = inflated[:2500]
    minima = []
    for start in range(0, len(inflated), 64):
        delta = inflated[start:start+64, None, :] - source[None, :, :]
        minima.extend(np.sqrt(np.sum(delta * delta, axis=2)).min(axis=1).tolist())
    distances = np.asarray(minima)
    aligned_fraction = float(np.mean(distances <= 1.0))
    expanded = int(np.count_nonzero((distances > 0.15) & (distances <= 1.0)))
    return {"aligned": aligned_fraction >= 0.5 and expanded > 0,
        "aligned_fraction_within_1m": aligned_fraction,
        "expanded_points_beyond_0_15m": expanded,
        "maximum_nearest_source_distance_m": float(distances.max())}


def _goal(run):
    try:
        values = json.loads((run / "config.json").read_text())["goal_lio_m"]
    except (OSError, ValueError, KeyError, TypeError):
        try:
            values = json.loads((run / "run.json").read_text())["configuration"]["goal_lio_m"]
        except (OSError, ValueError, KeyError, TypeError):
            values = [12.0, 0.0, 2.0]
    return [float(values[0]), float(values[1]), float(values[2])]


def _decode_content(run):
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run / "rosbag"), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    classes, errors = {}, []
    odometry, grids, information, clouds, transforms = [], [], [], {topic: [] for topic in MAP_TOPICS}, []
    selected = set(MAP_TOPICS) | {"/localization/odom", "/tf_static"}
    while reader.has_next():
        topic, raw, record_ns = reader.read_next()
        if topic not in selected:
            continue
        try:
            classes.setdefault(topic, get_message(types[topic]))
            message = deserialize_message(raw, classes[topic])
            if topic == "/localization/odom":
                odometry.append({"record_ns": int(record_ns), "stamp_s": _stamp(message.header),
                    "frame_id": str(message.header.frame_id), "x": float(message.pose.pose.position.x),
                    "y": float(message.pose.pose.position.y)})
            elif topic == "/integrity/information_map":
                information.append(_information_summary(message))
            elif topic == "/xq/p5/navigation_map":
                grids.append((message, int(record_ns)))
            elif topic == "/tf_static":
                for transform in message.transforms:
                    t, q = transform.transform.translation, transform.transform.rotation
                    transforms.append({"parent": str(transform.header.frame_id),
                        "child": str(transform.child_frame_id), "translation": [t.x, t.y, t.z],
                        "quaternion_xyzw": [q.x, q.y, q.z, q.w]})
            else:
                clouds[topic].append(_cloud_summary(message))
        except Exception as error:  # Preserve evidence and continue across individual corrupt messages.
            if len(errors) < 20:
                errors.append(f"{topic}: {type(error).__name__}: {error}")
    goal = _goal(run)
    grid_summaries = [_grid_summary(message, record, odometry, goal[:2]) for message, record in grids]
    navigation = max(grid_summaries, key=lambda item: (item.get("goal_reachable", False),
        item.get("reachable_free_cells", 0), item.get("free_cells", 0)), default={})
    best_information = max(information, key=lambda item: (item["content_valid"], item["surfels"]), default={})
    best_clouds = {topic: max(items, key=lambda item: item["sampled_finite_points"], default={})
                   for topic, items in clouds.items()}
    source = best_clouds.get("/xq/p5/cloud_map", {}).get("_points")
    inflated = best_clouds.get("/grid_map/occupancy_inflate", {}).get("_points")
    inflation = _inflation_alignment(source, inflated)
    static_alignment = any(item["parent"] == "map" and item["child"] == "xq_lio_map"
        and np.linalg.norm(item["translation"]) <= 1e-6
        and abs(abs(item["quaternion_xyzw"][3]) - 1.0) <= 1e-6
        and np.linalg.norm(item["quaternion_xyzw"][:3]) <= 1e-6 for item in transforms)
    odom_stamps = [item["stamp_s"] for item in odometry if math.isfinite(item["stamp_s"])]
    map_summaries = [best_information, navigation] + list(best_clouds.values())
    stamp_gaps = {item.get("frame_id", "") + ":" + str(index):
        min((abs(item["stamp_s"] - value) for value in odom_stamps), default=None)
        for index, item in enumerate(map_summaries) if item and item.get("stamp_s", 0.0) > 0.0}
    time_aligned = bool(stamp_gaps) and all(value is not None and value <= MAX_MAP_ODOM_STAMP_GAP_S
                                            for value in stamp_gaps.values())
    frames = [item.get("frame_id") for item in map_summaries if item]
    frame_aligned = bool(frames) and all(frame == "xq_lio_map" for frame in frames) and static_alignment
    checks = {
        "information_map_valid": bool(best_information.get("content_valid")),
        "information_cloud_nonempty": best_clouds.get("/impact/information_cloud", {}).get("sampled_finite_points", 0) >= MIN_SURFELS,
        "frontier_cloud_nonempty": best_clouds.get("/xq/p5/cloud_map", {}).get("sampled_finite_points", 0) >= MIN_SURFELS,
        "navigation_grid_structurally_valid": bool(navigation.get("structurally_valid")),
        "navigation_grid_has_obstacles": navigation.get("occupied_cells", 0) > 0,
        "reachable_free_component": navigation.get("reachable_free_cells", 0) >= MIN_REACHABLE_FREE_CELLS,
        "fixed_goal_reachable": navigation.get("goal_reachable") is True and (navigation.get("goal_snap_m") or 0.0) <= 0.5,
        "ego_inflated_cloud_nonempty": best_clouds.get("/grid_map/occupancy_inflate", {}).get("sampled_finite_points", 0) > 0,
        "ego_inflation_aligned_with_source": inflation["aligned"],
        "map_frames_aligned": frame_aligned,
        "map_stamps_aligned_with_odometry": time_aligned,
    }
    for summary in best_clouds.values():
        summary.pop("_points", None)
    return {"goal_lio_m": goal, "checks": checks, "passed": all(checks.values()),
        "information_map": best_information, "clouds": best_clouds, "navigation_map": navigation,
        "inflation_alignment": inflation, "static_map_alignment": static_alignment,
        "map_to_odom_stamp_gaps_s": stamp_gaps, "decode_errors": errors,
        "criteria": {"minimum_surfels": MIN_SURFELS,
            "minimum_reachable_free_cells": MIN_REACHABLE_FREE_CELLS,
            "maximum_map_odom_stamp_gap_s": MAX_MAP_ODOM_STAMP_GAP_S,
            "maximum_goal_snap_m": 0.5}}


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
            raise ValueError("only sqlite3 bag evidence is supported")
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
                rows = connection.execute("SELECT t.name,t.type,COUNT(m.id) FROM topics t "
                    "LEFT JOIN messages m ON m.topic_id=t.id GROUP BY t.id").fetchall()
            for name, message_type, count in rows:
                entry = topics.setdefault(name, {"types": [], "message_count": 0})
                if message_type not in entry["types"]:
                    entry["types"].append(message_type)
                entry["message_count"] += count
            files.append({"path": path.relative_to(run).as_posix(), "bytes": path.stat().st_size,
                          "sha256": file_hash(path)})
    except (OSError, ValueError, KeyError, TypeError, sqlite3.Error, yaml.YAMLError) as error:
        errors.append(str(error))
    recorded = {name: topics.get(name, {}).get("message_count", 0) > 0
                and topics.get(name, {}).get("types") == [kind] for name, kind in EXPECTED.items()}
    ready = not errors and bool(files) and all(recorded.values())
    content = {"passed": False, "checks": {}, "decode_errors": ["inventory incomplete"]}
    if ready:
        try:
            content = _decode_content(run)
        except Exception as error:
            content = {"passed": False, "checks": {},
                       "decode_errors": [f"{type(error).__name__}: {error}"]}
    verified = ready and content.get("passed") is True
    return {"schema_version": 3, "gate": "STAGE_A_SHARED_MAP_OFFLINE_AUDIT",
        "run": str(run), "status": "MAP_CONTENT_VERIFIED" if verified else
            ("READY_FOR_REVIEW" if ready else "UNVERIFIED"),
        "map_content_verified": verified, "metadata_sha256": metadata_hash,
        "bag_files": files, "topics": topics, "topics_present": recorded,
        "artifacts": {"rosbag_metadata": metadata.is_file(),
            "information_map_artifact": recorded["/impact/information_cloud"] and recorded["/integrity/information_map"],
            "ego_collision_map_artifact": recorded["/grid_map/occupancy_inflate"]},
        "content": content, "errors": errors,
        "limitations": [
            "Verification applies to the recorded Stage A run and fixed goal, not every possible exploration target.",
            "The reachable-component check prevents a one-cell map from passing but is not a P5 exploration acceptance.",
            "Stage A map verification does not replace legacy P5 frontier completion evidence."]}


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
