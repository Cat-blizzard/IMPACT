#!/usr/bin/env python3
"""Offline Stage A map artifact audit; never publishes or changes a run."""
from __future__ import annotations
import argparse
import json
from pathlib import Path

EXPECTED = (
    "/localization/odom",
    "/impact/information_cloud",
    "/impact/position_cmd",
    "/impact/authorization",
    "/uav1/mavros/local_position/odom",
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--output")
    args = parser.parse_args()
    run = Path(args.run_dir).resolve()
    metadata = next(iter(run.glob("rosbag/metadata.yaml")), None)
    graph_files = list(run.glob("*topic-graph.txt"))
    text = "\n".join(p.read_text(errors="replace") for p in graph_files)
    if metadata and metadata.is_file():
        text += "\n" + metadata.read_text(errors="replace")
    present = {topic: topic in text for topic in EXPECTED}
    result = {
        "schema_version": 1,
        "gate": "STAGE_A_SHARED_MAP_OFFLINE_AUDIT",
        "run": str(run),
        "artifacts": {
            "rosbag_metadata": bool(metadata),
            "topic_graph_files": [str(p) for p in graph_files],
            "information_map_artifact": any(run.glob("*information*map*")),
            "ego_collision_map_artifact": any(run.glob("*ego*occupancy*") ) or any(run.glob("*collision*map*")),
        },
        "topics_present": present,
        "status": "UNVERIFIED" if not all(present.values()) else "READY_FOR_REVIEW",
        "limitations": [
            "Topic presence does not prove map correctness.",
            "Review time alignment, TF, occupancy inflation, reachable components and frontier rejection reasons from raw bag.",
            "A selected goal cannot by itself clear the historical reachable_free_cells=1 issue.",
        ],
    }
    output = Path(args.output) if args.output else run / "stage-a-map-audit.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
