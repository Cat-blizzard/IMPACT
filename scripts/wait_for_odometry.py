#!/usr/bin/env python3
"""Wait for the first Odometry sample and preserve discovery diagnostics."""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _endpoint_info(endpoint: Any) -> dict[str, Any]:
    qos = endpoint.qos_profile
    return {
        "node_name": endpoint.node_name,
        "node_namespace": endpoint.node_namespace,
        "topic_type": endpoint.topic_type,
        "endpoint_type": endpoint.endpoint_type,
        "qos": {
            "reliability": str(qos.reliability),
            "durability": str(qos.durability),
            "history": str(qos.history),
            "depth": int(qos.depth),
        },
    }


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--diagnostics", type=Path, required=True)
    args = parser.parse_args()

    timeout_s = max(0.1, args.timeout)
    started = time.monotonic()
    diagnostics: dict[str, Any] = {
        "topic": args.topic,
        "timeout_s": timeout_s,
        "node_name": "impact_wait_for_odometry",
        "pid": os.getpid(),
        "qos": {
            "reliability": "RELIABLE",
            "durability": "VOLATILE",
            "history": "KEEP_LAST",
            "depth": 20,
        },
        "received": False,
    }

    node = None
    try:
        rclpy.init()
        node = rclpy.create_node("impact_wait_for_odometry")
    except Exception as error:
        diagnostics["elapsed_s"] = round(time.monotonic() - started, 6)
        diagnostics["error"] = f"{type(error).__name__}: {error}"
        _write_json(args.diagnostics, diagnostics)
        if rclpy.ok():
            rclpy.shutdown()
        return 2
    sample: Odometry | None = None

    def on_message(message: Odometry) -> None:
        nonlocal sample
        if sample is None:
            sample = message

    qos = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    node.create_subscription(Odometry, args.topic, on_message, qos)
    deadline = started + timeout_s
    return_code = 2
    try:
        while rclpy.ok() and sample is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.5, max(0.0, deadline - time.monotonic())))

        diagnostics["elapsed_s"] = round(time.monotonic() - started, 6)
        diagnostics["publisher_count"] = node.count_publishers(args.topic)
        diagnostics["subscription_count"] = node.count_subscribers(args.topic)
        diagnostics["publishers"] = [
            _endpoint_info(endpoint) for endpoint in node.get_publishers_info_by_topic(args.topic)
        ]
        diagnostics["subscriptions"] = [
            _endpoint_info(endpoint) for endpoint in node.get_subscriptions_info_by_topic(args.topic)
        ]
        if sample is not None:
            diagnostics["received"] = True
            diagnostics["sample"] = {
                "stamp": {
                    "sec": int(sample.header.stamp.sec),
                    "nanosec": int(sample.header.stamp.nanosec),
                },
                "frame_id": sample.header.frame_id,
                "child_frame_id": sample.child_frame_id,
            }
            _write_json(args.output, {"topic": args.topic, **diagnostics["sample"]})
            return_code = 0
        else:
            diagnostics["error"] = "timeout_waiting_for_odometry"
            return_code = 1
    except Exception as error:  # preserve the failure context for the launcher
        diagnostics["elapsed_s"] = round(time.monotonic() - started, 6)
        diagnostics["error"] = f"{type(error).__name__}: {error}"
    finally:
        _write_json(args.diagnostics, diagnostics)
        node.destroy_node()
        rclpy.shutdown()

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
