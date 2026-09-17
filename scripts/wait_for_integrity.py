#!/usr/bin/env python3
"""Wait for one DirectionalIntegrity sample and preserve discovery evidence."""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import rclpy
from xq_sim_interfaces.msg import DirectionalIntegrity
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def endpoint_info(endpoint):
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


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/integrity/directional")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--diagnostics", type=Path, required=True)
    args = parser.parse_args()
    started = time.monotonic()
    diagnostics = {
        "topic": args.topic,
        "timeout_s": max(0.1, args.timeout),
        "node_name": "impact_wait_for_integrity",
        "pid": os.getpid(),
        "received": False,
    }
    node = None
    message = None
    try:
        rclpy.init()
        node = rclpy.create_node("impact_wait_for_integrity")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        received = []
        node.create_subscription(DirectionalIntegrity, args.topic, received.append, qos)
        deadline = started + diagnostics["timeout_s"]
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.5, max(0.0, deadline - time.monotonic())))
            if received:
                message = received[0]
                break
        diagnostics.update({
            "elapsed_s": round(time.monotonic() - started, 6),
            "publisher_count": node.count_publishers(args.topic),
            "subscription_count": node.count_subscribers(args.topic),
            "publishers": [endpoint_info(x) for x in node.get_publishers_info_by_topic(args.topic)],
            "subscriptions": [endpoint_info(x) for x in node.get_subscriptions_info_by_topic(args.topic)],
        })
        if message is None:
            diagnostics["error"] = "timeout_waiting_for_integrity"
            return 1
        diagnostics["received"] = True
        stamp = message.header.stamp
        write_json(args.output, {"topic": args.topic, "stamp": {
            "sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "lambda_min": float(message.lambda_min),
            "effective_points": int(message.effective_points)})
        return 0
    except Exception as error:
        diagnostics["elapsed_s"] = round(time.monotonic() - started, 6)
        diagnostics["error"] = f"{type(error).__name__}: {error}"
        return 2
    finally:
        write_json(args.diagnostics, diagnostics)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
