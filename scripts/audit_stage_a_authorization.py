#!/usr/bin/env python3
"""Audit recorded P16 command authorization; this is not a physical stopping test.

Input records: {topic, recorded_ns, message}; message is normalized by the bag
adapter below. Receipt times from arbiter telemetry, not cross-topic SQLite row
order, define revocation handling. Expiry is checked in ROS simulation time.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sqlite3

AUTH = "/impact/authorization"
STATUS = "/impact/arbiter_status"
COMMAND = "/impact/position_cmd"
OUTPUT = "/uav1/mavros/setpoint_raw/local"
SPLINE = "/impact/certified_bspline"
TYPES = {AUTH: "xq_sim_interfaces/msg/TrajectoryAuthorization", STATUS: "std_msgs/msg/String",
         COMMAND: "quadrotor_msgs/msg/PositionCommand", OUTPUT: "mavros_msgs/msg/PositionTarget",
         SPLINE: "traj_utils/msg/Bspline"}
EPS = 1e-8


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def positive_id(value):
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def auth_tuple(message):
    return (message["session_id"], message["request_id"], message["trajectory_id"],
            message["issued"], message["expires"], message["accepted"])


def valid_auth(message):
    return (isinstance(message, dict) and isinstance(message.get("session_id"), str)
            and positive_id(message.get("request_id")) and positive_id(message.get("trajectory_id"))
            and finite(message.get("issued")) and finite(message.get("expires"))
            and message["expires"] >= message["issued"]
            and isinstance(message.get("accepted"), bool))


def same_values(execution, output):
    point = execution.get("position")
    other = output.get("position")
    return (isinstance(point, list) and len(point) == 3
            and isinstance(other, list) and len(other) == 3
            and all(finite(x) for x in point + other)
            and all(abs(x-y) <= 1e-6 for x, y in zip(point, other))
            and finite(execution.get("yaw")) and finite(output.get("yaw"))
            and abs(math.remainder(execution["yaw"]-output["yaw"], 2*math.pi)) <= 1e-6)


def same_vector(first, second, name):
    left, right = first.get(name), second.get(name)
    return (isinstance(left, list) and len(left) == 3
            and isinstance(right, list) and len(right) == 3
            and all(finite(value) for value in left + right)
            and all(abs(x-y) <= 1e-6 for x, y in zip(left, right)))


def same_target(execution, output):
    return (execution.get("frame_id") == output.get("frame_id") == "map"
            and execution.get("coordinate_frame") == output.get("coordinate_frame") == 1
            and execution.get("type_mask") == output.get("type_mask")
            and same_values(execution, output)
            and same_vector(execution, output, "velocity")
            and same_vector(execution, output, "acceleration")
            and finite(execution.get("yaw_rate")) and finite(output.get("yaw_rate"))
            and abs(execution["yaw_rate"]-output["yaw_rate"]) <= 1e-6)


def same_command(execution, command):
    return (same_values(dict(position=execution.get("command_position"),
                             yaw=execution.get("command_yaw")), command)
            and same_vector(dict(velocity=execution.get("command_velocity")), command, "velocity")
            and same_vector(dict(acceleration=execution.get("command_acceleration")),
                            command, "acceleration")
            and finite(execution.get("command_yaw_rate"))
            and finite(command.get("yaw_rate"))
            and abs(execution["command_yaw_rate"]-command["yaw_rate"]) <= 1e-6)


def audit_records(records, session_id, *, read_errors=()):
    """Return PASS only with correlated TRACK, revoke and expiry observations.

    INCOMPLETE means absent/undecodable evidence, not a successful behavior test.
    FAIL means recorded telemetry/outputs contradict an authorization invariant.
    """
    missing = list(read_errors)
    violations = []
    grouped = {topic: [] for topic in TYPES}
    for record in records:
        if record.get("topic") in grouped:
            grouped[record["topic"]].append(record.get("message"))
    if not isinstance(session_id, str) or not session_id:
        missing.append("run session_id is missing")
    for topic, messages in grouped.items():
        if not messages:
            missing.append("no readable messages: " + topic)
    auths = {auth_tuple(msg) for msg in grouped[AUTH]
             if valid_auth(msg) and msg.get("frame_id") == "xq_lio_map"}
    commands = {}
    for msg in grouped[COMMAND]:
        if isinstance(msg, dict) and positive_id(msg.get("trajectory_id")) and finite(msg.get("stamp")):
            commands.setdefault((msg["trajectory_id"], msg["stamp"]), []).append(msg)
    certified_ids = {msg.get("trajectory_id") for msg in grouped[SPLINE] if isinstance(msg, dict)}
    outputs = {}
    for msg in grouped[OUTPUT]:
        if not isinstance(msg, dict) or not isinstance(msg.get("stamp_ns"), int):
            missing.append("MAVROS output lacks a usable timestamp")
            continue
        outputs.setdefault(msg["stamp_ns"], []).append(msg)
    statuses = []
    receipts = {}
    decision_sequences = set()
    for status in grouped[STATUS]:
        if not isinstance(status, dict) or status.get("schema_version") != 3:
            missing.append("arbiter schema_version=3 telemetry is missing")
            continue
        if status.get("session_id") != session_id:
            violations.append("arbiter session mismatch")
            continue
        if not finite(status.get("sim_time")) or not isinstance(status.get("execution"), dict):
            missing.append("arbiter time/execution telemetry is missing")
            continue
        sequence = status.get("decision_sequence")
        if not positive_id(sequence):
            missing.append("arbiter decision_sequence is missing")
            continue
        if sequence in decision_sequences:
            missing.append("arbiter decision_sequence is duplicated")
            continue
        decision_sequences.add(sequence)
        status = dict(status)
        status["_output_matched"] = False
        execution = status["execution"]
        if not isinstance(execution.get("emitted"), bool):
            missing.append("arbiter execution.emitted is missing")
            continue
        receipt = status.get("authorization")
        if receipt is not None:
            if (not valid_auth(receipt) or not finite(receipt.get("received_sim_time"))
                    or not positive_id(receipt.get("receipt_sequence"))):
                missing.append("authorization receipt telemetry is incomplete")
            elif receipt["receipt_sequence"] >= sequence:
                violations.append("authorization receipt is not before the arbiter decision")
            elif not (receipt["issued"]-EPS <= receipt["received_sim_time"] <= status["sim_time"]+EPS
                      and receipt["received_sim_time"]-receipt["issued"] <= 0.5+EPS):
                violations.append("authorization receipt violates its freshness interval")
            elif auth_tuple(receipt) not in auths:
                missing.append("authorization receipt has no matching recorded source message")
            else:
                key = auth_tuple(receipt) + (receipt["received_sim_time"], receipt["receipt_sequence"])
                receipts[key] = receipt
        statuses.append(status)
    statuses.sort(key=lambda item: item["decision_sequence"])
    receipt_sequences = {}
    for item in receipts.values():
        sequence = item["receipt_sequence"]
        identity = auth_tuple(item) + (item["received_sim_time"],)
        if sequence in decision_sequences or (sequence in receipt_sequences and receipt_sequences[sequence] != identity):
            violations.append("arbiter event sequence has contradictory identities")
        receipt_sequences[sequence] = identity
    last_sim = None
    reset_latched = False
    used_output_ids = set()
    tracks = []
    for status in statuses:
        now, execution = status["sim_time"], status["execution"]
        if last_sim is not None and now < last_sim-1e-6:
            reset_latched = True
        last_sim = now
        if status.get("reset") is True:
            reset_latched = True
        if execution["emitted"]:
            if status.get("phase") not in ("ACTIVE", "HOVER") or status.get("mode") not in ("TRACK", "BRAKE", "STALE_ODOM_HOLD"):
                violations.append("setpoint emitted outside an allowed control phase/mode")
            stamp = execution.get("setpoint_stamp_ns")
            candidates = outputs.get(stamp, []) if isinstance(stamp, int) else []
            if not candidates:
                missing.append("emitted setpoint has no matching recorded MAVROS output")
            elif len(candidates) != 1:
                missing.append("final setpoint timestamp is ambiguous or duplicated")
            else:
                matches = [item for item in candidates if same_target(execution, item)]
                if not matches:
                    violations.append("arbiter target differs from recorded MAVROS output")
                elif id(matches[0]) in used_output_ids:
                    missing.append("multiple execution records reference the same final setpoint")
                else:
                    status["_output_matched"] = True
                    used_output_ids.add(id(matches[0]))
        if status.get("mode") != "TRACK":
            continue
        if reset_latched:
            violations.append("TRACK resumes after a simulation clock reset in the same session")
        if not execution["emitted"]:
            violations.append("TRACK reported without a published setpoint")
            continue
        if (status.get("phase") != "ACTIVE" or status.get("phase_fresh") is not True
                or status.get("fresh_odom") is not True or status.get("reset") is not False
                or status.get("authorized") is not True):
            violations.append("TRACK while task/data/authorization is invalid")
        ages = status.get("input_age_wall")
        if not isinstance(ages, dict) or any(not finite(ages.get(name)) for name in ("authorization", "command", "stage", "odom")):
            missing.append("TRACK wall-time input ages are missing")
        elif (not all(0 <= ages[name] < 0.5 for name in ("authorization", "stage", "odom"))
              or not 0 <= ages["command"] <= 0.5):
            violations.append("TRACK uses stale wall-time inputs")
        request, trajectory = execution.get("request_id"), execution.get("trajectory_id")
        issued, expires = execution.get("authorization_issued"), execution.get("authorization_expires")
        if not (positive_id(request) and positive_id(trajectory) and finite(issued) and finite(expires)):
            missing.append("TRACK lacks its exact authorization identity and interval")
            continue
        lease = (session_id, request, trajectory, issued, expires, True)
        if lease not in auths:
            missing.append("TRACK has no matching accepted source authorization")
        if not issued <= now < expires:
            violations.append("TRACK outside its authorization interval")
        if (execution.get("coordinate_frame") != 1
                or execution.get("type_mask") != 2048):
            violations.append("TRACK MAVROS target does not enable position/velocity/acceleration/yaw")
        command_stamp = execution.get("command_stamp")
        if not finite(command_stamp) or not -EPS <= now-command_stamp <= 0.5+EPS:
            violations.append("TRACK uses a stale or future input command")
        inputs = commands.get((trajectory, command_stamp), []) if finite(command_stamp) else []
        if not inputs:
            missing.append("TRACK input command is not present in the bag")
        elif not any(m.get("frame_id") == "xq_lio_map" and same_command(execution, m)
                     for m in inputs):
            violations.append("TRACK command values differ from recorded PositionCommand")
        if trajectory not in certified_ids:
            missing.append("TRACK trajectory has no recorded certified B-spline")
        matching_receipts = [r for r in receipts.values() if auth_tuple(r) == lease
                             and r["received_sim_time"] <= now + EPS
                             and r["receipt_sequence"] < status["decision_sequence"]]
        if not matching_receipts:
            missing.append("TRACK lease has no observed arbiter receipt")
        else:
            latest_used = max(matching_receipts, key=lambda r: r["receipt_sequence"])
            accepted_before = [r for r in receipts.values() if r["accepted"]
                               and r["session_id"] == session_id
                               and r["receipt_sequence"] < status["decision_sequence"]]
            latest_accepted = max(accepted_before, key=lambda r: r["receipt_sequence"])
            if latest_accepted["receipt_sequence"] > latest_used["receipt_sequence"]:
                violations.append("TRACK uses a superseded authorization revision")
        tracks.append({"key": (session_id, request, trajectory), "lease": lease,
                       "time": now, "status": status})
    if any(id(msg) not in used_output_ids for values in outputs.values() for msg in values):
        missing.append("MAVROS output is not correlated to arbiter execution telemetry")
    if not any(t["status"]["_output_matched"] for t in tracks):
        missing.append("no correlated authorized TRACK output observed")

    revoke_events = []
    expiry_events = []
    for receipt in receipts.values():
        key = (receipt["session_id"], receipt["request_id"], receipt["trajectory_id"])
        if key[0] != session_id:
            continue
        if not receipt["accepted"]:
            received = receipt["received_sim_time"]
            earlier = [t for t in tracks if t["key"] == key
                       and t["status"]["decision_sequence"] < receipt["receipt_sequence"]
                       and t["lease"][3] <= receipt["issued"] + EPS]
            resumed = [t for t in tracks if t["key"] == key
                       and t["status"]["decision_sequence"] > receipt["receipt_sequence"]
                       and t["lease"][3] <= receipt["issued"] + EPS]
            if resumed:
                violations.append("revoked old lease continues to TRACK after arbiter receipt")
            if not earlier:
                continue  # Rejecting a never-executed candidate is not revocation coverage.
            observed = any(s.get("authorization") == receipt and s["decision_sequence"] > receipt["receipt_sequence"]
                           and s.get("phase") == "ACTIVE" and s["_output_matched"]
                           and not any(t["status"] is s for t in resumed) for s in statuses)
            revoke_events.append(dict(trajectory_id=key[2], issued=receipt["issued"],
                                      received_sim_time=received, post_event_output=observed))
        else:
            lease = auth_tuple(receipt)
            earlier = [t for t in tracks if t["lease"] == lease and t["time"] < receipt["expires"]]
            if not earlier:
                continue
            # A renewed/replaced lease is not an expiry-stop experiment. Require
            # telemetry showing this expired receipt remained the last accepted input.
            expired = [s for s in statuses if s.get("authorization") == receipt
                       and s["sim_time"] >= receipt["expires"] and s.get("phase") == "ACTIVE"]
            if not expired:
                continue
            observed = any(s["_output_matched"] and s.get("mode") != "TRACK"
                           and s.get("authorized") is False for s in expired)
            expiry_events.append(dict(trajectory_id=key[2], expires=receipt["expires"],
                                      post_event_output=observed))
    if not revoke_events or not all(event["post_event_output"] for event in revoke_events):
        missing.append("revocation of an executed trajectory lacks correlated post-event output")
    if not expiry_events or not all(event["post_event_output"] for event in expiry_events):
        missing.append("lease expiration lacks correlated post-expiry output")
    missing, violations = sorted(set(missing)), sorted(set(violations))
    return dict(schema_version=1, gate="STAGE_A_AUTHORIZED_COMMAND_EXECUTION", session_id=session_id,
        status="FAIL" if violations else "INCOMPLETE" if missing else "PASS",
        checks=dict(valid_track_observed=any(t["status"]["_output_matched"] for t in tracks),
                    revocation_observed=bool(revoke_events),
                    revocation_output_observed=bool(revoke_events) and all(e["post_event_output"] for e in revoke_events),
                    expiration_observed=bool(expiry_events),
                    expiration_output_observed=bool(expiry_events) and all(e["post_event_output"] for e in expiry_events),
                    no_recorded_authorization_violation=not violations,
                    evidence_complete=not missing),
        counts=dict(authorized_track_samples=len(tracks), final_output_samples=sum(map(len, outputs.values())),
                    arbiter_samples=len(statuses)), revocations=revoke_events, expirations=expiry_events,
        violations=violations, incomplete_reasons=missing, physical_stop_verified=False,
        scope="Recorded software command authorization and final MAVROS target output only; no full TF reconstruction, physical stopping-distance or flight-dynamics claim.",
        timing="Revocation ordering uses arbiter receipt_sequence/decision_sequence; receipt simulation time and wall-time input ages are checked. Expiry uses the lease simulation-time deadline. Cross-topic bag row order is not a causal clock.")


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def normalize(topic, message):
    if topic == STATUS:
        return json.loads(message.data)
    if topic == AUTH:
        return dict(session_id=message.session_id, request_id=message.request_id,
                    trajectory_id=message.trajectory_id, issued=stamp_seconds(message.header.stamp),
                    expires=stamp_seconds(message.valid_until), accepted=message.authorized,
                    frame_id=message.header.frame_id)
    if topic == COMMAND:
        point = message.position
        velocity = message.velocity
        acceleration = message.acceleration
        return dict(trajectory_id=message.trajectory_id, stamp=stamp_seconds(message.header.stamp),
                    frame_id=message.header.frame_id, position=[point.x, point.y, point.z],
                    velocity=[velocity.x, velocity.y, velocity.z],
                    acceleration=[acceleration.x, acceleration.y, acceleration.z],
                    yaw=message.yaw, yaw_rate=message.yaw_dot)
    if topic == SPLINE:
        return dict(trajectory_id=message.traj_id)
    point = message.position
    velocity = message.velocity
    acceleration = message.acceleration_or_force
    return dict(stamp_ns=message.header.stamp.sec*1000000000+message.header.stamp.nanosec,
                frame_id=message.header.frame_id, coordinate_frame=message.coordinate_frame,
                type_mask=message.type_mask, position=[point.x, point.y, point.z],
                velocity=[velocity.x, velocity.y, velocity.z],
                acceleration=[acceleration.x, acceleration.y, acceleration.z],
                yaw=message.yaw, yaw_rate=message.yaw_rate)


def read_bag(run_dir):
    import yaml
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = Path(run_dir).resolve() / "rosbag"
    info = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    if info["storage_identifier"] != "sqlite3" or info.get("compression_format"):
        raise ValueError("authorization audit requires uncompressed sqlite3 rosbag storage")
    records, errors, seen = [], [], set()
    message_types = {kind: get_message(kind) for kind in set(TYPES.values())}
    paths = info["relative_file_paths"]
    if not isinstance(paths, list) or not paths:
        raise ValueError("rosbag metadata has no storage files")
    for relative in paths:
        path = (bag / relative).resolve()
        if not path.is_relative_to(bag) or path in seen:
            raise ValueError("rosbag storage path escapes its directory or is duplicated")
        seen.add(path)
        with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as connection:
            placeholders = ",".join("?" for _ in TYPES)
            rows = connection.execute("SELECT t.name,t.type,m.timestamp,m.data FROM messages m "
                                      "JOIN topics t ON m.topic_id=t.id WHERE t.name IN ("
                                      + placeholders + ") ORDER BY m.timestamp", tuple(TYPES))
            for topic, kind, recorded_ns, payload in rows:
                if topic not in TYPES:
                    continue
                try:
                    if kind != TYPES[topic]:
                        raise ValueError("unexpected message type " + kind)
                    message = deserialize_message(payload, message_types[kind])
                    records.append(dict(topic=topic, recorded_ns=recorded_ns,
                                        message=normalize(topic, message)))
                except Exception as error:
                    errors.append(f"cannot decode {topic} at {recorded_ns}: {error}")
    return records, errors


def audit(run_dir):
    run = Path(run_dir).resolve()
    errors = []
    session = ""
    try:
        session = json.loads((run / "run.json").read_text())["session_id"]
        records, errors = read_bag(run)
    except Exception as error:
        records = []
        errors.append(str(error))
    result = audit_records(records, session, read_errors=errors)
    result["run"] = str(run)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir")
    parser.add_argument("--output")
    args = parser.parse_args()
    result = audit(args.run_dir)
    output = Path(args.output) if args.output else Path(args.run_dir) / "stage-a-authorization-audit.json"
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
