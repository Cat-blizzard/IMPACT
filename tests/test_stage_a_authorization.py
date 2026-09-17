"""Authorization acceptance uses actual command evidence, never event names alone."""
import copy
import json
import math
import sqlite3

import pytest

from audit_stage_a_authorization import AUTH, STATUS, COMMAND, OUTPUT, SPLINE, TYPES, audit, audit_records


def record(topic, message):
    return dict(topic=topic, recorded_ns=123, message=message)


def receipt(trajectory, issued, expires, *, accepted=True, received=None):
    return dict(session_id="case", request_id=trajectory, trajectory_id=trajectory,
                issued=issued, expires=expires, accepted=accepted,
                received_sim_time=issued if received is None else received,
                receipt_sequence=round((issued if received is None else received)*2000))


def source_auth(item):
    data = {key: value for key, value in item.items() if key not in ("received_sim_time", "receipt_sequence")}
    return record(AUTH, dict(data, frame_id="xq_lio_map"))


def decision(now, source, stamp, *, mode="TRACK", active_lease=None):
    lease = active_lease or source
    position = [.2, 0., 2.] if mode == "TRACK" else [0., 0., 2.]
    velocity = [.3, 0., 0.] if mode == "TRACK" else [0., 0., 0.]
    acceleration = [.1, 0., 0.] if mode == "TRACK" else [0., 0., 0.]
    type_mask = 2048 if mode == "TRACK" else 2552
    execution = dict(emitted=True, setpoint_stamp_ns=stamp, frame_id="map",
        coordinate_frame=1, type_mask=type_mask, position=position,
        velocity=velocity, acceleration=acceleration, yaw=.1, yaw_rate=0.,
        trajectory_id=lease["trajectory_id"] if mode == "TRACK" else None,
        request_id=lease["request_id"] if mode == "TRACK" else None,
        command_stamp=now if mode == "TRACK" else None,
        command_position=position if mode == "TRACK" else None,
        command_velocity=velocity if mode == "TRACK" else None,
        command_acceleration=acceleration if mode == "TRACK" else None,
        command_yaw=.1 if mode == "TRACK" else None,
        command_yaw_rate=0. if mode == "TRACK" else None,
        authorization_issued=lease["issued"] if mode == "TRACK" else None,
        authorization_expires=lease["expires"] if mode == "TRACK" else None)
    status = dict(schema_version=3, session_id="case", sim_time=now, phase="ACTIVE",
        decision_sequence=round(now*2000)+1,
        input_age_wall=dict(authorization=.01, command=.01, stage=.01, odom=.01),
        phase_fresh=True, fresh_odom=True, reset=False, mode=mode, authorized=mode == "TRACK",
        authorization=source, execution=execution)
    records = [record(STATUS, status), record(OUTPUT, dict(stamp_ns=stamp, frame_id="map",
        coordinate_frame=1, type_mask=type_mask, position=position, velocity=velocity,
        acceleration=acceleration, yaw=.1, yaw_rate=0.))]
    if mode == "TRACK":
        records.append(record(COMMAND, dict(trajectory_id=lease["trajectory_id"], stamp=now,
            frame_id="xq_lio_map", position=position, velocity=velocity,
            acceleration=acceleration, yaw=.1, yaw_rate=0.)))
    return records


@pytest.fixture
def records():
    first = receipt(7, 10., 11.)
    revoke = receipt(7, 10.2, 10.2, accepted=False, received=10.25)
    second = receipt(8, 12., 12.3)
    return ([source_auth(first), source_auth(revoke), source_auth(second),
             record(SPLINE, dict(trajectory_id=7)), record(SPLINE, dict(trajectory_id=8))]
            + decision(10.1, first, 100) + decision(10.25, revoke, 101, mode="BRAKE")
            + decision(12.1, second, 200) + decision(12.31, second, 201, mode="BRAKE"))


def test_complete_command_evidence_passes_without_claiming_physical_stop(records):
    result = audit_records(records, "case")
    assert result["status"] == "PASS", result
    assert all(result["checks"].values())
    assert not result["physical_stop_verified"]
    assert len(result["revocations"]) == len(result["expirations"]) == 1


def test_cross_topic_recording_order_is_not_used_as_causal_time(records):
    assert audit_records(list(reversed(records)), "case")["status"] == "PASS"


def test_certify_and_revoke_log_names_are_not_execution_evidence():
    rows = [record("events", dict(event="CERTIFY", trajectory_id=7, accepted=True)),
            record("events", dict(event="REVOKE", trajectory_id=7))]
    assert audit_records(rows, "case")["status"] == "INCOMPLETE"


@pytest.mark.parametrize("topic", list(TYPES))
def test_missing_required_topic_cannot_pass(records, topic):
    result = audit_records([r for r in records if r["topic"] != topic], "case")
    assert result["status"] != "PASS"


def test_rejected_unexecuted_candidate_is_not_revocation_coverage(records):
    for row in records:
        msg = row["message"]
        if row["topic"] == AUTH and not msg["accepted"]:
            msg["request_id"] = msg["trajectory_id"] = 99
        if row["topic"] == STATUS and not msg["authorization"]["accepted"]:
            msg["authorization"]["request_id"] = msg["authorization"]["trajectory_id"] = 99
    result = audit_records(records, "case")
    assert result["status"] == "INCOMPLETE"
    assert not result["checks"]["revocation_observed"]


def test_revoked_old_lease_cannot_keep_tracking(records):
    revoke = receipt(7, 10.2, 10.2, accepted=False, received=10.25)
    records += decision(10.4, revoke, 102, active_lease=receipt(7, 10., 11.))
    result = audit_records(records, "case")
    assert result["status"] == "FAIL"
    assert any("revoked old lease" in error for error in result["violations"])


def test_expired_lease_cannot_keep_tracking(records):
    records += decision(12.32, receipt(8, 12., 12.3), 202)
    result = audit_records(records, "case")
    assert result["status"] == "FAIL"
    assert any("outside its authorization interval" in error for error in result["violations"])


def test_wrong_session_cannot_pass(records):
    next(row["message"] for row in records if row["topic"] == STATUS)["session_id"] = "old"
    assert audit_records(records, "case")["status"] == "FAIL"


def test_final_target_must_equal_the_recorded_mavros_output(records):
    next(row["message"] for row in records if row["topic"] == OUTPUT)["position"] = [9., 9., 9.]
    assert audit_records(records, "case")["status"] == "FAIL"


@pytest.mark.parametrize("defect", ["duplicate_output", "extra_output", "duplicate_telemetry"])
def test_output_matching_is_one_to_one(records, defect):
    topic = STATUS if defect == "duplicate_telemetry" else OUTPUT
    item = copy.deepcopy(next(row for row in records if row["topic"] == topic))
    if defect == "extra_output":
        item["message"]["stamp_ns"] = 999
    records.append(item)
    assert audit_records(records, "case")["status"] == "INCOMPLETE"


def test_track_input_values_must_match_position_command(records):
    next(row["message"] for row in records if row["topic"] == COMMAND)["position"] = [8., 0., 2.]
    assert audit_records(records, "case")["status"] == "FAIL"


def test_track_feedforward_must_match_position_command(records):
    next(row["message"] for row in records if row["topic"] == COMMAND)["velocity"] = [8., 0., 0.]
    assert audit_records(records, "case")["status"] == "FAIL"


def test_track_output_mask_cannot_drop_feedforward(records):
    output = next(row["message"] for row in records if row["topic"] == OUTPUT)
    output["type_mask"] = 2552
    assert audit_records(records, "case")["status"] == "FAIL"


def test_latest_unrelated_rejection_does_not_erase_active_lease_evidence(records):
    unrelated = receipt(99, 10.15, 10.15, accepted=False)
    records += [source_auth(unrelated)]
    records += decision(10.16, unrelated, 105, active_lease=receipt(7, 10., 11.))
    assert audit_records(records, "case")["status"] == "PASS"


def test_unrelated_late_brake_does_not_prove_revoke_response(records):
    records = [row for row in records if not (
        row["topic"] == STATUS and row["message"]["sim_time"] == 10.25
        or row["topic"] == OUTPUT and row["message"]["stamp_ns"] == 101)]
    result = audit_records(records, "case")
    assert result["status"] == "INCOMPLETE"
    assert not result["checks"]["revocation_output_observed"]


def test_partial_decode_error_cannot_pass(records):
    assert audit_records(records, "case", read_errors=["missing CDR message"])["status"] == "INCOMPLETE"


def test_missing_ros_or_bag_produces_incomplete(tmp_path):
    (tmp_path / "run.json").write_text('{"session_id":"case"}')
    result = audit(tmp_path)
    assert result["status"] == "INCOMPLETE" and result["incomplete_reasons"]


def write_ros_bag(root, records):
    pytest.importorskip("rclpy")
    import yaml
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message
    from xq_autonomy.sitl_supervisor_node import ros_stamp

    (root / "run.json").write_text('{"session_id":"case"}')
    bag = root / "rosbag"
    bag.mkdir()
    with sqlite3.connect(bag / "evidence.db3") as db:
        db.execute("CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT, type TEXT)")
        db.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
        for index, (topic, kind) in enumerate(TYPES.items(), 1):
            db.execute("INSERT INTO topics VALUES (?, ?, ?)", (index, topic, kind))
        for index, row in enumerate(records, 1):
            topic, data = row["topic"], row["message"]
            message = get_message(TYPES[topic])()
            if topic == STATUS:
                message.data = json.dumps(data)
            elif topic == AUTH:
                for name in ("session_id", "request_id", "trajectory_id"):
                    setattr(message, name, data[name])
                message.header.frame_id = data["frame_id"]
                message.header.stamp = ros_stamp(data["issued"])
                message.valid_until = ros_stamp(data["expires"])
                message.authorized = data["accepted"]
            elif topic == COMMAND:
                message.trajectory_id = data["trajectory_id"]
                message.header.stamp = ros_stamp(data["stamp"])
                message.header.frame_id = data["frame_id"]
                message.position.x, message.position.y, message.position.z = data["position"]
                message.velocity.x, message.velocity.y, message.velocity.z = data["velocity"]
                (message.acceleration.x, message.acceleration.y,
                 message.acceleration.z) = data["acceleration"]
                message.yaw = data["yaw"]
                message.yaw_dot = data["yaw_rate"]
            elif topic == SPLINE:
                message.traj_id = data["trajectory_id"]
            elif topic == OUTPUT:
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(data["stamp_ns"], 1000000000)
                message.header.frame_id = data["frame_id"]
                message.coordinate_frame = data["coordinate_frame"]
                message.type_mask = data["type_mask"]
                message.position.x, message.position.y, message.position.z = data["position"]
                message.velocity.x, message.velocity.y, message.velocity.z = data["velocity"]
                (message.acceleration_or_force.x, message.acceleration_or_force.y,
                 message.acceleration_or_force.z) = data["acceleration"]
                message.yaw = data["yaw"]
                message.yaw_rate = data["yaw_rate"]
            db.execute("INSERT INTO messages VALUES (?, ?, ?, ?)",
                       (index, list(TYPES).index(topic) + 1, 1000-index, serialize_message(message)))
    (bag / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {
        "storage_identifier": "sqlite3", "relative_file_paths": ["evidence.db3"]}}))


def test_actual_sqlite_cdr_messages_are_audited(tmp_path, records):
    write_ros_bag(tmp_path, records)
    result = audit(tmp_path)
    assert result["status"] == "PASS", result
    assert result["counts"]["final_output_samples"] == 4


def test_corrupt_cdr_is_not_silently_skipped(tmp_path, records):
    write_ros_bag(tmp_path, records)
    with sqlite3.connect(tmp_path / "rosbag/evidence.db3") as db:
        db.execute("UPDATE messages SET data=? WHERE id=1", (b"invalid",))
    result = audit(tmp_path)
    assert result["status"] != "PASS"
    assert any("cannot decode" in item for item in result["incomplete_reasons"])


@pytest.mark.parametrize("continues_after_receipt", [False, True])
def test_same_simulation_tick_uses_callback_sequence(records, continues_after_receipt):
    new_receipt = receipt(7, 10.1, 10.1, accepted=False)
    new_receipt["receipt_sequence"] = 20202
    for row in records:
        message = row["message"]
        if row["topic"] == AUTH and not message["accepted"]:
            row["message"] = source_auth(new_receipt)["message"]
        if row["topic"] == STATUS and message["mode"] == "BRAKE" and message["sim_time"] == 10.25:
            message["sim_time"] = 10.1
            message["decision_sequence"] = 20203
            message["authorization"] = new_receipt
        elif row["topic"] == STATUS and message["sim_time"] == 10.1 and continues_after_receipt:
            message["decision_sequence"] = 20204
    assert audit_records(records, "case")["status"] == ("FAIL" if continues_after_receipt else "PASS")


@pytest.mark.parametrize("field", ["authorization", "command", "stage", "odom"])
def test_track_rejects_stale_wall_time_inputs(records, field):
    next(row["message"] for row in records if row["topic"] == STATUS)["input_age_wall"][field] = .6
    assert audit_records(records, "case")["status"] == "FAIL"


def test_stale_command_is_rejected_even_if_its_source_message_exists(records):
    status = next(row["message"] for row in records if row["topic"] == STATUS)
    status["execution"]["command_stamp"] = 9.
    next(row["message"] for row in records if row["topic"] == COMMAND)["stamp"] = 9.
    assert audit_records(records, "case")["status"] == "FAIL"


def test_malformed_command_stamp_fails_without_crashing(records):
    next(row["message"] for row in records if row["topic"] == STATUS)["execution"]["command_stamp"] = []
    assert audit_records(records, "case")["status"] == "FAIL"


def test_authorization_receipt_must_be_fresh_and_before_decision(records):
    first = next(row["message"] for row in records if row["topic"] == STATUS)
    first["authorization"]["received_sim_time"] = 9.9
    assert audit_records(records, "case")["status"] == "FAIL"


def test_legacy_telemetry_without_sequence_is_incomplete(records):
    for row in records:
        if row["topic"] == STATUS:
            row["message"].pop("decision_sequence")
    assert audit_records(records, "case")["status"] == "INCOMPLETE"


def test_cli_incomplete_exits_one_and_writes_readable_report(tmp_path, monkeypatch, capsys):
    from audit_stage_a_authorization import main
    monkeypatch.setattr("sys.argv", ["audit_stage_a_authorization.py", str(tmp_path)])
    assert main() == 1
    report = json.loads((tmp_path / "stage-a-authorization-audit.json").read_text())
    assert report["status"] == "INCOMPLETE"
    assert json.loads(capsys.readouterr().out)["status"] == "INCOMPLETE"


@pytest.mark.parametrize("replacement_id", [7, 9])
def test_new_accepted_revision_replaces_still_unexpired_old_lease(records, replacement_id):
    replacement = receipt(replacement_id, 10.12, 11.3)
    records += [source_auth(replacement), record(SPLINE, dict(trajectory_id=replacement_id))]
    records += decision(10.15, replacement, 150)
    records += decision(10.16, replacement, 151, active_lease=receipt(7, 10., 11.))
    result = audit_records(records, "case")
    assert result["status"] == "FAIL"
    assert "TRACK uses a superseded authorization revision" in result["violations"]


def test_same_session_cannot_resume_track_after_simulation_clock_reset(records):
    after_reset = receipt(9, 1., 1.3)
    after_reset["receipt_sequence"] = 30000
    added = decision(1.1, after_reset, 300)
    added[0]["message"]["decision_sequence"] = 30001
    records += [source_auth(after_reset), record(SPLINE, dict(trajectory_id=9))] + added
    result = audit_records(records, "case")
    assert result["status"] == "FAIL"
    assert any("simulation clock reset" in error for error in result["violations"])


@pytest.mark.parametrize("phase,mode", [("LAND", "INACTIVE"), ("ACTIVE", "TF_FAILURE"),
                                        ("ACTIVE", "UNRECOGNIZED"), ("TAKEOFF", "BRAKE")])
def test_nontrack_labels_cannot_hide_outputs_outside_control_contract(records, phase, mode):
    added = decision(12.4, receipt(8, 12., 12.3), 400, mode=mode)
    added[0]["message"]["phase"] = phase
    records += added
    result = audit_records(records, "case")
    assert result["status"] == "FAIL"
    assert any("allowed control phase/mode" in error for error in result["violations"])


def test_position_command_cannot_relabel_map_as_the_required_lio_frame(records):
    next(row["message"] for row in records if row["topic"] == COMMAND)["frame_id"] = "map"
    result = audit_records(records, "case")
    assert result["status"] == "FAIL"
    assert any("PositionCommand" in error for error in result["violations"])
