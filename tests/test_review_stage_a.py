"""Regression tests for Stage A evidence contracts, without launching flight."""
import argparse
import json
import sqlite3
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

import impact
from diagnose_stage_a_maps import (
    EGO_SOURCE_TOPIC, EXPECTED, _aligned_cloud_pair, _grid_summary, audit,
    reachable_free_cells,
)


def recorded_bag(root, *, missing=None, zero_count=None, wrong_type=None):
    bag = root / "rosbag"
    bag.mkdir()
    storage = bag / "case.db3"
    with sqlite3.connect(storage) as db:
        db.execute("CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT, type TEXT)")
        db.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, data BLOB)")
        for index, (topic, kind) in enumerate(EXPECTED.items(), 1):
            if topic == missing:
                continue
            db.execute("INSERT INTO topics VALUES (?, ?, ?)",
                       (index, topic, "std_msgs/msg/String" if topic == wrong_type else kind))
            if topic != zero_count:
                # Inventory does not certify CDR payloads or map contents.
                db.execute("INSERT INTO messages VALUES (?, ?, ?)", (index, index, b"inventory-only"))
    (bag / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {
        "storage_identifier": "sqlite3", "relative_file_paths": [storage.name]}}))
    return bag


def test_arbitrary_map_file_and_topic_strings_cannot_verify_map(tmp_path):
    (tmp_path / "rosbag").mkdir()
    (tmp_path / "rosbag/metadata.yaml").write_text("\n".join(EXPECTED))
    (tmp_path / "map.txt").write_text("not a map")
    result = audit(tmp_path)
    assert result["status"] == "UNVERIFIED"
    assert result["map_content_verified"] is False
    assert not result["artifacts"]["ego_collision_map_artifact"]


def test_recorded_topics_are_only_ready_for_content_review(tmp_path):
    recorded_bag(tmp_path)
    result = audit(tmp_path)
    assert result["status"] == "READY_FOR_REVIEW"
    assert result["map_content_verified"] is False
    assert result["artifacts"]["information_map_artifact"]
    assert result["artifacts"]["ego_collision_map_artifact"]
    assert len(result["bag_files"][0]["sha256"]) == 64


@pytest.mark.parametrize("defect", ["missing", "zero_count", "wrong_type"])
def test_collision_map_must_have_recorded_messages_of_correct_type(tmp_path, defect):
    recorded_bag(tmp_path, **{defect: "/grid_map/occupancy_inflate"})
    assert audit(tmp_path)["status"] == "UNVERIFIED"


def test_bag_inventory_rejects_escaping_storage_paths(tmp_path):
    bag = recorded_bag(tmp_path)
    (bag / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {
        "storage_identifier": "sqlite3", "relative_file_paths": ["../outside.db3"]}}))
    result = audit(tmp_path)
    assert result["status"] == "UNVERIFIED" and result["errors"]
    assert not (tmp_path / "outside.db3").exists()


def test_reachable_free_cells_does_not_cross_occupied_barrier():
    grid = np.zeros((5, 7), dtype=np.int8)
    grid[:, 3] = 100
    component = reachable_free_cells(grid, (1, 2))
    assert len(component) == 15
    assert (5, 2) not in component


def test_reachable_free_cells_rejects_unknown_start():
    grid = np.zeros((3, 3), dtype=np.int8)
    grid[1, 1] = -1
    assert reachable_free_cells(grid, (1, 1)) == set()


def test_navigation_reachability_uses_fixed_mission_start_not_snapshot_pose():
    info = SimpleNamespace(width=8, height=3, resolution=1.0,
        origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)))
    message = SimpleNamespace(header=SimpleNamespace(frame_id="xq_lio_map",
        stamp=SimpleNamespace(sec=12, nanosec=0)), info=info,
        data=[0] * 24)
    odometry = [
        {"record_ns": 1, "stamp_s": 1.0, "frame_id": "xq_lio_map", "x": 0.5, "y": 1.5},
        {"record_ns": 12_000_000_000, "stamp_s": 12.0, "frame_id": "xq_lio_map",
         "x": 6.5, "y": 1.5},
    ]
    summary = _grid_summary(message, 12_000_000_000, odometry, (6.5, 1.5), odometry[0])
    assert summary["start_cell"] == [0, 1]
    assert summary["goal_cell"] == [6, 1]
    assert summary["mission_start_xy"] == [0.5, 1.5]


def test_inflation_alignment_pair_is_latest_within_time_window():
    cloud = lambda stamp, count: {"stamp_s": stamp, "sampled_finite_points": count}
    source, inflated, gap = _aligned_cloud_pair(
        [cloud(10.0, 100), cloud(20.0, 100)],
        [cloud(10.1, 100), cloud(19.8, 50), cloud(40.0, 5000)])
    assert source["stamp_s"] == 20.0
    assert inflated["stamp_s"] == 19.8
    assert gap == pytest.approx(0.2)


def test_inflation_audit_uses_recorded_registered_cloud_source():
    assert EGO_SOURCE_TOPIC == "/cloud_registered"
    assert EGO_SOURCE_TOPIC in EXPECTED
    assert "/impact/legacy_frontier_cloud" in EXPECTED


def authorization_contract(source="source"):
    return {"status": "PASS", "physical_stop_verified": False,
        "build": {"source_sha256": source},
        "checks": {name: True for name in impact.AUTHORIZATION_CONTRACT_CHECKS}}


def test_separate_bound_contract_completes_only_missing_expiry(tmp_path):
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(authorization_contract()))
    contract = impact.review_authorization_contract(path, "source")
    audit_report = {"status": "INCOMPLETE",
        "incomplete_reasons": ["lease expiration lacks correlated post-expiry output"],
        "checks": {"valid_track_observed": True, "revocation_observed": True,
            "revocation_output_observed": True, "no_recorded_authorization_violation": True},
        "violations": []}
    assert contract["verified"]
    assert impact.authorization_evidence_complete(audit_report, contract)


@pytest.mark.parametrize("defect", ["wrong_source", "missing_check", "extra_gap"])
def test_contract_cannot_cover_unrelated_or_incomplete_flight_evidence(tmp_path, defect):
    data = authorization_contract("other" if defect == "wrong_source" else "source")
    if defect == "missing_check":
        data["checks"]["expired_lease_brakes"] = False
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(data))
    contract = impact.review_authorization_contract(path, "source")
    reasons = ["lease expiration lacks correlated post-expiry output"]
    if defect == "extra_gap":
        reasons.append("MAVROS output is not correlated to arbiter execution telemetry")
    audit_report = {"status": "INCOMPLETE", "incomplete_reasons": sorted(reasons),
        "checks": {"valid_track_observed": True, "revocation_observed": True,
            "revocation_output_observed": True, "no_recorded_authorization_violation": True},
        "violations": []}
    assert not impact.authorization_evidence_complete(audit_report, contract)


def write_actual_evaluation(root, *, samples=100, collisions=0, clearance=1.):
    pytest.importorskip("rclpy")
    from xq_autonomy.sitl_evaluator_node import SITLEvaluator
    evaluator = SimpleNamespace(root=root, last_active_sim=10., start_sim=0.,
        samples=samples, collision_samples=collisions, collision_events=collisions,
        hmi=0, dropped_pairs=0, path_length=12., stopped_time=0., stopping_durations=[],
        minimum_clearance=clearance, squared_errors=[0.0001], started_wall=0.,
        coverage=[True], integrity_violations=0, authorized_samples=1)
    SITLEvaluator.write(evaluator)
    return impact.read_json(root / "evaluation.json")


@pytest.mark.parametrize("samples,collisions,expected", [
    (100, 0, "PASS"), (0, 0, "IN_PROGRESS"), (100, 1, "FAIL"),
])
def test_evaluator_emits_actual_geometric_status(tmp_path, samples, collisions, expected):
    report = write_actual_evaluation(tmp_path, samples=samples, collisions=collisions)
    assert report["status"] == expected


def test_actual_evaluator_output_still_requires_map_and_execution_evidence(tmp_path, monkeypatch):
    run = tmp_path / "normal-baseline-s1000"
    run.mkdir()
    recorded_bag(run)
    evaluation = write_actual_evaluation(run)
    report = dict(status="PASS", completed_record=True, source_sha256="source", config_sha256="cfg",
        mission=dict(gate="P16", task_success=True, termination_confirmed=True), evaluation=evaluation)
    (run / "sitl_runtime/logs").mkdir(parents=True)
    (run / "sitl_runtime/logs/1.BIN").write_bytes(b"fixture")
    (run / "events.jsonl").write_text(
        json.dumps(dict(event="CERTIFY", trajectory_id=1, accepted=True)) + "\n" +
        json.dumps(dict(event="REVOKE")) + "\n")
    monkeypatch.setattr(impact, "experiment", lambda args: (run, report))
    result = impact.stage_a(argparse.Namespace(results=str(tmp_path), scenario="normal",
        strategy="baseline", seed=1000))
    gate = impact.read_json(run / "stage-a-validation.json")
    assert gate["checks"]["evaluation_pass"]
    assert gate["failed_checks"] == ["authorized_command_execution_verified", "map_content_verified"]
    assert gate["authorization_audit"]["status"] == "INCOMPLETE"
    assert gate["status"] == "REVIEW_REQUIRED" and result != 0


def test_runner_records_required_map_evidence():
    # Check the actual rosbag invocation, not comments or unused topic constants.
    script = (impact.ROOT / "scripts/impact_run.sh").read_text()
    recorder = script.split("start rosbag ros2 bag record", 1)[1].split("start stack", 1)[0]
    for topic in EXPECTED:
        assert topic in recorder.split(), topic


def test_stage_a_publishes_registered_cloud_for_information_map():
    launch = (impact.ROOT / "src/xq_sim_bringup/launch/impact_sitl.launch.py").read_text()
    fast_lio = launch.split('executable="fastlio_mapping"', 1)[1].split(
        'autonomy("xq_p4_external_nav"', 1
    )[0]
    assert '"publish.scan_publish_en": True' in fast_lio
    information_map = launch.split('autonomy("xq_p10_information_map"', 1)[1].split(
        'Node(package="ego_planner"', 1
    )[0]
    assert 'remappings=[("/xq/p5/cloud_map", "/impact/information_cloud")]' in information_map


def test_stage_a_freezes_p15_validated_information_memory_contract():
    configuration = impact.config()
    assert configuration["integrity_information_memory_horizon_s"] == 3.0
    assert configuration["integrity_information_memory_max_frames"] == 20.0
    launch = (impact.ROOT / "src/xq_sim_bringup/launch/impact_sitl.launch.py").read_text()
    integrity = launch.split('autonomy("xq_p6_directional_integrity"', 1)[1].split(
        'autonomy("xq_p10_information_map"', 1
    )[0]
    assert '"information_memory_horizon_s": config["configuration"]' in integrity
    assert '"information_memory_max_frames": config["configuration"]' in integrity


def test_runtime_bag_records_actual_integrity_memory_configuration():
    script = (impact.ROOT / "scripts/impact_run.sh").read_text()
    recorder = script.split("start rosbag ros2 bag record", 1)[1].split("start stack", 1)[0]
    assert "/integrity/debug" in recorder.split()


def test_runtime_graph_probes_retry_required_endpoint_and_record_identity_context():
    script = (impact.ROOT / "scripts/impact_run.sh").read_text()
    probe = script.split("graph_probe() {", 1)[1].split("sha256sum", 1)[0]
    assert "SECONDS+20" in probe
    assert "--no-daemon" in probe and '"ros_domain_id":domain' in probe
    assert '"command":["ros2","topic","info"' in probe
    assert "attempts.jsonl" in probe
    assert '"Node name: impact_arbiter"' in script


@pytest.mark.parametrize("mission_status,evaluation_status,expected", [
    ("PASS", "PASS", "PASS"), ("FAIL", "PASS", "FAIL"), ("PASS", "FAIL", "FAIL"),
])
def test_goal_reached_does_not_override_failed_termination_deadline(mission_status, evaluation_status, expected):
    mission = dict(status=mission_status, task_success=True, termination_confirmed=True)
    evaluation = dict(status=evaluation_status, samples=100, collision_events=0)
    result = impact.classify_outcome(0, mission, evaluation)
    assert result == dict(status=expected, completed_record=True)


def test_unconfirmed_termination_cannot_be_completed_outcome():
    result = impact.classify_outcome(0,
        dict(status="PASS", task_success=True, termination_confirmed=False),
        dict(status="PASS", samples=100, collision_events=0))
    assert result == dict(status="FAIL", completed_record=False)


@pytest.mark.parametrize("audit_status,expected", [("PASS", "REVIEW_REQUIRED"),
    ("INCOMPLETE", "REVIEW_REQUIRED"), ("FAIL", "FAIL")])
def test_stage_a_propagates_execution_violations_without_downgrading_to_review(tmp_path, monkeypatch, audit_status, expected):
    run = tmp_path / "case"
    run.mkdir()
    recorded_bag(run)
    (run / "sitl_runtime/logs").mkdir(parents=True)
    (run / "sitl_runtime/logs/1.BIN").write_bytes(b"fixture")
    # Event labels alone no longer satisfy authorization acceptance.
    (run / "events.jsonl").write_text(json.dumps(dict(event="CERTIFY", trajectory_id=17, accepted=False)))
    report = dict(status="PASS", completed_record=True,
        mission=dict(gate="P16", task_success=True, termination_confirmed=True),
        evaluation=dict(status="PASS", samples=100, collision_events=0))
    monkeypatch.setattr(impact, "experiment", lambda args: (run, report))
    monkeypatch.setattr(impact, "audit_stage_a_authorization", lambda path: dict(status=audit_status))
    result = impact.stage_a(argparse.Namespace(results=str(tmp_path), scenario="normal", strategy="baseline", seed=1000))
    gate = impact.read_json(run / "stage-a-validation.json")
    assert gate["status"] == expected and result == 1
    assert gate["checks"]["authorized_command_execution_verified"] is (audit_status == "PASS")


@pytest.mark.parametrize("status", ["FAIL", "INCOMPLETE"])
def test_audit_cli_nonzero_status_preserves_report(tmp_path, monkeypatch, status):
    def execute(command, **kwargs):
        impact.write_json(tmp_path / "stage-a-authorization-audit.json", dict(status=status, errors=["fixture"]))
        return SimpleNamespace(returncode=1, stderr="")
    monkeypatch.setattr(impact.subprocess, "run", execute)
    assert impact.audit_stage_a_authorization(tmp_path)["status"] == status


def test_failed_audit_process_cannot_reuse_previous_pass(tmp_path, monkeypatch):
    impact.write_json(tmp_path / "stage-a-authorization-audit.json", dict(status="PASS"))
    monkeypatch.setattr(impact.subprocess, "run", lambda *a, **kw: SimpleNamespace(returncode=2, stderr="decode failed"))
    result = impact.audit_stage_a_authorization(tmp_path)
    assert result["status"] == "INCOMPLETE" and "decode failed" in result["errors"][0]


def test_map_audit_uses_run_bound_install(tmp_path, monkeypatch):
    install = tmp_path / "install"
    install.mkdir()
    (install / "setup.bash").write_text("# fixture\n")
    impact.write_json(tmp_path / "build-manifest.json", {"install_root": str(install)})

    def execute(command, **kwargs):
        assert command[:2] == ["bash", "-c"]
        assert "source \"$1/setup.bash\"" in command[2]
        assert command[3] == "impact-map-audit"
        assert str(install) in command
        impact.write_json(tmp_path / "stage-a-map-audit.json",
                          dict(status="READY_FOR_REVIEW", map_content_verified=False))
        return SimpleNamespace(returncode=0, stderr="")

    monkeypatch.setattr(impact.subprocess, "run", execute)
    assert impact.audit_stage_a_maps(tmp_path)["status"] == "READY_FOR_REVIEW"


def test_failed_map_audit_cannot_reuse_previous_pass(tmp_path, monkeypatch):
    impact.write_json(tmp_path / "stage-a-map-audit.json",
                      dict(status="MAP_CONTENT_VERIFIED", map_content_verified=True))
    monkeypatch.setattr(impact.subprocess, "run",
                        lambda *a, **kw: SimpleNamespace(returncode=2, stderr="decode failed"))
    result = impact.audit_stage_a_maps(tmp_path)
    assert result["status"] == "UNVERIFIED"
    assert result["map_content_verified"] is False
    assert "decode failed" in result["errors"][0]


@pytest.mark.parametrize("status,returncode,expected", [("PASS", 1, "INCOMPLETE"), ("FAIL", 2, "FAIL")])
def test_audit_process_exit_must_agree_with_pass(tmp_path, monkeypatch, status, returncode, expected):
    def execute(command, **kwargs):
        impact.write_json(tmp_path / "stage-a-authorization-audit.json", dict(status=status))
        return SimpleNamespace(returncode=returncode, stderr="unexpected exit")
    monkeypatch.setattr(impact.subprocess, "run", execute)
    assert impact.audit_stage_a_authorization(tmp_path)["status"] == expected
