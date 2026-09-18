import json

from audit_actual_goal import audit, goal_world


def write_json(path, value):
    path.write_text(json.dumps(value) + "\n", encoding="utf-8")


def fixture_run(tmp_path, endpoint):
    scenario = {"start_world_m": [1., 2., .2], "start_yaw": 0.,
                "goal_lio_m": [12., 0., 2.], "actual_goal_tolerance_m": .45}
    write_json(tmp_path / "scenario.json", scenario)
    (tmp_path / "telemetry.jsonl").write_text(
        json.dumps({"truth": [1., 2., .2]}) + "\n" +
        json.dumps({"truth": endpoint}) + "\n", encoding="utf-8")
    write_json(tmp_path / "mission.json",
               {"status": "PASS", "task_success": True, "termination_confirmed": True})
    write_json(tmp_path / "evaluation.json", {"status": "PASS", "collision_events": 0})
    write_json(tmp_path / "run.json", {"status": "PASS", "launcher_exit_code": 0,
               "dataflash_termination": {"confirmed": True}})
    return scenario


def test_goal_world_applies_declared_start_pose_and_yaw():
    target = goal_world({"start_world_m": [1., 2., .2], "start_yaw": 1.5707963267948966,
                         "goal_lio_m": [3., 0., 2.]})
    assert target == [1.0000000000000002, 5.0, 2.2]


def test_audit_corrects_false_positive_without_rewriting_original(tmp_path):
    fixture_run(tmp_path, [80., 2., 2.2])
    original = (tmp_path / "run.json").read_bytes()
    report = audit(tmp_path)
    assert report["status"] == "FAIL"
    assert report["task_status"] == "FAIL"
    assert report["termination_status"] == "PASS"
    assert report["checks"]["actual_goal_reached"] is False
    assert report["original_classification"]["run_status"] == "PASS"
    assert (tmp_path / "run.json").read_bytes() == original


def test_audit_accepts_truth_endpoint_inside_frozen_tolerance(tmp_path):
    fixture_run(tmp_path, [13.2, 2., 2.2])
    report = audit(tmp_path)
    assert report["status"] == "PASS"
    assert report["checks"]["actual_goal_reached"] is True


def test_legacy_audit_requires_explicit_verified_tolerance(tmp_path):
    scenario = fixture_run(tmp_path, [13.2, 2., 2.2])
    del scenario["actual_goal_tolerance_m"]
    write_json(tmp_path / "scenario.json", scenario)
    try:
        audit(tmp_path)
    except ValueError as error:
        assert "--legacy-goal-tolerance-m" in str(error)
    else:
        raise AssertionError("legacy scenario unexpectedly inferred a tolerance")
    report = audit(tmp_path, legacy_goal_tolerance_m=.45)
    assert report["status"] == "PASS"
    assert report["actual_goal_tolerance_source"] == "explicit legacy command-line value"
