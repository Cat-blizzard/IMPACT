import argparse
import json
from pathlib import Path
import socket
import pytest
import impact
from impact_scenarios import scenario_geometry, generate
from impact_runtime_audit import check_graph
from impact_runtime_audit import check_extnav


def test_seed_changes_real_geometry_and_paired_arms_share_world(tmp_path):
    a=scenario_geometry("recoverable",7)
    assert a == scenario_geometry("recoverable",7)
    assert a != scenario_geometry("recoverable",8)
    assert scenario_geometry("unrecoverable",7)["seed_effect"].startswith("bridge")
    generate(impact.ROOT,tmp_path,"normal",7)
    world=(tmp_path/"world.sdf").read_text()
    assert "ArduPilot" not in world  # plugin belongs to the included actuation model
    assert "xq_iris_mid360_ardupilot" in world
    assert "VelocityControl" not in world
    assert "feature_0" in world


def test_failed_preflight_is_preserved_and_bundled(tmp_path,monkeypatch):
    monkeypatch.setattr(impact,"doctor",lambda *a,**kw:dict(ready=False,checks=dict(gazebo=False)))
    monkeypatch.setattr(impact,"source_hash",lambda:"source")
    args=argparse.Namespace(results=str(tmp_path),scenario="normal",strategy="recovery",seed=1000,profile="local_cpu")
    run,result=impact.experiment(args)
    assert result["status"] == "ERROR" and not result["completed_record"]
    assert "gazebo" in result["reason"]
    assert (run/"doctor.json").exists()
    assert list(tmp_path.glob("*-diagnostics.tar.gz"))


def test_statistics_keep_failures_and_retries(tmp_path):
    for i,status in enumerate(("PASS","FAIL","ERROR")):
        impact.write_json(tmp_path/str(i)/"run.json",dict(scenario="normal",strategy="recovery",seed=i,
                           status=status,session_id=str(i)))
    impact.summarize(tmp_path)
    group=impact.read_json(tmp_path/"summary.json")["groups"]["normal/recovery"]
    assert group["attempts"] == 3
    assert (group["success"],group["failure"],group["error"]) == (1,1,1)


def test_graph_audit_checks_endpoint_owners():
    truth="Publisher count: 1\n\nNode name: impact_evaluator\nEndpoint type: SUBSCRIPTION\n"
    setpoint="Publisher count: 1\nNode name: impact_arbiter\nEndpoint type: PUBLISHER\n"
    result=check_graph(truth,setpoint)
    assert result["truth_isolation"] and result["sole_setpoint_publisher"]
    assert not check_graph(truth+"\nNode name: impact_supervisor\nEndpoint type: SUBSCRIPTION\n",setpoint)["truth_isolation"]
    assert not check_graph(truth,setpoint.replace("count: 1","count: 2"))["sole_setpoint_publisher"]


def test_extnav_graph_checks_direction_and_unique_endpoints():
    status = "Publisher count: 1\nNode name: xq_p4_external_nav\nEndpoint type: PUBLISHER\nGID: aa\n"
    output = ("Publisher count: 1\nNode name: xq_p4_external_nav\nEndpoint type: PUBLISHER\nGID: aa\n"
              "Subscription count: 1\nNode name: mavros\nEndpoint type: SUBSCRIPTION\nGID: bb\n")
    result = check_extnav(status, output)
    assert result["status_single_publisher"]
    assert result["output_single_adapter_publisher"]
    assert result["mavros_output_subscription"]
    duplicate = status + status.replace("GID: aa", "GID: cc")
    assert not check_extnav(duplicate, output)["status_single_publisher"]


def test_busy_port_is_not_stolen(monkeypatch,tmp_path):
    if not impact.sys.platform.startswith("linux"): pytest.skip("Linux locking contract")
    with socket.socket() as sock:
        try: sock.bind(("127.0.0.1",5760))
        except OSError: pytest.skip("port already occupied")
        sock.listen()
        with pytest.raises(RuntimeError,match="5760"):
            with impact.slot_lock(): pass


def test_summary_counts_tasks_not_retried_frames(tmp_path):
    for i,(status,complete) in enumerate((("ERROR",False),("FAIL",True),("PASS",True))):
        impact.write_json(tmp_path/str(i)/"run.json",dict(scenario="normal",strategy="recovery",seed=4,
            status=status,session_id=str(i),completed_record=complete,
            mission=dict(task_reason="TASK_TIMEOUT"),evaluation={}))
    impact.summarize(tmp_path)
    stats=impact.read_json(tmp_path/"task-statistics.json")["groups"]["normal/recovery"]
    assert stats == dict(n_tasks=1,success_rate=0.,timeout_rate=1.)
    assert len((tmp_path/"tasks.csv").read_text().splitlines()) == 2


def test_source_change_during_build_invalidates_manifest(tmp_path,monkeypatch):
    if not impact.sys.platform.startswith("linux"): pytest.skip("Linux build")
    monkeypatch.setattr(impact,"build_root",lambda:tmp_path)
    hashes=iter(("before","after"))
    monkeypatch.setattr(impact,"source_hash",lambda:next(hashes))
    monkeypatch.setattr(impact.shutil,"copytree",lambda *a,**kw:None)
    monkeypatch.setattr(impact.subprocess,"call",lambda *a,**kw:0)
    impact.write_json(tmp_path/"build-manifest.json",dict(source_sha256="old"))
    with pytest.raises(RuntimeError,match="source changed"):
        impact.build(cpu_only=True)
    assert not (tmp_path/"build-manifest.json").exists()


def test_batch_resume_preserves_failed_outcome(tmp_path,monkeypatch):
    cfg=dict(scenarios=["normal"],strategies=["recovery"],test_seeds=[0,1],calibration="cal.json")
    monkeypatch.setattr(impact,"config",lambda:cfg)
    monkeypatch.setattr(impact,"source_hash",lambda:"frozen")
    monkeypatch.setattr(impact,"digest",lambda p:"hash")
    monkeypatch.setattr(impact,"external_fingerprint",lambda:{})
    marker=tmp_path/"validation.json"
    impact.write_json(marker,dict(status="PASS",source_sha256="frozen",profile="server_gpu",external={}))
    impact.write_json(tmp_path/"failed"/"run.json",dict(completed_record=True,status="FAIL",source_sha256="frozen",
        scenario="normal",strategy="recovery",seed=0,session_id="failed"))
    calls=[]
    def run(args):
        calls.append(args.seed)
        return tmp_path,dict(status="FAIL",completed_record=True)
    monkeypatch.setattr(impact,"experiment",run)
    impact.batch(argparse.Namespace(jobs=1,profile="server_gpu",results=str(tmp_path),validation=str(marker)))
    assert calls == [1]


def test_stage_a_report_is_per_run_and_summary_accumulates(tmp_path, monkeypatch):
    run = tmp_path / "normal-baseline-s1000"
    run.mkdir()
    impact.write_json(run / "run.json", dict(status="PASS", source_sha256="frozen", config_sha256="cfg",
        completed_record=True, scenario="normal", strategy="baseline", seed=1000,
        mission=dict(gate="STAGE_A", task_success=True, termination_confirmed=True),
        evaluation=dict(status="PASS", samples=100, collision_events=0)))
    (run / "rosbag").mkdir(); (run / "rosbag/metadata.yaml").write_text("topics:\n")
    (run / "sitl_runtime/logs").mkdir(parents=True); (run / "sitl_runtime/logs/1.BIN").write_bytes(b"x")
    (run / "events.jsonl").write_text('{"event":"CERTIFY","accepted":true,"trajectory_id":1}\n')
    (run / "stage-a-map-audit.json").write_text('{"status":"UNVERIFIED"}\n')
    monkeypatch.setattr(impact, "experiment", lambda args: (run, impact.read_json(run / "run.json")))
    args = argparse.Namespace(results=str(tmp_path), scenario="normal", strategy="baseline", seed=1000)
    assert impact.stage_a(args) == 1
    assert (run / "stage-a-validation.json").exists()
    assert len(impact.read_json(tmp_path / "stage-a-summary.json")["runs"]) == 1
