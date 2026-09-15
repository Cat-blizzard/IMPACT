#!/usr/bin/env python3
"""Portable IMPACT development, isolated SITL experiments and evidence delivery."""
from __future__ import annotations
import argparse
import contextlib
import csv
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import socket
import subprocess
import sys
import tarfile
import time
import uuid
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from impact_scenarios import generate


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    temp.replace(path)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1048576), b""):
            h.update(block)
    return h.hexdigest()


def capture(command, timeout=20):
    try:
        p = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           timeout=timeout, errors="replace")
        return dict(code=p.returncode, output=p.stdout.strip())
    except (OSError, subprocess.TimeoutExpired) as error:
        return dict(code=-1, output=str(error))


def config():
    return read_json(ROOT / "config/impact_v1.json")


def build_root():
    suffix = hashlib.sha256(str(ROOT).encode()).hexdigest()[:10]
    return Path(os.environ.get("IMPACT_BUILD_ROOT", f"/var/tmp/impact-{os.getuid() if hasattr(os,'getuid') else 'win'}-{suffix}"))


def source_hash():
    records = []
    for directory in ("src", "scripts", "config"):
        for path in sorted((ROOT / directory).rglob("*")):
            if path.is_file() and not any(p in ("__pycache__", ".pytest_cache") for p in path.parts) and path.suffix != ".pyc":
                records.append((path.relative_to(ROOT).as_posix(), digest(path)))
    # pathlib ordering differs on Windows (case-folded) and Linux. Hash a
    # canonical POSIX-path ordering so the same snapshot has one identity.
    return hashlib.sha256(json.dumps(sorted(records)).encode()).hexdigest()


def external_fingerprint():
    paths = {"arducopter": Path(os.environ.get("ARDUPILOT_ROOT", "/nonexistent"))/"build/sitl/bin/arducopter",
             "gazebo_plugin": Path(os.environ.get("ARDUPILOT_GAZEBO_ROOT", "/nonexistent"))/"build/libArduPilotPlugin.so"}
    return {name:digest(path) if path.is_file() else None for name,path in paths.items()}


def doctor(profile, runtime=False):
    report = dict(profile=profile, platform=platform.platform(), checks={}, ready=False)
    checks = report["checks"]
    checks["linux"] = sys.platform.startswith("linux")
    checks["ros_humble"] = Path("/opt/ros/humble/setup.bash").is_file()
    checks["colcon"] = bool(shutil.which("colcon"))
    checks["calibration"] = (ROOT / config()["calibration"]).is_file()
    report["tools"] = {name: capture(cmd) for name, cmd in {
        "python": [sys.executable,"--version"], "gazebo": ["gz","sim","--version"],
        "gpu": ["nvidia-smi","--query-gpu=name,uuid,driver_version","--format=csv,noheader"],
        "glx": ["glxinfo","-B"], "ffmpeg": ["ffmpeg","-version"]}.items()}
    if runtime:
        checks["gazebo"] = report["tools"]["gazebo"]["code"] == 0
        checks["installation"] = (build_root() / "install/setup.bash").is_file()
        ap = Path(os.environ.get("ARDUPILOT_ROOT", "/nonexistent"))
        plugin = Path(os.environ.get("ARDUPILOT_GAZEBO_ROOT", "/nonexistent"))
        checks["arducopter"] = (ap / "build/sitl/bin/arducopter").is_file()
        checks["ardupilot_plugin"] = (plugin / "build/libArduPilotPlugin.so").is_file()
        if profile == "server_gpu":
            glx = report["tools"]["glx"]
            renderer = glx["output"].lower()
            checks["hardware_gl"] = glx["code"] == 0 and "nvidia" in renderer and not any(
                token in renderer for token in ("llvmpipe", "softpipe", "software rasterizer"))
            checks["nvidia_driver"] = report["tools"]["gpu"]["code"] == 0
    report["ready"] = all(checks.values())
    report["resources"] = dict(cpu_count=os.cpu_count(),
        free_disk_bytes=shutil.disk_usage(ROOT).free,
        wsl=Path("/proc/version").exists() and "microsoft" in Path("/proc/version").read_text().lower(),
        memory=capture(["free","-b"]))
    return report


def build(cpu_only=False):
    if not sys.platform.startswith("linux"):
        raise RuntimeError("Build inside WSL Ubuntu 22.04, not native Windows")
    owned = build_root().resolve()
    if str(owned).startswith("/mnt/") or owned in (Path("/"), Path("/var"), Path("/var/tmp")):
        raise ValueError("IMPACT_BUILD_ROOT must be a dedicated Linux-filesystem directory")
    owned.mkdir(parents=True, exist_ok=True)
    frozen_source = source_hash()
    # An interrupted/failed build must not leave a usable old approval manifest.
    (owned / "build-manifest.json").unlink(missing_ok=True)
    source = owned / "source"
    # Owned staging copy: build only the current source tree, including uncommitted edits.
    if source.exists():
        if not (owned / ".impact-managed").exists():
            raise RuntimeError("refusing to replace an unowned staging tree")
        shutil.rmtree(source)
    (owned / ".impact-managed").touch()
    shutil.copytree(ROOT / "src", source / "src", ignore=shutil.ignore_patterns("__pycache__", ".pytest_cache"))
    command = ["bash", str(ROOT / "scripts/impact_build.sh"), str(owned), "cpu" if cpu_only else "full"]
    status = subprocess.call(command)
    if status:
        raise RuntimeError(f"colcon failed ({status}); logs: {owned / 'log'}")
    if source_hash() != frozen_source:
        raise RuntimeError("source changed during staging/build; run build again")
    manifest = dict(source_sha256=frozen_source, build_kind="cpu" if cpu_only else "full",
                    platform=platform.platform(), git=capture(["git","-C",str(ROOT),"rev-parse","HEAD"]),
                    created_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ",time.gmtime()))
    manifest["packages"] = capture(["dpkg-query","-W","ros-humble-*","libgz-*","gz-*"])
    write_json(owned / "build-manifest.json", manifest)
    print(f"Build passed: {owned}; runtime flight validation still pending")


def tests(pure=False):
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join((str(ROOT / "src/xq_autonomy"), str(ROOT / "scripts"), env.get("PYTHONPATH", "")))
    if pure:
        return subprocess.call([sys.executable, "-m", "pytest", "-q", "--rootdir",str(ROOT),
                                str(ROOT / "tests")], env=env,cwd=ROOT)
    return subprocess.call(["bash",str(ROOT/"scripts/impact_test.sh"),str(ROOT),str(build_root())],env=env,cwd=ROOT)


@contextlib.contextmanager
def slot_lock():
    """One host-wide P16 slot until independently verified multi-GPU isolation exists."""
    if not sys.platform.startswith("linux"):
        raise RuntimeError("run requires Linux/WSL")
    import fcntl
    with open("/var/tmp/impact-sitl-slot.lock", "a") as lock:
        try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError: raise RuntimeError("another IMPACT SITL owns this host slot")
        # Check every P5 fixed endpoint before spawning; never kill existing owners.
        for port, kind in ((5760, socket.SOCK_STREAM), (9002, socket.SOCK_DGRAM)):
            with socket.socket(socket.AF_INET, kind) as s:
                try: s.bind(("127.0.0.1", port))
                except OSError: raise RuntimeError(f"required port {port} is already in use")
        yield


def runtime_env(profile, run):
    env = os.environ.copy()
    for key in ("LIBGL_ALWAYS_SOFTWARE", "MESA_LOADER_DRIVER_OVERRIDE", "GALLIUM_DRIVER", "EGL_PLATFORM",
                "QT_QPA_PLATFORM", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH", "PYTHONPATH",
                "LD_LIBRARY_PATH", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE", "CYCLONEDDS_URI"):
        env.pop(key, None)
    env.update(IMPACT_INSTALL=str(build_root()/"install"), IMPACT_RUN=str(run),
        ROS_DOMAIN_ID="170", ROS_LOCALHOST_ONLY="1", ROS2CLI_NO_DAEMON="1",
        GZ_PARTITION="impact_"+run.name, ROS_LOG_DIR=str(run/"ros_logs"))
    if profile == "local_cpu":
        env.update(LIBGL_ALWAYS_SOFTWARE="1", MESA_LOADER_DRIVER_OVERRIDE="llvmpipe",
                   GALLIUM_DRIVER="llvmpipe", EGL_PLATFORM="surfaceless", QT_QPA_PLATFORM="offscreen")
    else:
        env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    return env


def bundle(run):
    run = Path(run).resolve()
    output = run.parent / (run.name + "-diagnostics.tar.gz")
    inventory = [dict(path=p.relative_to(run).as_posix(), bytes=p.stat().st_size, sha256=digest(p))
                 for p in sorted(run.rglob("*")) if p.is_file() and not p.is_symlink()
                 and p.name != "artifact-inventory.json"]
    write_json(run/"artifact-inventory.json", inventory)
    with tarfile.open(output, "w:gz") as archive:
        for file in sorted(run.rglob("*")):
            if file.is_file() and file.suffix in (".json", ".jsonl", ".log", ".txt", ".yaml", ".sdf"):
                # Keep huge bags/replay assets in place; record their inventory separately.
                if file.stat().st_size <= 20*1024*1024:
                    archive.add(file, arcname=str(file.relative_to(run)), recursive=False)
    return output


def experiment(args):
    root = Path(args.results).resolve()
    run = root / f"{args.scenario}-{args.strategy}-s{args.seed}-{time.strftime('%Y%m%dT%H%M%S')}-{uuid.uuid4().hex[:8]}"
    run.mkdir(parents=True)
    report = dict(schema_version=1, validation="SIMULATED", status="PREPARING", profile=args.profile,
                  scenario=args.scenario, strategy=args.strategy, seed=args.seed,
                  session_id=uuid.uuid4().hex, source_sha256=source_hash(), config_sha256=digest(ROOT/"config/impact_v1.json"))
    report["configuration"] = config()
    write_json(run/"run.json", report)
    try:
        preflight = doctor(args.profile, runtime=True)
        write_json(run/"doctor.json", preflight)
        if not preflight["ready"]:
            raise RuntimeError("preflight failed: " + ", ".join(k for k,v in preflight["checks"].items() if not v))
        manifest = read_json(build_root()/"build-manifest.json")
        if manifest["source_sha256"] != report["source_sha256"] or manifest["build_kind"] != "full":
            raise RuntimeError("full build is missing or source changed; rebuild first")
        write_json(run/"build-manifest.json", manifest)
        shutil.copy2(ROOT/config()["calibration"], run/"calibration.json")
        generate(ROOT, run, args.scenario, args.seed)
        write_json(run/"config.json", config())
        report["world_sha256"] = digest(run/"world.sdf")
        report["calibration_sha256"] = digest(run/"calibration.json")
        report["status"] = "RUNNING"
        write_json(run/"run.json", report)
        env = runtime_env(args.profile, run)
        with slot_lock(), open(run/"launcher.log", "w") as log:
            process = subprocess.Popen(["bash",str(ROOT/"scripts/impact_run.sh"), str(run), args.profile],
                                       env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = process.wait(timeout=config()["run_timeout_wall_s"])
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                raise RuntimeError("launcher interrupted or wall watchdog expired")
        mission = read_json(run/"mission.json")
        evaluation = read_json(run/"evaluation.json")
        performance(run)
        report.update(mission=mission, evaluation=evaluation, launcher_exit_code=code)
        # A task failure is a valid experimental outcome, not an infrastructure pass.
        report["status"] = "PASS" if (code == 0 and mission["task_success"] and mission["termination_confirmed"]
            and evaluation["samples"] >= 50 and evaluation["collision_events"] == 0) else "FAIL"
        report["completed_record"] = bool(code == 0 and mission["termination_confirmed"] and evaluation["samples"] >= 50)
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        report.update(status="ERROR", reason=str(error), completed_record=False)
    finally:
        write_json(run/"run.json", report)
        if report["status"] != "PASS": bundle(run)
    print(f"{report['status']}: {run}")
    return run, report


def batch(args):
    if args.jobs != 1:
        raise RuntimeError("parallel flight disabled until hardware GPU-binding and isolation are verified; use --jobs 1")
    root = Path(args.results).resolve()
    root.mkdir(parents=True, exist_ok=True)
    validation = read_json(Path(args.validation))
    if (validation.get("status") != "PASS" or validation.get("source_sha256") != source_hash()
        or validation.get("profile") != args.profile or validation.get("external") != external_fingerprint()):
        raise RuntimeError("server validation missing/stale: rerun validate-server before formal batch")
    frozen_hash = source_hash()
    protocol = dict(source_sha256=frozen_hash, config_sha256=digest(ROOT/"config/impact_v1.json"),
                    profile=args.profile, calibration_sha256=digest(ROOT/config()["calibration"]),
                    matrix=config(), external=external_fingerprint())
    path = root/"protocol.json"
    if path.exists() and read_json(path) != protocol:
        raise RuntimeError("results directory belongs to a different frozen protocol; use a new results directory")
    write_json(path, protocol)
    completed = set()
    for file in root.glob("*/run.json"):
        r = read_json(file)
        if r.get("completed_record") and r.get("source_sha256") == frozen_hash:
            completed.add((r["scenario"],r["strategy"],r["seed"]))
    for scenario in config()["scenarios"]:
        for seed in config()["test_seeds"]:
            for strategy in config()["strategies"]:
                if (scenario,strategy,seed) in completed:
                    continue
                if source_hash() != frozen_hash or external_fingerprint() != protocol["external"]:
                    raise RuntimeError("source/external binaries changed during batch")
                args.scenario, args.strategy, args.seed = scenario, strategy, seed
                _, report = experiment(args)
                summarize(root)
                if report["status"] == "ERROR" or not report.get("completed_record"):
                    raise RuntimeError("infrastructure/termination failure; batch paused, inspect diagnostic bundle")
    summarize(root)


def validate_server(args):
    """Serial stage gates: original P5, ordinary closed loop, then all four arms."""
    if args.profile != "server_gpu":
        raise RuntimeError("formal server validation requires server_gpu")
    root = Path(args.results).resolve()
    root.mkdir(parents=True, exist_ok=True)
    marker = root/"server-validation.json"
    marker.unlink(missing_ok=True)
    frozen = source_hash()
    legacy = ROOT/"experiments/results/baseline_v1"/("p5_server_"+uuid.uuid4().hex[:8])
    legacy.mkdir(parents=True)
    passed = []
    try:
        preflight=doctor(args.profile, runtime=True)
        write_json(legacy/"doctor.json",preflight)
        if not preflight["ready"]: raise RuntimeError("server runtime doctor failed")
        manifest=read_json(build_root()/"build-manifest.json")
        if manifest["source_sha256"] != frozen or manifest["build_kind"] != "full":
            raise RuntimeError("current full build required")
        env=runtime_env(args.profile,legacy)
        env.update(IMPACT_PROFILE=args.profile,IMPACT_BUILD_MANIFEST=str(build_root()/"build-manifest.json"))
        with slot_lock(), (legacy/"launcher.log").open("w") as log:
            code=subprocess.call(["bash",str(ROOT/"scripts/run_p5_baseline.sh"),"--run-dir",str(legacy)],env=env,stdout=log,stderr=subprocess.STDOUT)
        if code: raise RuntimeError(f"original P5 failed: {legacy}")
        passed.append(str(legacy))
        cases=[("normal","baseline"),("normal","recovery")]+[("recoverable",s) for s in config()["strategies"]]
        for scenario,strategy in cases:
            args.scenario,args.strategy,args.seed=scenario,strategy,1000
            run,report=experiment(args)
            if not report.get("completed_record") or (scenario == "normal" and report["status"] != "PASS"):
                raise RuntimeError(f"stage failed: {run}")
            passed.append(str(run))
        if source_hash() != frozen: raise RuntimeError("source changed during validation")
        write_json(marker,dict(status="PASS",source_sha256=frozen,profile=args.profile,runs=passed,external=external_fingerprint(),
            note="Infrastructure and normal-task validation passed. Recovery benefit is an experimental result, not assumed."))
        print(f"Server stages passed: {marker}")
    except (OSError,ValueError,RuntimeError,KeyError) as error:
        write_json(marker,dict(status="FAIL",source_sha256=frozen,profile=args.profile,runs=passed,reason=str(error)))
        bundle(legacy)
        raise


def performance(run):
    import numpy as np
    path=Path(run)/"events.jsonl"
    events=[json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []
    values=[r["elapsed_wall_s"] for r in events if r["event"] == "CERTIFICATION_TIMING"]
    timing=dict(samples=len(values), p50_s=None, p95_s=None, p99_s=None, over_100ms=0)
    if values:
        timing.update(zip(("p50_s","p95_s","p99_s"),map(float,np.percentile(values,[50,95,99]))))
        timing["over_100ms"]=sum(v>0.1 for v in values)
    span=events[-1]["wall_time"]-events[0]["wall_time"] if len(events)>1 else 0
    write_json(Path(run)/"performance.json",dict(certification_compute_wall=timing,
        observed_sim_realtime_factor=(events[-1]["sim_time"]-events[0]["sim_time"])/span if span>0 else None,
        recovery_events=[r for r in events if r["event"] == "RECOVERY_CONFIRMED"],
        note="Certification timing only, not end-to-end latency. RTF is the event observation interval; performance comparison requires serial controlled load."))


def summarize(root):
    rows = [read_json(p) for p in sorted(Path(root).glob("*/run.json"))]
    groups = {}
    # Preserve failed tasks; retries remain separate records and are explicitly marked.
    for row in rows:
        key = row["scenario"]+"/"+row["strategy"]
        group = groups.setdefault(key, dict(attempts=0, success=0, failure=0, error=0, runs=[]))
        group["attempts"] += 1
        group[{"PASS":"success","FAIL":"failure"}.get(row["status"],"error")] += 1
        group["runs"].append(dict(seed=row["seed"], status=row["status"], session_id=row["session_id"],
                                  evaluation=row.get("evaluation"), mission=row.get("mission")))
    write_json(Path(root)/"summary.json", dict(validation="SIMULATED", groups=groups,
        note="Attempts include retries; inspect per-seed records before statistical inference. Failures are not discarded."))
    # One independent completed task per paired seed; infrastructure attempts
    # remain in summary.json and never become successful experimental samples.
    selected = {}
    for row in rows:
        if row.get("completed_record"):
            key = (row["scenario"], row["strategy"], row["seed"])
            selected.setdefault(key, row)
    columns = ["scenario", "strategy", "seed", "session_id", "status", "task_success", "timeout",
               "task_time_sim_s", "ate_rms_m", "minimum_truth_clearance_m", "collision_events",
               "path_length_m", "stopped_time_sim_s", "pl_coverage", "availability"]
    with (Path(root)/"tasks.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns); writer.writeheader()
        for key, row in sorted(selected.items()):
            mission, evaluation = row.get("mission", {}), row.get("evaluation", {})
            writer.writerow(dict(**{k: row[k] for k in columns[:5]},
                task_success=int(row["status"] == "PASS"), timeout=int(mission.get("task_reason") == "TASK_TIMEOUT"),
                task_time_sim_s=mission.get("elapsed_sim_s"), **{k:evaluation.get(k) for k in columns[8:]}))
    statistics = {}
    for scenario in config()["scenarios"]:
        for strategy in config()["strategies"]:
            tasks = [v for k,v in selected.items() if k[:2] == (scenario,strategy)]
            if tasks:
                statistics[scenario+"/"+strategy] = dict(n_tasks=len(tasks),
                    success_rate=sum(r["status"] == "PASS" for r in tasks)/len(tasks),
                    timeout_rate=sum(r.get("mission",{}).get("task_reason") == "TASK_TIMEOUT" for r in tasks)/len(tasks))
    write_json(Path(root)/"task-statistics.json", dict(unit="independent task", groups=statistics,
        note="First completed record per scenario/strategy/seed. All task failures retained; infrastructure retries separate. No frame-level significance tests."))


def stage_a(args):
    """Run one independent Stage A goal-navigation task and write its gate record."""
    root = Path(args.results).resolve()
    root.mkdir(parents=True, exist_ok=True)
    run, report = experiment(args)
    map_audit_path = run / "stage-a-map-audit.json"
    subprocess.run([sys.executable, str(ROOT / "scripts/diagnose_stage_a_maps.py"), str(run),
                    "--output", str(map_audit_path)], check=True)
    mission = report.get("mission", {})
    evaluation = report.get("evaluation", {})
    events = []
    event_file = run / "events.jsonl"
    if event_file.is_file():
        for line in event_file.read_text().splitlines():
            if line.strip():
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    map_audit = run / "stage-a-map-audit.json"
    map_report = read_json(map_audit) if map_audit.is_file() else {}
    checks = {
        "protocol_is_stage_a": True,
        "not_legacy_p5_gate": mission.get("gate") != "P5_BASELINE_MAP_FRONTIER_EGO",
        "mission_record_present": bool(mission),
        "evaluation_pass": evaluation.get("status") == "PASS",
        "evaluation_samples": int(evaluation.get("samples", 0)) >= 50,
        "evaluation_collision_free": int(evaluation.get("collision_events", 0)) == 0,
        "termination_record_present": "termination_confirmed" in mission,
        "termination_confirmed": bool(mission.get("termination_confirmed")),
        "rosbag_metadata_present": (run / "rosbag" / "metadata.yaml").is_file(),
        "dataflash_present": any((run / "sitl_runtime" / "logs").glob("*.BIN")),
        "certification_events_recorded": any(event.get("event") == "CERTIFY" for event in events),
        "authorization_linkage_recorded": any(
            event.get("event") == "CERTIFY" and "trajectory_id" in event and "accepted" in event
            for event in events
        ),
        "revocation_behavior_recorded": any(event.get("event") == "REVOKE" for event in events) or
            any(event.get("event") == "CERTIFY" and event.get("accepted") is False for event in events),
        "map_content_verified": map_report.get("status") == "MAP_CONTENT_VERIFIED",
        "completed_task": bool(report.get("completed_record")),
        "task_success": report.get("status") == "PASS" and bool(mission.get("task_success")),
    }
    gate = {
        "schema_version": 1,
        "gate": "STAGE_A_GOAL_NAVIGATION_ACCEPTANCE",
        "status": "PASS" if all(checks.values()) and checks["task_success"] else "FAIL",
        "run": str(run),
        "source_sha256": report.get("source_sha256"),
        "config_sha256": report.get("config_sha256"),
        "scenario": args.scenario,
        "strategy": args.strategy,
        "seed": args.seed,
        "checks": checks,
        "map_audit": map_report,
        "event_evidence": {
            "count": len(events),
            "certify": sum(event.get("event") == "CERTIFY" for event in events),
            "revoke": sum(event.get("event") == "REVOKE" for event in events),
            "clock_reset": sum(event.get("event") == "CLOCK_RESET" for event in events),
        },
        "task_result": mission.get("task_result", {"status": report.get("status")} ),
        "termination": mission.get("termination", {"confirmed": mission.get("termination_confirmed")} ),
        "note": "Stage A target-navigation acceptance; never evidence for legacy P5 exploration or Stage B recovery benefit.",
    }
    write_json(run / "stage-a-validation.json", gate)
    summary_path = root / "stage-a-summary.json"
    summary = read_json(summary_path) if summary_path.is_file() else {
        "schema_version": 1, "gate": "STAGE_A_GOAL_NAVIGATION_ACCEPTANCE", "runs": []
    }
    summary["runs"] = [item for item in summary["runs"] if item.get("run") != str(run)]
    summary["runs"].append({"run": str(run), "status": gate["status"], "checks": checks})
    write_json(summary_path, summary)
    print(json.dumps(gate, ensure_ascii=False, indent=2))
    return 0 if gate["status"] == "PASS" else 1


def package(output):
    """Export tracked and non-ignored source, including uncommitted modifications."""
    output = Path(output).resolve()
    listed = subprocess.run(["git", "-C", str(ROOT), "ls-files", "-z", "--cached", "--others", "--exclude-standard"],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if listed.returncode:
        raise RuntimeError("package requires the development Git checkout")
    names = sorted(set(n.decode("utf-8") for n in listed.stdout.split(b"\0") if n))
    files = []
    for name in names:
        p = ROOT/name
        if p.is_file() and not p.is_symlink() and p.resolve() != output:
            files.append((name, p))
    manifest = dict(schema_version=1, source_sha256=source_hash(), git=capture(["git","-C",str(ROOT),"rev-parse","HEAD"]),
        dirty_worktree=True, includes_uncommitted_files=True,
        files=[dict(path=n,bytes=p.stat().st_size,sha256=digest(p)) for n,p in files])
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output,"w",compression=zipfile.ZIP_DEFLATED) as archive:
        for name, p in files: archive.write(p, "IMPACT/"+name)
        archive.writestr("IMPACT/source-snapshot.json", json.dumps(manifest,indent=2))
    output.with_suffix(output.suffix+".sha256").write_text(digest(output)+"  "+output.name+"\n")
    print(f"Source snapshot: {output} ({len(files)} files); rebuild on destination")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    d = sub.add_parser("doctor")
    d.add_argument("--profile", choices=("local_cpu","server_gpu"), default="local_cpu")
    d.add_argument("--runtime", action="store_true")
    b = sub.add_parser("build"); b.add_argument("--cpu-only", action="store_true")
    t=sub.add_parser("test"); t.add_argument("--pure", action="store_true")
    for name in ("run", "stage-a", "batch", "validate-server"):
        s = sub.add_parser(name)
        s.add_argument("--profile", choices=("local_cpu","server_gpu"), default="server_gpu")
        s.add_argument("--results", default=str(ROOT/"experiments/results/impact_v1"))
        if name in ("run", "stage-a"):
            s.add_argument("--scenario", choices=config()["scenarios"], default="normal")
            s.add_argument("--strategy", choices=config()["strategies"], default="recovery")
            s.add_argument("--seed", type=int, default=1000)
        elif name == "batch":
            s.add_argument("--jobs", type=int, default=1)
            s.add_argument("--validation",default=str(ROOT/"experiments/results/impact_v1/server-validation.json"))
    s=sub.add_parser("bundle"); s.add_argument("run_dir")
    s=sub.add_parser("package"); s.add_argument("--output", required=True)
    s=sub.add_parser("summarize"); s.add_argument("results")
    s=sub.add_parser("replay"); s.add_argument("run_dir"); s.add_argument("--kind",choices=("rosbag","gazebo"),default="gazebo")
    s=sub.add_parser("render"); s.add_argument("run_dirs", nargs=3); s.add_argument("--output", required=True)
    args=p.parse_args()
    try:
        if args.command == "doctor":
            data=doctor(args.profile,args.runtime); print(json.dumps(data,indent=2)); return 0 if data["ready"] else 2
        if args.command == "build": build(args.cpu_only)
        elif args.command == "test": return tests(args.pure)
        elif args.command == "run": return 0 if experiment(args)[1]["status"] == "PASS" else 1
        elif args.command == "stage-a": return stage_a(args)
        elif args.command == "batch": batch(args)
        elif args.command == "validate-server": validate_server(args)
        elif args.command == "bundle": print(bundle(args.run_dir))
        elif args.command == "package": package(args.output)
        elif args.command == "summarize": summarize(Path(args.results))
        elif args.command == "replay":
            return subprocess.call(["bash",str(ROOT/"scripts/impact_replay.sh"),str(Path(args.run_dir).resolve()),args.kind,str(build_root()/"install")])
        elif args.command == "render":
            from impact_render import render
            render([Path(p) for p in args.run_dirs],Path(args.output))
        return 0
    except (RuntimeError, ValueError, OSError) as error:
        print(f"ERROR: {error}",file=sys.stderr); return 2


if __name__ == "__main__":
    raise SystemExit(main())
