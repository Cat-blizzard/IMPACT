#!/usr/bin/env python3
"""Pause only a selected run's estimator, then resume it for landing evidence."""

import argparse
import json
import os
from pathlib import Path
import signal
import time


def identity(pid):
    proc = Path("/proc") / str(pid)
    fields = (proc / "stat").read_text().rsplit(")", 1)[1].split()
    return dict(pid=pid, pgid=int(fields[2]), start_ticks=int(fields[19]),
                argv=(proc / "cmdline").read_bytes().split(b"\0"))


def latest_telemetry(path):
    with path.open("rb") as stream:
        stream.seek(max(0, path.stat().st_size - 65536))
        data = stream.read()
    complete = data.splitlines()[1:]
    if complete and not data.endswith(b"\n"):
        complete.pop()
    return [json.loads(line) for line in complete if line.strip()]


def active_phase(run):
    path = run / "mission.log"
    if not path.exists() or (run / "mission.json").exists():
        return False
    transitions = [line.split("TRANSITION: ", 1)[1].split(":", 1)[0]
                   for line in path.read_text().splitlines() if "TRANSITION: " in line]
    return bool(transitions and transitions[-1] == "ACTIVE")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--pause-s", type=float, default=2.0)
    parser.add_argument("--wait-s", type=float, default=180.0)
    args = parser.parse_args()
    if not 0.8 <= args.pause_s <= 3.0 or not 0 < args.wait_s <= 300:
        raise ValueError("probe requires a bounded 0.8-3 s pause and <=300 s wait")
    run = args.run_dir.resolve()
    if not run.is_relative_to(Path(__file__).resolve().parents[1] / "experiments/results"):
        raise ValueError("probe must target an IMPACT experiment directory")
    record = json.loads((run / "run.json").read_text())
    if record["run_kind"] != "development":
        raise ValueError("fault probe is forbidden for smoke, acceptance and formal runs")
    output = run / "inflight-localization-loss-probe.json"
    if output.exists():
        raise ValueError("refusing to repeat or overwrite a fault probe")
    report = dict(schema_version=1, diagnostic_only=True, acceptance_sample=False,
                  fault="FAST_LIO_PROCESS_PAUSE", status="WAITING", run=str(run),
                  session_id=record["session_id"], source_sha256=record["source_sha256"],
                  pause_requested_s=args.pause_s, events=[])

    def save():
        temporary = output.with_suffix(".tmp")
        temporary.write_text(json.dumps(report, indent=2) + "\n")
        temporary.replace(output)

    def event(name, **values):
        report["events"].append(dict(event=name, wall_time_s=time.time(), **values))
        save()

    save()
    target = None
    stopped = False
    try:
        deadline = time.monotonic() + args.wait_s
        while time.monotonic() < deadline:
            if (run / "mission.json").exists():
                raise RuntimeError("mission ended before fault injection")
            telemetry_path = run / "telemetry.jsonl"
            if telemetry_path.exists():
                telemetry = latest_telemetry(telemetry_path)
                # Enough ACTIVE samples for independent task-failure evaluation.
                if (len(telemetry) >= 60 and telemetry[-1].get("authorized") is True
                        and time.time() - telemetry_path.stat().st_mtime < 0.5
                        and active_phase(run)):
                    stack_group = int((run / "stack.pid").read_text().strip())
                    matches = []
                    for proc in Path("/proc").iterdir():
                        if not proc.name.isdigit():
                            continue
                        try:
                            candidate = identity(int(proc.name))
                        except (FileNotFoundError, ProcessLookupError, PermissionError):
                            continue
                        if candidate["pgid"] == stack_group and any(
                                Path(arg.decode()).name == "fastlio_mapping"
                                for arg in candidate["argv"] if arg):
                            matches.append(candidate)
                    if len(matches) != 1:
                        raise RuntimeError(f"expected one run-local estimator; found {len(matches)}")
                    target = matches[0]
                    break
            time.sleep(0.1)
        if target is None:
            raise RuntimeError("no authorized ACTIVE flight before probe deadline")
        if identity(target["pid"]) != target:
            raise RuntimeError("estimator process identity changed")
        if not active_phase(run) or time.time() - telemetry_path.stat().st_mtime >= 0.5:
            raise RuntimeError("ACTIVE flight evidence expired before fault injection")
        event("TARGET_VERIFIED", pid=target["pid"], pgid=target["pgid"],
              start_ticks=target["start_ticks"], telemetry=telemetry[-1])
        os.kill(target["pid"], signal.SIGSTOP)
        stopped = True
        pause_start = time.monotonic()
        event("ESTIMATOR_PAUSED")
        time.sleep(args.pause_s)
        os.kill(target["pid"], signal.SIGCONT)
        stopped = False
        event("ESTIMATOR_RESUMED", actual_pause_s=time.monotonic() - pause_start)
        report["status"] = "FAULT_INJECTED_AND_RESUMED"
        save()
    except BaseException as error:
        report["status"] = "ERROR"
        report["error"] = f"{type(error).__name__}: {error}"
        save()
        raise
    finally:
        if stopped and target is not None:
            try:
                if identity(target["pid"]) == target:
                    os.kill(target["pid"], signal.SIGCONT)
                    event("ESTIMATOR_RESUMED_IN_FINALLY")
            except (FileNotFoundError, ProcessLookupError):
                pass
    print(json.dumps(dict(status=report["status"], output=str(output))))


if __name__ == "__main__":
    main()
