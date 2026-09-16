#!/usr/bin/env python3
"""Read-only historical P4 comparison; outputs are separate from original runs."""
import argparse
from bisect import bisect_right
from collections import defaultdict
import json
import math
from pathlib import Path
import re
import sqlite3

from pymavlink import mavutil
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from verify_runtime_build import sha256


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def analyse(run):
    manifest = json.loads((run / "xq-build-manifest.json").read_text())
    mission = json.loads((run / "mission-result.json").read_text())
    df = defaultdict(list)
    bins = sorted((run / "sitl_runtime/logs").glob("*.BIN"))
    for p in bins:
        reader = mavutil.mavlink_connection(str(p))
        while (m := reader.recv_match()) is not None:
            if m.get_type() in ("PARM", "MSG", "ARM", "IMU", "VISP", "VISV", "SIM", "XKF1"):
                df[m.get_type()].append(m.to_dict())
    params = {x["Name"]: x["Value"] for x in df["PARM"]}
    imu = defaultdict(dict)
    for m in df["IMU"]:
        imu[m["TimeUS"]][m["I"]] = m
    differences = [(t / 1e6, math.degrees(math.sqrt(sum(
        (v[0][k] - v[1][k])**2 for k in ("GyrX", "GyrY", "GyrZ")))))
        for t, v in imu.items() if 0 in v and 1 in v]
    arm_df = next((x["TimeUS"] / 1e6 for x in df["ARM"] if x["ArmState"]), math.inf)
    prearm = [x for x in differences if x[0] < arm_df]
    connection = sqlite3.connect(f"file:{run / 'rosbag/rosbag_0.db3'}?mode=ro", uri=True)
    assert connection.execute("pragma quick_check").fetchone()[0] == "ok"
    topics = connection.execute("select id,name,type from topics").fetchall()
    data = {}
    metrics = {}
    selected = ("/clock", "/localization/odom", "/uav1/mavros/odometry/out",
                "/xq/p4/extnav/status", "/uav1/mavros/state",
                "/uav1/mavros/statustext/recv", "/livox/imu", "/xq/eval/p4/ground_truth")
    for tid, name, typ in topics:
        times = [x[0] / 1e9 for x in connection.execute(
            "select timestamp from messages where topic_id=? order by timestamp", (tid,))]
        if not times:
            continue
        metrics[name] = dict(count=len(times), wall_span_s=times[-1] - times[0],
            wall_hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0,
            max_wall_gap_s=max((b-a for a,b in zip(times,times[1:])), default=0))
        if name in selected:
            cls = get_message(typ)
            data[name] = [(t/1e9, deserialize_message(b, cls)) for t,b in connection.execute(
                "select timestamp,data from messages where topic_id=? order by timestamp", (tid,))]
    connection.close()
    clocks = [(t, stamp(m.clock)) for t,m in data["/clock"]]
    clock_times = [x[0] for x in clocks]

    def clock_at(t):
        i = bisect_right(clock_times, t)-1
        return clocks[i][1] if i >= 0 else None

    events = []
    for line in (run / "mavros.log").read_text().splitlines():
        match = re.search(r"\[(\d+\.\d+)\].*FCU: (.*)", line)
        if match:
            wall, message = float(match[1]), match[2]
            events.append(dict(wall_s=wall, ros_clock_s=clock_at(wall), message=message))
    arm_wall = next(e["wall_s"] for e in events if "Arm: Gyros inconsistent" in e["message"])
    window = (arm_wall-10, arm_wall+0.1)
    streams = {}
    for name in selected:
        rows = [(t,m) for t,m in data.get(name, []) if window[0] <= t <= window[1]]
        if not rows:
            continue
        times = [t for t,_ in rows]
        item = dict(count=len(rows), max_wall_gap_s=max((b-a for a,b in zip(times,times[1:])), default=0))
        if hasattr(rows[0][1], "header"):
            stamps = [stamp(m.header.stamp) for _,m in rows]
            item.update(first_stamp=stamps[0], last_stamp=stamps[-1],
                nonincreasing=sum(b<=a for a,b in zip(stamps,stamps[1:])),
                max_stamp_gap_s=max((b-a for a,b in zip(stamps,stamps[1:])), default=0))
        if name == "/xq/p4/extnav/status":
            status = [json.loads(m.data) for _,m in rows]
            item.update(healthy=sum(m.get("healthy") is True for m in status),
                last=status[-1], max_reported_age_s=max(m["source_age_s"] or 0 for m in status))
        streams[name] = item
    output_rows = data["/uav1/mavros/odometry/out"]
    matches = []
    for vis in df["VISP"][:8]:
        wall_stamp = vis["RTimeUS"] / 1e6
        receipt, odom = min(output_rows, key=lambda r:abs(stamp(r[1].header.stamp)-wall_stamp))
        matches.append(dict(dataflash=vis, matched_ros_stamp_error_s=stamp(odom.header.stamp)-wall_stamp,
            ros_bag_receipt_s=receipt, ros_clock_at_publication_s=clock_at(receipt),
            publication_before_arm_warning_s=arm_wall-wall_stamp))
    hashes = {str(p.relative_to(run)):sha256(p) for p in [*bins,
        run/"sitl_runtime/eeprom.bin", run/"rosbag/rosbag_0.db3", run/"mission-result.json",
        run/"mavros.log", run/"sitl.log", run/"xq-build-manifest.json", run/"runtime-dependencies.sha256"]}
    return dict(run=str(run), build_git=manifest["git"], source_sha256=manifest["source_sha256"],
        mission_status=mission["status"], arm_requests=[e for e in mission["events"]
            if e["kind"]=="REQUEST" and e["detail"]=="arm"],
        fcu_timeline=events, imu_instances=sorted({x["I"] for x in df["IMU"]}),
        imu_record_start_s=min(imu)/1e6, imu_record_end_s=max(imu)/1e6,
        gyro_pair_samples_before_successful_arm=len(prearm),
        gyro_pair_max_deg_s_before_successful_arm=max((x[1] for x in prearm), default=None),
        df_visp_count=len(df["VISP"]), df_visp_first=matches, parameters=params,
        bag_metrics=metrics, pre_first_arm_window=streams,
        ros_clock_start_s=clocks[0][1], ros_clock_end_s=clocks[-1][1],
        sim_to_wall_ratio=(clocks[-1][1]-clocks[0][1])/(clocks[-1][0]-clocks[0][0]),
        hashes=hashes,
        limitations=["DataFlash LOG_DISARMED=0 leaves startup/first rejection unobserved.",
            "DataFlash TimeUS and ROS /clock are different epochs; no assumed equality.",
            "ROS publication is not proof of FCU reception or fusion."])


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("runs", nargs=2, type=Path)
    ap.add_argument("--output", required=True, type=Path)
    a=ap.parse_args()
    runs=[analyse(p.resolve()) for p in a.runs]
    p,q=[r.pop("parameters") for r in runs]
    differences={k:[p.get(k),q.get(k)] for k in sorted(p.keys()|q.keys()) if p.get(k)!=q.get(k)}
    report=dict(runs=runs, parameter_differences=differences,
        common_sensor_parameters={k:v for k,v in p.items() if k.startswith(("INS_","VISO_","LOG_","SIM_GYR")) and k not in differences})
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
    print(a.output)


if __name__=="__main__":
    main()
