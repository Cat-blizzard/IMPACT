"""Validate live node graph before arming, plus actual Gazebo renderer libraries."""
import json
from pathlib import Path
import re
import sys


def _endpoints(text, kind):
    result = []
    for section in re.split(r"\n(?=Node name:)", text):
        if f"Endpoint type: {kind}" not in section:
            continue
        node = re.search(r"Node name: (\S+)", section)
        gid = re.search(r"GID: ([0-9a-fA-F.]+)", section)
        ns = re.search(r"Node namespace: (\S+)", section)
        full_gid = gid[1] if gid else None
        result.append({"node": node[1] if node else None, "namespace": ns[1] if ns else None,
                       "gid": full_gid,
                       "participant_gid": ".".join(full_gid.split(".")[:8]) if full_gid else None})
    return result


def check_graph(truth, setpoint, require_evaluator=True, recorder_participants=(),
                mavros_participants=()):
    # ros2 topic info -v sections name endpoint type and node name.
    endpoints = _endpoints(truth, "SUBSCRIPTION")
    subscribers = [item["node"] for item in endpoints]
    recorder_participants = set(recorder_participants)
    resolved = []
    for item in endpoints:
        node = item["node"]
        if node == "_NODE_NAME_UNKNOWN_" and item["participant_gid"] in recorder_participants:
            node = "rosbag2_recorder@gid"
        resolved.append({**item, "resolved_node": node})
    allowed = all(item["resolved_node"] == "impact_evaluator" or
                  item["resolved_node"].startswith("rosbag2_recorder") for item in resolved)
    count = re.search(r"Publisher count: (\d+)", setpoint)
    owner = "Node name: impact_arbiter" in setpoint
    setpoint_subscribers = _endpoints(setpoint, "SUBSCRIPTION")
    mavros_participants = set(mavros_participants)
    resolved_setpoint_subscribers = []
    for item in setpoint_subscribers:
        node, namespace = item["node"], item["namespace"]
        resolution = "reported"
        if node == "_NODE_NAME_UNKNOWN_" and item.get("participant_gid") in mavros_participants:
            node, namespace = "mavros_process@gid", "/uav1/mavros"
            resolution = "participant_gid_from_mavros_state"
        resolved_setpoint_subscribers.append({**item, "resolved_node": node,
            "resolved_namespace": namespace, "identity_resolution": resolution})
    mavros_subscription = any(
        (item["resolved_node"] == "setpoint_raw"
         and item["resolved_namespace"] == "/uav1/mavros")
        or (item["resolved_node"] == "mavros_process@gid"
            and item["resolved_namespace"] == "/uav1/mavros")
        for item in resolved_setpoint_subscribers)
    expected = "impact_evaluator" in [x["resolved_node"] for x in resolved] if require_evaluator else bool(resolved)
    return dict(truth_subscribers=subscribers, truth_endpoints=resolved,
        truth_isolation=allowed and expected,
        sole_setpoint_publisher=bool(count and count[1] == "1" and owner),
        setpoint_subscribers=resolved_setpoint_subscribers,
        mavros_setpoint_subscription=mavros_subscription)


def check_extnav(status_text, output_text, mavros_identity_text="",
                 odometry_node_present=False, odometry_service_present=False):
    """Reject duplicate/stale ExternalNav adapters before arming."""
    status_publishers = _endpoints(status_text, "PUBLISHER")
    output_publishers = _endpoints(output_text, "PUBLISHER")
    output_subscribers = _endpoints(output_text, "SUBSCRIPTION")
    identity_publishers = _endpoints(mavros_identity_text, "PUBLISHER")
    mavros_participants = {
        item["participant_gid"] for item in identity_publishers
        if item.get("node") in {"sys", "sys_status"}
        and item.get("namespace") == "/uav1/mavros"
        and item.get("participant_gid")
    }
    resolved_subscribers = []
    for item in output_subscribers:
        resolved_node = item.get("node")
        resolved_namespace = item.get("namespace")
        resolution = "reported"
        if (resolved_node == "_NODE_NAME_UNKNOWN_"
                and item.get("participant_gid") in mavros_participants):
            resolved_node = "mavros_process@gid"
            resolved_namespace = "/uav1/mavros"
            resolution = "participant_gid_from_mavros_state"
        resolved_subscribers.append({
            **item,
            "resolved_node": resolved_node,
            "resolved_namespace": resolved_namespace,
            "identity_resolution": resolution,
        })
    return dict(status_publishers=status_publishers,
                status_single_publisher=(len(status_publishers) == 1 and
                                         status_publishers[0]["node"] == "xq_p4_external_nav"),
                output_publishers=output_publishers,
                output_single_adapter_publisher=(len(output_publishers) == 1 and
                                                 output_publishers[0]["node"] == "xq_p4_external_nav"),
                output_subscribers=resolved_subscribers,
                mavros_identity_publishers=identity_publishers,
                mavros_identity_participant_gids=sorted(mavros_participants),
                odometry_node_present=odometry_node_present,
                odometry_service_present=odometry_service_present,
                mavros_output_subscription=any(
                    (x.get("namespace") == "/uav1/mavros" and x.get("node") == "odometry")
                    or (x.get("resolved_namespace") == "/uav1/mavros"
                        and x.get("resolved_node") == "mavros_process@gid"
                        and odometry_node_present and odometry_service_present)
                    for x in resolved_subscribers))


def renderer_maps(pid):
    """Collect only this run's Gazebo process group, never a global glxinfo substitute."""
    import os
    maps = []
    pids = []
    for path in Path("/proc").iterdir():
        if path.name.isdigit():
            try:
                if os.getpgid(int(path.name)) == pid:
                    maps.append((path/"maps").read_text())
                    pids.append(int(path.name))
            except (OSError, ProcessLookupError): pass
    content = "\n".join(maps).lower()
    hardware = any(token in content for token in ("d3d12_dri.so", "libnvidia-glcore", "libnvidia-eglcore"))
    software = any(token in content for token in ("swrast_dri.so", "kms_swrast_dri.so"))
    return dict(gazebo_pids=pids, hardware_driver_mapped=hardware, software_driver_mapped=software,
                note="Driver-mapping audit, alongside glxinfo; GPU utilization is retained separately when available")


def main():
    run, profile = Path(sys.argv[1]), sys.argv[2]
    smoke = "--smoke" in sys.argv
    status_graph = run/"extnav-status-graph.txt"
    output_graph = run/"extnav-output-graph.txt"
    identity_graph = run/"mavros-state-graph.txt"
    nodes = (run/"ros-nodes-prearm.txt").read_text().splitlines()
    services = (run/"ros-services-prearm.txt").read_text().splitlines()
    extnav = check_extnav(
        status_graph.read_text() if status_graph.exists() else "",
        output_graph.read_text() if output_graph.exists() else "",
        identity_graph.read_text() if identity_graph.exists() else "",
        "/uav1/mavros/odometry" in nodes,
        any(line.startswith("/uav1/mavros/odometry/") for line in services),
    )
    recorder_participants = {x["participant_gid"] for x in extnav["output_subscribers"]
                             if x["node"].startswith("rosbag2_recorder")}
    data = check_graph((run/"truth-graph.txt").read_text(), (run/"setpoint-graph.txt").read_text(),
                       require_evaluator=not smoke, recorder_participants=recorder_participants,
                       mavros_participants=extnav["mavros_identity_participant_gids"])
    data["extnav"] = extnav
    data.update(renderer_maps(int((run/"gazebo.pid").read_text())))
    data["passed"] = (data["truth_isolation"]
                      and (smoke or (data["sole_setpoint_publisher"]
                                     and data["mavros_setpoint_subscription"]))
                      and data["extnav"]["status_single_publisher"]
                      and data["extnav"]["output_single_adapter_publisher"]
                      and data["extnav"]["mavros_output_subscription"])
    if profile == "server_gpu":
        data["passed"] &= data["hardware_driver_mapped"]
    (run/"runtime-audit.json").write_text(json.dumps(data,indent=2)+"\n")
    if not data["passed"]:
        raise SystemExit("Live isolation/render audit failed; see runtime-audit.json")


if __name__ == "__main__": main()
