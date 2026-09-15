"""Validate live node graph before arming, plus actual Gazebo renderer libraries."""
import json
from pathlib import Path
import re
import sys


def check_graph(truth, setpoint):
    # ros2 topic info -v sections name endpoint type and node name.
    subscribers = []
    for section in re.split(r"\n(?=Node name:)", truth):
        if "Endpoint type: SUBSCRIPTION" in section:
            match = re.search(r"Node name: (\S+)", section)
            if match: subscribers.append(match[1])
    allowed = all(name == "impact_evaluator" or name.startswith("rosbag2_recorder") for name in subscribers)
    count = re.search(r"Publisher count: (\d+)", setpoint)
    owner = "Node name: impact_arbiter" in setpoint
    return dict(truth_subscribers=subscribers,
        truth_isolation=allowed and "impact_evaluator" in subscribers,
        sole_setpoint_publisher=bool(count and count[1] == "1" and owner))


def check_extnav(status_text, output_text):
    """Reject duplicate/stale ExternalNav adapters before arming."""
    def endpoints(text, kind):
        result = []
        for section in re.split(r"\n(?=Node name:)", text):
            if f"Endpoint type: {kind}" not in section:
                continue
            node = re.search(r"Node name: (\S+)", section)
            gid = re.search(r"GID: ([0-9a-fA-F]+)", section)
            ns = re.search(r"Node namespace: (\S+)", section)
            result.append({"node": node[1] if node else None, "namespace": ns[1] if ns else None,
                           "gid": gid[1] if gid else None})
        return result
    status_publishers = endpoints(status_text, "PUBLISHER")
    output_publishers = endpoints(output_text, "PUBLISHER")
    output_subscribers = endpoints(output_text, "SUBSCRIPTION")
    return dict(status_publishers=status_publishers,
                status_single_publisher=(len(status_publishers) == 1 and
                                         status_publishers[0]["node"] == "xq_p4_external_nav"),
                output_publishers=output_publishers,
                output_single_adapter_publisher=(len(output_publishers) == 1 and
                                                 output_publishers[0]["node"] == "xq_p4_external_nav"),
                output_subscribers=output_subscribers,
                mavros_output_subscription=any(
                    x.get("namespace") == "/uav1/mavros" and x.get("node") == "odometry"
                    for x in output_subscribers))


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
    data = check_graph((run/"truth-graph.txt").read_text(), (run/"setpoint-graph.txt").read_text())
    status_graph = run/"extnav-status-graph.txt"
    output_graph = run/"extnav-output-graph.txt"
    data["extnav"] = check_extnav(
        status_graph.read_text() if status_graph.exists() else "",
        output_graph.read_text() if output_graph.exists() else "",
    )
    data.update(renderer_maps(int((run/"gazebo.pid").read_text())))
    smoke = "--smoke" in sys.argv
    data["passed"] = (data["truth_isolation"] and (smoke or data["sole_setpoint_publisher"])
                      and data["extnav"]["status_single_publisher"]
                      and data["extnav"]["output_single_adapter_publisher"]
                      and data["extnav"]["mavros_output_subscription"])
    if profile == "server_gpu":
        data["passed"] &= data["hardware_driver_mapped"]
    (run/"runtime-audit.json").write_text(json.dumps(data,indent=2)+"\n")
    if not data["passed"]:
        raise SystemExit("Live isolation/render audit failed; see runtime-audit.json")


if __name__ == "__main__": main()
