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
    data.update(renderer_maps(int((run/"gazebo.pid").read_text())))
    data["passed"] = data["truth_isolation"] and data["sole_setpoint_publisher"]
    if profile == "server_gpu":
        data["passed"] &= data["hardware_driver_mapped"] and not data["software_driver_mapped"]
    (run/"runtime-audit.json").write_text(json.dumps(data,indent=2)+"\n")
    if not data["passed"]:
        raise SystemExit("Live isolation/render audit failed; see runtime-audit.json")


if __name__ == "__main__": main()
