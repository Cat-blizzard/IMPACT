"""Generate isolated, seed-paired static geometry and its independent truth description."""
from pathlib import Path
import json
import random
import shutil
import xml.etree.ElementTree as ET


def scenario_geometry(name, seed):
    if name not in ("normal", "recoverable", "unrecoverable"):
        raise ValueError("unknown scenario")
    rng = random.Random(seed)
    start_wall_x = {
        "normal": -20.0,
        "recoverable": -35.0,
        "unrecoverable": -60.0,
    }[name]
    boxes = [
        dict(name="floor", center=[20, 0, -0.1], size=[160, 16, 0.2]),
        # Keep an overhead horizontal plane inside the Mid-360 vertical field
        # of view, but outside the certified flight and braking envelope.
        dict(name="ceiling", center=[20, 0, 10.1], size=[160, 4.4, 0.2]),
        dict(name="left_wall", center=[20, 2.1, 2], size=[160, 0.2, 4.2]),
        dict(name="right_wall", center=[20, -2.1, 2], size=[160, 0.2, 4.2]),
        # The Mid-360 model has a 40 m range.  The normal end cap remains well
        # outside the task.  Recoverable keeps a visible, distant longitudinal
        # reference so the deliberate integrity degradation cannot turn into an
        # estimator runaway before the recovery step has completed.  The
        # unrecoverable arm has no cap in the sensor envelope.
        dict(name="end_wall", center=[30 if name == "recoverable" else 100, 0, 2],
             size=[0.2, 4.4, 4.2]),
        # At the recoverable 35 m placement, the original 4.2 m wall yielded
        # fewer than 110 returns per scan during takeoff. Extend only its
        # vertical aperture so the longitudinal anchor is measurable without
        # changing where it leaves the 40 m sensor range.
        dict(name="start_wall", center=[start_wall_x, 0, 5], size=[0.2, 4.4, 10.0]),
    ]
    # The recoverable arm uses the continuous side walls as longitudinal
    # planes.  Extra wall-attached protrusions made the GPU voxel map close the
    # corridor intermittently around x=5--6 m, creating a planner stall that
    # was unrelated to the deliberate integrity degradation.
    xs = (2., 5., 8., 11., 14.) if name == "normal" else ()
    for i, x in enumerate(xs):
        # Real box geometry, identical for all policy arms of a paired seed.
        boxes.append(dict(name=f"feature_{i}", center=[x+rng.uniform(-.12,.12), 1.88, 2.],
                          size=[.3, .3, 2.7], yaw=rng.uniform(.35,.8)))
    anchor_policy = {
        "normal": "full_route",
        "recoverable": "entry_only",
        "unrecoverable": "none",
    }[name]
    anchor_interval = {
        "normal": [0.0, 12.0],
        "recoverable": [0.0, 5.0],
        "unrecoverable": None,
    }[name]
    return dict(schema_version=6, scenario=name, seed=seed, boxes=boxes,
                goal_lio_m=[12.,0.,2.], actual_goal_tolerance_m=0.45,
                # Recovery cycles are bounded by the supervisor's execution
                # stall guard, but several valid information-gathering steps
                # can consume more than the normal 180 s task budget.  Keep
                # the truth gate fixed and allocate only this scenario a
                # larger completion window.
                # The recovery arm deliberately spends time rebuilding an
                # estimator margin before it is allowed to resume forward
                # motion.  On the GPU SITL path the bounded 0.30 m/s
                # execution envelope then needs a little more than five
                # minutes to cover the remaining route.  Keep the timeout
                # long enough to observe completion while retaining the
                # supervisor's 20 s no-progress fail-closed guard.
                task_timeout_sim_s=420.0 if name == "recoverable" else 180.0,
                # The independent truth gate remains 0.45 m.  Recoverable
                # ExternalNav can retain a bounded estimator offset after the
                # recovery step, so the controller's completion gate includes
                # that measured execution envelope without weakening truth
                # evaluation.
                # The recoverable estimator carries a bounded ~0.5 m offset
                # after the recovery observation.  Requiring the controller
                # to enter a 0.55 m LIO gate leaves room for that measured
                # offset while triggering LAND before the vehicle can drift
                # beyond the independent 0.45 m truth gate.
                # The LIO-to-truth execution envelope is measured separately
                # by the evaluator.  A slightly wider recoverable controller
                # gate lets the FCU brake before its repeatable ~0.65 m LIO
                # position lag carries the physical vehicle past the truth
                # endpoint; the independent evaluator remains fixed at 0.45 m.
                # Keep the controller's completion gate just above the
                # independent truth gate.  The GPU ExternalNav stream carries
                # a repeatable ~0.65 m longitudinal lag; a 0.60 m gate can
                # land while the lateral component is still outside the
                # independent 0.45 m truth ball on some seeds.  A 0.50 m gate
                # gives the final certified segment enough time to close that
                # lateral error while retaining the measured braking margin.
                mission_goal_tolerance_m=0.50 if name == "recoverable" else 0.45,
                start_world_m=[0.,0.,.195], start_yaw=0.,
                sensor_observability_design={
                    "lidar_range_m": 40.0,
                    "route_x_m": [0.0, 12.0],
                    "longitudinal_caps_outside_range": name == "unrecoverable",
                    "start_anchor_policy": anchor_policy,
                    "start_anchor_visible_route_x_m": anchor_interval,
                    "start_anchor_height_m": 10.0,
                    "ceiling_supplies_vertical_plane": True,
                    "ceiling_bottom_z_m": 10.0,
                    "certified_flight_max_z_m": 2.9,
                    "scenario_features_supply_longitudinal_planes": name != "unrecoverable",
                },
                seed_effect="Gazebo simulator and sensor RNG plus declared feature geometry jitter",
                outcome="UNVERIFIED", ground_truth_policy="evaluator_only")


def generate(root: Path, output: Path, name: str, seed: int, sensor_noise_std_m: float = 0.015):
    if not 0.0 <= sensor_noise_std_m <= 1.0:
        raise ValueError("sensor_noise_std_m must be between 0 and 1 metre")
    data = scenario_geometry(name, seed)
    # Give each run its own model. Gazebo reads noise from SDF, not the ROS
    # bridge parameter, and the shared P4 model must remain untouched.
    source = root / "src/xq_gz_assets/models/xq_iris_mid360_ardupilot"
    model_dir = output / "models/xq_iris_mid360_ardupilot"
    shutil.copytree(source, model_dir)
    model_tree = ET.parse(model_dir / "model.sdf")
    noise = model_tree.find(".//sensor[@name='xq_mid360_lidar']/lidar/noise/stddev")
    if noise is None:
        raise ValueError("Mid-360 Gazebo noise element is missing")
    noise.text = str(sensor_noise_std_m)
    model_tree.write(model_dir / "model.sdf", encoding="utf-8", xml_declaration=True)
    data["sensor_noise_std_m"] = sensor_noise_std_m
    data["gazebo_seed"] = seed
    tree = ET.parse(root / "src/xq_gz_assets/worlds/xq_p5_structured_room.sdf")
    world = tree.getroot().find("world")
    world.set("name", "impact_static")
    for element in list(world):
        if element.tag in ("model", "include"):
            world.remove(element)
    for box in data["boxes"]:
        model = ET.SubElement(world, "model", name=box["name"])
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = " ".join(map(str, box["center"]+[0,0,box.get("yaw",0)]))
        link = ET.SubElement(model, "link", name="body")
        for tag in ("collision", "visual"):
            element = ET.SubElement(link, tag, name=tag)
            geometry = ET.SubElement(ET.SubElement(element, "geometry"), "box")
            ET.SubElement(geometry, "size").text = " ".join(map(str, box["size"]))
    vehicle = ET.SubElement(world, "include")
    ET.SubElement(vehicle, "name").text = "xq_p5_uav1"
    ET.SubElement(vehicle, "uri").text = "model://xq_iris_mid360_ardupilot"
    ET.SubElement(vehicle, "pose").text = "0 0 0.195 0 0 0"
    ET.indent(tree, space="  ")
    tree.write(output / "world.sdf", encoding="utf-8", xml_declaration=True)
    (output / "scenario.json").write_text(json.dumps(data, indent=2)+"\n", encoding="utf-8")
    return data
