"""Generate isolated, seed-paired static geometry and its independent truth description."""
from pathlib import Path
import json
import random
import xml.etree.ElementTree as ET


def scenario_geometry(name, seed):
    if name not in ("normal", "recoverable", "unrecoverable"):
        raise ValueError("unknown scenario")
    rng = random.Random(seed)
    boxes = [
        dict(name="floor", center=[20, 0, -0.1], size=[80, 16, 0.2]),
        dict(name="left_wall", center=[20, 2.1, 2], size=[80, 0.2, 4]),
        dict(name="right_wall", center=[20, -2.1, 2], size=[80, 0.2, 4]),
        dict(name="end_wall", center=[60, 0, 2], size=[0.2, 4.4, 4]),
        dict(name="start_wall", center=[-20, 0, 2], size=[0.2, 4.4, 4]),
    ]
    xs = (2., 5., 8., 11., 14.) if name == "normal" else (5., 6.) if name == "recoverable" else ()
    for i, x in enumerate(xs):
        # Real box geometry, identical for all policy arms of a paired seed.
        boxes.append(dict(name=f"feature_{i}", center=[x+rng.uniform(-.12,.12), 1.88, 2.],
                          size=[.3, .3, 2.7], yaw=rng.uniform(.35,.8)))
    return dict(schema_version=1, scenario=name, seed=seed, boxes=boxes,
                goal_lio_m=[12.,0.,2.], start_world_m=[0.,0.,.195], start_yaw=0.,
                seed_effect="bridge sensor-noise RNG plus declared feature geometry jitter",
                outcome="UNVERIFIED", ground_truth_policy="evaluator_only")


def generate(root: Path, output: Path, name: str, seed: int):
    data = scenario_geometry(name, seed)
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
