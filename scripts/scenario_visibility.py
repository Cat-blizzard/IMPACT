#!/usr/bin/env python3
"""Ideal first-return ray audit of generated static boxes, including occlusion."""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def first_returns(origin, directions, boxes, maximum_range):
    directions = np.asarray(directions, float).reshape(-1, 3)
    origin = np.asarray(origin, float).reshape(3)
    if (not np.isfinite(directions).all() or not np.isfinite(origin).all()
            or not np.allclose(np.linalg.norm(directions, axis=1), 1.)
            or not np.isfinite(maximum_range) or maximum_range <= 0):
        raise ValueError("finite origin, unit rays and positive range are required")
    distances = np.full(len(directions), maximum_range)
    labels = np.full(len(directions), -1, dtype=int)
    normals = np.zeros_like(directions)
    for index, box in enumerate(boxes):
        yaw = box.get("yaw", 0.)
        cosine, sine = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[cosine, -sine, 0.], [sine, cosine, 0.], [0., 0., 1.]])
        local_origin = (origin - np.asarray(box["center"])) @ rotation
        local_rays = directions @ rotation
        half_size = np.asarray(box["size"]) / 2
        parallel = np.abs(local_rays) < 1e-12
        safe_rays = np.where(parallel, 1., local_rays)
        near = (-half_size - local_origin) / safe_rays
        far = (half_size - local_origin) / safe_rays
        entries = np.where(parallel, -np.inf, np.minimum(near, far))
        exits = np.where(parallel, np.inf, np.maximum(near, far))
        entry = entries.max(axis=1)
        leave = exits.min(axis=1)
        inside_parallel = np.all(~parallel | (np.abs(local_origin) <= half_size), axis=1)
        valid = inside_parallel & (leave >= entry) & (entry > 0.) & (entry < distances)
        distances[valid] = entry[valid]
        labels[valid] = index
        normals[valid] = np.eye(3)[np.argmax(entries[valid], axis=1)] @ rotation.T
    return labels, normals, distances


def audit(scenario, model):
    lidar = ET.parse(model).getroot().find(".//sensor[@name='xq_mid360_lidar']/lidar")
    if lidar is None:
        raise ValueError("Mid-360 LiDAR is missing")
    angles = []
    for axis in ("horizontal", "vertical"):
        scan = lidar.find(f"scan/{axis}")
        angles.append(np.linspace(float(scan.findtext("min_angle")),
                                  float(scan.findtext("max_angle")), int(scan.findtext("samples"))))
    horizontal, vertical = np.meshgrid(*angles)
    directions = np.column_stack((np.cos(vertical).ravel() * np.cos(horizontal).ravel(),
                                  np.cos(vertical).ravel() * np.sin(horizontal).ravel(),
                                  np.sin(vertical).ravel()))
    maximum_range = float(lidar.findtext("range/max"))
    results = []
    for x in (0., 2., 3., 4., 5., 6., 8., 12.):
        for y, z in ((0., 2.32), (-0.4, 2.32), (0.4, 2.32), (0., 2.57)):
            labels, normals, distances = first_returns([x, y, z], directions,
                                                        scenario["boxes"], maximum_range)
            information = normals.T @ normals
            eigenvalues = np.linalg.eigvalsh(information)
            results.append(dict(sensor_position_world_m=[x, y, z],
                hit_counts={box["name"]: int(np.count_nonzero(labels == i))
                            for i, box in enumerate(scenario["boxes"])},
                missed_rays=int(np.count_nonzero(labels < 0)),
                minimum_range_m=float(distances.min()),
                ideal_information_eigenvalues=eigenvalues.tolist(),
                ideal_weak_fraction=float(eigenvalues[0] / max(eigenvalues.sum(), 1e-9))))
    return dict(analysis="IDEAL_STATIC_FIRST_RETURN_VISIBILITY", acceptance_evidence=False,
                lidar_range_m=maximum_range, rays=len(directions), poses=results,
                limitations=["Ideal rays include static occlusion but omit noise, motion and scan matching.",
                             "Sensor poses are hypothetical probes, not controller targets.",
                             "Raw first-return information is not FAST-LIO accepted constraint information."])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    report = audit(json.loads((args.run / "scenario.json").read_text()),
                   args.run / "models/xq_iris_mid360_ardupilot/model.sdf")
    (args.run / "ideal-visibility-audit.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"analysis": report["analysis"], "poses": len(report["poses"])}))


if __name__ == "__main__":
    main()
