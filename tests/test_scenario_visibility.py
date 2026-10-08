"""Occlusion must prevent a hidden informative surface from being counted."""

import numpy as np
import pytest

from scenario_visibility import first_returns
from impact_scenarios import scenario_geometry


def box(center, size, yaw=0.):
    return dict(center=center, size=size, yaw=yaw)


def test_near_wall_occludes_far_anchor_independently_of_box_order():
    near = box([2., 0., 0.], [.2, 4., 4.])
    far = box([4., 0., 0.], [.2, 4., 4.])
    for boxes in ([near, far], [far, near]):
        labels, normals, distance = first_returns([0., 0., 0.], [[1., 0., 0.]], boxes, 10.)
        assert boxes[labels[0]] is near
        assert distance[0] == pytest.approx(1.9)
        assert normals[0] == pytest.approx([1., 0., 0.])


def test_parallel_ray_outside_box_and_range_exit_are_misses():
    obstacles = [box([2., 3., 0.], [.2, 1., 1.]), box([6., 0., 0.], [.2, 1., 1.])]
    labels, normals, _ = first_returns([0., 0., 0.], [[1., 0., 0.]], obstacles, 5.)
    assert labels.tolist() == [-1]
    assert np.count_nonzero(normals) == 0


def test_rotated_wall_returns_rotated_normal():
    wall = box([0., 2., 0.], [.2, 4., 4.], np.pi / 2)
    labels, normals, distances = first_returns([0., 0., 0.], [[0., 1., 0.]], [wall], 10.)
    assert labels.tolist() == [0]
    assert distances[0] == pytest.approx(1.9)
    assert abs(normals[0, 1]) == pytest.approx(1.)


def test_elevated_recovery_anchor_is_outside_flight_and_measurable_after_up_step():
    scene = scenario_geometry("recoverable", 1000)
    anchor = next(box for box in scene["boxes"] if box["name"] == "overhead_anchor")
    assert anchor["center"][2] - anchor["size"][2] / 2 > 2.9 + .35
    assert anchor["center"][0] - anchor["size"][0] / 2 == pytest.approx(5.7)
    assert anchor["size"][0] == pytest.approx(0.6)
    assert anchor["size"][1] == pytest.approx(3.6)
    horizontal, vertical = np.meshgrid(np.linspace(-np.pi, np.pi, 720),
                                       np.linspace(-.12217304764, .90757121104, 32))
    directions = np.column_stack((np.cos(vertical).ravel() * np.cos(horizontal).ravel(),
                                  np.cos(vertical).ravel() * np.sin(horizontal).ravel(),
                                  np.sin(vertical).ravel()))
    fractions = []
    for z in (2.32, 2.57):
        _, normals, _ = first_returns([5.7, 0., z], directions, scene["boxes"], 22.)
        values = np.linalg.eigvalsh(normals.T @ normals)
        fractions.append(values[0] / values.sum())
    assert fractions[0] < .02 < fractions[1]


def test_recoverable_forward_anchor_enters_range_after_entry_anchor_exits():
    scene = scenario_geometry("recoverable", 1000)
    design = scene["sensor_observability_design"]
    assert design["forward_anchor_visible_route_x_m"] == pytest.approx([4.5, 12.0])
    assert "recovery_release_waypoint_lio_m" not in design
    assert design["lidar_range_m"] == 22.0
    end_wall = next(box for box in scene["boxes"] if box["name"] == "end_wall")
    assert end_wall["center"][0] == pytest.approx(26.5)

    horizontal, vertical = np.meshgrid(np.linspace(-np.pi, np.pi, 720),
                                       np.linspace(-.12217304764, .90757121104, 32))
    directions = np.column_stack((np.cos(vertical).ravel() * np.cos(horizontal).ravel(),
                                  np.cos(vertical).ravel() * np.sin(horizontal).ravel(),
                                  np.sin(vertical).ravel()))
    fractions = []
    for x in (5.7, 6.5, 8.0):
        labels, normals, _ = first_returns([x, 0., 2.32], directions,
                                           scene["boxes"], 22.)
        eigenvalues = np.linalg.eigvalsh(normals.T @ normals)
        fractions.append(eigenvalues[0] / eigenvalues.sum())
        assert any(box["name"] == "end_wall" and np.any(labels == index)
                   for index, box in enumerate(scene["boxes"]))
    assert fractions[0] < .02
    assert all(value >= .02 for value in fractions[1:])

    _, raised_normals, _ = first_returns([5.7, 0., 2.57], directions,
                                         scene["boxes"], 22.)
    raised = np.linalg.eigvalsh(raised_normals.T @ raised_normals)
    assert raised[0] / raised.sum() >= .02
