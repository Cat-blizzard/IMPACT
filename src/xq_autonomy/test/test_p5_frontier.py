import numpy as np
from types import SimpleNamespace
from builtin_interfaces.msg import Time

from xq_autonomy.p5_frontier_node import (
    P5FrontierNode, _cloud_xyz, _known_viewpoint_candidates, _mark_pose_free, _xyz_cloud,
)


def test_pose_marks_unknown_cell_free() -> None:
    free = np.zeros((5, 5), dtype=bool)
    occupied = np.zeros_like(free)
    _mark_pose_free(free, occupied, 0.05, -0.05, -0.25, 0.1)
    assert free[2, 2]


def test_pose_does_not_clear_obstacle_cell() -> None:
    free = np.zeros((5, 5), dtype=bool)
    occupied = np.zeros_like(free)
    occupied[2, 2] = True
    _mark_pose_free(free, occupied, 0.05, -0.05, -0.25, 0.1)
    assert not free[2, 2]


def test_viewpoint_candidates_exclude_frontier_cells() -> None:
    reachable = np.asarray(((2, 2), (2, 3), (4, 4)), dtype=np.int32)
    distance = np.full((6, 6), -1, dtype=np.int32)
    distance[2, 2] = 5
    distance[2, 3] = 6
    distance[4, 4] = 8
    frontier = np.zeros((6, 6), dtype=bool)
    frontier[2, 2] = True
    candidates = _known_viewpoint_candidates(
        reachable,
        distance,
        frontier,
        np.asarray((2.0, 2.5)),
        maximum_distance_cells=30,
        minimum_path_m=0.45,
        resolution=0.1,
    )
    assert (2, 2) not in candidates
    assert candidates[0] == (2, 3)


def test_viewpoint_candidates_require_known_reachable_path() -> None:
    reachable = np.asarray(((1, 1), (4, 4)), dtype=np.int32)
    distance = np.full((6, 6), -1, dtype=np.int32)
    distance[1, 1] = 4
    distance[4, 4] = 9
    frontier = np.zeros((6, 6), dtype=bool)
    candidates = _known_viewpoint_candidates(
        reachable,
        distance,
        frontier,
        np.asarray((1.0, 1.0)),
        maximum_distance_cells=30,
        minimum_path_m=0.45,
        resolution=0.1,
    )
    assert candidates == [(4, 4)]


def test_viewpoint_candidates_require_extra_known_clearance() -> None:
    reachable = np.asarray(((2, 2), (2, 3)), dtype=np.int32)
    distance = np.full((6, 6), -1, dtype=np.int32)
    distance[2, 2] = 5
    distance[2, 3] = 6
    frontier = np.zeros((6, 6), dtype=bool)
    safe_free = np.zeros((6, 6), dtype=bool)
    safe_free[2, 3] = True
    candidates = _known_viewpoint_candidates(
        reachable,
        distance,
        frontier,
        np.asarray((2.0, 2.5)),
        maximum_distance_cells=30,
        minimum_path_m=0.45,
        resolution=0.1,
        safe_free=safe_free,
    )
    assert candidates == [(2, 3)]


def test_registered_cloud_retains_near_obstacle_without_latest_odom_transform() -> None:
    node = object.__new__(P5FrontierNode)
    node.have_odom = True
    node.position = np.asarray((1., 0., 2.))  # later odometry must not move the scan
    node.last_scan_stamp = -float("inf")
    node.resolution, node.origin, node.size = .1, -2., 40
    node.free = np.zeros((40, 40), dtype=bool)
    node.occupied = np.zeros_like(node.free)
    node.scan_count = 0
    published = []
    node.cloud_pub = SimpleNamespace(publish=published.append)
    node.get_parameter = lambda name: SimpleNamespace(value={
        "minimum_mapping_range_m": .8, "flight_altitude_m": 2.,
    }[name])
    stamp = Time(sec=10)
    cloud = _xyz_cloud(np.asarray(((.5, 0., 2.),), dtype=np.float32), stamp, "xq_lio_map")
    node._cloud_cb(cloud)
    assert len(published) == 1
    np.testing.assert_array_equal(_cloud_xyz(published[0])[0],
                                  np.asarray((.5, 0., 2.), dtype=np.float32))
    assert published[0].header.stamp == stamp
