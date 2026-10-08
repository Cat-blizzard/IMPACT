"""The gate must reject missing axes even when point count/information are large."""

import numpy as np
import pytest

from xq_autonomy.localization_health import geometry_health, geometry_observation_current


def test_repeated_parallel_constraints_do_not_make_translation_observable():
    for repeats in (1, 20, 1000):
        result = geometry_health(np.diag([0.8, 60., 39.2]) * repeats, 5000)
        assert result["reasons"] == ["geometry_translation_unobservable"]
        assert result["weak_fraction"] == pytest.approx(0.008)


def test_observability_is_rotation_invariant():
    direction = np.array([1., 2., 3.])
    direction /= np.linalg.norm(direction)
    matrix = 40 * np.eye(3) - 39.9 * np.outer(direction, direction)
    result = geometry_health(matrix, 300)
    assert not result["healthy"]
    assert abs(np.dot(direction, result["weak_direction"])) == pytest.approx(1)


@pytest.mark.parametrize("information, reason", [
    ([float("nan")] * 9, "geometry_invalid_information"),
    ([1, 2], "geometry_invalid_information"),
    (np.zeros((3, 3)), "geometry_nonpositive_information"),
    (np.diag([-1., 2., 3.]), "geometry_nonpositive_information"),
    ([[1, 1, 0], [0, 1, 0], [0, 0, 1]], "geometry_asymmetric_information"),
])
def test_invalid_information_fails_closed(information, reason):
    result = geometry_health(information, 300)
    assert result["reasons"] == [reason]


def test_full_rank_requires_sufficient_support():
    assert geometry_health(np.eye(3), 30)["healthy"]
    assert geometry_health(np.eye(3), 2)["reasons"] == ["geometry_insufficient_points"]


def test_map_insertion_requires_matching_healthy_geometry():
    matrix = np.diag([2., 60., 38.])
    assert geometry_observation_current(matrix, 1000, 10.0, 10.1)
    assert not geometry_observation_current(matrix, 1000, 10.0, 10.3)
    assert not geometry_observation_current(np.diag([1., 60., 39.]), 1000, 10.0, 10.0)
    assert not geometry_observation_current(np.eye(3), 2, 10.0, 10.0)
