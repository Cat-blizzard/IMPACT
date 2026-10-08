"""Measured translational observability for the SITL takeoff health gate."""

import math

import numpy as np


def geometry_health(information, effective_points, *, minimum_weak_fraction=0.02,
                    minimum_points=30):
    """Require support in every axis, independently of temporal accumulation."""
    if not math.isfinite(minimum_weak_fraction) or not 0 < minimum_weak_fraction <= 1 / 3:
        raise ValueError("minimum weak fraction must be in (0, 1/3]")
    if minimum_points < 3:
        raise ValueError("at least three effective points are required")
    matrix = np.asarray(information, dtype=float)
    if matrix.size != 9 or not np.all(np.isfinite(matrix)):
        return dict(healthy=False, reasons=["geometry_invalid_information"])
    matrix = matrix.reshape(3, 3)
    if not np.allclose(matrix, matrix.T, rtol=1e-6, atol=1e-9):
        return dict(healthy=False, reasons=["geometry_asymmetric_information"])
    eigenvalues, eigenvectors = np.linalg.eigh(0.5 * (matrix + matrix.T))
    total = float(eigenvalues.sum())
    if eigenvalues[0] < -1e-9 or total <= 0:
        return dict(healthy=False, reasons=["geometry_nonpositive_information"])
    fraction = max(0., float(eigenvalues[0])) / total
    reasons = []
    if effective_points < minimum_points:
        reasons.append("geometry_insufficient_points")
    if fraction < minimum_weak_fraction:
        reasons.append("geometry_translation_unobservable")
    return dict(healthy=not reasons, reasons=reasons, eigenvalues=eigenvalues.tolist(),
                weak_direction=eigenvectors[:, 0].tolist(), weak_fraction=fraction,
                minimum_weak_fraction=minimum_weak_fraction,
                effective_points=int(effective_points), minimum_points=minimum_points)


def geometry_observation_current(information, effective_points, geometry_stamp,
                                 cloud_stamp, *, minimum_weak_fraction=0.02,
                                 maximum_stamp_gap_s=0.25, minimum_points=30):
    """Trust registered geometry for map insertion only with a matching healthy scan."""
    values = (geometry_stamp, cloud_stamp, maximum_stamp_gap_s)
    if not all(math.isfinite(value) for value in values) or maximum_stamp_gap_s < 0:
        return False
    if abs(geometry_stamp - cloud_stamp) > maximum_stamp_gap_s:
        return False
    return geometry_health(information, effective_points,
                           minimum_weak_fraction=minimum_weak_fraction,
                           minimum_points=minimum_points)["healthy"]
