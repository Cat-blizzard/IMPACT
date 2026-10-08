#pragma once

#include <Eigen/Eigen>
#include <cmath>

inline bool localization_map_update_allowed(const Eigen::Vector3d &information_eigenvalues,
                                            double minimum_fraction)
{
    if (!information_eigenvalues.allFinite() || !std::isfinite(minimum_fraction)
        || minimum_fraction < 0.0 || minimum_fraction > 1.0 / 3.0)
        return false;
    const double total = information_eigenvalues.sum();
    return total > 0.0 && information_eigenvalues(0) >= minimum_fraction * total;
}

inline Eigen::Matrix3d observable_translation_projector(const Eigen::Matrix3d &information,
                                                        double minimum_fraction)
{
    if (minimum_fraction <= 0.0) return Eigen::Matrix3d::Identity();
    if (!information.allFinite()) return Eigen::Matrix3d::Zero();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
        0.5 * (information + information.transpose()));
    if (solver.info() != Eigen::Success || solver.eigenvalues().sum() <= 0.0)
        return Eigen::Matrix3d::Zero();
    Eigen::Vector3d observed;
    for (int axis = 0; axis < 3; ++axis)
        observed(axis) = solver.eigenvalues()(axis) >=
            minimum_fraction * solver.eigenvalues().sum() ? 1.0 : 0.0;
    return solver.eigenvectors() * observed.asDiagonal() * solver.eigenvectors().transpose();
}
