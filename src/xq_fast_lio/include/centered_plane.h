#pragma once

#include <Eigen/Eigen>
#include <cmath>

template <typename T, int N>
bool fit_centered_plane(const Eigen::Matrix<T, N, 3> &points,
                        Eigen::Matrix<T, 4, 1> &plane, T tolerance)
{
    if (!points.allFinite() || !std::isfinite(tolerance) || tolerance <= T(0))
        return false;
    const Eigen::Matrix<T, 1, 3> centroid = points.colwise().mean();
    const Eigen::Matrix<T, N, 3> centered = points.rowwise() - centroid;
    const Eigen::Matrix<T, 3, 3> covariance = centered.transpose() * centered / T(N);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 3, 3>> solver(covariance);
    if (solver.info() != Eigen::Success)
        return false;
    const auto eigenvalues = solver.eigenvalues();
    // A line or noisy isotropic patch cannot define a reliable plane normal.
    if (eigenvalues(1) <= T(1e-8) || eigenvalues(1) <= T(4) * eigenvalues(0))
        return false;
    const Eigen::Matrix<T, 3, 1> normal = solver.eigenvectors().col(0);
    const T offset = -centroid.dot(normal);
    if ((points * normal).array().isFinite().all()
        && ((points * normal).array() + offset).abs().maxCoeff() <= tolerance)
    {
        plane.template head<3>() = normal;
        plane(3) = offset;
        return true;
    }
    return false;
}
