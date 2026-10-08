"""Compile the production plane fit and exercise numerical degeneracies."""

from pathlib import Path
import subprocess


def test_production_plane_fit(tmp_path):
    root = Path(__file__).resolve().parents[1]
    source = tmp_path / "plane.cpp"
    source.write_text(r'''
#include "centered_plane.h"
#include "observable_translation.h"
#include <limits>
int main() {
    Eigen::Matrix<double, 5, 3> points;
    points << -1, -1, 0, 1, -1, 0, 1, 1, 0, -1, 1, 0, 0, 0, 0;
    Eigen::Vector4d plane;
    if (!fit_centered_plane(points, plane, 0.1)) return 1;
    if (std::abs(plane(2)) < 0.999 || std::abs(plane(3)) > 1e-8) return 2;
    points.rowwise() += Eigen::RowVector3d(100, -40, 15);
    if (!fit_centered_plane(points, plane, 0.1)) return 3;
    if (std::abs(plane(2)) < 0.999 || std::abs(std::abs(plane(3)) - 15) > 1e-8) return 4;
    points << -2, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 2, 0, 0;
    if (fit_centered_plane(points, plane, 0.1)) return 5;
    points << -2, .001, -.002, -1, -.001, .001, 0, .002, .002,
              1, -.002, -.001, 2, 0, 0;
    if (fit_centered_plane(points, plane, 0.1)) return 6;
    points(0, 0) = std::numeric_limits<double>::quiet_NaN();
    if (fit_centered_plane(points, plane, 0.1)) return 7;
    const Eigen::Matrix3d information = Eigen::Vector3d(1, 60, 39).asDiagonal();
    const auto projector = observable_translation_projector(information, .02);
    if ((projector * Eigen::Vector3d::UnitX()).norm() > 1e-8) return 8;
    if ((projector * Eigen::Vector3d::UnitY() - Eigen::Vector3d::UnitY()).norm() > 1e-8) return 9;
    Eigen::Matrix3d rotation = Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto rotated = observable_translation_projector(rotation * information * rotation.transpose(), .02);
    if ((rotated - rotation * projector * rotation.transpose()).norm() > 1e-8) return 10;
    if ((observable_translation_projector(Eigen::Matrix3d::Identity(), .02)
         - Eigen::Matrix3d::Identity()).norm() > 1e-8) return 11;
    if (!localization_map_update_allowed(Eigen::Vector3d(2, 60, 38), .02)) return 12;
    if (localization_map_update_allowed(Eigen::Vector3d(1, 60, 39), .02)) return 13;
    if (localization_map_update_allowed(Eigen::Vector3d(1, 2, 3), .34)) return 14;
    return 0;
}
''')
    executable = tmp_path / "plane"
    subprocess.run(["g++", "-std=c++17", "-O1", "-I/usr/include/eigen3",
                    "-I" + str(root / "src/xq_fast_lio/include"), str(source),
                    "-o", str(executable)], check=True, capture_output=True, text=True)
    subprocess.run([str(executable)], check=True)
