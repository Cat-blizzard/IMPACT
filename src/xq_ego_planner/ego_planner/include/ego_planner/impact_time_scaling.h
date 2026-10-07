#pragma once

#include <Eigen/Core>

namespace ego_planner
{
inline Eigen::Vector3d impactPlannerVelocity(const Eigen::Vector3d &velocity, double scale)
{
  return velocity / scale;
}

inline double impactPlannerElapsed(double elapsed, double scale)
{
  return elapsed * scale;
}

inline double impactExecutionDuration(double duration, double scale)
{
  return duration / scale;
}
}  // namespace ego_planner
