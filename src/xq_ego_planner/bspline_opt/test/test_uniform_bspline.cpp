#include <gtest/gtest.h>

#include "bspline_opt/uniform_bspline.h"

namespace
{
Eigen::MatrixXd linearControlPoints(double x_step, double y_step)
{
  Eigen::MatrixXd points(3, 10);
  for (int index = 0; index < points.cols(); ++index) {
    points.col(index) = Eigen::Vector3d(index * x_step, index * y_step, 2.0);
  }
  return points;
}
}  // namespace

TEST(UniformBsplineFeasibility, RejectsCombinedVelocityAboveVectorLimit)
{
  ego_planner::UniformBspline spline(linearControlPoints(0.5, 0.5), 3, 1.0);
  spline.setPhysicalLimits(0.65, 1.0, 0.05);

  double ratio = 0.0;
  EXPECT_FALSE(spline.checkFeasibility(ratio));
  EXPECT_NEAR(ratio, std::sqrt(0.5) / 0.65, 1e-9);
}

TEST(UniformBsplineFeasibility, AllowsVectorVelocityInsideTolerance)
{
  ego_planner::UniformBspline spline(linearControlPoints(0.67, 0.0), 3, 1.0);
  spline.setPhysicalLimits(0.65, 1.0, 0.05);

  double ratio = 0.0;
  EXPECT_TRUE(spline.checkFeasibility(ratio));
}

TEST(UniformBsplineFeasibility, TimeScalingEnforcesVectorLimit)
{
  ego_planner::UniformBspline spline(linearControlPoints(0.5, 0.5), 3, 1.0);
  spline.setPhysicalLimits(0.65, 1.0, 0.05);

  double ratio = 0.0;
  ASSERT_FALSE(spline.checkFeasibility(ratio));
  spline.scaleTime(ratio);

  double residual_ratio = 0.0;
  EXPECT_TRUE(spline.checkFeasibility(residual_ratio));
}
