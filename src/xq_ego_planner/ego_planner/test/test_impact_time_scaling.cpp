#include <gtest/gtest.h>

#include "bspline_opt/uniform_bspline.h"
#include "ego_planner/impact_time_scaling.h"

TEST(ImpactTimeScaling, RepeatedReplansPreserveMeasuredBoundaryVelocity)
{
  const double scale = 0.30 / 0.65;
  const Eigen::Vector3d measured(0.21, 0.03, 0.0);
  Eigen::Vector3d executed = measured;
  for (int replan = 0; replan < 20; ++replan) {
    const Eigen::Vector3d nominal = ego_planner::impactPlannerVelocity(executed, scale);
    Eigen::MatrixXd points(3, 10);
    for (int i = 0; i < points.cols(); ++i)
      points.col(i) = Eigen::Vector3d(0., 0., 2.) + i * nominal;
    ego_planner::UniformBspline spline(points, 3, 1.0);
    spline.setKnot(spline.getKnot() / scale);
    executed = spline.getDerivative().evaluateDeBoorT(0.0);
    EXPECT_LT((executed - measured).norm(), 1e-9);
  }
}

TEST(ImpactTimeScaling, InternalAndExecutedSplineAgreeAtPhysicalTime)
{
  const double scale = 0.30 / 0.65;
  Eigen::MatrixXd points(3, 10);
  for (int i = 0; i < points.cols(); ++i)
    points.col(i) = Eigen::Vector3d(i * 0.4, 0., 2.);
  ego_planner::UniformBspline nominal(points, 3, 1.0), executed = nominal;
  executed.setKnot(executed.getKnot() / scale);
  const double wall_time = 2.5;
  EXPECT_LT((nominal.evaluateDeBoorT(ego_planner::impactPlannerElapsed(wall_time, scale))
             - executed.evaluateDeBoorT(wall_time)).norm(), 1e-9);
  EXPECT_NEAR(ego_planner::impactExecutionDuration(nominal.getTimeSum(), scale),
              executed.getTimeSum(), 1e-9);
  EXPECT_DOUBLE_EQ(ego_planner::impactPlannerElapsed(wall_time, 1.0), wall_time);
  EXPECT_DOUBLE_EQ(ego_planner::impactExecutionDuration(wall_time, 1.0), wall_time);
}
