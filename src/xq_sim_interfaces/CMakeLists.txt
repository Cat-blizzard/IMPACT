cmake_minimum_required(VERSION 3.8)
project(xq_sim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(traj_utils REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlannerGoal.msg"
  "msg/PlannerCandidate.msg"
  "msg/TrajectoryAuthorization.msg"
  "msg/AgentState.msg"
  "msg/FaultEvent.msg"
  "msg/FrontierBid.msg"
  "msg/HealthStatus.msg"
  "msg/LocalizationQuality.msg"
  "msg/LocalizationGeometry.msg"
  "msg/DirectionalIntegrity.msg"
  "msg/AlertLimit.msg"
  "msg/AlertLimitProfile.msg"
  "msg/IntegrityMargin.msg"
  "msg/InformationMap.msg"
  "msg/ActivePerceptionDecision.msg"
  "msg/ExplorationCandidateSet.msg"
  "msg/IntegrityExplorationDecision.msg"
  "msg/ReplanEvent.msg"
  "msg/SubmapMeta.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs traj_utils
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
