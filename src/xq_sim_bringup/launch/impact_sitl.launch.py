"""P16 static point-goal task with EGO, final certification and ArduPilot actuation."""
import json
import runpy
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    root = Path(get_package_share_directory("xq_sim_bringup"))
    bridge = Path(get_package_share_directory("xq_gz_bridge"))
    lio = Path(get_package_share_directory("xq_fast_lio"))
    run = Path(LaunchConfiguration("run_dir").perform(context))
    config = json.loads((run / "run.json").read_text())
    session, strategy = config["session_id"], config["strategy"]
    parameters = runpy.run_path(str(root / "launch/xq_p5_baseline.launch.py"))["baseline_ego_parameters"]()
    speed = 0.3 if strategy == "conservative" else 0.65
    parameters.update({"fsm/impact_mode": True, "fsm/impact_session": session,
        "manager/max_vel": speed, "optimization/max_vel": speed, "bspline/limit_vel": speed,
        "grid_map/map_size_x": 70., "grid_map/map_size_y": 16., "fsm/planning_horizon": 3.,
        "manager/planning_horizon": 3.})
    def autonomy(executable, extra=None, sim=True, remappings=None):
        return Node(package="xq_autonomy", executable=executable, name=executable,
            parameters=[dict(use_sim_time=sim, **(extra or {}))],
            remappings=remappings or [], output="screen")
    nodes = [
        Node(package="tf2_ros", executable="static_transform_publisher", name="impact_map_convention",
             arguments=["--frame-id", "map", "--child-frame-id", "xq_lio_map"],
             parameters=[{"use_sim_time": True}]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="impact_lidar_extrinsic",
             arguments=["--x", "0.04", "--z", "0.12", "--frame-id", "xq_base_link", "--child-frame-id", "livox_frame"],
             parameters=[{"use_sim_time": True}]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="impact_imu_extrinsic",
             arguments=["--frame-id", "xq_base_link", "--child-frame-id", "livox_imu"],
             parameters=[{"use_sim_time": True}]),
        Node(package="xq_gz_bridge", executable="xq_gz_bridge_node", name="xq_gz_bridge_node",
              parameters=[str(bridge / "config/p5_structured_room.yaml"),
                          {"random_seed": config["seed"], "gz_ground_truth_topic": "/world/impact_static/pose/info"}], output="screen"),
        Node(package="xq_fast_lio", executable="fastlio_mapping", name="xq_fast_lio",
              parameters=[str(lio / "config/xq_p4.yaml"), {"use_sim_time": True,
                  "integrity_geometry.enable": True, "publish.scan_publish_en": True,
                  "publish.dense_publish_en": True}], output="screen"),
        autonomy("xq_p4_external_nav", sim=False),
        # Frontier retains its legacy exploration view, but P16 collision checks
        # consume FAST-LIO's time-aligned registered cloud directly.
        autonomy("xq_p5_frontier", {"map_half_extent_m": 35.},
                 remappings=[("/xq/p5/frontier_goal", "/impact/unused_frontier_goal"),
                             ("/xq/p5/cloud_map", "/impact/legacy_frontier_cloud")]),
        autonomy("xq_p6_directional_integrity"),
        autonomy("xq_p10_information_map", {"publish_period_s": 0.2},
                 remappings=[("/xq/p5/cloud_map", "/impact/information_cloud")]),
        Node(package="ego_planner", executable="ego_planner_node", name="impact_ego",
              parameters=[parameters], remappings=[("odom_world", "/xq/p5/ego_odom"),
                 ("grid_map/odom", "/xq/p5/ego_odom"), ("grid_map/cloud", "/cloud_registered")], output="screen"),
        Node(package="ego_planner", executable="traj_server", name="impact_traj_server",
             parameters=[{"use_sim_time": True, "traj_server/time_forward": 1., "traj_server/command_frame": "xq_lio_map"}],
             remappings=[("planning/bspline", "/impact/certified_bspline"), ("/position_cmd", "/impact/position_cmd")], output="screen"),
        autonomy("impact_supervisor", dict(session_id=session, strategy=strategy,
            calibration_file=str(run / "calibration.json"), goal=config["configuration"]["goal_lio_m"], speed_limit=speed,
            event_file=str(run / "events.jsonl"))),
        autonomy("impact_arbiter", dict(session_id=session)),
        autonomy("impact_evaluator", dict(result_dir=str(run), scenario_file=str(run / "scenario.json"))),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("run_dir"), OpaqueFunction(function=setup)])
