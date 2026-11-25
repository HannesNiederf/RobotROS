"""Mapping or navigation stack of robotROS (softwarespezifikation.md sections 3.3, 7.1, 7.2).

Arguments:
  mode          (both: activity_narrator, ADR 0004)
                mapping: scan_masker, slam_toolbox (online_async), twist_mux A/B, collision_monitor
                nav:     scan_masker, map_server, amcl, Nav2 (planner, controller, behaviors, bt_navigator,
                         velocity_smoother), twist_mux A/B, collision_monitor
  map           map YAML for mode nav
  use_sim_time  true in the simulation

Velocity chain: controller -> /cmd_vel_ctrl -> velocity_smoother -> /cmd_vel_nav -> twist_mux_a (with
/cmd_vel_manual) -> /cmd_vel_mux_a -> collision_monitor -> /cmd_vel_cm -> twist_mux_b (with /cmd_vel_docking)
-> /cmd_vel -> nano_bridge (simulation: sim_hw_adapter).
The footprint of the costmaps and the collision monitor is the square footprint_robot from robot.yaml.
"""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def footprint_string(robot_cfg: dict) -> str:
    h = float(robot_cfg["footprint_robot"]) / 2.0
    return f"[[{h}, {h}], [{h}, {-h}], [{-h}, {-h}], [{-h}, {h}]]"


def _setup(context: LaunchContext) -> list:
    share = get_package_share_directory("robotros_bringup")
    cfg = os.path.join(share, "config")
    mode = LaunchConfiguration("mode").perform(context)
    sim = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    map_yaml = LaunchConfiguration("map").perform(context)
    if mode not in ("mapping", "nav"):
        raise RuntimeError(f"mode must be mapping or nav, got {mode!r}")
    with open(os.path.join(cfg, "robot.yaml"), encoding="utf-8") as f:
        fp = footprint_string(yaml.safe_load(f))
    nav2 = os.path.join(cfg, "nav2.yaml")
    t = {"use_sim_time": sim}
    actions: list = [
        Node(package="robotros_obstacles", executable="scan_masker", parameters=[t], output="screen"),
        # Plain-language "now / plan" display, topic /activity (ADR 0004).
        Node(package="robotros_mission", executable="activity_narrator", name="activity_narrator",
             parameters=[os.path.join(cfg, "activity.yaml"), t], output="screen"),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux_a",
            parameters=[os.path.join(cfg, "twist_mux.yaml"), t],
            remappings=[("cmd_vel_out", "cmd_vel_mux_a")],
            output="screen",
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux_b",
            parameters=[os.path.join(cfg, "twist_mux.yaml"), t],
            remappings=[("cmd_vel_out", "cmd_vel")],
            output="screen",
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            parameters=[nav2, t, {"FootprintApproach.points": fp}],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_safety",
            parameters=[nav2, t],
            output="screen",
        ),
    ]
    if mode == "mapping":
        slam_launch = os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "slam_params_file": os.path.join(cfg, "slam.yaml"),
                    "use_sim_time": str(sim).lower(),
                }.items(),
            )
        )
        return actions

    if not map_yaml:
        raise RuntimeError("mode nav needs map:=<map.yaml>")
    bt = os.path.join(share, "behavior_trees", "navigate_wait_replan.xml")
    ctrl = [("cmd_vel", "cmd_vel_ctrl")]
    actions += [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             parameters=[nav2, t, {"yaml_filename": map_yaml}], output="screen"),
        Node(package="nav2_amcl", executable="amcl", name="amcl", parameters=[nav2, t], output="screen"),
        # A parameter dict applies to every node of the process (launch_ros writes it for "/**"), so the
        # footprint also reaches the costmap nodes inside controller_server (local) and planner_server (global).
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             parameters=[nav2, t, {"footprint": fp}], remappings=ctrl, output="screen"),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             parameters=[nav2, t, {"footprint": fp}], output="screen"),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             parameters=[nav2, t], remappings=ctrl, output="screen"),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             parameters=[nav2, t, {"default_nav_to_pose_bt_xml": bt}], output="screen"),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             parameters=[nav2, t],
             remappings=[("cmd_vel", "cmd_vel_ctrl"), ("cmd_vel_smoothed", "cmd_vel_nav")], output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             parameters=[nav2, t], output="screen"),
    ]
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="nav"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
