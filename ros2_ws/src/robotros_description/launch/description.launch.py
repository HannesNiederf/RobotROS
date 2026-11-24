"""Publish the robotROS model: robot_state_publisher with the xacro from this package.

Arguments: use_sim_time (false), sim (false: plain model; true: with Gazebo plugins),
robot_yaml (installed robotros_bringup/config/robot.yaml).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim = LaunchConfiguration("sim")
    robot_yaml = LaunchConfiguration("robot_yaml")
    xacro_file = PathJoinSubstitution([FindPackageShare("robotros_description"), "urdf", "robotros.urdf.xacro"])
    description = Command(["xacro ", xacro_file, " sim:=", sim, " robot_yaml:=", robot_yaml])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("sim", default_value="false"),
            DeclareLaunchArgument(
                "robot_yaml",
                default_value=PathJoinSubstitution([FindPackageShare("robotros_bringup"), "config", "robot.yaml"]),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": ParameterValue(description, value_type=str),
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
        ]
    )
