"""Publish the robotROS model: robot_state_publisher with the xacro from this package.

Arguments: use_sim_time (false), sim (false: plain model; true: with Gazebo plugins),
robot_yaml (installed robotros_bringup/config/robot.yaml), gz_track_scale (1.0; only with sim, see the xacro).
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
    gz_track_scale = LaunchConfiguration("gz_track_scale")
    xacro_file = PathJoinSubstitution([FindPackageShare("robotros_description"), "urdf", "robotros.urdf.xacro"])
    description = Command(
        ["xacro ", xacro_file, " sim:=", sim, " robot_yaml:=", robot_yaml, " gz_track_scale:=", gz_track_scale]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("sim", default_value="false"),
            DeclareLaunchArgument("gz_track_scale", default_value="1.0"),
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
