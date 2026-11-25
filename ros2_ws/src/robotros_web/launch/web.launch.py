"""Web UI: static server (port 8080) and rosbridge (port 9090), softwarespezifikation.md 13.1.

Arguments: use_sim_time (false), http_port (8080), rosbridge_port (9090).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    t = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("http_port", default_value="8080"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        Node(package="robotros_web", executable="web_server", name="web_server", output="screen",
             parameters=[t, {"port": ParameterValue(LaunchConfiguration("http_port"), value_type=int)}]),
        Node(package="rosbridge_server", executable="rosbridge_websocket", name="rosbridge_websocket",
             output="screen",
             parameters=[t, {"port": ParameterValue(LaunchConfiguration("rosbridge_port"), value_type=int)}]),
    ])
