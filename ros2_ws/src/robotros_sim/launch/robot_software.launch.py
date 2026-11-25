"""Robot software of the simulation, without Gazebo (ADR 0006): what runs on the Pi in the real robot.

sim_hw_adapter (stands in for nano_bridge and both Nanos), navigation (mode mapping|nav), sim_helper (auto) and the
web UI (web). sim.launch.py includes it (robot_software:=true, everything on one computer); for the distributed
simulation the Pi starts it alone (tools/sim_start.sh --pi) while the Surface runs Gazebo and RViz.

Arguments: world (generic), mode (none|mapping|nav), map, auto (false), web (false).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    sim_share = FindPackageShare("robotros_sim")
    world = LaunchConfiguration("world")
    world_file = PathJoinSubstitution([sim_share, "worlds", PythonExpression(["'", world, "' + '.sdf'"])])

    # Stands in for nano_bridge and both Nanos: /cmd_vel -> simulated safety logic -> /sim/cmd_vel -> Gazebo.
    hw_adapter = Node(
        package="robotros_sim",
        executable="sim_hw_adapter",
        parameters=[{"use_sim_time": True, "world_sdf": world_file}],
        output="screen",
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("robotros_bringup"), "launch", "navigation.launch.py"])
        ),
        launch_arguments={
            "mode": LaunchConfiguration("mode"),
            "map": LaunchConfiguration("map"),
            "use_sim_time": "true",
        }.items(),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("mode"), "' != 'none'"])),
    )

    helper = Node(
        package="robotros_sim",
        executable="sim_helper.py",
        parameters=[
            {
                "use_sim_time": True,
                "auto_arm": True,
                "initial_pose": ParameterValue(
                    PythonExpression(["'", LaunchConfiguration("mode"), "' == 'nav'"]), value_type=bool
                ),
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto")),
    )

    # Browser UI (robotros_web): http://localhost:8080, rosbridge on port 9090 (ADR 0005).
    web = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("robotros_web"), "launch", "web.launch.py"])
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
        condition=IfCondition(LaunchConfiguration("web")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="generic"),
            DeclareLaunchArgument("mode", default_value="none"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("auto", default_value="false"),
            DeclareLaunchArgument("web", default_value="false"),
            hw_adapter,
            navigation,
            helper,
            web,
        ]
    )
