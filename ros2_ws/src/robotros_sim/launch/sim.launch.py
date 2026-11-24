"""Start Gazebo Harmonic with a robotROS world, spawn the robot and bridge the topics to ROS.

Arguments:
  world     world name in robotros_sim/worlds (default generic)
  headless  true: server only, rendering without a display (CI, smoke test)
  x, y, yaw spawn pose of the robot

  mode      none (default): only the simulation; mapping: slam_toolbox; nav: map_server, AMCL, Nav2
            (robotros_bringup/launch/navigation.launch.py, spec section 14: `world:=generic mode:=nav`)
  map       map YAML for mode nav
  auto      true: sim_helper arms the robot once and (mode nav) sets the initial pose (tools/sim_start.sh)
  rviz      true: RViz with robotros_sim/rviz/robotros.rviz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    sim_share = FindPackageShare("robotros_sim")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    world_file = PathJoinSubstitution([sim_share, "worlds", PythonExpression(["'", world, "' + '.sdf'"])])
    gz_sim_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])

    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": ["-r -s --headless-rendering ", world_file], "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(headless),
    )
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": ["-r ", world_file], "on_exit_shutdown": "true"}.items(),
        condition=UnlessCondition(headless),
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("robotros_description"), "launch", "description.launch.py"])
        ),
        launch_arguments={"use_sim_time": "true", "sim": "true"}.items(),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "robotros",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", "0.005",
            "-Y", LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    bridge_config = PathJoinSubstitution([sim_share, "config", "gz_bridge.yaml"])
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
        output="screen",
    )

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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([sim_share, "rviz", "robotros.rviz"])],
        parameters=[{"use_sim_time": True}],
        output="log",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="generic"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("mode", default_value="none"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("auto", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            gz_headless,
            gz_gui,
            description,
            spawn,
            bridge,
            hw_adapter,
            navigation,
            helper,
            rviz,
        ]
    )
