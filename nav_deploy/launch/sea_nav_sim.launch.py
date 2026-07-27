"""Start Go2 Gazebo, rl_sar locomotion, and SEA-Nav together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("nav_deploy"), "config", "sea_nav_go2.yaml"]
    )
    model_path = PathJoinSubstitution(
        [FindPackageShare("nav_deploy"), "models", "sea_nav_go2.onnx"]
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rl_sar"), "launch", "simulation_robot.launch.py"]
            )
        ),
        launch_arguments={"rname": "go2"}.items(),
    )
    policy = Node(
        package="nav_deploy",
        executable="nav_policy_node",
        name="sea_nav_policy",
        output="screen",
        parameters=[
            config_file,
            {"model_path": model_path, "use_sim_time": True},
        ],
    )
    rl_sim = Node(
        package="rl_sar",
        executable="rl_sim",
        name="rl_sim_node",
        output="screen",
    )
    autostart = Node(
        package="nav_deploy",
        executable="rl_sar_autostart",
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto_start")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Stand Go2 up and enable rl_sar navigation mode",
            ),
            simulation,
            policy,
            # Give Gazebo time to create ros2_control before rl_sim starts its
            # joint controller. The action client then waits for rl_sim itself.
            TimerAction(period=5.0, actions=[rl_sim, autostart]),
        ]
    )
